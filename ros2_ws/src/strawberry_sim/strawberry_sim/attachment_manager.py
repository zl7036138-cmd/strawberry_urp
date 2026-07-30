"""Fail-safe ROS facade for Gazebo detachable joints and bin verification."""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import partial
import inspect
import signal
import threading
import time

from .core import (
    AttachmentGate,
    BinStabilityTracker,
    Pose3D,
    dual_pad_geometric_contact,
    load_scene_config,
    parse_attachment_state,
)


def _shutdown_executor_and_wait(executor, timeout_sec: float | None = None) -> bool:
    """Stop an rclpy executor without destroying nodes under active callbacks.

    Newer rclpy releases can wait for ``MultiThreadedExecutor`` workers through
    a public ``wait_for_threads`` argument.  Jazzy's rclpy 7.1.x does not expose
    that argument and its executor shutdown may return before its private
    ``ThreadPoolExecutor`` has joined.  Keep the private fallback isolated and
    feature-detected so it can disappear when the public API is available.
    """

    try:
        supports_thread_wait = (
            "wait_for_threads" in inspect.signature(executor.shutdown).parameters
        )
    except (TypeError, ValueError):
        supports_thread_wait = False

    if supports_thread_wait:
        callbacks_stopped = bool(
            executor.shutdown(
                timeout_sec=timeout_sec,
                wait_for_threads=True,
            )
        )
    else:
        callbacks_stopped = bool(executor.shutdown(timeout_sec=timeout_sec))
        worker_pool = getattr(executor, "_executor", None)
        if worker_pool is not None and hasattr(worker_pool, "shutdown"):
            worker_pool.shutdown(wait=True)

    # Jazzy also retains the rclpy Tasks separately from the Python thread
    # pool.  Consume their terminal exceptions before node destruction so Task
    # finalizers cannot report misleading "exception was never retrieved"
    # messages during shutdown.
    tasks = getattr(executor, "_futures", None)
    if tasks is not None:
        for task in tuple(tasks):
            if not task.done():
                continue
            try:
                task.result()
            except BaseException:
                pass
        tasks.clear()
    return callbacks_stopped


def main(args=None) -> None:  # pragma: no cover - exercised in ROS integration
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.clock import Clock, ClockType
        from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from ros_gz_interfaces.srv import ControlWorld
        from std_msgs.msg import Bool, Empty, String
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    @dataclass
    class FruitRuntime:
        target_id: int
        condition: threading.Condition = field(
            default_factory=threading.Condition
        )
        state_known: bool = False
        attached: bool = False
        left_contact: bool = False
        left_stamp_sec: float | None = None
        right_contact: bool = False
        right_stamp_sec: float | None = None
        latest_pose: Pose3D | None = None
        latest_pose_stamp_sec: float | None = None
        attach_publisher: object | None = None
        detach_publisher: object | None = None

    class AttachmentManager(Node):
        def __init__(self) -> None:
            super().__init__("attachment_manager")
            self.declare_parameter("scene_config_file", "")
            self.declare_parameter("attachment_backend_enabled", False)
            self.declare_parameter("contact_freshness_sec", 0.25)
            self.declare_parameter("geometric_contact_fallback", True)
            self.declare_parameter("geometric_contact_margin_m", 0.003)
            self.declare_parameter("confirmation_timeout_wall_sec", 0.75)
            self.declare_parameter("verification_timeout_wall_sec", 3.0)
            self.declare_parameter("backend_initialization_attempts", 10)
            self.declare_parameter("backend_initialization_period_sec", 0.20)
            self.declare_parameter("resume_world_after_initialization", False)
            self.declare_parameter(
                "world_control_service",
                "/world/strawberry_orchard/control",
            )

            config_path = str(self.get_parameter("scene_config_file").value)
            if not config_path:
                raise RuntimeError("scene_config_file parameter is required")
            self._scene = load_scene_config(config_path)
            self._backend_enabled = bool(
                self.get_parameter("attachment_backend_enabled").value
            )
            self._confirmation_timeout = float(
                self.get_parameter("confirmation_timeout_wall_sec").value
            )
            self._verification_timeout = float(
                self.get_parameter("verification_timeout_wall_sec").value
            )
            self._max_initialization_attempts = int(
                self.get_parameter("backend_initialization_attempts").value
            )
            self._resume_world = bool(
                self.get_parameter("resume_world_after_initialization").value
            )
            if self._confirmation_timeout <= 0.0 or self._verification_timeout <= 0.0:
                raise RuntimeError("service wall timeouts must be positive")
            if self._max_initialization_attempts <= 0:
                raise RuntimeError("backend_initialization_attempts must be positive")
            if self._resume_world and not self._backend_enabled:
                raise RuntimeError(
                    "world resume gate requires the attachment backend"
                )

            self._gate = AttachmentGate(
                float(self.get_parameter("contact_freshness_sec").value)
            )
            self._geometric_contact_fallback = bool(
                self.get_parameter("geometric_contact_fallback").value
            )
            fruit_radius = self._scene.fruit_collision_radius_m
            geometric_margin = float(
                self.get_parameter("geometric_contact_margin_m").value
            )
            if fruit_radius <= 0.0 or geometric_margin < 0.0:
                raise RuntimeError(
                    "geometric contact radius must be positive and margin non-negative"
                )
            self._geometric_contact_distance = fruit_radius + geometric_margin
            self._stability = BinStabilityTracker(
                self._scene.bin_bounds, self._scene.bin_stability_sec
            )
            self._group = ReentrantCallbackGroup()
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(
                self._tf_buffer, self, spin_thread=False
            )
            self._runtime = {
                fruit.target_id: FruitRuntime(fruit.target_id)
                for fruit in self._scene.ordered_fruits
            }
            self._managed_subscriptions: list[object] = []
            self._managed_services: list[object] = []
            self._initialization_attempts = 0
            self._initialization_complete = False
            self._initialization_timer = None
            self._initialization_lock = threading.RLock()
            self._world_control_client = None
            self._world_resume_future = None
            self._shutting_down = False
            if self._resume_world:
                world_control_service = str(
                    self.get_parameter("world_control_service").value
                )
                if not world_control_service:
                    raise RuntimeError("world_control_service must not be empty")
                self._world_control_client = self.create_client(
                    ControlWorld,
                    world_control_service,
                    callback_group=self._group,
                )

            for fruit in self._scene.ordered_fruits:
                runtime = self._runtime[fruit.target_id]
                prefix = f"/strawberry/sim/fruit_{fruit.target_id}"
                runtime.attach_publisher = self.create_publisher(
                    Empty, f"{prefix}/attach_command", 10
                )
                runtime.detach_publisher = self.create_publisher(
                    Empty, f"{prefix}/detach_command", 10
                )
                self._managed_subscriptions.extend(
                    [
                        self.create_subscription(
                            String,
                            f"{prefix}/attached_state",
                            partial(self._on_attachment_state, fruit.target_id),
                            10,
                            callback_group=self._group,
                        ),
                        self.create_subscription(
                            Bool,
                            f"{prefix}/left_contact",
                            partial(self._on_contact, fruit.target_id, "left"),
                            qos_profile_sensor_data,
                            callback_group=self._group,
                        ),
                        self.create_subscription(
                            Bool,
                            f"{prefix}/right_contact",
                            partial(self._on_contact, fruit.target_id, "right"),
                            qos_profile_sensor_data,
                            callback_group=self._group,
                        ),
                        self.create_subscription(
                            PoseStamped,
                            fruit.ground_truth_pose_topic,
                            partial(self._on_pose, fruit.target_id),
                            qos_profile_sensor_data,
                            callback_group=self._group,
                        ),
                    ]
                )
                self._managed_services.extend(
                    [
                        self.create_service(
                            Trigger,
                            f"{prefix}/attach",
                            partial(self._attach, fruit.target_id),
                            callback_group=self._group,
                        ),
                        self.create_service(
                            Trigger,
                            f"{prefix}/detach",
                            partial(self._detach, fruit.target_id),
                            callback_group=self._group,
                        ),
                        self.create_service(
                            Trigger,
                            f"{prefix}/verify_in_bin",
                            partial(self._verify_in_bin, fruit.target_id),
                            callback_group=self._group,
                        ),
                    ]
                )

            if self._backend_enabled:
                period = float(
                    self.get_parameter("backend_initialization_period_sec").value
                )
                if period <= 0.0:
                    raise RuntimeError("backend initialization period must be positive")
                # Startup can precede the first /clock message. A ROS-time timer
                # created before that transition may be scheduled against wall
                # time and then jump far into the future when simulation time
                # starts. Infrastructure retries therefore use a steady clock.
                self._initialization_clock = Clock(
                    clock_type=ClockType.STEADY_TIME
                )
                self._initialization_timer = self.create_timer(
                    period,
                    self._initialize_backend,
                    callback_group=self._group,
                    clock=self._initialization_clock,
                )
                self.get_logger().warn(
                    "attachment backend enabled; waiting for every DetachableJoint "
                    "to confirm the detached startup state"
                )
            else:
                self.get_logger().warn(
                    "attachment backend disabled: attach, detach, and verification "
                    "services will reject requests safely"
                )

        def _sim_time_sec(self) -> float:
            return self.get_clock().now().nanoseconds / 1_000_000_000.0

        def _is_shutting_down(self) -> bool:
            with self._initialization_lock:
                return self._shutting_down

        def _on_attachment_state(self, target_id: int, message) -> None:
            if self._is_shutting_down():
                return
            try:
                attached = parse_attachment_state(message.data)
            except ValueError as exc:
                self.get_logger().error(str(exc))
                return
            runtime = self._runtime[target_id]
            with runtime.condition:
                runtime.state_known = True
                runtime.attached = attached
                runtime.condition.notify_all()
            self._complete_initialization_if_ready()

        def _on_contact(self, target_id: int, side: str, message) -> None:
            if self._is_shutting_down():
                return
            runtime = self._runtime[target_id]
            stamp = self._sim_time_sec()
            with runtime.condition:
                if side == "left":
                    runtime.left_contact = bool(message.data)
                    runtime.left_stamp_sec = stamp
                else:
                    runtime.right_contact = bool(message.data)
                    runtime.right_stamp_sec = stamp
                runtime.condition.notify_all()

        def _on_pose(self, target_id: int, message) -> None:
            if self._is_shutting_down():
                return
            stamp = (
                float(message.header.stamp.sec)
                + float(message.header.stamp.nanosec) / 1_000_000_000.0
            )
            if stamp == 0.0:
                stamp = self._sim_time_sec()
            position = message.pose.position
            pose = Pose3D(float(position.x), float(position.y), float(position.z))
            self._stability.update(target_id, pose, stamp)
            runtime = self._runtime[target_id]
            with runtime.condition:
                runtime.latest_pose = pose
                runtime.latest_pose_stamp_sec = stamp
                runtime.condition.notify_all()

        def _geometric_dual_contact(self, runtime: FruitRuntime) -> bool:
            if self._is_shutting_down() or not self._geometric_contact_fallback:
                return False
            with runtime.condition:
                fruit_pose = runtime.latest_pose
                pose_stamp = runtime.latest_pose_stamp_sec
            if fruit_pose is None or pose_stamp is None:
                return False
            now = self._sim_time_sec()
            age = now - pose_stamp
            if age < 0.0 or age > self._gate.contact_freshness_sec:
                return False
            pads = []
            try:
                for side in ("left", "right"):
                    transform = self._tf_buffer.lookup_transform(
                        self._scene.base_frame,
                        f"panda_{side}_contact_pad",
                        Time(),
                    ).transform.translation
                    pads.append(
                        Pose3D(
                            float(transform.x),
                            float(transform.y),
                            float(transform.z),
                        )
                    )
            except TransformException as exc:
                self.get_logger().warn(
                    f"geometric contact fallback has no current pad TF: {exc}"
                )
                return False
            return dual_pad_geometric_contact(
                fruit_pose,
                pads[0],
                pads[1],
                self._geometric_contact_distance,
            )

        def _finish_initialization(self, message: str) -> None:
            with self._initialization_lock:
                if self._initialization_complete or self._shutting_down:
                    return
                self._initialization_complete = True
                if self._initialization_timer is not None:
                    self._initialization_timer.cancel()
            self.get_logger().info(message)

        def _on_world_resume(self, future) -> None:
            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001 - report and retry boundedly
                with self._initialization_lock:
                    if self._world_resume_future is future:
                        self._world_resume_future = None
                    shutting_down = self._shutting_down
                # Always consume the future exception, but do not emit a false
                # infrastructure error for the cancellation requested by
                # prepare_shutdown().
                if not shutting_down:
                    self.get_logger().error(
                        f"world resume request failed: {exc}"
                    )
                return
            with self._initialization_lock:
                if self._world_resume_future is future:
                    self._world_resume_future = None
                shutting_down = self._shutting_down
            if shutting_down:
                return
            if response is None or not response.success:
                self.get_logger().error(
                    "Gazebo rejected the world resume request"
                )
                return
            self._finish_initialization(
                "attachment backend initialized with all fruits detached; "
                "Gazebo physics resumed"
            )

        def _complete_initialization_if_ready(self) -> None:
            with self._initialization_lock:
                if self._initialization_complete or self._shutting_down:
                    return
                detached = all(
                    runtime.state_known and not runtime.attached
                    for runtime in self._runtime.values()
                )
                if not detached:
                    return
                if not self._resume_world:
                    self._finish_initialization(
                        "attachment backend initialized with all fruits detached"
                    )
                    return
                if self._world_resume_future is not None:
                    return
                if (
                    self._world_control_client is None
                    or not self._world_control_client.service_is_ready()
                ):
                    return
                request = ControlWorld.Request()
                request.world_control.pause = False
                future = self._world_control_client.call_async(request)
                self._world_resume_future = future
            future.add_done_callback(self._on_world_resume)
            self.get_logger().info(
                "all fruits detached at their initial poses; requesting Gazebo "
                "physics resume"
            )

        def _initialize_backend(self) -> None:
            if self._is_shutting_down():
                return
            self._complete_initialization_if_ready()
            if self._initialization_complete:
                return
            if self._initialization_attempts >= self._max_initialization_attempts:
                if self._initialization_timer is not None:
                    self._initialization_timer.cancel()
                self.get_logger().error(
                    "attachment backend did not confirm a detached startup state; "
                    "services remain locked"
                )
                return
            self._initialization_attempts += 1
            for runtime in self._runtime.values():
                if not runtime.state_known or runtime.attached:
                    runtime.detach_publisher.publish(Empty())

        def _wait_for_state(self, runtime: FruitRuntime, desired: bool) -> bool:
            deadline = time.monotonic() + self._confirmation_timeout
            with runtime.condition:
                while not (runtime.state_known and runtime.attached is desired):
                    if self._is_shutting_down():
                        return False
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        return False
                    runtime.condition.wait(timeout=remaining)
                return True

        def _attach(self, target_id: int, request, response):
            del request
            if self._is_shutting_down():
                response.success = False
                response.message = "attachment manager is shutting down"
                return response
            runtime = self._runtime[target_id]
            geometric_fallback_used = False
            with runtime.condition:
                decision = self._gate.assess_attach(
                    now_sec=self._sim_time_sec(),
                    backend_enabled=self._backend_enabled,
                    backend_initialized=self._initialization_complete,
                    state_known=runtime.state_known,
                    attached=runtime.attached,
                    left_contact=runtime.left_contact,
                    left_stamp_sec=runtime.left_stamp_sec,
                    right_contact=runtime.right_contact,
                    right_stamp_sec=runtime.right_stamp_sec,
                )
                already_attached = runtime.state_known and runtime.attached
            if not decision.allowed and self._geometric_dual_contact(runtime):
                now = self._sim_time_sec()
                with runtime.condition:
                    decision = self._gate.assess_attach(
                        now_sec=now,
                        backend_enabled=self._backend_enabled,
                        backend_initialized=self._initialization_complete,
                        state_known=runtime.state_known,
                        attached=runtime.attached,
                        left_contact=True,
                        left_stamp_sec=now,
                        right_contact=True,
                        right_stamp_sec=now,
                    )
                    already_attached = runtime.state_known and runtime.attached
                geometric_fallback_used = decision.allowed
            if not decision.allowed:
                response.success = False
                response.message = decision.reason
                return response
            if already_attached:
                response.success = True
                response.message = decision.reason
                return response
            if geometric_fallback_used:
                self.get_logger().warn(
                    f"target {target_id} attachment accepted by strict geometric "
                    "dual-pad contact fallback"
                )
            runtime.attach_publisher.publish(Empty())
            response.success = self._wait_for_state(runtime, True)
            response.message = (
                "Gazebo attachment confirmed"
                if response.success
                else "no Gazebo attachment confirmation before timeout"
            )
            return response

        def _detach(self, target_id: int, request, response):
            del request
            if self._is_shutting_down():
                response.success = False
                response.message = "attachment manager is shutting down"
                return response
            runtime = self._runtime[target_id]
            if not self._backend_enabled or not self._initialization_complete:
                response.success = False
                response.message = "attachment backend is not ready"
                return response
            with runtime.condition:
                if not runtime.state_known:
                    response.success = False
                    response.message = "attachment state is unknown"
                    return response
                if not runtime.attached:
                    response.success = True
                    response.message = "fruit is already detached"
                    return response
            runtime.detach_publisher.publish(Empty())
            response.success = self._wait_for_state(runtime, False)
            response.message = (
                "Gazebo detachment confirmed"
                if response.success
                else "no Gazebo detachment confirmation before timeout"
            )
            return response

        def _verify_in_bin(self, target_id: int, request, response):
            del request
            if self._is_shutting_down():
                response.success = False
                response.message = "attachment manager is shutting down"
                return response
            runtime = self._runtime[target_id]
            if not self._backend_enabled or not self._initialization_complete:
                response.success = False
                response.message = "attachment backend is not ready"
                return response
            deadline = time.monotonic() + self._verification_timeout
            with runtime.condition:
                while True:
                    if self._is_shutting_down():
                        response.success = False
                        response.message = "attachment manager is shutting down"
                        return response
                    if not runtime.state_known:
                        response.success = False
                        response.message = "attachment state is unknown"
                        return response
                    if runtime.attached:
                        response.success = False
                        response.message = "fruit is still attached"
                        return response
                    if self._stability.is_stable(target_id):
                        response.success = True
                        response.message = (
                            f"fruit remained inside bin for "
                            f"{self._scene.bin_stability_sec:.3f} simulated seconds"
                        )
                        return response
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        response.success = False
                        response.message = (
                            "no stable in-bin ground-truth confirmation before timeout"
                        )
                        return response
                    runtime.condition.wait(timeout=remaining)

        def prepare_shutdown(self) -> None:
            with self._initialization_lock:
                self._shutting_down = True
                if self._initialization_timer is not None:
                    self._initialization_timer.cancel()
                future = self._world_resume_future
                self._world_resume_future = None
            # Wake service callbacks that may be blocked waiting for a Gazebo
            # attachment-state or in-bin update.  They observe the shutdown
            # flag and return before node destruction.
            for runtime in self._runtime.values():
                with runtime.condition:
                    runtime.condition.notify_all()
            if future is not None and not future.done():
                future.cancel()

    rclpy.init(args=args)
    node = AttachmentManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.prepare_shutdown()
        _shutdown_executor_and_wait(executor)
        node.destroy_node()
        rclpy.try_shutdown()
