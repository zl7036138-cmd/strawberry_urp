"""ROS/MoveIt implementation of the dependency-light MotionBackend protocol."""

from __future__ import annotations

import copy
import math
import threading
import time
from types import MappingProxyType
from typing import Callable, Mapping

from .core import MotionOutcome, Pose
from .moveit_scene import (
    apply_fruit_collision_scene,
    apply_static_collision_scene,
    set_target_fruit_collision,
)
from .scene_geometry import FRUIT_COLLISION_RADIUS_M, STATIC_COLLISION_OBJECTS


def gripper_result_allows_command(
    *,
    target_position_m: float,
    observed_position_m: float,
    open_position_m: float,
    closed_position_m: float,
    stalled: bool,
    reached_goal: bool,
    position_tolerance_m: float = 0.003,
    minimum_close_travel_m: float = 0.002,
) -> bool:
    """Reject a false gripper stall that occurred without meaningful travel."""

    values = (
        target_position_m,
        observed_position_m,
        open_position_m,
        closed_position_m,
        position_tolerance_m,
        minimum_close_travel_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("gripper result positions must be finite")
    if not 0.0 <= closed_position_m < open_position_m:
        raise ValueError("gripper result limits are invalid")
    if position_tolerance_m <= 0.0:
        raise ValueError("gripper result tolerance must be positive")
    if minimum_close_travel_m <= 0.0:
        raise ValueError("minimum close travel must be positive")
    closing = target_position_m <= closed_position_m + 1.0e-6
    if not closing:
        return reached_goal and (
            abs(observed_position_m - target_position_m)
            <= position_tolerance_m
        )
    if reached_goal and (
        abs(observed_position_m - target_position_m)
        <= position_tolerance_m
    ):
        return True
    return stalled and (
        observed_position_m
        <= open_position_m - minimum_close_travel_m
    )


class MoveItBackend:
    """Plan with MoveItPy and execute through ros2_control controllers.

    ROS imports are intentionally delayed until construction so unit tests can
    import the package on machines without ROS 2.
    """

    def __init__(
        self,
        node,
        *,
        planning_group: str = "panda_arm",
        pose_link: str = "panda_hand",
        base_frame: str = "panda_link0",
        home_configuration: str = "ready",
        gripper_action: str = "/panda_gripper_controller/gripper_cmd",
        gripper_secondary_action: str = (
            "/panda_gripper_right_controller/gripper_cmd"
        ),
        arm_action: str = "/panda_arm_controller/follow_joint_trajectory",
        gripper_joint: str = "panda_finger_joint1",
        gripper_secondary_joint: str = "panda_finger_joint2",
        open_width_m: float = 0.04,
        closed_width_m: float = 0.025,
        gripper_position_tolerance_m: float = 0.003,
        max_effort_n: float = 40.0,
        request_timeout_sec: float = 5.0,
        startup_timeout_sec: float = 30.0,
        trajectory_timeout_margin_sec: float = 25.0,
        settle_timeout_sec: float = 1.5,
        settle_sample_period_sec: float = 0.05,
        settle_delta_rad: float = 0.002,
        settle_stable_samples: int = 3,
        pose_position_tolerance_m: float = 0.01,
        pose_orientation_tolerance_rad: float = math.radians(5.0),
        intermediate_position_tolerance_m: float = 0.02,
        intermediate_orientation_tolerance_rad: float = math.radians(8.0),
        cartesian_endpoint_retry_limit: int = 1,
        max_grasp_segment_m: float = 0.01,
        max_orientation_segment_rad: float = math.radians(10.0),
        max_collision_joint_step_rad: float = 0.01,
        safe_transit_clearance_m: float = 0.02,
        place_transit_clearance_m: float | None = None,
        safe_transit_corridor_y_m: float = -0.10,
        fruit_obstacles: dict[int, Pose] | None = None,
        fruit_collision_radius_m: float = FRUIT_COLLISION_RADIUS_M,
        static_collision_objects=STATIC_COLLISION_OBJECTS,
        fruit_pose_provider: Callable[
            [], Mapping[int, tuple[float, float, float]]
        ]
        | None = None,
        dynamic_fruit_manifest: bool = False,
        entity_id_resolver: Callable[[int, Pose], int] | None = None,
        config_dict: dict | None = None,
    ) -> None:
        try:
            from control_msgs.action import (
                FollowJointTrajectory,
                ParallelGripperCommand,
            )
            from geometry_msgs.msg import PoseStamped
            from moveit.planning import MoveItPy
            from rclpy.action import ActionClient
            from rclpy.callback_groups import ReentrantCallbackGroup
            from rclpy.duration import Duration
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import JointState
            from std_srvs.srv import Trigger
            from trajectory_msgs.msg import JointTrajectoryPoint
        except ImportError as exc:  # pragma: no cover - ROS integration only
            raise RuntimeError(
                "MoveIt backend requires moveit_py, control_msgs, geometry_msgs, and std_srvs"
            ) from exc

        if (
            not planning_group
            or not pose_link
            or not base_frame
            or not gripper_action
            or not gripper_secondary_action
            or not gripper_joint
            or not gripper_secondary_joint
            or not arm_action
        ):
            raise ValueError("planning group and frame names must be non-empty")
        if gripper_joint == gripper_secondary_joint:
            raise ValueError("gripper joint names must be distinct")
        if not 0.0 <= closed_width_m < open_width_m:
            raise ValueError("gripper widths must satisfy 0 <= closed < open")
        if gripper_position_tolerance_m <= 0.0:
            raise ValueError("gripper position tolerance must be positive")
        if request_timeout_sec <= 0.0:
            raise ValueError("request timeout must be positive")
        if startup_timeout_sec <= 0.0:
            raise ValueError("startup timeout must be positive")
        if trajectory_timeout_margin_sec <= 0.0:
            raise ValueError("trajectory timeout margin must be positive")
        if settle_timeout_sec <= 0.0 or settle_sample_period_sec <= 0.0:
            raise ValueError("settling timeouts must be positive")
        if settle_delta_rad <= 0.0 or settle_stable_samples <= 0:
            raise ValueError("settling threshold and sample count must be positive")
        if pose_position_tolerance_m <= 0.0 or pose_orientation_tolerance_rad <= 0.0:
            raise ValueError("pose verification tolerances must be positive")
        if (
            intermediate_position_tolerance_m <= 0.0
            or intermediate_orientation_tolerance_rad <= 0.0
        ):
            raise ValueError("intermediate pose tolerances must be positive")
        if (
            isinstance(cartesian_endpoint_retry_limit, bool)
            or not isinstance(cartesian_endpoint_retry_limit, int)
            or not 0 <= cartesian_endpoint_retry_limit <= 1
        ):
            raise ValueError("Cartesian endpoint retry limit must be zero or one")
        if max_grasp_segment_m <= 0.0:
            raise ValueError("grasp segment length must be positive")
        if max_orientation_segment_rad <= 0.0:
            raise ValueError("orientation segment angle must be positive")
        if max_collision_joint_step_rad <= 0.0:
            raise ValueError("collision-check joint step must be positive")
        if safe_transit_clearance_m <= 0.0:
            raise ValueError("safe transit clearance must be positive")
        if place_transit_clearance_m is None:
            place_transit_clearance_m = safe_transit_clearance_m
        if place_transit_clearance_m <= 0.0:
            raise ValueError("place transit clearance must be positive")
        if not math.isfinite(safe_transit_corridor_y_m):
            raise ValueError("safe transit corridor must be finite")
        if fruit_collision_radius_m <= 0.0:
            raise ValueError("fruit collision radius must be positive")
        fruit_manifest = self._build_fruit_manifest(fruit_obstacles or {})

        self.node = node
        self.planning_group = planning_group
        self.pose_link = pose_link
        self.base_frame = base_frame
        self.home_configuration = home_configuration
        self.gripper_joint = gripper_joint
        self.gripper_secondary_joint = gripper_secondary_joint
        self.gripper_joints = (
            self.gripper_joint,
            self.gripper_secondary_joint,
        )
        self.open_width_m = float(open_width_m)
        self.closed_width_m = float(closed_width_m)
        self.gripper_position_tolerance_m = float(
            gripper_position_tolerance_m
        )
        self.max_effort_n = float(max_effort_n)
        self.request_timeout_sec = float(request_timeout_sec)
        self.startup_timeout_sec = float(startup_timeout_sec)
        self.trajectory_timeout_margin_sec = float(
            trajectory_timeout_margin_sec
        )
        self.settle_timeout_sec = float(settle_timeout_sec)
        self.settle_sample_period_sec = float(settle_sample_period_sec)
        self.settle_delta_rad = float(settle_delta_rad)
        self.settle_stable_samples = int(settle_stable_samples)
        self.pose_position_tolerance_m = float(pose_position_tolerance_m)
        self.pose_orientation_tolerance_rad = float(
            pose_orientation_tolerance_rad
        )
        self.intermediate_position_tolerance_m = float(
            intermediate_position_tolerance_m
        )
        self.intermediate_orientation_tolerance_rad = float(
            intermediate_orientation_tolerance_rad
        )
        self.cartesian_endpoint_retry_limit = cartesian_endpoint_retry_limit
        self.max_grasp_segment_m = float(max_grasp_segment_m)
        self.max_orientation_segment_rad = float(max_orientation_segment_rad)
        self.max_collision_joint_step_rad = float(
            max_collision_joint_step_rad
        )
        self.safe_transit_clearance_m = float(safe_transit_clearance_m)
        self.place_transit_clearance_m = float(
            place_transit_clearance_m
        )
        self.safe_transit_corridor_y_m = float(
            safe_transit_corridor_y_m
        )
        self.fruit_collision_radius_m = float(fruit_collision_radius_m)
        self.static_collision_objects = tuple(static_collision_objects)
        static_collision_ids = [
            specification.object_id
            for specification in self.static_collision_objects
        ]
        if not static_collision_ids:
            raise ValueError("static collision scene must not be empty")
        if len(static_collision_ids) != len(set(static_collision_ids)):
            raise ValueError("static collision object IDs must be unique")
        # Preserve the declared manifest as a fallback only for dependency-light
        # tests. Runtime simulation supplies a fresh all-fruit truth snapshot
        # before every goal so dynamic scenes cannot leave ghost obstacles.
        self.fruit_obstacle_centers_m = fruit_manifest
        self.fruit_pose_provider = fruit_pose_provider
        self.dynamic_fruit_manifest = bool(dynamic_fruit_manifest)
        self.entity_id_resolver = entity_id_resolver
        self._PoseStamped = PoseStamped
        self._FollowJointTrajectory = FollowJointTrajectory
        self._ParallelGripperCommand = ParallelGripperCommand
        self._Duration = Duration
        self._JointTrajectoryPoint = JointTrajectoryPoint
        self._Trigger = Trigger
        self._arm_joint_names = tuple(f"panda_joint{index}" for index in range(1, 8))
        self._callback_group = ReentrantCallbackGroup()
        moveit_arguments = {"node_name": "strawberry_moveit_py"}
        if config_dict is not None:
            moveit_arguments["config_dict"] = config_dict
        self._moveit = MoveItPy(**moveit_arguments)
        self._arm = self._moveit.get_planning_component(planning_group)
        self._planning_scene_monitor = self._moveit.get_planning_scene_monitor()
        self._arm_action_probe = ActionClient(
            node,
            FollowJointTrajectory,
            arm_action,
            callback_group=self._callback_group,
        )
        self._gripper = ActionClient(
            node,
            ParallelGripperCommand,
            gripper_action,
            callback_group=self._callback_group,
        )
        self._gripper_secondary = ActionClient(
            node,
            ParallelGripperCommand,
            gripper_secondary_action,
            callback_group=self._callback_group,
        )
        self._gripper_state_lock = threading.Lock()
        self._latest_gripper_positions_m = {
            joint_name: None for joint_name in self.gripper_joints
        }
        self._gripper_state_subscription = node.create_subscription(
            JointState,
            "/joint_states",
            self._on_gripper_joint_state,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self._service_clients: dict[tuple[int, str], object] = {}
        self._wait_until_runtime_ready()
        self.static_collision_ids = apply_static_collision_scene(
            self._planning_scene_monitor,
            self.base_frame,
            self.static_collision_objects,
        )
        self.node.get_logger().info(
            "MoveIt static collision scene loaded: "
            + ", ".join(self.static_collision_ids)
        )
        self.fruit_collision_ids = apply_fruit_collision_scene(
            self._planning_scene_monitor,
            self.base_frame,
            {
                target_id: center
                for target_id, center in self.fruit_obstacle_centers_m.items()
            },
            self.fruit_collision_radius_m,
        )
        self._dynamic_collision_target_ids = set(self.fruit_obstacle_centers_m)
        self._prepared_target_id = None
        self._prepared_entity_id = None
        self._target_contact_open = False
        self._prepared_scene_centers_m = None
        if self.fruit_collision_ids:
            self.node.get_logger().info(
                "MoveIt fruit collision scene loaded: "
                + ", ".join(self.fruit_collision_ids)
            )

    def _on_gripper_joint_state(self, message) -> None:
        for name, position in zip(
            message.name, message.position, strict=False
        ):
            joint_name = str(name)
            if joint_name in self.gripper_joints:
                with self._gripper_state_lock:
                    self._latest_gripper_positions_m[joint_name] = float(
                        position
                    )

    @staticmethod
    def _build_fruit_manifest(fruit_obstacles) -> MappingProxyType:
        centers: dict[int, tuple[float, float, float]] = {}
        for target_id, pose in dict(fruit_obstacles).items():
            if target_id <= 0:
                raise ValueError("fruit obstacle target IDs must be positive")
            center = (float(pose.x), float(pose.y), float(pose.z))
            if not all(math.isfinite(value) for value in center):
                raise ValueError("fruit obstacle centres must be finite")
            centers[target_id] = center
        return MappingProxyType(centers)

    def _fruit_centers_for_planning(self) -> MappingProxyType:
        provider = getattr(self, "fruit_pose_provider", None)
        if provider is None:
            return self.fruit_obstacle_centers_m
        raw = provider()
        centers = {
            int(target_id): tuple(float(value) for value in center)
            for target_id, center in dict(raw).items()
        }
        expected_ids = set(self.fruit_obstacle_centers_m)
        dynamic_manifest = bool(getattr(self, "dynamic_fruit_manifest", False))
        if not dynamic_manifest and set(centers) != expected_ids:
            raise ValueError("live fruit pose IDs differ from the scene manifest")
        if dynamic_manifest and any(target_id <= 0 for target_id in centers):
            raise ValueError("dynamic tracked fruit IDs must be positive")
        if any(
            len(center) != 3 or not all(math.isfinite(value) for value in center)
            for center in centers.values()
        ):
            raise ValueError("live fruit poses must contain three finite coordinates")
        return MappingProxyType(centers)

    def _synchronize_fruit_collision_scene(self, centers) -> None:
        current_ids = set(int(target_id) for target_id in centers)
        if bool(getattr(self, "dynamic_fruit_manifest", False)):
            previous_ids = set(
                getattr(self, "_dynamic_collision_target_ids", self.fruit_obstacle_centers_m)
            )
            for target_id in sorted(previous_ids - current_ids):
                set_target_fruit_collision(
                    self._planning_scene_monitor,
                    self.base_frame,
                    target_id,
                )
        apply_fruit_collision_scene(
            self._planning_scene_monitor,
            self.base_frame,
            centers,
            self.fruit_collision_radius_m,
        )
        self._dynamic_collision_target_ids = current_ids

    def evaluate_pose_sequence(
        self, poses: tuple[Pose, ...]
    ) -> tuple[bool, bool, float, float]:
        """Check seeded IK and interpolated joint collisions without execution."""

        if not poses:
            raise ValueError("pose feasibility sequence cannot be empty")
        started = time.perf_counter()
        joint_travel = 0.0
        with self._planning_scene_monitor.read_only() as scene:
            state = copy.deepcopy(scene.current_state)
            previous = tuple(
                float(value)
                for value in state.get_joint_group_positions(self.planning_group)
            )
            for pose in poses:
                solved = state.set_from_ik(
                    self.planning_group,
                    self._pose_message(pose.normalized()).pose,
                    self.pose_link,
                    0.2,
                )
                if not solved:
                    return False, False, time.perf_counter() - started, joint_travel
                state.update()
                current = tuple(
                    float(value)
                    for value in state.get_joint_group_positions(self.planning_group)
                )
                maximum_delta = max(abs(right - left) for left, right in zip(previous, current))
                joint_travel += sum(abs(right - left) for left, right in zip(previous, current))
                sample_count = max(1, math.ceil(maximum_delta / self.max_collision_joint_step_rad))
                for sample_index in range(1, sample_count + 1):
                    fraction = sample_index / sample_count
                    sample = tuple(
                        left + fraction * (right - left)
                        for left, right in zip(previous, current)
                    )
                    state.set_joint_group_positions(self.planning_group, sample)
                    state.update()
                    if not scene.is_state_valid(state, self.planning_group, False):
                        return False, True, time.perf_counter() - started, joint_travel
                previous = current
        return True, False, time.perf_counter() - started, joint_travel

    def prepare_pick(self, target_id: int, target_pose: Pose) -> bool:
        """Keep the selected fruit solid while planning the transit motion."""

        if self._prepared_target_id is not None:
            self.node.get_logger().error(
                "cannot prepare a target while another collision lifecycle is active"
            )
            return False
        try:
            scene_centers = self._fruit_centers_for_planning()
            if target_id not in scene_centers:
                raise ValueError(f"target {target_id} is absent from the live fruit scene")
            if getattr(self, "fruit_pose_provider", None) is not None:
                self._synchronize_fruit_collision_scene(scene_centers)
        except Exception as exc:  # pragma: no cover - ROS integration only
            self.node.get_logger().error(
                f"failed to synchronize live fruit collision scene: {exc}"
            )
            return False
        target_center = (target_pose.x, target_pose.y, target_pose.z)
        if not all(math.isfinite(value) for value in target_center):
            self.node.get_logger().error("target fruit pose must be finite")
            return False
        try:
            object_id = set_target_fruit_collision(
                self._planning_scene_monitor,
                self.base_frame,
                target_id,
                center_m=target_center,
                radius_m=self.fruit_collision_radius_m,
            )
        except Exception as exc:  # pragma: no cover - ROS integration only
            self.node.get_logger().error(
                f"failed to prepare target fruit collision object: {exc}"
            )
            return False
        self._prepared_target_id = target_id
        try:
            resolver = getattr(self, "entity_id_resolver", None)
            self._prepared_entity_id = (
                int(resolver(target_id, target_pose))
                if resolver is not None
                else target_id
            )
        except Exception as exc:
            self.node.get_logger().error(f"failed to resolve simulated fruit entity: {exc}")
            try:
                set_target_fruit_collision(
                    self._planning_scene_monitor, self.base_frame, target_id
                )
            except Exception as cleanup_exc:  # pragma: no cover - ROS integration only
                self.node.get_logger().error(
                    f"failed to remove unresolved target obstacle: {cleanup_exc}"
                )
            self._prepared_target_id = None
            return False
        if self._prepared_entity_id <= 0:
            self.node.get_logger().error("resolved simulated fruit entity ID must be positive")
            try:
                set_target_fruit_collision(
                    self._planning_scene_monitor, self.base_frame, target_id
                )
            except Exception as cleanup_exc:  # pragma: no cover - ROS integration only
                self.node.get_logger().error(
                    f"failed to remove invalid target obstacle: {cleanup_exc}"
                )
            self._prepared_target_id = None
            self._prepared_entity_id = None
            return False
        self._target_contact_open = False
        self._prepared_scene_centers_m = scene_centers
        self.node.get_logger().info(
            f"MoveIt live fruit scene synchronized; target obstacle {object_id} "
            f"enabled for transit; perception target {target_id} maps to "
            f"simulation entity {self._prepared_entity_id}"
        )
        return True

    def allow_target_contact(self, target_id: int) -> bool:
        """Remove only the selected fruit before the straight grasp descent."""

        if self._prepared_target_id != target_id:
            self.node.get_logger().error(
                "target contact requested without a matching prepared obstacle"
            )
            return False
        try:
            object_id = set_target_fruit_collision(
                self._planning_scene_monitor,
                self.base_frame,
                target_id,
            )
        except Exception as exc:  # pragma: no cover - ROS integration only
            self.node.get_logger().error(
                f"failed to remove target fruit collision object: {exc}"
            )
            return False
        self._target_contact_open = True
        self.node.get_logger().info(
            f"MoveIt target obstacle {object_id} removed for grasp contact"
        )
        return True

    def restore_target_collision(self, target_id: int) -> bool:
        """Restore the selected fruit sphere at its latest live position."""

        if self._prepared_target_id != target_id:
            self.node.get_logger().error(
                "target collision restoration requested without a matching lifecycle"
            )
            return False
        try:
            try:
                restore_centers = self._fruit_centers_for_planning()
            except Exception as live_exc:
                self.node.get_logger().warning(
                    "live fruit scene unavailable during collision restore; "
                    f"using the prepared fail-closed snapshot: {live_exc}"
                )
                restore_centers = {}
            if target_id in restore_centers:
                restore_center = restore_centers[target_id]
            elif (
                self._prepared_scene_centers_m is not None
                and target_id in self._prepared_scene_centers_m
            ):
                # The overview tracker can lose the fruit while the arm or
                # gripper occludes it.  That is not evidence that the physical
                # fruit disappeared, so restore the fail-closed obstacle at
                # the position captured when this collision lifecycle began.
                restore_center = self._prepared_scene_centers_m[target_id]
            else:
                raise ValueError(
                    f"target {target_id} has no live or prepared restore pose"
                )
            object_id = set_target_fruit_collision(
                self._planning_scene_monitor,
                self.base_frame,
                target_id,
                center_m=restore_center,
                radius_m=self.fruit_collision_radius_m,
            )
        except Exception as exc:  # pragma: no cover - ROS integration only
            self.node.get_logger().error(
                f"failed to restore target fruit collision object: {exc}"
            )
            return False
        self._prepared_target_id = None
        self._prepared_entity_id = None
        self._target_contact_open = False
        self._prepared_scene_centers_m = None
        self.node.get_logger().info(
            f"MoveIt target obstacle {object_id} restored from the live scene"
        )
        return True

    def _wait_until_runtime_ready(self) -> None:
        deadline = time.monotonic() + self.startup_timeout_sec
        while time.monotonic() < deadline:
            with self._planning_scene_monitor.read_only() as scene:
                positions = scene.current_state.get_joint_group_positions(
                    self.planning_group
                )
            # The deterministic Panda startup posture has non-zero joints 2,
            # 4, 6, and 7.  An all-zero vector is MoveIt's pre-message default.
            if len(positions) > 0 and max(
                abs(float(value)) for value in positions
            ) > 0.1:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("MoveIt did not receive a live Panda joint state")

        remaining = max(0.0, deadline - time.monotonic())
        if not self._arm_action_probe.wait_for_server(timeout_sec=remaining):
            raise RuntimeError("Panda arm trajectory action server is unavailable")
        remaining = max(0.0, deadline - time.monotonic())
        if not self._gripper.wait_for_server(timeout_sec=remaining):
            raise RuntimeError("Panda left gripper action server is unavailable")
        remaining = max(0.0, deadline - time.monotonic())
        if not self._gripper_secondary.wait_for_server(timeout_sec=remaining):
            raise RuntimeError("Panda right gripper action server is unavailable")

    @staticmethod
    def _wait_future(future, timeout_sec: float):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_sec):
            return None
        try:
            return future.result()
        except Exception:
            return None

    @staticmethod
    def _execution_succeeded(result) -> bool:
        """Normalize MoveItPy's version-dependent execution return value."""

        if result is None:
            return True
        if isinstance(result, bool):
            return result
        value = getattr(result, "val", None)
        if value is not None:
            # moveit_msgs/MoveItErrorCodes.SUCCESS
            return int(value) == 1
        if hasattr(result, "status"):
            return bool(result)
        return False

    @staticmethod
    def _pose_errors(requested: Pose, observed) -> tuple[float, float]:
        """Return translation and shortest quaternion-angle errors."""

        requested = requested.normalized()
        position = observed.position
        orientation = observed.orientation
        translation_error = math.dist(
            (requested.x, requested.y, requested.z),
            (float(position.x), float(position.y), float(position.z)),
        )
        dot = abs(
            requested.qx * float(orientation.x)
            + requested.qy * float(orientation.y)
            + requested.qz * float(orientation.z)
            + requested.qw * float(orientation.w)
        )
        orientation_error = 2.0 * math.acos(min(1.0, max(0.0, dot)))
        return translation_error, orientation_error

    def _pose_is_within_tolerance(
        self,
        requested: Pose,
        observed,
        label: str,
        *,
        position_tolerance_m: float | None = None,
        orientation_tolerance_rad: float | None = None,
    ) -> bool:
        translation_error, orientation_error = self._pose_errors(
            requested, observed
        )
        position_tolerance_m = (
            self.pose_position_tolerance_m
            if position_tolerance_m is None
            else position_tolerance_m
        )
        orientation_tolerance_rad = (
            self.pose_orientation_tolerance_rad
            if orientation_tolerance_rad is None
            else orientation_tolerance_rad
        )
        self.node.get_logger().info(
            f"{label} pose error: {translation_error:.4f} m, "
            f"{math.degrees(orientation_error):.2f} deg"
        )
        if (
            translation_error > position_tolerance_m
            or orientation_error > orientation_tolerance_rad
        ):
            self.node.get_logger().error(
                f"{label} pose is outside the configured tolerance"
            )
            return False
        return True

    def _pose_message(self, pose: Pose):
        pose = pose.normalized()
        message = self._PoseStamped()
        message.header.frame_id = self.base_frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        message.pose.position.z = pose.z
        message.pose.orientation.x = pose.qx
        message.pose.orientation.y = pose.qy
        message.pose.orientation.z = pose.qz
        message.pose.orientation.w = pose.qw
        return message

    def _wait_until_arm_settled(self) -> bool:
        """Wait for consecutive low-delta joint samples before replanning."""

        deadline = time.monotonic() + self.settle_timeout_sec
        previous = None
        stable_samples = 0
        while time.monotonic() < deadline:
            with self._planning_scene_monitor.read_only() as scene:
                positions = tuple(
                    float(value)
                    for value in scene.current_state.get_joint_group_positions(
                        self.planning_group
                    )
                )
            if previous is not None and len(positions) == len(previous):
                maximum_delta = max(
                    abs(current - prior)
                    for current, prior in zip(positions, previous)
                )
                if maximum_delta <= self.settle_delta_rad:
                    stable_samples += 1
                    if stable_samples >= self.settle_stable_samples:
                        return True
                else:
                    stable_samples = 0
            previous = positions
            time.sleep(self.settle_sample_period_sec)
        self.node.get_logger().error(
            "Panda arm did not settle before the next planning request"
        )
        return False

    def _plan_and_execute(self, *, pose: Pose | None = None, configuration: str | None = None) -> MotionOutcome:
        if not self._wait_until_arm_settled():
            return MotionOutcome(False, 0.0, 0.0)
        self._arm.set_start_state_to_current_state()
        if pose is not None:
            self._arm.set_goal_state(
                pose_stamped_msg=self._pose_message(pose),
                pose_link=self.pose_link,
            )
        elif configuration is not None:
            self._arm.set_goal_state(configuration_name=configuration)
        else:
            raise ValueError("pose or named configuration is required")

        planning_started = time.perf_counter()
        try:
            plan_result = self._arm.plan()
        except Exception as exc:
            self.node.get_logger().error(f"MoveIt planning exception: {exc}")
            return MotionOutcome(False, time.perf_counter() - planning_started, 0.0)
        planning_time = time.perf_counter() - planning_started
        if not plan_result:
            return MotionOutcome(False, planning_time, 0.0)

        if pose is not None:
            trajectory = plan_result.trajectory
            planned_endpoint = trajectory[len(trajectory) - 1].get_pose(
                self.pose_link
            )
            if not self._pose_is_within_tolerance(
                pose, planned_endpoint, "planned endpoint"
            ):
                return MotionOutcome(False, planning_time, 0.0)

        execution_started = time.perf_counter()
        try:
            execution_result = self._moveit.execute(
                plan_result.trajectory, controllers=[]
            )
        except Exception as exc:
            self.node.get_logger().error(f"MoveIt execution exception: {exc}")
            return MotionOutcome(
                False, planning_time, time.perf_counter() - execution_started
            )
        execution_time = time.perf_counter() - execution_started
        execution_succeeded = self._execution_succeeded(execution_result)
        if execution_succeeded and pose is not None:
            if not self._wait_until_arm_settled():
                execution_succeeded = False
            else:
                with self._planning_scene_monitor.read_only() as scene:
                    actual_pose = scene.current_state.get_pose(self.pose_link)
                execution_succeeded = self._pose_is_within_tolerance(
                    pose, actual_pose, "executed endpoint"
                )
        return MotionOutcome(execution_succeeded, planning_time, execution_time)

    def _current_link_pose(self) -> Pose:
        with self._planning_scene_monitor.read_only() as scene:
            message = scene.current_state.get_pose(self.pose_link)
        return Pose(
            x=float(message.position.x),
            y=float(message.position.y),
            z=float(message.position.z),
            qx=float(message.orientation.x),
            qy=float(message.orientation.y),
            qz=float(message.orientation.z),
            qw=float(message.orientation.w),
        ).normalized()

    @staticmethod
    def _interpolate_pose(start: Pose, target: Pose, fraction: float) -> Pose:
        """Linearly interpolate position and nlerp the shortest quaternion arc."""

        if not 0.0 <= fraction <= 1.0:
            raise ValueError("pose interpolation fraction must be within [0, 1]")
        start = start.normalized()
        target = target.normalized()
        dot = (
            start.qx * target.qx
            + start.qy * target.qy
            + start.qz * target.qz
            + start.qw * target.qw
        )
        sign = -1.0 if dot < 0.0 else 1.0
        return Pose(
            x=start.x + fraction * (target.x - start.x),
            y=start.y + fraction * (target.y - start.y),
            z=start.z + fraction * (target.z - start.z),
            qx=(1.0 - fraction) * start.qx + fraction * sign * target.qx,
            qy=(1.0 - fraction) * start.qy + fraction * sign * target.qy,
            qz=(1.0 - fraction) * start.qz + fraction * sign * target.qz,
            qw=(1.0 - fraction) * start.qw + fraction * sign * target.qw,
        ).normalized()

    @staticmethod
    def _orientation_distance(start: Pose, target: Pose) -> float:
        start = start.normalized()
        target = target.normalized()
        dot = abs(
            start.qx * target.qx
            + start.qy * target.qy
            + start.qz * target.qz
            + start.qw * target.qw
        )
        return 2.0 * math.acos(min(1.0, max(0.0, dot)))

    def _dense_pose_waypoints(self, start: Pose, target: Pose) -> tuple[Pose, ...]:
        distance = math.dist(
            (start.x, start.y, start.z), (target.x, target.y, target.z)
        )
        orientation_distance = self._orientation_distance(start, target)
        segment_count = max(
            1,
            math.ceil(distance / self.max_grasp_segment_m),
            math.ceil(
                orientation_distance / self.max_orientation_segment_rad
            ),
        )
        return tuple(
            self._interpolate_pose(start, target, index / segment_count)
            for index in range(1, segment_count + 1)
        )

    def _solve_cartesian_joint_path(
        self, waypoints: tuple[Pose, ...]
    ) -> tuple[tuple[float, ...], tuple[tuple[float, ...], ...], bool, float] | None:
        """Use seeded MoveIt IK and scene checks for a dense pose path."""

        planning_started = time.perf_counter()
        with self._planning_scene_monitor.read_only() as scene:
            state = copy.deepcopy(scene.current_state)
            start_positions = tuple(
                float(value)
                for value in state.get_joint_group_positions(self.planning_group)
            )
            joint_path = []
            previous_positions = start_positions
            for index, waypoint in enumerate(waypoints, start=1):
                solved = state.set_from_ik(
                    self.planning_group,
                    self._pose_message(waypoint).pose,
                    self.pose_link,
                    0.2,
                )
                if not solved:
                    self.node.get_logger().error(
                        f"MoveIt IK failed at Cartesian waypoint {index}/"
                        f"{len(waypoints)}"
                    )
                    return None
                state.update()
                observed = state.get_pose(self.pose_link)
                position_error, orientation_error = self._pose_errors(
                    waypoint, observed
                )
                if (
                    position_error > self.pose_position_tolerance_m
                    or orientation_error > self.pose_orientation_tolerance_rad
                ):
                    self.node.get_logger().error(
                        f"MoveIt IK waypoint {index} exceeds pose tolerance"
                    )
                    return None
                solved_positions = tuple(
                    float(value)
                    for value in state.get_joint_group_positions(
                        self.planning_group
                    )
                )
                maximum_joint_delta = max(
                    abs(current - previous)
                    for current, previous in zip(
                        solved_positions, previous_positions
                    )
                )
                collision_samples = max(
                    1,
                    math.ceil(
                        maximum_joint_delta
                        / self.max_collision_joint_step_rad
                    ),
                )
                for sample_index in range(1, collision_samples + 1):
                    fraction = sample_index / collision_samples
                    sample_positions = tuple(
                        previous
                        + fraction * (current - previous)
                        for current, previous in zip(
                            solved_positions, previous_positions
                        )
                    )
                    state.set_joint_group_positions(
                        self.planning_group, sample_positions
                    )
                    state.update()
                    if not scene.is_state_valid(
                        state, self.planning_group, False
                    ):
                        self.node.get_logger().error(
                            "MoveIt rejected interpolated collision state at "
                            f"Cartesian waypoint {index}/{len(waypoints)}, "
                            f"sample {sample_index}/{collision_samples}"
                        )
                        scene.is_state_valid(
                            state, self.planning_group, True
                        )
                        return (
                            start_positions,
                            tuple(joint_path),
                            True,
                            time.perf_counter() - planning_started,
                        )
                joint_path.append(solved_positions)
                previous_positions = solved_positions
        return (
            start_positions,
            tuple(joint_path),
            False,
            time.perf_counter() - planning_started,
        )

    def _execute_joint_path(
        self,
        start_positions: tuple[float, ...],
        joint_path: tuple[tuple[float, ...], ...],
    ) -> tuple[bool, float]:
        if not joint_path:
            return False, 0.0
        if not self._arm_action_probe.wait_for_server(
            timeout_sec=self.request_timeout_sec
        ):
            self.node.get_logger().error(
                "Panda arm trajectory action server became unavailable"
            )
            return False, 0.0
        goal = self._FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self._arm_joint_names)
        previous = start_positions
        elapsed = 0.0
        for positions in joint_path:
            maximum_delta = max(
                abs(current - prior)
                for current, prior in zip(positions, previous)
            )
            elapsed += max(0.25, maximum_delta / 0.30)
            point = self._JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = self._Duration(seconds=elapsed).to_msg()
            goal.trajectory.points.append(point)
            previous = positions
        execution_started = time.perf_counter()
        goal_handle = self._wait_future(
            self._arm_action_probe.send_goal_async(goal),
            self.request_timeout_sec,
        )
        if goal_handle is None or not goal_handle.accepted:
            return False, time.perf_counter() - execution_started
        wrapped = self._wait_future(
            goal_handle.get_result_async(),
            4.0 * elapsed + self.trajectory_timeout_margin_sec,
        )
        execution_time = time.perf_counter() - execution_started
        if wrapped is None:
            self.node.get_logger().error(
                "Cartesian arm trajectory exceeded its wall-time deadline; "
                "requesting cancellation"
            )
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_future(cancel_future, self.request_timeout_sec)
            return False, execution_time
        error_code = int(wrapped.result.error_code)
        succeeded = error_code == int(
            self._FollowJointTrajectory.Result.SUCCESSFUL
        )
        if not succeeded:
            self.node.get_logger().error(
                f"Cartesian arm trajectory failed with controller code "
                f"{error_code}: {wrapped.result.error_string}"
            )
        return succeeded, execution_time

    def _move_segmented_between(
        self,
        start: Pose,
        target: Pose,
        *,
        intermediate_endpoint: bool = False,
    ) -> MotionOutcome:
        """Execute a collision-checked Cartesian approximation without RRT."""

        planning_time = 0.0
        execution_time = 0.0
        current = start
        for attempt in range(self.cartesian_endpoint_retry_limit + 1):
            waypoints = self._dense_pose_waypoints(current, target)
            solution = self._solve_cartesian_joint_path(waypoints)
            if solution is None:
                return MotionOutcome(False, planning_time, execution_time)
            start_positions, joint_path, collision, attempt_planning = solution
            planning_time += attempt_planning
            if collision:
                return MotionOutcome(
                    False,
                    planning_time,
                    execution_time,
                    collision=True,
                )
            executed, attempt_execution = self._execute_joint_path(
                start_positions, joint_path
            )
            execution_time += attempt_execution
            if not executed or not self._wait_until_arm_settled():
                return MotionOutcome(False, planning_time, execution_time)
            with self._planning_scene_monitor.read_only() as scene:
                actual_pose = scene.current_state.get_pose(self.pose_link)
            succeeded = self._pose_is_within_tolerance(
                target,
                actual_pose,
                "executed Cartesian endpoint",
                position_tolerance_m=(
                    self.intermediate_position_tolerance_m
                    if intermediate_endpoint
                    else None
                ),
                orientation_tolerance_rad=(
                    self.intermediate_orientation_tolerance_rad
                    if intermediate_endpoint
                    else None
                ),
            )
            if succeeded:
                return MotionOutcome(True, planning_time, execution_time)
            if attempt >= self.cartesian_endpoint_retry_limit:
                break
            self.node.get_logger().warning(
                "Cartesian endpoint missed its tolerance; applying one "
                "bounded correction from the measured pose"
            )
            current = self._current_link_pose()
        return MotionOutcome(False, planning_time, execution_time)

    def _move_to_segmented(self, target: Pose) -> MotionOutcome:
        return self._move_segmented_between(self._current_link_pose(), target)

    def _move_guarded_approach(self, target: Pose) -> MotionOutcome:
        """Lift, reorient over the target, then descend without table sweeps."""

        current = self._current_link_pose()
        safe_z = max(current.z, target.z) + self.safe_transit_clearance_m
        waypoints = (
            ("vertical lift", Pose(
                current.x,
                current.y,
                safe_z,
                current.qx,
                current.qy,
                current.qz,
                current.qw,
            )),
            ("in-place reorientation", Pose(
                current.x,
                current.y,
                safe_z,
                target.qx,
                target.qy,
                target.qz,
                target.qw,
            )),
            ("move to clear corridor", Pose(
                current.x,
                self.safe_transit_corridor_y_m,
                safe_z,
                target.qx,
                target.qy,
                target.qz,
                target.qw,
            )),
            ("corridor translation", Pose(
                target.x,
                self.safe_transit_corridor_y_m,
                safe_z,
                target.qx,
                target.qy,
                target.qz,
                target.qw,
            )),
            ("align over target", Pose(
                target.x,
                target.y,
                safe_z,
                target.qx,
                target.qy,
                target.qz,
                target.qw,
            )),
            ("pre-grasp descent", target),
        )
        planning_time = 0.0
        execution_time = 0.0
        for label, waypoint in waypoints:
            self.node.get_logger().info(
                f"MoveIt guarded approach phase: {label}"
            )
            outcome = self._move_segmented_between(
                current,
                waypoint,
                intermediate_endpoint=label != "pre-grasp descent",
            )
            planning_time += outcome.planning_time_sec
            execution_time += outcome.execution_time_sec
            if not outcome.success:
                return MotionOutcome(
                    False,
                    planning_time,
                    execution_time,
                    collision=outcome.collision,
                )
            current = self._current_link_pose()
        return MotionOutcome(True, planning_time, execution_time)

    def _move_guarded_place(self, target: Pose) -> MotionOutcome:
        """Enter the open collection bin vertically instead of through a wall."""

        current = self._current_link_pose()
        safe_z = (
            max(current.z, target.z) + self.place_transit_clearance_m
        )
        waypoints = (
            ("vertical lift", Pose(
                current.x,
                current.y,
                safe_z,
                current.qx,
                current.qy,
                current.qz,
                current.qw,
            )),
            ("in-place reorientation", Pose(
                current.x,
                current.y,
                safe_z,
                target.qx,
                target.qy,
                target.qz,
                target.qw,
            )),
            ("align above bin", Pose(
                target.x,
                target.y,
                safe_z,
                target.qx,
                target.qy,
                target.qz,
                target.qw,
            )),
            ("vertical descent", target),
        )
        planning_time = 0.0
        execution_time = 0.0
        for label, waypoint in waypoints:
            self.node.get_logger().info(
                f"MoveIt guarded place phase: {label}"
            )
            outcome = self._move_segmented_between(
                current,
                waypoint,
                intermediate_endpoint=label != "vertical descent",
            )
            planning_time += outcome.planning_time_sec
            execution_time += outcome.execution_time_sec
            if not outcome.success:
                return MotionOutcome(
                    False,
                    planning_time,
                    execution_time,
                    collision=outcome.collision,
                )
            current = self._current_link_pose()
        return MotionOutcome(True, planning_time, execution_time)

    def _move_to_planned_joint_path(self, pose: Pose) -> MotionOutcome:
        """Plan with MoveIt, then use the startup-verified action client.

        MoveIt's simple controller manager creates its action client only when
        execution starts and can observe a transient not-connected state.  The
        bounded wrist observation motion instead sends the same collision-
        checked joint path through the action client that was required ready
        during backend startup.
        """

        if not self._wait_until_arm_settled():
            return MotionOutcome(False, 0.0, 0.0)
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(
            pose_stamped_msg=self._pose_message(pose),
            pose_link=self.pose_link,
        )
        planning_started = time.perf_counter()
        try:
            plan_result = self._arm.plan()
        except Exception as exc:
            self.node.get_logger().error(f"MoveIt planning exception: {exc}")
            return MotionOutcome(
                False, time.perf_counter() - planning_started, 0.0
            )
        planning_time = time.perf_counter() - planning_started
        if not plan_result:
            return MotionOutcome(False, planning_time, 0.0)
        trajectory = plan_result.trajectory
        waypoint_count = len(trajectory)
        if waypoint_count < 2:
            self.node.get_logger().error(
                "MoveIt observation plan contains fewer than two waypoints"
            )
            return MotionOutcome(False, planning_time, 0.0)
        planned_endpoint = trajectory[waypoint_count - 1].get_pose(
            self.pose_link
        )
        if not self._pose_is_within_tolerance(
            pose, planned_endpoint, "planned endpoint"
        ):
            return MotionOutcome(False, planning_time, 0.0)
        positions = tuple(
            tuple(
                float(value)
                for value in trajectory[index].get_joint_group_positions(
                    self.planning_group
                )
            )
            for index in range(waypoint_count)
        )
        if any(
            len(values) != len(self._arm_joint_names)
            for values in positions
        ):
            self.node.get_logger().error(
                "MoveIt observation plan has an invalid arm state"
            )
            return MotionOutcome(False, planning_time, 0.0)
        executed, execution_time = self._execute_joint_path(
            positions[0], positions[1:]
        )
        if not executed or not self._wait_until_arm_settled():
            return MotionOutcome(False, planning_time, execution_time)
        with self._planning_scene_monitor.read_only() as scene:
            actual_pose = scene.current_state.get_pose(self.pose_link)
        executed = self._pose_is_within_tolerance(
            pose, actual_pose, "executed endpoint"
        )
        return MotionOutcome(
            executed,
            planning_time,
            execution_time,
        )

    def _move_to_named_configuration_direct(
        self, configuration: str
    ) -> MotionOutcome:
        """Plan a named posture and execute it through the bounded arm client.

        Recovery and final-home motions must not depend on MoveItPy's
        controller-manager wait, which can remain blocked after the underlying
        trajectory controller has already reported success. The direct client
        is probed during startup and enforces a wall-time deadline.
        """

        if not self._wait_until_arm_settled():
            return MotionOutcome(False, 0.0, 0.0)
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(configuration_name=configuration)
        planning_started = time.perf_counter()
        try:
            plan_result = self._arm.plan()
        except Exception as exc:
            self.node.get_logger().error(f"MoveIt planning exception: {exc}")
            return MotionOutcome(
                False, time.perf_counter() - planning_started, 0.0
            )
        planning_time = time.perf_counter() - planning_started
        if not plan_result:
            return MotionOutcome(False, planning_time, 0.0)

        trajectory = plan_result.trajectory
        waypoint_count = len(trajectory)
        if waypoint_count < 2:
            self.node.get_logger().error(
                "MoveIt named-configuration plan contains fewer than two "
                "waypoints"
            )
            return MotionOutcome(False, planning_time, 0.0)
        positions = tuple(
            tuple(
                float(value)
                for value in trajectory[index].get_joint_group_positions(
                    self.planning_group
                )
            )
            for index in range(waypoint_count)
        )
        if any(
            len(values) != len(self._arm_joint_names)
            for values in positions
        ):
            self.node.get_logger().error(
                "MoveIt named-configuration plan has an invalid arm state"
            )
            return MotionOutcome(False, planning_time, 0.0)
        executed, execution_time = self._execute_joint_path(
            positions[0], positions[1:]
        )
        if executed:
            executed = self._wait_until_arm_settled()
        return MotionOutcome(executed, planning_time, execution_time)

    def move_to(self, pose: Pose, stage: str) -> MotionOutcome:
        self.node.get_logger().info(f"MoveIt stage {stage}")
        if stage == "WRIST_OBSERVATION":
            return self._move_to_planned_joint_path(pose)
        if stage in {"APPROACH", "APPROACH_RETRY"}:
            return self._move_guarded_approach(pose)
        if stage in {
            "GRASP_POSE",
            "GRASP_RETRY_PREP",
            "GRASP_POSE_RETRY",
            "RETREAT",
        } or stage.startswith(("GRASP_RETRY_PREP_", "GRASP_POSE_RETRY_")):
            return self._move_to_segmented(pose)
        if stage == "PLACE":
            return self._move_guarded_place(pose)
        return self._plan_and_execute(pose=pose)

    def _gripper_command(self, position: float) -> bool:
        grippers = (
            (self.gripper_joint, self._gripper),
            (self.gripper_secondary_joint, self._gripper_secondary),
        )
        for joint_name, client in grippers:
            if not client.wait_for_server(timeout_sec=self.request_timeout_sec):
                self.node.get_logger().error(
                    f"gripper action server is unavailable for {joint_name}"
                )
                return False

        pending_goals = []
        for joint_name, client in grippers:
            goal = self._ParallelGripperCommand.Goal()
            goal.command.name = [joint_name]
            goal.command.position = [float(position)]
            goal.command.effort = [self.max_effort_n]
            pending_goals.append(
                (
                    joint_name,
                    self._wait_future(
                        client.send_goal_async(goal),
                        self.request_timeout_sec,
                    ),
                )
            )

        goal_handles = []
        for joint_name, goal_handle in pending_goals:
            if goal_handle is None or not goal_handle.accepted:
                self.node.get_logger().error(
                    f"gripper goal was rejected for {joint_name}"
                )
                return False
            goal_handles.append((joint_name, goal_handle))

        accepted = True
        for joint_name, goal_handle in goal_handles:
            wrapped = self._wait_future(
                goal_handle.get_result_async(), self.request_timeout_sec
            )
            if wrapped is None:
                self.node.get_logger().error(
                    f"gripper result timed out for {joint_name}"
                )
                accepted = False
                continue
            result = wrapped.result
            state_positions = dict(
                zip(result.state.name, result.state.position, strict=False)
            )
            observed_position = state_positions.get(joint_name)
            observed_source = "controller_result"
            if observed_position is None:
                with self._gripper_state_lock:
                    observed_position = self._latest_gripper_positions_m.get(
                        joint_name
                    )
                observed_source = "joint_states_fallback"
            if observed_position is None:
                self.node.get_logger().error(
                    f"{joint_name} position is absent from both the controller "
                    "result and live joint states"
                )
                accepted = False
                continue
            self.node.get_logger().info(
                f"gripper result ({joint_name}): target_m="
                f"{position:.6f}, observed_m={float(observed_position):.6f}, "
                f"source={observed_source}, "
                f"stalled={bool(result.stalled)}, "
                f"reached_goal={bool(result.reached_goal)}"
            )
            joint_accepted = gripper_result_allows_command(
                target_position_m=position,
                observed_position_m=float(observed_position),
                open_position_m=self.open_width_m,
                closed_position_m=self.closed_width_m,
                stalled=bool(result.stalled),
                reached_goal=bool(result.reached_goal),
                position_tolerance_m=self.gripper_position_tolerance_m,
            )
            if not joint_accepted:
                self.node.get_logger().error(
                    f"gripper result failed measured-position validation for "
                    f"{joint_name}"
                )
                accepted = False
        return accepted

    def close_gripper(self) -> bool:
        if self._gripper_command(self.closed_width_m):
            return True
        self.node.get_logger().warning(
            "gripper close produced no validated travel; resetting the open "
            "command before one bounded retry"
        )
        if not self._gripper_command(self.open_width_m):
            return False
        time.sleep(self.settle_sample_period_sec)
        return self._gripper_command(self.closed_width_m)

    def open_gripper(self) -> bool:
        return self._gripper_command(self.open_width_m)

    def _trigger(self, target_id: int, operation: str) -> bool:
        if target_id <= 0:
            return False
        key = (target_id, operation)
        client = self._service_clients.get(key)
        if client is None:
            topic = f"/strawberry/sim/fruit_{target_id}/{operation}"
            client = self.node.create_client(
                self._Trigger, topic, callback_group=self._callback_group
            )
            self._service_clients[key] = client
        if not client.wait_for_service(timeout_sec=self.request_timeout_sec):
            self.node.get_logger().error(
                f"simulation service unavailable for target {target_id}: {operation}"
            )
            return False
        response = self._wait_future(
            client.call_async(self._Trigger.Request()), self.request_timeout_sec
        )
        if response is None or not response.success:
            message = "timeout" if response is None else response.message
            self.node.get_logger().error(
                f"simulation {operation} failed for target {target_id}: {message}"
            )
            return False
        return True

    def attach(self, target_id: int) -> bool:
        entity_id = getattr(self, "_prepared_entity_id", None) if self._prepared_target_id == target_id else target_id
        entity_id = target_id if entity_id is None else entity_id
        return self._trigger(entity_id, "attach")

    def detach(self, target_id: int) -> bool:
        entity_id = getattr(self, "_prepared_entity_id", None) if self._prepared_target_id == target_id else target_id
        entity_id = target_id if entity_id is None else entity_id
        return self._trigger(entity_id, "detach")

    def fruit_in_bin(self, target_id: int, stable_for_sec: float) -> bool:
        del stable_for_sec  # enforced by strawberry_sim scene configuration
        entity_id = getattr(self, "_prepared_entity_id", None) if self._prepared_target_id == target_id else target_id
        entity_id = target_id if entity_id is None else entity_id
        return self._trigger(entity_id, "verify_in_bin")

    def move_home(self) -> bool:
        return self._move_to_named_configuration_direct(
            self.home_configuration
        ).success

    def shutdown(self) -> bool:
        """Stop MoveItPy's worker thread while retaining the Python object.

        Retaining the object is deliberate on Jazzy MoveItPy 2.12.4 because
        its native destructor has an upstream teardown defect.  The process
        wrapper decides whether to bypass that destructor after this method.
        """

        try:
            self._moveit.shutdown()
            return True
        except Exception as exc:  # pragma: no cover - ROS integration only
            self.node.get_logger().error(f"MoveIt shutdown exception: {exc}")
            return False
