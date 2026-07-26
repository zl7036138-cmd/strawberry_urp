"""ROS 2 action server wrapper for the deterministic executor."""

from __future__ import annotations

import os
import signal
import sys
import threading
import time

from .core import FailureCode, PickAndPlaceExecutor, Pose
from .lifecycle import ExclusiveGoalGate, shutdown_executor_and_wait


def _to_pose(message) -> Pose:
    return Pose(
        x=float(message.position.x),
        y=float(message.position.y),
        z=float(message.position.z),
        qx=float(message.orientation.x),
        qy=float(message.orientation.y),
        qz=float(message.orientation.z),
        qw=float(message.orientation.w),
    )


def main(args=None) -> None:  # pragma: no cover - exercised in ROS integration
    try:
        from ament_index_python.packages import get_package_share_directory
        import rclpy
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.action import ActionServer, GoalResponse
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import PoseArray
        from strawberry_interfaces.action import PickAndPlace
        from strawberry_sim.core import load_scene_config
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    from .moveit_backend import MoveItBackend
    from .moveit_config import build_moveit_config

    class PickAndPlaceServer(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_pick_and_place")
            default_scene_config = os.path.join(
                get_package_share_directory("strawberry_sim"),
                "config",
                "scene.yaml",
            )
            self.declare_parameter("planning_group", "panda_arm")
            self.declare_parameter("pose_link", "panda_hand")
            self.declare_parameter("camera_mount", "fixed")
            self.declare_parameter("base_frame", "panda_link0")
            self.declare_parameter("home_configuration", "ready")
            self.declare_parameter(
                "gripper_action", "/panda_gripper_controller/gripper_cmd"
            )
            self.declare_parameter(
                "arm_action", "/panda_arm_controller/follow_joint_trajectory"
            )
            self.declare_parameter("gripper_joint", "panda_finger_joint1")
            # The parallel-gripper controller commands panda_finger_joint1;
            # 0.04 m per finger corresponds to the 0.08 m total opening.
            self.declare_parameter("gripper_open_width_m", 0.04)
            # The pad midpoint is 20 mm above the fruit equator after applying
            # the palm-clearance tool offset.  At that section a 35 mm sphere
            # has a 28.7 mm radius.  A 25 mm command supplies enough simulated
            # over-travel for both rigid contact pads to stall on the surface;
            # the controller accepts that contact stall as a successful close.
            self.declare_parameter("gripper_closed_width_m", 0.025)
            self.declare_parameter("gripper_max_effort_n", 40.0)
            self.declare_parameter("request_timeout_sec", 5.0)
            self.declare_parameter("startup_timeout_sec", 30.0)
            # The trajectory controller may spend up to eight seconds after
            # the nominal timestamp waiting for the actual joints to converge.
            self.declare_parameter("trajectory_timeout_margin_sec", 25.0)
            self.declare_parameter("settle_timeout_sec", 1.5)
            self.declare_parameter("settle_sample_period_sec", 0.05)
            self.declare_parameter("settle_delta_rad", 0.002)
            self.declare_parameter("settle_stable_samples", 3)
            self.declare_parameter("pose_position_tolerance_m", 0.01)
            self.declare_parameter("pose_orientation_tolerance_rad", 0.0872665)
            self.declare_parameter("intermediate_position_tolerance_m", 0.02)
            self.declare_parameter(
                "intermediate_orientation_tolerance_rad", 0.139626
            )
            self.declare_parameter("max_grasp_segment_m", 0.01)
            self.declare_parameter("max_orientation_segment_rad", 0.174533)
            self.declare_parameter("max_collision_joint_step_rad", 0.01)
            self.declare_parameter("safe_transit_clearance_m", 0.02)
            self.declare_parameter("safe_transit_corridor_y_m", -0.10)
            self.declare_parameter("scene_config_file", default_scene_config)
            self.declare_parameter("fruit_collision_radius_m", 0.026)
            self.declare_parameter("ground_truth_pose_timeout_sec", 2.0)
            self.declare_parameter("pregrasp_offset_m", 0.15)
            self.declare_parameter("retreat_distance_m", 0.08)
            self.declare_parameter("bin_stability_sec", 1.0)
            # The pad centre is 85.4 mm from panda_hand, but centring a 70 mm
            # fruit there intersects the hand collision mesh.  A 105.4 mm
            # fruit-centre offset leaves 4.4 mm nominal palm clearance while
            # keeping the equator inside the 54 mm pad length.
            self.declare_parameter("tool_center_offset_m", 0.1054)
            self.declare_parameter("shutdown_timeout_sec", 60.0)
            self.declare_parameter("moveit_teardown_workaround", True)
            scene = load_scene_config(
                str(self.get_parameter("scene_config_file").value)
            )
            base_frame = str(self.get_parameter("base_frame").value)
            if scene.base_frame != base_frame:
                raise RuntimeError(
                    "manipulation base frame does not match the scene manifest"
                )
            fruit_obstacles = {
                fruit.target_id: Pose(
                    fruit.initial_pose.x,
                    fruit.initial_pose.y,
                    fruit.initial_pose.z,
                )
                for fruit in scene.ordered_fruits
            }
            self._fruit_target_ids = tuple(
                fruit.target_id for fruit in scene.ordered_fruits
            )
            self._fruit_pose_lock = threading.Lock()
            self._fruit_pose_centers = None
            self._fruit_pose_received_monotonic = None
            self._ground_truth_pose_timeout_sec = float(
                self.get_parameter("ground_truth_pose_timeout_sec").value
            )
            if self._ground_truth_pose_timeout_sec <= 0.0:
                raise ValueError("ground_truth_pose_timeout_sec must be positive")
            self._fruit_pose_subscription = self.create_subscription(
                PoseArray,
                "/strawberry/ground_truth/poses",
                self._on_ground_truth_poses,
                qos_profile_sensor_data,
            )
            backend = MoveItBackend(
                self,
                planning_group=str(self.get_parameter("planning_group").value),
                pose_link=str(self.get_parameter("pose_link").value),
                base_frame=base_frame,
                home_configuration=str(
                    self.get_parameter("home_configuration").value
                ),
                gripper_action=str(self.get_parameter("gripper_action").value),
                arm_action=str(self.get_parameter("arm_action").value),
                gripper_joint=str(self.get_parameter("gripper_joint").value),
                open_width_m=float(
                    self.get_parameter("gripper_open_width_m").value
                ),
                closed_width_m=float(
                    self.get_parameter("gripper_closed_width_m").value
                ),
                max_effort_n=float(
                    self.get_parameter("gripper_max_effort_n").value
                ),
                request_timeout_sec=float(
                    self.get_parameter("request_timeout_sec").value
                ),
                startup_timeout_sec=float(
                    self.get_parameter("startup_timeout_sec").value
                ),
                trajectory_timeout_margin_sec=float(
                    self.get_parameter("trajectory_timeout_margin_sec").value
                ),
                settle_timeout_sec=float(
                    self.get_parameter("settle_timeout_sec").value
                ),
                settle_sample_period_sec=float(
                    self.get_parameter("settle_sample_period_sec").value
                ),
                settle_delta_rad=float(
                    self.get_parameter("settle_delta_rad").value
                ),
                settle_stable_samples=int(
                    self.get_parameter("settle_stable_samples").value
                ),
                pose_position_tolerance_m=float(
                    self.get_parameter("pose_position_tolerance_m").value
                ),
                pose_orientation_tolerance_rad=float(
                    self.get_parameter("pose_orientation_tolerance_rad").value
                ),
                intermediate_position_tolerance_m=float(
                    self.get_parameter(
                        "intermediate_position_tolerance_m"
                    ).value
                ),
                intermediate_orientation_tolerance_rad=float(
                    self.get_parameter(
                        "intermediate_orientation_tolerance_rad"
                    ).value
                ),
                max_grasp_segment_m=float(
                    self.get_parameter("max_grasp_segment_m").value
                ),
                max_orientation_segment_rad=float(
                    self.get_parameter("max_orientation_segment_rad").value
                ),
                max_collision_joint_step_rad=float(
                    self.get_parameter("max_collision_joint_step_rad").value
                ),
                safe_transit_clearance_m=float(
                    self.get_parameter("safe_transit_clearance_m").value
                ),
                safe_transit_corridor_y_m=float(
                    self.get_parameter("safe_transit_corridor_y_m").value
                ),
                fruit_obstacles=fruit_obstacles,
                fruit_collision_radius_m=float(
                    self.get_parameter("fruit_collision_radius_m").value
                ),
                fruit_pose_provider=self._fruit_pose_snapshot,
                config_dict=build_moveit_config(
                    str(self.get_parameter("camera_mount").value)
                ),
            )
            self._executor_core = PickAndPlaceExecutor(
                backend,
                pregrasp_offset_m=float(
                    self.get_parameter("pregrasp_offset_m").value
                ),
                retreat_distance_m=float(
                    self.get_parameter("retreat_distance_m").value
                ),
                bin_stability_sec=float(
                    self.get_parameter("bin_stability_sec").value
                ),
                tool_center_offset_m=float(
                    self.get_parameter("tool_center_offset_m").value
                ),
            )
            self._goal_gate = ExclusiveGoalGate()
            self.shutdown_timeout_sec = float(
                self.get_parameter("shutdown_timeout_sec").value
            )
            self.moveit_teardown_workaround = bool(
                self.get_parameter("moveit_teardown_workaround").value
            )
            if self.shutdown_timeout_sec <= 0.0:
                raise ValueError("shutdown_timeout_sec must be positive")
            self._callback_group = ReentrantCallbackGroup()
            self._server = ActionServer(
                self,
                PickAndPlace,
                "/strawberry/pick_and_place",
                execute_callback=self._execute,
                goal_callback=self._goal_callback,
                callback_group=self._callback_group,
            )

        def _on_ground_truth_poses(self, message) -> None:
            if message.header.frame_id != str(self.get_parameter("base_frame").value):
                self.get_logger().error(
                    "ignoring fruit truth whose frame differs from manipulation base"
                )
                return
            if len(message.poses) != len(self._fruit_target_ids):
                self.get_logger().error(
                    "ignoring fruit truth whose pose count differs from scene manifest"
                )
                return
            centers = {
                target_id: (
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                )
                for target_id, pose in zip(self._fruit_target_ids, message.poses)
            }
            with self._fruit_pose_lock:
                self._fruit_pose_centers = centers
                self._fruit_pose_received_monotonic = time.monotonic()

        def _fruit_pose_snapshot(self):
            with self._fruit_pose_lock:
                centers = self._fruit_pose_centers
                received = self._fruit_pose_received_monotonic
                if centers is None or received is None:
                    raise RuntimeError("live fruit ground truth is unavailable")
                age = time.monotonic() - received
                if age > self._ground_truth_pose_timeout_sec:
                    raise RuntimeError(
                        f"live fruit ground truth is stale by {age:.3f} seconds"
                    )
                return dict(centers)

        def _goal_callback(self, goal_request):
            del goal_request
            if not self._goal_gate.try_acquire():
                self.get_logger().warning(
                    "Rejecting pick-and-place goal while another goal is active"
                )
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

        def _execute(self, goal_handle):
            try:
                def feedback(stage: str, progress: float) -> None:
                    message = PickAndPlace.Feedback()
                    message.stage = stage
                    message.progress = progress
                    goal_handle.publish_feedback(message)

                outcome = self._executor_core.execute(
                    int(goal_handle.request.target_id),
                    _to_pose(goal_handle.request.target_pose.pose),
                    _to_pose(goal_handle.request.place_pose.pose),
                    feedback,
                )
                result = PickAndPlace.Result()
                result.success = outcome.success
                result.failure_code = int(outcome.failure_code)
                result.planning_time_sec = outcome.planning_time_sec
                result.execution_time_sec = outcome.execution_time_sec
                result.message = outcome.message
                if outcome.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
            finally:
                self._goal_gate.release()

        @property
        def has_active_goal(self) -> bool:
            return self._goal_gate.active

        def wait_until_idle(self) -> bool:
            return self._goal_gate.wait_until_idle(self.shutdown_timeout_sec)

        def shutdown_moveit(self) -> bool:
            return self._executor_core.backend.shutdown()

    rclpy.init(args=args)
    node = PickAndPlaceServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    status = 0
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except BaseException as exc:
        node.get_logger().error(f"action server terminated unexpectedly: {exc}")
        status = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        callbacks_stopped = shutdown_executor_and_wait(
            executor, timeout_sec=node.shutdown_timeout_sec
        )
        goal_stopped = node.wait_until_idle()
        if not callbacks_stopped or not goal_stopped or node.has_active_goal:
            node.get_logger().error(
                "action callback did not stop within the shutdown deadline"
            )
            status = 1
        if not node.shutdown_moveit():
            status = 1
        use_teardown_workaround = node.moveit_teardown_workaround
        node.destroy_node()
        rclpy.try_shutdown()
        sys.stdout.flush()
        sys.stderr.flush()
        if use_teardown_workaround:
            # MoveItPy 2.12.4 can crash in the native destructor even after
            # shutdown(): https://github.com/moveit/moveit2/issues/3721
            # The object is intentionally kept alive until this process exit.
            os._exit(status)

    if status:
        raise SystemExit(status)
