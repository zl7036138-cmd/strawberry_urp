"""ROS/MoveIt implementation of the dependency-light MotionBackend protocol."""

from __future__ import annotations

import copy
from dataclasses import dataclass
import json
import math
import threading
import time
from types import MappingProxyType
from typing import Callable, Mapping

from .core import CONTACT_CLASS_UNAVAILABLE, MotionOutcome, Pose, rotate_about_base_z
from .motion_evidence import MotionEvidence, serialize_joint_feedback, serialize_trajectory
from .moveit_scene import (
    apply_fruit_collision_scene,
    apply_static_collision_scene,
    set_target_fruit_collision,
)
from .scene_geometry import FRUIT_COLLISION_RADIUS_M, STATIC_COLLISION_OBJECTS


PANDA_ARM_JOINT_LIMITS_RAD = (
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
)


def joint_limit_margin_violation(
    positions: tuple[float, ...], margin_rad: float
) -> tuple[int, float, float, float] | None:
    """Return the first Panda arm joint outside conservative soft bounds."""

    if len(positions) != len(PANDA_ARM_JOINT_LIMITS_RAD):
        raise ValueError("Panda arm state must contain seven joint positions")
    margin = float(margin_rad)
    if not math.isfinite(margin) or margin <= 0.0:
        raise ValueError("joint limit margin must be positive")
    for index, (value, bounds) in enumerate(
        zip(positions, PANDA_ARM_JOINT_LIMITS_RAD), start=1
    ):
        lower, upper = bounds
        safe_lower = lower + margin
        safe_upper = upper - margin
        if not math.isfinite(value) or not safe_lower <= value <= safe_upper:
            return index, float(value), safe_lower, safe_upper
    return None


@dataclass(frozen=True)
class ArmStateSnapshot:
    """One complete arm sample published atomically by the joint callback."""

    sequence: int
    positions: tuple[float, ...]
    feedback_json: str
    receipt_monotonic_ns: int

    @property
    def feedback(self) -> dict:
        return json.loads(self.feedback_json)


@dataclass(frozen=True)
class ArmTrajectoryWaitResult:
    """One explicit terminal state for an action result wait."""

    kind: str
    result: object | None = None
    detail: str | None = None


@dataclass(frozen=True)
class PathAssessment:
    """Zero-execution result for one connected, collision-checked path."""

    feasible: bool
    collision: bool
    planning_time_sec: float
    joint_travel_rad: float
    end_joint_positions: tuple[float, ...] = ()
    end_pose: Pose | None = None


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
            abs(observed_position_m - target_position_m) <= position_tolerance_m
        )
    if reached_goal and (
        abs(observed_position_m - target_position_m) <= position_tolerance_m
    ):
        return True
    return stalled and (observed_position_m <= open_position_m - minimum_close_travel_m)


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
        home_joint_positions_rad: tuple[float, ...] | None = None,
        home_joint_tolerance_rad: float = 0.03,
        gripper_action: str = "/panda_gripper_controller/gripper_cmd",
        gripper_secondary_action: str = ("/panda_gripper_right_controller/gripper_cmd"),
        arm_action: str = "/panda_arm_controller/follow_joint_trajectory",
        zero_velocity_band_rad_per_sec: float = 0.05,
        zero_velocity_required_samples: int = 3,
        zero_velocity_sample_period_sec: float = 0.02,
        zero_velocity_timeout_sec: float = 2.0,
        gripper_joint: str = "panda_finger_joint1",
        gripper_secondary_joint: str = "panda_finger_joint2",
        open_width_m: float = 0.04,
        closed_width_m: float = 0.025,
        gripper_position_tolerance_m: float = 0.003,
        max_effort_n: float = 40.0,
        request_timeout_sec: float = 5.0,
        gripper_result_timeout_sec: float = 15.0,
        bin_verification_timeout_sec: float = 15.0,
        startup_timeout_sec: float = 30.0,
        trajectory_timeout_margin_sec: float = 25.0,
        maximum_joint_trajectory_duration_sec: float = 60.0,
        maximum_joint_trajectory_travel_rad: float = 40.0,
        maximum_joint_trajectory_points: int = 512,
        joint_trajectory_velocity_rad_per_sec: float = 0.30,
        grasp_joint_trajectory_velocity_rad_per_sec: float = 0.25,
        home_joint_trajectory_velocity_rad_per_sec: float = 0.08,
        home_joint_trajectory_velocity_limits_rad_per_sec: tuple[
            float, ...
        ] = (0.08, 0.08, 0.08, 0.04, 0.08, 0.08, 0.08),
        home_joint_trajectory_segment_duration_sec: float = 4.0,
        joint_trajectory_start_tolerance_rad: float = 0.05,
        minimum_joint_limit_margin_rad: float = 0.01,
        observation_joint_limit_margin_rad: float = 0.02,
        execution_joint_limit_margin_rad: float = 0.02,
        minimum_joint_waypoint_duration_sec: float = 0.05,
        settle_timeout_sec: float = 1.5,
        settle_window_sec: float = 0.5,
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
        max_joint_edge_step_rad: float = 0.02,
        safe_transit_clearance_m: float = 0.02,
        place_transit_clearance_m: float | None = None,
        safe_transit_corridor_y_m: float = -0.10,
        fruit_obstacles: dict[int, Pose] | None = None,
        fruit_collision_radius_m: float = FRUIT_COLLISION_RADIUS_M,
        static_collision_objects=STATIC_COLLISION_OBJECTS,
        fruit_pose_provider: Callable[[], Mapping[int, tuple[float, float, float]]]
        | None = None,
        dynamic_fruit_manifest: bool = False,
        contact_resolved_attachment: bool = False,
        maximum_cached_collision_scene_age_sec: float = 120.0,
        maximum_cached_target_drift_m: float = 0.05,
        config_dict: dict | None = None,
        evidence_sink: Callable | None = None,
        evidence_run_id: str | None = None,
        evidence_scenario_id: str | None = None,
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
        if (
            not math.isfinite(gripper_result_timeout_sec)
            or gripper_result_timeout_sec <= 0.0
        ):
            raise ValueError("gripper result timeout must be positive")
        if (
            not math.isfinite(bin_verification_timeout_sec)
            or bin_verification_timeout_sec <= 0.0
        ):
            raise ValueError("bin verification timeout must be positive")
        if (
            home_joint_positions_rad is not None
            and len(home_joint_positions_rad) != 7
        ):
            raise ValueError("home joint positions must contain seven values")
        if not math.isfinite(home_joint_tolerance_rad) or home_joint_tolerance_rad <= 0.0:
            raise ValueError("home joint tolerance must be positive")
        if startup_timeout_sec <= 0.0:
            raise ValueError("startup timeout must be positive")
        if trajectory_timeout_margin_sec <= 0.0:
            raise ValueError("trajectory timeout margin must be positive")
        if (
            maximum_joint_trajectory_duration_sec <= 0.0
            or maximum_joint_trajectory_travel_rad <= 0.0
        ):
            raise ValueError("joint trajectory safety limits must be positive")
        if (
            isinstance(maximum_joint_trajectory_points, bool)
            or not isinstance(maximum_joint_trajectory_points, int)
            or maximum_joint_trajectory_points < 2
        ):
            raise ValueError("joint trajectory point limit must be at least two")
        if (
            not math.isfinite(joint_trajectory_velocity_rad_per_sec)
            or joint_trajectory_velocity_rad_per_sec <= 0.0
            or not math.isfinite(grasp_joint_trajectory_velocity_rad_per_sec)
            or grasp_joint_trajectory_velocity_rad_per_sec <= 0.0
            or grasp_joint_trajectory_velocity_rad_per_sec
            > joint_trajectory_velocity_rad_per_sec
            or not math.isfinite(home_joint_trajectory_velocity_rad_per_sec)
            or home_joint_trajectory_velocity_rad_per_sec <= 0.0
            or home_joint_trajectory_velocity_rad_per_sec
            > grasp_joint_trajectory_velocity_rad_per_sec
            or not math.isfinite(home_joint_trajectory_segment_duration_sec)
            or home_joint_trajectory_segment_duration_sec <= 0.0
            or home_joint_trajectory_segment_duration_sec
            > maximum_joint_trajectory_duration_sec
            or not math.isfinite(joint_trajectory_start_tolerance_rad)
            or joint_trajectory_start_tolerance_rad <= 0.0
            or joint_trajectory_start_tolerance_rad > 0.10
            or not math.isfinite(minimum_joint_limit_margin_rad)
            or minimum_joint_limit_margin_rad <= 0.0
            or minimum_joint_limit_margin_rad >= 0.05
            or not math.isfinite(observation_joint_limit_margin_rad)
            or observation_joint_limit_margin_rad < minimum_joint_limit_margin_rad
            or observation_joint_limit_margin_rad >= 0.10
            or not math.isfinite(execution_joint_limit_margin_rad)
            or execution_joint_limit_margin_rad < minimum_joint_limit_margin_rad
            or execution_joint_limit_margin_rad >= 0.10
            or not math.isfinite(minimum_joint_waypoint_duration_sec)
            or minimum_joint_waypoint_duration_sec <= 0.0
        ):
            raise ValueError("joint trajectory timing parameters must be positive")
        if (
            len(home_joint_trajectory_velocity_limits_rad_per_sec) != 7
            or any(
                not math.isfinite(value)
                or value <= 0.0
                or value > grasp_joint_trajectory_velocity_rad_per_sec
                for value in home_joint_trajectory_velocity_limits_rad_per_sec
            )
        ):
            raise ValueError(
                "home joint velocity limits must contain seven positive values"
            )
        if (
            settle_timeout_sec <= 0.0
            or settle_window_sec <= 0.0
            or settle_window_sec >= settle_timeout_sec
            or settle_sample_period_sec <= 0.0
        ):
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
        if (
            not math.isfinite(max_joint_edge_step_rad)
            or max_joint_edge_step_rad <= 0.0
        ):
            raise ValueError("joint edge step cap must be positive")
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
        if (
            not math.isfinite(maximum_cached_collision_scene_age_sec)
            or maximum_cached_collision_scene_age_sec <= 0.0
            or not math.isfinite(maximum_cached_target_drift_m)
            or maximum_cached_target_drift_m <= 0.0
        ):
            raise ValueError("cached collision scene bounds must be positive")
        fruit_manifest = self._build_fruit_manifest(fruit_obstacles or {})

        self.node = node
        evidence_run_id = str(evidence_run_id or "").strip()
        evidence_scenario_id = str(evidence_scenario_id or "").strip()
        if bool(evidence_run_id) != bool(evidence_scenario_id):
            raise ValueError(
                "motion evidence run_id and scenario_id must be configured together"
            )
        self._motion_evidence = (
            MotionEvidence(evidence_sink, run_id=evidence_run_id,
                           scenario_id=evidence_scenario_id,
                           ros_now_ns=self._evidence_ros_now_ns)
            if evidence_sink is not None and evidence_run_id else None
        )
        self._latest_arm_feedback = None
        self._latest_arm_snapshot = None
        self.planning_group = planning_group
        self.pose_link = pose_link
        self.base_frame = base_frame
        self.home_configuration = home_configuration
        self.home_joint_positions_rad = (
            None
            if home_joint_positions_rad is None
            else tuple(float(value) for value in home_joint_positions_rad)
        )
        if self.home_joint_positions_rad is not None and not all(
            math.isfinite(value) for value in self.home_joint_positions_rad
        ):
            raise ValueError("home joint positions must be finite")
        self.home_joint_tolerance_rad = float(home_joint_tolerance_rad)
        self.gripper_joint = gripper_joint
        self.gripper_secondary_joint = gripper_secondary_joint
        self.gripper_joints = (
            self.gripper_joint,
            self.gripper_secondary_joint,
        )
        self.open_width_m = float(open_width_m)
        self.closed_width_m = float(closed_width_m)
        self.gripper_position_tolerance_m = float(gripper_position_tolerance_m)
        self.max_effort_n = float(max_effort_n)
        self.request_timeout_sec = float(request_timeout_sec)
        self.gripper_result_timeout_sec = float(gripper_result_timeout_sec)
        self.bin_verification_timeout_sec = float(bin_verification_timeout_sec)
        self.startup_timeout_sec = float(startup_timeout_sec)
        self.trajectory_timeout_margin_sec = float(trajectory_timeout_margin_sec)
        self.maximum_joint_trajectory_duration_sec = float(
            maximum_joint_trajectory_duration_sec
        )
        self.maximum_joint_trajectory_travel_rad = float(
            maximum_joint_trajectory_travel_rad
        )
        self.maximum_joint_trajectory_points = int(maximum_joint_trajectory_points)
        self.joint_trajectory_velocity_rad_per_sec = float(
            joint_trajectory_velocity_rad_per_sec
        )
        self.grasp_joint_trajectory_velocity_rad_per_sec = float(
            grasp_joint_trajectory_velocity_rad_per_sec
        )
        self.home_joint_trajectory_velocity_rad_per_sec = float(
            home_joint_trajectory_velocity_rad_per_sec
        )
        self.home_joint_trajectory_velocity_limits_rad_per_sec = tuple(
            float(value)
            for value in home_joint_trajectory_velocity_limits_rad_per_sec
        )
        self.home_joint_trajectory_segment_duration_sec = float(
            home_joint_trajectory_segment_duration_sec
        )
        self.joint_trajectory_start_tolerance_rad = float(
            joint_trajectory_start_tolerance_rad
        )
        self.minimum_joint_limit_margin_rad = float(minimum_joint_limit_margin_rad)
        self.observation_joint_limit_margin_rad = float(
            observation_joint_limit_margin_rad
        )
        self.execution_joint_limit_margin_rad = float(
            execution_joint_limit_margin_rad
        )
        self.minimum_joint_waypoint_duration_sec = float(
            minimum_joint_waypoint_duration_sec
        )
        self.settle_timeout_sec = float(settle_timeout_sec)
        self.settle_window_sec = float(settle_window_sec)
        self.settle_sample_period_sec = float(settle_sample_period_sec)
        self.settle_delta_rad = float(settle_delta_rad)
        self.settle_stable_samples = int(settle_stable_samples)
        if zero_velocity_band_rad_per_sec <= 0.0:
            raise ValueError("zero-velocity band must be positive")
        if zero_velocity_required_samples < 1:
            raise ValueError("zero-velocity sample count must be >= 1")
        if zero_velocity_sample_period_sec < 0.0:
            raise ValueError("zero-velocity sample period must be >= 0")
        if zero_velocity_timeout_sec < 0.0:
            raise ValueError("zero-velocity timeout must be >= 0")
        self.zero_velocity_band_rad_per_sec = float(
            zero_velocity_band_rad_per_sec
        )
        self.zero_velocity_required_samples = int(
            zero_velocity_required_samples
        )
        self.zero_velocity_sample_period_sec = float(
            zero_velocity_sample_period_sec
        )
        self.zero_velocity_timeout_sec = float(zero_velocity_timeout_sec)
        self.pose_position_tolerance_m = float(pose_position_tolerance_m)
        self.pose_orientation_tolerance_rad = float(pose_orientation_tolerance_rad)
        self.intermediate_position_tolerance_m = float(
            intermediate_position_tolerance_m
        )
        self.intermediate_orientation_tolerance_rad = float(
            intermediate_orientation_tolerance_rad
        )
        self.cartesian_endpoint_retry_limit = cartesian_endpoint_retry_limit
        self.max_grasp_segment_m = float(max_grasp_segment_m)
        self.max_orientation_segment_rad = float(max_orientation_segment_rad)
        self.max_collision_joint_step_rad = float(max_collision_joint_step_rad)
        self.max_joint_edge_step_rad = float(max_joint_edge_step_rad)
        self.safe_transit_clearance_m = float(safe_transit_clearance_m)
        self.place_transit_clearance_m = float(place_transit_clearance_m)
        self.safe_transit_corridor_y_m = float(safe_transit_corridor_y_m)
        self.fruit_collision_radius_m = float(fruit_collision_radius_m)
        self.static_collision_objects = tuple(static_collision_objects)
        static_collision_ids = [
            specification.object_id for specification in self.static_collision_objects
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
        self.contact_resolved_attachment = bool(contact_resolved_attachment)
        self.maximum_cached_collision_scene_age_sec = float(
            maximum_cached_collision_scene_age_sec
        )
        self.maximum_cached_target_drift_m = float(maximum_cached_target_drift_m)
        self._cached_collision_scene_centers_m: MappingProxyType | None = None
        self._cached_collision_scene_monotonic: float | None = None
        self._locked_collision_scene_target_id: int | None = None
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
        self._latest_arm_positions_rad = {
            joint_name: None for joint_name in self._arm_joint_names
        }
        self._arm_state_sequence = 0
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

    def _evidence_ros_now_ns(self):
        try:
            return self.node.get_clock().now().nanoseconds
        except AttributeError:
            return None

    def _emit_motion_evidence(self, event_type, payload, *, command_id=None):
        evidence = getattr(self, "_motion_evidence", None)
        if evidence is not None:
            try:
                evidence.emit(event_type, payload, command_id=command_id)
            except Exception as exc:
                # Recording is diagnostic: a missing event is an evidence gap,
                # never a reason to report motion success or start a fallback.
                self.node.get_logger().error(f"motion evidence emission failed: {exc}")

    def _on_gripper_joint_state(self, message) -> None:
        receipt_ns = time.monotonic_ns()
        ros_now_ns = self._evidence_ros_now_ns()
        feedback = serialize_joint_feedback(
            message, receipt_monotonic_ns=receipt_ns,
            received_ros_ns=ros_now_ns,
        )
        observed_arm_positions: dict[str, float] = {}
        with self._gripper_state_lock:
            for name, position in zip(message.name, message.position, strict=False):
                joint_name = str(name)
                if joint_name in self.gripper_joints:
                    try:
                        value = float(position)
                    except (TypeError, ValueError, OverflowError):
                        continue
                    if math.isfinite(value):
                        self._latest_gripper_positions_m[joint_name] = value
                if joint_name in self._latest_arm_positions_rad:
                    try:
                        observed_arm_positions[joint_name] = float(position)
                    except (TypeError, ValueError, OverflowError):
                        observed_arm_positions[joint_name] = math.nan
            required_invalid = any(
                issue.startswith(
                    ("acquisition_stamp_ns", "joint_names", "positions")
                )
                for issue in feedback["invalid_fields"]
            )
            if (
                not required_invalid
                and set(observed_arm_positions) == set(self._arm_joint_names)
                and all(math.isfinite(value) for value in observed_arm_positions.values())
            ):
                ordered = tuple(
                    observed_arm_positions[name] for name in self._arm_joint_names
                )
                self._latest_arm_positions_rad.update(observed_arm_positions)
                self._arm_state_sequence += 1
                # Publish one immutable tuple while holding the same lock used
                # for every constituent field. Readers can never mix samples.
                snapshot = ArmStateSnapshot(
                    sequence=int(self._arm_state_sequence),
                    positions=ordered,
                    feedback_json=json.dumps(
                        feedback, allow_nan=False, separators=(",", ":")
                    ),
                    receipt_monotonic_ns=receipt_ns,
                )
                self._latest_arm_snapshot = snapshot
                self._latest_arm_feedback = feedback
                self._latest_arm_complete_monotonic_ns = receipt_ns

    def _latest_live_arm_snapshot(self):
        lock = getattr(self, "_gripper_state_lock", None)
        if lock is None:
            return None
        with lock:
            return getattr(self, "_latest_arm_snapshot", None)

    def _latest_live_arm_positions(self) -> tuple[int, tuple[float, ...]] | None:
        snapshot = self._latest_live_arm_snapshot()
        if snapshot is None:
            return None
        return snapshot.sequence, snapshot.positions

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
                getattr(
                    self, "_dynamic_collision_target_ids", self.fruit_obstacle_centers_m
                )
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

    def _fruit_centers_for_target_lifecycle(
        self, target_id: int, target_pose: Pose
    ) -> MappingProxyType:
        """Return a live scene or a bounded pre-occlusion visual snapshot.

        The fixed arm and plants do not move fruit before a pick. Once the arm
        occludes the base camera, a recent complete visual scene may therefore
        remain the safest available collision model. The selected target must
        still agree with that snapshot, and its current fused visual pose
        replaces the cached centre. No simulator truth is used.
        """

        dynamic_manifest = bool(getattr(self, "dynamic_fruit_manifest", False))
        locked_target = getattr(self, "_locked_collision_scene_target_id", None)
        if dynamic_manifest and locked_target == target_id:
            return self._bounded_cached_collision_scene(target_id, target_pose)

        live_error: Exception | None = None
        live_centers: MappingProxyType | None = None
        try:
            live_centers = self._fruit_centers_for_planning()
        except Exception as exc:
            live_error = exc

        if live_centers is not None and target_id in live_centers:
            if dynamic_manifest:
                self._cached_collision_scene_centers_m = live_centers
                self._cached_collision_scene_monotonic = time.monotonic()
            return live_centers
        if not dynamic_manifest:
            if live_error is not None:
                raise live_error
            raise ValueError(f"target {target_id} is absent from the live fruit scene")

        return self._bounded_cached_collision_scene(
            target_id,
            target_pose,
            live_error=live_error,
        )

    def _bounded_cached_collision_scene(
        self,
        target_id: int,
        target_pose: Pose,
        *,
        live_error: Exception | None = None,
    ) -> MappingProxyType:
        cached = getattr(self, "_cached_collision_scene_centers_m", None)
        captured = getattr(self, "_cached_collision_scene_monotonic", None)
        if cached is None or captured is None or target_id not in cached:
            detail = (
                f": {live_error}" if live_error is not None else ""
            )
            raise ValueError(
                "target is absent and no pre-occlusion visual collision snapshot "
                f"is available{detail}"
            )
        age = time.monotonic() - float(captured)
        maximum_age = float(self.maximum_cached_collision_scene_age_sec)
        if age < 0.0 or age > maximum_age:
            raise ValueError(
                f"pre-occlusion visual collision snapshot is stale by {age:.3f} "
                f"seconds (limit {maximum_age:.3f})"
            )
        cached_target = cached[target_id]
        target_center = (target_pose.x, target_pose.y, target_pose.z)
        drift = math.dist(cached_target, target_center)
        maximum_drift = float(self.maximum_cached_target_drift_m)
        if drift > maximum_drift:
            raise ValueError(
                f"selected target moved {drift:.3f} m from the pre-occlusion "
                f"visual snapshot (limit {maximum_drift:.3f} m)"
            )
        recovered = dict(cached)
        recovered[target_id] = target_center
        self.node.get_logger().warning(
            "using bounded pre-occlusion visual collision snapshot: "
            f"age_sec={age:.3f}, selected_target_drift_m={drift:.3f}, "
            f"obstacle_count={len(recovered)}"
        )
        return MappingProxyType(recovered)

    def lock_pre_observation_collision_scene(
        self, target_id: int, target_pose: Pose
    ) -> bool:
        """Freeze one visual obstacle inventory before eye-in-hand motion.

        The fixed base camera can be partially occluded by the arm while the
        wrist camera approaches a fruit.  Rebuilding the obstacle set from
        those intermediate frames can therefore add transient tracks or drop
        real ones between feasibility checks.  Capture the last complete
        visual inventory before motion and retain it for this target's bounded
        observation/reobservation/pick attempt.  A different target always
        requires a new live capture.
        """

        if not bool(getattr(self, "dynamic_fruit_manifest", False)):
            return True
        locked_target = getattr(self, "_locked_collision_scene_target_id", None)
        if locked_target == target_id:
            try:
                self._bounded_cached_collision_scene(target_id, target_pose)
            except Exception as exc:
                self.node.get_logger().error(
                    f"pre-observation collision scene lock is invalid: {exc}"
                )
                return False
            return True
        try:
            centers = self._fruit_centers_for_planning()
            if target_id not in centers:
                raise ValueError(
                    f"target {target_id} is absent from the live visual scene"
                )
            self._synchronize_fruit_collision_scene(centers)
        except Exception as exc:
            self.node.get_logger().error(
                f"failed to lock pre-observation collision scene: {exc}"
            )
            return False
        self._cached_collision_scene_centers_m = MappingProxyType(dict(centers))
        self._cached_collision_scene_monotonic = time.monotonic()
        self._locked_collision_scene_target_id = int(target_id)
        self.node.get_logger().info(
            "locked pre-observation visual collision scene: "
            f"target_id={target_id}, obstacle_count={len(centers)}"
        )
        return True

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
                maximum_delta = max(
                    abs(right - left) for left, right in zip(previous, current)
                )
                joint_travel += sum(
                    abs(right - left) for left, right in zip(previous, current)
                )
                sample_count = max(
                    1, math.ceil(maximum_delta / self.max_collision_joint_step_rad)
                )
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
            scene_centers = self._fruit_centers_for_target_lifecycle(
                target_id, target_pose
            )
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
        contact_resolved = bool(getattr(self, "contact_resolved_attachment", False))
        self._prepared_entity_id = None if contact_resolved else target_id
        self._target_contact_open = False
        prepared_centers = dict(scene_centers)
        prepared_centers[target_id] = target_center
        self._prepared_scene_centers_m = MappingProxyType(prepared_centers)
        attachment_detail = (
            "physical bilateral contact will resolve the simulation entity"
            if contact_resolved
            else f"simulation entity {self._prepared_entity_id} selected"
        )
        self.node.get_logger().info(
            f"MoveIt live fruit scene synchronized; target obstacle {object_id} "
            f"enabled for transit; {attachment_detail}"
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
            if (
                len(positions) > 0
                and max(abs(float(value)) for value in positions) > 0.1
            ):
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
        translation_error, orientation_error = self._pose_errors(requested, observed)
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

    @staticmethod
    def _joint_path_travel(positions: tuple[tuple[float, ...], ...]) -> float:
        return sum(
            sum(abs(current - previous) for current, previous in zip(right, left))
            for left, right in zip(positions, positions[1:])
        )

    def _plan_joint_path_to_pose(
        self, pose: Pose
    ) -> tuple[tuple[tuple[float, ...], ...], PathAssessment]:
        """Plan one collision-checked path without executing it."""

        if not self._wait_until_arm_settled():
            return (), PathAssessment(False, False, 0.0, 0.0)
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
            return (), PathAssessment(
                False, False, time.perf_counter() - planning_started, 0.0
            )
        planning_time = time.perf_counter() - planning_started
        if not plan_result:
            return (), PathAssessment(False, False, planning_time, 0.0)
        trajectory = plan_result.trajectory
        waypoint_count = len(trajectory)
        if waypoint_count < 2:
            self.node.get_logger().error(
                "MoveIt pose plan contains fewer than two waypoints"
            )
            return (), PathAssessment(False, False, planning_time, 0.0)
        planned_endpoint = trajectory[waypoint_count - 1].get_pose(self.pose_link)
        if not self._pose_is_within_tolerance(
            pose, planned_endpoint, "planned endpoint"
        ):
            return (), PathAssessment(False, False, planning_time, 0.0)
        positions = tuple(
            tuple(
                float(value)
                for value in trajectory[index].get_joint_group_positions(
                    self.planning_group
                )
            )
            for index in range(waypoint_count)
        )
        if any(len(values) != len(self._arm_joint_names) for values in positions):
            self.node.get_logger().error("MoveIt pose plan has an invalid arm state")
            return (), PathAssessment(False, False, planning_time, 0.0)
        if not self._joint_path_within_safety_limits(positions):
            return (), PathAssessment(False, False, planning_time, 0.0)
        return positions, PathAssessment(
            True,
            False,
            planning_time,
            self._joint_path_travel(positions),
            positions[-1],
            Pose(
                x=float(planned_endpoint.position.x),
                y=float(planned_endpoint.position.y),
                z=float(planned_endpoint.position.z),
                qx=float(planned_endpoint.orientation.x),
                qy=float(planned_endpoint.orientation.y),
                qz=float(planned_endpoint.orientation.z),
                qw=float(planned_endpoint.orientation.w),
            ).normalized(),
        )

    def _joint_path_within_safety_limits(
        self,
        positions: tuple[tuple[float, ...], ...],
        *,
        velocity_rad_per_sec: float | None = None,
        joint_velocity_limits_rad_per_sec: tuple[float, ...] | None = None,
    ) -> bool:
        if len(positions) < 2:
            self.node.get_logger().error(
                "joint trajectory contains fewer than two waypoints"
            )
            return False
        if len(positions) > self.maximum_joint_trajectory_points:
            self.node.get_logger().error(
                "joint trajectory rejected: "
                f"{len(positions)} points exceed the configured "
                f"{self.maximum_joint_trajectory_points}-point limit"
            )
            return False
        travel = self._joint_path_travel(positions)
        if travel > self.maximum_joint_trajectory_travel_rad:
            self.node.get_logger().error(
                "joint trajectory rejected: cumulative travel "
                f"{travel:.3f} rad exceeds the configured "
                f"{self.maximum_joint_trajectory_travel_rad:.3f} rad limit"
            )
            return False
        if not self._joint_path_within_limit_margin(positions):
            return False
        duration = self._joint_path_nominal_duration(
            positions,
            velocity_rad_per_sec=velocity_rad_per_sec,
            joint_velocity_limits_rad_per_sec=(
                joint_velocity_limits_rad_per_sec
            ),
        )
        if duration > self.maximum_joint_trajectory_duration_sec:
            self.node.get_logger().error(
                "joint trajectory rejected before execution: nominal "
                f"duration {duration:.3f} s exceeds the configured "
                f"{self.maximum_joint_trajectory_duration_sec:.3f} s limit"
            )
            return False
        return True

    def _joint_path_within_limit_margin(
        self,
        positions: tuple[tuple[float, ...], ...],
        *,
        margin_rad: float | None = None,
    ) -> bool:
        if not all(
            len(values) == len(PANDA_ARM_JOINT_LIMITS_RAD) for values in positions
        ):
            return True
        for waypoint_index, values in enumerate(positions):
            violation = joint_limit_margin_violation(
                values,
                (
                    getattr(self, "minimum_joint_limit_margin_rad", 0.01)
                    if margin_rad is None
                    else float(margin_rad)
                ),
            )
            if violation is not None:
                joint_index, value, lower, upper = violation
                self.node.get_logger().error(
                    "joint trajectory rejected before execution: "
                    f"waypoint {waypoint_index + 1} places panda_joint"
                    f"{joint_index} at {value:.6f} rad outside the "
                    f"conservative [{lower:.6f}, {upper:.6f}] rad bounds"
                )
                return False
        return True

    def _wait_arm_trajectory_result(
        self, future, timeout_sec: float
    ) -> ArmTrajectoryWaitResult:
        """Wait while classifying terminal, timeout, monitor and future errors."""

        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        deadline = time.monotonic() + float(timeout_sec)
        previous_sequence = None
        while True:
            live = self._latest_live_arm_positions()
            if live is not None:
                sequence, positions = live
                if sequence != previous_sequence:
                    previous_sequence = sequence
                    violation = joint_limit_margin_violation(
                        positions,
                        getattr(self, "execution_joint_limit_margin_rad", 0.02),
                    )
                    if violation is not None:
                        joint_index, value, lower, upper = violation
                        detail = (
                            f"panda_joint{joint_index} reached {value:.6f} rad "
                            f"outside [{lower:.6f}, {upper:.6f}] rad"
                        )
                        self.node.get_logger().error(
                            "live arm limit monitor rejected trajectory: " + detail
                        )
                        return ArmTrajectoryWaitResult(
                            "LIVE_JOINT_LIMIT_ABORT", detail=detail
                        )
            if event.is_set():
                try:
                    return ArmTrajectoryWaitResult("TERMINAL", result=future.result())
                except Exception as exc:
                    return ArmTrajectoryWaitResult(
                        "RESULT_FUTURE_ERROR",
                        detail=f"{type(exc).__name__}: {exc}",
                    )
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return ArmTrajectoryWaitResult("WALL_TIMEOUT")
            event.wait(min(0.01, remaining))

    def _subdivide_joint_path_by_step(
        self,
        positions: tuple[tuple[float, ...], ...],
        max_step_rad: float | None = None,
    ) -> tuple[tuple[float, ...], ...]:
        """Insert intermediate points so no edge exceeds a joint-step cap.

        v9 evidence: near the bin-edge singularity one Cartesian segment
        produced a joint5 edge larger than the 0.05 rad controller path
        tolerance; the simulation lagged and the goal aborted (code -4).
        Subdividing such edges keeps every commanded step strictly inside
        the tracking envelope without changing the route or its endpoints.
        """

        if max_step_rad is None:
            max_step_rad = getattr(self, "max_joint_edge_step_rad", None)
        if (
            not isinstance(max_step_rad, (int, float))
            or isinstance(max_step_rad, bool)
            or not math.isfinite(max_step_rad)
            or max_step_rad <= 0.0
        ):
            raise ValueError("joint edge step cap must be positive")
        if len(positions) < 2:
            return positions
        if any(
            len(row) != len(positions[0]) for row in positions
        ):
            return positions
        subdivided: list[tuple[float, ...]] = [tuple(positions[0])]
        for left, right in zip(positions, positions[1:]):
            maximum_step = max(
                abs(a - b) for a, b in zip(right, left)
            )
            count = 1
            if maximum_step > max_step_rad:
                count = int(math.ceil(maximum_step / max_step_rad))
            for index in range(1, count + 1):
                fraction = index / count
                subdivided.append(
                    tuple(
                        a + fraction * (b - a)
                        for a, b in zip(left, right)
                    )
                )
        return tuple(subdivided)

    def _joint_path_nominal_duration(
        self,
        positions: tuple[tuple[float, ...], ...],
        *,
        velocity_rad_per_sec: float | None = None,
        joint_velocity_limits_rad_per_sec: tuple[float, ...] | None = None,
    ) -> float:
        if joint_velocity_limits_rad_per_sec is None:
            velocity = (
                self.joint_trajectory_velocity_rad_per_sec
                if velocity_rad_per_sec is None
                else float(velocity_rad_per_sec)
            )
            if not math.isfinite(velocity) or velocity <= 0.0:
                raise ValueError("joint trajectory velocity must be positive")
            velocity_limits = None
        else:
            if velocity_rad_per_sec is not None:
                raise ValueError(
                    "specify scalar or per-joint velocity limits, not both"
                )
            velocity_limits = tuple(
                float(value) for value in joint_velocity_limits_rad_per_sec
            )
            joint_count = len(positions[0]) if positions else 0
            if (
                joint_count == 0
                or len(velocity_limits) != joint_count
                or any(len(row) != joint_count for row in positions)
                or any(
                    not math.isfinite(value) or value <= 0.0
                    for value in velocity_limits
                )
            ):
                raise ValueError(
                    "per-joint trajectory velocity limits must match the path"
                )
        return sum(
            max(
                self.minimum_joint_waypoint_duration_sec,
                max(
                    abs(current - prior)
                    / (velocity if velocity_limits is None else velocity_limits[index])
                    for index, (current, prior) in enumerate(zip(right, left))
                ),
            )
            for left, right in zip(positions, positions[1:])
        )

    def _partition_joint_path_by_duration(
        self,
        positions: tuple[tuple[float, ...], ...],
        *,
        velocity_rad_per_sec: float | None = None,
        joint_velocity_limits_rad_per_sec: tuple[float, ...] | None = None,
        maximum_segment_duration_sec: float,
    ) -> tuple[tuple[tuple[float, ...], ...], ...]:
        """Split a planned path at existing waypoints without changing its route."""

        if len(positions) < 2:
            raise ValueError("joint path must contain at least two waypoints")
        segments: list[tuple[tuple[float, ...], ...]] = []
        segment_start = 0
        segment_duration = 0.0
        for right_index in range(1, len(positions)):
            edge_duration = self._joint_path_nominal_duration(
                positions[right_index - 1 : right_index + 1],
                velocity_rad_per_sec=velocity_rad_per_sec,
                joint_velocity_limits_rad_per_sec=(
                    joint_velocity_limits_rad_per_sec
                ),
            )
            if edge_duration > maximum_segment_duration_sec + 1.0e-9:
                raise ValueError(
                    "planned joint-path edge exceeds the bounded segment duration"
                )
            if (
                segment_duration > 0.0
                and segment_duration + edge_duration
                > maximum_segment_duration_sec + 1.0e-9
            ):
                segments.append(positions[segment_start:right_index])
                segment_start = right_index - 1
                segment_duration = 0.0
            segment_duration += edge_duration
        segments.append(positions[segment_start:])
        return tuple(segments)

    def _wait_until_arm_settled(self) -> bool:
        """Wait for consecutive fresh, low-delta joint samples before replanning."""

        deadline = time.monotonic() + self.settle_timeout_sec
        anchor = None
        anchor_monotonic = None
        previous_sequence = None
        previous_sample_monotonic = None
        previous_acquisition_ns = None
        anchor_acquisition_ns = None
        stable_samples = 0
        while time.monotonic() < deadline:
            snapshot = self._latest_live_arm_snapshot()
            if snapshot is None:
                # The planning scene may be stale or merely mirror the last
                # commanded state.  It is not an independent physical-stop
                # observation, so never use it to satisfy this gate.
                anchor = None
                anchor_monotonic = None
                stable_samples = 0
                time.sleep(self.settle_sample_period_sec)
                continue
            sequence = snapshot.sequence
            positions = snapshot.positions
            feedback = snapshot.feedback
            complete_receipt_ns = snapshot.receipt_monotonic_ns
            if (
                not positions
                or len(positions) != len(self._arm_joint_names)
                or not all(math.isfinite(value) for value in positions)
            ):
                self.node.get_logger().error(
                    "Panda arm stop confirmation rejected invalid joint feedback"
                )
                return False
            if previous_sequence is not None and sequence <= previous_sequence:
                time.sleep(self.settle_sample_period_sec)
                continue
            previous_sequence = sequence
            acquisition_ns = feedback.get("acquisition_stamp_ns")
            if acquisition_ns is None or complete_receipt_ns is None:
                anchor = None
                anchor_monotonic = None
                anchor_acquisition_ns = None
                stable_samples = 0
                time.sleep(self.settle_sample_period_sec)
                continue
            now_monotonic_ns = time.monotonic_ns()
            ros_now_ns = self._evidence_ros_now_ns()
            receipt_age_ns = now_monotonic_ns - complete_receipt_ns
            acquisition_age_ns = (
                None if ros_now_ns is None else ros_now_ns - acquisition_ns
            )
            freshness_limit_ns = round(self.settle_window_sec * 1e9)
            if (
                receipt_age_ns < 0
                or receipt_age_ns > freshness_limit_ns
                or acquisition_age_ns is None
                or acquisition_age_ns < 0
                or acquisition_age_ns > freshness_limit_ns
            ):
                anchor = None
                anchor_monotonic = None
                anchor_acquisition_ns = None
                stable_samples = 0
                time.sleep(self.settle_sample_period_sec)
                continue
            if previous_acquisition_ns is not None and acquisition_ns <= previous_acquisition_ns:
                self._emit_motion_evidence("STOP_GATE_REJECTED", {
                    "reason": "ACQUISITION_STAMP_NOT_ADVANCING",
                    "sample": feedback,
                    "previous_acquisition_stamp_ns": previous_acquisition_ns,
                })
                self.node.get_logger().error("Panda arm stop rejected repeated or regressing acquisition stamp")
                return False
            previous_acquisition_ns = acquisition_ns
            sample_monotonic = time.monotonic()
            if (
                previous_sample_monotonic is not None
                and sample_monotonic - previous_sample_monotonic
                >= self.settle_window_sec
            ):
                # Time without feedback cannot prove a stable interval.  A
                # gap as long as the required window starts a new observation.
                anchor = None
                anchor_monotonic = None
                stable_samples = 0
            previous_sample_monotonic = sample_monotonic
            if anchor is not None and len(positions) == len(anchor):
                maximum_delta = max(
                    abs(current - initial) for current, initial in zip(positions, anchor)
                )
                if maximum_delta <= self.settle_delta_rad:
                    stable_samples += 1
                    if (
                        stable_samples >= self.settle_stable_samples
                        and anchor_monotonic is not None
                        and sample_monotonic - anchor_monotonic
                        >= self.settle_window_sec
                        and anchor_acquisition_ns is not None
                        and acquisition_ns > anchor_acquisition_ns
                    ):
                        self._emit_motion_evidence("STOP_GATE_POSITION_WINDOW", {
                            "first_acquisition_stamp_ns": anchor_acquisition_ns,
                            "last_sample": feedback, "samples": stable_samples,
                            "window_sec": self.settle_window_sec,
                            "position_delta_rad": self.settle_delta_rad,
                            "independent_stop_evidence_required": True,
                        })
                        return True
                else:
                    anchor = positions
                    anchor_monotonic = sample_monotonic
                    anchor_acquisition_ns = acquisition_ns
                    stable_samples = 1
            else:
                anchor = positions
                anchor_monotonic = sample_monotonic
                anchor_acquisition_ns = acquisition_ns
                stable_samples = 1
            time.sleep(self.settle_sample_period_sec)
        self.node.get_logger().error(
            "Panda arm did not settle before the next planning request"
        )
        return False

    def _plan_and_execute(
        self, *, pose: Pose | None = None, configuration: str | None = None
    ) -> MotionOutcome:
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
            planned_endpoint = trajectory[len(trajectory) - 1].get_pose(self.pose_link)
            if not self._pose_is_within_tolerance(
                pose, planned_endpoint, "planned endpoint"
            ):
                return MotionOutcome(False, planning_time, 0.0)

        evidence = getattr(self, "_motion_evidence", None)
        command_id = None
        if evidence is not None:
            try:
                trajectory_message = plan_result.trajectory.get_robot_trajectory_msg()
                command_id = evidence.start_command("MOVEIT_EXECUTE", {
                    "trajectory": serialize_trajectory(trajectory_message.joint_trajectory),
                    "latest_joint_feedback": getattr(self, "_latest_arm_feedback", None),
                })
            except Exception as exc:
                self.node.get_logger().error(f"motion evidence emission failed: {exc}")
        execution_started = time.perf_counter()
        try:
            execution_result = self._moveit.execute(
                plan_result.trajectory, controllers=[]
            )
        except Exception as exc:
            self.node.get_logger().error(f"MoveIt execution exception: {exc}")
            self._emit_motion_evidence("MOVEIT_EXECUTION_EXCEPTION", {
                "error": str(exc), "physical_state": "UNKNOWN",
            }, command_id=command_id)
            return MotionOutcome(
                False, planning_time, time.perf_counter() - execution_started
            )
        execution_time = time.perf_counter() - execution_started
        execution_succeeded = self._execution_succeeded(execution_result)
        self._emit_motion_evidence("MOVEIT_EXECUTION_RETURNED", {
            "return_type": type(execution_result).__name__,
            "normalized_success": execution_succeeded,
            "action_accepted": None,
            "physical_state": "AWAITING_INDEPENDENT_OBSERVATION",
        }, command_id=command_id)
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

    def _current_joint_positions(self) -> tuple[float, ...]:
        live = self._latest_live_arm_positions()
        if live is not None:
            return live[1]
        with self._planning_scene_monitor.read_only() as scene:
            return tuple(
                float(value)
                for value in scene.current_state.get_joint_group_positions(
                    self.planning_group
                )
            )

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
            math.ceil(orientation_distance / self.max_orientation_segment_rad),
        )
        return tuple(
            self._interpolate_pose(start, target, index / segment_count)
            for index in range(1, segment_count + 1)
        )

    def _solve_cartesian_joint_path(
        self,
        waypoints: tuple[Pose, ...],
        *,
        start_joint_positions: tuple[float, ...] | None = None,
    ) -> tuple[tuple[float, ...], tuple[tuple[float, ...], ...], bool, float] | None:
        """Use seeded MoveIt IK and scene checks for a dense pose path."""

        planning_started = time.perf_counter()
        with self._planning_scene_monitor.read_only() as scene:
            state = copy.deepcopy(scene.current_state)
            current_positions = tuple(
                float(value)
                for value in state.get_joint_group_positions(self.planning_group)
            )
            if start_joint_positions is None:
                start_positions = current_positions
            else:
                start_positions = tuple(float(value) for value in start_joint_positions)
                if len(start_positions) != len(current_positions):
                    raise ValueError(
                        "Cartesian preview start state has the wrong joint count"
                    )
                state.set_joint_group_positions(self.planning_group, start_positions)
                state.update()
                if not scene.is_state_valid(state, self.planning_group, False):
                    self.node.get_logger().error(
                        "MoveIt rejected the Cartesian preview start state"
                    )
                    return (
                        start_positions,
                        (),
                        True,
                        time.perf_counter() - planning_started,
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
                    for value in state.get_joint_group_positions(self.planning_group)
                )
                maximum_joint_delta = max(
                    abs(current - previous)
                    for current, previous in zip(solved_positions, previous_positions)
                )
                collision_samples = max(
                    1,
                    math.ceil(maximum_joint_delta / self.max_collision_joint_step_rad),
                )
                for sample_index in range(1, collision_samples + 1):
                    fraction = sample_index / collision_samples
                    sample_positions = tuple(
                        previous + fraction * (current - previous)
                        for current, previous in zip(
                            solved_positions, previous_positions
                        )
                    )
                    state.set_joint_group_positions(
                        self.planning_group, sample_positions
                    )
                    state.update()
                    if not scene.is_state_valid(state, self.planning_group, False):
                        self.node.get_logger().error(
                            "MoveIt rejected interpolated collision state at "
                            f"Cartesian waypoint {index}/{len(waypoints)}, "
                            f"sample {sample_index}/{collision_samples}"
                        )
                        scene.is_state_valid(state, self.planning_group, True)
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

    def _latest_arm_velocities_rad_per_sec(self) -> tuple[float, ...]:
        """Return the freshest reported joint velocities (fail-closed empty)."""

        feedback = getattr(self, "_latest_arm_feedback", None) or {}
        velocities = feedback.get("velocities") or []
        if len(velocities) != len(self._arm_joint_names):
            return ()
        return tuple(
            float(value) if value is not None else float("nan")
            for value in velocities
        )

    def _wait_for_zero_velocity_between_goals(self) -> bool:
        """Block until reported joint velocities settle near zero.

        v20 receipt: the gz_ros2_control effort path injected velocity kicks
        at goal boundaries and mid-goal; plan A returns the arm to the
        position interface (v10 baseline) and guards the surviving
        goal-boundary kick by refusing to send the next goal until every
        reported joint velocity stays inside a small band for the required
        consecutive samples, bounded by a timeout (fail-closed).
        """

        deadline = time.monotonic() + self.zero_velocity_timeout_sec
        settled = 0
        while time.monotonic() < deadline:
            velocities = self._latest_arm_velocities_rad_per_sec()
            if (
                velocities
                and all(math.isfinite(value) for value in velocities)
                and all(
                    abs(value) <= self.zero_velocity_band_rad_per_sec
                    for value in velocities
                )
            ):
                settled += 1
                if settled >= self.zero_velocity_required_samples:
                    return True
            else:
                settled = 0
            time.sleep(self.zero_velocity_sample_period_sec)
        node = getattr(self, "node", None)
        if node is not None:
            node.get_logger().error(
                "Arm velocities did not settle inside the zero-velocity band "
                f"(+/-{self.zero_velocity_band_rad_per_sec} rad/s) within "
                f"{self.zero_velocity_timeout_sec:.2f} s; withholding the next goal"
            )
        return False

    def _split_arm_goal(
        self,
        joint_names: tuple[str, ...],
        timed_points: tuple[tuple[tuple[float, ...], float], ...],
    ) -> tuple[
        tuple[str, ...],
        tuple[str, ...],
        tuple[tuple[float, ...], float],
        tuple[tuple[float, ...], float],
    ]:
        """Split a 7-joint goal into (j1-j6 arm, j7 wrist) timed points.

        Hybrid control (v19 receipt): joint1-6 run the effort JTC with
        integral gravity compensation; joint7's tiny inertia saturates the
        effort path's velocity clamp regardless of gains, so it runs its own
        position-interface JTC. One action cannot mix interfaces, so every
        goal is split with identical timing on both halves.
        """

        configured = getattr(self, "_arm_joint_names", None)
        if configured is not None and tuple(joint_names) != tuple(configured):
            raise ValueError(
                "arm goal joint names do not match the configured arm joints"
            )
        if not timed_points or any(
            len(positions) != len(joint_names)
            for positions, _ in timed_points
        ):
            raise ValueError(
                "arm goal points do not match the goal joint names"
            )
        if len(joint_names) < 2:
            raise ValueError("arm goal has no wrist joint to split off")
        arm_names = tuple(joint_names[:-1])
        wrist_names = (joint_names[-1],)
        arm_points = tuple(
            (tuple(positions[:-1]), stamp)
            for positions, stamp in timed_points
        )
        wrist_points = tuple(
            ((positions[-1],), stamp)
            for positions, stamp in timed_points
        )
        return arm_names, wrist_names, arm_points, wrist_points

    def _execute_joint_path(
        self,
        start_positions: tuple[float, ...],
        joint_path: tuple[tuple[float, ...], ...],
        *,
        velocity_rad_per_sec: float | None = None,
        joint_velocity_limits_rad_per_sec: tuple[float, ...] | None = None,
    ) -> tuple[bool, float]:
        if not joint_path:
            return False, 0.0
        all_positions = (start_positions,) + tuple(joint_path)
        live = self._latest_live_arm_positions()
        if live is not None:
            _, measured_start = live
            if len(measured_start) != len(start_positions):
                self.node.get_logger().error(
                    "joint trajectory rejected before execution: live arm state "
                    "has the wrong dimension"
                )
                return False, 0.0
            maximum_start_error = max(
                abs(measured - planned)
                for measured, planned in zip(measured_start, start_positions)
            )
            if maximum_start_error > self.joint_trajectory_start_tolerance_rad:
                self.node.get_logger().error(
                    "joint trajectory rejected before execution: planned start "
                    f"differs from fresh live joints by {maximum_start_error:.6f} "
                    "rad, exceeding the configured "
                    f"{self.joint_trajectory_start_tolerance_rad:.6f} rad limit"
                )
                return False, 0.0
        if joint_velocity_limits_rad_per_sec is None:
            velocity = (
                self.joint_trajectory_velocity_rad_per_sec
                if velocity_rad_per_sec is None
                else float(velocity_rad_per_sec)
            )
        else:
            velocity = None
        if not self._joint_path_within_limit_margin(
            all_positions,
            margin_rad=getattr(
                self, "execution_joint_limit_margin_rad", 0.02
            ),
        ):
            return False, 0.0
        if not self._joint_path_within_safety_limits(
            all_positions,
            velocity_rad_per_sec=velocity,
            joint_velocity_limits_rad_per_sec=(
                joint_velocity_limits_rad_per_sec
            ),
        ):
            return False, 0.0
        # Zero-velocity confirmation window (plan A): the position interface
        # still kicks the joint at goal boundaries (v9/v10, v20 receipt), so
        # never send a new goal while any reported joint velocity is outside
        # the settled band. Fail closed: the caller treats False as failure.
        window_timeout = getattr(self, "zero_velocity_timeout_sec", None)
        if (
            isinstance(window_timeout, (int, float))
            and not isinstance(window_timeout, bool)
            and math.isfinite(window_timeout)
            and window_timeout > 0.0
            and not self._wait_for_zero_velocity_between_goals()
        ):
            return False, 0.0
        nominal_duration = self._joint_path_nominal_duration(
            all_positions,
            velocity_rad_per_sec=velocity,
            joint_velocity_limits_rad_per_sec=(
                joint_velocity_limits_rad_per_sec
            ),
        )
        cumulative_travel = self._joint_path_travel(all_positions)
        self.node.get_logger().info(
            "Executing collision-checked joint path: "
            f"{len(all_positions)} points, {cumulative_travel:.3f} rad "
            f"cumulative travel, {nominal_duration:.3f} s nominal duration"
        )
        if not self._arm_action_probe.wait_for_server(
            timeout_sec=self.request_timeout_sec
        ):
            self.node.get_logger().error(
                "Panda arm trajectory action server became unavailable"
            )
            return False, 0.0
        configured_step_cap = getattr(
            self, "max_joint_edge_step_rad", None
        )
        if (
            isinstance(configured_step_cap, (int, float))
            and not isinstance(configured_step_cap, bool)
            and math.isfinite(configured_step_cap)
            and configured_step_cap > 0.0
        ):
            joint_path = self._subdivide_joint_path_by_step(
                tuple(joint_path)
            )
        timed_points = []
        previous = start_positions
        elapsed = 0.0
        for positions in joint_path:
            elapsed += self._joint_path_nominal_duration(
                (previous, positions),
                velocity_rad_per_sec=velocity,
                joint_velocity_limits_rad_per_sec=(
                    joint_velocity_limits_rad_per_sec
                ),
            )
            timed_points.append((tuple(positions), elapsed))
            previous = positions
        goal = self._FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self._arm_joint_names)
        for positions, stamp in timed_points:
            point = self._JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = self._Duration(seconds=stamp).to_msg()
            goal.trajectory.points.append(point)
        evidence = getattr(self, "_motion_evidence", None)
        command_id = None
        if evidence is not None:
            try:
                command_id = evidence.start_command("DIRECT_ARM_TRAJECTORY", {
                    "trajectory": serialize_trajectory(goal.trajectory),
                    "planned_start_positions_rad": list(start_positions),
                    "latest_joint_feedback": getattr(self, "_latest_arm_feedback", None),
                    "nominal_duration_sec": elapsed,
                })
            except Exception as exc:
                self.node.get_logger().error(f"motion evidence emission failed: {exc}")
        execution_started = time.perf_counter()
        goal_handle = self._wait_future(
            self._arm_action_probe.send_goal_async(goal),
            self.request_timeout_sec,
        )
        if goal_handle is None or not goal_handle.accepted:
            self._emit_motion_evidence(
                "ACTION_ACCEPTANCE_UNKNOWN" if goal_handle is None else "ACTION_REJECTED",
                {"physical_state": "UNKNOWN"}, command_id=command_id,
            )
            return False, time.perf_counter() - execution_started
        goal_uuid = getattr(getattr(goal_handle, "goal_id", None), "uuid", None)
        self._emit_motion_evidence("ACTION_ACCEPTED", {
            "goal_uuid": bytes(goal_uuid).hex() if goal_uuid is not None else None,
            "physical_motion_started": None,
        }, command_id=command_id)
        wait_result = self._wait_arm_trajectory_result(
            goal_handle.get_result_async(),
            4.0 * elapsed + self.trajectory_timeout_margin_sec,
        )
        execution_time = time.perf_counter() - execution_started
        if wait_result.kind != "TERMINAL":
            event_type = {
                "LIVE_JOINT_LIMIT_ABORT": "ACTION_LIVE_JOINT_LIMIT_ABORT",
                "RESULT_FUTURE_ERROR": "ACTION_RESULT_FUTURE_ERROR",
                "WALL_TIMEOUT": "ACTION_RESULT_TIMEOUT",
            }[wait_result.kind]
            if wait_result.kind == "WALL_TIMEOUT":
                self.node.get_logger().error(
                    "Cartesian arm trajectory exceeded its wall-time deadline; "
                    "requesting cancellation"
                )
            elif wait_result.kind == "RESULT_FUTURE_ERROR":
                self.node.get_logger().error(
                    "Cartesian arm trajectory result future failed: "
                    f"{wait_result.detail}"
                )
            self._emit_motion_evidence(event_type, {
                "wait_outcome": wait_result.kind,
                "detail": wait_result.detail,
                "physical_state": "UNKNOWN",
            }, command_id=command_id)
            cancel_future = goal_handle.cancel_goal_async()
            cancel_response = self._wait_future(cancel_future, self.request_timeout_sec)
            self._emit_motion_evidence("CANCEL_RESPONSE", {
                "reason": wait_result.kind,
                "response_received": cancel_response is not None,
                "return_code": getattr(cancel_response, "return_code", None),
                "physical_state": "UNKNOWN",
            }, command_id=command_id)
            return False, execution_time
        wrapped = wait_result.result
        error_code = int(wrapped.result.error_code)
        self._emit_motion_evidence("ACTION_TERMINAL", {
            "result_status": getattr(wrapped, "status", None),
            "error_code": error_code, "error_string": wrapped.result.error_string,
            "latest_joint_feedback": getattr(self, "_latest_arm_feedback", None),
            "physical_state": "AWAITING_INDEPENDENT_OBSERVATION",
        }, command_id=command_id)
        succeeded = error_code == int(self._FollowJointTrajectory.Result.SUCCESSFUL)
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
        velocity_rad_per_sec: float | None = None,
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
            execute_kwargs = (
                {}
                if velocity_rad_per_sec is None
                else {"velocity_rad_per_sec": velocity_rad_per_sec}
            )
            executed, attempt_execution = self._execute_joint_path(
                start_positions,
                joint_path,
                **execute_kwargs,
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

    def _move_to_segmented(
        self,
        target: Pose,
        *,
        velocity_rad_per_sec: float | None = None,
    ) -> MotionOutcome:
        return self._move_segmented_between(
            self._current_link_pose(),
            target,
            velocity_rad_per_sec=velocity_rad_per_sec,
        )

    def _guarded_approach_waypoints(
        self, current: Pose, target: Pose
    ) -> tuple[tuple[str, Pose], ...]:
        """Build the one authoritative safe-transit route to pre-grasp."""

        safe_z = max(current.z, target.z) + self.safe_transit_clearance_m
        return (
            (
                "vertical lift",
                Pose(
                    current.x,
                    current.y,
                    safe_z,
                    current.qx,
                    current.qy,
                    current.qz,
                    current.qw,
                ),
            ),
            (
                "in-place reorientation",
                Pose(
                    current.x,
                    current.y,
                    safe_z,
                    target.qx,
                    target.qy,
                    target.qz,
                    target.qw,
                ),
            ),
            (
                "move to clear corridor",
                Pose(
                    current.x,
                    self.safe_transit_corridor_y_m,
                    safe_z,
                    target.qx,
                    target.qy,
                    target.qz,
                    target.qw,
                ),
            ),
            (
                "corridor translation",
                Pose(
                    target.x,
                    self.safe_transit_corridor_y_m,
                    safe_z,
                    target.qx,
                    target.qy,
                    target.qz,
                    target.qw,
                ),
            ),
            (
                "align over target",
                Pose(
                    target.x,
                    target.y,
                    safe_z,
                    target.qx,
                    target.qy,
                    target.qz,
                    target.qw,
                ),
            ),
            ("pre-grasp descent", target),
        )

    def preview_guarded_approach(
        self,
        observation_pose: Pose,
        observation_joint_positions: tuple[float, ...],
        pregrasp_pose: Pose,
    ) -> PathAssessment:
        """Prove the exact guarded approach from a planned observation state.

        This method never commands a controller.  Every Cartesian waypoint is
        solved from the prior hypothetical joint state and collision sampled
        with the same limits used by execution.
        """

        waypoints = tuple(
            pose
            for _label, pose in self._guarded_approach_waypoints(
                observation_pose.normalized(), pregrasp_pose.normalized()
            )
        )
        return self.preview_cartesian_segments(
            observation_pose,
            observation_joint_positions,
            waypoints,
        )

    def preview_guarded_approach_from_current(
        self, pregrasp_pose: Pose
    ) -> PathAssessment:
        return self.preview_guarded_approach(
            self._current_link_pose(),
            self._current_joint_positions(),
            pregrasp_pose,
        )

    def preview_cartesian_segments(
        self,
        start_pose: Pose,
        start_joint_positions: tuple[float, ...],
        target_poses: tuple[Pose, ...],
    ) -> PathAssessment:
        """Preview consecutive segmented motions without controller commands."""

        if not target_poses:
            raise ValueError("Cartesian preview requires at least one target pose")
        current_pose = start_pose.normalized()
        current_joints = tuple(float(value) for value in start_joint_positions)
        planning_time = 0.0
        joint_travel = 0.0
        for waypoint in target_poses:
            waypoint = waypoint.normalized()
            dense_waypoints = self._dense_pose_waypoints(current_pose, waypoint)
            solution = self._solve_cartesian_joint_path(
                dense_waypoints,
                start_joint_positions=current_joints,
            )
            if solution is None:
                return PathAssessment(
                    False, False, planning_time, joint_travel, current_joints
                )
            start_positions, joint_path, collision, elapsed = solution
            planning_time += elapsed
            joint_travel += self._joint_path_travel((start_positions,) + joint_path)
            if collision or not joint_path:
                return PathAssessment(
                    False,
                    collision,
                    planning_time,
                    joint_travel,
                    current_joints,
                )
            if not self._joint_path_within_limit_margin(
                (start_positions,) + joint_path
            ):
                return PathAssessment(
                    False,
                    False,
                    planning_time,
                    joint_travel,
                    current_joints,
                )
            current_joints = joint_path[-1]
            current_pose = waypoint
        return PathAssessment(
            True,
            False,
            planning_time,
            joint_travel,
            current_joints,
        )

    def move_to_observation_if_approach_feasible(
        self, observation_pose: Pose, pregrasp_pose: Pose
    ) -> MotionOutcome:
        """Execute an observation plan only after its approach continuation passes."""

        positions, observation = self._plan_joint_path_to_pose(
            observation_pose.normalized()
        )
        if not observation.feasible:
            return MotionOutcome(
                False,
                observation.planning_time_sec,
                0.0,
                collision=observation.collision,
            )
        if not self._joint_path_within_limit_margin(
            positions,
            margin_rad=getattr(
                self, "observation_joint_limit_margin_rad", 0.02
            ),
        ):
            self.node.get_logger().warning(
                "Wrist observation rejected before motion: its joint path "
                "does not preserve the observation-specific limit margin"
            )
            return MotionOutcome(
                False,
                observation.planning_time_sec,
                0.0,
                collision=False,
            )
        continuation = self.preview_guarded_approach(
            observation.end_pose or observation_pose,
            observation.end_joint_positions,
            pregrasp_pose,
        )
        planning_time = observation.planning_time_sec + continuation.planning_time_sec
        if not continuation.feasible:
            self.node.get_logger().warning(
                "Wrist observation rejected before motion: its planned endpoint "
                "cannot continue through the guarded pre-grasp route"
            )
            return MotionOutcome(
                False,
                planning_time,
                0.0,
                collision=continuation.collision,
            )
        executed, execution_time = self._execute_joint_path(positions[0], positions[1:])
        if not executed or not self._wait_until_arm_settled():
            return MotionOutcome(False, planning_time, execution_time)
        with self._planning_scene_monitor.read_only() as scene:
            actual_pose = scene.current_state.get_pose(self.pose_link)
        executed = self._pose_is_within_tolerance(
            observation_pose, actual_pose, "executed endpoint"
        )
        return MotionOutcome(executed, planning_time, execution_time)

    def _move_guarded_approach(self, target: Pose) -> MotionOutcome:
        """Lift, reorient over the target, then descend without table sweeps."""

        current = self._current_link_pose()
        preview = self.preview_guarded_approach(
            current,
            self._current_joint_positions(),
            target,
        )
        if not preview.feasible:
            self.node.get_logger().warning(
                "Guarded approach rejected before motion because the connected "
                "Cartesian preview failed"
            )
            return MotionOutcome(
                False,
                preview.planning_time_sec,
                0.0,
                collision=preview.collision,
            )
        planning_time = preview.planning_time_sec
        execution_time = 0.0
        for label, waypoint in self._guarded_approach_waypoints(current, target):
            self.node.get_logger().info(f"MoveIt guarded approach phase: {label}")
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

    def _guarded_place_waypoints(
        self, current: Pose, target: Pose
    ) -> tuple[tuple[str, Pose], ...]:
        """Build a connected plant-to-bin route with a clear transit corridor.

        The high lift is used only while leaving the plant.  Keeping that same
        height all the way to the bin can put the arm outside its Cartesian IK
        envelope, so the route lowers to the configured bin clearance only
        after reaching the obstacle-free corridor.
        """

        lift_z = max(current.z, target.z) + self.place_transit_clearance_m
        bin_transit_z = target.z + self.place_transit_clearance_m
        return (
            (
                "vertical lift",
                Pose(
                    current.x,
                    current.y,
                    lift_z,
                    current.qx,
                    current.qy,
                    current.qz,
                    current.qw,
                ),
            ),
            (
                "move to clear corridor",
                Pose(
                    current.x,
                    self.safe_transit_corridor_y_m,
                    lift_z,
                    current.qx,
                    current.qy,
                    current.qz,
                    current.qw,
                ),
            ),
            (
                "corridor reorientation",
                Pose(
                    current.x,
                    self.safe_transit_corridor_y_m,
                    lift_z,
                    target.qx,
                    target.qy,
                    target.qz,
                    target.qw,
                ),
            ),
            (
                "lower to bin transit height",
                Pose(
                    current.x,
                    self.safe_transit_corridor_y_m,
                    bin_transit_z,
                    target.qx,
                    target.qy,
                    target.qz,
                    target.qw,
                ),
            ),
            (
                "corridor translation",
                Pose(
                    target.x,
                    self.safe_transit_corridor_y_m,
                    bin_transit_z,
                    target.qx,
                    target.qy,
                    target.qz,
                    target.qw,
                ),
            ),
            (
                "align above bin",
                Pose(
                    target.x,
                    target.y,
                    bin_transit_z,
                    target.qx,
                    target.qy,
                    target.qz,
                    target.qw,
                ),
            ),
            ("vertical descent", target),
        )

    def preview_guarded_place_from_current(self, target: Pose) -> PathAssessment:
        """Prove the entire place route without sending a controller command."""

        current = self._current_link_pose()
        waypoints = tuple(
            pose for _label, pose in self._guarded_place_waypoints(current, target)
        )
        return self.preview_cartesian_segments(
            current,
            self._current_joint_positions(),
            waypoints,
        )

    @staticmethod
    def _bounded_place_orientation_candidates(target: Pose) -> tuple[Pose, ...]:
        """Keep the downward tool axis while offering four wrist-roll branches."""

        return tuple(
            rotate_about_base_z(target, quarter_turn * math.pi / 2.0)
            for quarter_turn in range(4)
        )

    def _move_guarded_place(self, target: Pose) -> MotionOutcome:
        """Enter the open collection bin vertically instead of through a wall."""

        current = self._current_link_pose()
        selected_target = None
        preview = None
        collision_rejected = False
        planning_time = 0.0
        for orientation_index, candidate in enumerate(
            self._bounded_place_orientation_candidates(target)
        ):
            candidate_preview = self.preview_guarded_place_from_current(candidate)
            planning_time += candidate_preview.planning_time_sec
            collision_rejected = collision_rejected or candidate_preview.collision
            if not candidate_preview.feasible:
                self.node.get_logger().info(
                    "MoveIt guarded place orientation rejected by connected "
                    f"preview: index={orientation_index}"
                )
                continue
            selected_target = candidate
            preview = candidate_preview
            self.node.get_logger().info(
                "MoveIt guarded place selected bounded wrist-roll orientation: "
                f"index={orientation_index}"
            )
            break
        if selected_target is None or preview is None:
            self.node.get_logger().warning(
                "Guarded place rejected before motion because all four "
                "connected Cartesian wrist-roll previews failed"
            )
            return MotionOutcome(
                False,
                planning_time,
                0.0,
                collision=collision_rejected,
            )
        execution_time = 0.0
        waypoints = self._guarded_place_waypoints(current, selected_target)
        for label, waypoint in waypoints:
            self.node.get_logger().info(f"MoveIt guarded place phase: {label}")
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

        positions, assessment = self._plan_joint_path_to_pose(pose)
        if not assessment.feasible:
            return MotionOutcome(
                False,
                assessment.planning_time_sec,
                0.0,
                collision=assessment.collision,
            )
        executed, execution_time = self._execute_joint_path(positions[0], positions[1:])
        if not executed or not self._wait_until_arm_settled():
            return MotionOutcome(False, assessment.planning_time_sec, execution_time)
        with self._planning_scene_monitor.read_only() as scene:
            actual_pose = scene.current_state.get_pose(self.pose_link)
        executed = self._pose_is_within_tolerance(
            pose, actual_pose, "executed endpoint"
        )
        return MotionOutcome(
            executed,
            assessment.planning_time_sec,
            execution_time,
        )

    def _move_to_named_configuration_direct(
        self,
        configuration: str,
        *,
        expected_joint_positions_rad: tuple[float, ...] | None = None,
        velocity_rad_per_sec: float | None = None,
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
            return MotionOutcome(False, time.perf_counter() - planning_started, 0.0)
        planning_time = time.perf_counter() - planning_started
        if not plan_result:
            return MotionOutcome(False, planning_time, 0.0)

        trajectory = plan_result.trajectory
        waypoint_count = len(trajectory)
        if waypoint_count < 2:
            current = self._current_joint_positions()
            expected = (
                self.home_joint_positions_rad
                if expected_joint_positions_rad is None
                else expected_joint_positions_rad
            )
            already_at_configuration = (
                expected is not None
                and len(current) == len(expected)
                and all(
                    abs(observed - target) <= self.home_joint_tolerance_rad
                    for observed, target in zip(current, expected)
                )
            )
            if already_at_configuration:
                self.node.get_logger().info(
                    "MoveIt returned a stationary named-configuration plan; "
                    "live joints independently confirm the requested posture"
                )
                return MotionOutcome(True, planning_time, 0.0)
            self.node.get_logger().error(
                "MoveIt named-configuration plan contains fewer than two "
                "waypoints and live joints do not confirm the requested posture"
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
        if any(len(values) != len(self._arm_joint_names) for values in positions):
            self.node.get_logger().error(
                "MoveIt named-configuration plan has an invalid arm state"
            )
            return MotionOutcome(False, planning_time, 0.0)
        velocity = (
            self.home_joint_trajectory_velocity_rad_per_sec
            if velocity_rad_per_sec is None
            else float(velocity_rad_per_sec)
        )
        home_velocity_limits = (
            getattr(
                self,
                "home_joint_trajectory_velocity_limits_rad_per_sec",
                None,
            )
            if velocity_rad_per_sec is None
            else None
        )
        timing_kwargs = (
            {"joint_velocity_limits_rad_per_sec": home_velocity_limits}
            if home_velocity_limits is not None
            else {"velocity_rad_per_sec": velocity}
        )
        # Validate the complete plan before partitioning so segmentation cannot
        # bypass the global point, travel, duration, or joint-limit bounds.
        if not self._joint_path_within_safety_limits(
            positions,
            **timing_kwargs,
        ):
            return MotionOutcome(False, planning_time, 0.0)
        try:
            segments = self._partition_joint_path_by_duration(
                positions,
                **timing_kwargs,
                maximum_segment_duration_sec=(
                    self.home_joint_trajectory_segment_duration_sec
                ),
            )
        except ValueError as exc:
            self.node.get_logger().error(
                f"named-configuration path cannot be safely segmented: {exc}"
            )
            return MotionOutcome(False, planning_time, 0.0)

        self.node.get_logger().info(
            "Executing named-configuration route in "
            f"{len(segments)} bounded controller segment(s)"
        )
        execution_time = 0.0
        for segment_index, segment in enumerate(segments, start=1):
            executed, segment_execution_time = self._execute_joint_path(
                segment[0],
                segment[1:],
                **timing_kwargs,
            )
            execution_time += segment_execution_time
            if not executed or not self._wait_until_arm_settled():
                self.node.get_logger().error(
                    "named-configuration route stopped at bounded segment "
                    f"{segment_index}/{len(segments)}"
                )
                return MotionOutcome(False, planning_time, execution_time)
            measured = self._current_joint_positions()
            endpoint = segment[-1]
            deviations = tuple(
                abs(observed - expected)
                for observed, expected in zip(measured, endpoint)
            )
            if len(measured) != len(endpoint) or any(
                deviation > self.home_joint_tolerance_rad
                for deviation in deviations
            ):
                deviation_detail = ", ".join(
                    f"joint{index}={deviation:.6f}"
                    for index, deviation in enumerate(deviations, start=1)
                )
                self.node.get_logger().error(
                    "named-configuration route stopped because bounded segment "
                    f"{segment_index}/{len(segments)} did not reach its verified "
                    "joint endpoint; absolute_errors_rad=["
                    f"{deviation_detail}]"
                )
                return MotionOutcome(False, planning_time, execution_time)
        return MotionOutcome(True, planning_time, execution_time)

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
            "CONTACT_RETRY_PREP",
            "CONTACT_RETRY_GRASP",
            "CONTACT_CENTERING_PREP",
            "CONTACT_CENTERING_GRASP",
            "RETREAT",
        } or stage.startswith(("GRASP_RETRY_PREP_", "GRASP_POSE_RETRY_")):
            if stage == "RETREAT":
                return self._move_to_segmented(pose)
            return self._move_to_segmented(
                pose,
                velocity_rad_per_sec=self.grasp_joint_trajectory_velocity_rad_per_sec,
            )
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

        pending_results = [
            (joint_name, goal_handle.get_result_async())
            for joint_name, goal_handle in goal_handles
        ]
        accepted = True
        result_deadline = time.monotonic() + self.gripper_result_timeout_sec
        for joint_name, result_future in pending_results:
            remaining = max(0.0, result_deadline - time.monotonic())
            wrapped = self._wait_future(
                result_future, remaining
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
                    observed_position = self._latest_gripper_positions_m.get(joint_name)
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

    def gripper_centering_offset_m(self) -> float | None:
        """Estimate centering magnitude from the two final finger poses.

        The sign is only a raw joint-space diagnostic. One controller can fail
        to travel while the other finger physically contacts fruit, so the
        executor obtains the correction direction from the pad-contact class.
        """

        with self._gripper_state_lock:
            left = self._latest_gripper_positions_m.get(self.gripper_joint)
            right = self._latest_gripper_positions_m.get(
                self.gripper_secondary_joint
            )
        if left is None or right is None:
            return None
        left = float(left)
        right = float(right)
        if not math.isfinite(left) or not math.isfinite(right):
            return None
        tolerance = self.gripper_position_tolerance_m
        if (
            left < self.closed_width_m - tolerance
            or right < self.closed_width_m - tolerance
            or left > self.open_width_m + tolerance
            or right > self.open_width_m + tolerance
        ):
            return None
        correction = 0.5 * (left - right)
        self.node.get_logger().info(
            "measured grasp centering magnitude from finger asymmetry: "
            f"left_m={left:.6f}, right_m={right:.6f}, "
            f"raw_signed_half_difference_m={correction:.6f}"
        )
        return correction

    def gripper_fruit_contact_class(self) -> str | None:
        """Read identity-free physical fruit contact for centering gating."""

        if not bool(getattr(self, "contact_resolved_attachment", False)):
            return None
        operation = "contact_class"
        key = (0, operation)
        client = self._service_clients.get(key)
        if client is None:
            topic = f"/strawberry/sim/contact_target/{operation}"
            client = self.node.create_client(
                self._Trigger, topic, callback_group=self._callback_group
            )
            self._service_clients[key] = client
        if not client.wait_for_service(timeout_sec=self.request_timeout_sec):
            self.node.get_logger().error(
                "anonymous fruit contact classification service unavailable"
            )
            return CONTACT_CLASS_UNAVAILABLE
        response = self._wait_future(
            client.call_async(self._Trigger.Request()), self.request_timeout_sec
        )
        if response is None or not response.success:
            message = "timeout" if response is None else response.message
            self.node.get_logger().error(
                f"anonymous fruit contact classification failed: {message}"
            )
            return CONTACT_CLASS_UNAVAILABLE
        contact_class = str(response.message).strip()
        self.node.get_logger().info(
            f"anonymous fruit contact classification: {contact_class}"
        )
        return contact_class or CONTACT_CLASS_UNAVAILABLE

    def open_gripper(self) -> bool:
        return self._gripper_command(self.open_width_m)

    def _trigger(
        self,
        target_id: int,
        operation: str,
        *,
        response_timeout_sec: float | None = None,
    ) -> bool:
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
        response_timeout = (
            self.request_timeout_sec
            if response_timeout_sec is None
            else float(response_timeout_sec)
        )
        response = self._wait_future(
            client.call_async(self._Trigger.Request()), response_timeout
        )
        if response is None or not response.success:
            message = "timeout" if response is None else response.message
            self.node.get_logger().error(
                f"simulation {operation} failed for target {target_id}: {message}"
            )
            return False
        return True

    def _trigger_contact_target(
        self, operation: str, *, response_timeout_sec: float | None = None
    ) -> bool:
        key = (0, operation)
        client = self._service_clients.get(key)
        if client is None:
            topic = f"/strawberry/sim/contact_target/{operation}"
            client = self.node.create_client(
                self._Trigger, topic, callback_group=self._callback_group
            )
            self._service_clients[key] = client
        if not client.wait_for_service(timeout_sec=self.request_timeout_sec):
            self.node.get_logger().error(
                f"contact-resolved simulation service unavailable: {operation}"
            )
            return False
        response_timeout = (
            self.request_timeout_sec
            if response_timeout_sec is None
            else float(response_timeout_sec)
        )
        response = self._wait_future(
            client.call_async(self._Trigger.Request()), response_timeout
        )
        if response is None or not response.success:
            message = "timeout" if response is None else response.message
            self.node.get_logger().error(
                f"contact-resolved simulation {operation} failed: {message}"
            )
            return False
        return True

    def attach(self, target_id: int) -> bool:
        if bool(getattr(self, "contact_resolved_attachment", False)):
            return self._trigger_contact_target("attach")
        entity_id = (
            getattr(self, "_prepared_entity_id", None)
            if self._prepared_target_id == target_id
            else target_id
        )
        entity_id = target_id if entity_id is None else entity_id
        return self._trigger(entity_id, "attach")

    def detach(self, target_id: int) -> bool:
        if bool(getattr(self, "contact_resolved_attachment", False)):
            return self._trigger_contact_target("detach")
        entity_id = (
            getattr(self, "_prepared_entity_id", None)
            if self._prepared_target_id == target_id
            else target_id
        )
        entity_id = target_id if entity_id is None else entity_id
        return self._trigger(entity_id, "detach")

    def fruit_in_bin(self, target_id: int, stable_for_sec: float) -> bool:
        del stable_for_sec  # enforced by strawberry_sim scene configuration
        if bool(getattr(self, "contact_resolved_attachment", False)):
            return self._trigger_contact_target(
                "verify_in_bin",
                response_timeout_sec=getattr(
                    self, "bin_verification_timeout_sec", 15.0
                ),
            )
        entity_id = (
            getattr(self, "_prepared_entity_id", None)
            if self._prepared_target_id == target_id
            else target_id
        )
        entity_id = target_id if entity_id is None else entity_id
        return self._trigger(
            entity_id,
            "verify_in_bin",
            response_timeout_sec=getattr(self, "bin_verification_timeout_sec", 15.0),
        )

    def move_home(self) -> bool:
        first = self._move_to_named_configuration_direct(self.home_configuration)
        if first.success:
            return True
        if first.execution_time_sec > 1.0e-6:
            self.node.get_logger().error(
                "home motion failed after controller execution began; "
                "withholding an automatic retry"
            )
            return False
        self.node.get_logger().warning(
            "zero-motion home plan failed; applying one bounded replan from "
            "the unchanged measured state"
        )
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
