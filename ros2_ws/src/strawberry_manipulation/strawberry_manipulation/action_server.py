"""ROS 2 action server wrapper for the deterministic executor."""

from __future__ import annotations

import math
import json
import os
import signal
import sys
import threading
import time

from .core import (
    PickAndPlaceExecutor,
    Pose,
    bounded_pregrasp_candidates_for_fruit_center,
    hand_pose_for_fruit_center,
    offset_along_local_z,
    rotate_about_base_z,
)
from .candidate_qualification import PlanOnlyCandidateQualifier
from .grasp_candidates import generate_grasp_candidates
from .grasp_geometry import load_grasp_geometry
from .lifecycle import ExclusiveGoalGate, shutdown_executor_and_wait
from .whole_chain import ChainEvaluation, ChainFailureCode


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


class _RuntimeWholeChainCandidateEvaluator:
    """Bind one runtime target/bin context to copied-scene candidate checks."""

    def __init__(self, backend, target_id: int, target_pose: Pose, bin_pose: Pose):
        self._backend = backend
        self._target_id = target_id
        self._target_pose = target_pose
        self._bin_pose = bin_pose

    def evaluate(self, candidate, time_budget_sec: float) -> ChainEvaluation:
        return self._backend.evaluate_nominal_whole_chain(
            target_id=self._target_id,
            target_pose=self._target_pose,
            pregrasp_pose=candidate.pregrasp_pose,
            grasp_pose=candidate.grasp_pose,
            escape_pose=candidate.escape_pose,
            bin_pose=self._bin_pose,
            time_budget_sec=time_budget_sec,
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
        from strawberry_interfaces.msg import (
            GraspCandidateEvaluation,
            TargetPose,
            TrackedTargetArray,
        )
        from strawberry_interfaces.srv import (
            EvaluateTarget,
            MoveToObservation,
            QualifyGraspCandidates,
        )
        from strawberry_sim.core import load_scene_config
        from std_srvs.srv import Trigger
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    from .moveit_backend import MoveItBackend
    from .moveit_config import build_moveit_config
    from .scene_geometry import static_collision_objects

    class PickAndPlaceServer(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_pick_and_place")
            default_scene_config = os.path.join(
                get_package_share_directory("strawberry_sim"),
                "config",
                "scene.yaml",
            )
            default_grasp_geometry = os.path.join(
                get_package_share_directory("strawberry_manipulation"),
                "config",
                "grasp_geometry.yaml",
            )
            self.declare_parameter("scene_config_file", default_scene_config)
            self.declare_parameter("grasp_geometry_config_file", default_grasp_geometry)
            scene = load_scene_config(
                str(self.get_parameter("scene_config_file").value)
            )
            grasp_geometry = load_grasp_geometry(
                str(self.get_parameter("grasp_geometry_config_file").value),
                world_name=scene.world_name,
                fruit_collision_radius_m=scene.fruit_collision_radius_m,
            )
            self.get_logger().info(
                "Loaded grasp geometry profile "
                f"{grasp_geometry.profile_id}: "
                f"tool_center_offset_m={grasp_geometry.tool_center_offset_m}, "
                "gripper_closed_width_m_per_finger="
                f"{grasp_geometry.gripper_closed_width_m_per_finger}"
            )
            self.declare_parameter("planning_group", "panda_arm")
            self.declare_parameter("pose_link", "panda_hand")
            self.declare_parameter("camera_mount", "fixed")
            self.declare_parameter("base_frame", "panda_link0")
            self.declare_parameter("home_configuration", "ready")
            self.declare_parameter(
                "home_joint_positions_rad",
                [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            )
            self.declare_parameter("home_joint_tolerance_rad", 0.03)
            self.declare_parameter(
                "gripper_action", "/panda_gripper_controller/gripper_cmd"
            )
            self.declare_parameter(
                "gripper_secondary_action",
                "/panda_gripper_right_controller/gripper_cmd",
            )
            self.declare_parameter(
                "arm_action", "/panda_arm_controller/follow_joint_trajectory"
            )
            self.declare_parameter("gripper_joint", "panda_finger_joint1")
            self.declare_parameter("gripper_secondary_joint", "panda_finger_joint2")
            # DART does not enforce the Panda mimic constraint. Two
            # single-joint actions therefore receive the same per-finger
            # target; 0.04 m each is the 0.08 m total opening.
            self.declare_parameter(
                "gripper_open_width_m",
                grasp_geometry.gripper_open_width_m_per_finger,
            )
            # Close width is scene-bound: tabletop-v1 retains 25 mm per finger,
            # while the qualified 26 mm Blender-v2 fruit uses 22 mm.
            self.declare_parameter(
                "gripper_closed_width_m",
                grasp_geometry.gripper_closed_width_m_per_finger,
            )
            self.declare_parameter(
                "gripper_position_tolerance_m",
                grasp_geometry.gripper_position_tolerance_m_per_finger,
            )
            self.declare_parameter(
                "maximum_grasp_centering_correction_m",
                0.010,
            )
            self.declare_parameter("gripper_max_effort_n", 40.0)
            self.declare_parameter("request_timeout_sec", 5.0)
            self.declare_parameter("gripper_result_timeout_sec", 15.0)
            self.declare_parameter("bin_verification_timeout_sec", 15.0)
            self.declare_parameter("startup_timeout_sec", 30.0)
            # The trajectory controller may spend up to eight seconds after
            # the nominal timestamp waiting for the actual joints to converge.
            self.declare_parameter("trajectory_timeout_margin_sec", 25.0)
            self.declare_parameter("maximum_joint_trajectory_duration_sec", 60.0)
            self.declare_parameter("maximum_joint_trajectory_travel_rad", 40.0)
            self.declare_parameter("maximum_joint_trajectory_points", 512)
            self.declare_parameter("joint_trajectory_velocity_rad_per_sec", 0.30)
            self.declare_parameter(
                # Development seed 45401 exceeded the controller's 0.05 rad
                # path tolerance by only 1.2-1.4 mrad on the second fruit's
                # final grasp segment. Keep transit speed unchanged and slow
                # only contact-bound Cartesian grasp segments.
                "grasp_joint_trajectory_velocity_rad_per_sec", 0.25
            )
            self.declare_parameter(
                # Only Panda joint 4 exceeded the unchanged controller path
                # tolerance in development. Retain the previously safe 0.08
                # rad/s bound for the other joints and apply the slower value
                # through the per-joint profile below.
                "home_joint_trajectory_velocity_rad_per_sec", 0.08
            )
            self.declare_parameter(
                "home_joint_trajectory_velocity_limits_rad_per_sec",
                [0.08, 0.08, 0.08, 0.04, 0.08, 0.08, 0.08],
            )
            self.declare_parameter(
                # Four seconds retains measured endpoint verification without
                # the 15-segment inactivity timeout observed at three seconds.
                "home_joint_trajectory_segment_duration_sec", 4.0
            )
            self.declare_parameter("joint_trajectory_start_tolerance_rad", 0.05)
            self.declare_parameter("minimum_joint_limit_margin_rad", 0.01)
            self.declare_parameter("observation_joint_limit_margin_rad", 0.02)
            self.declare_parameter("execution_joint_limit_margin_rad", 0.02)
            self.declare_parameter("minimum_joint_waypoint_duration_sec", 0.05)
            self.declare_parameter("settle_timeout_sec", 1.5)
            self.declare_parameter("settle_window_sec", 0.5)
            self.declare_parameter("settle_sample_period_sec", 0.05)
            self.declare_parameter("settle_delta_rad", 0.002)
            self.declare_parameter("settle_stable_samples", 3)
            self.declare_parameter("pose_position_tolerance_m", 0.01)
            self.declare_parameter("pose_orientation_tolerance_rad", 0.0872665)
            self.declare_parameter("intermediate_position_tolerance_m", 0.02)
            self.declare_parameter("intermediate_orientation_tolerance_rad", 0.139626)
            self.declare_parameter("max_grasp_segment_m", 0.01)
            self.declare_parameter("max_orientation_segment_rad", 0.174533)
            self.declare_parameter("max_collision_joint_step_rad", 0.01)
            self.declare_parameter("max_joint_edge_step_rad", 0.02)
            self.declare_parameter("safe_transit_clearance_m", 0.02)
            self.declare_parameter("place_transit_clearance_m", 0.02)
            self.declare_parameter("safe_transit_corridor_y_m", -0.10)
            self.declare_parameter("ground_truth_pose_timeout_sec", 2.0)
            self.declare_parameter("fruit_pose_source", "ground_truth")
            self.declare_parameter(
                "tracked_targets_topic", "/strawberry/tracked_targets"
            )
            self.declare_parameter("tracked_pose_timeout_sec", 0.75)
            self.declare_parameter(
                "maximum_cached_collision_scene_age_sec", 120.0
            )
            self.declare_parameter("maximum_cached_target_drift_m", 0.05)
            self.declare_parameter("target_refinement_topic", "")
            self.declare_parameter("target_refinement_timeout_sec", 3.0)
            self.declare_parameter("target_refinement_max_correction_m", 0.05)
            self.declare_parameter("target_refinement_min_confidence", 0.31)
            self.declare_parameter("target_refinement_max_sigma_m", 0.015)
            self.declare_parameter("pregrasp_offset_m", 0.15)
            self.declare_parameter("retreat_distance_m", 0.08)
            self.declare_parameter("whole_chain_evaluation_timeout_sec", 8.0)
            self.declare_parameter("candidate_search_per_candidate_budget_sec", 2.0)
            self.declare_parameter("candidate_search_total_budget_sec", 12.0)
            self.declare_parameter("whole_chain_virtual_bin_blocker_enabled", False)
            self.declare_parameter("bin_stability_sec", 1.0)
            self.declare_parameter(
                "tool_center_offset_m",
                grasp_geometry.tool_center_offset_m,
            )
            self.declare_parameter("shutdown_timeout_sec", 60.0)
            self.declare_parameter("moveit_teardown_workaround", True)
            self.declare_parameter("motion_evidence_run_id", "")
            self.declare_parameter("motion_evidence_scenario_id", "")
            self._motion_evidence_publisher = self.create_publisher(
                String, "/strawberry/motion_evidence", 100
            )

            def publish_motion_evidence(event):
                message = String()
                message.data = json.dumps(event, allow_nan=False, separators=(",", ":"))
                self._motion_evidence_publisher.publish(message)

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
            fruit_pose_source = (
                str(self.get_parameter("fruit_pose_source").value).strip().lower()
            )
            if fruit_pose_source not in {"ground_truth", "tracked"}:
                raise ValueError("fruit_pose_source must be ground_truth or tracked")
            self._fruit_pose_source = fruit_pose_source
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
            self._fruit_pose_subscription = (
                self.create_subscription(
                    PoseArray,
                    "/strawberry/ground_truth/poses",
                    self._on_ground_truth_poses,
                    qos_profile_sensor_data,
                )
                if fruit_pose_source == "ground_truth"
                else None
            )
            self._tracked_pose_lock = threading.Lock()
            self._tracked_pose_centers = None
            self._tracked_pose_received_monotonic = None
            self._tracked_pose_timeout_sec = float(
                self.get_parameter("tracked_pose_timeout_sec").value
            )
            if self._tracked_pose_timeout_sec <= 0.0:
                raise ValueError("tracked pose timeout must be positive")
            self._tracked_pose_subscription = self.create_subscription(
                TrackedTargetArray,
                str(self.get_parameter("tracked_targets_topic").value),
                self._on_tracked_targets,
                qos_profile_sensor_data,
            )
            target_refinement_topic = str(
                self.get_parameter("target_refinement_topic").value
            ).strip()
            self._target_refinement_lock = threading.Lock()
            self._target_refinement_samples = {}
            self._target_refinement_timeout_sec = float(
                self.get_parameter("target_refinement_timeout_sec").value
            )
            self._target_refinement_max_correction_m = float(
                self.get_parameter("target_refinement_max_correction_m").value
            )
            self._target_refinement_min_confidence = float(
                self.get_parameter("target_refinement_min_confidence").value
            )
            self._target_refinement_max_sigma_m = float(
                self.get_parameter("target_refinement_max_sigma_m").value
            )
            if (
                self._target_refinement_timeout_sec <= 0.0
                or self._target_refinement_max_correction_m <= 0.0
                or not 0.0 <= self._target_refinement_min_confidence <= 1.0
                or self._target_refinement_max_sigma_m <= 0.0
            ):
                raise ValueError("target refinement limits are invalid")
            self._target_refinement_subscription = None
            target_pose_refiner = None
            if target_refinement_topic:
                self._target_refinement_subscription = self.create_subscription(
                    TargetPose,
                    target_refinement_topic,
                    self._on_target_refinement,
                    qos_profile_sensor_data,
                )
                target_pose_refiner = self._refine_target_pose
                self.get_logger().info(
                    f"Near-grasp visual refinement enabled on {target_refinement_topic}"
                )
            backend = MoveItBackend(
                self,
                evidence_sink=publish_motion_evidence,
                evidence_run_id=str(self.get_parameter("motion_evidence_run_id").value),
                evidence_scenario_id=str(self.get_parameter("motion_evidence_scenario_id").value),
                planning_group=str(self.get_parameter("planning_group").value),
                pose_link=str(self.get_parameter("pose_link").value),
                base_frame=base_frame,
                home_configuration=str(self.get_parameter("home_configuration").value),
                home_joint_positions_rad=tuple(
                    float(value)
                    for value in self.get_parameter(
                        "home_joint_positions_rad"
                    ).value
                ),
                home_joint_tolerance_rad=float(
                    self.get_parameter("home_joint_tolerance_rad").value
                ),
                gripper_action=str(self.get_parameter("gripper_action").value),
                gripper_secondary_action=str(
                    self.get_parameter("gripper_secondary_action").value
                ),
                arm_action=str(self.get_parameter("arm_action").value),
                gripper_joint=str(self.get_parameter("gripper_joint").value),
                gripper_secondary_joint=str(
                    self.get_parameter("gripper_secondary_joint").value
                ),
                open_width_m=float(self.get_parameter("gripper_open_width_m").value),
                closed_width_m=float(
                    self.get_parameter("gripper_closed_width_m").value
                ),
                gripper_position_tolerance_m=float(
                    self.get_parameter("gripper_position_tolerance_m").value
                ),
                max_effort_n=float(self.get_parameter("gripper_max_effort_n").value),
                request_timeout_sec=float(
                    self.get_parameter("request_timeout_sec").value
                ),
                gripper_result_timeout_sec=float(
                    self.get_parameter("gripper_result_timeout_sec").value
                ),
                bin_verification_timeout_sec=float(
                    self.get_parameter("bin_verification_timeout_sec").value
                ),
                startup_timeout_sec=float(
                    self.get_parameter("startup_timeout_sec").value
                ),
                trajectory_timeout_margin_sec=float(
                    self.get_parameter("trajectory_timeout_margin_sec").value
                ),
                maximum_joint_trajectory_duration_sec=float(
                    self.get_parameter("maximum_joint_trajectory_duration_sec").value
                ),
                maximum_joint_trajectory_travel_rad=float(
                    self.get_parameter("maximum_joint_trajectory_travel_rad").value
                ),
                maximum_joint_trajectory_points=int(
                    self.get_parameter("maximum_joint_trajectory_points").value
                ),
                joint_trajectory_velocity_rad_per_sec=float(
                    self.get_parameter("joint_trajectory_velocity_rad_per_sec").value
                ),
                grasp_joint_trajectory_velocity_rad_per_sec=float(
                    self.get_parameter(
                        "grasp_joint_trajectory_velocity_rad_per_sec"
                    ).value
                ),
                home_joint_trajectory_velocity_rad_per_sec=float(
                    self.get_parameter(
                        "home_joint_trajectory_velocity_rad_per_sec"
                    ).value
                ),
                home_joint_trajectory_velocity_limits_rad_per_sec=tuple(
                    float(value)
                    for value in self.get_parameter(
                        "home_joint_trajectory_velocity_limits_rad_per_sec"
                    ).value
                ),
                home_joint_trajectory_segment_duration_sec=float(
                    self.get_parameter(
                        "home_joint_trajectory_segment_duration_sec"
                    ).value
                ),
                joint_trajectory_start_tolerance_rad=float(
                    self.get_parameter(
                        "joint_trajectory_start_tolerance_rad"
                    ).value
                ),
                minimum_joint_limit_margin_rad=float(
                    self.get_parameter("minimum_joint_limit_margin_rad").value
                ),
                observation_joint_limit_margin_rad=float(
                    self.get_parameter("observation_joint_limit_margin_rad").value
                ),
                execution_joint_limit_margin_rad=float(
                    self.get_parameter("execution_joint_limit_margin_rad").value
                ),
                minimum_joint_waypoint_duration_sec=float(
                    self.get_parameter("minimum_joint_waypoint_duration_sec").value
                ),
                settle_timeout_sec=float(
                    self.get_parameter("settle_timeout_sec").value
                ),
                settle_window_sec=float(
                    self.get_parameter("settle_window_sec").value
                ),
                settle_sample_period_sec=float(
                    self.get_parameter("settle_sample_period_sec").value
                ),
                settle_delta_rad=float(self.get_parameter("settle_delta_rad").value),
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
                    self.get_parameter("intermediate_position_tolerance_m").value
                ),
                intermediate_orientation_tolerance_rad=float(
                    self.get_parameter("intermediate_orientation_tolerance_rad").value
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
                max_joint_edge_step_rad=float(
                    self.get_parameter("max_joint_edge_step_rad").value
                ),
                safe_transit_clearance_m=float(
                    self.get_parameter("safe_transit_clearance_m").value
                ),
                place_transit_clearance_m=float(
                    self.get_parameter("place_transit_clearance_m").value
                ),
                safe_transit_corridor_y_m=float(
                    self.get_parameter("safe_transit_corridor_y_m").value
                ),
                fruit_obstacles=(
                    fruit_obstacles if fruit_pose_source == "ground_truth" else {}
                ),
                fruit_collision_radius_m=scene.fruit_collision_radius_m,
                static_collision_objects=static_collision_objects(
                    scene.static_collision_profile,
                    plant_positions_m=tuple(
                        plant.position_m for plant in scene.plants
                    ),
                ),
                fruit_pose_provider=(
                    self._fruit_pose_snapshot
                    if fruit_pose_source == "ground_truth"
                    else self._tracked_pose_snapshot
                ),
                dynamic_fruit_manifest=(fruit_pose_source == "tracked"),
                contact_resolved_attachment=(fruit_pose_source == "tracked"),
                carried_position_uncertainty_m=self._target_refinement_max_sigma_m,
                maximum_cached_collision_scene_age_sec=float(
                    self.get_parameter(
                        "maximum_cached_collision_scene_age_sec"
                    ).value
                ),
                maximum_cached_target_drift_m=float(
                    self.get_parameter("maximum_cached_target_drift_m").value
                ),
                config_dict=build_moveit_config(
                    str(self.get_parameter("camera_mount").value)
                ),
                whole_chain_virtual_bin_blocker_enabled=bool(
                    self.get_parameter(
                        "whole_chain_virtual_bin_blocker_enabled"
                    ).value
                ),
            )
            self._backend = backend
            self._executor_core = PickAndPlaceExecutor(
                backend,
                pregrasp_offset_m=float(self.get_parameter("pregrasp_offset_m").value),
                retreat_distance_m=float(
                    self.get_parameter("retreat_distance_m").value
                ),
                bin_stability_sec=float(self.get_parameter("bin_stability_sec").value),
                tool_center_offset_m=float(
                    self.get_parameter("tool_center_offset_m").value
                ),
                target_pose_refiner=target_pose_refiner,
                whole_chain_authorizer=self._authorize_nominal_whole_chain,
                maximum_grasp_centering_correction_m=float(
                    self.get_parameter(
                        "maximum_grasp_centering_correction_m"
                    ).value
                ),
            )
            self._goal_gate = ExclusiveGoalGate()
            self._callback_group = ReentrantCallbackGroup()
            self._observation_service = self.create_service(
                MoveToObservation,
                "/strawberry/move_to_observation",
                self._move_to_observation,
                callback_group=self._callback_group,
            )
            self._evaluation_service = self.create_service(
                EvaluateTarget,
                "/strawberry/evaluate_target",
                self._evaluate_target,
                callback_group=self._callback_group,
            )
            self._candidate_qualification_service = self.create_service(
                QualifyGraspCandidates,
                "/strawberry/qualify_grasp_candidates",
                self._qualify_grasp_candidates,
                callback_group=self._callback_group,
            )
            self._home_service = self.create_service(
                Trigger,
                "/strawberry/move_home",
                self._move_home,
                callback_group=self._callback_group,
            )
            self.shutdown_timeout_sec = float(
                self.get_parameter("shutdown_timeout_sec").value
            )
            self.moveit_teardown_workaround = bool(
                self.get_parameter("moveit_teardown_workaround").value
            )
            if self.shutdown_timeout_sec <= 0.0:
                raise ValueError("shutdown_timeout_sec must be positive")
            self._server = ActionServer(
                self,
                PickAndPlace,
                "/strawberry/pick_and_place",
                execute_callback=self._execute,
                goal_callback=self._goal_callback,
                callback_group=self._callback_group,
            )

        def _move_to_observation(self, request, response):
            base_frame = str(self.get_parameter("base_frame").value)
            if int(request.target_id) <= 0:
                response.success = False
                response.message = "target_id must be positive"
                return response
            if (
                request.target_pose.header.frame_id != base_frame
                or request.observation_pose.header.frame_id != base_frame
            ):
                response.success = False
                response.message = (
                    "target and observation poses are not in the manipulation "
                    "base frame"
                )
                return response
            if not self._goal_gate.try_acquire():
                response.success = False
                response.message = "motion backend is busy"
                return response
            try:
                try:
                    target = _to_pose(request.target_pose.pose).normalized()
                    pose = _to_pose(request.observation_pose.pose).normalized()
                except (TypeError, ValueError) as exc:
                    response.success = False
                    response.message = f"invalid target or observation pose: {exc}"
                    return response
                if not self._backend.lock_pre_observation_collision_scene(
                    int(request.target_id), target
                ):
                    response.success = False
                    response.message = (
                        "failed to lock the pre-observation visual collision scene"
                    )
                    return response
                executor = self._executor_core
                pregrasp_candidates = bounded_pregrasp_candidates_for_fruit_center(
                    target,
                    quaternion=executor.grasp_quaternion,
                    tool_center_offset_m=executor.tool_center_offset_m,
                    pregrasp_offset_m=executor.pregrasp_offset_m,
                )
                outcome = None
                for pregrasp in pregrasp_candidates:
                    outcome = self._backend.move_to_observation_if_approach_feasible(
                        pose, pregrasp
                    )
                    if outcome.success or outcome.execution_time_sec > 1.0e-6:
                        break
                if outcome is None:
                    raise RuntimeError("bounded pregrasp candidate set is empty")
                response.success = bool(outcome.success)
                response.planning_time_sec = float(outcome.planning_time_sec)
                response.execution_time_sec = float(outcome.execution_time_sec)
                response.message = (
                    "wrist observation pose reached with a connected pre-grasp route"
                    if outcome.success
                    else "wrist observation or connected pre-grasp preview failed"
                )
                return response
            finally:
                self._goal_gate.release()

        def _move_home(self, request, response):
            del request
            if not self._goal_gate.try_acquire():
                response.success = False
                response.message = "motion backend is busy"
                return response
            try:
                response.success = bool(self._backend.move_home())
                response.message = (
                    "collision-checked recovery home reached"
                    if response.success
                    else "failed to reach collision-checked recovery home"
                )
                return response
            except Exception as exc:
                response.success = False
                response.message = f"recovery home motion failed: {exc}"
                return response
            finally:
                self._goal_gate.release()

        def _authorize_nominal_whole_chain(
            self, target_id: int, target: Pose, place: Pose
        ):
            """Return the ADR 0086 certificate used before any gripper command."""

            executor = self._executor_core
            try:
                before_fingerprint = self._backend.authorization_scene_fingerprint()
            except Exception as exc:
                return ChainEvaluation(
                    ChainFailureCode.SCENE_INVALID,
                    0.0,
                    0.0,
                    detail=f"cannot snapshot live scene before evaluation: {exc}",
                )
            self._backend.record_whole_chain_evaluation(
                "WHOLE_CHAIN_EVALUATION_STARTED",
                {
                    "target_id": int(target_id),
                    "live_payload_state": executor.payload_state.name,
                    "scene_fingerprint_before": before_fingerprint,
                    "controller_commands_during_evaluation": 0,
                    "gripper_commands_during_evaluation": 0,
                    "physical_attach_during_evaluation": 0,
                },
            )
            try:
                grasp = hand_pose_for_fruit_center(
                    target,
                    quaternion=executor.grasp_quaternion,
                    tool_center_offset_m=executor.tool_center_offset_m,
                )
                pregrasp = bounded_pregrasp_candidates_for_fruit_center(
                    target,
                    quaternion=executor.grasp_quaternion,
                    tool_center_offset_m=executor.tool_center_offset_m,
                    pregrasp_offset_m=executor.pregrasp_offset_m,
                )[0]
                escape = offset_along_local_z(grasp, -executor.retreat_distance_m)
                bin_hand_pose = hand_pose_for_fruit_center(
                    place,
                    quaternion=executor.place_quaternion,
                    tool_center_offset_m=executor.tool_center_offset_m,
                )
                evaluation = self._backend.evaluate_nominal_whole_chain(
                    target_id=target_id,
                    target_pose=target,
                    pregrasp_pose=pregrasp,
                    grasp_pose=grasp,
                    escape_pose=escape,
                    bin_pose=bin_hand_pose,
                    time_budget_sec=float(self.get_parameter(
                        "whole_chain_evaluation_timeout_sec").value),
                )
            except Exception as exc:
                # Preserve a paired evidence receipt even when the virtual
                # planning path itself cannot be constructed or queried.
                evaluation = ChainEvaluation(
                    ChainFailureCode.SCENE_INVALID,
                    0.0,
                    0.0,
                    detail=f"whole-chain evaluator raised: {exc}",
                )
            try:
                after_fingerprint = self._backend.authorization_scene_fingerprint()
            except Exception as exc:
                evaluation = ChainEvaluation(
                    ChainFailureCode.SCENE_INVALID,
                    evaluation.planning_time_sec,
                    evaluation.joint_travel_rad,
                    evaluation.stages,
                    f"cannot snapshot live scene after evaluation: {exc}",
                )
                after_fingerprint = None
            isolated = before_fingerprint == after_fingerprint
            if not isolated and evaluation.feasible:
                evaluation = ChainEvaluation(
                    ChainFailureCode.SCENE_INVALID,
                    evaluation.planning_time_sec,
                    evaluation.joint_travel_rad,
                    evaluation.stages,
                    "live planning-scene fingerprint changed during evaluation",
                )
            self._backend.record_whole_chain_evaluation(
                "WHOLE_CHAIN_EVALUATION_RESULT",
                {
                    "target_id": int(target_id),
                    "result": evaluation.code.value,
                    "feasible": evaluation.feasible,
                    "planning_time_sec": evaluation.planning_time_sec,
                    "joint_travel_rad": evaluation.joint_travel_rad,
                    "stages": list(evaluation.stages),
                    "detail": evaluation.detail,
                    "live_payload_state": executor.payload_state.name,
                    "scene_fingerprint_before": before_fingerprint,
                    "scene_fingerprint_after": after_fingerprint,
                    "scene_isolated": isolated,
                    "controller_commands_during_evaluation": 0,
                    "gripper_commands_during_evaluation": 0,
                    "physical_attach_during_evaluation": 0,
                },
            )
            return evaluation

        @staticmethod
        def _initialize_candidate_qualification_response(response) -> None:
            response.feasible = False
            response.plan_only = True
            response.scene_isolated = False
            response.cleanup_succeeded = False
            response.status = ""
            response.selected_candidate_id = ""
            response.geometry_fingerprint = ""
            response.certificate_fingerprint = ""
            response.scene_signature = ""
            response.planning_time_sec = 0.0
            response.joint_travel_rad = 0.0
            response.evaluations = []
            response.message = ""

        @staticmethod
        def _copy_candidate_qualification_response(
            response, qualification, *, cleanup_succeeded: bool
        ) -> None:
            certificate = qualification.certificate
            response.plan_only = True
            response.scene_isolated = bool(qualification.scene_isolated)
            response.cleanup_succeeded = bool(cleanup_succeeded)
            response.feasible = bool(qualification.feasible and cleanup_succeeded)
            response.status = (
                qualification.status.value
                if cleanup_succeeded
                else "CLEANUP_FAILED"
            )
            response.selected_candidate_id = (
                "" if certificate is None else certificate.candidate.candidate_id
            )
            response.geometry_fingerprint = (
                "" if certificate is None else certificate.candidate.geometry_fingerprint
            )
            response.certificate_fingerprint = (
                "" if certificate is None else certificate.certificate_fingerprint
            )
            response.scene_signature = qualification.scene_signature_before or ""
            response.planning_time_sec = float(qualification.planning_time_sec)
            response.joint_travel_rad = float(qualification.joint_travel_rad)
            evaluations = []
            for trace in qualification.evaluations:
                item = GraspCandidateEvaluation()
                item.candidate_id = trace.candidate_id
                item.geometry_fingerprint = trace.geometry_fingerprint
                item.result = trace.result.code.value
                item.feasible = trace.result.feasible
                item.allocated_budget_sec = float(trace.allocated_budget_sec)
                item.planning_time_sec = float(trace.result.planning_time_sec)
                item.joint_travel_rad = float(trace.result.joint_travel_rad)
                item.stages = list(trace.result.stages)
                item.detail = trace.result.detail
                evaluations.append(item)
            response.evaluations = evaluations
            response.message = qualification.detail
            if not cleanup_succeeded:
                response.message = (
                    f"{response.message}; " if response.message else ""
                ) + "failed to restore target collision object after plan-only qualification"

        def _qualify_grasp_candidates(self, request, response):
            """Expose ADR 0087-D without dispatching a pick or any trajectory."""

            self._initialize_candidate_qualification_response(response)
            base_frame = str(self.get_parameter("base_frame").value)
            if (
                int(request.target_id) <= 0
                or request.target_pose.header.frame_id != base_frame
                or request.place_pose.header.frame_id != base_frame
            ):
                response.status = "REQUEST_INVALID"
                response.message = (
                    "target ID and target/place poses must be valid in the "
                    "manipulation base frame"
                )
                return response
            if not self._goal_gate.try_acquire():
                response.status = "BACKEND_BUSY"
                response.message = "motion backend is busy"
                return response

            target_id = int(request.target_id)
            prepared = False
            cleanup_succeeded = False
            qualification = None
            try:
                target = _to_pose(request.target_pose.pose).normalized()
                place = _to_pose(request.place_pose.pose).normalized()
                executor = self._executor_core
                if executor.payload_state.name != "EMPTY":
                    response.status = "PAYLOAD_NOT_EMPTY"
                    response.message = (
                        "plan-only candidate qualification requires an EMPTY payload"
                    )
                    self._backend.record_whole_chain_evaluation(
                        "CANDIDATE_QUALIFICATION_REJECTED",
                        {
                            "target_id": target_id,
                            "mode": "PLAN_ONLY",
                            "live_payload_state": executor.payload_state.name,
                            "reason": "PAYLOAD_NOT_EMPTY",
                            "execution_dispatched": False,
                        },
                    )
                elif not self._backend.prepare_pick(target_id, target):
                    response.status = "SETUP_FAILED"
                    response.message = (
                        "failed to synchronize target collision scene for "
                        "plan-only qualification"
                    )
                    self._backend.record_whole_chain_evaluation(
                        "CANDIDATE_QUALIFICATION_SETUP_FAILED",
                        {
                            "target_id": target_id,
                            "mode": "PLAN_ONLY",
                            "reason": "PREPARE_PICK_FAILED",
                            "execution_dispatched": False,
                        },
                    )
                else:
                    prepared = True
                    candidates = generate_grasp_candidates(
                        target,
                        nominal_quaternion=executor.grasp_quaternion,
                        tool_center_offset_m=executor.tool_center_offset_m,
                        pregrasp_offset_m=executor.pregrasp_offset_m,
                        escape_offset_m=executor.retreat_distance_m,
                    )
                    bin_hand_pose = hand_pose_for_fruit_center(
                        place,
                        quaternion=executor.place_quaternion,
                        tool_center_offset_m=executor.tool_center_offset_m,
                    )
                    qualification = PlanOnlyCandidateQualifier(
                        evaluator=_RuntimeWholeChainCandidateEvaluator(
                            self._backend, target_id, target, bin_hand_pose
                        ),
                        scene_signature_provider=(
                            self._backend.authorization_scene_fingerprint
                        ),
                        per_candidate_budget_sec=float(
                            self.get_parameter(
                                "candidate_search_per_candidate_budget_sec"
                            ).value
                        ),
                        total_budget_sec=float(
                            self.get_parameter(
                                "candidate_search_total_budget_sec"
                            ).value
                        ),
                        event_sink=self._backend.record_whole_chain_evaluation,
                    ).qualify(
                        target_id=target_id,
                        candidates=candidates,
                        payload_state=executor.payload_state.name,
                    )
            except Exception as exc:
                response.status = "EVALUATION_ERROR"
                response.message = f"plan-only candidate qualification failed: {exc}"
                self._backend.record_whole_chain_evaluation(
                    "CANDIDATE_QUALIFICATION_ERROR",
                    {
                        "target_id": target_id,
                        "mode": "PLAN_ONLY",
                        "reason": str(exc),
                        "execution_dispatched": False,
                    },
                )
            finally:
                if prepared:
                    try:
                        cleanup_succeeded = bool(
                            self._backend.restore_target_collision(target_id)
                        )
                    except Exception as exc:
                        cleanup_succeeded = False
                        self.get_logger().error(
                            "candidate qualification collision cleanup raised: "
                            f"{exc}"
                        )
                    if qualification is not None:
                        self._backend.record_whole_chain_evaluation(
                            "CANDIDATE_QUALIFICATION_CLEANUP",
                            {
                                "target_id": target_id,
                                "mode": "PLAN_ONLY",
                                "qualification_status": qualification.status.value,
                                "target_collision_restored": cleanup_succeeded,
                                "execution_dispatched": False,
                            },
                        )
                        self._copy_candidate_qualification_response(
                            response,
                            qualification,
                            cleanup_succeeded=cleanup_succeeded,
                        )
                    elif not cleanup_succeeded:
                        response.status = "CLEANUP_FAILED"
                        response.message = (
                            f"{response.message}; " if response.message else ""
                        ) + "failed to restore target collision object"
                self._goal_gate.release()
            return response

        def _evaluate_target(self, request, response):
            base_frame = str(self.get_parameter("base_frame").value)
            if (
                int(request.target_id) <= 0
                or request.target_pose.header.frame_id != base_frame
            ):
                response.feasible = False
                response.message = "target evaluation request is invalid"
                return response
            if not self._goal_gate.try_acquire():
                response.feasible = False
                response.message = "motion backend is busy"
                return response
            target_id = int(request.target_id)
            prepared = False
            try:
                target = _to_pose(request.target_pose.pose).normalized()
                prepared = self._backend.prepare_pick(target_id, target)
                if not prepared:
                    response.feasible = False
                    response.message = (
                        "failed to synchronize perception collision scene"
                    )
                    return response
                executor = self._executor_core
                pregrasp_candidates = bounded_pregrasp_candidates_for_fruit_center(
                    target,
                    quaternion=executor.grasp_quaternion,
                    tool_center_offset_m=executor.tool_center_offset_m,
                    pregrasp_offset_m=executor.pregrasp_offset_m,
                )
                primary_grasp = hand_pose_for_fruit_center(
                    target,
                    quaternion=executor.grasp_quaternion,
                    tool_center_offset_m=executor.tool_center_offset_m,
                )
                connected_routes = []
                first_planning_time = 0.0
                first_collision = False
                first_joint_travel = 0.0
                for orientation_index, candidate in enumerate(
                    pregrasp_candidates
                ):
                    assessment = self._backend.evaluate_pose_sequence((candidate,))
                    first_planning_time += float(assessment[2])
                    first_collision = first_collision or bool(assessment[1])
                    first_joint_travel = max(
                        first_joint_travel, float(assessment[3])
                    )
                    if not assessment[0]:
                        continue
                    connected = self._backend.preview_guarded_approach_from_current(
                        candidate
                    )
                    first_planning_time += connected.planning_time_sec
                    first_collision = first_collision or bool(connected.collision)
                    first_joint_travel = max(
                        first_joint_travel,
                        float(assessment[3]) + connected.joint_travel_rad,
                    )
                    if not connected.feasible:
                        continue
                    connected_routes.append(
                        (orientation_index, candidate, assessment, connected)
                    )
                if not connected_routes:
                    response.feasible = False
                    response.collision = first_collision
                    response.planning_time_sec = first_planning_time
                    response.joint_travel_rad = first_joint_travel
                    response.message = (
                        "no connected pregrasp route among four bounded "
                        "orientations"
                    )
                    return response
                if not self._backend.allow_target_contact(target_id):
                    response.feasible = False
                    response.message = (
                        "failed to open target contact corridor for evaluation"
                    )
                    return response
                route_collision = False
                route_planning_time = 0.0
                route_joint_travel = 0.0
                for (
                    orientation_index,
                    pregrasp,
                    _assessment,
                    connected,
                ) in connected_routes:
                    grasp = rotate_about_base_z(
                        primary_grasp,
                        orientation_index * math.pi / 2.0,
                    )
                    # Match the executor's direction-preserving ESCAPE
                    # primitive exactly; a preflight must not certify a
                    # base-Z retreat that execution will not perform.
                    retreat = offset_along_local_z(
                        grasp, -executor.retreat_distance_m
                    )
                    route = self._backend.evaluate_pose_sequence(
                        (pregrasp, grasp, retreat)
                    )
                    route_planning_time += float(route[2])
                    route_collision = route_collision or bool(route[1])
                    route_joint_travel = max(
                        route_joint_travel,
                        float(route[3]) + connected.joint_travel_rad,
                    )
                    if not route[0]:
                        continue
                    response.feasible = True
                    response.collision = False
                    response.planning_time_sec = float(
                        first_planning_time + route_planning_time
                    )
                    response.joint_travel_rad = float(
                        route[3] + connected.joint_travel_rad
                    )
                    response.message = (
                        "current state, pregrasp, grasp, and retreat are "
                        "connected and feasible; bounded_orientation_index="
                        f"{orientation_index}"
                    )
                    return response
                response.feasible = False
                response.collision = route_collision
                response.planning_time_sec = float(
                    first_planning_time + route_planning_time
                )
                response.joint_travel_rad = float(route_joint_travel)
                response.message = (
                    "grasp or retreat IK/collision check failed for every "
                    "connected bounded orientation"
                )
                return response
            except Exception as exc:
                response.feasible = False
                response.message = f"target feasibility evaluation failed: {exc}"
                return response
            finally:
                if prepared:
                    if not self._backend.restore_target_collision(target_id):
                        response.feasible = False
                        response.message = (
                            f"{response.message}; " if response.message else ""
                        ) + "failed to restore the target collision object"
                self._goal_gate.release()

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

        def _on_tracked_targets(self, message) -> None:
            base_frame = str(self.get_parameter("base_frame").value)
            if message.header.frame_id != base_frame:
                self.get_logger().error(
                    "ignoring tracked targets outside the manipulation base frame"
                )
                return
            centers = {
                int(item.track_id): (
                    float(item.pose.position.x),
                    float(item.pose.position.y),
                    float(item.pose.position.z),
                )
                for item in message.targets
                if int(item.track_id) > 0
            }
            with self._tracked_pose_lock:
                self._tracked_pose_centers = centers
                self._tracked_pose_received_monotonic = time.monotonic()

        def _tracked_pose_snapshot(self):
            with self._tracked_pose_lock:
                centers = self._tracked_pose_centers
                received = self._tracked_pose_received_monotonic
                if centers is None or received is None:
                    raise RuntimeError("tracked fruit collision scene is unavailable")
                age = time.monotonic() - received
                if age > self._tracked_pose_timeout_sec:
                    raise RuntimeError(
                        f"tracked fruit collision scene is stale by {age:.3f} seconds"
                    )
                return dict(centers)

        def _on_target_refinement(self, message) -> None:
            base_frame = str(self.get_parameter("base_frame").value)
            if message.header.frame_id != base_frame:
                self.get_logger().error(
                    "ignoring visual refinement whose frame differs from "
                    "the manipulation base"
                )
                return
            target_id = int(message.target_id)
            position = message.pose.position
            center = (
                float(position.x),
                float(position.y),
                float(position.z),
            )
            confidence = float(message.detection_confidence)
            sigma_m = float(message.position_sigma_m)
            if (
                target_id <= 0
                or not all(math.isfinite(value) for value in center)
                or not math.isfinite(confidence)
                or not math.isfinite(sigma_m)
            ):
                self.get_logger().error("ignoring non-finite visual target refinement")
                return
            with self._target_refinement_lock:
                self._target_refinement_samples[target_id] = (
                    center,
                    confidence,
                    sigma_m,
                    time.monotonic(),
                )

        def _refine_target_pose(self, target_id: int, initial_pose: Pose) -> Pose:
            with self._target_refinement_lock:
                sample = self._target_refinement_samples.get(target_id)
            if sample is None:
                raise RuntimeError(
                    f"no visual refinement sample for target {target_id}"
                )
            center, confidence, sigma_m, received = sample
            age = time.monotonic() - received
            if age > self._target_refinement_timeout_sec:
                raise RuntimeError(f"visual refinement is stale by {age:.3f} seconds")
            if confidence < self._target_refinement_min_confidence:
                raise RuntimeError(
                    "visual refinement confidence "
                    f"{confidence:.3f} is below "
                    f"{self._target_refinement_min_confidence:.3f}"
                )
            if not 0.0 <= sigma_m <= self._target_refinement_max_sigma_m:
                raise RuntimeError(
                    f"visual refinement sigma {sigma_m:.4f} m exceeds "
                    f"{self._target_refinement_max_sigma_m:.4f} m"
                )
            correction_m = math.dist(
                (initial_pose.x, initial_pose.y, initial_pose.z),
                center,
            )
            if correction_m > self._target_refinement_max_correction_m:
                raise RuntimeError(
                    f"visual refinement correction {correction_m:.4f} m "
                    "exceeds the configured limit"
                )
            self._backend.update_prepared_visual_center(target_id, center)
            self.get_logger().info(
                "Near-grasp visual target refined: "
                f"target={target_id}, correction={correction_m * 1000.0:.1f} "
                f"mm, confidence={confidence:.3f}, sigma={sigma_m * 1000.0:.1f} mm"
            )
            return Pose(
                x=center[0],
                y=center[1],
                z=center[2],
                qx=initial_pose.qx,
                qy=initial_pose.qy,
                qz=initial_pose.qz,
                qw=initial_pose.qw,
            )

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
                result.recovery_disposition = int(outcome.recovery_disposition)
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
