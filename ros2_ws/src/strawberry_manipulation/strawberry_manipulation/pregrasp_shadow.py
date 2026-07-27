"""Plan a pre-grasp trajectory, audit it, and discard it without execution."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Mapping, Sequence

from .core import Pose, pregrasp_pose_for_fruit_center
from .grasp_geometry import load_grasp_geometry
from .handoff_shadow import ARM_JOINT_NAMES, target_clock_is_coherent
from .scene_geometry import STATIC_COLLISION_OBJECTS, fruit_collision_id


def _finite_float(value) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _pose_errors(requested: Pose, observed) -> tuple[float, float]:
    requested = requested.normalized()
    translation_error = math.dist(
        (requested.x, requested.y, requested.z),
        (
            float(observed.position.x),
            float(observed.position.y),
            float(observed.position.z),
        ),
    )
    dot = abs(
        requested.qx * float(observed.orientation.x)
        + requested.qy * float(observed.orientation.y)
        + requested.qz * float(observed.orientation.z)
        + requested.qw * float(observed.orientation.w)
    )
    orientation_error = 2.0 * math.acos(min(1.0, max(0.0, dot)))
    return translation_error, orientation_error


def _maximum_joint_range(
    samples: Sequence[Mapping[str, object]],
) -> float | None:
    if not samples or any(
        any(name not in sample for name in ARM_JOINT_NAMES)
        for sample in samples
    ):
        return None
    try:
        values = {
            name: [float(sample[name]) for sample in samples]
            for name in ARM_JOINT_NAMES
        }
    except (TypeError, ValueError):
        return None
    if any(
        not all(math.isfinite(value) for value in positions)
        for positions in values.values()
    ):
        return None
    return max(
        max(positions) - min(positions) for positions in values.values()
    )


def summarize_pregrasp_shadow(
    *,
    handoff: Mapping[str, object],
    target_samples: Sequence[Mapping[str, object]],
    joint_samples: Sequence[Mapping[str, object]],
    planning_attempts: Sequence[Mapping[str, object]],
    expected_collision_ids: Sequence[str],
    collision_ids_before: Sequence[str],
    collision_ids_after: Sequence[str],
    expected_frame_id: str = "panda_link0",
    minimum_target_samples: int = 15,
    minimum_joint_samples: int = 10,
    maximum_target_age_sec: float = 0.5,
    maximum_joint_delta_rad: float = 0.002,
    plan_completion_target_age_sec: float | None = None,
    moveit_state_delta_rad: float | None = None,
    controller_configuration_keys: Sequence[str] = (),
    control_interface_created: bool = False,
    trajectory_executed: bool = False,
    pick_action_called: bool = False,
    gripper_command_sent: bool = False,
    runtime_errors: Sequence[str] = (),
) -> dict[str, object]:
    """Validate a controller-free plan and preserve the no-motion boundary."""

    if minimum_target_samples <= 0 or minimum_joint_samples <= 0:
        raise ValueError("minimum sample counts must be positive")
    if maximum_target_age_sec <= 0.0 or maximum_joint_delta_rad <= 0.0:
        raise ValueError("age and stationarity bounds must be positive")

    violations = [str(value) for value in runtime_errors]
    expected_target_id = int(handoff.get("expected_target_id", 0))
    if handoff.get("handoff_passed") is not True:
        violations.append("upstream handoff audit did not pass")
    if handoff.get("pick_authorized") is not False:
        violations.append("upstream handoff authorized a pick")
    if expected_target_id <= 0:
        violations.append("upstream handoff has no positive target identity")

    target_ids: list[int] = []
    target_ages: list[float] = []
    if len(target_samples) < minimum_target_samples:
        violations.append("insufficient planning TargetPose samples")
    for sample in target_samples:
        target_id = int(sample.get("target_id", 0))
        target_ids.append(target_id)
        if target_id != expected_target_id:
            violations.append("planning target identity changed after handoff")
        if str(sample.get("frame_id", "")) != expected_frame_id:
            violations.append("planning target uses an unexpected frame")
        age = _finite_float(sample.get("age_sec"))
        if age is None:
            violations.append("planning target age is non-finite")
        else:
            target_ages.append(age)
            if age < -0.05:
                violations.append("planning target stamp is in the future")
            if age > maximum_target_age_sec:
                violations.append("planning target sample is stale")

    completion_age = _finite_float(plan_completion_target_age_sec)
    if completion_age is None:
        violations.append("planning completion target age is unavailable")
    elif completion_age < -0.05:
        violations.append("planning completion target stamp is in the future")
    elif completion_age > maximum_target_age_sec:
        violations.append("target became stale before planning completed")

    joint_delta = _maximum_joint_range(joint_samples)
    if len(joint_samples) < minimum_joint_samples:
        violations.append("insufficient planning joint-state samples")
    if joint_delta is None:
        violations.append("planning joint-state samples are incomplete")
    elif joint_delta > maximum_joint_delta_rad:
        violations.append("arm moved while freezing the planning target")
    moveit_delta = _finite_float(moveit_state_delta_rad)
    if moveit_delta is None:
        violations.append("MoveIt live-state delta is unavailable")
    elif moveit_delta > maximum_joint_delta_rad:
        violations.append("MoveIt observed motion during planning")

    expected_ids = sorted(set(str(value) for value in expected_collision_ids))
    before_ids = sorted(set(str(value) for value in collision_ids_before))
    after_ids = sorted(set(str(value) for value in collision_ids_after))
    missing_before = sorted(set(expected_ids) - set(before_ids))
    missing_after = sorted(set(expected_ids) - set(after_ids))
    if missing_before:
        violations.append(
            "pre-planning collision scene is missing: "
            + ", ".join(missing_before)
        )
    if missing_after:
        violations.append(
            "post-planning collision scene is missing: "
            + ", ".join(missing_after)
        )
    target_collision_id = (
        fruit_collision_id(expected_target_id)
        if expected_target_id > 0
        else ""
    )
    target_retained = (
        bool(target_collision_id)
        and target_collision_id in before_ids
        and target_collision_id in after_ids
    )
    if not target_retained:
        violations.append("selected fruit collision was not retained")

    successful_attempts = [
        attempt
        for attempt in planning_attempts
        if attempt.get("success") is True
    ]
    if not planning_attempts:
        violations.append("no planning attempt was recorded")
    if len(successful_attempts) != 1:
        violations.append("planning did not produce exactly one accepted plan")
    accepted_attempt = successful_attempts[0] if successful_attempts else {}
    endpoint_position_error = _finite_float(
        accepted_attempt.get("endpoint_position_error_m")
    )
    endpoint_orientation_error = _finite_float(
        accepted_attempt.get("endpoint_orientation_error_rad")
    )
    if endpoint_position_error is None or endpoint_position_error > 0.01:
        violations.append("planned endpoint position is outside tolerance")
    if (
        endpoint_orientation_error is None
        or endpoint_orientation_error > 0.0872665
    ):
        violations.append("planned endpoint orientation is outside tolerance")
    if int(accepted_attempt.get("trajectory_waypoint_count", 0)) < 2:
        violations.append("accepted plan has fewer than two waypoints")

    controller_keys = sorted(str(value) for value in controller_configuration_keys)
    if controller_keys:
        violations.append("planning config retained controller parameters")
    if control_interface_created:
        violations.append("planning Shadow created a control interface")
    if trajectory_executed:
        violations.append("planned trajectory was executed")
    if pick_action_called:
        violations.append("pick action was called")
    if gripper_command_sent:
        violations.append("gripper command was sent")
    violations = list(dict.fromkeys(violations))

    return {
        "schema_version": 1,
        "scope": "NON_ACCEPTANCE_PREGRASP_PLANNING_SHADOW",
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "planning_passed": not violations,
        "pick_authorized": False,
        "expected_target_id": expected_target_id,
        "observed_target_ids": sorted(set(target_ids)),
        "target_sample_count": len(target_samples),
        "minimum_target_samples": minimum_target_samples,
        "target_age_sec": {
            "minimum": min(target_ages) if target_ages else None,
            "maximum": max(target_ages) if target_ages else None,
            "at_plan_completion": completion_age,
        },
        "joint_sample_count": len(joint_samples),
        "observed_joint_delta_rad": joint_delta,
        "moveit_state_delta_rad": moveit_delta,
        "collision_scene": {
            "expected_ids": expected_ids,
            "before_ids": before_ids,
            "after_ids": after_ids,
            "missing_before": missing_before,
            "missing_after": missing_after,
            "selected_fruit_collision_retained": target_retained,
        },
        "planning_attempt_count": len(planning_attempts),
        "planning_attempts": [dict(value) for value in planning_attempts],
        "trajectory_generated": bool(successful_attempts),
        "trajectory_discarded": bool(successful_attempts) and not trajectory_executed,
        "trajectory_executed": trajectory_executed,
        "trajectory_execution_requested": False,
        "trajectory_execution_time_sec": 0.0,
        "controller_configuration_keys": controller_keys,
        "control_interface_created": control_interface_created,
        "pick_action_called": pick_action_called,
        "gripper_command_sent": gripper_command_sent,
        "control_command_count": int(trajectory_executed)
        + int(pick_action_called)
        + int(gripper_command_sent),
        "violations": violations,
        "state_history": [
            "HANDOFF_RECEIPT_VALIDATED",
            "READ_ONLY_MOVEIT_READY",
            "PLANNING_COLLISION_SCENE_VALIDATED",
            "PLANNING_TARGET_FROZEN",
            "PREGRASP_GOAL_CONSTRUCTED",
            "PREGRASP_PLAN_REQUESTED",
            "TRAJECTORY_DISCARDED"
            if not violations
            else "PREGRASP_PLAN_REJECTED",
            "STOP_BEFORE_TRAJECTORY_EXECUTION",
        ],
    }


def _fingerprint(path: Path) -> dict[str, object]:
    data = path.read_bytes()
    return {
        "path": str(path),
        "bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def main(args=None) -> int:  # pragma: no cover - ROS / MoveIt integration
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--handoff-json", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument(
        "--target-topic", default="/strawberry/shadow/target_pose"
    )
    parser.add_argument(
        "--camera-mount", choices=("fixed", "wrist", "dual"), default="dual"
    )
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--planning-attempts", type=int, default=3)
    parser.add_argument("--minimum-target-samples", type=int, default=15)
    parser.add_argument("--minimum-joint-samples", type=int, default=10)
    parser.add_argument("--maximum-target-age-sec", type=float, default=0.5)
    parser.add_argument("--maximum-joint-delta-rad", type=float, default=0.002)
    options, ros_args = parser.parse_known_args(args)
    if options.output_json.exists():
        raise ValueError(f"refusing to overwrite {options.output_json}")
    if options.planning_attempts <= 0 or options.timeout_sec <= 0.0:
        raise ValueError("attempt and timeout bounds must be positive")

    handoff = json.loads(options.handoff_json.read_text(encoding="utf-8"))
    expected_target_id = int(handoff.get("expected_target_id", 0))
    try:
        from ament_index_python.packages import get_package_share_directory
        from geometry_msgs.msg import PoseArray, PoseStamped
        from moveit.planning import MoveItPy
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from strawberry_interfaces.msg import TargetPose
        from strawberry_sim.core import load_scene_config
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 pre-grasp Shadow dependencies are unavailable"
        ) from exc

    from .moveit_config import build_moveit_config
    from .moveit_scene import (
        apply_fruit_collision_scene,
        apply_static_collision_scene,
    )

    scene_path = (
        Path(get_package_share_directory("strawberry_sim"))
        / "config"
        / "scene.yaml"
    )
    scene = load_scene_config(scene_path)
    grasp_geometry_path = (
        Path(get_package_share_directory("strawberry_manipulation"))
        / "config"
        / "grasp_geometry.yaml"
    )
    grasp_geometry = load_grasp_geometry(
        grasp_geometry_path,
        world_name=scene.world_name,
        fruit_collision_radius_m=scene.fruit_collision_radius_m,
    )
    expected_frame_id = scene.base_frame
    fruit_ids = tuple(fruit.target_id for fruit in scene.ordered_fruits)
    expected_collision_ids = [
        specification.object_id for specification in STATIC_COLLISION_OBJECTS
    ] + [fruit_collision_id(target_id) for target_id in fruit_ids]

    class PlanningInputNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_pregrasp_shadow_inputs")
            self.target_samples: list[dict[str, object]] = []
            self.joint_samples: list[dict[str, float]] = []
            self.truth_centers: dict[int, tuple[float, float, float]] | None = None
            self.truth_frame_id: str | None = None
            self.clock_unsynchronized_target_samples = 0
            self.create_subscription(
                TargetPose, options.target_topic, self._on_target, 20
            )
            self.create_subscription(
                JointState,
                "/joint_states",
                self._on_joint_state,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PoseArray,
                "/strawberry/ground_truth/poses",
                self._on_truth,
                qos_profile_sensor_data,
            )

        def _on_target(self, message) -> None:
            stamp_sec = float(message.header.stamp.sec) + (
                float(message.header.stamp.nanosec) * 1.0e-9
            )
            now_sec = self.get_clock().now().nanoseconds * 1.0e-9
            if not target_clock_is_coherent(
                receipt_stamp_sec=now_sec,
                acquisition_stamp_sec=stamp_sec,
                maximum_startup_skew_sec=options.maximum_target_age_sec,
            ):
                self.clock_unsynchronized_target_samples += 1
                return
            self.target_samples.append(
                {
                    "target_id": int(message.target_id),
                    "frame_id": str(message.header.frame_id),
                    "acquisition_stamp_sec": stamp_sec,
                    "receipt_stamp_sec": now_sec,
                    "age_sec": now_sec - stamp_sec,
                    "position_xyz_m": [
                        float(message.pose.position.x),
                        float(message.pose.position.y),
                        float(message.pose.position.z),
                    ],
                    "detection_confidence": float(
                        message.detection_confidence
                    ),
                    "position_sigma_m": float(message.position_sigma_m),
                }
            )

        def _on_joint_state(self, message) -> None:
            by_name = {
                str(name): float(position)
                for name, position in zip(message.name, message.position)
            }
            if all(name in by_name for name in ARM_JOINT_NAMES):
                self.joint_samples.append(
                    {name: by_name[name] for name in ARM_JOINT_NAMES}
                )

        def _on_truth(self, message) -> None:
            if len(message.poses) != len(fruit_ids):
                return
            self.truth_frame_id = str(message.header.frame_id)
            self.truth_centers = {
                target_id: (
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                )
                for target_id, pose in zip(fruit_ids, message.poses)
            }

    target_samples: list[dict[str, object]] = []
    joint_samples: list[dict[str, float]] = []
    planning_attempts: list[dict[str, object]] = []
    collision_ids_before: list[str] = []
    collision_ids_after: list[str] = []
    controller_configuration_keys: list[str] = []
    plan_completion_target_age_sec: float | None = None
    moveit_state_delta_rad: float | None = None
    pregrasp: Pose | None = None
    runtime_errors: list[str] = []
    clock_unsynchronized_target_samples = 0
    moveit = None
    arm = None
    planning_scene_monitor = None
    node = None
    rclpy_initialized = False

    try:
        if handoff.get("handoff_passed") is not True:
            raise RuntimeError("upstream handoff audit did not pass")
        if handoff.get("pick_authorized") is not False:
            raise RuntimeError("upstream handoff did not preserve pick denial")
        if expected_target_id <= 0:
            raise RuntimeError("upstream handoff has no positive target identity")
        moveit_config = build_moveit_config(
            camera_mount=options.camera_mount,
            enable_trajectory_execution=False,
        )
        controller_configuration_keys = sorted(
            key
            for key in moveit_config
            if (
                "controller" in key.lower()
                or "trajectory_execution" in key.lower()
            )
        )
        if controller_configuration_keys:
            raise RuntimeError(
                "read-only MoveIt config retained control keys: "
                + ", ".join(controller_configuration_keys)
            )
        moveit = MoveItPy(
            node_name="strawberry_pregrasp_shadow_moveit",
            config_dict=moveit_config,
        )
        arm = moveit.get_planning_component("panda_arm")
        planning_scene_monitor = moveit.get_planning_scene_monitor()
        state_deadline = time.monotonic() + min(options.timeout_sec, 20.0)
        initial_positions: list[float] = []
        while time.monotonic() < state_deadline:
            with planning_scene_monitor.read_only() as planning_scene:
                initial_positions = list(
                    planning_scene.current_state.get_joint_group_positions(
                        "panda_arm"
                    )
                )
            if (
                len(initial_positions) == len(ARM_JOINT_NAMES)
                and max(abs(value) for value in initial_positions) > 0.1
            ):
                break
            time.sleep(0.1)
        if len(initial_positions) != len(ARM_JOINT_NAMES):
            raise RuntimeError("MoveIt did not receive a complete arm state")

        rclpy.init(args=ros_args)
        rclpy_initialized = True
        node = PlanningInputNode()
        input_deadline = time.monotonic() + options.timeout_sec
        while (
            node.truth_centers is None
            and time.monotonic() < input_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.truth_centers is None:
            raise RuntimeError("live fruit truth timed out")
        if node.truth_frame_id != expected_frame_id:
            raise RuntimeError("live fruit truth uses an unexpected frame")

        apply_static_collision_scene(
            planning_scene_monitor, expected_frame_id
        )
        apply_fruit_collision_scene(
            planning_scene_monitor,
            expected_frame_id,
            node.truth_centers,
            scene.fruit_collision_radius_m,
        )
        with planning_scene_monitor.read_only() as planning_scene:
            collision_ids_before = sorted(
                str(value.id)
                for value in (
                    planning_scene.planning_scene_message.world.collision_objects
                )
            )
        node.target_samples.clear()
        node.joint_samples.clear()
        input_deadline = time.monotonic() + options.timeout_sec
        while (
            (
                len(node.target_samples) < options.minimum_target_samples
                or len(node.joint_samples) < options.minimum_joint_samples
            )
            and time.monotonic() < input_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        target_samples = list(
            node.target_samples[: options.minimum_target_samples]
        )
        joint_samples = list(
            node.joint_samples[: options.minimum_joint_samples]
        )
        clock_unsynchronized_target_samples = (
            node.clock_unsynchronized_target_samples
        )
        if (
            len(target_samples) < options.minimum_target_samples
            or len(joint_samples) < options.minimum_joint_samples
        ):
            raise RuntimeError("fresh planning inputs timed out")

        frozen = target_samples[-1]
        center_xyz = list(frozen["position_xyz_m"])
        center = Pose(
            float(center_xyz[0]),
            float(center_xyz[1]),
            float(center_xyz[2]),
        )
        pregrasp = pregrasp_pose_for_fruit_center(
            center,
            tool_center_offset_m=grasp_geometry.tool_center_offset_m,
        )
        goal = PoseStamped()
        goal.header.frame_id = expected_frame_id
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = pregrasp.x
        goal.pose.position.y = pregrasp.y
        goal.pose.position.z = pregrasp.z
        goal.pose.orientation.x = pregrasp.qx
        goal.pose.orientation.y = pregrasp.qy
        goal.pose.orientation.z = pregrasp.qz
        goal.pose.orientation.w = pregrasp.qw

        with planning_scene_monitor.read_only() as planning_scene:
            before_positions = list(
                planning_scene.current_state.get_joint_group_positions(
                    "panda_arm"
                )
            )
        for attempt_index in range(1, options.planning_attempts + 1):
            arm.set_start_state_to_current_state()
            arm.set_goal_state(
                pose_stamped_msg=goal,
                pose_link="panda_hand",
            )
            planning_started = time.perf_counter()
            plan_result = arm.plan()
            planning_time = time.perf_counter() - planning_started
            receipt: dict[str, object] = {
                "attempt": attempt_index,
                "success": False,
                "planning_time_sec": planning_time,
                "trajectory_waypoint_count": 0,
                "endpoint_position_error_m": None,
                "endpoint_orientation_error_rad": None,
            }
            if plan_result:
                trajectory = plan_result.trajectory
                waypoint_count = len(trajectory)
                endpoint = trajectory[waypoint_count - 1].get_pose(
                    "panda_hand"
                )
                position_error, orientation_error = _pose_errors(
                    pregrasp, endpoint
                )
                receipt.update(
                    {
                        "success": (
                            waypoint_count >= 2
                            and position_error <= 0.01
                            and orientation_error <= 0.0872665
                        ),
                        "trajectory_waypoint_count": waypoint_count,
                        "endpoint_position_error_m": position_error,
                        "endpoint_orientation_error_rad": orientation_error,
                    }
                )
            planning_attempts.append(receipt)
            if receipt["success"] is True:
                break

        # Refresh simulated time after planning before calculating target age.
        rclpy.spin_once(node, timeout_sec=0.05)
        plan_completion_target_age_sec = (
            node.get_clock().now().nanoseconds * 1.0e-9
            - float(frozen["acquisition_stamp_sec"])
        )
        with planning_scene_monitor.read_only() as planning_scene:
            after_positions = list(
                planning_scene.current_state.get_joint_group_positions(
                    "panda_arm"
                )
            )
            collision_ids_after = sorted(
                str(value.id)
                for value in (
                    planning_scene.planning_scene_message.world.collision_objects
                )
            )
        if len(after_positions) != len(before_positions):
            raise RuntimeError("MoveIt arm state changed shape during planning")
        moveit_state_delta_rad = max(
            abs(after - before)
            for before, after in zip(before_positions, after_positions)
        )
    except Exception as exc:
        runtime_errors.append(f"pre-grasp planning audit failed: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy_initialized:
            rclpy.try_shutdown()

    result = summarize_pregrasp_shadow(
        handoff=handoff,
        target_samples=target_samples,
        joint_samples=joint_samples,
        planning_attempts=planning_attempts,
        expected_collision_ids=expected_collision_ids,
        collision_ids_before=collision_ids_before,
        collision_ids_after=collision_ids_after,
        expected_frame_id=expected_frame_id,
        minimum_target_samples=options.minimum_target_samples,
        minimum_joint_samples=options.minimum_joint_samples,
        maximum_target_age_sec=options.maximum_target_age_sec,
        maximum_joint_delta_rad=options.maximum_joint_delta_rad,
        plan_completion_target_age_sec=plan_completion_target_age_sec,
        moveit_state_delta_rad=moveit_state_delta_rad,
        controller_configuration_keys=controller_configuration_keys,
        control_interface_created=bool(controller_configuration_keys),
        runtime_errors=runtime_errors,
    )
    result["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    result["handoff_input"] = _fingerprint(options.handoff_json)
    result["target_topic"] = options.target_topic
    result["camera_mount"] = options.camera_mount
    result["scene_config"] = str(scene_path)
    result["grasp_geometry_config"] = str(grasp_geometry_path)
    result["grasp_geometry_profile"] = grasp_geometry.profile_id
    result["clock_unsynchronized_target_samples_ignored"] = (
        clock_unsynchronized_target_samples
    )
    result["pregrasp_pose"] = (
        {
            "frame_id": expected_frame_id,
            "position_xyz_m": [pregrasp.x, pregrasp.y, pregrasp.z],
            "orientation_xyzw": [
                pregrasp.qx,
                pregrasp.qy,
                pregrasp.qz,
                pregrasp.qw,
            ],
            "tool_center_offset_m": grasp_geometry.tool_center_offset_m,
            "pregrasp_offset_m": 0.15,
        }
        if pregrasp is not None
        else None
    )
    options.output_json.parent.mkdir(parents=True, exist_ok=True)
    options.output_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, sort_keys=True), flush=True)
    status = 0 if result["planning_passed"] else 1
    # Keep MoveItPy alive until process exit to avoid the Jazzy native
    # destructor defect.  No trajectory execution method is called.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(status)


if __name__ == "__main__":
    main()
