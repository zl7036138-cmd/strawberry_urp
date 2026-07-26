"""Audit an observation-to-control handoff without creating control clients."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Mapping, Sequence

from .scene_geometry import STATIC_COLLISION_OBJECTS, fruit_collision_id


ARM_JOINT_NAMES = tuple(f"panda_joint{index}" for index in range(1, 8))


def _all_finite(values: Sequence[object]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def target_clock_is_coherent(
    *,
    receipt_stamp_sec: float,
    acquisition_stamp_sec: float,
    maximum_startup_skew_sec: float,
    maximum_future_skew_sec: float = 0.05,
) -> bool:
    """Reject samples received before a simulation clock becomes coherent."""

    if maximum_startup_skew_sec <= 0.0 or maximum_future_skew_sec <= 0.0:
        raise ValueError("clock skew limits must be positive")
    if maximum_future_skew_sec > maximum_startup_skew_sec:
        raise ValueError(
            "future clock skew cannot exceed the startup skew bound"
        )
    if not _all_finite((receipt_stamp_sec, acquisition_stamp_sec)):
        return False
    return (
        receipt_stamp_sec > 0.0
        and acquisition_stamp_sec - receipt_stamp_sec
        <= maximum_future_skew_sec
    )


def _joint_stationarity(
    samples: Sequence[Mapping[str, object]],
    *,
    minimum_samples: int,
    maximum_delta_rad: float,
) -> tuple[float | None, list[str]]:
    violations: list[str] = []
    if len(samples) < minimum_samples:
        violations.append("insufficient complete joint-state samples")
        return None, violations
    missing = [
        name
        for name in ARM_JOINT_NAMES
        if any(name not in sample for sample in samples)
    ]
    if missing:
        violations.append(
            "joint-state samples are missing arm joints: " + ", ".join(missing)
        )
        return None, violations
    values_by_name = {
        name: [float(sample[name]) for sample in samples]
        for name in ARM_JOINT_NAMES
    }
    if not all(_all_finite(values) for values in values_by_name.values()):
        violations.append("joint-state samples contain non-finite positions")
        return None, violations
    max_delta = max(
        max(values) - min(values) for values in values_by_name.values()
    )
    if max_delta > maximum_delta_rad:
        violations.append(
            "arm moved during the shadow handoff "
            f"({max_delta:.6f} rad > {maximum_delta_rad:.6f} rad)"
        )
    return max_delta, violations


def summarize_handoff_shadow(
    *,
    expected_target_id: int,
    target_samples: Sequence[Mapping[str, object]],
    joint_samples: Sequence[Mapping[str, object]],
    expected_collision_ids: Sequence[str],
    observed_collision_ids: Sequence[str],
    observed_collision_frames: Mapping[str, object] | None = None,
    expected_frame_id: str = "panda_link0",
    allowed_collision_frame_ids: Sequence[str] | None = None,
    minimum_target_samples: int = 15,
    minimum_joint_samples: int = 10,
    maximum_target_age_sec: float = 0.5,
    maximum_joint_delta_rad: float = 0.002,
    moveit_state_delta_rad: float | None = None,
    control_interfaces_created: bool = False,
    pick_action_called: bool = False,
    trajectory_command_sent: bool = False,
    gripper_command_sent: bool = False,
    runtime_errors: Sequence[str] = (),
) -> dict[str, object]:
    """Return a fail-closed, explicitly non-actuating handoff receipt."""

    if expected_target_id <= 0:
        raise ValueError("expected_target_id must be positive")
    if minimum_target_samples <= 0 or minimum_joint_samples <= 0:
        raise ValueError("minimum sample counts must be positive")
    if maximum_target_age_sec <= 0.0 or maximum_joint_delta_rad <= 0.0:
        raise ValueError("age and stationarity bounds must be positive")
    if not expected_frame_id:
        raise ValueError("expected_frame_id must be non-empty")

    violations = [str(error) for error in runtime_errors]
    if len(target_samples) < minimum_target_samples:
        violations.append("insufficient TargetPose handoff samples")

    target_ids: list[int] = []
    target_ages: list[float] = []
    for sample in target_samples:
        target_id = int(sample.get("target_id", 0))
        target_ids.append(target_id)
        if target_id != expected_target_id:
            violations.append(
                f"handoff target identity {target_id} differs from "
                f"selected identity {expected_target_id}"
            )
        frame_id = str(sample.get("frame_id", ""))
        if frame_id != expected_frame_id:
            violations.append(
                f"handoff frame {frame_id!r} differs from {expected_frame_id!r}"
            )
        try:
            age_sec = float(sample.get("age_sec"))
        except (TypeError, ValueError):
            age_sec = math.nan
        target_ages.append(age_sec)
        if not math.isfinite(age_sec):
            violations.append("TargetPose age is non-finite")
        elif age_sec < -0.05:
            violations.append("TargetPose acquisition stamp is in the future")
        elif age_sec > maximum_target_age_sec:
            violations.append(
                f"TargetPose is stale ({age_sec:.6f} s > "
                f"{maximum_target_age_sec:.6f} s)"
            )
        pose_values = list(sample.get("position_xyz_m", [])) + list(
            sample.get("orientation_xyzw", [])
        )
        if len(pose_values) != 7 or not _all_finite(pose_values):
            violations.append("TargetPose contains an invalid pose")
        try:
            confidence = float(sample.get("detection_confidence"))
            sigma = float(sample.get("position_sigma_m"))
        except (TypeError, ValueError):
            confidence, sigma = math.nan, math.nan
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            violations.append("TargetPose confidence is outside [0, 1]")
        if not math.isfinite(sigma) or sigma <= 0.0:
            violations.append("TargetPose position uncertainty is not positive")

    joint_delta, joint_violations = _joint_stationarity(
        joint_samples,
        minimum_samples=minimum_joint_samples,
        maximum_delta_rad=maximum_joint_delta_rad,
    )
    violations.extend(joint_violations)
    if moveit_state_delta_rad is None:
        violations.append("MoveIt did not provide a bounded live-state delta")
    elif not math.isfinite(float(moveit_state_delta_rad)):
        violations.append("MoveIt live-state delta is non-finite")
    elif float(moveit_state_delta_rad) > maximum_joint_delta_rad:
        violations.append(
            "MoveIt observed arm motion during collision-scene audit "
            f"({float(moveit_state_delta_rad):.6f} rad)"
        )

    expected_ids = sorted(set(str(value) for value in expected_collision_ids))
    observed_ids = sorted(set(str(value) for value in observed_collision_ids))
    missing_ids = sorted(set(expected_ids) - set(observed_ids))
    if missing_ids:
        violations.append(
            "MoveIt collision scene is missing: " + ", ".join(missing_ids)
        )
    target_collision_id = fruit_collision_id(expected_target_id)
    if target_collision_id not in observed_ids:
        violations.append("selected fruit was removed from the collision scene")
    collision_frames = {
        str(key): str(value)
        for key, value in (observed_collision_frames or {}).items()
    }
    allowed_collision_frames = set(
        allowed_collision_frame_ids or (expected_frame_id,)
    )
    wrong_frames = sorted(
        object_id
        for object_id in expected_ids
        if collision_frames.get(object_id) not in allowed_collision_frames
    )
    if wrong_frames:
        violations.append(
            "collision objects use an unexpected frame: "
            + ", ".join(wrong_frames)
        )

    if control_interfaces_created:
        violations.append("the shadow probe created a control interface")
    if pick_action_called:
        violations.append("the pick action was called")
    if trajectory_command_sent:
        violations.append("an arm trajectory command was sent")
    if gripper_command_sent:
        violations.append("a gripper command was sent")

    # Preserve ordering while avoiding an unreadable list of duplicate reasons.
    violations = list(dict.fromkeys(violations))
    frozen = dict(target_samples[-1]) if target_samples else None
    return {
        "schema_version": 1,
        "scope": "NON_ACCEPTANCE_OBSERVATION_TO_CONTROL_HANDOFF_SHADOW",
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "handoff_passed": not violations,
        "pick_authorized": False,
        "expected_target_id": expected_target_id,
        "observed_target_ids": sorted(set(target_ids)),
        "target_sample_count": len(target_samples),
        "minimum_target_samples": minimum_target_samples,
        "maximum_target_age_sec": maximum_target_age_sec,
        "target_age_sec": {
            "minimum": min(target_ages) if target_ages else None,
            "maximum": max(target_ages) if target_ages else None,
        },
        "frozen_target_pose": frozen,
        "joint_sample_count": len(joint_samples),
        "minimum_joint_samples": minimum_joint_samples,
        "maximum_joint_delta_rad": maximum_joint_delta_rad,
        "observed_joint_delta_rad": joint_delta,
        "moveit_state_delta_rad": moveit_state_delta_rad,
        "collision_scene": {
            "expected_ids": expected_ids,
            "observed_ids": observed_ids,
            "missing_ids": missing_ids,
            "frames": collision_frames,
            "allowed_frame_ids": sorted(allowed_collision_frames),
            "selected_fruit_collision_retained": (
                target_collision_id in observed_ids
            ),
        },
        "control_interface_created": control_interfaces_created,
        "pick_action_called": pick_action_called,
        "trajectory_command_sent": trajectory_command_sent,
        "gripper_command_sent": gripper_command_sent,
        "violations": violations,
        "state_history": [
            "SHADOW_TARGET_RECEIVE",
            "SHADOW_TARGET_FROZEN",
            "SHADOW_IDENTITY_VALIDATED",
            "SHADOW_AGE_VALIDATED",
            "SHADOW_STATIONARITY_VALIDATED",
            "SHADOW_COLLISION_SCENE_VALIDATED",
            "STOP_BEFORE_PICK_ACTION" if not violations else "HANDOFF_REJECTED",
        ],
    }


def main(args=None) -> int:  # pragma: no cover - ROS / MoveIt integration
    """Collect live evidence and stop before any motion-control interface."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--expected-target-id", type=int, required=True)
    parser.add_argument(
        "--target-topic", default="/strawberry/shadow/target_pose"
    )
    parser.add_argument("--camera-mount", choices=("fixed", "wrist", "dual"), default="dual")
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--minimum-target-samples", type=int, default=15)
    parser.add_argument("--minimum-joint-samples", type=int, default=10)
    parser.add_argument("--maximum-target-age-sec", type=float, default=0.5)
    parser.add_argument("--maximum-joint-delta-rad", type=float, default=0.002)
    options, ros_args = parser.parse_known_args(args)
    if options.output_json.exists():
        raise ValueError(f"refusing to overwrite {options.output_json}")
    if options.timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be positive")

    try:
        from ament_index_python.packages import get_package_share_directory
        from geometry_msgs.msg import PoseArray
        from moveit.planning import MoveItPy
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from strawberry_interfaces.msg import TargetPose
        from strawberry_sim.core import load_scene_config
    except ImportError as exc:
        raise RuntimeError("ROS 2 handoff-shadow dependencies are unavailable") from exc

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
    expected_frame_id = scene.base_frame
    fruit_ids = tuple(fruit.target_id for fruit in scene.ordered_fruits)
    expected_collision_ids = [
        specification.object_id for specification in STATIC_COLLISION_OBJECTS
    ] + [fruit_collision_id(target_id) for target_id in fruit_ids]

    class HandoffShadowNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_handoff_shadow_probe")
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
                    "orientation_xyzw": [
                        float(message.pose.orientation.x),
                        float(message.pose.orientation.y),
                        float(message.pose.orientation.z),
                        float(message.pose.orientation.w),
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

        @property
        def ready(self) -> bool:
            return (
                len(self.target_samples) >= options.minimum_target_samples
                and len(self.joint_samples) >= options.minimum_joint_samples
                and self.truth_centers is not None
            )

    target_samples: list[dict[str, object]] = []
    clock_unsynchronized_target_samples = 0
    joint_samples: list[dict[str, float]] = []
    truth_centers: dict[int, tuple[float, float, float]] | None = None
    truth_frame_id: str | None = None
    observed_collision_ids: list[str] = []
    observed_collision_frames: dict[str, str] = {}
    moveit_state_delta_rad: float | None = None
    runtime_errors: list[str] = []
    controller_configuration_keys: list[str] = []
    moveit = None
    planning_scene_monitor = None

    rclpy.init(args=ros_args)
    node = HandoffShadowNode()
    try:
        deadline = time.monotonic() + options.timeout_sec
        while not node.ready and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        target_samples = list(
            node.target_samples[: options.minimum_target_samples]
        )
        clock_unsynchronized_target_samples = (
            node.clock_unsynchronized_target_samples
        )
        joint_samples = list(
            node.joint_samples[: options.minimum_joint_samples]
        )
        truth_centers = (
            dict(node.truth_centers)
            if node.truth_centers is not None
            else None
        )
        truth_frame_id = node.truth_frame_id
        if not node.ready:
            runtime_errors.append("live handoff inputs timed out")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if truth_frame_id not in (None, expected_frame_id):
        runtime_errors.append(
            f"ground-truth frame {truth_frame_id!r} differs from "
            f"{expected_frame_id!r}"
        )

    if not runtime_errors and truth_centers is not None:
        try:
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
                node_name="strawberry_handoff_shadow_moveit",
                config_dict=moveit_config,
            )
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
            apply_static_collision_scene(
                planning_scene_monitor, expected_frame_id
            )
            apply_fruit_collision_scene(
                planning_scene_monitor,
                expected_frame_id,
                truth_centers,
                scene.fruit_collision_radius_m,
            )
            time.sleep(0.5)
            with planning_scene_monitor.read_only() as planning_scene:
                final_positions = list(
                    planning_scene.current_state.get_joint_group_positions(
                        "panda_arm"
                    )
                )
                collision_objects = list(
                    planning_scene.planning_scene_message.world.collision_objects
                )
            if len(final_positions) != len(initial_positions):
                raise RuntimeError("MoveIt live arm state changed shape")
            moveit_state_delta_rad = max(
                abs(after - before)
                for before, after in zip(initial_positions, final_positions)
            )
            observed_collision_ids = [
                str(collision_object.id)
                for collision_object in collision_objects
            ]
            observed_collision_frames = {
                str(collision_object.id): str(
                    collision_object.header.frame_id
                )
                for collision_object in collision_objects
            }
        except Exception as exc:  # retain a structured failure receipt
            runtime_errors.append(f"MoveIt collision-scene audit failed: {exc}")

    result = summarize_handoff_shadow(
        expected_target_id=options.expected_target_id,
        target_samples=target_samples,
        joint_samples=joint_samples,
        expected_collision_ids=expected_collision_ids,
        observed_collision_ids=observed_collision_ids,
        observed_collision_frames=observed_collision_frames,
        expected_frame_id=expected_frame_id,
        # MoveIt canonicalizes world objects from panda_link0 into its fixed
        # planning frame.  Both are valid only because the fixed transform is
        # part of the loaded robot model.
        allowed_collision_frame_ids=(expected_frame_id, "world"),
        minimum_target_samples=options.minimum_target_samples,
        minimum_joint_samples=options.minimum_joint_samples,
        maximum_target_age_sec=options.maximum_target_age_sec,
        maximum_joint_delta_rad=options.maximum_joint_delta_rad,
        moveit_state_delta_rad=moveit_state_delta_rad,
        control_interfaces_created=bool(controller_configuration_keys),
        runtime_errors=runtime_errors,
    )
    result["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    result["target_topic"] = options.target_topic
    result["clock_unsynchronized_target_samples_ignored"] = (
        clock_unsynchronized_target_samples
    )
    result["camera_mount"] = options.camera_mount
    result["scene_config"] = str(scene_path)
    result["scene_source"] = (
        "tracked static geometry plus live simulation fruit truth"
    )
    result["controller_configuration_keys"] = controller_configuration_keys
    result["trajectory_execution_configured"] = bool(
        controller_configuration_keys
    )
    options.output_json.parent.mkdir(parents=True, exist_ok=True)
    options.output_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, sort_keys=True), flush=True)
    status = 0 if result["handoff_passed"] else 1
    # Keep MoveItPy alive until process exit.  Jazzy MoveItPy 2.12.4 can
    # segfault in its native destructor even after a read-only audit.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(status)


if __name__ == "__main__":
    main()
