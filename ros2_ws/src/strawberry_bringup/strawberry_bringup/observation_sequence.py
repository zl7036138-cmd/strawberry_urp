"""Summarize one same-world base-to-wrist sequential observation."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from typing import Mapping


def summarize_observation_sequence(
    selection: Mapping[str, object],
    motion: Mapping[str, object],
    wrist_window: Mapping[str, object],
    handoff_shadow: Mapping[str, object] | None = None,
    pregrasp_shadow: Mapping[str, object] | None = None,
    *,
    minimum_matching_pose_frames: int = 48,
) -> dict[str, object]:
    """Validate the fail-closed transition without authorizing a pick."""

    if minimum_matching_pose_frames <= 0:
        raise ValueError("minimum matching pose frames must be positive")
    expected_target_id = int(selection.get("candidate_target_id", 0))
    frames = list(wrist_window.get("frames", []))
    pose_counts: Counter[int] = Counter(
        int(identity)
        for frame in frames
        for identity in set(frame.get("target_pose_ids", []))
        if int(identity) > 0
    )
    matching_pose_frames = pose_counts.get(expected_target_id, 0)
    unexpected_pose_ids = sorted(set(pose_counts) - {expected_target_id})
    motion_obstacles = sorted(
        int(value) for value in motion.get("fruit_collision_obstacle_ids", [])
    )
    summary = wrist_window.get("summary", {})
    readiness = wrist_window.get("readiness_gate", {})
    readiness_required = int(
        readiness.get("required_consecutive_target_pose_frames", 0)
    )
    violations = []
    if selection.get("wrist_observation_authorized") is not True:
        violations.append("base selection did not authorize wrist observation")
    if selection.get("pick_authorized") is not False:
        violations.append("base selection unexpectedly authorized a pick")
    if motion.get("success") is not True:
        violations.append("wrist observation motion failed")
    if motion.get("fruit_manipulation_started") is not False:
        violations.append("fruit manipulation started during observation motion")
    if motion_obstacles != [1, 2, 3]:
        violations.append("observation motion omitted one or more fruit obstacles")
    if wrist_window.get("formal_acceptance") is not False:
        violations.append("wrist window is not explicitly non-acceptance")
    if (
        wrist_window.get("window_boundary")
        != "after_wrist_observation_settle_before_pick_motion"
    ):
        violations.append("wrist window is not after settle and before pick motion")
    if readiness.get("satisfied") is not True or readiness_required < 10:
        violations.append(
            "wrist pipeline did not prove at least 10 consecutive ready frames"
        )
    if int(summary.get("frame_count", 0)) != 60:
        violations.append("wrist window does not contain exactly 60 frames")
    if matching_pose_frames < minimum_matching_pose_frames:
        violations.append("wrist target identity support is below the threshold")
    if unexpected_pose_ids:
        violations.append("wrist window published an unexpected target identity")
    if handoff_shadow is not None:
        if (
            handoff_shadow.get("scope")
            != "NON_ACCEPTANCE_OBSERVATION_TO_CONTROL_HANDOFF_SHADOW"
        ):
            violations.append("control handoff has an unexpected scope")
        if handoff_shadow.get("handoff_passed") is not True:
            violations.append("control handoff shadow audit did not pass")
        if int(handoff_shadow.get("expected_target_id", 0)) != expected_target_id:
            violations.append("control handoff expected a different target identity")
        if handoff_shadow.get("pick_authorized") is not False:
            violations.append("control handoff unexpectedly authorized a pick")
        if handoff_shadow.get("pick_action_called") is not False:
            violations.append("control handoff called the pick action")
        if handoff_shadow.get("trajectory_command_sent") is not False:
            violations.append("control handoff sent an arm trajectory command")
        if handoff_shadow.get("gripper_command_sent") is not False:
            violations.append("control handoff sent a gripper command")
        collision_scene = handoff_shadow.get("collision_scene", {})
        if collision_scene.get("missing_ids"):
            violations.append("control handoff collision scene is incomplete")
        if collision_scene.get("selected_fruit_collision_retained") is not True:
            violations.append(
                "control handoff removed the selected fruit collision object"
            )
    if pregrasp_shadow is not None:
        if handoff_shadow is None:
            violations.append("pre-grasp planning has no upstream handoff audit")
        if (
            pregrasp_shadow.get("scope")
            != "NON_ACCEPTANCE_PREGRASP_PLANNING_SHADOW"
        ):
            violations.append("pre-grasp planning has an unexpected scope")
        if pregrasp_shadow.get("planning_passed") is not True:
            violations.append("pre-grasp planning shadow audit did not pass")
        if int(pregrasp_shadow.get("expected_target_id", 0)) != expected_target_id:
            violations.append("pre-grasp planning expected a different target")
        if pregrasp_shadow.get("pick_authorized") is not False:
            violations.append("pre-grasp planning unexpectedly authorized a pick")
        if pregrasp_shadow.get("trajectory_generated") is not True:
            violations.append("pre-grasp planning did not generate a trajectory")
        if pregrasp_shadow.get("trajectory_discarded") is not True:
            violations.append("pre-grasp trajectory was not discarded")
        if pregrasp_shadow.get("trajectory_executed") is not False:
            violations.append("pre-grasp trajectory was executed")
        if pregrasp_shadow.get("trajectory_execution_requested") is not False:
            violations.append("pre-grasp trajectory execution was requested")
        if float(
            pregrasp_shadow.get("trajectory_execution_time_sec", -1.0)
        ) != 0.0:
            violations.append("pre-grasp trajectory execution time is nonzero")
        if int(pregrasp_shadow.get("control_command_count", -1)) != 0:
            violations.append("pre-grasp planning emitted a control command")
        if pregrasp_shadow.get("controller_configuration_keys"):
            violations.append("pre-grasp planning retained controller parameters")
        if pregrasp_shadow.get("control_interface_created") is not False:
            violations.append("pre-grasp planning created a control interface")
        if pregrasp_shadow.get("pick_action_called") is not False:
            violations.append("pre-grasp planning called the pick action")
        if pregrasp_shadow.get("gripper_command_sent") is not False:
            violations.append("pre-grasp planning sent a gripper command")
        pregrasp_collision_scene = pregrasp_shadow.get("collision_scene", {})
        if (
            pregrasp_collision_scene.get("missing_before")
            or pregrasp_collision_scene.get("missing_after")
        ):
            violations.append("pre-grasp planning collision scene is incomplete")
        if (
            pregrasp_collision_scene.get(
                "selected_fruit_collision_retained"
            )
            is not True
        ):
            violations.append(
                "pre-grasp planning removed the selected fruit collision object"
            )

    state_history = [
        "OVERVIEW_ACQUIRE",
        "OVERVIEW_CANDIDATE_VALIDATED",
        "WRIST_PRESET_SELECTED",
        "BASE_PIPELINE_STOPPED",
        "WRIST_OBSERVATION_MOVE",
        "WRIST_STATIONARY",
        "WRIST_PIPELINE_READY",
        "WRIST_ATTENTION_LOCALIZE",
    ]
    if handoff_shadow is not None:
        state_history.append("CONTROL_HANDOFF_SHADOW_VALIDATED")
    if pregrasp_shadow is not None:
        state_history.append("PREGRASP_PLANNING_SHADOW_VALIDATED")
    state_history.append(
        "OBSERVATION_COMPLETE" if not violations else "OBSERVATION_FAILED"
    )

    return {
        "schema_version": (
            3
            if pregrasp_shadow is not None
            else 2
            if handoff_shadow is not None
            else 1
        ),
        "scope": "NON_ACCEPTANCE_DUAL_CAMERA_SEQUENTIAL_OBSERVATION",
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "sequence_passed": not violations,
        "pick_authorized": False,
        "candidate_target_id": expected_target_id,
        "selected_preset": selection.get("selected_preset"),
        "focus_roi_xyxy_px": selection.get("focus_roi_xyxy_px"),
        "base_support_frames": int(selection.get("support_frames", 0)),
        "base_frame_count": int(selection.get("frame_count", 0)),
        "wrist_frame_count": int(summary.get("frame_count", 0)),
        "wrist_detection_count": int(summary.get("detection_count", 0)),
        "wrist_ripe_detection_count": int(
            summary.get("ripe_detection_count", 0)
        ),
        "wrist_unripe_detection_count": int(
            summary.get("unripe_detection_count", 0)
        ),
        "wrist_readiness_consecutive_frames": readiness_required,
        "wrist_readiness_satisfied": readiness.get("satisfied") is True,
        "wrist_matching_target_pose_frames": matching_pose_frames,
        "wrist_target_pose_counts": {
            str(target_id): count
            for target_id, count in sorted(pose_counts.items())
        },
        "handoff_shadow_included": handoff_shadow is not None,
        "handoff_shadow_passed": (
            handoff_shadow.get("handoff_passed") is True
            if handoff_shadow is not None
            else None
        ),
        "handoff_target_sample_count": (
            int(handoff_shadow.get("target_sample_count", 0))
            if handoff_shadow is not None
            else 0
        ),
        "handoff_joint_sample_count": (
            int(handoff_shadow.get("joint_sample_count", 0))
            if handoff_shadow is not None
            else 0
        ),
        "handoff_observed_joint_delta_rad": (
            handoff_shadow.get("observed_joint_delta_rad")
            if handoff_shadow is not None
            else None
        ),
        "handoff_moveit_state_delta_rad": (
            handoff_shadow.get("moveit_state_delta_rad")
            if handoff_shadow is not None
            else None
        ),
        "pregrasp_shadow_included": pregrasp_shadow is not None,
        "pregrasp_shadow_passed": (
            pregrasp_shadow.get("planning_passed") is True
            if pregrasp_shadow is not None
            else None
        ),
        "pregrasp_target_sample_count": (
            int(pregrasp_shadow.get("target_sample_count", 0))
            if pregrasp_shadow is not None
            else 0
        ),
        "pregrasp_joint_sample_count": (
            int(pregrasp_shadow.get("joint_sample_count", 0))
            if pregrasp_shadow is not None
            else 0
        ),
        "pregrasp_planning_attempt_count": (
            int(pregrasp_shadow.get("planning_attempt_count", 0))
            if pregrasp_shadow is not None
            else 0
        ),
        "pregrasp_target_age_at_plan_completion_sec": (
            pregrasp_shadow.get("target_age_sec", {}).get(
                "at_plan_completion"
            )
            if pregrasp_shadow is not None
            else None
        ),
        "pregrasp_trajectory_discarded": (
            pregrasp_shadow.get("trajectory_discarded") is True
            if pregrasp_shadow is not None
            else None
        ),
        "pregrasp_control_command_count": (
            int(pregrasp_shadow.get("control_command_count", -1))
            if pregrasp_shadow is not None
            else None
        ),
        "motion_planning_time_sec": float(
            motion.get("planning_time_sec", 0.0)
        ),
        "motion_execution_time_sec": float(
            motion.get("execution_time_sec", 0.0)
        ),
        "violations": violations,
        "state_history": state_history,
    }


def _fingerprint(path: Path) -> dict[str, object]:
    data = path.read_bytes()
    return {
        "path": str(path),
        "bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection-json", type=Path, required=True)
    parser.add_argument("--motion-json", type=Path, required=True)
    parser.add_argument("--wrist-window-json", type=Path, required=True)
    parser.add_argument("--handoff-shadow-json", type=Path)
    parser.add_argument("--pregrasp-shadow-json", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    options = parser.parse_args(args)
    if options.output_json.exists():
        raise ValueError(f"refusing to overwrite {options.output_json}")
    selection = json.loads(options.selection_json.read_text(encoding="utf-8"))
    motion = json.loads(options.motion_json.read_text(encoding="utf-8"))
    wrist_window = json.loads(
        options.wrist_window_json.read_text(encoding="utf-8")
    )
    handoff_shadow = (
        json.loads(options.handoff_shadow_json.read_text(encoding="utf-8"))
        if options.handoff_shadow_json is not None
        else None
    )
    pregrasp_shadow = (
        json.loads(options.pregrasp_shadow_json.read_text(encoding="utf-8"))
        if options.pregrasp_shadow_json is not None
        else None
    )
    result = summarize_observation_sequence(
        selection, motion, wrist_window, handoff_shadow, pregrasp_shadow
    )
    result["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    result["inputs"] = {
        "selection": _fingerprint(options.selection_json),
        "motion": _fingerprint(options.motion_json),
        "wrist_window": _fingerprint(options.wrist_window_json),
    }
    if options.handoff_shadow_json is not None:
        result["inputs"]["handoff_shadow"] = _fingerprint(
            options.handoff_shadow_json
        )
    if options.pregrasp_shadow_json is not None:
        result["inputs"]["pregrasp_shadow"] = _fingerprint(
            options.pregrasp_shadow_json
        )
    options.output_json.parent.mkdir(parents=True, exist_ok=True)
    options.output_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, sort_keys=True))
    return 0 if result["sequence_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
