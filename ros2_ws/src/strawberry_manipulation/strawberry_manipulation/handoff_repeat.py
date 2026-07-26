"""Summarize repeated no-motion observation-to-control handoff audits."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from typing import Mapping, Sequence


ERROR_LOG_TOKENS = (
    "traceback",
    "runtimeerror",
    "process has died",
    "exception was never retrieved",
    "segmentation fault",
)
CONTROL_LOG_TOKENS = (
    "added followjointtrajectory",
    "added grippercommand",
    "send_goal",
    "pick_and_place",
)


def _fingerprint(path: Path) -> dict[str, object]:
    data = path.read_bytes()
    return {
        "path": str(path),
        "bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def _contains_token(path: Path, tokens: Sequence[str]) -> bool:
    text = path.read_text(encoding="utf-8", errors="replace").lower()
    return any(token in text for token in tokens)


def summarize_handoff_repeats(
    runs: Sequence[Mapping[str, object]],
    *,
    minimum_runs: int = 3,
) -> dict[str, object]:
    """Require every repeated handoff to pass the same fail-closed gates."""

    if minimum_runs <= 0:
        raise ValueError("minimum_runs must be positive")
    violations: list[str] = []
    if len(runs) < minimum_runs:
        violations.append(
            f"repeat requires at least {minimum_runs} complete runs"
        )

    run_receipts: list[dict[str, object]] = []
    expected_target_ids: set[int] = set()
    maximum_target_ages: list[float] = []
    observed_joint_deltas: list[float] = []
    moveit_state_deltas: list[float] = []
    total_target_samples = 0
    total_wrist_frames = 0
    total_matching_frames = 0

    for index, run in enumerate(runs, start=1):
        label = str(run.get("label", f"run_{index}"))
        handoff = run.get("handoff", {})
        sequence = run.get("sequence", {})
        motion = run.get("motion", {})
        clean_logs = run.get("clean_logs") is True
        no_control_log = run.get("no_control_log") is True
        run_violations: list[str] = []

        expected_target_id = int(handoff.get("expected_target_id", 0))
        expected_target_ids.add(expected_target_id)
        observed_target_ids = sorted(
            int(value) for value in handoff.get("observed_target_ids", [])
        )
        target_sample_count = int(handoff.get("target_sample_count", 0))
        joint_sample_count = int(handoff.get("joint_sample_count", 0))
        try:
            maximum_age = float(
                handoff.get("target_age_sec", {}).get("maximum")
            )
            joint_delta = float(handoff.get("observed_joint_delta_rad"))
            moveit_delta = float(handoff.get("moveit_state_delta_rad"))
        except (TypeError, ValueError):
            maximum_age = joint_delta = moveit_delta = float("inf")
        collision_scene = handoff.get("collision_scene", {})

        if handoff.get("handoff_passed") is not True:
            run_violations.append("handoff audit did not pass")
        if sequence.get("sequence_passed") is not True:
            run_violations.append("dual-camera sequence did not pass")
        if handoff.get("pick_authorized") is not False:
            run_violations.append("handoff authorized a pick")
        if sequence.get("pick_authorized") is not False:
            run_violations.append("sequence authorized a pick")
        if any(
            handoff.get(field) is not False
            for field in (
                "control_interface_created",
                "pick_action_called",
                "trajectory_command_sent",
                "gripper_command_sent",
                "trajectory_execution_configured",
            )
        ):
            run_violations.append("handoff exposed or used a control interface")
        if handoff.get("controller_configuration_keys"):
            run_violations.append("handoff retained controller configuration")
        if expected_target_id <= 0 or observed_target_ids != [expected_target_id]:
            run_violations.append("handoff target identity is inconsistent")
        if target_sample_count < 15:
            run_violations.append("handoff has fewer than 15 target samples")
        if maximum_age > 0.5:
            run_violations.append("handoff target age exceeds 0.5 seconds")
        if joint_sample_count < 10:
            run_violations.append("handoff has fewer than 10 joint samples")
        if joint_delta > 0.002 or moveit_delta > 0.002:
            run_violations.append("arm was not stationary during handoff")
        if collision_scene.get("missing_ids"):
            run_violations.append("MoveIt collision scene is incomplete")
        if collision_scene.get("selected_fruit_collision_retained") is not True:
            run_violations.append("selected fruit collision was not retained")
        if len(collision_scene.get("observed_ids", [])) != 7:
            run_violations.append("MoveIt did not retain all seven scene objects")
        if int(sequence.get("wrist_frame_count", 0)) != 60:
            run_violations.append("wrist observation is not exactly 60 frames")
        if int(sequence.get("wrist_matching_target_pose_frames", 0)) != 60:
            run_violations.append("wrist TargetPose support is not 60/60")
        if motion.get("success") is not True:
            run_violations.append("wrist observation motion did not succeed")
        if motion.get("fruit_manipulation_started") is not False:
            run_violations.append("fruit manipulation started")
        if sorted(motion.get("fruit_collision_obstacle_ids", [])) != [1, 2, 3]:
            run_violations.append("observation motion omitted fruit obstacles")
        if not clean_logs:
            run_violations.append("run logs contain an unhandled failure")
        if not no_control_log:
            run_violations.append("handoff log contains a control endpoint")

        if run_violations:
            violations.extend(
                f"{label}: {reason}" for reason in run_violations
            )
        maximum_target_ages.append(maximum_age)
        observed_joint_deltas.append(joint_delta)
        moveit_state_deltas.append(moveit_delta)
        total_target_samples += target_sample_count
        total_wrist_frames += int(sequence.get("wrist_frame_count", 0))
        total_matching_frames += int(
            sequence.get("wrist_matching_target_pose_frames", 0)
        )
        run_receipts.append(
            {
                "label": label,
                "passed": not run_violations,
                "expected_target_id": expected_target_id,
                "target_sample_count": target_sample_count,
                "maximum_target_age_sec": maximum_age,
                "joint_sample_count": joint_sample_count,
                "observed_joint_delta_rad": joint_delta,
                "moveit_state_delta_rad": moveit_delta,
                "collision_object_count": len(
                    collision_scene.get("observed_ids", [])
                ),
                "wrist_matching_target_pose_frames": int(
                    sequence.get("wrist_matching_target_pose_frames", 0)
                ),
                "planning_attempt_count": int(
                    motion.get("planning_attempt_count", 0)
                ),
                "clean_logs": clean_logs,
                "no_control_log": no_control_log,
                "violations": run_violations,
            }
        )

    if len(expected_target_ids) != 1 or next(
        iter(expected_target_ids), 0
    ) <= 0:
        violations.append("repeated runs do not share one positive target identity")
    violations = list(dict.fromkeys(violations))
    return {
        "schema_version": 1,
        "scope": "NON_ACCEPTANCE_OBSERVATION_TO_CONTROL_HANDOFF_REPEAT",
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "repeat_passed": not violations,
        "pick_authorized": False,
        "run_count": len(runs),
        "minimum_runs": minimum_runs,
        "target_ids": sorted(expected_target_ids),
        "total_target_samples": total_target_samples,
        "maximum_target_age_sec": (
            max(maximum_target_ages) if maximum_target_ages else None
        ),
        "maximum_observed_joint_delta_rad": (
            max(observed_joint_deltas) if observed_joint_deltas else None
        ),
        "maximum_moveit_state_delta_rad": (
            max(moveit_state_deltas) if moveit_state_deltas else None
        ),
        "total_wrist_frames": total_wrist_frames,
        "total_wrist_matching_target_pose_frames": total_matching_frames,
        "control_command_count": 0,
        "runs": run_receipts,
        "violations": violations,
    }


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, action="append", required=True)
    parser.add_argument("--minimum-runs", type=int, default=3)
    parser.add_argument("--output-json", type=Path, required=True)
    options = parser.parse_args(args)
    if options.output_json.exists():
        raise ValueError(f"refusing to overwrite {options.output_json}")

    runs = []
    inputs = []
    for run_dir in options.run_dir:
        handoff_path = run_dir / "handoff_shadow.json"
        sequence_path = run_dir / "sequence_summary.json"
        motion_path = run_dir / "observation_motion.json"
        log_paths = sorted(run_dir.glob("*.log"))
        clean_logs = not any(
            _contains_token(path, ERROR_LOG_TOKENS) for path in log_paths
        )
        handoff_log = run_dir / "handoff_shadow.log"
        no_control_log = (
            handoff_log.is_file()
            and not _contains_token(handoff_log, CONTROL_LOG_TOKENS)
        )
        runs.append(
            {
                "label": run_dir.name,
                "handoff": json.loads(
                    handoff_path.read_text(encoding="utf-8")
                ),
                "sequence": json.loads(
                    sequence_path.read_text(encoding="utf-8")
                ),
                "motion": json.loads(
                    motion_path.read_text(encoding="utf-8")
                ),
                "clean_logs": clean_logs,
                "no_control_log": no_control_log,
            }
        )
        inputs.append(
            {
                "run_dir": str(run_dir),
                "handoff": _fingerprint(handoff_path),
                "sequence": _fingerprint(sequence_path),
                "motion": _fingerprint(motion_path),
                "logs": [_fingerprint(path) for path in log_paths],
            }
        )

    result = summarize_handoff_repeats(
        runs, minimum_runs=options.minimum_runs
    )
    result["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    result["inputs"] = inputs
    options.output_json.parent.mkdir(parents=True, exist_ok=True)
    options.output_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, sort_keys=True))
    return 0 if result["repeat_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
