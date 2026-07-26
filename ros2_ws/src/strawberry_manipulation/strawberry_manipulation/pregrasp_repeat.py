"""Aggregate controller-free pre-grasp planning Shadow repetitions."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics
from typing import Mapping, Sequence


def _finite_float(value) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def summarize_pregrasp_repeats(
    runs: Sequence[Mapping[str, object]],
    *,
    minimum_runs: int = 3,
    maximum_target_age_sec: float = 0.5,
    maximum_joint_delta_rad: float = 0.002,
) -> dict[str, object]:
    """Require every completed planning run to preserve the no-motion boundary."""

    if minimum_runs <= 0:
        raise ValueError("minimum run count must be positive")
    if len(runs) < minimum_runs:
        raise ValueError(
            f"only {len(runs)} completed planning runs supplied; "
            f"need at least {minimum_runs}"
        )

    violations: list[str] = []
    per_run: list[dict[str, object]] = []
    candidate_ids: set[int] = set()
    waypoint_counts: list[int] = []
    position_errors: list[float] = []
    orientation_errors: list[float] = []
    completion_ages: list[float] = []
    joint_deltas: list[float] = []
    moveit_deltas: list[float] = []
    total_target_samples = 0
    total_joint_samples = 0
    total_control_commands = 0
    total_wrist_target_pose_frames = 0

    for index, run in enumerate(runs, start=1):
        label = str(run.get("run_label", f"run_{index}"))
        sequence = run.get("sequence", {})
        handoff = run.get("handoff", {})
        pregrasp = run.get("pregrasp", {})
        run_violations: list[str] = []
        target_id = int(pregrasp.get("expected_target_id", 0))
        candidate_ids.add(target_id)

        if sequence.get("sequence_passed") is not True:
            run_violations.append("complete sequence did not pass")
        if sequence.get("formal_acceptance") is not False:
            run_violations.append("sequence was not explicitly non-acceptance")
        if sequence.get("held_out_test_consumed") is not False:
            run_violations.append("sequence consumed the held-out test")
        if sequence.get("pick_authorized") is not False:
            run_violations.append("sequence authorized a pick")
        if int(sequence.get("candidate_target_id", 0)) != target_id:
            run_violations.append("sequence and plan target identities differ")
        wrist_frames = int(
            sequence.get("wrist_matching_target_pose_frames", 0)
        )
        total_wrist_target_pose_frames += wrist_frames
        if wrist_frames != 60:
            run_violations.append("wrist target was not localized in all 60 frames")

        if handoff.get("handoff_passed") is not True:
            run_violations.append("upstream handoff did not pass")
        if int(handoff.get("expected_target_id", 0)) != target_id:
            run_violations.append("handoff and plan target identities differ")
        if handoff.get("pick_authorized") is not False:
            run_violations.append("handoff authorized a pick")

        if pregrasp.get("scope") != "NON_ACCEPTANCE_PREGRASP_PLANNING_SHADOW":
            run_violations.append("pre-grasp plan has an unexpected scope")
        if pregrasp.get("planning_passed") is not True:
            run_violations.append("pre-grasp planning audit did not pass")
        if pregrasp.get("formal_acceptance") is not False:
            run_violations.append("pre-grasp plan was not explicitly non-acceptance")
        if pregrasp.get("held_out_test_consumed") is not False:
            run_violations.append("pre-grasp plan consumed the held-out test")
        if pregrasp.get("pick_authorized") is not False:
            run_violations.append("pre-grasp plan authorized a pick")
        if pregrasp.get("trajectory_generated") is not True:
            run_violations.append("pre-grasp plan generated no trajectory")
        if pregrasp.get("trajectory_discarded") is not True:
            run_violations.append("pre-grasp trajectory was not discarded")
        if pregrasp.get("trajectory_executed") is not False:
            run_violations.append("pre-grasp trajectory was executed")
        if pregrasp.get("trajectory_execution_requested") is not False:
            run_violations.append("pre-grasp trajectory execution was requested")
        if float(pregrasp.get("trajectory_execution_time_sec", -1.0)) != 0.0:
            run_violations.append("pre-grasp trajectory execution time is nonzero")
        if pregrasp.get("control_interface_created") is not False:
            run_violations.append("pre-grasp plan created a control interface")
        if pregrasp.get("controller_configuration_keys"):
            run_violations.append("pre-grasp config retained controller parameters")
        command_count = int(pregrasp.get("control_command_count", -1))
        total_control_commands += max(0, command_count)
        if command_count != 0:
            run_violations.append("pre-grasp plan emitted a control command")
        if pregrasp.get("pick_action_called") is not False:
            run_violations.append("pre-grasp plan called the pick action")
        if pregrasp.get("gripper_command_sent") is not False:
            run_violations.append("pre-grasp plan sent a gripper command")
        if run.get("control_logs_clean") is not True:
            run_violations.append("control-side logs violate the no-motion audit")
        if run.get("all_logs_clean") is not True:
            run_violations.append("run logs contain an unhandled failure")

        target_samples = int(pregrasp.get("target_sample_count", 0))
        joint_samples = int(pregrasp.get("joint_sample_count", 0))
        total_target_samples += target_samples
        total_joint_samples += joint_samples
        if target_samples < 15:
            run_violations.append("fewer than 15 fresh planning targets")
        if joint_samples < 10:
            run_violations.append("fewer than 10 stationary joint samples")

        joint_delta = _finite_float(pregrasp.get("observed_joint_delta_rad"))
        moveit_delta = _finite_float(pregrasp.get("moveit_state_delta_rad"))
        if joint_delta is None or joint_delta > maximum_joint_delta_rad:
            run_violations.append("joint-state stationarity bound failed")
        else:
            joint_deltas.append(joint_delta)
        if moveit_delta is None or moveit_delta > maximum_joint_delta_rad:
            run_violations.append("MoveIt stationarity bound failed")
        else:
            moveit_deltas.append(moveit_delta)

        completion_age = _finite_float(
            pregrasp.get("target_age_sec", {}).get("at_plan_completion")
        )
        if (
            completion_age is None
            or completion_age < -0.05
            or completion_age > maximum_target_age_sec
        ):
            run_violations.append("target was not fresh at plan completion")
        else:
            completion_ages.append(completion_age)

        collision_scene = pregrasp.get("collision_scene", {})
        before_ids = list(collision_scene.get("before_ids", []))
        after_ids = list(collision_scene.get("after_ids", []))
        if len(before_ids) != 7 or len(after_ids) != 7:
            run_violations.append("planning collision scene did not retain 7 objects")
        if (
            collision_scene.get("missing_before")
            or collision_scene.get("missing_after")
        ):
            run_violations.append("planning collision scene is incomplete")
        if collision_scene.get("selected_fruit_collision_retained") is not True:
            run_violations.append("selected fruit collision object was removed")

        successful_attempts = [
            attempt
            for attempt in pregrasp.get("planning_attempts", [])
            if attempt.get("success") is True
        ]
        accepted = successful_attempts[0] if len(successful_attempts) == 1 else {}
        if len(successful_attempts) != 1:
            run_violations.append("run does not contain exactly one accepted plan")
        waypoint_count = int(accepted.get("trajectory_waypoint_count", 0))
        position_error = _finite_float(
            accepted.get("endpoint_position_error_m")
        )
        orientation_error = _finite_float(
            accepted.get("endpoint_orientation_error_rad")
        )
        if waypoint_count < 2:
            run_violations.append("accepted trajectory has fewer than 2 waypoints")
        else:
            waypoint_counts.append(waypoint_count)
        if position_error is None or position_error > 0.01:
            run_violations.append("accepted endpoint position error is too large")
        else:
            position_errors.append(position_error)
        if orientation_error is None or orientation_error > 0.0872665:
            run_violations.append("accepted endpoint orientation error is too large")
        else:
            orientation_errors.append(orientation_error)

        if run_violations:
            violations.extend(
                f"{label}: {message}" for message in run_violations
            )
        per_run.append(
            {
                "run_label": label,
                "target_id": target_id,
                "wrist_matching_target_pose_frames": wrist_frames,
                "target_sample_count": target_samples,
                "joint_sample_count": joint_samples,
                "observed_joint_delta_rad": joint_delta,
                "moveit_state_delta_rad": moveit_delta,
                "target_age_at_plan_completion_sec": completion_age,
                "planning_attempt_count": int(
                    pregrasp.get("planning_attempt_count", 0)
                ),
                "trajectory_waypoint_count": waypoint_count,
                "endpoint_position_error_m": position_error,
                "endpoint_orientation_error_rad": orientation_error,
                "trajectory_discarded": (
                    pregrasp.get("trajectory_discarded") is True
                ),
                "control_command_count": command_count,
                "collision_object_count_before": len(before_ids),
                "collision_object_count_after": len(after_ids),
                "violations": run_violations,
            }
        )

    if candidate_ids != {1}:
        violations.append("completed planning runs do not all target fruit 1")

    def _stats(values: Sequence[float | int]) -> dict[str, float | int | None]:
        return {
            "minimum": min(values) if values else None,
            "mean": statistics.fmean(values) if values else None,
            "maximum": max(values) if values else None,
        }

    return {
        "schema_version": 1,
        "scope": (
            "NON_ACCEPTANCE_PREGRASP_PLANNING_REPEAT_"
            "CONDITIONAL_ON_UPSTREAM_HANDOFF"
        ),
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "pick_authorized": False,
        "repeat_passed": not violations,
        "completed_planning_run_count": len(runs),
        "planning_success_count": sum(
            pregrasp.get("planning_passed") is True
            for pregrasp in (run.get("pregrasp", {}) for run in runs)
        ),
        "candidate_target_ids": sorted(candidate_ids),
        "total_wrist_target_pose_frames": total_wrist_target_pose_frames,
        "total_target_samples": total_target_samples,
        "total_joint_samples": total_joint_samples,
        "total_control_command_count": total_control_commands,
        "trajectory_waypoint_count": _stats(waypoint_counts),
        "endpoint_position_error_m": _stats(position_errors),
        "endpoint_orientation_error_rad": _stats(orientation_errors),
        "target_age_at_plan_completion_sec": _stats(completion_ages),
        "observed_joint_delta_rad": _stats(joint_deltas),
        "moveit_state_delta_rad": _stats(moveit_deltas),
        "violations": violations,
        "runs": per_run,
    }


def _fingerprint(path: Path) -> dict[str, object]:
    data = path.read_bytes()
    return {
        "path": str(path),
        "bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def _read_logs(run_dir: Path) -> tuple[list[Path], str]:
    paths = sorted(run_dir.glob("*.log"))
    return paths, "\n".join(
        path.read_text(encoding="utf-8", errors="replace") for path in paths
    )


def main(args=None) -> int:
    """Read immutable run directories and write one conditional aggregate."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, action="append", required=True)
    parser.add_argument("--excluded-run-dir", type=Path, action="append", default=[])
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--minimum-runs", type=int, default=3)
    options = parser.parse_args(args)
    if options.output_json.exists():
        raise ValueError(f"refusing to overwrite {options.output_json}")

    runs = []
    inputs = []
    unhandled_tokens = (
        "Traceback",
        "process has died",
        "RuntimeError",
        "exception was never retrieved",
    )
    controller_tokens = ("Added FollowJointTrajectory", "Added GripperCommand")
    for run_dir in options.run_dir:
        sequence_path = run_dir / "sequence_summary.json"
        handoff_path = run_dir / "handoff_shadow.json"
        pregrasp_path = run_dir / "pregrasp_shadow.json"
        log_paths, all_log_text = _read_logs(run_dir)
        control_log_paths = [
            run_dir / "handoff_shadow.log",
            run_dir / "pregrasp_shadow.log",
        ]
        control_log_text = "\n".join(
            path.read_text(encoding="utf-8", errors="replace")
            for path in control_log_paths
        )
        runs.append(
            {
                "run_label": run_dir.name,
                "sequence": json.loads(
                    sequence_path.read_text(encoding="utf-8")
                ),
                "handoff": json.loads(
                    handoff_path.read_text(encoding="utf-8")
                ),
                "pregrasp": json.loads(
                    pregrasp_path.read_text(encoding="utf-8")
                ),
                "all_logs_clean": not any(
                    token in all_log_text for token in unhandled_tokens
                ),
                "control_logs_clean": not any(
                    token in control_log_text for token in controller_tokens
                ),
            }
        )
        inputs.append(
            {
                "run_dir": str(run_dir),
                "sequence_summary": _fingerprint(sequence_path),
                "handoff_shadow": _fingerprint(handoff_path),
                "pregrasp_shadow": _fingerprint(pregrasp_path),
                "logs": [_fingerprint(path) for path in log_paths],
            }
        )

    excluded_runs = []
    for run_dir in options.excluded_run_dir:
        log_paths, all_log_text = _read_logs(run_dir)
        reason = "upstream sequence incomplete before pre-grasp planning"
        if "timed out waiting for 15 consecutive detection frames" in all_log_text:
            reason = (
                "wrist readiness timed out before handoff: no 15 consecutive "
                "detection frames with matching TargetPose"
            )
        excluded_runs.append(
            {
                "run_dir": str(run_dir),
                "reason": reason,
                "pregrasp_planning_started": (
                    (run_dir / "pregrasp_shadow.json").exists()
                ),
                "logs": [_fingerprint(path) for path in log_paths],
            }
        )

    result = summarize_pregrasp_repeats(
        runs, minimum_runs=options.minimum_runs
    )
    attempted_count = len(runs) + len(excluded_runs)
    result["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    result["attempted_fresh_world_count"] = attempted_count
    result["upstream_incomplete_run_count"] = len(excluded_runs)
    result["end_to_end_sequence_completion_rate"] = (
        len(runs) / attempted_count if attempted_count else 0.0
    )
    result["all_attempted_worlds_completed"] = not excluded_runs
    result["end_to_end_repeat_passed"] = (
        result["repeat_passed"] and not excluded_runs
    )
    result["excluded_runs"] = excluded_runs
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
