"""Aggregate fresh-world dual-camera observation-only repetitions."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import statistics
from typing import Mapping, Sequence


def summarize_observation_repeats(
    runs: Sequence[Mapping[str, object]],
    *,
    minimum_runs: int = 5,
) -> dict[str, object]:
    """Require exact target localization in every non-pick repetition."""

    if minimum_runs <= 0:
        raise ValueError("minimum run count must be positive")
    if len(runs) < minimum_runs:
        raise ValueError(
            f"only {len(runs)} runs supplied; need at least {minimum_runs}"
        )

    violations = []
    target_pose_frames = []
    per_run = []
    candidate_ids = set()
    presets = set()
    readiness_status_counts: Counter[str] = Counter()
    readiness_reset_reason_counts: Counter[str] = Counter()
    readiness_reset_count = 0
    readiness_delay_count = 0
    readiness_delay_weighted_sum = 0.0
    readiness_delay_minima = []
    readiness_delay_maxima = []
    readiness_maximum_streaks = []
    for index, run in enumerate(runs, start=1):
        label = str(run.get("run_label", f"run_{index}"))
        matching_frames = int(
            run.get("wrist_matching_target_pose_frames", 0)
        )
        required_streak = int(
            run.get("wrist_readiness_consecutive_frames", 0)
        )
        maximum_streak = int(
            run.get("wrist_readiness_maximum_consecutive_frames", 0)
        )
        status_counts = run.get("wrist_readiness_status_counts", {})
        reset_reason_counts = run.get(
            "wrist_readiness_streak_reset_reason_counts", {}
        )
        delay_summary = run.get(
            "wrist_readiness_target_pose_delay_sec", {}
        )
        if not isinstance(status_counts, Mapping):
            status_counts = {}
        if not isinstance(reset_reason_counts, Mapping):
            reset_reason_counts = {}
        if not isinstance(delay_summary, Mapping):
            delay_summary = {}
        run_reset_count = int(
            run.get("wrist_readiness_streak_reset_count", 0)
        )
        delay_count = int(delay_summary.get("count", 0))
        delay_mean = delay_summary.get("mean")
        delay_minimum = delay_summary.get("minimum")
        delay_maximum = delay_summary.get("maximum")

        target_pose_frames.append(matching_frames)
        candidate_ids.add(int(run.get("candidate_target_id", 0)))
        presets.add(str(run.get("selected_preset", "")))
        readiness_maximum_streaks.append(maximum_streak)
        readiness_reset_count += run_reset_count
        readiness_status_counts.update(
            {
                str(name): int(count)
                for name, count in status_counts.items()
            }
        )
        readiness_reset_reason_counts.update(
            {
                str(name): int(count)
                for name, count in reset_reason_counts.items()
            }
        )
        if delay_count and delay_mean is not None:
            readiness_delay_count += delay_count
            readiness_delay_weighted_sum += (
                delay_count * float(delay_mean)
            )
        if delay_minimum is not None:
            readiness_delay_minima.append(float(delay_minimum))
        if delay_maximum is not None:
            readiness_delay_maxima.append(float(delay_maximum))

        run_violations = []
        if run.get("sequence_passed") is not True:
            run_violations.append("sequence did not pass")
        if run.get("clean_shutdown") is not True:
            run_violations.append("run logs contain an unhandled failure")
        if run.get("pick_authorized") is not False:
            run_violations.append("pick authorization was not false")
        if int(run.get("wrist_frame_count", 0)) != 60:
            run_violations.append("window did not contain 60 frames")
        if matching_frames != 60:
            run_violations.append("target identity was not localized in all frames")
        if run.get("wrist_readiness_satisfied") is not True:
            run_violations.append("readiness gate was not satisfied")
        if required_streak < 15:
            run_violations.append("readiness proof was shorter than 15 frames")
        if (
            int(run.get("schema_version", 1)) >= 3
            and maximum_streak < required_streak
        ):
            run_violations.append(
                "readiness telemetry did not prove the required streak"
            )
        if int(run.get("wrist_unripe_detection_count", 0)) != 0:
            run_violations.append("unripe detections were unexpectedly published")
        if run_violations:
            violations.extend(
                f"{label}: {message}" for message in run_violations
            )
        per_run.append(
            {
                "run_label": label,
                "candidate_target_id": int(
                    run.get("candidate_target_id", 0)
                ),
                "selected_preset": run.get("selected_preset"),
                "base_support_frames": int(
                    run.get("base_support_frames", 0)
                ),
                "wrist_matching_target_pose_frames": matching_frames,
                "wrist_detection_count": int(
                    run.get("wrist_detection_count", 0)
                ),
                "wrist_unripe_detection_count": int(
                    run.get("wrist_unripe_detection_count", 0)
                ),
                "stationary_sync_fallback_count": int(
                    run.get("stationary_sync_fallback_count", 0)
                ),
                "deferred_recovery_count": int(
                    run.get("deferred_recovery_count", 0)
                ),
                "wrist_readiness_maximum_consecutive_frames": maximum_streak,
                "wrist_readiness_streak_reset_count": run_reset_count,
                "wrist_readiness_streak_reset_reason_counts": dict(
                    sorted(
                        (str(name), int(count))
                        for name, count in reset_reason_counts.items()
                    )
                ),
                "wrist_readiness_status_counts": dict(
                    sorted(
                        (str(name), int(count))
                        for name, count in status_counts.items()
                    )
                ),
                "wrist_readiness_target_pose_delay_sec": dict(
                    delay_summary
                ),
                "clean_shutdown": run.get("clean_shutdown") is True,
                "violations": run_violations,
            }
        )

    if candidate_ids != {1}:
        violations.append("repetitions did not all select candidate target 1")
    if presets != {"lower"}:
        violations.append("repetitions did not all select the lower wrist preset")

    return {
        "schema_version": 2,
        "scope": "NON_ACCEPTANCE_DUAL_CAMERA_OBSERVATION_REPEAT",
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "pick_authorized": False,
        "repeat_passed": not violations,
        "run_count": len(runs),
        "total_wrist_frames": 60 * len(runs),
        "target_pose_frames": {
            "minimum": min(target_pose_frames),
            "mean": statistics.fmean(target_pose_frames),
            "maximum": max(target_pose_frames),
            "total": sum(target_pose_frames),
        },
        "wrist_readiness": {
            "maximum_consecutive_ready_frames": {
                "minimum": min(readiness_maximum_streaks),
                "maximum": max(readiness_maximum_streaks),
            },
            "streak_reset_count": readiness_reset_count,
            "streak_reset_reason_counts": dict(
                sorted(readiness_reset_reason_counts.items())
            ),
            "status_counts": dict(sorted(readiness_status_counts.items())),
            "target_pose_delay_sec": {
                "count": readiness_delay_count,
                "minimum": (
                    min(readiness_delay_minima)
                    if readiness_delay_minima
                    else None
                ),
                "mean": (
                    readiness_delay_weighted_sum / readiness_delay_count
                    if readiness_delay_count
                    else None
                ),
                "maximum": (
                    max(readiness_delay_maxima)
                    if readiness_delay_maxima
                    else None
                ),
            },
        },
        "candidate_target_ids": sorted(candidate_ids),
        "selected_presets": sorted(presets),
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


def main(args=None) -> int:
    """Read fresh result directories and write one immutable aggregate."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, action="append", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    options = parser.parse_args(args)
    if options.output_json.exists():
        raise ValueError(f"refusing to overwrite {options.output_json}")

    runs = []
    inputs = []
    for run_dir in options.run_dir:
        summary_path = run_dir / "sequence_summary.json"
        log_path = run_dir / "wrist_localization.log"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        log_text = log_path.read_text(encoding="utf-8")
        all_log_paths = sorted(run_dir.glob("*.log"))
        all_log_text = "\n".join(
            path.read_text(encoding="utf-8", errors="replace")
            for path in all_log_paths
        )
        summary["run_label"] = run_dir.name
        summary["clean_shutdown"] = not any(
            token in all_log_text
            for token in ("Traceback", "process has died", "RuntimeError")
        )
        summary["stationary_sync_fallback_count"] = log_text.count(
            "used bounded stationary sensor-sync fallback"
        )
        summary["deferred_recovery_count"] = log_text.count(
            "recovered deferred localization"
        )
        runs.append(summary)
        inputs.append(
            {
                "run_dir": str(run_dir),
                "sequence_summary": _fingerprint(summary_path),
                "wrist_localization_log": _fingerprint(log_path),
                "logs": [_fingerprint(path) for path in all_log_paths],
            }
        )

    result = summarize_observation_repeats(runs)
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
