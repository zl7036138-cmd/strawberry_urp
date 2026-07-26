#!/usr/bin/env python3
"""Summarize T60 oracle-controlled orchestrator trials."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


SHUTDOWN_ERROR_MARKERS = (
    "exception was never retrieved",
    "failed to terminate",
    "traceback (most recent call last)",
    "process has died",
)

REPO_ROOT = Path(__file__).resolve().parents[1]
PROVENANCE_PATHS = (
    "ros2_ws/src/strawberry_bringup/strawberry_bringup/core.py",
    "ros2_ws/src/strawberry_bringup/strawberry_bringup/oracle_target_provider.py",
    "ros2_ws/src/strawberry_bringup/strawberry_bringup/orchestrator.py",
    "ros2_ws/src/strawberry_bringup/launch/system.launch.py",
    "ros2_ws/src/strawberry_localization/strawberry_localization/node.py",
    "ros2_ws/src/strawberry_manipulation/strawberry_manipulation/moveit_backend.py",
    "scripts/test_orchestrated_trial.py",
    "scripts/run_t60_oracle_gate.sh",
    "scripts/summarize_t60_oracle_gate.py",
)


def _fingerprint(path: Path) -> dict:
    content = path.read_bytes()
    return {
        "path": path.relative_to(REPO_ROOT).as_posix(),
        "size_bytes": len(content),
        "sha256": hashlib.sha256(content).hexdigest(),
    }


def _file_sha256(path: Path) -> str | None:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def _percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def summarize(
    output_dir: Path,
    expected_trials: int,
    base_domain_id: int | None = None,
) -> dict:
    if expected_trials <= 0:
        raise ValueError("expected_trials must be positive")
    if base_domain_id is not None and not 0 <= base_domain_id <= 232:
        raise ValueError("base_domain_id must be within [0, 232]")
    if base_domain_id is not None and base_domain_id + expected_trials - 1 > 232:
        raise ValueError("ROS domain range exceeds 232")
    records = []
    for index in range(1, expected_trials + 1):
        label = f"trial_{index:02d}"
        result_path = output_dir / f"{label}.json"
        launch_path = output_dir / f"{label}_launch.log"
        payload = (
            json.loads(result_path.read_text(encoding="utf-8"))
            if result_path.exists()
            else {"success": False, "error": "result JSON was not produced"}
        )
        launch_text = (
            launch_path.read_text(encoding="utf-8", errors="replace").lower()
            if launch_path.exists()
            else "launch log missing"
        )
        planning_time = payload.get("planning_time_sec")
        record = {
            "trial_id": label,
            "success": bool(payload.get("success", False)),
            "source_isolated": bool(payload.get("source_isolated", False)),
            "state_machine_complete": bool(
                payload.get("state_machine_complete", False)
            ),
            "planning_time_sec": planning_time,
            "planning_time_within_5_sec": (
                planning_time is not None and float(planning_time) <= 5.0
            ),
            "shutdown_clean": not any(
                marker in launch_text for marker in SHUTDOWN_ERROR_MARKERS
            ),
            "endpoint_correction_count": launch_text.count(
                "applying one bounded correction"
            ),
            "ros_domain_id": (
                base_domain_id + index - 1
                if base_domain_id is not None
                else None
            ),
            "failure_code": payload.get("failure_code"),
            "message": payload.get("failure_message", payload.get("error", "")),
            "result_file": result_path.name,
            "launch_log": launch_path.name,
            "result_sha256": _file_sha256(result_path),
            "launch_log_sha256": _file_sha256(launch_path),
        }
        records.append(record)
    successes = sum(record["success"] for record in records)
    success_rate = successes / expected_trials
    planning_times = [
        float(record["planning_time_sec"])
        for record in records
        if record["planning_time_sec"] is not None
    ]
    planning_time_p95 = _percentile(planning_times, 0.95)
    gate_passed = (
        success_rate >= 0.90
        and all(record["source_isolated"] for record in records)
        and all(record["state_machine_complete"] for record in records)
        and all(record["shutdown_clean"] for record in records)
        and all(
            record["planning_time_within_5_sec"]
            for record in records
            if record["success"]
        )
    )
    return {
        "schema_version": 1,
        "gate": "T60_ORACLE_INTEGRATION",
        "control_source": "oracle",
        "held_out_test_consumed": False,
        "trial_count": expected_trials,
        "successes": successes,
        "success_rate": success_rate,
        "required_success_rate": 0.90,
        "gate_passed": gate_passed,
        "planning_time_p95_sec": planning_time_p95,
        "planning_time_p95_limit_sec": 5.0,
        "planning_time_p95_passed": (
            planning_time_p95 is not None and planning_time_p95 <= 5.0
        ),
        "endpoint_correction_count": sum(
            record["endpoint_correction_count"] for record in records
        ),
        "ros_domain_ids": [record["ros_domain_id"] for record in records],
        "provenance": [
            _fingerprint(REPO_ROOT / relative_path)
            for relative_path in PROVENANCE_PATHS
        ],
        "trials": records,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-trials", type=int, required=True)
    parser.add_argument("--base-domain-id", type=int)
    arguments = parser.parse_args()
    result = summarize(
        arguments.output_dir,
        arguments.expected_trials,
        arguments.base_domain_id,
    )
    summary_path = arguments.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["gate_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
