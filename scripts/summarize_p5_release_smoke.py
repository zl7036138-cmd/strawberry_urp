#!/usr/bin/env python3
"""Summarize a ten-trial P5 release smoke and optionally compare a reference."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "scripts"))

from validate_p5_release_smoke import load_contract, sha256_file  # noqa: E402


SHUTDOWN_ERROR_MARKERS = (
    "exception was never retrieved",
    "failed to terminate",
    "traceback (most recent call last)",
    "process has died",
)


def _percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def _binding(path: Path) -> dict:
    return {
        "path": path.resolve().relative_to(REPOSITORY_ROOT.resolve()).as_posix(),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def summarize(
    output_dir: Path,
    config_path: Path,
    environment_role: str,
    base_domain_id: int,
    reference_summary: Path | None,
) -> dict:
    contract = load_contract(config_path)
    root = contract["raw"]
    source = contract["source"]
    waiver = contract["waiver"]
    verified = contract["verified"]
    execution = root["execution"]
    acceptance = root["acceptance"]
    records = []
    global_index = 0
    for mode, count in (
        ("negative", execution["negative_trials"]),
        ("positive", execution["positive_trials"]),
    ):
        for index in range(1, count + 1):
            global_index += 1
            label = f"{mode}_{index:02d}"
            result_path = output_dir / f"{label}.json"
            launch_path = output_dir / f"{label}_launch.log"
            payload = (
                json.loads(result_path.read_text(encoding="utf-8"))
                if result_path.is_file()
                else {"error": "result JSON missing"}
            )
            launch_text = (
                launch_path.read_text(encoding="utf-8", errors="replace").lower()
                if launch_path.is_file()
                else "launch log missing"
            )
            final_status = payload.get("final_status", {})
            routing_valid = (
                final_status.get("target_source") == "perception"
                and final_status.get("control_target_topic") == waiver.control_target_topic
                and final_status.get("shadow_target_topic") != waiver.control_target_topic
            )
            scene_configured = bool(payload.get("position_configuration_applied")) and bool(
                payload.get("parked_model_configuration_applied")
            )
            settled_frames = int(payload.get("settled_window_detection_frames", 0))
            liveness_valid = mode != "negative" or settled_frames >= source.minimum_settled_detection_frames
            shutdown_clean = launch_path.is_file() and not any(
                marker in launch_text for marker in SHUTDOWN_ERROR_MARKERS
            )
            infrastructure_valid = (
                result_path.is_file()
                and "error" not in payload
                and scene_configured
                and routing_valid
                and liveness_valid
            )
            positive_success = (
                mode == "positive"
                and bool(payload.get("manipulation_success"))
                and bool(payload.get("state_machine_complete"))
                and final_status.get("outcome") == "SUCCESS"
            )
            negative_no_pick = (
                mode == "negative"
                and bool(payload.get("safe_no_pick"))
                and final_status.get("outcome") == "NO_PICK"
            )
            records.append(
                {
                    "trial_id": label,
                    "mode": mode,
                    "ros_domain_id": base_domain_id + global_index - 1,
                    "infrastructure_valid": infrastructure_valid,
                    "routing_valid": routing_valid,
                    "scene_configured": scene_configured,
                    "liveness_valid": liveness_valid,
                    "shutdown_clean": shutdown_clean,
                    "positive_success": positive_success,
                    "negative_no_pick": negative_no_pick,
                    "false_pick": mode == "negative" and bool(payload.get("fruit_picked")),
                    "negative_control_attempt": mode == "negative" and bool(payload.get("unsafe_control_attempt")),
                    "terminal_state": final_status.get("state"),
                    "outcome": final_status.get("outcome"),
                    "planning_time_sec": payload.get("planning_time_sec"),
                    "execution_time_sec": payload.get("execution_time_sec"),
                    "result": _binding(result_path) if result_path.is_file() else None,
                    "launch_log": _binding(launch_path) if launch_path.is_file() else None,
                }
            )
    positives = [item for item in records if item["mode"] == "positive"]
    negatives = [item for item in records if item["mode"] == "negative"]
    positive_successes = sum(item["positive_success"] for item in positives)
    negative_no_picks = sum(item["negative_no_pick"] for item in negatives)
    false_picks = sum(item["false_pick"] for item in negatives)
    negative_control_attempts = sum(item["negative_control_attempt"] for item in negatives)
    positive_rate = positive_successes / len(positives)
    negative_rate = negative_no_picks / len(negatives)
    behavior_passes = positive_successes + negative_no_picks
    behavior_rate = behavior_passes / len(records)
    planning_times = [
        float(item["planning_time_sec"])
        for item in positives
        if item["positive_success"] and item["planning_time_sec"] is not None
    ]
    planning_p95 = _percentile(planning_times, 0.95)
    smoke_passed = (
        all(item["infrastructure_valid"] for item in records)
        and all(item["shutdown_clean"] for item in records)
        and positive_rate >= acceptance["minimum_positive_success_rate"]
        and negative_rate >= acceptance["minimum_negative_no_pick_rate"]
        and false_picks <= acceptance["maximum_negative_false_picks"]
        and negative_control_attempts <= acceptance["maximum_negative_control_attempts"]
        and planning_p95 is not None
        and planning_p95 <= acceptance["planning_time_p95_limit_sec"]
    )
    comparison = {
        "reference_provided": reference_summary is not None,
        "reference_behavior_pass_rate": None,
        "absolute_difference": None,
        "maximum_difference": acceptance["maximum_reference_difference"],
        "passed": None,
    }
    if reference_summary is not None:
        reference = json.loads(reference_summary.read_text(encoding="utf-8"))
        if reference.get("smoke_id") != root["smoke_id"]:
            raise ValueError("reference smoke ID differs")
        if reference.get("model", {}).get("sha256") != waiver.model_sha256:
            raise ValueError("reference model digest differs")
        reference_rate = float(reference["behavior_pass_rate"])
        difference = abs(behavior_rate - reference_rate)
        comparison.update(
            {
                "reference_behavior_pass_rate": reference_rate,
                "absolute_difference": difference,
                "passed": difference <= acceptance["maximum_reference_difference"],
                "reference": _binding(reference_summary),
            }
        )
    environment_path = output_dir / "environment.json"
    environment = json.loads(environment_path.read_text(encoding="utf-8"))
    clean_reproduction_passed = (
        environment_role == "clean"
        and environment.get("declared_role") == "clean"
        and smoke_passed
        and comparison["passed"] is True
    )
    return {
        "schema_version": 1,
        "kind": "p5_release_smoke",
        "smoke_id": root["smoke_id"],
        "scope": root["scope"],
        "environment_role": environment_role,
        "trial_count": len(records),
        "positive_trials": len(positives),
        "positive_successes": positive_successes,
        "positive_success_rate": positive_rate,
        "negative_trials": len(negatives),
        "negative_no_picks": negative_no_picks,
        "negative_no_pick_rate": negative_rate,
        "negative_false_picks": false_picks,
        "negative_control_attempts": negative_control_attempts,
        "behavior_passes": behavior_passes,
        "behavior_pass_rate": behavior_rate,
        "planning_time_p95_sec": planning_p95,
        "infrastructure_passed": all(item["infrastructure_valid"] for item in records),
        "shutdown_passed": all(item["shutdown_clean"] for item in records),
        "smoke_passed": smoke_passed,
        "reference_comparison": comparison,
        "clean_reproduction_passed": clean_reproduction_passed,
        "model": {
            "path": verified["model"].resolve().relative_to(REPOSITORY_ROOT.resolve()).as_posix(),
            "size_bytes": verified["model"].stat().st_size,
            "sha256": waiver.model_sha256,
            "confidence_threshold": waiver.confidence_threshold,
        },
        "environment": _binding(environment_path),
        "config": _binding(config_path),
        "source_contract": _binding(contract["source_path"]),
        "safety": {
            "formal_acceptance": False,
            "formal_p3_rerun": False,
            "p4_intervention_used": False,
            "held_out_real_test_consumed": False,
            "physical_robot_evidence": False,
            "sim_to_real_claim": False,
        },
        "interpretation": root["interpretation"],
        "trials": records,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--environment-role", choices=("reference", "clean"), required=True)
    parser.add_argument("--base-domain-id", type=int, required=True)
    parser.add_argument("--reference-summary", type=Path)
    arguments = parser.parse_args()
    result = summarize(
        arguments.output_dir,
        arguments.config,
        arguments.environment_role,
        arguments.base_domain_id,
        arguments.reference_summary,
    )
    summary_path = arguments.output_dir / "summary.json"
    summary_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    if arguments.environment_role == "reference":
        return 0 if result["smoke_passed"] else 1
    return 0 if result["clean_reproduction_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
