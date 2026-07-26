"""Summarize the repeated perception-controlled simple-scene development gate."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from .perception_control import sha256_file
from .perception_dev_gate import (
    load_perception_repeated_dev_gate,
    verify_perception_repeated_dev_gate,
)


SHUTDOWN_ERROR_MARKERS = (
    "exception was never retrieved",
    "failed to terminate",
    "traceback (most recent call last)",
    "process has died",
)


def _optional_sha256(path: Path) -> str | None:
    return sha256_file(path) if path.exists() else None


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


def _fingerprint(path: Path, repository_root: Path) -> dict:
    content = path.read_bytes()
    return {
        "path": path.resolve().relative_to(repository_root.resolve()).as_posix(),
        "size_bytes": len(content),
        "sha256": hashlib.sha256(content).hexdigest(),
    }


def _first_failure_stage(final_status: dict) -> str | None:
    for transition in reversed(final_status.get("state_history", [])):
        if transition.get("current") == "FAILED":
            return transition.get("previous")
    return None


def summarize_repeated_dev_gate(
    output_dir: Path,
    manifest_path: Path,
    repository_root: Path,
    *,
    model_path: Path | None = None,
    base_domain_id: int | None = None,
    provenance_paths: tuple[str, ...] = (),
) -> dict:
    """Load all fresh-world trial records and apply the frozen development gate."""

    contract = load_perception_repeated_dev_gate(manifest_path)
    waiver, verified = verify_perception_repeated_dev_gate(
        contract, repository_root, model_path=model_path
    )
    domain_start = (
        contract.base_ros_domain_id
        if base_domain_id is None
        else base_domain_id
    )
    total_trials = contract.positive_trials + contract.negative_trials
    if not isinstance(domain_start, int) or not 0 <= domain_start <= 232:
        raise ValueError("base ROS domain ID must be within [0, 232]")
    if domain_start + total_trials - 1 > 232:
        raise ValueError("repeated gate ROS domain range exceeds 232")

    records = []
    for mode, count in (
        ("positive", contract.positive_trials),
        ("negative", contract.negative_trials),
    ):
        for index in range(1, count + 1):
            label = f"{mode}_{index:02d}"
            global_index = (
                index - 1
                if mode == "positive"
                else contract.positive_trials + index - 1
            )
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
            final_status = payload.get("final_status", {})
            routing_valid = (
                final_status.get("target_source") == "perception"
                and final_status.get("control_target_topic")
                == waiver.control_target_topic
                and final_status.get("shadow_target_topic")
                != waiver.control_target_topic
            )
            scene_configured = bool(
                payload.get("position_configuration_applied", False)
            ) and bool(payload.get("parked_model_configuration_applied", False))
            shutdown_clean = launch_path.exists() and not any(
                marker in launch_text for marker in SHUTDOWN_ERROR_MARKERS
            )
            settled_frames = int(payload.get("settled_window_detection_frames", 0))
            negative_liveness_valid = (
                mode != "negative"
                or settled_frames >= contract.minimum_settled_detection_frames
            )
            infrastructure_valid = (
                result_path.exists()
                and "error" not in payload
                and scene_configured
                and routing_valid
                and negative_liveness_valid
            )
            positive_success = (
                mode == "positive"
                and bool(payload.get("manipulation_success", False))
                and bool(payload.get("state_machine_complete", False))
                and final_status.get("outcome") == "SUCCESS"
            )
            negative_no_pick = (
                mode == "negative"
                and bool(payload.get("safe_no_pick", False))
                and final_status.get("outcome") == "NO_PICK"
            )
            false_pick = mode == "negative" and bool(
                payload.get("fruit_picked", False)
            )
            negative_control_attempt = mode == "negative" and bool(
                payload.get("unsafe_control_attempt", False)
            )
            records.append(
                {
                    "trial_id": label,
                    "mode": mode,
                    "ros_domain_id": domain_start + global_index,
                    "infrastructure_valid": infrastructure_valid,
                    "routing_valid": routing_valid,
                    "scene_configured": scene_configured,
                    "negative_liveness_valid": negative_liveness_valid,
                    "settled_window_detection_frames": settled_frames,
                    "settled_window_detection_count": int(
                        payload.get("settled_window_detection_count", 0)
                    ),
                    "settled_window_ripe_detection_count": int(
                        payload.get("settled_window_ripe_detection_count", 0)
                    ),
                    "settled_window_total_control_samples": int(
                        payload.get("settled_window_total_control_samples", 0)
                    ),
                    "shutdown_clean": shutdown_clean,
                    "positive_success": positive_success,
                    "negative_no_pick": negative_no_pick,
                    "false_pick": false_pick,
                    "negative_control_attempt": negative_control_attempt,
                    "terminal_state": final_status.get("state"),
                    "outcome": final_status.get("outcome"),
                    "control_target_id": final_status.get("control_target_id"),
                    "planning_time_sec": payload.get("planning_time_sec"),
                    "execution_time_sec": payload.get("execution_time_sec"),
                    "failure_code": payload.get("failure_code"),
                    "failure_message": payload.get(
                        "failure_message", payload.get("error", "")
                    ),
                    "first_failure_stage": _first_failure_stage(final_status),
                    "result_file": result_path.name,
                    "launch_log": launch_path.name,
                    "result_sha256": _optional_sha256(result_path),
                    "launch_log_sha256": _optional_sha256(launch_path),
                }
            )

    positives = [item for item in records if item["mode"] == "positive"]
    negatives = [item for item in records if item["mode"] == "negative"]
    positive_successes = sum(item["positive_success"] for item in positives)
    negative_no_picks = sum(item["negative_no_pick"] for item in negatives)
    false_picks = sum(item["false_pick"] for item in negatives)
    negative_control_attempts = sum(
        item["negative_control_attempt"] for item in negatives
    )
    positive_success_rate = positive_successes / len(positives)
    negative_no_pick_rate = negative_no_picks / len(negatives)
    negative_false_pick_rate = false_picks / len(negatives)
    planning_times = [
        float(item["planning_time_sec"])
        for item in positives
        if item["positive_success"] and item["planning_time_sec"] is not None
    ]
    planning_p95 = _percentile(planning_times, 0.95)
    infrastructure_passed = all(
        item["infrastructure_valid"] for item in records
    )
    shutdown_passed = all(item["shutdown_clean"] for item in records)
    gate_passed = (
        infrastructure_passed
        and shutdown_passed
        and positive_success_rate >= contract.minimum_positive_success_rate
        and negative_no_pick_rate >= contract.minimum_negative_no_pick_rate
        and negative_false_pick_rate
        <= contract.maximum_negative_false_pick_rate
        and negative_control_attempts
        <= contract.maximum_negative_control_attempts
        and planning_p95 is not None
        and planning_p95 <= contract.planning_time_p95_limit_sec
    )

    return {
        "schema_version": 1,
        "kind": "p3_perception_repeated_development_gate",
        "gate_id": contract.gate_id,
        "scope": "NON_FORMAL_SIMPLE_SCENE_DEVELOPMENT_GATE",
        "formal_acceptance": False,
        "may_close_formal_p3_gate": False,
        "formal_135_plus_30_matrix_consumed": False,
        "held_out_real_test_consumed": False,
        "held_out_test_receipt_exists": verified["held_out_receipt"].exists(),
        "engineering_waiver_active": True,
        "numeric_t30_gate_passed": False,
        "control_source": "perception",
        "oracle_provider_started": False,
        "model": {
            "path": str(verified["model"]),
            "size_bytes": verified["model"].stat().st_size,
            "sha256": sha256_file(verified["model"]),
            "confidence_threshold": waiver.confidence_threshold,
            "image_size": waiver.image_size,
        },
        "trial_count": total_trials,
        "positive_trials": len(positives),
        "positive_successes": positive_successes,
        "positive_success_rate": positive_success_rate,
        "minimum_positive_success_rate": (
            contract.minimum_positive_success_rate
        ),
        "negative_trials": len(negatives),
        "negative_no_picks": negative_no_picks,
        "negative_no_pick_rate": negative_no_pick_rate,
        "minimum_negative_no_pick_rate": contract.minimum_negative_no_pick_rate,
        "negative_false_picks": false_picks,
        "negative_false_pick_rate": negative_false_pick_rate,
        "maximum_negative_false_pick_rate": (
            contract.maximum_negative_false_pick_rate
        ),
        "negative_control_attempts": negative_control_attempts,
        "maximum_negative_control_attempts": (
            contract.maximum_negative_control_attempts
        ),
        "planning_time_p95_sec": planning_p95,
        "planning_time_p95_limit_sec": contract.planning_time_p95_limit_sec,
        "infrastructure_passed": infrastructure_passed,
        "shutdown_passed": shutdown_passed,
        "development_gate_passed": gate_passed,
        "base_ros_domain_id": domain_start,
        "ros_domain_ids": [item["ros_domain_id"] for item in records],
        "interpretation": (
            "Fresh-world repeatability and only-unripe safety at one clear "
            "base pose under ADR 0026/0027. Not formal P3, robustness, real-image, "
            "or sim-to-real evidence."
        ),
        "manifest": _fingerprint(manifest_path, repository_root),
        "provenance": [
            _fingerprint(repository_root / relative_path, repository_root)
            for relative_path in provenance_paths
        ],
        "trials": records,
    }
