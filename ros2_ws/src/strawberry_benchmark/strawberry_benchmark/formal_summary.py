"""Formal P3 simulator-matrix aggregation and fail-closed acceptance."""

from __future__ import annotations

from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable

from .formal_matrix import (
    FormalMatrixContract,
    generate_formal_schedule,
    load_formal_matrix_contract,
    verify_formal_matrix_contract,
)
from .perception_control import sha256_file


FAILURE_CODE_NAMES = {
    1: "NO_TARGET",
    2: "LOW_CONFIDENCE",
    3: "DEPTH_INVALID",
    4: "TF_TIMEOUT",
    5: "UNREACHABLE",
    6: "PLANNING_FAILED",
    7: "COLLISION",
    8: "GRASP_FAILED",
    9: "PLACE_FAILED",
    10: "STALE_DATA",
}
SHUTDOWN_ERROR_MARKERS = (
    "exception was never retrieved",
    "failed to terminate",
    "traceback (most recent call last)",
    "process has died",
)


def _percentile(values: Iterable[float], quantile: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def _rate(numerator: int, denominator: int) -> float:
    return numerator / denominator if denominator else 0.0


def evaluate_formal_records(
    contract: FormalMatrixContract, records: list[dict[str, Any]]
) -> dict[str, Any]:
    """Apply the frozen behavioral and integrity gates to normalized records."""

    positives = [item for item in records if item.get("kind") == "POSITIVE"]
    negatives = [item for item in records if item.get("kind") == "NEGATIVE"]
    complete = len(records) == 165 and len(positives) == 135 and len(negatives) == 30
    unique_ids = len({item.get("trial_id") for item in records}) == len(records)
    all_infrastructure = complete and all(
        bool(item.get("infrastructure_valid")) for item in records
    )
    all_shutdowns = complete and all(bool(item.get("shutdown_clean")) for item in records)
    positive_successes = sum(bool(item.get("positive_success")) for item in positives)
    negative_no_picks = sum(bool(item.get("negative_safe_no_pick")) for item in negatives)
    negative_pick_attempts = sum(bool(item.get("pick_attempted")) for item in negatives)
    negative_fruit_picked = sum(bool(item.get("fruit_picked")) for item in negatives)
    positive_success_rate = _rate(positive_successes, len(positives))
    negative_no_pick_rate = _rate(negative_no_picks, len(negatives))
    negative_pick_attempt_rate = _rate(negative_pick_attempts, len(negatives))
    negative_fruit_picked_rate = _rate(negative_fruit_picked, len(negatives))
    planning_values = [
        float(item["planning_time_sec"])
        for item in positives
        if item.get("planning_time_sec") is not None
    ]
    planning_p95 = _percentile(planning_values, 0.95)
    failed_behavior = [
        item
        for item in records
        if item.get("behavior_started")
        and not item.get("positive_success")
        and not item.get("negative_safe_no_pick")
    ]
    unattributed = [
        item.get("trial_id")
        for item in failed_behavior
        if not item.get("failure_category")
    ]
    failures_by_category = Counter(
        str(item["failure_category"])
        for item in failed_behavior
        if item.get("failure_category")
    )

    subgroup: dict[str, dict[str, dict[str, int | float]]] = {}
    for field in ("lighting", "occlusion", "position", "simulation_seed"):
        grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for item in positives:
            grouped[str(item.get(field))].append(item)
        subgroup[field] = {
            key: {
                "trials": len(items),
                "successes": sum(bool(item.get("positive_success")) for item in items),
                "success_rate": _rate(
                    sum(bool(item.get("positive_success")) for item in items),
                    len(items),
                ),
            }
            for key, items in sorted(grouped.items())
        }

    gate_passed = (
        complete
        and unique_ids
        and all_infrastructure
        and all_shutdowns
        and not unattributed
        and positive_success_rate >= contract.minimum_positive_success_rate
        and negative_no_pick_rate >= contract.minimum_negative_no_pick_rate
        and negative_pick_attempt_rate <= contract.maximum_negative_pick_attempt_rate
        and negative_fruit_picked_rate <= contract.maximum_negative_fruit_picked_rate
        and planning_p95 is not None
        and planning_p95 <= contract.maximum_planning_p95_sec
    )
    return {
        "scenario_count": len(records),
        "positive_trials": len(positives),
        "positive_successes": positive_successes,
        "positive_success_rate": positive_success_rate,
        "minimum_positive_success_rate": contract.minimum_positive_success_rate,
        "negative_trials": len(negatives),
        "negative_safe_no_picks": negative_no_picks,
        "negative_safe_no_pick_rate": negative_no_pick_rate,
        "minimum_negative_safe_no_pick_rate": contract.minimum_negative_no_pick_rate,
        "negative_pick_attempts": negative_pick_attempts,
        "negative_pick_attempt_rate": negative_pick_attempt_rate,
        "maximum_negative_pick_attempt_rate": contract.maximum_negative_pick_attempt_rate,
        "negative_fruit_picked": negative_fruit_picked,
        "negative_fruit_picked_rate": negative_fruit_picked_rate,
        "maximum_negative_fruit_picked_rate": contract.maximum_negative_fruit_picked_rate,
        "planning_observation_count": len(planning_values),
        "planning_time_p95_sec": planning_p95,
        "maximum_planning_time_p95_sec": contract.maximum_planning_p95_sec,
        "all_scenarios_complete": complete,
        "unique_trial_ids": unique_ids,
        "infrastructure_passed": all_infrastructure,
        "shutdown_passed": all_shutdowns,
        "unattributed_failure_count": len(unattributed),
        "unattributed_trial_ids": unattributed,
        "failure_by_category": dict(sorted(failures_by_category.items())),
        "positive_subgroups": subgroup,
        "formal_p3_simulator_gate_passed": gate_passed,
    }


def _load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot load {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return payload


def _fingerprint(path: Path, root: Path) -> dict[str, Any]:
    return {
        "path": path.resolve().relative_to(root.resolve()).as_posix(),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def _failure_attribution(payload: dict[str, Any], kind: str, contract: FormalMatrixContract) -> tuple[str | None, str | None]:
    final = payload.get("final_status") or {}
    outcome = final.get("outcome")
    if kind == "POSITIVE" and outcome == "NO_PICK":
        return "NO_TARGET", contract.failure_taxonomy["NO_TARGET"]
    if kind == "NEGATIVE" and payload.get("pick_attempted"):
        return "FALSE_PICK_ATTEMPT", "perception"
    raw_code = payload.get("failure_code", final.get("failure_code"))
    try:
        code_name = FAILURE_CODE_NAMES.get(int(raw_code)) if raw_code is not None else None
    except (TypeError, ValueError):
        code_name = str(raw_code) if raw_code else None
    return code_name, contract.failure_taxonomy.get(code_name) if code_name else None


def summarize_formal_matrix(
    output_dir: Path,
    manifest_path: Path,
    preflight_receipt_path: Path,
    repository_root: Path,
    *,
    model_path: Path | None = None,
    provenance_paths: tuple[str, ...] = (),
) -> dict[str, Any]:
    """Verify the preflight and normalize every preserved formal attempt."""

    contract = load_formal_matrix_contract(manifest_path)
    verified = verify_formal_matrix_contract(contract, repository_root, model_path=model_path)
    expected_preflight_path = (verified["preflight_dir"] / "preflight_receipt.json").resolve()
    if preflight_receipt_path.resolve() != expected_preflight_path:
        raise ValueError("formal summary preflight path differs from the current contract")
    preflight = _load_json(preflight_receipt_path)
    if preflight.get("matrix_id") != contract.matrix_id or preflight.get("formal_run_unlocked") is not True:
        raise ValueError("formal preflight is not unlocked for this matrix")
    if preflight.get("held_out_test_receipt_exists") is not False:
        raise ValueError("formal preflight did not preserve the held-out-test seal")
    schedule_path = repository_root / str(preflight["schedule"]["path"])
    if schedule_path.stat().st_size != int(preflight["schedule"]["size_bytes"]):
        raise ValueError("formal schedule size differs from its preflight")
    if sha256_file(schedule_path) != str(preflight["schedule"]["sha256"]):
        raise ValueError("formal schedule digest differs from its preflight")
    expected_schedule = generate_formal_schedule(contract, verified["benchmark"])
    scheduled_payloads = [json.loads(line) for line in schedule_path.read_text(encoding="utf-8").splitlines() if line]
    if scheduled_payloads != [item.to_dict() for item in expected_schedule]:
        raise ValueError("formal schedule content differs from the generated contract")

    normalized: list[dict[str, Any]] = []
    attempt_audit = []
    for scenario in expected_schedule:
        trial_dir = output_dir / "trials" / f"{scenario.order_index:03d}"
        attempts = []
        for attempt_index in range(1, contract.maximum_infrastructure_attempts + 1):
            attempt_dir = trial_dir / f"attempt_{attempt_index:02d}"
            if not attempt_dir.exists():
                continue
            result_path = attempt_dir / "result.json"
            payload = _load_json(result_path) if result_path.exists() else {"error": "result missing"}
            behavior_started = bool(payload.get("behavior_started"))
            attempts.append((attempt_index, attempt_dir, payload, behavior_started))
        behavior_attempts = [item for item in attempts if item[3]]
        selected = behavior_attempts[0] if len(behavior_attempts) == 1 else None
        retry_valid = (
            len(attempts) <= contract.maximum_infrastructure_attempts
            and len(behavior_attempts) <= 1
            and all(not item[3] for item in attempts if selected and item[0] < selected[0])
            and not any(item[0] > selected[0] for item in attempts) if selected else len(attempts) <= contract.maximum_infrastructure_attempts
        )
        if selected:
            attempt_index, attempt_dir, payload, _ = selected
            launch_path = attempt_dir / "launch.log"
            probe_path = attempt_dir / "condition_probe.json"
            receipt_path = attempt_dir / "condition_receipt.json"
            launch_text = launch_path.read_text(encoding="utf-8", errors="replace").lower() if launch_path.exists() else ""
            shutdown_clean = launch_path.exists() and not any(marker in launch_text for marker in SHUTDOWN_ERROR_MARKERS)
            final = payload.get("final_status") or {}
            scenario_matches = (
                payload.get("formal_kind") == scenario.kind
                and payload.get("formal_lighting") == scenario.lighting
                and payload.get("formal_occlusion") == scenario.occlusion
                and payload.get("formal_position") == scenario.position.label
                and payload.get("formal_simulation_seed") == scenario.simulation_seed
                and payload.get("formal_order_index") == scenario.order_index
                and payload.get("scenario_id") == scenario.trial_id
            )
            receipt_valid = receipt_path.exists() and payload.get("condition_receipt", {}).get("sha256") == sha256_file(receipt_path)
            probe_valid = probe_path.exists() and payload.get("condition_probe", {}).get("sha256") == sha256_file(probe_path)
            scene_configured = bool(
                payload.get("position_configuration_applied")
                and payload.get("parked_model_configuration_applied")
            )
            infrastructure_valid = bool(
                retry_valid and scenario_matches and receipt_valid and probe_valid
                and payload.get("source_isolated") and scene_configured
                and "error" not in payload
            )
            positive_success = scenario.kind == "POSITIVE" and bool(payload.get("manipulation_success")) and final.get("outcome") == "SUCCESS"
            negative_safe = scenario.kind == "NEGATIVE" and bool(payload.get("safe_no_pick")) and final.get("outcome") == "NO_PICK"
            failure_code, failure_category = _failure_attribution(payload, scenario.kind, contract)
            normalized.append(
                {
                    **scenario.to_dict(),
                    "attempt_index": attempt_index,
                    "behavior_started": True,
                    "scene_configured": scene_configured,
                    "infrastructure_valid": infrastructure_valid,
                    "shutdown_clean": shutdown_clean,
                    "positive_success": positive_success,
                    "negative_safe_no_pick": negative_safe,
                    "pick_attempted": bool(payload.get("pick_attempted")),
                    "fruit_picked": bool(payload.get("fruit_picked")),
                    "planning_time_sec": payload.get("planning_time_sec"),
                    "execution_time_sec": payload.get("execution_time_sec"),
                    "localization_error_mm": payload.get("localization_error_mm"),
                    "terminal_state": final.get("state"),
                    "outcome": final.get("outcome"),
                    "failure_code": failure_code,
                    "failure_category": failure_category,
                    "result": _fingerprint(attempt_dir / "result.json", repository_root),
                    "launch_log": _fingerprint(launch_path, repository_root) if launch_path.exists() else None,
                }
            )
        else:
            normalized.append(
                {
                    **scenario.to_dict(),
                    "attempt_index": None,
                    "behavior_started": False,
                    "infrastructure_valid": False,
                    "shutdown_clean": False,
                    "positive_success": False,
                    "negative_safe_no_pick": False,
                    "pick_attempted": False,
                    "fruit_picked": False,
                    "planning_time_sec": None,
                    "execution_time_sec": None,
                    "localization_error_mm": None,
                    "terminal_state": None,
                    "outcome": None,
                    "failure_code": None,
                    "failure_category": None,
                    "result": None,
                    "launch_log": None,
                }
            )
        attempt_audit.append(
            {
                "trial_id": scenario.trial_id,
                "attempt_count": len(attempts),
                "behavior_attempt_count": len(behavior_attempts),
                "retry_policy_valid": retry_valid,
            }
        )

    metrics = evaluate_formal_records(contract, normalized)
    claim_exists = verified["claim"].exists()
    formal_passed = bool(metrics["formal_p3_simulator_gate_passed"] and claim_exists)
    return {
        "schema_version": 1,
        "kind": "p3_formal_simulator_matrix_summary",
        "matrix_id": contract.matrix_id,
        "formal_matrix_claim_exists": claim_exists,
        "formal_matrix_consumed": claim_exists,
        "formal_p3_simulator_gate_passed": formal_passed,
        "engineering_waiver_active": True,
        "numeric_t30_gate_passed": False,
        "held_out_real_test_consumed": False,
        "held_out_test_receipt_exists": verified["held_out_receipt"].exists(),
        "model": _fingerprint(verified["model"], repository_root),
        "confidence_threshold": contract.confidence_threshold,
        "metrics": metrics,
        "manifest": _fingerprint(manifest_path, repository_root),
        "preflight_receipt": _fingerprint(preflight_receipt_path, repository_root),
        "schedule": _fingerprint(schedule_path, repository_root),
        "provenance": [
            _fingerprint(repository_root / relative_path, repository_root)
            for relative_path in provenance_paths
        ],
        "attempt_audit": attempt_audit,
        "trials": normalized,
        "interpretation": (
            "Formal P3 simulator evidence under the ADR-0026 metric waiver. "
            "Not an independent real-image, physical-robot, or sim-to-real result."
        ),
    }
