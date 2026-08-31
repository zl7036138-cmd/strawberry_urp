"""Strict provenance and metric gate for the 30-seed generalized matrix."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Mapping, Sequence


SHA256 = re.compile(r"[0-9a-f]{64}")
GIT_COMMIT = re.compile(r"[0-9a-f]{40,64}")
OUTCOMES = {"SUCCESS", "PARTIAL_SUCCESS", "NO_PICK", "FAILED"}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_matrix(matrix: Mapping[str, object]) -> tuple[Mapping[str, object], ...]:
    if int(matrix.get("schema_version", 0)) != 1:
        raise ValueError("unsupported generalized matrix schema")
    if matrix.get("split") != "formal_held_out" or matrix.get("development_or_training_allowed") is not False:
        raise ValueError("formal seeds must be held out from development and training")
    if int(matrix.get("behavioral_attempts_per_scenario", 0)) != 1:
        raise ValueError("formal scenarios permit one behavioral attempt")
    scenarios = matrix.get("scenarios")
    if not isinstance(scenarios, list) or len(scenarios) != 30 or int(matrix.get("scenario_count", 0)) != 30:
        raise ValueError("generalized matrix must contain exactly 30 scenarios")
    identifiers = [str(row.get("scenario_id", "")) for row in scenarios]
    seeds = [int(row.get("seed", -1)) for row in scenarios]
    if any(not value for value in identifiers) or len(identifiers) != len(set(identifiers)):
        raise ValueError("scenario IDs must be non-empty and unique")
    if any(value < 0 for value in seeds) or len(seeds) != len(set(seeds)):
        raise ValueError("scenario seeds must be non-negative and unique")
    profiles = [row.get("profile") for row in scenarios]
    if profiles.count("mixed") != 18 or profiles.count("all_unripe") != 6 or profiles.count("unsafe") != 6:
        raise ValueError("matrix profile distribution must be 18 positive, 6 negative, 6 unsafe")
    positives = [row for row in scenarios if row.get("profile") == "mixed"]
    cells = {(row.get("position_band"), row.get("occlusion")) for row in positives}
    expected_cells = {(band, occlusion) for band in ("near", "middle", "far") for occlusion in ("none", "partial", "heavy")}
    if cells != expected_cells or any(
        sum(row.get("position_band") == band and row.get("occlusion") == occlusion for row in positives) != 2
        for band, occlusion in expected_cells
    ):
        raise ValueError("positive scenarios must balance position and occlusion cells")
    return tuple(scenarios)


def validate_materialization_manifest(
    matrix: Mapping[str, object], manifest: Mapping[str, object]
) -> dict[str, Mapping[str, object]]:
    scenarios = validate_matrix(matrix)
    if int(manifest.get("schema_version", 0)) != 1:
        raise ValueError("unsupported materialization manifest schema")
    if manifest.get("matrix_id") != matrix.get("matrix_id"):
        raise ValueError("materialization manifest matrix binding mismatch")
    if manifest.get("formal_held_out") is not True or manifest.get("truth_for_runtime_control") is not False:
        raise ValueError("materialization manifest violates formal truth boundary")
    rows = manifest.get("scenarios")
    if not isinstance(rows, list):
        raise ValueError("materialization manifest scenarios are missing")
    actual = {str(row.get("scenario_id", "")): row for row in rows}
    expected = {str(row["scenario_id"]): row for row in scenarios}
    if len(actual) != len(rows) or set(actual) != set(expected):
        raise ValueError("materialization scenarios must match the formal matrix exactly")
    for scenario_id, row in actual.items():
        if int(row.get("seed", -1)) != int(expected[scenario_id]["seed"]):
            raise ValueError(f"materialization seed mismatch for {scenario_id}")
        for field in ("scene_sha256", "world_sha256"):
            if not SHA256.fullmatch(str(row.get(field, ""))):
                raise ValueError(f"invalid {field} for {scenario_id}")
    return actual


def _sha256(value, name: str) -> str:
    normalized = str(value).lower()
    if not SHA256.fullmatch(normalized):
        raise ValueError(f"{name} must be a lowercase SHA-256")
    return normalized


def _nonnegative_int(row: Mapping[str, object], name: str) -> int:
    value = row.get(name)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _finite_nonnegative_list(row: Mapping[str, object], name: str) -> list[float]:
    raw = row.get(name)
    if not isinstance(raw, list):
        raise ValueError(f"{name} must be a list")
    values = [float(value) for value in raw]
    if any(not math.isfinite(value) or value < 0.0 for value in values):
        raise ValueError(f"{name} values must be finite and non-negative")
    return values


def validate_results_payload(
    matrix: Mapping[str, object],
    manifest: Mapping[str, object],
    payload: Mapping[str, object],
    *,
    matrix_sha256: str,
    manifest_sha256: str,
) -> tuple[dict[str, object], ...]:
    scenarios = validate_matrix(matrix)
    materialized = validate_materialization_manifest(matrix, manifest)
    if int(payload.get("schema_version", 0)) != 2:
        raise ValueError("formal generalized results must use schema version 2")
    if payload.get("matrix_id") != matrix.get("matrix_id"):
        raise ValueError("formal result matrix ID mismatch")
    bindings = payload.get("bindings")
    if not isinstance(bindings, Mapping):
        raise ValueError("formal result bindings are missing")
    if _sha256(bindings.get("matrix_sha256"), "bindings.matrix_sha256") != matrix_sha256:
        raise ValueError("formal result matrix hash mismatch")
    if _sha256(bindings.get("materialization_manifest_sha256"), "bindings.materialization_manifest_sha256") != manifest_sha256:
        raise ValueError("formal result materialization hash mismatch")
    for name in ("model_sha256", "runtime_config_sha256"):
        _sha256(bindings.get(name), f"bindings.{name}")
    if not GIT_COMMIT.fullmatch(str(bindings.get("git_commit", "")).lower()):
        raise ValueError("bindings.git_commit must be a full Git object ID")
    rows = payload.get("results")
    if not isinstance(rows, list):
        raise ValueError("formal results list is missing")
    actual = {str(row.get("scenario_id", "")): row for row in rows}
    expected = {str(row["scenario_id"]): row for row in scenarios}
    if len(actual) != len(rows) or set(actual) != set(expected):
        raise ValueError("result IDs must match the 30 formal scenarios exactly once")
    normalized = []
    for scenario in scenarios:
        scenario_id = str(scenario["scenario_id"])
        row = actual[scenario_id]
        if int(row.get("schema_version", 0)) != 2:
            raise ValueError(f"{scenario_id} result schema must be version 2")
        if int(row.get("seed", -1)) != int(scenario["seed"]) or row.get("profile") != scenario.get("profile"):
            raise ValueError(f"{scenario_id} seed or profile binding mismatch")
        if _sha256(row.get("scene_sha256"), f"{scenario_id}.scene_sha256") != materialized[scenario_id]["scene_sha256"]:
            raise ValueError(f"{scenario_id} scene hash mismatch")
        if _sha256(row.get("world_sha256"), f"{scenario_id}.world_sha256") != materialized[scenario_id]["world_sha256"]:
            raise ValueError(f"{scenario_id} world hash mismatch")
        attempts = _nonnegative_int(row, "behavior_attempt_count")
        if attempts > 1:
            raise ValueError(f"{scenario_id} consumed more than one behavior attempt")
        if not isinstance(row.get("terminal_status_received"), bool):
            raise ValueError("terminal_status_received must be a boolean")
        if row.get("cleanup_outcome") not in {"CLEAN", "FAILED"}:
            raise ValueError("cleanup_outcome must be CLEAN or FAILED")
        if row.get("outcome") not in OUTCOMES:
            raise ValueError(f"unsupported outcome for {scenario_id}")
        integer_names = (
            "visible_ripe_truth_count",
            "ripe_true_positive_count",
            "ripe_prediction_count",
            "identity_switch_count",
            "association_count",
            "unripe_pick_count",
            "collision_count",
            "joint_limit_violation_count",
            "unsafe_motion_attempt_count",
            "reachable_ripe_truth_count",
            "harvested_reachable_ripe_count",
            "accepted_target_count",
            "successful_accepted_target_count",
        )
        values = {name: _nonnegative_int(row, name) for name in integer_names}
        if values["ripe_true_positive_count"] > min(values["visible_ripe_truth_count"], values["ripe_prediction_count"]):
            raise ValueError(f"{scenario_id} true positives exceed truth or predictions")
        if values["identity_switch_count"] > values["association_count"]:
            raise ValueError(f"{scenario_id} identity switches exceed associations")
        if values["harvested_reachable_ripe_count"] > values["reachable_ripe_truth_count"]:
            raise ValueError(f"{scenario_id} harvested count exceeds reachable truth")
        if values["successful_accepted_target_count"] > values["accepted_target_count"]:
            raise ValueError(f"{scenario_id} successful accepted targets exceed accepted targets")
        localization_errors = _finite_nonnegative_list(row, "localization_errors_m")
        sigmas = _finite_nonnegative_list(row, "accepted_sigmas_m")
        if not isinstance(row.get("scene_complete"), bool):
            raise ValueError("scene_complete must be a boolean")
        normalized.append(
            {
                **dict(row),
                **values,
                "localization_errors_m": localization_errors,
                "accepted_sigmas_m": sigmas,
                "evidence_complete": (
                    attempts == 1
                    and row["terminal_status_received"] is True
                    and row["cleanup_outcome"] == "CLEAN"
                ),
            }
        )
    return tuple(normalized)


def _ratio(numerator: float, denominator: float) -> float:
    return numerator / denominator if denominator > 0 else 0.0


def _quantile(values: Sequence[float], probability: float) -> float:
    if not values:
        return math.inf
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (position - lower) * (ordered[upper] - ordered[lower])


def summarize_results(
    matrix: Mapping[str, object],
    manifest: Mapping[str, object],
    payload: Mapping[str, object],
    *,
    matrix_sha256: str,
    manifest_sha256: str,
) -> dict[str, object]:
    scenarios = validate_matrix(matrix)
    ordered = validate_results_payload(
        matrix,
        manifest,
        payload,
        matrix_sha256=matrix_sha256,
        manifest_sha256=manifest_sha256,
    )
    expected = {str(row["scenario_id"]): row for row in scenarios}
    localization_errors = [value for row in ordered for value in row["localization_errors_m"]]
    sigmas = [value for row in ordered for value in row["accepted_sigmas_m"]]
    visible_truth = sum(row["visible_ripe_truth_count"] for row in ordered)
    true_positive = sum(row["ripe_true_positive_count"] for row in ordered)
    predicted = sum(row["ripe_prediction_count"] for row in ordered)
    switches = sum(row["identity_switch_count"] for row in ordered)
    associations = sum(row["association_count"] for row in ordered)
    reachable = sum(row["reachable_ripe_truth_count"] for row in ordered)
    harvested = sum(row["harvested_reachable_ripe_count"] for row in ordered)
    negative = [row for row in ordered if expected[str(row["scenario_id"])]["profile"] == "all_unripe"]
    positive = [row for row in ordered if expected[str(row["scenario_id"])]["profile"] == "mixed"]
    metrics = {
        "visible_ripe_recall": _ratio(true_positive, visible_truth),
        "ripe_precision": _ratio(true_positive, predicted),
        "identity_switch_rate": _ratio(switches, associations),
        "localization_p95_m": _quantile(localization_errors, 0.95),
        "sigma_pass_fraction": _ratio(sum(value <= float(matrix["acceptance"]["maximum_sigma_m"]) for value in sigmas), len(sigmas)),
        "unripe_pick_count": sum(row["unripe_pick_count"] for row in ordered),
        "collision_count": sum(row["collision_count"] for row in ordered),
        "joint_limit_violation_count": sum(row["joint_limit_violation_count"] for row in ordered),
        "unsafe_motion_attempt_count": sum(row["unsafe_motion_attempt_count"] for row in ordered),
        "negative_safe_no_pick_rate": _ratio(sum(row["outcome"] == "NO_PICK" for row in negative), len(negative)),
        "reachable_target_success_rate": _ratio(harvested, reachable),
        "positive_scene_completion_rate": _ratio(sum(bool(row["scene_complete"]) for row in positive), len(positive)),
        "complete_evidence_rate": _ratio(sum(bool(row["evidence_complete"]) for row in ordered), len(ordered)),
    }
    thresholds = matrix["acceptance"]
    gates = {
        "visible_ripe_recall": metrics["visible_ripe_recall"] >= float(thresholds["minimum_visible_ripe_recall"]),
        "ripe_precision": metrics["ripe_precision"] >= float(thresholds["minimum_ripe_precision"]),
        "identity_switch_rate": metrics["identity_switch_rate"] <= float(thresholds["maximum_identity_switch_rate"]),
        "localization_p95_m": metrics["localization_p95_m"] <= float(thresholds["maximum_localization_p95_m"]),
        "sigma_pass_fraction": metrics["sigma_pass_fraction"] >= float(thresholds["minimum_sigma_pass_fraction"]),
        "unripe_pick_count": metrics["unripe_pick_count"] <= int(thresholds["maximum_unripe_picks"]),
        "collision_count": metrics["collision_count"] <= int(thresholds["maximum_collisions"]),
        "joint_limit_violation_count": metrics["joint_limit_violation_count"] == 0,
        "unsafe_motion_attempt_count": metrics["unsafe_motion_attempt_count"] == 0,
        "negative_safe_no_pick_rate": metrics["negative_safe_no_pick_rate"] >= float(thresholds["minimum_negative_safe_no_pick_rate"]),
        "reachable_target_success_rate": metrics["reachable_target_success_rate"] >= float(thresholds["minimum_reachable_target_success_rate"]),
        "positive_scene_completion_rate": metrics["positive_scene_completion_rate"] >= float(thresholds["minimum_positive_scene_completion_rate"]),
        "evidence_integrity": metrics["complete_evidence_rate"] == 1.0,
    }
    return {
        "schema_version": 2,
        "matrix_id": matrix["matrix_id"],
        "bindings": dict(payload["bindings"]),
        "scenario_count": len(ordered),
        "metrics": metrics,
        "gates": gates,
        "overall_pass": all(gates.values()),
    }


def main(args: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args(args)
    if options.output.exists():
        raise FileExistsError(f"refusing to overwrite {options.output}")
    matrix = json.loads(options.matrix.read_text(encoding="utf-8"))
    manifest = json.loads(options.manifest.read_text(encoding="utf-8"))
    payload = json.loads(options.results.read_text(encoding="utf-8"))
    summary = summarize_results(
        matrix,
        manifest,
        payload,
        matrix_sha256=sha256_file(options.matrix),
        manifest_sha256=sha256_file(options.manifest),
    )
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, sort_keys=True))
    return 0 if summary["overall_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
