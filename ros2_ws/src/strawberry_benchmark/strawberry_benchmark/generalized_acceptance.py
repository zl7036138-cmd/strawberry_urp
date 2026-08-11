"""Validation and metric gate for the 30-seed generalized harvest matrix."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable, Mapping, Sequence


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
    results: Iterable[Mapping[str, object]],
) -> dict[str, object]:
    scenarios = validate_matrix(matrix)
    expected = {str(row["scenario_id"]): row for row in scenarios}
    rows = tuple(results)
    actual = {str(row.get("scenario_id", "")): row for row in rows}
    if set(actual) != set(expected) or len(actual) != len(rows):
        raise ValueError("result IDs must match the 30 formal scenarios exactly once")
    ordered = [actual[str(row["scenario_id"])] for row in scenarios]
    localization_errors = [
        float(value)
        for row in ordered
        for value in row.get("localization_errors_m", [])
    ]
    sigmas = [float(value) for row in ordered for value in row.get("accepted_sigmas_m", [])]
    visible_truth = sum(int(row.get("visible_ripe_truth_count", 0)) for row in ordered)
    true_positive = sum(int(row.get("ripe_true_positive_count", 0)) for row in ordered)
    predicted = sum(int(row.get("ripe_prediction_count", 0)) for row in ordered)
    switches = sum(int(row.get("identity_switch_count", 0)) for row in ordered)
    associations = sum(int(row.get("association_count", 0)) for row in ordered)
    reachable = sum(int(row.get("reachable_ripe_truth_count", 0)) for row in ordered)
    harvested = sum(int(row.get("harvested_reachable_ripe_count", 0)) for row in ordered)
    negative = [row for row in ordered if expected[str(row["scenario_id"])]["profile"] == "all_unripe"]
    positive = [row for row in ordered if expected[str(row["scenario_id"])]["profile"] == "mixed"]
    metrics = {
        "visible_ripe_recall": _ratio(true_positive, visible_truth),
        "ripe_precision": _ratio(true_positive, predicted),
        "identity_switch_rate": _ratio(switches, associations),
        "localization_p95_m": _quantile(localization_errors, 0.95),
        "sigma_pass_fraction": _ratio(sum(value <= float(matrix["acceptance"]["maximum_sigma_m"]) for value in sigmas), len(sigmas)),
        "unripe_pick_count": sum(int(row.get("unripe_pick_count", 0)) for row in ordered),
        "collision_count": sum(int(row.get("collision_count", 0)) for row in ordered),
        "negative_safe_no_pick_rate": _ratio(sum(row.get("outcome") == "NO_PICK" for row in negative), len(negative)),
        "reachable_target_success_rate": _ratio(harvested, reachable),
        "positive_scene_completion_rate": _ratio(sum(bool(row.get("scene_complete", False)) for row in positive), len(positive)),
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
        "negative_safe_no_pick_rate": metrics["negative_safe_no_pick_rate"] >= float(thresholds["minimum_negative_safe_no_pick_rate"]),
        "reachable_target_success_rate": metrics["reachable_target_success_rate"] >= float(thresholds["minimum_reachable_target_success_rate"]),
        "positive_scene_completion_rate": metrics["positive_scene_completion_rate"] >= float(thresholds["minimum_positive_scene_completion_rate"]),
    }
    return {
        "schema_version": 1,
        "matrix_id": matrix["matrix_id"],
        "scenario_count": len(ordered),
        "metrics": metrics,
        "gates": gates,
        "overall_pass": all(gates.values()),
    }


def main(args: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args(args)
    if options.output.exists():
        raise FileExistsError(f"refusing to overwrite {options.output}")
    matrix = json.loads(options.matrix.read_text(encoding="utf-8"))
    payload = json.loads(options.results.read_text(encoding="utf-8"))
    summary = summarize_results(matrix, payload["results"])
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, sort_keys=True))
    return 0 if summary["overall_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
