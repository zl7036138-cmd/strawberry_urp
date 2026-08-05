#!/usr/bin/env python3
"""Summarize the frozen rendered-occlusion localization matrix."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from validate_rendered_occlusion_localization_matrix import (  # noqa: E402
    load_manifest,
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fingerprint(path: Path) -> dict[str, Any]:
    path = path.resolve(strict=True)
    try:
        label = path.relative_to(ROOT).as_posix()
    except ValueError:
        label = path.as_posix()
    return {
        "path": label,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _nearest_rank(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def _aggregate(trials: list[dict[str, Any]], mode: str) -> dict[str, Any]:
    samples = [record[mode] for trial in trials for record in trial["records"]]
    estimated = [sample for sample in samples if sample["status"] == "ESTIMATED"]
    errors = [float(sample["error_mm"]) for sample in estimated]
    sigmas = [float(sample["position_sigma_m"]) for sample in estimated]
    return {
        "requested_samples": len(samples),
        "estimated_samples": len(estimated),
        "no_pose_samples": len(samples) - len(estimated),
        "acceptance_rate": len(estimated) / len(samples) if samples else 0.0,
        "median_error_mm": statistics.median(errors) if errors else None,
        "p95_error_mm": _nearest_rank(errors, 0.95),
        "maximum_error_mm": max(errors) if errors else None,
        "median_sigma_m": statistics.median(sigmas) if sigmas else None,
        "p95_sigma_m": _nearest_rank(sigmas, 0.95),
        "sigma_under_15mm_rate": (
            sum(value <= 0.015 for value in sigmas) / len(sigmas)
            if sigmas
            else 0.0
        ),
    }


def summarize(output_directory: Path, manifest_path: Path) -> dict[str, Any]:
    output_directory = output_directory.resolve(strict=True)
    manifest = load_manifest(manifest_path)
    thresholds = manifest["thresholds"]
    samples_per_scenario = int(manifest["runtime"]["samples_per_scenario"])
    scenarios = []
    infrastructure_errors = []
    by_occlusion: dict[str, list[dict[str, Any]]] = {
        "none": [],
        "partial": [],
        "heavy": [],
    }
    coverage_by_position: dict[str, dict[str, float]] = {}

    for position in manifest["positions"]:
        position_label = str(position["label"])
        coverage_by_position[position_label] = {}
        for condition in manifest["conditions"]:
            occlusion = str(condition["occlusion"])
            scenario_id = f"rendered__{position_label}__{occlusion}"
            directory = output_directory / scenario_id
            trial_path = directory / "paired_trial.json"
            probe_path = directory / "condition_probe.json"
            receipt_path = directory / "condition_receipt.json"
            try:
                trial = _load(trial_path)
                probe = _load(probe_path)
                receipt = _load(receipt_path)
            except (FileNotFoundError, ValueError, json.JSONDecodeError) as exc:
                infrastructure_errors.append(f"{scenario_id}: {exc}")
                continue
            if (
                trial.get("scenario_id") != scenario_id
                or trial.get("position_label") != position_label
                or trial.get("occlusion") != occlusion
                or trial.get("completed") is not True
                or int(trial.get("sample_count", -1)) != samples_per_scenario
            ):
                infrastructure_errors.append(f"{scenario_id}: trial contract mismatch")
            safety = trial.get("safety", {})
            if (
                safety.get("robot_motion_started") is not False
                or safety.get("manipulation_started") is not False
                or safety.get("orchestrator_started") is not False
                or safety.get("attachment_enabled") is not False
                or safety.get("perception_model_started") is not False
                or safety.get("control_commands_sent") != 0
            ):
                infrastructure_errors.append(f"{scenario_id}: safety violation")
            if (
                probe.get("occlusion_level") != occlusion
                or receipt.get("occlusion_level") != occlusion
            ):
                infrastructure_errors.append(f"{scenario_id}: condition mismatch")
            coverage = float(
                trial.get("rendered_occlusion", {}).get(
                    "mean_blue_coverage_ratio", -1.0
                )
            )
            coverage_by_position[position_label][occlusion] = coverage
            by_occlusion[occlusion].append(trial)
            scenarios.append(
                {
                    "scenario_id": scenario_id,
                    "position_label": position_label,
                    "position_m": position["position_m"],
                    "occlusion": occlusion,
                    "mean_blue_coverage_ratio": coverage,
                    "center_median": trial["summaries"]["center_median"],
                    "geometry_layer": trial["summaries"]["geometry_layer"],
                    "bindings": {
                        "condition_receipt": _fingerprint(receipt_path),
                        "condition_probe": _fingerprint(probe_path),
                        "paired_trial": _fingerprint(trial_path),
                    },
                }
            )

    aggregates = {
        occlusion: {
            mode: _aggregate(trials, mode)
            for mode in ("center_median", "geometry_layer")
        }
        for occlusion, trials in by_occlusion.items()
    }
    coverage_checks = {}
    for position_label, values in coverage_by_position.items():
        complete = set(values) == {"none", "partial", "heavy"}
        coverage_checks[position_label] = bool(
            complete
            and values["none"] <= thresholds["none_blue_coverage_max"]
            and values["partial"] >= thresholds["partial_blue_coverage_min"]
            and thresholds["heavy_blue_coverage_min"]
            <= values["heavy"]
            <= thresholds["heavy_blue_coverage_max"]
            and values["heavy"] - values["partial"]
            >= thresholds["heavy_minus_partial_coverage_min"]
        )

    acceptance_minima = {
        "none": thresholds["geometry_none_acceptance_min_each_position"],
        "partial": thresholds[
            "geometry_partial_acceptance_min_each_position"
        ],
        "heavy": thresholds["geometry_heavy_acceptance_min_each_position"],
    }
    acceptance_checks = {}
    for occlusion, minimum in acceptance_minima.items():
        relevant = [
            scenario
            for scenario in scenarios
            if scenario["occlusion"] == occlusion
        ]
        acceptance_checks[occlusion] = bool(
            len(relevant) == 5
            and all(
                float(scenario["geometry_layer"]["acceptance_rate"])
                >= float(minimum)
                for scenario in relevant
            )
        )
    error_checks = {
        occlusion: bool(
            aggregates[occlusion]["geometry_layer"]["p95_error_mm"]
            is not None
            and aggregates[occlusion]["geometry_layer"]["p95_error_mm"]
            <= thresholds["geometry_p95_error_mm_max_each_condition"]
        )
        for occlusion in ("none", "partial", "heavy")
    }
    diagnostic_checks = {
        "fifteen_complete_scenarios": (
            len(scenarios) == 15 and not infrastructure_errors
        ),
        "five_distinct_target_positions": len(coverage_by_position) == 5,
        "rendered_coverage_valid_each_position": all(coverage_checks.values()),
        "geometry_acceptance_valid_each_position": all(
            acceptance_checks.values()
        ),
        "geometry_p95_error_valid_each_condition": all(error_checks.values()),
        "zero_robot_or_control_motion": not infrastructure_errors,
    }
    diagnostic_passed = all(diagnostic_checks.values())
    sigma_eligible = all(
        aggregates[occlusion]["geometry_layer"]["sigma_under_15mm_rate"]
        == 1.0
        for occlusion in ("none", "partial", "heavy")
    )
    promotion_checks = {
        "diagnostic_passed": diagnostic_passed,
        "geometry_sigma_under_15mm_all_samples": sigma_eligible,
        "non_oracle_detector_boxes_tested": False,
        "natural_leaf_occlusion_tested": False,
        "field_v3_wrist_view_tested": False,
    }
    return {
        "schema_version": 1,
        "kind": "rendered_occlusion_localization_matrix_summary",
        "diagnostic_status": "PASS" if diagnostic_passed else "FAIL",
        "runtime_promotion_status": (
            "ELIGIBLE" if all(promotion_checks.values()) else "BLOCKED"
        ),
        "diagnostic_checks": diagnostic_checks,
        "promotion_checks": promotion_checks,
        "coverage_checks": coverage_checks,
        "acceptance_checks": acceptance_checks,
        "error_checks": error_checks,
        "infrastructure_errors": infrastructure_errors,
        "aggregates": aggregates,
        "scenarios": scenarios,
        "manifest": _fingerprint(manifest_path),
        "implementation": {
            "summarizer": _fingerprint(Path(__file__)),
            "localization_core": _fingerprint(
                ROOT
                / "ros2_ws"
                / "src"
                / "strawberry_localization"
                / "strawberry_localization"
                / "core.py"
            ),
        },
        "scope": {
            "simulator_only": True,
            "oracle_bounding_boxes": True,
            "rendered_visual_occlusion": True,
            "robot_motion": False,
            "formal_p3_or_p4_evidence": False,
            "runtime_promotion_authorized": False,
            "held_out_real_test_consumed": False,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    options = parser.parse_args()
    result = summarize(options.output_dir, options.manifest)
    output = options.output_dir.resolve() / "summary.json"
    if output.exists():
        raise SystemExit(f"refusing to overwrite {output}")
    with output.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(
            json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
            + "\n"
        )
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result["diagnostic_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
