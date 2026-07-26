#!/usr/bin/env python3
"""Correct background-aware condition checks for the preserved T70-D2 run."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from summarize_t70_multiposition_diagnostic import summarize  # noqa: E402
from strawberry_sim.scene_conditions import (  # noqa: E402
    OCCLUDER_MODEL_NAME,
    fingerprint,
)


def background_aware_position_check(
    baseline_coverage: float,
    dim_coverage: float,
    heavy_coverage: float,
    *,
    background_stability_maximum: float,
    heavy_foreground_delta_minimum: float,
    heavy_coverage_maximum: float,
    no_occluder_worlds_verified: bool,
    heavy_occluder_world_verified: bool,
) -> dict[str, bool | float]:
    background = max(float(baseline_coverage), float(dim_coverage))
    delta = float(heavy_coverage) - background
    return {
        "none_worlds_have_no_benchmark_occluder": no_occluder_worlds_verified,
        "heavy_world_has_visual_only_benchmark_occluder": (
            heavy_occluder_world_verified
        ),
        "none_background_is_stable_across_lighting": (
            abs(float(baseline_coverage) - float(dim_coverage))
            <= background_stability_maximum
        ),
        "heavy_additional_blue_coverage": delta,
        "heavy_adds_visible_foreground": delta >= heavy_foreground_delta_minimum,
        "heavy_is_not_total": float(heavy_coverage) <= heavy_coverage_maximum,
    }


def _world_occluder_contract(directory: Path, expected_enabled: bool) -> bool:
    receipt = json.loads((directory / "receipt.json").read_text(encoding="utf-8"))
    world_path = Path(receipt["materialized_world"]["path"])
    root = ET.parse(world_path).getroot()
    models = root.findall(f"./world/model[@name='{OCCLUDER_MODEL_NAME}']")
    parameters = receipt.get("occlusion_parameters", {})
    if expected_enabled:
        return bool(
            receipt.get("visual_only_occluder") is True
            and parameters.get("enabled") is True
            and parameters.get("visual_only") is True
            and len(models) == 1
            and not models[0].findall(".//collision")
        )
    return bool(
        receipt.get("visual_only_occluder") is False
        and parameters.get("enabled") is False
        and not models
    )


def summarize_v2(
    output_dir: Path,
    manifest_path: Path,
    model_path: Path | None = None,
    base_domain_id: int | None = None,
) -> dict[str, object]:
    preserved_path = output_dir / "summary.json"
    if not preserved_path.exists():
        raise ValueError("the immutable v1 summary is required before v2 correction")
    preserved = json.loads(preserved_path.read_text(encoding="utf-8"))
    if preserved.get("diagnostic_passed") is not False:
        raise ValueError("v2 correction is only defined for the preserved v1 failure")
    result = summarize(output_dir, manifest_path, model_path, base_domain_id)
    if not result["diagnostic_complete"] or result["blockers"]:
        raise ValueError("cannot correct an incomplete or blocked diagnostic")

    config_path = ROOT / result["condition_config"]["path"]
    config = json.loads(config_path.read_text(encoding="utf-8"))
    thresholds = config["pre_gate_thresholds"]
    by_key = {
        (record["position_label"], record["condition_label"]): record
        for record in result["scenarios"]
    }
    checks = {}
    for position in ("near_left", "near_right", "center", "far_left", "far_right"):
        baseline = by_key[(position, "nominal_none")]
        dim = by_key[(position, "dim_none")]
        heavy = by_key[(position, "nominal_heavy")]
        baseline_dir = output_dir / baseline["scenario_id"]
        dim_dir = output_dir / dim["scenario_id"]
        heavy_dir = output_dir / heavy["scenario_id"]
        corrected = background_aware_position_check(
            baseline["blue_coverage_ratio"],
            dim["blue_coverage_ratio"],
            heavy["blue_coverage_ratio"],
            background_stability_maximum=float(
                thresholds["none_blue_coverage_maximum"]
            ),
            heavy_foreground_delta_minimum=float(
                thresholds["heavy_blue_coverage_minimum"]
            ),
            heavy_coverage_maximum=float(
                thresholds["heavy_blue_coverage_maximum"]
            ),
            no_occluder_worlds_verified=(
                _world_occluder_contract(baseline_dir, False)
                and _world_occluder_contract(dim_dir, False)
            ),
            heavy_occluder_world_verified=_world_occluder_contract(
                heavy_dir, True
            ),
        )
        corrected["dim_is_darker_than_nominal"] = result[
            "position_condition_checks"
        ][position]["dim_is_darker_than_nominal"]
        checks[position] = corrected

    injection_valid = all(
        all(value for value in check.values() if isinstance(value, bool))
        for check in checks.values()
    )
    result.update(
        {
            "schema_version": 2,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "preserved_v1_summary": fingerprint(preserved_path),
            "condition_check_revision": "BACKGROUND_AWARE_BLUE_FOREGROUND_V2",
            "v1_failure_preserved": True,
            "position_condition_checks": checks,
            "condition_injection_valid": injection_valid,
            "diagnostic_passed": bool(
                result["diagnostic_complete"] and injection_valid
            ),
            "interpretation": (
                "Five-position, one-seed, no-motion Shadow diagnostic only. "
                "Schema v2 corrects an injection-only blue-background confound "
                "while preserving every v1 runtime artifact and all YOLO counts. "
                "A pass is not a model-accuracy or formal-robustness acceptance."
            ),
        }
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--base-domain-id", type=int)
    options = parser.parse_args()
    summary_path = options.output_dir / "summary_v2.json"
    if summary_path.exists():
        raise ValueError(f"refusing to overwrite summary: {summary_path}")
    result = summarize_v2(
        options.output_dir,
        options.manifest.resolve(strict=True),
        options.model,
        options.base_domain_id,
    )
    summary_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "diagnostic_passed": result["diagnostic_passed"],
                "summary": str(summary_path),
            },
            sort_keys=True,
        )
    )
    return 0 if result["diagnostic_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
