#!/usr/bin/env python3
"""Validate and emit the rendered-occlusion localization matrix contract."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "strawberry_sim"))

from strawberry_sim.scene_conditions import (  # noqa: E402
    load_scene_condition_config,
    resolve_occluder_parameters,
)


EXPECTED_POSITIONS = (
    ("near_left", (0.44, -0.10, 0.48)),
    ("near_right", (0.50, -0.05, 0.52)),
    ("center", (0.47, -0.075, 0.50)),
    ("far_left", (0.44, 0.00, 0.52)),
    ("far_right", (0.47, -0.025, 0.48)),
)
EXPECTED_CONDITIONS = (
    ("nominal_none", "nominal", "none"),
    ("nominal_partial", "nominal", "partial"),
    ("nominal_heavy", "nominal", "heavy"),
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _bound_path(record: dict, label: str) -> Path:
    root = ROOT.resolve(strict=True)
    path = (root / str(record.get("path"))).resolve(strict=True)
    if path != root and root not in path.parents:
        raise ValueError(f"{label} escapes the repository")
    if (
        path.stat().st_size != int(record.get("size_bytes", -1))
        or _sha256(path) != record.get("sha256")
    ):
        raise ValueError(f"{label} binding changed")
    return path


def _position(value: object) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("target position must contain three values")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError("target position must be finite")
    return result  # type: ignore[return-value]


def load_manifest(path: Path) -> dict:
    manifest_path = path.resolve(strict=True)
    raw = json.loads(manifest_path.read_text(encoding="utf-8"))
    if (
        raw.get("schema_version") != 1
        or raw.get("matrix_id") != "rendered_occlusion_localization_matrix_v1"
        or raw.get("scope") != "NON_FORMAL_RENDERED_OCCLUSION_LOCALIZATION"
        or raw.get("status") != "FROZEN_EXECUTION_AUTHORIZED"
    ):
        raise ValueError("unexpected rendered-localization matrix contract")
    if raw.get("formal_acceptance") or raw.get("held_out_test_consumed"):
        raise ValueError("matrix cannot claim formal acceptance or held-out access")

    safety = raw.get("safety", {})
    expected_safety = {
        "robot_motion_authorized": False,
        "gripper_command_authorized": False,
        "manipulation_authorized": False,
        "orchestrator_authorized": False,
        "attachment_authorized": False,
        "perception_model_authorized": False,
        "simulated_fruit_pose_motion_authorized": True,
        "visual_only_occluder_authorized": True,
    }
    if safety != expected_safety:
        raise ValueError("matrix safety boundary changed")

    runtime = raw.get("runtime", {})
    if runtime != {
        "samples_per_scenario": 20,
        "scenario_count": 15,
        "simulation_seed": 20260805,
        "base_ros_domain_id": 150,
        "pair_wait_sec": 0.5,
        "oracle_bbox_padding": 1.0,
        "ground_truth_association_in_localizers": False,
    }:
        raise ValueError("matrix runtime changed")

    thresholds = raw.get("thresholds", {})
    if thresholds != {
        "none_blue_coverage_max": 0.02,
        "partial_blue_coverage_min": 0.05,
        "heavy_blue_coverage_min": 0.35,
        "heavy_blue_coverage_max": 0.98,
        "heavy_minus_partial_coverage_min": 0.15,
        "geometry_none_acceptance_min_each_position": 0.95,
        "geometry_partial_acceptance_min_each_position": 0.80,
        "geometry_heavy_acceptance_min_each_position": 0.60,
        "geometry_p95_error_mm_max_each_condition": 30.0,
        "planning_sigma_m_max": 0.015,
    }:
        raise ValueError("matrix thresholds changed")

    execution = raw.get("execution", {})
    if execution != {
        "maximum_claims": 1,
        "retry_authorized": False,
        "output_directory": (
            "results/development/rendered_occlusion_localization_matrix_v1"
        ),
    }:
        raise ValueError("matrix execution policy changed")

    positions = tuple(
        (str(item["label"]), _position(item["position_m"]))
        for item in raw.get("positions", [])
    )
    if positions != EXPECTED_POSITIONS:
        raise ValueError("matrix positions changed")
    conditions = tuple(
        (str(item["label"]), str(item["lighting"]), str(item["occlusion"]))
        for item in raw.get("conditions", [])
    )
    if conditions != EXPECTED_CONDITIONS:
        raise ValueError("matrix conditions changed")

    bindings = raw.get("bindings", {})
    expected_bindings = {
        "decision_record",
        "base_world",
        "scene_manifest",
        "condition_config",
        "center_localization_config",
        "geometry_localization_config",
        "localization_core",
        "localization_node",
        "trial_probe",
        "validator",
        "summarizer",
        "runner",
    }
    if set(bindings) != expected_bindings:
        raise ValueError("matrix binding set changed")
    resolved = {label: _bound_path(record, label) for label, record in bindings.items()}
    condition_config = load_scene_condition_config(resolved["condition_config"])
    if float(condition_config["validation_scene"]["fruit_radius_m"]) != 0.026:
        raise ValueError("condition config must use the Blender-v2 26 mm radius")
    for _, position in positions:
        for _, _, occlusion in conditions:
            resolve_occluder_parameters(condition_config, occlusion, position)

    raw["_manifest_path"] = str(manifest_path)
    raw["_manifest_sha256"] = _sha256(manifest_path)
    raw["_resolved_paths"] = {
        label: str(value) for label, value in resolved.items()
    }
    return raw


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--emit-tsv", action="store_true")
    options = parser.parse_args()
    manifest = load_manifest(options.manifest)
    if options.emit_tsv:
        index = 0
        for position in manifest["positions"]:
            for condition in manifest["conditions"]:
                index += 1
                print(
                    "\t".join(
                        (
                            str(index),
                            f"rendered__{position['label']}__{condition['occlusion']}",
                            str(position["label"]),
                            str(condition["label"]),
                            str(condition["lighting"]),
                            str(condition["occlusion"]),
                            *(format(float(value), ".12g") for value in position["position_m"]),
                        )
                    )
                )
    else:
        print(
            json.dumps(
                {
                    "matrix_id": manifest["matrix_id"],
                    "scenario_count": manifest["runtime"]["scenario_count"],
                    "samples_per_scenario": manifest["runtime"]["samples_per_scenario"],
                    "robot_motion_authorized": False,
                    "perception_model_authorized": False,
                },
                sort_keys=True,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
