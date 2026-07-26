#!/usr/bin/env python3
"""Validate and emit the frozen T70-D2 five-position diagnostic matrix."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_benchmark"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from strawberry_benchmark.scenarios import load_benchmark_spec  # noqa: E402
from strawberry_sim.scene_conditions import (  # noqa: E402
    load_scene_condition_config,
    resolve_occluder_parameters,
    validate_benchmark_condition_mapping,
)


EXPECTED_POSITIONS = (
    ("near_left", "camera_clear_01", (0.44, -0.10, 0.48)),
    ("near_right", "camera_clear_03", (0.50, -0.05, 0.52)),
    ("center", "camera_clear_02", (0.47, -0.075, 0.50)),
    ("far_left", "camera_clear_05", (0.44, 0.00, 0.52)),
    ("far_right", "camera_clear_04", (0.47, -0.025, 0.48)),
)
EXPECTED_CONDITIONS = (
    ("nominal_none", "nominal", "none", "camera_clear_baseline"),
    ("dim_none", "dim", "none", "one_factor_lighting_stress"),
    ("nominal_heavy", "nominal", "heavy", "one_factor_occlusion_stress"),
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _bound_path(record: dict, path_key: str, hash_key: str) -> Path:
    path = (ROOT / str(record[path_key])).resolve(strict=True)
    if path != ROOT.resolve() and ROOT.resolve() not in path.parents:
        raise ValueError(f"{path_key} escapes the repository")
    if _sha256(path) != str(record[hash_key]):
        raise ValueError(f"{path_key} hash mismatch")
    return path


def _vector(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must contain three values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain finite values")
    return result  # type: ignore[return-value]


def load_diagnostic_manifest(path: Path, model_override: Path | None = None) -> dict:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if int(raw.get("schema_version", 0)) != 1:
        raise ValueError("unsupported multiposition diagnostic schema")
    if raw.get("diagnostic_id") != "t70_shadow_multiposition_fixed_window_v1":
        raise ValueError("unexpected diagnostic ID")
    if raw.get("scope") != "NON_ACCEPTANCE_NO_MOTION_DIAGNOSTIC":
        raise ValueError("diagnostic scope must remain non-acceptance and no-motion")
    if raw.get("formal_acceptance") or raw.get("held_out_test_consumed"):
        raise ValueError("diagnostic cannot assert acceptance or held-out access")
    if raw.get("control") != {
        "robot_motion_started": False,
        "perception_role": "shadow_observation_only",
        "detections_topic": "/strawberry/shadow/detections",
        "target_pose_topic": "/strawberry/shadow/target_pose",
    }:
        raise ValueError("control boundary differs from the frozen no-motion design")
    if raw.get("measurement") != {
        "boundary": "after_condition_probe_before_robot_motion",
        "fixed_detection_frames": 60,
        "post_window_wait_sec": 1.0,
        "condition_probe_frames": 5,
        "expected_distinct_materialized_worlds": 7,
    }:
        raise ValueError("fixed-window measurement contract changed")

    contract = raw["condition_contract"]
    benchmark_path = _bound_path(
        contract, "benchmark_config_relative_path", "benchmark_config_sha256"
    )
    scene_path = _bound_path(
        contract, "scene_config_relative_path", "scene_config_sha256"
    )
    _bound_path(contract, "base_world_relative_path", "base_world_sha256")
    benchmark = load_benchmark_spec(benchmark_path)
    scene = load_scene_condition_config(scene_path)
    mapping = validate_benchmark_condition_mapping(
        scene,
        lighting_levels=benchmark.lighting_levels,
        occlusion_levels=benchmark.occlusion_levels,
    )
    seed = int(contract.get("condition_world_seed", -1))
    if seed != int(scene["validation_scene"]["seed"]):
        raise ValueError("diagnostic seed differs from the scene-config seed")
    if contract.get("seed_role") != "single_diagnostic_materialization_only":
        raise ValueError("single-seed role must be explicit")
    if int(contract.get("independent_random_seed_count_claimed", -1)) != 0:
        raise ValueError("this diagnostic cannot claim independent random seeds")

    shadow = raw["shadow"]
    model_path = (
        model_override.resolve(strict=True)
        if model_override
        else (ROOT / str(shadow["model_relative_path"])).resolve(strict=True)
    )
    if model_path.stat().st_size != int(shadow["model_size_bytes"]):
        raise ValueError("Shadow model size mismatch")
    if _sha256(model_path) != str(shadow["model_sha256"]):
        raise ValueError("Shadow model hash mismatch")
    if float(shadow["confidence_threshold"]) != 0.31:
        raise ValueError("Shadow threshold must remain 0.31")
    if shadow.get("model_role") != "baseline__best_below_t30_gate":
        raise ValueError("model role must retain the below-gate warning")

    setup = raw["scene_setup"]
    if setup.get("mode") != "single_target_isolation":
        raise ValueError("diagnostic must use single-target isolation")
    if setup.get("target_model_name") != "strawberry_1" or int(
        setup.get("target_id", 0)
    ) != 1:
        raise ValueError("target identity must remain strawberry_1 / ID 1")
    parked = setup.get("parked_models", [])
    if len(parked) != 2 or {
        (str(item["model_name"]), int(item["target_id"])) for item in parked
    } != {("strawberry_2", 2), ("strawberry_3", 3)}:
        raise ValueError("diagnostic must park strawberry IDs 2 and 3")

    positions = tuple(
        (
            str(item["position_label"]),
            str(item["source_position_label"]),
            _vector(item["target_position_m"], "target_position_m"),
        )
        for item in raw.get("positions", [])
    )
    if positions != EXPECTED_POSITIONS:
        raise ValueError("positions differ from the five frozen reachable candidates")
    conditions = tuple(
        (
            str(item["condition_label"]),
            str(item["lighting"]),
            str(item["occlusion"]),
            str(item["purpose"]),
        )
        for item in raw.get("conditions", [])
    )
    if conditions != EXPECTED_CONDITIONS:
        raise ValueError("conditions differ from the frozen one-factor design")
    valid_pairs = {(item["lighting"], item["occlusion"]) for item in mapping}
    if any((light, occ) not in valid_pairs for _, light, occ, _ in conditions):
        raise ValueError("diagnostic contains a condition outside the benchmark mapping")
    for _, _, position in positions:
        resolve_occluder_parameters(scene, "heavy", position)

    raw["_resolved_model_path"] = str(model_path)
    raw["_condition_mapping"] = mapping
    raw["_scenario_count"] = len(positions) * len(conditions)
    return raw


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--emit-tsv", action="store_true")
    parser.add_argument("--emit-park-tsv", action="store_true")
    options = parser.parse_args()
    manifest = load_diagnostic_manifest(
        options.manifest.resolve(strict=True), options.model
    )
    setup = manifest["scene_setup"]
    if options.emit_tsv:
        for position in manifest["positions"]:
            for condition in manifest["conditions"]:
                scenario_id = (
                    f"multipos__{position['position_label']}__"
                    f"{condition['condition_label']}"
                )
                print(
                    "\t".join(
                        (
                            scenario_id,
                            position["position_label"],
                            condition["condition_label"],
                            condition["lighting"],
                            condition["occlusion"],
                            setup["target_model_name"],
                            str(setup["target_id"]),
                            *(
                                format(float(value), ".12g")
                                for value in position["target_position_m"]
                            ),
                        )
                    )
                )
    elif options.emit_park_tsv:
        for item in setup["parked_models"]:
            print(
                "\t".join(
                    (
                        item["model_name"],
                        str(item["target_id"]),
                        *(format(float(value), ".12g") for value in item["position_m"]),
                    )
                )
            )
    else:
        print(
            json.dumps(
                {
                    "diagnostic_id": manifest["diagnostic_id"],
                    "scenario_count": manifest["_scenario_count"],
                    "position_count": len(manifest["positions"]),
                    "condition_count": len(manifest["conditions"]),
                    "fixed_shadow_frames": (
                        manifest["_scenario_count"]
                        * manifest["measurement"]["fixed_detection_frames"]
                    ),
                    "robot_motion_started": False,
                    "formal_acceptance": False,
                    "held_out_test_consumed": False,
                },
                sort_keys=True,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
