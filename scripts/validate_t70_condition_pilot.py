#!/usr/bin/env python3
"""Validate and optionally emit the frozen T70 condition pilot manifest."""

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
    validate_benchmark_condition_mapping,
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _bound_path(record: dict, path_key: str, hash_key: str) -> Path:
    path = (ROOT / str(record[path_key])).resolve(strict=True)
    if ROOT.resolve() not in path.parents:
        raise ValueError(f"{path_key} escapes the repository")
    if _sha256(path) != record[hash_key]:
        raise ValueError(f"{path_key} hash mismatch")
    return path


def load_pilot_manifest(path: Path, model_override: Path | None = None) -> dict:
    raw = json.loads(path.read_text(encoding="utf-8"))
    schema_version = int(raw.get("schema_version", 0))
    if schema_version not in (1, 2):
        raise ValueError("unsupported pilot schema")
    if raw.get("scope") != "NON_ACCEPTANCE_CONDITION_PILOT":
        raise ValueError("pilot scope must remain non-acceptance")
    if raw.get("formal_acceptance") or raw.get("held_out_test_consumed"):
        raise ValueError("pilot cannot assert formal acceptance or test access")
    if raw.get("control") != {
        "source": "oracle",
        "topic": "/strawberry/oracle/target_pose",
        "target_id": 1,
    }:
        raise ValueError("pilot control must remain frozen to Oracle target 1")

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
    if int(contract["condition_world_seed"]) != int(
        scene["validation_scene"]["seed"]
    ):
        raise ValueError("condition world seed differs from the pre-gate seed")

    shadow = raw["shadow"]
    model_path = (
        model_override.resolve(strict=True)
        if model_override
        else (ROOT / shadow["model_relative_path"]).resolve(strict=True)
    )
    if model_path.stat().st_size != int(shadow["model_size_bytes"]):
        raise ValueError("shadow model size mismatch")
    if _sha256(model_path) != shadow["model_sha256"]:
        raise ValueError("shadow model hash mismatch")
    if float(shadow["confidence_threshold"]) != 0.31:
        raise ValueError("pilot confidence threshold must remain 0.31")

    setup = raw["scene_setup"]
    if setup.get("mode") != "single_target_isolation":
        raise ValueError("pilot must use single-target isolation")
    if setup.get("formal_position_label") is not False:
        raise ValueError("pilot position cannot be called formal")
    if setup.get("target_model_name") != "strawberry_1":
        raise ValueError("pilot target model must be strawberry_1")
    if int(setup.get("target_id")) != 1:
        raise ValueError("pilot target ID must be 1")
    target = tuple(float(value) for value in setup["target_position_m"])
    if len(target) != 3 or not all(math.isfinite(value) for value in target):
        raise ValueError("target position must contain three finite values")
    parked = setup.get("parked_models", [])
    if len(parked) != 2 or {int(item["target_id"]) for item in parked} != {2, 3}:
        raise ValueError("pilot must park target IDs 2 and 3")

    expected = (
        ("pilot_nominal_none", "nominal", "none"),
        ("pilot_dim_none", "dim", "none"),
        ("pilot_nominal_heavy", "nominal", "heavy"),
    )
    actual = tuple(
        (item["scenario_id"], item["lighting"], item["occlusion"])
        for item in raw.get("pilot_conditions", [])
    )
    if actual != expected:
        raise ValueError("pilot conditions differ from the frozen one-factor design")
    valid_pairs = {(item["lighting"], item["occlusion"]) for item in mapping}
    if any((light, occ) not in valid_pairs for _, light, occ in actual):
        raise ValueError("pilot contains a condition outside the benchmark mapping")
    if schema_version == 2 and raw.get("shadow_measurement") != {
        "boundary": "after_condition_probe_before_robot_motion",
        "fixed_detection_frames": 60,
        "post_window_wait_sec": 1.0,
    }:
        raise ValueError("v2 pilot must freeze the exact post-setup Shadow window")
    raw["_resolved_model_path"] = str(model_path)
    raw["_condition_mapping"] = mapping
    return raw


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--emit-tsv", action="store_true")
    parser.add_argument("--emit-park-tsv", action="store_true")
    options = parser.parse_args()
    manifest = load_pilot_manifest(options.manifest.resolve(strict=True), options.model)
    setup = manifest["scene_setup"]
    if options.emit_tsv:
        position = setup["target_position_m"]
        for condition in manifest["pilot_conditions"]:
            print(
                "\t".join(
                    (
                        condition["scenario_id"],
                        condition["lighting"],
                        condition["occlusion"],
                        setup["target_model_name"],
                        str(setup["target_id"]),
                        *(format(float(value), ".12g") for value in position),
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
                    "pilot_conditions": len(manifest["pilot_conditions"]),
                    "mapped_benchmark_conditions": len(
                        manifest["_condition_mapping"]
                    ),
                    "formal_acceptance": False,
                    "held_out_test_consumed": False,
                    "schema_version": manifest["schema_version"],
                },
                sort_keys=True,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
