#!/usr/bin/env python3
"""Validate and emit the frozen ADR 0019 synthetic-capture groups."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from strawberry_sim.core import load_scene_config  # noqa: E402
from strawberry_sim.scene_conditions import (  # noqa: E402
    LIGHTING_LEVELS,
    OCCLUSION_LEVELS,
    load_scene_condition_config,
    resolve_occluder_parameters,
)


EXPECTED_CLASS_ROWS = (
    ("RIPE", 0, 1, "strawberry_1"),
    ("UNRIPE", 1, 2, "strawberry_2"),
)
EXPECTED_CONDITIONS = tuple(
    (f"{light}_{occlusion}", light, occlusion)
    for light in LIGHTING_LEVELS
    for occlusion in OCCLUSION_LEVELS
)
EXPECTED_POSITION_COUNTS = {"train": 12, "heldout": 4}
EXPECTED_MATERIALIZATION_IDS = {"train": 20260601, "heldout": 20260602}
FORMAL_SEED_LABELS = {20260710, 20260711, 20260712}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def bound_path(record: dict, path_key: str, hash_key: str) -> Path:
    path = (ROOT / str(record[path_key])).resolve(strict=True)
    if path != ROOT.resolve() and ROOT.resolve() not in path.parents:
        raise ValueError(f"{path_key} escapes the repository")
    if sha256(path) != str(record[hash_key]):
        raise ValueError(f"{path_key} hash mismatch")
    return path


def vector3(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must contain three values")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{name} must contain finite values")
    return result  # type: ignore[return-value]


def load_capture_manifest(path: Path) -> dict:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if int(raw.get("schema_version", 0)) != 1:
        raise ValueError("unsupported synthetic-capture schema")
    if raw.get("capture_id") != "t70_synthetic_capture_preflight_v1":
        raise ValueError("unexpected synthetic-capture ID")
    if raw.get("scope") != "NON_ACCEPTANCE_SYNTHETIC_CAPTURE_PREFLIGHT":
        raise ValueError("synthetic capture must remain non-acceptance")
    if (
        raw.get("formal_acceptance")
        or raw.get("held_out_test_consumed")
        or raw.get("training_started")
    ):
        raise ValueError("capture preflight cannot assert acceptance, test access, or training")

    contracts = raw["contracts"]
    intervention_path = bound_path(
        contracts, "intervention_relative_path", "intervention_sha256"
    )
    scene_condition_path = bound_path(
        contracts, "scene_condition_relative_path", "scene_condition_sha256"
    )
    d2_path = bound_path(
        contracts, "d2_manifest_relative_path", "d2_manifest_sha256"
    )
    simulation_scene_path = bound_path(
        contracts, "simulation_scene_relative_path", "simulation_scene_sha256"
    )
    bound_path(contracts, "base_world_relative_path", "base_world_sha256")

    intervention = json.loads(intervention_path.read_text(encoding="utf-8"))
    authorization = intervention.get("authorization", {})
    if authorization.get("synthetic_capture_preflight") is not True:
        raise ValueError("intervention does not authorize synthetic capture")
    if any(
        authorization.get(key) is not False
        for key in ("training_now", "perception_control", "formal_matrix", "formal_real_test")
    ):
        raise ValueError("intervention must keep training, motion, and formal gates locked")
    synthetic_contract = intervention["synthetic_data_contract"]
    exclusion_distance = float(
        raw["split_contract"]["minimum_distance_from_exact_d2_position_m"]
    )
    if exclusion_distance != float(
        synthetic_contract["exclude_exact_d2_target_positions_within_m"]
    ):
        raise ValueError("D2 exclusion distance differs from the intervention")
    if raw["split_contract"].get("formal_seed_labels") != [
        20260710,
        20260711,
        20260712,
    ]:
        raise ValueError("formal seed labels differ from ADR 0019")
    if raw["split_contract"].get("formal_seed_labels_allowed") is not False:
        raise ValueError("formal seed labels cannot be allowed")

    scene_condition = load_scene_condition_config(scene_condition_path)
    if int(scene_condition["schema_version"]) != 3:
        raise ValueError("capture must use scene-condition schema v3")
    if set(scene_condition["allowed_materialization_ids"]) & FORMAL_SEED_LABELS:
        raise ValueError("scene-condition config uses a formal seed label")
    if tuple(scene_condition["allowed_materialization_ids"]) != (20260601, 20260602):
        raise ValueError("scene-condition materialization IDs changed")

    scene = load_scene_config(simulation_scene_path)
    scene_classes = {
        (fruit.maturity, fruit.target_id, fruit.model_name)
        for fruit in scene.ordered_fruits
    }
    class_rows = tuple(
        (
            str(item["maturity"]),
            int(item["class_id"]),
            int(item["target_id"]),
            str(item["target_model_name"]),
        )
        for item in raw.get("classes", [])
    )
    if class_rows != EXPECTED_CLASS_ROWS:
        raise ValueError("synthetic class mapping changed")
    if any((maturity, target_id, model) not in scene_classes for maturity, _, target_id, model in class_rows):
        raise ValueError("synthetic class target disagrees with simulation scene")

    conditions = tuple(
        (
            str(item["condition_id"]),
            str(item["lighting"]),
            str(item["occlusion"]),
        )
        for item in raw.get("conditions", [])
    )
    if conditions != EXPECTED_CONDITIONS:
        raise ValueError("synthetic conditions must be the ordered 3x3 matrix")

    d2 = json.loads(d2_path.read_text(encoding="utf-8"))
    d2_positions = tuple(
        vector3(item["target_position_m"], "D2 position")
        for item in d2.get("positions", [])
    )
    if len(d2_positions) != 5:
        raise ValueError("D2 exclusion source must contain five positions")

    split_positions: dict[str, tuple[tuple[str, tuple[float, float, float]], ...]] = {}
    all_position_ids = set()
    all_positions = set()
    for split, expected_count in EXPECTED_POSITION_COUNTS.items():
        entry = raw.get("splits", {}).get(split, {})
        materialization_id = int(entry.get("materialization_id", 0))
        if materialization_id != EXPECTED_MATERIALIZATION_IDS[split]:
            raise ValueError(f"{split} materialization ID changed")
        if materialization_id in FORMAL_SEED_LABELS:
            raise ValueError("formal seed label appears in a synthetic split")
        positions = tuple(
            (
                str(item["position_id"]),
                vector3(item["position_m"], f"{split} position"),
            )
            for item in entry.get("positions", [])
        )
        if len(positions) != expected_count:
            raise ValueError(f"{split} must contain {expected_count} positions")
        for position_id, position in positions:
            if position_id in all_position_ids or position in all_positions:
                raise ValueError("train and heldout positions must be disjoint")
            all_position_ids.add(position_id)
            all_positions.add(position)
            minimum = min(math.dist(position, reference) for reference in d2_positions)
            if minimum < exclusion_distance - 1e-12:
                raise ValueError(f"{position_id} violates the D2 exclusion radius")
            resolve_occluder_parameters(scene_condition, "heavy", position)
        split_positions[split] = positions

    capture = raw["capture"]
    if capture != {
        "image_width": 640,
        "image_height": 480,
        "image_format": "png",
        "png_compression": 3,
        "fruit_radius_m": 0.035,
        "settled_frames_after_pose": 5,
        "images_per_render_configuration": 1,
        "label_format": "yolo_normalized_xywh",
        "label_source": "gazebo_truth_sphere_projection",
        "labels_per_image": 1,
        "group_count": 36,
        "expected_train_images": 216,
        "expected_heldout_images": 72,
        "expected_total_images": 288,
    }:
        raise ValueError("capture counts or encoding contract changed")
    expected_groups = len(EXPECTED_POSITION_COUNTS) * len(class_rows) * len(conditions)
    if int(capture["group_count"]) != expected_groups:
        raise ValueError("capture group count does not match the matrix")
    expected_images = sum(
        len(split_positions[split]) * len(class_rows) * len(conditions)
        for split in EXPECTED_POSITION_COUNTS
    )
    if int(capture["expected_total_images"]) != expected_images:
        raise ValueError("capture image count does not match the matrix")

    parked = raw.get("parked_models", [])
    if len(parked) != 3 or {
        (str(item["model_name"]), int(item["target_id"])) for item in parked
    } != {("strawberry_1", 1), ("strawberry_2", 2), ("strawberry_3", 3)}:
        raise ValueError("all three fruit models require frozen parked poses")
    for item in parked:
        vector3(item["position_m"], "parked position")

    raw["_resolved_paths"] = {
        "intervention": str(intervention_path),
        "scene_condition": str(scene_condition_path),
        "d2_manifest": str(d2_path),
        "simulation_scene": str(simulation_scene_path),
    }
    raw["_split_positions"] = split_positions
    return raw


def iter_groups(manifest: dict):
    for split in ("train", "heldout"):
        materialization_id = manifest["splits"][split]["materialization_id"]
        first_position = manifest["splits"][split]["positions"][0]["position_m"]
        for class_entry in manifest["classes"]:
            maturity_lower = class_entry["maturity"].lower()
            for condition in manifest["conditions"]:
                group_id = (
                    f"syn__{split}__{maturity_lower}__{condition['condition_id']}"
                )
                yield {
                    "group_id": group_id,
                    "split": split,
                    "maturity": class_entry["maturity"],
                    "class_id": class_entry["class_id"],
                    "target_id": class_entry["target_id"],
                    "target_model_name": class_entry["target_model_name"],
                    "lighting": condition["lighting"],
                    "occlusion": condition["occlusion"],
                    "condition_id": condition["condition_id"],
                    "materialization_id": materialization_id,
                    "first_position_m": first_position,
                }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--emit-groups-tsv", action="store_true")
    options = parser.parse_args()
    manifest = load_capture_manifest(options.manifest.resolve(strict=True))
    groups = list(iter_groups(manifest))
    if options.emit_groups_tsv:
        for group in groups:
            print(
                "\t".join(
                    (
                        str(group["group_id"]),
                        str(group["split"]),
                        str(group["maturity"]),
                        str(group["class_id"]),
                        str(group["target_id"]),
                        str(group["target_model_name"]),
                        str(group["condition_id"]),
                        str(group["lighting"]),
                        str(group["occlusion"]),
                        str(group["materialization_id"]),
                        *(format(float(value), ".12g") for value in group["first_position_m"]),
                    )
                )
            )
    else:
        print(
            json.dumps(
                {
                    "capture_id": manifest["capture_id"],
                    "groups": len(groups),
                    "train_images": manifest["capture"]["expected_train_images"],
                    "heldout_images": manifest["capture"]["expected_heldout_images"],
                    "total_images": manifest["capture"]["expected_total_images"],
                    "training_started": False,
                    "formal_acceptance": False,
                    "held_out_test_consumed": False,
                },
                sort_keys=True,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
