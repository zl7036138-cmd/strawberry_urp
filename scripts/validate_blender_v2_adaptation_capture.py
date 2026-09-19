#!/usr/bin/env python3
"""Validate and emit ADR-0037 Blender-v2 capture groups."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from strawberry_sim.core import load_scene_config  # noqa: E402
from strawberry_sim.scene_conditions import (  # noqa: E402
    load_scene_condition_config,
)
from strawberry_sim.synthetic_capture import (  # noqa: E402
    normalized_orientation_xyzw,
)


EXPECTED_CLASSES = (
    ("RIPE", 0, 1, "strawberry_1"),
    ("UNRIPE", 1, 2, "strawberry_2"),
)
EXPECTED_CONDITIONS = (
    ("dim_none", "dim", "none"),
    ("nominal_none", "nominal", "none"),
    ("bright_none", "bright", "none"),
)
EXPECTED_POSITION_COUNTS = {"train": 12, "heldout": 4}
EXPECTED_MATERIALIZATION_IDS = {"train": 20260601, "heldout": 20260602}


def _repository_path(value: object, label: str) -> Path:
    path = (ROOT / str(value)).resolve(strict=True)
    if ROOT.resolve() != path and ROOT.resolve() not in path.parents:
        raise ValueError(f"{label} escapes the repository")
    return path


def _position(value: object, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{label} must contain three values")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{label} must be finite")
    if not (
        0.40 <= result[0] <= 0.60
        and -0.08 <= result[1] <= 0.14
        and 0.51 <= result[2] <= 0.59
    ):
        raise ValueError(f"{label} is outside the frozen plant capture volume")
    return result


def load_capture_manifest(path: Path) -> dict:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if raw.get("capture_id") != "blender_v2_unripe_adapt_capture_v1":
        raise ValueError("unexpected Blender-v2 capture ID")
    if raw.get("scope") != "NON_ACCEPTANCE_BLENDER_V2_ADAPTATION_CAPTURE":
        raise ValueError("capture scope changed")
    if (
        raw.get("formal_acceptance")
        or raw.get("held_out_test_consumed")
        or raw.get("training_started")
    ):
        raise ValueError("capture crosses a frozen safety boundary")

    decision = _repository_path(raw["decision_record"], "decision record")
    if decision.name != "0037-freeze-blender-v2-unripe-adaptation-research.md":
        raise ValueError("capture is not bound to ADR 0037")
    scene_path = _repository_path(raw["simulation_scene"], "scene manifest")
    base_world = _repository_path(raw["base_world"], "base world")
    condition_path = _repository_path(
        raw["scene_condition_config"], "scene condition config"
    )
    scene = load_scene_config(scene_path)
    conditions = load_scene_condition_config(condition_path)
    if scene.fruit_collision_radius_m != 0.026:
        raise ValueError("Blender-v2 scene radius must be 0.026 m")
    if float(
        conditions["validation_scene"]["fruit_radius_m"]
    ) != scene.fruit_collision_radius_m:
        raise ValueError("capture and scene-condition radii differ")
    if base_world.name != "strawberry_orchard.sdf":
        raise ValueError("capture must use the canonical Blender-v2 world")

    class_rows = tuple(
        (
            str(item["maturity"]),
            int(item["class_id"]),
            int(item["target_id"]),
            str(item["target_model_name"]),
        )
        for item in raw.get("classes", [])
    )
    if class_rows != EXPECTED_CLASSES:
        raise ValueError("class contract changed")
    scene_rows = {
        (fruit.maturity, fruit.target_id, fruit.model_name)
        for fruit in scene.ordered_fruits
    }
    if any(
        (maturity, target_id, model) not in scene_rows
        for maturity, _, target_id, model in class_rows
    ):
        raise ValueError("capture class does not exist in the scene")

    condition_rows = tuple(
        (
            str(item["condition_id"]),
            str(item["lighting"]),
            str(item["occlusion"]),
        )
        for item in raw.get("conditions", [])
    )
    if condition_rows != EXPECTED_CONDITIONS:
        raise ValueError("capture must use the frozen three no-occlusion lights")

    capture = raw.get("capture", {})
    expected_capture = {
        "image_width": 640,
        "image_height": 480,
        "image_format": "png",
        "png_compression": 3,
        "fruit_radius_m": 0.026,
        "settled_frames_after_pose": 5,
        "images_per_render_configuration": 1,
        "label_format": "yolo_normalized_xywh",
        "label_source": "gazebo_truth_sphere_projection",
        "labels_per_image": 1,
        "group_count": 12,
        "expected_train_images": 72,
        "expected_heldout_images": 24,
        "expected_total_images": 96,
    }
    if capture != expected_capture:
        raise ValueError("capture encoding or counts changed")

    seen_ids: set[str] = set()
    seen_pose_keys: set[tuple[float, ...]] = set()
    for split, expected_count in EXPECTED_POSITION_COUNTS.items():
        split_row = raw.get("splits", {}).get(split, {})
        if (
            int(split_row.get("materialization_id", 0))
            != EXPECTED_MATERIALIZATION_IDS[split]
        ):
            raise ValueError(f"{split} materialization ID changed")
        poses = split_row.get("positions", [])
        if not isinstance(poses, list) or len(poses) != expected_count:
            raise ValueError(f"{split} must contain {expected_count} poses")
        for row in poses:
            pose_id = str(row.get("position_id", ""))
            if not pose_id or pose_id in seen_ids:
                raise ValueError("pose IDs must be non-empty and unique")
            seen_ids.add(pose_id)
            position = _position(row.get("position_m"), f"{pose_id} position")
            orientation = normalized_orientation_xyzw(
                row.get("orientation_xyzw")
            )
            key = (*position, *orientation)
            if key in seen_pose_keys:
                raise ValueError("train and held-out pose tuples must be disjoint")
            seen_pose_keys.add(key)

    parked = raw.get("parked_models", [])
    if {
        (str(item["model_name"]), int(item["target_id"]))
        for item in parked
    } != {
        ("strawberry_1", 1),
        ("strawberry_2", 2),
        ("strawberry_3", 3),
    }:
        raise ValueError("all three fruit identities require parked poses")

    raw["_resolved_paths"] = {
        "decision": str(decision),
        "scene": str(scene_path),
        "base_world": str(base_world),
        "conditions": str(condition_path),
    }
    return raw


def iter_groups(manifest: dict):
    for split in ("train", "heldout"):
        materialization_id = manifest["splits"][split]["materialization_id"]
        first_position = manifest["splits"][split]["positions"][0]["position_m"]
        for class_entry in manifest["classes"]:
            maturity = str(class_entry["maturity"])
            for condition in manifest["conditions"]:
                yield {
                    "group_id": (
                        f"syn__{split}__{maturity.lower()}__"
                        f"{condition['condition_id']}"
                    ),
                    "split": split,
                    "maturity": maturity,
                    "class_id": int(class_entry["class_id"]),
                    "target_id": int(class_entry["target_id"]),
                    "target_model_name": str(
                        class_entry["target_model_name"]
                    ),
                    "condition_id": str(condition["condition_id"]),
                    "lighting": str(condition["lighting"]),
                    "occlusion": str(condition["occlusion"]),
                    "materialization_id": int(materialization_id),
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
                        str(group["condition_id"]),
                        str(group["lighting"]),
                        str(group["occlusion"]),
                        str(group["materialization_id"]),
                        *(
                            format(float(value), ".12g")
                            for value in group["first_position_m"]
                        ),
                    )
                )
            )
    else:
        print(
            json.dumps(
                {
                    "capture_id": manifest["capture_id"],
                    "groups": len(groups),
                    "train_images": 72,
                    "heldout_images": 24,
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
