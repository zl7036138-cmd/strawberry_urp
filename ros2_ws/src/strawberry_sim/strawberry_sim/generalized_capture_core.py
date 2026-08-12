"""Development-only capture contracts for randomized multi-plant scenes."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from .synthetic_capture_core import project_sphere_in_camera, xyxy_to_yolo


SPLIT_ORDER = ("train", "validation", "qualification")
VALID_PROFILES = {"mixed", "all_unripe"}
VALID_POSITION_BANDS = {"near", "middle", "far"}
VALID_OCCLUSIONS = {"none", "partial", "heavy"}


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _positive_integer(value: object, label: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a positive integer")
    result = int(value)
    if result <= 0 or result != value:
        raise ValueError(f"{label} must be a positive integer")
    return result


def _string_schedule(
    raw: object,
    *,
    label: str,
    allowed: set[str],
) -> tuple[str, ...]:
    if not isinstance(raw, list) or not raw:
        raise ValueError(f"{label} schedule must be a non-empty list")
    result = tuple(str(value) for value in raw)
    if any(value not in allowed for value in result):
        raise ValueError(f"{label} schedule contains an unsupported value")
    return result


def load_capture_plan(
    path: str | Path,
    formal_matrix_path: str | Path,
) -> dict[str, Any]:
    """Load a development plan and prove it is disjoint from formal seeds."""

    plan_path = Path(path).resolve(strict=True)
    matrix_path = Path(formal_matrix_path).resolve(strict=True)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    matrix = json.loads(matrix_path.read_text(encoding="utf-8"))
    if int(plan.get("schema_version", 0)) != 1:
        raise ValueError("development capture schema must be 1")
    if plan.get("capture_id") != "generalized_development_capture_v1":
        raise ValueError("unexpected development capture ID")
    if plan.get("scope") != "DEVELOPMENT_ONLY_RANDOMIZED_MULTI_PLANT_CAPTURE":
        raise ValueError("capture scope is not development-only")
    if plan.get("formal_acceptance") or plan.get("formal_results_consumed"):
        raise ValueError("development capture crosses the formal boundary")
    if matrix.get("development_or_training_allowed") is not False:
        raise ValueError("formal matrix does not prohibit development use")
    if Path(str(plan.get("formal_seed_matrix", ""))).name != matrix_path.name:
        raise ValueError("capture plan is not bound to the supplied formal matrix")
    if plan.get("formal_seed_matrix_sha256") != sha256_file(matrix_path):
        raise ValueError("formal matrix hash differs from the capture plan")

    capture = plan.get("capture")
    if not isinstance(capture, Mapping):
        raise ValueError("capture settings are missing")
    if (int(capture.get("image_width", 0)), int(capture.get("image_height", 0))) != (
        640,
        480,
    ):
        raise ValueError("development capture must use 640x480 images")
    if capture.get("class_ids") != {"RIPE": 0, "UNRIPE": 1}:
        raise ValueError("development class mapping changed")
    for key in ("minimum_visible_fraction", "depth_surface_padding_m"):
        value = float(capture.get(key, 0.0))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"capture {key} must be finite and positive")
    _positive_integer(capture.get("minimum_visible_pixels"), "minimum_visible_pixels")
    _positive_integer(capture.get("settled_frames"), "settled_frames")

    schedule = plan.get("schedule")
    if not isinstance(schedule, Mapping):
        raise ValueError("capture schedule is missing")
    _string_schedule(
        schedule.get("profiles"), label="profile", allowed=VALID_PROFILES
    )
    plant_counts = tuple(int(value) for value in schedule.get("plant_counts", []))
    if set(plant_counts) != {1, 2, 3}:
        raise ValueError("plant-count schedule must cover 1, 2, and 3")
    _string_schedule(
        schedule.get("position_bands"),
        label="position band",
        allowed=VALID_POSITION_BANDS,
    )
    _string_schedule(
        schedule.get("occlusions"),
        label="occlusion",
        allowed=VALID_OCCLUSIONS,
    )

    splits = plan.get("splits")
    if not isinstance(splits, Mapping) or set(splits) != set(SPLIT_ORDER):
        raise ValueError("capture requires train, validation, and qualification splits")
    all_seeds: set[int] = set()
    for split in SPLIT_ORDER:
        row = splits[split]
        start = _positive_integer(row.get("seed_start"), f"{split} seed_start")
        count = _positive_integer(row.get("count"), f"{split} count")
        if bool(row.get("training_allowed")) != (split == "train"):
            raise ValueError("only the train split may authorize optimization")
        seeds = set(range(start, start + count))
        if all_seeds & seeds:
            raise ValueError("development split seed ranges overlap")
        all_seeds.update(seeds)

    formal_seeds = {int(row["seed"]) for row in matrix.get("scenarios", [])}
    overlap = all_seeds & formal_seeds
    if overlap:
        raise ValueError(f"development seeds overlap formal seeds: {sorted(overlap)}")
    plan["_plan_path"] = str(plan_path)
    plan["_formal_matrix_path"] = str(matrix_path)
    plan["_formal_seeds"] = sorted(formal_seeds)
    return plan


def iter_capture_specs(plan: Mapping[str, Any]) -> tuple[dict[str, Any], ...]:
    """Expand compact seed ranges to deterministic, balanced scene specs."""

    schedule = plan["schedule"]
    profiles = tuple(str(value) for value in schedule["profiles"])
    plants = tuple(int(value) for value in schedule["plant_counts"])
    bands = tuple(str(value) for value in schedule["position_bands"])
    occlusions = tuple(str(value) for value in schedule["occlusions"])
    specs: list[dict[str, Any]] = []
    for split_index, split in enumerate(SPLIT_ORDER):
        split_row = plan["splits"][split]
        start = int(split_row["seed_start"])
        for index in range(int(split_row["count"])):
            seed = start + index
            specs.append(
                {
                    "sample_id": f"generalized__{split}__seed_{seed:06d}",
                    "split": split,
                    "seed": seed,
                    "profile": profiles[index % len(profiles)],
                    "plant_count": plants[(index + split_index) % len(plants)],
                    "position_band": bands[
                        (index // len(profiles) + split_index) % len(bands)
                    ],
                    "occlusion": occlusions[
                        (index + index // len(plants) + split_index)
                        % len(occlusions)
                    ],
                    "training_allowed": split == "train",
                }
            )
    return tuple(specs)


def find_capture_spec(
    plan: Mapping[str, Any], *, split: str, seed: int
) -> dict[str, Any]:
    matches = [
        row
        for row in iter_capture_specs(plan)
        if row["split"] == split and int(row["seed"]) == int(seed)
    ]
    if len(matches) != 1:
        raise ValueError("split/seed is outside the frozen development plan")
    return matches[0]


def depth_visible_yolo_labels(
    fruit_rows: Sequence[Mapping[str, Any]],
    *,
    depth_image_m: np.ndarray,
    intrinsics: Sequence[float],
    image_size: tuple[int, int],
    minimum_visible_pixels: int,
    minimum_visible_fraction: float,
    depth_surface_padding_m: float,
) -> tuple[tuple[dict[str, Any], ...], tuple[dict[str, Any], ...]]:
    """Project all truth fruit and retain only depth-supported visible boxes.

    Truth supplies development labels only. The depth gate prevents a fully
    hidden projected fruit from becoming a positive training label.
    """

    depth = np.asarray(depth_image_m, dtype=np.float32)
    width, height = image_size
    if depth.shape != (height, width):
        raise ValueError("depth image dimensions differ from the capture plan")
    if minimum_visible_pixels <= 0 or not 0.0 < minimum_visible_fraction <= 1.0:
        raise ValueError("visibility thresholds are invalid")
    if not math.isfinite(depth_surface_padding_m) or depth_surface_padding_m <= 0.0:
        raise ValueError("depth surface padding must be finite and positive")

    labels: list[dict[str, Any]] = []
    excluded: list[dict[str, Any]] = []
    for row in sorted(fruit_rows, key=lambda value: int(value["target_id"])):
        target_id = int(row["target_id"])
        center = tuple(float(value) for value in row["center_camera_m"])
        radius = float(row["radius_m"])
        maturity = str(row["maturity"])
        if maturity not in {"RIPE", "UNRIPE"} or radius <= 0.0:
            raise ValueError("fruit label metadata is invalid")
        try:
            bbox = project_sphere_in_camera(center, intrinsics, image_size, radius)
        except ValueError as error:
            excluded.append({"target_id": target_id, "reason": str(error)})
            continue
        x0 = max(0, int(math.floor(bbox[0])))
        y0 = max(0, int(math.floor(bbox[1])))
        x1 = min(width, int(math.ceil(bbox[2])))
        y1 = min(height, int(math.ceil(bbox[3])))
        crop = depth[y0:y1, x0:x1]
        surface_min = center[2] - radius - depth_surface_padding_m
        surface_max = center[2] + depth_surface_padding_m
        visible = np.isfinite(crop) & (crop >= surface_min) & (crop <= surface_max)
        visible_pixels = int(np.count_nonzero(visible))
        projected_pixels = int(crop.size)
        visible_fraction = (
            float(visible_pixels) / float(projected_pixels)
            if projected_pixels
            else 0.0
        )
        if (
            visible_pixels < minimum_visible_pixels
            or visible_fraction < minimum_visible_fraction
        ):
            excluded.append(
                {
                    "target_id": target_id,
                    "reason": "insufficient_depth_supported_visibility",
                    "visible_pixels": visible_pixels,
                    "visible_fraction": visible_fraction,
                }
            )
            continue
        labels.append(
            {
                "target_id": target_id,
                "maturity": maturity,
                "class_id": 0 if maturity == "RIPE" else 1,
                "bbox_xyxy": list(bbox),
                "yolo_xywh": list(xyxy_to_yolo(bbox, image_size)),
                "center_camera_m": list(center),
                "radius_m": radius,
                "visible_pixels": visible_pixels,
                "visible_fraction": visible_fraction,
            }
        )
    return tuple(labels), tuple(excluded)


def validate_visibility_partition(
    fruit_rows: Sequence[Mapping[str, Any]],
    labels: Sequence[Mapping[str, Any]],
    excluded: Sequence[Mapping[str, Any]],
) -> bool:
    """Prove every truth target is labeled or explicitly visibility-excluded.

    Returns ``True`` for a legitimate negative image containing no visible
    labels. Duplicate, missing, or unexpected identities fail closed.
    """

    expected = [int(row["target_id"]) for row in fruit_rows]
    labeled = [int(row["target_id"]) for row in labels]
    hidden = [int(row["target_id"]) for row in excluded]
    if len(expected) != len(set(expected)):
        raise ValueError("truth target IDs are not unique")
    if len(labeled) != len(set(labeled)) or len(hidden) != len(set(hidden)):
        raise ValueError("visibility partition contains duplicate target IDs")
    if set(labeled) & set(hidden):
        raise ValueError("a target cannot be both visible and excluded")
    if set(labeled) | set(hidden) != set(expected):
        raise ValueError("visibility partition does not cover every truth target")
    return not labeled
