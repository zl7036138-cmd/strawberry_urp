#!/usr/bin/env python3
"""Compare both depth estimators on identical recorded RGB-D observations.

The first invocation can build a compact, depth-only evidence bundle from one
or more synchronized capture NPZ files.  Later invocations need only that
bundle, so the comparison remains reproducible without ROS, Gazebo, or motion.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "strawberry_localization"
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_localization.core import (  # noqa: E402
    BoundingBox,
    CameraIntrinsics,
    LocalizationError,
    localize_bbox,
    robust_center_depth,
)


TARGET_RADIUS_M = 0.026
BUNDLE_MARGIN_FRACTION = 0.15
EXPECTED_SAMPLE_COUNT = 40
MAXIMUM_GEOMETRY_ERROR_MM = 30.0
MAXIMUM_CLEAR_MEDIAN_REGRESSION_MM = 2.0
PLANNING_SIGMA_LIMIT_M = 0.015

CONDITIONS = (
    {
        "id": "observed_bbox",
        "bbox_scale": 1.0,
        "occlusion_fraction": 0.0,
        "foreground_offset_m": 0.0,
        "positive_observation": True,
    },
    {
        "id": "bbox_shrink_10pct",
        "bbox_scale": 0.9,
        "occlusion_fraction": 0.0,
        "foreground_offset_m": 0.0,
        "positive_observation": True,
    },
    {
        "id": "bbox_expand_10pct",
        "bbox_scale": 1.1,
        "occlusion_fraction": 0.0,
        "foreground_offset_m": 0.0,
        "positive_observation": True,
    },
    {
        "id": "center_occlusion_25pct",
        "bbox_scale": 1.0,
        "occlusion_fraction": 0.25,
        "foreground_offset_m": 0.04,
        "positive_observation": True,
    },
    {
        "id": "center_occlusion_45pct",
        "bbox_scale": 1.0,
        "occlusion_fraction": 0.45,
        "foreground_offset_m": 0.04,
        "positive_observation": True,
    },
    {
        "id": "fruit_absent_100pct",
        "bbox_scale": 1.0,
        "occlusion_fraction": 1.0,
        "foreground_offset_m": 0.10,
        "positive_observation": False,
    },
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _relative_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def _fingerprint(path: Path) -> dict[str, Any]:
    return {
        "path": _relative_path(path),
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _read_json_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _validate_capture(
    archive: Any,
    *,
    source_path: Path,
) -> int:
    required = {
        "depth_m",
        "roi_xywh",
        "intrinsics_fx_fy_cx_cy",
        "base_from_camera_translation_m",
        "base_from_camera_rotation_xyzw",
        "truth_target_xyz_m",
        "stamp_sec_nanosec",
    }
    missing = sorted(required - set(archive.files))
    if missing:
        raise ValueError(f"{source_path} is missing arrays: {missing}")
    count = int(archive["depth_m"].shape[0])
    expected_shapes = {
        "roi_xywh": (count, 4),
        "intrinsics_fx_fy_cx_cy": (count, 4),
        "base_from_camera_translation_m": (count, 3),
        "base_from_camera_rotation_xyzw": (count, 4),
        "truth_target_xyz_m": (count, 3),
        "stamp_sec_nanosec": (count, 2),
    }
    if archive["depth_m"].ndim != 3 or count <= 0:
        raise ValueError(f"{source_path} must contain one or more depth images")
    for name, shape in expected_shapes.items():
        if archive[name].shape != shape:
            raise ValueError(
                f"{source_path}:{name} has shape {archive[name].shape}; "
                f"expected {shape}"
            )
    return count


def build_depth_bundle(
    source_npz_paths: Iterable[Path],
    source_receipt_paths: Iterable[Path],
    output_path: Path,
) -> dict[str, Any]:
    """Create a compact depth-only bundle while retaining source bindings."""

    npz_paths = tuple(path.resolve() for path in source_npz_paths)
    receipt_paths = tuple(path.resolve() for path in source_receipt_paths)
    if not npz_paths or len(npz_paths) != len(receipt_paths):
        raise ValueError("source NPZ and receipt lists must be non-empty and paired")
    output_path = output_path.resolve()
    if output_path.exists():
        raise FileExistsError(f"refusing to overwrite bundle: {output_path}")

    arrays: dict[str, np.ndarray] = {}
    boxes = []
    intrinsics = []
    translations = []
    rotations = []
    truths = []
    stamps = []
    source_indices = []
    source_bindings = []
    sample_index = 0

    for source_index, (npz_path, receipt_path) in enumerate(
        zip(npz_paths, receipt_paths, strict=True)
    ):
        receipt = _read_json_object(receipt_path)
        archive_fingerprint = _fingerprint(npz_path)
        declared = receipt.get("dataset", {})
        if (
            declared.get("sha256") != archive_fingerprint["sha256"]
            or int(declared.get("size_bytes", -1))
            != archive_fingerprint["size_bytes"]
        ):
            raise ValueError(f"capture receipt does not bind {npz_path}")
        if receipt.get("control_commands_sent") != 0:
            raise ValueError(f"capture receipt authorizes control: {receipt_path}")
        if receipt.get("pick_action_started") is not False:
            raise ValueError(f"capture receipt started a pick: {receipt_path}")

        with np.load(npz_path, allow_pickle=False) as archive:
            count = _validate_capture(archive, source_path=npz_path)
            if int(receipt.get("sample_count", -1)) != count:
                raise ValueError(f"capture count differs from receipt: {npz_path}")
            for local_index in range(count):
                depth = np.asarray(archive["depth_m"][local_index], dtype=np.float32)
                x, y, width, height = (
                    int(value) for value in archive["roi_xywh"][local_index]
                )
                box = BoundingBox(x, y, width, height)
                margin_x = int(math.ceil(width * BUNDLE_MARGIN_FRACTION))
                margin_y = int(math.ceil(height * BUNDLE_MARGIN_FRACTION))
                crop_x0 = max(0, box.x - margin_x)
                crop_y0 = max(0, box.y - margin_y)
                crop_x1 = min(depth.shape[1], box.x + box.width + margin_x)
                crop_y1 = min(depth.shape[0], box.y + box.height + margin_y)
                cropped = depth[crop_y0:crop_y1, crop_x0:crop_x1].copy()
                arrays[f"depth_{sample_index:03d}"] = cropped
                boxes.append(
                    [box.x - crop_x0, box.y - crop_y0, box.width, box.height]
                )
                fx, fy, cx, cy = (
                    float(value)
                    for value in archive["intrinsics_fx_fy_cx_cy"][local_index]
                )
                intrinsics.append([fx, fy, cx - crop_x0, cy - crop_y0])
                translations.append(
                    archive["base_from_camera_translation_m"][local_index]
                )
                rotations.append(
                    archive["base_from_camera_rotation_xyzw"][local_index]
                )
                truths.append(archive["truth_target_xyz_m"][local_index])
                stamps.append(archive["stamp_sec_nanosec"][local_index])
                source_indices.append(source_index)
                sample_index += 1

        source_bindings.append(
            {
                "capture": archive_fingerprint,
                "receipt": _fingerprint(receipt_path),
                "sample_count": count,
                "scope": receipt.get("scope"),
                "truth_use": receipt.get("truth_use"),
            }
        )

    metadata = {
        "schema_version": 1,
        "kind": "paired_rgbd_depth_only_bundle",
        "sample_count": sample_index,
        "source_count": len(source_bindings),
        "crop_margin_fraction": BUNDLE_MARGIN_FRACTION,
        "source_bindings": source_bindings,
        "excluded_arrays": ["color_bgr", "confidence"],
        "scope": {
            "recorded_simulator_depth": True,
            "paired_estimator_input": True,
            "robot_motion_during_evaluation": False,
            "formal_acceptance": False,
        },
    }
    arrays.update(
        {
            "bbox_xywh": np.asarray(boxes, dtype=np.int32),
            "intrinsics_fx_fy_cx_cy": np.asarray(intrinsics, dtype=np.float64),
            "base_from_camera_translation_m": np.asarray(
                translations, dtype=np.float64
            ),
            "base_from_camera_rotation_xyzw": np.asarray(
                rotations, dtype=np.float64
            ),
            "truth_target_xyz_m": np.asarray(truths, dtype=np.float64),
            "stamp_sec_nanosec": np.asarray(stamps, dtype=np.int64),
            "source_index": np.asarray(source_indices, dtype=np.int16),
            "metadata_json_utf8": np.frombuffer(
                json.dumps(
                    metadata,
                    ensure_ascii=False,
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8"),
                dtype=np.uint8,
            ),
        }
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output_path, **arrays)
    return metadata


def _scaled_box(
    box: BoundingBox,
    scale: float,
    *,
    image_width: int,
    image_height: int,
) -> BoundingBox:
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("bounding-box scale must be finite and positive")
    center_x = box.x + 0.5 * box.width
    center_y = box.y + 0.5 * box.height
    width = box.width * scale
    height = box.height * scale
    x0 = max(0, int(math.floor(center_x - 0.5 * width)))
    y0 = max(0, int(math.floor(center_y - 0.5 * height)))
    x1 = min(image_width, int(math.ceil(center_x + 0.5 * width)))
    y1 = min(image_height, int(math.ceil(center_y + 0.5 * height)))
    return BoundingBox(x0, y0, x1 - x0, y1 - y0)


def _inject_foreground(
    depth_image_m: np.ndarray,
    box: BoundingBox,
    *,
    width_fraction: float,
    foreground_depth_m: float,
) -> np.ndarray:
    if not 0.0 <= width_fraction <= 1.0:
        raise ValueError("occluder width fraction must be in [0, 1]")
    if not math.isfinite(foreground_depth_m) or foreground_depth_m <= 0.0:
        raise ValueError("foreground depth must be finite and positive")
    modified = np.asarray(depth_image_m, dtype=np.float32).copy()
    if width_fraction == 0.0:
        return modified
    band_width = max(1, int(math.ceil(box.width * width_fraction)))
    center_x = box.x + 0.5 * box.width
    x0 = max(box.x, int(math.floor(center_x - 0.5 * band_width)))
    x1 = min(box.x + box.width, x0 + band_width)
    modified[box.y : box.y + box.height, x0:x1] = foreground_depth_m
    return modified


def _rotate_vector_by_quaternion(
    vector: np.ndarray,
    quaternion_xyzw: np.ndarray,
) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("camera transform quaternion must be finite and non-zero")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    rotation = np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )
    return rotation @ np.asarray(vector, dtype=np.float64)


def _percentile(values: list[float], probability: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(values, probability, method="linear"))


def _summarize_records(records: list[dict[str, Any]]) -> dict[str, Any]:
    accepted = [record for record in records if record["status"] == "ESTIMATED"]
    errors = [float(record["error_mm"]) for record in accepted]
    sigmas = [float(record["sigma_m"]) for record in accepted]
    foreground = [
        bool(record["selected_foreground"])
        for record in accepted
        if record.get("selected_foreground") is not None
    ]
    return {
        "requested_samples": len(records),
        "accepted_samples": len(accepted),
        "rejected_samples": len(records) - len(accepted),
        "acceptance_rate": len(accepted) / len(records) if records else 0.0,
        "median_error_mm": _percentile(errors, 50.0),
        "p95_error_mm": _percentile(errors, 95.0),
        "maximum_error_mm": max(errors) if errors else None,
        "median_sigma_m": _percentile(sigmas, 50.0),
        "p95_sigma_m": _percentile(sigmas, 95.0),
        "sigma_under_15mm_rate": (
            sum(value <= PLANNING_SIGMA_LIMIT_M for value in sigmas) / len(sigmas)
            if sigmas
            else 0.0
        ),
        "foreground_selection_rate": (
            sum(foreground) / len(foreground) if foreground else None
        ),
    }


def evaluate_bundle(bundle_path: Path) -> dict[str, Any]:
    """Evaluate all frozen conditions against both estimators."""

    bundle_path = bundle_path.resolve()
    with np.load(bundle_path, allow_pickle=False) as bundle:
        metadata = json.loads(
            bytes(bundle["metadata_json_utf8"].tolist()).decode("utf-8")
        )
        sample_count = int(metadata["sample_count"])
        required_shapes = {
            "bbox_xywh": (sample_count, 4),
            "intrinsics_fx_fy_cx_cy": (sample_count, 4),
            "base_from_camera_translation_m": (sample_count, 3),
            "base_from_camera_rotation_xyzw": (sample_count, 4),
            "truth_target_xyz_m": (sample_count, 3),
            "stamp_sec_nanosec": (sample_count, 2),
            "source_index": (sample_count,),
        }
        for name, shape in required_shapes.items():
            if name not in bundle or bundle[name].shape != shape:
                raise ValueError(f"bundle array {name} must have shape {shape}")

        condition_records: dict[str, dict[str, list[dict[str, Any]]]] = {
            str(condition["id"]): {
                "center_median": [],
                "geometry_layer": [],
            }
            for condition in CONDITIONS
        }
        for sample_index in range(sample_count):
            depth_key = f"depth_{sample_index:03d}"
            if depth_key not in bundle:
                raise ValueError(f"bundle is missing {depth_key}")
            original_depth = np.asarray(bundle[depth_key], dtype=np.float32)
            original_box = BoundingBox(
                *(int(value) for value in bundle["bbox_xywh"][sample_index])
            )
            intrinsics = CameraIntrinsics(
                *(float(value)
                  for value in bundle["intrinsics_fx_fy_cx_cy"][sample_index])
            )
            observed_surface_depth = robust_center_depth(
                original_depth,
                original_box,
            ).depth_m
            translation = np.asarray(
                bundle["base_from_camera_translation_m"][sample_index],
                dtype=np.float64,
            )
            rotation = np.asarray(
                bundle["base_from_camera_rotation_xyzw"][sample_index],
                dtype=np.float64,
            )
            truth = np.asarray(
                bundle["truth_target_xyz_m"][sample_index],
                dtype=np.float64,
            )
            stamp = [
                int(value)
                for value in bundle["stamp_sec_nanosec"][sample_index]
            ]

            for condition in CONDITIONS:
                condition_id = str(condition["id"])
                box = _scaled_box(
                    original_box,
                    float(condition["bbox_scale"]),
                    image_width=original_depth.shape[1],
                    image_height=original_depth.shape[0],
                )
                offset_m = float(condition["foreground_offset_m"])
                foreground_depth_m = (
                    max(0.055, observed_surface_depth - offset_m)
                    if offset_m > 0.0
                    else None
                )
                depth = _inject_foreground(
                    original_depth,
                    box,
                    width_fraction=float(condition["occlusion_fraction"]),
                    foreground_depth_m=(
                        foreground_depth_m
                        if foreground_depth_m is not None
                        else observed_surface_depth
                    ),
                )
                for mode in ("center_median", "geometry_layer"):
                    record: dict[str, Any] = {
                        "sample_index": sample_index,
                        "source_index": int(bundle["source_index"][sample_index]),
                        "stamp_sec_nanosec": stamp,
                        "status": "REJECTED",
                        "foreground_depth_m": foreground_depth_m,
                    }
                    try:
                        camera_point, estimate = localize_bbox(
                            depth,
                            box,
                            intrinsics,
                            surface_to_center_offset_m=TARGET_RADIUS_M,
                            depth_estimator_mode=mode,
                        )
                    except (LocalizationError, TypeError, ValueError) as exc:
                        record["reason"] = str(exc)
                    else:
                        base_point = (
                            _rotate_vector_by_quaternion(camera_point, rotation)
                            + translation
                        )
                        record.update(
                            {
                                "status": "ESTIMATED",
                                "error_mm": float(
                                    np.linalg.norm(base_point - truth) * 1000.0
                                ),
                                "depth_m": float(estimate.depth_m),
                                "sigma_m": float(estimate.sigma_m),
                                "valid_pixels": int(estimate.valid_pixels),
                                "selected_foreground": (
                                    abs(estimate.depth_m - foreground_depth_m)
                                    < abs(
                                        estimate.depth_m - observed_surface_depth
                                    )
                                    if foreground_depth_m is not None
                                    else None
                                ),
                            }
                        )
                    condition_records[condition_id][mode].append(record)

    summaries = {
        condition_id: {
            mode: _summarize_records(records)
            for mode, records in estimators.items()
        }
        for condition_id, estimators in condition_records.items()
    }
    positive_condition_ids = [
        str(condition["id"])
        for condition in CONDITIONS
        if condition["positive_observation"]
    ]
    observed = summaries["observed_bbox"]
    diagnostic_checks = {
        "two_source_captures": int(metadata["source_count"]) == 2,
        "forty_paired_samples": sample_count == EXPECTED_SAMPLE_COUNT,
        "observed_both_accept_all": all(
            observed[mode]["accepted_samples"] == sample_count
            for mode in ("center_median", "geometry_layer")
        ),
        "geometry_accepts_all_positive_conditions": all(
            summaries[condition_id]["geometry_layer"]["accepted_samples"]
            == sample_count
            for condition_id in positive_condition_ids
        ),
        "geometry_positive_p95_within_30mm": all(
            summaries[condition_id]["geometry_layer"]["p95_error_mm"]
            is not None
            and summaries[condition_id]["geometry_layer"]["p95_error_mm"]
            <= MAXIMUM_GEOMETRY_ERROR_MM
            for condition_id in positive_condition_ids
        ),
        "geometry_clear_median_not_regressed": (
            observed["geometry_layer"]["median_error_mm"]
            <= observed["center_median"]["median_error_mm"]
            + MAXIMUM_CLEAR_MEDIAN_REGRESSION_MM
        ),
        "geometry_rejects_absent_fruit": (
            summaries["fruit_absent_100pct"]["geometry_layer"]
            ["rejected_samples"]
            == sample_count
        ),
        "legacy_center_failure_is_exposed": (
            summaries["center_occlusion_45pct"]["center_median"]
            ["foreground_selection_rate"]
            == 1.0
        ),
    }
    planning_sigma_rates = [
        summaries[condition_id]["geometry_layer"]["sigma_under_15mm_rate"]
        for condition_id in positive_condition_ids
    ]
    promotion_checks = {
        "diagnostic_passed": all(diagnostic_checks.values()),
        "at_least_five_physical_viewpoints": False,
        "controlled_occlusion_rendered_in_simulator": False,
        "geometry_sigma_under_15mm_for_all_positive_samples": all(
            rate == 1.0 for rate in planning_sigma_rates
        ),
    }
    return {
        "schema_version": 1,
        "kind": "paired_rgbd_depth_estimator_matrix",
        "diagnostic_status": (
            "PASS" if all(diagnostic_checks.values()) else "FAIL"
        ),
        "runtime_promotion_status": (
            "ELIGIBLE" if all(promotion_checks.values()) else "BLOCKED"
        ),
        "diagnostic_checks": diagnostic_checks,
        "promotion_checks": promotion_checks,
        "thresholds": {
            "expected_sample_count": EXPECTED_SAMPLE_COUNT,
            "maximum_geometry_error_mm": MAXIMUM_GEOMETRY_ERROR_MM,
            "maximum_clear_median_regression_mm": (
                MAXIMUM_CLEAR_MEDIAN_REGRESSION_MM
            ),
            "planning_sigma_limit_m": PLANNING_SIGMA_LIMIT_M,
        },
        "conditions": summaries,
        "records": condition_records,
        "bundle": _fingerprint(bundle_path),
        "bundle_metadata": metadata,
        "implementation": {
            "evaluator": _fingerprint(Path(__file__)),
            "localization_core": _fingerprint(
                PACKAGE_ROOT / "strawberry_localization" / "core.py"
            ),
        },
        "scope": {
            "identical_frames_for_both_estimators": True,
            "recorded_rgbd_depth": True,
            "synthetic_depth_occlusion": True,
            "physical_viewpoint_count": 2,
            "robot_motion_during_evaluation": False,
            "gazebo_runtime_during_evaluation": False,
            "formal_p3_or_p4_evidence": False,
            "runtime_promotion_authorized": False,
            "held_out_real_test_consumed": False,
        },
    }


def _write_json_lf(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(
            json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
            + "\n"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path)
    parser.add_argument("--source-npz", action="append", type=Path, default=[])
    parser.add_argument(
        "--source-receipt", action="append", type=Path, default=[]
    )
    parser.add_argument("--bundle-output", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args()

    output = options.output.resolve()
    if output.exists():
        raise SystemExit(f"refusing to overwrite receipt: {output}")
    if options.bundle is not None:
        if options.source_npz or options.source_receipt or options.bundle_output:
            raise SystemExit("--bundle cannot be combined with source arguments")
        bundle = options.bundle.resolve()
    else:
        if (
            not options.source_npz
            or len(options.source_npz) != len(options.source_receipt)
            or options.bundle_output is None
        ):
            raise SystemExit(
                "paired --source-npz/--source-receipt and --bundle-output "
                "are required when --bundle is omitted"
            )
        bundle = options.bundle_output.resolve()
        build_depth_bundle(options.source_npz, options.source_receipt, bundle)

    result = evaluate_bundle(bundle)
    _write_json_lf(output, result)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result["diagnostic_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
