"""Dependency-light geometry and accounting for shadow perception forensics."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class ProjectedTruth:
    """One ground-truth fruit projected into the RGB image."""

    target_id: int
    maturity: str
    bbox_xyxy: tuple[float, float, float, float]
    center_uv: tuple[float, float]
    depth_m: float


def bgr_from_rgb(rgb: np.ndarray) -> np.ndarray:
    """Return the BGR ndarray required by Ultralytics' NumPy source path."""

    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("RGB image must have shape HxWx3")
    if image.dtype != np.uint8:
        raise ValueError("RGB image must use uint8 channels")
    return np.ascontiguousarray(image[..., ::-1])


def quaternion_rotation_matrix(
    quaternion_xyzw: Sequence[float],
) -> np.ndarray:
    """Convert a finite, non-zero xyzw quaternion to a 3x3 matrix."""

    if len(quaternion_xyzw) != 4:
        raise ValueError("quaternion must contain x, y, z, w")
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion must be finite")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0:
        raise ValueError("quaternion must be non-zero")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def project_sphere(
    *,
    target_id: int,
    maturity: str,
    center_in_source_m: Sequence[float],
    source_to_camera_translation_m: Sequence[float],
    source_to_camera_quaternion_xyzw: Sequence[float],
    intrinsics: Sequence[float],
    image_size: tuple[int, int],
    radius_m: float = 0.035,
) -> ProjectedTruth | None:
    """Project an approximate fruit sphere into the camera optical frame."""

    if len(center_in_source_m) != 3 or len(source_to_camera_translation_m) != 3:
        raise ValueError("points and translations must contain three values")
    if len(intrinsics) != 4:
        raise ValueError("intrinsics must contain fx, fy, cx, cy")
    width, height = image_size
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    if not math.isfinite(radius_m) or radius_m <= 0.0:
        raise ValueError("radius_m must be positive and finite")

    center = np.asarray(center_in_source_m, dtype=np.float64)
    translation = np.asarray(source_to_camera_translation_m, dtype=np.float64)
    if not np.all(np.isfinite(center)) or not np.all(np.isfinite(translation)):
        raise ValueError("points and translations must be finite")
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        raise ValueError("intrinsics must be finite")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("focal lengths must be positive")

    rotation = quaternion_rotation_matrix(source_to_camera_quaternion_xyzw)
    camera = rotation @ center + translation
    x_m, y_m, z_m = (float(value) for value in camera)
    if z_m <= 0.0:
        return None
    u = fx * x_m / z_m + cx
    v = fy * y_m / z_m + cy
    radius_u = fx * radius_m / z_m
    radius_v = fy * radius_m / z_m
    x1 = max(0.0, u - radius_u)
    y1 = max(0.0, v - radius_v)
    x2 = min(float(width), u + radius_u)
    y2 = min(float(height), v + radius_v)
    if x2 <= x1 or y2 <= y1:
        return None
    return ProjectedTruth(
        target_id=int(target_id),
        maturity=str(maturity).upper(),
        bbox_xyxy=(x1, y1, x2, y2),
        center_uv=(u, v),
        depth_m=z_m,
    )


def box_iou(
    left: Sequence[float], right: Sequence[float]
) -> float:
    """Return intersection-over-union for two xyxy boxes."""

    if len(left) != 4 or len(right) != 4:
        raise ValueError("boxes must contain x1, y1, x2, y2")
    lx1, ly1, lx2, ly2 = (float(value) for value in left)
    rx1, ry1, rx2, ry2 = (float(value) for value in right)
    if lx2 <= lx1 or ly2 <= ly1 or rx2 <= rx1 or ry2 <= ry1:
        raise ValueError("boxes must have positive area")
    intersection_width = max(0.0, min(lx2, rx2) - max(lx1, rx1))
    intersection_height = max(0.0, min(ly2, ry2) - max(ly1, ry1))
    intersection = intersection_width * intersection_height
    union = (lx2 - lx1) * (ly2 - ly1) + (rx2 - rx1) * (ry2 - ry1) - intersection
    return intersection / union


def associate_rows_to_truth(
    rows: Iterable[Sequence[float]],
    class_names: Mapping[int, str] | Sequence[str],
    projections: Sequence[ProjectedTruth],
) -> list[dict]:
    """Associate each YOLO row to the projected fruit with maximum IoU."""

    associations = []
    for row in rows:
        values = list(row)
        if len(values) != 6:
            raise ValueError("YOLO rows must contain xyxy, confidence, class id")
        box = tuple(float(value) for value in values[:4])
        confidence = float(values[4])
        class_id = int(values[5])
        class_name = (
            class_names[class_id]
            if not isinstance(class_names, Mapping)
            else class_names[class_id]
        )
        overlaps = [box_iou(box, truth.bbox_xyxy) for truth in projections]
        best_index = int(np.argmax(overlaps)) if overlaps else None
        best_iou = overlaps[best_index] if best_index is not None else 0.0
        truth = projections[best_index] if best_index is not None and best_iou > 0.0 else None
        associations.append(
            {
                "bbox_xyxy": list(box),
                "confidence": confidence,
                "class_id": class_id,
                "class_name": str(class_name),
                "associated_target_id": None if truth is None else truth.target_id,
                "associated_truth_maturity": None if truth is None else truth.maturity,
                "association_iou": best_iou,
            }
        )
    return associations


def summarize_modes(frame_records: Sequence[Mapping[str, object]]) -> dict:
    """Aggregate class and truth-association evidence from diagnostic frames."""

    summary: dict[str, dict] = {}
    for mode in ("legacy_rgb", "correct_bgr"):
        detections = [
            detection
            for frame in frame_records
            for detection in frame[mode]  # type: ignore[index,union-attr]
        ]
        by_class: dict[str, int] = {}
        by_truth: dict[str, int] = {}
        unassociated = 0
        for detection in detections:
            class_name = str(detection["class_name"])
            by_class[class_name] = by_class.get(class_name, 0) + 1
            truth_maturity = detection["associated_truth_maturity"]
            if truth_maturity is None:
                unassociated += 1
            else:
                key = f"{truth_maturity}->{class_name}"
                by_truth[key] = by_truth.get(key, 0) + 1
        summary[mode] = {
            "detection_count": len(detections),
            "by_predicted_class": dict(sorted(by_class.items())),
            "by_associated_truth_and_prediction": dict(sorted(by_truth.items())),
            "unassociated_detection_count": unassociated,
        }
    return summary
