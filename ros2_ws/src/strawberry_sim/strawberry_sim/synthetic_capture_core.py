"""Dependency-light geometry and validation helpers for synthetic capture."""

from __future__ import annotations

import math
from typing import Sequence


def project_sphere_in_camera(
    center_xyz_m: Sequence[float],
    intrinsics: Sequence[float],
    image_size: tuple[int, int],
    radius_m: float,
) -> tuple[float, float, float, float]:
    """Project an approximate sphere to a clipped xyxy pixel box."""

    if len(center_xyz_m) != 3 or len(intrinsics) != 4:
        raise ValueError("camera point and intrinsics have invalid dimensions")
    x_m, y_m, z_m = (float(value) for value in center_xyz_m)
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    width, height = (int(value) for value in image_size)
    values = (x_m, y_m, z_m, fx, fy, cx, cy, float(radius_m))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("projection inputs must be finite")
    if z_m <= radius_m or fx <= 0.0 or fy <= 0.0 or radius_m <= 0.0:
        raise ValueError("projection depth, focal lengths, and radius must be positive")
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    u = cx + fx * x_m / z_m
    v = cy + fy * y_m / z_m
    radius_u = fx * radius_m / z_m
    radius_v = fy * radius_m / z_m
    box = (
        max(0.0, u - radius_u),
        max(0.0, v - radius_v),
        min(float(width), u + radius_u),
        min(float(height), v + radius_v),
    )
    if box[2] <= box[0] or box[3] <= box[1]:
        raise ValueError("projected sphere does not intersect the image")
    return box


def xyxy_to_yolo(
    bbox_xyxy: Sequence[float], image_size: tuple[int, int]
) -> tuple[float, float, float, float]:
    """Convert a clipped pixel xyxy box to normalized YOLO xywh."""

    if len(bbox_xyxy) != 4:
        raise ValueError("bbox must contain four values")
    width, height = (int(value) for value in image_size)
    x1, y1, x2, y2 = (float(value) for value in bbox_xyxy)
    if width <= 0 or height <= 0 or not all(
        math.isfinite(value) for value in (x1, y1, x2, y2)
    ):
        raise ValueError("image size and bbox must be valid")
    if not (0.0 <= x1 < x2 <= width and 0.0 <= y1 < y2 <= height):
        raise ValueError("bbox must be clipped inside the image")
    result = (
        (x1 + x2) / (2.0 * width),
        (y1 + y2) / (2.0 * height),
        (x2 - x1) / width,
        (y2 - y1) / height,
    )
    if not all(0.0 < value <= 1.0 for value in result):
        raise ValueError("normalized YOLO box is invalid")
    return result


def parse_yolo_label(text: str) -> tuple[int, tuple[float, float, float, float]]:
    """Parse exactly one two-class YOLO label row."""

    rows = [row.strip() for row in text.splitlines() if row.strip()]
    if len(rows) != 1:
        raise ValueError("synthetic label must contain exactly one row")
    parts = rows[0].split()
    if len(parts) != 5:
        raise ValueError("YOLO label row must contain five fields")
    class_id = int(parts[0])
    box = tuple(float(value) for value in parts[1:])
    if class_id not in (0, 1):
        raise ValueError("synthetic class ID must be 0 or 1")
    if not all(math.isfinite(value) and 0.0 < value <= 1.0 for value in box):
        raise ValueError("YOLO label values must be finite and normalized")
    x, y, width, height = box
    if x - width / 2.0 < -1e-9 or x + width / 2.0 > 1.0 + 1e-9:
        raise ValueError("YOLO label exceeds horizontal image bounds")
    if y - height / 2.0 < -1e-9 or y + height / 2.0 > 1.0 + 1e-9:
        raise ValueError("YOLO label exceeds vertical image bounds")
    return class_id, box  # type: ignore[return-value]
