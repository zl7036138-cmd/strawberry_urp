"""Dependency-light geometry and statistics for the localization gate."""

from __future__ import annotations

import math
from typing import Iterable, Sequence

from .core import BoundingBox, CameraIntrinsics, LocalizationError


# The T40 accuracy gate intentionally samples the camera-clear calibration
# volume. Panda/fruit occlusion is varied later by the T70 robustness matrix.
DEFAULT_X_POSITIONS_M = (0.44, 0.465, 0.49, 0.515, 0.54)
DEFAULT_Y_POSITIONS_M = (-0.10, -0.075, -0.05, -0.025, 0.00)
DEFAULT_Z_POSITIONS_M = (0.46, 0.48, 0.50, 0.52)


def benchmark_positions(
    x_values: Sequence[float] = DEFAULT_X_POSITIONS_M,
    y_values: Sequence[float] = DEFAULT_Y_POSITIONS_M,
    z_values: Sequence[float] = DEFAULT_Z_POSITIONS_M,
) -> tuple[tuple[float, float, float], ...]:
    """Return a stable Cartesian grid containing 100 distinct truth poses."""

    if not x_values or not y_values or not z_values:
        raise ValueError("localization position axes must be non-empty")
    positions = tuple(
        (float(x), float(y), float(z))
        for z in z_values
        for y in y_values
        for x in x_values
    )
    if not all(math.isfinite(value) for point in positions for value in point):
        raise ValueError("localization positions must be finite")
    if len(set(positions)) != len(positions):
        raise ValueError("localization positions must be distinct")
    return positions


def sphere_projection_bbox(
    point_camera_xyz: Sequence[float],
    intrinsics: CameraIntrinsics,
    *,
    sphere_radius_m: float,
    image_width: int,
    image_height: int,
    padding: float = 1.05,
) -> BoundingBox:
    """Project a known rigid sphere to an oracle detector bounding box.

    The gate uses this box in place of YOLO so the depth/TF measurement can be
    evaluated independently.  A small padding absorbs pixel quantisation while
    the production central crop remains entirely on the fruit surface.
    """

    if len(point_camera_xyz) != 3:
        raise LocalizationError("camera point must contain x, y, and z")
    x, y, z = (float(value) for value in point_camera_xyz)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise LocalizationError("camera point must be finite")
    if not math.isfinite(sphere_radius_m) or sphere_radius_m <= 0.0:
        raise ValueError("sphere radius must be finite and positive")
    if not math.isfinite(padding) or padding < 1.0:
        raise ValueError("bounding-box padding must be finite and at least one")
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")
    if z <= sphere_radius_m:
        raise LocalizationError("sphere is behind or intersects the optical origin")

    center_u = intrinsics.fx * x / z + intrinsics.cx
    center_v = intrinsics.fy * y / z + intrinsics.cy
    tangent_depth = math.sqrt(z * z - sphere_radius_m * sphere_radius_m)
    radius_u = padding * intrinsics.fx * sphere_radius_m / tangent_depth
    radius_v = padding * intrinsics.fy * sphere_radius_m / tangent_depth

    x0 = max(0, int(math.floor(center_u - radius_u)))
    y0 = max(0, int(math.floor(center_v - radius_v)))
    x1 = min(image_width, int(math.ceil(center_u + radius_u)))
    y1 = min(image_height, int(math.ceil(center_v + radius_v)))
    if x0 >= x1 or y0 >= y1:
        raise LocalizationError("projected fruit does not overlap the image")
    if not (0.0 <= center_u < image_width and 0.0 <= center_v < image_height):
        raise LocalizationError("projected fruit centre is outside the image")
    return BoundingBox(x=x0, y=y0, width=x1 - x0, height=y1 - y0)


def linear_quantile(values: Iterable[float], probability: float) -> float:
    """Calculate the deterministic type-7 quantile used by NumPy defaults."""

    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise ValueError("cannot calculate a quantile of an empty sequence")
    if not all(math.isfinite(value) and value >= 0.0 for value in ordered):
        raise ValueError("localization errors must be finite and non-negative")
    if not 0.0 <= probability <= 1.0:
        raise ValueError("quantile probability must be in [0, 1]")
    rank = (len(ordered) - 1) * probability
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize_gate(
    errors_mm: Sequence[float],
    *,
    requested_positions: int,
    minimum_positions: int = 100,
    median_limit_mm: float = 15.0,
    p95_limit_mm: float = 30.0,
) -> dict[str, object]:
    """Build the machine-readable T40 acceptance summary."""

    if requested_positions <= 0 or minimum_positions <= 0:
        raise ValueError("position counts must be positive")
    valid = tuple(float(value) for value in errors_mm)
    median = linear_quantile(valid, 0.5) if valid else None
    p95 = linear_quantile(valid, 0.95) if valid else None
    enough_requested = requested_positions >= minimum_positions
    all_measured = len(valid) == requested_positions
    passed = bool(
        enough_requested
        and all_measured
        and median is not None
        and p95 is not None
        and median <= median_limit_mm
        and p95 <= p95_limit_mm
    )
    return {
        "requested_positions": requested_positions,
        "minimum_positions": minimum_positions,
        "valid_measurements": len(valid),
        "failed_positions": requested_positions - len(valid),
        "median_error_mm": median,
        "p95_error_mm": p95,
        "median_limit_mm": median_limit_mm,
        "p95_limit_mm": p95_limit_mm,
        "passed": passed,
    }
