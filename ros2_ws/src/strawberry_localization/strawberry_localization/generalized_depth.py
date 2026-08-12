"""Depth disambiguation used only by the generalized multi-target runtime."""

from __future__ import annotations

import math

import numpy as np

from .core import BoundingBox, DepthEstimate, LocalizationError


def calibrate_runtime_geometry_uncertainty(
    estimate: DepthEstimate,
    *,
    weight: float,
    minimum_sigma_m: float = 0.0,
) -> DepthEstimate:
    """Calibrate geometry uncertainty for the generalized runtime only.

    The frozen localization core deliberately reports the full apparent-size
    mismatch as uncertainty. Eye-in-hand detections have quantized boxes, so
    the generalized runtime applies a separately tested, bounded calibration
    after all depth-layer rejection gates have already passed.
    """

    if not math.isfinite(weight) or not 0.0 < weight <= 1.0:
        raise ValueError("runtime geometry uncertainty weight must be in (0, 1]")
    if not math.isfinite(minimum_sigma_m) or minimum_sigma_m < 0.0:
        raise ValueError("minimum runtime geometry sigma must be finite and non-negative")
    return DepthEstimate(
        depth_m=float(estimate.depth_m),
        sigma_m=max(minimum_sigma_m, weight * float(estimate.sigma_m)),
        valid_pixels=int(estimate.valid_pixels),
        center_u=float(estimate.center_u),
        center_v=float(estimate.center_v),
    )


def retain_support_ranked_geometry_layer(
    depth_image_m: np.ndarray,
    box: BoundingBox,
    *,
    fx: float,
    fy: float,
    target_radius_m: float,
    search_fraction: float,
    min_depth_m: float,
    max_depth_m: float,
    min_layer_pixels: int,
    min_layer_fraction: float,
    layer_gap_m: float,
    expected_depth_tolerance_m: float,
    ambiguity_margin_m: float,
    ambiguity_min_support_ratio: float,
    bbox_quantization_margin_px: float,
) -> np.ndarray:
    """Mask a weak near-tied geometry layer in the generalized runtime.

    This preprocessing keeps the frozen core estimator and its evidence hash
    unchanged. It only resolves a near tie when one layer has strictly dominant
    pixel support; similarly supported layers remain untouched and therefore
    continue to fail closed in the core estimator.
    """

    values_to_validate = (
        fx,
        fy,
        target_radius_m,
        search_fraction,
        min_depth_m,
        max_depth_m,
        min_layer_fraction,
        layer_gap_m,
        expected_depth_tolerance_m,
        ambiguity_margin_m,
        ambiguity_min_support_ratio,
        bbox_quantization_margin_px,
    )
    if depth_image_m.ndim != 2:
        raise LocalizationError("depth image must be a two-dimensional array")
    if not all(math.isfinite(value) for value in values_to_validate):
        raise ValueError("support-ranked geometry parameters must be finite")
    if fx <= 0.0 or fy <= 0.0 or target_radius_m <= 0.0:
        raise ValueError("support-ranked geometry scale must be positive")
    if not 0.0 < search_fraction <= 1.0:
        raise ValueError("search fraction must be in (0, 1]")
    if min_depth_m <= 0.0 or max_depth_m <= min_depth_m:
        raise ValueError("depth limits must satisfy 0 < min_depth < max_depth")
    if min_layer_pixels <= 0 or not 0.0 <= min_layer_fraction <= 1.0:
        raise ValueError("support-ranked layer support limits are invalid")
    if layer_gap_m <= 0.0 or expected_depth_tolerance_m <= 0.0:
        raise ValueError("support-ranked geometry gaps must be positive")
    if ambiguity_margin_m < 0.0:
        raise ValueError("ambiguity margin must be non-negative")
    if not 0.0 <= ambiguity_min_support_ratio <= 1.0:
        raise ValueError("ambiguity support ratio must be in [0, 1]")
    if not 0.0 <= bbox_quantization_margin_px <= 2.0:
        raise ValueError("bounding-box quantization margin must be in [0, 2]")

    inset_x = 0.5 * (1.0 - search_fraction) * box.width
    inset_y = 0.5 * (1.0 - search_fraction) * box.height
    x0 = max(0, int(math.floor(box.x + inset_x)))
    y0 = max(0, int(math.floor(box.y + inset_y)))
    x1 = min(
        depth_image_m.shape[1],
        int(math.ceil(box.x + box.width - inset_x)),
    )
    y1 = min(
        depth_image_m.shape[0],
        int(math.ceil(box.y + box.height - inset_y)),
    )
    if x0 >= x1 or y0 >= y1:
        raise LocalizationError("bounding box does not overlap the depth image")

    crop = np.asarray(depth_image_m[y0:y1, x0:x1], dtype=np.float64)
    valid_mask = np.isfinite(crop) & (crop >= min_depth_m) & (crop <= max_depth_m)
    values = crop[valid_mask]
    required = max(
        min_layer_pixels,
        int(math.ceil(float(crop.size) * min_layer_fraction)),
    )
    if values.size < required:
        return depth_image_m

    apparent_radius_max_px = 0.5 * math.sqrt(float(box.width * box.height))
    apparent_radius_min_px = 0.5 * math.sqrt(
        max(float(box.width) - bbox_quantization_margin_px, 1e-6)
        * max(float(box.height) - bbox_quantization_margin_px, 1e-6)
    )
    effective_focal_px = math.sqrt(fx * fy)

    def expected_surface_depth(apparent_radius_px: float) -> float:
        center_depth = target_radius_m * math.sqrt(
            1.0 + (effective_focal_px / apparent_radius_px) ** 2
        )
        return center_depth - target_radius_m

    expected_min = expected_surface_depth(apparent_radius_max_px)
    expected_max = expected_surface_depth(apparent_radius_min_px)
    order = np.argsort(values, kind="stable")
    boundaries = np.flatnonzero(np.diff(values[order]) > layer_gap_m) + 1
    candidates = []
    for indices in np.split(order, boundaries):
        if indices.size < required:
            continue
        median = float(np.median(values[indices]))
        error = max(expected_min - median, median - expected_max, 0.0)
        candidates.append((error, -int(indices.size), median, indices))
    if len(candidates) < 2:
        return depth_image_m
    candidates.sort(key=lambda item: (item[0], item[1], item[2]))
    minimum_error = candidates[0][0]
    if minimum_error > expected_depth_tolerance_m:
        return depth_image_m
    near_tied = [
        candidate
        for candidate in candidates
        if candidate[0] <= expected_depth_tolerance_m
        and candidate[0] - minimum_error < ambiguity_margin_m
    ]
    if len(near_tied) < 2:
        return depth_image_m
    support_ranked = sorted(near_tied, key=lambda item: (item[1], item[0], item[2]))
    dominant = support_ranked[0]
    runner_up_support_ratio = (-support_ranked[1][1]) / (-dominant[1])
    if runner_up_support_ratio >= ambiguity_min_support_ratio:
        return depth_image_m

    valid_y, valid_x = np.nonzero(valid_mask)
    filtered = np.array(depth_image_m, dtype=np.float32, copy=True)
    selected = filtered[y0:y1, x0:x1]
    for candidate in near_tied:
        if candidate is dominant:
            continue
        selected[
            valid_y[candidate[3]],
            valid_x[candidate[3]],
        ] = np.nan
    return filtered


def adjust_point_along_optical_ray(
    point_xyz: np.ndarray, additional_distance_m: float
) -> np.ndarray:
    """Apply a signed Euclidean range correction without changing bearing."""

    point = np.asarray(point_xyz, dtype=np.float64)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError("optical-ray point must contain three finite values")
    if not math.isfinite(additional_distance_m):
        raise ValueError("optical-ray distance correction must be finite")
    distance = float(np.linalg.norm(point))
    if distance <= 1e-9 or distance + additional_distance_m <= 0.0:
        raise LocalizationError("optical-ray distance correction is invalid")
    return point * ((distance + additional_distance_m) / distance)


def point_on_pixel_bearing(
    point_xyz: np.ndarray,
    *,
    u: float,
    v: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    """Keep estimated range but use a robust pixel center for ray bearing."""

    point = np.asarray(point_xyz, dtype=np.float64)
    scalars = (u, v, fx, fy, cx, cy)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError("optical-ray point must contain three finite values")
    if not all(math.isfinite(value) for value in scalars) or fx <= 0.0 or fy <= 0.0:
        raise ValueError("pixel-bearing camera parameters are invalid")
    distance = float(np.linalg.norm(point))
    if distance <= 1e-9:
        raise LocalizationError("pixel-bearing range must be positive")
    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
    return ray * (distance / float(np.linalg.norm(ray)))


def expand_bounding_box(
    box: BoundingBox,
    *,
    padding_px: int,
    image_width: int,
    image_height: int,
) -> BoundingBox:
    """Apply a bounded symmetric correction for detector under-fitting."""

    if padding_px < 0 or image_width <= 0 or image_height <= 0:
        raise ValueError("bounding-box expansion limits are invalid")
    x0 = max(0, box.x - padding_px)
    y0 = max(0, box.y - padding_px)
    x1 = min(image_width, box.x + box.width + padding_px)
    y1 = min(image_height, box.y + box.height + padding_px)
    if x0 >= x1 or y0 >= y1:
        raise LocalizationError("expanded bounding box has no image area")
    return BoundingBox(x0, y0, x1 - x0, y1 - y0)


def retain_foreground_depth_band(
    depth_image_m: np.ndarray,
    box: BoundingBox,
    *,
    search_fraction: float,
    min_depth_m: float,
    max_depth_m: float,
    min_layer_pixels: int,
    min_layer_fraction: float,
    layer_gap_m: float,
    maximum_band_width_m: float,
) -> np.ndarray:
    """Mask distant background while retaining nearby leaf/fruit layers.

    Detector boxes can under-fit a partially occluded fruit. Apparent-size
    geometry alone may then prefer the planter or table behind the fruit. This
    bounded foreground band rejects a distant surface only when a sufficiently
    supported nearer layer exists; the frozen geometry-layer estimator remains
    responsible for choosing between nearby leaf and fruit surfaces.
    """

    if depth_image_m.ndim != 2:
        raise LocalizationError("depth image must be a two-dimensional array")
    values = (
        search_fraction,
        min_depth_m,
        max_depth_m,
        min_layer_fraction,
        layer_gap_m,
        maximum_band_width_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("foreground depth-band parameters must be finite")
    if not 0.0 < search_fraction <= 1.0:
        raise ValueError("search fraction must be in (0, 1]")
    if min_depth_m <= 0.0 or max_depth_m <= min_depth_m:
        raise ValueError("depth limits must satisfy 0 < min_depth < max_depth")
    if min_layer_pixels <= 0 or not 0.0 <= min_layer_fraction <= 1.0:
        raise ValueError("foreground layer support limits are invalid")
    if layer_gap_m <= 0.0 or maximum_band_width_m <= layer_gap_m:
        raise ValueError("foreground band must exceed the depth layer gap")

    inset_x = 0.5 * (1.0 - search_fraction) * box.width
    inset_y = 0.5 * (1.0 - search_fraction) * box.height
    x0 = max(0, int(math.floor(box.x + inset_x)))
    y0 = max(0, int(math.floor(box.y + inset_y)))
    x1 = min(depth_image_m.shape[1], int(math.ceil(box.x + box.width - inset_x)))
    y1 = min(depth_image_m.shape[0], int(math.ceil(box.y + box.height - inset_y)))
    if x0 >= x1 or y0 >= y1:
        raise LocalizationError("bounding box does not overlap the depth image")

    crop = np.asarray(depth_image_m[y0:y1, x0:x1], dtype=np.float64)
    valid = crop[
        np.isfinite(crop) & (crop >= min_depth_m) & (crop <= max_depth_m)
    ]
    required = max(min_layer_pixels, int(math.ceil(crop.size * min_layer_fraction)))
    if valid.size < required:
        raise LocalizationError("too few valid pixels for foreground depth band")

    ordered = np.sort(valid, kind="stable")
    boundaries = np.flatnonzero(np.diff(ordered) > layer_gap_m) + 1
    supported = [group for group in np.split(ordered, boundaries) if group.size >= required]
    if not supported:
        raise LocalizationError("no supported foreground depth layer")
    nearest_depth_m = float(np.median(supported[0]))
    cutoff_m = nearest_depth_m + maximum_band_width_m

    filtered = np.array(depth_image_m, dtype=np.float32, copy=True)
    selected = filtered[y0:y1, x0:x1]
    selected[np.isfinite(selected) & (selected > cutoff_m)] = np.nan
    return filtered
