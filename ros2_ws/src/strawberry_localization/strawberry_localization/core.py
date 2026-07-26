"""Dependency-light localization primitives used by the ROS node and tests."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


class LocalizationError(ValueError):
    """Raised when a target cannot be localized from the available depth."""


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("camera intrinsics must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("focal lengths must be positive")


@dataclass(frozen=True)
class BoundingBox:
    x: int
    y: int
    width: int
    height: int

    def __post_init__(self) -> None:
        if self.x < 0 or self.y < 0:
            raise ValueError("bounding box origin must be non-negative")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("bounding box width and height must be positive")


@dataclass(frozen=True)
class DepthEstimate:
    depth_m: float
    sigma_m: float
    valid_pixels: int
    center_u: float
    center_v: float


@dataclass(frozen=True)
class LocalizationMetrics:
    sample_count: int
    median_error_mm: float
    p95_error_mm: float


@dataclass(frozen=True)
class CachedDepthFrame:
    """One validated depth frame retained for delayed detections."""

    stamp_s: float
    frame_id: str
    image: np.ndarray

    @property
    def width(self) -> int:
        return int(self.image.shape[1])

    @property
    def height(self) -> int:
        return int(self.image.shape[0])


@dataclass(frozen=True)
class CachedCameraInfo:
    """Validated camera calibration plus its original ROS message payload."""

    stamp_s: float
    frame_id: str
    width: int
    height: int
    intrinsics: CameraIntrinsics
    payload: Any = None


@dataclass(frozen=True)
class MatchedSensorFrames:
    """The coherent depth/calibration pair nearest a detection timestamp."""

    depth: CachedDepthFrame
    camera_info: CachedCameraInfo


class SensorFrameCache:
    """Bounded timestamp cache used to absorb detector inference latency.

    Entries are bounded both by count and by an explicit retention horizon.
    Matching considers every sample within the synchronization tolerance and
    only returns a pair whose frame IDs and image dimensions agree.  Invalid
    depth arrays and camera intrinsics never enter the cache.
    """

    def __init__(self, *, capacity: int = 60, retention_sec: float = 2.0) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity <= 0:
            raise ValueError("sensor cache capacity must be a positive integer")
        if not math.isfinite(retention_sec) or retention_sec <= 0.0:
            raise ValueError("sensor cache retention must be finite and positive")
        self.capacity = capacity
        self.retention_sec = float(retention_sec)
        self._depth_frames: deque[CachedDepthFrame] = deque(maxlen=capacity)
        self._camera_infos: deque[CachedCameraInfo] = deque(maxlen=capacity)

    @staticmethod
    def _validate_stamp_and_frame(stamp_s: float, frame_id: str) -> None:
        if not math.isfinite(stamp_s) or stamp_s < 0.0:
            raise ValueError("sensor timestamp must be finite and non-negative")
        if not frame_id:
            raise ValueError("sensor frame ID must be non-empty")

    @property
    def depth_count(self) -> int:
        return len(self._depth_frames)

    @property
    def camera_info_count(self) -> int:
        return len(self._camera_infos)

    def diagnostics(
        self, *, detection_stamp_s: float, detection_frame: str
    ) -> dict[str, float | int | None]:
        """Describe cache timing without changing cache contents."""

        self._validate_stamp_and_frame(detection_stamp_s, detection_frame)
        depth = [
            sample
            for sample in self._depth_frames
            if sample.frame_id == detection_frame
        ]
        camera_info = [
            sample
            for sample in self._camera_infos
            if sample.frame_id == detection_frame
        ]

        def nearest_delta(samples) -> float | None:
            if not samples:
                return None
            return min(
                (sample.stamp_s - detection_stamp_s for sample in samples),
                key=lambda value: (abs(value), value),
            )

        return {
            "depth_count": len(depth),
            "camera_info_count": len(camera_info),
            "nearest_depth_delta_sec": nearest_delta(depth),
            "nearest_camera_info_delta_sec": nearest_delta(camera_info),
            "latest_depth_stamp_sec": depth[-1].stamp_s if depth else None,
            "latest_camera_info_stamp_sec": (
                camera_info[-1].stamp_s if camera_info else None
            ),
        }

    def add_depth(
        self, *, stamp_s: float, frame_id: str, image: np.ndarray
    ) -> CachedDepthFrame:
        self._validate_stamp_and_frame(stamp_s, frame_id)
        array = np.asarray(image)
        if array.ndim != 2 or array.shape[0] <= 0 or array.shape[1] <= 0:
            raise ValueError("cached depth image must be a non-empty 2-D array")
        sample = CachedDepthFrame(float(stamp_s), frame_id, array)
        self._depth_frames.append(sample)
        self.prune(float(stamp_s))
        return sample

    def add_camera_info(
        self,
        *,
        stamp_s: float,
        frame_id: str,
        width: int,
        height: int,
        intrinsics: CameraIntrinsics,
        payload: Any = None,
    ) -> CachedCameraInfo:
        self._validate_stamp_and_frame(stamp_s, frame_id)
        if isinstance(width, bool) or isinstance(height, bool):
            raise ValueError("camera dimensions must be positive integers")
        if not isinstance(width, int) or not isinstance(height, int):
            raise ValueError("camera dimensions must be positive integers")
        if width <= 0 or height <= 0:
            raise ValueError("camera dimensions must be positive integers")
        if not isinstance(intrinsics, CameraIntrinsics):
            raise TypeError("camera intrinsics must be a CameraIntrinsics instance")
        sample = CachedCameraInfo(
            float(stamp_s), frame_id, width, height, intrinsics, payload
        )
        self._camera_infos.append(sample)
        self.prune(float(stamp_s))
        return sample

    def prune(self, reference_stamp_s: float) -> int:
        """Drop entries outside the retention horizon and return their count."""

        if not math.isfinite(reference_stamp_s) or reference_stamp_s < 0.0:
            raise ValueError(
                "cache reference timestamp must be finite and non-negative"
            )
        cutoff = float(reference_stamp_s) - self.retention_sec
        previous_count = len(self._depth_frames) + len(self._camera_infos)
        self._depth_frames = deque(
            (sample for sample in self._depth_frames if sample.stamp_s >= cutoff),
            maxlen=self.capacity,
        )
        self._camera_infos = deque(
            (sample for sample in self._camera_infos if sample.stamp_s >= cutoff),
            maxlen=self.capacity,
        )
        return previous_count - len(self._depth_frames) - len(self._camera_infos)

    def match(
        self,
        *,
        detection_stamp_s: float,
        detection_frame: str,
        sync_tolerance_sec: float,
    ) -> MatchedSensorFrames:
        """Return the nearest coherent pair for one delayed detection."""

        self._validate_stamp_and_frame(detection_stamp_s, detection_frame)
        if not math.isfinite(sync_tolerance_sec) or sync_tolerance_sec < 0.0:
            raise ValueError("sync tolerance must be finite and non-negative")

        depth_candidates = [
            sample
            for sample in self._depth_frames
            if sample.frame_id == detection_frame
            and abs(sample.stamp_s - detection_stamp_s) <= sync_tolerance_sec
        ]
        info_candidates = [
            sample
            for sample in self._camera_infos
            if sample.frame_id == detection_frame
            and abs(sample.stamp_s - detection_stamp_s) <= sync_tolerance_sec
        ]
        compatible_pairs = [
            (depth, info)
            for depth in depth_candidates
            for info in info_candidates
            if depth.width == info.width and depth.height == info.height
        ]
        if not compatible_pairs:
            raise LocalizationError(
                "no synchronized depth/CameraInfo pair matches the detection"
            )

        depth, info = min(
            compatible_pairs,
            key=lambda pair: (
                max(
                    abs(pair[0].stamp_s - detection_stamp_s),
                    abs(pair[1].stamp_s - detection_stamp_s),
                ),
                abs(pair[0].stamp_s - detection_stamp_s)
                + abs(pair[1].stamp_s - detection_stamp_s),
                abs(pair[0].stamp_s - pair[1].stamp_s),
                -pair[0].stamp_s,
                -pair[1].stamp_s,
            ),
        )
        return MatchedSensorFrames(depth=depth, camera_info=info)


def validate_sensor_metadata(
    *,
    detection_stamp_s: float,
    detection_frame: str,
    depth_stamp_s: float,
    depth_frame: str,
    camera_info_stamp_s: float,
    camera_info_frame: str,
    now_stamp_s: float,
    sync_tolerance_sec: float,
    stale_after_sec: float,
) -> None:
    """Validate that one RGB-D localization input set is coherent and fresh."""

    stamps = (
        detection_stamp_s,
        depth_stamp_s,
        camera_info_stamp_s,
        now_stamp_s,
    )
    if not all(math.isfinite(value) and value >= 0.0 for value in stamps):
        raise LocalizationError("sensor timestamps must be finite and non-negative")
    if not math.isfinite(sync_tolerance_sec) or sync_tolerance_sec < 0.0:
        raise ValueError("sync tolerance must be finite and non-negative")
    if not math.isfinite(stale_after_sec) or stale_after_sec <= 0.0:
        raise ValueError("stale timeout must be finite and positive")

    frames = (detection_frame, depth_frame, camera_info_frame)
    if any(not frame for frame in frames):
        raise LocalizationError("sensor frame IDs must be non-empty")
    if len(set(frames)) != 1:
        raise LocalizationError(
            "detection, depth, and CameraInfo frame IDs must match"
        )

    if abs(detection_stamp_s - depth_stamp_s) > sync_tolerance_sec:
        raise LocalizationError("detection and depth timestamps are not synchronized")
    if abs(detection_stamp_s - camera_info_stamp_s) > sync_tolerance_sec:
        raise LocalizationError(
            "detection and CameraInfo timestamps are not synchronized"
        )

    age = now_stamp_s - detection_stamp_s
    if age < -sync_tolerance_sec:
        raise LocalizationError("sensor acquisition timestamp is in the future")
    if age > stale_after_sec:
        raise LocalizationError("sensor acquisition is stale")


def validate_bbox_within_image(
    box: BoundingBox, image_width: int, image_height: int
) -> None:
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")
    if box.x + box.width > image_width or box.y + box.height > image_height:
        raise LocalizationError("bounding box extends outside the depth image")


def associate_nearest_target(
    point_xyz: Sequence[float],
    candidates_xyz: Mapping[int, Sequence[float]],
    *,
    max_distance_m: float = 0.08,
) -> int:
    """Associate an estimated position with a simulation entity identity.

    This function is simulation plumbing only: it copies the identity of the
    nearest ground-truth fruit but never replaces the estimated position or
    detector maturity.  Deterministic target-id tie breaking makes runs stable.
    """

    point = np.asarray(point_xyz, dtype=np.float64)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise LocalizationError("association point must contain three finite values")
    if not math.isfinite(max_distance_m) or max_distance_m <= 0.0:
        raise ValueError("association distance must be finite and positive")
    if not candidates_xyz:
        raise LocalizationError("ground-truth association candidates are unavailable")

    distances: list[tuple[float, int]] = []
    for target_id, values in candidates_xyz.items():
        if not isinstance(target_id, int) or target_id <= 0:
            raise ValueError("association target IDs must be positive integers")
        candidate = np.asarray(values, dtype=np.float64)
        if candidate.shape != (3,) or not np.all(np.isfinite(candidate)):
            raise ValueError(f"candidate {target_id} must contain three finite values")
        distances.append((float(np.linalg.norm(candidate - point)), target_id))
    distance, target_id = min(distances, key=lambda item: (item[0], item[1]))
    if distance > max_distance_m:
        raise LocalizationError(
            f"nearest ground-truth fruit is {distance:.3f} m away; "
            f"limit is {max_distance_m:.3f} m"
        )
    return target_id


def _central_crop(box: BoundingBox, fraction: float) -> tuple[int, int, int, int]:
    if not 0.0 < fraction <= 1.0:
        raise ValueError("center fraction must be in (0, 1]")
    crop_w = max(1, int(round(box.width * fraction)))
    crop_h = max(1, int(round(box.height * fraction)))
    x0 = box.x + (box.width - crop_w) // 2
    y0 = box.y + (box.height - crop_h) // 2
    return x0, y0, x0 + crop_w, y0 + crop_h


def robust_center_depth(
    depth_image_m: np.ndarray,
    box: BoundingBox,
    *,
    center_fraction: float = 0.30,
    min_depth_m: float = 0.05,
    max_depth_m: float = 5.0,
    min_valid_pixels: int = 9,
) -> DepthEstimate:
    """Estimate target depth from the median of a central bounding-box crop."""

    if depth_image_m.ndim != 2:
        raise LocalizationError("depth image must be a two-dimensional array")
    if not math.isfinite(min_depth_m) or not math.isfinite(max_depth_m):
        raise ValueError("depth limits must be finite")
    if min_depth_m <= 0.0 or max_depth_m <= min_depth_m:
        raise ValueError("depth limits must satisfy 0 < min_depth < max_depth")
    if isinstance(min_valid_pixels, bool) or min_valid_pixels <= 0:
        raise ValueError("minimum valid-pixel count must be positive")
    height, width = depth_image_m.shape
    x0, y0, x1, y1 = _central_crop(box, center_fraction)
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(width, x1), min(height, y1)
    if x0 >= x1 or y0 >= y1:
        raise LocalizationError("bounding box does not overlap the depth image")

    crop = np.asarray(depth_image_m[y0:y1, x0:x1], dtype=np.float64)
    valid_mask = np.isfinite(crop) & (crop >= min_depth_m) & (crop <= max_depth_m)
    values = crop[valid_mask]
    if values.size < min_valid_pixels:
        raise LocalizationError(
            f"only {values.size} valid depth pixels; need at least {min_valid_pixels}"
        )

    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    sigma = 1.4826 * mad
    return DepthEstimate(
        depth_m=median,
        sigma_m=sigma,
        valid_pixels=int(values.size),
        center_u=(box.x + box.width / 2.0),
        center_v=(box.y + box.height / 2.0),
    )


def project_pixel_to_camera(
    u: float, v: float, depth_m: float, intrinsics: CameraIntrinsics
) -> np.ndarray:
    if not np.isfinite(depth_m) or depth_m <= 0.0:
        raise LocalizationError("depth must be finite and positive")
    x = (u - intrinsics.cx) * depth_m / intrinsics.fx
    y = (v - intrinsics.cy) * depth_m / intrinsics.fy
    return np.array([x, y, depth_m], dtype=np.float64)


def localize_bbox(
    depth_image_m: np.ndarray,
    box: BoundingBox,
    intrinsics: CameraIntrinsics,
    *,
    center_fraction: float = 0.30,
    min_depth_m: float = 0.05,
    max_depth_m: float = 5.0,
    min_valid_pixels: int = 9,
    surface_to_center_offset_m: float = 0.0,
) -> tuple[np.ndarray, DepthEstimate]:
    """Localize a box and optionally shift a rigid surface hit to its centre.

    ``surface_to_center_offset_m`` is a Euclidean distance along the optical
    ray, not an optical-Z increment.  It defaults to zero so non-rigid or
    unknown-size targets retain the raw median-depth behavior.
    """

    if (
        not math.isfinite(surface_to_center_offset_m)
        or surface_to_center_offset_m < 0.0
    ):
        raise ValueError("surface-to-center offset must be finite and non-negative")
    estimate = robust_center_depth(
        depth_image_m,
        box,
        center_fraction=center_fraction,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
        min_valid_pixels=min_valid_pixels,
    )
    point = project_pixel_to_camera(
        estimate.center_u, estimate.center_v, estimate.depth_m, intrinsics
    )
    if surface_to_center_offset_m:
        ray_norm = float(np.linalg.norm(point))
        if not math.isfinite(ray_norm) or ray_norm <= 0.0:
            raise LocalizationError("camera ray is invalid")
        point = point + point * (surface_to_center_offset_m / ray_norm)
    return point, estimate


def summarize_position_errors(
    estimated_xyz: Iterable[Sequence[float]],
    truth_xyz: Iterable[Sequence[float]],
    *,
    minimum_samples: int = 100,
) -> LocalizationMetrics:
    """Compute deterministic 3-D error metrics for the T40 acceptance gate."""

    if isinstance(minimum_samples, bool) or minimum_samples <= 0:
        raise ValueError("minimum sample count must be positive")
    estimated = np.asarray(list(estimated_xyz), dtype=np.float64)
    truth = np.asarray(list(truth_xyz), dtype=np.float64)
    if estimated.ndim != 2 or estimated.shape[1:] != (3,):
        raise ValueError("estimated positions must have shape (N, 3)")
    if truth.shape != estimated.shape:
        raise ValueError("truth positions must match estimated position shape")
    if estimated.shape[0] < minimum_samples:
        raise LocalizationError(
            f"only {estimated.shape[0]} localization samples; "
            f"need at least {minimum_samples}"
        )
    if not np.all(np.isfinite(estimated)) or not np.all(np.isfinite(truth)):
        raise LocalizationError("localization samples must contain finite values")

    errors_mm = np.linalg.norm(estimated - truth, axis=1) * 1000.0
    return LocalizationMetrics(
        sample_count=int(errors_mm.size),
        median_error_mm=float(np.percentile(errors_mm, 50, method="linear")),
        p95_error_mm=float(np.percentile(errors_mm, 95, method="linear")),
    )
