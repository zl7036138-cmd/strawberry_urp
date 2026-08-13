"""Deterministic target ranking and bounded eye-in-hand view generation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Iterable, Sequence


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _point(value: Sequence[float], name: str) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError(f"{name} must contain x, y, z")
    return tuple(_finite(component, name) for component in value)


@dataclass(frozen=True)
class HarvestCandidate:
    track_id: int
    maturity: int
    position: tuple[float, float, float]
    confidence: float
    sigma_m: float
    observation_count: int
    last_seen_sec: float
    clearance_m: float
    joint_travel_rad: float
    pregrasp_feasible: bool
    grasp_feasible: bool
    retreat_feasible: bool

    def __post_init__(self) -> None:
        if self.track_id <= 0:
            raise ValueError("track_id must be positive")
        if self.maturity not in {0, 1, 2}:
            raise ValueError("maturity is invalid")
        object.__setattr__(self, "position", _point(self.position, "position"))
        for name in (
            "confidence",
            "sigma_m",
            "last_seen_sec",
            "clearance_m",
            "joint_travel_rad",
        ):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        if self.sigma_m <= 0.0 or self.clearance_m < 0.0 or self.joint_travel_rad < 0.0:
            raise ValueError("quality and motion values are outside their valid ranges")
        if self.observation_count <= 0:
            raise ValueError("observation_count must be positive")

    @property
    def path_feasible(self) -> bool:
        return self.pregrasp_feasible and self.grasp_feasible and self.retreat_feasible


def rank_safe_targets(
    candidates: Iterable[HarvestCandidate],
    *,
    now_sec: float,
    confidence_threshold: float = 0.60,
    maximum_sigma_m: float = 0.015,
    minimum_observations: int = 3,
    maximum_age_sec: float = 0.50,
    minimum_clearance_m: float = 0.02,
    excluded_track_ids: Iterable[int] = (),
) -> tuple[HarvestCandidate, ...]:
    """Filter fail-closed and rank by safety, quality, then motion cost."""

    now = _finite(now_sec, "now_sec")
    values = (
        confidence_threshold,
        maximum_sigma_m,
        maximum_age_sec,
        minimum_clearance_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("selection limits must be finite")
    if not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("confidence_threshold must be in [0, 1]")
    if maximum_sigma_m <= 0.0 or maximum_age_sec <= 0.0 or minimum_clearance_m < 0.0:
        raise ValueError("selection bounds are invalid")
    if minimum_observations <= 0:
        raise ValueError("minimum_observations must be positive")
    excluded = {int(value) for value in excluded_track_ids}
    safe = [
        candidate
        for candidate in candidates
        if candidate.track_id not in excluded
        and candidate.maturity == 1
        and candidate.confidence >= confidence_threshold
        and candidate.sigma_m <= maximum_sigma_m
        and candidate.observation_count >= minimum_observations
        and 0.0 <= now - candidate.last_seen_sec <= maximum_age_sec
        and candidate.clearance_m >= minimum_clearance_m
        and candidate.path_feasible
    ]
    return tuple(
        sorted(
            safe,
            key=lambda candidate: (
                -candidate.clearance_m,
                candidate.sigma_m,
                -candidate.confidence,
                candidate.joint_travel_rad,
                candidate.track_id,
            ),
        )
    )


def target_rejection_reasons(
    candidate: HarvestCandidate,
    *,
    now_sec: float,
    confidence_threshold: float = 0.60,
    maximum_sigma_m: float = 0.015,
    minimum_observations: int = 3,
    maximum_age_sec: float = 0.50,
    minimum_clearance_m: float = 0.02,
    excluded_track_ids: Iterable[int] = (),
) -> tuple[str, ...]:
    """Explain every fail-closed target gate with stable reason codes."""

    now = _finite(now_sec, "now_sec")
    excluded = {int(value) for value in excluded_track_ids}
    age_sec = now - candidate.last_seen_sec
    reasons = []
    if candidate.track_id in excluded:
        reasons.append("EXCLUDED")
    if candidate.maturity != 1:
        reasons.append("NOT_RIPE")
    if candidate.confidence < confidence_threshold:
        reasons.append("LOW_CONFIDENCE")
    if candidate.sigma_m > maximum_sigma_m:
        reasons.append("HIGH_UNCERTAINTY")
    if candidate.observation_count < minimum_observations:
        reasons.append("INSUFFICIENT_OBSERVATIONS")
    if not 0.0 <= age_sec <= maximum_age_sec:
        reasons.append("STALE_OR_FUTURE_DATA")
    if candidate.clearance_m < minimum_clearance_m:
        reasons.append("LOW_CLEARANCE")
    if not candidate.path_feasible:
        reasons.append("OUTSIDE_CONSERVATIVE_REACH")
    return tuple(reasons)


def wrist_refinement_rejection_reason(
    *,
    expected_target_id: int,
    observed_target_id: int,
    correction_m: float,
    confidence: float,
    sigma_m: float,
    maximum_correction_m: float,
    minimum_confidence: float,
    maximum_sigma_m: float,
) -> str | None:
    """Return one stable fail-closed reason for a wrist confirmation."""

    values = (
        correction_m,
        confidence,
        sigma_m,
        maximum_correction_m,
        minimum_confidence,
        maximum_sigma_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return "NONFINITE_MEASUREMENT"
    if int(expected_target_id) <= 0 or int(observed_target_id) <= 0:
        return "INVALID_TARGET_ID"
    if int(observed_target_id) != int(expected_target_id):
        return "TARGET_ID_MISMATCH"
    if correction_m < 0.0 or sigma_m < 0.0:
        return "INVALID_QUALITY"
    if correction_m > maximum_correction_m:
        return "CORRECTION_TOO_LARGE"
    if confidence < minimum_confidence:
        return "LOW_CONFIDENCE"
    if sigma_m > maximum_sigma_m:
        return "HIGH_UNCERTAINTY"
    return None


def wrist_refinement_diagnostics(
    *,
    expected_target_id: int,
    observed_target_id: int,
    correction_m: float,
    confidence: float,
    sigma_m: float,
    maximum_correction_m: float,
    minimum_confidence: float,
    maximum_sigma_m: float,
) -> dict[str, int | float | str]:
    """Return auditable wrist measurements together with the gate result."""

    reason = wrist_refinement_rejection_reason(
        expected_target_id=expected_target_id,
        observed_target_id=observed_target_id,
        correction_m=correction_m,
        confidence=confidence,
        sigma_m=sigma_m,
        maximum_correction_m=maximum_correction_m,
        minimum_confidence=minimum_confidence,
        maximum_sigma_m=maximum_sigma_m,
    )
    return {
        "expected_target_id": int(expected_target_id),
        "observed_target_id": int(observed_target_id),
        "correction_m": float(correction_m),
        "maximum_correction_m": float(maximum_correction_m),
        "confidence": float(confidence),
        "minimum_confidence": float(minimum_confidence),
        "sigma_m": float(sigma_m),
        "maximum_sigma_m": float(maximum_sigma_m),
        "gate_result": "ACCEPTED" if reason is None else reason,
    }


def fuse_position_estimates(
    base_position: Sequence[float],
    wrist_position: Sequence[float],
    *,
    base_sigma_m: float,
    wrist_sigma_m: float,
    wrist_systematic_sigma_m: float,
) -> tuple[tuple[float, float, float], float]:
    """Fuse overview and wrist estimates with a calibrated systematic floor."""

    base = _point(base_position, "base_position")
    wrist = _point(wrist_position, "wrist_position")
    base_sigma = _finite(base_sigma_m, "base_sigma_m")
    wrist_sigma = _finite(wrist_sigma_m, "wrist_sigma_m")
    systematic = _finite(wrist_systematic_sigma_m, "wrist_systematic_sigma_m")
    if base_sigma <= 0.0 or wrist_sigma <= 0.0 or systematic < 0.0:
        raise ValueError("position fusion uncertainty is invalid")
    effective_wrist_sigma = math.hypot(wrist_sigma, systematic)
    base_weight = 1.0 / (base_sigma * base_sigma)
    wrist_weight = 1.0 / (effective_wrist_sigma * effective_wrist_sigma)
    total_weight = base_weight + wrist_weight
    fused = tuple(
        (base_weight * base[index] + wrist_weight * wrist[index]) / total_weight
        for index in range(3)
    )
    return fused, math.sqrt(1.0 / total_weight)


@dataclass(frozen=True)
class ViewPose:
    target_id: int
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    azimuth_rad: float
    elevation_rad: float
    distance_m: float


@dataclass(frozen=True)
class ViewAssessment:
    ik_reachable: bool
    collision_free: bool
    target_in_view: bool
    clearance_m: float
    joint_travel_rad: float

    @property
    def feasible(self) -> bool:
        return self.ik_reachable and self.collision_free and self.target_in_view


@dataclass(frozen=True)
class HandObservationPose:
    target_id: int
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


def _normalize(vector: Sequence[float]) -> tuple[float, float, float]:
    norm = math.sqrt(sum(float(value) ** 2 for value in vector))
    if norm <= 1e-9:
        raise ValueError("cannot normalize a zero vector")
    return tuple(float(value) / norm for value in vector)


def _cross(left: Sequence[float], right: Sequence[float]) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _quaternion_from_basis(
    x_axis: Sequence[float], y_axis: Sequence[float], z_axis: Sequence[float]
) -> tuple[float, float, float, float]:
    # Rotation matrix columns are the optical-frame axes expressed in base.
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m21 - m12) / scale
        qy = (m02 - m20) / scale
        qz = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / scale
        qx = 0.25 * scale
        qy = (m01 + m10) / scale
        qz = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / scale
        qx = (m01 + m10) / scale
        qy = 0.25 * scale
        qz = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / scale
        qx = (m02 + m20) / scale
        qy = (m12 + m21) / scale
        qz = 0.25 * scale
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / norm, qy / norm, qz / norm, qw / norm


def _quaternion_multiply(left, right) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(quaternion, vector) -> tuple[float, float, float]:
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    # Rodrigues-equivalent quaternion vector rotation.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def hand_pose_for_optical_view(
    view: ViewPose,
    *,
    camera_translation_in_hand_m: Sequence[float] = (-0.065, 0.0, 0.045),
) -> HandObservationPose:
    """Convert a desired wrist optical pose into the Panda hand goal pose.

    The dual/wrist URDF's combined hand-to-optical rotation is -90 degrees
    around hand Z and its optical origin is the camera-link translation.
    """

    translation = _point(camera_translation_in_hand_m, "camera translation")
    half = math.pi / 4.0
    optical_to_hand = (0.0, 0.0, math.sin(half), math.cos(half))
    hand_quaternion = _quaternion_multiply(view.quaternion_xyzw, optical_to_hand)
    norm = math.sqrt(sum(value * value for value in hand_quaternion))
    hand_quaternion = tuple(value / norm for value in hand_quaternion)
    translated = _rotate_vector(hand_quaternion, translation)
    hand_position = tuple(
        view.position[index] - translated[index] for index in range(3)
    )
    return HandObservationPose(view.target_id, hand_position, hand_quaternion)


def generate_dynamic_views(
    target_id: int,
    target_position: Sequence[float],
    *,
    distances_m: Sequence[float] = (0.22, 0.26),
    azimuths_deg: Sequence[float] = (-35.0, 0.0, 35.0),
    elevations_deg: Sequence[float] = (15.0, 30.0),
) -> tuple[ViewPose, ...]:
    """Generate a finite eye-in-hand pose bank that looks at one 3-D target."""

    if target_id <= 0:
        raise ValueError("target_id must be positive")
    target = _point(target_position, "target_position")
    result = []
    for distance in distances_m:
        distance = _finite(distance, "view distance")
        if distance <= 0.0:
            raise ValueError("view distances must be positive")
        for elevation_deg in elevations_deg:
            elevation = math.radians(_finite(elevation_deg, "elevation"))
            for azimuth_deg in azimuths_deg:
                azimuth = math.radians(_finite(azimuth_deg, "azimuth"))
                horizontal = distance * math.cos(elevation)
                position = (
                    target[0] - horizontal * math.cos(azimuth),
                    target[1] - horizontal * math.sin(azimuth),
                    target[2] + distance * math.sin(elevation),
                )
                forward = _normalize(
                    tuple(target[index] - position[index] for index in range(3))
                )
                right = _normalize(_cross(forward, (0.0, 0.0, 1.0)))
                down = _normalize(_cross(forward, right))
                result.append(
                    ViewPose(
                        target_id=target_id,
                        position=position,
                        quaternion_xyzw=_quaternion_from_basis(right, down, forward),
                        azimuth_rad=azimuth,
                        elevation_rad=elevation,
                        distance_m=distance,
                    )
                )
    return tuple(result)


def select_dynamic_view(
    views: Iterable[ViewPose],
    evaluator: Callable[[ViewPose], ViewAssessment],
) -> tuple[ViewPose, ViewAssessment] | None:
    ranked = rank_dynamic_views(views, evaluator)
    if not ranked:
        return None
    return ranked[0]


def rank_dynamic_views(
    views: Iterable[ViewPose],
    evaluator: Callable[[ViewPose], ViewAssessment],
) -> tuple[tuple[ViewPose, ViewAssessment], ...]:
    """Return every conservatively feasible view in deterministic order.

    The runtime motion service remains the authority for IK and collision
    feasibility.  This first pass only removes views that are certainly
    outside the configured workspace or camera constraints.
    """

    assessed = [(view, evaluator(view)) for view in views]
    feasible = [
        (view, assessment) for view, assessment in assessed if assessment.feasible
    ]
    return tuple(
        sorted(
            feasible,
            key=lambda pair: (
                -pair[1].clearance_m,
                pair[1].joint_travel_rad,
                pair[0].distance_m,
                abs(pair[0].azimuth_rad),
                pair[0].elevation_rad,
            ),
        )
    )


def rank_distinct_view_indices(
    view_positions: Sequence[Sequence[float]],
    previous_position: Sequence[float],
    *,
    minimum_baseline_m: float = 0.04,
) -> tuple[int, ...]:
    """Rank safe re-observation views by parallax from the previous view.

    The caller passes only views that survived the normal visibility and
    conservative feasibility gates. Candidates too close to the previous
    camera pose are rejected; the rest are ordered by decreasing translation
    baseline with their original order used as a deterministic tie-breaker.
    """

    previous = _point(previous_position, "previous_position")
    baseline_limit = _finite(minimum_baseline_m, "minimum_baseline_m")
    if baseline_limit < 0.0:
        raise ValueError("minimum_baseline_m must be non-negative")

    ranked: list[tuple[float, int]] = []
    for index, position in enumerate(view_positions):
        candidate = _point(position, "view_position")
        baseline = math.dist(previous, candidate)
        if baseline + 1e-12 >= baseline_limit:
            ranked.append((baseline, index))

    ranked.sort(key=lambda item: (-item[0], item[1]))
    return tuple(index for _baseline, index in ranked)
