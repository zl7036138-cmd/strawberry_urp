"""Deterministic, ROS-free grasp geometry for ADR 0087.

This module deliberately generates geometry only.  It neither scores a grasp
nor touches MoveIt, a PlanningScene, or an executor.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import hmac
import json
import math

from .core import (
    DEFAULT_GRASP_QUATERNION,
    DEFAULT_PREGRASP_OFFSET_M,
    DEFAULT_TOOL_CENTER_OFFSET_M,
    Pose,
    hand_pose_for_fruit_center,
    offset_along_local_y,
    offset_along_local_z,
)


# Frozen first-version search order.  The nominal candidate is always first;
# remaining candidates increase local approach-axis tilt in deterministic rings.
TILT_SEQUENCE_DEGREES: tuple[tuple[int, int], ...] = (
    (0, 0),
    (10, 0), (-10, 0), (0, 10), (0, -10),
    (10, 10), (10, -10), (-10, 10), (-10, -10),
    (20, 0), (-20, 0),
    (20, 10), (20, -10), (-20, 10), (-20, -10),
)
DEFAULT_ESCAPE_OFFSET_M = 0.08
CONTACT_CENTERING_SCALE_SEQUENCE: tuple[float, ...] = (1.0, 0.75, 0.5, 0.25)


@dataclass(frozen=True)
class GraspCandidate:
    """One indivisible candidate geometry contract.

    The evaluator and executor must use these three poses as a unit.  The
    fingerprint binds the candidate's identity to its complete geometry.
    """

    candidate_id: str
    grasp_pose: Pose
    pregrasp_pose: Pose
    escape_pose: Pose
    tilt_x_rad: float
    tilt_y_rad: float
    geometry_fingerprint: str

    def __post_init__(self) -> None:
        if not isinstance(self.candidate_id, str) or not self.candidate_id:
            raise ValueError("candidate ID must be a non-empty string")
        if not isinstance(self.geometry_fingerprint, str) or not self.geometry_fingerprint:
            raise ValueError("candidate geometry fingerprint must be non-empty")
        if not math.isfinite(self.tilt_x_rad) or not math.isfinite(self.tilt_y_rad):
            raise ValueError("candidate tilt values must be finite")
        for label, pose in (
            ("grasp", self.grasp_pose),
            ("pregrasp", self.pregrasp_pose),
            ("escape", self.escape_pose),
        ):
            if not isinstance(pose, Pose):
                raise ValueError(f"{label} pose must be a Pose")
            if not all(
                math.isfinite(value)
                for value in (pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw)
            ):
                raise ValueError(f"{label} pose must be finite")
        expected = _fingerprint(
            self.candidate_id,
            self.grasp_pose,
            self.pregrasp_pose,
            self.escape_pose,
            self.tilt_x_rad,
            self.tilt_y_rad,
        )
        if not hmac.compare_digest(self.geometry_fingerprint, expected):
            raise ValueError("candidate geometry fingerprint does not match geometry")


def _quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Return Hamilton product ``left * right`` in xyzw order."""

    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _axis_angle_x(angle_rad: float) -> tuple[float, float, float, float]:
    half = angle_rad / 2.0
    return (math.sin(half), 0.0, 0.0, math.cos(half))


def _axis_angle_y(angle_rad: float) -> tuple[float, float, float, float]:
    half = angle_rad / 2.0
    return (0.0, math.sin(half), 0.0, math.cos(half))


def _normalized_quaternion(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    normalized = Pose(0.0, 0.0, 0.0, *quaternion).normalized()
    return normalized.qx, normalized.qy, normalized.qz, normalized.qw


def _fingerprint(
    candidate_id: str,
    grasp: Pose,
    pregrasp: Pose,
    escape: Pose,
    tilt_x_rad: float,
    tilt_y_rad: float,
) -> str:
    payload = {
        "candidate_id": candidate_id,
        "grasp": tuple(vars(grasp.normalized()).values()),
        "pregrasp": tuple(vars(pregrasp.normalized()).values()),
        "escape": tuple(vars(escape.normalized()).values()),
        "tilt_x_rad": tilt_x_rad,
        "tilt_y_rad": tilt_y_rad,
    }
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def generate_grasp_candidates(
    target_pose: Pose,
    *,
    nominal_quaternion: tuple[float, float, float, float] = DEFAULT_GRASP_QUATERNION,
    tool_center_offset_m: float = DEFAULT_TOOL_CENTER_OFFSET_M,
    pregrasp_offset_m: float = DEFAULT_PREGRASP_OFFSET_M,
    escape_offset_m: float = DEFAULT_ESCAPE_OFFSET_M,
) -> tuple[GraspCandidate, ...]:
    """Generate the frozen 15-candidate local-tilt cone around nominal grasp."""

    if not all(
        math.isfinite(float(value))
        for value in (target_pose.x, target_pose.y, target_pose.z)
    ):
        raise ValueError("target position must be finite")
    if len(nominal_quaternion) != 4:
        raise ValueError("nominal quaternion must contain four values")
    if not all(math.isfinite(float(value)) for value in nominal_quaternion):
        raise ValueError("nominal quaternion must be finite")
    if any(
        not math.isfinite(distance) or distance <= 0.0
        for distance in (tool_center_offset_m, pregrasp_offset_m, escape_offset_m)
    ):
        raise ValueError("candidate offsets must be positive and finite")

    nominal = _normalized_quaternion(tuple(float(value) for value in nominal_quaternion))
    candidates: list[GraspCandidate] = []
    for index, (tilt_x_deg, tilt_y_deg) in enumerate(TILT_SEQUENCE_DEGREES):
        tilt_x_rad = math.radians(tilt_x_deg)
        tilt_y_rad = math.radians(tilt_y_deg)
        # Local rotations: R = R_nominal * R_x * R_y.  These alter the tool
        # approach axis rather than merely rolling fingers about that axis.
        orientation = _normalized_quaternion(
            _quaternion_multiply(
                _quaternion_multiply(nominal, _axis_angle_x(tilt_x_rad)),
                _axis_angle_y(tilt_y_rad),
            )
        )
        grasp = hand_pose_for_fruit_center(
            target_pose,
            quaternion=orientation,
            tool_center_offset_m=tool_center_offset_m,
        )
        pregrasp = offset_along_local_z(grasp, -pregrasp_offset_m)
        escape = offset_along_local_z(grasp, -escape_offset_m)
        candidate_id = f"G{index:02d}"
        candidates.append(
            GraspCandidate(
                candidate_id=candidate_id,
                grasp_pose=grasp,
                pregrasp_pose=pregrasp,
                escape_pose=escape,
                tilt_x_rad=tilt_x_rad,
                tilt_y_rad=tilt_y_rad,
                geometry_fingerprint=_fingerprint(
                    candidate_id, grasp, pregrasp, escape, tilt_x_rad, tilt_y_rad
                ),
            )
        )
    return tuple(candidates)


def recenter_grasp_candidate(
    source: GraspCandidate,
    *,
    local_y_offset_m: float,
    correction_index: int = 1,
) -> GraspCandidate:
    """Create one translated-contact candidate for whole-chain certification.

    The measured correction is applied to pre-grasp and grasp so the fingers
    meet the fruit around a corrected centre.  The candidate then rejoins the
    source candidate's already feasible escape waypoint.  That diagonal
    post-grasp segment is not assumed safe: it remains part of this new,
    indivisible geometry contract and must pass ADR 0086 before the candidate
    can receive a new ADR 0087-C execution certificate.

    Keeping the source escape waypoint is deliberate.  Runtime evidence from
    the controlled challenge showed that translating the escape waypoint by
    only 1.5--5.8 mm destroyed an otherwise valid narrow transport corridor.
    """

    if not isinstance(source, GraspCandidate):
        raise ValueError("source must be a GraspCandidate")
    if (
        not isinstance(correction_index, int)
        or isinstance(correction_index, bool)
        or correction_index <= 0
    ):
        raise ValueError("correction index must be a positive integer")
    if not math.isfinite(local_y_offset_m) or abs(local_y_offset_m) <= 0.0:
        raise ValueError("centering offset must be finite and non-zero")

    candidate_id = f"{source.candidate_id}-C{correction_index:02d}"
    grasp = offset_along_local_y(source.grasp_pose, local_y_offset_m)
    pregrasp = offset_along_local_y(source.pregrasp_pose, local_y_offset_m)
    escape = source.escape_pose
    return GraspCandidate(
        candidate_id=candidate_id,
        grasp_pose=grasp,
        pregrasp_pose=pregrasp,
        escape_pose=escape,
        tilt_x_rad=source.tilt_x_rad,
        tilt_y_rad=source.tilt_y_rad,
        geometry_fingerprint=_fingerprint(
            candidate_id,
            grasp,
            pregrasp,
            escape,
            source.tilt_x_rad,
            source.tilt_y_rad,
        ),
    )


def generate_recentered_grasp_candidates(
    source: GraspCandidate,
    *,
    measured_local_y_offset_m: float,
    scale_sequence: tuple[float, ...] = CONTACT_CENTERING_SCALE_SEQUENCE,
) -> tuple[GraspCandidate, ...]:
    """Generate the frozen full-to-partial centering correction ladder."""

    if not isinstance(source, GraspCandidate):
        raise ValueError("source must be a GraspCandidate")
    if (
        not math.isfinite(measured_local_y_offset_m)
        or abs(measured_local_y_offset_m) <= 0.0
    ):
        raise ValueError("measured centering offset must be finite and non-zero")
    if not isinstance(scale_sequence, tuple) or not scale_sequence:
        raise ValueError("centering scale sequence must be a non-empty tuple")
    normalized_scales = tuple(float(scale) for scale in scale_sequence)
    if any(
        not math.isfinite(scale) or scale <= 0.0 or scale > 1.0
        for scale in normalized_scales
    ):
        raise ValueError("centering scales must be finite in (0, 1]")
    if len(set(normalized_scales)) != len(normalized_scales):
        raise ValueError("centering scales must be unique")

    return tuple(
        recenter_grasp_candidate(
            source,
            local_y_offset_m=measured_local_y_offset_m * scale,
            correction_index=index,
        )
        for index, scale in enumerate(normalized_scales, start=1)
    )
