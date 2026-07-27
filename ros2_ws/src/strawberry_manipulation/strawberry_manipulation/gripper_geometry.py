"""Exact mesh geometry helpers for Blender-v2 gripper qualification."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import struct
from typing import Iterable, Mapping, Sequence


Point3 = tuple[float, float, float]
Triangle = tuple[Point3, Point3, Point3]

GATE_ID = "blender_v2_gripper_geometry_sweep_v1"
GATE_SCOPE = "NON_ACCEPTANCE_BLENDER_V2_GRIPPER_GEOMETRY"
EXPECTED_TOOL_OFFSETS_M = (0.0944, 0.0954, 0.0964, 0.0974, 0.1004, 0.1054)
EXPECTED_CLOSE_WIDTHS_M = (0.018, 0.020, 0.022, 0.025)
EXPECTED_LOCAL_BINDINGS = {
    "scene_manifest",
    "panda_extension_xacro",
    "manipulation_core",
    "action_server",
    "attachment_manager",
}
EXPECTED_EXTERNAL_BINDINGS = {
    "upstream_panda_xacro",
    "upstream_hand_collision_mesh",
    "upstream_finger_collision_mesh",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _vector_sub(left: Point3, right: Point3) -> Point3:
    return tuple(left[index] - right[index] for index in range(3))  # type: ignore[return-value]


def _vector_add(left: Point3, right: Point3) -> Point3:
    return tuple(left[index] + right[index] for index in range(3))  # type: ignore[return-value]


def _vector_scale(value: Point3, scalar: float) -> Point3:
    return tuple(component * scalar for component in value)  # type: ignore[return-value]


def _dot(left: Point3, right: Point3) -> float:
    return sum(left[index] * right[index] for index in range(3))


def _squared_norm(value: Point3) -> float:
    return _dot(value, value)


def closest_point_on_triangle(
    point: Point3,
    triangle: Triangle,
) -> Point3:
    """Return the closest point using the region tests from RTCD."""

    a, b, c = triangle
    ab = _vector_sub(b, a)
    ac = _vector_sub(c, a)
    ap = _vector_sub(point, a)
    d1 = _dot(ab, ap)
    d2 = _dot(ac, ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return a

    bp = _vector_sub(point, b)
    d3 = _dot(ab, bp)
    d4 = _dot(ac, bp)
    if d3 >= 0.0 and d4 <= d3:
        return b

    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        divisor = d1 - d3
        if abs(divisor) <= 1.0e-15:
            return a
        return _vector_add(a, _vector_scale(ab, d1 / divisor))

    cp = _vector_sub(point, c)
    d5 = _dot(ab, cp)
    d6 = _dot(ac, cp)
    if d6 >= 0.0 and d5 <= d6:
        return c

    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        divisor = d2 - d6
        if abs(divisor) <= 1.0e-15:
            return a
        return _vector_add(a, _vector_scale(ac, d2 / divisor))

    va = d3 * d6 - d5 * d4
    edge_d43 = d4 - d3
    edge_d56 = d5 - d6
    if va <= 0.0 and edge_d43 >= 0.0 and edge_d56 >= 0.0:
        edge = _vector_sub(c, b)
        divisor = edge_d43 + edge_d56
        if abs(divisor) <= 1.0e-15:
            return b
        return _vector_add(b, _vector_scale(edge, edge_d43 / divisor))

    divisor = va + vb + vc
    if abs(divisor) <= 1.0e-15:
        return min(
            (a, b, c),
            key=lambda candidate: _squared_norm(
                _vector_sub(point, candidate)
            ),
        )
    inverse = 1.0 / divisor
    v = vb * inverse
    w = vc * inverse
    return _vector_add(
        a,
        _vector_add(_vector_scale(ab, v), _vector_scale(ac, w)),
    )


def nearest_mesh_point(
    point: Point3,
    triangles: Sequence[Triangle],
) -> tuple[float, Point3, int]:
    if not triangles:
        raise ValueError("mesh must contain at least one triangle")
    best_distance_squared = math.inf
    best_point: Point3 | None = None
    best_index = -1
    for index, triangle in enumerate(triangles):
        candidate = closest_point_on_triangle(point, triangle)
        distance_squared = _squared_norm(_vector_sub(point, candidate))
        if distance_squared < best_distance_squared:
            best_distance_squared = distance_squared
            best_point = candidate
            best_index = index
    assert best_point is not None
    return math.sqrt(best_distance_squared), best_point, best_index


def read_binary_stl(path: Path) -> tuple[Triangle, ...]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path} is too short to be a binary STL")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + 50 * triangle_count
    if len(data) != expected_size:
        raise ValueError(
            f"{path} binary STL size mismatch: {len(data)} != {expected_size}"
        )
    triangles: list[Triangle] = []
    for index in range(triangle_count):
        offset = 84 + 50 * index + 12
        vertices = tuple(
            tuple(
                float(value)
                for value in struct.unpack_from(
                    "<3f", data, offset + 12 * vertex_index
                )
            )
            for vertex_index in range(3)
        )
        triangles.append(vertices)  # type: ignore[arg-type]
    return tuple(triangles)


def transform_finger_mesh(
    triangles: Sequence[Triangle],
    *,
    side: str,
    joint_width_m: float,
    joint_z_m: float,
) -> tuple[Triangle, ...]:
    if side not in {"left", "right"}:
        raise ValueError("finger side must be left or right")
    if not 0.0 <= joint_width_m <= 0.04:
        raise ValueError("finger width must be within [0, 0.04] m")

    def transform(point: Point3) -> Point3:
        x, y, z = point
        if side == "left":
            return (x, y + joint_width_m, z + joint_z_m)
        return (-x, -y - joint_width_m, z + joint_z_m)

    return tuple(
        tuple(transform(vertex) for vertex in triangle)  # type: ignore[arg-type]
        for triangle in triangles
    )


def point_to_oriented_wrist_housing_distance(
    point: Point3,
    *,
    center_m: Point3,
    size_m: Point3,
) -> float:
    """Distance to the fixed -pi/2-pitch wrist-camera housing box."""

    delta = _vector_sub(point, center_m)
    # Inverse of R_y(-pi/2): local x=hand z, y=hand y, local z=-hand x.
    local = (delta[2], delta[1], -delta[0])
    half = tuple(component / 2.0 for component in size_m)
    outside = tuple(
        max(abs(local[index]) - half[index], 0.0) for index in range(3)
    )
    return math.sqrt(_squared_norm(outside))


@dataclass(frozen=True)
class GripperGeometry:
    fruit_radius_m: float = 0.026
    finger_joint_z_m: float = 0.0584
    contact_pad_z_from_finger_m: float = 0.027
    open_width_m: float = 0.04
    contact_margin_m: float = 0.003
    pregrasp_offset_m: float = 0.15
    wrist_housing_center_m: Point3 = (-0.065, 0.0, 0.045)
    wrist_housing_size_m: Point3 = (0.055, 0.080, 0.040)

    @property
    def contact_pad_z_m(self) -> float:
        return self.finger_joint_z_m + self.contact_pad_z_from_finger_m


def finger_contact_width(
    fruit_center: Point3,
    finger_triangles: Sequence[Triangle],
    *,
    side: str,
    geometry: GripperGeometry,
) -> tuple[float, float, Point3]:
    """Solve the first symmetric joint width at which a finger touches."""

    def clearance(width: float) -> tuple[float, Point3]:
        transformed = transform_finger_mesh(
            finger_triangles,
            side=side,
            joint_width_m=width,
            joint_z_m=geometry.finger_joint_z_m,
        )
        distance, nearest, _ = nearest_mesh_point(fruit_center, transformed)
        return distance - geometry.fruit_radius_m, nearest

    closed_clearance, closed_point = clearance(0.0)
    open_clearance, open_point = clearance(geometry.open_width_m)
    if closed_clearance > 0.0:
        raise ValueError(f"{side} finger cannot reach the fruit")
    if open_clearance <= 0.0:
        raise ValueError(f"{side} finger already contacts fruit when open")

    contact_bound = 0.0
    clear_bound = geometry.open_width_m
    nearest = closed_point
    for _ in range(60):
        midpoint = 0.5 * (contact_bound + clear_bound)
        midpoint_clearance, midpoint_point = clearance(midpoint)
        if midpoint_clearance <= 0.0:
            contact_bound = midpoint
            nearest = midpoint_point
        else:
            clear_bound = midpoint
            open_point = midpoint_point
    contact_width = 0.5 * (contact_bound + clear_bound)
    contact_clearance, nearest = clearance(contact_width)
    return contact_width, contact_clearance, nearest


def _projection_between(
    fruit: Point3,
    left: Point3,
    right: Point3,
) -> tuple[bool, float]:
    separation = _vector_sub(right, left)
    denominator = _squared_norm(separation)
    if denominator <= 1.0e-15:
        return False, math.nan
    projection = _dot(_vector_sub(fruit, left), separation) / denominator
    return 0.0 <= projection <= 1.0, projection


def _minimum_normalized_margin(
    *,
    palm_clearance_m: float,
    wrist_clearance_m: float,
    open_clearance_m: float,
    overtravel_m: float,
    fallback_margin_m: float,
    pregrasp_clearance_m: float,
    thresholds: Mapping[str, float],
) -> float:
    margins = (
        (palm_clearance_m - thresholds["minimum_palm_clearance_m"])
        / thresholds["minimum_palm_clearance_m"],
        (wrist_clearance_m - thresholds["minimum_wrist_housing_clearance_m"])
        / thresholds["minimum_wrist_housing_clearance_m"],
        (open_clearance_m - thresholds["minimum_open_finger_clearance_m"])
        / thresholds["minimum_open_finger_clearance_m"],
        (overtravel_m - thresholds["minimum_close_overtravel_m"])
        / thresholds["minimum_close_overtravel_m"],
        (thresholds["maximum_close_overtravel_m"] - overtravel_m)
        / thresholds["maximum_close_overtravel_m"],
        (fallback_margin_m - thresholds["minimum_fallback_margin_m"])
        / thresholds["minimum_fallback_margin_m"],
        (
            pregrasp_clearance_m
            - thresholds["minimum_pregrasp_clearance_m"]
        )
        / thresholds["minimum_pregrasp_clearance_m"],
    )
    return min(margins)


def evaluate_candidate(
    *,
    hand_triangles: Sequence[Triangle],
    finger_triangles: Sequence[Triangle],
    tool_center_offset_m: float,
    close_width_m: float,
    geometry: GripperGeometry,
    thresholds: Mapping[str, float],
) -> dict[str, object]:
    fruit_center = (0.0, 0.0, float(tool_center_offset_m))
    hand_distance, hand_point, _ = nearest_mesh_point(
        fruit_center, hand_triangles
    )
    palm_clearance = hand_distance - geometry.fruit_radius_m
    wrist_distance = point_to_oriented_wrist_housing_distance(
        fruit_center,
        center_m=geometry.wrist_housing_center_m,
        size_m=geometry.wrist_housing_size_m,
    )
    wrist_clearance = wrist_distance - geometry.fruit_radius_m

    contact_records: dict[str, dict[str, object]] = {}
    open_clearances: list[float] = []
    contact_widths: list[float] = []
    for side in ("left", "right"):
        open_mesh = transform_finger_mesh(
            finger_triangles,
            side=side,
            joint_width_m=geometry.open_width_m,
            joint_z_m=geometry.finger_joint_z_m,
        )
        open_distance, open_point, _ = nearest_mesh_point(
            fruit_center, open_mesh
        )
        open_clearance = open_distance - geometry.fruit_radius_m
        contact_width, contact_error, contact_point = finger_contact_width(
            fruit_center,
            finger_triangles,
            side=side,
            geometry=geometry,
        )
        open_clearances.append(open_clearance)
        contact_widths.append(contact_width)
        contact_records[side] = {
            "open_clearance_m": open_clearance,
            "open_nearest_point_hand_m": list(open_point),
            "first_contact_width_m": contact_width,
            "first_contact_solver_error_m": contact_error,
            "first_contact_point_hand_m": list(contact_point),
        }

    contact_width = min(contact_widths)
    contact_symmetry_error = abs(contact_widths[0] - contact_widths[1])
    overtravel = contact_width - close_width_m
    pad_z = geometry.contact_pad_z_m
    left_pad = (0.0, contact_widths[0], pad_z)
    right_pad = (0.0, -contact_widths[1], pad_z)
    fallback_limit = geometry.fruit_radius_m + geometry.contact_margin_m
    pad_distances = (
        math.dist(fruit_center, left_pad),
        math.dist(fruit_center, right_pad),
    )
    fallback_margin = fallback_limit - max(pad_distances)
    projected_between, projection = _projection_between(
        fruit_center, left_pad, right_pad
    )

    finger_z_values = [
        vertex[2] + geometry.finger_joint_z_m
        for triangle in finger_triangles
        for vertex in triangle
    ]
    finger_z_range = (min(finger_z_values), max(finger_z_values))
    contact_points = [
        contact_records[side]["first_contact_point_hand_m"]
        for side in ("left", "right")
    ]
    contact_axial_inside = all(
        finger_z_range[0] <= float(point[2]) <= finger_z_range[1]
        for point in contact_points
    )

    pregrasp_center = (
        0.0,
        0.0,
        tool_center_offset_m + geometry.pregrasp_offset_m,
    )
    pregrasp_distances = [
        nearest_mesh_point(pregrasp_center, hand_triangles)[0],
        point_to_oriented_wrist_housing_distance(
            pregrasp_center,
            center_m=geometry.wrist_housing_center_m,
            size_m=geometry.wrist_housing_size_m,
        ),
    ]
    for side in ("left", "right"):
        open_mesh = transform_finger_mesh(
            finger_triangles,
            side=side,
            joint_width_m=geometry.open_width_m,
            joint_z_m=geometry.finger_joint_z_m,
        )
        pregrasp_distances.append(
            nearest_mesh_point(pregrasp_center, open_mesh)[0]
        )
    pregrasp_clearance = min(pregrasp_distances) - geometry.fruit_radius_m

    checks = {
        "palm_clearance": (
            palm_clearance >= thresholds["minimum_palm_clearance_m"]
        ),
        "wrist_housing_clearance": (
            wrist_clearance
            >= thresholds["minimum_wrist_housing_clearance_m"]
        ),
        "open_finger_clearance": (
            min(open_clearances)
            >= thresholds["minimum_open_finger_clearance_m"]
        ),
        "symmetric_first_contact": (
            contact_symmetry_error
            <= thresholds["maximum_contact_symmetry_error_m"]
        ),
        "minimum_close_overtravel": (
            overtravel >= thresholds["minimum_close_overtravel_m"]
        ),
        "maximum_close_overtravel": (
            overtravel <= thresholds["maximum_close_overtravel_m"]
        ),
        "fallback_pad_margin": (
            fallback_margin >= thresholds["minimum_fallback_margin_m"]
        ),
        "fruit_between_pads": projected_between,
        "contact_inside_finger_axial_section": contact_axial_inside,
        "pregrasp_clearance": (
            pregrasp_clearance
            >= thresholds["minimum_pregrasp_clearance_m"]
        ),
    }
    feasible = all(checks.values())
    normalized_margin = _minimum_normalized_margin(
        palm_clearance_m=palm_clearance,
        wrist_clearance_m=wrist_clearance,
        open_clearance_m=min(open_clearances),
        overtravel_m=overtravel,
        fallback_margin_m=fallback_margin,
        pregrasp_clearance_m=pregrasp_clearance,
        thresholds=thresholds,
    )
    return {
        "tool_center_offset_m": tool_center_offset_m,
        "close_width_m_per_finger": close_width_m,
        "feasible": feasible,
        "checks": checks,
        "minimum_normalized_threshold_margin": normalized_margin,
        "fruit_center_in_hand_m": list(fruit_center),
        "palm_clearance_m": palm_clearance,
        "palm_nearest_point_hand_m": list(hand_point),
        "wrist_housing_clearance_m": wrist_clearance,
        "finger": contact_records,
        "minimum_open_finger_clearance_m": min(open_clearances),
        "first_contact_width_m_per_finger": contact_width,
        "contact_symmetry_error_m": contact_symmetry_error,
        "close_command_overtravel_m": overtravel,
        "contact_pad_centers_hand_m": {
            "left": list(left_pad),
            "right": list(right_pad),
        },
        "contact_pad_center_distances_m": {
            "left": pad_distances[0],
            "right": pad_distances[1],
        },
        "fallback_distance_limit_m": fallback_limit,
        "fallback_margin_m": fallback_margin,
        "fruit_projection_between_pads": projection,
        "finger_axial_range_hand_z_m": list(finger_z_range),
        "pregrasp_clearance_m": pregrasp_clearance,
    }


def select_recommendation(
    candidates: Iterable[Mapping[str, object]],
) -> Mapping[str, object] | None:
    feasible = [candidate for candidate in candidates if candidate["feasible"]]
    if not feasible:
        return None
    return max(
        feasible,
        key=lambda candidate: (
            float(candidate["minimum_normalized_threshold_margin"]),
            -float(candidate["close_command_overtravel_m"]),
            float(candidate["palm_clearance_m"]),
            -float(candidate["tool_center_offset_m"]),
            -float(candidate["close_width_m_per_finger"]),
        ),
    )


def _finite_tuple(
    value: object,
    *,
    expected: tuple[float, ...],
    label: str,
) -> tuple[float, ...]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != len(expected)
    ):
        raise ValueError(f"{label} changed")
    converted = tuple(float(component) for component in value)
    if converted != expected or not all(math.isfinite(x) for x in converted):
        raise ValueError(f"{label} changed")
    return converted


def _bound_path(
    root: Path,
    record: Mapping[str, object],
    label: str,
) -> Path:
    root = root.resolve(strict=True)
    path = (root / str(record.get("path"))).resolve(strict=True)
    if root not in path.parents:
        raise ValueError(f"{label} escapes repository")
    if (
        path.stat().st_size != int(record.get("size_bytes", -1))
        or sha256(path) != record.get("sha256")
    ):
        raise ValueError(f"{label} binding changed")
    return path


def _external_bound_path(
    record: Mapping[str, object],
    label: str,
) -> Path:
    path = Path(str(record.get("path"))).resolve(strict=True)
    if (
        path.stat().st_size != int(record.get("size_bytes", -1))
        or sha256(path) != record.get("sha256")
    ):
        raise ValueError(f"{label} external binding changed")
    return path


def load_frozen_contract(path: Path, repository_root: Path) -> dict:
    contract_path = path.resolve(strict=True)
    root = repository_root.resolve(strict=True)
    if root not in contract_path.parents:
        raise ValueError("gripper geometry contract must be in repository")
    raw = json.loads(contract_path.read_text(encoding="utf-8"))
    if (
        raw.get("schema_version") != 1
        or raw.get("gate_id") != GATE_ID
        or raw.get("scope") != GATE_SCOPE
        or raw.get("status") != "FROZEN_EXECUTION_AUTHORIZED"
    ):
        raise ValueError("unexpected gripper geometry contract")
    decision = _bound_path(root, raw["decision_record"], "decision_record")
    if set(raw.get("bindings", {})) != EXPECTED_LOCAL_BINDINGS:
        raise ValueError("local gripper geometry bindings changed")
    if set(raw.get("external_bindings", {})) != EXPECTED_EXTERNAL_BINDINGS:
        raise ValueError("external gripper geometry bindings changed")
    resolved: dict[str, Path] = {"decision_record": decision}
    for label, record in raw["bindings"].items():
        resolved[label] = _bound_path(root, record, label)
    for label, record in raw["external_bindings"].items():
        resolved[label] = _external_bound_path(record, label)

    grid = raw.get("candidate_grid", {})
    _finite_tuple(
        grid.get("tool_center_offsets_m"),
        expected=EXPECTED_TOOL_OFFSETS_M,
        label="candidate tool offsets",
    )
    _finite_tuple(
        grid.get("close_widths_m_per_finger"),
        expected=EXPECTED_CLOSE_WIDTHS_M,
        label="candidate close widths",
    )
    if (
        grid.get("order") != "tool_offset_then_close_width"
        or int(grid.get("expected_candidate_count", 0)) != 24
    ):
        raise ValueError("candidate grid order changed")

    geometry = raw.get("geometry", {})
    expected_geometry = {
        "fruit_radius_m": 0.026,
        "finger_joint_z_m": 0.0584,
        "contact_pad_z_from_finger_m": 0.027,
        "open_width_m_per_finger": 0.04,
        "geometric_contact_margin_m": 0.003,
        "pregrasp_offset_m": 0.15,
        "wrist_housing_center_hand_m": [-0.065, 0.0, 0.045],
        "wrist_housing_size_m": [0.055, 0.080, 0.040],
        "wrist_housing_pitch_rad": -1.5707963268,
    }
    if geometry != expected_geometry:
        raise ValueError("frozen gripper geometry changed")

    thresholds = raw.get("thresholds", {})
    expected_thresholds = {
        "minimum_palm_clearance_m": 0.003,
        "minimum_wrist_housing_clearance_m": 0.010,
        "minimum_open_finger_clearance_m": 0.010,
        "maximum_contact_symmetry_error_m": 0.000001,
        "minimum_close_overtravel_m": 0.002,
        "maximum_close_overtravel_m": 0.010,
        "minimum_fallback_margin_m": 0.0005,
        "minimum_pregrasp_clearance_m": 0.100,
        "minimum_feasible_candidates": 1,
    }
    if thresholds != expected_thresholds:
        raise ValueError("frozen gripper thresholds changed")

    current = raw.get("current_production_parameters", {})
    if current != {
        "tool_center_offset_m": 0.1054,
        "close_width_m_per_finger": 0.025,
    }:
        raise ValueError("current production comparison changed")
    safety = raw.get("safety", {})
    if any(
        safety.get(key) is not False
        for key in (
            "formal_acceptance",
            "formal_held_out_test_access_authorized",
            "robot_motion_authorized",
            "gripper_command_authorized",
            "simulated_fruit_motion_authorized",
            "attachment_authorized",
            "planning_authorized",
            "pick_action_authorized",
        )
    ):
        raise ValueError("unsafe gripper geometry authorization")
    execution = raw.get("execution", {})
    if execution != {
        "maximum_claims": 1,
        "retry_authorized": False,
        "output_directory": (
            "results/development/blender_v2_gripper_geometry_sweep_v1"
        ),
    }:
        raise ValueError("gripper geometry execution boundary changed")

    raw["_contract_path"] = str(contract_path)
    raw["_contract_sha256"] = sha256(contract_path)
    raw["_resolved_paths"] = {
        label: str(resolved_path)
        for label, resolved_path in resolved.items()
    }
    return raw
