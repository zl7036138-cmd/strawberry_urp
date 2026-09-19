"""Collision geometry shared by MoveIt runtime code and source tests.

The coordinates mirror the Gazebo assets, with explicit conservative padding
where motion planning needs clearance from contact-rich simulation geometry.
Keeping the specification dependency-free makes drift checks runnable without
ROS 2.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


Vector3 = tuple[float, float, float]
TABLE_TOP_PADDING_M = 0.05
FRUIT_COLLISION_RADIUS_M = 0.026
PLANT_CROWN_CENTER_OFFSET_Z_M = 0.012


@dataclass(frozen=True)
class BoxPrimitive:
    """Axis-aligned box expressed in the Panda base frame, in metres."""

    center_m: Vector3
    size_m: Vector3

    def __post_init__(self) -> None:
        if len(self.center_m) != 3 or len(self.size_m) != 3:
            raise ValueError("box center and size must contain three values")
        if any(float(size) <= 0.0 for size in self.size_m):
            raise ValueError("box dimensions must be positive")


@dataclass(frozen=True)
class SpherePrimitive:
    """Sphere expressed in the Panda base frame, in metres."""

    center_m: Vector3
    radius_m: float

    def __post_init__(self) -> None:
        if len(self.center_m) != 3:
            raise ValueError("sphere center must contain three values")
        if float(self.radius_m) <= 0.0:
            raise ValueError("sphere radius must be positive")


def fruit_collision_id(target_id: int) -> str:
    """Return the stable MoveIt object ID for one benchmark fruit."""

    if target_id <= 0:
        raise ValueError("target_id must be positive")
    return f"strawberry_fruit_{target_id}"


@dataclass(frozen=True)
class CollisionObjectSpec:
    """One MoveIt collision object composed of one or more boxes."""

    object_id: str
    boxes: tuple[BoxPrimitive, ...]

    def __post_init__(self) -> None:
        if not self.object_id:
            raise ValueError("collision object id must be non-empty")
        if not self.boxes:
            raise ValueError("collision object must contain at least one box")


BLENDER_V2_STATIC_COLLISION_OBJECTS = (
    CollisionObjectSpec(
        object_id="work_table",
        boxes=(
            BoxPrimitive(
                # Preserve the physical bottom while raising the planning-only
                # top surface by 5 cm to prevent joint-space finger sweeps.
                center_m=(0.58, 0.0, 0.405),
                size_m=(0.72, 0.72, 0.11),
            ),
        ),
    ),
    CollisionObjectSpec(
        object_id="strawberry_planter",
        boxes=(
            BoxPrimitive(
                # Conservative square proxy for the 0.10 m radius planter.
                center_m=(0.50, 0.0, 0.44),
                size_m=(0.20, 0.20, 0.06),
            ),
        ),
    ),
    CollisionObjectSpec(
        object_id="strawberry_plant_crown",
        boxes=(
            BoxPrimitive(
                # Conservative MoveIt proxy for the Gazebo crown cylinder.
                center_m=(0.50, 0.0, 0.482),
                size_m=(0.04, 0.04, 0.024),
            ),
        ),
    ),
    CollisionObjectSpec(
        object_id="collection_bin",
        boxes=(
            BoxPrimitive(
                center_m=(0.35, -0.45, 0.26),
                size_m=(0.38, 0.42, 0.02),
            ),
            BoxPrimitive(
                center_m=(0.15, -0.45, 0.39),
                size_m=(0.02, 0.42, 0.28),
            ),
            BoxPrimitive(
                center_m=(0.55, -0.45, 0.39),
                size_m=(0.02, 0.42, 0.28),
            ),
            BoxPrimitive(
                center_m=(0.35, -0.67, 0.39),
                size_m=(0.42, 0.02, 0.28),
            ),
            BoxPrimitive(
                center_m=(0.35, -0.23, 0.39),
                size_m=(0.42, 0.02, 0.28),
            ),
        ),
    ),
)

# The field asset is expressed directly in panda_link0 coordinates. Keep each
# Gazebo collision as its own MoveIt object so a live planning-scene audit can
# identify a missing ridge instead of hiding it inside a compound object.
FIELD_V3_STATIC_COLLISION_OBJECTS = (
    CollisionObjectSpec(
        object_id="field_ground",
        boxes=(
            BoxPrimitive(
                center_m=(2.699999809, 0.150000095, -0.100000001),
                size_m=(5.000000000, 12.000000000, 0.200000003),
            ),
        ),
    ),
    CollisionObjectSpec(
        object_id="field_ridge_1",
        boxes=(
            BoxPrimitive(
                center_m=(0.637499988, 0.150000095, 0.140000001),
                size_m=(0.874999940, 11.500000000, 0.280000001),
            ),
        ),
    ),
    CollisionObjectSpec(
        object_id="field_ridge_2",
        boxes=(
            BoxPrimitive(
                center_m=(2.700000048, 0.150000095, 0.140000001),
                size_m=(1.049999714, 11.500000000, 0.280000001),
            ),
        ),
    ),
    CollisionObjectSpec(
        object_id="field_ridge_3",
        boxes=(
            BoxPrimitive(
                center_m=(4.850000381, 0.150000095, 0.140000001),
                size_m=(1.049999714, 11.500000000, 0.280000001),
            ),
        ),
    ),
    CollisionObjectSpec(
        object_id="strawberry_plant_crown",
        boxes=(
            BoxPrimitive(
                center_m=(0.55, 0.0, 0.312),
                size_m=(0.04, 0.04, 0.024),
            ),
        ),
    ),
    CollisionObjectSpec(
        object_id="collection_bin",
        boxes=(
            BoxPrimitive(
                center_m=(-0.55, 0.35, 0.26),
                size_m=(0.38, 0.42, 0.02),
            ),
            BoxPrimitive(
                center_m=(-0.75, 0.35, 0.39),
                size_m=(0.02, 0.42, 0.28),
            ),
            BoxPrimitive(
                center_m=(-0.35, 0.35, 0.39),
                size_m=(0.02, 0.42, 0.28),
            ),
            BoxPrimitive(
                center_m=(-0.55, 0.13, 0.39),
                size_m=(0.42, 0.02, 0.28),
            ),
            BoxPrimitive(
                center_m=(-0.55, 0.57, 0.39),
                size_m=(0.42, 0.02, 0.28),
            ),
        ),
    ),
)

# Backward-compatible name for the accepted default Blender-v2 scene.
STATIC_COLLISION_OBJECTS = BLENDER_V2_STATIC_COLLISION_OBJECTS

STATIC_COLLISION_PROFILES = {
    "blender_v2": BLENDER_V2_STATIC_COLLISION_OBJECTS,
    "field_v3": FIELD_V3_STATIC_COLLISION_OBJECTS,
}


def static_collision_objects(
    profile_id: str,
    *,
    plant_positions_m: tuple[Vector3, ...] | None = None,
) -> tuple[CollisionObjectSpec, ...]:
    """Resolve a fail-closed, scene-bound static collision profile."""

    normalized = str(profile_id).strip()
    try:
        profile = STATIC_COLLISION_PROFILES[normalized]
    except KeyError as exc:
        raise ValueError(
            f"unknown static collision profile: {profile_id!r}"
        ) from exc
    if plant_positions_m is None:
        return profile
    positions = tuple(tuple(float(value) for value in row) for row in plant_positions_m)
    if any(
        len(row) != 3 or not all(math.isfinite(value) for value in row)
        for row in positions
    ):
        raise ValueError("plant positions must contain finite xyz triples")
    crown_templates = tuple(
        specification
        for specification in profile
        if specification.object_id == "strawberry_plant_crown"
    )
    if not positions:
        return profile
    if len(crown_templates) != 1 or len(crown_templates[0].boxes) != 1:
        raise ValueError("static collision profile must contain one plant crown box")
    crown_size = crown_templates[0].boxes[0].size_m
    resolved: list[CollisionObjectSpec] = []
    for specification in profile:
        if specification.object_id != "strawberry_plant_crown":
            resolved.append(specification)
            continue
        for index, (x, y, z) in enumerate(positions, start=1):
            resolved.append(
                CollisionObjectSpec(
                    object_id=(
                        "strawberry_plant_crown"
                        if len(positions) == 1
                        else f"strawberry_plant_crown_{index}"
                    ),
                    boxes=(
                        BoxPrimitive(
                            center_m=(x, y, z + PLANT_CROWN_CENTER_OFFSET_Z_M),
                            size_m=crown_size,
                        ),
                    ),
                )
            )
    return tuple(resolved)
