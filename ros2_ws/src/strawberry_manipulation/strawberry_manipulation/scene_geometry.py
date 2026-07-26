"""Collision geometry shared by MoveIt runtime code and source tests.

The coordinates mirror the Gazebo assets, with explicit conservative padding
where motion planning needs clearance from contact-rich simulation geometry.
Keeping the specification dependency-free makes drift checks runnable without
ROS 2.
"""

from __future__ import annotations

from dataclasses import dataclass


Vector3 = tuple[float, float, float]
TABLE_TOP_PADDING_M = 0.05
FRUIT_COLLISION_RADIUS_M = 0.026


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


STATIC_COLLISION_OBJECTS = (
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
