"""Load scene-bound Panda grasp geometry without changing old scene assets."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping


@dataclass(frozen=True)
class GraspGeometryProfile:
    profile_id: str
    world_name: str
    fruit_collision_radius_m: float
    tool_center_offset_m: float
    gripper_open_width_m_per_finger: float
    gripper_closed_width_m_per_finger: float
    gripper_position_tolerance_m_per_finger: float

    def __post_init__(self) -> None:
        values = (
            self.fruit_collision_radius_m,
            self.tool_center_offset_m,
            self.gripper_open_width_m_per_finger,
            self.gripper_closed_width_m_per_finger,
            self.gripper_position_tolerance_m_per_finger,
        )
        if not self.profile_id or not self.world_name:
            raise ValueError("grasp profile identity must be non-empty")
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("grasp geometry values must be positive and finite")
        if self.gripper_open_width_m_per_finger > 0.04:
            raise ValueError("Panda open width cannot exceed 0.04 m per finger")
        if (
            self.gripper_closed_width_m_per_finger
            >= self.fruit_collision_radius_m
        ):
            raise ValueError("closed width must command contact over-travel")
        if (
            self.gripper_closed_width_m_per_finger
            >= self.gripper_open_width_m_per_finger
        ):
            raise ValueError("closed width must be below open width")


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def profiles_from_mapping(
    data: Mapping[str, Any],
) -> tuple[GraspGeometryProfile, ...]:
    if int(data.get("schema_version", 0)) != 1:
        raise ValueError("unsupported grasp geometry schema")
    rows = data.get("profiles")
    if not isinstance(rows, list) or not rows:
        raise ValueError("grasp geometry profiles must be a non-empty list")
    profiles = []
    for index, raw in enumerate(rows):
        row = _mapping(raw, f"profiles[{index}]")
        profiles.append(
            GraspGeometryProfile(
                profile_id=str(row.get("profile_id", "")).strip(),
                world_name=str(row.get("world_name", "")).strip(),
                fruit_collision_radius_m=float(
                    row.get("fruit_collision_radius_m", 0.0)
                ),
                tool_center_offset_m=float(
                    row.get("tool_center_offset_m", 0.0)
                ),
                gripper_open_width_m_per_finger=float(
                    row.get("gripper_open_width_m_per_finger", 0.0)
                ),
                gripper_closed_width_m_per_finger=float(
                    row.get("gripper_closed_width_m_per_finger", 0.0)
                ),
                gripper_position_tolerance_m_per_finger=float(
                    row.get(
                        "gripper_position_tolerance_m_per_finger",
                        0.0,
                    )
                ),
            )
        )
    identities = [profile.profile_id for profile in profiles]
    keys = [
        (profile.world_name, profile.fruit_collision_radius_m)
        for profile in profiles
    ]
    if len(identities) != len(set(identities)):
        raise ValueError("grasp profile IDs must be unique")
    if len(keys) != len(set(keys)):
        raise ValueError("grasp scene/radius bindings must be unique")
    return tuple(profiles)


def load_grasp_geometry(
    path: str | Path,
    *,
    world_name: str,
    fruit_collision_radius_m: float,
) -> GraspGeometryProfile:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("loading grasp geometry requires PyYAML") from exc
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    profiles = profiles_from_mapping(_mapping(data, "grasp geometry"))
    matches = [
        profile
        for profile in profiles
        if (
            profile.world_name == world_name
            and abs(
                profile.fruit_collision_radius_m
                - float(fruit_collision_radius_m)
            )
            <= 1.0e-9
        )
    ]
    if len(matches) != 1:
        raise ValueError(
            "grasp geometry requires exactly one matching world/radius profile"
        )
    return matches[0]
