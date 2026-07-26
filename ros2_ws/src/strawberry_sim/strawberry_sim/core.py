"""Pure simulation contract logic with no ROS or Gazebo imports."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping


VALID_MATURITIES = frozenset({"RIPE", "UNRIPE"})
VALID_ATTACHMENT_STATES = {
    "attached": True,
    "detached": False,
}


def parse_attachment_state(value: str) -> bool:
    """Convert the DetachableJoint StringMsg payload to a strict Boolean."""

    normalized = str(value).strip().lower()
    try:
        return VALID_ATTACHMENT_STATES[normalized]
    except KeyError as exc:
        raise ValueError(f"unknown detachable-joint state: {value!r}") from exc


def _finite(value: Any, field: str) -> float:
    import math

    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


@dataclass(frozen=True)
class Pose3D:
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    @classmethod
    def from_sequence(cls, values: Any) -> "Pose3D":
        if not isinstance(values, (list, tuple)) or len(values) not in (3, 7):
            raise ValueError("pose must contain [x, y, z] or [x, y, z, qx, qy, qz, qw]")
        numbers = [_finite(value, "pose") for value in values]
        if len(numbers) == 3:
            return cls(*numbers)
        pose = cls(*numbers)
        if pose.qx**2 + pose.qy**2 + pose.qz**2 + pose.qw**2 <= 1e-12:
            raise ValueError("pose quaternion must be non-zero")
        return pose


def dual_pad_geometric_contact(
    fruit: Pose3D,
    left_pad: Pose3D,
    right_pad: Pose3D,
    max_center_distance_m: float,
) -> bool:
    """Return whether two pad centres form a geometric rigid-fruit grasp.

    Gazebo contact sensors remain the primary source.  This deterministic
    fallback is intentionally strict: each pad centre must enter the configured
    fruit envelope and the fruit centre must project between the two pads.
    """

    import math

    maximum = _finite(max_center_distance_m, "max_center_distance_m")
    if maximum <= 0.0:
        raise ValueError("maximum pad-centre distance must be positive")
    fruit_xyz = (fruit.x, fruit.y, fruit.z)
    left_xyz = (left_pad.x, left_pad.y, left_pad.z)
    right_xyz = (right_pad.x, right_pad.y, right_pad.z)
    if math.dist(fruit_xyz, left_xyz) > maximum:
        return False
    if math.dist(fruit_xyz, right_xyz) > maximum:
        return False
    separation = tuple(
        right_xyz[index] - left_xyz[index] for index in range(3)
    )
    separation_squared = sum(value * value for value in separation)
    if separation_squared <= 1e-12:
        return False
    fruit_from_left = tuple(
        fruit_xyz[index] - left_xyz[index] for index in range(3)
    )
    projection = sum(
        fruit_from_left[index] * separation[index] for index in range(3)
    ) / separation_squared
    return 0.0 <= projection <= 1.0


@dataclass(frozen=True)
class FruitSpec:
    target_id: int
    model_name: str
    maturity: str
    initial_pose: Pose3D

    @property
    def pose_tf_topic(self) -> str:
        return f"/strawberry/sim/fruit_{self.target_id}/pose_tf"

    @property
    def ground_truth_pose_topic(self) -> str:
        return f"/strawberry/ground_truth/fruit_{self.target_id}/pose"


@dataclass(frozen=True)
class BinBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float

    def __post_init__(self) -> None:
        for axis in "xyz":
            minimum = getattr(self, f"min_{axis}")
            maximum = getattr(self, f"max_{axis}")
            if minimum >= maximum:
                raise ValueError(f"bin min_{axis} must be smaller than max_{axis}")

    def contains(self, pose: Pose3D) -> bool:
        return (
            self.min_x <= pose.x <= self.max_x
            and self.min_y <= pose.y <= self.max_y
            and self.min_z <= pose.z <= self.max_z
        )


@dataclass(frozen=True)
class SceneConfig:
    schema_version: int
    world_name: str
    base_frame: str
    camera_optical_frame: str
    fruit_collision_radius_m: float
    fruits: tuple[FruitSpec, ...]
    bin_bounds: BinBounds
    bin_stability_sec: float

    def __post_init__(self) -> None:
        if self.schema_version != 1:
            raise ValueError(f"unsupported scene schema_version {self.schema_version}")
        if not self.world_name or not self.base_frame or not self.camera_optical_frame:
            raise ValueError("world and frame names must be non-empty")
        if self.fruit_collision_radius_m <= 0.0:
            raise ValueError("fruit collision radius must be positive")
        if not self.fruits:
            raise ValueError("scene must contain at least one fruit")
        target_ids = [fruit.target_id for fruit in self.fruits]
        model_names = [fruit.model_name for fruit in self.fruits]
        if any(target_id <= 0 for target_id in target_ids):
            raise ValueError("target_id values must be positive")
        if len(target_ids) != len(set(target_ids)):
            raise ValueError("target_id values must be unique")
        if len(model_names) != len(set(model_names)):
            raise ValueError("fruit model names must be unique")
        if self.bin_stability_sec <= 0.0:
            raise ValueError("bin stability duration must be positive")

    @property
    def ordered_fruits(self) -> tuple[FruitSpec, ...]:
        return tuple(sorted(self.fruits, key=lambda fruit: fruit.target_id))

    def fruit(self, target_id: int) -> FruitSpec:
        for fruit in self.fruits:
            if fruit.target_id == target_id:
                return fruit
        raise KeyError(target_id)


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    return value


def scene_config_from_mapping(data: Mapping[str, Any]) -> SceneConfig:
    frames = _mapping(data.get("frames"), "frames")
    bin_data = _mapping(data.get("bin"), "bin")
    bounds_data = _mapping(bin_data.get("interior_bounds_m"), "bin.interior_bounds_m")
    fruit_rows = data.get("fruits")
    if not isinstance(fruit_rows, list):
        raise ValueError("fruits must be a list")

    fruits: list[FruitSpec] = []
    for index, raw in enumerate(fruit_rows):
        row = _mapping(raw, f"fruits[{index}]")
        maturity = str(row.get("maturity", "")).upper()
        if maturity not in VALID_MATURITIES:
            raise ValueError(f"fruits[{index}].maturity must be RIPE or UNRIPE")
        model_name = str(row.get("model_name", "")).strip()
        if not model_name:
            raise ValueError(f"fruits[{index}].model_name must be non-empty")
        fruits.append(
            FruitSpec(
                target_id=int(row.get("target_id")),
                model_name=model_name,
                maturity=maturity,
                initial_pose=Pose3D.from_sequence(row.get("initial_pose_m")),
            )
        )

    bounds = BinBounds(
        min_x=_finite(bounds_data.get("min_x"), "bin.min_x"),
        max_x=_finite(bounds_data.get("max_x"), "bin.max_x"),
        min_y=_finite(bounds_data.get("min_y"), "bin.min_y"),
        max_y=_finite(bounds_data.get("max_y"), "bin.max_y"),
        min_z=_finite(bounds_data.get("min_z"), "bin.min_z"),
        max_z=_finite(bounds_data.get("max_z"), "bin.max_z"),
    )
    return SceneConfig(
        schema_version=int(data.get("schema_version", 0)),
        world_name=str(data.get("world_name", "")).strip(),
        base_frame=str(frames.get("robot_base", "")).strip(),
        camera_optical_frame=str(frames.get("camera_optical", "")).strip(),
        fruit_collision_radius_m=_finite(
            # Schema-v1 manifests predate this explicit field.  Preserve the
            # hash-frozen tabletop-v1 contract with its historical 35 mm
            # sphere while requiring the canonical v2 scene to override it.
            data.get("fruit_collision_radius_m", 0.035),
            "fruit_collision_radius_m",
        ),
        fruits=tuple(fruits),
        bin_bounds=bounds,
        bin_stability_sec=_finite(
            bin_data.get("required_stability_sec"), "bin.required_stability_sec"
        ),
    )


def load_scene_config(path: str | Path) -> SceneConfig:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            "loading scene.yaml requires PyYAML (Ubuntu package python3-yaml)"
        ) from exc
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return scene_config_from_mapping(_mapping(data, "scene config"))


class BinStabilityTracker:
    """Track uninterrupted simulated time spent inside fixed bin bounds."""

    def __init__(self, bounds: BinBounds, required_stability_sec: float) -> None:
        if required_stability_sec <= 0.0:
            raise ValueError("required stability must be positive")
        self.bounds = bounds
        self.required_stability_sec = float(required_stability_sec)
        self._entered_at: dict[int, float] = {}
        self._last_stamp: dict[int, float] = {}
        self._latest_pose: dict[int, Pose3D] = {}

    def update(self, target_id: int, pose: Pose3D, stamp_sec: float) -> bool:
        if target_id <= 0:
            raise ValueError("target_id must be positive")
        stamp = _finite(stamp_sec, "stamp_sec")
        previous_stamp = self._last_stamp.get(target_id)
        if previous_stamp is not None and stamp < previous_stamp:
            self._entered_at.pop(target_id, None)
        self._last_stamp[target_id] = stamp
        self._latest_pose[target_id] = pose

        if not self.bounds.contains(pose):
            self._entered_at.pop(target_id, None)
            return False
        self._entered_at.setdefault(target_id, stamp)
        return stamp - self._entered_at[target_id] >= self.required_stability_sec

    def is_stable(self, target_id: int) -> bool:
        stamp = self._last_stamp.get(target_id)
        entered = self._entered_at.get(target_id)
        pose = self._latest_pose.get(target_id)
        return bool(
            stamp is not None
            and entered is not None
            and pose is not None
            and self.bounds.contains(pose)
            and stamp - entered >= self.required_stability_sec
        )

    def has_pose(self, target_id: int) -> bool:
        return target_id in self._latest_pose

    def reset(self, target_id: int) -> None:
        self._entered_at.pop(target_id, None)
        self._last_stamp.pop(target_id, None)
        self._latest_pose.pop(target_id, None)


@dataclass(frozen=True)
class AttachmentDecision:
    allowed: bool
    reason: str


class AttachmentGate:
    """Pure validation for a physical attach request.

    The gate never treats missing state or contact as success. Contact timestamps
    are simulation times so paused simulations cannot age into a false success.
    """

    def __init__(self, contact_freshness_sec: float = 0.25) -> None:
        if contact_freshness_sec <= 0.0:
            raise ValueError("contact freshness must be positive")
        self.contact_freshness_sec = float(contact_freshness_sec)

    def assess_attach(
        self,
        *,
        now_sec: float,
        backend_enabled: bool,
        backend_initialized: bool,
        state_known: bool,
        attached: bool,
        left_contact: bool,
        left_stamp_sec: float | None,
        right_contact: bool,
        right_stamp_sec: float | None,
    ) -> AttachmentDecision:
        now = _finite(now_sec, "now_sec")
        if not backend_enabled:
            return AttachmentDecision(False, "attachment backend is disabled")
        if not backend_initialized:
            return AttachmentDecision(False, "attachment backend is not initialized detached")
        if not state_known:
            return AttachmentDecision(False, "attachment state is unknown")
        if attached:
            return AttachmentDecision(True, "fruit is already attached")
        contacts = (
            ("left", left_contact, left_stamp_sec),
            ("right", right_contact, right_stamp_sec),
        )
        for side, active, stamp in contacts:
            if not active or stamp is None:
                return AttachmentDecision(False, f"{side} gripper contact is not active")
            age = now - _finite(stamp, f"{side}_stamp_sec")
            if age < 0.0 or age > self.contact_freshness_sec:
                return AttachmentDecision(False, f"{side} gripper contact is stale")
        return AttachmentDecision(True, "dual gripper contact confirmed")


def normalize_entity_name(frame_id: str) -> str:
    """Return the last Gazebo scope/path component for diagnostics."""

    normalized = frame_id.strip().strip("/")
    if "::" in normalized:
        normalized = normalized.rsplit("::", 1)[-1]
    if "/" in normalized:
        normalized = normalized.rsplit("/", 1)[-1]
    return normalized


def fruit_models_in_contacts(
    collision_pairs: list[tuple[str, str]],
    model_by_target_id: Mapping[int, str],
) -> frozenset[int]:
    """Return fruit IDs mentioned by a Gazebo contact message.

    Gazebo collision names are scoped (for example
    ``strawberry_1::fruit_link::fruit_collision``). Matching complete scope
    components prevents strawberry_1 from accidentally matching strawberry_10.
    """

    if any(target_id <= 0 for target_id in model_by_target_id):
        raise ValueError("contact target IDs must be positive")
    if len(set(model_by_target_id.values())) != len(model_by_target_id):
        raise ValueError("contact model names must be unique")

    def components(name: str) -> frozenset[str]:
        return frozenset(
            part for part in name.replace("::", "/").split("/") if part
        )

    active: set[int] = set()
    for first, second in collision_pairs:
        names = components(first) | components(second)
        for target_id, model_name in model_by_target_id.items():
            if model_name in names:
                active.add(target_id)
    return frozenset(active)
