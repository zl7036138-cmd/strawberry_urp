"""Finite, explicit orchestration state machine independent of ROS."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
from typing import Any


class State(str, Enum):
    IDLE = "IDLE"
    INIT = "INIT"
    ACQUIRE = "ACQUIRE"
    DETECT = "DETECT"
    LOCALIZE = "LOCALIZE"
    SELECT = "SELECT"
    PLAN = "PLAN"
    APPROACH = "APPROACH"
    GRASP = "GRASP"
    RETREAT = "RETREAT"
    PLACE = "PLACE"
    VERIFY = "VERIFY"
    DONE = "DONE"
    FAILED = "FAILED"


class Event(str, Enum):
    START = "START"
    READY = "READY"
    FRAME = "FRAME"
    DETECTIONS = "DETECTIONS"
    TARGET = "TARGET"
    NO_PICK = "NO_PICK"
    PLANNED = "PLANNED"
    APPROACHED = "APPROACHED"
    GRASPED = "GRASPED"
    RETREATED = "RETREATED"
    PLACED = "PLACED"
    VERIFIED = "VERIFIED"
    RETRY_SENSOR = "RETRY_SENSOR"
    ERROR = "ERROR"
    RESET = "RESET"


class TargetSource(str, Enum):
    ORACLE = "oracle"
    PERCEPTION = "perception"


TRANSITIONS: dict[tuple[State, Event], State] = {
    (State.IDLE, Event.START): State.INIT,
    (State.INIT, Event.READY): State.ACQUIRE,
    (State.ACQUIRE, Event.NO_PICK): State.DONE,
    (State.ACQUIRE, Event.FRAME): State.DETECT,
    (State.DETECT, Event.DETECTIONS): State.LOCALIZE,
    (State.DETECT, Event.NO_PICK): State.DONE,
    (State.LOCALIZE, Event.TARGET): State.SELECT,
    (State.SELECT, Event.TARGET): State.PLAN,
    (State.PLAN, Event.PLANNED): State.APPROACH,
    (State.APPROACH, Event.APPROACHED): State.GRASP,
    (State.GRASP, Event.GRASPED): State.RETREAT,
    (State.RETREAT, Event.RETREATED): State.PLACE,
    (State.PLACE, Event.PLACED): State.VERIFY,
    (State.VERIFY, Event.VERIFIED): State.DONE,
    (State.DONE, Event.RESET): State.IDLE,
    (State.FAILED, Event.RESET): State.IDLE,
}


@dataclass(frozen=True)
class TransitionRecord:
    previous: State
    event: Event
    current: State
    reason: str = ""


class TrialStateMachine:
    def __init__(self, sensor_retry_limit: int = 3) -> None:
        if sensor_retry_limit < 0:
            raise ValueError("sensor retry limit cannot be negative")
        self.state = State.IDLE
        self.sensor_retry_limit = sensor_retry_limit
        self.sensor_retries = 0
        self.history: list[TransitionRecord] = []

    @property
    def terminal(self) -> bool:
        return self.state in (State.DONE, State.FAILED)

    def apply(self, event: Event, reason: str = "") -> State:
        previous = self.state
        if event is Event.ERROR:
            self.state = State.FAILED
        elif event is Event.RETRY_SENSOR:
            if self.state not in (State.ACQUIRE, State.DETECT, State.LOCALIZE):
                raise ValueError(f"sensor retry is invalid in {self.state}")
            self.sensor_retries += 1
            if self.sensor_retries > self.sensor_retry_limit:
                self.state = State.FAILED
            else:
                self.state = State.ACQUIRE
        else:
            try:
                self.state = TRANSITIONS[(self.state, event)]
            except KeyError as exc:
                raise ValueError(
                    f"event {event.value} is invalid in state {self.state.value}"
                ) from exc

        self.history.append(TransitionRecord(previous, event, self.state, reason))
        return self.state


@dataclass(frozen=True)
class Candidate:
    target_id: int
    maturity: int
    confidence: float
    distance_m: float
    valid_depth: bool
    reachable: bool


@dataclass(frozen=True)
class TargetRouting:
    source: TargetSource
    control_topic: str
    shadow_enabled: bool
    shadow_topic: str


def _absolute_topic(value: Any, field_name: str) -> str:
    if not isinstance(value, str) or not value.startswith("/") or value == "/":
        raise ValueError(f"{field_name} must be an absolute ROS topic")
    if "//" in value or value.endswith("/") or any(char.isspace() for char in value):
        raise ValueError(f"{field_name} is not a canonical ROS topic")
    return value


def validate_target_routing(
    *,
    source: Any,
    control_topic: Any,
    shadow_enabled: Any,
    shadow_topic: Any,
) -> TargetRouting:
    """Validate the fail-closed control/observation topic separation."""

    try:
        normalized_source = (
            source
            if isinstance(source, TargetSource)
            else TargetSource(str(source).strip().lower())
        )
    except (TypeError, ValueError) as exc:
        allowed = ", ".join(item.value for item in TargetSource)
        raise ValueError(f"target_source must be one of: {allowed}") from exc
    if not isinstance(shadow_enabled, bool):
        raise ValueError("shadow_enabled must be a boolean")
    normalized_control = _absolute_topic(control_topic, "control_target_topic")
    normalized_shadow = _absolute_topic(shadow_topic, "shadow_target_topic")
    if shadow_enabled and normalized_control == normalized_shadow:
        raise ValueError("shadow target topic must differ from the control topic")
    return TargetRouting(
        source=normalized_source,
        control_topic=normalized_control,
        shadow_enabled=shadow_enabled,
        shadow_topic=normalized_shadow,
    )


@dataclass(frozen=True)
class OracleFruit:
    target_id: int
    maturity: str


@dataclass(frozen=True)
class OracleCatalog:
    frame_id: str
    pose_array_order: tuple[int, ...]
    fruits: tuple[OracleFruit, ...]


def parse_oracle_catalog(payload: str) -> OracleCatalog:
    """Parse the simulation truth catalog without importing ROS messages."""

    try:
        raw = json.loads(payload)
        if int(raw["schema_version"]) != 1:
            raise ValueError("unsupported oracle catalog schema version")
        frame_id = str(raw["frame_id"])
        order = tuple(int(value) for value in raw["pose_array_order"])
        fruits = tuple(
            OracleFruit(
                target_id=int(item["target_id"]),
                maturity=str(item["maturity"]).strip().upper(),
            )
            for item in raw["fruits"]
        )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid oracle catalog: {exc}") from exc
    if not frame_id:
        raise ValueError("oracle catalog frame_id cannot be empty")
    if not order or any(target_id <= 0 for target_id in order):
        raise ValueError("oracle target IDs must be positive")
    if len(order) != len(set(order)):
        raise ValueError("oracle target IDs must be unique")
    fruit_ids = tuple(fruit.target_id for fruit in fruits)
    if fruit_ids != order:
        raise ValueError("oracle fruit order must match pose_array_order")
    if any(fruit.maturity not in {"RIPE", "UNRIPE"} for fruit in fruits):
        raise ValueError("oracle maturity must be RIPE or UNRIPE")
    return OracleCatalog(frame_id=frame_id, pose_array_order=order, fruits=fruits)


def select_oracle_pose_index(
    catalog: OracleCatalog,
    pose_count: int,
    requested_target_id: int = 0,
) -> int | None:
    """Select one ripe truth target, or safely return no-pick."""

    if pose_count != len(catalog.pose_array_order):
        raise ValueError("oracle pose count does not match its catalog")
    if requested_target_id < 0:
        raise ValueError("oracle_target_id cannot be negative")
    eligible = [
        index
        for index, fruit in enumerate(catalog.fruits)
        if fruit.maturity == "RIPE"
        and (requested_target_id == 0 or fruit.target_id == requested_target_id)
    ]
    if requested_target_id and not any(
        fruit.target_id == requested_target_id for fruit in catalog.fruits
    ):
        raise ValueError("requested oracle target is absent from the catalog")
    return eligible[0] if eligible else None


def select_target(
    candidates: list[Candidate],
    *,
    ripe_value: int = 1,
    confidence_threshold: float = 0.60,
) -> Candidate | None:
    valid = [
        item
        for item in candidates
        if item.maturity == ripe_value
        and item.confidence >= confidence_threshold
        and item.valid_depth
        and item.reachable
    ]
    if not valid:
        return None
    return sorted(valid, key=lambda item: (-item.confidence, item.distance_m, item.target_id))[0]
