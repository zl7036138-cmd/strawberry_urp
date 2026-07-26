"""Versioned schemas shared by the benchmark generator and trial runner.

This module deliberately contains no ROS imports.  A future ROS trial runner can
translate messages/actions into :class:`TrialResult` without coupling report
generation to a ROS installation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Any, Mapping


SCHEMA_VERSION = 1


class Maturity(str, Enum):
    UNKNOWN = "UNKNOWN"
    RIPE = "RIPE"
    UNRIPE = "UNRIPE"


class ScenarioKind(str, Enum):
    POSITIVE = "POSITIVE"
    NEGATIVE = "NEGATIVE"


class TrialMode(str, Enum):
    ORACLE = "ORACLE"
    END_TO_END = "END_TO_END"


class FailureStage(str, Enum):
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


class FailureCode(str, Enum):
    NO_TARGET = "NO_TARGET"
    LOW_CONFIDENCE = "LOW_CONFIDENCE"
    DEPTH_INVALID = "DEPTH_INVALID"
    TF_TIMEOUT = "TF_TIMEOUT"
    UNREACHABLE = "UNREACHABLE"
    PLANNING_FAILED = "PLANNING_FAILED"
    COLLISION = "COLLISION"
    GRASP_FAILED = "GRASP_FAILED"
    PLACE_FAILED = "PLACE_FAILED"
    STALE_DATA = "STALE_DATA"


def _enum_value(enum_type: type[Enum], value: Any, field_name: str) -> Enum:
    if isinstance(value, enum_type):
        return value
    try:
        return enum_type(value)
    except (TypeError, ValueError) as exc:
        allowed = ", ".join(item.value for item in enum_type)
        raise ValueError(f"{field_name} must be one of: {allowed}") from exc


def _optional_enum_value(
    enum_type: type[Enum], value: Any, field_name: str
) -> Enum | None:
    if value is None or value == "":
        return None
    return _enum_value(enum_type, value, field_name)


def _bool_value(value: Any, field_name: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes"}:
            return True
        if normalized in {"false", "0", "no"}:
            return False
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    raise ValueError(f"{field_name} must be a boolean")


def _optional_float(value: Any, field_name: str) -> float | None:
    if value is None or value == "":
        return None
    try:
        converted = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be numeric") from exc
    if not math.isfinite(converted) or converted < 0.0:
        raise ValueError(f"{field_name} must be finite and non-negative")
    return converted


@dataclass(frozen=True)
class Scenario:
    """One deterministic condition in the robustness benchmark."""

    trial_id: str
    kind: ScenarioKind
    occlusion: str
    lighting: str
    position: str
    seed: int
    expected_maturity: Maturity
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        object.__setattr__(self, "kind", _enum_value(ScenarioKind, self.kind, "kind"))
        object.__setattr__(
            self,
            "expected_maturity",
            _enum_value(Maturity, self.expected_maturity, "expected_maturity"),
        )
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError(f"unsupported scenario schema_version: {self.schema_version}")
        for name in ("trial_id", "occlusion", "lighting", "position"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise ValueError("seed must be an integer")
        expected = (
            Maturity.RIPE if self.kind is ScenarioKind.POSITIVE else Maturity.UNRIPE
        )
        if self.expected_maturity is not expected:
            raise ValueError(
                f"{self.kind.value} scenarios must expect {expected.value} fruit"
            )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "trial_id": self.trial_id,
            "kind": self.kind.value,
            "occlusion": self.occlusion,
            "lighting": self.lighting,
            "position": self.position,
            "seed": self.seed,
            "expected_maturity": self.expected_maturity.value,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Scenario":
        return cls(
            schema_version=int(data.get("schema_version", SCHEMA_VERSION)),
            trial_id=str(data["trial_id"]),
            kind=data["kind"],
            occlusion=str(data["occlusion"]),
            lighting=str(data["lighting"]),
            position=str(data["position"]),
            seed=int(data["seed"]),
            expected_maturity=data["expected_maturity"],
        )


@dataclass(frozen=True)
class TrialResult:
    """Machine-readable result for one scenario execution.

    ``success`` means the scenario behaved as expected: a positive trial placed
    a ripe fruit, while a negative trial completed with no pick.  ``fruit_picked``
    records physical acquisition independently, allowing false-pick calculation.
    """

    trial_id: str
    mode: TrialMode
    scenario_kind: ScenarioKind
    occlusion: str
    lighting: str
    position: str
    seed: int
    expected_maturity: Maturity
    predicted_maturity: Maturity | None
    detection_confidence: float | None
    pick_attempted: bool
    fruit_picked: bool
    success: bool
    first_failure_stage: FailureStage | None = None
    failure_code: FailureCode | None = None
    localization_error_mm: float | None = None
    planning_time_sec: float | None = None
    execution_time_sec: float | None = None
    timestamp_utc: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        object.__setattr__(self, "mode", _enum_value(TrialMode, self.mode, "mode"))
        object.__setattr__(
            self,
            "scenario_kind",
            _enum_value(ScenarioKind, self.scenario_kind, "scenario_kind"),
        )
        object.__setattr__(
            self,
            "expected_maturity",
            _enum_value(Maturity, self.expected_maturity, "expected_maturity"),
        )
        object.__setattr__(
            self,
            "predicted_maturity",
            _optional_enum_value(
                Maturity, self.predicted_maturity, "predicted_maturity"
            ),
        )
        object.__setattr__(
            self,
            "first_failure_stage",
            _optional_enum_value(
                FailureStage, self.first_failure_stage, "first_failure_stage"
            ),
        )
        object.__setattr__(
            self,
            "failure_code",
            _optional_enum_value(FailureCode, self.failure_code, "failure_code"),
        )
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError(f"unsupported trial schema_version: {self.schema_version}")
        for name in ("trial_id", "occlusion", "lighting", "position"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise ValueError("seed must be an integer")
        expected = (
            Maturity.RIPE
            if self.scenario_kind is ScenarioKind.POSITIVE
            else Maturity.UNRIPE
        )
        if self.expected_maturity is not expected:
            raise ValueError(
                f"{self.scenario_kind.value} results must expect {expected.value} fruit"
            )
        if self.detection_confidence is not None:
            confidence = _optional_float(
                self.detection_confidence, "detection_confidence"
            )
            if confidence is not None and confidence > 1.0:
                raise ValueError("detection_confidence must be at most 1.0")
            object.__setattr__(self, "detection_confidence", confidence)
        for field_name in (
            "localization_error_mm",
            "planning_time_sec",
            "execution_time_sec",
        ):
            object.__setattr__(
                self, field_name, _optional_float(getattr(self, field_name), field_name)
            )
        if not isinstance(self.pick_attempted, bool):
            raise ValueError("pick_attempted must be a boolean")
        if not isinstance(self.fruit_picked, bool):
            raise ValueError("fruit_picked must be a boolean")
        if not isinstance(self.success, bool):
            raise ValueError("success must be a boolean")
        if self.fruit_picked and not self.pick_attempted:
            raise ValueError("fruit_picked cannot be true when pick_attempted is false")
        if self.success and (
            self.first_failure_stage is not None or self.failure_code is not None
        ):
            raise ValueError("successful trials cannot contain failure attribution")
        if (self.first_failure_stage is None) != (self.failure_code is None):
            raise ValueError(
                "first_failure_stage and failure_code must both be set or both be empty"
            )
        if not isinstance(self.metadata, Mapping):
            raise ValueError("metadata must be a mapping")
        object.__setattr__(self, "metadata", dict(self.metadata))

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "trial_id": self.trial_id,
            "mode": self.mode.value,
            "scenario_kind": self.scenario_kind.value,
            "occlusion": self.occlusion,
            "lighting": self.lighting,
            "position": self.position,
            "seed": self.seed,
            "expected_maturity": self.expected_maturity.value,
            "predicted_maturity": (
                self.predicted_maturity.value if self.predicted_maturity else None
            ),
            "detection_confidence": self.detection_confidence,
            "pick_attempted": self.pick_attempted,
            "fruit_picked": self.fruit_picked,
            "success": self.success,
            "first_failure_stage": (
                self.first_failure_stage.value if self.first_failure_stage else None
            ),
            "failure_code": self.failure_code.value if self.failure_code else None,
            "localization_error_mm": self.localization_error_mm,
            "planning_time_sec": self.planning_time_sec,
            "execution_time_sec": self.execution_time_sec,
            "timestamp_utc": self.timestamp_utc,
            "metadata": dict(self.metadata),
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "TrialResult":
        metadata = data.get("metadata", {})
        return cls(
            schema_version=int(data.get("schema_version", SCHEMA_VERSION)),
            trial_id=str(data["trial_id"]),
            mode=data["mode"],
            scenario_kind=data["scenario_kind"],
            occlusion=str(data["occlusion"]),
            lighting=str(data["lighting"]),
            position=str(data["position"]),
            seed=int(data["seed"]),
            expected_maturity=data["expected_maturity"],
            predicted_maturity=data.get("predicted_maturity"),
            detection_confidence=data.get("detection_confidence"),
            pick_attempted=_bool_value(data["pick_attempted"], "pick_attempted"),
            fruit_picked=_bool_value(data["fruit_picked"], "fruit_picked"),
            success=_bool_value(data["success"], "success"),
            first_failure_stage=data.get("first_failure_stage"),
            failure_code=data.get("failure_code"),
            localization_error_mm=data.get("localization_error_mm"),
            planning_time_sec=data.get("planning_time_sec"),
            execution_time_sec=data.get("execution_time_sec"),
            timestamp_utc=data.get("timestamp_utc") or None,
            metadata=metadata,
        )
