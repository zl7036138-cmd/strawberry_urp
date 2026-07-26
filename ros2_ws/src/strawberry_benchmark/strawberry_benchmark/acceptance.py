"""Acceptance-gate evaluation against the T00 project thresholds."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import math
from pathlib import Path
from typing import Any, Mapping

from .metrics import BenchmarkMetrics
from .yaml_subset import load_yaml_mapping


class GateStatus(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    NOT_EVALUATED = "NOT_EVALUATED"


@dataclass(frozen=True)
class GateDefinition:
    threshold_key: str
    metric_name: str
    comparator: str


GATE_DEFINITIONS = (
    GateDefinition("perception_macro_f1_min", "perception_macro_f1", ">="),
    GateDefinition(
        "localization_median_error_mm_max",
        "localization_median_error_mm",
        "<=",
    ),
    GateDefinition(
        "localization_p95_error_mm_max", "localization_p95_error_mm", "<="
    ),
    GateDefinition("oracle_success_rate_min", "oracle_success_rate", ">="),
    GateDefinition(
        "end_to_end_success_rate_min", "end_to_end_success_rate", ">="
    ),
    GateDefinition("false_pick_rate_max", "false_pick_rate", "<="),
    GateDefinition("planning_p95_sec_max", "planning_p95_sec", "<="),
)


@dataclass(frozen=True)
class GateResult:
    name: str
    metric_name: str
    comparator: str
    threshold: float
    observed: float | None
    status: GateStatus

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["status"] = self.status.value
        return data


@dataclass(frozen=True)
class AcceptanceReport:
    status: GateStatus
    gates: tuple[GateResult, ...]

    @property
    def passed(self) -> bool:
        return self.status is GateStatus.PASS

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status.value,
            "passed": self.passed,
            "gates": [gate.to_dict() for gate in self.gates],
        }


def load_acceptance_thresholds(path: str | Path) -> dict[str, float]:
    raw = load_yaml_mapping(path)
    if not isinstance(raw, Mapping) or not isinstance(raw.get("acceptance"), Mapping):
        raise ValueError("project config must contain an 'acceptance' mapping")
    acceptance = raw["acceptance"]
    required = {definition.threshold_key for definition in GATE_DEFINITIONS}
    missing = sorted(required - set(acceptance))
    if missing:
        raise ValueError(f"project acceptance config is missing: {', '.join(missing)}")
    thresholds: dict[str, float] = {}
    for name in sorted(required):
        value = acceptance[name]
        if isinstance(value, bool):
            raise ValueError(f"acceptance threshold {name} must be numeric")
        try:
            converted = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"acceptance threshold {name} must be numeric") from exc
        if not math.isfinite(converted):
            raise ValueError(f"acceptance threshold {name} must be finite")
        thresholds[name] = converted
    return thresholds


def evaluate_acceptance(
    metrics: BenchmarkMetrics, thresholds: Mapping[str, float]
) -> AcceptanceReport:
    """Compare metrics with all fixed gates.

    Missing observations are explicitly ``NOT_EVALUATED``.  The overall report
    is ``FAIL`` if any gate fails, ``NOT_EVALUATED`` if no gate fails but at
    least one is missing, and ``PASS`` only when every gate passes.
    """

    missing = [
        definition.threshold_key
        for definition in GATE_DEFINITIONS
        if definition.threshold_key not in thresholds
    ]
    if missing:
        raise ValueError(f"acceptance thresholds are missing: {', '.join(missing)}")

    gate_results: list[GateResult] = []
    for definition in GATE_DEFINITIONS:
        threshold = float(thresholds[definition.threshold_key])
        observed = getattr(metrics, definition.metric_name)
        if observed is None:
            status = GateStatus.NOT_EVALUATED
        elif definition.comparator == ">=":
            status = GateStatus.PASS if observed >= threshold else GateStatus.FAIL
        else:
            status = GateStatus.PASS if observed <= threshold else GateStatus.FAIL
        gate_results.append(
            GateResult(
                name=definition.threshold_key,
                metric_name=definition.metric_name,
                comparator=definition.comparator,
                threshold=threshold,
                observed=observed,
                status=status,
            )
        )

    statuses = {gate.status for gate in gate_results}
    if GateStatus.FAIL in statuses:
        overall = GateStatus.FAIL
    elif GateStatus.NOT_EVALUATED in statuses:
        overall = GateStatus.NOT_EVALUATED
    else:
        overall = GateStatus.PASS
    return AcceptanceReport(status=overall, gates=tuple(gate_results))
