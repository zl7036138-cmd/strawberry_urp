"""ROS-independent benchmark primitives for the Strawberry URP project."""

from .acceptance import AcceptanceReport, GateResult, GateStatus, evaluate_acceptance
from .metrics import BenchmarkMetrics, compute_metrics
from .models import (
    FailureCode,
    FailureStage,
    Maturity,
    Scenario,
    ScenarioKind,
    TrialMode,
    TrialResult,
)
from .scenarios import BenchmarkSpec, generate_scenarios, load_benchmark_spec

__all__ = [
    "AcceptanceReport",
    "BenchmarkMetrics",
    "BenchmarkSpec",
    "FailureCode",
    "FailureStage",
    "GateResult",
    "GateStatus",
    "Maturity",
    "Scenario",
    "ScenarioKind",
    "TrialMode",
    "TrialResult",
    "compute_metrics",
    "evaluate_acceptance",
    "generate_scenarios",
    "load_benchmark_spec",
]
