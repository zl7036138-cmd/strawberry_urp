"""Deterministic aggregate metrics for version-1 benchmark logs."""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass
import math
from typing import Any, Iterable, Sequence

from .models import Maturity, ScenarioKind, TrialMode, TrialResult


def _rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * percentile
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _perception_macro_f1(results: Sequence[TrialResult]) -> tuple[int, float | None]:
    candidates = [result for result in results if result.mode is TrialMode.END_TO_END]
    true_support = Counter(result.expected_maturity for result in candidates)
    classes = (Maturity.RIPE, Maturity.UNRIPE)
    if any(true_support[maturity] == 0 for maturity in classes):
        return len(candidates), None

    per_class: list[float] = []
    for maturity in classes:
        true_positive = sum(
            result.expected_maturity is maturity
            and result.predicted_maturity is maturity
            for result in candidates
        )
        false_positive = sum(
            result.expected_maturity is not maturity
            and result.predicted_maturity is maturity
            for result in candidates
        )
        false_negative = sum(
            result.expected_maturity is maturity
            and result.predicted_maturity is not maturity
            for result in candidates
        )
        denominator = 2 * true_positive + false_positive + false_negative
        per_class.append(2 * true_positive / denominator if denominator else 0.0)
    return len(candidates), sum(per_class) / len(per_class)


@dataclass(frozen=True)
class BenchmarkMetrics:
    total_trials: int
    positive_trials: int
    negative_trials: int
    positive_success_rate: float | None
    oracle_positive_trials: int
    oracle_success_rate: float | None
    end_to_end_positive_trials: int
    end_to_end_success_rate: float | None
    end_to_end_negative_trials: int
    false_pick_count: int
    false_pick_rate: float | None
    perception_evaluated_trials: int
    perception_macro_f1: float | None
    localization_sample_count: int
    localization_median_error_mm: float | None
    localization_p95_error_mm: float | None
    planning_sample_count: int
    planning_p95_sec: float | None
    failed_trials: int
    unattributed_failure_count: int
    failure_by_stage: dict[str, int]
    failure_by_code: dict[str, int]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def compute_metrics(results: Iterable[TrialResult]) -> BenchmarkMetrics:
    """Aggregate validated trial results.

    The false-pick denominator contains end-to-end negative trials only.  A
    false pick requires ``fruit_picked=true``; arm motion by itself is not
    counted as a physical pick.  Perception F1 uses end-to-end results and treats
    a missing/UNKNOWN prediction as a false negative for the expected class.
    """

    records = list(results)
    trial_ids = [result.trial_id for result in records]
    if len(trial_ids) != len(set(trial_ids)):
        duplicates = sorted(
            trial_id for trial_id, count in Counter(trial_ids).items() if count > 1
        )
        raise ValueError(f"duplicate trial_id values: {', '.join(duplicates)}")

    positive = [
        result for result in records if result.scenario_kind is ScenarioKind.POSITIVE
    ]
    negative = [
        result for result in records if result.scenario_kind is ScenarioKind.NEGATIVE
    ]
    oracle_positive = [
        result for result in positive if result.mode is TrialMode.ORACLE
    ]
    end_to_end_positive = [
        result for result in positive if result.mode is TrialMode.END_TO_END
    ]
    end_to_end_negative = [
        result for result in negative if result.mode is TrialMode.END_TO_END
    ]
    false_picks = sum(result.fruit_picked for result in end_to_end_negative)

    localization_errors = [
        result.localization_error_mm
        for result in records
        if result.localization_error_mm is not None
    ]
    planning_times = [
        result.planning_time_sec
        for result in records
        if result.planning_time_sec is not None
    ]
    failed = [result for result in records if not result.success]
    by_stage = Counter(
        result.first_failure_stage.value
        for result in failed
        if result.first_failure_stage is not None
    )
    by_code = Counter(
        result.failure_code.value for result in failed if result.failure_code is not None
    )
    unattributed = sum(
        result.first_failure_stage is None or result.failure_code is None
        for result in failed
    )
    perception_count, perception_f1 = _perception_macro_f1(records)

    return BenchmarkMetrics(
        total_trials=len(records),
        positive_trials=len(positive),
        negative_trials=len(negative),
        positive_success_rate=_rate(sum(result.success for result in positive), len(positive)),
        oracle_positive_trials=len(oracle_positive),
        oracle_success_rate=_rate(
            sum(result.success for result in oracle_positive), len(oracle_positive)
        ),
        end_to_end_positive_trials=len(end_to_end_positive),
        end_to_end_success_rate=_rate(
            sum(result.success for result in end_to_end_positive),
            len(end_to_end_positive),
        ),
        end_to_end_negative_trials=len(end_to_end_negative),
        false_pick_count=false_picks,
        false_pick_rate=_rate(false_picks, len(end_to_end_negative)),
        perception_evaluated_trials=perception_count,
        perception_macro_f1=perception_f1,
        localization_sample_count=len(localization_errors),
        localization_median_error_mm=_percentile(localization_errors, 0.5),
        localization_p95_error_mm=_percentile(localization_errors, 0.95),
        planning_sample_count=len(planning_times),
        planning_p95_sec=_percentile(planning_times, 0.95),
        failed_trials=len(failed),
        unattributed_failure_count=unattributed,
        failure_by_stage=dict(sorted(by_stage.items())),
        failure_by_code=dict(sorted(by_code.items())),
    )
