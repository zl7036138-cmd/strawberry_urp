"""Dependency-light scenarios and metrics for the simulator perception pre-gate."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence


RIPE = "RIPE"
UNRIPE = "UNRIPE"


@dataclass(frozen=True)
class SimPerceptionScenario:
    """One single-visible-fruit simulator observation scenario."""

    scenario_id: str
    target_id: int
    model_name: str
    expected_maturity: str
    position_m: tuple[float, float, float]

    def __post_init__(self) -> None:
        if not self.scenario_id:
            raise ValueError("scenario_id must be non-empty")
        if self.target_id <= 0:
            raise ValueError("target_id must be positive")
        if not self.model_name:
            raise ValueError("model_name must be non-empty")
        if self.expected_maturity not in {RIPE, UNRIPE}:
            raise ValueError("expected_maturity must be RIPE or UNRIPE")
        if len(self.position_m) != 3 or not all(
            math.isfinite(value) for value in self.position_m
        ):
            raise ValueError("position_m must contain three finite values")


DEFAULT_CLEAR_POSITIONS_M = (
    (0.44, -0.10, 0.48),
    (0.47, -0.075, 0.50),
    (0.50, -0.05, 0.52),
    (0.47, -0.025, 0.48),
    (0.44, 0.00, 0.52),
)


def default_sim_perception_scenarios(
    positions: Sequence[Sequence[float]] = DEFAULT_CLEAR_POSITIONS_M,
) -> tuple[SimPerceptionScenario, ...]:
    """Return five ripe and five only-unripe camera-clear scenarios."""

    normalized = tuple(tuple(float(value) for value in point) for point in positions)
    if len(normalized) != 5 or len(set(normalized)) != 5:
        raise ValueError("the pre-gate requires five distinct clear positions")
    scenarios = tuple(
        SimPerceptionScenario(
            scenario_id=f"ripe_position_{index:02d}",
            target_id=1,
            model_name="strawberry_1",
            expected_maturity=RIPE,
            position_m=point,
        )
        for index, point in enumerate(normalized, start=1)
    ) + tuple(
        SimPerceptionScenario(
            scenario_id=f"unripe_only_position_{index:02d}",
            target_id=2,
            model_name="strawberry_2",
            expected_maturity=UNRIPE,
            position_m=point,
        )
        for index, point in enumerate(normalized, start=1)
    )
    if len({scenario.scenario_id for scenario in scenarios}) != len(scenarios):
        raise ValueError("scenario IDs must be unique")
    return scenarios


def summarize_sim_perception_gate(
    frame_records: Sequence[Mapping[str, object]],
    *,
    expected_scenarios: Sequence[SimPerceptionScenario],
    frames_per_scenario: int,
    ripe_recall_minimum: float = 0.90,
    false_ripe_rate_maximum: float = 0.05,
    target_pose_rate_minimum: float = 0.90,
) -> dict[str, object]:
    """Apply the frozen simple-scene simulator perception pre-gate."""

    if frames_per_scenario <= 0:
        raise ValueError("frames_per_scenario must be positive")
    for name, value in (
        ("ripe_recall_minimum", ripe_recall_minimum),
        ("false_ripe_rate_maximum", false_ripe_rate_maximum),
        ("target_pose_rate_minimum", target_pose_rate_minimum),
    ):
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be finite and in [0, 1]")
    scenario_ids = tuple(scenario.scenario_id for scenario in expected_scenarios)
    if not scenario_ids or len(set(scenario_ids)) != len(scenario_ids):
        raise ValueError("expected scenarios must be non-empty and unique")
    expected_by_id = {
        scenario.scenario_id: scenario.expected_maturity
        for scenario in expected_scenarios
    }
    counts_by_scenario = {scenario_id: 0 for scenario_id in scenario_ids}
    for record in frame_records:
        scenario_id = str(record.get("scenario_id", ""))
        if scenario_id not in expected_by_id:
            raise ValueError(f"unexpected scenario ID {scenario_id!r}")
        expected_maturity = str(record.get("expected_maturity", ""))
        if expected_maturity != expected_by_id[scenario_id]:
            raise ValueError("frame expected maturity differs from its scenario")
        for field in ("ripe_truth_match", "unripe_truth_match", "false_ripe", "target_pose_received"):
            if not isinstance(record.get(field), bool):
                raise ValueError(f"frame field {field} must be boolean")
        counts_by_scenario[scenario_id] += 1

    positive = [
        record for record in frame_records if record["expected_maturity"] == RIPE
    ]
    negative = [
        record for record in frame_records if record["expected_maturity"] == UNRIPE
    ]
    expected_complete = all(
        count == frames_per_scenario for count in counts_by_scenario.values()
    )
    ripe_hits = sum(bool(record["ripe_truth_match"]) for record in positive)
    false_ripe_frames = sum(bool(record["false_ripe"]) for record in negative)
    target_pose_hits = sum(bool(record["target_pose_received"]) for record in positive)
    unripe_hits = sum(bool(record["unripe_truth_match"]) for record in negative)
    ripe_recall = ripe_hits / len(positive) if positive else None
    false_ripe_rate = false_ripe_frames / len(negative) if negative else None
    target_pose_rate = target_pose_hits / len(positive) if positive else None
    unripe_observation_recall = unripe_hits / len(negative) if negative else None
    passed = bool(
        expected_complete
        and positive
        and negative
        and ripe_recall is not None
        and ripe_recall >= ripe_recall_minimum
        and false_ripe_rate is not None
        and false_ripe_rate <= false_ripe_rate_maximum
        and target_pose_rate is not None
        and target_pose_rate >= target_pose_rate_minimum
    )
    return {
        "scenario_count": len(scenario_ids),
        "frames_per_scenario": frames_per_scenario,
        "expected_frame_count": len(scenario_ids) * frames_per_scenario,
        "observed_frame_count": len(frame_records),
        "counts_by_scenario": counts_by_scenario,
        "all_scenarios_complete": expected_complete,
        "positive_frame_count": len(positive),
        "negative_frame_count": len(negative),
        "ripe_truth_match_count": ripe_hits,
        "ripe_frame_recall": ripe_recall,
        "ripe_frame_recall_minimum": ripe_recall_minimum,
        "false_ripe_frame_count": false_ripe_frames,
        "false_ripe_frame_rate": false_ripe_rate,
        "false_ripe_frame_rate_maximum": false_ripe_rate_maximum,
        "target_pose_frame_count": target_pose_hits,
        "target_pose_frame_rate": target_pose_rate,
        "target_pose_frame_rate_minimum": target_pose_rate_minimum,
        "unripe_truth_match_count": unripe_hits,
        "unripe_observation_recall_diagnostic": unripe_observation_recall,
        "passed": passed,
    }
