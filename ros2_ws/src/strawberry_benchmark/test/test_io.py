from pathlib import Path
import tempfile
import unittest

from strawberry_benchmark.io import (
    read_trial_results,
    write_trial_results,
)
from strawberry_benchmark.models import (
    FailureCode,
    FailureStage,
    Maturity,
    ScenarioKind,
    TrialMode,
    TrialResult,
)


def sample_results():
    return [
        TrialResult(
            trial_id="positive-001",
            mode=TrialMode.END_TO_END,
            scenario_kind=ScenarioKind.POSITIVE,
            occlusion="none",
            lighting="nominal",
            position="center",
            seed=20260710,
            expected_maturity=Maturity.RIPE,
            predicted_maturity=Maturity.RIPE,
            detection_confidence=0.91,
            pick_attempted=True,
            fruit_picked=True,
            success=True,
            localization_error_mm=8.5,
            planning_time_sec=1.2,
            execution_time_sec=4.3,
            timestamp_utc="2026-07-10T12:00:00Z",
            metadata={"world": "simple", "attempt": 1},
        ),
        TrialResult(
            trial_id="positive-002",
            mode=TrialMode.END_TO_END,
            scenario_kind=ScenarioKind.POSITIVE,
            occlusion="partial",
            lighting="dim",
            position="far_left",
            seed=20260711,
            expected_maturity=Maturity.RIPE,
            predicted_maturity=Maturity.UNKNOWN,
            detection_confidence=0.2,
            pick_attempted=False,
            fruit_picked=False,
            success=False,
            first_failure_stage=FailureStage.DETECT,
            failure_code=FailureCode.LOW_CONFIDENCE,
            metadata={"note": "低置信度"},
        ),
    ]


class TrialIoTests(unittest.TestCase):
    def test_jsonl_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trials.jsonl"
            self.assertEqual(2, write_trial_results(sample_results(), path))
            actual = read_trial_results(path)
        self.assertEqual(
            [item.to_dict() for item in sample_results()],
            [item.to_dict() for item in actual],
        )

    def test_csv_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trials.csv"
            self.assertEqual(2, write_trial_results(sample_results(), path))
            actual = read_trial_results(path)
        self.assertEqual(
            [item.to_dict() for item in sample_results()],
            [item.to_dict() for item in actual],
        )

    def test_invalid_physical_pick_is_rejected(self):
        values = sample_results()[0].to_dict()
        values["pick_attempted"] = False
        with self.assertRaisesRegex(ValueError, "fruit_picked"):
            TrialResult.from_dict(values)

    def test_partial_failure_attribution_is_rejected(self):
        values = sample_results()[1].to_dict()
        values["failure_code"] = None
        with self.assertRaisesRegex(ValueError, "both be set"):
            TrialResult.from_dict(values)


if __name__ == "__main__":
    unittest.main()
