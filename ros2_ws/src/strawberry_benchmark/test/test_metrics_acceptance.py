from pathlib import Path
import tempfile
import unittest

from strawberry_benchmark.acceptance import (
    GateStatus,
    evaluate_acceptance,
    load_acceptance_thresholds,
)
from strawberry_benchmark.metrics import compute_metrics
from strawberry_benchmark.models import (
    FailureCode,
    FailureStage,
    Maturity,
    ScenarioKind,
    TrialMode,
    TrialResult,
)


def result(
    trial_id,
    *,
    kind=ScenarioKind.POSITIVE,
    mode=TrialMode.END_TO_END,
    predicted=Maturity.RIPE,
    success=True,
    picked=True,
    failure_stage=None,
    failure_code=None,
    localization=None,
    planning=None,
):
    expected = Maturity.RIPE if kind is ScenarioKind.POSITIVE else Maturity.UNRIPE
    attempted = picked or (kind is ScenarioKind.POSITIVE and success)
    return TrialResult(
        trial_id=trial_id,
        mode=mode,
        scenario_kind=kind,
        occlusion="none",
        lighting="nominal",
        position="center",
        seed=20260710,
        expected_maturity=expected,
        predicted_maturity=predicted,
        detection_confidence=0.9 if predicted is not None else None,
        pick_attempted=attempted,
        fruit_picked=picked,
        success=success,
        first_failure_stage=failure_stage,
        failure_code=failure_code,
        localization_error_mm=localization,
        planning_time_sec=planning,
    )


def metric_fixture():
    return [
        result("p1", localization=10.0, planning=1.0),
        result("p2", localization=20.0, planning=2.0),
        result("p3", localization=30.0, planning=3.0),
        result(
            "p4",
            predicted=Maturity.UNRIPE,
            success=False,
            picked=False,
            failure_stage=FailureStage.DETECT,
            failure_code=FailureCode.LOW_CONFIDENCE,
            localization=40.0,
            planning=4.0,
        ),
        result(
            "n1",
            kind=ScenarioKind.NEGATIVE,
            predicted=Maturity.UNRIPE,
            picked=False,
        ),
        result(
            "n2",
            kind=ScenarioKind.NEGATIVE,
            predicted=Maturity.RIPE,
            success=False,
            picked=True,
            failure_stage=FailureStage.VERIFY,
            failure_code=FailureCode.GRASP_FAILED,
        ),
        result(
            "o1",
            mode=TrialMode.ORACLE,
            predicted=None,
            localization=5.0,
            planning=5.0,
        ),
        result(
            "o2",
            mode=TrialMode.ORACLE,
            predicted=None,
            success=False,
            picked=False,
            failure_stage=FailureStage.PLAN,
            failure_code=FailureCode.PLANNING_FAILED,
            planning=6.0,
        ),
    ]


PASSING_THRESHOLDS = {
    "perception_macro_f1_min": 0.60,
    "localization_median_error_mm_max": 25.0,
    "localization_p95_error_mm_max": 40.0,
    "oracle_success_rate_min": 0.40,
    "end_to_end_success_rate_min": 0.70,
    "false_pick_rate_max": 0.60,
    "planning_p95_sec_max": 6.0,
}


class MetricTests(unittest.TestCase):
    def test_rates_percentiles_and_attribution(self):
        metrics = compute_metrics(metric_fixture())
        self.assertEqual(8, metrics.total_trials)
        self.assertAlmostEqual(4 / 6, metrics.positive_success_rate)
        self.assertAlmostEqual(0.5, metrics.oracle_success_rate)
        self.assertAlmostEqual(0.75, metrics.end_to_end_success_rate)
        self.assertEqual(1, metrics.false_pick_count)
        self.assertAlmostEqual(0.5, metrics.false_pick_rate)
        self.assertAlmostEqual(0.625, metrics.perception_macro_f1)
        self.assertEqual(20.0, metrics.localization_median_error_mm)
        self.assertEqual(38.0, metrics.localization_p95_error_mm)
        self.assertEqual(5.75, metrics.planning_p95_sec)
        self.assertEqual(3, metrics.failed_trials)
        self.assertEqual(0, metrics.unattributed_failure_count)
        self.assertEqual(
            {"DETECT": 1, "PLAN": 1, "VERIFY": 1}, metrics.failure_by_stage
        )

    def test_duplicate_trial_ids_are_rejected(self):
        duplicate = result("duplicate")
        with self.assertRaisesRegex(ValueError, "duplicate trial_id"):
            compute_metrics([duplicate, duplicate])


class AcceptanceTests(unittest.TestCase):
    def test_all_gates_pass_at_or_inside_boundaries(self):
        report = evaluate_acceptance(compute_metrics(metric_fixture()), PASSING_THRESHOLDS)
        self.assertEqual(GateStatus.PASS, report.status)
        self.assertTrue(report.passed)
        self.assertTrue(all(gate.status is GateStatus.PASS for gate in report.gates))

    def test_any_failed_gate_fails_the_report(self):
        thresholds = dict(PASSING_THRESHOLDS)
        thresholds["false_pick_rate_max"] = 0.1
        report = evaluate_acceptance(compute_metrics(metric_fixture()), thresholds)
        self.assertEqual(GateStatus.FAIL, report.status)
        failed_names = {gate.name for gate in report.gates if gate.status is GateStatus.FAIL}
        self.assertEqual({"false_pick_rate_max"}, failed_names)

    def test_missing_observations_are_not_evaluated(self):
        empty_metrics = compute_metrics([])
        report = evaluate_acceptance(empty_metrics, PASSING_THRESHOLDS)
        self.assertEqual(GateStatus.NOT_EVALUATED, report.status)
        self.assertFalse(report.passed)

    def test_project_yaml_threshold_loader(self):
        content = "acceptance:\n" + "".join(
            f"  {name}: {value}\n" for name, value in PASSING_THRESHOLDS.items()
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "project.yaml"
            path.write_text(content, encoding="utf-8")
            loaded = load_acceptance_thresholds(path)
        self.assertEqual(PASSING_THRESHOLDS, loaded)


if __name__ == "__main__":
    unittest.main()
