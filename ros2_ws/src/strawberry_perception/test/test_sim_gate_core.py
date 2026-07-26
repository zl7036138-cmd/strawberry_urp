import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_perception.sim_gate_core import (  # noqa: E402
    RIPE,
    UNRIPE,
    default_sim_perception_scenarios,
    summarize_sim_perception_gate,
)


def _records(*, ripe_hits=50, false_ripe=2, pose_hits=50, unripe_hits=50):
    scenarios = default_sim_perception_scenarios()
    records = []
    positive_index = 0
    negative_index = 0
    for scenario in scenarios:
        for _ in range(10):
            if scenario.expected_maturity == RIPE:
                records.append(
                    {
                        "scenario_id": scenario.scenario_id,
                        "expected_maturity": RIPE,
                        "ripe_truth_match": positive_index < ripe_hits,
                        "unripe_truth_match": False,
                        "false_ripe": False,
                        "target_pose_received": positive_index < pose_hits,
                    }
                )
                positive_index += 1
            else:
                records.append(
                    {
                        "scenario_id": scenario.scenario_id,
                        "expected_maturity": UNRIPE,
                        "ripe_truth_match": False,
                        "unripe_truth_match": negative_index < unripe_hits,
                        "false_ripe": negative_index < false_ripe,
                        "target_pose_received": False,
                    }
                )
                negative_index += 1
    return scenarios, records


class ScenarioTests(unittest.TestCase):
    def test_default_matrix_has_balanced_unique_scenarios(self):
        scenarios = default_sim_perception_scenarios()
        self.assertEqual(len(scenarios), 10)
        self.assertEqual(sum(item.expected_maturity == RIPE for item in scenarios), 5)
        self.assertEqual(sum(item.expected_maturity == UNRIPE for item in scenarios), 5)
        self.assertEqual(len({item.scenario_id for item in scenarios}), 10)
        self.assertEqual(len({item.position_m for item in scenarios}), 5)

    def test_requires_exactly_five_distinct_positions(self):
        with self.assertRaises(ValueError):
            default_sim_perception_scenarios(((0.4, 0.0, 0.5),))


class SummaryTests(unittest.TestCase):
    def test_passes_at_frozen_boundaries(self):
        scenarios, records = _records(
            ripe_hits=45, false_ripe=2, pose_hits=45, unripe_hits=0
        )
        summary = summarize_sim_perception_gate(
            records, expected_scenarios=scenarios, frames_per_scenario=10
        )
        self.assertTrue(summary["passed"])
        self.assertEqual(summary["ripe_frame_recall"], 0.9)
        self.assertEqual(summary["false_ripe_frame_rate"], 0.04)
        self.assertEqual(summary["target_pose_frame_rate"], 0.9)

    def test_fails_each_safety_or_liveness_gate(self):
        for overrides in (
            {"ripe_hits": 44},
            {"false_ripe": 3},
            {"pose_hits": 44},
        ):
            scenarios, records = _records(**overrides)
            summary = summarize_sim_perception_gate(
                records, expected_scenarios=scenarios, frames_per_scenario=10
            )
            self.assertFalse(summary["passed"], overrides)

    def test_incomplete_scenario_and_unknown_scenario_fail_closed(self):
        scenarios, records = _records()
        incomplete = summarize_sim_perception_gate(
            records[:-1], expected_scenarios=scenarios, frames_per_scenario=10
        )
        self.assertFalse(incomplete["passed"])
        records[0] = {**records[0], "scenario_id": "unknown"}
        with self.assertRaises(ValueError):
            summarize_sim_perception_gate(
                records, expected_scenarios=scenarios, frames_per_scenario=10
            )


if __name__ == "__main__":
    unittest.main()
