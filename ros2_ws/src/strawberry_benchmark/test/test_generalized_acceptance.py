import json
from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE))

from strawberry_benchmark.generalized_acceptance import summarize_results, validate_matrix  # noqa: E402


class GeneralizedAcceptanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.matrix = json.loads((REPOSITORY / "config" / "generalized_harvest_matrix_v1.json").read_text(encoding="utf-8"))

    def perfect_results(self):
        rows = []
        for scenario in self.matrix["scenarios"]:
            profile = scenario["profile"]
            positive = profile == "mixed"
            negative = profile == "all_unripe"
            rows.append(
                {
                    "scenario_id": scenario["scenario_id"],
                    "visible_ripe_truth_count": 2 if positive else 0,
                    "ripe_true_positive_count": 2 if positive else 0,
                    "ripe_prediction_count": 2 if positive else 0,
                    "identity_switch_count": 0,
                    "association_count": 20,
                    "localization_errors_m": [0.01, 0.02] if positive else [],
                    "accepted_sigmas_m": [0.01, 0.012] if positive else [],
                    "unripe_pick_count": 0,
                    "collision_count": 0,
                    "reachable_ripe_truth_count": 2 if positive else 0,
                    "harvested_reachable_ripe_count": 2 if positive else 0,
                    "scene_complete": positive,
                    "outcome": "NO_PICK" if negative else ("SUCCESS" if positive else "FAILED"),
                }
            )
        return rows

    def test_frozen_matrix_has_required_distribution(self):
        scenarios = validate_matrix(self.matrix)
        self.assertEqual(len(scenarios), 30)

    def test_perfect_formal_results_pass_every_gate(self):
        summary = summarize_results(self.matrix, self.perfect_results())
        self.assertTrue(summary["overall_pass"])
        self.assertTrue(all(summary["gates"].values()))

    def test_collision_or_wrong_negative_behavior_fails(self):
        rows = self.perfect_results()
        rows[0]["collision_count"] = 1
        negative_index = next(index for index, scenario in enumerate(self.matrix["scenarios"]) if scenario["profile"] == "all_unripe")
        rows[negative_index]["outcome"] = "SUCCESS"
        summary = summarize_results(self.matrix, rows)
        self.assertFalse(summary["overall_pass"])
        self.assertFalse(summary["gates"]["collision_count"])
        self.assertFalse(summary["gates"]["negative_safe_no_pick_rate"])

    def test_missing_or_duplicate_scenario_fails_closed(self):
        rows = self.perfect_results()
        rows[-1] = dict(rows[0])
        with self.assertRaises(ValueError):
            summarize_results(self.matrix, rows)


if __name__ == "__main__":
    unittest.main()
