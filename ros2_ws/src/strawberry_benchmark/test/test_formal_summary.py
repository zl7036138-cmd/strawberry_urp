from pathlib import Path
import unittest

from strawberry_benchmark.formal_matrix import load_formal_matrix_contract
from strawberry_benchmark.formal_summary import evaluate_formal_records


ROOT = Path(__file__).resolve().parents[4]
CONTRACT = load_formal_matrix_contract(ROOT / "config/p3_formal_matrix_v1.json")


def passing_records():
    records = []
    for index in range(135):
        records.append(
            {
                "trial_id": f"positive-{index}",
                "kind": "POSITIVE",
                "lighting": ("dim", "nominal", "bright")[index % 3],
                "occlusion": ("none", "partial", "heavy")[index % 3],
                "position": ("near_left", "near_right", "center", "far_left", "far_right")[index % 5],
                "simulation_seed": (20260710, 20260711, 20260712)[index % 3],
                "behavior_started": True,
                "infrastructure_valid": True,
                "shutdown_clean": True,
                "positive_success": True,
                "negative_safe_no_pick": False,
                "pick_attempted": True,
                "fruit_picked": True,
                "planning_time_sec": 0.1,
                "failure_category": None,
            }
        )
    for index in range(30):
        records.append(
            {
                "trial_id": f"negative-{index}",
                "kind": "NEGATIVE",
                "lighting": ("dim", "nominal", "bright")[index % 3],
                "occlusion": ("none", "partial", "heavy")[index % 3],
                "position": ("near_left", "near_right", "center", "far_left", "far_right")[index % 5],
                "simulation_seed": (20260710, 20260711, 20260712)[index % 3],
                "behavior_started": True,
                "infrastructure_valid": True,
                "shutdown_clean": True,
                "positive_success": False,
                "negative_safe_no_pick": True,
                "pick_attempted": False,
                "fruit_picked": False,
                "planning_time_sec": None,
                "failure_category": None,
            }
        )
    return records


class FormalSummaryTests(unittest.TestCase):
    def test_complete_passing_matrix_passes(self):
        result = evaluate_formal_records(CONTRACT, passing_records())
        self.assertTrue(result["formal_p3_simulator_gate_passed"])
        self.assertEqual(1.0, result["positive_success_rate"])
        self.assertEqual(1.0, result["negative_safe_no_pick_rate"])

    def test_two_negative_attempts_exceed_five_percent(self):
        records = passing_records()
        for item in records[135:137]:
            item["negative_safe_no_pick"] = False
            item["pick_attempted"] = True
            item["failure_category"] = "perception"
        result = evaluate_formal_records(CONTRACT, records)
        self.assertFalse(result["formal_p3_simulator_gate_passed"])
        self.assertAlmostEqual(2 / 30, result["negative_pick_attempt_rate"])

    def test_unattributed_behavior_failure_fails_closed(self):
        records = passing_records()
        records[0]["positive_success"] = False
        records[0]["failure_category"] = None
        result = evaluate_formal_records(CONTRACT, records)
        self.assertFalse(result["formal_p3_simulator_gate_passed"])
        self.assertEqual(1, result["unattributed_failure_count"])

    def test_infrastructure_or_shutdown_failure_cannot_be_hidden(self):
        records = passing_records()
        records[10]["infrastructure_valid"] = False
        records[11]["shutdown_clean"] = False
        result = evaluate_formal_records(CONTRACT, records)
        self.assertFalse(result["infrastructure_passed"])
        self.assertFalse(result["shutdown_passed"])
        self.assertFalse(result["formal_p3_simulator_gate_passed"])


if __name__ == "__main__":
    unittest.main()
