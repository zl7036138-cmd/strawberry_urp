from copy import deepcopy
from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.development_gate import (  # noqa: E402
    match_runtime_tracks_to_scene,
    score_runtime_run,
    summarize_runtime_gate,
)


def scene():
    return {
        "fruits": [
            {
                "target_id": 1,
                "maturity": "RIPE",
                "reachable_by_construction": True,
                "initial_pose_m": [0.35, 0.10, 0.54],
            },
            {
                "target_id": 2,
                "maturity": "RIPE",
                "reachable_by_construction": True,
                "initial_pose_m": [0.43, 0.12, 0.55],
            },
            {
                "target_id": 3,
                "maturity": "UNRIPE",
                "reachable_by_construction": True,
                "initial_pose_m": [0.55, 0.00, 0.54],
            },
        ]
    }


def score_event(event, target_id, maturity="RIPE"):
    return {
        "scope": "SIMULATION_GROUND_TRUTH_SCORING_ONLY",
        "event": event,
        "target_id": target_id,
        "maturity": maturity,
    }


def passing_receipt():
    return {
        "schema_version": 3,
        "terminal_status_received": True,
        "ground_truth_score_events_used_for_control": False,
        "selection_events": [
            {
                "outcome": "TARGET_SELECTED",
                "track_id": 10,
                "position_m": [0.352, 0.10, 0.54],
            },
            {
                "outcome": "TARGET_SELECTED",
                "track_id": 20,
                "position_m": [0.429, 0.12, 0.55],
            },
        ],
        "ground_truth_score_events": [
            score_event("CONTACT_RESOLVED", 1),
            score_event("PLACED", 1),
            score_event("CONTACT_RESOLVED", 2),
            score_event("PLACED", 2),
        ],
        "events": [
            {"outcome": "OBSERVATION_MOTION_1_OF_4", "current_target_id": 10},
            {"outcome": "PICK_SENT", "current_target_id": 10},
            {"outcome": "OBSERVATION_MOTION_1_OF_4", "current_target_id": 20},
            {"outcome": "PICK_SENT", "current_target_id": 20},
            {
                "state": "DONE",
                "outcome": "SUCCESS",
                "harvested_target_ids": [10, 20],
                "failures": [],
            },
        ],
    }


class DevelopmentRuntimeGateTests(unittest.TestCase):
    def test_post_run_matching_is_one_to_one_and_bounded(self):
        mapping, distances = match_runtime_tracks_to_scene(
            passing_receipt()["selection_events"],
            scene(),
            track_ids=[10, 20],
        )

        self.assertEqual(mapping, {10: 1, 20: 2})
        self.assertLess(distances[10], 0.01)

    def test_two_physical_ripe_placements_pass_one_run(self):
        result = score_runtime_run(
            passing_receipt(),
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )

        self.assertTrue(result["run_safety_pass"])
        self.assertEqual(result["harvested_distinct_count"], 2)
        self.assertEqual(result["accepted_target_success_rate"], 1.0)
        self.assertEqual(result["runtime_to_sim_target_id"], {"10": 1, "20": 2})

    def test_unripe_contact_and_motion_are_hard_failures(self):
        receipt = passing_receipt()
        receipt["selection_events"][1]["position_m"] = [0.55, 0.0, 0.54]
        receipt["ground_truth_score_events"][2:] = [
            score_event("CONTACT_RESOLVED", 3, "UNRIPE"),
            score_event("PLACED", 3, "UNRIPE"),
        ]
        result = score_runtime_run(
            receipt,
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )

        self.assertEqual(result["unripe_pick_count"], 1)
        self.assertEqual(result["unsafe_motion_attempt_count"], 1)
        self.assertFalse(result["run_safety_pass"])

    def test_unmatched_motion_and_physical_count_mismatch_fail_integrity(self):
        receipt = passing_receipt()
        receipt["selection_events"][1]["position_m"] = [9.0, 9.0, 9.0]
        receipt["ground_truth_score_events"] = receipt[
            "ground_truth_score_events"
        ][:2]
        result = score_runtime_run(
            receipt,
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )

        self.assertFalse(result["evidence_integrity_pass"])
        self.assertFalse(result["run_safety_pass"])

    def test_five_two_fruit_runs_pass_the_aggregate_gate(self):
        run = score_runtime_run(
            passing_receipt(),
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )
        summary = summarize_runtime_gate([deepcopy(run) for _ in range(5)])

        self.assertTrue(summary["overall_pass"])
        self.assertEqual(summary["runs_with_two_or_more_harvests"], 5)

    def test_four_of_five_and_eighty_percent_are_independent_gates(self):
        run = score_runtime_run(
            passing_receipt(),
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )
        scores = [deepcopy(run) for _ in range(5)]
        scores[-1]["harvested_distinct_count"] = 0
        scores[-1]["accepted_target_count"] = 2
        summary = summarize_runtime_gate(scores)
        self.assertTrue(summary["gates"]["four_of_five_multi_fruit"])
        self.assertTrue(summary["gates"]["accepted_target_success_rate"])

        scores[-1]["accepted_target_count"] = 20
        summary = summarize_runtime_gate(scores)
        self.assertFalse(summary["gates"]["accepted_target_success_rate"])


if __name__ == "__main__":
    unittest.main()
