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
        "recorder_outcome": "SUCCESS",
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
                "skipped_targets": {},
                "failures": [],
                "state_history": [
                    {
                        "state": "DONE",
                        "target_id": None,
                        "outcome": "SUCCESS",
                        "detail": "",
                    }
                ],
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

    def test_two_physical_ripe_placements_match_but_safety_is_indeterminate(self):
        result = score_runtime_run(
            passing_receipt(),
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )

        self.assertFalse(result["run_safety_pass"])
        self.assertEqual(result["physical_identity_status"], "PASS")
        self.assertEqual(result["evidence_status"], "INDETERMINATE")
        self.assertEqual(result["harvested_distinct_count"], 2)
        self.assertEqual(result["identity_correct_physical_harvest_count"], 2)
        self.assertIsNone(result["planner_accepted_target_count"])
        self.assertIsNone(result["pick_started_target_count"])
        self.assertEqual(result["accepted_target_success_rate"], 1.0)
        self.assertEqual(
            result["accepted_target_success_rate_status"], "INDETERMINATE"
        )
        self.assertEqual(result["runtime_to_sim_target_id"], {"10": 1, "20": 2})

    def test_recovery_home_failure_cannot_pass_safety_or_aggregate_gate(self):
        receipt = passing_receipt()
        receipt["recorder_outcome"] = "RECOVERY_HOME_FAILED"
        receipt["events"][-1]["outcome"] = "RECOVERY_HOME_FAILED"
        result = score_runtime_run(
            receipt,
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )

        self.assertEqual(result["home_stop_evidence_status"], "FAIL")
        self.assertFalse(result["run_safety_pass"])
        self.assertFalse(
            summarize_runtime_gate([deepcopy(result) for _ in range(5)])["overall_pass"]
        )

    def test_wrong_physical_ripe_target_fails_per_track_identity(self):
        receipt = passing_receipt()
        receipt["selection_events"] = receipt["selection_events"][:1]
        receipt["events"] = [
            event
            for event in receipt["events"]
            if event.get("current_target_id") != 20
        ]
        receipt["events"][-1]["harvested_target_ids"] = [10]
        receipt["ground_truth_score_events"] = [
            score_event("CONTACT_RESOLVED", 2),
            score_event("PLACED", 2),
        ]
        result = score_runtime_run(
            receipt,
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )

        self.assertEqual(result["physical_identity_status"], "FAIL")
        self.assertEqual(
            result["physical_harvest_correspondence"],
            [
                {
                    "runtime_track_id": 10,
                    "mapped_sim_target_id": 1,
                    "physical_sim_target_id": 2,
                    "identity_match": False,
                }
            ],
        )
        self.assertFalse(result["evidence_integrity_pass"])
        self.assertFalse(result["run_safety_pass"])

    def test_terminal_outcome_contradiction_fails_integrity(self):
        receipt = passing_receipt()
        receipt["recorder_outcome"] = "PARTIAL_SUCCESS"
        receipt["events"][-1]["outcome"] = "PARTIAL_SUCCESS"
        receipt["events"][-1]["state_history"][-1][
            "outcome"
        ] = "PARTIAL_SUCCESS"
        result = score_runtime_run(
            receipt,
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )

        self.assertEqual(result["terminal_evidence_status"], "FAIL")
        self.assertFalse(result["evidence_integrity_pass"])
        self.assertFalse(result["run_safety_pass"])

    def test_missing_independent_physical_safety_state_is_indeterminate(self):
        result = score_runtime_run(
            passing_receipt(),
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )

        self.assertEqual(result["collision_evidence_status"], "INDETERMINATE")
        self.assertEqual(result["joint_limit_evidence_status"], "INDETERMINATE")
        self.assertEqual(result["home_stop_evidence_status"], "INDETERMINATE")
        self.assertEqual(
            result["attachment_terminal_evidence_status"], "PASS"
        )
        self.assertEqual(result["scene_terminal_evidence_status"], "INDETERMINATE")
        self.assertEqual(result["safety_evidence_status"], "INDETERMINATE")
        self.assertFalse(result["run_safety_pass"])

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

    def test_duplicate_or_out_of_order_physical_lifecycle_is_rejected(self):
        for events in (
            [score_event("PLACED", 1)],
            [score_event("CONTACT_RESOLVED", 1)] * 2
            + [score_event("PLACED", 1)],
            [
                score_event("CONTACT_RESOLVED", 1),
                score_event("PLACED", 1),
                score_event("CONTACT_RESOLVED", 1),
                score_event("PLACED", 1),
            ],
        ):
            with self.subTest(events=events):
                receipt = passing_receipt()
                receipt["ground_truth_score_events"] = events
                result = score_runtime_run(
                    receipt,
                    scene(),
                    {"outcome": "CLEAN"},
                    {"overall_pass": True},
                )
                self.assertEqual(result["physical_identity_status"], "FAIL")
                self.assertFalse(result["run_safety_pass"])

    def test_physical_event_maturity_must_match_scene_manifest(self):
        receipt = passing_receipt()
        receipt["ground_truth_score_events"][0]["maturity"] = "UNRIPE"
        result = score_runtime_run(
            receipt,
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )

        self.assertEqual(result["physical_identity_status"], "FAIL")
        self.assertIn(
            "physical event maturity disagrees with scene for fruit 1",
            result["evidence_errors"],
        )

    def test_pre_motion_collision_rejection_remains_a_legacy_diagnostic(self):
        receipt = passing_receipt()
        receipt["events"].insert(
            -1,
            {
                "state": "SCANNING",
                "outcome": "TARGET_SKIPPED",
                "current_target_id": None,
                "state_history": [
                    {
                        "state": "SCANNING",
                        "target_id": 30,
                        "outcome": "FINAL_PICK_FEASIBILITY_FAILED_SKIPPED",
                        "detail": "connected path intersects a collision object",
                    }
                ],
            },
        )
        receipt["events"][-1]["failures"] = [
            {
                "target_id": 30,
                "failure_code": 7,
                "message": "connected path intersects a collision object",
            }
        ]

        result = score_runtime_run(
            receipt,
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )

        self.assertEqual(result["collision_count"], 0)
        self.assertEqual(result["collision_rejection_count"], 1)
        self.assertEqual(result["collision_evidence_status"], "INDETERMINATE")
        self.assertFalse(result["run_safety_pass"])

    def test_action_collision_without_pre_motion_rejection_is_scored(self):
        receipt = passing_receipt()
        receipt["events"][-1]["failures"] = [
            {
                "target_id": 10,
                "failure_code": 7,
                "message": "collision during manipulation action",
            }
        ]

        result = score_runtime_run(
            receipt,
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )

        self.assertEqual(result["collision_count"], 1)
        self.assertEqual(result["collision_rejection_count"], 0)
        self.assertFalse(result["run_safety_pass"])

    def test_five_two_fruit_runs_need_independent_safety_evidence(self):
        run = score_runtime_run(
            passing_receipt(),
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )
        summary = summarize_runtime_gate([deepcopy(run) for _ in range(5)])

        self.assertFalse(summary["overall_pass"])
        self.assertEqual(summary["overall_status"], "INDETERMINATE")
        self.assertFalse(summary["gates"]["run_safety_evidence"])
        self.assertTrue(
            summary["raw_threshold_checks"]["four_of_five_multi_fruit"]
        )
        self.assertTrue(
            summary["raw_threshold_checks"]["accepted_target_success_rate"]
        )
        self.assertEqual(
            summary["gate_statuses"]["four_of_five_multi_fruit"],
            "INDETERMINATE",
        )
        self.assertEqual(
            summary["gate_statuses"]["accepted_target_success_rate"],
            "INDETERMINATE",
        )
        self.assertEqual(summary["runs_with_two_or_more_harvests"], 5)

    def test_four_of_five_and_eighty_percent_are_independent_gates(self):
        run = score_runtime_run(
            passing_receipt(),
            scene(),
            {"outcome": "CLEAN"},
            {"overall_pass": True},
        )
        scores = [deepcopy(run) for _ in range(5)]
        scores[-1]["identity_correct_physical_harvest_count"] = 0
        scores[-1]["accepted_target_count"] = 2
        summary = summarize_runtime_gate(scores)
        self.assertTrue(
            summary["raw_threshold_checks"]["four_of_five_multi_fruit"]
        )
        self.assertTrue(
            summary["raw_threshold_checks"]["accepted_target_success_rate"]
        )

        scores[-1]["accepted_target_count"] = 20
        summary = summarize_runtime_gate(scores)
        self.assertFalse(
            summary["raw_threshold_checks"]["accepted_target_success_rate"]
        )
        self.assertEqual(
            summary["gate_statuses"]["accepted_target_success_rate"], "FAIL"
        )


if __name__ == "__main__":
    unittest.main()
