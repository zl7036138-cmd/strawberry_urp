from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.harvest_sequence import HarvestSequence, HarvestState  # noqa: E402
from strawberry_bringup.harvest_orchestrator import build_scan_diagnostics  # noqa: E402


class HarvestSequenceTests(unittest.TestCase):
    def test_scan_diagnostics_explain_truth_free_no_pick(self):
        diagnostics = build_scan_diagnostics(
            tracks=[
                {
                    "track_id": 2,
                    "maturity": 1,
                    "confidence": 0.91,
                    "sigma_m": 0.017,
                    "observation_count": 7,
                    "position_m": [0.4, 0.1, 0.55],
                },
                {
                    "track_id": 3,
                    "maturity": 2,
                    "confidence": 0.94,
                    "sigma_m": 0.008,
                    "observation_count": 8,
                    "position_m": [0.5, 0.0, 0.54],
                },
            ],
            selection_status={
                "outcome": "NO_PICK",
                "rejections": [{"track_id": 2, "reasons": ["UNCERTAIN"]}],
            },
            completed_ids={1},
            track_snapshot_age_wall_sec=0.25,
            selection_status_age_wall_sec=0.50,
        )

        self.assertEqual(diagnostics["visible_track_count"], 2)
        self.assertEqual(diagnostics["ripe_track_ids"], [2])
        self.assertEqual(diagnostics["completed_target_ids"], [1])
        self.assertEqual(diagnostics["track_snapshot_age_wall_sec"], 0.25)
        self.assertEqual(diagnostics["selection_status_age_wall_sec"], 0.50)
        self.assertEqual(
            diagnostics["latest_selection_status"]["rejections"][0]["track_id"],
            2,
        )

    def test_multiple_targets_continue_until_scan_finishes(self):
        sequence = HarvestSequence()
        sequence.start()
        for target_id in (3, 7):
            sequence.select(target_id)
            sequence.observation_result(True)
            sequence.confirmation_result(True)
            sequence.pick_result(True, "placed")
        self.assertEqual(sequence.harvested, [3, 7])
        self.assertEqual(sequence.finish(), "SUCCESS")

    def test_one_failure_gets_exactly_one_retry_then_is_skipped(self):
        sequence = HarvestSequence()
        sequence.start()
        sequence.select(4)
        sequence.observation_result(False, "view blocked")
        self.assertEqual(sequence.retry_target_id, 4)
        sequence.select(4)
        sequence.observation_result(True)
        sequence.confirmation_result(False, "no stable wrist pose")
        self.assertIn(4, sequence.skipped)
        self.assertIsNone(sequence.retry_target_id)

    def test_failed_target_does_not_stop_later_success(self):
        sequence = HarvestSequence()
        sequence.start()
        for _ in range(2):
            sequence.select(1)
            sequence.observation_result(True)
            sequence.confirmation_result(True)
            sequence.pick_result(False, "collision", 7)
        sequence.select(2)
        sequence.observation_result(True)
        sequence.confirmation_result(True)
        sequence.pick_result(True)
        self.assertEqual(sequence.finish(), "PARTIAL_SUCCESS")
        self.assertEqual(sequence.harvested, [2])
        self.assertIn(1, sequence.skipped)

    def test_cached_distinct_view_consumes_the_single_retry_in_place(self):
        sequence = HarvestSequence()
        sequence.start()
        sequence.select(1)
        sequence.observation_result(True)
        sequence.confirmation_result(True)

        self.assertTrue(
            sequence.begin_bounded_reobservation(
                "FINAL_PICK_FEASIBILITY_FAILED",
                "connected route collides",
                failure_code=7,
            )
        )
        self.assertEqual(sequence.state, HarvestState.OBSERVING)
        self.assertEqual(sequence.current_target_id, 1)
        self.assertEqual(sequence.attempts[1], 2)
        self.assertIsNone(sequence.retry_target_id)
        self.assertEqual(sequence.failures[-1]["failure_code"], 7)

        sequence.observation_result(True)
        sequence.confirmation_result(True)
        sequence.pick_result(True, "placed")
        self.assertEqual(sequence.harvested, [1])

    def test_cached_reobservation_never_creates_a_third_attempt(self):
        sequence = HarvestSequence()
        sequence.start()
        sequence.select(2)
        sequence.observation_result(True)
        self.assertTrue(
            sequence.begin_bounded_reobservation(
                "CONFIRMATION_FAILED",
                "uncertain",
            )
        )
        sequence.observation_result(True)
        sequence.confirmation_result(True)
        self.assertFalse(
            sequence.begin_bounded_reobservation(
                "FINAL_PICK_FEASIBILITY_FAILED",
                "still blocked",
                failure_code=7,
            )
        )
        self.assertIn(2, sequence.skipped)
        self.assertEqual(sequence.attempts[2], 2)

    def test_no_candidate_is_safe_no_pick(self):
        sequence = HarvestSequence()
        sequence.start()
        self.assertEqual(sequence.finish("timeout"), "NO_PICK")
        self.assertEqual(sequence.state, HarvestState.DONE)

    def test_unobservable_retry_is_skipped_and_later_targets_continue(self):
        sequence = HarvestSequence()
        sequence.start()
        sequence.select(1)
        sequence.observation_result(False, "occluded")
        self.assertEqual(sequence.retry_unavailable("still occluded"), 1)
        self.assertIn(1, sequence.skipped)
        sequence.select(2)
        sequence.observation_result(True)
        sequence.confirmation_result(True)
        sequence.pick_result(True)
        self.assertEqual(sequence.finish(), "PARTIAL_SUCCESS")

    def test_repeated_skips_cannot_exceed_batch_target_limit(self):
        sequence = HarvestSequence(max_targets=2)
        sequence.start()
        for target_id in (1, 2):
            sequence.select(target_id)
            sequence.observation_result(False, "unsafe")
            sequence.select(target_id)
            sequence.observation_result(False, "unsafe")
        self.assertTrue(sequence.terminal)
        self.assertEqual(sequence.history[-1].detail, "MAX_TARGETS_REACHED")


if __name__ == "__main__":
    unittest.main()
