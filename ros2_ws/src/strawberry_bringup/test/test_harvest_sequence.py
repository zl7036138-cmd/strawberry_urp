from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.harvest_sequence import HarvestSequence, HarvestState  # noqa: E402


class HarvestSequenceTests(unittest.TestCase):
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
