from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_localization.tracking import (  # noqa: E402
    LocalizedObservation,
    MultiTargetTracker,
)


def observation(identity, x, maturity=1, confidence=0.9, sigma=0.01):
    return LocalizedObservation(identity, maturity, (x, 0.0, 0.55), confidence, sigma)


class MultiTargetTrackerTests(unittest.TestCase):
    def test_frame_local_ids_can_change_without_changing_track_ids(self):
        tracker = MultiTargetTracker(minimum_observations=2)
        first = tracker.update([observation(1, 0.40), observation(2, 0.58)], stamp_sec=1.0)
        second = tracker.update([observation(20, 0.405), observation(10, 0.575)], stamp_sec=1.1)
        self.assertEqual([item.track_id for item in first], [1, 2])
        self.assertEqual([item.track_id for item in second], [1, 2])
        self.assertEqual([item.source_detection_id for item in second], [20, 10])
        self.assertEqual(len(tracker.snapshot(stamp_sec=1.1, stable_only=True)), 2)

    def test_assignment_is_one_to_one_and_deterministic(self):
        tracker = MultiTargetTracker(association_distance_m=0.08)
        tracker.update([observation(1, 0.40), observation(2, 0.55)], stamp_sec=0.0)
        tracks = tracker.update([observation(7, 0.44), observation(8, 0.51)], stamp_sec=0.1)
        self.assertEqual([item.track_id for item in tracks], [1, 2])
        self.assertEqual([item.observation_count for item in tracks], [2, 2])

    def test_maturity_uses_a_stable_vote(self):
        tracker = MultiTargetTracker(minimum_observations=3)
        tracker.update([observation(1, 0.4, maturity=1)], stamp_sec=0.0)
        tracker.update([observation(2, 0.4, maturity=2)], stamp_sec=0.1)
        tracks = tracker.update([observation(3, 0.4, maturity=1)], stamp_sec=0.2)
        self.assertEqual(tracks[0].maturity, 1)

    def test_stale_and_harvested_tracks_are_suppressed(self):
        tracker = MultiTargetTracker(max_track_age_sec=0.5)
        tracker.update([observation(1, 0.4)], stamp_sec=1.0)
        tracker.mark_harvested(1)
        self.assertEqual(tracker.snapshot(stamp_sec=1.1), ())
        tracker.update([observation(2, 0.6)], stamp_sec=2.0)
        self.assertEqual([row.track_id for row in tracker.snapshot(stamp_sec=2.0)], [2])

    def test_stale_static_fruit_recovers_its_original_identity(self):
        tracker = MultiTargetTracker(max_track_age_sec=0.5)
        tracker.update([observation(1, 0.4)], stamp_sec=1.0)
        self.assertEqual(tracker.snapshot(stamp_sec=2.0), ())
        tracks = tracker.update([observation(8, 0.402)], stamp_sec=2.1)
        self.assertEqual([row.track_id for row in tracks], [1])
        self.assertEqual(tracker._next_track_id, 2)

    def test_completed_track_is_a_persistent_position_tombstone(self):
        tracker = MultiTargetTracker(max_track_age_sec=0.5)
        tracker.update([observation(1, 0.4)], stamp_sec=1.0)
        tracker.mark_harvested(1)
        # The same physical fruit remains visible after a safe skip and after
        # the normal track-age interval.  It must not return under a fresh ID.
        self.assertEqual(
            tracker.update([observation(9, 0.402)], stamp_sec=2.0),
            (),
        )
        self.assertEqual(tracker._next_track_id, 2)

    def test_completed_tombstone_does_not_hide_a_distinct_neighbour(self):
        tracker = MultiTargetTracker(association_distance_m=0.06)
        tracker.update([observation(1, 0.400)], stamp_sec=1.0)
        tracker.mark_harvested(1)
        tracks = tracker.update([observation(20, 0.454)], stamp_sec=1.1)
        self.assertEqual([row.track_id for row in tracks], [2])
        self.assertEqual(tracks[0].source_detection_id, 20)

    def test_completed_suppression_gate_must_be_narrower_than_association(self):
        with self.assertRaises(ValueError):
            MultiTargetTracker(
                association_distance_m=0.06,
                completed_suppression_distance_m=0.06,
            )

    def test_invalid_observation_fails_closed(self):
        with self.assertRaises(ValueError):
            observation(0, 0.4)
        with self.assertRaises(ValueError):
            MultiTargetTracker(association_distance_m=0.0)


if __name__ == "__main__":
    unittest.main()
