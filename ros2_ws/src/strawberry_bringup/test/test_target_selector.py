from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.harvest_planning import generate_dynamic_views  # noqa: E402
from strawberry_bringup.target_selector import (  # noqa: E402
    axis_aligned_box_clearance,
    expanded_bin_bounds,
    inside_conservative_reach,
    inside_observation_reach,
    register_completed_track,
)


class TargetSelectorWorkspaceTests(unittest.TestCase):
    def test_new_completion_reselects_from_cached_tracks_once(self):
        excluded = set()

        self.assertTrue(
            register_completed_track(excluded, 4, has_cached_targets=True)
        )
        self.assertEqual(excluded, {4})
        self.assertFalse(
            register_completed_track(excluded, 4, has_cached_targets=True)
        )

    def test_completion_without_cache_or_valid_identity_does_not_reselect(self):
        excluded = set()

        self.assertFalse(
            register_completed_track(excluded, 4, has_cached_targets=False)
        )
        self.assertEqual(excluded, {4})
        self.assertFalse(
            register_completed_track(excluded, 0, has_cached_targets=True)
        )
        self.assertEqual(excluded, {4})

    def test_valid_fruit_and_nearer_wrist_view_use_distinct_bounds(self):
        fruit = (0.50, 0.0, 0.55)
        forward_view = generate_dynamic_views(
            1,
            fruit,
            distances_m=(0.22,),
            azimuths_deg=(0.0,),
            elevations_deg=(15.0,),
        )[0]

        self.assertTrue(inside_conservative_reach(fruit))
        self.assertFalse(inside_conservative_reach(forward_view.position))
        self.assertTrue(inside_observation_reach(forward_view.position))

    def test_observation_prefilter_rejects_clearly_impossible_pose(self):
        self.assertFalse(inside_observation_reach((-0.40, 0.0, 1.20)))

    def test_bin_clearance_rejects_target_at_wall_and_keeps_distant_target(self):
        bounds = expanded_bin_bounds(
            (0.19, 0.51, -0.63, -0.27, 0.28, 0.55)
        )
        self.assertLess(
            axis_aligned_box_clearance((0.3818, -0.2608, 0.5493), bounds),
            0.02,
        )
        self.assertGreater(
            axis_aligned_box_clearance((0.5086, -0.1065, 0.5457), bounds),
            0.10,
        )

    def test_bin_outer_bounds_cover_the_physical_back_wall(self):
        bounds = expanded_bin_bounds(
            (0.19, 0.51, -0.63, -0.27, 0.28, 0.55)
        )
        for actual, expected in zip(
            bounds, (0.14, 0.56, -0.68, -0.22, 0.25, 0.55)
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertLess(
            axis_aligned_box_clearance((0.31, -0.215, 0.541), bounds),
            0.01,
        )


if __name__ == "__main__":
    unittest.main()
