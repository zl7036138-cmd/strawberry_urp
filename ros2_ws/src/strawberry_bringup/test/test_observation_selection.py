import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_bringup.observation_selection import (  # noqa: E402
    select_observation_preset,
)


class ObservationSelectionTests(unittest.TestCase):
    @staticmethod
    def frames(identity, count=60):
        return [
            {"frame_index": index + 1, "target_pose_ids": [identity]}
            for index in range(count)
        ]

    def test_target_one_selects_lower_attention_preset(self):
        result = select_observation_preset(self.frames(1))
        self.assertEqual(result["selected_preset"], "lower")
        self.assertEqual(result["focus_roi_xyxy_px"], [320, 240, 640, 480])
        self.assertEqual(result["support_fraction"], 1.0)

    def test_target_three_selects_center_attention_preset(self):
        result = select_observation_preset(self.frames(3))
        self.assertEqual(result["selected_preset"], "center")
        self.assertEqual(result["focus_roi_xyxy_px"], [0, 0, 320, 480])

    def test_missing_weak_ambiguous_or_unmapped_candidate_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "no localized"):
            select_observation_preset([{"target_pose_ids": []}] * 60)
        with self.assertRaisesRegex(ValueError, "below"):
            select_observation_preset(
                self.frames(1, 47) + [{"target_pose_ids": []}] * 13
            )
        with self.assertRaisesRegex(ValueError, "ambiguous"):
            select_observation_preset(
                self.frames(1, 30) + self.frames(3, 30)
            )
        with self.assertRaisesRegex(ValueError, "no bounded"):
            select_observation_preset(self.frames(2))


if __name__ == "__main__":
    unittest.main()
