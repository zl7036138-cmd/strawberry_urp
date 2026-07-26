import unittest

from strawberry_perception.shadow_window_probe import (
    find_consecutive_true_run,
    summarize_window_frames,
)


class ShadowWindowSummaryTests(unittest.TestCase):
    def test_readiness_run_requires_consecutive_successes(self):
        self.assertEqual(
            find_consecutive_true_run(
                [True, True, False, True, True, True],
                3,
            ),
            (3, 6),
        )
        self.assertIsNone(find_consecutive_true_run([True, False, True], 2))
        with self.assertRaises(ValueError):
            find_consecutive_true_run([True], 0)

    def test_exact_window_is_summarized_by_frame(self):
        frames = [
            {
                "detection_count": 1,
                "ripe_detection_count": 1,
                "unripe_detection_count": 0,
                "target_pose_received": True,
            },
            {
                "detection_count": 0,
                "ripe_detection_count": 0,
                "unripe_detection_count": 0,
                "target_pose_received": False,
            },
        ]
        result = summarize_window_frames(frames, 2)
        self.assertEqual(1, result["frames_with_ripe_detection"])
        self.assertEqual(1, result["frames_with_target_pose"])
        self.assertEqual(1, result["detection_count"])

    def test_incomplete_window_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "wrong frame count"):
            summarize_window_frames([], 60)

    def test_confidence_summary_is_split_by_maturity(self):
        frames = [
            {
                "detection_count": 2,
                "ripe_detection_count": 1,
                "unripe_detection_count": 1,
                "detection_confidences": [0.9, 0.5],
                "ripe_confidences": [0.9],
                "unripe_confidences": [0.5],
                "target_pose_received": True,
            }
        ]
        result = summarize_window_frames(frames, 1)
        self.assertAlmostEqual(0.7, result["confidence"]["all"]["mean"])
        self.assertAlmostEqual(0.9, result["confidence"]["ripe"]["median"])
        self.assertAlmostEqual(0.5, result["confidence"]["unripe"]["maximum"])


if __name__ == "__main__":
    unittest.main()
