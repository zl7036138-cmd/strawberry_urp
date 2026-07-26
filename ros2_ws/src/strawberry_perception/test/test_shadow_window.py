import unittest

from strawberry_perception.shadow_window_probe import (
    classify_readiness_frame,
    find_consecutive_true_run,
    summarize_readiness_trace,
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

    def test_readiness_frame_reason_is_fail_closed_and_identity_aware(self):
        self.assertEqual(
            "NO_DETECTION",
            classify_readiness_frame(
                detection_ids=[],
                ripe_detection_ids=[],
                target_pose_ids=[],
            ),
        )
        self.assertEqual(
            "NO_RIPE_DETECTION",
            classify_readiness_frame(
                detection_ids=[2],
                ripe_detection_ids=[],
                target_pose_ids=[2],
            ),
        )
        self.assertEqual(
            "TARGET_POSE_MISSING",
            classify_readiness_frame(
                detection_ids=[1],
                ripe_detection_ids=[1],
                target_pose_ids=[],
            ),
        )
        self.assertEqual(
            "TARGET_IDENTITY_MISMATCH",
            classify_readiness_frame(
                detection_ids=[1],
                ripe_detection_ids=[1],
                target_pose_ids=[3],
            ),
        )
        self.assertEqual(
            "READY",
            classify_readiness_frame(
                detection_ids=[1, 2],
                ripe_detection_ids=[1],
                target_pose_ids=[1],
            ),
        )

    def test_readiness_trace_reports_streak_resets_and_delay(self):
        statuses = [
            ("READY", 0.02),
            ("READY", 0.03),
            ("TARGET_POSE_MISSING", None),
            ("READY", 0.01),
            ("TARGET_IDENTITY_MISMATCH", 0.04),
            ("READY", 0.02),
            ("READY", 0.02),
            ("READY", 0.02),
        ]
        frames = [
            {
                "detection_index": index,
                "status": status,
                "target_pose_delay_sec": delay,
            }
            for index, (status, delay) in enumerate(statuses, start=1)
        ]
        result = summarize_readiness_trace(frames, 3)
        self.assertEqual(3, result["maximum_consecutive_ready_frames"])
        self.assertEqual(3, result["terminal_consecutive_ready_frames"])
        self.assertEqual(2, result["streak_reset_count"])
        self.assertEqual(
            {
                "TARGET_IDENTITY_MISMATCH": 1,
                "TARGET_POSE_MISSING": 1,
            },
            result["streak_reset_reason_counts"],
        )
        self.assertEqual(6, result["qualifying_run_start_detection_index"])
        self.assertEqual(8, result["qualifying_run_end_detection_index"])
        self.assertAlmostEqual(
            0.04, result["matched_target_pose_delay_sec"]["maximum"]
        )

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
