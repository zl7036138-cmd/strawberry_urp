import unittest

from strawberry_bringup.observation_repeat import (
    summarize_observation_repeats,
)


class ObservationRepeatTests(unittest.TestCase):
    @staticmethod
    def sample_run(label: str, matching_frames: int = 60):
        return {
            "run_label": label,
            "sequence_passed": True,
            "clean_shutdown": True,
            "pick_authorized": False,
            "candidate_target_id": 1,
            "selected_preset": "lower",
            "base_support_frames": 60,
            "wrist_frame_count": 60,
            "wrist_matching_target_pose_frames": matching_frames,
            "wrist_detection_count": 120,
            "wrist_unripe_detection_count": 0,
            "wrist_readiness_satisfied": True,
            "wrist_readiness_consecutive_frames": 15,
            "schema_version": 3,
            "wrist_readiness_maximum_consecutive_frames": 15,
            "wrist_readiness_streak_reset_count": 0,
            "wrist_readiness_streak_reset_reason_counts": {},
            "wrist_readiness_status_counts": {
                "READY": 15,
                "TARGET_POSE_MISSING": 5,
            },
            "wrist_readiness_target_pose_delay_sec": {
                "count": 15,
                "minimum": 0.01,
                "mean": 0.02,
                "maximum": 0.04,
            },
        }

    def test_five_exact_non_pick_repetitions_pass(self):
        result = summarize_observation_repeats(
            [self.sample_run(f"run_{index}") for index in range(5)]
        )
        self.assertTrue(result["repeat_passed"])
        self.assertFalse(result["pick_authorized"])
        self.assertEqual(result["target_pose_frames"]["total"], 300)
        self.assertEqual(
            75, result["wrist_readiness"]["status_counts"]["READY"]
        )
        self.assertEqual(
            25,
            result["wrist_readiness"]["status_counts"][
                "TARGET_POSE_MISSING"
            ],
        )
        self.assertAlmostEqual(
            0.02,
            result["wrist_readiness"]["target_pose_delay_sec"]["mean"],
        )

    def test_readiness_resets_are_aggregated(self):
        runs = [self.sample_run(f"run_{index}") for index in range(5)]
        runs[1]["wrist_readiness_streak_reset_count"] = 2
        runs[1]["wrist_readiness_streak_reset_reason_counts"] = {
            "TARGET_POSE_MISSING": 2,
        }
        result = summarize_observation_repeats(runs)
        self.assertTrue(result["repeat_passed"])
        self.assertEqual(2, result["wrist_readiness"]["streak_reset_count"])
        self.assertEqual(
            {"TARGET_POSE_MISSING": 2},
            result["wrist_readiness"]["streak_reset_reason_counts"],
        )

    def test_schema_three_short_telemetry_streak_fails_closed(self):
        runs = [self.sample_run(f"run_{index}") for index in range(5)]
        runs[3]["wrist_readiness_maximum_consecutive_frames"] = 14
        result = summarize_observation_repeats(runs)
        self.assertFalse(result["repeat_passed"])
        self.assertIn(
            "run_3: readiness telemetry did not prove the required streak",
            result["violations"],
        )

    def test_one_incomplete_run_fails_the_repeat(self):
        runs = [self.sample_run(f"run_{index}") for index in range(5)]
        runs[-1]["wrist_matching_target_pose_frames"] = 59
        result = summarize_observation_repeats(runs)
        self.assertFalse(result["repeat_passed"])
        self.assertIn(
            "run_4: target identity was not localized in all frames",
            result["violations"],
        )

    def test_traceback_marked_run_fails_the_repeat(self):
        runs = [self.sample_run(f"run_{index}") for index in range(5)]
        runs[2]["clean_shutdown"] = False
        result = summarize_observation_repeats(runs)
        self.assertFalse(result["repeat_passed"])
        self.assertIn(
            "run_2: run logs contain an unhandled failure",
            result["violations"],
        )

    def test_fewer_than_five_runs_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "need at least 5"):
            summarize_observation_repeats([self.sample_run("run_1")])


if __name__ == "__main__":
    unittest.main()
