import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.pregrasp_repeat import (  # noqa: E402
    summarize_pregrasp_repeats,
)


class PregraspRepeatTests(unittest.TestCase):
    @staticmethod
    def sample_run(label):
        collision_ids = [
            "collection_bin",
            "strawberry_fruit_1",
            "strawberry_fruit_2",
            "strawberry_fruit_3",
            "strawberry_plant_crown",
            "strawberry_planter",
            "work_table",
        ]
        return {
            "run_label": label,
            "all_logs_clean": True,
            "control_logs_clean": True,
            "sequence": {
                "sequence_passed": True,
                "formal_acceptance": False,
                "held_out_test_consumed": False,
                "pick_authorized": False,
                "candidate_target_id": 1,
                "wrist_matching_target_pose_frames": 60,
            },
            "handoff": {
                "handoff_passed": True,
                "pick_authorized": False,
                "expected_target_id": 1,
            },
            "pregrasp": {
                "scope": "NON_ACCEPTANCE_PREGRASP_PLANNING_SHADOW",
                "planning_passed": True,
                "formal_acceptance": False,
                "held_out_test_consumed": False,
                "pick_authorized": False,
                "expected_target_id": 1,
                "target_sample_count": 15,
                "joint_sample_count": 10,
                "observed_joint_delta_rad": 0.0,
                "moveit_state_delta_rad": 0.0,
                "target_age_sec": {"at_plan_completion": 0.08},
                "planning_attempt_count": 1,
                "planning_attempts": [
                    {
                        "success": True,
                        "trajectory_waypoint_count": 38,
                        "endpoint_position_error_m": 0.0008,
                        "endpoint_orientation_error_rad": 0.01,
                    }
                ],
                "trajectory_generated": True,
                "trajectory_discarded": True,
                "trajectory_executed": False,
                "trajectory_execution_requested": False,
                "trajectory_execution_time_sec": 0.0,
                "control_interface_created": False,
                "controller_configuration_keys": [],
                "control_command_count": 0,
                "pick_action_called": False,
                "gripper_command_sent": False,
                "collision_scene": {
                    "before_ids": collision_ids,
                    "after_ids": collision_ids,
                    "missing_before": [],
                    "missing_after": [],
                    "selected_fruit_collision_retained": True,
                },
            },
        }

    def test_three_controller_free_discarded_plans_pass(self):
        result = summarize_pregrasp_repeats(
            [self.sample_run(f"run_{index}") for index in range(1, 4)]
        )
        self.assertTrue(result["repeat_passed"])
        self.assertEqual(result["planning_success_count"], 3)
        self.assertEqual(result["total_wrist_target_pose_frames"], 180)
        self.assertEqual(result["total_target_samples"], 45)
        self.assertEqual(result["total_control_command_count"], 0)
        self.assertEqual(result["trajectory_waypoint_count"]["minimum"], 38)

    def test_execution_or_controller_log_fails_closed(self):
        runs = [self.sample_run(f"run_{index}") for index in range(1, 4)]
        runs[1]["pregrasp"]["trajectory_executed"] = True
        runs[1]["pregrasp"]["trajectory_discarded"] = False
        runs[1]["pregrasp"]["control_command_count"] = 1
        runs[1]["control_logs_clean"] = False
        result = summarize_pregrasp_repeats(runs)
        self.assertFalse(result["repeat_passed"])
        self.assertIn(
            "run_2: pre-grasp trajectory was executed",
            result["violations"],
        )
        self.assertIn(
            "run_2: control-side logs violate the no-motion audit",
            result["violations"],
        )

    def test_stale_target_or_missing_collision_fails_closed(self):
        runs = [self.sample_run(f"run_{index}") for index in range(1, 4)]
        runs[2]["pregrasp"]["target_age_sec"]["at_plan_completion"] = 0.8
        runs[2]["pregrasp"]["collision_scene"]["after_ids"] = []
        result = summarize_pregrasp_repeats(runs)
        self.assertFalse(result["repeat_passed"])
        self.assertIn(
            "run_3: target was not fresh at plan completion",
            result["violations"],
        )
        self.assertIn(
            "run_3: planning collision scene did not retain 7 objects",
            result["violations"],
        )

    def test_fewer_than_three_completed_runs_is_rejected(self):
        with self.assertRaises(ValueError):
            summarize_pregrasp_repeats(
                [self.sample_run("run_1"), self.sample_run("run_2")]
            )


if __name__ == "__main__":
    unittest.main()
