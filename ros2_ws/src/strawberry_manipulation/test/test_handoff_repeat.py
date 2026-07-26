import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.handoff_repeat import (  # noqa: E402
    summarize_handoff_repeats,
)


class HandoffRepeatTests(unittest.TestCase):
    @staticmethod
    def sample_run(label="v1"):
        collisions = [
            "work_table",
            "strawberry_planter",
            "strawberry_plant_crown",
            "collection_bin",
            "strawberry_fruit_1",
            "strawberry_fruit_2",
            "strawberry_fruit_3",
        ]
        return {
            "label": label,
            "handoff": {
                "handoff_passed": True,
                "pick_authorized": False,
                "expected_target_id": 1,
                "observed_target_ids": [1],
                "target_sample_count": 15,
                "target_age_sec": {"maximum": 0.08},
                "joint_sample_count": 10,
                "observed_joint_delta_rad": 0.0,
                "moveit_state_delta_rad": 0.0,
                "control_interface_created": False,
                "pick_action_called": False,
                "trajectory_command_sent": False,
                "gripper_command_sent": False,
                "trajectory_execution_configured": False,
                "controller_configuration_keys": [],
                "collision_scene": {
                    "observed_ids": collisions,
                    "missing_ids": [],
                    "selected_fruit_collision_retained": True,
                },
            },
            "sequence": {
                "sequence_passed": True,
                "pick_authorized": False,
                "wrist_frame_count": 60,
                "wrist_matching_target_pose_frames": 60,
            },
            "motion": {
                "success": True,
                "fruit_manipulation_started": False,
                "fruit_collision_obstacle_ids": [1, 2, 3],
                "planning_attempt_count": 1,
            },
            "clean_logs": True,
            "no_control_log": True,
        }

    def test_three_clean_runs_pass_without_authorizing_pick(self):
        result = summarize_handoff_repeats(
            [
                self.sample_run("v1"),
                self.sample_run("v2"),
                self.sample_run("v3"),
            ]
        )
        self.assertTrue(result["repeat_passed"])
        self.assertFalse(result["pick_authorized"])
        self.assertEqual(result["total_target_samples"], 45)
        self.assertEqual(
            result["total_wrist_matching_target_pose_frames"], 180
        )
        self.assertEqual(result["control_command_count"], 0)

    def test_fewer_than_three_runs_fail_closed(self):
        result = summarize_handoff_repeats(
            [self.sample_run("v1"), self.sample_run("v2")]
        )
        self.assertFalse(result["repeat_passed"])

    def test_one_control_command_or_missing_collision_fails_all_runs(self):
        runs = [
            self.sample_run("v1"),
            self.sample_run("v2"),
            self.sample_run("v3"),
        ]
        runs[1]["handoff"]["trajectory_command_sent"] = True
        runs[2]["handoff"]["collision_scene"]["missing_ids"] = [
            "strawberry_fruit_1"
        ]
        result = summarize_handoff_repeats(runs)
        self.assertFalse(result["repeat_passed"])
        self.assertEqual(len(result["violations"]), 2)


if __name__ == "__main__":
    unittest.main()
