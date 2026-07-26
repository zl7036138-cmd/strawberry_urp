import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_bringup.observation_sequence import (  # noqa: E402
    summarize_observation_sequence,
)


class ObservationSequenceTests(unittest.TestCase):
    @staticmethod
    def inputs(target_id=1):
        selection = {
            "candidate_target_id": target_id,
            "selected_preset": "lower",
            "focus_roi_xyxy_px": [320, 240, 640, 480],
            "support_frames": 60,
            "frame_count": 60,
            "wrist_observation_authorized": True,
            "pick_authorized": False,
        }
        motion = {
            "success": True,
            "fruit_manipulation_started": False,
            "fruit_collision_obstacle_ids": [1, 2, 3],
            "planning_time_sec": 0.05,
            "execution_time_sec": 10.0,
        }
        wrist = {
            "formal_acceptance": False,
            "window_boundary": (
                "after_wrist_observation_settle_before_pick_motion"
            ),
            "readiness_gate": {
                "required_consecutive_target_pose_frames": 15,
                "satisfied": True,
            },
            "summary": {
                "frame_count": 60,
                "detection_count": 120,
                "ripe_detection_count": 120,
                "unripe_detection_count": 0,
            },
            "frames": [{"target_pose_ids": [target_id]} for _ in range(60)],
        }
        return selection, motion, wrist

    @staticmethod
    def valid_handoff(target_id=1):
        return {
            "scope": "NON_ACCEPTANCE_OBSERVATION_TO_CONTROL_HANDOFF_SHADOW",
            "handoff_passed": True,
            "pick_authorized": False,
            "expected_target_id": target_id,
            "target_sample_count": 15,
            "joint_sample_count": 10,
            "observed_joint_delta_rad": 0.0001,
            "moveit_state_delta_rad": 0.0001,
            "pick_action_called": False,
            "trajectory_command_sent": False,
            "gripper_command_sent": False,
            "collision_scene": {
                "missing_ids": [],
                "selected_fruit_collision_retained": True,
            },
        }

    @staticmethod
    def valid_pregrasp(target_id=1):
        return {
            "scope": "NON_ACCEPTANCE_PREGRASP_PLANNING_SHADOW",
            "planning_passed": True,
            "pick_authorized": False,
            "expected_target_id": target_id,
            "target_sample_count": 15,
            "joint_sample_count": 10,
            "planning_attempt_count": 1,
            "target_age_sec": {"at_plan_completion": 0.09},
            "trajectory_generated": True,
            "trajectory_discarded": True,
            "trajectory_executed": False,
            "trajectory_execution_requested": False,
            "trajectory_execution_time_sec": 0.0,
            "control_command_count": 0,
            "controller_configuration_keys": [],
            "control_interface_created": False,
            "pick_action_called": False,
            "gripper_command_sent": False,
            "collision_scene": {
                "missing_before": [],
                "missing_after": [],
                "selected_fruit_collision_retained": True,
            },
        }

    def test_matching_same_world_sequence_passes_without_authorizing_pick(self):
        result = summarize_observation_sequence(*self.inputs())
        self.assertTrue(result["sequence_passed"])
        self.assertFalse(result["pick_authorized"])
        self.assertEqual(result["wrist_matching_target_pose_frames"], 60)
        self.assertEqual(result["wrist_readiness_consecutive_frames"], 15)

    def test_missing_readiness_proof_fails_closed(self):
        selection, motion, wrist = self.inputs()
        wrist["readiness_gate"]["satisfied"] = False
        result = summarize_observation_sequence(selection, motion, wrist)
        self.assertFalse(result["sequence_passed"])
        self.assertIn(
            "wrist pipeline did not prove at least 10 consecutive ready frames",
            result["violations"],
        )

    def test_missing_obstacle_or_wrong_wrist_identity_fails_closed(self):
        selection, motion, wrist = self.inputs()
        motion["fruit_collision_obstacle_ids"] = [1, 3]
        wrist["frames"] = [{"target_pose_ids": [3]} for _ in range(60)]
        result = summarize_observation_sequence(selection, motion, wrist)
        self.assertFalse(result["sequence_passed"])
        self.assertIn(
            "observation motion omitted one or more fruit obstacles",
            result["violations"],
        )
        self.assertIn(
            "wrist window published an unexpected target identity",
            result["violations"],
        )

    def test_control_handoff_shadow_is_integrated_without_authorizing_pick(self):
        selection, motion, wrist = self.inputs()
        handoff = self.valid_handoff()
        result = summarize_observation_sequence(
            selection, motion, wrist, handoff
        )
        self.assertTrue(result["sequence_passed"])
        self.assertTrue(result["handoff_shadow_included"])
        self.assertTrue(result["handoff_shadow_passed"])
        self.assertFalse(result["pick_authorized"])
        self.assertIn(
            "CONTROL_HANDOFF_SHADOW_VALIDATED", result["state_history"]
        )

    def test_pregrasp_plan_is_integrated_and_discarded_without_control(self):
        selection, motion, wrist = self.inputs()
        result = summarize_observation_sequence(
            selection,
            motion,
            wrist,
            self.valid_handoff(),
            self.valid_pregrasp(),
        )
        self.assertTrue(result["sequence_passed"])
        self.assertEqual(result["schema_version"], 3)
        self.assertTrue(result["pregrasp_shadow_included"])
        self.assertTrue(result["pregrasp_shadow_passed"])
        self.assertTrue(result["pregrasp_trajectory_discarded"])
        self.assertEqual(result["pregrasp_control_command_count"], 0)
        self.assertFalse(result["pick_authorized"])
        self.assertIn(
            "PREGRASP_PLANNING_SHADOW_VALIDATED", result["state_history"]
        )

    def test_pregrasp_execution_or_missing_collision_fails_closed(self):
        selection, motion, wrist = self.inputs()
        pregrasp = self.valid_pregrasp()
        pregrasp["trajectory_executed"] = True
        pregrasp["trajectory_discarded"] = False
        pregrasp["control_command_count"] = 1
        pregrasp["collision_scene"]["missing_after"] = [
            "strawberry_fruit_1"
        ]
        result = summarize_observation_sequence(
            selection, motion, wrist, self.valid_handoff(), pregrasp
        )
        self.assertFalse(result["sequence_passed"])
        self.assertIn(
            "pre-grasp trajectory was executed", result["violations"]
        )
        self.assertIn(
            "pre-grasp planning emitted a control command",
            result["violations"],
        )
        self.assertIn(
            "pre-grasp planning collision scene is incomplete",
            result["violations"],
        )

    def test_pregrasp_without_handoff_fails_closed(self):
        selection, motion, wrist = self.inputs()
        result = summarize_observation_sequence(
            selection, motion, wrist, None, self.valid_pregrasp()
        )
        self.assertFalse(result["sequence_passed"])
        self.assertIn(
            "pre-grasp planning has no upstream handoff audit",
            result["violations"],
        )

    def test_failed_control_handoff_fails_the_whole_sequence(self):
        selection, motion, wrist = self.inputs()
        handoff = {
            "scope": "NON_ACCEPTANCE_OBSERVATION_TO_CONTROL_HANDOFF_SHADOW",
            "handoff_passed": False,
            "pick_authorized": False,
            "expected_target_id": 1,
            "pick_action_called": False,
            "trajectory_command_sent": False,
            "gripper_command_sent": False,
            "collision_scene": {
                "missing_ids": ["strawberry_fruit_1"],
                "selected_fruit_collision_retained": False,
            },
        }
        result = summarize_observation_sequence(
            selection, motion, wrist, handoff
        )
        self.assertFalse(result["sequence_passed"])
        self.assertIn(
            "control handoff shadow audit did not pass",
            result["violations"],
        )


if __name__ == "__main__":
    unittest.main()
