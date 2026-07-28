import json
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.gripper_contact_gate import (  # noqa: E402
    summarize_contact_trial,
)


def passing_contract():
    return {
        "gate_id": "blender_v2_gripper_contact_round_trip_v1",
        "scope": "NON_ACCEPTANCE_BLENDER_V2_GRIPPER_CONTACT_RUNTIME",
        "parameters": {
            "predicted_contact_width_m_per_finger": 0.025857178634063095,
        },
        "thresholds": {
            "maximum_arm_joint_delta_rad": 0.002,
            "maximum_finger_symmetry_error_m": 0.002,
            "maximum_contact_width_error_m": 0.003,
            "maximum_pad_center_distance_m": 0.029,
            "maximum_target_pose_error_m": 0.002,
            "maximum_restore_pose_error_m": 0.002,
            "maximum_open_recovery_error_m": 0.003,
            "required_gripper_command_count": 3,
        },
    }


def passing_observations():
    return {
        "maximum_arm_joint_delta_rad": 0.0001,
        "gripper_command_count": 3,
        "initial_open_succeeded": True,
        "close_succeeded_or_stalled": True,
        "reopen_succeeded": True,
        "closed_finger_symmetry_error_m": 0.0001,
        "measured_contact_width_m_per_finger": 0.0259,
        "raw_left_target_contact_seen": True,
        "raw_right_target_contact_seen": True,
        "non_target_contact_seen": False,
        "pad_center_distances_m": {"left": 0.028, "right": 0.028},
        "fruit_projected_between_pads": True,
        "target_pose_error_m": 0.0001,
        "attach_succeeded": True,
        "detach_succeeded": True,
        "restore_pose_error_m": 0.0001,
        "open_recovery_error_m": 0.0001,
        "planning_started": False,
        "pick_action_started": False,
    }


class GripperContactGateTests(unittest.TestCase):
    def test_pass_requires_complete_round_trip_and_keeps_pick_denied(self):
        summary = summarize_contact_trial(
            contract=passing_contract(),
            observations=passing_observations(),
            runtime_errors=[],
        )
        self.assertTrue(summary["passed"])
        self.assertFalse(summary["pick_authorized"])
        self.assertFalse(summary["robot_arm_motion_started"])

    def test_missing_raw_contact_fails_closed(self):
        observations = passing_observations()
        observations["raw_right_target_contact_seen"] = False
        summary = summarize_contact_trial(
            contract=passing_contract(),
            observations=observations,
            runtime_errors=[],
        )
        self.assertFalse(summary["passed"])
        self.assertIn(
            "raw right-finger target contact was not observed",
            summary["violations"],
        )

    def test_arm_drift_and_failed_restore_are_independent_violations(self):
        observations = passing_observations()
        observations["maximum_arm_joint_delta_rad"] = 0.003
        observations["restore_pose_error_m"] = 0.003
        summary = summarize_contact_trial(
            contract=passing_contract(),
            observations=observations,
            runtime_errors=[],
        )
        self.assertFalse(summary["passed"])
        self.assertEqual(len(summary["violations"]), 2)

    def test_runtime_topology_excludes_planning_and_pick(self):
        runner = (
            REPOSITORY_ROOT
            / "scripts"
            / "run_blender_v2_gripper_contact_gate.sh"
        ).read_text(encoding="utf-8")
        probe = (
            REPOSITORY_ROOT
            / "scripts"
            / "run_blender_v2_gripper_contact_trial.py"
        ).read_text(encoding="utf-8")
        self.assertIn("ros2 launch strawberry_sim sim.launch.py", runner)
        self.assertNotIn("system.launch.py", runner)
        self.assertNotIn("PickAndPlace", probe)
        self.assertNotIn("MoveIt", probe)
        self.assertIn('"pick_action_started": False', probe)
        self.assertIn(
            '"/panda_gripper_right_controller/gripper_cmd"',
            probe,
        )

    def test_contract_if_present_has_narrow_authority(self):
        path = (
            REPOSITORY_ROOT
            / "config"
            / "blender_v2_gripper_contact_round_trip_v1.json"
        )
        if not path.exists():
            self.skipTest("contract is added after runtime sources are bound")
        contract = json.loads(path.read_text(encoding="utf-8"))
        safety = contract["safety"]
        self.assertTrue(safety["gripper_command_authorized"])
        self.assertTrue(safety["simulated_fruit_motion_authorized"])
        self.assertTrue(safety["attachment_authorized"])
        self.assertFalse(safety["robot_arm_motion_authorized"])
        self.assertFalse(safety["planning_authorized"])
        self.assertFalse(safety["pick_action_authorized"])


if __name__ == "__main__":
    unittest.main()
