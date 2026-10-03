import importlib.util
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
VALIDATOR = (
    REPOSITORY_ROOT
    / "scripts"
    / "validate_blender_v2_pregrasp_requalification.py"
)
spec = importlib.util.spec_from_file_location(
    "_pregrasp_requalification_validator", VALIDATOR
)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def contract():
    return {
        "parameters": {
            "expected_target_id": 1,
            "expected_grasp_profile": "blender_v2_26mm",
            "expected_tool_center_offset_m": 0.0964,
            "pregrasp_offset_m": 0.15,
            "minimum_target_samples": 15,
            "minimum_joint_samples": 10,
            "maximum_joint_delta_rad": 0.002,
        },
        "thresholds": {
            "maximum_moveit_state_delta_rad": 0.002,
            "required_control_command_count": 0,
        },
    }


def passing_result():
    collisions = sorted(module.EXPECTED_COLLISIONS)
    return {
        "planning_passed": True,
        "pick_authorized": False,
        "expected_target_id": 1,
        "observed_target_ids": [1],
        "target_sample_count": 15,
        "joint_sample_count": 10,
        "grasp_geometry_profile": "blender_v2_26mm",
        "pregrasp_pose": {
            "tool_center_offset_m": 0.0964,
            "pregrasp_offset_m": 0.15,
        },
        "collision_scene": {
            "expected_ids": collisions,
            "before_ids": collisions,
            "after_ids": collisions,
            "selected_fruit_collision_retained": True,
        },
        "observed_joint_delta_rad": 0.0,
        "moveit_state_delta_rad": 0.0,
        "trajectory_generated": True,
        "trajectory_discarded": True,
        "trajectory_executed": False,
        "control_command_count": 0,
        "controller_configuration_keys": [],
        "control_interface_created": False,
        "gripper_command_sent": False,
        "pick_action_called": False,
        "violations": [],
    }


class PregraspRequalificationTests(unittest.TestCase):
    def test_complete_read_only_plan_passes(self):
        self.assertEqual(
            module.evaluate_pregrasp(contract(), passing_result()), []
        )

    def test_wrong_profile_and_executed_trajectory_fail(self):
        result = passing_result()
        result["grasp_geometry_profile"] = "tabletop_v1_35mm"
        result["trajectory_discarded"] = False
        result["trajectory_executed"] = True
        violations = module.evaluate_pregrasp(contract(), result)
        self.assertIn("wrong grasp geometry profile", violations)
        self.assertIn("trajectory was not explicitly discarded", violations)
        self.assertIn("trajectory unexpectedly executed", violations)

    def test_selected_fruit_must_remain_collision_object(self):
        result = passing_result()
        result["collision_scene"]["after_ids"].remove("strawberry_fruit_1")
        result["collision_scene"]["selected_fruit_collision_retained"] = False
        violations = module.evaluate_pregrasp(contract(), result)
        self.assertIn(
            "collision objects were missing before or after planning",
            violations,
        )
        self.assertIn("selected fruit collision was removed", violations)

    def test_runner_has_no_motion_side_path(self):
        runner = (
            REPOSITORY_ROOT
            / "scripts"
            / "run_blender_v2_pregrasp_requalification.sh"
        ).read_text(encoding="utf-8")
        for expected in (
            "start_manipulation:=false",
            "start_orchestrator:=false",
            "enable_attachment:=false",
            "enable_pose_control:=false",
            "--target-topic /strawberry/oracle/target_pose",
        ):
            self.assertIn(expected, runner)
        self.assertNotIn("test_oracle_pick_and_place.py", runner)


if __name__ == "__main__":
    unittest.main()
