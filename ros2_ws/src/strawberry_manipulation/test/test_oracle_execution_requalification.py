import importlib.util
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
VALIDATOR = (
    REPOSITORY_ROOT
    / "scripts"
    / "validate_blender_v2_oracle_execution.py"
)
spec = importlib.util.spec_from_file_location(
    "_oracle_execution_validator", VALIDATOR
)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def contract():
    return {
        "parameters": {
            "target_id": 1,
            "target_topic": "/strawberry/ground_truth/fruit_1/pose",
            "place_position_m": [0.35, -0.45, 0.45],
            "gripper_open_width_m_per_finger": 0.04,
        },
        "thresholds": {
            "maximum_planning_time_sec": 5.0,
            "maximum_ready_joint_error_rad": 0.02,
            "maximum_home_return_delta_rad": 0.02,
            "maximum_open_recovery_error_m": 0.003,
        },
    }


def passing_trial():
    ready = dict(module.READY_JOINTS)
    return {
        "success": True,
        "result_success": True,
        "action_status": 4,
        "failure_code": 0,
        "planning_time_sec": 0.5,
        "target_id": 1,
        "target_topic": "/strawberry/ground_truth/fruit_1/pose",
        "place_position_m": [0.35, -0.45, 0.45],
        "feedback": [
            {"stage": stage} for stage in module.EXPECTED_STAGES
        ],
        "diagnostics": {
            "contacts": {
                side: {
                    "processed_seen_true": True,
                    "raw_contacts": 2,
                }
                for side in ("left", "right")
            },
            "fruit_contacts": {"first_pair_events": []},
            "attachment_state": {
                "initial_attached": False,
                "final_attached": False,
                "events": [
                    {"attached": False, "stage": "WAITING"},
                    {"attached": True, "stage": "GRASP"},
                    {"attached": False, "stage": "PLACE"},
                ],
            },
            "arm_recovery": {
                "initial_positions_rad": ready,
                "final_positions_rad": ready,
                "maximum_initial_final_delta_rad": 0.0,
            },
            "gripper_recovery": {
                "final_positions_m": {
                    "panda_finger_joint1": 0.04,
                    "panda_finger_joint2": 0.04,
                }
            },
        },
    }


PROFILE_LOG = (
    "Loaded grasp geometry profile blender_v2_26mm: "
    "tool_center_offset_m=0.0964, "
    "gripper_closed_width_m_per_finger=0.022"
)


class OracleExecutionRequalificationTests(unittest.TestCase):
    def test_complete_round_trip_passes(self):
        violations, observations = module.evaluate_execution(
            contract(), passing_trial(), PROFILE_LOG
        )
        self.assertEqual(violations, [])
        self.assertEqual(observations["home_return_delta_rad"], 0.0)

    def test_missing_contact_and_home_recovery_fail(self):
        trial = passing_trial()
        trial["diagnostics"]["contacts"]["left"]["raw_contacts"] = 0
        trial["diagnostics"]["arm_recovery"][
            "maximum_initial_final_delta_rad"
        ] = 0.03
        violations, _ = module.evaluate_execution(
            contract(), trial, PROFILE_LOG
        )
        self.assertIn("left raw contact missing", violations)
        self.assertIn("final arm state did not return home", violations)

    def test_attachment_round_trip_and_unexpected_contact_fail_closed(self):
        trial = passing_trial()
        trial["diagnostics"]["attachment_state"]["events"] = [
            {"attached": False, "stage": "WAITING"},
            {"attached": True, "stage": "GRASP"},
        ]
        trial["diagnostics"]["attachment_state"]["final_attached"] = True
        trial["diagnostics"]["fruit_contacts"]["first_pair_events"] = [
            {
                "pair": [
                    "strawberry_1::fruit_link::fruit_collision",
                    "strawberry_2::fruit_link::fruit_collision",
                ],
                "stage": "RETREAT",
            }
        ]
        violations, _ = module.evaluate_execution(
            contract(), trial, PROFILE_LOG
        )
        self.assertIn(
            "attach/detach round trip was not observed", violations
        )
        self.assertIn("target remained attached", violations)
        self.assertIn("unexpected fruit contact occurred", violations)

    def test_runner_is_single_truth_controlled_execution(self):
        runner = (
            REPOSITORY_ROOT
            / "scripts"
            / "run_blender_v2_oracle_execution_round_trip.sh"
        ).read_text(encoding="utf-8")
        for expected in (
            "camera_mount:=dual",
            "start_perception:=false",
            "start_oracle_provider:=false",
            "start_manipulation:=true",
            "start_orchestrator:=false",
            "enable_attachment:=true",
            "enable_pose_control:=false",
            "--target-id 1",
            "--target-topic /strawberry/ground_truth/fruit_1/pose",
        ):
            self.assertIn(expected, runner)
        self.assertNotIn("for trial_", runner)
        self.assertIn("refusing to overwrite frozen output", runner)


if __name__ == "__main__":
    unittest.main()
