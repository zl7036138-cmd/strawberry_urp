import importlib.util
import pathlib


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[4]
SCRIPT = (
    REPOSITORY_ROOT
    / "scripts"
    / "evaluate_blender_v2_perception_execution_readiness.py"
)
SPEC = importlib.util.spec_from_file_location("execution_readiness", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def inputs(estimated):
    return {
        "handoff": {
            "handoff_passed": True,
            "expected_target_id": 1,
            "frozen_target_pose": {
                "target_id": 1,
                "position_xyz_m": estimated,
            },
        },
        "no_motion_gate": {"passed": True},
        "scene": {
            "fruit_collision_radius_m": 0.026,
            "fruits": [
                {
                    "target_id": 1,
                    "initial_pose_m": [0.42, -0.05, 0.55],
                }
            ],
        },
        "geometry_gate": {
            "passed": True,
            "recommendation": {
                "feasible": True,
                "tool_center_offset_m": 0.0964,
                "close_width_m_per_finger": 0.022,
                "first_contact_width_m_per_finger": 0.025857,
                "finger_axial_range_hand_z_m": [0.0585, 0.1122],
            },
        },
    }


def test_exact_target_is_execution_ready_but_does_not_authorize_pick():
    result = MODULE.evaluate_execution_readiness(
        **inputs([0.42, -0.05, 0.55])
    )
    assert result["execution_readiness_passed"] is True
    assert result["perception_execution_authorized"] is False
    assert result["pick_authorized"] is False
    assert result["violations"] == []


def test_calyx_depth_bias_fails_cross_jaw_and_axial_interlocks():
    result = MODULE.evaluate_execution_readiness(
        **inputs([0.411, -0.044, 0.578])
    )
    assert result["execution_readiness_passed"] is False
    assert len(result["violations"]) == 2
    assert (
        result["observed_cross_jaw_error_m"]
        > result["grasp_geometry"]["maximum_cross_jaw_error_m"]
    )
    assert (
        result["observed_hand_axial_position_m"]
        > result["grasp_geometry"]["finger_axial_range_hand_z_m"][1]
    )


def test_live_sequence_summary_is_accepted_only_when_motion_stays_discarded():
    values = inputs([0.42, -0.05, 0.55])
    values["no_motion_gate"] = {
        "scope": "NON_ACCEPTANCE_DUAL_CAMERA_SEQUENTIAL_OBSERVATION",
        "sequence_passed": True,
        "handoff_shadow_passed": True,
        "pregrasp_shadow_passed": True,
        "pregrasp_trajectory_discarded": True,
        "pregrasp_control_command_count": 0,
        "pick_authorized": False,
        "violations": [],
    }
    result = MODULE.evaluate_execution_readiness(**values)
    assert result["execution_readiness_passed"] is True

    values["no_motion_gate"]["pregrasp_trajectory_discarded"] = False
    try:
        MODULE.evaluate_execution_readiness(**values)
    except ValueError as exc:
        assert str(exc) == "no-motion gate input did not pass"
    else:
        raise AssertionError("an executable trajectory must fail the no-motion gate")


def test_pregrasp_shadow_is_a_valid_same_world_no_motion_gate():
    values = inputs([0.42, -0.05, 0.55])
    values["no_motion_gate"] = {
        "scope": "NON_ACCEPTANCE_PREGRASP_PLANNING_SHADOW",
        "planning_passed": True,
        "trajectory_generated": True,
        "trajectory_discarded": True,
        "trajectory_executed": False,
        "trajectory_execution_requested": False,
        "control_command_count": 0,
        "pick_action_called": False,
        "pick_authorized": False,
        "violations": [],
    }
    result = MODULE.evaluate_execution_readiness(**values)
    assert result["execution_readiness_passed"] is True
