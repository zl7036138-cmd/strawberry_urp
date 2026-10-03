"""Plan-A final control contract (v32 receipt).

The gravity-compensation route is closed on ROS 2 Jazzy: the upstream
compensate_gravity implementation reads command-interface parameters,
which the Jazzy hardware_interface InterfaceInfo struct does not carry
(name/min/max/initial_value/data_type only). The arm therefore runs the
position interface, with unchanged tolerances, zero-velocity confirmation
and a stationary head. Path-tolerance aborts are not classified as kicks;
old paths must not be replayed after a displaced-state failure. Historical
release during failure recovery is not verified placement.
"""

import unittest
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]
CONTROLLERS_YAML = (
    REPO_ROOT
    / "ros2_ws/src/strawberry_sim/config/panda_controllers.yaml"
)
XACRO = REPO_ROOT / "ros2_ws/src/strawberry_sim/urdf/panda_gz.urdf.xacro"

ARM_JOINTS = tuple(f"panda_joint{i}" for i in range(1, 8))


def _load_controllers_yaml() -> dict:
    return yaml.safe_load(CONTROLLERS_YAML.read_text(encoding="utf-8"))


def _load_xacro_text() -> str:
    return XACRO.read_text(encoding="utf-8")


class PlanAFinalControlContractTests(unittest.TestCase):
    def test_urdf_declares_position_only_command_interfaces(self):
        text = _load_xacro_text()
        macro_start = text.index('<xacro:macro name="arm_control_joint"')
        macro_end = text.index("</xacro:macro>", macro_start)
        macro = text[macro_start:macro_end]
        self.assertIn('<command_interface name="position"/>', macro)
        self.assertNotIn('<command_interface name="effort">', macro)
        for joint in ARM_JOINTS:
            self.assertIn(
                f'<xacro:arm_control_joint name="{joint}"/>', text
            )

    def test_arm_controller_commands_position_for_all_seven_joints(self):
        config = _load_controllers_yaml()
        arm = config["panda_arm_controller"]["ros__parameters"]
        self.assertEqual(arm["command_interfaces"], ["position"])
        self.assertEqual(list(arm["joints"]), list(ARM_JOINTS))
        self.assertNotIn("gains", arm)

    def test_arm_controller_keeps_strict_tolerances(self):
        config = _load_controllers_yaml()
        constraints = config["panda_arm_controller"]["ros__parameters"][
            "constraints"
        ]
        for joint in ARM_JOINTS:
            self.assertEqual(constraints[joint]["goal"], 0.05)
            self.assertEqual(constraints[joint]["trajectory"], 0.05)

    def test_launch_keeps_zero_velocity_window_defenses(self):
        backend = (
            REPO_ROOT
            / "ros2_ws/src/strawberry_manipulation/strawberry_manipulation/"
            "moveit_backend.py"
        )
        text = backend.read_text(encoding="utf-8")
        self.assertIn("_wait_for_zero_velocity_between_goals", text)
        self.assertIn("_stationary_head_points", text)
