"""Plan-A control contract tests (v20 receipt, user decision).

The gz_ros2_control effort path injects velocity kicks scaled by inverse
joint inertia (j7 bang-bang at the velocity clamp, j4 kicked 0.09 rad);
the position interface tracks cleanly (v9-v11). The arm therefore runs
the position interface for all seven joints, and the surviving
goal-boundary kick is guarded by the backend zero-velocity confirmation
window instead of force-level gravity compensation.
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


class PlanAControlContractTests(unittest.TestCase):
    def test_urdf_declares_position_only_command_interfaces(self):
        text = _load_xacro_text()
        macro_start = text.index('<xacro:macro name="arm_control_joint"')
        macro_end = text.index("</xacro:macro>", macro_start)
        macro = text[macro_start:macro_end]
        self.assertIn('<command_interface name="position"/>', macro)
        self.assertNotIn('<command_interface name="effort"/>', macro)
        for joint in ARM_JOINTS:
            self.assertIn(
                f'<xacro:arm_control_joint name="{joint}"/>', text
            )

    def test_arm_controller_commands_position_for_all_seven_joints(self):
        config = _load_controllers_yaml()
        arm = config["panda_arm_controller"]["ros__parameters"]
        self.assertEqual(arm["command_interfaces"], ["position"])
        self.assertEqual(list(arm["joints"]), list(ARM_JOINTS))

    def test_no_wrist_roll_controller_remains(self):
        config = _load_controllers_yaml()
        self.assertNotIn("panda_wrist_roll_controller", config)

    def test_arm_controller_keeps_strict_tolerances(self):
        config = _load_controllers_yaml()
        arm = config["panda_arm_controller"]["ros__parameters"]
        constraints = arm["constraints"]
        for joint in ARM_JOINTS:
            self.assertEqual(constraints[joint]["goal"], 0.05)
            self.assertEqual(constraints[joint]["trajectory"], 0.05)

    def test_gz_plugin_keeps_documented_position_gain(self):
        text = _load_xacro_text()
        self.assertIn("<position_proportional_gain>1.0", text)
