"""Contract tests for the effort-interface arm control change.

v9/v10/v11 telemetry (seed 45504): the gz_ros2_control position interface tracks
with a single proportional velocity law; with the fruit attached the wrist
pitch (panda_joint5) diverged from its setpoint and aborted recovery home at
0.0531 rad vs the 0.05 tolerance. Raising the plugin gain to 3.0 produced a
5.5 Hz limit cycle (v11, reverted). True gravity compensation requires effort
command interfaces with closed-loop trajectory control.

These tests pin the configuration contract:
- the URDF declares an effort command interface for every arm joint;
- the arm controller commands effort and carries per-joint PID gains plus a
  velocity feedforward scale;
- the finger controllers keep their position interfaces (unchanged).
"""

import math
from pathlib import Path

import unittest

import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]
CONTROLLERS_YAML = (
    REPO_ROOT
    / "ros2_ws/src/strawberry_sim/config/panda_controllers.yaml"
)
XACRO = REPO_ROOT / "ros2_ws/src/strawberry_sim/urdf/panda_gz.urdf.xacro"

ARM_JOINTS = tuple(f"panda_joint{index}" for index in range(1, 8))


def _load_controllers_yaml() -> dict:
    return yaml.safe_load(CONTROLLERS_YAML.read_text(encoding="utf-8"))


def _load_xacro_text() -> str:
    return XACRO.read_text(encoding="utf-8")


class EffortInterfaceContractTests(unittest.TestCase):
    def test_urdf_declares_effort_only_command_interface_for_j1_to_j6(self):
        """v17: an unclaimed position command interface fights the effort PID.

        With both interfaces declared, the unclaimed position path still
        applies stale commands. Effort is the only command interface in the
        shared j1-j6 macro.
        """
        text = _load_xacro_text()
        macro_start = text.index('<xacro:macro name="arm_control_joint"')
        macro_end = text.index("</xacro:macro>", macro_start)
        macro = text[macro_start:macro_end]
        self.assertIn('<command_interface name="effort"/>', macro)
        self.assertNotIn('<command_interface name="position"/>', macro)

    def test_joint7_runs_its_own_position_interface_joint(self):
        """v19: j7's tiny inertia saturates the effort path's velocity clamp.

        Four gain variants produced identical bang-bang signatures. The
        hybrid contract gives joint7 its own position-interface joint in the
        URDF, driven by a dedicated single-joint JTC.
        """
        text = _load_xacro_text()
        self.assertIn('<joint name="panda_joint7">', text)
        j7_start = text.index('<joint name="panda_joint7">')
        j7_end = text.index("</joint>", j7_start)
        j7_block = text[j7_start:j7_end]
        self.assertIn('<command_interface name="position"/>', j7_block)
        self.assertNotIn('<command_interface name="effort"/>', j7_block)

    def test_controllers_split_six_effort_and_one_position(self):
        config = _load_controllers_yaml()
        arm = config["panda_arm_controller"]["ros__parameters"]
        wrist = config["panda_wrist_roll_controller"]["ros__parameters"]
        self.assertEqual(
            arm["joints"], [f"panda_joint{i}" for i in range(1, 7)]
        )
        self.assertEqual(arm["command_interfaces"], ["effort"])
        self.assertEqual(wrist["joints"], ["panda_joint7"])
        self.assertEqual(wrist["command_interfaces"], ["position"])

    def test_arm_controller_commands_effort(self):
        config = _load_controllers_yaml()
        arm = config["panda_arm_controller"]["ros__parameters"]
        self.assertEqual(arm["command_interfaces"], ["effort"])
        self.assertEqual(list(arm["joints"]), list(ARM_JOINTS[:6]))

    def test_arm_controller_declares_pid_gains_for_every_effort_joint(self):
        config = _load_controllers_yaml()
        arm = config["panda_arm_controller"]["ros__parameters"]
        gains = arm["gains"]
        for joint in ARM_JOINTS[:6]:
            self.assertIn(joint, gains)
            self.assertIn(joint, gains)
            proportional = float(gains[joint]["p"])
            self.assertGreater(proportional, 0.0)
            self.assertTrue(math.isfinite(proportional))

    def test_arm_controller_keeps_velocity_feedforward_enabled(self):
        config = _load_controllers_yaml()
        arm = config["panda_arm_controller"]["ros__parameters"]
        for joint in ARM_JOINTS[:6]:
            self.assertGreaterEqual(
                float(arm["gains"][joint]["ff_velocity_scale"]), 0.0
            )

    def test_finger_controllers_keep_position_interfaces(self):
        config = _load_controllers_yaml()
        for name in ("panda_gripper_controller", "panda_gripper_right_controller"):
            parameters = config[name]["ros__parameters"]
            self.assertIn("position", parameters["state_interfaces"])

    def test_gz_plugin_keeps_documented_position_gain(self):
        text = _load_xacro_text()
        self.assertIn("<position_proportional_gain>1.0", text)
