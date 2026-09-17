"""Gravity-compensated effort control contract (v24-v26 -> rebuild).

Upstream branch add/gravity_compensation implements the setLoad semantics
for simulation: with <param compensate_gravity>true</param> on the effort
command interface, the plugin adds the measured load torque (gravity +
attached fruit) to every effort command, so the JTC PID only tracks the
trajectory. The v12-v20 kicks came from running effort WITHOUT this
compensation; the v21-v26 position-interface workarounds (zero-velocity
window, stationary head, kick retry) remain as defense in depth.
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
LAUNCH = REPO_ROOT / "ros2_ws/src/strawberry_sim/launch/sim.launch.py"

ARM_JOINTS = tuple(f"panda_joint{i}" for i in range(1, 8))


def _load_controllers_yaml() -> dict:
    return yaml.safe_load(CONTROLLERS_YAML.read_text(encoding="utf-8"))


def _load_xacro_text() -> str:
    return XACRO.read_text(encoding="utf-8")


class GravityCompensatedEffortContractTests(unittest.TestCase):
    def test_urdf_declares_effort_with_compensate_gravity(self):
        text = _load_xacro_text()
        macro_start = text.index('<xacro:macro name="arm_control_joint"')
        macro_end = text.index("</xacro:macro>", macro_start)
        macro = text[macro_start:macro_end]
        self.assertIn('<command_interface name="effort">', macro)
        self.assertIn('<param name="compensate_gravity">true</param>', macro)

    def test_arm_controller_commands_effort_for_all_seven_joints(self):
        config = _load_controllers_yaml()
        arm = config["panda_arm_controller"]["ros__parameters"]
        self.assertEqual(arm["command_interfaces"], ["effort"])
        self.assertEqual(list(arm["joints"]), list(ARM_JOINTS))

    def test_arm_controller_keeps_v14_pid_gains(self):
        config = _load_controllers_yaml()
        gains = config["panda_arm_controller"]["ros__parameters"]["gains"]
        expected = {
            "panda_joint1": (300.0, 150.0, 10.0),
            "panda_joint2": (300.0, 150.0, 10.0),
            "panda_joint3": (300.0, 150.0, 10.0),
            "panda_joint4": (300.0, 150.0, 10.0),
            "panda_joint5": (60.0, 40.0, 3.0),
            "panda_joint6": (60.0, 40.0, 3.0),
            "panda_joint7": (60.0, 40.0, 3.0),
        }
        for joint, (p, i, d) in expected.items():
            self.assertEqual(gains[joint]["p"], p)
            self.assertEqual(gains[joint]["i"], i)
            self.assertEqual(gains[joint]["d"], d)
            self.assertEqual(gains[joint]["ff_velocity_scale"], 1.0)

    def test_launch_shadows_plugin_with_gc_build_when_configured(self):
        text = (LAUNCH).read_text(encoding="utf-8")
        self.assertIn("STRAWBERRY_GC_PLUGIN_LIB_DIR", text)
        self.assertIn("GZ_SIM_SYSTEM_PLUGIN_PATH", text)

    def test_gc_plugin_build_script_exists(self):
        script = (
            REPO_ROOT / "scripts" / "build_gc_plugin.sh"
        )
        self.assertTrue(script.exists())
        self.assertIn("add/gravity_compensation", script.read_text(
            encoding="utf-8"
        ))