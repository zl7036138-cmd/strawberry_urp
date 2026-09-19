import importlib.util
from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET

spec = importlib.util.spec_from_file_location("payload_fixture", Path(__file__).resolve().parents[1] / "payload_fixture.py")
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


class PayloadFixtureTests(unittest.TestCase):
    def robot(self):
        return ET.fromstring('''<robot><link name="base"/><link name="turn"/><link name="panda_link7"/>
          <joint name="q" type="revolute"><parent link="base"/><child link="turn"/>
            <origin xyz="1 0 0"/><axis xyz="0 0 1"/></joint>
          <joint name="fixed" type="fixed"><parent link="turn"/><child link="panda_link7"/>
            <origin xyz="1 0 0"/></joint></robot>''')

    def test_fk_uses_parent_rotation_then_child_offset(self):
        frame = fixture.link_transform(self.robot(), {"q": math.pi/2}, "panda_link7")
        self.assertAlmostEqual(frame[0][3], 1)
        self.assertAlmostEqual(frame[1][3], 1)
        self.assertAlmostEqual(frame[0][1], -1)

    def test_fk_rejects_missing_link_and_zero_axis(self):
        with self.assertRaises(ValueError): fixture.link_transform(self.robot(), {}, "absent")
        robot = self.robot(); robot.find("joint/axis").set("xyz", "0 0 0")
        with self.assertRaises(ValueError): fixture.link_transform(robot, {}, "panda_link7")

    def test_fixture_has_no_contacts_truth_or_robot_changes(self):
        world = ET.Element("world"); robot = self.robot(); original = ET.tostring(robot)
        fixture.add_payload_fixture(world, robot, {"q": 0}, "/built/plugin.so")
        self.assertEqual(ET.tostring(robot), original)
        self.assertFalse(world.findall(".//collision"))
        self.assertFalse(world.findall(".//sensor"))
        self.assertEqual(world.findtext("model[@name='diagnostic_payload']/link/inertial/mass"), "0.030")
        self.assertEqual(world.findtext("plugin/parent_model"), "panda")
        self.assertEqual(world.findtext("model[@name='diagnostic_stem']/joint/parent"), "world")

    def test_runtime_model_parent_path_remains_strict(self):
        source = (Path(__file__).resolve().parents[3] / "ros2_ws/src/strawberry_gazebo_plugins/src/LazyDetachableJoint.cc").read_text()
        self.assertIn("model.Valid(ecm)", source)
        self.assertIn("components::World>(entity)", source)
        self.assertIn("matches == 1", source)
        self.assertIn("parent != child", source)

    def test_component_state_without_motion_does_not_prove_carry(self):
        events = [{"sim_time_sec": i, "detachable_joint_count": v} for i,v in enumerate((1,0,1))]
        rows = [{"phase": "FIXTURE_POSE", "sim_time_sec": 3+i*0.1,
                 "parent_xyz": [0,0,0], "child_xyz": [0,0,0.16], "relative_xyz": [0,0,0.16],
                 "relative_quat_wxyz": [1,0,0,0]} for i in range(4)]
        self.assertEqual(fixture.motion_metrics(rows, events, "welded")["physical_fixture_status"], "INDETERMINATE")
        for i,row in enumerate(rows): row["parent_xyz"][0] = row["child_xyz"][0] = i*0.01
        self.assertEqual(fixture.motion_metrics(rows, events, "welded")["physical_fixture_status"], "CARRY_OBSERVED")
        rows[-1]["relative_xyz"][0] = 0.01
        self.assertEqual(fixture.motion_metrics(rows, events, "welded")["physical_fixture_status"], "INDETERMINATE")

    def test_missing_and_nonfinite_pose_withholds_evidence(self):
        self.assertEqual(fixture.motion_metrics([], [], "welded")["physical_fixture_status"], "INDETERMINATE")
        events = [{"sim_time_sec": i, "detachable_joint_count": v} for i,v in enumerate((1,0,1))]
        rows = [{"phase": "FIXTURE_POSE", "sim_time_sec": 3+i*0.1,
                 "parent_xyz": [i*0.01,0,0], "child_xyz": [i*0.01,0,0.16], "relative_xyz": [0,0,0.16],
                 "relative_quat_wxyz": [1,0,0,0]} for i in range(4)]
        rows[-1]["relative_xyz"][0] = float("nan")
        self.assertEqual(fixture.motion_metrics(rows, events, "welded")["physical_fixture_status"], "INDETERMINATE")
