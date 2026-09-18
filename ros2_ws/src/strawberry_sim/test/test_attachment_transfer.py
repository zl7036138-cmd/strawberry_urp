from pathlib import Path
import unittest
from strawberry_sim.attachment_transfer import transfer_stem_to_gripper, state_confirmed
from strawberry_sim.attachment_world import add_external_stem_support

ROOT = Path(__file__).resolve().parents[1]


class TransferTests(unittest.TestCase):
    def run_transfer(self, stem=True, gripper=True, cancelled=True):
        self.events = []
        def call(name, value=None):
            def callback():
                self.events.append(name)
                return value
            return callback
        return transfer_stem_to_gripper(
            release_stem=call("release_stem"), stem_released=call("stem_confirm", stem),
            attach_gripper=call("attach_gripper"), gripper_attached=call("gripper_confirm", gripper),
            detach_gripper=call("detach_gripper"), gripper_detached=call("detach_confirm", cancelled))

    def test_success_has_no_simultaneous_parent_constraint(self):
        success, _ = self.run_transfer()
        self.assertTrue(success)
        self.assertEqual(self.events, ["release_stem", "stem_confirm", "attach_gripper", "gripper_confirm"])

    def test_unconfirmed_stem_never_requests_gripper_attachment(self):
        success, message = self.run_transfer(stem=False)
        self.assertFalse(success)
        self.assertEqual(self.events, ["release_stem", "stem_confirm"])
        self.assertIn("no gripper attachment requested", message)

    def test_failed_gripper_cancels_even_a_pending_request(self):
        success, message = self.run_transfer(gripper=False)
        self.assertFalse(success)
        self.assertEqual(self.events[-2:], ["detach_gripper", "detach_confirm"])
        self.assertIn("detach confirmed", message)

    def test_unconfirmed_cancellation_does_not_claim_success(self):
        success, message = self.run_transfer(gripper=False, cancelled=False)
        self.assertFalse(success)
        self.assertIn("detach unconfirmed", message)

    def test_opt_in_preserves_historical_backend(self):
        launch = (ROOT / "launch/sim.launch.py").read_text()
        self.assertIn('"gripper_attachment_backend", default_value="upstream"', launch)
        self.assertIn('"stem_release_before_gripper_attach": attachment_backend == "lazy"', launch)
        manager = (ROOT / "strawberry_sim/attachment_manager.py").read_text()
        self.assertIn('self.declare_parameter("stem_release_before_gripper_attach", False)', manager)

    def test_stem_remains_upstream_and_only_gripper_is_configurable(self):
        import xml.etree.ElementTree as ET
        root = ET.fromstring((ROOT / "urdf/dynamic_attachments.xacro").read_text())
        macros = root.findall("{http://www.ros.org/wiki/xacro}macro")
        self.assertEqual(macros[0].find("plugin").get("filename"), "$(arg gripper_attachment_plugin_file)")
        self.assertEqual(macros[1].find("plugin").get("filename"), "gz-sim-detachable-joint-system")

    def test_first_three_gripper_plugins_also_use_the_backend_override(self):
        import xml.etree.ElementTree as ET
        root = ET.fromstring((ROOT / "urdf/panda_gz.urdf.xacro").read_text())
        grippers = [plugin for plugin in root.findall(".//plugin")
                    if plugin.findtext("parent_link") == "panda_link7"]
        self.assertEqual(len(grippers), 3)
        for plugin in grippers:
            self.assertEqual(plugin.get("filename"), "$(arg gripper_attachment_plugin_file)")
            self.assertEqual(plugin.get("name"), "$(arg gripper_attachment_plugin_name)")

    def test_confirmation_requires_a_post_request_matching_state(self):
        for generation, value, expected in ((4, False, False), (5, True, False), (5, False, True)):
            self.assertEqual(state_confirmed(known=True, value=value, generation=generation,
                             desired=False, after_generation=4), expected)

    def test_unknown_state_never_confirms_and_legacy_call_still_works(self):
        self.assertFalse(state_confirmed(known=False, value=False, generation=5, desired=False))
        self.assertTrue(state_confirmed(known=True, value=False, generation=0, desired=False))

    def test_external_stems_are_separate_world_fixed_geometry_free_support(self):
        import xml.etree.ElementTree as ET
        root = ET.fromstring('<sdf><world name="test"><model name="panda"/></world></sdf>')
        add_external_stem_support(root, 5)
        support = root.find("world/model[@name='strawberry_stem_support']")
        self.assertEqual(support.findtext("joint/parent"), "world")
        self.assertEqual(support.findtext("joint/child"), "stem_support_link")
        self.assertEqual(len(support.findall("plugin")), 5)
        self.assertFalse(support.findall(".//collision"))
        self.assertFalse(root.findall(".//self_collide"))
        for plugin in support.findall("plugin"):
            self.assertEqual(plugin.findtext("parent_link"), "stem_support_link")
        self.assertFalse(root.find("world/model[@name='panda']").findall("plugin"))

    def test_external_stems_reject_duplicates_and_invalid_capacity(self):
        import xml.etree.ElementTree as ET
        for count in (0, 10, True):
            with self.assertRaises(ValueError): add_external_stem_support(ET.fromstring('<sdf><world/></sdf>'), count)
        root = ET.fromstring('<sdf><world/></sdf>')
        add_external_stem_support(root, 1)
        with self.assertRaises(ValueError): add_external_stem_support(root, 1)


if __name__ == "__main__": unittest.main()
