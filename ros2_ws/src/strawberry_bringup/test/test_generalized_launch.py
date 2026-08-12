from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class GeneralizedLaunchContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / "launch" / "generalized_harvest.launch.py").read_text(encoding="utf-8")

    def test_dual_camera_topics_are_distinct(self):
        self.assertIn('"camera_mount": "dual"', self.source)
        self.assertIn('"/camera/base/color/image_raw"', self.source)
        self.assertIn('"/camera/wrist/color/image_raw"', self.source)
        self.assertIn('"/strawberry/tracked_targets"', self.source)
        self.assertIn('"/strawberry/wrist/target_pose"', self.source)
        self.assertIn(
            '"target_hint_topic": "/strawberry/wrist/target_hint"', self.source
        )
        self.assertIn('"require_target_hint": True', self.source)
        self.assertIn(
            '"geometry_size_residual_sigma_weight": 0.5', self.source
        )
        self.assertIn('"completed_track_topic": ""', self.source)

    def test_generalized_control_disables_truth_association(self):
        self.assertGreaterEqual(self.source.count('"ground_truth_association_enabled": False'), 2)
        self.assertIn('"fruit_pose_source": "tracked"', self.source)

    def test_every_renamed_camera_node_uses_simulation_time(self):
        self.assertGreaterEqual(self.source.count('"use_sim_time": True'), 4)

    def test_continuous_orchestrator_and_dynamic_selector_are_started(self):
        self.assertEqual(self.source.count('executable="generalized_localization_node"'), 2)
        self.assertIn('executable="target_selector"', self.source)
        self.assertIn('executable="harvest_orchestrator"', self.source)
        self.assertIn('"scene_config_file": scene_config', self.source)
        self.assertNotIn("oracle_target_provider", self.source)

    def test_qualified_detector_settings_are_shared_end_to_end(self):
        self.assertIn("device_parameter = ParameterValue(device, value_type=str)", self.source)
        self.assertEqual(self.source.count('"device": device_parameter'), 2)
        self.assertIn('DeclareLaunchArgument("image_size", default_value="800")', self.source)
        self.assertIn(
            'DeclareLaunchArgument("nms_iou_threshold", default_value="0.50")',
            self.source,
        )
        self.assertIn('default_value="0.20524564385414124"', self.source)
        self.assertEqual(self.source.count('"image_size": image_size'), 2)
        self.assertEqual(
            self.source.count('"nms_iou_threshold": nms_iou_threshold'), 2
        )
        self.assertGreaterEqual(
            self.source.count('"confidence_threshold": confidence_threshold'), 5
        )
        self.assertIn('"wrist_min_confidence": confidence_threshold', self.source)


if __name__ == "__main__":
    unittest.main()
