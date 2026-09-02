from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class GeneralizedLaunchContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / "launch" / "generalized_harvest.launch.py").read_text(
            encoding="utf-8"
        )

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
        self.assertEqual(
            self.source.count('"geometry_size_residual_sigma_weight": 0.5'), 1
        )
        self.assertEqual(
            self.source.count('"geometry_target_radius_m": 0.0235'), 1
        )
        localization_config = (
            ROOT.parent
            / "strawberry_localization"
            / "config"
            / "localization_generalized.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn("geometry_size_residual_sigma_weight: 1.0", localization_config)
        self.assertIn(
            "geometry_max_selected_centroid_distance_fraction: 0.15",
            localization_config,
        )
        self.assertIn("center_layer_fallback_enabled: true", localization_config)
        self.assertEqual(
            self.source.count('"center_layer_fallback_enabled": False'), 1
        )
        self.assertEqual(
            self.source.count(
                '"geometry_max_selected_centroid_distance_fraction": 1.0'
            ),
            1,
        )
        self.assertIn('"completed_track_topic": ""', self.source)
        self.assertEqual(
            self.source.count('"tracking_reference_gate_enabled": False'), 1
        )
        self.assertIn("tracking_reference_gate_enabled: true", localization_config)
        self.assertIn(
            "tracking_reference_joint_positions_rad: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]",
            localization_config,
        )

    def test_generalized_control_disables_truth_association(self):
        self.assertGreaterEqual(
            self.source.count('"ground_truth_association_enabled": False'), 2
        )
        self.assertIn('"fruit_pose_source": "tracked"', self.source)
        manipulation = (
            ROOT.parent
            / "strawberry_manipulation"
            / "strawberry_manipulation"
            / "action_server.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'contact_resolved_attachment=(fruit_pose_source == "tracked")',
            manipulation,
        )
        self.assertNotIn("_resolve_sim_entity", manipulation)

    def test_every_renamed_camera_node_uses_simulation_time(self):
        self.assertGreaterEqual(self.source.count('"use_sim_time": True'), 4)

    def test_continuous_orchestrator_and_dynamic_selector_are_started(self):
        self.assertEqual(
            self.source.count('executable="generalized_localization_node"'), 2
        )
        self.assertIn('executable="target_selector"', self.source)
        self.assertIn('executable="harvest_orchestrator"', self.source)
        orchestrator = (
            ROOT / "strawberry_bringup" / "harvest_orchestrator.py"
        ).read_text(encoding="utf-8")
        manipulation = (
            ROOT.parent
            / "strawberry_manipulation"
            / "strawberry_manipulation"
            / "action_server.py"
        ).read_text(encoding="utf-8")
        self.assertIn('Trigger, "/strawberry/move_home"', orchestrator)
        self.assertIn('"/strawberry/move_home"', manipulation)
        self.assertIn(
            '"home_joint_trajectory_velocity_rad_per_sec", 0.08',
            manipulation,
        )
        self.assertIn('"scene_config_file": scene_config', self.source)
        self.assertNotIn("oracle_target_provider", self.source)
        simulation = (
            ROOT.parent / "strawberry_sim" / "launch" / "sim.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"verification_timeout_wall_sec": (', simulation)
        self.assertIn("12.0 if generalized_scene else 3.0", simulation)

    def test_no_motion_screen_can_disable_control_without_disabling_moveit(self):
        self.assertIn(
            'DeclareLaunchArgument(\n                "harvest_control_enabled", default_value="true"',
            self.source,
        )
        self.assertEqual(
            self.source.count("condition=IfCondition(harvest_control_enabled)"),
            2,
        )
        manipulation_index = self.source.index(
            'executable="pick_and_place_server"'
        )
        selector_index = self.source.index('executable="target_selector"')
        orchestrator_index = self.source.index('executable="harvest_orchestrator"')
        self.assertNotIn(
            "condition=IfCondition(harvest_control_enabled)",
            self.source[manipulation_index:orchestrator_index],
        )
        self.assertLess(selector_index, manipulation_index)

    def test_qualified_detector_settings_are_shared_end_to_end(self):
        self.assertIn(
            "device_parameter = ParameterValue(device, value_type=str)", self.source
        )
        self.assertEqual(self.source.count('"device": device_parameter'), 2)
        self.assertIn(
            'DeclareLaunchArgument("image_size", default_value="800")', self.source
        )
        self.assertIn(
            'DeclareLaunchArgument("nms_iou_threshold", default_value="0.50")',
            self.source,
        )
        self.assertIn('default_value="0.20524564385414124"', self.source)
        self.assertEqual(self.source.count('"image_size": image_size'), 2)
        self.assertEqual(self.source.count('"nms_iou_threshold": nms_iou_threshold'), 2)
        self.assertGreaterEqual(
            self.source.count('"confidence_threshold": confidence_threshold'), 5
        )
        self.assertIn('"wrist_min_confidence": confidence_threshold', self.source)
        self.assertIn('"wrist_confirmation_timeout_sec": 3.0', self.source)
        self.assertIn('"minimum_reobservation_baseline_m": 0.04', self.source)

    def test_base_camera_layout_is_global_and_forwarded_to_simulation(self):
        for name, default in (
            ("base_camera_mast_xyz", "-0.35 0.45 0.05"),
            ("base_camera_xyz", "0 0 1.00"),
            ("base_camera_rpy", "0 0.543 -0.480"),
            ("base_camera_resolution", "320x240"),
        ):
            self.assertRegex(
                self.source,
                rf'DeclareLaunchArgument\(\s*"{name}",\s*default_value="{re.escape(default)}"',
            )
            self.assertRegex(
                self.source,
                rf'"{name}": LaunchConfiguration\(\s*"{name}"\s*\)',
            )


if __name__ == "__main__":
    unittest.main()
