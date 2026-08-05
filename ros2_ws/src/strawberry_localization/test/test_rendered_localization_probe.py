import importlib.util
import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
SIM_PACKAGE_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "strawberry_sim"
sys.path.insert(0, str(SIM_PACKAGE_ROOT))

from strawberry_sim.scene_conditions import (  # noqa: E402
    load_scene_condition_config,
)


SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "probe_paired_rendered_localization.py"
SPEC = importlib.util.spec_from_file_location("rendered_localization_probe", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class RenderedLocalizationProbeTests(unittest.TestCase):
    def test_summary_preserves_missing_pose_samples(self) -> None:
        records = [
            {
                "geometry_layer": {
                    "status": "ESTIMATED",
                    "error_mm": 4.0,
                    "position_sigma_m": 0.01,
                }
            },
            {"geometry_layer": {"status": "NO_POSE"}},
            {
                "geometry_layer": {
                    "status": "ESTIMATED",
                    "error_mm": 8.0,
                    "position_sigma_m": 0.02,
                }
            },
        ]

        summary = MODULE.summarize_mode(records, "geometry_layer")

        self.assertEqual(summary["requested_samples"], 3)
        self.assertEqual(summary["estimated_samples"], 2)
        self.assertEqual(summary["no_pose_samples"], 1)
        self.assertAlmostEqual(summary["acceptance_rate"], 2.0 / 3.0)
        self.assertEqual(summary["median_error_mm"], 6.0)
        self.assertEqual(summary["p95_error_mm"], 8.0)
        self.assertEqual(summary["sigma_under_15mm_rate"], 0.5)

    def test_scene_contract_uses_v2_radius_and_visual_only_occluders(self) -> None:
        config = load_scene_condition_config(
            REPOSITORY_ROOT
            / "config"
            / "rendered_occlusion_localization_conditions_v1.json"
        )

        self.assertEqual(config["validation_scene"]["fruit_radius_m"], 0.026)
        self.assertTrue(config["occlusion"]["partial"]["visual_only"])
        self.assertTrue(config["occlusion"]["heavy"]["visual_only"])

    def test_probe_is_oracle_box_no_control_localization_isolation(self) -> None:
        source = SCRIPT_PATH.read_text(encoding="utf-8")

        self.assertIn("sphere_projection_bbox", source)
        self.assertIn("padding=options.bbox_padding", source)
        self.assertIn('"oracle_bbox_from_truth": True', source)
        self.assertIn('"perception_model_started": False', source)
        self.assertIn('"control_commands_sent": 0', source)
        self.assertIn("count_subscribers(options.depth_topic) >= 3", source)
        self.assertIn("count_subscribers(options.camera_info_topic) >= 3", source)
        self.assertIn("and transform_is_ready()", source)
        self.assertNotIn("SetEntityPose", source)
        self.assertNotIn("PickAndPlace", source)


if __name__ == "__main__":
    unittest.main()
