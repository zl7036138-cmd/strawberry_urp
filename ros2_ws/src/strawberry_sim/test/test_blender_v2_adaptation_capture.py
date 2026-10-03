from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "scripts"))

from validate_blender_v2_adaptation_capture import (  # noqa: E402
    iter_groups,
    load_capture_manifest,
)


MANIFEST = ROOT / "config" / "blender_v2_adaptation_capture_v1.json"


class BlenderV2AdaptationCaptureTests(unittest.TestCase):
    def test_manifest_freezes_nonformal_disjoint_96_image_capture(self):
        manifest = load_capture_manifest(MANIFEST)
        groups = list(iter_groups(manifest))
        self.assertEqual(len(groups), 12)
        self.assertEqual(
            manifest["capture"]["expected_train_images"],
            72,
        )
        self.assertEqual(
            manifest["capture"]["expected_heldout_images"],
            24,
        )
        self.assertEqual(
            manifest["capture"]["fruit_radius_m"],
            0.026,
        )
        self.assertFalse(manifest["training_started"])
        self.assertFalse(manifest["held_out_test_consumed"])

    def test_every_capture_pose_has_an_explicit_orientation(self):
        manifest = load_capture_manifest(MANIFEST)
        for split in ("train", "heldout"):
            for row in manifest["splits"][split]["positions"]:
                self.assertEqual(len(row["orientation_xyzw"]), 4)

    def test_runner_keeps_motion_and_perception_disabled(self):
        source = (
            ROOT / "scripts" / "run_blender_v2_adaptation_capture.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("start_perception:=false", source)
        self.assertIn("start_manipulation:=false", source)
        self.assertIn("start_orchestrator:=false", source)
        self.assertIn("enable_attachment:=false", source)
        self.assertIn('scene_config_file:="${scene_manifest}"', source)


if __name__ == "__main__":
    unittest.main()
