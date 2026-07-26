import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "scripts"))

from summarize_t70_multiposition_diagnostic import (  # noqa: E402
    classify_shadow_summary,
)
from summarize_t70_multiposition_diagnostic_v2 import (  # noqa: E402
    background_aware_position_check,
)
from validate_t70_multiposition_diagnostic import (  # noqa: E402
    load_diagnostic_manifest,
)


MANIFEST = ROOT / "config" / "t70_shadow_multiposition_fixed_window.json"


class MultipositionManifestTests(unittest.TestCase):
    def test_frozen_manifest_expands_to_fifteen_no_motion_scenarios(self):
        manifest = load_diagnostic_manifest(MANIFEST)
        self.assertEqual(15, manifest["_scenario_count"])
        self.assertFalse(manifest["control"]["robot_motion_started"])
        self.assertEqual(
            "single_diagnostic_materialization_only",
            manifest["condition_contract"]["seed_role"],
        )
        self.assertEqual(
            0,
            manifest["condition_contract"]["independent_random_seed_count_claimed"],
        )

    def test_manifest_rejects_a_formal_or_motion_claim(self):
        raw = json.loads(MANIFEST.read_text(encoding="utf-8"))
        raw["formal_acceptance"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "cannot assert"):
                load_diagnostic_manifest(path)

        raw = json.loads(MANIFEST.read_text(encoding="utf-8"))
        raw["control"]["robot_motion_started"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "control boundary"):
                load_diagnostic_manifest(path)

    def test_manifest_rejects_position_drift(self):
        raw = json.loads(MANIFEST.read_text(encoding="utf-8"))
        changed = copy.deepcopy(raw)
        changed["positions"][0]["target_position_m"][0] += 0.001
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps(changed), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "five frozen"):
                load_diagnostic_manifest(path)


class ShadowClassificationTests(unittest.TestCase):
    def test_classifies_detection_absence(self):
        self.assertEqual(
            "DETECTION_ABSENT",
            classify_shadow_summary(
                {"frames_with_ripe_detection": 0, "frames_with_target_pose": 0}
            ),
        )

    def test_classifies_localization_gap(self):
        self.assertEqual(
            "LOCALIZATION_OR_PIPELINE_GAP",
            classify_shadow_summary(
                {"frames_with_ripe_detection": 60, "frames_with_target_pose": 42}
            ),
        )

    def test_classifies_complete_observation_without_setting_a_rate_gate(self):
        self.assertEqual(
            "OBSERVATION_COMPLETE",
            classify_shadow_summary(
                {"frames_with_ripe_detection": 1, "frames_with_target_pose": 1}
            ),
        )


class BackgroundAwareConditionTests(unittest.TestCase):
    def test_blue_scene_background_is_not_mistaken_for_injected_occlusion(self):
        check = background_aware_position_check(
            0.25,
            0.25,
            0.78,
            background_stability_maximum=0.02,
            heavy_foreground_delta_minimum=0.35,
            heavy_coverage_maximum=0.98,
            no_occluder_worlds_verified=True,
            heavy_occluder_world_verified=True,
        )
        self.assertTrue(check["none_background_is_stable_across_lighting"])
        self.assertTrue(check["heavy_adds_visible_foreground"])
        self.assertAlmostEqual(0.53, check["heavy_additional_blue_coverage"])

    def test_world_structure_still_fails_closed(self):
        check = background_aware_position_check(
            0.0,
            0.0,
            0.75,
            background_stability_maximum=0.02,
            heavy_foreground_delta_minimum=0.35,
            heavy_coverage_maximum=0.98,
            no_occluder_worlds_verified=False,
            heavy_occluder_world_verified=True,
        )
        self.assertFalse(check["none_worlds_have_no_benchmark_occluder"])


if __name__ == "__main__":
    unittest.main()
