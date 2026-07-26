import json
import math
from pathlib import Path
import tempfile
import unittest

from strawberry_benchmark.diagnostics import (
    load_oracle_shadow_diagnostic_manifest,
    verify_shadow_model,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
MANIFEST_PATH = REPOSITORY_ROOT / "config" / "t60_oracle_shadow_position_diagnostic.json"
SINGLE_TARGET_MANIFEST_PATH = (
    REPOSITORY_ROOT
    / "config"
    / "t60_oracle_shadow_single_target_position_diagnostic.json"
)
CLEARANCE_MANIFEST_PATH = (
    REPOSITORY_ROOT
    / "config"
    / "t60_oracle_shadow_multifruit_clearance_diagnostic.json"
)


class OracleShadowDiagnosticContractTests(unittest.TestCase):
    def test_frozen_manifest_is_non_acceptance_and_source_isolated(self):
        manifest = load_oracle_shadow_diagnostic_manifest(MANIFEST_PATH)
        self.assertFalse(manifest.formal_acceptance)
        self.assertFalse(manifest.held_out_test_consumed)
        self.assertFalse(manifest.formal_position_labels)
        self.assertEqual("baseline__best", manifest.shadow_model_role)
        self.assertEqual("/strawberry/oracle/target_pose", manifest.control_topic)
        self.assertNotEqual(manifest.control_topic, manifest.shadow_target_topic)
        self.assertEqual(5, len(manifest.scenarios))

    def test_rejects_perception_control(self):
        raw = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        raw["control"]["source"] = "perception"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "must use oracle"):
                load_oracle_shadow_diagnostic_manifest(path)

    def test_rejects_formal_position_claim(self):
        raw = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        raw["conditions"]["formal_position_labels"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "cannot masquerade"):
                load_oracle_shadow_diagnostic_manifest(path)

    def test_model_verification_fails_closed_on_wrong_bytes(self):
        manifest = load_oracle_shadow_diagnostic_manifest(MANIFEST_PATH)
        with tempfile.TemporaryDirectory() as directory:
            wrong_model = Path(directory) / "best.pt"
            wrong_model.write_bytes(b"not the frozen model")
            with self.assertRaisesRegex(ValueError, "differs"):
                verify_shadow_model(manifest, REPOSITORY_ROOT, wrong_model)

    def test_single_target_manifest_parks_both_non_targets(self):
        manifest = load_oracle_shadow_diagnostic_manifest(
            SINGLE_TARGET_MANIFEST_PATH
        )
        self.assertEqual("single_target_isolation", manifest.scene_mode)
        self.assertEqual(
            {"strawberry_2", "strawberry_3"},
            {model.model_name for model in manifest.parked_models},
        )
        self.assertEqual({2, 3}, {model.target_id for model in manifest.parked_models})
        self.assertFalse(manifest.formal_position_labels)

    def test_single_target_manifest_rejects_missing_parked_model(self):
        raw = json.loads(SINGLE_TARGET_MANIFEST_PATH.read_text(encoding="utf-8"))
        raw["scene_setup"]["parked_models"].pop()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "exactly two"):
                load_oracle_shadow_diagnostic_manifest(path)

    def test_clearance_manifest_freezes_non_overlapping_decreasing_sweep(self):
        manifest = load_oracle_shadow_diagnostic_manifest(CLEARANCE_MANIFEST_PATH)
        self.assertEqual("multi_fruit_clearance", manifest.scene_mode)
        diameter = 2.0 * manifest.fruit_collision_radius_m
        clearances = [
            min(
                math.dist(scenario.target_position_m, model.position_m)
                for model in manifest.parked_models
            )
            - diameter
            for scenario in manifest.scenarios
        ]
        self.assertTrue(all(value > 0.0 for value in clearances))
        self.assertTrue(
            all(first > second for first, second in zip(clearances, clearances[1:]))
        )
        self.assertAlmostEqual(0.03, clearances[-1], places=9)

    def test_clearance_manifest_rejects_overlapping_neighbor(self):
        raw = json.loads(CLEARANCE_MANIFEST_PATH.read_text(encoding="utf-8"))
        raw["scene_setup"]["parked_models"][0]["position_m"] = [
            0.44,
            0.01,
            0.52,
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "must not overlap"):
                load_oracle_shadow_diagnostic_manifest(path)


if __name__ == "__main__":
    unittest.main()
