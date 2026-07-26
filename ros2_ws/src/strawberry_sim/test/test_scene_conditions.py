import copy
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from strawberry_sim.scene_conditions import (
    LIGHTING_LEVELS,
    OCCLUSION_LEVELS,
    load_scene_condition_config,
    materialize_condition_world,
    summarize_condition_matrix,
    validate_benchmark_condition_mapping,
)


ROOT = Path(__file__).resolve().parents[4]
CONFIG = ROOT / "config" / "t70_scene_conditions.json"
CONFIG_V2 = ROOT / "config" / "t70_scene_conditions_v2.json"
CONFIG_V3 = ROOT / "config" / "t70_synthetic_capture_scene_conditions.json"
BASE_WORLD = ROOT / "ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf"


class SceneConditionContractTests(unittest.TestCase):
    def test_frozen_config_is_non_acceptance_and_matches_benchmark_levels(self):
        config = load_scene_condition_config(CONFIG)
        self.assertFalse(config["formal_acceptance"])
        self.assertFalse(config["held_out_test_consumed"])
        self.assertEqual(set(LIGHTING_LEVELS), set(config["lighting"]))
        self.assertEqual(set(OCCLUSION_LEVELS), set(config["occlusion"]))

    def test_materializer_changes_light_and_adds_visual_only_occluder(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            world = root / "condition.sdf"
            receipt = root / "receipt.json"
            result = materialize_condition_world(
                base_world=BASE_WORLD,
                config_path=CONFIG,
                lighting_level="dim",
                occlusion_level="partial",
                seed=20260710,
                output_world=world,
                output_receipt=receipt,
            )
            parsed = ET.parse(world).getroot()
            model = parsed.find("./world/model[@name='benchmark_visual_occluder']")
            self.assertIsNotNone(model)
            self.assertEqual([], model.findall(".//collision"))
            self.assertEqual("0.08 0.08 0.08 1", parsed.findtext("./world/scene/ambient"))
            self.assertEqual(
                result["materialized_world"]["sha256"],
                json.loads(receipt.read_text())["materialized_world"]["sha256"],
            )

    def test_rejects_non_visual_occluder(self):
        raw = json.loads(CONFIG.read_text(encoding="utf-8"))
        raw["occlusion"]["partial"]["visual_only"] = False
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "visual-only"):
                load_scene_condition_config(path)

    def test_materializer_rejects_a_seed_outside_the_frozen_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(ValueError, "frozen validation seed"):
                materialize_condition_world(
                    base_world=BASE_WORLD,
                    config_path=CONFIG,
                    lighting_level="nominal",
                    occlusion_level="none",
                    seed=1,
                    output_world=root / "condition.sdf",
                    output_receipt=root / "receipt.json",
                )

    def test_v2_occluder_tracks_target_and_receipt_scene(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            receipt = materialize_condition_world(
                base_world=BASE_WORLD,
                config_path=CONFIG_V2,
                lighting_level="nominal",
                occlusion_level="heavy",
                seed=20260710,
                target_position_m=(0.44, -0.10, 0.48),
                output_world=root / "condition.sdf",
                output_receipt=root / "receipt.json",
            )
            pose = receipt["occlusion_parameters"]["pose_xyz_rpy"]
            self.assertAlmostEqual(0.325, pose[0])
            self.assertAlmostEqual(-0.400, pose[1])
            self.assertAlmostEqual(0.651, pose[2])
            self.assertEqual(
                [0.44, -0.10, 0.48],
                receipt["validation_scene"]["target_position_m"],
            )

    def test_v1_rejects_target_override_and_v2_bounds_tracking(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(ValueError, "does not allow"):
                materialize_condition_world(
                    base_world=BASE_WORLD,
                    config_path=CONFIG,
                    lighting_level="nominal",
                    occlusion_level="none",
                    seed=20260710,
                    target_position_m=(0.44, -0.10, 0.48),
                    output_world=root / "v1.sdf",
                    output_receipt=root / "v1.json",
                )
            with self.assertRaisesRegex(ValueError, "tracking range"):
                materialize_condition_world(
                    base_world=BASE_WORLD,
                    config_path=CONFIG_V2,
                    lighting_level="nominal",
                    occlusion_level="heavy",
                    seed=20260710,
                    target_position_m=(0.10, 0.10, 0.10),
                    output_world=root / "v2.sdf",
                    output_receipt=root / "v2.json",
                )

    def test_v3_uses_nonformal_split_materialization_identifiers(self):
        config = load_scene_condition_config(CONFIG_V3)
        self.assertEqual([20260601, 20260602], config["allowed_materialization_ids"])
        self.assertTrue(
            {20260710, 20260711, 20260712}.isdisjoint(
                config["allowed_materialization_ids"]
            )
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            receipt = materialize_condition_world(
                base_world=BASE_WORLD,
                config_path=CONFIG_V3,
                lighting_level="bright",
                occlusion_level="partial",
                seed=20260602,
                target_position_m=(0.435, -0.085, 0.515),
                output_world=root / "condition.sdf",
                output_receipt=root / "receipt.json",
            )
            self.assertEqual(
                "synthetic_split_identifier_only",
                receipt["materialization_id_role"],
            )

    def test_v3_rejects_a_formal_seed_label(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(ValueError, "materialization contract"):
                materialize_condition_world(
                    base_world=BASE_WORLD,
                    config_path=CONFIG_V3,
                    lighting_level="nominal",
                    occlusion_level="none",
                    seed=20260710,
                    output_world=root / "condition.sdf",
                    output_receipt=root / "receipt.json",
                )


class SceneConditionSummaryTests(unittest.TestCase):
    def _probes(self):
        probes = []
        luminance = {"dim": 40.0, "nominal": 100.0, "bright": 160.0}
        coverage = {"none": 0.0, "partial": 0.20, "heavy": 0.65}
        for light in LIGHTING_LEVELS:
            for occ in OCCLUSION_LEVELS:
                probes.append(
                    {
                        "lighting_level": light,
                        "occlusion_level": occ,
                        "materialized_world_sha256": f"{light}-{occ}",
                        "frames": [
                            {
                                "luminance_mean_8bit": luminance[light],
                                "blue_coverage_ratio": coverage[occ],
                            }
                            for _ in range(5)
                        ],
                    }
                )
        return probes

    def test_complete_ordered_matrix_passes(self):
        config = load_scene_condition_config(CONFIG)
        result = summarize_condition_matrix(self._probes(), config)
        self.assertTrue(result["passed"])
        self.assertTrue(result["unique_materialized_worlds"])

    def test_lighting_or_occlusion_regression_fails(self):
        config = load_scene_condition_config(CONFIG)
        probes = self._probes()
        broken_light = copy.deepcopy(probes)
        for probe in broken_light:
            if probe["lighting_level"] == "bright":
                for frame in probe["frames"]:
                    frame["luminance_mean_8bit"] = 104.0
        self.assertFalse(summarize_condition_matrix(broken_light, config)["passed"])
        broken_occ = copy.deepcopy(probes)
        for probe in broken_occ:
            if probe["occlusion_level"] == "heavy":
                for frame in probe["frames"]:
                    frame["blue_coverage_ratio"] = 0.22
        self.assertFalse(summarize_condition_matrix(broken_occ, config)["passed"])

    def test_benchmark_labels_map_exactly_to_nine_condition_ids(self):
        config = load_scene_condition_config(CONFIG)
        mapping = validate_benchmark_condition_mapping(
            config,
            lighting_levels=("dim", "nominal", "bright"),
            occlusion_levels=("none", "partial", "heavy"),
        )
        self.assertEqual(9, len(mapping))
        self.assertEqual("dim__none", mapping[0]["condition_id"])
        self.assertEqual("bright__heavy", mapping[-1]["condition_id"])

    def test_benchmark_condition_order_mismatch_fails_closed(self):
        config = load_scene_condition_config(CONFIG)
        with self.assertRaisesRegex(ValueError, "lighting order"):
            validate_benchmark_condition_mapping(
                config,
                lighting_levels=("nominal", "dim", "bright"),
                occlusion_levels=("none", "partial", "heavy"),
            )


if __name__ == "__main__":
    unittest.main()
