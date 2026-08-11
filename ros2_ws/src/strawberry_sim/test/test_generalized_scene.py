from copy import deepcopy
from pathlib import Path
import sys
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_sim.generalized_scene import (  # noqa: E402
    FRUIT_COUNT_RANGE,
    MIN_FRUIT_SEPARATION_M,
    generate_scene,
    materialize_world,
    validate_generated_scene,
)


class GeneralizedSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base_scene = yaml.safe_load((PACKAGE / "config" / "scene.yaml").read_text(encoding="utf-8"))
        cls.base_world = PACKAGE / "worlds" / "strawberry_orchard.sdf"

    def test_same_seed_is_exactly_reproducible(self):
        first = generate_scene(self.base_scene, seed=17036, profile="mixed")
        second = generate_scene(self.base_scene, seed=17036, profile="mixed")
        self.assertEqual(first, second)
        self.assertFalse(first["generator"]["truth_for_runtime_control"])

    def test_counts_names_maturity_and_geometry_are_valid(self):
        scene = generate_scene(self.base_scene, seed=31, profile="mixed", plant_count=3)
        validate_generated_scene(scene)
        self.assertEqual(len(scene["plants"]), 3)
        counts = {plant["plant_id"]: 0 for plant in scene["plants"]}
        for fruit in scene["fruits"]:
            counts[fruit["plant_id"]] += 1
        self.assertTrue(all(FRUIT_COUNT_RANGE[0] <= count <= FRUIT_COUNT_RANGE[1] for count in counts.values()))
        self.assertGreaterEqual(sum(row["maturity"] == "RIPE" for row in scene["fruits"]), 2)
        self.assertTrue(any(row["maturity"] == "UNRIPE" for row in scene["fruits"]))

    def test_negative_and_unsafe_profiles_are_fail_closed(self):
        negative = generate_scene(self.base_scene, seed=8, profile="all_unripe")
        self.assertTrue(all(row["maturity"] == "UNRIPE" for row in negative["fruits"]))
        unsafe = generate_scene(self.base_scene, seed=9, profile="unsafe")
        self.assertTrue(unsafe["evaluation"]["unreachable_ripe_target_ids"])

    def test_position_bands_shift_the_same_seed_reproducibly(self):
        near = generate_scene(self.base_scene, seed=20, profile="mixed", position_band="near")
        far = generate_scene(self.base_scene, seed=20, profile="mixed", position_band="far")
        self.assertEqual(near["generator"]["position_band"], "near")
        self.assertEqual(far["generator"]["position_band"], "far")
        self.assertLess(near["plants"][0]["pose_in_robot_base"][0], far["plants"][0]["pose_in_robot_base"][0])

    def test_validator_rejects_overlapping_fruit(self):
        scene = generate_scene(self.base_scene, seed=12, profile="mixed")
        broken = deepcopy(scene)
        broken["fruits"][1]["initial_pose_m"] = list(broken["fruits"][0]["initial_pose_m"])
        with self.assertRaisesRegex(ValueError, "overlap"):
            validate_generated_scene(broken)

    def test_world_replaces_fixed_models_and_uses_visual_only_occluder(self):
        scene = generate_scene(self.base_scene, seed=44, profile="mixed", plant_count=2, occlusion="heavy")
        tree = materialize_world(self.base_world, scene)
        world = tree.getroot().find("world")
        names = [(item.findtext("name") or "") for item in world.findall("include")]
        self.assertNotIn("strawberry_plant", names)
        self.assertIn("strawberry_plant_1", names)
        self.assertEqual(sum(name.startswith("strawberry_plant_") for name in names), 2)
        occluder = world.find("./model[@name='generalization_occluder_1']")
        self.assertIsNotNone(occluder.find("./link/visual"))
        self.assertIsNone(occluder.find("./link/collision"))


if __name__ == "__main__":
    unittest.main()
