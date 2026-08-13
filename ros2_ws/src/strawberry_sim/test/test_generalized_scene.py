from copy import deepcopy
from pathlib import Path
import sys
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_sim.generalized_scene import (  # noqa: E402
    FRUIT_COUNT_RANGE,
    MIN_INITIAL_STATIC_COLLISION_GAP_M,
    MIN_STATIC_OBSTACLE_CENTER_CLEARANCE_M,
    _axis_aligned_box_clearance,
    generate_scene,
    materialize_world,
    validate_generated_scene,
)


class GeneralizedSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base_scene = yaml.safe_load(
            (PACKAGE / "config" / "scene.yaml").read_text(encoding="utf-8")
        )
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
            self.assertTrue(fruit["asset"].endswith("_generalized"))
        self.assertEqual(scene["generator"]["released_fruit_physics"], "gravity")
        self.assertTrue(
            all(
                FRUIT_COUNT_RANGE[0] <= count <= FRUIT_COUNT_RANGE[1]
                for count in counts.values()
            )
        )
        self.assertGreaterEqual(
            sum(row["maturity"] == "RIPE" for row in scene["fruits"]), 2
        )
        self.assertTrue(any(row["maturity"] == "UNRIPE" for row in scene["fruits"]))

    def test_negative_and_unsafe_profiles_are_fail_closed(self):
        negative = generate_scene(self.base_scene, seed=8, profile="all_unripe")
        self.assertTrue(all(row["maturity"] == "UNRIPE" for row in negative["fruits"]))
        unsafe = generate_scene(self.base_scene, seed=9, profile="unsafe")
        self.assertTrue(unsafe["evaluation"]["unreachable_ripe_target_ids"])

    def test_generated_fruit_clear_bin_and_use_runtime_reach_margin(self):
        scene = generate_scene(self.base_scene, seed=17036, profile="mixed")
        bin_bounds = scene["generator"]["bin_exclusion_bounds_m"]
        self.assertTrue(
            any(row["reachable_by_construction"] for row in scene["fruits"])
        )
        for row in scene["fruits"]:
            clearance = _axis_aligned_box_clearance(row["initial_pose_m"], bin_bounds)
            required = (
                float(scene["fruit_collision_radius_m"]) * float(row["scale"])
                + MIN_INITIAL_STATIC_COLLISION_GAP_M
            )
            self.assertGreaterEqual(clearance, required)
        self.assertEqual(
            scene["generator"]["minimum_static_obstacle_center_clearance_m"],
            MIN_STATIC_OBSTACLE_CENTER_CLEARANCE_M,
        )

    def test_validator_rejects_initial_bin_collision(self):
        scene = generate_scene(self.base_scene, seed=17036, profile="mixed")
        broken = deepcopy(scene)
        broken["fruits"][0]["initial_pose_m"] = [0.35, -0.23, 0.54]
        broken["fruits"][0]["reachable_by_construction"] = False
        with self.assertRaisesRegex(ValueError, "collection bin"):
            validate_generated_scene(broken)

    def test_validator_rejects_a_forged_reachability_label(self):
        scene = generate_scene(self.base_scene, seed=17036, profile="mixed")
        broken = deepcopy(scene)
        broken["fruits"][0]["reachable_by_construction"] = not bool(
            broken["fruits"][0]["reachable_by_construction"]
        )
        with self.assertRaisesRegex(ValueError, "reachability label"):
            validate_generated_scene(broken)

    def test_position_bands_shift_the_same_seed_reproducibly(self):
        near = generate_scene(
            self.base_scene, seed=20, profile="mixed", position_band="near"
        )
        far = generate_scene(
            self.base_scene, seed=20, profile="mixed", position_band="far"
        )
        self.assertEqual(near["generator"]["position_band"], "near")
        self.assertEqual(far["generator"]["position_band"], "far")
        self.assertLess(
            near["plants"][0]["pose_in_robot_base"][0],
            far["plants"][0]["pose_in_robot_base"][0],
        )

    def test_validator_rejects_overlapping_fruit(self):
        scene = generate_scene(self.base_scene, seed=12, profile="mixed")
        broken = deepcopy(scene)
        broken["fruits"][1]["initial_pose_m"] = list(
            broken["fruits"][0]["initial_pose_m"]
        )
        with self.assertRaisesRegex(ValueError, "overlap"):
            validate_generated_scene(broken)

    def test_world_replaces_fixed_models_and_uses_visual_only_occluder(self):
        scene = generate_scene(
            self.base_scene, seed=44, profile="mixed", plant_count=2, occlusion="heavy"
        )
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
