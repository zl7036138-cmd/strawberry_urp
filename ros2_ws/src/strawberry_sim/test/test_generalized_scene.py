from copy import deepcopy
import hashlib
import json
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
    MULTI_PICK_V2_ANCHORS,
    MULTI_PICK_V2_PRIMARY_YAW_RANGE_RAD,
    MULTI_PICK_V2_X_SHIFT_M,
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

    def test_legacy_layout_payload_remains_frozen(self):
        scene = generate_scene(
            self.base_scene,
            seed=17036,
            profile="mixed",
            plant_count=2,
            position_band="middle",
            occlusion="partial",
            layout_contract="legacy_random_v1",
        )
        payload = {
            key: scene[key]
            for key in ("condition", "plants", "fruits", "evaluation")
        }
        digest = hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        self.assertEqual(
            digest,
            "bebda12eceeead9ee6dc0050da4be83d19a24ccf31aef78eb77ec2a8ca23c845",
        )

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

    def test_multi_pick_v2_guarantees_two_primary_ripe_slots_without_truth_control(self):
        positions_by_band = {}
        seed = 45101
        for plant_count in (2, 3):
            for band in ("near", "middle", "far"):
                for occlusion in ("none", "partial", "heavy"):
                    scene = generate_scene(
                        self.base_scene,
                        seed=seed,
                        profile="mixed",
                        plant_count=plant_count,
                        position_band=band,
                        occlusion=occlusion,
                        layout_contract="multi_pick_v2",
                    )
                    seed += 1
                    validate_generated_scene(scene)
                    primary = [
                        row
                        for row in scene["fruits"]
                        if row["multi_pick_primary"]
                    ]
                    self.assertEqual(
                        scene["generator"]["layout_contract"], "multi_pick_v2"
                    )
                    self.assertFalse(scene["generator"]["truth_for_runtime_control"])
                    roles = {row["layout_role"] for row in scene["plants"]}
                    self.assertEqual(
                        {"primary_upper", "primary_lower"},
                        roles - {"distractor"},
                    )
                    self.assertEqual(len(primary), 2)
                    self.assertEqual(
                        {(row["layout_role"], row["slot_id"]) for row in primary},
                        {("primary_upper", 1), ("primary_lower", 2)},
                    )
                    self.assertTrue(
                        all(row["maturity"] == "RIPE" for row in primary)
                    )
                    self.assertTrue(
                        all(row["reachable_by_construction"] for row in primary)
                    )
                    self.assertTrue(
                        any(
                            row["maturity"] == "UNRIPE" for row in scene["fruits"]
                        )
                    )
                    positions_by_band.setdefault(
                        band,
                        tuple(
                            tuple(round(value, 6) for value in row["initial_pose_m"])
                            for row in primary
                        ),
                    )
        self.assertEqual(len(set(positions_by_band.values())), 3)

    def test_multi_pick_v2_three_plant_scene_keeps_a_distractor(self):
        scene = generate_scene(
            self.base_scene,
            seed=45110,
            profile="mixed",
            plant_count=3,
            layout_contract="multi_pick_v2",
        )
        self.assertEqual(
            {row["layout_role"] for row in scene["plants"]},
            {"primary_upper", "primary_lower", "distractor"},
        )
        self.assertTrue(
            any(
                row["layout_role"] == "distractor"
                and not row["multi_pick_primary"]
                for row in scene["fruits"]
            )
        )

    def test_multi_pick_v2_validator_rejects_primary_contract_drift(self):
        scene = generate_scene(
            self.base_scene,
            seed=45101,
            profile="mixed",
            plant_count=2,
            layout_contract="multi_pick_v2",
        )
        broken = deepcopy(scene)
        primary = next(row for row in broken["fruits"] if row["multi_pick_primary"])
        primary["maturity"] = "UNRIPE"
        primary["asset"] = "strawberry_unripe_generalized"
        with self.assertRaisesRegex(ValueError, "multi-pick primary"):
            validate_generated_scene(broken)

    def test_multi_pick_v2_policy_matches_hash_bound_development_diagnostic(self):
        diagnostic = json.loads(
            (
                PACKAGE.parents[2]
                / "config"
                / "generalized_feasibility_observed_envelope_v1.json"
            ).read_text(encoding="utf-8")
        )
        policy = diagnostic["generator_policy"]
        self.assertFalse(diagnostic["formal_acceptance"])
        self.assertFalse(diagnostic["runtime_control_input"])
        self.assertFalse(policy["moveit_feasibility_guaranteed"])
        self.assertEqual(
            policy["primary_spawn_patterns"],
            [
                {
                    "layout_role": role,
                    "plant_anchor_xy_m": list(anchor),
                    "required_slot_id": int(slot_index) + 1,
                }
                for role, anchor, slot_index in MULTI_PICK_V2_ANCHORS[:2]
            ],
        )
        self.assertEqual(
            policy["primary_yaw_range_rad"],
            list(MULTI_PICK_V2_PRIMARY_YAW_RANGE_RAD),
        )
        self.assertEqual(policy["position_band_x_shift_m"], MULTI_PICK_V2_X_SHIFT_M)

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
