import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_sim.core import (  # noqa: E402
    AttachmentGate,
    BinBounds,
    BinStabilityTracker,
    ContactStabilityTracker,
    AMBIGUOUS_FRUIT_CONTACT,
    BILATERAL_SAME_FRUIT,
    LEFT_SINGLE_FRUIT,
    NO_FRUIT_CONTACT,
    Pose3D,
    RIGHT_SINGLE_FRUIT,
    classify_anonymous_fruit_contacts,
    dual_pad_geometric_contact,
    fruit_models_in_contacts,
    fruit_models_contacting_entity,
    parse_attachment_state,
    scene_config_from_mapping,
    select_unique_contact_target,
)


def scene_mapping():
    return {
        "schema_version": 1,
        "world_name": "test_world",
        "fruit_collision_radius_m": 0.026,
        "frames": {"robot_base": "panda_link0", "camera_optical": "camera_optical"},
        "fruits": [
            {
                "target_id": 1,
                "model_name": "fruit_1",
                "maturity": "RIPE",
                "initial_pose_m": [0.4, 0.0, 0.5],
            },
            {
                "target_id": 2,
                "model_name": "fruit_2",
                "maturity": "UNRIPE",
                "initial_pose_m": [0.5, 0.0, 0.5],
            },
        ],
        "bin": {
            "interior_bounds_m": {
                "min_x": 0.0,
                "max_x": 1.0,
                "min_y": -1.0,
                "max_y": 0.0,
                "min_z": 0.0,
                "max_z": 1.0,
            },
            "required_stability_sec": 1.0,
        },
    }


class SimulationCoreTests(unittest.TestCase):
    def test_scene_contract(self):
        scene = scene_config_from_mapping(scene_mapping())
        self.assertEqual(scene.fruit_collision_radius_m, 0.026)
        self.assertEqual(scene.static_collision_profile, "blender_v2")
        self.assertEqual([item.target_id for item in scene.ordered_fruits], [1, 2])
        self.assertEqual(scene.fruit(1).maturity, "RIPE")
        self.assertEqual(
            scene.fruit(1).pose_tf_topic,
            "/strawberry/sim/fruit_1/pose_tf",
        )
        self.assertEqual(
            scene.fruit(1).ground_truth_pose_topic,
            "/strawberry/ground_truth/fruit_1/pose",
        )
        for topic in (
            scene.fruit(1).pose_tf_topic,
            scene.fruit(1).ground_truth_pose_topic,
        ):
            self.assertTrue(all(not token[:1].isdigit() for token in topic.split("/")))

    def test_scene_parses_legacy_and_multi_plant_positions(self):
        legacy = scene_mapping()
        legacy["plant"] = {
            "model_name": "plant_legacy",
            "pose_in_robot_base": [0.50, 0.0, 0.47, 0.0, 0.0, 0.2],
        }
        legacy_scene = scene_config_from_mapping(legacy)
        self.assertEqual(legacy_scene.plants[0].plant_id, 1)
        self.assertEqual(legacy_scene.plants[0].position_m, (0.50, 0.0, 0.47))

        generalized = scene_mapping()
        generalized["plants"] = [
            {
                "plant_id": 1,
                "model_name": "plant_1",
                "pose_in_robot_base": [0.40, -0.10, 0.47, 0.0, 0.0, 0.1],
            },
            {
                "plant_id": 2,
                "model_name": "plant_2",
                "pose_in_robot_base": [0.46, 0.20, 0.47, 0.0, 0.0, -0.1],
            },
        ]
        generalized_scene = scene_config_from_mapping(generalized)
        self.assertEqual(
            [plant.position_m for plant in generalized_scene.plants],
            [(0.40, -0.10, 0.47), (0.46, 0.20, 0.47)],
        )

    def test_scene_requires_positive_fruit_collision_radius(self):
        mapping = scene_mapping()
        mapping["fruit_collision_radius_m"] = 0.0
        with self.assertRaisesRegex(
            ValueError, "fruit collision radius must be positive"
        ):
            scene_config_from_mapping(mapping)

    def test_legacy_schema_v1_scene_defaults_to_35_mm_radius(self):
        mapping = scene_mapping()
        del mapping["fruit_collision_radius_m"]
        scene = scene_config_from_mapping(mapping)
        self.assertEqual(scene.fruit_collision_radius_m, 0.035)

    def test_scene_accepts_explicit_static_collision_profile(self):
        mapping = scene_mapping()
        mapping["planning_scene"] = {
            "static_collision_profile": "field_v3",
        }
        scene = scene_config_from_mapping(mapping)
        self.assertEqual(scene.static_collision_profile, "field_v3")

    def test_bin_stability_requires_uninterrupted_time(self):
        bounds = BinBounds(0, 1, -1, 0, 0, 1)
        tracker = BinStabilityTracker(bounds, 1.0)
        inside = Pose3D(0.5, -0.5, 0.5)
        outside = Pose3D(1.5, -0.5, 0.5)
        self.assertFalse(tracker.update(1, inside, 0.0))
        self.assertFalse(tracker.update(1, outside, 0.5))
        self.assertFalse(tracker.update(1, inside, 1.0))
        self.assertTrue(tracker.update(1, inside, 2.0))

    def test_contact_stability_requires_fresh_uninterrupted_contact(self):
        tracker = ContactStabilityTracker(1.0, 0.25)
        tracker.update(1, True, 1.0)
        self.assertFalse(tracker.is_stable(1, 1.9))
        tracker.update(1, True, 1.9)
        self.assertFalse(tracker.is_stable(1, 2.0))
        for stamp in (2.0, 2.2, 2.4, 2.6, 2.8, 3.0):
            tracker.update(1, True, stamp)
        self.assertTrue(tracker.is_stable(1, 3.0))
        tracker.update(1, False, 3.1)
        self.assertFalse(tracker.is_stable(1, 3.1))

    def test_attach_gate_requires_fresh_dual_contact(self):
        gate = AttachmentGate(0.25)
        accepted = gate.assess_attach(
            now_sec=2.0,
            backend_enabled=True,
            backend_initialized=True,
            state_known=True,
            attached=False,
            left_contact=True,
            left_stamp_sec=1.9,
            right_contact=True,
            right_stamp_sec=1.8,
        )
        self.assertTrue(accepted.allowed)
        rejected = gate.assess_attach(
            now_sec=2.0,
            backend_enabled=True,
            backend_initialized=True,
            state_known=True,
            attached=False,
            left_contact=True,
            left_stamp_sec=1.0,
            right_contact=True,
            right_stamp_sec=1.9,
        )
        self.assertFalse(rejected.allowed)

    def test_contact_target_requires_exactly_one_bilateral_candidate(self):
        self.assertEqual(
            select_unique_contact_target({1: False, 2: True, 3: False}),
            (2, "unique dual-contact fruit confirmed"),
        )
        target_id, reason = select_unique_contact_target({1: False, 2: False})
        self.assertIsNone(target_id)
        self.assertIn("no fruit", reason)
        target_id, reason = select_unique_contact_target({1: True, 2: True})
        self.assertIsNone(target_id)
        self.assertIn("ambiguous", reason)

    def test_contact_target_rejects_nonpositive_entity_id(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            select_unique_contact_target({0: True})

    def test_anonymous_contact_class_distinguishes_safe_centering_cases(self):
        classify = lambda contacts: classify_anonymous_fruit_contacts(
            contacts, now_sec=2.0, freshness_sec=0.25
        )
        self.assertEqual(
            classify({1: (False, 2.0, False, 2.0)}), NO_FRUIT_CONTACT
        )
        self.assertEqual(
            classify({1: (True, 1.9, False, 2.0)}), LEFT_SINGLE_FRUIT
        )
        self.assertEqual(
            classify({1: (False, 2.0, True, 1.9)}), RIGHT_SINGLE_FRUIT
        )
        self.assertEqual(
            classify({1: (True, 1.9, True, 1.9)}), BILATERAL_SAME_FRUIT
        )
        self.assertEqual(
            classify(
                {
                    1: (True, 1.9, False, 2.0),
                    2: (False, 2.0, True, 1.9),
                }
            ),
            AMBIGUOUS_FRUIT_CONTACT,
        )

    def test_anonymous_contact_class_ignores_stale_contact(self):
        self.assertEqual(
            classify_anonymous_fruit_contacts(
                {1: (True, 1.0, False, 2.0)},
                now_sec=2.0,
                freshness_sec=0.25,
            ),
            NO_FRUIT_CONTACT,
        )

    def test_geometric_contact_requires_close_pads_straddling_fruit(self):
        fruit = Pose3D(0.42, -0.12, 0.52)
        left = Pose3D(0.42, -0.145, 0.54)
        right = Pose3D(0.42, -0.095, 0.54)
        self.assertTrue(dual_pad_geometric_contact(fruit, left, right, 0.038))
        self.assertFalse(
            dual_pad_geometric_contact(
                fruit,
                Pose3D(0.42, -0.16, 0.54),
                Pose3D(0.42, -0.08, 0.54),
                0.038,
            )
        )
        self.assertFalse(
            dual_pad_geometric_contact(
                fruit,
                Pose3D(0.42, -0.145, 0.54),
                Pose3D(0.42, -0.135, 0.54),
                0.038,
            )
        )

    def test_detachable_joint_state_is_parsed_strictly(self):
        self.assertTrue(parse_attachment_state("attached"))
        self.assertFalse(parse_attachment_state(" detached "))
        with self.assertRaises(ValueError):
            parse_attachment_state("false")

    def test_contact_pairs_map_exact_scoped_fruit_models(self):
        models = {1: "strawberry_1", 2: "strawberry_2"}
        active = fruit_models_in_contacts(
            [
                (
                    "panda::panda_left_contact_pad::pad_collision",
                    "strawberry_2::fruit_link::fruit_collision",
                ),
                (
                    "panda::other",
                    "strawberry_10::fruit_link::fruit_collision",
                ),
            ],
            models,
        )
        self.assertEqual(active, frozenset({2}))

    def test_contact_pairs_identify_only_fruits_touching_bin(self):
        models = {1: "strawberry_1", 2: "strawberry_2"}
        active = fruit_models_contacting_entity(
            [
                (
                    "strawberry_1::fruit_link::fruit_collision",
                    "collection_bin::bin_link::floor_collision",
                ),
                (
                    "strawberry_2::fruit_link::fruit_collision",
                    "plant_2::stem::collision",
                ),
            ],
            models,
            "floor_collision",
        )
        self.assertEqual(active, frozenset({1}))


if __name__ == "__main__":
    unittest.main()
