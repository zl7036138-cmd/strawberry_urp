from pathlib import Path
import sys
import tempfile
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.grasp_geometry import (  # noqa: E402
    load_grasp_geometry,
    profiles_from_mapping,
)


class GraspGeometryProfileTests(unittest.TestCase):
    def test_canonical_v2_and_legacy_profiles_are_distinct(self):
        path = PACKAGE_ROOT / "config" / "grasp_geometry.yaml"
        v2 = load_grasp_geometry(
            path,
            world_name="strawberry_orchard",
            fruit_collision_radius_m=0.026,
        )
        legacy = load_grasp_geometry(
            path,
            world_name="strawberry_orchard",
            fruit_collision_radius_m=0.035,
        )
        self.assertEqual(v2.profile_id, "blender_v2_26mm")
        self.assertEqual(v2.tool_center_offset_m, 0.0964)
        self.assertEqual(v2.gripper_closed_width_m_per_finger, 0.022)
        self.assertEqual(
            v2.gripper_position_tolerance_m_per_finger, 0.0039
        )
        self.assertEqual(legacy.profile_id, "tabletop_v1_35mm")
        self.assertEqual(legacy.tool_center_offset_m, 0.1054)
        self.assertEqual(legacy.gripper_closed_width_m_per_finger, 0.025)

    def test_unknown_radius_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "exactly one matching"):
            load_grasp_geometry(
                PACKAGE_ROOT / "config" / "grasp_geometry.yaml",
                world_name="strawberry_orchard",
                fruit_collision_radius_m=0.030,
            )

    def test_duplicate_scene_radius_binding_is_rejected(self):
        data = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "grasp_geometry.yaml").read_text(
                encoding="utf-8"
            )
        )
        duplicate = dict(data["profiles"][0])
        duplicate["profile_id"] = "duplicate"
        data["profiles"].append(duplicate)
        with self.assertRaisesRegex(ValueError, "bindings must be unique"):
            profiles_from_mapping(data)

    def test_invalid_close_width_is_rejected(self):
        data = {
            "schema_version": 1,
            "profiles": [
                {
                    "profile_id": "bad",
                    "world_name": "world",
                    "fruit_collision_radius_m": 0.026,
                    "tool_center_offset_m": 0.0964,
                    "gripper_open_width_m_per_finger": 0.04,
                    "gripper_closed_width_m_per_finger": 0.026,
                    "gripper_position_tolerance_m_per_finger": 0.003,
                }
            ],
        }
        with self.assertRaisesRegex(ValueError, "contact over-travel"):
            profiles_from_mapping(data)

    def test_yaml_round_trip_does_not_change_numeric_selection(self):
        source = (
            PACKAGE_ROOT / "config" / "grasp_geometry.yaml"
        ).read_text(encoding="utf-8")
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "grasp.yaml"
            path.write_text(source, encoding="utf-8")
            selected = load_grasp_geometry(
                path,
                world_name="strawberry_orchard",
                fruit_collision_radius_m=0.026,
            )
        self.assertEqual(
            (
                selected.tool_center_offset_m,
                selected.gripper_closed_width_m_per_finger,
                selected.gripper_position_tolerance_m_per_finger,
            ),
            (0.0964, 0.022, 0.0039),
        )


if __name__ == "__main__":
    unittest.main()
