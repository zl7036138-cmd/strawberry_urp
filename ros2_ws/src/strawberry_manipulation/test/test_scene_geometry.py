import pathlib
import sys
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
SIM_ROOT = PACKAGE_ROOT.parent / "strawberry_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.scene_geometry import (  # noqa: E402
    BoxPrimitive,
    CollisionObjectSpec,
    FIELD_V3_STATIC_COLLISION_OBJECTS,
    FRUIT_COLLISION_RADIUS_M,
    SpherePrimitive,
    STATIC_COLLISION_OBJECTS,
    TABLE_TOP_PADDING_M,
    fruit_collision_id,
    static_collision_objects,
)


def _collision_boxes(parent):
    boxes = []
    for collision in parent.findall("./link/collision"):
        pose = tuple(float(value) for value in collision.findtext("pose").split())
        size = tuple(
            float(value)
            for value in collision.findtext("./geometry/box/size").split()
        )
        boxes.append((pose[:3], size))
    return tuple(boxes)


class SceneGeometryTests(unittest.TestCase):
    def test_ids_are_unique_and_dimensions_are_positive(self):
        ids = [specification.object_id for specification in STATIC_COLLISION_OBJECTS]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertEqual(
            set(ids),
            {
                "work_table",
                "strawberry_planter",
                "strawberry_plant_crown",
                "collection_bin",
            },
        )
        for specification in STATIC_COLLISION_OBJECTS:
            for box in specification.boxes:
                self.assertTrue(all(size > 0.0 for size in box.size_m))

    def test_invalid_specs_fail_fast(self):
        with self.assertRaises(ValueError):
            BoxPrimitive((0.0, 0.0, 0.0), (1.0, 0.0, 1.0))
        with self.assertRaises(ValueError):
            CollisionObjectSpec("", (BoxPrimitive((0, 0, 0), (1, 1, 1)),))
        with self.assertRaises(ValueError):
            SpherePrimitive((0.0, 0.0, 0.0), 0.0)
        with self.assertRaises(ValueError):
            fruit_collision_id(0)
        with self.assertRaisesRegex(
            ValueError, "unknown static collision profile"
        ):
            static_collision_objects("missing")

    def test_randomized_plant_crowns_follow_every_scene_plant(self):
        objects = static_collision_objects(
            "blender_v2",
            plant_positions_m=((0.40, -0.10, 0.47), (0.46, 0.20, 0.47)),
        )
        crowns = {
            specification.object_id: specification.boxes[0]
            for specification in objects
            if specification.object_id.startswith("strawberry_plant_crown")
        }
        self.assertEqual(
            set(crowns),
            {"strawberry_plant_crown_1", "strawberry_plant_crown_2"},
        )
        self.assertEqual(
            crowns["strawberry_plant_crown_1"].center_m, (0.40, -0.10, 0.482)
        )
        self.assertEqual(
            crowns["strawberry_plant_crown_2"].center_m, (0.46, 0.20, 0.482)
        )
        self.assertEqual(
            len({specification.object_id for specification in objects}), len(objects)
        )

    def test_field_v3_geometry_matches_gazebo_sdf(self):
        world_root = ET.parse(
            SIM_ROOT / "worlds" / "strawberry_field_v3.sdf"
        ).getroot()
        field_root = ET.parse(
            SIM_ROOT / "models" / "strawberry_field_v3" / "model.sdf"
        ).getroot()
        bin_root = ET.parse(
            SIM_ROOT / "models" / "collection_bin" / "model.sdf"
        ).getroot()
        plant_root = ET.parse(
            SIM_ROOT / "models" / "strawberry_plant_v2" / "model.sdf"
        ).getroot()

        field_boxes = _collision_boxes(field_root.find("./model"))
        bin_boxes = _collision_boxes(bin_root.find("./model"))
        crown = plant_root.find(
            "./model/link/collision[@name='crown_collision']"
        )
        self.assertIsNotNone(crown)
        crown_pose = tuple(
            float(value) for value in crown.findtext("pose").split()
        )
        crown_radius = float(crown.findtext("geometry/cylinder/radius"))
        crown_length = float(crown.findtext("geometry/cylinder/length"))

        includes = {
            include.findtext("name"): include
            for include in world_root.findall("./world/include")
        }
        bin_pose = tuple(
            float(value)
            for value in includes["collection_bin"].findtext("pose").split()
        )
        plant_pose = tuple(
            float(value)
            for value in includes["strawberry_plant"].findtext("pose").split()
        )
        moveit_boxes = {
            specification.object_id: tuple(
                (box.center_m, box.size_m) for box in specification.boxes
            )
            for specification in FIELD_V3_STATIC_COLLISION_OBJECTS
        }

        for index, object_id in enumerate(
            ("field_ground", "field_ridge_1", "field_ridge_2", "field_ridge_3")
        ):
            self.assertEqual(moveit_boxes[object_id], (field_boxes[index],))
        self.assertEqual(
            moveit_boxes["collection_bin"],
            tuple(
                (
                    tuple(
                        round(center[axis] + bin_pose[axis], 9)
                        for axis in range(3)
                    ),
                    size,
                )
                for center, size in bin_boxes
            ),
        )
        self.assertEqual(
            moveit_boxes["strawberry_plant_crown"],
            (
                (
                    tuple(
                        round(crown_pose[axis] + plant_pose[axis], 9)
                        for axis in range(3)
                    ),
                    (
                        2.0 * crown_radius,
                        2.0 * crown_radius,
                        crown_length,
                    ),
                ),
            ),
        )
        self.assertIs(
            static_collision_objects("field_v3"),
            FIELD_V3_STATIC_COLLISION_OBJECTS,
        )

    def test_fruit_collision_contract_matches_gazebo_assets(self):
        self.assertEqual(fruit_collision_id(3), "strawberry_fruit_3")
        for model_name in ("strawberry_ripe", "strawberry_unripe"):
            root = ET.parse(
                SIM_ROOT / "models" / model_name / "model.sdf"
            ).getroot()
            radius = float(
                root.findtext(
                    "./model/link/collision/geometry/sphere/radius"
                )
            )
            self.assertAlmostEqual(radius, FRUIT_COLLISION_RADIUS_M)

    def test_moveit_geometry_matches_gazebo_sdf(self):
        world_root = ET.parse(
            SIM_ROOT / "worlds" / "strawberry_orchard.sdf"
        ).getroot()
        table_model = world_root.find("./world/model[@name='work_table']")
        self.assertIsNotNone(table_model)
        plant_include = world_root.find("./world/include[name='strawberry_plant']")
        self.assertIsNotNone(plant_include)
        plant_pose = tuple(
            float(value) for value in plant_include.findtext("pose").split()
        )
        self.assertEqual(plant_pose, (0.50, 0.0, 0.47, 0.0, 0.0, 0.0))
        planter_model = world_root.find("./world/model[@name='strawberry_planter']")
        self.assertIsNotNone(planter_model)
        planter = planter_model.find("./link/collision[@name='planter_collision']")
        self.assertIsNotNone(planter)
        bin_include = world_root.find("./world/include[name='collection_bin']")
        self.assertIsNotNone(bin_include)
        self.assertEqual(
            tuple(float(value) for value in bin_include.findtext("pose").split()),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        bin_root = ET.parse(
            SIM_ROOT / "models" / "collection_bin" / "model.sdf"
        ).getroot()
        bin_model = bin_root.find("./model[@name='collection_bin']")
        self.assertIsNotNone(bin_model)

        sdf_boxes = {
            "work_table": _collision_boxes(table_model),
            "collection_bin": _collision_boxes(bin_model),
        }
        moveit_boxes = {
            specification.object_id: tuple(
                (box.center_m, box.size_m) for box in specification.boxes
            )
            for specification in STATIC_COLLISION_OBJECTS
        }
        self.assertEqual(
            moveit_boxes["collection_bin"], sdf_boxes["collection_bin"]
        )
        moveit_table = moveit_boxes["work_table"][0]
        sdf_table = sdf_boxes["work_table"][0]
        self.assertEqual(moveit_table[0][:2], sdf_table[0][:2])
        self.assertEqual(moveit_table[1][:2], sdf_table[1][:2])
        sdf_bottom = sdf_table[0][2] - sdf_table[1][2] / 2.0
        sdf_top = sdf_table[0][2] + sdf_table[1][2] / 2.0
        moveit_bottom = moveit_table[0][2] - moveit_table[1][2] / 2.0
        moveit_top = moveit_table[0][2] + moveit_table[1][2] / 2.0
        self.assertAlmostEqual(moveit_bottom, sdf_bottom)
        self.assertAlmostEqual(moveit_top, sdf_top + TABLE_TOP_PADDING_M)
        plant_root = ET.parse(
            SIM_ROOT / "models" / "strawberry_plant_v2" / "model.sdf"
        ).getroot()
        crown = plant_root.find("./model/link/collision[@name='crown_collision']")
        self.assertIsNotNone(crown)
        crown_pose = tuple(
            float(value) for value in crown.findtext("pose").split()
        )
        radius = float(crown.findtext("geometry/cylinder/radius"))
        length = float(crown.findtext("geometry/cylinder/length"))
        self.assertEqual(
            moveit_boxes["strawberry_plant_crown"],
            (
                (
                    tuple(
                        plant_pose[index] + crown_pose[index]
                        for index in range(3)
                    ),
                    (2.0 * radius, 2.0 * radius, length),
                ),
            ),
        )
        planter_pose = tuple(float(value) for value in planter.findtext("pose").split())
        planter_radius = float(planter.findtext("geometry/cylinder/radius"))
        planter_length = float(planter.findtext("geometry/cylinder/length"))
        self.assertEqual(
            moveit_boxes["strawberry_planter"],
            (
                (
                    planter_pose[:3],
                    (2.0 * planter_radius, 2.0 * planter_radius, planter_length),
                ),
            ),
        )


if __name__ == "__main__":
    unittest.main()
