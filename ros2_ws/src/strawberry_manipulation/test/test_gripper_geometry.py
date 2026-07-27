import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.gripper_geometry import (  # noqa: E402
    GripperGeometry,
    closest_point_on_triangle,
    point_to_oriented_wrist_housing_distance,
    read_binary_stl,
    select_recommendation,
    transform_finger_mesh,
)


class GripperGeometryTests(unittest.TestCase):
    def test_closest_point_covers_face_edge_and_vertex_regions(self):
        triangle = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        self.assertEqual(
            closest_point_on_triangle((0.25, 0.25, 1.0), triangle),
            (0.25, 0.25, 0.0),
        )
        edge = closest_point_on_triangle((0.75, 0.75, 0.0), triangle)
        self.assertAlmostEqual(edge[0], 0.5)
        self.assertAlmostEqual(edge[1], 0.5)
        self.assertEqual(
            closest_point_on_triangle((-1.0, -1.0, 0.0), triangle),
            (0.0, 0.0, 0.0),
        )

    def test_binary_stl_reader_is_strict(self):
        header = b"\0" * 80 + struct.pack("<I", 1)
        record = (
            struct.pack("<3f", 0.0, 0.0, 1.0)
            + struct.pack("<3f", 0.0, 0.0, 0.0)
            + struct.pack("<3f", 1.0, 0.0, 0.0)
            + struct.pack("<3f", 0.0, 1.0, 0.0)
            + struct.pack("<H", 0)
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "triangle.stl"
            path.write_bytes(header + record)
            triangles = read_binary_stl(path)
        self.assertEqual(len(triangles), 1)
        self.assertEqual(triangles[0][1], (1.0, 0.0, 0.0))

    def test_right_finger_transform_mirrors_about_hand_center(self):
        source = (((1.0, 2.0, 3.0),) * 3,)
        left = transform_finger_mesh(
            source, side="left", joint_width_m=0.02, joint_z_m=0.0584
        )
        right = transform_finger_mesh(
            source, side="right", joint_width_m=0.02, joint_z_m=0.0584
        )
        self.assertEqual(left[0][0], (1.0, 2.02, 3.0584))
        self.assertEqual(right[0][0], (-1.0, -2.02, 3.0584))

    def test_wrist_housing_distance_uses_frozen_pitch(self):
        geometry = GripperGeometry()
        distance = point_to_oriented_wrist_housing_distance(
            (0.0, 0.0, 0.0964),
            center_m=geometry.wrist_housing_center_m,
            size_m=geometry.wrist_housing_size_m,
        )
        expected = math.hypot(0.065 - 0.020, 0.0514 - 0.0275)
        self.assertAlmostEqual(distance, expected)

    def test_recommendation_selection_is_deterministic(self):
        candidates = [
            {
                "feasible": True,
                "minimum_normalized_threshold_margin": 0.2,
                "close_command_overtravel_m": 0.004,
                "palm_clearance_m": 0.004,
                "tool_center_offset_m": 0.0964,
                "close_width_m_per_finger": 0.022,
            },
            {
                "feasible": True,
                "minimum_normalized_threshold_margin": 0.1,
                "close_command_overtravel_m": 0.002,
                "palm_clearance_m": 0.006,
                "tool_center_offset_m": 0.1004,
                "close_width_m_per_finger": 0.025,
            },
        ]
        self.assertIs(select_recommendation(candidates), candidates[0])

    def test_frozen_adr_and_runner_keep_motion_denied(self):
        adr = (
            REPOSITORY_ROOT
            / "docs"
            / "decisions"
            / "0043-freeze-blender-v2-gripper-geometry-sweep.md"
        ).read_text(encoding="utf-8")
        runner = (
            REPOSITORY_ROOT
            / "scripts"
            / "qualify_blender_v2_gripper_geometry.py"
        ).read_text(encoding="utf-8")
        self.assertIn("Arm motion", adr)
        self.assertIn('"robot_motion_started": False', runner)
        self.assertIn('"gripper_command_started": False', runner)
        self.assertNotIn("ParallelGripperCommand", runner)
        self.assertNotIn("SetEntityPose", runner)

    def test_contract_if_present_is_safe(self):
        path = (
            REPOSITORY_ROOT
            / "config"
            / "blender_v2_gripper_geometry_sweep_v1.json"
        )
        if not path.exists():
            self.skipTest("contract is added after source hashes are frozen")
        contract = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(contract["candidate_grid"]["expected_candidate_count"], 24)
        self.assertFalse(contract["safety"]["robot_motion_authorized"])
        self.assertFalse(contract["safety"]["gripper_command_authorized"])
        self.assertFalse(contract["safety"]["attachment_authorized"])


if __name__ == "__main__":
    unittest.main()
