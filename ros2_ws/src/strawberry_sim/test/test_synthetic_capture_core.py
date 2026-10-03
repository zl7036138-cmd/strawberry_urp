import unittest

from strawberry_sim.synthetic_capture import normalized_orientation_xyzw
from strawberry_sim.synthetic_capture_core import (
    parse_yolo_label,
    project_sphere_in_camera,
    xyxy_to_yolo,
)


class SyntheticCaptureGeometryTests(unittest.TestCase):
    def test_manifest_orientation_is_normalized_and_defaults_to_identity(self):
        self.assertEqual(
            normalized_orientation_xyzw(None),
            (0.0, 0.0, 0.0, 1.0),
        )
        normalized = normalized_orientation_xyzw([0.0, 0.0, 1.0, 1.0])
        self.assertAlmostEqual(sum(value * value for value in normalized), 1.0)
        with self.assertRaisesRegex(ValueError, "non-zero"):
            normalized_orientation_xyzw([0.0, 0.0, 0.0, 0.0])

    def test_principal_point_projection_round_trips_to_yolo(self):
        box = project_sphere_in_camera(
            (0.0, 0.0, 1.0),
            (500.0, 500.0, 320.0, 240.0),
            (640, 480),
            0.035,
        )
        self.assertEqual((302.5, 222.5, 337.5, 257.5), box)
        yolo = xyxy_to_yolo(box, (640, 480))
        self.assertAlmostEqual(0.5, yolo[0])
        self.assertAlmostEqual(0.5, yolo[1])
        class_id, parsed = parse_yolo_label(
            "0 " + " ".join(format(value, ".10f") for value in yolo) + "\n"
        )
        self.assertEqual(0, class_id)
        for actual, expected in zip(parsed, yolo):
            self.assertAlmostEqual(expected, actual, places=9)

    def test_clipped_projection_remains_inside_image(self):
        box = project_sphere_in_camera(
            (-0.60, 0.0, 1.0),
            (500.0, 500.0, 320.0, 240.0),
            (640, 480),
            0.05,
        )
        self.assertEqual(0.0, box[0])
        self.assertGreater(box[2], box[0])
        xyxy_to_yolo(box, (640, 480))

    def test_invalid_depth_and_multiline_label_fail_closed(self):
        with self.assertRaisesRegex(ValueError, "depth"):
            project_sphere_in_camera(
                (0.0, 0.0, 0.01),
                (500.0, 500.0, 320.0, 240.0),
                (640, 480),
                0.035,
            )
        with self.assertRaisesRegex(ValueError, "exactly one"):
            parse_yolo_label("0 0.5 0.5 0.1 0.1\n1 0.5 0.5 0.1 0.1\n")


if __name__ == "__main__":
    unittest.main()
