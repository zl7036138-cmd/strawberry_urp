import pathlib
import sys
import unittest

import numpy as np


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_perception.shadow_diagnostic_core import (  # noqa: E402
    ProjectedTruth,
    associate_rows_to_truth,
    bgr_from_rgb,
    box_iou,
    project_sphere,
    quaternion_rotation_matrix,
    summarize_modes,
)


class ChannelContractTests(unittest.TestCase):
    def test_rgb_to_bgr_is_an_exact_contiguous_swap(self):
        rgb = np.array([[[255, 7, 2], [1, 2, 3]]], dtype=np.uint8)
        bgr = bgr_from_rgb(rgb)
        np.testing.assert_array_equal(
            bgr, np.array([[[2, 7, 255], [3, 2, 1]]], dtype=np.uint8)
        )
        self.assertTrue(bgr.flags.c_contiguous)

    def test_rejects_wrong_shape_and_dtype(self):
        with self.assertRaises(ValueError):
            bgr_from_rgb(np.zeros((4, 4), dtype=np.uint8))
        with self.assertRaises(ValueError):
            bgr_from_rgb(np.zeros((4, 4, 3), dtype=np.float32))


class ProjectionTests(unittest.TestCase):
    def test_identity_quaternion_has_identity_rotation(self):
        np.testing.assert_allclose(
            quaternion_rotation_matrix((0.0, 0.0, 0.0, 1.0)), np.eye(3)
        )

    def test_projects_sphere_at_principal_point(self):
        truth = project_sphere(
            target_id=1,
            maturity="ripe",
            center_in_source_m=(0.0, 0.0, 1.0),
            source_to_camera_translation_m=(0.0, 0.0, 0.0),
            source_to_camera_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            intrinsics=(100.0, 100.0, 50.0, 40.0),
            image_size=(100, 80),
            radius_m=0.1,
        )
        self.assertIsNotNone(truth)
        self.assertEqual(truth.center_uv, (50.0, 40.0))
        self.assertEqual(truth.bbox_xyxy, (40.0, 30.0, 60.0, 50.0))
        self.assertEqual(truth.maturity, "RIPE")

    def test_rejects_points_behind_camera(self):
        self.assertIsNone(
            project_sphere(
                target_id=1,
                maturity="RIPE",
                center_in_source_m=(0.0, 0.0, -1.0),
                source_to_camera_translation_m=(0.0, 0.0, 0.0),
                source_to_camera_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                intrinsics=(100.0, 100.0, 50.0, 40.0),
                image_size=(100, 80),
            )
        )


class AssociationTests(unittest.TestCase):
    def setUp(self):
        self.truth = (
            ProjectedTruth(1, "RIPE", (10.0, 10.0, 30.0, 30.0), (20.0, 20.0), 1.0),
            ProjectedTruth(2, "UNRIPE", (50.0, 10.0, 70.0, 30.0), (60.0, 20.0), 1.0),
        )

    def test_box_iou_and_best_truth_association(self):
        self.assertEqual(box_iou((10, 10, 30, 30), (10, 10, 30, 30)), 1.0)
        rows = [[11, 11, 29, 29, 0.9, 0], [80, 10, 90, 20, 0.8, 1]]
        associated = associate_rows_to_truth(rows, ("ripe", "unripe"), self.truth)
        self.assertEqual(associated[0]["associated_target_id"], 1)
        self.assertEqual(associated[0]["associated_truth_maturity"], "RIPE")
        self.assertIsNone(associated[1]["associated_target_id"])

    def test_summary_keeps_modes_and_truth_confusion_separate(self):
        detection = {
            "class_name": "ripe",
            "associated_truth_maturity": "RIPE",
        }
        summary = summarize_modes(
            [{"legacy_rgb": [], "correct_bgr": [detection]}]
        )
        self.assertEqual(summary["legacy_rgb"]["detection_count"], 0)
        self.assertEqual(summary["correct_bgr"]["by_predicted_class"], {"ripe": 1})
        self.assertEqual(
            summary["correct_bgr"]["by_associated_truth_and_prediction"],
            {"RIPE->ripe": 1},
        )


if __name__ == "__main__":
    unittest.main()
