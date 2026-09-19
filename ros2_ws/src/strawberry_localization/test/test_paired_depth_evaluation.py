import hashlib
import importlib.util
import json
import pathlib
import tempfile
import unittest

import numpy as np


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
SCRIPT_PATH = (
    REPOSITORY_ROOT / "scripts" / "evaluate_paired_rgbd_depth_estimators.py"
)
SPEC = importlib.util.spec_from_file_location("paired_depth_evaluation", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class PairedDepthEvaluationTests(unittest.TestCase):
    def test_scaled_box_preserves_center_and_clips_to_image(self) -> None:
        box = MODULE.BoundingBox(20, 20, 60, 40)

        expanded = MODULE._scaled_box(
            box,
            1.1,
            image_width=100,
            image_height=80,
        )

        self.assertEqual(expanded, MODULE.BoundingBox(17, 18, 66, 44))
        self.assertAlmostEqual(expanded.x + 0.5 * expanded.width, 50.0)
        self.assertAlmostEqual(expanded.y + 0.5 * expanded.height, 40.0)

    def test_foreground_injection_is_central_and_non_mutating(self) -> None:
        source = np.full((20, 20), 0.50, dtype=np.float32)

        modified = MODULE._inject_foreground(
            source,
            MODULE.BoundingBox(5, 5, 10, 10),
            width_fraction=0.40,
            foreground_depth_m=0.20,
        )

        self.assertTrue(np.all(source == 0.50))
        self.assertTrue(np.all(modified[5:15, 8:12] == np.float32(0.20)))
        self.assertTrue(np.all(modified[5:15, 5:8] == np.float32(0.50)))
        self.assertTrue(np.all(modified[5:15, 12:15] == np.float32(0.50)))

    def test_quaternion_rotation_maps_x_to_y(self) -> None:
        half_sqrt_two = 2.0**-0.5

        result = MODULE._rotate_vector_by_quaternion(
            np.asarray([1.0, 0.0, 0.0]),
            np.asarray([0.0, 0.0, half_sqrt_two, half_sqrt_two]),
        )

        np.testing.assert_allclose(result, [0.0, 1.0, 0.0], atol=1e-12)

    def test_bundle_crops_depth_and_rebinds_intrinsics(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = pathlib.Path(temporary_directory)
            capture_path = root / "capture.npz"
            receipt_path = root / "capture.json"
            bundle_path = root / "bundle.npz"
            np.savez_compressed(
                capture_path,
                depth_m=np.full((1, 20, 20), 0.50, dtype=np.float32),
                roi_xywh=np.asarray([[5, 5, 10, 10]], dtype=np.int32),
                intrinsics_fx_fy_cx_cy=np.asarray(
                    [[100.0, 100.0, 10.0, 10.0]], dtype=np.float64
                ),
                base_from_camera_translation_m=np.zeros((1, 3)),
                base_from_camera_rotation_xyzw=np.asarray(
                    [[0.0, 0.0, 0.0, 1.0]], dtype=np.float64
                ),
                truth_target_xyz_m=np.asarray(
                    [[0.0, 0.0, 0.526]], dtype=np.float64
                ),
                stamp_sec_nanosec=np.asarray([[1, 2]], dtype=np.int64),
            )
            receipt_path.write_text(
                json.dumps(
                    {
                        "sample_count": 1,
                        "scope": "TEST_CAPTURE",
                        "truth_use": "TEST_ONLY",
                        "control_commands_sent": 0,
                        "pick_action_started": False,
                        "dataset": {
                            "size_bytes": capture_path.stat().st_size,
                            "sha256": hashlib.sha256(
                                capture_path.read_bytes()
                            ).hexdigest(),
                        },
                    }
                ),
                encoding="utf-8",
            )

            metadata = MODULE.build_depth_bundle(
                [capture_path],
                [receipt_path],
                bundle_path,
            )

            self.assertEqual(metadata["sample_count"], 1)
            with np.load(bundle_path, allow_pickle=False) as bundle:
                self.assertEqual(bundle["depth_000"].shape, (14, 14))
                np.testing.assert_array_equal(bundle["bbox_xywh"], [[2, 2, 10, 10]])
                np.testing.assert_allclose(
                    bundle["intrinsics_fx_fy_cx_cy"],
                    [[100.0, 100.0, 7.0, 7.0]],
                )

    def test_bundle_rejects_a_receipt_hash_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = pathlib.Path(temporary_directory)
            capture_path = root / "capture.npz"
            receipt_path = root / "capture.json"
            np.savez_compressed(
                capture_path,
                depth_m=np.ones((1, 4, 4), dtype=np.float32),
                roi_xywh=np.asarray([[0, 0, 4, 4]], dtype=np.int32),
                intrinsics_fx_fy_cx_cy=np.asarray(
                    [[10.0, 10.0, 2.0, 2.0]], dtype=np.float64
                ),
                base_from_camera_translation_m=np.zeros((1, 3)),
                base_from_camera_rotation_xyzw=np.asarray(
                    [[0.0, 0.0, 0.0, 1.0]], dtype=np.float64
                ),
                truth_target_xyz_m=np.zeros((1, 3)),
                stamp_sec_nanosec=np.asarray([[1, 2]], dtype=np.int64),
            )
            receipt_path.write_text(
                json.dumps(
                    {
                        "sample_count": 1,
                        "control_commands_sent": 0,
                        "pick_action_started": False,
                        "dataset": {
                            "size_bytes": capture_path.stat().st_size,
                            "sha256": "0" * 64,
                        },
                    }
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "does not bind"):
                MODULE.build_depth_bundle(
                    [capture_path],
                    [receipt_path],
                    root / "bundle.npz",
                )

    def test_matrix_scope_is_offline_and_no_motion(self) -> None:
        source = SCRIPT_PATH.read_text(encoding="utf-8")

        self.assertIn('"robot_motion_during_evaluation": False', source)
        self.assertIn('"runtime_promotion_authorized": False', source)
        self.assertNotIn("rclpy", source)
        self.assertNotIn("control_msgs", source)


if __name__ == "__main__":
    unittest.main()
