import pathlib
import sys
import unittest

import numpy as np


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_localization.core import (  # noqa: E402
    BoundingBox,
    CameraIntrinsics,
    LocalizationError,
    SensorFrameCache,
    associate_nearest_target,
    localize_bbox,
    robust_center_depth,
    robust_geometry_layer_depth,
    summarize_position_errors,
    validate_bbox_within_image,
    validate_sensor_metadata,
)


class LocalizationCoreTests(unittest.TestCase):
    def test_depth_median_rejects_nan_and_outlier(self) -> None:
        depth = np.full((100, 100), 1.2, dtype=np.float32)
        depth[45:55, 45:55] = 0.8
        depth[50, 50] = np.nan
        depth[49, 49] = 4.0
        estimate = robust_center_depth(depth, BoundingBox(35, 35, 30, 30))
        self.assertAlmostEqual(estimate.depth_m, 0.8, places=5)
        self.assertGreaterEqual(estimate.valid_pixels, 9)

    def test_geometry_layer_recovers_fruit_behind_center_occluder(self) -> None:
        depth = np.full((100, 100), 1.8, dtype=np.float32)
        yy, xx = np.ogrid[:100, :100]
        fruit_mask = (xx - 50) ** 2 + (yy - 50) ** 2 <= 28**2
        depth[fruit_mask] = 0.50
        depth[20:80, 40:60] = 0.18
        box = BoundingBox(20, 20, 60, 60)
        intrinsics = CameraIntrinsics(600.0, 600.0, 50.0, 50.0)

        legacy = robust_center_depth(depth, box)
        recovered = robust_geometry_layer_depth(
            depth,
            box,
            intrinsics,
            target_radius_m=0.026,
            expected_depth_tolerance_m=0.08,
        )

        self.assertAlmostEqual(legacy.depth_m, 0.18, places=5)
        self.assertAlmostEqual(recovered.depth_m, 0.50, places=5)
        self.assertGreater(recovered.valid_pixels, 100)
        self.assertAlmostEqual(recovered.center_u, 50.0, delta=1.0)
        self.assertAlmostEqual(recovered.center_v, 50.0, delta=1.0)

    def test_geometry_layer_rejects_when_fruit_depth_is_absent(self) -> None:
        depth = np.full((100, 100), 1.8, dtype=np.float32)
        depth[20:80, 40:60] = 0.18

        with self.assertRaisesRegex(
            LocalizationError,
            "inconsistent with detected fruit size",
        ):
            robust_geometry_layer_depth(
                depth,
                BoundingBox(20, 20, 60, 60),
                CameraIntrinsics(600.0, 600.0, 50.0, 50.0),
                target_radius_m=0.026,
                expected_depth_tolerance_m=0.08,
            )

    def test_geometry_layer_rejects_ambiguous_layers(self) -> None:
        depth = np.full((100, 100), 1.8, dtype=np.float32)
        depth[20:80, 20:50] = 0.48
        depth[20:80, 50:80] = 0.51

        with self.assertRaisesRegex(LocalizationError, "geometrically ambiguous"):
            robust_geometry_layer_depth(
                depth,
                BoundingBox(20, 20, 60, 60),
                CameraIntrinsics(600.0, 600.0, 50.0, 50.0),
                target_radius_m=0.026,
                expected_depth_tolerance_m=0.08,
                ambiguity_margin_m=0.01,
            )

    def test_geometry_layer_does_not_count_box_quantization_as_sigma(self) -> None:
        depth = np.full((40, 40), 2.0, dtype=np.float32)
        depth[7:33, 7:34] = 1.14

        estimate = robust_geometry_layer_depth(
            depth,
            BoundingBox(7, 7, 27, 26),
            CameraIntrinsics(554.0, 554.0, 20.0, 20.0),
            target_radius_m=0.026,
            expected_depth_tolerance_m=0.08,
            bbox_quantization_margin_px=2.0,
        )

        self.assertAlmostEqual(estimate.depth_m, 1.14, places=5)
        self.assertAlmostEqual(estimate.sigma_m, 0.0, places=5)

    def test_geometry_layer_accepts_a_dominant_near_tied_layer(self) -> None:
        depth = np.full((100, 100), 1.8, dtype=np.float32)
        depth[20:80, 20:75] = 0.481
        depth[20:80, 75:80] = 0.509

        estimate = robust_geometry_layer_depth(
            depth,
            BoundingBox(20, 20, 60, 60),
            CameraIntrinsics(600.0, 600.0, 50.0, 50.0),
            target_radius_m=0.026,
            expected_depth_tolerance_m=0.08,
            ambiguity_margin_m=0.01,
            ambiguity_min_support_ratio=0.50,
        )

        self.assertAlmostEqual(estimate.depth_m, 0.481, places=5)
        self.assertEqual(estimate.valid_pixels, 3300)

    def test_geometry_layer_localize_mode_uses_selected_pixel_centroid(self) -> None:
        depth = np.full((100, 100), 1.8, dtype=np.float32)
        depth[35:65, 25:45] = 0.50
        point, estimate = localize_bbox(
            depth,
            BoundingBox(20, 20, 60, 60),
            CameraIntrinsics(600.0, 600.0, 50.0, 50.0),
            surface_to_center_offset_m=0.026,
            depth_estimator_mode="geometry_layer",
            geometry_expected_depth_tolerance_m=0.08,
        )

        self.assertLess(point[0], 0.0)
        self.assertAlmostEqual(estimate.center_u, 34.5)
        self.assertAlmostEqual(estimate.center_v, 49.5)

    def test_geometry_layer_requires_target_radius(self) -> None:
        with self.assertRaisesRegex(ValueError, "target radius must be positive"):
            localize_bbox(
                np.ones((20, 20), dtype=np.float32),
                BoundingBox(5, 5, 10, 10),
                CameraIntrinsics(100.0, 100.0, 10.0, 10.0),
                depth_estimator_mode="geometry_layer",
            )

    def test_projection_at_principal_point(self) -> None:
        depth = np.full((20, 20), 2.0, dtype=np.float32)
        point, estimate = localize_bbox(
            depth,
            BoundingBox(8, 8, 4, 4),
            CameraIntrinsics(fx=100.0, fy=100.0, cx=10.0, cy=10.0),
            center_fraction=1.0,
        )
        np.testing.assert_allclose(point, np.array([0.0, 0.0, 2.0]))
        self.assertAlmostEqual(estimate.sigma_m, 0.0)

    def test_surface_to_center_offset_moves_exactly_along_camera_ray(self) -> None:
        depth = np.full((20, 20), 2.0, dtype=np.float32)
        intrinsics = CameraIntrinsics(fx=100.0, fy=100.0, cx=10.0, cy=10.0)
        surface, _ = localize_bbox(
            depth,
            BoundingBox(10, 8, 4, 4),
            intrinsics,
            center_fraction=1.0,
        )
        center, _ = localize_bbox(
            depth,
            BoundingBox(10, 8, 4, 4),
            intrinsics,
            center_fraction=1.0,
            surface_to_center_offset_m=0.035,
        )
        self.assertAlmostEqual(float(np.linalg.norm(center - surface)), 0.035)
        np.testing.assert_allclose(
            center / np.linalg.norm(center),
            surface / np.linalg.norm(surface),
        )

    def test_surface_to_center_offset_rejects_invalid_value(self) -> None:
        with self.assertRaises(ValueError):
            localize_bbox(
                np.ones((10, 10), dtype=np.float32),
                BoundingBox(2, 2, 4, 4),
                CameraIntrinsics(fx=100.0, fy=100.0, cx=5.0, cy=5.0),
                surface_to_center_offset_m=-0.001,
            )

    def test_invalid_depth_is_rejected(self) -> None:
        depth = np.zeros((50, 50), dtype=np.float32)
        with self.assertRaises(LocalizationError):
            robust_center_depth(depth, BoundingBox(10, 10, 20, 20))

    def test_box_outside_image_is_rejected(self) -> None:
        depth = np.ones((20, 20), dtype=np.float32)
        with self.assertRaises(LocalizationError):
            robust_center_depth(depth, BoundingBox(30, 30, 10, 10))

    def test_bbox_contract_rejects_partial_image_overflow(self) -> None:
        with self.assertRaises(LocalizationError):
            validate_bbox_within_image(BoundingBox(15, 5, 10, 10), 20, 20)

    def test_nonfinite_camera_intrinsics_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            CameraIntrinsics(fx=float("nan"), fy=100.0, cx=10.0, cy=10.0)

    def test_sensor_metadata_accepts_fresh_synchronized_frames(self) -> None:
        validate_sensor_metadata(
            detection_stamp_s=10.0,
            detection_frame="camera_optical",
            depth_stamp_s=10.01,
            depth_frame="camera_optical",
            camera_info_stamp_s=9.99,
            camera_info_frame="camera_optical",
            now_stamp_s=10.10,
            sync_tolerance_sec=0.02,
            stale_after_sec=0.50,
        )

    def test_sensor_metadata_rejects_frame_mismatch(self) -> None:
        with self.assertRaises(LocalizationError):
            validate_sensor_metadata(
                detection_stamp_s=10.0,
                detection_frame="camera_optical",
                depth_stamp_s=10.0,
                depth_frame="camera_link",
                camera_info_stamp_s=10.0,
                camera_info_frame="camera_optical",
                now_stamp_s=10.1,
                sync_tolerance_sec=0.02,
                stale_after_sec=0.50,
            )

    def test_sensor_metadata_rejects_unsynchronized_or_stale_input(self) -> None:
        common = {
            "detection_frame": "camera_optical",
            "depth_frame": "camera_optical",
            "camera_info_frame": "camera_optical",
            "sync_tolerance_sec": 0.02,
            "stale_after_sec": 0.50,
        }
        with self.assertRaises(LocalizationError):
            validate_sensor_metadata(
                detection_stamp_s=10.0,
                depth_stamp_s=10.10,
                camera_info_stamp_s=10.0,
                now_stamp_s=10.1,
                **common,
            )
        with self.assertRaises(LocalizationError):
            validate_sensor_metadata(
                detection_stamp_s=10.0,
                depth_stamp_s=10.0,
                camera_info_stamp_s=10.0,
                now_stamp_s=10.6,
                **common,
            )

    def test_sensor_cache_retrieves_old_frame_after_new_frames_arrive(self) -> None:
        cache = SensorFrameCache(capacity=8, retention_sec=2.0)
        intrinsics = CameraIntrinsics(320.0, 320.0, 2.0, 2.0)
        old_depth = np.full((4, 4), 1.0, dtype=np.float32)
        new_depth = np.full((4, 4), 2.0, dtype=np.float32)
        cache.add_depth(stamp_s=10.00, frame_id="camera_optical", image=old_depth)
        cache.add_camera_info(
            stamp_s=10.00,
            frame_id="camera_optical",
            width=4,
            height=4,
            intrinsics=intrinsics,
            payload="old calibration",
        )
        cache.add_depth(stamp_s=10.10, frame_id="camera_optical", image=new_depth)
        cache.add_camera_info(
            stamp_s=10.10,
            frame_id="camera_optical",
            width=4,
            height=4,
            intrinsics=intrinsics,
            payload="new calibration",
        )

        matched = cache.match(
            detection_stamp_s=10.00,
            detection_frame="camera_optical",
            sync_tolerance_sec=0.02,
        )

        self.assertIs(matched.depth.image, old_depth)
        self.assertEqual(matched.camera_info.payload, "old calibration")

    def test_sensor_cache_prunes_expired_entries(self) -> None:
        cache = SensorFrameCache(capacity=8, retention_sec=1.0)
        intrinsics = CameraIntrinsics(320.0, 320.0, 2.0, 2.0)
        cache.add_depth(
            stamp_s=5.0,
            frame_id="camera_optical",
            image=np.ones((4, 4), dtype=np.float32),
        )
        cache.add_camera_info(
            stamp_s=5.0,
            frame_id="camera_optical",
            width=4,
            height=4,
            intrinsics=intrinsics,
        )

        removed = cache.prune(6.01)

        self.assertEqual(removed, 2)
        self.assertEqual(cache.depth_count, 0)
        self.assertEqual(cache.camera_info_count, 0)
        with self.assertRaises(LocalizationError):
            cache.match(
                detection_stamp_s=5.0,
                detection_frame="camera_optical",
                sync_tolerance_sec=0.02,
            )

    def test_sensor_cache_diagnostics_report_nearest_signed_deltas(self) -> None:
        cache = SensorFrameCache(capacity=4, retention_sec=2.0)
        intrinsics = CameraIntrinsics(320.0, 320.0, 2.0, 2.0)
        for stamp in (10.0, 10.2):
            cache.add_depth(
                stamp_s=stamp,
                frame_id="camera_optical",
                image=np.ones((4, 4), dtype=np.float32),
            )
        cache.add_camera_info(
            stamp_s=10.15,
            frame_id="camera_optical",
            width=4,
            height=4,
            intrinsics=intrinsics,
        )

        diagnostics = cache.diagnostics(
            detection_stamp_s=10.12,
            detection_frame="camera_optical",
        )

        self.assertEqual(diagnostics["depth_count"], 2)
        self.assertEqual(diagnostics["camera_info_count"], 1)
        self.assertAlmostEqual(
            diagnostics["nearest_depth_delta_sec"], 0.08
        )
        self.assertAlmostEqual(
            diagnostics["nearest_camera_info_delta_sec"], 0.03
        )

    def test_sensor_cache_skips_nearer_dimension_mismatch(self) -> None:
        cache = SensorFrameCache(capacity=8, retention_sec=2.0)
        intrinsics = CameraIntrinsics(320.0, 320.0, 2.0, 2.0)
        valid_depth = np.ones((4, 4), dtype=np.float32)
        cache.add_depth(
            stamp_s=10.0, frame_id="camera_optical", image=valid_depth
        )
        cache.add_camera_info(
            stamp_s=10.0,
            frame_id="camera_optical",
            width=8,
            height=8,
            intrinsics=intrinsics,
            payload="incompatible",
        )
        cache.add_camera_info(
            stamp_s=9.99,
            frame_id="camera_optical",
            width=4,
            height=4,
            intrinsics=intrinsics,
            payload="compatible",
        )

        matched = cache.match(
            detection_stamp_s=10.0,
            detection_frame="camera_optical",
            sync_tolerance_sec=0.02,
        )

        self.assertEqual(matched.camera_info.payload, "compatible")

    def test_position_error_summary_requires_and_reports_100_samples(self) -> None:
        truth = [(0.0, 0.0, 0.0)] * 100
        estimated = [(index / 1000.0, 0.0, 0.0) for index in range(100)]
        metrics = summarize_position_errors(estimated, truth)
        self.assertEqual(metrics.sample_count, 100)
        self.assertAlmostEqual(metrics.median_error_mm, 49.5)
        self.assertAlmostEqual(metrics.p95_error_mm, 94.05)

    def test_position_error_summary_rejects_too_few_samples(self) -> None:
        with self.assertRaises(LocalizationError):
            summarize_position_errors(
                [(0.0, 0.0, 0.0)] * 99,
                [(0.0, 0.0, 0.0)] * 99,
            )

    def test_simulation_identity_association_uses_nearest_position(self) -> None:
        target_id = associate_nearest_target(
            (0.49, 0.19, 0.65),
            {
                1: (0.55, -0.18, 0.68),
                2: (0.62, 0.02, 0.72),
                3: (0.48, 0.20, 0.64),
            },
        )
        self.assertEqual(target_id, 3)

    def test_simulation_identity_association_rejects_distant_match(self) -> None:
        with self.assertRaises(LocalizationError):
            associate_nearest_target(
                (1.0, 1.0, 1.0),
                {1: (0.55, -0.18, 0.68)},
                max_distance_m=0.05,
            )


if __name__ == "__main__":
    unittest.main()
