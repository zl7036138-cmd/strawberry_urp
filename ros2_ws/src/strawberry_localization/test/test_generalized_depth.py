import pathlib
import sys
import unittest

import numpy as np


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_localization.core import (  # noqa: E402
    BoundingBox,
    CameraIntrinsics,
    DepthEstimate,
    LocalizationError,
    robust_geometry_layer_depth,
)
from strawberry_localization.generalized_depth import (  # noqa: E402
    adjust_point_along_optical_ray,
    calibrate_runtime_geometry_uncertainty,
    center_seeded_geometry_layer_depth,
    expand_bounding_box,
    point_on_pixel_bearing,
    retain_foreground_depth_band,
    retain_support_ranked_geometry_layer,
    validate_geometry_layer_centroid,
)


class GeneralizedDepthBandTests(unittest.TestCase):
    def test_geometry_layer_centroid_gate_is_detector_anchored(self):
        box = BoundingBox(100, 60, 40, 20)
        accepted = DepthEstimate(0.8, 0.01, 20, 124.0, 71.0)
        rejected = DepthEstimate(0.8, 0.01, 20, 132.0, 78.0)

        self.assertLess(
            validate_geometry_layer_centroid(
                accepted, box, maximum_distance_fraction=0.15
            ),
            0.15,
        )
        with self.assertRaisesRegex(LocalizationError, "off-centre"):
            validate_geometry_layer_centroid(
                rejected, box, maximum_distance_fraction=0.15
            )

    @staticmethod
    def _center_seeded(depth):
        return center_seeded_geometry_layer_depth(
            depth,
            BoundingBox(0, 0, 26, 26),
            BoundingBox(1, 1, 24, 24),
            fx=554.256,
            fy=554.256,
            target_radius_m=0.026,
            min_depth_m=0.05,
            max_depth_m=5.0,
            min_layer_pixels=9,
            layer_gap_m=0.015,
            minimum_support_fraction=0.14,
            maximum_centroid_distance_fraction=0.25,
            maximum_geometry_residual_m=0.30,
            ambiguity_margin_fraction=0.05,
            minimum_sigma_m=0.015,
        )

    def test_center_seeded_fallback_selects_supported_central_fruit_layer(self):
        depth = np.full((26, 26), np.nan, dtype=np.float32)
        depth[:, :8] = 0.85
        depth[:, 18:] = 1.30
        depth[8:18, 8:18] = 1.03

        estimate = self._center_seeded(depth)

        self.assertAlmostEqual(estimate.depth_m, 1.03, places=5)
        self.assertEqual(estimate.valid_pixels, 100)
        self.assertEqual(estimate.sigma_m, 0.015)

    def test_center_seeded_fallback_rejects_weak_central_layer(self):
        depth = np.full((26, 26), np.nan, dtype=np.float32)
        depth[:, :10] = 0.85
        depth[:, 16:] = 1.30
        depth[10:16, 10:16] = 1.03

        with self.assertRaisesRegex(LocalizationError, "weak support"):
            self._center_seeded(depth)

    def test_center_seeded_fallback_rejects_implausible_or_ambiguous_layer(self):
        implausible = np.full((26, 26), np.nan, dtype=np.float32)
        implausible[:, :8] = 0.85
        implausible[8:18, 8:18] = 1.50
        with self.assertRaisesRegex(LocalizationError, "geometrically implausible"):
            self._center_seeded(implausible)

        ambiguous = np.full((26, 26), np.nan, dtype=np.float32)
        ambiguous[:, 4:13] = 1.02
        ambiguous[:, 13:22] = 1.08
        with self.assertRaisesRegex(LocalizationError, "ambiguous"):
            self._center_seeded(ambiguous)

    def test_support_ranker_prefers_dominant_near_tie(self):
        depth = np.full((21, 23), 0.765, dtype=np.float32)
        flattened = depth.reshape(-1)
        flattened[:280] = 0.627
        flattened[280:297] = 0.682

        filtered = retain_support_ranked_geometry_layer(
            depth,
            BoundingBox(0, 0, 23, 21),
            fx=277.128,
            fy=277.128,
            target_radius_m=0.026,
            search_fraction=1.0,
            min_depth_m=0.05,
            max_depth_m=5.0,
            min_layer_pixels=9,
            min_layer_fraction=0.03,
            layer_gap_m=0.015,
            expected_depth_tolerance_m=0.08,
            ambiguity_margin_m=0.01,
            ambiguity_min_support_ratio=0.50,
            bbox_quantization_margin_px=2.0,
        )
        estimate = robust_geometry_layer_depth(
            filtered,
            BoundingBox(0, 0, 23, 21),
            CameraIntrinsics(277.128, 277.128, 11.5, 10.5),
            target_radius_m=0.026,
            min_layer_fraction=0.03,
            expected_depth_tolerance_m=0.08,
            ambiguity_margin_m=0.01,
            ambiguity_min_support_ratio=0.50,
            bbox_quantization_margin_px=2.0,
        )

        self.assertAlmostEqual(estimate.depth_m, 0.627, places=5)
        self.assertEqual(estimate.valid_pixels, 280)

    def test_support_ranker_rejects_weak_natural_occluder_layer(self):
        depth = np.full((19, 20), 1.024, dtype=np.float32)
        flattened = depth.reshape(-1)
        flattened[:33] = 0.774
        flattened[33:205] = 0.819
        flattened[205:312] = 0.914

        filtered = retain_support_ranked_geometry_layer(
            depth,
            BoundingBox(0, 0, 20, 19),
            fx=277.128,
            fy=277.128,
            target_radius_m=0.026,
            search_fraction=1.0,
            min_depth_m=0.05,
            max_depth_m=5.0,
            min_layer_pixels=9,
            min_layer_fraction=0.03,
            layer_gap_m=0.015,
            expected_depth_tolerance_m=0.08,
            ambiguity_margin_m=0.025,
            ambiguity_min_support_ratio=0.50,
            bbox_quantization_margin_px=2.0,
        )
        estimate = robust_geometry_layer_depth(
            filtered,
            BoundingBox(0, 0, 20, 19),
            CameraIntrinsics(277.128, 277.128, 10.0, 9.5),
            target_radius_m=0.026,
            min_layer_fraction=0.03,
            expected_depth_tolerance_m=0.08,
            ambiguity_margin_m=0.025,
            ambiguity_min_support_ratio=0.50,
            bbox_quantization_margin_px=2.0,
        )

        self.assertAlmostEqual(estimate.depth_m, 0.819, places=5)
        self.assertEqual(estimate.valid_pixels, 172)

    def test_support_ranker_keeps_similarly_supported_conflict_ambiguous(self):
        depth = np.full((19, 20), 0.819, dtype=np.float32)
        depth.reshape(-1)[:150] = 0.774

        filtered = retain_support_ranked_geometry_layer(
            depth,
            BoundingBox(0, 0, 20, 19),
            fx=277.128,
            fy=277.128,
            target_radius_m=0.026,
            search_fraction=1.0,
            min_depth_m=0.05,
            max_depth_m=5.0,
            min_layer_pixels=9,
            min_layer_fraction=0.03,
            layer_gap_m=0.015,
            expected_depth_tolerance_m=0.08,
            ambiguity_margin_m=0.025,
            ambiguity_min_support_ratio=0.50,
            bbox_quantization_margin_px=2.0,
        )

        with self.assertRaisesRegex(LocalizationError, "geometrically ambiguous"):
            robust_geometry_layer_depth(
                filtered,
                BoundingBox(0, 0, 20, 19),
                CameraIntrinsics(277.128, 277.128, 10.0, 9.5),
                target_radius_m=0.026,
                min_layer_fraction=0.03,
                expected_depth_tolerance_m=0.08,
                ambiguity_margin_m=0.025,
                ambiguity_min_support_ratio=0.50,
                bbox_quantization_margin_px=2.0,
            )

    def test_support_ranker_leaves_zero_margin_input_unchanged(self):
        depth = np.full((21, 23), 0.627, dtype=np.float32)
        depth.reshape(-1)[:17] = 0.682

        filtered = retain_support_ranked_geometry_layer(
            depth,
            BoundingBox(0, 0, 23, 21),
            fx=277.128,
            fy=277.128,
            target_radius_m=0.026,
            search_fraction=1.0,
            min_depth_m=0.05,
            max_depth_m=5.0,
            min_layer_pixels=9,
            min_layer_fraction=0.03,
            layer_gap_m=0.015,
            expected_depth_tolerance_m=0.08,
            ambiguity_margin_m=0.0,
            ambiguity_min_support_ratio=0.50,
            bbox_quantization_margin_px=2.0,
        )

        np.testing.assert_array_equal(filtered, depth)

    def test_runtime_uncertainty_calibration_preserves_estimate_metadata(self):
        estimate = DepthEstimate(0.55, 0.024, 123, 40.5, 60.5)

        result = calibrate_runtime_geometry_uncertainty(
            estimate,
            weight=0.5,
            minimum_sigma_m=0.005,
        )

        self.assertEqual(result, DepthEstimate(0.55, 0.012, 123, 40.5, 60.5))

    def test_runtime_uncertainty_calibration_is_bounded(self):
        estimate = DepthEstimate(0.55, 0.002, 123, 40.5, 60.5)

        result = calibrate_runtime_geometry_uncertainty(
            estimate,
            weight=0.5,
            minimum_sigma_m=0.005,
        )

        self.assertEqual(result.sigma_m, 0.005)
        with self.assertRaises(ValueError):
            calibrate_runtime_geometry_uncertainty(estimate, weight=0.0)

    def test_distant_background_is_masked_after_supported_foreground(self):
        depth = np.full((20, 20), 0.84, dtype=np.float32)
        depth[4:16, 4:16] = 0.67
        depth[7:13, 7:13] = 0.65

        result = retain_foreground_depth_band(
            depth,
            BoundingBox(2, 2, 16, 16),
            search_fraction=1.0,
            min_depth_m=0.05,
            max_depth_m=5.0,
            min_layer_pixels=9,
            min_layer_fraction=0.03,
            layer_gap_m=0.015,
            maximum_band_width_m=0.06,
            minimum_band_fraction=0.20,
        )

        self.assertTrue(np.isnan(result[2, 2]))
        self.assertAlmostEqual(float(result[8, 8]), 0.65, places=5)
        self.assertAlmostEqual(float(result[5, 5]), 0.67, places=5)

    def test_input_depth_is_not_modified(self):
        depth = np.full((10, 10), 0.60, dtype=np.float32)
        original = depth.copy()
        retain_foreground_depth_band(
            depth,
            BoundingBox(0, 0, 10, 10),
            search_fraction=1.0,
            min_depth_m=0.05,
            max_depth_m=5.0,
            min_layer_pixels=9,
            min_layer_fraction=0.03,
            layer_gap_m=0.015,
            maximum_band_width_m=0.06,
            minimum_band_fraction=0.20,
        )
        np.testing.assert_array_equal(depth, original)

    def test_weak_near_sliver_does_not_truncate_supported_fruit_band(self):
        depth = np.full((16, 16), np.nan, dtype=np.float32)
        flattened = depth.reshape(-1)
        flattened[:15] = 0.930
        flattened[15:41] = 1.000
        flattened[41:148] = 1.042
        flattened[148:248] = 1.304

        result = retain_foreground_depth_band(
            depth,
            BoundingBox(0, 0, 16, 16),
            search_fraction=1.0,
            min_depth_m=0.05,
            max_depth_m=5.0,
            min_layer_pixels=9,
            min_layer_fraction=0.03,
            layer_gap_m=0.015,
            maximum_band_width_m=0.06,
            minimum_band_fraction=0.20,
        )

        self.assertAlmostEqual(float(result.reshape(-1)[0]), 0.930, places=5)
        self.assertAlmostEqual(float(result.reshape(-1)[41]), 1.042, places=5)
        self.assertTrue(np.isnan(result.reshape(-1)[148]))

    def test_foreground_filter_does_not_leave_a_cutoff_tail_as_a_new_layer(self):
        depth = np.full((20, 22), 0.614, dtype=np.float32)
        flattened = depth.reshape(-1)
        farther = (
            [0.672] * 18
            + [0.684, 0.696, 0.708, 0.720, 0.732, 0.744, 0.756, 0.768] * 9
            + [0.774] * 87
        )
        self.assertEqual(len(farther), 177)
        flattened[263:] = farther

        result = retain_foreground_depth_band(
            depth,
            BoundingBox(0, 0, 22, 20),
            search_fraction=1.0,
            min_depth_m=0.05,
            max_depth_m=5.0,
            min_layer_pixels=9,
            min_layer_fraction=0.03,
            layer_gap_m=0.015,
            maximum_band_width_m=0.06,
            minimum_band_fraction=0.20,
        )

        self.assertEqual(int(np.isfinite(result).sum()), 263)
        self.assertTrue(np.all(np.isnan(result.reshape(-1)[263:])))

    def test_bbox_expansion_is_symmetric_and_clamped(self):
        self.assertEqual(
            expand_bounding_box(
                BoundingBox(0, 2, 19, 20),
                padding_px=1,
                image_width=20,
                image_height=24,
            ),
            BoundingBox(0, 1, 20, 22),
        )

    def test_visual_radius_can_be_extended_to_physical_center(self):
        point = adjust_point_along_optical_ray(
            np.array([0.0, 0.0, 0.218]),
            0.026 - 0.018,
        )
        np.testing.assert_allclose(point, [0.0, 0.0, 0.226], atol=1e-9)

    def test_bbox_center_bearing_preserves_estimated_range(self):
        point = point_on_pixel_bearing(
            np.array([0.0, 0.0, 0.22]),
            u=330.0,
            v=220.0,
            fx=550.0,
            fy=550.0,
            cx=320.0,
            cy=240.0,
        )
        self.assertAlmostEqual(float(np.linalg.norm(point)), 0.22, places=9)
        self.assertGreater(point[0], 0.0)
        self.assertLess(point[1], 0.0)

    def test_bbox_center_bearing_removes_one_sided_centroid_bias(self):
        biased = np.array([0.018, 0.011, 0.88])
        corrected = point_on_pixel_bearing(
            biased,
            u=160.0,
            v=120.0,
            fx=277.128,
            fy=277.128,
            cx=160.0,
            cy=120.0,
        )

        self.assertAlmostEqual(float(np.linalg.norm(corrected)), float(np.linalg.norm(biased)))
        self.assertAlmostEqual(float(corrected[0]), 0.0)
        self.assertAlmostEqual(float(corrected[1]), 0.0)


if __name__ == "__main__":
    unittest.main()
