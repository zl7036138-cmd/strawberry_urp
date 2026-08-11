import pathlib
import sys
import unittest

import numpy as np


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_localization.core import BoundingBox  # noqa: E402
from strawberry_localization.generalized_depth import (  # noqa: E402
    adjust_point_along_optical_ray,
    expand_bounding_box,
    point_on_pixel_bearing,
    retain_foreground_depth_band,
)


class GeneralizedDepthBandTests(unittest.TestCase):
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
        )
        np.testing.assert_array_equal(depth, original)

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


if __name__ == "__main__":
    unittest.main()
