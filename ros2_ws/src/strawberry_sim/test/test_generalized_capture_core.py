import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE))

from strawberry_sim.generalized_capture_core import (  # noqa: E402
    SPLIT_ORDER,
    depth_visible_yolo_labels,
    find_capture_spec,
    iter_capture_specs,
    load_capture_plan,
    validate_visibility_partition,
)


class GeneralizedCapturePlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan_path = ROOT / "config" / "generalized_development_capture_v1.json"
        cls.formal_path = ROOT / "config" / "generalized_harvest_matrix_v1.json"
        cls.plan = load_capture_plan(cls.plan_path, cls.formal_path)

    def test_splits_are_disjoint_from_formal_matrix(self):
        specs = iter_capture_specs(self.plan)
        self.assertEqual(len(specs), 120)
        self.assertEqual(len({row["seed"] for row in specs}), 120)
        formal = json.loads(self.formal_path.read_text(encoding="utf-8"))
        formal_seeds = {int(row["seed"]) for row in formal["scenarios"]}
        self.assertFalse(formal_seeds & {int(row["seed"]) for row in specs})

    def test_every_split_covers_scene_variation(self):
        specs = iter_capture_specs(self.plan)
        for split in SPLIT_ORDER:
            rows = [row for row in specs if row["split"] == split]
            self.assertEqual({row["profile"] for row in rows}, {"mixed", "all_unripe"})
            self.assertEqual({row["plant_count"] for row in rows}, {1, 2, 3})
            self.assertEqual({row["position_band"] for row in rows}, {"near", "middle", "far"})
            self.assertEqual({row["occlusion"] for row in rows}, {"none", "partial", "heavy"})

    def test_only_train_authorizes_optimization(self):
        specs = iter_capture_specs(self.plan)
        self.assertTrue(all(row["training_allowed"] for row in specs if row["split"] == "train"))
        self.assertFalse(any(row["training_allowed"] for row in specs if row["split"] != "train"))
        selected = find_capture_spec(self.plan, split="qualification", seed=43001)
        self.assertFalse(selected["training_allowed"])
        with self.assertRaisesRegex(ValueError, "outside"):
            find_capture_spec(self.plan, split="train", seed=31001)

    def test_training_config_cannot_see_qualification_split(self):
        training = (
            PACKAGE.parents[0]
            / "strawberry_perception"
            / "config"
            / "train_yolo11s_640_generalized_dev_v1.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn("generalized_development_capture_v1/dataset.yaml", training)
        self.assertNotIn("qualification.yaml", training)

    def test_formal_seed_overlap_is_rejected(self):
        raw = json.loads(self.plan_path.read_text(encoding="utf-8"))
        raw["splits"]["train"]["seed_start"] = 31001
        with tempfile.TemporaryDirectory() as directory:
            broken = Path(directory) / self.formal_path.name
            broken.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "overlap formal"):
                load_capture_plan(broken, self.formal_path)


class GeneralizedCaptureVisibilityTests(unittest.TestCase):
    def test_depth_support_retains_visible_and_rejects_hidden_fruit(self):
        depth = np.full((100, 100), 2.0, dtype=np.float32)
        depth[44:56, 44:56] = 0.91
        labels, excluded = depth_visible_yolo_labels(
            (
                {
                    "target_id": 1,
                    "maturity": "RIPE",
                    "center_camera_m": (0.0, 0.0, 1.0),
                    "radius_m": 0.1,
                },
                {
                    "target_id": 2,
                    "maturity": "UNRIPE",
                    "center_camera_m": (0.5, 0.0, 1.0),
                    "radius_m": 0.1,
                },
            ),
            depth_image_m=depth,
            intrinsics=(50.0, 50.0, 50.0, 50.0),
            image_size=(100, 100),
            minimum_visible_pixels=8,
            minimum_visible_fraction=0.05,
            depth_surface_padding_m=0.02,
        )
        self.assertEqual([row["target_id"] for row in labels], [1])
        self.assertEqual([row["target_id"] for row in excluded], [2])
        self.assertEqual(labels[0]["class_id"], 0)
        self.assertGreater(labels[0]["visible_fraction"], 0.5)

    def test_invalid_depth_shape_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "dimensions"):
            depth_visible_yolo_labels(
                (),
                depth_image_m=np.zeros((10, 10), dtype=np.float32),
                intrinsics=(1.0, 1.0, 1.0, 1.0),
                image_size=(20, 20),
                minimum_visible_pixels=1,
                minimum_visible_fraction=0.1,
                depth_surface_padding_m=0.01,
            )

    def test_fully_hidden_truth_is_a_complete_negative_partition(self):
        truth = (
            {"target_id": 1},
            {"target_id": 2},
        )
        excluded = (
            {"target_id": 1, "reason": "hidden"},
            {"target_id": 2, "reason": "hidden"},
        )
        self.assertTrue(validate_visibility_partition(truth, (), excluded))
        with self.assertRaisesRegex(ValueError, "cover"):
            validate_visibility_partition(truth, (), excluded[:1])


if __name__ == "__main__":
    unittest.main()
