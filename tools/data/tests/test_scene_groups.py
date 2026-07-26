from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

import cv2
import numpy as np


TOOLS_ROOT = Path(__file__).resolve().parents[1]
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

import build_scene_groups as scene_groups  # noqa: E402


def _pair_metrics(left: str, right: str, decision: str) -> scene_groups.PairMetrics:
    return scene_groups.PairMetrics(
        image_a=left,
        image_b=right,
        reasons=("test",),
        phash_distance=0,
        dhash_distance=0,
        embedding_similarity=None,
        good_matches=100,
        inliers=90,
        inlier_ratio=0.9,
        homography_valid=True,
        projected_area_ratio=1.0,
        condition_number=1.0,
        source_inlier_coverage=0.5,
        target_inlier_coverage=0.5,
        decision=decision,
    )


def _match(left: scene_groups.ImageFeatures, right: scene_groups.ImageFeatures):
    cv2.setRNGSeed(20260710)
    return scene_groups._match_pair(
        left,
        right,
        reasons={"test_candidate"},
        embedding_similarity=None,
        ratio_threshold=0.75,
        ransac_threshold=5.0,
        min_area_ratio=0.05,
        max_area_ratio=20.0,
        max_condition_number=1e8,
        min_inlier_coverage=0.005,
        strict_good=50,
        strict_inliers=40,
        strict_inlier_ratio=0.50,
        conservative_good=25,
        conservative_inliers=20,
        conservative_inlier_ratio=0.35,
        merge_conservative=True,
    )


def _synthetic_scene(seed: int) -> np.ndarray:
    """Create a deterministic, feature-rich image without external fixtures."""

    rng = np.random.default_rng(seed)
    image = np.full((480, 640, 3), 24, dtype=np.uint8)
    for _ in range(180):
        centre = tuple(int(value) for value in rng.integers((15, 15), (625, 465)))
        radius = int(rng.integers(3, 13))
        colour = tuple(int(value) for value in rng.integers(40, 256, size=3))
        cv2.circle(image, centre, radius, colour, -1, lineType=cv2.LINE_AA)
    for _ in range(35):
        start = tuple(int(value) for value in rng.integers((0, 0), (640, 480)))
        end = tuple(int(value) for value in rng.integers((0, 0), (640, 480)))
        colour = tuple(int(value) for value in rng.integers(40, 256, size=3))
        cv2.line(image, start, end, colour, 2, lineType=cv2.LINE_AA)
    cv2.putText(
        image,
        f"SCENE-{seed}",
        (120, 250),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.4,
        (255, 255, 255),
        3,
        lineType=cv2.LINE_AA,
    )
    return image


class EmbeddingChunkTest(unittest.TestCase):
    def test_embedding_paths_are_really_chunked_before_yolo_embed(self):
        calls: list[dict[str, object]] = []
        all_paths: list[Path] = []

        class FakeYOLO:
            def __init__(self, weights: str):
                self.weights = weights

            def embed(self, **kwargs):
                source = list(kwargs["source"])
                calls.append(
                    {
                        "source": source,
                        "batch": kwargs["batch"],
                        "imgsz": kwargs["imgsz"],
                        "device": kwargs["device"],
                    }
                )
                vectors = []
                for item in source:
                    index = all_paths.index(Path(item))
                    vectors.append(np.asarray([index + 1.0, 1.0], dtype=np.float32))
                return vectors

        fake_ultralytics = types.ModuleType("ultralytics")
        fake_ultralytics.YOLO = FakeYOLO
        fake_ultralytics.__version__ = "test-version"

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            weights = root / "yolo11n.pt"
            weights.write_bytes(b"test weights")
            all_paths.extend(root / f"image_{index}.jpg" for index in range(7))

            with mock.patch.dict(sys.modules, {"ultralytics": fake_ultralytics}):
                pairs, similarities, metadata = scene_groups._embedding_candidates(
                    all_paths,
                    weights,
                    top_k=1,
                    imgsz=320,
                    batch=3,
                    device="mock-device",
                )

        self.assertEqual([len(call["source"]) for call in calls], [3, 3, 1])
        self.assertEqual([call["batch"] for call in calls], [3, 3, 1])
        self.assertEqual(
            [Path(item) for call in calls for item in call["source"]],
            all_paths,
        )
        self.assertTrue(pairs)
        self.assertEqual(similarities.shape, (7, 7))
        self.assertEqual(metadata["embedding_dimension"], 2)
        self.assertEqual(metadata["batch"], 3)
        self.assertEqual(metadata["ultralytics_version"], "test-version")


class GeometryTest(unittest.TestCase):
    def _features(self, directory: Path, images: dict[str, np.ndarray]):
        relative_paths = []
        for name, image in images.items():
            path = directory / name
            self.assertTrue(cv2.imwrite(str(path), image))
            relative_paths.append(name)
        return scene_groups._extract_features(
            directory,
            relative_paths,
            max_dimension=800,
            orb_features=1200,
            orb_fast_threshold=12,
        )

    def test_perspective_duplicate_merges_on_geometry(self):
        base = _synthetic_scene(20260710)
        source_corners = np.float32(
            [[0, 0], [639, 0], [639, 479], [0, 479]]
        )
        target_corners = np.float32(
            [[18, 12], [622, 7], [634, 465], [9, 474]]
        )
        transform = cv2.getPerspectiveTransform(source_corners, target_corners)
        perspective = cv2.warpPerspective(base, transform, (640, 480))

        with tempfile.TemporaryDirectory() as temporary:
            features = self._features(
                Path(temporary),
                {"base.png": base, "perspective.png": perspective},
            )
        result = _match(features[0], features[1])

        self.assertTrue(result.homography_valid)
        self.assertTrue(result.decision.startswith("merge_"))
        self.assertGreaterEqual(result.good_matches, 50)
        self.assertGreaterEqual(result.inliers, 40)
        self.assertGreaterEqual(result.inlier_ratio, 0.50)

    def test_unrelated_scenes_remain_separate(self):
        with tempfile.TemporaryDirectory() as temporary:
            features = self._features(
                Path(temporary),
                {
                    "scene_a.png": _synthetic_scene(101),
                    "scene_b.png": _synthetic_scene(909),
                },
            )
        result = _match(features[0], features[1])

        self.assertEqual(result.decision, "separate_no_geometric_evidence")
        self.assertFalse(
            result.homography_valid
            and result.good_matches >= 25
            and result.inliers >= 20
            and result.inlier_ratio >= 0.35
        )


class OverrideConflictTest(unittest.TestCase):
    def test_transitive_merge_cannot_silently_violate_separate_override(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            source.mkdir()
            names = ["a.jpg", "b.jpg", "c.jpg"]
            for name in names:
                (source / name).write_bytes(b"fixture")

            overrides = root / "overrides.csv"
            with overrides.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream, lineterminator="\n")
                writer.writerow(("image_a", "image_b", "decision", "reason"))
                writer.writerow(("a.jpg", "c.jpg", "separate", "different scenes"))

            dummy_features = [
                scene_groups.ImageFeatures(
                    relative_path=name,
                    path=source / name,
                    file_sha256=str(index),
                    decoded_sha256=str(index),
                    width=1,
                    height=1,
                    feature_width=1,
                    feature_height=1,
                    phash=np.zeros(63, dtype=bool),
                    dhash=np.zeros(256, dtype=bool),
                    keypoints=np.empty((0, 2), dtype=np.float32),
                    descriptors=None,
                )
                for index, name in enumerate(names)
            ]
            candidates = {
                (0, 1): {"test"},
                (1, 2): {"test"},
            }

            def merged_pair(left, right, **_kwargs):
                return _pair_metrics(
                    left.relative_path,
                    right.relative_path,
                    "merge_strict_geometry",
                )

            with (
                mock.patch.object(
                    scene_groups, "_extract_features", return_value=dummy_features
                ),
                mock.patch.object(
                    scene_groups,
                    "_hash_candidates",
                    return_value=(candidates, {"0": 3}),
                ),
                mock.patch.object(scene_groups, "_match_pair", side_effect=merged_pair),
            ):
                with self.assertRaisesRegex(
                    ValueError, "separate overrides conflict with transitive merge"
                ):
                    scene_groups.main(
                        [
                            "--source",
                            str(source),
                            "--output",
                            str(root / "groups.csv"),
                            "--audit-json",
                            str(root / "audit.json"),
                            "--overrides",
                            str(overrides),
                            "--embedding-top-k",
                            "0",
                        ]
                    )

            self.assertFalse((root / "groups.csv").exists())
            self.assertFalse((root / "audit.json").exists())


if __name__ == "__main__":
    unittest.main()
