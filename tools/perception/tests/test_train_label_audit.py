from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


TOOLS = Path(__file__).resolve().parents[1]


def _load(name: str):
    path = TOOLS / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


exporter = _load("export_train_audit_predictions")
packet = _load("build_train_label_audit_packet")


class TrainSplitLoaderTest(unittest.TestCase):
    def test_loader_rejects_validation_path(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            image = root / "images/val/a.jpg"
            label = root / "labels/train/a.txt"
            image.parent.mkdir(parents=True)
            label.parent.mkdir(parents=True)
            image.write_bytes(b"image")
            label.write_text("0 0.5 0.5 0.2 0.2\n", encoding="utf-8")
            manifest = root / "split_manifest.json"
            manifest.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "counts": {"train": 1},
                        "splits": {
                            "train": [
                                {
                                    "output_image": "images/val/a.jpg",
                                    "output_label": "labels/train/a.txt",
                                    "output_image_size_bytes": image.stat().st_size,
                                    "output_label_size_bytes": label.stat().st_size,
                                    "output_image_sha256": exporter.sha256_file(image),
                                    "output_label_sha256": exporter.sha256_file(label),
                                }
                            ]
                        },
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "outside images/train"):
                exporter.load_verified_train_samples(manifest, expected_count=1)


class CandidateScreenTest(unittest.TestCase):
    def test_screen_separates_cross_class_and_unmatched(self):
        records = [
            {
                "image": "images/train/sample.jpg",
                "ground_truth": [
                    {"class_id": 1, "bbox_xyxy_normalized": [0.1, 0.1, 0.3, 0.3]}
                ],
                "predictions": [
                    {
                        "class_id": 0,
                        "confidence": 0.9,
                        "bbox_xyxy_normalized": [0.1, 0.1, 0.3, 0.3],
                    },
                    {
                        "class_id": 1,
                        "confidence": 0.8,
                        "bbox_xyxy_normalized": [0.6, 0.6, 0.8, 0.8],
                    },
                    {
                        "class_id": 1,
                        "confidence": 0.7,
                        "bbox_xyxy_normalized": [0.4, 0.4, 0.5, 0.5],
                    },
                ],
            }
        ]
        result = packet.screen_candidates(records)
        self.assertEqual(2, len(result))
        self.assertEqual("HIGH_CONFIDENCE_CROSS_CLASS", result[0]["reason"])
        self.assertEqual("HIGH_CONFIDENCE_UNMATCHED_PREDICTION", result[1]["reason"])

    def test_screen_refuses_non_train_records(self):
        with self.assertRaisesRegex(ValueError, "outside the training split"):
            packet.screen_candidates(
                [{"image": "images/test/x.jpg", "ground_truth": [], "predictions": []}]
            )


if __name__ == "__main__":
    unittest.main()
