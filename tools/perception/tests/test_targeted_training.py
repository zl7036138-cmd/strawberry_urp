import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


TOOLS = Path(__file__).resolve().parents[1]
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import evaluate_targeted_training as evaluator
import materialize_audited_validation as audited
import materialize_unripe_exposure_training as materializer
import run_targeted_training as runner


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def binding(root, path):
    value = root / path
    return {"path": path, "size_bytes": value.stat().st_size, "sha256": sha(value)}


class TargetedTrainingCoreTests(unittest.TestCase):
    def test_class_parser_and_path_guard_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            label = Path(directory) / "label.txt"
            label.write_text(
                "0 0.5 0.5 0.2 0.2\n1 0.3 0.3 0.1 0.1\n1 0.7 0.7 0.1 0.1\n",
                encoding="utf-8",
            )
            self.assertEqual({0: 1, 1: 2}, dict(materializer._count_classes(label)))
            with self.assertRaisesRegex(ValueError, "unsafe"):
                materializer._safe_split_path("images/test/sealed.jpg", "images", "train")
            label.write_text("1 0.5 0.5 0.0 0.1\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-positive"):
                materializer._count_classes(label)

    def test_checkpoint_winner_uses_frozen_tie_breaks(self):
        def item(name, macro, ripe, unripe, threshold):
            return {
                "checkpoint_name": name,
                "validation_metrics": {
                    "macro_f1": macro,
                    "confidence_threshold": threshold,
                    "per_class": {"0": {"f1": ripe}, "1": {"f1": unripe}},
                },
            }

        candidates = [
            item("epoch20.pt", 0.86, 0.91, 0.81, 0.4),
            item("best.pt", 0.86, 0.91, 0.81, 0.4),
            item("epoch10.pt", 0.86, 0.90, 0.80, 0.4),
        ]
        self.assertEqual("best.pt", evaluator.select_winner(candidates)["checkpoint_name"])

    def test_checkpoint_inventory_rejects_uncontracted_weight(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "best.pt").write_bytes(b"best")
            (root / "last.pt").write_bytes(b"last")
            (root / "manual.pt").write_bytes(b"manual")
            with self.assertRaisesRegex(ValueError, "uncontracted"):
                runner._checkpoint_inventory(root)

    def test_materializes_exact_unripe_duplicate_and_no_test(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            dataset = root / "data/original"
            audited_root = root / "data/audited"
            for base in (dataset, audited_root):
                (base / "labels/val").mkdir(parents=True)
            (dataset / "images/train").mkdir(parents=True)
            (dataset / "labels/train").mkdir(parents=True)
            (dataset / "images/val").mkdir(parents=True)

            train_items = []
            for name, label_text in (
                ("ripe", "0 0.5 0.5 0.2 0.2\n"),
                ("mixed", "0 0.5 0.5 0.2 0.2\n1 0.3 0.3 0.1 0.1\n"),
            ):
                image = dataset / f"images/train/{name}.jpg"
                label = dataset / f"labels/train/{name}.txt"
                image.write_bytes(f"image-{name}".encode())
                label.write_text(label_text, encoding="utf-8")
                counts = materializer._count_classes(label)
                train_items.append(
                    {
                        "output_image": f"images/train/{name}.jpg",
                        "output_label": f"labels/train/{name}.txt",
                        "output_image_sha256": sha(image),
                        "output_label_sha256": sha(label),
                        "output_image_size_bytes": image.stat().st_size,
                        "output_label_size_bytes": label.stat().st_size,
                        "v1_class_counts": {"0": counts[0], "1": counts[1]},
                    }
                )
            val_image = dataset / "images/val/val.jpg"
            val_image.write_bytes(b"validation-image")
            val_label = audited_root / "labels/val/val.txt"
            val_label.write_text("1 0.5 0.5 0.2 0.2\n", encoding="utf-8")
            split = dataset / "split_manifest.json"
            split.write_text(
                json.dumps(
                    {
                        "splits": {"train": train_items, "val": [], "test": [{"sealed": True}]},
                        "content_binding": {"canonical_split_sha256": {"train": "a" * 64}},
                    }
                ),
                encoding="utf-8",
            )

            audit_entry = {
                "image": "images/val/val.jpg",
                "source_image_sha256": sha(val_image),
                "source_image_size_bytes": val_image.stat().st_size,
                "source_label": "labels/val/val.txt",
                "source_label_sha256": sha(val_label),
                "source_label_size_bytes": val_label.stat().st_size,
                "audited_label": "labels/val/val.txt",
                "audited_label_sha256": sha(val_label),
                "audited_label_size_bytes": val_label.stat().st_size,
                "class_counts": {"0": 0, "1": 1},
                "additions": [],
            }
            audit_manifest = audited_root / "audit_manifest.json"
            audit_manifest.write_text(
                json.dumps(
                    {
                        "kind": "validation_label_audit_derivative",
                        "validation_sample_count": 1,
                        "entries": [audit_entry],
                        "content_binding": {
                            "canonical_validation_derivative_sha256": audited._canonical_sha256(
                                audited._canonical_rows([audit_entry])
                            )
                        },
                        "test_split_accessed": False,
                    }
                ),
                encoding="utf-8",
            )

            for path, content in (
                ("artifacts/handoff.json", "{}"),
                ("docs/adr.md", "decision"),
                ("config/train.yaml", "model: base.pt\n"),
            ):
                target = root / path
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(content, encoding="utf-8")

            contract_path = root / "contract.json"
            output = "data/derivative"
            contract = {
                "kind": "targeted_unripe_exposure_training_contract",
                "status": "AUTHORIZED",
                "variant": "fixture",
                "decision_record": "docs/adr.md",
                "authorization_receipt": "artifacts/auth.json",
                "upstream_handoff": binding(root, "artifacts/handoff.json"),
                "source_split_manifest": {
                    **binding(root, "data/original/split_manifest.json"),
                    "canonical_train_sha256": "a" * 64,
                },
                "audited_validation_manifest": {
                    **binding(root, "data/audited/audit_manifest.json"),
                    "canonical_sha256": audited._canonical_sha256(audited._canonical_rows([audit_entry])),
                },
                "dataset_derivative": {
                    "output_dir": output,
                    "manifest": f"{output}/training_manifest.json",
                    "dataset_yaml": f"{output}/dataset.yaml",
                    "test_split_present": False,
                    "original_train_image_count": 2,
                    "unripe_containing_image_count": 1,
                    "effective_train_image_count": 3,
                    "validation_image_count": 1,
                    "oversample_rule": "fixture",
                    "original_train_class_counts": {"0": 2, "1": 1},
                    "effective_train_class_counts": {"0": 3, "1": 2},
                },
                "training": {"config": "config/train.yaml"},
                "formal_test": {"access_authorized": False},
            }
            contract_path.write_text(json.dumps(contract), encoding="utf-8")
            authorization = {
                "kind": "targeted_training_user_authorization",
                "status": "AUTHORIZED",
                "new_training_authorized": True,
                "formal_test_authorized": False,
                "test_split_access_authorized": False,
                "contract": binding(root, "contract.json"),
                "decision_record": binding(root, "docs/adr.md"),
                "training_config": binding(root, "config/train.yaml"),
            }
            (root / "artifacts/auth.json").write_text(json.dumps(authorization), encoding="utf-8")

            with mock.patch.object(materializer, "REPOSITORY_ROOT", root):
                manifest = materializer.materialize(contract_path)
                verified = materializer.verify_training_derivative(
                    root / output / "training_manifest.json", contract_path
                )
            self.assertEqual(4, verified["entry_count"])
            self.assertEqual(3, manifest["counts"]["effective_train_images"])
            self.assertTrue((root / output / "images/train/unripe_x2__mixed.jpg").is_file())
            self.assertFalse((root / output / "images/test").exists())
            self.assertNotIn("test:", (root / output / "dataset.yaml").read_text())


if __name__ == "__main__":
    unittest.main()
