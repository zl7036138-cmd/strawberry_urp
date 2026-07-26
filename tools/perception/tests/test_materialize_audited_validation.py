import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "materialize_audited_validation.py"
SPEC = importlib.util.spec_from_file_location("materialize_audited_validation", MODULE_PATH)
audit = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(audit)


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class MaterializeAuditedValidationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.dataset = self.root / "dataset"
        (self.dataset / "images" / "val").mkdir(parents=True)
        (self.dataset / "labels" / "val").mkdir(parents=True)
        self.items = []
        for name in ("a", "b"):
            image = self.dataset / "images" / "val" / f"{name}.jpg"
            label = self.dataset / "labels" / "val" / f"{name}.txt"
            image.write_bytes(f"image-{name}".encode())
            label.write_text("1 0.700000 0.700000 0.100000 0.100000\n", encoding="utf-8")
            self.items.append(
                {
                    "output_image": f"images/val/{name}.jpg",
                    "output_label": f"labels/val/{name}.txt",
                    "output_image_sha256": _sha256(image),
                    "output_label_sha256": _sha256(label),
                    "output_image_size_bytes": image.stat().st_size,
                    "output_label_size_bytes": label.stat().st_size,
                }
            )
        self.manifest = self.dataset / "split_manifest.json"
        self.manifest.write_text(
            json.dumps(
                {
                    "dataset_id": "fixture",
                    "splits": {
                        "train": [{"output_image": "images/train/missing.jpg"}],
                        "val": self.items,
                        "test": [{"output_image": "images/test/sealed-and-missing.jpg"}],
                    },
                    "content_binding": {
                        "canonical_split_sha256": {"val": "a" * 64}
                    },
                }
            ),
            encoding="utf-8",
        )
        self.resolution = self.root / "resolution.json"
        self.approval = self.root / "approval.json"
        self._write_review_files([0.2, 0.2, 0.4, 0.4])

    def tearDown(self):
        self.temp.cleanup()

    def _write_review_files(self, bbox):
        self.resolution.write_text(
            json.dumps(
                {
                    "kind": "validation_label_audit_resolution",
                    "status": "READY_FOR_MANUAL_BOX_ANNOTATION",
                    "labels_modified": False,
                    "test_split_accessed": False,
                    "manual_annotation_worklist": [
                        {
                            "candidate_id": "C001",
                            "image": "images/val/a.jpg",
                            "label": "labels/val/a.txt",
                            "class_name": "ripe",
                            "candidate_box_for_review_only": bbox,
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        self.approval.write_text(
            json.dumps(
                {
                    "kind": "validation_label_audit_human_box_approval",
                    "resolution_receipt": {
                        "sha256": _sha256(self.resolution),
                        "size_bytes": self.resolution.stat().st_size,
                    },
                    "approval_mode": "APPROVE_PROPOSED_BOX_AS_FINAL",
                    "original_labels_modified": False,
                    "test_split_accessed": False,
                    "approved_candidate_ids": ["C001"],
                    "approved_count": 1,
                }
            ),
            encoding="utf-8",
        )

    def test_materializes_only_validation_labels_and_preserves_sources(self):
        original = (self.dataset / "labels" / "val" / "a.txt").read_bytes()
        output = self.root / "audited"
        result = audit.materialize(
            self.resolution, self.approval, self.manifest, output
        )
        self.assertEqual(result["validation_sample_count"], 2)
        self.assertEqual(result["modified_label_count"], 1)
        self.assertEqual(result["added_instance_counts"], {"0": 1, "1": 0})
        self.assertFalse(result["test_split_accessed"])
        self.assertEqual(
            original, (self.dataset / "labels" / "val" / "a.txt").read_bytes()
        )
        audited = (output / "labels" / "val" / "a.txt").read_text()
        self.assertIn("0 0.3000000000 0.3000000000 0.2000000000 0.2000000000", audited)
        self.assertFalse((output / "images").exists())
        self.assertTrue(
            audit.verify_audited_derivative(output / "audit_manifest.json")["valid"]
        )

    def test_approval_must_cover_exact_worklist(self):
        value = json.loads(self.approval.read_text())
        value["approved_candidate_ids"] = []
        self.approval.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "cover exactly"):
            audit.materialize(
                self.resolution, self.approval, self.manifest, self.root / "audited"
            )

    def test_changed_source_label_fails_before_output(self):
        (self.dataset / "labels" / "val" / "a.txt").write_text("changed\n")
        output = self.root / "audited"
        with self.assertRaisesRegex(ValueError, "registered validation file changed"):
            audit.materialize(self.resolution, self.approval, self.manifest, output)
        self.assertFalse(output.exists())

    def test_approved_box_cannot_duplicate_existing_label(self):
        self._write_review_files([0.65, 0.65, 0.75, 0.75])
        with self.assertRaisesRegex(ValueError, "overlaps an existing label"):
            audit.materialize(
                self.resolution, self.approval, self.manifest, self.root / "audited"
            )

    def test_existing_output_is_never_overwritten(self):
        output = self.root / "audited"
        output.mkdir()
        (output / "keep.txt").write_text("keep", encoding="utf-8")
        with self.assertRaises(FileExistsError):
            audit.materialize(self.resolution, self.approval, self.manifest, output)
        self.assertEqual((output / "keep.txt").read_text(), "keep")


if __name__ == "__main__":
    unittest.main()
