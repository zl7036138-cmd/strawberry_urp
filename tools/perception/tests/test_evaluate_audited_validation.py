import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "evaluate_audited_validation.py"
SPEC = importlib.util.spec_from_file_location("evaluate_audited_validation", MODULE_PATH)
audit_eval = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(audit_eval)
materialize = audit_eval.sys.modules["materialize_audited_validation"]


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class EvaluateAuditedValidationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.derivative = self.root / "derivative"
        (self.derivative / "labels" / "val").mkdir(parents=True)
        entries = []
        records = []
        for name, class_id in (("a", 0), ("b", 1)):
            label = self.derivative / "labels" / "val" / f"{name}.txt"
            label.write_text(f"{class_id} 0.5 0.5 0.2 0.2\n", encoding="utf-8")
            entries.append(
                {
                    "image": f"images/val/{name}.jpg",
                    "source_image_sha256": name * 64,
                    "source_label_sha256": str(class_id) * 64,
                    "audited_label": f"labels/val/{name}.txt",
                    "audited_label_sha256": _sha256(label),
                    "audited_label_size_bytes": label.stat().st_size,
                    "class_counts": {"0": int(class_id == 0), "1": int(class_id == 1)},
                    "additions": [],
                }
            )
            records.append(
                {
                    "image": f"images/val/{name}.jpg",
                    "ground_truth": [],
                    "predictions": [
                        {
                            "class_id": class_id,
                            "confidence": 0.9,
                            "bbox_xyxy_normalized": [0.4, 0.4, 0.6, 0.6],
                        }
                    ],
                    "timing_ms": {"inference": 1.0, "total": 2.0},
                }
            )
        derivative_manifest = {
            "kind": "validation_label_audit_derivative",
            "validation_sample_count": 2,
            "test_split_accessed": False,
            "added_instance_counts": {"0": 1, "1": 1},
            "entries": entries,
            "content_binding": {
                "canonical_validation_derivative_sha256": materialize._canonical_sha256(
                    materialize._canonical_rows(entries)
                )
            },
        }
        self.derivative_manifest = self.derivative / "audit_manifest.json"
        self.derivative_manifest.write_text(json.dumps(derivative_manifest), encoding="utf-8")
        self.bundle = self.root / "bundle.json"
        self.bundle.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "kind": "prediction_bundle",
                    "split": "val",
                    "records": records,
                }
            ),
            encoding="utf-8",
        )
        self.experiments = self.root / "experiments.json"
        self.experiments.write_text(
            json.dumps(
                {
                    "metric": {
                        "name": "greedy_per_class_detection_macro_f1_v1",
                        "iou_threshold": 0.5,
                        "class_ids": [0, 1],
                        "macro_f1_min": 0.85,
                        "threshold_grid": {"start": 0.5, "stop": 0.9, "step": 0.1},
                    }
                }
            ),
            encoding="utf-8",
        )
        for record, class_id in zip(records, (0, 1)):
            record["ground_truth"] = [
                {
                    "class_id": class_id,
                    "bbox_xyxy_normalized": [0.4, 0.4, 0.6, 0.6],
                }
            ]
        self.bundle.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "kind": "prediction_bundle",
                    "split": "val",
                    "records": records,
                }
            ),
            encoding="utf-8",
        )
        baseline = {
            "confidence_threshold": 0.9,
            "iou_threshold": 0.5,
            "macro_f1": 1.0,
            "per_class": {
                "0": {
                    "support": 1,
                    "predictions": 1,
                    "tp": 1,
                    "fp": 0,
                    "fn": 0,
                    "precision": 1.0,
                    "recall": 1.0,
                    "f1": 1.0,
                },
                "1": {
                    "support": 1,
                    "predictions": 1,
                    "tp": 1,
                    "fp": 0,
                    "fn": 0,
                    "precision": 1.0,
                    "recall": 1.0,
                    "f1": 1.0,
                },
            },
        }
        self.threshold = self.root / "threshold.json"
        self.threshold.write_text(
            json.dumps(
                {
                    "kind": "frozen_threshold",
                    "metric_name": "greedy_per_class_detection_macro_f1_v1",
                    "class_ids": [0, 1],
                    "iou_threshold": 0.5,
                    "confidence_threshold": 0.9,
                    "bindings": {
                        "validation_bundle_sha256": _sha256(self.bundle),
                        "experiments_sha256": _sha256(self.experiments),
                    },
                    "selection": {
                        "candidate_count": 5,
                        "selected_threshold": 0.9,
                        "validation_metrics": baseline,
                    },
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self):
        self.temp.cleanup()

    def test_same_predictions_are_scored_against_audited_labels(self):
        result = audit_eval.evaluate(
            self.derivative_manifest,
            self.bundle,
            self.threshold,
            self.experiments,
            self.root / "result.json",
        )
        self.assertEqual(result["validation_gate"]["actual"], 1.0)
        self.assertTrue(result["validation_gate"]["passed"])
        self.assertFalse(result["formal_test_authorized"])
        self.assertFalse(result["test_split_accessed"])

    def test_bundle_binding_mismatch_fails_closed(self):
        self.bundle.write_text(self.bundle.read_text() + "\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "differs from the frozen"):
            audit_eval.evaluate(
                self.derivative_manifest,
                self.bundle,
                self.threshold,
                self.experiments,
                self.root / "result.json",
            )

    def test_membership_mismatch_fails_closed(self):
        value = json.loads(self.bundle.read_text())
        value["records"].pop()
        self.bundle.write_text(json.dumps(value), encoding="utf-8")
        threshold = json.loads(self.threshold.read_text())
        threshold["bindings"]["validation_bundle_sha256"] = _sha256(self.bundle)
        self.threshold.write_text(json.dumps(threshold), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "membership mismatch"):
            audit_eval.evaluate(
                self.derivative_manifest,
                self.bundle,
                self.threshold,
                self.experiments,
                self.root / "result.json",
            )

    def test_existing_report_is_never_overwritten(self):
        output = self.root / "result.json"
        output.write_text("keep", encoding="utf-8")
        with self.assertRaises(FileExistsError):
            audit_eval.evaluate(
                self.derivative_manifest,
                self.bundle,
                self.threshold,
                self.experiments,
                output,
            )
        self.assertEqual(output.read_text(), "keep")


if __name__ == "__main__":
    unittest.main()
