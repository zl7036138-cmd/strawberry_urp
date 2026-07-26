import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "finalize_label_audit.py"
SPEC = importlib.util.spec_from_file_location("finalize_label_audit", MODULE_PATH)
finalizer = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(finalizer)


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class FinalizeAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.packet = self.root / "packet"
        self.packet.mkdir()
        manifest = {
            "candidates": [
                {
                    "candidate_id": "C001",
                    "image": "images/val/a.jpg",
                    "label": "labels/val/a.txt",
                    "bbox_xyxy_normalized": [0.1, 0.2, 0.3, 0.4],
                },
                {
                    "candidate_id": "C002",
                    "image": "images/val/b.jpg",
                    "label": "labels/val/b.txt",
                    "bbox_xyxy_normalized": [0.2, 0.3, 0.4, 0.5],
                },
            ]
        }
        manifest_path = self.packet / "candidate_manifest.json"
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        summary = {
            "kind": "validation_label_audit_packet_summary",
            "candidate_count": 2,
            "labels_modified": False,
            "test_split_accessed": False,
            "candidate_manifest": "candidate_manifest.json",
            "candidate_manifest_sha256": _sha256(manifest_path),
        }
        (self.packet / "packet_summary.json").write_text(
            json.dumps(summary), encoding="utf-8"
        )
        self.policy = self.root / "frozen-policy.md"
        self.policy.write_text("frozen conservative policy", encoding="utf-8")

    def tearDown(self):
        self.temp.cleanup()

    def _review(self, name, reviewer, decisions):
        path = self.root / name
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=["candidate_id", "reviewer", "decision", "confidence", "notes"],
            )
            writer.writeheader()
            for candidate_id, decision in zip(("C001", "C002"), decisions):
                writer.writerow(
                    {
                        "candidate_id": candidate_id,
                        "reviewer": reviewer,
                        "decision": decision,
                        "confidence": "HIGH",
                        "notes": "reviewed",
                    }
                )
        return path

    def _adjudication(self, candidate_id="C001"):
        path = self.root / "adjudication.csv"
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=["candidate_id", "adjudicator", "final_decision", "notes"],
            )
            writer.writeheader()
            writer.writerow(
                {
                    "candidate_id": candidate_id,
                    "adjudicator": "reviewer-c",
                    "final_decision": "CONFIRMED_MISSING_RIPE",
                    "notes": "resolved",
                }
            )
        return path

    def test_consensus_emits_manual_annotation_worklist_without_modifying_labels(self):
        review_1 = self._review(
            "r1.csv", "reviewer-a", ["CONFIRMED_MISSING_RIPE", "MODEL_FALSE_POSITIVE"]
        )
        review_2 = self._review(
            "r2.csv", "reviewer-b", ["CONFIRMED_MISSING_RIPE", "MODEL_FALSE_POSITIVE"]
        )
        result = finalizer.finalize_audit(
            self.packet, review_1, review_2, self.root / "receipt.json"
        )
        self.assertEqual(result["status"], "READY_FOR_MANUAL_BOX_ANNOTATION")
        self.assertEqual(len(result["manual_annotation_worklist"]), 1)
        self.assertFalse(result["labels_modified"])
        self.assertFalse(result["corrected_metric_claimed"])

    def test_disagreement_requires_adjudication(self):
        review_1 = self._review(
            "r1.csv", "reviewer-a", ["CONFIRMED_MISSING_RIPE", "MODEL_FALSE_POSITIVE"]
        )
        review_2 = self._review(
            "r2.csv", "reviewer-b", ["MODEL_FALSE_POSITIVE", "MODEL_FALSE_POSITIVE"]
        )
        with self.assertRaisesRegex(ValueError, "adjudication required.*C001"):
            finalizer.finalize_audit(
                self.packet, review_1, review_2, self.root / "receipt.json"
            )

    def test_independent_adjudicator_resolves_exact_disagreement_set(self):
        review_1 = self._review(
            "r1.csv", "reviewer-a", ["CONFIRMED_MISSING_RIPE", "MODEL_FALSE_POSITIVE"]
        )
        review_2 = self._review(
            "r2.csv", "reviewer-b", ["MODEL_FALSE_POSITIVE", "MODEL_FALSE_POSITIVE"]
        )
        result = finalizer.finalize_audit(
            self.packet,
            review_1,
            review_2,
            self.root / "receipt.json",
            self._adjudication(),
        )
        self.assertEqual(result["disagreement_count"], 1)
        self.assertEqual(result["resolutions"][0]["resolution"], "ADJUDICATED")

    def test_conservative_policy_excludes_every_disagreement(self):
        review_1 = self._review(
            "r1.csv", "reviewer-a", ["CONFIRMED_MISSING_RIPE", "MODEL_FALSE_POSITIVE"]
        )
        review_2 = self._review(
            "r2.csv", "reviewer-b", ["CONFIRMED_MISSING_UNRIPE", "MODEL_FALSE_POSITIVE"]
        )
        result = finalizer.finalize_audit(
            self.packet,
            review_1,
            review_2,
            self.root / "receipt.json",
            disagreement_policy="conservative_exclude",
            policy_record_path=self.policy,
        )
        self.assertEqual(result["disagreement_policy"], "conservative_exclude")
        self.assertEqual(result["decision_counts"]["AMBIGUOUS_EXCLUDE"], 1)
        self.assertEqual(
            result["resolutions"][0]["resolution"], "CONSERVATIVE_EXCLUSION"
        )
        self.assertEqual(result["manual_annotation_worklist"], [])

    def test_conservative_policy_rejects_an_adjudication_file(self):
        review_1 = self._review(
            "r1.csv", "reviewer-a", ["CONFIRMED_MISSING_RIPE", "MODEL_FALSE_POSITIVE"]
        )
        review_2 = self._review(
            "r2.csv", "reviewer-b", ["MODEL_FALSE_POSITIVE", "MODEL_FALSE_POSITIVE"]
        )
        with self.assertRaisesRegex(ValueError, "cannot be combined"):
            finalizer.finalize_audit(
                self.packet,
                review_1,
                review_2,
                self.root / "receipt.json",
                self._adjudication(),
                disagreement_policy="conservative_exclude",
                policy_record_path=self.policy,
            )

    def test_conservative_policy_requires_a_frozen_record(self):
        review_1 = self._review(
            "r1.csv", "reviewer-a", ["CONFIRMED_MISSING_RIPE", "MODEL_FALSE_POSITIVE"]
        )
        review_2 = self._review(
            "r2.csv", "reviewer-b", ["MODEL_FALSE_POSITIVE", "MODEL_FALSE_POSITIVE"]
        )
        with self.assertRaisesRegex(ValueError, "requires an existing frozen policy"):
            finalizer.finalize_audit(
                self.packet,
                review_1,
                review_2,
                self.root / "receipt.json",
                disagreement_policy="conservative_exclude",
            )

    def test_same_reviewer_identity_is_rejected(self):
        decisions = ["MODEL_FALSE_POSITIVE", "MODEL_FALSE_POSITIVE"]
        review_1 = self._review("r1.csv", "reviewer-a", decisions)
        review_2 = self._review("r2.csv", "reviewer-a", decisions)
        with self.assertRaisesRegex(ValueError, "identities must be different"):
            finalizer.finalize_audit(
                self.packet, review_1, review_2, self.root / "receipt.json"
            )

    def test_unknown_decision_and_overwrite_fail_closed(self):
        review_1 = self._review("r1.csv", "reviewer-a", ["NOT_VALID", "MODEL_FALSE_POSITIVE"])
        review_2 = self._review(
            "r2.csv", "reviewer-b", ["MODEL_FALSE_POSITIVE", "MODEL_FALSE_POSITIVE"]
        )
        with self.assertRaisesRegex(ValueError, "invalid decision"):
            finalizer.finalize_audit(
                self.packet, review_1, review_2, self.root / "receipt.json"
            )

        review_1 = self._review(
            "r1-valid.csv", "reviewer-a", ["MODEL_FALSE_POSITIVE", "MODEL_FALSE_POSITIVE"]
        )
        output = self.root / "existing.json"
        output.write_text("do not overwrite", encoding="utf-8")
        with self.assertRaises(FileExistsError):
            finalizer.finalize_audit(self.packet, review_1, review_2, output)


if __name__ == "__main__":
    unittest.main()
