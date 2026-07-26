from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "finalize_train_label_audit.py"
SPEC = importlib.util.spec_from_file_location("finalize_train_label_audit", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


class FinalizeTrainLabelAuditTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.packet = self.root / "packet"
        self.packet.mkdir()
        self.policy = self.root / "policy.md"
        self.policy.write_text("frozen conservative policy", encoding="utf-8")
        candidates = []
        for index, (reason, predicted) in enumerate(
            [
                ("HIGH_CONFIDENCE_CROSS_CLASS", "ripe"),
                ("HIGH_CONFIDENCE_UNMATCHED_PREDICTION", "unripe"),
            ],
            1,
        ):
            candidates.append(
                {
                    "candidate_id": f"T{index:03d}",
                    "reason": reason,
                    "image": f"images/train/{index}.jpg",
                    "label": f"labels/train/{index}.txt",
                    "predicted_class_id": 0 if predicted == "ripe" else 1,
                    "predicted_class_name": predicted,
                    "confidence": 0.8,
                    "bbox_xyxy_normalized": [0.1, 0.2, 0.3, 0.4],
                }
            )
        self.manifest = {
            "kind": "training_label_audit_packet",
            "scope": {
                "split": "train",
                "validation_split_accessed": False,
                "held_out_test_accessed": False,
                "training_started": False,
                "labels_modified": False,
            },
            "candidates": candidates * 17,
        }
        for index, item in enumerate(self.manifest["candidates"], 1):
            item = dict(item)
            item["candidate_id"] = f"T{index:03d}"
            item["image"] = f"images/train/{index}.jpg"
            item["label"] = f"labels/train/{index}.txt"
            self.manifest["candidates"][index - 1] = item
        manifest_path = self.packet / "candidate_manifest.json"
        manifest_path.write_text(json.dumps(self.manifest), encoding="utf-8")
        (self.packet / "packet_summary.json").write_text(
            json.dumps(
                {
                    "kind": "training_label_audit_packet_summary",
                    "candidate_manifest": {
                        "path": "candidate_manifest.json",
                        "sha256": audit._sha256(manifest_path),
                    },
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self):
        self.temporary.cleanup()

    def _review(self, name: str, decisions: list[str]) -> Path:
        path = self.root / name
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=audit.FIELDS)
            writer.writeheader()
            for candidate, decision in zip(self.manifest["candidates"], decisions):
                writer.writerow(
                    {
                        **dict(zip(audit.FIELDS[:6], audit._expected_source(candidate))),
                        "decision": decision,
                        "notes": "",
                    }
                )
        return path

    def test_disagreement_is_conservatively_excluded(self):
        first = ["CONFIRMED_RELABEL_RIPE", "CONFIRMED_ADD_UNRIPE"] * 17
        second = list(first)
        second[0] = "AMBIGUOUS_EXCLUDE"
        output = self.root / "resolution.json"
        result = audit.finalize(
            self.packet,
            self._review("r1.csv", first),
            self._review("r2.csv", second),
            self.policy,
            output,
        )
        self.assertEqual(1, result["disagreement_count"])
        self.assertEqual("AMBIGUOUS_EXCLUDE", result["resolutions"][0]["final_decision"])
        self.assertFalse(result["labels_modified"])

    def test_semantic_no_op_is_rejected(self):
        decisions = ["CONFIRMED_RELABEL_UNRIPE", "CONFIRMED_ADD_UNRIPE"] * 17
        with self.assertRaisesRegex(ValueError, "semantic no-op"):
            audit.finalize(
                self.packet,
                self._review("r1.csv", decisions),
                self._review("r2.csv", decisions),
                self.policy,
                self.root / "resolution.json",
            )


if __name__ == "__main__":
    unittest.main()
