from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "materialize_train_label_audit.py"
SPEC = importlib.util.spec_from_file_location("materialize_train_label_audit", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
materializer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(materializer)


class ApplyConfirmedChangesTest(unittest.TestCase):
    def test_relabel_and_add_are_deterministic(self):
        with tempfile.TemporaryDirectory() as temporary:
            label = Path(temporary) / "sample.txt"
            label.write_text("1 0.500000 0.500000 0.200000 0.200000\n", encoding="utf-8")
            text, applied = materializer.apply_confirmed_changes(
                label,
                [
                    {
                        "candidate_id": "T001",
                        "final_decision": "CONFIRMED_RELABEL_RIPE",
                        "bbox_xyxy_normalized": [0.4, 0.4, 0.6, 0.6],
                    },
                    {
                        "candidate_id": "T002",
                        "final_decision": "CONFIRMED_ADD_UNRIPE",
                        "bbox_xyxy_normalized": [0.1, 0.2, 0.3, 0.4],
                    },
                ],
            )
        self.assertEqual(
            "0 0.500000 0.500000 0.200000 0.200000\n"
            "1 0.200000 0.300000 0.200000 0.200000\n",
            text,
        )
        self.assertEqual(["relabel", "add"], [item["operation"] for item in applied])
        self.assertAlmostEqual(1.0, applied[0]["matched_iou"])

    def test_relabel_requires_frozen_iou(self):
        with tempfile.TemporaryDirectory() as temporary:
            label = Path(temporary) / "sample.txt"
            label.write_text("1 0.8 0.8 0.1 0.1\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "below frozen IoU"):
                materializer.apply_confirmed_changes(
                    label,
                    [
                        {
                            "candidate_id": "T001",
                            "final_decision": "CONFIRMED_RELABEL_RIPE",
                            "bbox_xyxy_normalized": [0.1, 0.1, 0.2, 0.2],
                        }
                    ],
                )


if __name__ == "__main__":
    unittest.main()
