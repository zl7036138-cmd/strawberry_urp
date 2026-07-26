import json
from pathlib import Path
import tempfile
import unittest

from strawberry_benchmark.perception_dev_gate import (
    load_perception_repeated_dev_gate,
    verify_perception_repeated_dev_gate,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
MANIFEST_PATH = (
    REPOSITORY_ROOT / "config" / "p3_perception_repeated_dev_gate_v1.json"
)


class PerceptionRepeatedDevGateTests(unittest.TestCase):
    def test_frozen_gate_is_non_formal_and_balanced(self):
        contract = load_perception_repeated_dev_gate(MANIFEST_PATH)
        self.assertEqual(10, contract.positive_trials)
        self.assertEqual(10, contract.negative_trials)
        self.assertEqual("RIPE", contract.positive_scene.target_maturity)
        self.assertEqual("UNRIPE", contract.negative_scene.target_maturity)
        self.assertEqual(
            contract.positive_scene.target.position_m,
            contract.negative_scene.target.position_m,
        )

    def test_bound_waiver_and_model_evidence_verify(self):
        contract = load_perception_repeated_dev_gate(MANIFEST_PATH)
        waiver, verified = verify_perception_repeated_dev_gate(
            contract, REPOSITORY_ROOT
        )
        self.assertEqual("ACCEPTED_WITH_WAIVER", waiver.engineering_status)
        self.assertFalse(verified["held_out_receipt"].exists())

    def test_rejects_formal_acceptance(self):
        raw = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        raw["formal_acceptance"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "formal_acceptance"):
                load_perception_repeated_dev_gate(path)

    def test_rejects_relaxed_negative_no_pick_rate(self):
        raw = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        raw["acceptance"]["minimum_negative_no_pick_rate"] = 0.9
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "success/no-pick"):
                load_perception_repeated_dev_gate(path)

    def test_rejects_duplicate_scene_identity(self):
        raw = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        raw["negative_scene"]["parked_models"][0]["target_id"] = 2
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "must be unique"):
                load_perception_repeated_dev_gate(path)


if __name__ == "__main__":
    unittest.main()
