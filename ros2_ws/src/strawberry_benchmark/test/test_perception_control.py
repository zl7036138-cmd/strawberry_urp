import json
from pathlib import Path
import tempfile
import unittest

from strawberry_benchmark.perception_control import (
    load_perception_control_waiver,
    verify_perception_control_waiver,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
MANIFEST_PATH = REPOSITORY_ROOT / "config" / "p3_perception_control_waiver_v1.json"


class PerceptionControlWaiverTests(unittest.TestCase):
    def test_frozen_manifest_authorizes_only_bounded_perception_control(self):
        waiver = load_perception_control_waiver(MANIFEST_PATH)
        self.assertEqual("ACCEPTED_WITH_WAIVER", waiver.engineering_status)
        self.assertLess(waiver.macro_f1, waiver.required_macro_f1)
        self.assertEqual("/strawberry/target_pose", waiver.control_target_topic)
        self.assertEqual(1, waiver.target.target_id)
        self.assertEqual({2, 3}, {item.target_id for item in waiver.parked_models})

    def test_rejects_rewriting_numeric_gate_as_passed(self):
        raw = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        raw["validation_result"]["numeric_gate_passed"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "numeric gate"):
                load_perception_control_waiver(path)

    def test_rejects_oracle_provider(self):
        raw = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        raw["runtime_routing"]["start_oracle_provider"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Oracle provider"):
                load_perception_control_waiver(path)

    def test_rejects_formal_matrix_authorization(self):
        raw = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        raw["authorization"]["formal_135_plus_30_matrix_authorized"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "formal_135_plus_30"):
                load_perception_control_waiver(path)

    def test_model_verification_fails_closed_on_wrong_bytes(self):
        waiver = load_perception_control_waiver(MANIFEST_PATH)
        with tempfile.TemporaryDirectory() as directory:
            wrong = Path(directory) / "best.pt"
            wrong.write_bytes(b"not the frozen model")
            with self.assertRaisesRegex(ValueError, "size differs"):
                verify_perception_control_waiver(
                    waiver, REPOSITORY_ROOT, model_path=wrong
                )


if __name__ == "__main__":
    unittest.main()
