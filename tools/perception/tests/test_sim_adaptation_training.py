import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


TOOLS = Path(__file__).resolve().parents[1]
ROOT = TOOLS.parents[1]
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import materialize_sim_adaptation_training as materializer
import run_sim_adaptation_training as runner
import evaluate_sim_adaptation_synthetic as synthetic_evaluator


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class SimulatorAdaptationTrainingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract_path = ROOT / "tools/perception/sim_adaptation_v1_contract.json"
        cls.contract = json.loads(cls.contract_path.read_text(encoding="utf-8"))

    def test_contract_freezes_exact_sources_and_safety_boundary(self):
        contract = self.contract
        self.assertEqual("FROZEN_TRAINING_AUTHORIZED", contract["status"])
        self.assertEqual({"real": 501, "synthetic": 216}, contract["dataset_derivative"]["train_source_counts"])
        self.assertEqual(72, contract["dataset_derivative"]["validation_image_count"])
        self.assertFalse(contract["dataset_derivative"]["test_split_present"])
        self.assertFalse(contract["audited_real_validation"]["training_access"])
        self.assertFalse(contract["audited_real_validation"]["checkpoint_selection_access"])
        self.assertFalse(contract["audited_real_validation"]["threshold_selection_access"])
        self.assertEqual(1, contract["training"]["maximum_claims"])
        self.assertFalse(contract["training"]["retry_authorized"])
        self.assertTrue(all(value is False for value in contract["safety"].values() if isinstance(value, bool)))

    def test_authorization_binds_contract_decision_and_config(self):
        path = ROOT / self.contract["authorization_receipt"]
        authorization = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual("好的，按照你的计划继续进行", authorization["user_message_exact"])
        for key in ("contract", "decision_record", "training_config"):
            binding = authorization[key]
            target = ROOT / binding["path"]
            self.assertEqual(target.stat().st_size, binding["size_bytes"])
            self.assertEqual(sha(target), binding["sha256"])
        self.assertFalse(authorization["retry_or_parameter_search_authorized"])
        self.assertFalse(authorization["formal_real_test_authorized"])

    def test_training_config_matches_every_frozen_hyperparameter(self):
        config_path = ROOT / self.contract["training"]["config"]
        actual = runner._validate_config(self.contract, config_path)
        self.assertEqual("30", actual["epochs"])
        self.assertEqual("0.001", actual["lr0"])
        self.assertEqual(self.contract["base_weight"]["path"], actual["model"])
        with tempfile.TemporaryDirectory() as directory:
            changed = Path(directory) / "changed.yaml"
            changed.write_text(config_path.read_text(encoding="utf-8").replace("epochs: 30", "epochs: 31"), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "epochs"):
                runner._validate_config(self.contract, changed)

    def test_label_parser_and_registered_path_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            label = Path(directory) / "label.txt"
            label.write_text("0 0.5 0.5 0.2 0.2\n1 0.4 0.4 0.1 0.1\n", encoding="utf-8")
            self.assertEqual({0: 1, 1: 1}, dict(materializer._count_classes(label)))
            with self.assertRaisesRegex(ValueError, "unsafe"):
                materializer._safe_registered_path("images/test/sealed.jpg", "images", "train")
            label.write_text("1 0.5 0.5 0.0 0.1\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-positive"):
                materializer._count_classes(label)

    def test_materialized_manifest_has_frozen_counts_and_canonical_digest(self):
        output = ROOT / self.contract["dataset_derivative"]["output_dir"]
        manifest = json.loads((output / "training_manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(717, manifest["train_image_count"])
        self.assertEqual(72, manifest["validation_image_count"])
        self.assertFalse(manifest["audited_real_validation_copied"])
        self.assertFalse(manifest["formal_real_test_accessed"])
        self.assertFalse((output / "images/test").exists())
        self.assertEqual(717, len(list((output / "images/train").iterdir())))
        self.assertEqual(72, len(list((output / "images/val").iterdir())))
        canonical = materializer._canonical_sha256(materializer._canonical_rows(manifest["entries"]))
        self.assertEqual("aa96dbfdd5bb7b74a52b9857ee114536b30e65584feb80a34428771554928582", canonical)
        self.assertEqual(canonical, manifest["content_binding"]["canonical_training_derivative_sha256"])

    def test_checkpoint_inventory_rejects_manual_weight(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "best.pt").write_bytes(b"best")
            (root / "last.pt").write_bytes(b"last")
            (root / "manual.pt").write_bytes(b"manual")
            with self.assertRaisesRegex(ValueError, "uncontracted"):
                runner._checkpoint_inventory(root)

    def test_training_environment_requires_frozen_packages(self):
        environment = {
            "packages": {
                "numpy": "1",
                "opencv-python": "1",
                "torch": None,
                "torchvision": "1",
                "ultralytics": "1",
            },
            "nvidia_smi": ["NVIDIA GeForce RTX 4060 Laptop GPU"],
        }
        with mock.patch.object(runner.sys, "prefix", "/opt/strawberry_venv"):
            with self.assertRaisesRegex(ValueError, "torch"):
                runner._validate_training_environment(environment, "/opt/strawberry_venv/bin/yolo")

    def test_frozen_preflight_rejects_binding_change(self):
        preview = {
            "kind": "simulator_adaptation_training_preflight",
            "variant": "yolo11s_640_sim_adapt_v1",
            "training_preflight_passed": True,
            "training_unlocked": True,
            "training_started": False,
            "claim_consumed": False,
            "bindings": {"contract": "abc"},
            "bindings_canonical_sha256": "digest",
        }
        with tempfile.TemporaryDirectory() as directory:
            receipt = Path(directory) / "receipt.json"
            receipt.write_text(json.dumps(preview), encoding="utf-8")
            runner._validate_frozen_preflight(preview, receipt)
            changed = dict(preview)
            changed["bindings"] = {"contract": "changed"}
            with self.assertRaisesRegex(ValueError, "files changed"):
                runner._validate_frozen_preflight(changed, receipt)

    def test_completed_claim_preserves_exact_checkpoint_inventory(self):
        claim = json.loads((ROOT / self.contract["training"]["claim"]).read_text(encoding="utf-8"))
        self.assertEqual("completed", claim["status"])
        self.assertEqual("simulator_adaptation_training:yolo11s_640_sim_adapt_v1", claim["purpose"])
        self.assertEqual(8, len(claim["details"]["checkpoint_inventory"]))
        self.assertFalse(claim["details"]["formal_real_test_accessed"])
        self.assertFalse(claim["details"]["formal_simulator_matrix_started"])

    def test_synthetic_winner_uses_frozen_tie_breaks(self):
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
            item("epoch25.pt", 1.0, 1.0, 1.0, 0.76),
            item("best.pt", 1.0, 1.0, 1.0, 0.80),
            item("last.pt", 1.0, 1.0, 1.0, 0.77),
        ]
        self.assertEqual("best.pt", synthetic_evaluator.select_winner(candidates)["checkpoint_name"])

    def test_synthetic_selection_passes_without_real_access(self):
        path = ROOT / "artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_synthetic_heldout_v1/summary.json"
        summary = json.loads(path.read_text(encoding="utf-8"))
        self.assertTrue(summary["synthetic_gate_passed"])
        self.assertEqual("best.pt", summary["selected_checkpoint"]["checkpoint_name"])
        self.assertEqual(0.8, summary["selected_checkpoint"]["selected_threshold"])
        self.assertEqual(1.0, summary["selected_checkpoint"]["validation_metrics"]["macro_f1"])
        self.assertFalse(summary["audited_real_validation_accessed"])
        self.assertFalse(summary["formal_real_test_accessed"])

    def test_real_failure_and_counterfactual_cannot_promote(self):
        real_path = ROOT / "artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_real_nonregression_v1/summary.json"
        analysis_path = ROOT / "artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_failure_analysis_v1.json"
        real = json.loads(real_path.read_text(encoding="utf-8"))
        analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
        self.assertFalse(real["real_nonregression_passed"])
        self.assertFalse(real["threshold_search_performed"])
        self.assertEqual(0.8, real["confidence_threshold"])
        self.assertFalse(analysis["frozen_decision"]["candidate_promoted"])
        self.assertFalse(analysis["frozen_decision"]["retry_authorized"])
        self.assertEqual("none", analysis["diagnostic_counterfactual_only"]["decision_effect"])
        self.assertFalse(analysis["diagnostic_counterfactual_only"]["would_pass_individual_minimums"]["unripe_f1"])


if __name__ == "__main__":
    unittest.main()
