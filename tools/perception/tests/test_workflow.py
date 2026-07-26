from __future__ import annotations

import copy
import hashlib
import json
from argparse import Namespace
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


TOOLS_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(TOOLS_ROOT))

from workflow import (  # noqa: E402
    FORMAL_NMS_IOU,
    FORMAL_VALIDATION_BINDING_FIELDS,
    METRIC_NAME,
    GroundTruth,
    ImageDetections,
    Prediction,
    acquire_one_time_claim,
    evaluate_detection_metrics,
    finalize_one_time_claim,
    intersection_over_union,
    load_experiment,
    load_prediction_bundle,
    load_split_samples,
    load_yolo_ground_truth,
    optimization_promotion_artifact_path,
    read_json,
    record_to_json,
    select_validation_threshold,
    sha256_file,
    summarize_speed,
    threshold_grid,
    validate_completed_training_receipt,
    validate_formal_test_contract,
    validate_formal_validation_bundle,
    validate_frozen_threshold,
    validate_optimization_promotion,
    validate_split_contract,
    validate_validation_records_against_split,
    verify_file_contract,
    write_json_exclusive,
)
from run_training import _resolve_yolo_executable, _training_command  # noqa: E402
from run_inference import (  # noqa: E402
    DEFAULT_EXPERIMENTS,
    _clip_normalized_bbox,
    _dataset_bindings,
    _formal_cli_contract,
    _replay_formal_validation_before_test_claim,
    _validate_formal_threshold_source,
)
import freeze_threshold as freeze_threshold_module  # noqa: E402
import run_inference as run_inference_module  # noqa: E402


RIPE_BOX = (0.1, 0.1, 0.3, 0.3)
UNRIPE_BOX = (0.6, 0.6, 0.9, 0.9)


class PredictionBoxAdapterTest(unittest.TestCase):
    def test_normalized_prediction_box_is_clipped_to_image_bounds(self):
        self.assertEqual(
            (0.0, 0.2, 0.3, 1.0),
            _clip_normalized_bbox((-1e-7, 0.2, 0.3, 1.0000001)),
        )

    def test_box_with_no_area_after_clipping_is_discarded(self):
        self.assertIsNone(_clip_normalized_bbox((1.0, 0.2, 1.000001, 0.8)))
        self.assertIsNone(_clip_normalized_bbox((-0.2, 0.2, -0.1, 0.8)))

    def test_non_finite_or_malformed_box_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "four coordinates"):
            _clip_normalized_bbox((0.1, 0.2, 0.3))
        with self.assertRaisesRegex(ValueError, "non-finite"):
            _clip_normalized_bbox((0.1, 0.2, float("nan"), 0.4))


def make_split_contract_fixture(root: Path):
    artifact_paths = {
        "groups_csv": "data/groups.csv",
        "group_overrides_csv": "data/group_overrides.csv",
        "exclusions_csv": "data/exclusions.csv",
        "group_audit": "artifacts/group_audit.json",
        "curation_audit": "artifacts/curation_audit.json",
        "split_manifest": "processed/split_manifest.json",
        "dataset_yaml": "processed/dataset.yaml",
    }
    artifacts = {}
    for index, (name, relative) in enumerate(artifact_paths.items()):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"{name}:{index}".encode("utf-8"))
        artifacts[name] = {"path": relative, "sha256": sha256_file(path)}
    validation_items = []
    for index in range(116):
        image_relative = f"images/val/sample_{index:03d}.jpg"
        label_relative = f"labels/val/sample_{index:03d}.txt"
        image_path = root / "processed" / image_relative
        label_path = root / "processed" / label_relative
        image_path.parent.mkdir(parents=True, exist_ok=True)
        label_path.parent.mkdir(parents=True, exist_ok=True)
        image_path.write_bytes(f"fixture-image-{index}".encode("utf-8"))
        label_path.write_text(
            "0 0.2 0.2 0.2 0.2\n1 0.75 0.75 0.3 0.3\n",
            encoding="utf-8",
        )
        validation_items.append(
            {"output_image": image_relative, "output_label": label_relative}
        )
    split_manifest_path = root / artifact_paths["split_manifest"]
    split_manifest_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "splits": {"train": [], "val": validation_items, "test": []},
            }
        ),
        encoding="utf-8",
    )
    artifacts["split_manifest"]["sha256"] = sha256_file(split_manifest_path)
    split_hashes = {"train": "1" * 64, "val": "2" * 64, "test": "3" * 64}
    contract = {
        "schema_version": 1,
        "strategy": "seeded_group_stratified_full_resplit",
        "seed": 20260710,
        "ratios": {"train": 0.70, "val": 0.15, "test": 0.15},
        "counts": {"train": 501, "val": 116, "test": 115},
        "source_image_count": 813,
        "excluded_count": 81,
        "retained_count": 732,
        "canonical_dataset_sha256": "4" * 64,
        "canonical_split_sha256": split_hashes,
        "artifacts": artifacts,
    }
    spec = {
        "schema_version": 1,
        "dataset_id": "zenodo_6126677",
        "dataset_yaml": artifact_paths["dataset_yaml"],
        "split_manifest": artifact_paths["split_manifest"],
        "split_contract": contract,
        "formal_test": {
            "variant": "yolo11s_640",
            "receipt": "artifacts/test_receipt.json",
            "task": "detect",
            "imgsz": 640,
        },
        "metric": {
            "name": METRIC_NAME,
            "iou_threshold": 0.5,
            "macro_f1_min": 0.85,
            "class_ids": [0, 1],
        },
        "variants": {
            "yolo11s_640": {
                "role": "primary",
                "training_config": "config/train.yaml",
                "validation_config": "config/eval.yaml",
                "trained_weight": "outputs/best.pt",
                "training_receipt": "artifacts/training.json",
            }
        },
    }
    report = {
        "valid": True,
        "dataset_id": "zenodo_6126677",
        "strategy": contract["strategy"],
        "seed": contract["seed"],
        "split_sample_count": dict(contract["counts"]),
        "sample_count": contract["retained_count"],
        "source_image_count": contract["source_image_count"],
        "excluded_count": contract["excluded_count"],
        "retained_count": contract["retained_count"],
        "canonical_dataset_sha256": contract["canonical_dataset_sha256"],
        "canonical_split_sha256": dict(split_hashes),
        "dataset_yaml_sha256": artifacts["dataset_yaml"]["sha256"],
        "exclusions_csv": artifacts["exclusions_csv"]["path"],
        "exclusions_csv_sha256": artifacts["exclusions_csv"]["sha256"],
        "verified_file_count": 1465,
    }
    validation_config = root / "config" / "eval.yaml"
    validation_config.parent.mkdir(parents=True, exist_ok=True)
    validation_config.write_text(
        "\n".join(
            (
                "model: outputs/best.pt",
                f"data: {artifact_paths['dataset_yaml']}",
                "task: detect",
                "mode: val",
                "split: val",
                "imgsz: 640",
                "batch: 8",
                "device: 0",
                "conf: 0.001",
                "iou: 0.70",
                "",
            )
        ),
        encoding="utf-8",
    )
    return spec, report


def make_inference_fixture(root: Path):
    spec, report = make_split_contract_fixture(root)
    experiment_path = root / "experiments.json"
    experiment_path.write_text(json.dumps(spec), encoding="utf-8")
    variant = spec["variants"]["yolo11s_640"]
    training_config = root / variant["training_config"]
    trained_weight = root / variant["trained_weight"]
    training_config.parent.mkdir(parents=True, exist_ok=True)
    trained_weight.parent.mkdir(parents=True, exist_ok=True)
    training_config.write_text("task: detect\n", encoding="utf-8")
    trained_weight.write_bytes(b"trained-weight")
    return spec, report, experiment_path, training_config, trained_weight


def write_completed_training_receipt(
    root: Path,
    spec,
    report,
    experiment_path: Path,
    training_config: Path,
    trained_weight: Path,
    *,
    trained_weight_sha256: str | None = None,
) -> Path:
    variant_name = "yolo11s_640"
    variant = spec["variants"][variant_name]
    receipt_path = root / variant["training_receipt"]
    split_contract = validate_split_contract(spec, root, report)
    write_json_exclusive(
        receipt_path,
        {
            "schema_version": 1,
            "kind": "one_time_claim",
            "purpose": f"formal_training:{variant_name}",
            "status": "completed",
            "bindings": {
                "variant": variant_name,
                "dataset_id": spec["dataset_id"],
                "expected_trained_weight": variant["trained_weight"],
                "training_config_sha256": sha256_file(training_config),
                "experiments_sha256": sha256_file(experiment_path),
                "canonical_dataset_sha256": report["canonical_dataset_sha256"],
                "split_contract": split_contract,
            },
            "details": {
                "trained_weight": variant["trained_weight"],
                "trained_weight_sha256": (
                    trained_weight_sha256 or sha256_file(trained_weight)
                ),
            },
        },
    )
    return receipt_path


def write_formal_validation_bundle(
    root: Path,
    spec,
    report,
    experiment_path: Path,
    training_config: Path,
    trained_weight: Path,
    training_receipt: Path,
    *,
    imgsz: int = 640,
    weights_source: str = "expected_trained_weight",
) -> tuple[Path, dict]:
    variant_name = "yolo11s_640"
    variant = spec["variants"][variant_name]
    split_contract = validate_split_contract(spec, root, report)
    bindings = {
        "variant": variant_name,
        "role": variant["role"],
        "dataset_id": spec["dataset_id"],
        "experiments_sha256": sha256_file(experiment_path),
        "training_config": variant["training_config"],
        "training_config_sha256": sha256_file(training_config),
        "validation_config": variant["validation_config"],
        "validation_config_sha256": sha256_file(
            root / variant["validation_config"]
        ),
        "validation_task": "detect",
        "validation_mode": "val",
        "validation_split": "val",
        "validation_imgsz": imgsz,
        "validation_batch": 8,
        "validation_device": "0",
        "validation_raw_confidence": 0.001,
        "validation_nms_iou": FORMAL_NMS_IOU,
        "dataset_yaml": spec["dataset_yaml"],
        "dataset_yaml_sha256": sha256_file(root / spec["dataset_yaml"]),
        "split_manifest": spec["split_manifest"],
        "split_manifest_sha256": sha256_file(root / spec["split_manifest"]),
        "weights": variant["trained_weight"],
        "weights_source": weights_source,
        "weights_sha256": sha256_file(trained_weight),
        "canonical_dataset_sha256": report["canonical_dataset_sha256"],
        "canonical_split_sha256": report["canonical_split_sha256"],
        "split_contract": split_contract,
        "training_receipt": variant["training_receipt"],
        "training_receipt_sha256": sha256_file(training_receipt),
    }
    path = root / "artifacts" / "validation_predictions.json"
    bundle = {
        "schema_version": 1,
        "kind": "prediction_bundle",
        "variant": variant_name,
        "role": variant["role"],
        "split": "val",
        "bindings": bindings,
        "inference": {
            "raw_confidence": 0.001,
            "imgsz": imgsz,
            "batch": 8,
            "device": "0",
            "nms_iou": FORMAL_NMS_IOU,
        },
        "records": [
            record_to_json(
                ImageDetections(
                    sample.image_key,
                    load_yolo_ground_truth(sample.label_path),
                    (
                        Prediction(0, 0.9, RIPE_BOX),
                        Prediction(1, 0.8, UNRIPE_BOX),
                    ),
                )
            )
            for sample in load_split_samples(root / spec["split_manifest"], "val")
        ],
    }
    write_json_exclusive(path, bundle)
    return path, bundle


class MetricTest(unittest.TestCase):
    def test_iou_uses_normalized_xyxy(self):
        self.assertAlmostEqual(1.0, intersection_over_union(RIPE_BOX, RIPE_BOX))
        self.assertEqual(0.0, intersection_over_union(RIPE_BOX, UNRIPE_BOX))
        self.assertAlmostEqual(
            1.0 / 7.0,
            intersection_over_union((0.0, 0.0, 0.4, 0.4), (0.2, 0.2, 0.6, 0.6)),
        )

    def test_macro_f1_is_class_balanced_and_fixed_threshold(self):
        records = (
            ImageDetections(
                "one.jpg",
                (GroundTruth(0, RIPE_BOX), GroundTruth(1, UNRIPE_BOX)),
                (
                    Prediction(0, 0.90, RIPE_BOX),
                    Prediction(0, 0.80, RIPE_BOX),
                    Prediction(1, 0.40, UNRIPE_BOX),
                ),
            ),
        )
        metrics = evaluate_detection_metrics(records, 0.50)
        self.assertEqual(METRIC_NAME, metrics["metric_name"])
        self.assertAlmostEqual(2.0 / 3.0, metrics["per_class"]["0"]["f1"])
        self.assertEqual(0.0, metrics["per_class"]["1"]["f1"])
        self.assertAlmostEqual(1.0 / 3.0, metrics["macro_f1"])

    def test_threshold_tie_prefers_higher_confidence(self):
        records = (
            ImageDetections(
                "one.jpg",
                (GroundTruth(0, RIPE_BOX), GroundTruth(1, UNRIPE_BOX)),
                (
                    Prediction(0, 0.80, RIPE_BOX),
                    Prediction(1, 0.80, UNRIPE_BOX),
                ),
            ),
        )
        selected = select_validation_threshold(records, (0.50, 0.80))
        self.assertEqual(0.80, selected["selected_threshold"])
        self.assertEqual(1.0, selected["validation_metrics"]["macro_f1"])

    def test_metric_rejects_class_without_support(self):
        records = (
            ImageDetections(
                "one.jpg", (GroundTruth(0, RIPE_BOX),), (Prediction(0, 0.9, RIPE_BOX),)
            ),
        )
        with self.assertRaisesRegex(ValueError, "class 1"):
            evaluate_detection_metrics(records, 0.5)

    def test_decimal_threshold_grid_has_stable_endpoints(self):
        values = threshold_grid(0.05, 0.08, 0.01)
        self.assertEqual((0.05, 0.06, 0.07, 0.08), values)


class ArtifactContractTest(unittest.TestCase):
    def test_hash_contract_checks_size_and_digest(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "weight.pt"
            path.write_bytes(b"pinned-weight")
            expected = hashlib.sha256(b"pinned-weight").hexdigest()
            self.assertEqual(expected, verify_file_contract(path, expected, 13))
            with self.assertRaisesRegex(ValueError, "size mismatch"):
                verify_file_contract(path, expected, 12)

    def test_one_time_claim_refuses_second_run_and_finalizes_once(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "test_receipt.json"
            token = acquire_one_time_claim(
                path, "held_out_test:model", {"weights_sha256": "a" * 64}, "start"
            )
            with self.assertRaises(FileExistsError):
                acquire_one_time_claim(path, "held_out_test:model", {}, "again")
            finalize_one_time_claim(
                path, token, "completed", "finish", {"macro_f1": 0.9}
            )
            receipt = read_json(path)
            self.assertEqual("completed", receipt["status"])
            self.assertEqual(0.9, receipt["details"]["macro_f1"])
            with self.assertRaisesRegex(ValueError, "claimed state"):
                finalize_one_time_claim(path, token, "completed", "again", {})

    def test_exclusive_json_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "freeze.json"
            write_json_exclusive(path, {"threshold": 0.6})
            with self.assertRaises(FileExistsError):
                write_json_exclusive(path, {"threshold": 0.7})
            self.assertEqual(0.6, read_json(path)["threshold"])

    @staticmethod
    def _fixture(*, unripe_f1=0.72, macro_f1=0.86, ripe_f1=0.88):
        spec = {
            "formal_test": {"variant": "yolo11s_640_cls_pw05_opt1"},
            "metric": {"macro_f1_min": 0.85},
            "optimization_policy": {
                "baseline_variant": "yolo11s_640",
                "candidate_variant": "yolo11s_640_cls_pw05_opt1",
                "validation_promotion": {
                    "unripe_f1_min": 0.709,
                    "macro_f1_min": 0.7971,
                    "ripe_f1_min": 0.8752,
                },
                "formal_acceptance": {"macro_f1_min": 0.85},
            },
        }
        actual = {
            "unripe_f1": unripe_f1,
            "macro_f1": macro_f1,
            "ripe_f1": ripe_f1,
        }
        required = {
            "unripe_f1": 0.709,
            "macro_f1": 0.7971,
            "ripe_f1": 0.8752,
        }
        passed = all(actual[name] >= value for name, value in required.items())
        promotion = {
            "schema_version": 1,
            "kind": "validation_promotion_decision",
            "status": "completed",
            "scope": {
                "baseline_variant": "yolo11s_640",
                "candidate_variant": "yolo11s_640_cls_pw05_opt1",
                "selection_split": "val",
                "held_out_test_accessed": False,
            },
            "bindings": {
                "experiments_sha256": "e" * 64,
                "candidate_frozen_threshold": "artifacts/candidate_freeze.json",
                "candidate_frozen_threshold_sha256": "f" * 64,
                "candidate_weights_sha256": "w" * 64,
                "candidate_validation_bundle": "artifacts/candidate_val.json",
                "candidate_validation_bundle_sha256": "v" * 64,
                "canonical_dataset_sha256": "d" * 64,
                "training_config_sha256": "c" * 64,
                "validation_config": "config/eval.yaml",
                "validation_config_sha256": "q" * 64,
                "validation_nms_iou": FORMAL_NMS_IOU,
            },
            "candidate_validation_metrics": actual,
            "checks": {
                name: {
                    "operator": ">=",
                    "required": value,
                    "actual": actual[name],
                    "passed": actual[name] >= value,
                }
                for name, value in required.items()
            },
            "all_validation_conditions_passed": passed,
            "promote_candidate": passed,
            "validation_authorization_gate": {
                "metric": "macro_f1",
                "operator": ">=",
                "required": 0.85,
                "actual": macro_f1,
                "passed": macro_f1 >= 0.85,
            },
            "authorize_held_out_test": passed and macro_f1 >= 0.85,
            "formal_test_gate": {
                "metric": "macro_f1",
                "operator": ">=",
                "required": 0.85,
                "evaluated": False,
            },
        }
        return spec, promotion

    def test_passed_validation_decision_unlocks_formal_candidate(self):
        spec, promotion = self._fixture()
        validate_optimization_promotion(
            promotion,
            spec,
            experiments_sha256="e" * 64,
            frozen_threshold_path="artifacts/candidate_freeze.json",
            frozen_threshold_sha256="f" * 64,
            weights_sha256="w" * 64,
            validation_bundle="artifacts/candidate_val.json",
            validation_bundle_sha256="v" * 64,
            canonical_dataset_sha256="d" * 64,
            training_config_sha256="c" * 64,
            validation_config="config/eval.yaml",
            validation_config_sha256="q" * 64,
            validation_nms_iou=FORMAL_NMS_IOU,
            expected_candidate_validation_metrics=promotion[
                "candidate_validation_metrics"
            ],
        )

    def test_failed_or_edited_validation_decision_cannot_unlock_test(self):
        spec, failed = self._fixture(unripe_f1=0.70)
        with self.assertRaisesRegex(ValueError, "did not pass"):
            validate_optimization_promotion(
                failed,
                spec,
                experiments_sha256="e" * 64,
                frozen_threshold_path="artifacts/candidate_freeze.json",
                frozen_threshold_sha256="f" * 64,
                weights_sha256="w" * 64,
                validation_bundle="artifacts/candidate_val.json",
                validation_bundle_sha256="v" * 64,
                canonical_dataset_sha256="d" * 64,
                training_config_sha256="c" * 64,
                validation_config="config/eval.yaml",
                validation_config_sha256="q" * 64,
                validation_nms_iou=FORMAL_NMS_IOU,
                expected_candidate_validation_metrics=failed[
                    "candidate_validation_metrics"
                ],
            )

        _, improved_but_below_formal_gate = self._fixture(macro_f1=0.81)
        self.assertTrue(improved_but_below_formal_gate["promote_candidate"])
        self.assertFalse(
            improved_but_below_formal_gate["authorize_held_out_test"]
        )
        with self.assertRaisesRegex(ValueError, "not authorized"):
            validate_optimization_promotion(
                improved_but_below_formal_gate,
                spec,
                experiments_sha256="e" * 64,
                frozen_threshold_path="artifacts/candidate_freeze.json",
                frozen_threshold_sha256="f" * 64,
                weights_sha256="w" * 64,
                validation_bundle="artifacts/candidate_val.json",
                validation_bundle_sha256="v" * 64,
                canonical_dataset_sha256="d" * 64,
                training_config_sha256="c" * 64,
                validation_config="config/eval.yaml",
                validation_config_sha256="q" * 64,
                validation_nms_iou=FORMAL_NMS_IOU,
                expected_candidate_validation_metrics=improved_but_below_formal_gate[
                    "candidate_validation_metrics"
                ],
            )

        _, edited = self._fixture()
        edited["checks"]["macro_f1"]["actual"] = 0.99
        with self.assertRaisesRegex(ValueError, "not reproducible"):
            validate_optimization_promotion(
                edited,
                spec,
                experiments_sha256="e" * 64,
                frozen_threshold_path="artifacts/candidate_freeze.json",
                frozen_threshold_sha256="f" * 64,
                weights_sha256="w" * 64,
                validation_bundle="artifacts/candidate_val.json",
                validation_bundle_sha256="v" * 64,
                canonical_dataset_sha256="d" * 64,
                training_config_sha256="c" * 64,
                validation_config="config/eval.yaml",
                validation_config_sha256="q" * 64,
                validation_nms_iou=FORMAL_NMS_IOU,
                expected_candidate_validation_metrics=edited[
                    "candidate_validation_metrics"
                ],
            )

    def test_optimization_artifact_path_is_fixed_and_safe(self):
        spec, _ = self._fixture()
        expected = (
            REPOSITORY_ROOT
            / "artifacts/perception/optimization/"
            "yolo11s_640_cls_pw05_opt1_promotion.json"
        ).resolve()
        self.assertEqual(
            expected, optimization_promotion_artifact_path(spec, REPOSITORY_ROOT)
        )
        spec["optimization_policy"]["candidate_variant"] = "../escape"
        with self.assertRaisesRegex(ValueError, "unsafe"):
            optimization_promotion_artifact_path(spec, REPOSITORY_ROOT)

    def test_frozen_threshold_is_bound_to_weight_and_split_hashes(self):
        freeze = {
            "schema_version": 1,
            "kind": "frozen_threshold",
            "metric_name": METRIC_NAME,
            "variant": "primary",
            "confidence_threshold": 0.67,
            "iou_threshold": 0.5,
            "class_ids": [0, 1],
            "bindings": {
                "weights_sha256": "a" * 64,
                "canonical_dataset_sha256": "b" * 64,
                "split_contract": {"split_contract_sha256": "s" * 64},
            },
        }
        self.assertEqual(
            0.67,
            validate_frozen_threshold(
                freeze,
                "primary",
                "a" * 64,
                "b" * 64,
                expected_split_contract={"split_contract_sha256": "s" * 64},
            ),
        )
        with self.assertRaisesRegex(ValueError, "different weights"):
            validate_frozen_threshold(freeze, "primary", "c" * 64, "b" * 64)
        with self.assertRaisesRegex(ValueError, "different canonical dataset"):
            validate_frozen_threshold(freeze, "primary", "a" * 64, "c" * 64)
        with self.assertRaisesRegex(ValueError, "different split contract"):
            validate_frozen_threshold(
                freeze,
                "primary",
                "a" * 64,
                "b" * 64,
                expected_split_contract={"split_contract_sha256": "x" * 64},
            )

    def test_training_receipt_binds_current_weight_and_canonical_dataset(self):
        receipt = {
            "schema_version": 1,
            "kind": "one_time_claim",
            "purpose": "formal_training:yolo11s_640",
            "status": "completed",
            "bindings": {
                "variant": "yolo11s_640",
                "dataset_id": "zenodo_6126677",
                "expected_trained_weight": "outputs/primary/best.pt",
                "training_config_sha256": "c" * 64,
                "experiments_sha256": "e" * 64,
                "canonical_dataset_sha256": "d" * 64,
                "split_contract": {"split_contract_sha256": "s" * 64},
            },
            "details": {
                "trained_weight": "outputs/primary/best.pt",
                "trained_weight_sha256": "w" * 64,
            },
        }
        self.assertEqual(
            "d" * 64,
            validate_completed_training_receipt(
                receipt,
                variant="yolo11s_640",
                dataset_id="zenodo_6126677",
                expected_trained_weight="outputs/primary/best.pt",
                weights_sha256="w" * 64,
                training_config_sha256="c" * 64,
                experiments_sha256="e" * 64,
                expected_split_contract={"split_contract_sha256": "s" * 64},
            ),
        )
        with self.assertRaisesRegex(ValueError, "split_contract"):
            validate_completed_training_receipt(
                receipt,
                variant="yolo11s_640",
                dataset_id="zenodo_6126677",
                expected_trained_weight="outputs/primary/best.pt",
                weights_sha256="w" * 64,
                training_config_sha256="c" * 64,
                experiments_sha256="e" * 64,
                expected_split_contract={"split_contract_sha256": "x" * 64},
            )
        with self.assertRaisesRegex(ValueError, "current trained weight hash"):
            validate_completed_training_receipt(
                receipt,
                variant="yolo11s_640",
                dataset_id="zenodo_6126677",
                expected_trained_weight="outputs/primary/best.pt",
                weights_sha256="x" * 64,
                training_config_sha256="c" * 64,
                experiments_sha256="e" * 64,
            )


class FormalThresholdContractTest(unittest.TestCase):
    def make_fixture(
        self,
        root: Path,
        *,
        imgsz: int = 640,
        weights_source: str = "expected_trained_weight",
    ):
        (
            spec,
            report,
            experiment_path,
            training_config,
            trained_weight,
        ) = make_inference_fixture(root)
        spec["metric"]["threshold_grid"] = {
            "start": 0.05,
            "stop": 0.95,
            "step": 0.01,
        }
        experiment_path.write_text(json.dumps(spec), encoding="utf-8")
        training_receipt = write_completed_training_receipt(
            root,
            spec,
            report,
            experiment_path,
            training_config,
            trained_weight,
        )
        validation_path, validation_bundle = write_formal_validation_bundle(
            root,
            spec,
            report,
            experiment_path,
            training_config,
            trained_weight,
            training_receipt,
            imgsz=imgsz,
            weights_source=weights_source,
        )
        return spec, report, experiment_path, validation_path, validation_bundle

    def run_freeze(
        self,
        root: Path,
        report,
        experiment_path: Path,
        validation_path: Path,
    ) -> Path:
        output = root / "artifacts" / "frozen_threshold.json"
        with (
            mock.patch.object(freeze_threshold_module, "REPOSITORY_ROOT", root),
            mock.patch.object(
                freeze_threshold_module,
                "verify_materialized_dataset",
                return_value=report,
            ),
        ):
            freeze_threshold_module.main(
                [
                    "--variant",
                    "yolo11s_640",
                    "--validation-bundle",
                    str(validation_path),
                    "--output",
                    str(output),
                    "--experiments",
                    str(experiment_path),
                ]
            )
        return output

    def test_formal_validation_requires_all_manifest_keys_and_current_ground_truth(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, _, _, validation_path, _ = self.make_fixture(root)
            _, records = load_prediction_bundle(validation_path, "val")
            manifest = root / spec["split_manifest"]
            contract = spec["split_contract"]
            kwargs = {
                "class_ids": (0, 1),
                "expected_count": contract["counts"]["val"],
            }
            with self.assertRaisesRegex(ValueError, "complete validation split"):
                validate_validation_records_against_split(
                    records[:-1], manifest, **kwargs
                )
            extra = records + (
                ImageDetections(
                    "images/val/extra.jpg",
                    (GroundTruth(0, RIPE_BOX), GroundTruth(1, UNRIPE_BOX)),
                    (),
                ),
            )
            with self.assertRaisesRegex(ValueError, "complete validation split"):
                validate_validation_records_against_split(extra, manifest, **kwargs)
            tampered = list(records)
            first = tampered[0]
            tampered[0] = ImageDetections(
                first.image,
                (GroundTruth(0, UNRIPE_BOX), GroundTruth(1, RIPE_BOX)),
                first.predictions,
            )
            with self.assertRaisesRegex(ValueError, "ground truth"):
                validate_validation_records_against_split(
                    tuple(tampered), manifest, **kwargs
                )

    def test_formal_validation_rejects_validation_config_and_nms_tampering(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, report, experiment_path, validation_path, bundle = self.make_fixture(root)
            expected = bundle["bindings"]
            edited_hash = copy.deepcopy(bundle)
            edited_hash["bindings"]["validation_config_sha256"] = "9" * 64
            with self.assertRaisesRegex(ValueError, "validation_config_sha256"):
                validate_formal_validation_bundle(
                    edited_hash,
                    variant="yolo11s_640",
                    expected_bindings=expected,
                    expected_imgsz=spec["formal_test"]["imgsz"],
                )
            config_path = root / spec["variants"]["yolo11s_640"]["validation_config"]
            config_path.write_text(
                config_path.read_text(encoding="utf-8").replace(
                    "iou: 0.70", "iou: 0.50"
                ),
                encoding="utf-8",
            )
            with (
                mock.patch.object(freeze_threshold_module, "REPOSITORY_ROOT", root),
                mock.patch.object(
                    freeze_threshold_module,
                    "verify_materialized_dataset",
                    return_value=report,
                ),
            ):
                with self.assertRaisesRegex(ValueError, "NMS IoU is fixed"):
                    freeze_threshold_module.main(
                        [
                            "--variant",
                            "yolo11s_640",
                            "--validation-bundle",
                            str(validation_path),
                            "--output",
                            str(root / "artifacts" / "tampered_freeze.json"),
                            "--experiments",
                            str(experiment_path),
                        ]
                    )
            edited_nms = copy.deepcopy(bundle)
            edited_nms["inference"]["nms_iou"] = 0.50
            with self.assertRaisesRegex(ValueError, "inference settings"):
                validate_formal_validation_bundle(
                    edited_nms,
                    variant="yolo11s_640",
                    expected_bindings=expected,
                    expected_imgsz=spec["formal_test"]["imgsz"],
                )

    def test_forged_high_validation_predictions_cannot_create_test_claim(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, report, experiment_path, validation_path, _ = self.make_fixture(root)
            freeze_path = self.run_freeze(
                root, report, experiment_path, validation_path
            )
            samples = load_split_samples(root / spec["split_manifest"], "val")
            low_records = tuple(
                ImageDetections(
                    sample.image_key,
                    load_yolo_ground_truth(sample.label_path),
                    (),
                )
                for sample in samples
            )
            output = root / "artifacts" / "test_predictions.json"
            test_receipt = root / spec["formal_test"]["receipt"]
            with (
                mock.patch.object(run_inference_module, "REPOSITORY_ROOT", root),
                mock.patch.object(
                    run_inference_module, "DEFAULT_EXPERIMENTS", experiment_path
                ),
                mock.patch.object(
                    run_inference_module,
                    "verify_materialized_dataset",
                    return_value=report,
                ),
                mock.patch.object(
                    run_inference_module, "_run_model", return_value=low_records
                ) as run_model,
            ):
                with self.assertRaisesRegex(ValueError, "fresh validation replay"):
                    run_inference_module.main(
                        [
                            "--variant",
                            "yolo11s_640",
                            "--split",
                            "test",
                            "--output",
                            str(output),
                            "--experiments",
                            str(experiment_path),
                            "--frozen-threshold",
                            str(freeze_path),
                        ]
                    )
            run_model.assert_called_once()
            self.assertFalse(test_receipt.exists())
            self.assertFalse(output.exists())

    def test_primary_freeze_records_complete_formal_provenance(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, report, experiment_path, validation_path, bundle = self.make_fixture(
                root
            )
            output = self.run_freeze(
                root, report, experiment_path, validation_path
            )
            freeze = read_json(output)
            bindings = freeze["bindings"]
            for field in FORMAL_VALIDATION_BINDING_FIELDS:
                self.assertEqual(bundle["bindings"][field], bindings[field])
            self.assertEqual(
                "artifacts/validation_predictions.json",
                bindings["validation_bundle"],
            )
            self.assertEqual(640, bindings["validation_imgsz"])
            self.assertEqual(sha256_file(validation_path), bindings["validation_bundle_sha256"])
            validate_formal_validation_bundle(
                bundle,
                variant="yolo11s_640",
                expected_bindings=bundle["bindings"],
                expected_imgsz=spec["formal_test"]["imgsz"],
            )

    def test_primary_freeze_rejects_explicit_weights_and_wrong_imgsz(self):
        cases = (
            ("explicit_validation_override", 640, "weights_source"),
            ("expected_trained_weight", 1280, "validation_imgsz"),
        )
        for weights_source, imgsz, message in cases:
            with self.subTest(weights_source=weights_source, imgsz=imgsz), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                (
                    _,
                    report,
                    experiment_path,
                    validation_path,
                    _,
                ) = self.make_fixture(
                    root, imgsz=imgsz, weights_source=weights_source
                )
                output = root / "artifacts" / "frozen_threshold.json"
                with (
                    mock.patch.object(
                        freeze_threshold_module, "REPOSITORY_ROOT", root
                    ),
                    mock.patch.object(
                        freeze_threshold_module,
                        "verify_materialized_dataset",
                        return_value=report,
                    ),
                ):
                    with self.assertRaisesRegex(ValueError, message):
                        freeze_threshold_module.main(
                            [
                                "--variant",
                                "yolo11s_640",
                                "--validation-bundle",
                                str(validation_path),
                                "--output",
                                str(output),
                                "--experiments",
                                str(experiment_path),
                            ]
                        )
                self.assertFalse(output.exists())

    def test_non_primary_diagnostic_freeze_remains_available(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            experiment_path = root / "experiments.json"
            spec = {
                "schema_version": 1,
                "formal_test": {
                    "variant": "yolo11s_640",
                    "receipt": "artifacts/test_receipt.json",
                    "task": "detect",
                    "imgsz": 640,
                },
                "metric": {
                    "name": METRIC_NAME,
                    "iou_threshold": 0.5,
                    "class_ids": [0, 1],
                    "threshold_grid": {
                        "start": 0.05,
                        "stop": 0.95,
                        "step": 0.01,
                    },
                },
                "variants": {
                    "yolo11n_640_baseline": {"role": "speed_baseline"}
                },
            }
            experiment_path.write_text(json.dumps(spec), encoding="utf-8")
            validation_path = root / "diagnostic_predictions.json"
            write_json_exclusive(
                validation_path,
                {
                    "schema_version": 1,
                    "kind": "prediction_bundle",
                    "variant": "yolo11n_640_baseline",
                    "role": "speed_baseline",
                    "split": "val",
                    "bindings": {
                        "experiments_sha256": sha256_file(experiment_path),
                        "weights_sha256": "w" * 64,
                        "canonical_dataset_sha256": "d" * 64,
                        "canonical_split_sha256": {
                            "train": "1" * 64,
                            "val": "2" * 64,
                            "test": "3" * 64,
                        },
                        "training_config_sha256": "c" * 64,
                        "split_contract": {"split_contract_sha256": "s" * 64},
                    },
                    "inference": {"raw_confidence": 0.001, "imgsz": 1280},
                    "records": [
                        record_to_json(
                            ImageDetections(
                                "images/val/one.jpg",
                                (
                                    GroundTruth(0, RIPE_BOX),
                                    GroundTruth(1, UNRIPE_BOX),
                                ),
                                (
                                    Prediction(0, 0.9, RIPE_BOX),
                                    Prediction(1, 0.8, UNRIPE_BOX),
                                ),
                            )
                        )
                    ],
                },
            )
            output = root / "diagnostic_threshold.json"
            self.assertEqual(
                0,
                freeze_threshold_module.main(
                    [
                        "--variant",
                        "yolo11n_640_baseline",
                        "--validation-bundle",
                        str(validation_path),
                        "--output",
                        str(output),
                        "--experiments",
                        str(experiment_path),
                    ]
                ),
            )
            self.assertTrue(output.is_file())
            self.assertNotIn("validation_bundle", read_json(output)["bindings"])

    def test_formal_test_rejects_edited_provenance_and_selection(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, report, experiment_path, validation_path, bundle = self.make_fixture(
                root
            )
            output = self.run_freeze(
                root, report, experiment_path, validation_path
            )
            freeze = read_json(output)
            expected = bundle["bindings"]

            edited_provenance = copy.deepcopy(freeze)
            edited_provenance["bindings"]["training_receipt_sha256"] = "9" * 64
            with self.assertRaisesRegex(ValueError, "training_receipt_sha256"):
                validate_frozen_threshold(
                    edited_provenance,
                    "yolo11s_640",
                    expected["weights_sha256"],
                    expected["canonical_dataset_sha256"],
                    training_config_sha256=expected["training_config_sha256"],
                    experiments_sha256=expected["experiments_sha256"],
                    expected_split_contract=expected["split_contract"],
                    expected_formal_validation_bindings=expected,
                    expected_validation_imgsz=640,
                )

            edited_selection = copy.deepcopy(freeze)
            edited_selection["confidence_threshold"] = 0.79
            edited_selection["selection"]["selected_threshold"] = 0.79
            self.assertEqual(
                0.79,
                validate_frozen_threshold(
                    edited_selection,
                    "yolo11s_640",
                    expected["weights_sha256"],
                    expected["canonical_dataset_sha256"],
                    training_config_sha256=expected["training_config_sha256"],
                    experiments_sha256=expected["experiments_sha256"],
                    expected_split_contract=expected["split_contract"],
                    expected_formal_validation_bindings=expected,
                    expected_validation_imgsz=640,
                ),
            )
            with mock.patch.object(run_inference_module, "REPOSITORY_ROOT", root):
                with self.assertRaisesRegex(ValueError, "cannot be reproduced"):
                    _validate_formal_threshold_source(
                        edited_selection,
                        spec["metric"],
                        spec["formal_test"],
                        expected,
                    )

    def test_edited_selection_cannot_consume_formal_test_claim(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, report, experiment_path, validation_path, _ = self.make_fixture(root)
            freeze_path = self.run_freeze(
                root, report, experiment_path, validation_path
            )
            edited = copy.deepcopy(read_json(freeze_path))
            edited["confidence_threshold"] = 0.79
            edited["selection"]["selected_threshold"] = 0.79
            edited_path = root / "artifacts" / "edited_threshold.json"
            write_json_exclusive(edited_path, edited)
            output = root / "artifacts" / "test_predictions.json"
            test_receipt = root / spec["formal_test"]["receipt"]
            with (
                mock.patch.object(run_inference_module, "REPOSITORY_ROOT", root),
                mock.patch.object(
                    run_inference_module, "DEFAULT_EXPERIMENTS", experiment_path
                ),
                mock.patch.object(
                    run_inference_module,
                    "verify_materialized_dataset",
                    return_value=report,
                ),
                mock.patch.object(run_inference_module, "_run_model") as run_model,
            ):
                with self.assertRaisesRegex(ValueError, "cannot be reproduced"):
                    run_inference_module.main(
                        [
                            "--variant",
                            "yolo11s_640",
                            "--split",
                            "test",
                            "--output",
                            str(output),
                            "--experiments",
                            str(experiment_path),
                            "--frozen-threshold",
                            str(edited_path),
                        ]
                    )
            run_model.assert_not_called()
            self.assertFalse(test_receipt.exists())
            self.assertFalse(output.exists())


class DatasetAdapterTest(unittest.TestCase):
    def test_loads_split_manifest_and_yolo_labels(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            image = root / "images" / "val" / "sample.jpg"
            label = root / "labels" / "val" / "sample.txt"
            image.parent.mkdir(parents=True)
            label.parent.mkdir(parents=True)
            image.write_bytes(b"image")
            label.write_text(
                "0 0.2 0.2 0.2 0.2\n1 0.75 0.75 0.3 0.3\n", encoding="utf-8"
            )
            manifest = root / "split_manifest.json"
            manifest.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "splits": {
                            "val": [
                                {
                                    "output_image": "images/val/sample.jpg",
                                    "output_label": "labels/val/sample.txt",
                                }
                            ],
                            "test": [],
                        },
                    }
                ),
                encoding="utf-8",
            )
            samples = load_split_samples(manifest, "val")
            self.assertEqual(1, len(samples))
            truths = load_yolo_ground_truth(samples[0].label_path)
            self.assertEqual([0, 1], [item.class_id for item in truths])
            for expected, actual in zip(RIPE_BOX, truths[0].bbox):
                self.assertAlmostEqual(expected, actual)

    def test_split_manifest_rejects_path_traversal(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = root / "split_manifest.json"
            manifest.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "splits": {
                            "val": [
                                {
                                    "output_image": "../outside.jpg",
                                    "output_label": "labels/val/outside.txt",
                                }
                            ]
                        },
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "unsafe relative path"):
                load_split_samples(manifest, "val")


class SplitContractTest(unittest.TestCase):
    def test_frozen_split_contract_validates_and_summarizes_every_binding(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, report = make_split_contract_fixture(root)
            summary = validate_split_contract(spec, root, report)
            self.assertEqual("seeded_group_stratified_full_resplit", summary["strategy"])
            self.assertEqual({"train": 501, "val": 116, "test": 115}, summary["counts"])
            self.assertEqual(732, summary["retained_count"])
            self.assertRegex(summary["split_contract_sha256"], r"^[0-9a-f]{64}$")
            self.assertEqual(
                spec["split_contract"]["artifacts"], summary["artifacts"]
            )
            with mock.patch.object(run_inference_module, "REPOSITORY_ROOT", root):
                bindings = _dataset_bindings(
                    report,
                    spec,
                    root / spec["dataset_yaml"],
                )
            self.assertEqual(summary, bindings["split_contract"])

    def test_every_report_binding_mismatch_fails_closed(self):
        mutations = {
            "strategy": lambda report: report.__setitem__("strategy", "official_three_way"),
            "seed": lambda report: report.__setitem__("seed", 1),
            "counts": lambda report: report["split_sample_count"].__setitem__("train", 500),
            "sample_count": lambda report: report.__setitem__("sample_count", 731),
            "source_count": lambda report: report.__setitem__("source_image_count", 812),
            "excluded_count": lambda report: report.__setitem__("excluded_count", 80),
            "retained_count": lambda report: report.__setitem__("retained_count", 731),
            "dataset_digest": lambda report: report.__setitem__(
                "canonical_dataset_sha256", "9" * 64
            ),
            "split_digest": lambda report: report["canonical_split_sha256"].__setitem__(
                "test", "9" * 64
            ),
            "dataset_yaml": lambda report: report.__setitem__(
                "dataset_yaml_sha256", "9" * 64
            ),
            "exclusion_path": lambda report: report.__setitem__(
                "exclusions_csv", "different.csv"
            ),
            "exclusion_hash": lambda report: report.__setitem__(
                "exclusions_csv_sha256", "9" * 64
            ),
        }
        for name, mutate in mutations.items():
            with self.subTest(field=name), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                spec, report = make_split_contract_fixture(root)
                mutate(report)
                with self.assertRaisesRegex(ValueError, "does not match"):
                    validate_split_contract(spec, root, report)

    def test_contract_policy_counts_hashes_and_each_artifact_are_immutable(self):
        contract_mutations = {
            "strategy": lambda contract: contract.__setitem__("strategy", "other"),
            "seed": lambda contract: contract.__setitem__("seed", 1),
            "ratios": lambda contract: contract["ratios"].__setitem__("train", 0.69),
            "counts": lambda contract: contract["counts"].__setitem__("val", 115),
            "source_count": lambda contract: contract.__setitem__("source_image_count", 812),
            "excluded_count": lambda contract: contract.__setitem__("excluded_count", 80),
            "retained_count": lambda contract: contract.__setitem__("retained_count", 731),
            "dataset_digest": lambda contract: contract.__setitem__(
                "canonical_dataset_sha256", "9" * 64
            ),
            "split_digest": lambda contract: contract["canonical_split_sha256"].__setitem__(
                "train", "9" * 64
            ),
        }
        for name, mutate in contract_mutations.items():
            with self.subTest(field=name), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                spec, report = make_split_contract_fixture(root)
                mutate(spec["split_contract"])
                with self.assertRaises(ValueError):
                    validate_split_contract(spec, root, report)

        for artifact_name in (
            "groups_csv",
            "group_overrides_csv",
            "exclusions_csv",
            "group_audit",
            "curation_audit",
            "split_manifest",
            "dataset_yaml",
        ):
            with self.subTest(artifact=artifact_name), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                spec, report = make_split_contract_fixture(root)
                spec["split_contract"]["artifacts"][artifact_name]["sha256"] = "9" * 64
                with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                    validate_split_contract(spec, root, report)

    def test_formal_test_contract_failure_cannot_create_claim(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, report = make_split_contract_fixture(root)
            experiment_path = root / "experiments.json"
            experiment_path.write_text(json.dumps(spec), encoding="utf-8")
            training_config = root / spec["variants"]["yolo11s_640"]["training_config"]
            trained_weight = root / spec["variants"]["yolo11s_640"]["trained_weight"]
            training_config.parent.mkdir(parents=True, exist_ok=True)
            trained_weight.parent.mkdir(parents=True, exist_ok=True)
            training_config.write_text("task: detect\n", encoding="utf-8")
            trained_weight.write_bytes(b"trained-weight")
            report["strategy"] = "official_three_way"
            receipt = root / spec["formal_test"]["receipt"]
            output = root / "test_predictions.json"
            with (
                mock.patch.object(run_inference_module, "REPOSITORY_ROOT", root),
                mock.patch.object(
                    run_inference_module, "DEFAULT_EXPERIMENTS", experiment_path
                ),
                mock.patch.object(
                    run_inference_module,
                    "verify_materialized_dataset",
                    return_value=report,
                ),
            ):
                with self.assertRaisesRegex(ValueError, "strategy does not match"):
                    run_inference_module.main(
                        [
                            "--variant",
                            "yolo11s_640",
                            "--split",
                            "test",
                            "--output",
                            str(output),
                            "--experiments",
                            str(experiment_path),
                            "--frozen-threshold",
                            str(root / "freeze.json"),
                        ]
                    )
            self.assertFalse(receipt.exists())
            self.assertFalse(output.exists())


class ValidationTrainingReceiptGateTest(unittest.TestCase):
    def test_formal_validation_cli_must_match_pinned_config(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, _, experiment_path, _, _ = make_inference_fixture(root)
            output = root / "validation_predictions.json"
            with (
                mock.patch.object(run_inference_module, "REPOSITORY_ROOT", root),
                mock.patch.object(
                    run_inference_module, "DEFAULT_EXPERIMENTS", experiment_path
                ),
                mock.patch.object(run_inference_module, "_run_model") as run_model,
            ):
                with self.assertRaisesRegex(ValueError, "does not match"):
                    run_inference_module.main(
                        [
                            "--variant",
                            "yolo11s_640",
                            "--split",
                            "val",
                            "--batch",
                            "4",
                            "--output",
                            str(output),
                            "--experiments",
                            str(experiment_path),
                        ]
                    )
            run_model.assert_not_called()
            self.assertFalse(output.exists())

    def test_default_validation_rejects_missing_or_claimed_receipt_before_model(self):
        for receipt_state in ("missing", "claimed"):
            with self.subTest(receipt_state=receipt_state), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                spec, report, experiment_path, _, _ = make_inference_fixture(root)
                variant = spec["variants"]["yolo11s_640"]
                training_receipt = root / variant["training_receipt"]
                if receipt_state == "claimed":
                    write_json_exclusive(
                        training_receipt,
                        {
                            "schema_version": 1,
                            "kind": "one_time_claim",
                            "purpose": "formal_training:yolo11s_640",
                            "status": "claimed",
                        },
                    )
                output = root / "validation_predictions.json"
                with (
                    mock.patch.object(run_inference_module, "REPOSITORY_ROOT", root),
                    mock.patch.object(
                        run_inference_module, "DEFAULT_EXPERIMENTS", experiment_path
                    ),
                    mock.patch.object(
                        run_inference_module,
                        "verify_materialized_dataset",
                        return_value=report,
                    ),
                    mock.patch.object(run_inference_module, "_run_model") as run_model,
                ):
                    error = FileNotFoundError if receipt_state == "missing" else ValueError
                    with self.assertRaises(error):
                        run_inference_module.main(
                            [
                                "--variant",
                                "yolo11s_640",
                                "--split",
                                "val",
                                "--output",
                                str(output),
                                "--experiments",
                                str(experiment_path),
                            ]
                        )
                run_model.assert_not_called()
                self.assertFalse(output.exists())

    def test_default_validation_rejects_receipt_weight_mismatch_before_model(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (
                spec,
                report,
                experiment_path,
                training_config,
                trained_weight,
            ) = make_inference_fixture(root)
            write_completed_training_receipt(
                root,
                spec,
                report,
                experiment_path,
                training_config,
                trained_weight,
                trained_weight_sha256="9" * 64,
            )
            output = root / "validation_predictions.json"
            with (
                mock.patch.object(run_inference_module, "REPOSITORY_ROOT", root),
                mock.patch.object(
                    run_inference_module, "DEFAULT_EXPERIMENTS", experiment_path
                ),
                mock.patch.object(
                    run_inference_module,
                    "verify_materialized_dataset",
                    return_value=report,
                ),
                mock.patch.object(run_inference_module, "_run_model") as run_model,
            ):
                with self.assertRaisesRegex(ValueError, "weight hash does not match"):
                    run_inference_module.main(
                        [
                            "--variant",
                            "yolo11s_640",
                            "--split",
                            "val",
                            "--output",
                            str(output),
                            "--experiments",
                            str(experiment_path),
                        ]
                    )
            run_model.assert_not_called()
            self.assertFalse(output.exists())

    def test_default_validation_binds_completed_training_receipt(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (
                spec,
                report,
                experiment_path,
                training_config,
                trained_weight,
            ) = make_inference_fixture(root)
            training_receipt = write_completed_training_receipt(
                root,
                spec,
                report,
                experiment_path,
                training_config,
                trained_weight,
            )
            output = root / "validation_predictions.json"
            records = (
                ImageDetections(
                    "images/val/one.jpg",
                    (GroundTruth(0, RIPE_BOX), GroundTruth(1, UNRIPE_BOX)),
                    (Prediction(0, 0.9, RIPE_BOX),),
                    inference_ms=4.0,
                    total_ms=5.0,
                ),
            )
            with (
                mock.patch.object(run_inference_module, "REPOSITORY_ROOT", root),
                mock.patch.object(
                    run_inference_module, "DEFAULT_EXPERIMENTS", experiment_path
                ),
                mock.patch.object(
                    run_inference_module,
                    "verify_materialized_dataset",
                    return_value=report,
                ),
                mock.patch.object(
                    run_inference_module, "load_split_samples", return_value=(object(),)
                ),
                mock.patch.object(
                    run_inference_module, "_run_model", return_value=records
                ) as run_model,
                mock.patch.object(
                    run_inference_module, "collect_environment", return_value={}
                ),
            ):
                self.assertEqual(
                    0,
                    run_inference_module.main(
                        [
                            "--variant",
                            "yolo11s_640",
                            "--split",
                            "val",
                            "--output",
                            str(output),
                            "--experiments",
                            str(experiment_path),
                        ]
                    ),
                )
            run_model.assert_called_once()
            bundle = read_json(output)
            self.assertEqual(
                spec["variants"]["yolo11s_640"]["training_receipt"],
                bundle["bindings"]["training_receipt"],
            )
            self.assertEqual(
                sha256_file(training_receipt),
                bundle["bindings"]["training_receipt_sha256"],
            )
            self.assertEqual("expected_trained_weight", bundle["bindings"]["weights_source"])

    def test_explicit_validation_override_remains_receipt_free_diagnostic(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, report, experiment_path, _, _ = make_inference_fixture(root)
            diagnostic_weight = root / "diagnostic.pt"
            diagnostic_weight.write_bytes(b"diagnostic-weight")
            output = root / "diagnostic_predictions.json"
            records = (
                ImageDetections(
                    "images/val/one.jpg",
                    (GroundTruth(0, RIPE_BOX),),
                    (),
                    inference_ms=4.0,
                    total_ms=5.0,
                ),
            )
            with (
                mock.patch.object(run_inference_module, "REPOSITORY_ROOT", root),
                mock.patch.object(
                    run_inference_module, "DEFAULT_EXPERIMENTS", experiment_path
                ),
                mock.patch.object(
                    run_inference_module,
                    "verify_materialized_dataset",
                    return_value=report,
                ),
                mock.patch.object(
                    run_inference_module, "load_split_samples", return_value=(object(),)
                ),
                mock.patch.object(
                    run_inference_module, "_run_model", return_value=records
                ) as run_model,
                mock.patch.object(
                    run_inference_module, "collect_environment", return_value={}
                ),
            ):
                self.assertEqual(
                    0,
                    run_inference_module.main(
                        [
                            "--variant",
                            "yolo11s_640",
                            "--split",
                            "val",
                            "--weights",
                            str(diagnostic_weight),
                            "--output",
                            str(output),
                            "--experiments",
                            str(experiment_path),
                        ]
                    ),
                )
            run_model.assert_called_once()
            bundle = read_json(output)
            self.assertEqual(
                "explicit_validation_override", bundle["bindings"]["weights_source"]
            )
            self.assertNotIn("training_receipt", bundle["bindings"])
            self.assertNotIn("training_receipt_sha256", bundle["bindings"])

    def test_formal_test_reuses_receipt_gate_before_one_time_claim(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, report, experiment_path, _, _ = make_inference_fixture(root)
            variant = spec["variants"]["yolo11s_640"]
            write_json_exclusive(
                root / variant["training_receipt"],
                {
                    "schema_version": 1,
                    "kind": "one_time_claim",
                    "purpose": "formal_training:yolo11s_640",
                    "status": "claimed",
                },
            )
            test_receipt = root / spec["formal_test"]["receipt"]
            output = root / "test_predictions.json"
            with (
                mock.patch.object(run_inference_module, "REPOSITORY_ROOT", root),
                mock.patch.object(
                    run_inference_module, "DEFAULT_EXPERIMENTS", experiment_path
                ),
                mock.patch.object(
                    run_inference_module,
                    "verify_materialized_dataset",
                    return_value=report,
                ),
                mock.patch.object(run_inference_module, "_run_model") as run_model,
            ):
                with self.assertRaisesRegex(ValueError, "training receipt is not completed"):
                    run_inference_module.main(
                        [
                            "--variant",
                            "yolo11s_640",
                            "--split",
                            "test",
                            "--output",
                            str(output),
                            "--experiments",
                            str(experiment_path),
                            "--frozen-threshold",
                            str(root / "freeze.json"),
                        ]
                    )
            run_model.assert_not_called()
            self.assertFalse(test_receipt.exists())
            self.assertFalse(output.exists())


class BundleAndExperimentTest(unittest.TestCase):
    def test_prediction_bundle_round_trip_and_speed_summary(self):
        record = ImageDetections(
            "images/val/one.jpg",
            (GroundTruth(0, RIPE_BOX), GroundTruth(1, UNRIPE_BOX)),
            (Prediction(0, 0.9, RIPE_BOX),),
            inference_ms=4.0,
            total_ms=5.0,
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "bundle.json"
            write_json_exclusive(
                path,
                {
                    "schema_version": 1,
                    "kind": "prediction_bundle",
                    "split": "val",
                    "records": [record_to_json(record)],
                },
            )
            _, loaded = load_prediction_bundle(path, "val")
            self.assertEqual((record,), loaded)
            speed = summarize_speed(loaded)
            self.assertEqual(4.0, speed["inference_ms_p95"])
            self.assertEqual(200.0, speed["throughput_fps_from_mean_total"])

    def test_experiment_matrix_pins_opt1_primary_and_historical_baseline(self):
        path = TOOLS_ROOT / "experiments.json"
        spec, historical = load_experiment(path, "yolo11s_640")
        _, primary = load_experiment(path, "yolo11s_640_cls_pw05_opt1")
        _, baseline = load_experiment(path, "yolo11n_640_baseline")
        self.assertEqual("zenodo_6126677", spec["dataset_id"])
        self.assertEqual(0.85, spec["metric"]["macro_f1_min"])
        split_contract = spec["split_contract"]
        self.assertEqual(
            "seeded_group_stratified_full_resplit", split_contract["strategy"]
        )
        self.assertEqual(20260710, split_contract["seed"])
        self.assertEqual(
            {"train": 0.70, "val": 0.15, "test": 0.15},
            split_contract["ratios"],
        )
        self.assertEqual(
            {"train": 501, "val": 116, "test": 115}, split_contract["counts"]
        )
        self.assertEqual(813, split_contract["source_image_count"])
        self.assertEqual(81, split_contract["excluded_count"])
        self.assertEqual(732, split_contract["retained_count"])
        self.assertEqual(
            "ae3f17e5929daac983feca57f6f36e4ca70ef2b8838155b350c5f1503a098514",
            split_contract["canonical_dataset_sha256"],
        )
        self.assertEqual(
            "4a146b792ec6838f0b00782dc6145ebce352034678ca35cdc3ae9436eacfe237",
            split_contract["artifacts"]["split_manifest"]["sha256"],
        )
        self.assertEqual(
            "5a6ce9b3bfefc76d70a3a395aaaf6eb98943e9fb28bef17fb13b89732dfea445",
            split_contract["artifacts"]["groups_csv"]["sha256"],
        )
        self.assertEqual("primary", primary["role"])
        self.assertEqual("historical_primary", historical["role"])
        self.assertEqual("speed_baseline", baseline["role"])
        self.assertEqual(
            "artifacts/perception/zenodo_6126677_held_out_test_receipt.json",
            spec["formal_test"]["receipt"],
        )
        self.assertEqual(
            "yolo11s_640_cls_pw05_opt1", spec["formal_test"]["variant"]
        )
        self.assertEqual("detect", spec["formal_test"]["task"])
        self.assertEqual(640, spec["formal_test"]["imgsz"])
        self.assertNotIn("test_receipt", primary)
        self.assertNotIn("test_receipt", baseline)
        self.assertIn("training_receipt", primary)
        self.assertEqual(64, len(primary["base_weight"]["sha256"]))
        self.assertTrue(primary["base_weight"]["url"].startswith("https://github.com/ultralytics/"))

        primary_config = REPOSITORY_ROOT / primary["training_config"]
        historical_config = REPOSITORY_ROOT / historical["training_config"]
        baseline_config = REPOSITORY_ROOT / baseline["training_config"]
        self.assertIn("model: weights/yolo11s.pt", primary_config.read_text(encoding="utf-8"))
        self.assertIn("model: weights/yolo11n.pt", baseline_config.read_text(encoding="utf-8"))

        def simple_yaml(path):
            values = {}
            for raw_line in path.read_text(encoding="utf-8").splitlines():
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                key, value = line.split(":", 1)
                values[key.strip()] = value.strip()
            return values

        historical_values = simple_yaml(historical_config)
        primary_values = simple_yaml(primary_config)
        self.assertNotIn("cls_pw", historical_values)
        self.assertEqual("0.5", primary_values.pop("cls_pw"))
        self.assertEqual(
            {**historical_values, "name": "yolo11s_640_cls_pw05_opt1"},
            primary_values,
        )

        historical_eval = simple_yaml(REPOSITORY_ROOT / historical["validation_config"])
        primary_eval = simple_yaml(REPOSITORY_ROOT / primary["validation_config"])
        self.assertEqual(
            {
                **historical_eval,
                "model": "outputs/perception/yolo11s_640_cls_pw05_opt1/weights/best.pt",
                "name": "yolo11s_640_cls_pw05_opt1_validation_raw",
            },
            primary_eval,
        )

        policy = spec["optimization_policy"]
        self.assertEqual("yolo11s_640", policy["baseline_variant"])
        self.assertEqual("yolo11s_640_cls_pw05_opt1", policy["candidate_variant"])
        self.assertEqual(0.0, policy["intervention"]["baseline_value"])
        self.assertEqual(0.5, policy["intervention"]["candidate_value"])
        self.assertEqual(
            {"unripe_f1_min": 0.709, "macro_f1_min": 0.7971, "ripe_f1_min": 0.8752},
            policy["validation_promotion"],
        )
        self.assertEqual(0.85, policy["formal_acceptance"]["macro_f1_min"])
        historical_contract = (
            REPOSITORY_ROOT
            / "artifacts/perception/contracts/experiments_yolo11s_640_pre_opt1.json"
        )
        self.assertEqual(
            "529dc43bddec4a1ba8061020aedb31640184552387ae78441f21c934a4a25f5e",
            sha256_file(historical_contract),
        )
        decision = read_json(
            REPOSITORY_ROOT
            / "artifacts/perception/optimization/yolo11s_640_opt1_decision.json"
        )
        self.assertEqual("validation_only_optimization_decision", decision["kind"])
        self.assertFalse(decision["scope"]["held_out_test_accessed"])
        self.assertEqual(
            0.7871205584751746,
            decision["baseline_provenance"]["validation_metrics"]["macro_f1"],
        )

    def test_formal_test_contract_accepts_only_opt1_primary(self):
        spec, historical = load_experiment(
            TOOLS_ROOT / "experiments.json", "yolo11s_640"
        )
        _, primary = load_experiment(
            TOOLS_ROOT / "experiments.json", "yolo11s_640_cls_pw05_opt1"
        )
        _, baseline = load_experiment(
            TOOLS_ROOT / "experiments.json", "yolo11n_640_baseline"
        )
        contract = validate_formal_test_contract(
            spec, "yolo11s_640_cls_pw05_opt1", primary
        )
        self.assertEqual("yolo11s_640_cls_pw05_opt1", contract["variant"])
        with self.assertRaisesRegex(ValueError, "validation-only"):
            validate_formal_test_contract(spec, "yolo11s_640", historical)
        with self.assertRaisesRegex(ValueError, "validation-only"):
            validate_formal_test_contract(
                spec, "yolo11n_640_baseline", baseline
            )

    def test_formal_cli_rejects_weight_contract_and_receipt_bypasses(self):
        spec, primary = load_experiment(
            DEFAULT_EXPERIMENTS, "yolo11s_640_cls_pw05_opt1"
        )
        base = Namespace(
            experiments=DEFAULT_EXPERIMENTS,
            variant="yolo11s_640_cls_pw05_opt1",
            weights=None,
            test_receipt=None,
            imgsz=640,
        )
        _, receipt = _formal_cli_contract(base, spec, primary)
        self.assertEqual(
            (REPOSITORY_ROOT / spec["formal_test"]["receipt"]).resolve(), receipt
        )

        with self.assertRaisesRegex(ValueError, "forbids --weights"):
            _formal_cli_contract(
                Namespace(**{**vars(base), "weights": Path("copied-best.pt")}),
                spec,
                primary,
            )
        with self.assertRaisesRegex(ValueError, "dataset-level"):
            _formal_cli_contract(
                Namespace(**{**vars(base), "test_receipt": Path("fresh-receipt.json")}),
                spec,
                primary,
            )
        with self.assertRaisesRegex(ValueError, "pinned experiments"):
            _formal_cli_contract(
                Namespace(**{**vars(base), "experiments": Path("copied.json")}),
                spec,
                primary,
            )
        with self.assertRaisesRegex(ValueError, "imgsz is fixed"):
            _formal_cli_contract(
                Namespace(**{**vars(base), "imgsz": 1280}),
                spec,
                primary,
            )


class TrainingLauncherTest(unittest.TestCase):
    def test_training_command_pins_the_contracted_output_directory(self):
        command = _training_command(
            "/opt/strawberry_venv/bin/yolo",
            Path("/repo/config.yaml"),
            Path("/repo/outputs/perception/yolo11s_640/weights/best.pt"),
        )
        self.assertEqual(
            command,
            [
                "/opt/strawberry_venv/bin/yolo",
                "detect",
                "train",
                "cfg=/repo/config.yaml",
                "project=/repo/outputs/perception",
                "name=yolo11s_640",
                "exist_ok=False",
            ],
        )

    @mock.patch("run_training.shutil.which")
    def test_yolo_falls_back_to_active_virtualenv_bin(self, which):
        which.side_effect = [None, "/opt/strawberry_venv/bin/yolo"]
        self.assertEqual(
            _resolve_yolo_executable("yolo"),
            "/opt/strawberry_venv/bin/yolo",
        )
        self.assertEqual(which.call_count, 2)
        self.assertEqual(which.call_args_list[0], mock.call("yolo"))
        self.assertEqual(
            which.call_args_list[1],
            mock.call("yolo", path=str(Path(sys.executable).parent)),
        )


if __name__ == "__main__":
    unittest.main()
