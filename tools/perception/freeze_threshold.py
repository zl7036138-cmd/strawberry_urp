#!/usr/bin/env python3
"""Freeze a detector confidence threshold using validation data only."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
from typing import Mapping, Sequence

from workflow import (
    FORMAL_NMS_IOU,
    FORMAL_VALIDATION_BINDING_FIELDS,
    METRIC_NAME,
    load_experiment,
    load_prediction_bundle,
    load_validation_inference_contract,
    read_json,
    select_validation_threshold,
    sha256_file,
    threshold_grid,
    validate_completed_training_receipt,
    validate_formal_test_contract,
    validate_formal_validation_bundle,
    validate_split_contract,
    validate_validation_records_against_split,
    verify_materialized_dataset,
    write_json_exclusive,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_EXPERIMENTS = Path(__file__).resolve().with_name("experiments.json")


def _repo_path(value: object) -> Path:
    return (REPOSITORY_ROOT / str(value)).resolve()


def _repository_relative(path: Path, artifact_name: str) -> str:
    root = REPOSITORY_ROOT.resolve()
    resolved = path.resolve()
    try:
        return resolved.relative_to(root).as_posix()
    except ValueError as error:
        raise ValueError(
            f"formal {artifact_name} must be stored inside the repository"
        ) from error


def _formal_expected_bindings(
    spec: Mapping[str, object],
    variant_name: str,
    variant: Mapping[str, object],
    experiments: Path,
) -> tuple[Mapping[str, object], Mapping[str, object]]:
    """Rebuild trusted bindings instead of trusting the validation JSON."""

    contract = validate_formal_test_contract(spec, variant_name, variant)
    split_manifest = _repo_path(spec["split_manifest"])
    dataset_yaml = _repo_path(spec["dataset_yaml"])
    training_config = _repo_path(variant["training_config"])
    validation_config_value = variant.get("validation_config")
    if not isinstance(validation_config_value, str) or not validation_config_value:
        raise ValueError("formal primary variant has no validation config")
    validation_config = _repo_path(validation_config_value)
    trained_weight = _repo_path(variant["trained_weight"])
    training_receipt_value = variant.get("training_receipt")
    if not isinstance(training_receipt_value, str) or not training_receipt_value:
        raise ValueError("formal primary variant has no training receipt")
    training_receipt = _repo_path(training_receipt_value)
    for path in (
        split_manifest,
        dataset_yaml,
        training_config,
        validation_config,
        trained_weight,
        training_receipt,
    ):
        if not path.is_file():
            raise FileNotFoundError(path)

    dataset_report = verify_materialized_dataset(split_manifest)
    if dataset_report.get("dataset_id") != spec.get("dataset_id"):
        raise ValueError("materialized dataset id does not match the experiment contract")
    dataset_yaml_sha256 = sha256_file(dataset_yaml)
    if dataset_report.get("dataset_yaml_sha256") != dataset_yaml_sha256:
        raise ValueError("materialized dataset.yaml does not match its verified digest")
    split_contract = validate_split_contract(spec, REPOSITORY_ROOT, dataset_report)
    weights_sha256 = sha256_file(trained_weight)
    training_config_sha256 = sha256_file(training_config)
    validation_contract = load_validation_inference_contract(validation_config)
    if validation_contract["model"] != variant.get("trained_weight"):
        raise ValueError("validation config model does not match the trained weight")
    if validation_contract["data"] != spec.get("dataset_yaml"):
        raise ValueError("validation config data does not match the dataset")
    if float(validation_contract["nms_iou"]) != FORMAL_NMS_IOU:
        raise ValueError(f"formal validation NMS IoU is fixed at {FORMAL_NMS_IOU}")
    experiments_sha256 = sha256_file(experiments)
    training_receipt_value_json = read_json(training_receipt)
    receipt_dataset_sha256 = validate_completed_training_receipt(
        training_receipt_value_json,
        variant=variant_name,
        dataset_id=str(spec["dataset_id"]),
        expected_trained_weight=str(variant["trained_weight"]),
        weights_sha256=weights_sha256,
        training_config_sha256=training_config_sha256,
        experiments_sha256=experiments_sha256,
        expected_split_contract=split_contract,
    )
    if receipt_dataset_sha256 != dataset_report["canonical_dataset_sha256"]:
        raise ValueError(
            "current materialized dataset differs from the formally trained dataset"
        )

    expected = {
        "variant": variant_name,
        "role": variant.get("role"),
        "dataset_id": spec.get("dataset_id"),
        "weights": str(variant["trained_weight"]),
        "weights_source": "expected_trained_weight",
        "weights_sha256": weights_sha256,
        "training_receipt": training_receipt_value,
        "training_receipt_sha256": sha256_file(training_receipt),
        "training_config": str(variant["training_config"]),
        "training_config_sha256": training_config_sha256,
        "validation_config": validation_config_value,
        "validation_config_sha256": sha256_file(validation_config),
        "validation_task": validation_contract["task"],
        "validation_mode": validation_contract["mode"],
        "validation_split": validation_contract["split"],
        "validation_imgsz": validation_contract["imgsz"],
        "validation_batch": validation_contract["batch"],
        "validation_device": validation_contract["device"],
        "validation_raw_confidence": validation_contract["raw_confidence"],
        "validation_nms_iou": validation_contract["nms_iou"],
        "experiments_sha256": experiments_sha256,
        "dataset_yaml": str(spec["dataset_yaml"]),
        "dataset_yaml_sha256": dataset_yaml_sha256,
        "split_manifest": str(spec["split_manifest"]),
        "split_manifest_sha256": sha256_file(split_manifest),
        "canonical_dataset_sha256": dataset_report["canonical_dataset_sha256"],
        "canonical_split_sha256": dataset_report["canonical_split_sha256"],
        "split_contract": split_contract,
    }
    return contract, expected


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", required=True)
    parser.add_argument("--validation-bundle", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--experiments", type=Path, default=DEFAULT_EXPERIMENTS)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    spec, variant = load_experiment(args.experiments, args.variant)
    bundle, records = load_prediction_bundle(args.validation_bundle, expected_split="val")
    if bundle.get("variant") != args.variant:
        raise ValueError("validation bundle variant does not match")
    bindings = bundle.get("bindings")
    if not isinstance(bindings, dict):
        raise ValueError("validation bundle has no immutable artifact bindings")

    metric = spec.get("metric")
    if not isinstance(metric, dict) or metric.get("name") != METRIC_NAME:
        raise ValueError("experiment metric contract is invalid")
    grid = metric.get("threshold_grid")
    if not isinstance(grid, dict):
        raise ValueError("experiment has no threshold grid")
    candidates = threshold_grid(
        float(grid["start"]), float(grid["stop"]), float(grid["step"])
    )
    inference = bundle.get("inference")
    if not isinstance(inference, dict):
        raise ValueError("validation bundle has no inference settings")
    raw_confidence = float(inference.get("raw_confidence", 1.0))
    if raw_confidence > min(candidates):
        raise ValueError(
            "validation predictions were filtered above the threshold-search floor"
        )
    class_ids = tuple(int(value) for value in metric["class_ids"])
    iou_threshold = float(metric["iou_threshold"])
    selection = select_validation_threshold(
        records, candidates, iou_threshold=iou_threshold, class_ids=class_ids
    )
    required_bindings = (
        "experiments_sha256",
        "weights_sha256",
        "canonical_dataset_sha256",
        "canonical_split_sha256",
        "training_config_sha256",
        "split_contract",
    )
    missing = [name for name in required_bindings if name not in bindings]
    if missing:
        raise ValueError(f"validation bundle bindings are missing: {', '.join(missing)}")
    experiments_sha256 = sha256_file(args.experiments)
    if bindings["experiments_sha256"] != experiments_sha256:
        raise ValueError("validation bundle is bound to a different experiment contract")

    formal_contract = None
    formal_expected = None
    formal_test = spec.get("formal_test")
    if isinstance(formal_test, dict) and formal_test.get("variant") == args.variant:
        formal_contract, formal_expected = _formal_expected_bindings(
            spec, args.variant, variant, args.experiments
        )
        validate_formal_validation_bundle(
            bundle,
            variant=args.variant,
            expected_bindings=formal_expected,
            expected_imgsz=int(formal_contract["imgsz"]),
        )
        counts = formal_expected["split_contract"].get("counts")
        if not isinstance(counts, dict):
            raise ValueError("formal split contract has no validation count")
        validate_validation_records_against_split(
            records,
            _repo_path(spec["split_manifest"]),
            class_ids=class_ids,
            expected_count=int(counts["val"]),
        )

    output_bindings = {
        "validation_bundle_sha256": sha256_file(args.validation_bundle),
        "weights_sha256": bindings["weights_sha256"],
        "canonical_dataset_sha256": bindings["canonical_dataset_sha256"],
        "canonical_split_sha256": bindings["canonical_split_sha256"],
        "split_contract": bindings["split_contract"],
        "split_manifest_sha256": bindings.get("split_manifest_sha256"),
        "training_config_sha256": bindings["training_config_sha256"],
        "experiments_sha256": experiments_sha256,
    }
    if formal_contract is not None and formal_expected is not None:
        output_bindings.update(
            {
                field: formal_expected[field]
                for field in FORMAL_VALIDATION_BINDING_FIELDS
            }
        )
        output_bindings.update(
            {
                "validation_bundle": _repository_relative(
                    args.validation_bundle, "validation bundle"
                ),
                "validation_imgsz": formal_contract["imgsz"],
            }
        )

    output = {
        "schema_version": 1,
        "kind": "frozen_threshold",
        "metric_name": METRIC_NAME,
        "variant": args.variant,
        "frozen_utc": datetime.now(timezone.utc).isoformat(),
        "confidence_threshold": selection["selected_threshold"],
        "iou_threshold": iou_threshold,
        "class_ids": list(class_ids),
        "selection": selection,
        "bindings": output_bindings,
    }
    write_json_exclusive(args.output, output)
    print(
        f"frozen {args.variant} confidence threshold at "
        f"{selection['selected_threshold']:.2f}: {args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
