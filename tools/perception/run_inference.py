#!/usr/bin/env python3
"""Export validation predictions or claim and evaluate the held-out test once."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path, PurePosixPath
import time
from typing import Any, Mapping, Sequence

from workflow import (
    FORMAL_NMS_IOU,
    ImageDetections,
    Prediction,
    acquire_one_time_claim,
    collect_environment,
    evaluate_detection_metrics,
    finalize_one_time_claim,
    load_experiment,
    load_prediction_bundle,
    load_split_samples,
    load_validation_inference_contract,
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
    verify_materialized_dataset,
    write_json_exclusive,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_EXPERIMENTS = Path(__file__).resolve().with_name("experiments.json")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", required=True)
    parser.add_argument("--split", choices=("val", "test"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--experiments", type=Path, default=DEFAULT_EXPERIMENTS)
    parser.add_argument("--weights", type=Path)
    parser.add_argument("--frozen-threshold", type=Path)
    parser.add_argument("--test-receipt", type=Path)
    parser.add_argument("--raw-confidence", type=float, default=0.001)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="0")
    return parser.parse_args(argv)


def _repo_path(value: object) -> Path:
    return (REPOSITORY_ROOT / str(value)).resolve()


def _validation_config_contract(
    spec: Mapping[str, object], variant: Mapping[str, object]
) -> tuple[Path, Mapping[str, object]]:
    value = variant.get("validation_config")
    if not isinstance(value, str) or not value:
        raise ValueError("experiment variant has no validation config")
    path = _repo_path(value)
    if not path.is_file():
        raise FileNotFoundError(path)
    contract = load_validation_inference_contract(path)
    if contract["model"] != variant.get("trained_weight"):
        raise ValueError("validation config model does not match the contracted weight")
    if contract["data"] != spec.get("dataset_yaml"):
        raise ValueError("validation config data does not match the contracted dataset")
    if not math.isclose(
        float(contract["nms_iou"]), FORMAL_NMS_IOU, rel_tol=0.0, abs_tol=1e-12
    ):
        raise ValueError(f"formal validation NMS IoU is fixed at {FORMAL_NMS_IOU}")
    return path, contract


def _validate_cli_against_validation_config(
    args: argparse.Namespace, contract: Mapping[str, object]
) -> None:
    expected = {
        "raw_confidence": float(contract["raw_confidence"]),
        "imgsz": int(contract["imgsz"]),
        "batch": int(contract["batch"]),
        "device": str(contract["device"]),
    }
    actual = {
        "raw_confidence": float(args.raw_confidence),
        "imgsz": int(args.imgsz),
        "batch": int(args.batch),
        "device": str(args.device),
    }
    if actual != expected:
        raise ValueError(
            "formal inference CLI does not match the validation config: "
            f"expected {expected}, got {actual}"
        )


def _formal_cli_contract(
    args: argparse.Namespace,
    spec: Mapping[str, object],
    variant: Mapping[str, object],
) -> tuple[Mapping[str, object], Path]:
    if args.experiments.resolve() != DEFAULT_EXPERIMENTS.resolve():
        raise ValueError("formal test requires the repository's pinned experiments.json")
    contract = validate_formal_test_contract(spec, args.variant, variant)
    if args.weights is not None:
        raise ValueError(
            "formal test forbids --weights; it uses the trained weight fixed by the contract"
        )
    if args.imgsz != contract["imgsz"]:
        raise ValueError(
            f"formal test imgsz is fixed by the contract at {contract['imgsz']}"
        )
    receipt = _repo_path(contract["receipt"])
    if args.test_receipt is not None and args.test_receipt.resolve() != receipt:
        raise ValueError(
            f"formal test receipt is dataset-level and fixed by the contract: {receipt}"
        )
    return contract, receipt


def _frozen_validation_bundle_path(freeze: Mapping[str, object]) -> Path:
    bindings = freeze.get("bindings")
    if not isinstance(bindings, dict):
        raise ValueError("formal threshold has no bindings")
    value = bindings.get("validation_bundle")
    if not isinstance(value, str) or not value:
        raise ValueError("formal threshold has no validation-bundle path")
    relative = PurePosixPath(value.replace("\\", "/"))
    if (
        relative.is_absolute()
        or not relative.parts
        or ".." in relative.parts
        or relative.parts[0].endswith(":")
    ):
        raise ValueError("formal threshold validation-bundle path must be repository-relative")
    root = REPOSITORY_ROOT.resolve()
    resolved = root.joinpath(*relative.parts).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError(
            "formal threshold validation-bundle path escapes the repository"
        ) from error
    return resolved


def _validate_formal_threshold_source(
    freeze: Mapping[str, object],
    metric: Mapping[str, object],
    formal_contract: Mapping[str, object],
    current_bindings: Mapping[str, object],
) -> None:
    """Re-open validation provenance and reproduce the frozen selection."""

    freeze_bindings = freeze.get("bindings")
    if not isinstance(freeze_bindings, dict):
        raise ValueError("formal threshold has no bindings")
    validation_bundle_path = _frozen_validation_bundle_path(freeze)
    if not validation_bundle_path.is_file():
        raise FileNotFoundError(validation_bundle_path)
    expected_sha256 = freeze_bindings.get("validation_bundle_sha256")
    if sha256_file(validation_bundle_path) != expected_sha256:
        raise ValueError("formal threshold validation-bundle digest does not match")
    validation_bundle, validation_records = load_prediction_bundle(
        validation_bundle_path, expected_split="val"
    )
    validate_formal_validation_bundle(
        validation_bundle,
        variant=str(current_bindings["variant"]),
        expected_bindings=current_bindings,
        expected_imgsz=int(formal_contract["imgsz"]),
    )
    split_contract = current_bindings.get("split_contract")
    if not isinstance(split_contract, dict):
        raise ValueError("formal validation bindings have no split contract")
    counts = split_contract.get("counts")
    if not isinstance(counts, dict):
        raise ValueError("formal validation split contract has no counts")
    validate_validation_records_against_split(
        validation_records,
        _repo_path(current_bindings["split_manifest"]),
        class_ids=tuple(int(value) for value in metric["class_ids"]),
        expected_count=int(counts["val"]),
    )

    grid = metric.get("threshold_grid")
    if not isinstance(grid, dict):
        raise ValueError("experiment has no threshold grid")
    candidates = threshold_grid(
        float(grid["start"]), float(grid["stop"]), float(grid["step"])
    )
    inference = validation_bundle.get("inference")
    if not isinstance(inference, dict):
        raise ValueError("formal validation bundle has no inference settings")
    raw_confidence = float(inference.get("raw_confidence", 1.0))
    if raw_confidence > min(candidates):
        raise ValueError(
            "formal validation predictions were filtered above the threshold-search floor"
        )
    class_ids = tuple(int(value) for value in metric["class_ids"])
    selection = select_validation_threshold(
        validation_records,
        candidates,
        iou_threshold=float(metric["iou_threshold"]),
        class_ids=class_ids,
    )
    if freeze.get("selection") != selection:
        raise ValueError(
            "formal threshold selection cannot be reproduced from its validation bundle"
        )
    if sha256_file(validation_bundle_path) != expected_sha256:
        raise ValueError("formal validation bundle changed while it was being verified")


def _dataset_bindings(
    report: Mapping[str, object],
    spec: Mapping[str, object],
    dataset_yaml: Path,
) -> Mapping[str, object]:
    if report.get("dataset_id") != spec.get("dataset_id"):
        raise ValueError("materialized dataset id does not match the experiment contract")
    dataset_yaml_sha256 = sha256_file(dataset_yaml)
    if report.get("dataset_yaml_sha256") != dataset_yaml_sha256:
        raise ValueError("materialized dataset.yaml does not match its verified digest")
    split_contract = validate_split_contract(spec, REPOSITORY_ROOT, report)
    return {
        "canonical_dataset_sha256": report["canonical_dataset_sha256"],
        "canonical_split_sha256": report["canonical_split_sha256"],
        "split_contract": split_contract,
        "dataset_yaml_sha256": dataset_yaml_sha256,
        "verified_sample_count": report["sample_count"],
        "verified_file_count": report["verified_file_count"],
    }


def _bind_completed_training_receipt(
    spec: Mapping[str, object],
    variant_name: str,
    variant: Mapping[str, object],
    weights_sha256: str,
    bindings: Mapping[str, object],
) -> Mapping[str, object]:
    """Bind inference to the completed formal-training output.

    Default validation and formal test both use the contracted ``trained_weight``.
    They must therefore reject a transient or otherwise unrecorded ``best.pt``
    before any samples are loaded or the model is run.  Validation with an
    explicit ``--weights`` override remains a separate diagnostic workflow.
    """

    training_receipt_value = variant.get("training_receipt")
    if not isinstance(training_receipt_value, str) or not training_receipt_value:
        raise ValueError("experiment variant has no formal training receipt")
    training_receipt_path = _repo_path(training_receipt_value)
    if not training_receipt_path.is_file():
        raise FileNotFoundError(training_receipt_path)
    training_receipt = read_json(training_receipt_path)
    canonical_dataset_sha256 = validate_completed_training_receipt(
        training_receipt,
        variant=variant_name,
        dataset_id=str(spec["dataset_id"]),
        expected_trained_weight=str(variant["trained_weight"]),
        weights_sha256=weights_sha256,
        training_config_sha256=str(bindings["training_config_sha256"]),
        experiments_sha256=str(bindings["experiments_sha256"]),
        expected_split_contract=bindings["split_contract"],
    )
    if canonical_dataset_sha256 != bindings["canonical_dataset_sha256"]:
        raise ValueError(
            "current materialized dataset differs from the formally trained dataset"
        )
    return {
        **bindings,
        "training_receipt": training_receipt_value,
        "training_receipt_sha256": sha256_file(training_receipt_path),
    }


def _result_predictions(result: Any, class_ids: Sequence[int]) -> tuple[Prediction, ...]:
    if result.boxes is None:
        return ()
    boxes = result.boxes.xyxyn.detach().cpu().tolist()
    confidences = result.boxes.conf.detach().cpu().tolist()
    classes = result.boxes.cls.detach().cpu().tolist()
    allowed = frozenset(class_ids)
    predictions = []
    for bbox, confidence, class_value in zip(boxes, confidences, classes):
        class_float = float(class_value)
        if not class_float.is_integer() or int(class_float) not in allowed:
            raise ValueError(f"model emitted class outside the v1 contract: {class_value}")
        clipped_bbox = _clip_normalized_bbox(bbox)
        if clipped_bbox is None:
            # Ultralytics can emit a box whose edge differs from the image
            # boundary only by floating-point round-off.  A box that collapses
            # after clipping has no measurable image area and must not enter
            # the metric matching set.
            continue
        predictions.append(
            Prediction(int(class_float), float(confidence), clipped_bbox)
        )
    return tuple(predictions)


def _clip_normalized_bbox(bbox: Sequence[object]) -> tuple[float, float, float, float] | None:
    """Clip one normalized ``xyxy`` box and discard an empty intersection."""

    if len(bbox) != 4:
        raise ValueError("model prediction bbox must contain four coordinates")
    coordinates = tuple(float(value) for value in bbox)
    if not all(math.isfinite(value) for value in coordinates):
        raise ValueError("model prediction bbox contains a non-finite coordinate")
    x1, y1, x2, y2 = (
        min(1.0, max(0.0, value)) for value in coordinates
    )
    if x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2, y2)


def _run_model(
    weights: Path,
    samples: Sequence[Any],
    class_ids: Sequence[int],
    raw_confidence: float,
    nms_iou: float,
    imgsz: int,
    batch_size: int,
    device: str,
) -> tuple[ImageDetections, ...]:
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise RuntimeError("formal inference requires the pinned Ultralytics environment") from error

    model = YOLO(str(weights))
    records = []
    for offset in range(0, len(samples), batch_size):
        batch = samples[offset : offset + batch_size]
        started = time.perf_counter()
        results = model.predict(
            source=[str(item.image_path) for item in batch],
            imgsz=imgsz,
            conf=raw_confidence,
            iou=nms_iou,
            device=device,
            batch=batch_size,
            verbose=False,
            stream=False,
        )
        elapsed_per_image_ms = (time.perf_counter() - started) * 1000.0 / len(batch)
        if len(results) != len(batch):
            raise RuntimeError("Ultralytics returned a different number of results than inputs")
        for sample, result in zip(batch, results):
            speed = result.speed if isinstance(result.speed, dict) else {}
            inference_ms = float(speed.get("inference", elapsed_per_image_ms))
            reported = [
                float(speed[name])
                for name in ("preprocess", "inference", "postprocess")
                if speed.get(name) is not None
            ]
            total_ms = sum(reported) if reported else elapsed_per_image_ms
            records.append(
                ImageDetections(
                    sample.image_key,
                    load_yolo_ground_truth(sample.label_path, class_ids),
                    _result_predictions(result, class_ids),
                    inference_ms=inference_ms,
                    total_ms=total_ms,
                )
            )
    return tuple(records)


def _replay_formal_validation_before_test_claim(
    *,
    freeze: Mapping[str, object],
    metric: Mapping[str, object],
    weights: Path,
    split_manifest: Path,
    validation_contract: Mapping[str, object],
    expected_count: int,
) -> Mapping[str, object]:
    """Re-run the current model on validation before consuming the test once.

    Hash-linked JSON alone cannot prove that stored predictions came from the
    contracted weight.  This replay is deliberately performed before the
    held-out receipt exists and must reproduce the complete frozen selection.
    """

    class_ids = tuple(int(value) for value in metric["class_ids"])
    samples = load_split_samples(split_manifest, "val")
    if len(samples) != expected_count:
        raise ValueError(
            "fresh validation replay count does not match the split contract"
        )
    records = _run_model(
        weights,
        samples,
        class_ids,
        float(validation_contract["raw_confidence"]),
        float(validation_contract["nms_iou"]),
        int(validation_contract["imgsz"]),
        int(validation_contract["batch"]),
        str(validation_contract["device"]),
    )
    validate_validation_records_against_split(
        records,
        split_manifest,
        class_ids=class_ids,
        expected_count=expected_count,
    )
    grid = metric.get("threshold_grid")
    if not isinstance(grid, dict):
        raise ValueError("experiment has no threshold grid")
    selection = select_validation_threshold(
        records,
        threshold_grid(float(grid["start"]), float(grid["stop"]), float(grid["step"])),
        iou_threshold=float(metric["iou_threshold"]),
        class_ids=class_ids,
    )
    if freeze.get("selection") != selection:
        raise ValueError(
            "fresh validation replay does not reproduce the frozen selection"
        )
    macro_f1 = float(selection["validation_metrics"]["macro_f1"])
    required = float(metric["macro_f1_min"])
    if not math.isfinite(macro_f1) or macro_f1 < required:
        raise ValueError(
            "fresh validation replay does not authorize the held-out test: "
            f"macro_f1={macro_f1}, required={required}"
        )
    return selection


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not 0.0 <= args.raw_confidence <= 1.0:
        raise ValueError("raw confidence must be in [0, 1]")
    if args.imgsz <= 0 or args.batch <= 0:
        raise ValueError("imgsz and batch must be positive")
    if args.output.exists():
        raise FileExistsError(args.output)

    if args.split == "test" and args.experiments.resolve() != DEFAULT_EXPERIMENTS.resolve():
        raise ValueError("formal test requires the repository's pinned experiments.json")
    spec, variant = load_experiment(args.experiments, args.variant)
    metric = spec.get("metric")
    if not isinstance(metric, dict):
        raise ValueError("experiment metric contract is missing")
    class_ids = tuple(int(value) for value in metric["class_ids"])
    iou_threshold = float(metric["iou_threshold"])
    split_manifest = _repo_path(spec["split_manifest"])
    dataset_yaml = _repo_path(spec["dataset_yaml"])
    training_config = _repo_path(variant["training_config"])
    validation_config, validation_contract = _validation_config_contract(spec, variant)
    receipt = None
    formal_contract = None
    if args.split == "test":
        formal_contract, receipt = _formal_cli_contract(args, spec, variant)
        weights = _repo_path(variant["trained_weight"])
    else:
        weights = (args.weights or _repo_path(variant["trained_weight"])).resolve()
    if args.weights is None:
        _validate_cli_against_validation_config(args, validation_contract)
    for path in (
        split_manifest,
        dataset_yaml,
        training_config,
        validation_config,
        weights,
    ):
        if not path.is_file():
            raise FileNotFoundError(path)
    weights_sha256 = sha256_file(weights)
    split_manifest_sha256 = sha256_file(split_manifest)
    bindings = {
        "variant": args.variant,
        "role": variant.get("role"),
        "dataset_id": spec.get("dataset_id"),
        "experiments_sha256": sha256_file(args.experiments),
        "training_config": str(variant["training_config"]),
        "training_config_sha256": sha256_file(training_config),
        "validation_config": str(variant["validation_config"]),
        "validation_config_sha256": sha256_file(validation_config),
        "validation_task": validation_contract["task"],
        "validation_mode": validation_contract["mode"],
        "validation_split": validation_contract["split"],
        "validation_imgsz": validation_contract["imgsz"],
        "validation_batch": validation_contract["batch"],
        "validation_device": validation_contract["device"],
        "validation_raw_confidence": validation_contract["raw_confidence"],
        "validation_nms_iou": validation_contract["nms_iou"],
        "dataset_yaml": str(spec["dataset_yaml"]),
        "split_manifest": str(spec["split_manifest"]),
        "split_manifest_sha256": split_manifest_sha256,
        "weights": (
            str(args.weights.resolve())
            if args.weights is not None
            else str(variant["trained_weight"])
        ),
        "weights_source": (
            "explicit_validation_override"
            if args.weights is not None
            else "expected_trained_weight"
        ),
        "weights_sha256": weights_sha256,
    }

    # The complete split/curation contract is verified for validation and test
    # before a formal one-time claim can be created.
    dataset_report = verify_materialized_dataset(split_manifest)
    verified_bindings = _dataset_bindings(dataset_report, spec, dataset_yaml)
    bindings = {**bindings, **verified_bindings}
    if args.split == "test" or args.weights is None:
        bindings = _bind_completed_training_receipt(
            spec,
            args.variant,
            variant,
            weights_sha256,
            bindings,
        )

    threshold = None
    freeze_sha256 = None
    token = None
    samples = None
    if args.split == "test":
        if args.frozen_threshold is None:
            raise ValueError("test inference requires --frozen-threshold")
        assert formal_contract is not None and receipt is not None
        freeze = read_json(args.frozen_threshold)
        threshold = validate_frozen_threshold(
            freeze,
            args.variant,
            weights_sha256,
            str(bindings["canonical_dataset_sha256"]),
            training_config_sha256=bindings["training_config_sha256"],
            experiments_sha256=bindings["experiments_sha256"],
            iou_threshold=iou_threshold,
            class_ids=class_ids,
            expected_split_contract=bindings["split_contract"],
            expected_formal_validation_bindings=bindings,
            expected_validation_imgsz=int(formal_contract["imgsz"]),
        )
        _validate_formal_threshold_source(
            freeze,
            metric,
            formal_contract,
            bindings,
        )
        promotion_path = optimization_promotion_artifact_path(
            spec, REPOSITORY_ROOT
        )
        if promotion_path is not None:
            if not promotion_path.is_file():
                raise FileNotFoundError(
                    "optimized primary has no validation-promotion artifact: "
                    f"{promotion_path}"
                )
            try:
                frozen_threshold_relative = args.frozen_threshold.resolve().relative_to(
                    REPOSITORY_ROOT.resolve()
                ).as_posix()
            except ValueError as error:
                raise ValueError(
                    "optimized primary threshold freeze must be inside the repository"
                ) from error
            freeze_bindings = freeze.get("bindings")
            assert isinstance(freeze_bindings, dict)
            freeze_selection = freeze.get("selection")
            if not isinstance(freeze_selection, dict):
                raise ValueError("optimized primary freeze has no validation selection")
            freeze_metrics = freeze_selection.get("validation_metrics")
            if not isinstance(freeze_metrics, dict):
                raise ValueError("optimized primary freeze has no validation metrics")
            freeze_per_class = freeze_metrics.get("per_class")
            if not isinstance(freeze_per_class, dict):
                raise ValueError("optimized primary freeze has no per-class metrics")
            expected_promotion_metrics = {
                "macro_f1": float(freeze_metrics["macro_f1"]),
                "ripe_f1": float(freeze_per_class["0"]["f1"]),
                "unripe_f1": float(freeze_per_class["1"]["f1"]),
            }
            promotion = read_json(promotion_path)
            validate_optimization_promotion(
                promotion,
                spec,
                experiments_sha256=str(bindings["experiments_sha256"]),
                frozen_threshold_path=frozen_threshold_relative,
                frozen_threshold_sha256=sha256_file(args.frozen_threshold),
                weights_sha256=weights_sha256,
                validation_bundle=str(freeze_bindings["validation_bundle"]),
                validation_bundle_sha256=str(
                    freeze_bindings["validation_bundle_sha256"]
                ),
                canonical_dataset_sha256=str(bindings["canonical_dataset_sha256"]),
                training_config_sha256=str(bindings["training_config_sha256"]),
                validation_config=str(bindings["validation_config"]),
                validation_config_sha256=str(bindings["validation_config_sha256"]),
                validation_nms_iou=float(bindings["validation_nms_iou"]),
                expected_candidate_validation_metrics=expected_promotion_metrics,
            )
            bindings = {
                **bindings,
                "optimization_promotion": promotion_path.relative_to(
                    REPOSITORY_ROOT.resolve()
                ).as_posix(),
                "optimization_promotion_sha256": sha256_file(promotion_path),
            }
        if args.raw_confidence > threshold:
            raise ValueError("raw inference confidence exceeds the frozen threshold")
        split_contract_value = bindings.get("split_contract")
        if not isinstance(split_contract_value, dict) or not isinstance(
            split_contract_value.get("counts"), dict
        ):
            raise ValueError("formal bindings have no validation split count")
        replay = _replay_formal_validation_before_test_claim(
            freeze=freeze,
            metric=metric,
            weights=weights,
            split_manifest=split_manifest,
            validation_contract=validation_contract,
            expected_count=int(split_contract_value["counts"]["val"]),
        )
        if sha256_file(weights) != weights_sha256:
            raise ValueError("trained weight changed during validation replay")
        if sha256_file(validation_config) != bindings["validation_config_sha256"]:
            raise ValueError("validation config changed during validation replay")
        bindings = {**bindings, "preclaim_validation_replay": replay}
        freeze_sha256 = sha256_file(args.frozen_threshold)
        if receipt.resolve() == args.output.resolve():
            raise ValueError("test receipt and prediction bundle must be different files")
        token = acquire_one_time_claim(
            receipt,
            f"held_out_test:{spec['dataset_id']}",
            {
                **bindings,
                "frozen_threshold_sha256": freeze_sha256,
                "confidence_threshold": threshold,
                "output": str(args.output.resolve()),
            },
            utc_now(),
        )
    else:
        samples = load_split_samples(split_manifest, args.split)

    try:
        if args.split == "test":
            dataset_report = verify_materialized_dataset(split_manifest)
            verified_bindings = _dataset_bindings(dataset_report, spec, dataset_yaml)
            if verified_bindings["split_contract"] != bindings["split_contract"]:
                raise ValueError(
                    "split contract changed after the formal test claim"
                )
            bindings = {**bindings, **verified_bindings}
            samples = load_split_samples(split_manifest, args.split)
        assert samples is not None
        records = _run_model(
            weights,
            samples,
            class_ids,
            args.raw_confidence,
            float(validation_contract["nms_iou"]),
            args.imgsz,
            args.batch,
            args.device,
        )
        bundle = {
            "schema_version": 1,
            "kind": "prediction_bundle",
            "created_utc": utc_now(),
            "variant": args.variant,
            "role": variant.get("role"),
            "split": args.split,
            "bindings": bindings,
            "inference": {
                "raw_confidence": args.raw_confidence,
                "imgsz": args.imgsz,
                "batch": args.batch,
                "device": args.device,
                "nms_iou": float(validation_contract["nms_iou"]),
            },
            "speed": summarize_speed(records),
            "environment": collect_environment(),
            "records": [record_to_json(item) for item in records],
        }
        write_json_exclusive(args.output, bundle)
        summary: Mapping[str, object] = {
            "bundle": str(args.output),
            "bundle_sha256": sha256_file(args.output),
            "split": args.split,
            "image_count": len(records),
            "speed": bundle["speed"],
        }
        exit_code = 0
        if args.split == "test":
            assert threshold is not None and token is not None and receipt is not None
            metrics = evaluate_detection_metrics(
                records, threshold, iou_threshold=iou_threshold, class_ids=class_ids
            )
            macro_f1_min = float(metric["macro_f1_min"])
            macro_f1 = float(metrics["macro_f1"])
            acceptance = {
                "metric": "macro_f1",
                "operator": ">=",
                "required": macro_f1_min,
                "actual": macro_f1,
                "passed": macro_f1 >= macro_f1_min,
            }
            summary = {
                **summary,
                "fixed_threshold_metrics": metrics,
                "acceptance": acceptance,
            }
            finalize_one_time_claim(
                receipt, token, "completed", utc_now(), summary
            )
            if not acceptance["passed"]:
                exit_code = 2
        print(json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True))
        return exit_code
    except BaseException as error:
        if token is not None and receipt is not None:
            try:
                finalize_one_time_claim(
                    receipt,
                    token,
                    "failed",
                    utc_now(),
                    {"error_type": type(error).__name__, "error": str(error)},
                )
            except Exception:
                pass
        raise


if __name__ == "__main__":
    raise SystemExit(main())
