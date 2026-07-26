"""Dependency-light contracts for reproducible perception experiments.

This module intentionally uses only the Python standard library.  Ultralytics
is imported by the runtime adapter only after all dataset, hash, threshold, and
one-time-test preconditions have passed.
"""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path, PurePosixPath
import platform
import subprocess
import sys
from typing import Iterable, Mapping, Sequence
from uuid import uuid4


SCHEMA_VERSION = 1
METRIC_NAME = "greedy_per_class_detection_macro_f1_v1"
SUPPORTED_SPLITS = frozenset({"val", "test"})
FORMAL_SPLIT_STRATEGY = "seeded_group_stratified_full_resplit"
FORMAL_SPLIT_SEED = 20260710
FORMAL_SPLIT_RATIOS = {"train": 0.70, "val": 0.15, "test": 0.15}
FORMAL_SPLIT_COUNTS = {"train": 501, "val": 116, "test": 115}
FORMAL_SOURCE_IMAGE_COUNT = 813
FORMAL_EXCLUDED_COUNT = 81
FORMAL_RETAINED_COUNT = 732
FORMAL_NMS_IOU = 0.70
SPLIT_CONTRACT_ARTIFACTS = frozenset(
    {
        "groups_csv",
        "group_overrides_csv",
        "exclusions_csv",
        "group_audit",
        "curation_audit",
        "split_manifest",
        "dataset_yaml",
    }
)
FORMAL_VALIDATION_BINDING_FIELDS = (
    "variant",
    "role",
    "dataset_id",
    "weights",
    "weights_source",
    "weights_sha256",
    "training_receipt",
    "training_receipt_sha256",
    "training_config",
    "training_config_sha256",
    "validation_config",
    "validation_config_sha256",
    "validation_task",
    "validation_mode",
    "validation_split",
    "validation_imgsz",
    "validation_batch",
    "validation_device",
    "validation_raw_confidence",
    "validation_nms_iou",
    "experiments_sha256",
    "dataset_yaml",
    "dataset_yaml_sha256",
    "split_manifest",
    "split_manifest_sha256",
    "canonical_dataset_sha256",
    "canonical_split_sha256",
    "split_contract",
)


@dataclass(frozen=True)
class GroundTruth:
    class_id: int
    bbox: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        _validate_class_id(self.class_id)
        _validate_normalized_bbox(self.bbox)


@dataclass(frozen=True)
class Prediction:
    class_id: int
    confidence: float
    bbox: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        _validate_class_id(self.class_id)
        if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("prediction confidence must be finite and in [0, 1]")
        _validate_normalized_bbox(self.bbox)


@dataclass(frozen=True)
class ImageDetections:
    image: str
    ground_truth: tuple[GroundTruth, ...]
    predictions: tuple[Prediction, ...]
    inference_ms: float | None = None
    total_ms: float | None = None

    def __post_init__(self) -> None:
        if not self.image:
            raise ValueError("image key must not be empty")
        for name, value in (
            ("inference_ms", self.inference_ms),
            ("total_ms", self.total_ms),
        ):
            if value is not None and (not math.isfinite(value) or value < 0.0):
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True)
class DatasetSample:
    image_key: str
    image_path: Path
    label_path: Path


def _validate_class_id(class_id: int) -> None:
    if not isinstance(class_id, int) or isinstance(class_id, bool) or class_id < 0:
        raise ValueError("class_id must be a non-negative integer")


def _validate_normalized_bbox(bbox: Sequence[float]) -> None:
    if len(bbox) != 4:
        raise ValueError("bbox must contain x1, y1, x2, y2")
    x1, y1, x2, y2 = bbox
    if not all(math.isfinite(value) for value in bbox):
        raise ValueError("bbox coordinates must be finite")
    if not (0.0 <= x1 < x2 <= 1.0 and 0.0 <= y1 < y2 <= 1.0):
        raise ValueError("bbox must have positive area inside normalized image bounds")


def normalized_xywh_to_xyxy(
    x_center: float, y_center: float, width: float, height: float
) -> tuple[float, float, float, float]:
    values = (x_center, y_center, width, height)
    if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in values):
        raise ValueError("YOLO coordinates must be finite and normalized")
    if width <= 0.0 or height <= 0.0:
        raise ValueError("YOLO width and height must be positive")
    bbox = (
        max(0.0, x_center - width / 2.0),
        max(0.0, y_center - height / 2.0),
        min(1.0, x_center + width / 2.0),
        min(1.0, y_center + height / 2.0),
    )
    _validate_normalized_bbox(bbox)
    return bbox


def intersection_over_union(
    left: Sequence[float], right: Sequence[float]
) -> float:
    _validate_normalized_bbox(left)
    _validate_normalized_bbox(right)
    x1 = max(left[0], right[0])
    y1 = max(left[1], right[1])
    x2 = min(left[2], right[2])
    y2 = min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = (left[2] - left[0]) * (left[3] - left[1])
    right_area = (right[2] - right[0]) * (right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union > 0.0 else 0.0


def _prediction_sort_key(item: Prediction) -> tuple[float, int, tuple[float, ...]]:
    return (-item.confidence, item.class_id, item.bbox)


def evaluate_detection_metrics(
    records: Sequence[ImageDetections],
    confidence_threshold: float,
    iou_threshold: float = 0.5,
    class_ids: Sequence[int] = (0, 1),
) -> Mapping[str, object]:
    """Compute class-macro F1 from deterministic greedy IoU matching.

    Predictions are sorted by descending confidence independently in every
    image and class.  Each prediction can match at most one ground-truth box,
    and each ground-truth box can be used once.  Classes with no ground-truth
    support are rejected so a nominal macro average cannot hide a bad split.
    """

    if not math.isfinite(confidence_threshold) or not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("confidence threshold must be finite and in [0, 1]")
    if not math.isfinite(iou_threshold) or not 0.0 < iou_threshold <= 1.0:
        raise ValueError("IoU threshold must be finite and in (0, 1]")
    normalized_class_ids = tuple(int(value) for value in class_ids)
    if not normalized_class_ids or len(set(normalized_class_ids)) != len(
        normalized_class_ids
    ):
        raise ValueError("class_ids must be non-empty and unique")
    for class_id in normalized_class_ids:
        _validate_class_id(class_id)

    counts = {
        class_id: {"support": 0, "predictions": 0, "tp": 0, "fp": 0, "fn": 0}
        for class_id in normalized_class_ids
    }
    allowed = frozenset(normalized_class_ids)
    for record in records:
        for class_id in normalized_class_ids:
            truths = [item for item in record.ground_truth if item.class_id == class_id]
            predictions = sorted(
                (
                    item
                    for item in record.predictions
                    if item.class_id == class_id
                    and item.confidence >= confidence_threshold
                ),
                key=_prediction_sort_key,
            )
            counts[class_id]["support"] += len(truths)
            counts[class_id]["predictions"] += len(predictions)
            unmatched = set(range(len(truths)))
            for prediction in predictions:
                candidates = [
                    (intersection_over_union(prediction.bbox, truths[index].bbox), index)
                    for index in unmatched
                ]
                best_iou, best_index = max(candidates, default=(0.0, -1))
                if best_iou >= iou_threshold:
                    counts[class_id]["tp"] += 1
                    unmatched.remove(best_index)
                else:
                    counts[class_id]["fp"] += 1
            counts[class_id]["fn"] += len(unmatched)

        unexpected_truths = {
            item.class_id for item in record.ground_truth if item.class_id not in allowed
        }
        unexpected_predictions = {
            item.class_id for item in record.predictions if item.class_id not in allowed
        }
        if unexpected_truths or unexpected_predictions:
            raise ValueError(
                "records contain classes outside the frozen metric contract: "
                f"{sorted(unexpected_truths | unexpected_predictions)}"
            )

    per_class: dict[str, Mapping[str, object]] = {}
    f1_values = []
    for class_id in normalized_class_ids:
        item = counts[class_id]
        if item["support"] == 0:
            raise ValueError(f"class {class_id} has no ground-truth support")
        tp, fp, fn = item["tp"], item["fp"], item["fn"]
        precision = tp / (tp + fp) if tp + fp else 0.0
        recall = tp / (tp + fn) if tp + fn else 0.0
        f1 = 2.0 * tp / (2.0 * tp + fp + fn) if 2 * tp + fp + fn else 0.0
        f1_values.append(f1)
        per_class[str(class_id)] = {
            **item,
            "precision": precision,
            "recall": recall,
            "f1": f1,
        }

    return {
        "metric_name": METRIC_NAME,
        "confidence_threshold": confidence_threshold,
        "iou_threshold": iou_threshold,
        "class_ids": list(normalized_class_ids),
        "image_count": len(records),
        "macro_f1": sum(f1_values) / len(f1_values),
        "per_class": per_class,
    }


def threshold_grid(start: float, stop: float, step: float) -> tuple[float, ...]:
    start_value = Decimal(str(start))
    stop_value = Decimal(str(stop))
    step_value = Decimal(str(step))
    if not (Decimal("0") <= start_value <= stop_value <= Decimal("1")):
        raise ValueError("threshold grid must remain inside [0, 1]")
    if step_value <= 0:
        raise ValueError("threshold step must be positive")
    values = []
    current = start_value
    while current <= stop_value:
        values.append(float(current))
        current += step_value
    if not values:
        raise ValueError("threshold grid is empty")
    return tuple(values)


def select_validation_threshold(
    records: Sequence[ImageDetections],
    candidates: Iterable[float],
    iou_threshold: float = 0.5,
    class_ids: Sequence[int] = (0, 1),
) -> Mapping[str, object]:
    """Select maximum validation macro-F1, preferring the safer high threshold."""

    unique_candidates = sorted({float(value) for value in candidates})
    if not unique_candidates:
        raise ValueError("at least one threshold candidate is required")
    evaluated = [
        evaluate_detection_metrics(records, value, iou_threshold, class_ids)
        for value in unique_candidates
    ]
    best = max(
        evaluated,
        key=lambda item: (float(item["macro_f1"]), float(item["confidence_threshold"])),
    )
    return {
        "selection_rule": "max_macro_f1_then_highest_confidence_threshold",
        "candidate_count": len(evaluated),
        "selected_threshold": best["confidence_threshold"],
        "validation_metrics": best,
    }


def _percentile(values: Sequence[float], probability: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    rank = max(1, math.ceil(probability * len(ordered)))
    return ordered[rank - 1]


def summarize_speed(records: Sequence[ImageDetections]) -> Mapping[str, object]:
    inference = [item.inference_ms for item in records if item.inference_ms is not None]
    total = [item.total_ms for item in records if item.total_ms is not None]
    mean_inference = sum(inference) / len(inference) if inference else None
    mean_total = sum(total) / len(total) if total else None
    return {
        "sample_count": len(records),
        "timed_inference_count": len(inference),
        "inference_ms_mean": mean_inference,
        "inference_ms_p50": _percentile(inference, 0.50),
        "inference_ms_p95": _percentile(inference, 0.95),
        "total_ms_mean": mean_total,
        "total_ms_p50": _percentile(total, 0.50),
        "total_ms_p95": _percentile(total, 0.95),
        "throughput_fps_from_mean_total": (
            1000.0 / mean_total if mean_total is not None and mean_total > 0.0 else None
        ),
    }


def sha256_file(path: Path, chunk_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json_sha256(value: object) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def read_json(path: Path) -> Mapping[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def load_flat_yaml(path: Path) -> Mapping[str, str]:
    """Read the project's deliberately flat Ultralytics YAML files.

    The formal perception configs contain scalar ``key: value`` pairs only.
    Keeping this parser dependency-light lets every provenance gate inspect the
    same config before importing Ultralytics.
    """

    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if ":" not in line:
            raise ValueError(f"{path}:{line_number}: expected a flat YAML key")
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if not key or key in values:
            raise ValueError(f"{path}:{line_number}: duplicate or empty YAML key")
        if not value:
            raise ValueError(f"{path}:{line_number}: empty YAML value")
        values[key] = value
    return values


def load_validation_inference_contract(path: Path) -> Mapping[str, object]:
    """Return the typed inference settings pinned by a validation config."""

    values = load_flat_yaml(path)
    required = {
        "model",
        "data",
        "task",
        "mode",
        "split",
        "imgsz",
        "batch",
        "device",
        "conf",
        "iou",
    }
    missing = sorted(required - set(values))
    if missing:
        raise ValueError(
            f"validation config is missing required keys: {', '.join(missing)}"
        )
    if values["task"] != "detect" or values["mode"] != "val" or values["split"] != "val":
        raise ValueError("validation config must pin task=detect, mode=val, split=val")
    try:
        imgsz = int(values["imgsz"])
        batch = int(values["batch"])
        raw_confidence = float(values["conf"])
        nms_iou = float(values["iou"])
    except ValueError as error:
        raise ValueError("validation config has a malformed numeric setting") from error
    if imgsz <= 0 or batch <= 0:
        raise ValueError("validation config imgsz and batch must be positive")
    if not math.isfinite(raw_confidence) or not 0.0 <= raw_confidence <= 1.0:
        raise ValueError("validation config conf must be finite and in [0, 1]")
    if not math.isfinite(nms_iou) or not 0.0 < nms_iou <= 1.0:
        raise ValueError("validation config iou must be finite and in (0, 1]")
    if not values["device"]:
        raise ValueError("validation config device must not be empty")
    return {
        "model": values["model"],
        "data": values["data"],
        "task": values["task"],
        "mode": values["mode"],
        "split": values["split"],
        "imgsz": imgsz,
        "batch": batch,
        "device": values["device"],
        "raw_confidence": raw_confidence,
        "nms_iou": nms_iou,
    }


def _json_bytes(value: object) -> bytes:
    return (json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n").encode(
        "utf-8"
    )


def write_json_exclusive(path: Path, value: object) -> None:
    """Create JSON exactly once, refusing even a zero-byte existing file."""

    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = -1
            stream.write(_json_bytes(value))
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def _replace_json(path: Path, value: object) -> None:
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    try:
        write_json_exclusive(temporary, value)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def acquire_one_time_claim(
    path: Path, purpose: str, bindings: Mapping[str, object], created_utc: str
) -> str:
    if not purpose:
        raise ValueError("claim purpose must not be empty")
    token = uuid4().hex
    write_json_exclusive(
        path,
        {
            "schema_version": SCHEMA_VERSION,
            "kind": "one_time_claim",
            "purpose": purpose,
            "status": "claimed",
            "claim_token": token,
            "created_utc": created_utc,
            "bindings": dict(bindings),
        },
    )
    return token


def finalize_one_time_claim(
    path: Path,
    token: str,
    status: str,
    completed_utc: str,
    details: Mapping[str, object],
) -> None:
    if status not in {"completed", "failed"}:
        raise ValueError("claim can only be finalized as completed or failed")
    current = dict(read_json(path))
    if current.get("kind") != "one_time_claim" or current.get("status") != "claimed":
        raise ValueError("claim is not in the claimed state")
    if current.get("claim_token") != token:
        raise ValueError("claim token does not match")
    current["status"] = status
    current["completed_utc"] = completed_utc
    current["details"] = dict(details)
    _replace_json(path, current)


def verify_file_contract(path: Path, expected_sha256: str, expected_size: int | None) -> str:
    if not path.is_file():
        raise FileNotFoundError(path)
    if expected_size is not None and path.stat().st_size != expected_size:
        raise ValueError(
            f"size mismatch for {path}: expected {expected_size}, got {path.stat().st_size}"
        )
    actual = sha256_file(path)
    if actual.lower() != expected_sha256.lower():
        raise ValueError(
            f"SHA-256 mismatch for {path}: expected {expected_sha256}, got {actual}"
        )
    return actual


def collect_environment() -> Mapping[str, object]:
    packages = {}
    for name in ("numpy", "opencv-python", "torch", "torchvision", "ultralytics"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    gpu = None
    try:
        probe = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=name,driver_version,memory.total",
                "--format=csv,noheader,nounits",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        if probe.returncode == 0:
            gpu = [line.strip() for line in probe.stdout.splitlines() if line.strip()]
    except (FileNotFoundError, subprocess.SubprocessError):
        pass
    return {
        "python": sys.version,
        "executable": sys.executable,
        "platform": platform.platform(),
        "packages": packages,
        "nvidia_smi": gpu,
    }


def load_experiment(path: Path, variant: str) -> tuple[Mapping[str, object], Mapping[str, object]]:
    spec = read_json(path)
    if spec.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported experiment schema")
    variants = spec.get("variants")
    if not isinstance(variants, dict) or variant not in variants:
        raise ValueError(f"unknown experiment variant: {variant}")
    selected = variants[variant]
    if not isinstance(selected, dict):
        raise ValueError(f"invalid experiment variant: {variant}")
    return spec, selected


def _contract_relative_path(repository_root: Path, value: object, field: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"split contract {field} path must be a non-empty string")
    relative = PurePosixPath(value.replace("\\", "/"))
    if (
        relative.is_absolute()
        or not relative.parts
        or ".." in relative.parts
        or relative.parts[0].endswith(":")
    ):
        raise ValueError(f"split contract {field} path must be repository-relative")
    root = repository_root.resolve()
    resolved = root.joinpath(*relative.parts).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError(f"split contract {field} path escapes the repository") from error
    return resolved


def _contract_sha256(value: object, field: str) -> str:
    digest = str(value).lower()
    if len(digest) != 64:
        raise ValueError(f"split contract {field} must be a SHA-256 digest")
    try:
        int(digest, 16)
    except ValueError as error:
        raise ValueError(f"split contract {field} must be a SHA-256 digest") from error
    return digest


def _contract_int(value: object, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"split contract {field} must be an integer")
    return value


def validate_split_contract(
    spec: Mapping[str, object],
    repository_root: Path,
    dataset_report: Mapping[str, object],
) -> Mapping[str, object]:
    """Validate and summarize the complete frozen data-curation identity."""

    contract = spec.get("split_contract")
    if not isinstance(contract, dict):
        raise ValueError("experiment has no split_contract")
    required_fields = {
        "schema_version",
        "strategy",
        "seed",
        "ratios",
        "counts",
        "source_image_count",
        "excluded_count",
        "retained_count",
        "canonical_dataset_sha256",
        "canonical_split_sha256",
        "artifacts",
    }
    if set(contract) != required_fields:
        raise ValueError("split_contract fields do not match the frozen schema")
    if contract.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("split_contract uses an unsupported schema")

    strategy = str(contract.get("strategy", ""))
    seed = _contract_int(contract.get("seed"), "seed")
    if strategy != FORMAL_SPLIT_STRATEGY:
        raise ValueError("split contract strategy is not the frozen full-resplit policy")
    if seed != FORMAL_SPLIT_SEED:
        raise ValueError("split contract seed is not the frozen seed")

    ratios_value = contract.get("ratios")
    if not isinstance(ratios_value, dict) or set(ratios_value) != set(
        FORMAL_SPLIT_RATIOS
    ):
        raise ValueError("split contract ratios must contain train, val, and test")
    ratios = {name: float(ratios_value[name]) for name in FORMAL_SPLIT_RATIOS}
    if any(
        not math.isclose(ratios[name], expected, rel_tol=0.0, abs_tol=1e-12)
        for name, expected in FORMAL_SPLIT_RATIOS.items()
    ):
        raise ValueError("split contract ratios differ from frozen 70/15/15")

    counts_value = contract.get("counts")
    if not isinstance(counts_value, dict) or set(counts_value) != set(
        FORMAL_SPLIT_COUNTS
    ):
        raise ValueError("split contract counts must contain train, val, and test")
    counts = {
        name: _contract_int(counts_value[name], f"counts.{name}")
        for name in FORMAL_SPLIT_COUNTS
    }
    if counts != FORMAL_SPLIT_COUNTS:
        raise ValueError("split contract counts differ from the frozen split")
    source_count = _contract_int(
        contract.get("source_image_count"), "source_image_count"
    )
    excluded_count = _contract_int(contract.get("excluded_count"), "excluded_count")
    retained_count = _contract_int(contract.get("retained_count"), "retained_count")
    if (
        source_count != FORMAL_SOURCE_IMAGE_COUNT
        or excluded_count != FORMAL_EXCLUDED_COUNT
        or retained_count != FORMAL_RETAINED_COUNT
    ):
        raise ValueError("split contract source/excluded/retained counts are not frozen")
    if source_count != excluded_count + retained_count or retained_count != sum(
        counts.values()
    ):
        raise ValueError("split contract image counts are internally inconsistent")

    canonical_dataset = _contract_sha256(
        contract.get("canonical_dataset_sha256"), "canonical_dataset_sha256"
    )
    split_hashes_value = contract.get("canonical_split_sha256")
    if not isinstance(split_hashes_value, dict) or set(split_hashes_value) != set(
        FORMAL_SPLIT_COUNTS
    ):
        raise ValueError("split contract canonical split hashes are incomplete")
    split_hashes = {
        name: _contract_sha256(
            split_hashes_value[name], f"canonical_split_sha256.{name}"
        )
        for name in FORMAL_SPLIT_COUNTS
    }

    artifacts_value = contract.get("artifacts")
    if not isinstance(artifacts_value, dict) or set(artifacts_value) != set(
        SPLIT_CONTRACT_ARTIFACTS
    ):
        raise ValueError("split contract artifact set is incomplete or unexpected")
    artifacts = {}
    for name in sorted(SPLIT_CONTRACT_ARTIFACTS):
        item = artifacts_value[name]
        if not isinstance(item, dict) or set(item) != {"path", "sha256"}:
            raise ValueError(f"split contract artifact {name} is invalid")
        path_value = item.get("path")
        path = _contract_relative_path(repository_root, path_value, name)
        digest = _contract_sha256(item.get("sha256"), f"artifacts.{name}.sha256")
        verify_file_contract(path, digest, None)
        artifacts[name] = {"path": str(path_value).replace("\\", "/"), "sha256": digest}

    if artifacts["split_manifest"]["path"] != str(spec.get("split_manifest", "")):
        raise ValueError("split contract split_manifest path differs from experiment")
    if artifacts["dataset_yaml"]["path"] != str(spec.get("dataset_yaml", "")):
        raise ValueError("split contract dataset_yaml path differs from experiment")

    report_checks = {
        "strategy": strategy,
        "seed": seed,
        "split_sample_count": counts,
        "sample_count": retained_count,
        "source_image_count": source_count,
        "excluded_count": excluded_count,
        "retained_count": retained_count,
        "canonical_dataset_sha256": canonical_dataset,
        "canonical_split_sha256": split_hashes,
        "dataset_yaml_sha256": artifacts["dataset_yaml"]["sha256"],
        "exclusions_csv": artifacts["exclusions_csv"]["path"],
        "exclusions_csv_sha256": artifacts["exclusions_csv"]["sha256"],
    }
    for field, expected in report_checks.items():
        if dataset_report.get(field) != expected:
            raise ValueError(
                f"verified dataset {field} does not match the split contract"
            )

    summary = {
        "schema_version": SCHEMA_VERSION,
        "strategy": strategy,
        "seed": seed,
        "ratios": ratios,
        "counts": counts,
        "source_image_count": source_count,
        "excluded_count": excluded_count,
        "retained_count": retained_count,
        "canonical_dataset_sha256": canonical_dataset,
        "canonical_split_sha256": split_hashes,
        "artifacts": artifacts,
    }
    return {
        **summary,
        "split_contract_sha256": canonical_json_sha256(summary),
    }


def verify_materialized_dataset(split_manifest: Path) -> Mapping[str, object]:
    """Run the canonical, content-level dataset verifier from ``tools/data``.

    Keeping this import lazy preserves the dependency-light metric helpers while
    still making every training/inference entry point fail closed on a stale or
    partially copied materialized dataset.
    """

    data_tools_root = Path(__file__).resolve().parents[1] / "data"
    data_tools_value = str(data_tools_root)
    if data_tools_value not in sys.path:
        sys.path.insert(0, data_tools_value)
    from dataset_tools import verify_materialized_split_manifest

    report = verify_materialized_split_manifest(split_manifest)
    if report.get("valid") is not True:
        raise ValueError("materialized dataset verifier returned an invalid report")
    digest = report.get("canonical_dataset_sha256")
    if not isinstance(digest, str) or len(digest) != 64:
        raise ValueError("materialized dataset report has no canonical dataset digest")
    return report


def validate_formal_test_contract(
    spec: Mapping[str, object],
    variant_name: str,
    variant: Mapping[str, object],
) -> Mapping[str, object]:
    """Require the single dataset-level held-out-test contract.

    A speed baseline is deliberately validation-only.  The receipt belongs to
    the dataset, rather than to a model variant, so trying another variant cannot
    create a fresh formal look at the same held-out images.
    """

    contract = spec.get("formal_test")
    if not isinstance(contract, dict):
        raise ValueError("experiment has no dataset-level formal-test contract")
    expected_variant = contract.get("variant")
    receipt = contract.get("receipt")
    task = contract.get("task")
    imgsz = contract.get("imgsz")
    if not isinstance(expected_variant, str) or not expected_variant:
        raise ValueError("formal-test contract has no primary variant")
    if not isinstance(receipt, str) or not receipt:
        raise ValueError("formal-test contract has no dataset-level receipt")
    if task != "detect":
        raise ValueError("formal-test contract must pin the detection task")
    if not isinstance(imgsz, int) or isinstance(imgsz, bool) or imgsz <= 0:
        raise ValueError("formal-test contract must pin a positive integer imgsz")
    if variant_name != expected_variant or variant.get("role") != "primary":
        raise ValueError(
            f"held-out test is reserved for primary variant {expected_variant}; "
            f"{variant_name} is validation-only"
        )
    return contract


def optimization_promotion_artifact_path(
    spec: Mapping[str, object], repository_root: Path
) -> Path | None:
    """Return the fixed validation-promotion artifact for an optimized primary.

    Experiments without an optimization policy keep the original one-stage
    formal-test contract. An optimized experiment must present a passed,
    immutable validation decision before it can claim the held-out test.
    """

    policy = spec.get("optimization_policy")
    if policy is None:
        return None
    if not isinstance(policy, dict):
        raise ValueError("optimization policy must be a JSON object")
    candidate = policy.get("candidate_variant")
    if (
        not isinstance(candidate, str)
        or not candidate
        or any(not (character.isalnum() or character in "_-") for character in candidate)
    ):
        raise ValueError("optimization candidate has an unsafe variant name")
    formal_test = spec.get("formal_test")
    if not isinstance(formal_test, dict) or formal_test.get("variant") != candidate:
        raise ValueError("formal test is not reserved for the optimization candidate")
    return (
        repository_root.resolve()
        / "artifacts"
        / "perception"
        / "optimization"
        / f"{candidate}_promotion.json"
    )


def validate_optimization_promotion(
    promotion: Mapping[str, object],
    spec: Mapping[str, object],
    *,
    experiments_sha256: str,
    frozen_threshold_path: str,
    frozen_threshold_sha256: str,
    weights_sha256: str,
    validation_bundle: str,
    validation_bundle_sha256: str,
    canonical_dataset_sha256: str,
    training_config_sha256: str,
    validation_config: str,
    validation_config_sha256: str,
    validation_nms_iou: float,
    expected_candidate_validation_metrics: Mapping[str, float],
) -> None:
    """Validate that Opt1 passed its predeclared validation-only promotion gate."""

    policy = spec.get("optimization_policy")
    if not isinstance(policy, dict):
        raise ValueError("experiment has no optimization promotion policy")
    candidate = policy.get("candidate_variant")
    baseline = policy.get("baseline_variant")
    if promotion.get("schema_version") != SCHEMA_VERSION or promotion.get("kind") != (
        "validation_promotion_decision"
    ):
        raise ValueError("optimization promotion has the wrong schema or kind")
    if promotion.get("status") != "completed":
        raise ValueError("optimization promotion is not completed")
    scope = promotion.get("scope")
    bindings = promotion.get("bindings")
    metrics = promotion.get("candidate_validation_metrics")
    checks = promotion.get("checks")
    if not all(isinstance(value, dict) for value in (scope, bindings, metrics, checks)):
        raise ValueError("optimization promotion is missing required records")
    assert isinstance(scope, dict) and isinstance(bindings, dict)
    assert isinstance(metrics, dict) and isinstance(checks, dict)
    if scope != {
        "baseline_variant": baseline,
        "candidate_variant": candidate,
        "selection_split": "val",
        "held_out_test_accessed": False,
    }:
        raise ValueError("optimization promotion scope does not match the policy")
    expected_bindings = {
        "experiments_sha256": experiments_sha256,
        "candidate_frozen_threshold": frozen_threshold_path,
        "candidate_frozen_threshold_sha256": frozen_threshold_sha256,
        "candidate_weights_sha256": weights_sha256,
        "candidate_validation_bundle": validation_bundle,
        "candidate_validation_bundle_sha256": validation_bundle_sha256,
        "canonical_dataset_sha256": canonical_dataset_sha256,
        "training_config_sha256": training_config_sha256,
        "validation_config": validation_config,
        "validation_config_sha256": validation_config_sha256,
        "validation_nms_iou": validation_nms_iou,
    }
    for name, expected in expected_bindings.items():
        if bindings.get(name) != expected:
            raise ValueError(f"optimization promotion {name} binding does not match")
    expected_metrics = {
        "macro_f1": float(expected_candidate_validation_metrics["macro_f1"]),
        "ripe_f1": float(expected_candidate_validation_metrics["ripe_f1"]),
        "unripe_f1": float(expected_candidate_validation_metrics["unripe_f1"]),
    }
    if metrics != expected_metrics:
        raise ValueError(
            "optimization promotion metrics do not match the frozen validation result"
        )

    criteria = policy.get("validation_promotion")
    if not isinstance(criteria, dict):
        raise ValueError("optimization policy has no validation criteria")
    thresholds = {
        "unripe_f1": float(criteria["unripe_f1_min"]),
        "macro_f1": float(criteria["macro_f1_min"]),
        "ripe_f1": float(criteria["ripe_f1_min"]),
    }
    passed = True
    for name, required in thresholds.items():
        actual = float(metrics.get(name, math.nan))
        expected_check = {
            "operator": ">=",
            "required": required,
            "actual": actual,
            "passed": actual >= required,
        }
        if not math.isfinite(actual) or checks.get(name) != expected_check:
            raise ValueError(f"optimization promotion {name} check is not reproducible")
        passed = passed and actual >= required
    if promotion.get("all_validation_conditions_passed") is not passed:
        raise ValueError("optimization promotion aggregate decision is inconsistent")
    if promotion.get("promote_candidate") is not passed:
        raise ValueError("optimization candidate promotion decision is inconsistent")
    if not passed:
        raise ValueError("optimization candidate did not pass validation promotion")

    formal_acceptance = policy.get("formal_acceptance")
    metric_contract = spec.get("metric")
    authorization_gate = promotion.get("validation_authorization_gate")
    formal_gate = promotion.get("formal_test_gate")
    if not all(
        isinstance(value, dict)
        for value in (
            formal_acceptance,
            metric_contract,
            authorization_gate,
            formal_gate,
        )
    ):
        raise ValueError("optimization promotion has no formal acceptance gate")
    assert isinstance(formal_acceptance, dict)
    assert isinstance(metric_contract, dict)
    assert isinstance(authorization_gate, dict) and isinstance(formal_gate, dict)
    required = float(metric_contract.get("macro_f1_min", math.nan))
    macro_f1 = float(metrics.get("macro_f1", math.nan))
    authorization_passed = passed and macro_f1 >= required
    if (
        not math.isfinite(required)
        or float(formal_acceptance.get("macro_f1_min", math.nan)) != required
        or authorization_gate
        != {
            "metric": "macro_f1",
            "operator": ">=",
            "required": required,
            "actual": macro_f1,
            "passed": macro_f1 >= required,
        }
        or promotion.get("authorize_held_out_test") is not authorization_passed
        or formal_gate
        != {
            "metric": "macro_f1",
            "operator": ">=",
            "required": required,
            "evaluated": False,
        }
    ):
        raise ValueError("optimization promotion formal acceptance gate does not match")
    if not authorization_passed:
        raise ValueError(
            "optimization candidate is not authorized for the held-out test"
        )


def validate_completed_training_receipt(
    receipt: Mapping[str, object],
    *,
    variant: str,
    dataset_id: str,
    expected_trained_weight: str,
    weights_sha256: str,
    training_config_sha256: str,
    experiments_sha256: str,
    expected_split_contract: Mapping[str, object] | None = None,
) -> str:
    """Validate that ``weights_sha256`` is the recorded formal train output.

    Returns the path-independent canonical dataset digest recorded before
    training.  The caller can claim the test receipt with this digest before it
    reads any held-out files, and then verify the materialized dataset against it.
    """

    if receipt.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("training receipt uses an unsupported schema")
    if receipt.get("kind") != "one_time_claim":
        raise ValueError("training receipt has the wrong artifact kind")
    if receipt.get("purpose") != f"formal_training:{variant}":
        raise ValueError("training receipt purpose does not match the primary variant")
    if receipt.get("status") != "completed":
        raise ValueError("formal training receipt is not completed")
    bindings = receipt.get("bindings")
    details = receipt.get("details")
    if not isinstance(bindings, dict) or not isinstance(details, dict):
        raise ValueError("training receipt is missing bindings or completion details")
    expected_bindings = {
        "variant": variant,
        "dataset_id": dataset_id,
        "expected_trained_weight": expected_trained_weight,
        "training_config_sha256": training_config_sha256,
        "experiments_sha256": experiments_sha256,
    }
    for name, expected in expected_bindings.items():
        if bindings.get(name) != expected:
            raise ValueError(f"training receipt {name} binding does not match")
    if (
        expected_split_contract is not None
        and bindings.get("split_contract") != expected_split_contract
    ):
        raise ValueError("training receipt split_contract binding does not match")
    if details.get("trained_weight") != expected_trained_weight:
        raise ValueError("training receipt records a different trained-weight path")
    if details.get("trained_weight_sha256") != weights_sha256:
        raise ValueError("current trained weight hash does not match the training receipt")
    dataset_digest = bindings.get("canonical_dataset_sha256")
    if not isinstance(dataset_digest, str) or len(dataset_digest) != 64:
        raise ValueError("training receipt has no canonical dataset digest")
    try:
        int(dataset_digest, 16)
    except ValueError as error:
        raise ValueError("training receipt canonical dataset digest is invalid") from error
    return dataset_digest


def _safe_relative_path(root: Path, value: str) -> Path:
    relative = PurePosixPath(value.replace("\\", "/"))
    if relative.is_absolute() or ".." in relative.parts or not relative.parts:
        raise ValueError(f"unsafe relative path in split manifest: {value}")
    resolved = root.joinpath(*relative.parts).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError(f"path escapes processed dataset root: {value}") from error
    return resolved


def load_split_samples(split_manifest: Path, split: str) -> tuple[DatasetSample, ...]:
    if split not in SUPPORTED_SPLITS:
        raise ValueError(f"unsupported inference split: {split}")
    manifest = read_json(split_manifest)
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported split manifest schema")
    splits = manifest.get("splits")
    if not isinstance(splits, dict) or not isinstance(splits.get(split), list):
        raise ValueError(f"split manifest has no {split} samples")
    root = split_manifest.resolve().parent
    samples = []
    seen = set()
    for item in splits[split]:
        if not isinstance(item, dict):
            raise ValueError("split sample must be a JSON object")
        image_key = str(item.get("output_image", ""))
        label_key = str(item.get("output_label", ""))
        if not image_key or image_key in seen:
            raise ValueError("split image keys must be non-empty and unique")
        seen.add(image_key)
        image_path = _safe_relative_path(root, image_key)
        label_path = _safe_relative_path(root, label_key)
        if not image_path.is_file() or not label_path.is_file():
            raise FileNotFoundError(f"missing split image or label for {image_key}")
        samples.append(DatasetSample(image_key, image_path, label_path))
    if not samples:
        raise ValueError(f"split {split} is empty")
    return tuple(samples)


def load_yolo_ground_truth(path: Path, class_ids: Sequence[int] = (0, 1)) -> tuple[GroundTruth, ...]:
    allowed = frozenset(int(value) for value in class_ids)
    records = []
    with path.open("r", encoding="utf-8-sig") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) != 5:
                raise ValueError(f"{path}:{line_number}: expected five YOLO fields")
            try:
                class_value = float(fields[0])
                coordinates = tuple(float(field) for field in fields[1:])
            except ValueError as error:
                raise ValueError(f"{path}:{line_number}: non-numeric YOLO field") from error
            if not class_value.is_integer() or int(class_value) not in allowed:
                raise ValueError(f"{path}:{line_number}: class is outside the v1 contract")
            records.append(
                GroundTruth(
                    int(class_value), normalized_xywh_to_xyxy(*coordinates)
                )
            )
    return tuple(records)


def record_to_json(record: ImageDetections) -> Mapping[str, object]:
    return {
        "image": record.image,
        "ground_truth": [
            {"class_id": item.class_id, "bbox_xyxy_normalized": list(item.bbox)}
            for item in record.ground_truth
        ],
        "predictions": [
            {
                "class_id": item.class_id,
                "confidence": item.confidence,
                "bbox_xyxy_normalized": list(item.bbox),
            }
            for item in record.predictions
        ],
        "timing_ms": {
            "inference": record.inference_ms,
            "total": record.total_ms,
        },
    }


def _record_from_json(value: object) -> ImageDetections:
    if not isinstance(value, dict):
        raise ValueError("prediction record must be a JSON object")
    truths = value.get("ground_truth")
    predictions = value.get("predictions")
    timing = value.get("timing_ms", {})
    if not isinstance(truths, list) or not isinstance(predictions, list):
        raise ValueError("prediction record must contain ground_truth and predictions")
    if not isinstance(timing, dict):
        raise ValueError("timing_ms must be a JSON object")
    def parse_truth(item: object) -> GroundTruth:
        if not isinstance(item, dict):
            raise ValueError("ground-truth entry must be a JSON object")
        return GroundTruth(
            int(item["class_id"]),
            tuple(float(value) for value in item["bbox_xyxy_normalized"]),
        )

    def parse_prediction(item: object) -> Prediction:
        if not isinstance(item, dict):
            raise ValueError("prediction entry must be a JSON object")
        return Prediction(
            int(item["class_id"]),
            float(item["confidence"]),
            tuple(float(value) for value in item["bbox_xyxy_normalized"]),
        )

    return ImageDetections(
        image=str(value.get("image", "")),
        ground_truth=tuple(parse_truth(item) for item in truths),
        predictions=tuple(parse_prediction(item) for item in predictions),
        inference_ms=(
            None if timing.get("inference") is None else float(timing["inference"])
        ),
        total_ms=None if timing.get("total") is None else float(timing["total"]),
    )


def load_prediction_bundle(
    path: Path, expected_split: str | None = None
) -> tuple[Mapping[str, object], tuple[ImageDetections, ...]]:
    bundle = read_json(path)
    if bundle.get("schema_version") != SCHEMA_VERSION or bundle.get("kind") != "prediction_bundle":
        raise ValueError("unsupported prediction bundle")
    split = bundle.get("split")
    if split not in SUPPORTED_SPLITS:
        raise ValueError("prediction bundle split must be val or test")
    if expected_split is not None and split != expected_split:
        raise ValueError(f"expected a {expected_split} prediction bundle")
    values = bundle.get("records")
    if not isinstance(values, list) or not values:
        raise ValueError("prediction bundle records must be a non-empty list")
    records = tuple(_record_from_json(item) for item in values)
    if len({item.image for item in records}) != len(records):
        raise ValueError("prediction bundle contains duplicate image keys")
    return bundle, records


def validate_validation_records_against_split(
    records: Sequence[ImageDetections],
    split_manifest: Path,
    *,
    class_ids: Sequence[int],
    expected_count: int,
) -> tuple[DatasetSample, ...]:
    """Require an exact, current manifest/ground-truth view of validation.

    Prediction JSON is not an authority for sample membership or labels.  This
    check re-opens only the validation labels from the verified materialized
    split and rejects truncated, padded, reordered, or relabelled bundles.
    """

    samples = load_split_samples(split_manifest, "val")
    if len(samples) != expected_count:
        raise ValueError(
            "validation manifest count does not match the frozen split contract: "
            f"expected {expected_count}, got {len(samples)}"
        )
    if len(records) != expected_count:
        raise ValueError(
            "validation bundle does not contain the complete validation split: "
            f"expected {expected_count}, got {len(records)}"
        )
    expected_keys = tuple(sample.image_key for sample in samples)
    actual_keys = tuple(record.image for record in records)
    if len(set(actual_keys)) != len(actual_keys):
        raise ValueError("validation bundle contains duplicate image keys")
    if actual_keys != expected_keys:
        expected_set = set(expected_keys)
        actual_set = set(actual_keys)
        missing = sorted(expected_set - actual_set)
        extra = sorted(actual_set - expected_set)
        if missing or extra:
            raise ValueError(
                "validation bundle image keys do not exactly cover the manifest; "
                f"missing={missing}, extra={extra}"
            )
        raise ValueError("validation bundle image order does not match the manifest")
    for sample, record in zip(samples, records):
        current_truth = load_yolo_ground_truth(sample.label_path, class_ids)
        if record.ground_truth != current_truth:
            raise ValueError(
                f"validation bundle ground truth does not match current label: {record.image}"
            )
    return samples


def validate_formal_validation_bundle(
    bundle: Mapping[str, object],
    *,
    variant: str,
    expected_bindings: Mapping[str, object],
    expected_imgsz: int,
) -> None:
    """Require validation provenance that is eligible for the formal test.

    Diagnostic validation is intentionally more permissive, but a threshold for
    the primary held-out test must come from the completed formal-training
    artifact at the contracted input size.  Every data, configuration, weight,
    and training-receipt identity is compared exactly with current trusted
    bindings rather than merely copied from the prediction bundle.
    """

    if (
        bundle.get("schema_version") != SCHEMA_VERSION
        or bundle.get("kind") != "prediction_bundle"
        or bundle.get("split") != "val"
    ):
        raise ValueError("formal threshold requires a validation prediction bundle")
    if bundle.get("variant") != variant:
        raise ValueError("formal validation bundle variant does not match")
    bindings = bundle.get("bindings")
    if not isinstance(bindings, dict):
        raise ValueError("formal validation bundle has no immutable bindings")
    if expected_bindings.get("weights_source") != "expected_trained_weight":
        raise ValueError("formal validation expected bindings have an invalid weight source")
    for field in FORMAL_VALIDATION_BINDING_FIELDS:
        if field not in expected_bindings:
            raise ValueError(f"formal validation expected bindings are missing {field}")
        if bindings.get(field) != expected_bindings[field]:
            raise ValueError(f"formal validation bundle {field} binding does not match")
    if bindings.get("weights_source") != "expected_trained_weight":
        raise ValueError("formal validation requires the expected trained weight")
    if bundle.get("role") != expected_bindings["role"]:
        raise ValueError("formal validation bundle role does not match")
    inference = bundle.get("inference")
    if not isinstance(inference, dict):
        raise ValueError("formal validation bundle has no inference settings")
    imgsz = inference.get("imgsz")
    if (
        not isinstance(imgsz, int)
        or isinstance(imgsz, bool)
        or imgsz != expected_imgsz
    ):
        raise ValueError(
            f"formal validation imgsz is fixed by the contract at {expected_imgsz}"
        )
    expected_inference = {
        "raw_confidence": float(expected_bindings["validation_raw_confidence"]),
        "imgsz": int(expected_bindings["validation_imgsz"]),
        "batch": int(expected_bindings["validation_batch"]),
        "device": str(expected_bindings["validation_device"]),
        "nms_iou": float(expected_bindings["validation_nms_iou"]),
    }
    actual_inference = {
        "raw_confidence": float(inference.get("raw_confidence", math.nan)),
        "imgsz": inference.get("imgsz"),
        "batch": inference.get("batch"),
        "device": str(inference.get("device", "")),
        "nms_iou": float(inference.get("nms_iou", math.nan)),
    }
    if actual_inference != expected_inference:
        raise ValueError("formal validation inference settings do not match its config")
    if not math.isclose(
        float(expected_bindings["validation_nms_iou"]),
        FORMAL_NMS_IOU,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError(f"formal validation NMS IoU is fixed at {FORMAL_NMS_IOU}")


def validate_frozen_threshold(
    freeze: Mapping[str, object],
    variant: str,
    weights_sha256: str,
    canonical_dataset_sha256: str,
    training_config_sha256: str | None = None,
    experiments_sha256: str | None = None,
    iou_threshold: float = 0.5,
    class_ids: Sequence[int] = (0, 1),
    expected_split_contract: Mapping[str, object] | None = None,
    expected_formal_validation_bindings: Mapping[str, object] | None = None,
    expected_validation_imgsz: int | None = None,
) -> float:
    if freeze.get("schema_version") != SCHEMA_VERSION or freeze.get("kind") != "frozen_threshold":
        raise ValueError("unsupported threshold freeze")
    if freeze.get("metric_name") != METRIC_NAME:
        raise ValueError("threshold freeze uses a different metric definition")
    if freeze.get("variant") != variant:
        raise ValueError("threshold freeze variant does not match")
    bindings = freeze.get("bindings")
    if not isinstance(bindings, dict):
        raise ValueError("threshold freeze has no bindings")
    if bindings.get("weights_sha256") != weights_sha256:
        raise ValueError("threshold freeze is bound to different weights")
    if bindings.get("canonical_dataset_sha256") != canonical_dataset_sha256:
        raise ValueError("threshold freeze is bound to a different canonical dataset")
    if (
        expected_split_contract is not None
        and bindings.get("split_contract") != expected_split_contract
    ):
        raise ValueError("threshold freeze is bound to a different split contract")
    if (
        training_config_sha256 is not None
        and bindings.get("training_config_sha256") != training_config_sha256
    ):
        raise ValueError("threshold freeze is bound to a different training config")
    if (
        experiments_sha256 is not None
        and bindings.get("experiments_sha256") != experiments_sha256
    ):
        raise ValueError("threshold freeze is bound to a different experiment contract")
    frozen_iou = float(freeze.get("iou_threshold", math.nan))
    if not math.isclose(frozen_iou, iou_threshold, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("threshold freeze uses a different IoU threshold")
    frozen_classes = freeze.get("class_ids")
    if not isinstance(frozen_classes, list) or tuple(
        int(value) for value in frozen_classes
    ) != tuple(int(value) for value in class_ids):
        raise ValueError("threshold freeze uses a different class contract")
    threshold = float(freeze.get("confidence_threshold", math.nan))
    if not math.isfinite(threshold) or not 0.0 <= threshold <= 1.0:
        raise ValueError("frozen threshold is invalid")
    if expected_formal_validation_bindings is not None:
        if expected_validation_imgsz is None:
            raise ValueError("formal threshold validation requires the contracted imgsz")
        for field in FORMAL_VALIDATION_BINDING_FIELDS:
            if field not in expected_formal_validation_bindings:
                raise ValueError(f"formal test expected bindings are missing {field}")
            if bindings.get(field) != expected_formal_validation_bindings[field]:
                raise ValueError(f"threshold freeze {field} binding does not match")
        if bindings.get("weights_source") != "expected_trained_weight":
            raise ValueError("formal threshold was not frozen from the trained weight")
        if bindings.get("validation_imgsz") != expected_validation_imgsz:
            raise ValueError("threshold freeze uses a different validation imgsz")
        validation_bundle = bindings.get("validation_bundle")
        if not isinstance(validation_bundle, str) or not validation_bundle:
            raise ValueError("formal threshold has no validation-bundle path")
        validation_bundle_sha256 = bindings.get("validation_bundle_sha256")
        if not isinstance(validation_bundle_sha256, str) or len(
            validation_bundle_sha256
        ) != 64:
            raise ValueError("formal threshold has no validation-bundle digest")
        try:
            int(validation_bundle_sha256, 16)
        except ValueError as error:
            raise ValueError("formal threshold validation-bundle digest is invalid") from error
        selection = freeze.get("selection")
        if not isinstance(selection, dict):
            raise ValueError("formal threshold has no validation selection record")
        selected_threshold = float(selection.get("selected_threshold", math.nan))
        if not math.isclose(
            selected_threshold, threshold, rel_tol=0.0, abs_tol=1e-12
        ):
            raise ValueError("formal threshold disagrees with its validation selection")
    return threshold
