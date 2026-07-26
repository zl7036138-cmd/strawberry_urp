#!/usr/bin/env python3
"""Export the frozen baseline predictions for the registered training split.

This is a label-audit screen, not a training or evaluation workflow.  It is
hard-coded to the ADR-0024 inference settings, verifies every train image and
label against the split manifest, and refuses paths outside ``images/train``
and ``labels/train``.  Validation and test samples are never loaded.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path, PurePosixPath
import sys
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_inference import _run_model  # noqa: E402
from workflow import (  # noqa: E402
    DatasetSample,
    collect_environment,
    read_json,
    record_to_json,
    sha256_file,
    summarize_speed,
    write_json_exclusive,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REBASELINE = Path(
    "artifacts/perception/rebaseline/t30_validation_rebaseline_v1.json"
)
DEFAULT_EXPERIMENTS = Path("tools/perception/experiments.json")
DEFAULT_SPLIT_MANIFEST = Path("data/processed/zenodo_6126677/split_manifest.json")
DEFAULT_WEIGHTS = Path("outputs/perception/yolo11s_640/weights/best.pt")
DEFAULT_OUTPUT = Path(
    "artifacts/perception/label_audit/t30_train_label_screen_v1/predictions.json"
)
EXPECTED_TRAIN_COUNT = 501
RAW_CONFIDENCE = 0.001
NMS_IOU = 0.70
IMAGE_SIZE = 640
BATCH_SIZE = 8
DEVICE = "0"


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def _safe_path(root: Path, value: object, prefix: str) -> Path:
    normalized = str(value).replace("\\", "/")
    relative = PurePosixPath(normalized)
    if (
        relative.is_absolute()
        or ".." in relative.parts
        or not normalized.startswith(prefix)
    ):
        raise ValueError(f"training audit path is outside {prefix}: {value}")
    resolved = root.joinpath(*relative.parts).resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as error:
        raise ValueError(f"training audit path escapes dataset root: {value}") from error
    return resolved


def load_verified_train_samples(
    split_manifest_path: Path, expected_count: int = EXPECTED_TRAIN_COUNT
) -> tuple[DatasetSample, ...]:
    manifest = read_json(split_manifest_path)
    if manifest.get("schema_version") != 1:
        raise ValueError("unsupported split manifest schema")
    counts = manifest.get("counts")
    splits = manifest.get("splits")
    if not isinstance(counts, dict) or int(counts.get("train", -1)) != expected_count:
        raise ValueError("split manifest training count differs from the audit contract")
    if not isinstance(splits, dict) or not isinstance(splits.get("train"), list):
        raise ValueError("split manifest has no training split")
    entries = splits["train"]
    if len(entries) != expected_count:
        raise ValueError("training entry count differs from the audit contract")
    root = split_manifest_path.resolve().parent
    samples = []
    seen = set()
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("training split entry must be an object")
        image_key = str(entry.get("output_image", "")).replace("\\", "/")
        label_key = str(entry.get("output_label", "")).replace("\\", "/")
        if not image_key or image_key in seen:
            raise ValueError("training image keys must be non-empty and unique")
        seen.add(image_key)
        image_path = _safe_path(root, image_key, "images/train/")
        label_path = _safe_path(root, label_key, "labels/train/")
        for path, size_field, hash_field in (
            (image_path, "output_image_size_bytes", "output_image_sha256"),
            (label_path, "output_label_size_bytes", "output_label_sha256"),
        ):
            if not path.is_file():
                raise FileNotFoundError(path)
            if path.stat().st_size != int(entry.get(size_field, -1)):
                raise ValueError(f"training audit file size mismatch: {path}")
            if sha256_file(path) != entry.get(hash_field):
                raise ValueError(f"training audit file hash mismatch: {path}")
        samples.append(DatasetSample(image_key, image_path, label_path))
    return tuple(samples)


def _repo_binding(path: Path) -> dict[str, Any]:
    try:
        name = path.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        name = str(path)
    return {
        "path": name,
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def export_predictions(
    *,
    rebaseline_path: Path,
    experiments_path: Path,
    split_manifest_path: Path,
    weights_path: Path,
    output_path: Path,
) -> Mapping[str, object]:
    for path in (rebaseline_path, experiments_path, split_manifest_path, weights_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    if output_path.exists():
        raise FileExistsError(f"refusing to overwrite training audit predictions: {output_path}")
    rebaseline = read_json(rebaseline_path)
    if rebaseline.get("kind") != "t30_validation_rebaseline_diagnostic":
        raise ValueError("unexpected T30 rebaseline artifact")
    authorization = rebaseline.get("decision", {}).get("authorization", {})
    if authorization.get("training_label_audit") is not True:
        raise ValueError("T30 rebaseline does not authorize a training-label audit")
    if any(
        authorization.get(name) is not False
        for name in ("new_training", "held_out_test", "perception_control")
    ):
        raise ValueError("training-label audit authorization boundary is not fail-closed")

    selected_binding = rebaseline.get("bindings", {}).get(
        "selected_checkpoint_artifact", {}
    )
    selected_path = _resolve(Path(str(selected_binding.get("path", "")))).resolve()
    if (
        not selected_path.is_file()
        or selected_path.stat().st_size != int(selected_binding.get("size_bytes", -1))
        or sha256_file(selected_path) != selected_binding.get("sha256")
    ):
        raise ValueError("selected checkpoint artifact differs from rebaseline binding")
    selected = read_json(selected_path)
    checkpoint = selected.get("checkpoint", {})
    if checkpoint.get("checkpoint_id") != "baseline__best":
        raise ValueError("training-label audit must use baseline__best")
    if weights_path.stat().st_size != int(checkpoint.get("size_bytes", -1)):
        raise ValueError("audit weight size differs from the selected checkpoint")
    if sha256_file(weights_path) != checkpoint.get("sha256"):
        raise ValueError("audit weight hash differs from the selected checkpoint")

    experiments = read_json(experiments_path)
    contract = experiments.get("split_contract", {})
    expected_manifest_hash = (
        contract.get("artifacts", {}).get("split_manifest", {}).get("sha256")
    )
    if sha256_file(split_manifest_path) != expected_manifest_hash:
        raise ValueError("split manifest differs from experiments.json contract")
    if int(contract.get("counts", {}).get("train", -1)) != EXPECTED_TRAIN_COUNT:
        raise ValueError("experiments.json training count differs from ADR-0024")

    samples = load_verified_train_samples(split_manifest_path)
    records = _run_model(
        weights_path,
        samples,
        (0, 1),
        RAW_CONFIDENCE,
        NMS_IOU,
        IMAGE_SIZE,
        BATCH_SIZE,
        DEVICE,
    )
    if len(records) != EXPECTED_TRAIN_COUNT:
        raise ValueError("model did not return the complete training split")
    if any(
        not record.image.replace("\\", "/").startswith("images/train/")
        for record in records
    ):
        raise ValueError("model output contains a non-training record")
    if sha256_file(weights_path) != checkpoint.get("sha256"):
        raise ValueError("audit weight changed during inference")
    if sha256_file(split_manifest_path) != expected_manifest_hash:
        raise ValueError("split manifest changed during inference")

    bundle = {
        "schema_version": 1,
        "kind": "training_label_audit_prediction_bundle",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": {
            "split": "train",
            "image_count": len(records),
            "validation_split_accessed": False,
            "held_out_test_accessed": False,
            "training_started": False,
            "formal_metric": False,
        },
        "bindings": {
            "rebaseline": _repo_binding(rebaseline_path),
            "selected_checkpoint_artifact": _repo_binding(selected_path),
            "experiments": _repo_binding(experiments_path),
            "split_manifest": _repo_binding(split_manifest_path),
            "weights": _repo_binding(weights_path),
            "canonical_train_sha256": contract.get("canonical_split_sha256", {}).get(
                "train"
            ),
        },
        "inference": {
            "raw_confidence": RAW_CONFIDENCE,
            "nms_iou": NMS_IOU,
            "imgsz": IMAGE_SIZE,
            "batch": BATCH_SIZE,
            "device": DEVICE,
        },
        "speed": summarize_speed(records),
        "environment": collect_environment(),
        "records": [record_to_json(record) for record in records],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_json_exclusive(output_path, bundle)
    return {
        "output": output_path.relative_to(REPOSITORY_ROOT).as_posix(),
        "output_sha256": sha256_file(output_path),
        "image_count": len(records),
        "speed": bundle["speed"],
        "held_out_test_accessed": False,
        "training_started": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rebaseline", type=Path, default=DEFAULT_REBASELINE)
    parser.add_argument("--experiments", type=Path, default=DEFAULT_EXPERIMENTS)
    parser.add_argument("--split-manifest", type=Path, default=DEFAULT_SPLIT_MANIFEST)
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    summary = export_predictions(
        rebaseline_path=_resolve(args.rebaseline).resolve(),
        experiments_path=_resolve(args.experiments).resolve(),
        split_manifest_path=_resolve(args.split_manifest).resolve(),
        weights_path=_resolve(args.weights).resolve(),
        output_path=_resolve(args.output).resolve(),
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
