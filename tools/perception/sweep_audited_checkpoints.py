#!/usr/bin/env python3
"""Evaluate the frozen set of existing checkpoints on audited validation labels."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gc
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from materialize_audited_validation import verify_audited_derivative  # noqa: E402
from run_inference import _run_model  # noqa: E402
from workflow import (  # noqa: E402
    DatasetSample,
    load_flat_yaml,
    record_to_json,
    select_validation_threshold,
    summarize_speed,
    threshold_grid,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DERIVATIVE = Path(
    "data/processed/zenodo_6126677_val_audit_v1/audit_manifest.json"
)
DEFAULT_METRICS = Path(
    "artifacts/perception/label_audit/yolo11s_val_threshold_031_audited_metrics_v1.json"
)
DEFAULT_ADR = Path("docs/decisions/0008-audited-validation-existing-checkpoint-sweep.md")
DEFAULT_EXPERIMENTS = Path("tools/perception/experiments.json")
DEFAULT_EVAL_CONFIG = Path(
    "ros2_ws/src/strawberry_perception/config/eval_yolo11s_640.yaml"
)
DEFAULT_OUTPUT = Path(
    "artifacts/perception/label_audit/yolo11s_audited_checkpoint_sweep_v1"
)
FAMILIES = {
    "baseline": Path("outputs/perception/yolo11s_640/weights"),
    "opt1": Path("outputs/perception/yolo11s_640_cls_pw05_opt1/weights"),
}
EXPECTED_WEIGHT_NAMES = frozenset(
    {"best.pt", "last.pt"} | {f"epoch{epoch}.pt" for epoch in range(0, 101, 10)}
)
WEIGHT_PATTERN = re.compile(r"^(?:best|last|epoch(?:0|10|20|30|40|50|60|70|80|90|100))\.pt$")


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def _display_path(path: Path) -> str:
    try:
        return path.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        return str(path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _write_exclusive(path: Path, value: object) -> None:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite sweep artifact: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_inventory(
    families: Mapping[str, Path] = FAMILIES,
) -> list[dict[str, Any]]:
    inventory = []
    for family in sorted(families):
        directory = _resolve(families[family]).resolve()
        if not directory.is_dir():
            raise ValueError(f"missing checkpoint directory: {directory}")
        actual_names = {path.name for path in directory.glob("*.pt") if path.is_file()}
        if actual_names != EXPECTED_WEIGHT_NAMES:
            raise ValueError(
                f"checkpoint set mismatch for {family}; "
                f"missing={sorted(EXPECTED_WEIGHT_NAMES - actual_names)}, "
                f"extra={sorted(actual_names - EXPECTED_WEIGHT_NAMES)}"
            )
        for name in sorted(actual_names):
            if WEIGHT_PATTERN.fullmatch(name) is None:
                raise ValueError(f"unsupported checkpoint name: {name}")
            path = directory / name
            inventory.append(
                {
                    "checkpoint_id": f"{family}__{path.stem}",
                    "family": family,
                    "name": name,
                    "path": _display_path(path),
                    "size_bytes": path.stat().st_size,
                    "sha256": _sha256(path),
                }
            )
    if len(inventory) != 26 or len({item["checkpoint_id"] for item in inventory}) != 26:
        raise ValueError("frozen checkpoint inventory must contain exactly 26 unique IDs")
    return inventory


def select_winner(candidates: Sequence[Mapping[str, Any]]) -> Mapping[str, Any]:
    if not candidates:
        raise ValueError("checkpoint selection requires candidates")

    def key(item: Mapping[str, Any]) -> tuple[Any, ...]:
        metrics = item["validation_metrics"]
        f1s = [float(metrics["per_class"][class_id]["f1"]) for class_id in ("0", "1")]
        return (
            -float(metrics["macro_f1"]),
            -min(f1s),
            -float(metrics["confidence_threshold"]),
            0 if item["family"] == "baseline" else 1,
            str(item["checkpoint_id"]),
        )

    return min(candidates, key=key)


def _release_gpu() -> None:
    gc.collect()
    try:
        import torch

        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    except ImportError:
        pass


def sweep(
    derivative_manifest_path: Path,
    audited_metrics_path: Path,
    adr_path: Path,
    experiments_path: Path,
    eval_config_path: Path,
    output_dir: Path,
    families: Mapping[str, Path] = FAMILIES,
) -> dict[str, Any]:
    derivative_manifest_path = _resolve(derivative_manifest_path).resolve()
    audited_metrics_path = _resolve(audited_metrics_path).resolve()
    adr_path = _resolve(adr_path).resolve()
    experiments_path = _resolve(experiments_path).resolve()
    eval_config_path = _resolve(eval_config_path).resolve()
    output_dir = _resolve(output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite checkpoint sweep: {output_dir}")

    verification = verify_audited_derivative(derivative_manifest_path)
    derivative = _load_object(derivative_manifest_path)
    audited_metrics = _load_object(audited_metrics_path)
    if audited_metrics.get("kind") != "audited_validation_label_only_diagnostic":
        raise ValueError("unexpected audited metric report")
    metric_binding = audited_metrics.get("bindings", {}).get("audited_derivative_manifest", {})
    if metric_binding.get("sha256") != _sha256(derivative_manifest_path):
        raise ValueError("audited metric report is bound to another derivative")
    if audited_metrics.get("test_split_accessed") is not False:
        raise ValueError("audited metric report does not preserve the test seal")

    experiments = _load_object(experiments_path)
    metric = experiments["metric"]
    candidates = threshold_grid(
        float(metric["threshold_grid"]["start"]),
        float(metric["threshold_grid"]["stop"]),
        float(metric["threshold_grid"]["step"]),
    )
    if len(candidates) != 91 or tuple(int(value) for value in metric["class_ids"]) != (0, 1):
        raise ValueError("checkpoint sweep metric contract changed")
    if float(metric["iou_threshold"]) != 0.5 or float(metric["macro_f1_min"]) != 0.85:
        raise ValueError("checkpoint sweep acceptance contract changed")

    config = load_flat_yaml(eval_config_path)
    expected_config = {
        "imgsz": "640",
        "batch": "8",
        "device": "0",
        "conf": "0.001",
        "iou": "0.70",
    }
    for name, expected in expected_config.items():
        if str(config.get(name)) != expected:
            raise ValueError(f"evaluation config changed: {name}")

    source_manifest_path = _resolve(Path(derivative["source_split_manifest"]["path"])).resolve()
    if _sha256(source_manifest_path) != derivative["source_split_manifest"]["sha256"]:
        raise ValueError("source split manifest changed")
    source_root = source_manifest_path.parent
    derivative_root = derivative_manifest_path.parent
    samples = tuple(
        DatasetSample(
            image_key=str(item["image"]),
            image_path=source_root / str(item["image"]),
            label_path=derivative_root / str(item["audited_label"]),
        )
        for item in sorted(derivative["entries"], key=lambda value: str(value["image"]))
    )
    if len(samples) != 116 or any(not sample.image_path.is_file() for sample in samples):
        raise ValueError("audited validation image set is incomplete")

    inventory = build_inventory(families)
    output_dir.mkdir(parents=True)
    inventory_receipt = {
        "schema_version": 1,
        "kind": "audited_checkpoint_inventory",
        "status": "FROZEN_BEFORE_INFERENCE",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "candidate_count": len(inventory),
        "candidates": inventory,
        "bindings": {
            "adr": {"path": str(adr_path), "sha256": _sha256(adr_path)},
            "audited_derivative": {
                "path": str(derivative_manifest_path),
                "sha256": _sha256(derivative_manifest_path),
                "canonical_sha256": verification[
                    "canonical_validation_derivative_sha256"
                ],
            },
            "audited_metrics": {
                "path": str(audited_metrics_path),
                "sha256": _sha256(audited_metrics_path),
            },
            "experiments": {"path": str(experiments_path), "sha256": _sha256(experiments_path)},
            "eval_config": {"path": str(eval_config_path), "sha256": _sha256(eval_config_path)},
        },
        "test_split_accessed": False,
    }
    inventory_path = output_dir / "inventory.json"
    _write_exclusive(inventory_path, inventory_receipt)

    results = []
    for index, checkpoint in enumerate(inventory, start=1):
        print(
            json.dumps(
                {
                    "event": "checkpoint_start",
                    "index": index,
                    "total": len(inventory),
                    "checkpoint_id": checkpoint["checkpoint_id"],
                }
            ),
            flush=True,
        )
        weights = _resolve(Path(checkpoint["path"])).resolve()
        if _sha256(weights) != checkpoint["sha256"]:
            raise ValueError(f"checkpoint changed after inventory: {weights}")
        records = _run_model(
            weights,
            samples,
            (0, 1),
            raw_confidence=0.001,
            nms_iou=0.70,
            imgsz=640,
            batch_size=8,
            device="0",
        )
        selection = select_validation_threshold(
            records, candidates, iou_threshold=0.5, class_ids=(0, 1)
        )
        candidate_artifact = {
            "schema_version": 1,
            "kind": "audited_checkpoint_prediction_bundle",
            "checkpoint": checkpoint,
            "split": "audited_val_v1",
            "records": [record_to_json(record) for record in records],
            "threshold_selection": selection,
            "speed": summarize_speed(records),
            "test_split_accessed": False,
        }
        artifact_path = output_dir / "candidates" / f"{checkpoint['checkpoint_id']}.json"
        _write_exclusive(artifact_path, candidate_artifact)
        validation_metrics = selection["validation_metrics"]
        summary = {
            "checkpoint_id": checkpoint["checkpoint_id"],
            "family": checkpoint["family"],
            "weight_sha256": checkpoint["sha256"],
            "artifact": artifact_path.relative_to(output_dir).as_posix(),
            "artifact_sha256": _sha256(artifact_path),
            "selected_threshold": selection["selected_threshold"],
            "validation_metrics": validation_metrics,
            "speed": candidate_artifact["speed"],
        }
        results.append(summary)
        print(
            json.dumps(
                {
                    "event": "checkpoint_complete",
                    "checkpoint_id": checkpoint["checkpoint_id"],
                    "macro_f1": validation_metrics["macro_f1"],
                    "threshold": selection["selected_threshold"],
                }
            ),
            flush=True,
        )
        del records
        _release_gpu()

    winner = select_winner(results)
    gate_required = float(metric["macro_f1_min"])
    summary = {
        "schema_version": 1,
        "kind": "audited_checkpoint_sweep_summary",
        "inventory": {
            "path": inventory_path.relative_to(output_dir).as_posix(),
            "sha256": _sha256(inventory_path),
        },
        "candidate_count": len(results),
        "selection_rule": (
            "max_macro_f1_then_max_min_class_f1_then_max_threshold_"
            "then_baseline_then_checkpoint_id"
        ),
        "selected_checkpoint_id": winner["checkpoint_id"],
        "selected_candidate": winner,
        "validation_gate": {
            "required": gate_required,
            "actual": float(winner["validation_metrics"]["macro_f1"]),
            "passed": float(winner["validation_metrics"]["macro_f1"]) >= gate_required,
        },
        "candidates": results,
        "formal_test_authorized": False,
        "training_authorized": False,
        "test_split_accessed": False,
    }
    _write_exclusive(output_dir / "summary.json", summary)
    print(json.dumps({"event": "sweep_complete", **summary["validation_gate"]}), flush=True)
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--derivative-manifest", type=Path, default=DEFAULT_DERIVATIVE)
    parser.add_argument("--audited-metrics", type=Path, default=DEFAULT_METRICS)
    parser.add_argument("--adr", type=Path, default=DEFAULT_ADR)
    parser.add_argument("--experiments", type=Path, default=DEFAULT_EXPERIMENTS)
    parser.add_argument("--eval-config", type=Path, default=DEFAULT_EVAL_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    sweep(
        arguments.derivative_manifest,
        arguments.audited_metrics,
        arguments.adr,
        arguments.experiments,
        arguments.eval_config,
        arguments.output_dir,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
