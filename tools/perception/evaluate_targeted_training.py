#!/usr/bin/env python3
"""Evaluate every preserved ADR-0009 checkpoint on audited validation only."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gc
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from materialize_unripe_exposure_training import (  # noqa: E402
    _load_object,
    _resolve,
    _sha256,
)
from run_inference import _run_model  # noqa: E402
from run_targeted_training import (  # noqa: E402
    _checkpoint_inventory,
    verify_training_derivative,
)
from workflow import (  # noqa: E402
    DatasetSample,
    record_to_json,
    select_validation_threshold,
    summarize_speed,
    threshold_grid,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT = Path("tools/perception/audited_opt2_contract.json")


def _write_exclusive(path: Path, value: object) -> None:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite validation artifact: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def select_winner(candidates: Sequence[Mapping[str, Any]]) -> Mapping[str, Any]:
    if not candidates:
        raise ValueError("targeted checkpoint selection requires candidates")

    def key(item: Mapping[str, Any]) -> tuple[Any, ...]:
        metrics = item["validation_metrics"]
        f1s = [float(metrics["per_class"][class_id]["f1"]) for class_id in ("0", "1")]
        return (
            -float(metrics["macro_f1"]),
            -min(f1s),
            -float(metrics["confidence_threshold"]),
            0 if item["checkpoint_name"] == "best.pt" else 1,
            str(item["checkpoint_name"]),
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


def _validate_completed_claim(
    contract: Mapping[str, Any],
    contract_path: Path,
    current_inventory: Sequence[Mapping[str, Any]],
) -> tuple[Path, dict[str, Any]]:
    claim_path = _resolve(Path(str(contract["training"]["claim"]))).resolve()
    claim = _load_object(claim_path)
    if claim.get("kind") != "one_time_claim" or claim.get("status") != "completed":
        raise ValueError("targeted training claim is not completed")
    if claim.get("purpose") != f"targeted_training:{contract['variant']}":
        raise ValueError("targeted training claim purpose mismatch")
    bindings = claim.get("bindings")
    if not isinstance(bindings, dict) or bindings.get("test_split_accessed") is not False:
        raise ValueError("targeted training claim bindings are incomplete")
    contract_binding = bindings.get("contract", {})
    if (
        contract_binding.get("path") != contract_path.relative_to(REPOSITORY_ROOT).as_posix()
        or contract_binding.get("size_bytes") != contract_path.stat().st_size
        or contract_binding.get("sha256") != _sha256(contract_path)
    ):
        raise ValueError("targeted training claim contract binding mismatch")
    derivative_manifest = _resolve(Path(str(contract["dataset_derivative"]["manifest"]))).resolve()
    derivative_binding = bindings.get("dataset_derivative_manifest", {})
    if (
        derivative_binding.get("path") != derivative_manifest.relative_to(REPOSITORY_ROOT).as_posix()
        or derivative_binding.get("size_bytes") != derivative_manifest.stat().st_size
        or derivative_binding.get("sha256") != _sha256(derivative_manifest)
    ):
        raise ValueError("targeted training claim derivative binding mismatch")
    details = claim.get("details")
    if not isinstance(details, dict) or details.get("return_code") != 0 or details.get("test_split_accessed") is not False:
        raise ValueError("targeted training claim details are incomplete")
    if details.get("checkpoint_inventory") != list(current_inventory):
        raise ValueError("targeted training checkpoints changed after claim completion")
    best = next(item for item in current_inventory if item["name"] == "best.pt")
    if details.get("trained_weight_sha256") != best["sha256"]:
        raise ValueError("targeted training best-weight binding mismatch")
    return claim_path, claim


def evaluate(contract_path: Path) -> dict[str, Any]:
    contract_path = _resolve(contract_path).resolve()
    contract = _load_object(contract_path)
    test_receipt = _resolve(Path(str(contract["formal_test"]["receipt_must_remain_absent"]))).resolve()
    if test_receipt.exists():
        raise FileExistsError("held-out-test receipt exists; refusing validation-only workflow")

    # The training output now exists, so the normal preflight must fail on that
    # condition. Reuse its lower-level contract and derivative validators here.
    derivative_manifest = _resolve(Path(str(contract["dataset_derivative"]["manifest"]))).resolve()
    verification = verify_training_derivative(derivative_manifest, contract_path)
    weights_dir = _resolve(Path(str(contract["training"]["output_dir"]))).resolve() / "weights"
    inventory = _checkpoint_inventory(weights_dir)
    claim_path, claim = _validate_completed_claim(contract, contract_path, inventory)

    output_dir = _resolve(Path(str(contract["validation"]["output_dir"]))).resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite targeted validation: {output_dir}")

    manifest = _load_object(derivative_manifest)
    root = derivative_manifest.parent
    val_entries = sorted(
        (item for item in manifest["entries"] if item["role"] == "audited_validation"),
        key=lambda item: str(item["source_image"]),
    )
    if len(val_entries) != int(contract["dataset_derivative"]["validation_image_count"]):
        raise ValueError("audited validation sample count changed")
    samples = tuple(
        DatasetSample(
            image_key=str(item["source_image"]),
            image_path=root / str(item["derived_image"]),
            label_path=root / str(item["derived_label"]),
        )
        for item in val_entries
    )

    validation = contract["validation"]
    thresholds = threshold_grid(
        float(validation["threshold_grid"]["start"]),
        float(validation["threshold_grid"]["stop"]),
        float(validation["threshold_grid"]["step"]),
    )
    if len(thresholds) != 91:
        raise ValueError("targeted validation threshold grid changed")

    output_dir.mkdir(parents=True)
    frozen_inventory = {
        "schema_version": 1,
        "kind": "targeted_training_checkpoint_inventory",
        "status": "FROZEN_BEFORE_INFERENCE",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "variant": contract["variant"],
        "contract": {"path": contract_path.relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": contract_path.stat().st_size, "sha256": _sha256(contract_path)},
        "training_claim": {"path": claim_path.relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": claim_path.stat().st_size, "sha256": _sha256(claim_path)},
        "training_derivative": {"path": derivative_manifest.relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": derivative_manifest.stat().st_size, "sha256": _sha256(derivative_manifest), "canonical_sha256": verification["canonical_training_derivative_sha256"]},
        "evaluation_runner": {"path": Path(__file__).resolve().relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": Path(__file__).stat().st_size, "sha256": _sha256(Path(__file__))},
        "candidate_count": len(inventory),
        "checkpoints": inventory,
        "selection_rule": validation["checkpoint_rule"],
        "test_split_accessed": False,
    }
    inventory_path = output_dir / "inventory.json"
    _write_exclusive(inventory_path, frozen_inventory)

    results = []
    for index, checkpoint in enumerate(inventory, 1):
        print(json.dumps({"event": "targeted_checkpoint_start", "index": index, "total": len(inventory), "checkpoint": checkpoint["name"]}), flush=True)
        weight_path = _resolve(Path(str(checkpoint["path"]))).resolve()
        if _sha256(weight_path) != checkpoint["sha256"]:
            raise ValueError(f"checkpoint changed after inventory freeze: {weight_path}")
        records = _run_model(
            weight_path,
            samples,
            (0, 1),
            raw_confidence=float(validation["raw_confidence"]),
            nms_iou=float(validation["nms_iou"]),
            imgsz=int(validation["imgsz"]),
            batch_size=int(validation["batch"]),
            device=str(validation["device"]),
        )
        selection = select_validation_threshold(
            records,
            thresholds,
            iou_threshold=float(validation["metric_iou"]),
            class_ids=(0, 1),
        )
        artifact = {
            "schema_version": 1,
            "kind": "targeted_training_audited_validation_predictions",
            "variant": contract["variant"],
            "checkpoint": checkpoint,
            "split": "audited_val_v1",
            "inference": {
                "raw_confidence": validation["raw_confidence"],
                "nms_iou": validation["nms_iou"],
                "imgsz": validation["imgsz"],
                "batch": validation["batch"],
                "device": validation["device"],
            },
            "threshold_selection": selection,
            "speed": summarize_speed(records),
            "records": [record_to_json(record) for record in records],
            "test_split_accessed": False,
        }
        artifact_path = output_dir / "candidates" / f"{weight_path.stem}.json"
        _write_exclusive(artifact_path, artifact)
        result = {
            "checkpoint_name": checkpoint["name"],
            "weight_path": checkpoint["path"],
            "weight_sha256": checkpoint["sha256"],
            "artifact": artifact_path.relative_to(output_dir).as_posix(),
            "artifact_sha256": _sha256(artifact_path),
            "selected_threshold": selection["selected_threshold"],
            "validation_metrics": selection["validation_metrics"],
            "speed": artifact["speed"],
        }
        results.append(result)
        print(json.dumps({"event": "targeted_checkpoint_complete", "checkpoint": checkpoint["name"], "macro_f1": result["validation_metrics"]["macro_f1"], "threshold": result["selected_threshold"]}), flush=True)
        del records
        _release_gpu()

    winner = select_winner(results)
    metrics = winner["validation_metrics"]
    promotion = validation["promotion"]
    checks = {
        "macro_f1": {"required": float(promotion["macro_f1_min"]), "actual": float(metrics["macro_f1"])},
        "ripe_f1": {"required": float(promotion["ripe_f1_min"]), "actual": float(metrics["per_class"]["0"]["f1"])},
        "unripe_f1": {"required": float(promotion["unripe_f1_min"]), "actual": float(metrics["per_class"]["1"]["f1"])},
    }
    for check in checks.values():
        check["passed"] = check["actual"] >= check["required"]
    passed = all(bool(check["passed"]) for check in checks.values())
    summary = {
        "schema_version": 1,
        "kind": "targeted_training_audited_validation_decision",
        "variant": contract["variant"],
        "inventory": {"path": inventory_path.relative_to(output_dir).as_posix(), "sha256": _sha256(inventory_path)},
        "candidate_count": len(results),
        "selection_rule": validation["checkpoint_rule"],
        "selected_checkpoint": winner,
        "promotion_checks": checks,
        "validation_promoted": passed,
        "formal_test_eligible": passed,
        "formal_test_authorized": False,
        "formal_test_run": False,
        "test_split_accessed": False,
        "candidates": results,
    }
    _write_exclusive(output_dir / "summary.json", summary)
    if test_receipt.exists():
        raise RuntimeError("held-out-test receipt appeared during validation")
    print(json.dumps({"event": "targeted_validation_complete", "selected_checkpoint": winner["checkpoint_name"], "macro_f1": metrics["macro_f1"], "validation_promoted": passed}, indent=2), flush=True)
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    arguments = parser.parse_args(argv)
    result = evaluate(arguments.contract)
    return 0 if result["validation_promoted"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
