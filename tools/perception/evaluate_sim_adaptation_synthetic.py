#!/usr/bin/env python3
"""Select ADR-0021 checkpoint and threshold on synthetic held-out data only."""

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

from materialize_sim_adaptation_training import (  # noqa: E402
    DEFAULT_CONTRACT,
    REPOSITORY_ROOT,
    _display,
    _load_object,
    _resolve,
    _sha256,
    verify_training_derivative,
)
from run_inference import _run_model  # noqa: E402
from run_sim_adaptation_training import _checkpoint_inventory  # noqa: E402
from workflow import (  # noqa: E402
    DatasetSample,
    record_to_json,
    select_validation_threshold,
    summarize_speed,
    threshold_grid,
    write_json_exclusive,
)


DEFAULT_OUTPUT = Path(
    "artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_synthetic_heldout_v1"
)


def select_winner(candidates: Sequence[Mapping[str, Any]]) -> Mapping[str, Any]:
    if not candidates:
        raise ValueError("synthetic checkpoint selection requires candidates")

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


def _validate_claim(
    contract: Mapping[str, Any],
    contract_path: Path,
    inventory: Sequence[Mapping[str, Any]],
) -> tuple[Path, dict[str, Any]]:
    claim_path = _resolve(Path(str(contract["training"]["claim"]))).resolve()
    claim = _load_object(claim_path)
    if claim.get("kind") != "one_time_claim" or claim.get("status") != "completed":
        raise ValueError("simulator-adaptation training claim is not completed")
    if claim.get("purpose") != f"simulator_adaptation_training:{contract['variant']}":
        raise ValueError("simulator-adaptation claim purpose mismatch")
    bindings = claim.get("bindings")
    details = claim.get("details")
    if not isinstance(bindings, dict) or not isinstance(details, dict):
        raise ValueError("simulator-adaptation claim is incomplete")
    contract_binding = bindings.get("contract", {})
    if (
        contract_binding.get("path") != _display(contract_path)
        or contract_binding.get("size_bytes") != contract_path.stat().st_size
        or contract_binding.get("sha256") != _sha256(contract_path)
    ):
        raise ValueError("simulator-adaptation claim contract binding mismatch")
    derivative_manifest = _resolve(Path(str(contract["dataset_derivative"]["manifest"]))).resolve()
    derivative_binding = bindings.get("dataset_manifest", {})
    if (
        derivative_binding.get("path") != _display(derivative_manifest)
        or derivative_binding.get("size_bytes") != derivative_manifest.stat().st_size
        or derivative_binding.get("sha256") != _sha256(derivative_manifest)
    ):
        raise ValueError("simulator-adaptation claim derivative binding mismatch")
    if details.get("return_code") != 0 or details.get("checkpoint_inventory") != list(inventory):
        raise ValueError("simulator-adaptation checkpoint inventory changed")
    for location in (bindings, details):
        if location.get("formal_real_test_accessed") is not False:
            raise ValueError("claim does not preserve the formal real-test seal")
        if location.get("formal_simulator_matrix_started") is not False:
            raise ValueError("claim crossed the formal simulator boundary")
        if location.get("perception_control_authorized") is not False or location.get("robot_motion_authorized") is not False:
            raise ValueError("claim crossed the no-motion safety boundary")
    return claim_path, claim


def evaluate(contract_path: Path, output_dir: Path) -> dict[str, Any]:
    contract_path = _resolve(contract_path).resolve()
    contract = _load_object(contract_path)
    if contract.get("kind") != "simulator_adaptation_training_contract":
        raise ValueError("unexpected simulator-adaptation contract")
    safety = contract["safety"]
    test_receipt = _resolve(Path(str(safety["formal_real_test_receipt_must_remain_absent"]))).resolve()
    if test_receipt.exists():
        raise FileExistsError("formal real-test receipt exists")
    if safety.get("formal_simulator_matrix_authorized") is not False:
        raise ValueError("formal simulator matrix must remain unauthorized")

    derivative_manifest = _resolve(Path(str(contract["dataset_derivative"]["manifest"]))).resolve()
    verification = verify_training_derivative(derivative_manifest, contract_path)
    weights_dir = _resolve(Path(str(contract["training"]["output_dir"]))).resolve() / "weights"
    inventory = _checkpoint_inventory(weights_dir)
    claim_path, _ = _validate_claim(contract, contract_path, inventory)

    output_dir = _resolve(output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite synthetic evaluation: {output_dir}")
    manifest = _load_object(derivative_manifest)
    root = derivative_manifest.parent
    entries = sorted(
        (item for item in manifest["entries"] if item["role"] == "synthetic_heldout"),
        key=lambda item: str(item["source_image"]),
    )
    if len(entries) != int(contract["dataset_derivative"]["validation_image_count"]):
        raise ValueError("synthetic held-out sample count changed")
    if any(item["role"] != "synthetic_heldout" for item in entries):
        raise ValueError("checkpoint selection includes a non-synthetic sample")
    samples = tuple(
        DatasetSample(
            image_key=str(item["source_image"]),
            image_path=root / str(item["derived_image"]),
            label_path=root / str(item["derived_label"]),
        )
        for item in entries
    )

    evaluation = contract["selection_and_evaluation"]
    if evaluation.get("synthetic_heldout_only_for_checkpoint_and_threshold_selection") is not True:
        raise ValueError("contract does not preserve synthetic-only selection")
    thresholds = threshold_grid(
        float(evaluation["threshold_grid"]["start"]),
        float(evaluation["threshold_grid"]["stop"]),
        float(evaluation["threshold_grid"]["step"]),
    )
    if len(thresholds) != 91:
        raise ValueError("synthetic threshold grid changed")

    output_dir.mkdir(parents=True)
    evaluator_path = Path(__file__).resolve()
    frozen_inventory = {
        "schema_version": 1,
        "kind": "simulator_adaptation_synthetic_checkpoint_inventory",
        "status": "FROZEN_BEFORE_INFERENCE",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "variant": contract["variant"],
        "contract": {"path": _display(contract_path), "size_bytes": contract_path.stat().st_size, "sha256": _sha256(contract_path)},
        "training_claim": {"path": _display(claim_path), "size_bytes": claim_path.stat().st_size, "sha256": _sha256(claim_path)},
        "training_derivative": {"path": _display(derivative_manifest), "size_bytes": derivative_manifest.stat().st_size, "sha256": _sha256(derivative_manifest), "canonical_sha256": verification["canonical_training_derivative_sha256"]},
        "evaluation_runner": {"path": _display(evaluator_path), "size_bytes": evaluator_path.stat().st_size, "sha256": _sha256(evaluator_path)},
        "candidate_count": len(inventory),
        "checkpoints": inventory,
        "selection_sample_role": "synthetic_heldout",
        "selection_sample_count": len(samples),
        "selection_rule": evaluation["checkpoint_rule"],
        "audited_real_validation_accessed": False,
        "formal_real_test_accessed": False,
        "formal_simulator_matrix_started": False,
    }
    inventory_path = output_dir / "inventory.json"
    write_json_exclusive(inventory_path, frozen_inventory)

    results: list[dict[str, Any]] = []
    for index, checkpoint in enumerate(inventory, 1):
        print(json.dumps({"event": "synthetic_checkpoint_start", "index": index, "total": len(inventory), "checkpoint": checkpoint["name"]}), flush=True)
        weight_path = _resolve(Path(str(checkpoint["path"]))).resolve()
        if _sha256(weight_path) != checkpoint["sha256"]:
            raise ValueError(f"checkpoint changed after inventory freeze: {weight_path}")
        records = _run_model(
            weight_path,
            samples,
            (0, 1),
            raw_confidence=float(evaluation["raw_confidence"]),
            nms_iou=float(evaluation["nms_iou"]),
            imgsz=int(evaluation["imgsz"]),
            batch_size=int(evaluation["batch"]),
            device=str(evaluation["device"]),
        )
        selection = select_validation_threshold(
            records,
            thresholds,
            iou_threshold=float(evaluation["metric_iou"]),
            class_ids=(0, 1),
        )
        artifact = {
            "schema_version": 1,
            "kind": "simulator_adaptation_synthetic_heldout_predictions",
            "variant": contract["variant"],
            "checkpoint": checkpoint,
            "split": "synthetic_heldout",
            "sample_count": len(samples),
            "inference": {
                "raw_confidence": evaluation["raw_confidence"],
                "nms_iou": evaluation["nms_iou"],
                "imgsz": evaluation["imgsz"],
                "batch": evaluation["batch"],
                "device": evaluation["device"],
            },
            "threshold_selection": selection,
            "speed": summarize_speed(records),
            "records": [record_to_json(record) for record in records],
            "audited_real_validation_accessed": False,
            "formal_real_test_accessed": False,
        }
        artifact_path = output_dir / "candidates" / f"{weight_path.stem}.json"
        write_json_exclusive(artifact_path, artifact)
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
        print(json.dumps({"event": "synthetic_checkpoint_complete", "checkpoint": checkpoint["name"], "macro_f1": result["validation_metrics"]["macro_f1"], "threshold": result["selected_threshold"]}), flush=True)
        del records
        _release_gpu()

    winner = select_winner(results)
    macro_f1 = float(winner["validation_metrics"]["macro_f1"])
    required = float(evaluation["promotion"]["synthetic_heldout_macro_f1_min"])
    summary = {
        "schema_version": 1,
        "kind": "simulator_adaptation_synthetic_selection_decision",
        "variant": contract["variant"],
        "inventory": {"path": inventory_path.relative_to(output_dir).as_posix(), "sha256": _sha256(inventory_path)},
        "candidate_count": len(results),
        "selection_source": "synthetic_heldout_only",
        "selection_sample_count": len(samples),
        "selection_rule": evaluation["checkpoint_rule"],
        "selected_checkpoint": winner,
        "synthetic_gate": {"metric": "macro_f1", "operator": ">=", "required": required, "actual": macro_f1, "passed": macro_f1 >= required},
        "synthetic_gate_passed": macro_f1 >= required,
        "audited_real_validation_accessed": False,
        "formal_real_test_accessed": False,
        "formal_simulator_matrix_started": False,
        "perception_control_authorized": False,
        "robot_motion_authorized": False,
        "formal_acceptance": False,
        "candidates": results,
    }
    write_json_exclusive(output_dir / "summary.json", summary)
    if test_receipt.exists():
        raise RuntimeError("formal real-test receipt appeared during synthetic evaluation")
    print(json.dumps({"event": "synthetic_selection_complete", "selected_checkpoint": winner["checkpoint_name"], "threshold": winner["selected_threshold"], "macro_f1": macro_f1, "synthetic_gate_passed": summary["synthetic_gate_passed"]}, indent=2), flush=True)
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    result = evaluate(arguments.contract, arguments.output)
    return 0 if result["synthetic_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
