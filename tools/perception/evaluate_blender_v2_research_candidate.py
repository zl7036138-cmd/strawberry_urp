#!/usr/bin/env python3
"""Evaluate ADR-0037's fixed last.pt at confidence 0.58."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parents[1]
sys.path.insert(0, str(SCRIPT_DIR))

from run_inference import _run_model  # noqa: E402
from workflow import (  # noqa: E402
    DatasetSample,
    evaluate_detection_metrics,
    record_to_json,
    summarize_speed,
    write_json_exclusive,
)


DEFAULT_CONTRACT = Path(
    "tools/perception/blender_v2_research_v1_contract.json"
)
DEFAULT_OUTPUT = Path(
    "artifacts/perception/optimization/"
    "yolo11s_640_blender_v2_research_v1_synthetic_heldout_v1"
)


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else ROOT / path


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    options = parser.parse_args()
    contract_path = _resolve(options.contract).resolve(strict=True)
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    claim_path = _resolve(Path(contract["training"]["claim"])).resolve(
        strict=True
    )
    claim = json.loads(claim_path.read_text(encoding="utf-8"))
    if (
        claim.get("status") != "completed"
        or claim.get("selected_checkpoint_rule") != "last_pt_only"
        or claim.get("formal_real_test_accessed") is not False
    ):
        raise ValueError("ADR-0037 training claim is not a completed last.pt claim")
    selected = claim["selected_weight"]
    weight_path = _resolve(Path(selected["path"])).resolve(strict=True)
    if (
        weight_path.stat().st_size != int(selected["size_bytes"])
        or sha256(weight_path) != selected["sha256"]
        or weight_path.name != "last.pt"
    ):
        raise ValueError("selected last.pt binding changed")
    threshold = float(contract["training"]["runtime_confidence_threshold"])
    if threshold != 0.58:
        raise ValueError("runtime confidence threshold changed")

    dataset_root = _resolve(
        Path("data/processed/yolo11s_640_blender_v2_research_v1")
    ).resolve(strict=True)
    image_paths = sorted((dataset_root / "images" / "val").glob("*.png"))
    label_paths = {
        path.stem: path
        for path in (dataset_root / "labels" / "val").glob("*.txt")
    }
    if len(image_paths) != 24 or {path.stem for path in image_paths} != set(
        label_paths
    ):
        raise ValueError("synthetic held-out split changed")
    samples = tuple(
        DatasetSample(
            image_key=image_path.relative_to(dataset_root).as_posix(),
            image_path=image_path,
            label_path=label_paths[image_path.stem],
        )
        for image_path in image_paths
    )
    output_dir = _resolve(options.output).resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite {output_dir}")
    output_dir.mkdir(parents=True)
    records = _run_model(
        weight_path,
        samples,
        (0, 1),
        raw_confidence=0.001,
        nms_iou=0.70,
        imgsz=640,
        batch_size=8,
        device="0",
    )
    metrics = evaluate_detection_metrics(
        records,
        confidence_threshold=threshold,
        iou_threshold=0.50,
        class_ids=(0, 1),
    )
    promotion = contract["promotion"]
    per_class = metrics["per_class"]
    checks = {
        "ripe_f1": {
            "required": float(
                promotion["synthetic_heldout_ripe_f1_min"]
            ),
            "actual": float(per_class["0"]["f1"]),
        },
        "unripe_f1": {
            "required": float(
                promotion["synthetic_heldout_unripe_f1_min"]
            ),
            "actual": float(per_class["1"]["f1"]),
        },
    }
    for check in checks.values():
        check["passed"] = check["actual"] >= check["required"]
    passed = all(check["passed"] for check in checks.values())
    predictions_path = output_dir / "predictions.json"
    write_json_exclusive(
        predictions_path,
        {
            "schema_version": 1,
            "kind": "blender_v2_research_synthetic_predictions",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "checkpoint": selected,
            "confidence_threshold": threshold,
            "metric_iou": 0.50,
            "metrics": metrics,
            "speed": summarize_speed(records),
            "records": [record_to_json(record) for record in records],
            "formal_real_test_accessed": False,
        },
    )
    summary = {
        "schema_version": 1,
        "kind": "blender_v2_research_synthetic_gate_decision",
        "variant": contract["variant"],
        "selected_checkpoint_rule": "last_pt_only",
        "selected_weight": selected,
        "confidence_threshold": threshold,
        "threshold_search_performed": False,
        "sample_count": 24,
        "metrics": metrics,
        "promotion_checks": checks,
        "synthetic_gate_passed": passed,
        "fail_fast": {
            "audited_real_validation_authorized_only_if_passed": True,
            "audited_real_validation_started": False,
            "live_no_motion_validation_started": False,
        },
        "formal_real_test_accessed": False,
        "formal_simulator_matrix_started": False,
        "perception_control_authorized": False,
        "robot_motion_authorized": False,
        "formal_acceptance": False,
        "predictions": {
            "path": "predictions.json",
            "sha256": sha256(predictions_path),
        },
    }
    write_json_exclusive(output_dir / "summary.json", summary)
    print(
        json.dumps(
            {
                "synthetic_gate_passed": passed,
                "ripe_f1": checks["ripe_f1"],
                "unripe_f1": checks["unripe_f1"],
                "audited_real_validation_started": False,
                "formal_real_test_accessed": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
