#!/usr/bin/env python3
"""Acquire and execute ADR-0037's single Blender-v2 training claim."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

import yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT = Path(
    "tools/perception/blender_v2_research_v1_contract.json"
)


def _resolve(value: object) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else ROOT / path


def _display(path: Path) -> str:
    return path.resolve().relative_to(ROOT.resolve()).as_posix()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _bound(record: dict, label: str) -> Path:
    path = _resolve(record["path"]).resolve(strict=True)
    if (
        path.stat().st_size != int(record["size_bytes"])
        or sha256(path) != record["sha256"]
    ):
        raise ValueError(f"{label} binding changed")
    return path


def _write_json_exclusive(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _replace_json(path: Path, value: dict) -> None:
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _preflight(contract_path: Path) -> tuple[dict, dict[str, Path]]:
    contract_path = contract_path.resolve(strict=True)
    contract = _load(contract_path)
    if (
        contract.get("kind")
        != "blender_v2_unripe_adaptation_research_contract"
        or contract.get("status") != "FROZEN_TRAINING_AUTHORIZED"
        or contract.get("variant")
        != "yolo11s_640_blender_v2_research_v1"
    ):
        raise ValueError("unexpected ADR-0037 training contract")
    training = contract["training"]
    if (
        int(training.get("maximum_claims", 0)) != 1
        or training.get("retry_authorized") is not False
        or training.get("selected_checkpoint_rule") != "last_pt_only"
        or float(training.get("runtime_confidence_threshold")) != 0.58
    ):
        raise ValueError("training claim or selection boundary changed")
    safety = contract["safety"]
    for key in (
        "formal_real_test_access_authorized",
        "formal_simulator_matrix_authorized",
        "perception_control_authorized",
        "robot_motion_authorized",
        "sim_to_real_claim_authorized",
    ):
        if safety.get(key) is not False:
            raise ValueError(f"unsafe authorization: {key}")
    if _resolve(
        safety["formal_real_test_receipt_must_remain_absent"]
    ).exists():
        raise FileExistsError("sealed real-test receipt exists")

    paths = {
        "decision": _bound(contract["decision_record"], "decision record"),
        "capture": _bound(contract["capture_receipt"], "capture receipt"),
        "visual_qa": _bound(contract["visual_qa"], "visual QA"),
        "dataset": _bound(
            contract["dataset_derivative"], "dataset derivative"
        ),
        "base_weight": _bound(contract["base_weight"], "base weight"),
        "config": _bound(training["config"], "training config"),
    }
    capture = _load(paths["capture"])
    if (
        capture.get("capture_preflight_passed") is not True
        or capture.get("canonical_synthetic_dataset_sha256")
        != contract["capture_receipt"][
            "canonical_synthetic_dataset_sha256"
        ]
        or capture.get("training_started") is not False
        or capture.get("held_out_test_consumed") is not False
    ):
        raise ValueError("capture receipt no longer unlocks training")
    visual = _load(paths["visual_qa"])
    if visual.get("status") != contract["visual_qa"]["required_status"]:
        raise ValueError("visual QA did not pass")
    derivative = _load(paths["dataset"])
    expected_counts = contract["dataset_derivative"]
    if (
        derivative.get("canonical_training_derivative_sha256")
        != expected_counts["canonical_training_derivative_sha256"]
        or derivative.get("test_split_present") is not False
        or derivative.get("training_started") is not False
        or derivative.get("counts", {}).get("train_images")
        != expected_counts["train_images"]
        or derivative.get("counts", {}).get("real_train_images")
        != expected_counts["real_train_images"]
        or derivative.get("counts", {}).get("synthetic_train_images")
        != expected_counts["synthetic_train_images"]
        or derivative.get("counts", {}).get("synthetic_validation_images")
        != expected_counts["synthetic_validation_images"]
    ):
        raise ValueError("training derivative contract changed")

    config = yaml.safe_load(paths["config"].read_text(encoding="utf-8"))
    required_config = {
        "model": contract["base_weight"]["path"],
        "data": (
            "data/processed/yolo11s_640_blender_v2_research_v1/"
            "dataset.yaml"
        ),
        "imgsz": 640,
        "epochs": 12,
        "batch": 8,
        "device": 0,
        "optimizer": "SGD",
        "lr0": 0.0001,
        "seed": 20260726,
        "deterministic": True,
        "amp": False,
        "patience": 0,
        "name": contract["variant"],
        "exist_ok": False,
    }
    for key, expected in required_config.items():
        if config.get(key) != expected:
            raise ValueError(f"training config changed: {key}")
    return contract, paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--preflight", action="store_true")
    options = parser.parse_args()
    contract_path = _resolve(options.contract).resolve(strict=True)
    contract, paths = _preflight(contract_path)
    training = contract["training"]
    output_dir = _resolve(training["output_dir"]).resolve()
    selected_weight = _resolve(training["selected_weight"]).resolve()
    claim_path = _resolve(training["claim"]).resolve()
    log_path = _resolve(training["log"]).resolve()
    for path, label in (
        (output_dir, "training output"),
        (claim_path, "one-time claim"),
        (log_path, "training log"),
    ):
        if path.exists():
            raise FileExistsError(f"{label} already exists: {path}")

    preview = {
        "schema_version": 1,
        "kind": "blender_v2_research_training_preflight",
        "variant": contract["variant"],
        "contract": {
            "path": _display(contract_path),
            "size_bytes": contract_path.stat().st_size,
            "sha256": sha256(contract_path),
        },
        "bindings": {
            key: {
                "path": _display(path),
                "size_bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
            for key, path in paths.items()
        },
        "selected_checkpoint_rule": "last_pt_only",
        "runtime_confidence_threshold": 0.58,
        "formal_real_test_accessed": False,
        "formal_simulator_matrix_started": False,
        "perception_control_authorized": False,
        "robot_motion_authorized": False,
        "training_preflight_passed": True,
        "training_started": False,
    }
    if options.preflight:
        print(json.dumps(preview, indent=2, sort_keys=True))
        return 0

    executable = Path("/opt/strawberry_venv/bin/yolo")
    if not executable.is_file():
        raise FileNotFoundError(f"YOLO executable is missing: {executable}")
    command = [
        str(executable),
        "detect",
        "train",
        f"cfg={paths['config']}",
        f"project={output_dir.parent}",
        f"name={output_dir.name}",
        "exist_ok=False",
    ]
    claim = {
        **preview,
        "kind": "blender_v2_research_training_claim",
        "status": "running",
        "claim_acquired_utc": datetime.now(timezone.utc).isoformat(),
        "training_started": True,
        "command": command,
    }
    _write_json_exclusive(claim_path, claim)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    process = None
    try:
        with log_path.open("x", encoding="utf-8", buffering=1) as log:
            process = subprocess.Popen(
                command,
                cwd=ROOT,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            assert process.stdout is not None
            for line in process.stdout:
                sys.stdout.write(line)
                log.write(line)
            return_code = process.wait()
        if return_code != 0:
            raise RuntimeError(f"training returned {return_code}")
        if not selected_weight.is_file():
            raise FileNotFoundError(
                f"last.pt is missing after training: {selected_weight}"
            )
        claim.update(
            {
                "status": "completed",
                "completed_utc": datetime.now(timezone.utc).isoformat(),
                "elapsed_sec": time.monotonic() - started,
                "return_code": 0,
                "selected_weight": {
                    "path": _display(selected_weight),
                    "size_bytes": selected_weight.stat().st_size,
                    "sha256": sha256(selected_weight),
                },
                "formal_real_test_accessed": False,
                "formal_simulator_matrix_started": False,
                "perception_control_authorized": False,
                "robot_motion_authorized": False,
            }
        )
        _replace_json(claim_path, claim)
        print(
            json.dumps(
                {
                    "status": "completed",
                    "selected_weight": claim["selected_weight"],
                    "elapsed_sec": claim["elapsed_sec"],
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0
    except BaseException as error:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=10)
        claim.update(
            {
                "status": "failed",
                "completed_utc": datetime.now(timezone.utc).isoformat(),
                "elapsed_sec": time.monotonic() - started,
                "error_type": type(error).__name__,
                "error": str(error),
            }
        )
        _replace_json(claim_path, claim)
        raise


if __name__ == "__main__":
    raise SystemExit(main())
