#!/usr/bin/env python3
"""Run the one ADR-0009 training claim after fail-closed derivative checks."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from materialize_unripe_exposure_training import (  # noqa: E402
    _load_object,
    _resolve,
    _sha256,
    _validate_contract as validate_unripe_exposure_contract,
    verify_training_derivative as verify_unripe_exposure_derivative,
)
from materialize_train_label_audit import (  # noqa: E402
    _validate_contract as validate_train_label_audit_contract,
    verify_training_derivative as verify_train_label_audit_derivative,
)
from workflow import (  # noqa: E402
    acquire_one_time_claim,
    collect_environment,
    finalize_one_time_claim,
    load_flat_yaml,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT = Path("tools/perception/audited_opt2_contract.json")
BASELINE_CONFIG = Path("ros2_ws/src/strawberry_perception/config/train_yolo11s_640.yaml")
CHECKPOINT_PATTERN = re.compile(r"^(?:best|last|epoch\d+)\.pt$")


def _validate_contract(contract_path: Path) -> tuple[dict[str, Any], dict[str, Path]]:
    contract = _load_object(_resolve(contract_path).resolve())
    if contract.get("kind") == "training_label_audit_training_contract":
        return validate_train_label_audit_contract(contract_path)
    return validate_unripe_exposure_contract(contract_path)


def verify_training_derivative(manifest_path: Path, contract_path: Path) -> dict[str, Any]:
    contract = _load_object(_resolve(contract_path).resolve())
    if contract.get("kind") == "training_label_audit_training_contract":
        return verify_train_label_audit_derivative(manifest_path, contract_path)
    return verify_unripe_exposure_derivative(manifest_path, contract_path)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _resolve_yolo(value: str) -> str | None:
    executable = shutil.which(value)
    if executable is not None:
        return executable
    return shutil.which(value, path=str(Path(sys.executable).parent))


def _validate_config(contract: Mapping[str, Any], config_path: Path) -> Mapping[str, str]:
    baseline_path = _resolve(BASELINE_CONFIG).resolve()
    baseline = dict(load_flat_yaml(baseline_path))
    candidate = dict(load_flat_yaml(config_path))
    expected_data = str(contract["dataset_derivative"]["dataset_yaml"])
    expected_name = str(contract["variant"])
    if candidate.get("data") != expected_data or candidate.get("name") != expected_name:
        raise ValueError("targeted training data or name does not match the contract")
    baseline["data"] = expected_data
    baseline["name"] = expected_name
    if candidate != baseline:
        changed = sorted(key for key in set(candidate) | set(baseline) if candidate.get(key) != baseline.get(key))
        raise ValueError(f"targeted training changes settings beyond data and name: {changed}")
    if candidate.get("model") != contract["base_weight"]["path"]:
        raise ValueError("targeted training model does not match pinned base weight")
    return candidate


def _checkpoint_inventory(weights_dir: Path) -> list[dict[str, Any]]:
    paths = sorted(path for path in weights_dir.glob("*.pt") if path.is_file())
    names = {path.name for path in paths}
    if not {"best.pt", "last.pt"}.issubset(names):
        raise ValueError("completed training must preserve best.pt and last.pt")
    if any(CHECKPOINT_PATTERN.fullmatch(path.name) is None for path in paths):
        raise ValueError("completed training contains an uncontracted checkpoint name")
    if len(names) != len(paths):
        raise ValueError("completed training checkpoint names are not unique")
    return [
        {
            "name": path.name,
            "path": path.relative_to(REPOSITORY_ROOT).as_posix(),
            "size_bytes": path.stat().st_size,
            "sha256": _sha256(path),
        }
        for path in paths
    ]


def _preflight(contract_path: Path) -> tuple[dict[str, Any], dict[str, Any], list[str], Path, Path]:
    contract_path = _resolve(contract_path).resolve()
    contract, contract_paths = _validate_contract(contract_path)
    derivative_manifest = _resolve(Path(str(contract["dataset_derivative"]["manifest"]))).resolve()
    derivative_verification = verify_training_derivative(derivative_manifest, contract_path)
    dataset_yaml = _resolve(Path(str(contract["dataset_derivative"]["dataset_yaml"]))).resolve()
    if dataset_yaml.parent != derivative_manifest.parent:
        raise ValueError("targeted dataset YAML is outside the verified derivative")

    config_path = contract_paths["config"]
    config = _validate_config(contract, config_path)
    base_weight_binding = contract["base_weight"]
    base_weight = _resolve(Path(str(base_weight_binding["path"]))).resolve()
    if not base_weight.is_file() or base_weight.stat().st_size != int(base_weight_binding["size_bytes"]) or _sha256(base_weight) != base_weight_binding["sha256"]:
        raise ValueError("pinned YOLO11s base weight changed")

    output_dir = _resolve(Path(str(contract["training"]["output_dir"]))).resolve()
    trained_weight = _resolve(Path(str(contract["training"]["trained_weight"]))).resolve()
    if trained_weight != output_dir / "weights" / "best.pt":
        raise ValueError("contracted best weight is outside the fixed output directory")
    if output_dir.exists():
        raise FileExistsError(f"targeted training output already exists: {output_dir}")

    executable = _resolve_yolo("yolo")
    command = [
        executable or "yolo",
        "detect",
        "train",
        f"cfg={config_path}",
        f"project={output_dir.parent}",
        f"name={output_dir.name}",
        "exist_ok=False",
    ]
    bindings = {
        "variant": contract["variant"],
        "contract": {"path": contract_path.relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": contract_path.stat().st_size, "sha256": _sha256(contract_path)},
        "authorization": {"path": contract_paths["authorization"].relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": contract_paths["authorization"].stat().st_size, "sha256": _sha256(contract_paths["authorization"])},
        "decision_record": {"path": contract_paths["decision"].relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": contract_paths["decision"].stat().st_size, "sha256": _sha256(contract_paths["decision"])},
        "training_config": {"path": config_path.relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": config_path.stat().st_size, "sha256": _sha256(config_path)},
        "baseline_training_config": {"path": BASELINE_CONFIG.as_posix(), "sha256": _sha256(_resolve(BASELINE_CONFIG))},
        "dataset_derivative_manifest": {"path": derivative_manifest.relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": derivative_manifest.stat().st_size, "sha256": _sha256(derivative_manifest), "canonical_sha256": derivative_verification["canonical_training_derivative_sha256"]},
        "dataset_yaml": {"path": dataset_yaml.relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": dataset_yaml.stat().st_size, "sha256": _sha256(dataset_yaml)},
        "base_weight": {"path": base_weight.relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": base_weight.stat().st_size, "sha256": _sha256(base_weight)},
        "training_runner": {"path": Path(__file__).resolve().relative_to(REPOSITORY_ROOT).as_posix(), "size_bytes": Path(__file__).stat().st_size, "sha256": _sha256(Path(__file__))},
        "expected_trained_weight": contract["training"]["trained_weight"],
        "command": command,
        "test_split_accessed": False,
    }
    preview = {
        "schema_version": 1,
        "kind": "targeted_training_preflight",
        "bindings": bindings,
        "training_config": config,
        "derivative_verification": derivative_verification,
        "environment": collect_environment(),
    }
    return preview, bindings, command, trained_weight, output_dir


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--preflight", action="store_true")
    arguments = parser.parse_args(argv)
    preview, bindings, command, trained_weight, output_dir = _preflight(arguments.contract)
    if arguments.preflight:
        print(json.dumps(preview, indent=2, sort_keys=True))
        return 0

    executable = _resolve_yolo("yolo")
    if executable is None:
        raise FileNotFoundError("YOLO executable not found")
    command[0] = executable
    contract = _load_object(_resolve(arguments.contract).resolve())
    claim_path = _resolve(Path(str(contract["training"]["claim"]))).resolve()
    log_path = _resolve(Path(str(contract["training"]["log"]))).resolve()
    if claim_path.exists() or log_path.exists():
        raise FileExistsError("targeted training claim or log already exists")

    token = acquire_one_time_claim(
        claim_path,
        f"targeted_training:{contract['variant']}",
        bindings,
        _utc_now(),
    )
    log_path.parent.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    process: subprocess.Popen[str] | None = None
    try:
        with log_path.open("x", encoding="utf-8", buffering=1) as log:
            process = subprocess.Popen(
                command,
                cwd=REPOSITORY_ROOT,
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
            finalize_one_time_claim(
                claim_path,
                token,
                "failed",
                _utc_now(),
                {"return_code": return_code, "elapsed_sec": time.monotonic() - start, "log": str(log_path)},
            )
            return return_code
        if not trained_weight.is_file():
            raise FileNotFoundError(f"training returned success without best.pt: {trained_weight}")
        inventory = _checkpoint_inventory(trained_weight.parent)
        finalize_one_time_claim(
            claim_path,
            token,
            "completed",
            _utc_now(),
            {
                "return_code": 0,
                "elapsed_sec": time.monotonic() - start,
                "log": str(log_path),
                "trained_weight": str(contract["training"]["trained_weight"]),
                "trained_weight_sha256": _sha256(trained_weight),
                "checkpoint_inventory": inventory,
                "environment": collect_environment(),
                "test_split_accessed": False,
            },
        )
        print(json.dumps({"status": "completed", "checkpoint_count": len(inventory), "claim": str(claim_path)}, indent=2))
        return 0
    except BaseException as error:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=10)
        try:
            finalize_one_time_claim(
                claim_path,
                token,
                "failed",
                _utc_now(),
                {"elapsed_sec": time.monotonic() - start, "log": str(log_path), "error_type": type(error).__name__, "error": str(error)},
            )
        except Exception:
            pass
        if output_dir.exists():
            print(f"failed output preserved at {output_dir}", file=sys.stderr)
        raise


if __name__ == "__main__":
    raise SystemExit(main())
