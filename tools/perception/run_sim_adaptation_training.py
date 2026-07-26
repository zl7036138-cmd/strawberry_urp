#!/usr/bin/env python3
"""Preflight or consume ADR-0021's single simulator-adaptation claim."""

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

from materialize_sim_adaptation_training import (  # noqa: E402
    DEFAULT_CONTRACT,
    REPOSITORY_ROOT,
    _canonical_sha256,
    _display,
    _load_object,
    _resolve,
    _sha256,
    _validate_contract,
    verify_training_derivative,
)
from workflow import (  # noqa: E402
    acquire_one_time_claim,
    collect_environment,
    finalize_one_time_claim,
    load_flat_yaml,
    write_json_exclusive,
)


CHECKPOINT_PATTERN = re.compile(r"^(?:best|last|epoch\d+)\.pt$")


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _resolve_yolo() -> str | None:
    executable = shutil.which("yolo")
    if executable is not None:
        return executable
    executable = shutil.which("yolo", path=str(Path(sys.executable).parent))
    if executable is not None:
        return executable
    frozen = Path("/opt/strawberry_venv/bin/yolo")
    return str(frozen) if frozen.is_file() else None


def _validate_training_environment(environment: Mapping[str, Any], executable: str | None) -> None:
    if executable is None or Path(executable).resolve() != Path("/opt/strawberry_venv/bin/yolo"):
        raise ValueError("frozen /opt/strawberry_venv/bin/yolo executable is required")
    if Path(sys.prefix).resolve() != Path("/opt/strawberry_venv").resolve():
        raise ValueError("preflight must run with /opt/strawberry_venv/bin/python")
    packages = environment.get("packages")
    if not isinstance(packages, Mapping):
        raise ValueError("training environment package inventory is missing")
    for name in ("numpy", "opencv-python", "torch", "torchvision", "ultralytics"):
        if not packages.get(name):
            raise ValueError(f"training environment is missing {name}")
    gpu = environment.get("nvidia_smi")
    if not isinstance(gpu, list) or not gpu or "RTX 4060" not in str(gpu[0]):
        raise ValueError("frozen RTX 4060 training GPU is unavailable")


def _yaml_scalar(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float, str)):
        return str(value)
    raise ValueError(f"training hyperparameter is not a flat YAML scalar: {value!r}")


def _validate_config(contract: Mapping[str, Any], config_path: Path) -> dict[str, str]:
    actual = dict(load_flat_yaml(config_path))
    expected = {
        str(key): _yaml_scalar(value)
        for key, value in contract["training"]["hyperparameters"].items()
    }
    if actual != expected:
        changed = sorted(key for key in set(actual) | set(expected) if actual.get(key) != expected.get(key))
        raise ValueError(f"training config differs from frozen hyperparameters: {changed}")
    if actual["model"] != contract["base_weight"]["path"]:
        raise ValueError("training config does not use baseline__best")
    if actual["data"] != contract["dataset_derivative"]["dataset_yaml"]:
        raise ValueError("training config does not use the frozen derivative")
    if actual["name"] != contract["variant"] or actual["exist_ok"] != "false":
        raise ValueError("training output identity is not fail-closed")
    return actual


def _checkpoint_inventory(weights_dir: Path) -> list[dict[str, Any]]:
    paths = sorted(path for path in weights_dir.glob("*.pt") if path.is_file())
    names = {path.name for path in paths}
    if not {"best.pt", "last.pt"}.issubset(names):
        raise ValueError("completed training must preserve best.pt and last.pt")
    if any(CHECKPOINT_PATTERN.fullmatch(path.name) is None for path in paths):
        raise ValueError("completed training contains an uncontracted checkpoint name")
    return [
        {
            "name": path.name,
            "path": path.relative_to(REPOSITORY_ROOT).as_posix(),
            "size_bytes": path.stat().st_size,
            "sha256": _sha256(path),
        }
        for path in paths
    ]


def _build_preflight(contract_path: Path) -> tuple[dict[str, Any], list[str], Path, Path, Path]:
    contract_path = _resolve(contract_path).resolve()
    contract, paths = _validate_contract(contract_path)
    derivative_manifest = _resolve(Path(str(contract["dataset_derivative"]["manifest"]))).resolve()
    derivative = verify_training_derivative(derivative_manifest, contract_path)
    dataset_yaml = _resolve(Path(str(contract["dataset_derivative"]["dataset_yaml"]))).resolve()
    if dataset_yaml.parent != derivative_manifest.parent:
        raise ValueError("dataset YAML is outside the verified derivative")
    config = _validate_config(contract, paths["config"])

    output_dir = _resolve(Path(str(contract["training"]["output_dir"]))).resolve()
    trained_weight = _resolve(Path(str(contract["training"]["trained_weight"]))).resolve()
    if trained_weight != output_dir / "weights" / "best.pt":
        raise ValueError("trained weight is outside the frozen output directory")
    if output_dir.exists():
        raise FileExistsError(f"simulator-adaptation output already exists: {output_dir}")
    claim_path = _resolve(Path(str(contract["training"]["claim"]))).resolve()
    log_path = _resolve(Path(str(contract["training"]["log"]))).resolve()
    if claim_path.exists() or log_path.exists():
        raise FileExistsError("simulator-adaptation claim or log already exists")

    executable = _resolve_yolo()
    environment = collect_environment()
    _validate_training_environment(environment, executable)
    command = [
        executable,
        "detect",
        "train",
        f"cfg={paths['config']}",
        f"project={output_dir.parent}",
        f"name={output_dir.name}",
        "exist_ok=False",
    ]
    runner_path = Path(__file__).resolve()
    materializer_path = SCRIPT_DIR / "materialize_sim_adaptation_training.py"
    bindings: dict[str, Any] = {
        "variant": contract["variant"],
        "contract": {"path": _display(contract_path), "size_bytes": contract_path.stat().st_size, "sha256": _sha256(contract_path)},
        "authorization": {"path": _display(paths["authorization"]), "size_bytes": paths["authorization"].stat().st_size, "sha256": _sha256(paths["authorization"])},
        "decision_record": {"path": _display(paths["decision"]), "size_bytes": paths["decision"].stat().st_size, "sha256": _sha256(paths["decision"])},
        "training_config": {"path": _display(paths["config"]), "size_bytes": paths["config"].stat().st_size, "sha256": _sha256(paths["config"])},
        "dataset_manifest": {"path": _display(derivative_manifest), "size_bytes": derivative_manifest.stat().st_size, "sha256": _sha256(derivative_manifest), "canonical_sha256": derivative["canonical_training_derivative_sha256"]},
        "dataset_yaml": {"path": _display(dataset_yaml), "size_bytes": dataset_yaml.stat().st_size, "sha256": _sha256(dataset_yaml)},
        "base_weight": {"path": _display(paths["base_weight"]), "size_bytes": paths["base_weight"].stat().st_size, "sha256": _sha256(paths["base_weight"])},
        "synthetic_preflight_receipt": {"path": _display(paths["receipt"]), "size_bytes": paths["receipt"].stat().st_size, "sha256": _sha256(paths["receipt"])},
        "audited_real_validation_manifest": {"path": _display(paths["audit_manifest"]), "size_bytes": paths["audit_manifest"].stat().st_size, "sha256": _sha256(paths["audit_manifest"]), "training_access": False},
        "materializer": {"path": _display(materializer_path), "size_bytes": materializer_path.stat().st_size, "sha256": _sha256(materializer_path)},
        "training_runner": {"path": _display(runner_path), "size_bytes": runner_path.stat().st_size, "sha256": _sha256(runner_path)},
        "expected_trained_weight": contract["training"]["trained_weight"],
        "formal_real_test_accessed": False,
        "formal_simulator_matrix_started": False,
        "perception_control_authorized": False,
        "robot_motion_authorized": False,
    }
    preview = {
        "schema_version": 1,
        "kind": "simulator_adaptation_training_preflight",
        "created_utc": _utc_now(),
        "variant": contract["variant"],
        "bindings": bindings,
        "bindings_canonical_sha256": _canonical_sha256(bindings),
        "training_config": config,
        "derivative_verification": derivative,
        "selection_and_evaluation": contract["selection_and_evaluation"],
        "command": command,
        "environment": environment,
        "training_preflight_passed": True,
        "training_unlocked": True,
        "training_started": False,
        "claim_consumed": False,
        "held_out_real_test_consumed": False,
        "formal_acceptance": False,
    }
    return preview, command, trained_weight, claim_path, log_path


def _validate_frozen_preflight(preview: Mapping[str, Any], receipt_path: Path) -> None:
    receipt = _load_object(receipt_path)
    if receipt.get("kind") != "simulator_adaptation_training_preflight":
        raise ValueError("unexpected frozen training preflight kind")
    if receipt.get("variant") != preview.get("variant"):
        raise ValueError("frozen training preflight variant mismatch")
    if receipt.get("training_preflight_passed") is not True or receipt.get("training_unlocked") is not True:
        raise ValueError("frozen training preflight does not unlock the claim")
    if receipt.get("training_started") is not False or receipt.get("claim_consumed") is not False:
        raise ValueError("frozen training preflight already crossed the claim boundary")
    if receipt.get("bindings_canonical_sha256") != preview.get("bindings_canonical_sha256"):
        raise ValueError("frozen training preflight bindings changed")
    if receipt.get("bindings") != preview.get("bindings"):
        raise ValueError("frozen training preflight files changed")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--preflight", action="store_true", help="print a read-only preflight")
    modes.add_argument("--write-preflight", action="store_true", help="write the frozen preflight receipt exactly once")
    arguments = parser.parse_args(argv)

    preview, command, trained_weight, claim_path, log_path = _build_preflight(arguments.contract)
    contract = _load_object(_resolve(arguments.contract).resolve())
    receipt_path = _resolve(Path(str(contract["training"]["preflight_receipt"]))).resolve()
    if arguments.preflight:
        print(json.dumps(preview, indent=2, sort_keys=True))
        return 0
    if arguments.write_preflight:
        if receipt_path.exists():
            raise FileExistsError(f"refusing to overwrite training preflight: {receipt_path}")
        write_json_exclusive(receipt_path, preview)
        print(json.dumps({"status": "preflight_frozen", "path": _display(receipt_path), "training_unlocked": True, "training_started": False}, indent=2))
        return 0

    if not receipt_path.is_file():
        raise FileNotFoundError("frozen training preflight receipt is required before claim acquisition")
    _validate_frozen_preflight(preview, receipt_path)
    executable = _resolve_yolo()
    if executable is None:
        raise FileNotFoundError("YOLO executable not found")
    command[0] = executable
    bindings = dict(preview["bindings"])
    bindings["frozen_preflight_receipt"] = {
        "path": _display(receipt_path),
        "size_bytes": receipt_path.stat().st_size,
        "sha256": _sha256(receipt_path),
    }
    token = acquire_one_time_claim(
        claim_path,
        f"simulator_adaptation_training:{contract['variant']}",
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
            finalize_one_time_claim(claim_path, token, "failed", _utc_now(), {"return_code": return_code, "elapsed_sec": time.monotonic() - start, "log": _display(log_path)})
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
                "log": _display(log_path),
                "trained_weight": contract["training"]["trained_weight"],
                "trained_weight_sha256": _sha256(trained_weight),
                "checkpoint_inventory": inventory,
                "environment": collect_environment(),
                "formal_real_test_accessed": False,
                "formal_simulator_matrix_started": False,
                "perception_control_authorized": False,
                "robot_motion_authorized": False,
            },
        )
        print(json.dumps({"status": "completed", "checkpoint_count": len(inventory), "claim": _display(claim_path)}, indent=2))
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
                {"elapsed_sec": time.monotonic() - start, "log": _display(log_path), "error_type": type(error).__name__, "error": str(error)},
            )
        except Exception:
            pass
        raise


if __name__ == "__main__":
    raise SystemExit(main())
