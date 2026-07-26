#!/usr/bin/env python3
"""Run one pinned Ultralytics training variant with artifact provenance."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Mapping, Sequence

from workflow import (
    acquire_one_time_claim,
    collect_environment,
    finalize_one_time_claim,
    load_experiment,
    sha256_file,
    validate_split_contract,
    verify_file_contract,
    verify_materialized_dataset,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_EXPERIMENTS = Path(__file__).resolve().with_name("experiments.json")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", required=True)
    parser.add_argument("--experiments", type=Path, default=DEFAULT_EXPERIMENTS)
    parser.add_argument("--claim", type=Path)
    parser.add_argument("--log", type=Path)
    parser.add_argument("--yolo-executable", default="yolo")
    parser.add_argument("--preflight", action="store_true")
    return parser.parse_args(argv)


def _repo_path(value: object) -> Path:
    return (REPOSITORY_ROOT / str(value)).resolve()


def _resolve_yolo_executable(value: str) -> str | None:
    """Resolve YOLO from PATH, then beside the active Python interpreter."""

    executable = shutil.which(value)
    if executable is not None:
        return executable
    # Do not resolve the interpreter symlink: a venv's ``python`` commonly
    # points back to /usr/bin, while its console scripts live beside the
    # unresolved /opt/.../bin/python path.
    return shutil.which(value, path=str(Path(sys.executable).parent))


def _training_command(
    executable: str,
    training_config: Path,
    trained_weight: Path,
) -> list[str]:
    """Pin Ultralytics' output directory to the contracted weight path.

    With ``cfg=...``, Ultralytics 8.4.92 nests a relative YAML ``project``
    below ``runs/detect``.  Absolute CLI overrides avoid that hidden prefix and
    make the produced ``best.pt`` land exactly where the receipt expects it.
    """

    run_directory = trained_weight.parents[1]
    return [
        executable,
        "detect",
        "train",
        f"cfg={training_config}",
        f"project={run_directory.parent}",
        f"name={run_directory.name}",
        "exist_ok=False",
    ]


def _preflight(
    args: argparse.Namespace, spec: Mapping[str, object], variant: Mapping[str, object]
) -> tuple[Mapping[str, object], Path]:
    base = variant.get("base_weight")
    if not isinstance(base, dict):
        raise ValueError("experiment variant has no base-weight contract")
    config = _repo_path(variant["training_config"])
    dataset_yaml = _repo_path(spec["dataset_yaml"])
    split_manifest = _repo_path(spec["split_manifest"])
    trained_weight = _repo_path(variant["trained_weight"])
    for path in (config, dataset_yaml, split_manifest):
        if not path.is_file():
            raise FileNotFoundError(path)
    dataset_report = verify_materialized_dataset(split_manifest)
    split_contract = validate_split_contract(
        spec, REPOSITORY_ROOT, dataset_report
    )
    if dataset_report.get("dataset_id") != spec.get("dataset_id"):
        raise ValueError("materialized dataset id does not match the experiment contract")
    dataset_yaml_sha256 = sha256_file(dataset_yaml)
    if dataset_report.get("dataset_yaml_sha256") != dataset_yaml_sha256:
        raise ValueError("materialized dataset.yaml does not match its verified digest")
    base_path = _repo_path(base["path"])
    verify_file_contract(
        base_path, str(base["sha256"]), int(base["size_bytes"])
    )
    run_directory = trained_weight.parents[1]
    if run_directory.exists():
        raise FileExistsError(
            f"formal training output already exists; refusing an Ultralytics suffix: {run_directory}"
        )
    bindings = {
        "variant": args.variant,
        "role": variant.get("role"),
        "dataset_id": spec.get("dataset_id"),
        "canonical_dataset_sha256": dataset_report["canonical_dataset_sha256"],
        "canonical_split_sha256": dataset_report["canonical_split_sha256"],
        "split_contract": split_contract,
        "verified_sample_count": dataset_report["sample_count"],
        "verified_file_count": dataset_report["verified_file_count"],
        "experiments_sha256": sha256_file(args.experiments),
        "training_config": str(variant["training_config"]),
        "training_config_sha256": sha256_file(config),
        "dataset_yaml": str(spec["dataset_yaml"]),
        "dataset_yaml_sha256": dataset_yaml_sha256,
        "split_manifest": str(spec["split_manifest"]),
        "split_manifest_sha256": sha256_file(split_manifest),
        "base_weight": str(base["path"]),
        "base_weight_sha256": sha256_file(base_path),
        "expected_trained_weight": str(variant["trained_weight"]),
    }
    return bindings, trained_weight


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    spec, variant = load_experiment(args.experiments, args.variant)
    bindings, trained_weight = _preflight(args, spec, variant)
    executable = _resolve_yolo_executable(args.yolo_executable)
    command = _training_command(
        executable or args.yolo_executable,
        _repo_path(variant["training_config"]),
        trained_weight,
    )
    preview = {
        "schema_version": 1,
        "kind": "training_preflight",
        "command": command,
        "bindings": bindings,
        "environment": collect_environment(),
    }
    if args.preflight:
        print(json.dumps(preview, indent=2, ensure_ascii=False, sort_keys=True))
        return 0
    if executable is None:
        raise FileNotFoundError(f"YOLO executable not found: {args.yolo_executable}")

    claim_path = _repo_path(variant["training_receipt"])
    if args.claim is not None and args.claim.resolve() != claim_path:
        raise ValueError(
            f"formal training receipt is fixed by the experiment contract: {claim_path}"
        )
    artifact_root = claim_path.parent
    log_path = args.log or artifact_root / f"{args.variant}.log"
    if log_path.exists():
        raise FileExistsError(log_path)
    token = acquire_one_time_claim(
        claim_path,
        f"formal_training:{args.variant}",
        {**bindings, "command": command},
        utc_now(),
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
                utc_now(),
                {
                    "return_code": return_code,
                    "elapsed_sec": time.monotonic() - start,
                    "log": str(log_path),
                },
            )
            return return_code
        if not trained_weight.is_file():
            raise FileNotFoundError(
                f"training returned success but best.pt is missing: {trained_weight}"
            )
        finalize_one_time_claim(
            claim_path,
            token,
            "completed",
            utc_now(),
            {
                "return_code": 0,
                "elapsed_sec": time.monotonic() - start,
                "log": str(log_path),
                "trained_weight": str(variant["trained_weight"]),
                "trained_weight_sha256": sha256_file(trained_weight),
                "environment": collect_environment(),
            },
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
        try:
            finalize_one_time_claim(
                claim_path,
                token,
                "failed",
                utc_now(),
                {
                    "elapsed_sec": time.monotonic() - start,
                    "log": str(log_path),
                    "error_type": type(error).__name__,
                    "error": str(error),
                },
            )
        except Exception:
            pass
        raise


if __name__ == "__main__":
    raise SystemExit(main())
