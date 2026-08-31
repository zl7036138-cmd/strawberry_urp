"""Atomically consume or verify one generalized formal-matrix claim."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
from typing import Mapping, Sequence

from .generalized_acceptance import (
    sha256_file,
    validate_materialization_manifest,
    validate_matrix,
)


def fingerprint(path: Path, repository_root: Path) -> dict[str, object]:
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(repository_root.resolve()).as_posix()
    except ValueError as exc:
        raise ValueError(f"bound file must stay inside repository: {path}") from exc
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return {
        "path": relative,
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def build_claim_payload(
    *,
    matrix_id: str,
    matrix: Mapping[str, object],
    manifest: Mapping[str, object],
    model: Mapping[str, object],
    runtime_config: Mapping[str, object],
    git_commit: str,
    result_directory: str,
) -> dict[str, object]:
    if len(str(git_commit)) not in {40, 64}:
        raise ValueError("formal claim requires a full Git commit ID")
    return {
        "schema_version": 1,
        "kind": "generalized_formal_matrix_single_use_claim",
        "matrix_id": str(matrix_id),
        "claim_consumed": True,
        "behavioral_attempts_per_scenario": 1,
        "bindings": {
            "matrix": dict(matrix),
            "materialization_manifest": dict(manifest),
            "model": dict(model),
            "runtime_config": dict(runtime_config),
            "git_commit": str(git_commit),
        },
        "result_directory": str(result_directory),
        "resume_policy": "same claim; preserve all evidence; no second behavior attempt",
    }


def verify_claim_payload(
    claim: Mapping[str, object],
    *,
    expected_matrix_id: str,
    expected_bindings: Mapping[str, object],
    expected_result_directory: str,
) -> None:
    if int(claim.get("schema_version", 0)) != 1 or claim.get("kind") != "generalized_formal_matrix_single_use_claim":
        raise ValueError("unsupported generalized formal claim")
    if claim.get("matrix_id") != expected_matrix_id or claim.get("claim_consumed") is not True:
        raise ValueError("formal claim matrix binding mismatch")
    if claim.get("bindings") != expected_bindings:
        raise ValueError("formal claim file bindings changed")
    if claim.get("result_directory") != expected_result_directory:
        raise ValueError("formal claim result directory changed")
    if int(claim.get("behavioral_attempts_per_scenario", 0)) != 1:
        raise ValueError("formal claim behavior-attempt policy changed")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, default=Path.cwd())
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--runtime-config", type=Path, required=True)
    parser.add_argument("--result-dir", type=Path, required=True)
    parser.add_argument("--claim", type=Path, required=True)
    parser.add_argument("--resume", action="store_true")
    options = parser.parse_args(argv)
    root = options.repository_root.resolve()
    matrix = json.loads(options.matrix.read_text(encoding="utf-8"))
    manifest = json.loads(options.manifest.read_text(encoding="utf-8"))
    validate_matrix(matrix)
    validate_materialization_manifest(matrix, manifest)
    status = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=all"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if status:
        raise RuntimeError("formal claim requires a clean Git worktree")
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    result_dir = options.result_dir.resolve()
    try:
        result_relative = result_dir.relative_to(root).as_posix()
    except ValueError as exc:
        raise ValueError("formal result directory must stay inside repository") from exc
    bindings = {
        "matrix": fingerprint(options.matrix, root),
        "materialization_manifest": fingerprint(options.manifest, root),
        "model": fingerprint(options.model, root),
        "runtime_config": fingerprint(options.runtime_config, root),
        "git_commit": commit,
    }
    claim_path = options.claim.resolve()
    try:
        claim_path.relative_to(root)
    except ValueError as exc:
        raise ValueError("formal claim must stay inside repository") from exc
    if options.resume:
        if not claim_path.is_file():
            raise FileNotFoundError("resume requires the existing formal claim")
        claim = json.loads(claim_path.read_text(encoding="utf-8"))
        verify_claim_payload(
            claim,
            expected_matrix_id=str(matrix["matrix_id"]),
            expected_bindings=bindings,
            expected_result_directory=result_relative,
        )
        print(json.dumps({"status": "RESUME_VERIFIED", "claim": str(claim_path)}))
        return 0
    if claim_path.exists():
        raise FileExistsError("single generalized formal claim is already consumed")
    if result_dir.exists() and any(result_dir.iterdir()):
        raise FileExistsError("formal result directory must be empty before claim")
    result_dir.mkdir(parents=True, exist_ok=True)
    claim_path.parent.mkdir(parents=True, exist_ok=True)
    claim = build_claim_payload(
        matrix_id=str(matrix["matrix_id"]),
        matrix=bindings["matrix"],
        manifest=bindings["materialization_manifest"],
        model=bindings["model"],
        runtime_config=bindings["runtime_config"],
        git_commit=commit,
        result_directory=result_relative,
    )
    with claim_path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(claim, stream, indent=2, sort_keys=True)
        stream.write("\n")
    snapshot = result_dir / "claim_snapshot.json"
    with snapshot.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(claim, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({"status": "CLAIM_CONSUMED", "claim": str(claim_path)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
