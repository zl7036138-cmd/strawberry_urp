#!/usr/bin/env python3
"""Aggregate read-only RGB-D diagnostics for one fixed camera layout."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
SIM_SOURCE = ROOT / "ros2_ws" / "src" / "strawberry_sim"
if str(SIM_SOURCE) not in sys.path:
    sys.path.insert(0, str(SIM_SOURCE))

from strawberry_sim.observability import (  # noqa: E402
    aggregate_observability_receipts,
)


def _load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--layout-id", required=True)
    parser.add_argument("--base-camera-mast-xyz", required=True)
    parser.add_argument("--base-camera-xyz", required=True)
    parser.add_argument("--base-camera-rpy", required=True)
    parser.add_argument("--maximum-localization-error-m", type=float, default=0.03)
    options = parser.parse_args()
    if options.output.exists():
        raise FileExistsError(f"refusing to overwrite {options.output}")
    paths = [path.resolve() for path in options.receipt]
    if len(paths) != len(set(paths)):
        raise ValueError("observability receipt paths must be unique")
    receipts = []
    files = []
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(path)
        cleanup_path = path.parent / "cleanup.json"
        truth_path = path.parent / "truth_isolation.json"
        cleanup = _load(cleanup_path)
        truth = _load(truth_path)
        if cleanup.get("outcome") != "CLEAN":
            raise ValueError(f"diagnostic process cleanup was not clean: {path.parent}")
        if truth.get("overall_pass") is not True:
            raise ValueError(f"truth-isolation audit failed: {path.parent}")
        receipts.append(_load(path))
        files.append(
            {
                "receipt": str(path),
                "receipt_sha256": _sha256(path),
                "cleanup_sha256": _sha256(cleanup_path),
                "truth_isolation_sha256": _sha256(truth_path),
            }
        )
    aggregate = aggregate_observability_receipts(
        receipts,
        maximum_localization_error_m=options.maximum_localization_error_m,
    )
    git_commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    git_dirty = bool(
        subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    )
    payload = {
        "schema_version": 1,
        "kind": "generalized_base_camera_observability_summary",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "DEVELOPMENT_ONLY_OFFLINE_TRUTH_SCORED_CAMERA_LAYOUT",
        "runtime_truth_use": False,
        "commands_published": 0,
        "formal_acceptance": False,
        "layout": {
            "layout_id": options.layout_id,
            "base_camera_mast_xyz": options.base_camera_mast_xyz,
            "base_camera_xyz": options.base_camera_xyz,
            "base_camera_rpy": options.base_camera_rpy,
        },
        "git_commit": git_commit,
        "git_worktree_dirty": git_dirty,
        "aggregate": aggregate,
        "receipts": files,
    }
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"output": str(options.output), **aggregate}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
