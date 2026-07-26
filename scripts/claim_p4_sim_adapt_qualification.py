#!/usr/bin/env python3
"""Consume or verify the single P4 qualification claim."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_benchmark"))

from strawberry_benchmark.p4_qualification import load_qualification_contract  # noqa: E402


def fingerprint(path: Path) -> dict:
    return {
        "path": str(path.resolve().relative_to(ROOT.resolve())).replace("\\", "/"),
        "size_bytes": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--preflight", type=Path, required=True)
    parser.add_argument("--result-dir", type=Path, required=True)
    parser.add_argument("--resume", action="store_true")
    options = parser.parse_args()
    manifest = options.manifest.resolve(strict=True)
    preflight_path = options.preflight.resolve(strict=True)
    contract = load_qualification_contract(manifest, ROOT)
    preflight = json.loads(preflight_path.read_text(encoding="utf-8"))
    if preflight.get("status") != "READY_UNCLAIMED":
        raise ValueError("qualification preflight is not READY_UNCLAIMED")
    if preflight.get("held_out_test_receipt_exists") is not False:
        raise ValueError("preflight does not preserve the held-out-test seal")
    if preflight.get("manifest", {}).get("sha256") != fingerprint(manifest)["sha256"]:
        raise ValueError("preflight manifest binding mismatch")
    claim_path = (ROOT / contract["runtime"]["claim_relative_path"]).resolve()
    result_dir = options.result_dir.resolve()
    if options.resume:
        if not claim_path.exists():
            raise ValueError("resume requires the existing claim")
        claim = json.loads(claim_path.read_text(encoding="utf-8"))
        if claim.get("manifest", {}).get("sha256") != fingerprint(manifest)["sha256"]:
            raise ValueError("claim manifest binding mismatch")
        if Path(claim["result_dir"]).as_posix() != str(result_dir.relative_to(ROOT.resolve())).replace("\\", "/"):
            raise ValueError("resume result directory differs from claim")
        print(json.dumps({"status": "RESUME_VERIFIED", "claim": str(claim_path)}))
        return 0
    if claim_path.exists():
        raise ValueError("single qualification claim is already consumed")
    if result_dir.exists() and any(result_dir.iterdir()):
        raise ValueError("qualification result directory is not empty")
    result_dir.mkdir(parents=True, exist_ok=True)
    claim_path.parent.mkdir(parents=True, exist_ok=True)
    held_out = ROOT / "artifacts/perception/zenodo_6126677_held_out_test_receipt.json"
    if held_out.exists():
        raise ValueError("held-out real test receipt exists")
    claim = {
        "schema_version": 1,
        "kind": "p4_sim_adapt_qualification_single_use_claim",
        "status": "CONSUMED_RUNNING",
        "consumed_at_utc": datetime.now(timezone.utc).isoformat(),
        "qualification_id": contract["qualification_id"],
        "manifest": fingerprint(manifest),
        "preflight": fingerprint(preflight_path),
        "model": fingerprint(Path(contract["_resolved_model_path"])),
        "result_dir": str(result_dir.relative_to(ROOT.resolve())).replace("\\", "/"),
        "held_out_real_test_consumed": False,
        "robot_motion_authorized": False,
        "training_authorized": False,
        "resume_policy": "same claim; preserve complete windows; execute missing scenarios only",
    }
    with claim_path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(claim, stream, indent=2, sort_keys=True)
        stream.write("\n")
    shutil.copyfile(claim_path, result_dir / "claim_snapshot.json")
    print(json.dumps({"status": "CLAIM_CONSUMED", "claim": str(claim_path)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
