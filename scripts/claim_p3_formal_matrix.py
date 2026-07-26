#!/usr/bin/env python3
"""Atomically consume or verify the single formal P3 matrix claim."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_benchmark"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from strawberry_benchmark.formal_matrix import (  # noqa: E402
    load_formal_matrix_contract,
    verify_formal_matrix_contract,
)
from strawberry_benchmark.perception_control import sha256_file  # noqa: E402


def _fingerprint(path: Path) -> dict:
    return {
        "path": path.resolve().relative_to(ROOT.resolve()).as_posix(),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def _verify_preflight_entry(entry: dict, name: str) -> None:
    path = (ROOT / str(entry.get("path", ""))).resolve()
    if ROOT.resolve() not in path.parents or not path.is_file():
        raise SystemExit(f"preflight {name} path is missing or unsafe")
    if path.stat().st_size != int(entry.get("size_bytes", -1)):
        raise SystemExit(f"preflight {name} size differs")
    if sha256_file(path) != str(entry.get("sha256", "")):
        raise SystemExit(f"preflight {name} digest differs")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--preflight", type=Path, required=True)
    parser.add_argument("--result-dir", type=Path, required=True)
    parser.add_argument("--resume", action="store_true")
    options = parser.parse_args()
    manifest_path = options.manifest.resolve(strict=True)
    preflight_path = options.preflight.resolve(strict=True)
    result_dir = options.result_dir.resolve()
    if ROOT.resolve() not in result_dir.parents:
        raise SystemExit("formal result directory must remain inside the repository")
    contract = load_formal_matrix_contract(manifest_path)
    verified = verify_formal_matrix_contract(contract, ROOT)
    expected_preflight = (verified["preflight_dir"] / "preflight_receipt.json").resolve()
    if preflight_path != expected_preflight:
        raise SystemExit("preflight path differs from the current formal contract")
    preflight = json.loads(preflight_path.read_text(encoding="utf-8"))
    if preflight.get("matrix_id") != contract.matrix_id:
        raise SystemExit("preflight matrix ID mismatch")
    if preflight.get("status") != "READY_UNCLAIMED" or preflight.get("formal_run_unlocked") is not True:
        raise SystemExit("formal preflight is not ready and unclaimed")
    if preflight.get("held_out_test_receipt_exists") is not False:
        raise SystemExit("formal preflight did not preserve the held-out-test seal")
    for name, entry in preflight.get("bindings", {}).items():
        _verify_preflight_entry(entry, f"binding {name}")
    for index, entry in enumerate(preflight.get("provenance", [])):
        _verify_preflight_entry(entry, f"provenance {index}")
    schedule_path = ROOT / str(preflight["schedule"]["path"])
    if not schedule_path.is_file() or sha256_file(schedule_path) != preflight["schedule"]["sha256"]:
        raise SystemExit("formal schedule differs from the preflight")

    claim_path = verified["claim"]
    if claim_path.exists():
        if not options.resume:
            raise SystemExit("formal claim already exists; use --resume for the same claim")
        existing = json.loads(claim_path.read_text(encoding="utf-8"))
        if existing.get("matrix_id") != contract.matrix_id:
            raise SystemExit("existing formal claim belongs to another matrix")
        if existing.get("preflight", {}).get("sha256") != sha256_file(preflight_path):
            raise SystemExit("resume preflight differs from the consumed claim")
        expected_result = result_dir.relative_to(ROOT.resolve()).as_posix()
        if existing.get("result_dir") != expected_result:
            raise SystemExit("resume result directory differs from the consumed claim")
        print(json.dumps({"status": "RESUME_VERIFIED", "claim": str(claim_path)}))
        return 0

    if options.resume:
        raise SystemExit("cannot resume because no formal claim exists")
    if result_dir.exists() and any(result_dir.iterdir()):
        raise SystemExit("new formal result directory must be empty")
    claim = {
        "schema_version": 1,
        "kind": "p3_formal_matrix_single_use_claim",
        "matrix_id": contract.matrix_id,
        "status": "CONSUMED_RUNNING",
        "claim_consumed": True,
        "formal_matrix_consumed": True,
        "held_out_real_test_consumed": False,
        "engineering_waiver_active": True,
        "numeric_t30_gate_passed": False,
        "manifest": _fingerprint(manifest_path),
        "preflight": _fingerprint(preflight_path),
        "schedule": _fingerprint(schedule_path),
        "model": _fingerprint(verified["model"]),
        "result_dir": result_dir.relative_to(ROOT.resolve()).as_posix(),
        "resume_policy": "same claim; missing prebehavior attempts only",
    }
    claim_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with claim_path.open("x", encoding="utf-8") as stream:
            json.dump(claim, stream, indent=2, sort_keys=True)
            stream.write("\n")
    except FileExistsError as exc:
        raise SystemExit("formal claim was consumed concurrently") from exc
    result_dir.mkdir(parents=True, exist_ok=True)
    (result_dir / "claim_snapshot.json").write_text(
        json.dumps(claim, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"status": "CLAIM_CONSUMED", "claim": str(claim_path)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
