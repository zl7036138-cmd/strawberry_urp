#!/usr/bin/env python3
"""Validate the P6 archive and write the terminal project handoff."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import zipfile
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT / "artifacts" / "p6" / "p6_delivery_handoff_v1.json"
)

EVIDENCE = {
    "archive": "artifacts/p6/strawberry_urp_release_v1.zip",
    "archive_receipt": "artifacts/p6/strawberry_urp_release_v1.receipt.json",
    "decision": "docs/decisions/0035-accept-p6-delivery-archive.md",
    "delivery_guide": "docs/delivery.md",
    "p5_handoff": "artifacts/p5/p5_final_release_handoff_v1.json",
    "final_report": "docs/final-report.md",
    "final_video": "artifacts/p5/video/strawberry_urp_demo_v1.mp4",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _binding(relative_path: str) -> dict[str, Any]:
    path = REPOSITORY_ROOT / relative_path
    if not path.is_file():
        raise ValueError(f"required P6 evidence is missing: {relative_path}")
    return {
        "path": relative_path,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def finalize(output: Path) -> dict[str, Any]:
    archive_path = REPOSITORY_ROOT / EVIDENCE["archive"]
    receipt = json.loads(
        (REPOSITORY_ROOT / EVIDENCE["archive_receipt"]).read_text(
            encoding="utf-8"
        )
    )
    if receipt["status"] != "PASS":
        raise ValueError("P6 archive receipt did not pass")
    if receipt["verification"]["status"] != "VERIFIED_FINAL_RELEASE":
        raise ValueError("nested P5 verification differs")
    if _sha256(archive_path) != receipt["archive"]["sha256"]:
        raise ValueError("P6 archive hash differs from its receipt")
    if archive_path.stat().st_size != receipt["archive"]["size_bytes"]:
        raise ValueError("P6 archive size differs from its receipt")
    with zipfile.ZipFile(archive_path) as archive:
        bad_member = archive.testzip()
        names = archive.namelist()
        inventory = json.loads(archive.read("DELIVERY_INVENTORY.json"))
    if bad_member is not None:
        raise ValueError(f"P6 archive CRC failed: {bad_member}")
    if len(names) != receipt["archive"]["member_count"]:
        raise ValueError("P6 archive member count differs")
    if inventory["file_count"] != receipt["inventory"]["file_count"]:
        raise ValueError("P6 embedded inventory count differs")
    if not receipt["scope"]["p6_packaging_complete"]:
        raise ValueError("P6 packaging was not marked complete")

    handoff = {
        "schema_version": 1,
        "kind": "p6_delivery_handoff",
        "status": "PROJECT_DELIVERY_COMPLETE_LIMITATIONS_PRESERVED",
        "recorded_date": "2026-07-24",
        "decision": "ADR-0035",
        "release_id": receipt["release_id"],
        "archive": receipt["archive"],
        "inventory": receipt["inventory"],
        "p5_verification": receipt["verification"],
        "bindings": {
            key: _binding(path) for key, path in EVIDENCE.items()
        },
        "scientific_status": {
            "t30_numeric_gate": "FAIL_ENGINEERING_WAIVER",
            "formal_p3": "FAIL_39_OF_135",
            "p4_qualification": "FAIL_0_OF_300_HEAVY_TARGET_POSES",
            "held_out_real_test": "SEALED_NOT_CONSUMED",
        },
        "delivery_status": {
            "source_and_tests": "INCLUDED",
            "published_checkpoints": "INCLUDED",
            "final_metrics_and_figures": "INCLUDED",
            "clean_reproduction_evidence": "INCLUDED",
            "final_report": "INCLUDED",
            "final_video": "INCLUDED",
            "archive_crc": "PASS",
        },
        "future_scope": [
            "user-requested proofreading or material corrections",
            "verified reproduction-defect fixes",
            "new research only after a separately approved phase",
        ],
        "safety": {
            "held_out_real_test_consumed": False,
            "formal_p3_rerun": False,
            "p4_intervention_repeated": False,
            "physical_robot_claim": False,
            "sim_to_real_claim": False,
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(handoff, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return handoff


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    result = finalize(arguments.output.resolve())
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
