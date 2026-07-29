#!/usr/bin/env python3
"""Verify every member of the deterministic submission-v2 archive."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import zipfile
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARCHIVE = (
    ROOT
    / "artifacts"
    / "submission_v2"
    / "package"
    / "strawberry_urp_submission_v2.zip"
)
REQUIRED = {
    "README.md",
    "docs/submission-report.md",
    "docs/submission-checklist.md",
    "artifacts/submission_v2/report/草莓采摘URP项目总结报告.docx",
    "artifacts/submission_v2/report/草莓采摘URP项目总结报告.pdf",
    "artifacts/submission_v2/video/strawberry_urp_submission_v2.mp4",
    "artifacts/submission_v2/video/strawberry_urp_submission_v2.receipt.json",
    "artifacts/submission_v2/test/colcon_test_receipt.json",
    "results/submission/field_v3_demo_v13/trial_01.json",
    "SUBMISSION_INVENTORY.json",
}


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def verify(archive: Path) -> dict[str, Any]:
    archive = archive.resolve()
    if not archive.is_file():
        raise ValueError(f"submission archive is missing: {archive}")
    with zipfile.ZipFile(archive) as value:
        bad_member = value.testzip()
        if bad_member is not None:
            raise ValueError(f"ZIP CRC verification failed: {bad_member}")
        names = set(value.namelist())
        missing = sorted(REQUIRED - names)
        if missing:
            raise ValueError(f"required archive members are missing: {missing}")
        manifest_bytes = value.read("SUBMISSION_INVENTORY.json")
        manifest = json.loads(manifest_bytes)
        rows = manifest["files"]
        if manifest["file_count"] != len(rows):
            raise ValueError("embedded inventory count differs")
        if len(names) != len(rows) + 1:
            raise ValueError("ZIP member count differs from inventory")
        for row in rows:
            path = row["path"]
            if path not in names:
                raise ValueError(f"inventory member is missing: {path}")
            payload = value.read(path)
            if len(payload) != row["size_bytes"]:
                raise ValueError(f"inventory size differs: {path}")
            if _sha256_bytes(payload) != row["sha256"]:
                raise ValueError(f"inventory hash differs: {path}")
        verification = manifest["verification"]
        scope = manifest["scientific_scope"]
        if verification != {
            "field_v3_submission_success": True,
            "repeat_successes": 3,
            "test_count": 396,
            "formal_p3_successes": 39,
            "formal_p3_trials": 135,
            "formal_p3_passed": False,
        }:
            raise ValueError("embedded verification summary differs")
        if any(
            scope[key]
            for key in (
                "field_v3_video_formal_evidence",
                "held_out_real_test_consumed",
                "formal_p3_rerun",
                "p4_intervention_repeated",
                "physical_robot_claim",
                "fruit_damage_claim",
                "sim_to_real_claim",
            )
        ):
            raise ValueError("scientific scope contains an unsafe claim")
    receipt = {
        "schema_version": 1,
        "kind": "submission_v2_archive_verification",
        "status": "VERIFIED_SUBMISSION_V2",
        "archive": {
            "path": archive.relative_to(ROOT).as_posix(),
            "size_bytes": archive.stat().st_size,
            "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
            "member_count": len(names),
            "crc_verified": True,
        },
        "inventory_file_count": manifest["file_count"],
        "verification": manifest["verification"],
        "scientific_scope": manifest["scientific_scope"],
    }
    output = archive.with_suffix(".verification.json")
    output.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    arguments = parser.parse_args()
    result = verify(arguments.archive)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
