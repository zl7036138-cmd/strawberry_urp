#!/usr/bin/env python3
"""Create the deterministic Strawberry URP submission-v2 archive."""

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
SOURCE_ROOTS = ("ros2_ws/src", "scripts", "config", "requirements", "tools", "docs")
FIXED_ZIP_TIME = (2026, 7, 29, 0, 0, 0)
MODEL = "outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt"
MODEL_SHA = "e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70"
REPEAT_TRIALS = {
    "results/development/field_v3_perception_pick_v15c/trial_01.json":
        "44d98f669260d488352e2f2192a5bde021d8dc1baa14303844859763cf3b8a4c",
    "results/development/field_v3_perception_pick_v16/trial_01.json":
        "06099efb7b49b2cd7f1d35978d7f3f65ce5db6216fbbd9728ffdeef540543074",
    "results/development/field_v3_perception_pick_v17/trial_01.json":
        "11b492152b358ef1302e64609432e9f79b664a3c8ab4b3490759329ffb5b5b8d",
}
KEY_FILES = (
    "README.md",
    ".gitignore",
    ".cache/wheels/wheelhouse-manifest.json",
    MODEL,
    "artifacts/submission_v2/report/草莓采摘URP项目总结报告.docx",
    "artifacts/submission_v2/report/草莓采摘URP项目总结报告.pdf",
    "artifacts/submission_v2/report/草莓采摘URP项目总结报告.build-receipt.json",
    "artifacts/submission_v2/report/草莓采摘URP项目总结报告.pdf-build-receipt.json",
    "artifacts/submission_v2/figures/field_v3_overview.png",
    "artifacts/submission_v2/video/strawberry_urp_submission_v2.mp4",
    "artifacts/submission_v2/video/strawberry_urp_submission_v2.receipt.json",
    "artifacts/submission_v2/test/colcon_test_receipt.json",
    "results/submission/field_v3_demo_v13/trial_01.json",
    "results/submission/field_v3_demo_v13/field_v3_live_demo.receipt.json",
    *REPEAT_TRIALS.keys(),
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _relative(path: Path) -> str:
    return path.resolve().relative_to(ROOT.resolve()).as_posix()


def _json(relative: str) -> dict[str, Any]:
    value = json.loads((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{relative} must contain a JSON object")
    return value


def _include(path: Path) -> bool:
    if not path.is_file():
        return False
    parts = path.relative_to(ROOT).parts
    if "__pycache__" in parts or path.suffix in {".pyc", ".pyo"}:
        return False
    return True


def _collect() -> list[Path]:
    paths: set[Path] = set()
    for relative in SOURCE_ROOTS:
        root = ROOT / relative
        paths.update(path for path in root.rglob("*") if _include(path))
    paths.update(ROOT / relative for relative in KEY_FILES)
    p5 = ROOT / "artifacts" / "p5" / "release_v1"
    paths.update(path for path in p5.rglob("*") if _include(path))
    field = ROOT / "results" / "submission" / "field_v3_demo_v13"
    paths.update(
        path
        for path in field.rglob("*")
        if _include(path) and path.suffix.lower() != ".avi"
    )
    for folder in (
        "field_v3_perception_pick_v15c",
        "field_v3_perception_pick_v16",
        "field_v3_perception_pick_v17",
    ):
        root = ROOT / "results" / "development" / folder
        paths.update(path for path in root.rglob("*") if _include(path))
    missing = sorted(_relative(path) for path in paths if not path.is_file())
    if missing:
        raise ValueError(f"submission files are missing: {missing}")
    return sorted(paths, key=_relative)


def _validate() -> dict[str, Any]:
    if _sha256(ROOT / MODEL) != MODEL_SHA:
        raise ValueError("accepted model hash differs")
    for relative, expected in REPEAT_TRIALS.items():
        if _sha256(ROOT / relative) != expected:
            raise ValueError(f"repeat trial hash differs: {relative}")
        if not _json(relative).get("success"):
            raise ValueError(f"repeat trial is not successful: {relative}")
    trial = _json("results/submission/field_v3_demo_v13/trial_01.json")
    if not trial.get("success") or not trial.get("result_success"):
        raise ValueError("submission field-v3 trial is not successful")
    contacts = trial["diagnostics"]["contacts"]
    if not all(contacts[side]["processed_seen_true"] for side in ("left", "right")):
        raise ValueError("submission trial lacks bilateral contact")
    attached = [
        event["attached"]
        for event in trial["diagnostics"]["attachment_state"]["events"]
    ]
    if attached != [True, False]:
        raise ValueError("submission trial attachment sequence differs")
    video = _json(
        "artifacts/submission_v2/video/"
        "strawberry_urp_submission_v2.receipt.json"
    )
    if video.get("status") != "PASS":
        raise ValueError("submission video is not verified")
    tests = _json("artifacts/submission_v2/test/colcon_test_receipt.json")
    if tests.get("status") != "PASS" or tests["totals"] != {
        "tests": 396,
        "errors": 0,
        "failures": 0,
        "skipped": 0,
    }:
        raise ValueError("submission test receipt differs")
    metrics = _json("artifacts/p5/release_v1/final_metrics_v1.json")
    if metrics["formal_p3"]["positive_successes"] != 39:
        raise ValueError("formal P3 result differs")
    if metrics["formal_p3"]["positive_trials"] != 135:
        raise ValueError("formal P3 denominator differs")
    return {
        "field_v3_submission_success": True,
        "repeat_successes": 3,
        "test_count": 396,
        "formal_p3_successes": 39,
        "formal_p3_trials": 135,
        "formal_p3_passed": False,
    }


def package(archive: Path) -> dict[str, Any]:
    verified = _validate()
    files = _collect()
    inventory = [
        {
            "path": _relative(path),
            "size_bytes": path.stat().st_size,
            "sha256": _sha256(path),
        }
        for path in files
    ]
    manifest = {
        "schema_version": 1,
        "kind": "submission_v2_inventory",
        "release_id": "strawberry_urp_submission_v2",
        "file_count": len(inventory),
        "uncompressed_size_bytes": sum(row["size_bytes"] for row in inventory),
        "verification": verified,
        "files": inventory,
        "omitted_large_artifacts": [
            "raw field-v3 MJPG recording",
            "complete per-attempt failed runtime directories",
            "training dataset archives",
            "colcon build/install/log trees",
            "wheel payloads (manifest retained)",
        ],
        "scientific_scope": {
            "field_v3_video_formal_evidence": False,
            "held_out_real_test_consumed": False,
            "formal_p3_rerun": False,
            "p4_intervention_repeated": False,
            "physical_robot_claim": False,
            "fruit_damage_claim": False,
            "sim_to_real_claim": False,
        },
    }
    manifest_bytes = (
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    archive = archive.resolve()
    archive.parent.mkdir(parents=True, exist_ok=True)
    temporary = archive.with_suffix(f"{archive.suffix}.tmp")
    if temporary.exists():
        temporary.unlink()
    with zipfile.ZipFile(
        temporary,
        "w",
        compression=zipfile.ZIP_DEFLATED,
        compresslevel=9,
    ) as output:
        for path in files:
            info = zipfile.ZipInfo(_relative(path), FIXED_ZIP_TIME)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            output.writestr(info, path.read_bytes(), compresslevel=9)
        info = zipfile.ZipInfo("SUBMISSION_INVENTORY.json", FIXED_ZIP_TIME)
        info.compress_type = zipfile.ZIP_DEFLATED
        info.external_attr = 0o100644 << 16
        output.writestr(info, manifest_bytes, compresslevel=9)
    temporary.replace(archive)
    with zipfile.ZipFile(archive) as value:
        bad_member = value.testzip()
        names = value.namelist()
    if bad_member is not None:
        raise ValueError(f"ZIP CRC verification failed: {bad_member}")
    receipt = {
        "schema_version": 1,
        "kind": "submission_v2_archive_receipt",
        "status": "PASS",
        "release_id": "strawberry_urp_submission_v2",
        "archive": {
            "path": _relative(archive),
            "size_bytes": archive.stat().st_size,
            "sha256": _sha256(archive),
            "member_count": len(names),
            "crc_verified": True,
        },
        "inventory": {
            "embedded_path": "SUBMISSION_INVENTORY.json",
            "file_count": len(inventory),
            "sha256": hashlib.sha256(manifest_bytes).hexdigest(),
        },
        "verification": verified,
    }
    receipt_path = archive.with_suffix(".receipt.json")
    receipt_path.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    arguments = parser.parse_args()
    result = package(arguments.archive)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
