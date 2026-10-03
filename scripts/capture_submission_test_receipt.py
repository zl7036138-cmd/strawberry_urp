#!/usr/bin/env python3
"""Capture the current colcon test summary as a hash-bound submission receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    ROOT
    / "artifacts"
    / "submission_v2"
    / "test"
    / "colcon_test_receipt.json"
)
SUMMARY = re.compile(
    r"Summary:\s+(?P<tests>\d+)\s+tests,\s+"
    r"(?P<errors>\d+)\s+errors,\s+"
    r"(?P<failures>\d+)\s+failures,\s+"
    r"(?P<skipped>\d+)\s+skipped"
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def capture(result_base: Path, output: Path) -> dict[str, object]:
    result_base = result_base.expanduser().resolve()
    completed = subprocess.run(
        [
            "colcon",
            "test-result",
            "--test-result-base",
            str(result_base),
            "--verbose",
        ],
        capture_output=True,
        text=True,
    )
    combined = f"{completed.stdout}\n{completed.stderr}"
    match = SUMMARY.search(combined)
    if match is None:
        raise ValueError(f"colcon summary is missing:\n{combined}")
    totals = {key: int(value) for key, value in match.groupdict().items()}
    if completed.returncode != 0 or any(
        totals[key] for key in ("errors", "failures")
    ):
        raise ValueError(f"colcon tests are not clean:\n{combined}")
    xml_files = sorted(
        path
        for path in result_base.rglob("*.xml")
        if "test_results" in path.parts or path.name in {"pytest.xml", "xunit.xml"}
    )
    if not xml_files:
        raise ValueError(f"no test result XML files found below {result_base}")
    bindings = [
        {
            "path": path.relative_to(result_base).as_posix(),
            "size_bytes": path.stat().st_size,
            "sha256": _sha256(path),
        }
        for path in xml_files
    ]
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    dirty = bool(
        subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    )
    receipt = {
        "schema_version": 1,
        "kind": "submission_v2_colcon_test_receipt",
        "status": "PASS",
        "command": "colcon test-result --test-result-base <artifact>/build --verbose",
        "totals": totals,
        "result_base": str(result_base),
        "result_xml_count": len(bindings),
        "result_xml": bindings,
        "source_tree": {
            "git_head": head,
            "dirty_at_capture": dirty,
        },
        "safety": {
            "physical_robot_test": False,
            "held_out_real_test_consumed": False,
        },
    }
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test-result-base", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    result = capture(arguments.test_result_base, arguments.output)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
