#!/usr/bin/env python3
"""Verify local artifacts and emit a machine-readable result."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from dataset_tools import load_manifest, verify_file


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = REPOSITORY_ROOT / "data" / "manifests" / "zenodo_6126677.json"
DEFAULT_INPUT = REPOSITORY_ROOT / "data" / "raw" / "zenodo_6126677"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--json-output", type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    manifest = load_manifest(args.manifest)
    statuses = []
    for artifact in manifest["files"]:
        checksum = artifact["checksum"]
        statuses.append(
            verify_file(
                args.input_dir / artifact["name"],
                checksum["algorithm"],
                checksum["value"],
            )
        )
    result = {
        "dataset_id": manifest["dataset_id"],
        "valid": all(status["valid"] for status in statuses),
        "files": statuses,
    }
    serialised = json.dumps(result, indent=2) + "\n"
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(serialised, encoding="utf-8")
    print(serialised, end="")
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
