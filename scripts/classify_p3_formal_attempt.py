#!/usr/bin/env python3
"""Classify a preserved formal attempt for fail-closed resume behavior."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-dir", type=Path, required=True)
    options = parser.parse_args()
    result_path = options.attempt_dir / "result.json"
    if not options.attempt_dir.exists():
        print("MISSING")
        return 0
    if not result_path.exists():
        print("PRE_BEHAVIOR_INFRASTRUCTURE")
        return 0
    try:
        result = json.loads(result_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        print("BEHAVIOR_STATUS_UNKNOWN")
        return 2
    if result.get("behavior_started") is True:
        print("BEHAVIOR_RECORDED")
        return 0
    print("PRE_BEHAVIOR_INFRASTRUCTURE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
