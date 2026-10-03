"""Run dependency-light tests and write one immutable final receipt."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def _run(*args: str) -> str:
    # Preserve the leading status column used by ``git status --porcelain``.
    return subprocess.check_output(args, cwd=ROOT, text=True).rstrip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--replace",
        action="store_true",
        help="replace an existing generated receipt after an intentional rerun",
    )
    args = parser.parse_args()
    output = args.output if args.output.is_absolute() else ROOT / args.output
    if output.exists() and not args.replace:
        raise SystemExit(f"refusing to overwrite {output}")

    command = [sys.executable, str(ROOT / "scripts" / "run_pure_tests.py")]
    completed = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    sys.stdout.write(completed.stdout)
    sys.stderr.write(completed.stderr)

    summary = None
    for line in reversed(completed.stdout.splitlines()):
        try:
            candidate = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(candidate, dict) and "tests_run" in candidate:
            summary = candidate
            break
    if summary is None:
        raise SystemExit("pure test summary JSON was not found")

    dirty_paths = [
        line[3:]
        for line in _run("git", "status", "--porcelain=v1").splitlines()
        if line
    ]
    receipt = {
        "schema_version": 1,
        "receipt_type": "final_dependency_light_test",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "code_baseline_commit": _run("git", "rev-parse", "HEAD"),
        "branch": _run("git", "branch", "--show-current"),
        "command": ["python", "scripts/run_pure_tests.py"],
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
        },
        "working_tree_clean": not dirty_paths,
        "dirty_paths_at_test_time": dirty_paths,
        "result": summary,
        "process_returncode": completed.returncode,
        "stdout_sha256": hashlib.sha256(completed.stdout.encode()).hexdigest(),
        "stderr_sha256": hashlib.sha256(completed.stderr.encode()).hexdigest(),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    return 0 if completed.returncode == 0 and summary.get("successful") else 1


if __name__ == "__main__":
    raise SystemExit(main())
