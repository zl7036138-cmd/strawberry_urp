"""Build and verify the permanent final-evidence manifest."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE_DIR = ROOT / "artifacts" / "final_evidence"
EXPECTED = {
    "ADR0086_positive_runtime_probe.json":
        "0900f0f55e5df97fda6e0cfeef7c2b476be644ff52c54d7c6fde53b89040830d",
    "ADR0086_blocked_transport_runtime_probe.json":
        "6d7f088e9942661fa68580fef1cacf30236429d557673cb7d7a9d224112fefc0",
    "ADR0087_plan_only_probe.json":
        "108993d9c6984cd54a9e3b71aa99f4ba46bd3fb6a27a6640653aa1f150829f49",
    "ADR0087_controlled_positive_probe.json":
        "50745b32644fe9d5154d5d887f3192b88166cb185285a799187509c0fc1a7431",
    "ADR0087_controlled_positive_truth_isolation.json":
        "2cd7d8e502766399524857b934ad18c5c0a7182d030a3538569791f5d874b0e2",
    "ADR0087_controlled_positive_cleanup.json":
        "9b1c8dc673cf3f5147ac0c1db28d1f3e06c3b00abedc1f276b185d09b9d9682f",
    "ADR0087_controlled_challenge_world.sdf":
        "d9b2efbdd0a95a67aacbd29d45efaa407630a5bf037acc34aeaac07cb074597b",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", type=Path, default=EVIDENCE_DIR / "manifest.json"
    )
    parser.add_argument(
        "--replace",
        action="store_true",
        help="replace an existing generated manifest after evidence changes",
    )
    args = parser.parse_args()
    output = args.output if args.output.is_absolute() else ROOT / args.output
    if output.exists() and not args.replace:
        raise SystemExit(f"refusing to overwrite {output}")

    files = []
    failures = []
    for path in sorted(EVIDENCE_DIR.iterdir()):
        if not path.is_file() or path == output:
            continue
        digest = sha256(path)
        expected = EXPECTED.get(path.name)
        verified = expected is None or digest == expected
        if not verified:
            failures.append(path.name)
        files.append(
            {
                "path": path.relative_to(ROOT).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": digest,
                "expected_sha256": expected,
                "expected_hash_verified": verified,
            }
        )

    manifest = {
        "schema_version": 1,
        "scope": "final_closure_key_runtime_evidence",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "code_baseline_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "formal_acceptance_claimed": False,
        "five_scene_multi_fruit_gate_passed": False,
        "formal_30_seed_matrix_opened": False,
        "files": files,
        "verification": {
            "critical_expected_hashes": len(EXPECTED),
            "critical_hash_failures": failures,
            "passed": not failures,
        },
    }
    output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
