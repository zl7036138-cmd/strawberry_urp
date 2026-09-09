#!/usr/bin/env python3
"""Read-only P0 recomputation of exhausted historical evidence.

This tool intentionally has no option for the protected generalized Formal-30
matrix or the protected 115-image real test. It never rewrites an old result.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Mapping


REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(
    0, str(REPOSITORY / "ros2_ws/src/strawberry_benchmark")
)
sys.path.insert(
    0, str(REPOSITORY / "ros2_ws/src/strawberry_bringup")
)

from strawberry_benchmark.formal_matrix import (  # noqa: E402
    load_formal_matrix_contract,
)
from strawberry_benchmark.formal_summary import (  # noqa: E402
    evaluate_formal_records,
)
from strawberry_bringup.development_gate import (  # noqa: E402
    score_runtime_run,
    summarize_runtime_gate,
)


ALLOWED_INPUT_PREFIXES = (
    "results/p3/formal_matrix_v1/",
    "config/p3_formal_matrix_v1.json",
    "artifacts/p3/p3_formal_matrix_preflight_v2/",
    ".codex_tmp/generalized_runtime_qualification_b01_640x480_camera_bound/",
    ".codex_tmp/generalized_feasibility_qualification_b01_640x480_camera_bound/",
    ".codex_tmp/generalized_runtime_dev_primary_clearance_45509_v1/",
    ".codex_tmp/generalized_runtime_dev_primary_clearance_45510_v1/",
    ".codex_tmp/generalized_feasibility_discovery_b04_640x480_primary_clearance/scenes/generalized_seed_045509.yaml",
    ".codex_tmp/generalized_feasibility_discovery_b04_640x480_primary_clearance/scenes/generalized_seed_045510.yaml",
)
PROTECTED_PREFIXES = (
    ".codex_tmp/generalized_formal_matrix_v1_final/",
    "data/processed/zenodo_6126677/",
)
QUALIFICATION_SUMMARY = (
    ".codex_tmp/generalized_runtime_qualification_b01_640x480_camera_bound/"
    "qualification_runtime_summary.json"
)
P3_SUMMARY = "results/p3/formal_matrix_v1/summary.json"


def _relative(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError as exc:
        raise ValueError(f"historical input escapes repository: {path}") from exc


def assert_allowed_input(path: Path, root: Path) -> str:
    relative = _relative(path, root)
    if any(
        relative == prefix.rstrip("/") or relative.startswith(prefix)
        for prefix in PROTECTED_PREFIXES
    ):
        raise PermissionError(f"protected resource access is forbidden: {relative}")
    if not any(
        relative == prefix.rstrip("/") or relative.startswith(prefix)
        for prefix in ALLOWED_INPUT_PREFIXES
    ):
        raise PermissionError(f"input is outside the P0 legacy allowlist: {relative}")
    return relative


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _fingerprint(path: Path, root: Path) -> dict[str, Any]:
    relative = assert_allowed_input(path, root)
    if not path.is_file():
        raise FileNotFoundError(relative)
    return {
        "path": relative,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _verify_reference(
    descriptor: Mapping[str, Any], root: Path
) -> dict[str, Any]:
    path = root / str(descriptor.get("path", ""))
    actual = _fingerprint(path, root)
    expected = {
        "path": str(descriptor.get("path", "")),
        "size_bytes": int(descriptor.get("size_bytes", -1)),
        "sha256": str(descriptor.get("sha256", "")),
    }
    if actual != expected:
        raise ValueError(f"historical fingerprint mismatch: {actual['path']}")
    return actual


def _check_reference(
    descriptor: Mapping[str, Any], root: Path
) -> dict[str, Any]:
    """Compare a historical reference without erasing a provenance mismatch."""

    path = root / str(descriptor.get("path", ""))
    actual = _fingerprint(path, root)
    expected = {
        "path": str(descriptor.get("path", "")),
        "size_bytes": int(descriptor.get("size_bytes", -1)),
        "sha256": str(descriptor.get("sha256", "")),
    }
    return {
        "status": "PASS" if actual == expected else "MISMATCH",
        "expected": expected,
        "actual": actual,
    }


def _load_json(path: Path, root: Path) -> dict[str, Any]:
    assert_allowed_input(path, root)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot load historical JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"historical JSON must contain an object: {path}")
    return value


def _load_yaml(path: Path, root: Path) -> dict[str, Any]:
    assert_allowed_input(path, root)
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - environment dependency
        raise RuntimeError("historical recomputation requires PyYAML") from exc
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"historical YAML must contain an object: {path}")
    return value


def _selected_score(score: Mapping[str, Any]) -> dict[str, Any]:
    return {
        key: score.get(key)
        for key in (
            "schema_version",
            "terminal_outcome",
            "terminal_evidence_status",
            "physical_identity_status",
            "identity_correct_physical_harvest_count",
            "accepted_target_count",
            "accepted_target_success_rate",
            "accepted_target_success_rate_status",
            "collision_evidence_status",
            "joint_limit_evidence_status",
            "home_stop_evidence_status",
            "attachment_terminal_evidence_status",
            "scene_terminal_evidence_status",
            "safety_evidence_status",
            "evidence_status",
            "evidence_errors",
            "evidence_unknowns",
            "run_safety_pass",
        )
    }


def _recompute_p3(root: Path) -> dict[str, Any]:
    summary_path = root / P3_SUMMARY
    summary = _load_json(summary_path, root)
    result_hashes_verified = 0
    condition_receipt_hashes_verified = 0
    condition_probe_hashes_verified = 0
    for row in summary.get("trials", []):
        result_ref = row.get("result")
        if not isinstance(result_ref, Mapping):
            raise ValueError("P3 normalized trial lacks a result fingerprint")
        _verify_reference(result_ref, root)
        result_hashes_verified += 1
        result = _load_json(root / str(result_ref["path"]), root)
        for field, counter_name in (
            ("condition_receipt", "receipt"),
            ("condition_probe", "probe"),
        ):
            descriptor = result.get(field)
            if not isinstance(descriptor, Mapping):
                raise ValueError(
                    f"P3 result lacks {field} fingerprint: {result_ref['path']}"
                )
            _verify_reference(descriptor, root)
            if counter_name == "receipt":
                condition_receipt_hashes_verified += 1
            else:
                condition_probe_hashes_verified += 1

    manifest_path = root / str(summary["manifest"]["path"])
    manifest_binding = _check_reference(summary["manifest"], root)
    preflight_binding = _check_reference(summary["preflight_receipt"], root)
    schedule_binding = _check_reference(summary["schedule"], root)
    contract = load_formal_matrix_contract(manifest_path)
    recomputed = evaluate_formal_records(contract, list(summary["trials"]))
    old_metrics = summary.get("metrics")
    if recomputed != old_metrics:
        raise ValueError("P3 metric recomputation differs from preserved summary")
    return {
        "resource_state": "EXHAUSTED",
        "protocol": "legacy P3 single-target simulator matrix",
        "preserved_summary": _fingerprint(summary_path, root),
        "verified_inputs": {
            "result_hashes": result_hashes_verified,
            "condition_receipt_hashes": condition_receipt_hashes_verified,
            "condition_probe_hashes": condition_probe_hashes_verified,
        },
        "historical_binding_checks": {
            "manifest": manifest_binding,
            "preflight_receipt": preflight_binding,
            "schedule": schedule_binding,
        },
        "legacy_metric_recomputation_exact": True,
        "legacy_result": {
            "positive_successes": recomputed["positive_successes"],
            "positive_trials": recomputed["positive_trials"],
            "positive_success_rate": recomputed["positive_success_rate"],
            "negative_safe_no_picks": recomputed["negative_safe_no_picks"],
            "negative_trials": recomputed["negative_trials"],
            "formal_p3_simulator_gate_passed": recomputed[
                "formal_p3_simulator_gate_passed"
            ],
        },
        "p0_evidence_status": "INDETERMINATE",
        "retained_historical_conclusion": "FAIL",
        "reasons": [
            "legacy records do not bind a complete P0 run identity",
            "the historical manifest fingerprint does not match the current file at the same path",
            "legacy single-target receipts do not contain the P0 per-operation physical identity chain",
            "legacy zero-event safety claims lack the P0 independent monitor coverage",
            "the failed 39/135 historical performance result is preserved and not rescored upward",
        ],
    }


def _recompute_qualification(root: Path) -> dict[str, Any]:
    summary_path = root / QUALIFICATION_SUMMARY
    summary = _load_json(summary_path, root)
    _verify_reference(summary["sweep_summary"], root)
    sweep = _load_json(root / str(summary["sweep_summary"]["path"]), root)
    sweep_rows = {
        str(row["scenario_id"]): row for row in sweep.get("scenarios", [])
    }
    scores = []
    run_rows = []
    referenced_hashes_verified = 0
    for run in summary.get("runs", []):
        scenario_id = str(run["scenario_id"])
        sweep_row = sweep_rows.get(scenario_id)
        if sweep_row is None:
            raise ValueError(f"qualification scenario absent from sweep: {scenario_id}")
        for descriptor in run.get("files", {}).values():
            _verify_reference(descriptor, root)
            referenced_hashes_verified += 1
        for descriptor in sweep_row.get("files", {}).values():
            _verify_reference(descriptor, root)
            referenced_hashes_verified += 1
        files = run["files"]
        old_score = _load_json(
            root / str(files["runtime_score.json"]["path"]), root
        )
        new_score = score_runtime_run(
            _load_json(root / str(files["runtime_probe.json"]["path"]), root),
            _load_yaml(root / str(sweep_row["files"]["scene"]["path"]), root),
            _load_json(root / str(files["cleanup_probe.json"]["path"]), root),
            _load_json(root / str(files["truth_isolation.json"]["path"]), root),
        )
        scores.append(new_score)
        run_rows.append(
            {
                "scenario_id": scenario_id,
                "seed": int(run["seed"]),
                "old_score": {
                    "fingerprint": dict(files["runtime_score.json"]),
                    "schema_version": old_score.get("schema_version"),
                    "run_safety_pass": old_score.get("run_safety_pass"),
                    "harvested_distinct_count": old_score.get(
                        "harvested_distinct_count"
                    ),
                    "accepted_target_count": old_score.get(
                        "accepted_target_count"
                    ),
                },
                "p0_recomputed_score": _selected_score(new_score),
            }
        )
    recomputed_gate = summarize_runtime_gate(scores)
    return {
        "resource_state": "EXHAUSTED",
        "preserved_summary": _fingerprint(summary_path, root),
        "referenced_file_hashes_verified": referenced_hashes_verified,
        "run_count": len(run_rows),
        "old_gate": summary.get("gate"),
        "p0_recomputed_gate": recomputed_gate,
        "old_run_safety_pass_count": sum(
            row["old_score"]["run_safety_pass"] is True for row in run_rows
        ),
        "p0_run_safety_pass_count": sum(
            row["p0_recomputed_score"]["run_safety_pass"] is True
            for row in run_rows
        ),
        "retained_conclusion": "FAIL",
        "runs": run_rows,
        "explanation": (
            "The old 0/5 multi-fruit failure remains. P0 additionally changes "
            "the five legacy run-safety PASS values to non-PASS because the "
            "v3 receipts lack independent collision, joint, home/stop, and "
            "scene-terminal evidence. The two identity-matched placements are "
            "diagnostic evidence, not terminally confirmed harvests."
        ),
    }


def _recompute_recovery_counterexamples(root: Path) -> dict[str, Any]:
    rows = []
    for seed in (45509, 45510):
        run_dir = root / (
            f".codex_tmp/generalized_runtime_dev_primary_clearance_{seed}_v1"
        )
        scene_path = root / (
            ".codex_tmp/generalized_feasibility_discovery_b04_640x480_primary_clearance/"
            f"scenes/generalized_seed_{seed:06d}.yaml"
        )
        paths = {
            "receipt": run_dir / "runtime_probe.json",
            "old_score": run_dir / "runtime_score.json",
            "cleanup": run_dir / "cleanup_probe.json",
            "truth_audit": run_dir / "truth_isolation.json",
            "scene": scene_path,
        }
        fingerprints = {
            name: _fingerprint(path, root) for name, path in paths.items()
        }
        old_score = _load_json(paths["old_score"], root)
        new_score = score_runtime_run(
            _load_json(paths["receipt"], root),
            _load_yaml(paths["scene"], root),
            _load_json(paths["cleanup"], root),
            _load_json(paths["truth_audit"], root),
        )
        rows.append(
            {
                "seed": seed,
                "inputs": fingerprints,
                "old_run_safety_pass": old_score.get("run_safety_pass"),
                "p0_run_safety_pass": new_score.get("run_safety_pass"),
                "p0_home_stop_evidence_status": new_score.get(
                    "home_stop_evidence_status"
                ),
                "p0_terminal_evidence_status": new_score.get(
                    "terminal_evidence_status"
                ),
                "p0_evidence_errors": new_score.get("evidence_errors"),
                "source_identity_status": "INDETERMINATE",
            }
        )
    return {
        "case": "RECOVERY_HOME_FAILED legacy scorer counterexample",
        "runs": rows,
        "old_pass_count": sum(row["old_run_safety_pass"] is True for row in rows),
        "p0_pass_count": sum(row["p0_run_safety_pass"] is True for row in rows),
        "conclusion": "COUNTEREXAMPLE_CLOSED",
    }


def build_recomputation(root: Path) -> dict[str, Any]:
    root = root.resolve()
    return {
        "schema_version": 1,
        "kind": "p0_historical_evidence_recomputation",
        "generated_at_utc": datetime.now(timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z"),
        "mode": "READ_ONLY_OFFLINE",
        "protected_resources": {
            "formal_simulation_generalized_v1_30_scenes": "NOT_ACCESSED",
            "formal_real_image_test_v1_115_images": "NOT_ACCESSED",
        },
        "p3_formal_legacy": _recompute_p3(root),
        "generalized_qualification_b01": _recompute_qualification(root),
        "recovery_home_failed_counterexamples": (
            _recompute_recovery_counterexamples(root)
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, default=REPOSITORY)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args(argv)
    output = options.output.resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite {output}")
    payload = build_recomputation(options.repository_root)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({"status": "WROTE", "output": str(output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
