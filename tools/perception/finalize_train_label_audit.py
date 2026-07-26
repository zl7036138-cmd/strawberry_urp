#!/usr/bin/env python3
"""Finalize the ADR-0024 training-label review without changing labels."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PACKET = Path(
    "artifacts/perception/label_audit/t30_train_label_screen_v1/review_packet"
)
DEFAULT_POLICY = Path("docs/decisions/0024-rebaseline-t30-and-audit-training-labels.md")
DEFAULT_OUTPUT = Path(
    "artifacts/perception/label_audit/t30_train_label_screen_v1/review_resolution_v1.json"
)
FIELDS = (
    "candidate_id",
    "reason",
    "image",
    "predicted_class_name",
    "confidence",
    "bbox_xyxy_normalized",
    "decision",
    "notes",
)
ALLOWED = {
    "CONFIRMED_ADD_RIPE",
    "CONFIRMED_ADD_UNRIPE",
    "CONFIRMED_RELABEL_RIPE",
    "CONFIRMED_RELABEL_UNRIPE",
    "MODEL_ERROR_KEEP_LABELS",
    "AMBIGUOUS_EXCLUDE",
}


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def _display(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPOSITORY_ROOT.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _binding(path: Path) -> dict[str, Any]:
    return {
        "path": _display(path),
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _expected_source(candidate: Mapping[str, Any]) -> tuple[str, ...]:
    return (
        str(candidate["candidate_id"]),
        str(candidate["reason"]),
        str(candidate["image"]),
        str(candidate["predicted_class_name"]),
        str(candidate["confidence"]),
        json.dumps(candidate["bbox_xyxy_normalized"], separators=(",", ":")),
    )


def _validate_decision(candidate: Mapping[str, Any], decision: str) -> None:
    if decision not in ALLOWED:
        raise ValueError(f"unsupported decision for {candidate['candidate_id']}: {decision}")
    predicted = str(candidate["predicted_class_name"]).upper()
    reason = str(candidate["reason"])
    if reason == "HIGH_CONFIDENCE_CROSS_CLASS":
        permitted = {
            f"CONFIRMED_RELABEL_{predicted}",
            "MODEL_ERROR_KEEP_LABELS",
            "AMBIGUOUS_EXCLUDE",
        }
    elif reason == "HIGH_CONFIDENCE_UNMATCHED_PREDICTION":
        permitted = {
            "CONFIRMED_ADD_RIPE",
            "CONFIRMED_ADD_UNRIPE",
            "MODEL_ERROR_KEEP_LABELS",
            "AMBIGUOUS_EXCLUDE",
        }
    else:
        raise ValueError(f"unexpected candidate reason: {reason}")
    if decision not in permitted:
        raise ValueError(
            f"decision is a semantic no-op or wrong operation for "
            f"{candidate['candidate_id']}: {decision}"
        )


def _load_review(
    path: Path, candidates: Mapping[str, Mapping[str, Any]]
) -> dict[str, dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != FIELDS:
            raise ValueError(f"review header mismatch: {path}")
        rows = list(reader)
    if len(rows) != len(candidates):
        raise ValueError(f"review row count mismatch: {path}")
    result: dict[str, dict[str, str]] = {}
    for row in rows:
        candidate_id = str(row.get("candidate_id", ""))
        if candidate_id in result:
            raise ValueError(f"duplicate review candidate: {candidate_id}")
        candidate = candidates.get(candidate_id)
        if candidate is None:
            raise ValueError(f"unknown review candidate: {candidate_id}")
        source = tuple(str(row[field]) for field in FIELDS[:6])
        if source != _expected_source(candidate):
            raise ValueError(f"candidate source columns changed: {candidate_id}")
        decision = str(row.get("decision", "")).strip()
        _validate_decision(candidate, decision)
        result[candidate_id] = {
            "decision": decision,
            "notes": str(row.get("notes", "")),
        }
    if set(result) != set(candidates):
        raise ValueError("review does not cover the exact candidate set")
    return result


def finalize(
    packet_dir: Path,
    reviewer_1_path: Path,
    reviewer_2_path: Path,
    policy_path: Path,
    output_path: Path,
) -> dict[str, Any]:
    packet_dir = _resolve(packet_dir).resolve()
    reviewer_1_path = _resolve(reviewer_1_path).resolve()
    reviewer_2_path = _resolve(reviewer_2_path).resolve()
    policy_path = _resolve(policy_path).resolve()
    output_path = _resolve(output_path).resolve()
    if output_path.exists():
        raise FileExistsError(f"refusing to overwrite review resolution: {output_path}")
    if reviewer_1_path == reviewer_2_path:
        raise ValueError("reviewers must use different files")
    if not policy_path.is_file():
        raise FileNotFoundError(policy_path)

    summary_path = packet_dir / "packet_summary.json"
    manifest_path = packet_dir / "candidate_manifest.json"
    summary = _load_object(summary_path)
    manifest = _load_object(manifest_path)
    if summary.get("kind") != "training_label_audit_packet_summary":
        raise ValueError("unexpected packet summary kind")
    if manifest.get("kind") != "training_label_audit_packet":
        raise ValueError("unexpected candidate manifest kind")
    manifest_binding = summary.get("candidate_manifest", {})
    if (
        manifest_binding.get("path") != "candidate_manifest.json"
        or manifest_binding.get("sha256") != _sha256(manifest_path)
    ):
        raise ValueError("packet summary candidate-manifest binding mismatch")
    scope = manifest.get("scope", {})
    if (
        scope.get("split") != "train"
        or scope.get("validation_split_accessed") is not False
        or scope.get("held_out_test_accessed") is not False
        or scope.get("training_started") is not False
        or scope.get("labels_modified") is not False
    ):
        raise ValueError("training-label packet scope changed")
    candidate_rows = manifest.get("candidates")
    if not isinstance(candidate_rows, list) or len(candidate_rows) != 34:
        raise ValueError("training-label resolution requires the frozen 34-item packet")
    candidates = {str(item["candidate_id"]): item for item in candidate_rows}
    if len(candidates) != len(candidate_rows):
        raise ValueError("candidate manifest contains duplicate IDs")

    reviewer_1 = _load_review(reviewer_1_path, candidates)
    reviewer_2 = _load_review(reviewer_2_path, candidates)
    resolutions = []
    disagreements = []
    agreements = []
    for candidate_id in sorted(candidates):
        first = reviewer_1[candidate_id]["decision"]
        second = reviewer_2[candidate_id]["decision"]
        agreed = first == second
        final_decision = first if agreed else "AMBIGUOUS_EXCLUDE"
        if agreed:
            agreements.append(candidate_id)
        else:
            disagreements.append(candidate_id)
        candidate = candidates[candidate_id]
        resolutions.append(
            {
                "candidate_id": candidate_id,
                "reason": candidate["reason"],
                "image": candidate["image"],
                "label": candidate["label"],
                "predicted_class_id": candidate["predicted_class_id"],
                "predicted_class_name": candidate["predicted_class_name"],
                "bbox_xyxy_normalized": candidate["bbox_xyxy_normalized"],
                "reviewer_1_decision": first,
                "reviewer_2_decision": second,
                "reviewer_2_notes": reviewer_2[candidate_id]["notes"],
                "agreed": agreed,
                "final_decision": final_decision,
            }
        )
    counts = Counter(item["final_decision"] for item in resolutions)
    change_ids = [
        item["candidate_id"]
        for item in resolutions
        if item["final_decision"].startswith("CONFIRMED_")
    ]
    result = {
        "schema_version": 1,
        "kind": "training_label_audit_resolution",
        "status": "REVIEW_FINALIZED_LABELS_UNCHANGED",
        "bindings": {
            "packet_summary": _binding(summary_path),
            "candidate_manifest": _binding(manifest_path),
            "reviewer_1": _binding(reviewer_1_path),
            "reviewer_2": _binding(reviewer_2_path),
            "policy_record": _binding(policy_path),
            "finalizer": _binding(Path(__file__)),
        },
        "policy": {
            "name": "conservative_exclude",
            "all_disagreements_become": "AMBIGUOUS_EXCLUDE",
            "case_by_case_override": False,
        },
        "candidate_count": len(resolutions),
        "agreement_count": len(agreements),
        "disagreement_count": len(disagreements),
        "agreement_ids": agreements,
        "disagreement_ids": disagreements,
        "decision_counts": dict(sorted(counts.items())),
        "confirmed_label_change_count": len(change_ids),
        "confirmed_label_change_ids": change_ids,
        "resolutions": resolutions,
        "labels_modified": False,
        "training_started": False,
        "validation_split_accessed": False,
        "held_out_test_accessed": False,
        "materialization_authorized": False,
        "training_authorized": False,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--packet-dir", type=Path, default=DEFAULT_PACKET)
    parser.add_argument("--reviewer-1", type=Path, required=True)
    parser.add_argument("--reviewer-2", type=Path, required=True)
    parser.add_argument("--policy-record", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    result = finalize(
        arguments.packet_dir,
        arguments.reviewer_1,
        arguments.reviewer_2,
        arguments.policy_record,
        arguments.output,
    )
    print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
