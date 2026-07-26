#!/usr/bin/env python3
"""Validate two independent reviews and emit a no-mutation audit receipt."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Sequence


ALLOWED_DECISIONS = frozenset(
    {
        "CONFIRMED_MISSING_RIPE",
        "CONFIRMED_MISSING_UNRIPE",
        "MODEL_FALSE_POSITIVE",
        "DUPLICATE_EXISTING_LABEL",
        "AMBIGUOUS_EXCLUDE",
    }
)
ALLOWED_CONFIDENCE = frozenset({"HIGH", "MEDIUM", "LOW"})
CONFIRMED_MISSING = frozenset(
    {"CONFIRMED_MISSING_RIPE", "CONFIRMED_MISSING_UNRIPE"}
)
DISAGREEMENT_POLICIES = frozenset(
    {"require_adjudication", "conservative_exclude"}
)
REVIEW_FIELDS = frozenset(
    {"candidate_id", "reviewer", "decision", "confidence", "notes"}
)
ADJUDICATION_FIELDS = frozenset(
    {"candidate_id", "adjudicator", "final_decision", "notes"}
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _safe_child(root: Path, relative: str) -> Path:
    candidate = (root / relative).resolve()
    root = root.resolve()
    if candidate.parent != root:
        raise ValueError(f"packet path must be a direct child: {relative}")
    return candidate


def _load_packet(packet_dir: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    packet_dir = packet_dir.resolve()
    summary_path = packet_dir / "packet_summary.json"
    if not summary_path.is_file():
        raise ValueError(f"missing packet summary: {summary_path}")
    summary = _load_json(summary_path)
    if summary.get("kind") != "validation_label_audit_packet_summary":
        raise ValueError("unexpected packet kind")
    if summary.get("labels_modified") is not False:
        raise ValueError("packet must declare labels_modified=false")
    if summary.get("test_split_accessed") is not False:
        raise ValueError("packet must declare test_split_accessed=false")

    manifest_path = _safe_child(packet_dir, str(summary.get("candidate_manifest", "")))
    expected_hash = str(summary.get("candidate_manifest_sha256", ""))
    if not manifest_path.is_file() or _sha256(manifest_path) != expected_hash:
        raise ValueError("candidate manifest hash mismatch")
    manifest = _load_json(manifest_path)
    candidates = manifest.get("candidates")
    if not isinstance(candidates, list) or not candidates:
        raise ValueError("candidate manifest must contain a non-empty candidates list")
    if len(candidates) != summary.get("candidate_count"):
        raise ValueError("packet candidate count mismatch")

    ids = [item.get("candidate_id") for item in candidates if isinstance(item, dict)]
    if len(ids) != len(candidates) or any(not isinstance(item, str) for item in ids):
        raise ValueError("every candidate must have a string candidate_id")
    if len(ids) != len(set(ids)):
        raise ValueError("duplicate candidate_id in packet")
    return summary, candidates


def _read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    if not path.is_file():
        raise ValueError(f"missing CSV: {path}")
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        return fields, [{key: (value or "").strip() for key, value in row.items()} for row in reader]


def _load_review(path: Path, expected_ids: set[str]) -> tuple[str, dict[str, dict[str, str]]]:
    fields, rows = _read_csv(path)
    if not REVIEW_FIELDS.issubset(fields):
        raise ValueError(f"review CSV missing fields: {sorted(REVIEW_FIELDS - set(fields))}")
    if len(rows) != len(expected_ids):
        raise ValueError("review CSV must contain exactly one row per candidate")

    decisions: dict[str, dict[str, str]] = {}
    reviewers: set[str] = set()
    for row in rows:
        candidate_id = row["candidate_id"]
        if candidate_id not in expected_ids or candidate_id in decisions:
            raise ValueError(f"unknown or duplicate review candidate: {candidate_id}")
        if not row["reviewer"]:
            raise ValueError(f"missing reviewer identity for {candidate_id}")
        if row["decision"] not in ALLOWED_DECISIONS:
            raise ValueError(f"invalid decision for {candidate_id}: {row['decision']}")
        if row["confidence"] not in ALLOWED_CONFIDENCE:
            raise ValueError(f"invalid confidence for {candidate_id}: {row['confidence']}")
        reviewers.add(row["reviewer"])
        decisions[candidate_id] = row
    if len(reviewers) != 1:
        raise ValueError("one review file must contain exactly one reviewer identity")
    return next(iter(reviewers)), decisions


def _load_adjudication(
    path: Path,
    disagreement_ids: set[str],
    prohibited_identities: set[str],
) -> tuple[str, dict[str, dict[str, str]]]:
    fields, rows = _read_csv(path)
    if not ADJUDICATION_FIELDS.issubset(fields):
        raise ValueError(
            f"adjudication CSV missing fields: {sorted(ADJUDICATION_FIELDS - set(fields))}"
        )
    if {row["candidate_id"] for row in rows} != disagreement_ids:
        raise ValueError("adjudication must cover exactly the disagreement candidates")
    if len(rows) != len(disagreement_ids):
        raise ValueError("adjudication contains duplicate candidate rows")

    decisions: dict[str, dict[str, str]] = {}
    adjudicators: set[str] = set()
    for row in rows:
        candidate_id = row["candidate_id"]
        if row["final_decision"] not in ALLOWED_DECISIONS:
            raise ValueError(f"invalid adjudicated decision for {candidate_id}")
        if not row["adjudicator"]:
            raise ValueError(f"missing adjudicator identity for {candidate_id}")
        adjudicators.add(row["adjudicator"])
        decisions[candidate_id] = row
    if len(adjudicators) != 1:
        raise ValueError("adjudication must contain exactly one adjudicator identity")
    adjudicator = next(iter(adjudicators))
    if adjudicator in prohibited_identities:
        raise ValueError("adjudicator must be independent of both reviewers")
    return adjudicator, decisions


def finalize_audit(
    packet_dir: Path,
    reviewer_1_path: Path,
    reviewer_2_path: Path,
    output_path: Path,
    adjudication_path: Path | None = None,
    disagreement_policy: str = "require_adjudication",
    policy_record_path: Path | None = None,
) -> dict[str, Any]:
    if disagreement_policy not in DISAGREEMENT_POLICIES:
        raise ValueError(f"unsupported disagreement policy: {disagreement_policy}")
    if disagreement_policy == "conservative_exclude":
        if policy_record_path is None or not policy_record_path.resolve().is_file():
            raise ValueError(
                "conservative exclusion requires an existing frozen policy record"
            )
    elif policy_record_path is not None:
        raise ValueError("policy record is only valid with conservative exclusion")
    summary, candidates = _load_packet(packet_dir)
    expected_ids = {str(item["candidate_id"]) for item in candidates}
    reviewer_1, review_1 = _load_review(reviewer_1_path.resolve(), expected_ids)
    reviewer_2, review_2 = _load_review(reviewer_2_path.resolve(), expected_ids)
    if reviewer_1 == reviewer_2:
        raise ValueError("reviewer identities must be different")

    disagreements = {
        candidate_id
        for candidate_id in expected_ids
        if review_1[candidate_id]["decision"] != review_2[candidate_id]["decision"]
    }
    adjudicator: str | None = None
    adjudication: dict[str, dict[str, str]] = {}
    if disagreements:
        if disagreement_policy == "require_adjudication":
            if adjudication_path is None:
                raise ValueError(
                    "adjudication required for disagreements: "
                    + ",".join(sorted(disagreements))
                )
            adjudicator, adjudication = _load_adjudication(
                adjudication_path.resolve(), disagreements, {reviewer_1, reviewer_2}
            )
        elif adjudication_path is not None:
            raise ValueError(
                "adjudication cannot be combined with conservative exclusion"
            )
    elif adjudication_path is not None:
        raise ValueError("adjudication file supplied but reviewers have no disagreements")

    candidate_by_id = {str(item["candidate_id"]): item for item in candidates}
    resolutions: list[dict[str, Any]] = []
    manual_annotation_worklist: list[dict[str, Any]] = []
    counts = {decision: 0 for decision in sorted(ALLOWED_DECISIONS)}
    for candidate_id in sorted(expected_ids):
        decision_1 = review_1[candidate_id]["decision"]
        decision_2 = review_2[candidate_id]["decision"]
        if decision_1 == decision_2:
            final_decision = decision_1
            resolution = "AGREEMENT"
        elif disagreement_policy == "conservative_exclude":
            final_decision = "AMBIGUOUS_EXCLUDE"
            resolution = "CONSERVATIVE_EXCLUSION"
        else:
            final_decision = adjudication[candidate_id]["final_decision"]
            resolution = "ADJUDICATED"
        counts[final_decision] += 1
        resolutions.append(
            {
                "candidate_id": candidate_id,
                "reviewer_1_decision": decision_1,
                "reviewer_2_decision": decision_2,
                "final_decision": final_decision,
                "resolution": resolution,
            }
        )
        if final_decision in CONFIRMED_MISSING:
            source = candidate_by_id[candidate_id]
            manual_annotation_worklist.append(
                {
                    "candidate_id": candidate_id,
                    "image": source["image"],
                    "label": source["label"],
                    "class_name": final_decision.removeprefix("CONFIRMED_MISSING_").lower(),
                    "candidate_box_for_review_only": source["bbox_xyxy_normalized"],
                    "required_action": "human must draw or explicitly approve a final annotation box",
                }
            )

    output_path = output_path.resolve()
    if output_path.exists():
        raise FileExistsError(f"refusing to overwrite audit receipt: {output_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    receipt = {
        "schema_version": 2,
        "kind": "validation_label_audit_resolution",
        "status": (
            "READY_FOR_MANUAL_BOX_ANNOTATION"
            if manual_annotation_worklist
            else "AUDIT_COMPLETE_NO_LABEL_CHANGES"
        ),
        "packet": {
            "path": str((packet_dir / "packet_summary.json").resolve()),
            "sha256": _sha256((packet_dir / "packet_summary.json").resolve()),
            "candidate_manifest_sha256": summary["candidate_manifest_sha256"],
        },
        "reviewer_1": {
            "identity": reviewer_1,
            "path": str(reviewer_1_path.resolve()),
            "sha256": _sha256(reviewer_1_path.resolve()),
        },
        "reviewer_2": {
            "identity": reviewer_2,
            "path": str(reviewer_2_path.resolve()),
            "sha256": _sha256(reviewer_2_path.resolve()),
        },
        "adjudication": (
            None
            if adjudication_path is None
            else {
                "identity": adjudicator,
                "path": str(adjudication_path.resolve()),
                "sha256": _sha256(adjudication_path.resolve()),
            }
        ),
        "candidate_count": len(expected_ids),
        "disagreement_count": len(disagreements),
        "disagreement_policy": disagreement_policy,
        "policy_record": (
            None
            if policy_record_path is None
            else {
                "path": str(policy_record_path.resolve()),
                "sha256": _sha256(policy_record_path.resolve()),
            }
        ),
        "decision_counts": counts,
        "resolutions": resolutions,
        "manual_annotation_worklist": manual_annotation_worklist,
        "labels_modified": False,
        "test_split_accessed": False,
        "corrected_metric_claimed": False,
        "warning": "Candidate boxes are model-generated review aids, not accepted ground truth.",
    }
    output_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--packet-dir", type=Path, required=True)
    parser.add_argument("--reviewer-1", type=Path, required=True)
    parser.add_argument("--reviewer-2", type=Path, required=True)
    parser.add_argument("--adjudication", type=Path)
    parser.add_argument(
        "--disagreement-policy",
        choices=sorted(DISAGREEMENT_POLICIES),
        default="require_adjudication",
    )
    parser.add_argument("--policy-record", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        result = finalize_audit(
            arguments.packet_dir,
            arguments.reviewer_1,
            arguments.reviewer_2,
            arguments.output,
            arguments.adjudication,
            arguments.disagreement_policy,
            arguments.policy_record,
        )
    except Exception as exc:
        print(json.dumps({"error": str(exc), "status": "FAILED_CLOSED"}), file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
