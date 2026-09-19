"""Fail-closed run identity and protected-resource validation.

The module deliberately records provenance in a sidecar document instead of
embedding it in a result.  That keeps the result fingerprint non-circular and
lets historical evidence be classified without rewriting it.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
from typing import Mapping, Sequence


SHA256 = re.compile(r"[0-9a-f]{64}")
GIT_OBJECT = re.compile(r"[0-9a-f]{40,64}")
RUN_IDENTITY_KIND = "strawberry_urp_run_identity"
DIRTY_LEDGER_KIND = "strawberry_urp_inherited_change_ledger"
RESOURCE_LEDGER_KIND = "strawberry_urp_protected_resource_ledger"
REQUIRED_BINDINGS = (
    "model",
    "configuration",
    "environment",
    "scene_or_resource",
    "runner",
    "scorer",
    "protocol",
    "result",
)
TREE_STATES = {"clean", "declared_dirty", "unknown"}
RESOURCE_STATES = {"AVAILABLE", "PROTECTED", "CONSUMED", "EXHAUSTED", "UNKNOWN"}
IDENTITY_STATUSES = {"BOUND", "LEGACY_INCOMPLETE"}
RUN_TYPES = {"BEHAVIOR", "PERCEPTION", "EVALUATION_ONLY", "ENGINEERING_TEST"}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fingerprint(path: Path, repository_root: Path) -> dict[str, object]:
    """Return a portable fingerprint for one repository-contained file."""

    root = repository_root.resolve()
    resolved = path.resolve(strict=True)
    if not resolved.is_file():
        raise ValueError(f"bound artifact is not a regular file: {path}")
    try:
        relative = resolved.relative_to(root).as_posix()
    except ValueError as exc:
        raise ValueError(f"bound artifact must stay inside repository: {path}") from exc
    return {
        "path": relative,
        "sha256": sha256_file(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _nonempty(value: object, label: str) -> str:
    normalized = str(value).strip()
    if not normalized:
        raise ValueError(f"{label} must be non-empty")
    return normalized


def _artifact_path(value: object, label: str) -> PurePosixPath:
    normalized = _nonempty(value, f"{label}.path")
    path = PurePosixPath(normalized)
    if path.is_absolute() or ".." in path.parts or "\\" in normalized:
        raise ValueError(f"{label}.path must be a repository-relative POSIX path")
    return path


def _validate_artifact_binding(
    value: object,
    label: str,
    *,
    repository_root: Path | None,
    verify_file: bool,
    allow_not_applicable: bool = False,
) -> None:
    binding = _mapping(value, label)
    if binding.get("status") == "NOT_APPLICABLE":
        if not allow_not_applicable:
            raise ValueError(f"{label} cannot be NOT_APPLICABLE for this run")
        _nonempty(binding.get("reason"), f"{label}.reason")
        return
    relative = _artifact_path(binding.get("path"), label)
    digest = str(binding.get("sha256", ""))
    if not SHA256.fullmatch(digest):
        raise ValueError(f"{label}.sha256 must be a lowercase SHA-256")
    size = binding.get("size_bytes")
    if isinstance(size, bool) or not isinstance(size, int) or size < 0:
        raise ValueError(f"{label}.size_bytes must be a non-negative integer")
    if not verify_file:
        return
    if repository_root is None:
        raise ValueError("repository_root is required to verify bound files")
    root = repository_root.resolve()
    candidate = (root / Path(*relative.parts)).resolve()
    try:
        candidate.relative_to(root)
    except ValueError as exc:  # pragma: no cover - guarded by PurePosixPath checks
        raise ValueError(f"{label}.path escapes the repository") from exc
    if not candidate.is_file():
        raise ValueError(f"{label} bound file is missing: {relative.as_posix()}")
    if candidate.stat().st_size != size:
        raise ValueError(f"{label} bound file size changed: {relative.as_posix()}")
    if sha256_file(candidate) != digest:
        raise ValueError(f"{label} bound file hash changed: {relative.as_posix()}")


def _git(root: Path, arguments: Sequence[str], *, binary: bool = False):
    return subprocess.run(
        ["git", *arguments],
        cwd=root,
        check=True,
        capture_output=True,
        text=not binary,
    ).stdout


def _nul_paths(root: Path, arguments: Sequence[str]) -> set[str]:
    output = _git(root, arguments, binary=True)
    return {
        value.decode("utf-8", errors="surrogateescape")
        for value in output.split(b"\0")
        if value
    }


def current_dirty_paths(repository_root: Path) -> set[str]:
    """Return staged, unstaged, and untracked paths without parsing display text."""

    root = repository_root.resolve()
    return set().union(
        _nul_paths(root, ("diff", "--name-only", "--no-renames", "-z")),
        _nul_paths(root, ("diff", "--cached", "--name-only", "--no-renames", "-z")),
        _nul_paths(root, ("ls-files", "--others", "--exclude-standard", "-z")),
    )


def validate_declared_dirty_tree(
    manifest: Mapping[str, object], repository_root: Path
) -> None:
    """Verify that a dirty tree exactly matches its immutable declaration."""

    root = repository_root.resolve()
    if int(manifest.get("schema_version", 0)) != 1:
        raise ValueError("inherited change ledger must use schema version 1")
    if manifest.get("kind") != DIRTY_LEDGER_KIND:
        raise ValueError("unsupported inherited change ledger kind")
    base_commit = str(manifest.get("base_commit", "")).lower()
    if not GIT_OBJECT.fullmatch(base_commit):
        raise ValueError("inherited change ledger base_commit is invalid")
    _git(root, ("merge-base", "--is-ancestor", base_commit, "HEAD"))
    rows = manifest.get("entries")
    if not isinstance(rows, list) or not rows:
        raise ValueError("inherited change ledger entries must be a non-empty list")
    declared: dict[str, Mapping[str, object]] = {}
    for index, raw in enumerate(rows):
        row = _mapping(raw, f"entries[{index}]")
        relative = _artifact_path(row.get("path"), f"entries[{index}]").as_posix()
        if relative in declared:
            raise ValueError("inherited change ledger paths must be unique")
        if row.get("status") not in {"modified", "added", "deleted"}:
            raise ValueError("inherited change status must be modified, added, or deleted")
        declared[relative] = row
    actual = current_dirty_paths(root)
    if actual != set(declared):
        missing = sorted(set(declared) - actual)
        unexpected = sorted(actual - set(declared))
        raise ValueError(
            f"dirty tree differs from declaration; missing={missing}, unexpected={unexpected}"
        )
    for relative, row in declared.items():
        status = str(row["status"])
        candidate = root / Path(*PurePosixPath(relative).parts)
        if status == "deleted":
            if candidate.exists():
                raise ValueError(f"declared deleted path still exists: {relative}")
            continue
        if not candidate.is_file():
            raise ValueError(f"declared dirty path is not a file: {relative}")
        digest = str(row.get("working_sha256", ""))
        if not SHA256.fullmatch(digest) or sha256_file(candidate) != digest:
            raise ValueError(f"declared dirty file hash changed: {relative}")
        if status == "modified":
            base_blob = str(row.get("base_blob", "")).lower()
            if not GIT_OBJECT.fullmatch(base_blob):
                raise ValueError(f"declared base blob is invalid: {relative}")
            actual_blob = str(_git(root, ("rev-parse", f"HEAD:{relative}"))).strip()
            if actual_blob != base_blob:
                raise ValueError(f"declared base blob changed at HEAD: {relative}")


def validate_inherited_change_ledger(
    payload: Mapping[str, object], *, repository_root: Path
) -> dict[str, object]:
    """Return a structured result for an inherited dirty-tree declaration."""

    try:
        validate_declared_dirty_tree(payload, repository_root)
    except (OSError, subprocess.CalledProcessError, TypeError, ValueError) as exc:
        return _report([str(exc)], [])
    return _report([], [])


def _report(
    errors: Sequence[str], unknowns: Sequence[str], warnings: Sequence[str] = ()
) -> dict[str, object]:
    status = "FAIL" if errors else "INDETERMINATE" if unknowns else "PASS"
    return {
        "schema_version": 1,
        "status": status,
        "errors": list(errors),
        "unknowns": list(unknowns),
        "warnings": list(warnings),
    }


def validate_run_identity(
    payload: Mapping[str, object],
    *,
    repository_root: Path | None = None,
    verify_files: bool = False,
    verify_current_source: bool = False,
) -> dict[str, object]:
    """Classify a run identity as PASS, FAIL, or INDETERMINATE."""

    errors: list[str] = []
    unknowns: list[str] = []
    if verify_current_source and repository_root is None:
        errors.append("repository_root is required to verify current source")
    try:
        if int(payload.get("schema_version", 0)) != 1:
            raise ValueError("run identity must use schema version 1")
        if payload.get("kind") != RUN_IDENTITY_KIND:
            raise ValueError("unsupported run identity kind")
        _nonempty(payload.get("run_id"), "run_id")
        _nonempty(payload.get("captured_at_utc"), "captured_at_utc")
        _nonempty(payload.get("purpose"), "purpose")
        run_type = str(payload.get("run_type", ""))
        if run_type not in RUN_TYPES:
            raise ValueError("run_type is unsupported")
        source = _mapping(payload.get("source"), "source")
        commit = str(source.get("commit", "")).lower()
        if not GIT_OBJECT.fullmatch(commit):
            raise ValueError("source.commit must be a full Git object ID")
        branch = _nonempty(source.get("branch"), "source.branch")
        tree_state = str(source.get("tree_state", ""))
        if tree_state not in TREE_STATES:
            raise ValueError("source.tree_state is unsupported")
        if tree_state == "unknown":
            unknowns.append("source tree state is unknown")
        bindings = _mapping(payload.get("bindings"), "bindings")
        missing = sorted(set(REQUIRED_BINDINGS) - set(bindings))
        if missing:
            raise ValueError(f"run identity is missing required bindings: {missing}")
        for name in REQUIRED_BINDINGS:
            _validate_artifact_binding(
                bindings[name],
                f"bindings.{name}",
                repository_root=repository_root,
                verify_file=verify_files,
                allow_not_applicable=(
                    run_type == "ENGINEERING_TEST"
                    and name in {"model", "scene_or_resource"}
                ),
            )
        if tree_state == "declared_dirty":
            _validate_artifact_binding(
                source.get("inherited_change_ledger"),
                "source.inherited_change_ledger",
                repository_root=repository_root,
                verify_file=verify_files or verify_current_source,
            )
        if verify_current_source and repository_root is not None:
            root = repository_root.resolve()
            actual_commit = str(_git(root, ("rev-parse", "HEAD"))).strip()
            if actual_commit != commit:
                raise ValueError("source.commit does not match current HEAD")
            actual_branch = str(_git(root, ("branch", "--show-current"))).strip()
            if actual_branch != branch:
                raise ValueError("source.branch does not match current branch")
            if tree_state == "clean" and current_dirty_paths(root):
                raise ValueError("source claims clean but current tree is dirty")
            if tree_state == "declared_dirty":
                binding = _mapping(
                    source.get("inherited_change_ledger"),
                    "source.inherited_change_ledger",
                )
                relative = _artifact_path(
                    binding.get("path"), "source.inherited_change_ledger"
                )
                manifest_path = root / Path(*relative.parts)
                manifest = _mapping(
                    json.loads(manifest_path.read_text(encoding="utf-8")),
                    "inherited change ledger",
                )
                validate_declared_dirty_tree(manifest, root)
    except (OSError, subprocess.CalledProcessError, TypeError, ValueError) as exc:
        errors.append(str(exc))
    return _report(errors, unknowns)


def build_run_identity(
    *,
    repository_root: Path,
    run_id: str,
    run_type: str,
    purpose: str,
    tree_state: str,
    bindings: Mapping[str, Path | None],
    inherited_change_ledger: Path | None = None,
    captured_at_utc: str | None = None,
) -> dict[str, object]:
    """Capture and self-verify one new result identity.

    A missing model or scene is represented explicitly and is accepted only
    for ``ENGINEERING_TEST`` by :func:`validate_run_identity`.
    """

    root = repository_root.resolve()
    if run_type not in RUN_TYPES:
        raise ValueError("run_type is unsupported")
    if tree_state not in {"clean", "declared_dirty"}:
        raise ValueError("new run identity requires clean or declared_dirty source")
    missing = sorted(set(REQUIRED_BINDINGS) - set(bindings))
    if missing:
        raise ValueError(f"run identity is missing required bindings: {missing}")
    source: dict[str, object] = {
        "commit": str(_git(root, ("rev-parse", "HEAD"))).strip(),
        "branch": str(_git(root, ("branch", "--show-current"))).strip(),
        "tree_state": tree_state,
    }
    if tree_state == "declared_dirty":
        if inherited_change_ledger is None:
            raise ValueError("declared_dirty source requires an inherited change ledger")
        if not inherited_change_ledger.is_absolute():
            inherited_change_ledger = root / inherited_change_ledger
        source["inherited_change_ledger"] = fingerprint(
            inherited_change_ledger, root
        )
    captured_bindings: dict[str, object] = {}
    for name in REQUIRED_BINDINGS:
        path = bindings[name]
        if path is None:
            captured_bindings[name] = {
                "status": "NOT_APPLICABLE",
                "reason": (
                    f"{name} is not used by this engineering-only provenance test."
                ),
            }
        else:
            if not path.is_absolute():
                path = root / path
            captured_bindings[name] = fingerprint(path, root)
    payload = {
        "schema_version": 1,
        "kind": RUN_IDENTITY_KIND,
        "run_id": _nonempty(run_id, "run_id"),
        "captured_at_utc": captured_at_utc
        or datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "purpose": _nonempty(purpose, "purpose"),
        "run_type": run_type,
        "source": source,
        "bindings": captured_bindings,
    }
    report = validate_run_identity(
        payload,
        repository_root=root,
        verify_files=True,
        verify_current_source=True,
    )
    if report["status"] != "PASS":
        raise ValueError(f"captured run identity did not verify: {report}")
    return payload


def validate_resource_ledger(payload: Mapping[str, object]) -> dict[str, object]:
    """Validate resource states and reject duplicate or protected consumption."""

    errors: list[str] = []
    unknowns: list[str] = []
    warnings: list[str] = []
    try:
        if int(payload.get("schema_version", 0)) != 1:
            raise ValueError("resource ledger must use schema version 1")
        if payload.get("kind") != RESOURCE_LEDGER_KIND:
            raise ValueError("unsupported resource ledger kind")
        _nonempty(payload.get("ledger_id"), "ledger_id")
        _nonempty(payload.get("updated_at_utc"), "updated_at_utc")
        rows = payload.get("resources")
        if not isinstance(rows, list) or not rows:
            raise ValueError("resource ledger must contain resources")
        seen_resources: set[str] = set()
        seen_consumptions: set[str] = set()
        for index, raw in enumerate(rows):
            row = _mapping(raw, f"resources[{index}]")
            resource_id = _nonempty(row.get("resource_id"), "resource_id")
            if resource_id in seen_resources:
                raise ValueError(f"duplicate resource_id: {resource_id}")
            seen_resources.add(resource_id)
            resource_class = _nonempty(row.get("resource_class"), "resource_class")
            state = str(row.get("state", ""))
            if state not in RESOURCE_STATES:
                raise ValueError(f"unsupported state for {resource_id}")
            if state == "UNKNOWN":
                unknowns.append(f"resource state is unknown: {resource_id}")
            maximum = row.get("maximum_consumptions")
            if isinstance(maximum, bool) or not isinstance(maximum, int) or maximum <= 0:
                raise ValueError(
                    f"maximum_consumptions must be positive for {resource_id}"
                )
            release_authorized = row.get("release_authorized")
            if not isinstance(release_authorized, bool):
                raise ValueError(f"release_authorized must be boolean for {resource_id}")
            consumptions = row.get("consumptions")
            if not isinstance(consumptions, list):
                raise ValueError(f"consumptions must be a list for {resource_id}")
            if state in {"PROTECTED", "AVAILABLE"} and consumptions:
                raise ValueError(f"{state.lower()} resource has consumption: {resource_id}")
            if state in {"CONSUMED", "EXHAUSTED"} and not consumptions:
                raise ValueError(f"{state.lower()} resource lacks consumption: {resource_id}")
            if len(consumptions) > maximum:
                raise ValueError(f"duplicate resource consumption: {resource_id}")
            if resource_class.startswith("formal") and state == "AVAILABLE" and not release_authorized:
                raise ValueError(f"formal resource is available without release: {resource_id}")
            if state == "PROTECTED" and release_authorized:
                raise ValueError(f"protected resource cannot be released: {resource_id}")
            for consumption_index, raw_consumption in enumerate(consumptions):
                consumption = _mapping(
                    raw_consumption,
                    f"resources[{index}].consumptions[{consumption_index}]",
                )
                consumption_id = _nonempty(
                    consumption.get("consumption_id"), "consumption_id"
                )
                if consumption_id in seen_consumptions:
                    raise ValueError(f"duplicate consumption_id: {consumption_id}")
                seen_consumptions.add(consumption_id)
                identity_status = str(consumption.get("run_identity_status", ""))
                if identity_status not in IDENTITY_STATUSES:
                    raise ValueError(
                        f"unsupported run_identity_status for {consumption_id}"
                    )
                if identity_status == "LEGACY_INCOMPLETE":
                    warnings.append(
                        f"legacy consumption lacks complete run identity: {consumption_id}"
                    )
                _nonempty(consumption.get("evidence"), "consumption evidence")
    except (TypeError, ValueError) as exc:
        errors.append(str(exc))
    return _report(errors, unknowns, warnings)


def authorize_resource_use(
    ledger: Mapping[str, object], *, resource_id: str, consumption_id: str
) -> dict[str, object]:
    """Check a proposed use without mutating the resource ledger."""

    validation = validate_resource_ledger(ledger)
    if validation["status"] != "PASS":
        return validation
    resources = {
        str(row["resource_id"]): row for row in ledger.get("resources", [])
    }
    row = resources.get(resource_id)
    if row is None:
        return _report([], [f"resource is absent from ledger: {resource_id}"])
    state = str(row["state"])
    if state == "PROTECTED":
        return _report([f"protected resource is not released: {resource_id}"], [])
    if state == "UNKNOWN":  # pragma: no cover - caught by ledger validation
        return _report([], [f"resource state is unknown: {resource_id}"])
    existing = [
        str(item["consumption_id"]) for item in row.get("consumptions", [])
    ]
    if consumption_id in existing:
        report = _report([], [])
        report["decision"] = "RESUME_EXISTING_ONLY"
        return report
    if len(existing) >= int(row["maximum_consumptions"]):
        return _report([f"resource consumption budget exhausted: {resource_id}"], [])
    if str(row["resource_class"]).startswith("formal") and row.get(
        "release_authorized"
    ) is not True:
        return _report([f"formal resource lacks owner release: {resource_id}"], [])
    report = _report([], [])
    report["decision"] = "NEW_CONSUMPTION_ALLOWED"
    return report


def _load(path: Path) -> Mapping[str, object]:
    return _mapping(json.loads(path.read_text(encoding="utf-8")), str(path))


def _exit_code(report: Mapping[str, object]) -> int:
    if report.get("status") == "PASS":
        return 0
    if report.get("status") == "INDETERMINATE":
        return 2
    return 1


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    run_parser = subparsers.add_parser("validate-run")
    run_parser.add_argument("identity", type=Path)
    run_parser.add_argument("--repository-root", type=Path)
    run_parser.add_argument("--verify-files", action="store_true")
    run_parser.add_argument("--verify-current-source", action="store_true")
    capture_parser = subparsers.add_parser("capture-run")
    capture_parser.add_argument("--repository-root", type=Path, required=True)
    capture_parser.add_argument("--output", type=Path, required=True)
    capture_parser.add_argument("--run-id", required=True)
    capture_parser.add_argument("--run-type", choices=sorted(RUN_TYPES), required=True)
    capture_parser.add_argument("--purpose", required=True)
    capture_parser.add_argument(
        "--tree-state", choices=("clean", "declared_dirty"), required=True
    )
    capture_parser.add_argument("--inherited-change-ledger", type=Path)
    for name in REQUIRED_BINDINGS:
        capture_parser.add_argument(
            f"--{name.replace('_', '-')}",
            type=Path,
            required=name not in {"model", "scene_or_resource"},
        )
    ledger_parser = subparsers.add_parser("validate-ledger")
    ledger_parser.add_argument("ledger", type=Path)
    dirty_parser = subparsers.add_parser("validate-dirty")
    dirty_parser.add_argument("ledger", type=Path)
    dirty_parser.add_argument("--repository-root", type=Path, required=True)
    access_parser = subparsers.add_parser("check-resource")
    access_parser.add_argument("ledger", type=Path)
    access_parser.add_argument("--resource-id", required=True)
    access_parser.add_argument("--consumption-id", required=True)
    options = parser.parse_args(argv)
    if options.command == "capture-run":
        output = options.output.resolve()
        if output.exists():
            raise FileExistsError(f"refusing to overwrite {output}")
        bindings = {
            name: getattr(options, name) for name in REQUIRED_BINDINGS
        }
        payload = build_run_identity(
            repository_root=options.repository_root,
            run_id=options.run_id,
            run_type=options.run_type,
            purpose=options.purpose,
            tree_state=options.tree_state,
            bindings=bindings,
            inherited_change_ledger=options.inherited_change_ledger,
        )
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
        report = _report([], [])
        report["identity"] = str(output)
    elif options.command == "validate-run":
        report = validate_run_identity(
            _load(options.identity),
            repository_root=options.repository_root,
            verify_files=options.verify_files,
            verify_current_source=options.verify_current_source,
        )
    elif options.command == "validate-ledger":
        report = validate_resource_ledger(_load(options.ledger))
    elif options.command == "validate-dirty":
        report = validate_inherited_change_ledger(
            _load(options.ledger), repository_root=options.repository_root
        )
    else:
        report = authorize_resource_use(
            _load(options.ledger),
            resource_id=options.resource_id,
            consumption_id=options.consumption_id,
        )
    print(json.dumps(report, sort_keys=True))
    return _exit_code(report)


if __name__ == "__main__":
    raise SystemExit(main())
