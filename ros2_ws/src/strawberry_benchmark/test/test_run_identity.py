import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_benchmark.run_identity import (  # noqa: E402
    DIRTY_LEDGER_KIND,
    RESOURCE_LEDGER_KIND,
    RUN_IDENTITY_KIND,
    authorize_resource_use,
    fingerprint,
    validate_inherited_change_ledger,
    validate_resource_ledger,
    validate_run_identity,
)


def _git(root: Path, *arguments: str) -> str:
    return subprocess.run(
        ["git", *arguments],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


class RunIdentityTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        _git(self.root, "init")
        _git(self.root, "config", "user.name", "Identity Test")
        _git(self.root, "config", "user.email", "identity@example.invalid")
        for name in (
            "model.bin",
            "configuration.json",
            "environment.json",
            "scene.yaml",
            "runner.py",
            "scorer.py",
            "protocol.json",
            "result.json",
        ):
            (self.root / name).write_text(f"{name}\n", encoding="utf-8")
        _git(self.root, "add", ".")
        _git(self.root, "commit", "-m", "baseline")

    def tearDown(self):
        self.temporary.cleanup()

    def identity(self, *, tree_state="clean"):
        names = {
            "model": "model.bin",
            "configuration": "configuration.json",
            "environment": "environment.json",
            "scene_or_resource": "scene.yaml",
            "runner": "runner.py",
            "scorer": "scorer.py",
            "protocol": "protocol.json",
            "result": "result.json",
        }
        return {
            "schema_version": 1,
            "kind": RUN_IDENTITY_KIND,
            "run_id": "test-run-001",
            "captured_at_utc": "2026-09-08T00:00:00Z",
            "purpose": "unit-test provenance",
            "source": {
                "commit": _git(self.root, "rev-parse", "HEAD"),
                "branch": _git(self.root, "branch", "--show-current"),
                "tree_state": tree_state,
            },
            "bindings": {
                key: fingerprint(self.root / value, self.root)
                for key, value in names.items()
            },
        }

    def test_complete_clean_identity_verifies_current_files_and_source(self):
        report = validate_run_identity(
            self.identity(),
            repository_root=self.root,
            verify_files=True,
            verify_current_source=True,
        )
        self.assertEqual("PASS", report["status"])

    def test_missing_binding_and_changed_hash_fail_closed(self):
        identity = self.identity()
        del identity["bindings"]["scorer"]
        report = validate_run_identity(identity)
        self.assertEqual("FAIL", report["status"])
        self.assertIn("missing required bindings", report["errors"][0])

        identity = self.identity()
        (self.root / "model.bin").write_text("changed\n", encoding="utf-8")
        report = validate_run_identity(
            identity, repository_root=self.root, verify_files=True
        )
        self.assertEqual("FAIL", report["status"])
        self.assertIn("model", report["errors"][0])

    def test_clean_claim_rejects_an_undeclared_dirty_tree(self):
        identity = self.identity()
        (self.root / "runner.py").write_text("dirty\n", encoding="utf-8")
        report = validate_run_identity(
            identity,
            repository_root=self.root,
            verify_current_source=True,
        )
        self.assertEqual("FAIL", report["status"])
        self.assertIn("claims clean", report["errors"][0])

    def test_declared_dirty_tree_requires_exact_paths_and_hashes(self):
        base_commit = _git(self.root, "rev-parse", "HEAD")
        base_blob = _git(self.root, "rev-parse", "HEAD:runner.py")
        (self.root / "runner.py").write_text("inherited change\n", encoding="utf-8")
        digest = hashlib.sha256((self.root / "runner.py").read_bytes()).hexdigest()
        ledger = {
            "schema_version": 1,
            "kind": DIRTY_LEDGER_KIND,
            "base_commit": base_commit,
            "entries": [
                {
                    "path": "runner.py",
                    "status": "modified",
                    "base_blob": base_blob,
                    "working_sha256": digest,
                }
            ],
        }
        ledger_path = self.root / "inherited_changes.json"
        ledger_path.write_text(json.dumps(ledger), encoding="utf-8")
        _git(self.root, "add", "inherited_changes.json")
        _git(self.root, "commit", "-m", "declare inherited change")
        identity = self.identity(tree_state="declared_dirty")
        identity["source"]["inherited_change_ledger"] = fingerprint(
            ledger_path, self.root
        )

        report = validate_run_identity(
            identity,
            repository_root=self.root,
            verify_files=True,
            verify_current_source=True,
        )
        self.assertEqual("PASS", report["status"])
        self.assertEqual(
            "PASS",
            validate_inherited_change_ledger(
                ledger, repository_root=self.root
            )["status"],
        )

        (self.root / "configuration.json").write_text("surprise\n", encoding="utf-8")
        report = validate_run_identity(
            identity,
            repository_root=self.root,
            verify_current_source=True,
        )
        self.assertEqual("FAIL", report["status"])
        self.assertIn("differs from declaration", report["errors"][0])

    def test_unknown_historical_source_is_indeterminate_not_pass(self):
        identity = self.identity(tree_state="unknown")
        report = validate_run_identity(identity)
        self.assertEqual("INDETERMINATE", report["status"])
        self.assertTrue(report["unknowns"])


class ProtectedResourceLedgerTests(unittest.TestCase):
    def ledger(self, *, state="PROTECTED", consumptions=None, released=False):
        return {
            "schema_version": 1,
            "kind": RESOURCE_LEDGER_KIND,
            "ledger_id": "p0-resource-ledger-v1",
            "updated_at_utc": "2026-09-08T00:00:00Z",
            "resources": [
                {
                    "resource_id": "formal-generalized-30-v1",
                    "resource_class": "formal_simulation",
                    "state": state,
                    "release_authorized": released,
                    "maximum_consumptions": 1,
                    "consumptions": [] if consumptions is None else consumptions,
                }
            ],
        }

    def test_protected_resource_is_valid_but_use_is_rejected(self):
        ledger = self.ledger()
        self.assertEqual("PASS", validate_resource_ledger(ledger)["status"])
        report = authorize_resource_use(
            ledger,
            resource_id="formal-generalized-30-v1",
            consumption_id="attempt-1",
        )
        self.assertEqual("FAIL", report["status"])
        self.assertIn("not released", report["errors"][0])

    def test_protected_or_duplicate_consumption_fails_validation(self):
        consumption = {
            "consumption_id": "attempt-1",
            "run_identity_status": "BOUND",
            "evidence": "results/formal/identity.json",
        }
        protected = self.ledger(consumptions=[consumption])
        self.assertEqual("FAIL", validate_resource_ledger(protected)["status"])

        duplicate = self.ledger(
            state="EXHAUSTED",
            released=True,
            consumptions=[consumption, {**consumption, "consumption_id": "attempt-2"}],
        )
        report = validate_resource_ledger(duplicate)
        self.assertEqual("FAIL", report["status"])
        self.assertIn("duplicate resource consumption", report["errors"][0])

    def test_legacy_consumption_is_explicit_warning_not_fabricated_binding(self):
        ledger = self.ledger(
            state="EXHAUSTED",
            released=True,
            consumptions=[
                {
                    "consumption_id": "legacy-attempt",
                    "run_identity_status": "LEGACY_INCOMPLETE",
                    "evidence": "results/legacy/summary.json",
                }
            ],
        )
        report = validate_resource_ledger(ledger)
        self.assertEqual("PASS", report["status"])
        self.assertTrue(report["warnings"])


if __name__ == "__main__":
    unittest.main()
