from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_benchmark.generalized_claim import (  # noqa: E402
    authorize_claim_resource_use,
    build_claim_payload,
    verify_claim_payload,
)
from strawberry_benchmark.run_identity import RESOURCE_LEDGER_KIND  # noqa: E402


class GeneralizedClaimTests(unittest.TestCase):
    def bindings(self):
        return {
            "matrix": {"path": "config/matrix.json", "sha256": "a" * 64, "size_bytes": 1},
            "materialization_manifest": {"path": "results/manifest.json", "sha256": "b" * 64, "size_bytes": 2},
            "model": {"path": "outputs/model.pt", "sha256": "c" * 64, "size_bytes": 3},
            "runtime_config": {"path": "launch/runtime.py", "sha256": "d" * 64, "size_bytes": 4},
            "git_commit": "e" * 40,
        }

    def claim(self):
        bindings = self.bindings()
        return build_claim_payload(
            matrix_id="generalized_harvest_formal_v1",
            matrix=bindings["matrix"],
            manifest=bindings["materialization_manifest"],
            model=bindings["model"],
            runtime_config=bindings["runtime_config"],
            git_commit=bindings["git_commit"],
            result_directory="results/generalized/formal_v1",
            resource_ledger={
                "path": "docs/p0/protected_resource_ledger_v1.json",
                "sha256": "1" * 64,
                "size_bytes": 100,
            },
            resource_id="formal-simulation-generalized-v1-30-scenes",
            consumption_id="formal-run-001",
        )

    def ledger(
        self,
        *,
        state="AVAILABLE",
        released=True,
        consumption_id=None,
    ):
        consumptions = []
        if consumption_id is not None:
            consumptions.append(
                {
                    "consumption_id": consumption_id,
                    "run_identity_status": "BOUND",
                    "evidence": "results/generalized/formal_v1/run_identity.json",
                }
            )
        return {
            "schema_version": 1,
            "kind": RESOURCE_LEDGER_KIND,
            "ledger_id": "p0-resource-ledger-v1",
            "updated_at_utc": "2026-09-09T00:00:00Z",
            "resources": [
                {
                    "resource_id": "formal-simulation-generalized-v1-30-scenes",
                    "resource_class": "formal_simulation",
                    "state": state,
                    "release_authorized": released,
                    "maximum_consumptions": 1,
                    "consumptions": consumptions,
                }
            ],
        }

    def test_claim_binds_code_model_config_and_single_attempt(self):
        claim = self.claim()

        self.assertTrue(claim["claim_consumed"])
        self.assertEqual(claim["behavioral_attempts_per_scenario"], 1)
        self.assertEqual(claim["bindings"], self.bindings())
        self.assertEqual(
            claim["resource_access"],
            {
                "ledger": {
                    "path": "docs/p0/protected_resource_ledger_v1.json",
                    "sha256": "1" * 64,
                    "size_bytes": 100,
                },
                "resource_id": "formal-simulation-generalized-v1-30-scenes",
                "consumption_id": "formal-run-001",
            },
        )
        self.assertEqual(claim["schema_version"], 2)

    def test_resume_accepts_only_identical_claim(self):
        verify_claim_payload(
            self.claim(),
            expected_matrix_id="generalized_harvest_formal_v1",
            expected_bindings=self.bindings(),
            expected_result_directory="results/generalized/formal_v1",
            expected_resource_access=self.claim()["resource_access"],
        )

    def test_resume_rejects_changed_model_or_result_directory(self):
        bindings = self.bindings()
        bindings["model"] = {**bindings["model"], "sha256": "f" * 64}
        with self.assertRaisesRegex(ValueError, "bindings changed"):
            verify_claim_payload(
                self.claim(),
                expected_matrix_id="generalized_harvest_formal_v1",
                expected_bindings=bindings,
                expected_result_directory="results/generalized/formal_v1",
                expected_resource_access=self.claim()["resource_access"],
            )
        with self.assertRaisesRegex(ValueError, "result directory changed"):
            verify_claim_payload(
                self.claim(),
                expected_matrix_id="generalized_harvest_formal_v1",
                expected_bindings=self.bindings(),
                expected_result_directory="results/generalized/formal_v2",
                expected_resource_access=self.claim()["resource_access"],
            )

    def test_protected_or_unreleased_formal_resource_is_rejected(self):
        for ledger in (
            self.ledger(state="PROTECTED", released=False),
            self.ledger(state="AVAILABLE", released=False),
        ):
            with self.subTest(state=ledger["resources"][0]["state"]):
                with self.assertRaisesRegex(PermissionError, "resource.*releas"):
                    authorize_claim_resource_use(
                        ledger,
                        resource_id="formal-simulation-generalized-v1-30-scenes",
                        consumption_id="formal-run-001",
                        resume=False,
                    )

    def test_consumed_resource_rejects_new_id_but_allows_same_id_resume(self):
        ledger = self.ledger(
            state="EXHAUSTED",
            released=True,
            consumption_id="formal-run-001",
        )
        with self.assertRaisesRegex(PermissionError, "budget exhausted"):
            authorize_claim_resource_use(
                ledger,
                resource_id="formal-simulation-generalized-v1-30-scenes",
                consumption_id="formal-run-002",
                resume=False,
            )
        self.assertEqual(
            "RESUME_EXISTING_ONLY",
            authorize_claim_resource_use(
                ledger,
                resource_id="formal-simulation-generalized-v1-30-scenes",
                consumption_id="formal-run-001",
                resume=True,
            ),
        )

    def test_existing_consumption_requires_explicit_resume(self):
        ledger = self.ledger(
            state="EXHAUSTED",
            released=True,
            consumption_id="formal-run-001",
        )
        with self.assertRaisesRegex(PermissionError, "requires --resume"):
            authorize_claim_resource_use(
                ledger,
                resource_id="formal-simulation-generalized-v1-30-scenes",
                consumption_id="formal-run-001",
                resume=False,
            )


if __name__ == "__main__":
    unittest.main()
