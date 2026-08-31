from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_benchmark.generalized_claim import (  # noqa: E402
    build_claim_payload,
    verify_claim_payload,
)


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
        )

    def test_claim_binds_code_model_config_and_single_attempt(self):
        claim = self.claim()

        self.assertTrue(claim["claim_consumed"])
        self.assertEqual(claim["behavioral_attempts_per_scenario"], 1)
        self.assertEqual(claim["bindings"], self.bindings())

    def test_resume_accepts_only_identical_claim(self):
        verify_claim_payload(
            self.claim(),
            expected_matrix_id="generalized_harvest_formal_v1",
            expected_bindings=self.bindings(),
            expected_result_directory="results/generalized/formal_v1",
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
            )
        with self.assertRaisesRegex(ValueError, "result directory changed"):
            verify_claim_payload(
                self.claim(),
                expected_matrix_id="generalized_harvest_formal_v1",
                expected_bindings=self.bindings(),
                expected_result_directory="results/generalized/formal_v2",
            )


if __name__ == "__main__":
    unittest.main()
