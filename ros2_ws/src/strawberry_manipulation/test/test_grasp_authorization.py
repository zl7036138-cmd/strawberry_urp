from dataclasses import replace
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.core import Pose  # noqa: E402
from strawberry_manipulation.grasp_authorization import (  # noqa: E402
    ExecutionAuthorizationError,
    ExecutionIdentity,
    authorize_grasp_candidate,
    require_execution_identity,
)
from strawberry_manipulation.grasp_candidates import generate_grasp_candidates  # noqa: E402
from strawberry_manipulation.whole_chain import (  # noqa: E402
    ChainEvaluation,
    ChainFailureCode,
)


class GraspAuthorizationTests(unittest.TestCase):
    def setUp(self):
        self.candidates = generate_grasp_candidates(Pose(0.4, 0.1, 0.55))
        self.feasible = ChainEvaluation(ChainFailureCode.FEASIBLE, .2, 1.0)
        self.plan = authorize_grasp_candidate(
            self.candidates[2], self.feasible,
            target_id=11,
            scene_signature="scene-a", certificate_timestamp_ns=42,
        )

    def test_feasible_certificate_authorizes_exact_execution_identity(self):
        authorized = require_execution_identity(
            self.plan, self.plan.execution_identity
        )
        self.assertIs(authorized, self.plan)
        self.assertEqual(authorized.candidate.candidate_id, "G02")

    def test_non_feasible_certificate_cannot_authorize(self):
        with self.assertRaises(ExecutionAuthorizationError):
            authorize_grasp_candidate(
                self.candidates[2],
                ChainEvaluation(ChainFailureCode.ESCAPE_FAILED, .1, .2),
                target_id=11,
                scene_signature="scene-a", certificate_timestamp_ns=42,
            )

    def test_fingerprint_and_candidate_mismatch_are_denied(self):
        with self.assertRaisesRegex(ExecutionAuthorizationError, "fingerprint"):
            require_execution_identity(
                self.plan,
                ExecutionIdentity(11, "G02", "different", "scene-a"),
            )
        with self.assertRaisesRegex(ExecutionAuthorizationError, "candidate ID"):
            require_execution_identity(
                self.plan,
                ExecutionIdentity(
                    11, "G03", self.plan.candidate.geometry_fingerprint, "scene-a"
                ),
            )

    def test_missing_authorization_or_scene_mismatch_is_denied(self):
        with self.assertRaisesRegex(ExecutionAuthorizationError, "requires"):
            require_execution_identity(None, self.plan.execution_identity)
        with self.assertRaisesRegex(ExecutionAuthorizationError, "scene signature"):
            require_execution_identity(
                self.plan,
                ExecutionIdentity(
                    11, "G02", self.plan.candidate.geometry_fingerprint, "scene-b"
                ),
            )

    def test_target_mismatch_and_tampered_certificate_are_denied(self):
        with self.assertRaisesRegex(ExecutionAuthorizationError, "target ID"):
            require_execution_identity(
                self.plan,
                ExecutionIdentity(
                    12,
                    "G02",
                    self.plan.candidate.geometry_fingerprint,
                    "scene-a",
                ),
            )
        with self.assertRaisesRegex(ExecutionAuthorizationError, "requested pick"):
            require_execution_identity(
                self.plan,
                self.plan.execution_identity,
                target_id=12,
            )
        with self.assertRaisesRegex(ValueError, "certificate fingerprint"):
            replace(self.plan, certificate_fingerprint="tampered")


if __name__ == "__main__":
    unittest.main()
