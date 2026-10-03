from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.candidate_qualification import (  # noqa: E402
    CandidateQualificationStatus,
    PlanOnlyCandidateQualifier,
)
from strawberry_manipulation.core import Pose  # noqa: E402
from strawberry_manipulation.grasp_candidates import generate_grasp_candidates  # noqa: E402
from strawberry_manipulation.whole_chain import (  # noqa: E402
    ChainEvaluation,
    ChainFailureCode,
)


class ScriptedEvaluator:
    def __init__(self, codes, *, raises=None):
        self.codes = list(codes)
        self.raises = raises
        self.calls = []

    def evaluate(self, candidate, time_budget_sec):
        self.calls.append((candidate.candidate_id, time_budget_sec))
        if self.raises is not None:
            raise self.raises
        code = self.codes[len(self.calls) - 1]
        return ChainEvaluation(
            code,
            0.1,
            0.2,
            ("PREGRASP",),
            f"{candidate.candidate_id} -> {code.value}",
        )


class SignatureProvider:
    def __init__(self, values):
        self.values = iter(values)

    def __call__(self):
        return next(self.values)


class CandidateQualificationTests(unittest.TestCase):
    def setUp(self):
        self.candidates = generate_grasp_candidates(Pose(0.4, 0.1, 0.55))

    def test_runtime_search_certifies_first_later_feasible_candidate(self):
        events = []
        evaluator = ScriptedEvaluator(
            (
                ChainFailureCode.ESCAPE_FAILED,
                ChainFailureCode.TRANSPORT_FAILED,
                ChainFailureCode.FEASIBLE,
            )
        )
        result = PlanOnlyCandidateQualifier(
            evaluator,
            SignatureProvider(("scene-a", "scene-a")),
            certificate_clock_ns=lambda: 123,
            event_sink=lambda kind, payload: events.append((kind, payload)),
        ).qualify(target_id=9, candidates=self.candidates)

        self.assertTrue(result.feasible)
        self.assertEqual(result.status, CandidateQualificationStatus.PLAN_ONLY_CERTIFIED)
        self.assertTrue(result.scene_isolated)
        self.assertEqual(result.certificate.target_id, 9)
        self.assertEqual(result.certificate.candidate.candidate_id, "G02")
        self.assertEqual(
            [row.candidate_id for row in result.evaluations], ["G00", "G01", "G02"]
        )
        self.assertEqual([call[0] for call in evaluator.calls], ["G00", "G01", "G02"])
        self.assertEqual(
            [kind for kind, _payload in events],
            [
                "CANDIDATE_QUALIFICATION_STARTED",
                "CANDIDATE_EVALUATION_RESULT",
                "CANDIDATE_EVALUATION_RESULT",
                "CANDIDATE_EVALUATION_RESULT",
                "CANDIDATE_QUALIFICATION_RESULT",
            ],
        )
        result_payload = events[-1][1]
        self.assertFalse(result_payload["execution_dispatched"])
        self.assertEqual(result_payload["certificate_fingerprint"], result.certificate.certificate_fingerprint)

    def test_scene_change_invalidates_otherwise_feasible_certificate(self):
        result = PlanOnlyCandidateQualifier(
            ScriptedEvaluator((ChainFailureCode.FEASIBLE,)),
            SignatureProvider(("scene-before", "scene-after")),
            certificate_clock_ns=lambda: 123,
        ).qualify(target_id=9, candidates=self.candidates)

        self.assertFalse(result.feasible)
        self.assertEqual(result.status, CandidateQualificationStatus.SCENE_CHANGED)
        self.assertIsNone(result.certificate)
        self.assertEqual(result.evaluations[-1].candidate_id, "G00")

    def test_all_candidates_rejected_retains_the_complete_runtime_trace(self):
        result = PlanOnlyCandidateQualifier(
            ScriptedEvaluator((ChainFailureCode.APPROACH_FAILED,) * 15),
            SignatureProvider(("scene-a", "scene-a")),
        ).qualify(target_id=9, candidates=self.candidates)

        self.assertFalse(result.feasible)
        self.assertEqual(
            result.status, CandidateQualificationStatus.NO_FEASIBLE_CANDIDATE
        )
        self.assertIsNone(result.certificate)
        self.assertEqual(len(result.evaluations), 15)

    def test_non_empty_payload_and_evaluator_error_fail_closed(self):
        payload_result = PlanOnlyCandidateQualifier(
            ScriptedEvaluator((ChainFailureCode.FEASIBLE,)),
            SignatureProvider(("unused",)),
        ).qualify(target_id=9, candidates=self.candidates, payload_state="HOLDING")
        self.assertEqual(
            payload_result.status, CandidateQualificationStatus.PAYLOAD_NOT_EMPTY
        )
        self.assertEqual(payload_result.evaluations, ())

        error_result = PlanOnlyCandidateQualifier(
            ScriptedEvaluator((), raises=RuntimeError("copy failed")),
            SignatureProvider(("scene-a", "scene-a")),
        ).qualify(target_id=9, candidates=self.candidates)
        self.assertEqual(error_result.status, CandidateQualificationStatus.EVALUATION_ERROR)
        self.assertIsNone(error_result.certificate)


if __name__ == "__main__":
    unittest.main()
