from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.core import Pose  # noqa: E402
from strawberry_manipulation.grasp_candidate_search import (  # noqa: E402
    CandidateSearchStatus,
    FirstFeasibleCandidateSearch,
    WholeChainCandidateEvaluator,
)
from strawberry_manipulation.grasp_candidates import generate_grasp_candidates  # noqa: E402
from strawberry_manipulation.whole_chain import (  # noqa: E402
    ChainEvaluation,
    ChainFailureCode,
)


class RecordingEvaluator:
    def __init__(self, codes, clock=None, advance_sec=0.0):
        self.codes = list(codes)
        self.calls = []
        self.clock = clock
        self.advance_sec = advance_sec

    def evaluate(self, candidate, time_budget_sec):
        self.calls.append((candidate.candidate_id, candidate.geometry_fingerprint, time_budget_sec))
        if self.clock is not None:
            self.clock.advance(self.advance_sec)
        code = self.codes[len(self.calls) - 1]
        return ChainEvaluation(code, 0.1, 0.2, ("PREGRASP",), code.value)


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value

    def advance(self, amount):
        self.value += amount


class RequestRecordingWholeChain:
    def __init__(self):
        self.requests = []

    def evaluate(self, request):
        self.requests.append(request)
        return ChainEvaluation(ChainFailureCode.FEASIBLE, 0.1, 0.2)


class GraspCandidateSearchTests(unittest.TestCase):
    def setUp(self):
        self.candidates = generate_grasp_candidates(Pose(0.4, 0.1, 0.55))

    def test_nominal_feasible_evaluates_only_once(self):
        evaluator = RecordingEvaluator([ChainFailureCode.FEASIBLE])
        result = FirstFeasibleCandidateSearch(evaluator).search(self.candidates)
        self.assertEqual(result.status, CandidateSearchStatus.FOUND)
        self.assertEqual(result.selected_candidate.candidate_id, "G00")
        self.assertEqual([call[0] for call in evaluator.calls], ["G00"])

    def test_automatic_manual_adjustment_replacement_selects_first_later_feasible(self):
        evaluator = RecordingEvaluator([
            ChainFailureCode.ESCAPE_FAILED,
            ChainFailureCode.TRANSPORT_FAILED,
            ChainFailureCode.FEASIBLE,
        ])
        result = FirstFeasibleCandidateSearch(evaluator).search(self.candidates)
        self.assertEqual(result.status, CandidateSearchStatus.FOUND)
        self.assertEqual(result.selected_candidate.candidate_id, "G02")
        self.assertEqual([trace.candidate_id for trace in result.evaluations], ["G00", "G01", "G02"])
        self.assertEqual(result.evaluations[-1].geometry_fingerprint, result.selected_candidate.geometry_fingerprint)

    def test_all_candidates_rejected_preserves_full_trace(self):
        evaluator = RecordingEvaluator([ChainFailureCode.APPROACH_FAILED] * 15)
        result = FirstFeasibleCandidateSearch(evaluator).search(self.candidates)
        self.assertEqual(result.status, CandidateSearchStatus.NO_FEASIBLE_CANDIDATE)
        self.assertIsNone(result.selected_candidate)
        self.assertEqual(len(result.evaluations), 15)
        self.assertEqual(len(evaluator.calls), 15)

    def test_total_budget_caps_candidate_count_and_per_candidate_budget(self):
        clock = FakeClock()
        evaluator = RecordingEvaluator(
            [ChainFailureCode.ESCAPE_FAILED] * 15, clock, advance_sec=0.6
        )
        result = FirstFeasibleCandidateSearch(
            evaluator, per_candidate_budget_sec=1.0, total_budget_sec=1.0, clock=clock
        ).search(self.candidates)
        self.assertEqual(result.status, CandidateSearchStatus.SEARCH_TIME_BUDGET_EXCEEDED)
        self.assertEqual([call[0] for call in evaluator.calls], ["G00", "G01"])
        self.assertEqual(evaluator.calls[0][2], 1.0)
        self.assertAlmostEqual(evaluator.calls[1][2], 0.4)

    def test_invalid_candidate_set_does_not_call_evaluator(self):
        evaluator = RecordingEvaluator([ChainFailureCode.FEASIBLE])
        result = FirstFeasibleCandidateSearch(evaluator).search(())
        self.assertEqual(result.status, CandidateSearchStatus.CANDIDATE_SET_INVALID)
        self.assertEqual(evaluator.calls, [])

    def test_whole_chain_adapter_preserves_candidate_geometry(self):
        whole_chain = RequestRecordingWholeChain()
        candidate = self.candidates[4]
        adapter = WholeChainCandidateEvaluator(
            whole_chain, 9, self.candidates[0].grasp_pose, Pose(0.3, -0.4, 0.4)
        )
        result = adapter.evaluate(candidate, 1.25)
        self.assertTrue(result.feasible)
        request = whole_chain.requests[0]
        self.assertEqual(request.pregrasp_pose, candidate.pregrasp_pose)
        self.assertEqual(request.grasp_pose, candidate.grasp_pose)
        self.assertEqual(request.escape_pose, candidate.escape_pose)
        self.assertEqual(request.time_budget_sec, 1.25)


if __name__ == "__main__":
    unittest.main()
