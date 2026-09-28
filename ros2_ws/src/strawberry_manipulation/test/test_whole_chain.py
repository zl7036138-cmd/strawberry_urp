"""Pure contract tests for ADR 0086 whole-chain authorization."""

from __future__ import annotations

import unittest

from strawberry_manipulation.core import Pose
from strawberry_manipulation.whole_chain import (
    ChainFailureCode,
    StageAssessment,
    WholeChainEvaluator,
    WholeChainRequest,
)


class RecordingBackend:
    def __init__(self, fail_at: str | None = None) -> None:
        self.fail_at = fail_at
        self.calls: list[str] = []
        self.live_scene_mutations = 0
        self.controller_commands = 0
        self.gripper_commands = 0
        self.attach_commands = 0

    def snapshot(self, _request):
        self.calls.append("snapshot")
        return ("scene", 0)

    def scene_is_valid(self, state):
        return state[0] == "scene"

    def _stage(self, name, state, _request):
        self.calls.append(name)
        if self.fail_at == name:
            return StageAssessment(False, collision=name != "grasp_state", detail=name)
        return StageAssessment(True, planning_time_sec=.1, joint_travel_rad=.2,
                               terminal_state=("scene", state[1] + 1))

    def pregrasp(self, state, request):
        return self._stage("pregrasp", state, request)

    def approach(self, state, request):
        return self._stage("approach", state, request)

    def grasp_state(self, state, request):
        return self._stage("grasp_state", state, request)

    def virtual_attach(self, state, request):
        return self._stage("virtual_attach", state, request)

    def escape(self, state, request):
        return self._stage("escape", state, request)

    def transport(self, state, request):
        return self._stage("transport", state, request)

    def bin_approach(self, state, request):
        return self._stage("bin_approach", state, request)


def request() -> WholeChainRequest:
    pose = Pose(.3, .1, .5)
    return WholeChainRequest(1, pose, pose, pose, pose, pose)


class WholeChainEvaluatorTests(unittest.TestCase):
    def test_feasible_chain_propagates_one_terminal_state_per_stage(self):
        backend = RecordingBackend()
        outcome = WholeChainEvaluator(backend).evaluate(request())
        self.assertTrue(outcome.feasible)
        self.assertEqual(outcome.code, ChainFailureCode.FEASIBLE)
        self.assertEqual(
            backend.calls,
            ["snapshot", "pregrasp", "approach", "grasp_state", "virtual_attach",
             "escape", "transport", "bin_approach"],
        )
        self.assertEqual(outcome.stages[-1], "BIN_APPROACH")
        self.assertAlmostEqual(outcome.planning_time_sec, .7)
        self.assertAlmostEqual(outcome.joint_travel_rad, 1.4)

    def test_each_stage_has_a_stable_rejection_code(self):
        expected = {
            "pregrasp": ChainFailureCode.PREGRASP_FAILED,
            "approach": ChainFailureCode.APPROACH_FAILED,
            "grasp_state": ChainFailureCode.GRASP_STATE_INVALID,
            "escape": ChainFailureCode.ESCAPE_FAILED,
            "transport": ChainFailureCode.TRANSPORT_FAILED,
            "bin_approach": ChainFailureCode.BIN_APPROACH_FAILED,
        }
        for stage, code in expected.items():
            with self.subTest(stage=stage):
                outcome = WholeChainEvaluator(RecordingBackend(stage)).evaluate(request())
                self.assertEqual(outcome.code, code)

    def test_virtual_attach_is_hypothetical_and_emits_no_command(self):
        backend = RecordingBackend()
        outcome = WholeChainEvaluator(backend).evaluate(request())
        self.assertTrue(outcome.feasible)
        self.assertEqual(backend.live_scene_mutations, 0)
        self.assertEqual(backend.controller_commands, 0)
        self.assertEqual(backend.gripper_commands, 0)
        self.assertEqual(backend.attach_commands, 0)

    def test_time_budget_fails_closed(self):
        ticks = iter((0.0, 2.0))
        outcome = WholeChainEvaluator(RecordingBackend(), clock=lambda: next(ticks)).evaluate(
            WholeChainRequest(1, Pose(0, 0, .5), Pose(0, 0, .5), Pose(0, 0, .5),
                              Pose(0, 0, .6), Pose(.2, -.3, .4), time_budget_sec=1.0)
        )
        self.assertEqual(outcome.code, ChainFailureCode.TIME_BUDGET_EXCEEDED)


if __name__ == "__main__":
    unittest.main()
