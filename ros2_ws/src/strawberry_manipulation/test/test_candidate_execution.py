import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.candidate_execution import (  # noqa: E402
    RuntimeCandidateExecutionCoordinator,
)
from strawberry_manipulation.core import (  # noqa: E402
    FailureCode,
    MotionOutcome,
    PayloadState,
    PickAndPlaceExecutor,
    Pose,
)
from strawberry_manipulation.grasp_candidates import generate_grasp_candidates  # noqa: E402
from strawberry_manipulation.whole_chain import (  # noqa: E402
    ChainEvaluation,
    ChainFailureCode,
)


class _Backend:
    def __init__(self, *, restore=True, signatures=("scene-a",)):
        self.restore_ok = restore
        self.signatures = list(signatures)
        self.calls = []
        self.poses = []

    def authorization_scene_fingerprint(self):
        if len(self.signatures) > 1:
            return self.signatures.pop(0)
        return self.signatures[0]

    def prepare_pick(self, target_id, target_pose):
        self.calls.append("prepare")
        return True

    def restore_target_collision(self, target_id):
        self.calls.append("restore")
        return self.restore_ok

    def allow_target_contact(self, target_id):
        self.calls.append("allow_contact")
        return True

    def move_to(self, pose, stage):
        self.calls.append(stage)
        self.poses.append((stage, pose))
        return MotionOutcome(True, 0.01, 0.02)

    def open_gripper(self):
        self.calls.append("open")
        return True

    def close_gripper(self):
        self.calls.append("close")
        return True

    def gripper_centering_offset_m(self):
        self.calls.append("centering_offset")
        return None

    def gripper_fruit_contact_class(self):
        self.calls.append("contact_class")
        return None

    def attach(self, target_id):
        self.calls.append("attach")
        return True

    def detach(self, target_id):
        self.calls.append("detach")
        return True

    def fruit_in_bin(self, target_id, stable_for_sec):
        self.calls.append("verify")
        return True

    def return_via_recorded_place_route(self):
        self.calls.append("return_route")
        return MotionOutcome(True, 0.01, 0.02)

    def move_home(self):
        self.calls.append("home")
        return True


class _Evaluator:
    def __init__(self, results):
        self.results = dict(results)
        self.calls = []

    def evaluate(self, candidate, time_budget_sec):
        self.calls.append(candidate.candidate_id)
        code = self.results[candidate.candidate_id]
        stages = (
            (
                "PREGRASP",
                "APPROACH",
                "GRASP_STATE",
                "VIRTUAL_ATTACH",
                "ESCAPE",
                "TRANSPORT",
                "BIN_APPROACH",
            )
            if code is ChainFailureCode.FEASIBLE
            else ("PREGRASP", "APPROACH")
        )
        return ChainEvaluation(code, 0.02, 0.1, stages, code.value)


class RuntimeCandidateExecutionTests(unittest.TestCase):
    def setUp(self):
        self.target = Pose(0.4, 0.1, 0.5)
        self.bin = Pose(0.3, -0.4, 0.4)
        self.candidates = generate_grasp_candidates(self.target)[:3]

    def _coordinator(self, backend, evaluator, events):
        return RuntimeCandidateExecutionCoordinator(
            executor=PickAndPlaceExecutor(backend),
            backend=backend,
            evaluator=evaluator,
            event_sink=lambda event_type, payload: events.append((event_type, payload)),
        )

    def test_searches_to_later_candidate_then_executes_exact_certificate(self):
        backend = _Backend()
        evaluator = _Evaluator(
            {
                "G00": ChainFailureCode.ESCAPE_FAILED,
                "G01": ChainFailureCode.TRANSPORT_FAILED,
                "G02": ChainFailureCode.FEASIBLE,
            }
        )
        events = []

        result = self._coordinator(backend, evaluator, events).execute(
            target_id=7,
            target_pose=self.target,
            place_pose=self.bin,
            candidates=self.candidates,
        )

        self.assertTrue(result.success)
        self.assertEqual(evaluator.calls, ["G00", "G01", "G02"])
        self.assertIn("AUTHORIZED_G02", result.stages)
        moved = dict(backend.poses)
        self.assertEqual(moved["APPROACH"], self.candidates[2].pregrasp_pose)
        self.assertEqual(moved["GRASP_POSE"], self.candidates[2].grasp_pose)
        self.assertEqual(moved["RETREAT"], self.candidates[2].escape_pose)
        self.assertEqual(backend.calls[:3], ["prepare", "restore", "prepare"])
        self.assertEqual(backend.calls[-1], "restore")
        self.assertEqual(
            [event for event, _payload in events],
            [
                "ADAPTIVE_CANDIDATE_EXECUTION_STARTED",
                "CANDIDATE_QUALIFICATION_STARTED",
                "CANDIDATE_EVALUATION_RESULT",
                "CANDIDATE_EVALUATION_RESULT",
                "CANDIDATE_EVALUATION_RESULT",
                "CANDIDATE_QUALIFICATION_RESULT",
                "ADAPTIVE_CANDIDATE_QUALIFICATION_CLEANUP",
                "AUTHORIZED_CANDIDATE_EXECUTION_DISPATCHED",
                "AUTHORIZED_CANDIDATE_EXECUTION_RESULT",
            ],
        )
        self.assertEqual(events[1][1]["mode"], "PRE_EXECUTION_QUALIFICATION")
        dispatched = events[-2][1]
        self.assertEqual(dispatched["candidate_id"], "G02")
        self.assertTrue(dispatched["execution_dispatched"])

    def test_all_rejected_candidates_never_reach_gripper_or_motion(self):
        backend = _Backend()
        evaluator = _Evaluator(
            {
                "G00": ChainFailureCode.ESCAPE_FAILED,
                "G01": ChainFailureCode.TRANSPORT_FAILED,
                "G02": ChainFailureCode.APPROACH_FAILED,
            }
        )
        events = []

        result = self._coordinator(backend, evaluator, events).execute(
            target_id=7,
            target_pose=self.target,
            place_pose=self.bin,
            candidates=self.candidates,
        )

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertEqual(result.payload_state, PayloadState.EMPTY)
        self.assertEqual(evaluator.calls, ["G00", "G01", "G02"])
        self.assertEqual(backend.calls, ["prepare", "restore"])
        self.assertFalse(backend.poses)
        denied = events[-1]
        self.assertEqual(denied[0], "ADAPTIVE_CANDIDATE_EXECUTION_DENIED")
        self.assertEqual(denied[1]["reason"], "NO_FEASIBLE_CANDIDATE")
        self.assertFalse(denied[1]["execution_dispatched"])

    def test_cleanup_failure_withholds_certified_execution(self):
        backend = _Backend(restore=False)
        evaluator = _Evaluator(
            {
                "G00": ChainFailureCode.FEASIBLE,
                "G01": ChainFailureCode.FEASIBLE,
                "G02": ChainFailureCode.FEASIBLE,
            }
        )
        events = []

        result = self._coordinator(backend, evaluator, events).execute(
            target_id=7,
            target_pose=self.target,
            place_pose=self.bin,
            candidates=self.candidates,
        )

        self.assertFalse(result.success)
        self.assertEqual(result.payload_state, PayloadState.EMPTY)
        self.assertEqual(evaluator.calls, ["G00"])
        self.assertEqual(backend.calls, ["prepare", "restore"])
        self.assertFalse(backend.poses)
        self.assertEqual(events[-1][0], "ADAPTIVE_CANDIDATE_EXECUTION_DENIED")
        self.assertEqual(events[-1][1]["reason"], "TARGET_COLLISION_CLEANUP_FAILED")

    def test_scene_drift_after_qualification_denies_before_gripper_or_motion(self):
        backend = _Backend(signatures=("scene-a", "scene-a", "scene-b"))
        evaluator = _Evaluator(
            {
                "G00": ChainFailureCode.FEASIBLE,
                "G01": ChainFailureCode.FEASIBLE,
                "G02": ChainFailureCode.FEASIBLE,
            }
        )
        events = []

        result = self._coordinator(backend, evaluator, events).execute(
            target_id=7,
            target_pose=self.target,
            place_pose=self.bin,
            candidates=self.candidates,
        )

        self.assertFalse(result.success)
        self.assertEqual(result.stages, ("AUTHORIZED_GRASP_DENIED",))
        self.assertEqual(result.payload_state, PayloadState.EMPTY)
        self.assertEqual(evaluator.calls, ["G00"])
        self.assertEqual(backend.calls, ["prepare", "restore", "prepare", "restore"])
        self.assertFalse(backend.poses)
        self.assertEqual(events[-1][0], "AUTHORIZED_CANDIDATE_EXECUTION_RESULT")
        self.assertFalse(events[-1][1]["success"])


if __name__ == "__main__":
    unittest.main()
