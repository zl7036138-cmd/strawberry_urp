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
from strawberry_manipulation.grasp_candidates import (  # noqa: E402
    generate_grasp_candidates,
    recenter_grasp_candidate,
)
from strawberry_manipulation.whole_chain import (  # noqa: E402
    ChainEvaluation,
    ChainFailureCode,
)


class _Backend:
    def __init__(
        self,
        *,
        restore=True,
        signatures=("scene-a",),
        close_results=(),
        contact_classes=(),
        centering_offsets=(),
        attach_results=(),
    ):
        self.restore_ok = restore
        self.signatures = list(signatures)
        self.close_results = list(close_results)
        self.contact_classes = list(contact_classes)
        self.centering_offsets = list(centering_offsets)
        self.attach_results = list(attach_results)
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
        return self.close_results.pop(0) if self.close_results else True

    def gripper_centering_offset_m(self):
        self.calls.append("centering_offset")
        return self.centering_offsets.pop(0) if self.centering_offsets else None

    def gripper_fruit_contact_class(self):
        self.calls.append("contact_class")
        return self.contact_classes.pop(0) if self.contact_classes else None

    def attach(self, target_id):
        self.calls.append("attach")
        return self.attach_results.pop(0) if self.attach_results else True

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
            executor=PickAndPlaceExecutor(
                backend,
                authorized_contact_settle_timeout_sec=0.0,
            ),
            backend=backend,
            evaluator=evaluator,
            event_sink=lambda event_type, payload: events.append((event_type, payload)),
        )

    def test_single_contact_is_recentered_recertified_and_executed_once(self):
        backend = _Backend(
            close_results=(False, True),
            contact_classes=("RIGHT_SINGLE_FRUIT", None),
            centering_offsets=(0.004,),
        )
        evaluator = _Evaluator(
            {
                "G00": ChainFailureCode.ESCAPE_FAILED,
                "G01": ChainFailureCode.TRANSPORT_FAILED,
                "G02": ChainFailureCode.FEASIBLE,
                "G02-C01": ChainFailureCode.FEASIBLE,
            }
        )
        events = []

        result = self._coordinator(backend, evaluator, events).execute(
            target_id=7,
            target_pose=self.target,
            place_pose=self.bin,
            candidates=self.candidates,
        )

        self.assertTrue(result.success, result)
        self.assertEqual(
            evaluator.calls,
            ["G00", "G01", "G02", "G02-C01"],
        )
        self.assertIn("AUTHORIZED_G02", result.stages)
        self.assertIn("CONTACT_CENTERING_REAUTHORIZED", result.stages)
        self.assertIn("AUTHORIZED_G02-C01", result.stages)
        self.assertEqual(backend.calls.count("close"), 2)
        self.assertEqual(backend.calls.count("home"), 2)
        self.assertNotIn("CONTACT_CENTERING_PREP", backend.calls)

        corrected = recenter_grasp_candidate(
            self.candidates[2],
            local_y_offset_m=0.004,
        )
        approach_poses = [
            pose for stage, pose in backend.poses if stage == "APPROACH"
        ]
        grasp_poses = [
            pose for stage, pose in backend.poses if stage == "GRASP_POSE"
        ]
        self.assertEqual(
            approach_poses,
            [self.candidates[2].pregrasp_pose, corrected.pregrasp_pose],
        )
        self.assertEqual(
            grasp_poses,
            [self.candidates[2].grasp_pose, corrected.grasp_pose],
        )

        event_names = [event for event, _payload in events]
        first_result_index = event_names.index(
            "AUTHORIZED_CANDIDATE_EXECUTION_RESULT"
        )
        correction_start_index = event_names.index(
            "CONTACT_CENTERING_REAUTHORIZATION_STARTED"
        )
        correction_dispatch_index = event_names.index(
            "CONTACT_CENTERING_AUTHORIZED_EXECUTION_DISPATCHED"
        )
        correction_result_index = event_names.index(
            "CONTACT_CENTERING_AUTHORIZED_EXECUTION_RESULT"
        )
        self.assertLess(first_result_index, correction_start_index)
        self.assertLess(correction_start_index, correction_dispatch_index)
        self.assertLess(correction_dispatch_index, correction_result_index)
        initial_payload = events[first_result_index][1]
        self.assertFalse(initial_payload["success"])
        self.assertEqual(
            initial_payload["contact_centering_contact_class"],
            "RIGHT_SINGLE_FRUIT",
        )
        self.assertAlmostEqual(
            initial_payload["contact_centering_local_y_offset_m"],
            0.004,
        )
        correction_payload = events[correction_result_index][1]
        self.assertTrue(correction_payload["success"])
        self.assertFalse(
            correction_payload["further_reauthorization_permitted"]
        )

    def test_centering_reauthorization_requires_safe_home_recovery(self):
        backend = _Backend(
            close_results=(False,),
            contact_classes=("LEFT_SINGLE_FRUIT",),
            centering_offsets=(0.004,),
        )
        backend.move_home = lambda: backend.calls.append("home") or False
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
            candidates=self.candidates[:1],
        )

        self.assertFalse(result.success)
        self.assertEqual(evaluator.calls, ["G00"])
        self.assertNotIn(
            "CONTACT_CENTERING_REAUTHORIZATION_STARTED",
            [event for event, _payload in events],
        )
        self.assertEqual(backend.calls.count("close"), 1)

    def test_rejected_centering_candidate_never_executes_second_grasp(self):
        backend = _Backend(
            close_results=(False,),
            contact_classes=("RIGHT_SINGLE_FRUIT",),
            centering_offsets=(0.004,),
        )
        evaluator = _Evaluator(
            {
                "G00": ChainFailureCode.FEASIBLE,
                "G00-C01": ChainFailureCode.APPROACH_FAILED,
                "G00-C02": ChainFailureCode.TRANSPORT_FAILED,
                "G00-C03": ChainFailureCode.APPROACH_FAILED,
                "G00-C04": ChainFailureCode.TRANSPORT_FAILED,
            }
        )
        events = []

        result = self._coordinator(backend, evaluator, events).execute(
            target_id=7,
            target_pose=self.target,
            place_pose=self.bin,
            candidates=self.candidates[:1],
        )

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.GRASP_FAILED)
        self.assertEqual(
            evaluator.calls,
            ["G00", "G00-C01", "G00-C02", "G00-C03", "G00-C04"],
        )
        self.assertEqual(backend.calls.count("close"), 1)
        self.assertEqual(backend.calls.count("GRASP_POSE"), 1)
        self.assertIn(
            "CONTACT_CENTERING_REAUTHORIZATION_REJECTED",
            result.stages,
        )
        denied = [
            payload
            for event, payload in events
            if event == "CONTACT_CENTERING_REAUTHORIZATION_DENIED"
        ]
        self.assertEqual(len(denied), 1)
        self.assertEqual(denied[0]["reason"], "NO_FEASIBLE_CANDIDATE")
        self.assertFalse(denied[0]["execution_dispatched"])

    def test_centering_search_executes_first_feasible_partial_correction(self):
        backend = _Backend(
            close_results=(False, True),
            contact_classes=("LEFT_SINGLE_FRUIT", None),
            centering_offsets=(0.008,),
        )
        evaluator = _Evaluator(
            {
                "G00": ChainFailureCode.FEASIBLE,
                "G00-C01": ChainFailureCode.TRANSPORT_FAILED,
                "G00-C02": ChainFailureCode.FEASIBLE,
            }
        )
        events = []

        result = self._coordinator(backend, evaluator, events).execute(
            target_id=7,
            target_pose=self.target,
            place_pose=self.bin,
            candidates=self.candidates[:1],
        )

        self.assertTrue(result.success, result)
        self.assertEqual(
            evaluator.calls,
            ["G00", "G00-C01", "G00-C02"],
        )
        self.assertIn("AUTHORIZED_G00-C02", result.stages)
        correction_dispatch = next(
            payload
            for event, payload in events
            if event == "CONTACT_CENTERING_AUTHORIZED_EXECUTION_DISPATCHED"
        )
        self.assertEqual(correction_dispatch["candidate_id"], "G00-C02")
        expected = recenter_grasp_candidate(
            self.candidates[0],
            local_y_offset_m=-0.006,
            correction_index=2,
        )
        approach_poses = [
            pose for stage, pose in backend.poses if stage == "APPROACH"
        ]
        self.assertEqual(approach_poses[-1], expected.pregrasp_pose)

    def test_single_contact_prefers_measured_translation_over_untried_orientation(self):
        backend = _Backend(
            close_results=(False, True),
            contact_classes=("RIGHT_SINGLE_FRUIT", None),
            centering_offsets=(0.004,),
        )
        evaluator = _Evaluator(
            {
                "G00": ChainFailureCode.FEASIBLE,
                "G00-C01": ChainFailureCode.FEASIBLE,
            }
        )
        events = []

        result = self._coordinator(backend, evaluator, events).execute(
            target_id=7,
            target_pose=self.target,
            place_pose=self.bin,
            candidates=self.candidates,
        )

        self.assertTrue(result.success, result)
        self.assertEqual(evaluator.calls, ["G00", "G00-C01"])
        started = next(
            payload
            for event, payload in events
            if event == "CONTACT_CENTERING_REAUTHORIZATION_STARTED"
        )
        self.assertEqual(
            started["reauthorization_strategy"],
            "CENTERING_TRANSLATION",
        )
        self.assertEqual(
            started["corrected_candidate_ids"],
            ["G00-C01", "G00-C02", "G00-C03", "G00-C04"],
        )
        self.assertEqual(
            started["correction_scale_factors"],
            [1.0, 0.75, 0.5, 0.25],
        )
        self.assertIn("AUTHORIZED_G00-C01", result.stages)

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
