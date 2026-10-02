import math
import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.core import (  # noqa: E402
    DEFAULT_TOOL_CENTER_OFFSET_M,
    FailureCode,
    MotionOutcome,
    PayloadState,
    PickAndPlaceExecutor,
    Pose,
    RecoveryDisposition,
    alternate_approach,
    bounded_pregrasp_candidates_for_fruit_center,
    offset_along_local_z,
    offset_along_local_y,
    pregrasp_pose_for_fruit_center,
    rotate_about_base_z,
    wait_for_bilateral_contact,
)
from strawberry_manipulation.grasp_authorization import (  # noqa: E402
    ExecutionIdentity,
    authorize_grasp_candidate,
)
from strawberry_manipulation.grasp_candidates import generate_grasp_candidates  # noqa: E402
from strawberry_manipulation.whole_chain import (  # noqa: E402
    ChainEvaluation,
    ChainFailureCode,
)


class BoundedPregraspCandidateTests(unittest.TestCase):
    def test_matches_primary_and_execution_retry_orientation(self):
        center = Pose(0.48, -0.12, 0.54)

        candidates = bounded_pregrasp_candidates_for_fruit_center(center)
        primary = candidates[0]

        self.assertEqual(len(candidates), 4)
        self.assertEqual(primary, pregrasp_pose_for_fruit_center(center))
        self.assertEqual(candidates[1], alternate_approach(primary))
        self.assertEqual(candidates[2], rotate_about_base_z(primary, math.pi))
        self.assertEqual(
            candidates[3], rotate_about_base_z(primary, 3.0 * math.pi / 2.0)
        )


class ContactSettleTests(unittest.TestCase):
    def test_waits_for_fresh_bilateral_contact_without_motion(self):
        samples = iter(("RIGHT_SINGLE_FRUIT", "BILATERAL_SAME_FRUIT"))
        now = [0.0]
        sleeps = []

        def sleep(duration):
            sleeps.append(duration)
            now[0] += duration

        result = wait_for_bilateral_contact(
            lambda: next(samples),
            timeout_sec=0.20,
            sample_period_sec=0.05,
            monotonic=lambda: now[0],
            sleep=sleep,
        )

        self.assertEqual(result, "BILATERAL_SAME_FRUIT")
        self.assertEqual(sleeps, [0.05])

    def test_returns_last_single_contact_when_window_expires(self):
        now = [0.0]

        def sleep(duration):
            now[0] += duration

        result = wait_for_bilateral_contact(
            lambda: "LEFT_SINGLE_FRUIT",
            timeout_sec=0.10,
            sample_period_sec=0.04,
            monotonic=lambda: now[0],
            sleep=sleep,
        )

        self.assertEqual(result, "LEFT_SINGLE_FRUIT")


class FakeBackend:
    def __init__(
        self,
        outcomes=None,
        *,
        prepare=True,
        allow_contact=True,
        restore=True,
        attach=True,
        attach_results=None,
        detach=True,
        in_bin=True,
        return_route=True,
        home=True,
        close_results=None,
        open_results=None,
        centering_offsets=None,
        contact_classes=None,
    ):
        self.outcomes = list(outcomes or [])
        self.prepare_ok = prepare
        self.allow_contact_ok = allow_contact
        self.restore_ok = restore
        self.attach_ok = attach
        self.attach_results = list(attach_results or [])
        self.detach_ok = detach
        self.in_bin_ok = in_bin
        self.return_route_ok = return_route
        self.home_ok = home
        self.close_results = list(close_results or [])
        self.open_results = list(open_results or [])
        self.centering_offsets = list(centering_offsets or [])
        self.contact_classes = list(contact_classes or [])
        self.calls = []
        self.poses = []

    def prepare_pick(self, target_id, target_pose):
        self.calls.append("prepare")
        return self.prepare_ok

    def allow_target_contact(self, target_id):
        self.calls.append("allow_contact")
        return self.allow_contact_ok

    def restore_target_collision(self, target_id):
        self.calls.append("restore")
        return self.restore_ok

    def move_to(self, pose, stage):
        self.calls.append(stage)
        self.poses.append((stage, pose))
        if self.outcomes:
            return self.outcomes.pop(0)
        return MotionOutcome(True, 0.1, 0.2)

    def close_gripper(self):
        self.calls.append("close")
        return self.close_results.pop(0) if self.close_results else True

    def gripper_centering_offset_m(self):
        self.calls.append("centering_offset")
        return self.centering_offsets.pop(0) if self.centering_offsets else None

    def gripper_fruit_contact_class(self):
        self.calls.append("contact_class")
        return self.contact_classes.pop(0) if self.contact_classes else None

    def open_gripper(self):
        self.calls.append("open")
        return self.open_results.pop(0) if self.open_results else True

    def attach(self, target_id):
        self.calls.append("attach")
        return self.attach_results.pop(0) if self.attach_results else self.attach_ok

    def detach(self, target_id):
        self.calls.append("detach")
        return self.detach_ok

    def move_home(self):
        self.calls.append("home")
        return self.home_ok

    def fruit_in_bin(self, target_id, stable_for_sec):
        self.calls.append("verify")
        return self.in_bin_ok

    def return_via_recorded_place_route(self):
        self.calls.append("return_route")
        return MotionOutcome(self.return_route_ok, 0.03, 0.04)


class PickAndPlaceTests(unittest.TestCase):
    def setUp(self):
        self.target = Pose(0.4, 0.1, 0.5)
        self.bin = Pose(0.3, -0.4, 0.4)

    def _authorized_plan(
        self, candidate_index=4, *, target_id=8, scene_signature="scene-a"
    ):
        candidate = generate_grasp_candidates(self.target)[candidate_index]
        return authorize_grasp_candidate(
            candidate,
            ChainEvaluation(
                ChainFailureCode.FEASIBLE,
                0.25,
                1.5,
                (
                    "PREGRASP",
                    "APPROACH",
                    "GRASP_STATE",
                    "VIRTUAL_ATTACH",
                    "ESCAPE",
                    "TRANSPORT",
                    "BIN_APPROACH",
                ),
                "certified in copied planning scene",
            ),
            target_id=target_id,
            scene_signature=scene_signature,
            certificate_timestamp_ns=42,
        )

    def test_successful_sequence(self):
        backend = FakeBackend()
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertTrue(result.success)
        self.assertEqual(result.failure_code, FailureCode.NONE)
        self.assertEqual(result.recovery_disposition, RecoveryDisposition.AT_HOME)
        self.assertEqual(result.stages[-1], "DONE")
        self.assertIn("attach", backend.calls)
        self.assertIn("detach", backend.calls)
        self.assertLess(backend.calls.index("detach"), backend.calls.index("verify"))
        release_detach = backend.calls.index("detach")
        self.assertEqual(backend.calls[release_detach : release_detach + 3], [
            "detach",
            "open",
            "verify",
        ])
        self.assertLess(backend.calls.index("prepare"), backend.calls.index("APPROACH"))
        self.assertLess(
            backend.calls.index("allow_contact"),
            backend.calls.index("GRASP_POSE"),
        )
        self.assertEqual(backend.calls[-3:], ["return_route", "home", "restore"])

    def test_release_detach_failure_keeps_recovery_motion_withheld(self):
        backend = FakeBackend(detach=False)

        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLACE_FAILED)
        self.assertIn("recovery motion are withheld", result.message)
        self.assertEqual(
            result.recovery_disposition, RecoveryDisposition.MOTION_WITHHELD
        )
        self.assertNotIn("verify", backend.calls)
        self.assertNotIn("home", backend.calls)
        self.assertNotIn("open", backend.calls[backend.calls.index("detach") + 1 :])
        self.assertNotIn("restore", backend.calls)
        self.assertEqual(result.payload_state, PayloadState.AT_BIN)
        self.assertIn("payload remains AT_BIN", result.message)

    def test_release_open_failure_occurs_only_after_detach(self):
        # The first successful open prepares the grasp; the second fails at
        # the release point after the rigid attachment has been removed.
        backend = FakeBackend(open_results=[True, False, True])

        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLACE_FAILED)
        self.assertNotIn("verify", backend.calls)
        release_detach = backend.calls.index("detach")
        self.assertEqual(
            backend.calls[release_detach : release_detach + 2],
            ["detach", "open"],
        )
        self.assertEqual(
            backend.calls[release_detach : release_detach + 5],
            ["detach", "open", "open", "return_route", "home"],
        )
        self.assertEqual(result.recovery_disposition, RecoveryDisposition.AT_HOME)

    def test_release_open_failure_withholds_home_if_recorded_return_fails(self):
        backend = FakeBackend(
            open_results=[True, False, True],
            return_route=False,
        )

        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLACE_FAILED)
        self.assertIn("recorded place-route recovery failed", result.message)
        self.assertIn("home motion withheld", result.message)
        self.assertNotIn("home", backend.calls)
        self.assertIn("RECOVERY_RETURN_ROUTE", result.stages)
        self.assertEqual(
            result.recovery_disposition, RecoveryDisposition.MOTION_WITHHELD
        )

    def test_verify_failure_returns_via_recorded_route_before_home(self):
        backend = FakeBackend(in_bin=False)

        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLACE_FAILED)
        verify = backend.calls.index("verify")
        self.assertEqual(
            backend.calls[verify : verify + 4],
            ["verify", "open", "return_route", "home"],
        )
        self.assertIn("RECOVERY_RETURN_ROUTE", result.stages)
        self.assertEqual(result.recovery_disposition, RecoveryDisposition.AT_HOME)

    def test_zero_motion_place_failure_recovers_without_recorded_replay(self):
        backend = FakeBackend(
            [
                MotionOutcome(True),
                MotionOutcome(True),
                MotionOutcome(True),
                MotionOutcome(False, collision=True),
            ]
        )

        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLACE_FAILED)
        self.assertNotIn("verify", backend.calls)
        self.assertNotIn("return_route", backend.calls)
        self.assertNotIn("detach", backend.calls)
        self.assertNotIn("home", backend.calls)
        self.assertNotIn("restore", backend.calls)
        self.assertEqual(result.recovery_disposition, RecoveryDisposition.MOTION_WITHHELD)
        self.assertEqual(result.payload_state, PayloadState.ESCAPED)

    def test_partial_place_failure_withholds_unrecorded_home_sweep(self):
        backend = FakeBackend(
            [
                MotionOutcome(True),
                MotionOutcome(True),
                MotionOutcome(True),
                MotionOutcome(False, execution_time_sec=0.2),
            ]
        )

        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLACE_FAILED)
        self.assertIn("no complete recorded return route", result.message)
        self.assertIn("home motion withheld", result.message)
        self.assertNotIn("verify", backend.calls)
        self.assertNotIn("return_route", backend.calls)
        self.assertNotIn("home", backend.calls)
        self.assertNotIn("detach", backend.calls)
        self.assertNotIn("restore", backend.calls)
        self.assertEqual(
            result.recovery_disposition, RecoveryDisposition.MOTION_WITHHELD
        )

    def test_target_collision_gate_fails_closed(self):
        backend = FakeBackend(prepare=False)
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertNotIn("APPROACH", backend.calls)
        self.assertNotIn("restore", backend.calls)
        self.assertEqual(
            result.recovery_disposition, RecoveryDisposition.HOME_REQUIRED
        )

        backend = FakeBackend(allow_contact=False)
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertIn("APPROACH", backend.calls)
        self.assertNotIn("GRASP_POSE", backend.calls)
        self.assertEqual(backend.calls[-3:], ["open", "home", "restore"])

    def test_one_approach_retry_then_success(self):
        backend = FakeBackend(
            [
                MotionOutcome(False),
                MotionOutcome(True),
                MotionOutcome(True),
                MotionOutcome(True),
                MotionOutcome(True),
            ]
        )
        result = PickAndPlaceExecutor(backend).execute(2, self.target, self.bin)
        self.assertTrue(result.success)
        self.assertEqual(backend.calls.count("APPROACH_RETRY"), 1)

    def test_four_approach_failures_abort_without_unsafe_home_sweep(self):
        backend = FakeBackend([MotionOutcome(False)] * 4)
        result = PickAndPlaceExecutor(backend).execute(3, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertEqual(backend.calls[-2:], ["open", "restore"])
        self.assertNotIn("home", backend.calls)

    def test_attach_failure_is_grasp_failure(self):
        backend = FakeBackend(attach=False)
        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.GRASP_FAILED)
        self.assertEqual(backend.calls.count("RECOVERY_RETREAT"), 1)
        self.assertLess(
            backend.calls.index("RECOVERY_RETREAT"), backend.calls.index("home")
        )
        self.assertEqual(result.recovery_disposition, RecoveryDisposition.AT_HOME)

    def test_failed_unattached_recovery_retreat_withholds_home(self):
        backend = FakeBackend(
            [
                MotionOutcome(True),
                MotionOutcome(True),
                MotionOutcome(True),
                MotionOutcome(True),
                MotionOutcome(False, collision=True),
            ],
            attach_results=[False, False],
        )

        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertIn("recovery retreat failed", result.message)
        self.assertNotIn("home", backend.calls)
        self.assertEqual(backend.calls[-1], "restore")
        self.assertEqual(
            result.recovery_disposition, RecoveryDisposition.MOTION_WITHHELD
        )

    def test_asymmetric_contact_gets_one_measured_centering_retry(self):
        backend = FakeBackend(
            close_results=[False, True],
            centering_offsets=[0.0045],
            contact_classes=["LEFT_SINGLE_FRUIT"],
        )

        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertTrue(result.success)
        self.assertEqual(backend.calls.count("CONTACT_CENTERING_PREP"), 1)
        self.assertEqual(backend.calls.count("CONTACT_CENTERING_GRASP"), 1)
        self.assertEqual(backend.calls.count("close"), 2)
        poses = dict(backend.poses)
        self.assertEqual(
            poses["CONTACT_CENTERING_GRASP"],
            offset_along_local_y(poses["GRASP_POSE"], 0.0045),
        )
        self.assertEqual(
            poses["RETREAT"].qz,
            poses["CONTACT_CENTERING_GRASP"].qz,
        )
        self.assertEqual(
            poses["RETREAT"],
            offset_along_local_z(poses["CONTACT_CENTERING_GRASP"], -0.08),
        )

    def test_missing_finger_measurement_keeps_orthogonal_fallback(self):
        backend = FakeBackend(close_results=[False, True])

        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertTrue(result.success)
        poses = dict(backend.poses)
        self.assertEqual(
            poses["CONTACT_RETRY_GRASP"],
            rotate_about_base_z(poses["GRASP_POSE"], math.pi / 2.0),
        )

    def test_right_contact_and_negative_asymmetry_get_centering_retry(self):
        backend = FakeBackend(
            close_results=[False, True],
            centering_offsets=[-0.0045],
            contact_classes=["RIGHT_SINGLE_FRUIT"],
        )

        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertTrue(result.success)
        poses = dict(backend.poses)
        self.assertEqual(
            poses["CONTACT_CENTERING_GRASP"],
            offset_along_local_y(poses["GRASP_POSE"], -0.0045),
        )

    def test_out_of_bounds_measured_centering_fails_closed(self):
        backend = FakeBackend(
            close_results=[False],
            centering_offsets=[0.011],
            contact_classes=["LEFT_SINGLE_FRUIT"],
        )

        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.GRASP_FAILED)
        self.assertNotIn("CONTACT_CENTERING_PREP", backend.calls)
        self.assertNotIn("CONTACT_RETRY_PREP", backend.calls)

    def test_contact_side_overrides_unreliable_joint_difference_sign(self):
        backend = FakeBackend(
            close_results=[False, True],
            centering_offsets=[0.0045],
            contact_classes=["RIGHT_SINGLE_FRUIT"],
        )

        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertTrue(result.success)
        poses = dict(backend.poses)
        self.assertEqual(
            poses["CONTACT_CENTERING_GRASP"],
            offset_along_local_y(poses["GRASP_POSE"], -0.0045),
        )

    def test_left_contact_corrects_positive_despite_negative_joint_difference(self):
        backend = FakeBackend(
            close_results=[False, True],
            centering_offsets=[-0.0065],
            contact_classes=["LEFT_SINGLE_FRUIT"],
        )

        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertTrue(result.success)
        poses = dict(backend.poses)
        self.assertEqual(
            poses["CONTACT_CENTERING_GRASP"],
            offset_along_local_y(poses["GRASP_POSE"], 0.0065),
        )

    def test_centering_rejects_nonfruit_or_ambiguous_contact(self):
        for contact_class in (
            "NO_FRUIT_CONTACT",
            "BILATERAL_SAME_FRUIT",
            "AMBIGUOUS_FRUIT_CONTACT",
            "CONTACT_CLASS_UNAVAILABLE",
        ):
            with self.subTest(contact_class=contact_class):
                backend = FakeBackend(
                    close_results=[False],
                    centering_offsets=[-0.0045],
                    contact_classes=[contact_class],
                )

                result = PickAndPlaceExecutor(backend).execute(
                    4, self.target, self.bin
                )

                self.assertFalse(result.success)
                self.assertNotIn("CONTACT_CENTERING_PREP", backend.calls)

    def test_single_side_attachment_rejection_gets_one_orthogonal_retry(self):
        backend = FakeBackend(attach_results=[False, True])

        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertTrue(result.success)
        self.assertEqual(backend.calls.count("attach"), 2)
        self.assertEqual(backend.calls.count("CONTACT_RETRY_PREP"), 1)
        self.assertEqual(backend.calls.count("CONTACT_RETRY_GRASP"), 1)
        self.assertEqual(backend.calls.count("close"), 2)

    def test_contact_retry_fails_closed_when_checked_retreat_is_unavailable(self):
        backend = FakeBackend(
            [MotionOutcome(True), MotionOutcome(True), MotionOutcome(False, collision=True)],
            close_results=[False],
        )

        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.GRASP_FAILED)
        self.assertEqual(backend.calls.count("CONTACT_RETRY_PREP"), 1)
        self.assertNotIn("CONTACT_RETRY_GRASP", backend.calls)
        self.assertNotIn("attach", backend.calls)

    def test_grasp_collision_gets_one_alternate_orientation_retry(self):
        backend = FakeBackend(
            [
                MotionOutcome(True),
                MotionOutcome(False, collision=True),
                MotionOutcome(True),
                MotionOutcome(True),
                MotionOutcome(True),
                MotionOutcome(True),
            ]
        )
        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertTrue(result.success)
        self.assertEqual(backend.calls.count("GRASP_RETRY_PREP"), 1)
        self.assertEqual(backend.calls.count("GRASP_POSE_RETRY"), 1)
        poses = dict(backend.poses)
        expected_grasp = alternate_approach(poses["GRASP_POSE"])
        self.assertEqual(poses["GRASP_POSE_RETRY"], expected_grasp)
        self.assertEqual(
            poses["RETREAT"].qx,
            expected_grasp.qx,
        )
        self.assertEqual(
            poses["RETREAT"].qy,
            expected_grasp.qy,
        )

    def test_noncollision_grasp_failure_does_not_retry(self):
        backend = FakeBackend([MotionOutcome(True), MotionOutcome(False)])
        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertNotIn("GRASP_RETRY_PREP", backend.calls)
        self.assertNotIn("GRASP_POSE_RETRY", backend.calls)

    def test_grasp_collision_checks_all_three_alternate_orientations(self):
        backend = FakeBackend(
            [
                MotionOutcome(True),
                MotionOutcome(False, collision=True),
                MotionOutcome(True),
                MotionOutcome(False, collision=True),
                MotionOutcome(True),
                MotionOutcome(False, collision=True),
                MotionOutcome(True),
                MotionOutcome(False, collision=True),
            ]
        )
        result = PickAndPlaceExecutor(backend).execute(4, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.COLLISION)
        self.assertIn("three alternate orientations", result.message)
        self.assertEqual(
            [stage for stage in backend.calls if stage.startswith("GRASP_RETRY_PREP")],
            [
                "GRASP_RETRY_PREP",
                "GRASP_RETRY_PREP_2",
                "GRASP_RETRY_PREP_3",
            ],
        )
        original_grasp = dict(backend.poses)["GRASP_POSE"]
        poses = dict(backend.poses)
        self.assertEqual(
            poses["GRASP_POSE_RETRY_2"],
            rotate_about_base_z(original_grasp, math.pi),
        )

    def test_failure_after_attachment_retains_payload_and_withholds_motion(self):
        backend = FakeBackend(
            [MotionOutcome(True), MotionOutcome(True), MotionOutcome(False)]
        )
        result = PickAndPlaceExecutor(backend).execute(5, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertEqual(result.payload_state, PayloadState.HOLDING)
        self.assertEqual(result.recovery_disposition, RecoveryDisposition.MOTION_WITHHELD)
        self.assertIn("PAYLOAD_HELD_MOTION_WITHHELD", result.stages)
        self.assertNotIn("detach", backend.calls)
        self.assertNotIn("open", backend.calls[backend.calls.index("attach") + 1 :])
        self.assertNotIn("home", backend.calls)
        self.assertNotIn("restore", backend.calls)

    def test_success_exposes_ordered_payload_lifecycle_feedback(self):
        backend = FakeBackend()
        feedback = []

        result = PickAndPlaceExecutor(backend).execute(
            5,
            self.target,
            self.bin,
            lambda stage, progress: feedback.append((stage, progress)),
        )

        self.assertTrue(result.success)
        self.assertEqual(result.payload_state, PayloadState.RELEASED)
        lifecycle = [stage for stage, _ in feedback if stage in {
            "CONTACT", "HOLDING", "ESCAPED", "AT_BIN", "RELEASED"
        }]
        self.assertEqual(
            lifecycle,
            ["CONTACT", "HOLDING", "ESCAPED", "AT_BIN", "RELEASED"],
        )

    def test_retained_payload_interlocks_later_goals(self):
        backend = FakeBackend(
            [MotionOutcome(True), MotionOutcome(True), MotionOutcome(False)],
        )
        executor = PickAndPlaceExecutor(backend)
        first = executor.execute(6, self.target, self.bin)
        calls_after_first = list(backend.calls)
        second = executor.execute(7, self.target, self.bin)

        self.assertEqual(first.payload_state, PayloadState.HOLDING)
        self.assertFalse(second.success)
        self.assertEqual(second.recovery_disposition, RecoveryDisposition.MOTION_WITHHELD)
        self.assertEqual(second.payload_state, PayloadState.HOLDING)
        self.assertEqual(second.stages, ("PAYLOAD_INTERLOCK",))
        self.assertIn("payload interlock active", second.message)
        self.assertEqual(backend.calls, calls_after_first)

    def test_invalid_target_never_moves(self):
        backend = FakeBackend()
        result = PickAndPlaceExecutor(backend).execute(0, self.target, self.bin)
        self.assertEqual(result.failure_code, FailureCode.NO_TARGET)
        self.assertEqual(backend.calls, [])
        self.assertEqual(
            result.recovery_disposition, RecoveryDisposition.HOME_REQUIRED
        )

    def test_restore_failure_converts_success_to_planning_failure(self):
        backend = FakeBackend(restore=False)
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertIn("failed to restore target collision obstacle", result.message)
        self.assertEqual(backend.calls[-2:], ["home", "restore"])
        self.assertEqual(
            result.recovery_disposition, RecoveryDisposition.MOTION_WITHHELD
        )

    def test_restore_failure_is_appended_to_existing_failure(self):
        backend = FakeBackend(
            [MotionOutcome(False)] * 4,
            restore=False,
        )
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertIn("approach planning failed", result.message)
        self.assertIn("failed to restore target collision obstacle", result.message)
        self.assertEqual(backend.calls[-2:], ["open", "restore"])

    def test_final_home_failure_converts_success_to_planning_failure(self):
        backend = FakeBackend(home=False)
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertIn("final home motion failed", result.message)
        self.assertEqual(backend.calls.count("home"), 1)
        self.assertEqual(backend.calls[-3:], ["return_route", "home", "restore"])
        self.assertEqual(
            result.recovery_disposition, RecoveryDisposition.MOTION_WITHHELD
        )

    def test_failed_recorded_return_withholds_independent_home_motion(self):
        backend = FakeBackend(return_route=False)

        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertIn("recorded place-route return failed", result.message)
        self.assertIn("home motion withheld", result.message)
        self.assertNotIn("home", backend.calls)
        self.assertEqual(backend.calls[-2:], ["return_route", "restore"])
        self.assertEqual(
            result.recovery_disposition, RecoveryDisposition.MOTION_WITHHELD
        )

    def test_post_attach_failure_does_not_issue_home_even_if_home_is_available(self):
        backend = FakeBackend(
            [MotionOutcome(True), MotionOutcome(True), MotionOutcome(False)],
            home=False,
        )
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertIn("payload remains HOLDING", result.message)
        self.assertEqual(backend.calls.count("home"), 0)
        self.assertEqual(
            result.recovery_disposition, RecoveryDisposition.MOTION_WITHHELD
        )

    def test_fixed_grasp_geometry_places_tool_axis_toward_positive_y(self):
        pose = Pose(0.5, 0.0, 0.6, qx=-(2**-0.5), qw=2**-0.5)
        moved = offset_along_local_z(pose, 0.1)
        self.assertAlmostEqual(moved.x, 0.5, places=6)
        self.assertAlmostEqual(moved.y, 0.1, places=6)
        self.assertAlmostEqual(moved.z, 0.6, places=6)

    def test_executor_keeps_hand_above_grasp_center(self):
        backend = FakeBackend()
        executor = PickAndPlaceExecutor(backend, tool_center_offset_m=0.1054)
        result = executor.execute(7, self.target, self.bin)
        self.assertTrue(result.success)
        grasp_pose = dict(backend.poses)["GRASP_POSE"]
        self.assertAlmostEqual(grasp_pose.x, self.target.x, places=6)
        self.assertAlmostEqual(grasp_pose.y, self.target.y, places=6)
        self.assertAlmostEqual(grasp_pose.z, self.target.z + 0.1054, places=6)

    def test_refines_target_after_approach_before_grasp(self):
        backend = FakeBackend()
        refined = Pose(0.415, 0.092, 0.487)
        executor = PickAndPlaceExecutor(
            backend,
            target_pose_refiner=lambda target_id, initial: refined,
        )
        result = executor.execute(7, self.target, self.bin)
        self.assertTrue(result.success)
        poses = dict(backend.poses)
        self.assertEqual(
            poses["APPROACH"],
            pregrasp_pose_for_fruit_center(self.target),
        )
        self.assertAlmostEqual(poses["GRASP_POSE"].x, refined.x, places=6)
        self.assertAlmostEqual(poses["GRASP_POSE"].y, refined.y, places=6)
        self.assertAlmostEqual(
            poses["GRASP_POSE"].z,
            refined.z + DEFAULT_TOOL_CENTER_OFFSET_M,
            places=6,
        )

    def test_refinement_failure_aborts_before_contact_corridor_opens(self):
        backend = FakeBackend()

        def fail_refinement(target_id, initial):
            raise RuntimeError("latest perception target is stale")

        executor = PickAndPlaceExecutor(
            backend,
            target_pose_refiner=fail_refinement,
        )
        result = executor.execute(7, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.STALE_DATA)
        self.assertIn("latest perception target is stale", result.message)
        self.assertNotIn("allow_contact", backend.calls)
        self.assertNotIn("GRASP_POSE", backend.calls)

    def test_planning_shadow_pregrasp_matches_executor_geometry(self):
        backend = FakeBackend()
        result = PickAndPlaceExecutor(backend).execute(7, self.target, self.bin)
        self.assertTrue(result.success)
        executor_pregrasp = dict(backend.poses)["APPROACH"]
        shared_pregrasp = pregrasp_pose_for_fruit_center(self.target)
        self.assertEqual(executor_pregrasp, shared_pregrasp)
        self.assertAlmostEqual(
            shared_pregrasp.z,
            self.target.z + 0.1054 + 0.15,
            places=6,
        )

    def test_place_keeps_hand_above_in_bin_release_point(self):
        backend = FakeBackend()
        executor = PickAndPlaceExecutor(backend, tool_center_offset_m=0.1054)
        result = executor.execute(8, self.target, self.bin)
        self.assertTrue(result.success)
        place_pose = dict(backend.poses)["PLACE"]
        self.assertAlmostEqual(place_pose.x, self.bin.x, places=6)
        self.assertAlmostEqual(place_pose.y, self.bin.y, places=6)
        self.assertAlmostEqual(place_pose.z, self.bin.z + 0.1054, places=6)
        self.assertAlmostEqual(place_pose.qx, 1.0, places=6)
        self.assertAlmostEqual(place_pose.qw, 0.0, places=6)

    def test_authorized_execution_uses_exact_certified_candidate_geometry(self):
        backend = FakeBackend()
        legacy_authorizer_calls = []

        def legacy_authorizer(*args):
            legacy_authorizer_calls.append(args)
            return ChainEvaluation(ChainFailureCode.SCENE_INVALID, 0.0, 0.0)

        plan = self._authorized_plan(candidate_index=4)
        result = PickAndPlaceExecutor(
            backend, whole_chain_authorizer=legacy_authorizer
        ).execute_authorized(
            8,
            self.target,
            self.bin,
            plan,
            execution_identity=plan.execution_identity,
            scene_signature_provider=lambda: "scene-a",
        )

        self.assertTrue(result.success)
        self.assertEqual(legacy_authorizer_calls, [])
        self.assertIn("AUTHORIZED_G04", result.stages)
        moved = dict(backend.poses)
        self.assertEqual(moved["APPROACH"], plan.candidate.pregrasp_pose)
        self.assertEqual(moved["GRASP_POSE"], plan.candidate.grasp_pose)
        self.assertEqual(moved["RETREAT"], plan.candidate.escape_pose)

    def test_authorized_execution_waits_for_contact_evidence_without_replacing_geometry(self):
        plan = self._authorized_plan(candidate_index=4)
        backend = FakeBackend(
            contact_classes=["RIGHT_SINGLE_FRUIT", "BILATERAL_SAME_FRUIT"]
        )

        result = PickAndPlaceExecutor(
            backend,
            authorized_contact_settle_timeout_sec=0.02,
            authorized_contact_settle_sample_period_sec=0.001,
        ).execute_authorized(
            8,
            self.target,
            self.bin,
            plan,
            execution_identity=plan.execution_identity,
            scene_signature_provider=lambda: "scene-a",
        )

        self.assertTrue(result.success)
        self.assertEqual(backend.calls.count("contact_class"), 2)
        self.assertLess(backend.calls.index("contact_class"), backend.calls.index("attach"))
        moved = dict(backend.poses)
        self.assertEqual(moved["APPROACH"], plan.candidate.pregrasp_pose)
        self.assertEqual(moved["GRASP_POSE"], plan.candidate.grasp_pose)
        self.assertEqual(moved["RETREAT"], plan.candidate.escape_pose)

    def test_authorized_execution_identity_mismatch_denies_before_backend_command(self):
        plan = self._authorized_plan(candidate_index=2)
        mismatches = (
            ExecutionIdentity(
                8, "G03", plan.candidate.geometry_fingerprint, "scene-a"
            ),
            ExecutionIdentity(8, "G02", "different", "scene-a"),
            ExecutionIdentity(
                8, "G02", plan.candidate.geometry_fingerprint, "scene-b"
            ),
            ExecutionIdentity(
                9, "G02", plan.candidate.geometry_fingerprint, "scene-a"
            ),
        )
        for identity in mismatches:
            with self.subTest(identity=identity):
                backend = FakeBackend()
                result = PickAndPlaceExecutor(backend).execute_authorized(
                    8,
                    self.target,
                    self.bin,
                    plan,
                    execution_identity=identity,
                    scene_signature_provider=lambda: "scene-a",
                )
                self.assertFalse(result.success)
                self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
                self.assertEqual(result.stages, ("AUTHORIZED_GRASP_DENIED",))
                self.assertEqual(result.payload_state, PayloadState.EMPTY)
                self.assertEqual(backend.calls, [])

    def test_authorized_execution_without_certificate_fails_closed(self):
        backend = FakeBackend()
        result = PickAndPlaceExecutor(backend).execute_authorized(
            8,
            self.target,
            self.bin,
            None,
            execution_identity=None,
            scene_signature_provider=lambda: "scene-a",
        )
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertEqual(result.stages, ("AUTHORIZED_GRASP_DENIED",))
        self.assertEqual(result.payload_state, PayloadState.EMPTY)
        self.assertEqual(backend.calls, [])

    def test_authorized_grasp_failure_does_not_try_uncertified_geometry(self):
        plan = self._authorized_plan(candidate_index=4)
        backend = FakeBackend(
            outcomes=[
                MotionOutcome(True, 0.1, 0.2),
                MotionOutcome(False, 0.1, 0.0, collision=True),
                MotionOutcome(True, 0.1, 0.2),
            ]
        )
        result = PickAndPlaceExecutor(backend).execute_authorized(
            8,
            self.target,
            self.bin,
            plan,
            execution_identity=plan.execution_identity,
            scene_signature_provider=lambda: "scene-a",
        )

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.COLLISION)
        self.assertIn("alternate geometry is forbidden", result.message)
        self.assertFalse(
            any(
                call.startswith(("GRASP_RETRY", "CONTACT_CENTERING", "CONTACT_RETRY"))
                for call in backend.calls
            )
        )
        moved = dict(backend.poses)
        self.assertEqual(moved["APPROACH"], plan.candidate.pregrasp_pose)
        self.assertEqual(moved["GRASP_POSE"], plan.candidate.grasp_pose)

    def test_authorized_execution_requires_matching_live_scene_after_prepare(self):
        plan = self._authorized_plan(candidate_index=4)
        backend = FakeBackend()

        result = PickAndPlaceExecutor(backend).execute_authorized(
            8,
            self.target,
            self.bin,
            plan,
            execution_identity=plan.execution_identity,
            scene_signature_provider=lambda: "scene-b",
        )

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertEqual(result.stages, ("AUTHORIZED_GRASP_DENIED",))
        self.assertEqual(result.payload_state, PayloadState.EMPTY)
        self.assertEqual(backend.calls, ["prepare", "restore"])
        self.assertEqual(backend.poses, [])

    def test_authorized_execution_requires_live_scene_signature_provider(self):
        plan = self._authorized_plan(candidate_index=4)
        backend = FakeBackend()

        result = PickAndPlaceExecutor(backend).execute_authorized(
            8,
            self.target,
            self.bin,
            plan,
            execution_identity=plan.execution_identity,
            scene_signature_provider=None,
        )

        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertEqual(result.stages, ("AUTHORIZED_GRASP_DENIED",))
        self.assertEqual(result.payload_state, PayloadState.EMPTY)
        self.assertEqual(backend.calls, [])

    def test_whole_chain_rejection_happens_before_any_gripper_or_motion_command(self):
        backend = FakeBackend()
        seen = []

        def reject(target_id, target, place):
            seen.append((target_id, target, place))
            return ChainEvaluation(
                ChainFailureCode.TRANSPORT_FAILED,
                0.25,
                1.5,
                ("PREGRASP", "TRANSPORT"),
                "virtual carried fruit cannot enter bin corridor",
            )

        result = PickAndPlaceExecutor(
            backend, whole_chain_authorizer=reject
        ).execute(8, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.payload_state, PayloadState.EMPTY)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertIn("TRANSPORT_FAILED", result.message)
        self.assertEqual(len(seen), 1)
        self.assertIn("prepare", backend.calls)
        self.assertIn("restore", backend.calls)
        self.assertNotIn("open", backend.calls)
        self.assertNotIn("close", backend.calls)
        self.assertFalse(any(isinstance(call, str) and call == "attach" for call in backend.calls))

    def test_whole_chain_authorizer_exception_fails_closed_before_any_command(self):
        backend = FakeBackend()

        def explode(*_args):
            raise RuntimeError("virtual scene copy failed")

        result = PickAndPlaceExecutor(
            backend, whole_chain_authorizer=explode
        ).execute(8, self.target, self.bin)

        self.assertFalse(result.success)
        self.assertEqual(result.payload_state, PayloadState.EMPTY)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertIn("authorization raised", result.message)
        self.assertIn("restore", backend.calls)
        self.assertNotIn("open", backend.calls)
        self.assertNotIn("close", backend.calls)
        self.assertNotIn("attach", backend.calls)


if __name__ == "__main__":
    unittest.main()
