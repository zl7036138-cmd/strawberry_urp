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
    PickAndPlaceExecutor,
    Pose,
    alternate_approach,
    bounded_pregrasp_candidates_for_fruit_center,
    offset_along_local_z,
    offset_along_local_y,
    pregrasp_pose_for_fruit_center,
    rotate_about_base_z,
)


class BoundedPregraspCandidateTests(unittest.TestCase):
    def test_matches_primary_and_execution_retry_orientation(self):
        center = Pose(0.48, -0.12, 0.54)

        primary, retry = bounded_pregrasp_candidates_for_fruit_center(center)

        self.assertEqual(primary, pregrasp_pose_for_fruit_center(center))
        self.assertEqual(retry, alternate_approach(primary))


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


class PickAndPlaceTests(unittest.TestCase):
    def setUp(self):
        self.target = Pose(0.4, 0.1, 0.5)
        self.bin = Pose(0.3, -0.4, 0.4)

    def test_successful_sequence(self):
        backend = FakeBackend()
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertTrue(result.success)
        self.assertEqual(result.failure_code, FailureCode.NONE)
        self.assertEqual(result.stages[-1], "DONE")
        self.assertIn("attach", backend.calls)
        self.assertIn("detach", backend.calls)
        self.assertLess(backend.calls.index("prepare"), backend.calls.index("APPROACH"))
        self.assertLess(
            backend.calls.index("allow_contact"),
            backend.calls.index("GRASP_POSE"),
        )
        self.assertEqual(backend.calls[-2:], ["home", "restore"])

    def test_target_collision_gate_fails_closed(self):
        backend = FakeBackend(prepare=False)
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertNotIn("APPROACH", backend.calls)
        self.assertNotIn("restore", backend.calls)

        backend = FakeBackend(allow_contact=False)
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertIn("APPROACH", backend.calls)
        self.assertNotIn("GRASP_POSE", backend.calls)
        self.assertEqual(backend.calls[-2:], ["home", "restore"])

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

    def test_two_approach_failures_abort_without_unsafe_home_sweep(self):
        backend = FakeBackend([MotionOutcome(False), MotionOutcome(False)])
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

    def test_failure_after_attachment_detaches_before_recovery(self):
        backend = FakeBackend(
            [MotionOutcome(True), MotionOutcome(True), MotionOutcome(False)]
        )
        result = PickAndPlaceExecutor(backend).execute(5, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertEqual(backend.calls[-4:], ["detach", "open", "home", "restore"])

    def test_failed_recovery_detach_withholds_home_motion(self):
        backend = FakeBackend(
            [MotionOutcome(True), MotionOutcome(True), MotionOutcome(False)],
            detach=False,
        )
        result = PickAndPlaceExecutor(backend).execute(6, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertIn("recovery motion withheld", result.message)
        self.assertEqual(backend.calls[-3:], ["detach", "open", "restore"])
        self.assertNotIn("home", backend.calls)

    def test_invalid_target_never_moves(self):
        backend = FakeBackend()
        result = PickAndPlaceExecutor(backend).execute(0, self.target, self.bin)
        self.assertEqual(result.failure_code, FailureCode.NO_TARGET)
        self.assertEqual(backend.calls, [])

    def test_restore_failure_converts_success_to_planning_failure(self):
        backend = FakeBackend(restore=False)
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertIn("failed to restore target collision obstacle", result.message)
        self.assertEqual(backend.calls[-2:], ["home", "restore"])

    def test_restore_failure_is_appended_to_existing_failure(self):
        backend = FakeBackend(
            [MotionOutcome(False), MotionOutcome(False)],
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
        self.assertEqual(backend.calls[-2:], ["home", "restore"])

    def test_failed_recovery_reports_home_failure_without_retry(self):
        backend = FakeBackend(
            [MotionOutcome(True), MotionOutcome(True), MotionOutcome(False)],
            home=False,
        )
        result = PickAndPlaceExecutor(backend).execute(1, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertIn("recovery home motion failed", result.message)
        self.assertEqual(backend.calls.count("home"), 1)

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


if __name__ == "__main__":
    unittest.main()
