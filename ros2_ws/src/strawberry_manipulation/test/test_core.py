import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.core import (  # noqa: E402
    FailureCode,
    MotionOutcome,
    PickAndPlaceExecutor,
    Pose,
    offset_along_local_z,
    pregrasp_pose_for_fruit_center,
)


class FakeBackend:
    def __init__(
        self,
        outcomes=None,
        *,
        prepare=True,
        allow_contact=True,
        restore=True,
        attach=True,
        detach=True,
        in_bin=True,
    ):
        self.outcomes = list(outcomes or [])
        self.prepare_ok = prepare
        self.allow_contact_ok = allow_contact
        self.restore_ok = restore
        self.attach_ok = attach
        self.detach_ok = detach
        self.in_bin_ok = in_bin
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

    def close_gripper(self): self.calls.append("close"); return True
    def open_gripper(self): self.calls.append("open"); return True
    def attach(self, target_id): self.calls.append("attach"); return self.attach_ok
    def detach(self, target_id): self.calls.append("detach"); return self.detach_ok
    def move_home(self): self.calls.append("home"); return True
    def fruit_in_bin(self, target_id, stable_for_sec): self.calls.append("verify"); return self.in_bin_ok


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
        self.assertLess(
            backend.calls.index("prepare"), backend.calls.index("APPROACH")
        )
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
            [MotionOutcome(False), MotionOutcome(True), MotionOutcome(True), MotionOutcome(True), MotionOutcome(True)]
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

    def test_failure_after_attachment_detaches_before_recovery(self):
        backend = FakeBackend(
            [MotionOutcome(True), MotionOutcome(True), MotionOutcome(False)]
        )
        result = PickAndPlaceExecutor(backend).execute(5, self.target, self.bin)
        self.assertFalse(result.success)
        self.assertEqual(result.failure_code, FailureCode.PLANNING_FAILED)
        self.assertEqual(
            backend.calls[-4:], ["detach", "open", "home", "restore"]
        )

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

    def test_planning_shadow_pregrasp_matches_executor_geometry(self):
        backend = FakeBackend()
        result = PickAndPlaceExecutor(backend).execute(
            7, self.target, self.bin
        )
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
