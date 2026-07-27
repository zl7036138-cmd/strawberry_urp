import math
import pathlib
import sys
import unittest
from types import MappingProxyType
from types import SimpleNamespace
from unittest.mock import patch


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.moveit_backend import (  # noqa: E402
    MoveItBackend,
    gripper_result_allows_command,
)
from strawberry_manipulation.core import MotionOutcome, Pose  # noqa: E402


class FakeFuture:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._result


class MoveItBackendStaticTests(unittest.TestCase):
    class Logger:
        def __init__(self):
            self.errors = []

        def error(self, message):
            self.errors.append(message)

        def info(self, message):
            pass

        def warning(self, message):
            pass

    @classmethod
    def lifecycle_backend(cls):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: cls.Logger())
        backend.fruit_obstacle_centers_m = MoveItBackend._build_fruit_manifest(
            {1: Pose(0.4, 0.1, 0.5)}
        )
        backend._planning_scene_monitor = object()
        backend.base_frame = "panda_link0"
        backend.fruit_collision_radius_m = 0.035
        backend._prepared_target_id = None
        backend._target_contact_open = False
        backend._prepared_scene_centers_m = None
        backend.fruit_pose_provider = None
        return backend

    def test_fruit_manifest_is_immutable(self):
        manifest = MoveItBackend._build_fruit_manifest(
            {1: Pose(0.4, 0.1, 0.5)}
        )
        self.assertIsInstance(manifest, MappingProxyType)
        self.assertEqual(manifest[1], (0.4, 0.1, 0.5))
        with self.assertRaises(TypeError):
            manifest[1] = (0.0, 0.0, 0.0)

    def test_close_accepts_real_contact_stall_after_measured_travel(self):
        self.assertTrue(
            gripper_result_allows_command(
                target_position_m=0.022,
                observed_position_m=0.0253,
                open_position_m=0.040,
                closed_position_m=0.022,
                stalled=True,
                reached_goal=False,
            )
        )

    def test_close_rejects_false_stall_at_fully_open_position(self):
        self.assertFalse(
            gripper_result_allows_command(
                target_position_m=0.022,
                observed_position_m=0.03999,
                open_position_m=0.040,
                closed_position_m=0.022,
                stalled=True,
                reached_goal=False,
            )
        )

    def test_prepare_rejects_unknown_target_before_scene_update(self):
        backend = self.lifecycle_backend()
        with patch(
            "strawberry_manipulation.moveit_backend.set_target_fruit_collision"
        ) as scene_update:
            self.assertFalse(backend.prepare_pick(99, Pose(0.0, 0.0, 0.0)))
        scene_update.assert_not_called()
        self.assertIsNone(backend._prepared_target_id)

    def test_allow_then_restore_uses_manifest_and_closes_lifecycle(self):
        backend = self.lifecycle_backend()
        with patch(
            "strawberry_manipulation.moveit_backend.set_target_fruit_collision",
            side_effect=["strawberry_fruit_1"] * 3,
        ) as scene_update:
            self.assertTrue(backend.prepare_pick(1, Pose(0.41, 0.11, 0.51)))
            self.assertTrue(backend.allow_target_contact(1))
            self.assertEqual(backend._prepared_target_id, 1)
            self.assertTrue(backend._target_contact_open)
            self.assertTrue(backend.restore_target_collision(1))
        self.assertEqual(scene_update.call_count, 3)
        restore_call = scene_update.call_args_list[-1]
        self.assertEqual(restore_call.kwargs["center_m"], (0.4, 0.1, 0.5))
        self.assertIsNone(backend._prepared_target_id)
        self.assertFalse(backend._target_contact_open)

    def test_prepare_synchronizes_all_live_fruit_before_target_update(self):
        backend = self.lifecycle_backend()
        backend.fruit_obstacle_centers_m = MoveItBackend._build_fruit_manifest(
            {
                1: Pose(0.4, 0.1, 0.5),
                2: Pose(0.4, 0.0, 0.5),
                3: Pose(0.4, -0.1, 0.5),
            }
        )
        live = {
            1: (0.45, -0.05, 0.52),
            2: (0.2, -2.0, 1.0),
            3: (0.4, -2.0, 1.0),
        }
        backend.fruit_pose_provider = lambda: live
        with patch(
            "strawberry_manipulation.moveit_backend.apply_fruit_collision_scene"
        ) as synchronize, patch(
            "strawberry_manipulation.moveit_backend.set_target_fruit_collision",
            return_value="strawberry_fruit_1",
        ) as target_update:
            self.assertTrue(
                backend.prepare_pick(1, Pose(0.45, -0.05, 0.52))
            )
        synchronize.assert_called_once()
        self.assertEqual(live, dict(synchronize.call_args.args[2]))
        self.assertEqual((0.45, -0.05, 0.52), target_update.call_args.kwargs["center_m"])

    def test_prepare_fails_closed_when_live_truth_ids_are_incomplete(self):
        backend = self.lifecycle_backend()
        backend.fruit_pose_provider = lambda: {}
        with patch(
            "strawberry_manipulation.moveit_backend.apply_fruit_collision_scene"
        ) as synchronize, patch(
            "strawberry_manipulation.moveit_backend.set_target_fruit_collision"
        ) as target_update:
            self.assertFalse(backend.prepare_pick(1, Pose(0.4, 0.1, 0.5)))
        synchronize.assert_not_called()
        target_update.assert_not_called()

    def test_restore_uses_latest_live_target_pose(self):
        backend = self.lifecycle_backend()
        backend.fruit_pose_provider = lambda: {1: (0.35, -0.45, 0.45)}
        backend._prepared_target_id = 1
        backend._target_contact_open = True
        with patch(
            "strawberry_manipulation.moveit_backend.set_target_fruit_collision",
            return_value="strawberry_fruit_1",
        ) as scene_update:
            self.assertTrue(backend.restore_target_collision(1))
        self.assertEqual(
            (0.35, -0.45, 0.45), scene_update.call_args.kwargs["center_m"]
        )

    def test_guarded_place_aligns_above_bin_before_vertical_descent(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.safe_transit_clearance_m = 0.02
        state = [Pose(0.42, -0.12, 0.70, qx=1.0, qw=0.0)]
        segments = []

        backend._current_link_pose = lambda: state[-1]

        def move_segment(start, target, *, intermediate_endpoint=False):
            self.assertEqual(start, state[-1])
            segments.append((target, intermediate_endpoint))
            state.append(target)
            return MotionOutcome(True, 0.01, 0.02)

        backend._move_segmented_between = move_segment
        target = Pose(0.35, -0.45, 0.5554, qx=1.0, qw=0.0)
        outcome = backend._move_guarded_place(target)

        self.assertTrue(outcome.success)
        self.assertEqual(len(segments), 4)
        self.assertAlmostEqual(segments[0][0].z, 0.72)
        self.assertEqual(
            (segments[2][0].x, segments[2][0].y, segments[2][0].z),
            (0.35, -0.45, 0.72),
        )
        self.assertEqual(segments[-1], (target, False))
        self.assertTrue(all(intermediate for _, intermediate in segments[:-1]))

    def test_retreat_uses_checked_cartesian_segments(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        expected = MotionOutcome(True, 0.1, 0.2)
        requested = []
        backend._move_to_segmented = lambda pose: requested.append(pose) or expected

        target = Pose(0.42, -0.12, 0.7054, qx=1.0, qw=0.0)
        self.assertIs(backend.move_to(target, "RETREAT"), expected)
        self.assertEqual(requested, [target])

    def test_wrist_observation_uses_startup_verified_action_path(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        expected = MotionOutcome(True, 0.1, 0.2)
        requested = []
        backend._move_to_planned_joint_path = (
            lambda pose: requested.append(pose) or expected
        )

        target = Pose(0.28, 0.0, 0.72, qy=0.95, qw=0.31)
        self.assertIs(
            backend.move_to(target, "WRIST_OBSERVATION"), expected
        )
        self.assertEqual(requested, [target])

    def test_direct_joint_path_waits_for_action_server_before_goal(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        logger = self.Logger()
        backend.node = SimpleNamespace(get_logger=lambda: logger)
        backend.request_timeout_sec = 5.0
        backend._arm_action_probe = SimpleNamespace(
            wait_for_server=lambda timeout_sec: False
        )

        succeeded, execution_time = backend._execute_joint_path(
            (0.0,), ((0.1,),)
        )

        self.assertFalse(succeeded)
        self.assertEqual(execution_time, 0.0)
        self.assertIn(
            "Panda arm trajectory action server became unavailable",
            logger.errors,
        )

    def test_cartesian_endpoint_miss_gets_exactly_one_measured_correction(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        logger = self.Logger()
        backend.node = SimpleNamespace(get_logger=lambda: logger)
        backend.cartesian_endpoint_retry_limit = 1
        backend.intermediate_position_tolerance_m = 0.02
        backend.intermediate_orientation_tolerance_rad = math.radians(8.0)
        backend.pose_link = "panda_hand"
        start = Pose(0.0, 0.0, 0.0)
        measured = Pose(0.09, 0.0, 0.0)
        target = Pose(0.10, 0.0, 0.0)
        dense_starts = []
        backend._dense_pose_waypoints = (
            lambda current, requested: dense_starts.append(current) or (requested,)
        )
        backend._solve_cartesian_joint_path = lambda waypoints: (
            (0.0,),
            ((0.1,),),
            False,
            0.01,
        )
        executions = []
        backend._execute_joint_path = (
            lambda initial, path: executions.append((initial, path)) or (True, 0.02)
        )
        backend._wait_until_arm_settled = lambda: True
        backend._current_link_pose = lambda: measured
        verification_results = iter((False, True))
        backend._pose_is_within_tolerance = lambda *args, **kwargs: next(
            verification_results
        )

        observed = SimpleNamespace()

        class Scene:
            current_state = SimpleNamespace(get_pose=lambda link: observed)

        class ReadOnly:
            def __enter__(self):
                return Scene()

            def __exit__(self, exc_type, exc_value, traceback):
                return False

        backend._planning_scene_monitor = SimpleNamespace(
            read_only=lambda: ReadOnly()
        )
        outcome = backend._move_segmented_between(start, target)

        self.assertTrue(outcome.success)
        self.assertEqual(dense_starts, [start, measured])
        self.assertEqual(len(executions), 2)
        self.assertAlmostEqual(outcome.planning_time_sec, 0.02)
        self.assertAlmostEqual(outcome.execution_time_sec, 0.04)

    def test_cartesian_endpoint_retry_is_bounded(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.cartesian_endpoint_retry_limit = 1
        backend.intermediate_position_tolerance_m = 0.02
        backend.intermediate_orientation_tolerance_rad = math.radians(8.0)
        backend.pose_link = "panda_hand"
        backend._dense_pose_waypoints = lambda current, target: (target,)
        backend._solve_cartesian_joint_path = lambda waypoints: (
            (0.0,),
            ((0.1,),),
            False,
            0.0,
        )
        execution_count = 0

        def execute(initial, path):
            nonlocal execution_count
            execution_count += 1
            return True, 0.0

        backend._execute_joint_path = execute
        backend._wait_until_arm_settled = lambda: True
        backend._current_link_pose = lambda: Pose(0.05, 0.0, 0.0)
        backend._pose_is_within_tolerance = lambda *args, **kwargs: False
        observed = SimpleNamespace()

        class ReadOnly:
            def __enter__(self):
                return SimpleNamespace(
                    current_state=SimpleNamespace(get_pose=lambda link: observed)
                )

            def __exit__(self, exc_type, exc_value, traceback):
                return False

        backend._planning_scene_monitor = SimpleNamespace(
            read_only=lambda: ReadOnly()
        )
        outcome = backend._move_segmented_between(
            Pose(0.0, 0.0, 0.0), Pose(0.1, 0.0, 0.0)
        )

        self.assertFalse(outcome.success)
        self.assertEqual(execution_count, 2)

    def test_future_wait_returns_completed_result(self):
        value = object()
        self.assertIs(MoveItBackend._wait_future(FakeFuture(value), 0.01), value)

    def test_future_wait_converts_exception_to_none(self):
        class BrokenFuture(FakeFuture):
            def result(self):
                raise RuntimeError("broken")

        self.assertIsNone(MoveItBackend._wait_future(BrokenFuture(None), 0.01))

    def test_execution_result_normalization_is_fail_closed(self):
        class Code:
            def __init__(self, value):
                self.val = value

        class Status:
            status = "SUCCEEDED"

            def __init__(self, successful):
                self.successful = successful

            def __bool__(self):
                return self.successful

        self.assertTrue(MoveItBackend._execution_succeeded(None))
        self.assertTrue(MoveItBackend._execution_succeeded(True))
        self.assertTrue(MoveItBackend._execution_succeeded(Code(1)))
        self.assertTrue(MoveItBackend._execution_succeeded(Status(True)))
        self.assertFalse(MoveItBackend._execution_succeeded(False))
        self.assertFalse(MoveItBackend._execution_succeeded(Code(-1)))
        self.assertFalse(MoveItBackend._execution_succeeded(Status(False)))
        self.assertFalse(MoveItBackend._execution_succeeded(object()))

    def test_pose_error_handles_equivalent_quaternion_signs(self):
        observed = SimpleNamespace(
            position=SimpleNamespace(x=0.1, y=0.2, z=0.3),
            orientation=SimpleNamespace(x=-1.0, y=0.0, z=0.0, w=0.0),
        )
        translation, orientation = MoveItBackend._pose_errors(
            Pose(0.1, 0.2, 0.3, qx=1.0, qw=0.0), observed
        )
        self.assertAlmostEqual(translation, 0.0)
        self.assertAlmostEqual(orientation, 0.0)

    def test_pose_error_reports_translation_and_rotation(self):
        observed = SimpleNamespace(
            position=SimpleNamespace(x=0.11, y=0.2, z=0.3),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        translation, orientation = MoveItBackend._pose_errors(
            Pose(0.1, 0.2, 0.3, qx=1.0, qw=0.0), observed
        )
        self.assertAlmostEqual(translation, 0.01)
        self.assertAlmostEqual(orientation, math.pi)

    def test_pose_interpolation_uses_shortest_quaternion_arc(self):
        start = Pose(0.0, 0.0, 0.0, qw=1.0)
        target = Pose(1.0, 2.0, 3.0, qz=-1.0, qw=0.0)
        midpoint = MoveItBackend._interpolate_pose(start, target, 0.5)
        self.assertEqual((midpoint.x, midpoint.y, midpoint.z), (0.5, 1.0, 1.5))
        self.assertAlmostEqual(midpoint.qz, -2**-0.5)
        self.assertAlmostEqual(midpoint.qw, 2**-0.5)
        self.assertAlmostEqual(
            MoveItBackend._orientation_distance(start, target), math.pi
        )


if __name__ == "__main__":
    unittest.main()
