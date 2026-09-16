import math
import pathlib
import sys
import threading
import unittest
from types import MappingProxyType
from types import SimpleNamespace
from unittest.mock import patch


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.moveit_backend import (  # noqa: E402
    ArmStateSnapshot,
    MoveItBackend,
    PathAssessment,
    gripper_result_allows_command,
    joint_limit_margin_violation,
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
    def test_joint_limit_margin_rejects_hard_limit_and_accepts_interior(self):
        interior = (0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0)
        at_joint_five_limit = (0.0, 0.0, 0.0, -1.5, -2.8973, 1.5, 0.0)

        self.assertIsNone(joint_limit_margin_violation(interior, 0.01))
        violation = joint_limit_margin_violation(at_joint_five_limit, 0.01)
        self.assertEqual(violation[0], 5)

    def test_arm_result_wait_aborts_on_live_joint_limit_excursion(self):
        class PendingFuture:
            def add_done_callback(self, callback):
                self.callback = callback

        backend = MoveItBackend.__new__(MoveItBackend)
        logger = self.Logger()
        backend.node = SimpleNamespace(get_logger=lambda: logger)
        backend.execution_joint_limit_margin_rad = 0.02
        backend._latest_live_arm_positions = lambda: (
            1,
            (0.0, -1.75, 0.0, -1.5, 0.0, 1.5, 0.0),
        )

        outcome = backend._wait_arm_trajectory_result(PendingFuture(), 1.0)
        self.assertEqual(outcome.kind, "LIVE_JOINT_LIMIT_ABORT")
        self.assertIsNone(outcome.result)
        self.assertIn("live arm limit monitor", logger.errors[-1])

    def test_arm_result_wait_distinguishes_future_error_and_timeout(self):
        class BrokenFuture:
            def add_done_callback(self, callback):
                callback(self)

            def result(self):
                raise RuntimeError("broken")

        backend = MoveItBackend.__new__(MoveItBackend)
        backend._latest_live_arm_positions = lambda: None
        broken = backend._wait_arm_trajectory_result(BrokenFuture(), 1.0)
        self.assertEqual(broken.kind, "RESULT_FUTURE_ERROR")
        self.assertIsNone(broken.result)

        class PendingFuture:
            def add_done_callback(self, callback):
                self.callback = callback

        with patch(
            "strawberry_manipulation.moveit_backend.time.monotonic",
            side_effect=(0.0, 2.0),
        ):
            timeout = backend._wait_arm_trajectory_result(PendingFuture(), 1.0)
        self.assertEqual(timeout.kind, "WALL_TIMEOUT")
        self.assertIsNone(timeout.result)

    def test_bin_verification_uses_dedicated_wall_timeout(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.contact_resolved_attachment = True
        backend.bin_verification_timeout_sec = 15.0
        calls = []
        backend._trigger_contact_target = lambda operation, **kwargs: (
            calls.append((operation, kwargs)) or True
        )

        self.assertTrue(backend.fruit_in_bin(8, 1.0))
        self.assertEqual(
            calls,
            [("verify_in_bin", {"response_timeout_sec": 15.0})],
        )

    class Logger:
        def __init__(self):
            self.errors = []
            self.warnings = []

        def error(self, message):
            self.errors.append(message)

        def info(self, message):
            pass

        def warning(self, message):
            self.warnings.append(message)

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
        backend.dynamic_fruit_manifest = False
        backend.maximum_cached_collision_scene_age_sec = 120.0
        backend.maximum_cached_target_drift_m = 0.05
        backend._cached_collision_scene_centers_m = None
        backend._cached_collision_scene_monotonic = None
        backend._locked_collision_scene_target_id = None
        return backend

    def test_fruit_manifest_is_immutable(self):
        manifest = MoveItBackend._build_fruit_manifest({1: Pose(0.4, 0.1, 0.5)})
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

    def test_close_accepts_reached_goal_within_controller_tolerance(self):
        self.assertTrue(
            gripper_result_allows_command(
                target_position_m=0.022,
                observed_position_m=0.02476,
                open_position_m=0.040,
                closed_position_m=0.022,
                stalled=False,
                reached_goal=True,
            )
        )

    def test_close_rejects_reached_goal_outside_controller_tolerance(self):
        self.assertFalse(
            gripper_result_allows_command(
                target_position_m=0.022,
                observed_position_m=0.0251,
                open_position_m=0.040,
                closed_position_m=0.022,
                stalled=False,
                reached_goal=True,
            )
        )

    def test_close_accepts_audited_v2_contact_residual(self):
        self.assertTrue(
            gripper_result_allows_command(
                target_position_m=0.022,
                observed_position_m=0.025692,
                open_position_m=0.040,
                closed_position_m=0.022,
                stalled=False,
                reached_goal=True,
                position_tolerance_m=0.0039,
            )
        )

    def test_gripper_command_uses_live_joint_fallback_for_empty_result_state(self):
        class Goal:
            def __init__(self):
                self.command = SimpleNamespace(name=[], position=[], effort=[])

        result = SimpleNamespace(
            state=SimpleNamespace(name=[], position=[]),
            stalled=False,
            reached_goal=True,
        )
        wrapped = SimpleNamespace(result=result)
        handle = SimpleNamespace(
            accepted=True,
            get_result_async=lambda: FakeFuture(wrapped),
        )
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend._ParallelGripperCommand = SimpleNamespace(Goal=Goal)
        client = SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=lambda goal: FakeFuture(handle),
        )
        backend._gripper = client
        backend._gripper_secondary = client
        backend.request_timeout_sec = 1.0
        backend.gripper_result_timeout_sec = 2.0
        backend.gripper_joint = "panda_finger_joint1"
        backend.gripper_secondary_joint = "panda_finger_joint2"
        backend.gripper_joints = (
            backend.gripper_joint,
            backend.gripper_secondary_joint,
        )
        backend.max_effort_n = 40.0
        backend.open_width_m = 0.04
        backend.closed_width_m = 0.022
        backend.gripper_position_tolerance_m = 0.0039
        backend._gripper_state_lock = threading.Lock()
        backend._latest_gripper_positions_m = {
            "panda_finger_joint1": 0.04,
            "panda_finger_joint2": 0.04,
        }

        self.assertTrue(backend._gripper_command(0.04))

    def test_dual_gripper_results_share_one_bounded_deadline(self):
        class Goal:
            def __init__(self):
                self.command = SimpleNamespace(name=[], position=[], effort=[])

        class PendingFuture:
            def add_done_callback(self, callback):
                del callback

        handle = SimpleNamespace(
            accepted=True,
            get_result_async=lambda: PendingFuture(),
        )
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend._ParallelGripperCommand = SimpleNamespace(Goal=Goal)
        client = SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=lambda goal: FakeFuture(handle),
        )
        backend._gripper = client
        backend._gripper_secondary = client
        backend.request_timeout_sec = 1.0
        backend.gripper_result_timeout_sec = 4.0
        backend.gripper_joint = "panda_finger_joint1"
        backend.gripper_secondary_joint = "panda_finger_joint2"
        backend.max_effort_n = 40.0
        waits = []

        def wait_future(future, timeout_sec):
            if isinstance(future, FakeFuture):
                return future.result()
            waits.append(timeout_sec)
            return None

        backend._wait_future = wait_future
        monotonic_values = iter((10.0, 11.0, 13.5))

        with patch(
            "strawberry_manipulation.moveit_backend.time.monotonic",
            side_effect=lambda: next(monotonic_values),
        ):
            self.assertFalse(backend._gripper_command(0.022))

        self.assertEqual(waits, [3.0, 0.5])

    def test_close_gripper_retries_one_no_travel_result(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.closed_width_m = 0.022
        backend.open_width_m = 0.04
        backend.settle_sample_period_sec = 0.05
        outcomes = iter((False, True, True))
        commands = []
        backend._gripper_command = lambda position: commands.append(position) or next(
            outcomes
        )

        with patch("strawberry_manipulation.moveit_backend.time.sleep") as sleep:
            self.assertTrue(backend.close_gripper())

        self.assertEqual(commands, [0.022, 0.04, 0.022])
        sleep.assert_called_once_with(0.05)

    def test_finger_asymmetry_reports_signed_local_y_centering_offset(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.gripper_joint = "panda_finger_joint1"
        backend.gripper_secondary_joint = "panda_finger_joint2"
        backend.open_width_m = 0.04
        backend.closed_width_m = 0.022
        backend.gripper_position_tolerance_m = 0.0039
        backend._gripper_state_lock = threading.Lock()
        backend._latest_gripper_positions_m = {
            "panda_finger_joint1": 0.03166,
            "panda_finger_joint2": 0.022,
        }

        self.assertAlmostEqual(backend.gripper_centering_offset_m(), 0.00483)

        backend._latest_gripper_positions_m = {
            "panda_finger_joint1": 0.022,
            "panda_finger_joint2": 0.030,
        }
        self.assertAlmostEqual(backend.gripper_centering_offset_m(), -0.004)

    def test_contact_resolved_backend_reads_anonymous_contact_class(self):
        response = SimpleNamespace(success=True, message="LEFT_SINGLE_FRUIT")
        client = SimpleNamespace(
            wait_for_service=lambda timeout_sec: True,
            call_async=lambda request: FakeFuture(response),
        )
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(
            get_logger=lambda: self.Logger(),
            create_client=lambda service_type, topic, callback_group: client,
        )
        backend.contact_resolved_attachment = True
        backend._service_clients = {}
        backend._Trigger = SimpleNamespace(Request=lambda: object())
        backend._callback_group = object()
        backend.request_timeout_sec = 1.0

        self.assertEqual(
            backend.gripper_fruit_contact_class(), "LEFT_SINGLE_FRUIT"
        )
        self.assertIn((0, "contact_class"), backend._service_clients)

    def test_contact_class_unavailable_fails_closed_for_generalized_runtime(self):
        client = SimpleNamespace(wait_for_service=lambda timeout_sec: False)
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(
            get_logger=lambda: self.Logger(),
            create_client=lambda service_type, topic, callback_group: client,
        )
        backend.contact_resolved_attachment = True
        backend._service_clients = {}
        backend._Trigger = SimpleNamespace(Request=lambda: object())
        backend._callback_group = object()
        backend.request_timeout_sec = 1.0

        self.assertEqual(
            backend.gripper_fruit_contact_class(), "CONTACT_CLASS_UNAVAILABLE"
        )

    def test_legacy_runtime_does_not_require_contact_class_service(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.contact_resolved_attachment = False

        self.assertIsNone(backend.gripper_fruit_contact_class())

    def test_gripper_joint_callback_tracks_both_commanded_joints(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.gripper_joint = "panda_finger_joint1"
        backend.gripper_secondary_joint = "panda_finger_joint2"
        backend.gripper_joints = (
            backend.gripper_joint,
            backend.gripper_secondary_joint,
        )
        backend._gripper_state_lock = threading.Lock()
        backend._latest_gripper_positions_m = {
            "panda_finger_joint1": None,
            "panda_finger_joint2": None,
        }
        backend._arm_joint_names = tuple(f"panda_joint{index}" for index in range(1, 8))
        backend._latest_arm_positions_rad = {
            joint_name: None for joint_name in backend._arm_joint_names
        }
        backend._arm_state_sequence = 0
        backend._latest_arm_snapshot = None
        backend.node = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
            )
        )

        backend._on_gripper_joint_state(
            SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(sec=1, nanosec=0), frame_id="base"
                ),
                name=[
                    "panda_joint1",
                    "panda_joint2",
                    "panda_joint3",
                    "panda_joint4",
                    "panda_joint5",
                    "panda_joint6",
                    "panda_joint7",
                    "panda_finger_joint1",
                    "panda_finger_joint2",
                ],
                position=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.031, 0.032],
                velocity=[0.0] * 9,
                effort=[],
            )
        )
        self.assertEqual(
            backend._latest_live_arm_positions(),
            (1, (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)),
        )
        backend._on_gripper_joint_state(
            SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(sec=1, nanosec=50_000_000)
                ),
                name=["panda_joint1"],
                position=[0.15],
                velocity=[0.0],
                effort=[],
            )
        )
        self.assertEqual(
            backend._latest_live_arm_positions(),
            (1, (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)),
        )

        self.assertEqual(
            backend._latest_gripper_positions_m,
            {
                "panda_finger_joint1": 0.031,
                "panda_finger_joint2": 0.032,
            },
        )

    def test_prepare_rejects_unknown_target_before_scene_update(self):
        backend = self.lifecycle_backend()
        with patch(
            "strawberry_manipulation.moveit_backend.set_target_fruit_collision"
        ) as scene_update:
            self.assertFalse(backend.prepare_pick(99, Pose(0.0, 0.0, 0.0)))
        scene_update.assert_not_called()
        self.assertIsNone(backend._prepared_target_id)

    def test_contact_resolved_attachment_ignores_tracker_identity(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.contact_resolved_attachment = True
        backend.bin_verification_timeout_sec = 15.0
        operations = []
        backend._trigger_contact_target = (
            lambda operation, **_kwargs: operations.append(operation) or True
        )

        self.assertTrue(backend.attach(934))
        self.assertTrue(backend.detach(934))
        self.assertTrue(backend.fruit_in_bin(934, 1.0))
        self.assertEqual(operations, ["attach", "detach", "verify_in_bin"])

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
        with (
            patch(
                "strawberry_manipulation.moveit_backend.apply_fruit_collision_scene"
            ) as synchronize,
            patch(
                "strawberry_manipulation.moveit_backend.set_target_fruit_collision",
                return_value="strawberry_fruit_1",
            ) as target_update,
        ):
            self.assertTrue(backend.prepare_pick(1, Pose(0.45, -0.05, 0.52)))
        synchronize.assert_called_once()
        self.assertEqual(live, dict(synchronize.call_args.args[2]))
        self.assertEqual(
            (0.45, -0.05, 0.52), target_update.call_args.kwargs["center_m"]
        )

    def test_prepare_fails_closed_when_live_truth_ids_are_incomplete(self):
        backend = self.lifecycle_backend()
        backend.fruit_pose_provider = lambda: {}
        with (
            patch(
                "strawberry_manipulation.moveit_backend.apply_fruit_collision_scene"
            ) as synchronize,
            patch(
                "strawberry_manipulation.moveit_backend.set_target_fruit_collision"
            ) as target_update,
        ):
            self.assertFalse(backend.prepare_pick(1, Pose(0.4, 0.1, 0.5)))
        synchronize.assert_not_called()
        target_update.assert_not_called()

    def test_dynamic_prepare_uses_bounded_pre_occlusion_visual_scene(self):
        backend = self.lifecycle_backend()
        backend.dynamic_fruit_manifest = True
        backend.fruit_pose_provider = lambda: {1: (0.40, 0.10, 0.50), 2: (0.5, 0.0, 0.5)}
        with (
            patch(
                "strawberry_manipulation.moveit_backend.apply_fruit_collision_scene"
            ),
            patch(
                "strawberry_manipulation.moveit_backend.set_target_fruit_collision",
                return_value="strawberry_fruit_1",
            ),
            patch(
                "strawberry_manipulation.moveit_backend.time.monotonic",
                return_value=10.0,
            ),
        ):
            self.assertTrue(backend.prepare_pick(1, Pose(0.40, 0.10, 0.50)))
            self.assertTrue(backend.restore_target_collision(1))

        backend.fruit_pose_provider = lambda: (_ for _ in ()).throw(
            RuntimeError("tracked fruit collision scene is stale")
        )
        with (
            patch(
                "strawberry_manipulation.moveit_backend.apply_fruit_collision_scene"
            ) as synchronize,
            patch(
                "strawberry_manipulation.moveit_backend.set_target_fruit_collision",
                return_value="strawberry_fruit_1",
            ),
            patch(
                "strawberry_manipulation.moveit_backend.time.monotonic",
                return_value=40.0,
            ),
        ):
            self.assertTrue(backend.prepare_pick(1, Pose(0.41, 0.10, 0.50)))

        synchronized = dict(synchronize.call_args.args[2])
        self.assertEqual(synchronized[1], (0.41, 0.10, 0.50))
        self.assertEqual(synchronized[2], (0.5, 0.0, 0.5))

    def test_pre_observation_lock_freezes_visual_inventory_for_target(self):
        backend = self.lifecycle_backend()
        backend.dynamic_fruit_manifest = True
        live = {1: (0.40, 0.10, 0.50), 2: (0.50, 0.00, 0.50)}
        backend.fruit_pose_provider = lambda: live
        with (
            patch(
                "strawberry_manipulation.moveit_backend.apply_fruit_collision_scene"
            ) as synchronize,
            patch(
                "strawberry_manipulation.moveit_backend.time.monotonic",
                return_value=10.0,
            ),
        ):
            self.assertTrue(
                backend.lock_pre_observation_collision_scene(
                    1, Pose(0.40, 0.10, 0.50)
                )
            )

        self.assertEqual(backend._locked_collision_scene_target_id, 1)
        self.assertEqual(dict(backend._cached_collision_scene_centers_m), live)
        synchronize.assert_called_once()

    def test_locked_scene_is_preferred_over_changed_live_inventory(self):
        backend = self.lifecycle_backend()
        backend.dynamic_fruit_manifest = True
        backend._cached_collision_scene_centers_m = MappingProxyType(
            {1: (0.40, 0.10, 0.50), 2: (0.50, 0.00, 0.50)}
        )
        backend._cached_collision_scene_monotonic = 10.0
        backend._locked_collision_scene_target_id = 1
        backend.fruit_pose_provider = lambda: {
            1: (0.40, 0.10, 0.50),
            9: (0.41, 0.11, 0.51),
        }
        with patch(
            "strawberry_manipulation.moveit_backend.time.monotonic",
            return_value=20.0,
        ):
            centers = backend._fruit_centers_for_target_lifecycle(
                1, Pose(0.405, 0.10, 0.50)
            )

        self.assertEqual(set(centers), {1, 2})
        self.assertEqual(centers[1], (0.405, 0.10, 0.50))

    def test_reobservation_does_not_replace_same_target_lock(self):
        backend = self.lifecycle_backend()
        backend.dynamic_fruit_manifest = True
        backend._cached_collision_scene_centers_m = MappingProxyType(
            {1: (0.40, 0.10, 0.50), 2: (0.50, 0.00, 0.50)}
        )
        backend._cached_collision_scene_monotonic = 10.0
        backend._locked_collision_scene_target_id = 1
        backend.fruit_pose_provider = lambda: {
            1: (0.40, 0.10, 0.50),
            9: (0.41, 0.11, 0.51),
        }
        with (
            patch(
                "strawberry_manipulation.moveit_backend.apply_fruit_collision_scene"
            ) as synchronize,
            patch(
                "strawberry_manipulation.moveit_backend.time.monotonic",
                return_value=20.0,
            ),
        ):
            self.assertTrue(
                backend.lock_pre_observation_collision_scene(
                    1, Pose(0.405, 0.10, 0.50)
                )
            )

        self.assertEqual(
            set(backend._cached_collision_scene_centers_m), {1, 2}
        )
        synchronize.assert_not_called()

    def test_dynamic_prepare_rejects_expired_pre_occlusion_scene(self):
        backend = self.lifecycle_backend()
        backend.dynamic_fruit_manifest = True
        backend.fruit_pose_provider = lambda: {}
        backend._cached_collision_scene_centers_m = MappingProxyType(
            {1: (0.40, 0.10, 0.50)}
        )
        backend._cached_collision_scene_monotonic = 10.0
        backend.maximum_cached_collision_scene_age_sec = 20.0
        with (
            patch(
                "strawberry_manipulation.moveit_backend.time.monotonic",
                return_value=31.0,
            ),
            patch(
                "strawberry_manipulation.moveit_backend.set_target_fruit_collision"
            ) as target_update,
        ):
            self.assertFalse(backend.prepare_pick(1, Pose(0.40, 0.10, 0.50)))
        target_update.assert_not_called()

    def test_dynamic_prepare_rejects_target_drift_from_cached_scene(self):
        backend = self.lifecycle_backend()
        backend.dynamic_fruit_manifest = True
        backend.fruit_pose_provider = lambda: {}
        backend._cached_collision_scene_centers_m = MappingProxyType(
            {1: (0.40, 0.10, 0.50)}
        )
        backend._cached_collision_scene_monotonic = 10.0
        with (
            patch(
                "strawberry_manipulation.moveit_backend.time.monotonic",
                return_value=20.0,
            ),
            patch(
                "strawberry_manipulation.moveit_backend.set_target_fruit_collision"
            ) as target_update,
        ):
            self.assertFalse(backend.prepare_pick(1, Pose(0.46, 0.10, 0.50)))
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
        self.assertEqual((0.35, -0.45, 0.45), scene_update.call_args.kwargs["center_m"])

    def test_restore_uses_prepared_pose_when_target_is_temporarily_occluded(self):
        backend = self.lifecycle_backend()
        backend.dynamic_fruit_manifest = True
        backend.fruit_pose_provider = lambda: {}
        backend._prepared_target_id = 1
        backend._target_contact_open = True
        backend._prepared_scene_centers_m = MappingProxyType({1: (0.38, -0.26, 0.55)})
        with patch(
            "strawberry_manipulation.moveit_backend.set_target_fruit_collision",
            return_value="strawberry_fruit_1",
        ) as scene_update:
            self.assertTrue(backend.restore_target_collision(1))
        self.assertEqual((0.38, -0.26, 0.55), scene_update.call_args.kwargs["center_m"])

    def test_restore_uses_prepared_pose_when_live_provider_is_stale(self):
        backend = self.lifecycle_backend()
        backend.dynamic_fruit_manifest = True
        backend.fruit_pose_provider = lambda: (_ for _ in ()).throw(
            RuntimeError("tracked fruit collision scene is stale")
        )
        backend._prepared_target_id = 1
        backend._target_contact_open = True
        backend._prepared_scene_centers_m = MappingProxyType({1: (0.38, -0.26, 0.55)})
        with patch(
            "strawberry_manipulation.moveit_backend.set_target_fruit_collision",
            return_value="strawberry_fruit_1",
        ) as scene_update:
            self.assertTrue(backend.restore_target_collision(1))
        self.assertEqual((0.38, -0.26, 0.55), scene_update.call_args.kwargs["center_m"])

    def test_guarded_place_aligns_above_bin_before_vertical_descent(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.safe_transit_clearance_m = 0.02
        backend.safe_transit_corridor_y_m = -0.10
        backend.place_transit_clearance_m = 0.13
        state = [Pose(0.42, -0.12, 0.70, qx=1.0, qw=0.0)]
        segments = []

        backend._current_link_pose = lambda: state[-1]
        backend.preview_guarded_place_from_current = lambda _target: PathAssessment(
            True, False, 0.07, 0.3, (0.1, 0.2)
        )

        def move_segment(start, target, *, intermediate_endpoint=False):
            self.assertEqual(start, state[-1])
            segments.append((target, intermediate_endpoint))
            state.append(target)
            return MotionOutcome(True, 0.01, 0.02)

        backend._move_segmented_between = move_segment
        target = Pose(0.35, -0.45, 0.5554, qx=1.0, qw=0.0)
        outcome = backend._move_guarded_place(target)

        self.assertTrue(outcome.success)
        self.assertEqual(len(segments), 7)
        self.assertAlmostEqual(segments[0][0].z, 0.83)
        self.assertEqual(
            (segments[1][0].x, segments[1][0].y, segments[1][0].z),
            (0.42, -0.10, 0.83),
        )
        self.assertEqual(
            (segments[4][0].x, segments[4][0].y, segments[4][0].z),
            (0.35, -0.10, 0.6854),
        )
        self.assertEqual(
            (segments[5][0].x, segments[5][0].y, segments[5][0].z),
            (0.35, -0.45, 0.6854),
        )
        self.assertEqual(segments[-1], (target, False))
        self.assertTrue(all(intermediate for _, intermediate in segments[:-1]))

    def test_guarded_place_preview_failure_is_zero_motion(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend._current_link_pose = lambda: Pose(0.20, 0.0, 0.70)
        backend.preview_guarded_place_from_current = lambda _target: PathAssessment(
            False, True, 0.04, 0.2, (0.1, 0.2)
        )
        segments = []
        backend._move_segmented_between = lambda *args, **kwargs: segments.append(
            (args, kwargs)
        )

        outcome = backend._move_guarded_place(
            Pose(0.35, -0.45, 0.5554, qx=1.0, qw=0.0)
        )

        self.assertFalse(outcome.success)
        self.assertTrue(outcome.collision)
        self.assertEqual(outcome.execution_time_sec, 0.0)
        self.assertEqual(segments, [])

    def test_guarded_place_uses_first_fully_feasible_wrist_roll(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.safe_transit_corridor_y_m = -0.10
        backend.place_transit_clearance_m = 0.12
        state = [Pose(0.42, -0.12, 0.70, qx=1.0, qw=0.0)]
        previews = []
        segments = []
        backend._current_link_pose = lambda: state[-1]

        def preview(candidate):
            previews.append(candidate)
            return PathAssessment(
                len(previews) == 2,
                False,
                0.01,
                0.2,
                (0.1, 0.2),
            )

        def move_segment(start, target, *, intermediate_endpoint=False):
            self.assertEqual(start, state[-1])
            segments.append((target, intermediate_endpoint))
            state.append(target)
            return MotionOutcome(True, 0.01, 0.02)

        backend.preview_guarded_place_from_current = preview
        backend._move_segmented_between = move_segment
        target = Pose(0.35, -0.45, 0.5554, qx=1.0, qw=0.0)

        outcome = backend._move_guarded_place(target)

        self.assertTrue(outcome.success)
        self.assertEqual(len(previews), 2)
        self.assertAlmostEqual(previews[1].qx, math.sqrt(0.5))
        self.assertAlmostEqual(previews[1].qy, math.sqrt(0.5))
        self.assertAlmostEqual(segments[-1][0].qx, math.sqrt(0.5))
        self.assertAlmostEqual(segments[-1][0].qy, math.sqrt(0.5))
        self.assertEqual(segments[-1][1], False)

    def test_guarded_approach_preview_failure_is_zero_motion(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend._current_link_pose = lambda: Pose(0.20, 0.0, 0.70)
        backend._current_joint_positions = lambda: (0.0, 0.1)
        backend.preview_guarded_approach = lambda *args: PathAssessment(
            False, True, 0.03, 0.2, (0.1, 0.2)
        )
        segments = []
        backend._move_segmented_between = lambda *args, **kwargs: segments.append(
            (args, kwargs)
        )

        outcome = backend._move_guarded_approach(Pose(0.42, 0.0, 0.62, qx=1.0, qw=0.0))

        self.assertFalse(outcome.success)
        self.assertTrue(outcome.collision)
        self.assertEqual(outcome.execution_time_sec, 0.0)
        self.assertEqual(segments, [])

    def test_retreat_uses_checked_cartesian_segments(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        expected = MotionOutcome(True, 0.1, 0.2)
        requested = []
        backend._move_to_segmented = lambda pose: requested.append(pose) or expected

        target = Pose(0.42, -0.12, 0.7054, qx=1.0, qw=0.0)
        self.assertIs(backend.move_to(target, "RETREAT"), expected)
        self.assertEqual(requested, [target])

    def test_grasp_retry_uses_checked_cartesian_segments(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.grasp_joint_trajectory_velocity_rad_per_sec = 0.25
        expected = MotionOutcome(True, 0.1, 0.2)
        requested = []
        backend._move_to_segmented = (
            lambda pose, **kwargs: requested.append((pose, kwargs)) or expected
        )

        preparation = Pose(0.42, -0.12, 0.7054, qy=1.0, qw=0.0)
        grasp = Pose(0.42, -0.12, 0.5554, qy=1.0, qw=0.0)
        self.assertIs(backend.move_to(preparation, "GRASP_RETRY_PREP"), expected)
        self.assertIs(backend.move_to(grasp, "GRASP_POSE_RETRY"), expected)
        self.assertEqual(
            requested,
            [
                (preparation, {"velocity_rad_per_sec": 0.25}),
                (grasp, {"velocity_rad_per_sec": 0.25}),
            ],
        )

    def test_grasp_segment_forwards_contact_bound_velocity(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.cartesian_endpoint_retry_limit = 0
        backend.intermediate_position_tolerance_m = 0.02
        backend.intermediate_orientation_tolerance_rad = math.radians(8.0)
        backend.pose_link = "panda_hand"
        backend._dense_pose_waypoints = lambda current, target: (target,)
        backend._solve_cartesian_joint_path = lambda waypoints: (
            (0.0,),
            ((0.1,),),
            False,
            0.01,
        )
        executions = []
        backend._execute_joint_path = (
            lambda initial, path, **kwargs: executions.append(kwargs)
            or (True, 0.02)
        )
        backend._wait_until_arm_settled = lambda: True
        backend._pose_is_within_tolerance = lambda *args, **kwargs: True
        observed = SimpleNamespace()

        class ReadOnly:
            def __enter__(self):
                return SimpleNamespace(
                    current_state=SimpleNamespace(get_pose=lambda link: observed)
                )

            def __exit__(self, exc_type, exc_value, traceback):
                return False

        backend._planning_scene_monitor = SimpleNamespace(read_only=lambda: ReadOnly())
        outcome = backend._move_segmented_between(
            Pose(0.0, 0.0, 0.0),
            Pose(0.1, 0.0, 0.0),
            velocity_rad_per_sec=0.25,
        )

        self.assertTrue(outcome.success)
        self.assertEqual(executions, [{"velocity_rad_per_sec": 0.25}])

    def test_wrist_observation_uses_startup_verified_action_path(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        expected = MotionOutcome(True, 0.1, 0.2)
        requested = []
        backend._move_to_planned_joint_path = (
            lambda pose: requested.append(pose) or expected
        )

        target = Pose(0.28, 0.0, 0.72, qy=0.95, qw=0.31)
        self.assertIs(backend.move_to(target, "WRIST_OBSERVATION"), expected)
        self.assertEqual(requested, [target])

    def test_observation_is_not_executed_when_approach_preview_fails(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        logger = self.Logger()
        backend.node = SimpleNamespace(get_logger=lambda: logger)
        observation_pose = Pose(0.28, 0.0, 0.72, qy=0.95, qw=0.31)
        pregrasp_pose = Pose(0.42, 0.0, 0.62, qx=1.0, qw=0.0)
        positions = ((0.0, 0.1), (0.2, 0.3))
        backend._plan_joint_path_to_pose = lambda pose: (
            positions,
            PathAssessment(
                True,
                False,
                0.11,
                0.4,
                positions[-1],
                observation_pose,
            ),
        )
        backend.preview_guarded_approach = lambda *args: PathAssessment(
            False, True, 0.07, 0.2, positions[-1]
        )
        executions = []
        backend._execute_joint_path = lambda *args: executions.append(args)

        outcome = backend.move_to_observation_if_approach_feasible(
            observation_pose, pregrasp_pose
        )

        self.assertFalse(outcome.success)
        self.assertTrue(outcome.collision)
        self.assertAlmostEqual(outcome.planning_time_sec, 0.18)
        self.assertEqual(outcome.execution_time_sec, 0.0)
        self.assertEqual(executions, [])

    def test_cartesian_preview_seeds_each_segment_from_prior_endpoint(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        start = Pose(0.20, 0.0, 0.70)
        middle = Pose(0.25, 0.0, 0.72)
        target = Pose(0.42, 0.0, 0.62)
        starts = []
        endpoint_positions = iter(((0.3, 0.4), (0.5, 0.6)))
        backend._dense_pose_waypoints = lambda left, right: (right,)

        def solve(_waypoints, *, start_joint_positions):
            starts.append(start_joint_positions)
            endpoint = next(endpoint_positions)
            return start_joint_positions, (endpoint,), False, 0.02

        backend._solve_cartesian_joint_path = solve

        result = backend.preview_cartesian_segments(start, (0.1, 0.2), (middle, target))

        self.assertTrue(result.feasible)
        self.assertEqual(starts, [(0.1, 0.2), (0.3, 0.4)])
        self.assertEqual(result.end_joint_positions, (0.5, 0.6))
        self.assertAlmostEqual(result.planning_time_sec, 0.04)
        self.assertAlmostEqual(result.joint_travel_rad, 0.8)

    def test_observation_executes_only_after_connected_preview_passes(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.pose_link = "panda_hand"
        observation_pose = Pose(0.28, 0.0, 0.72, qy=1.0, qw=0.0)
        pregrasp_pose = Pose(0.42, 0.0, 0.62, qx=1.0, qw=0.0)
        positions = ((0.0, 0.1), (0.2, 0.3))
        endpoint_message = SimpleNamespace(
            position=SimpleNamespace(x=0.28, y=0.0, z=0.72),
            orientation=SimpleNamespace(x=0.0, y=1.0, z=0.0, w=0.0),
        )
        backend._plan_joint_path_to_pose = lambda pose: (
            positions,
            PathAssessment(
                True,
                False,
                0.11,
                0.4,
                positions[-1],
                observation_pose,
            ),
        )
        backend.preview_guarded_approach = lambda *args: PathAssessment(
            True, False, 0.07, 0.2, (0.4, 0.5)
        )
        backend._joint_path_within_limit_margin = lambda *args, **kwargs: True
        backend._execute_joint_path = lambda start, path: (True, 0.3)
        backend._wait_until_arm_settled = lambda: True

        class SceneContext:
            current_state = SimpleNamespace(get_pose=lambda _link: endpoint_message)

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

        backend._planning_scene_monitor = SimpleNamespace(
            read_only=lambda: SceneContext()
        )
        backend._pose_is_within_tolerance = lambda *args, **kwargs: True

        outcome = backend.move_to_observation_if_approach_feasible(
            observation_pose, pregrasp_pose
        )

        self.assertTrue(outcome.success)
        self.assertAlmostEqual(outcome.planning_time_sec, 0.18)
        self.assertAlmostEqual(outcome.execution_time_sec, 0.3)

    def test_observation_rejects_path_without_larger_limit_margin(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        logger = self.Logger()
        backend.node = SimpleNamespace(get_logger=lambda: logger)
        backend.observation_joint_limit_margin_rad = 0.02
        pose = Pose(0.28, 0.0, 0.72, qy=1.0, qw=0.0)
        positions = ((0.0,) * 7, (0.0, -1.75, 0.0, -1.5, 0.0, 1.5, 0.0))
        backend._plan_joint_path_to_pose = lambda _pose: (
            positions,
            PathAssessment(True, False, 0.1, 0.2, positions[-1], pose),
        )
        backend._joint_path_within_limit_margin = lambda *args, **kwargs: False
        backend.preview_guarded_approach = lambda *args: self.fail(
            "unsafe observation path reached connected preview"
        )

        outcome = backend.move_to_observation_if_approach_feasible(pose, pose)

        self.assertFalse(outcome.success)
        self.assertIn("observation-specific limit margin", logger.warnings[-1])

    def test_home_uses_bounded_direct_action_path(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.home_configuration = "ready"
        requested = []
        backend._move_to_named_configuration_direct = (
            lambda configuration: requested.append(configuration)
            or MotionOutcome(True, 0.1, 0.2)
        )

        self.assertTrue(backend.move_home())
        self.assertEqual(requested, ["ready"])

    def test_home_replans_once_after_zero_motion_failure(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.home_configuration = "ready"
        outcomes = iter(
            (
                MotionOutcome(False, 0.1, 0.0),
                MotionOutcome(True, 0.1, 1.0),
            )
        )
        requested = []
        backend._move_to_named_configuration_direct = (
            lambda configuration: requested.append(configuration) or next(outcomes)
        )

        self.assertTrue(backend.move_home())
        self.assertEqual(requested, ["ready", "ready"])

    def test_home_does_not_retry_after_controller_execution_started(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.home_configuration = "ready"
        requested = []
        backend._move_to_named_configuration_direct = (
            lambda configuration: requested.append(configuration)
            or MotionOutcome(False, 0.1, 0.01)
        )

        self.assertFalse(backend.move_home())
        self.assertEqual(requested, ["ready"])

    def test_named_home_uses_reduced_tracking_velocity(self):
        class Arm:
            def set_start_state_to_current_state(self):
                pass

            def set_goal_state(self, **kwargs):
                pass

            def plan(self):
                states = [
                    SimpleNamespace(
                        get_joint_group_positions=lambda _group, value=value: (value,)
                    )
                    for value in (0.0, 0.5)
                ]
                return SimpleNamespace(trajectory=states)

        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend._arm = Arm()
        backend.planning_group = "panda_arm"
        backend._arm_joint_names = ("panda_joint1",)
        backend.home_joint_trajectory_velocity_rad_per_sec = 0.10
        backend.home_joint_trajectory_segment_duration_sec = 6.0
        backend.home_joint_tolerance_rad = 0.03
        backend.maximum_joint_trajectory_points = 512
        backend.maximum_joint_trajectory_travel_rad = 40.0
        backend.maximum_joint_trajectory_duration_sec = 60.0
        backend.minimum_joint_limit_margin_rad = 0.01
        backend.minimum_joint_waypoint_duration_sec = 0.05
        backend._wait_until_arm_settled = lambda: True
        backend._current_joint_positions = lambda: (0.5,)
        calls = []
        backend._execute_joint_path = lambda start, path, **kwargs: (
            calls.append((start, path, kwargs)) or (True, 1.0)
        )

        outcome = backend._move_to_named_configuration_direct("ready")

        self.assertTrue(outcome.success)
        self.assertEqual(
            calls[0][2]["velocity_rad_per_sec"],
            0.10,
        )

    def test_named_home_partitions_long_plan_and_verifies_each_endpoint(self):
        class Arm:
            def set_start_state_to_current_state(self):
                pass

            def set_goal_state(self, **kwargs):
                pass

            def plan(self):
                states = [
                    SimpleNamespace(
                        get_joint_group_positions=lambda _group, value=value: (value,)
                    )
                    for value in (0.0, 0.2, 0.4, 0.6)
                ]
                return SimpleNamespace(trajectory=states)

        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend._arm = Arm()
        backend.planning_group = "panda_arm"
        backend._arm_joint_names = ("panda_joint1",)
        backend.home_joint_trajectory_velocity_rad_per_sec = 0.10
        backend.home_joint_trajectory_segment_duration_sec = 3.0
        backend.home_joint_tolerance_rad = 0.03
        backend.maximum_joint_trajectory_points = 512
        backend.maximum_joint_trajectory_travel_rad = 40.0
        backend.maximum_joint_trajectory_duration_sec = 60.0
        backend.minimum_joint_limit_margin_rad = 0.01
        backend.minimum_joint_waypoint_duration_sec = 0.05
        backend._wait_until_arm_settled = lambda: True
        measured = [0.0]
        calls = []

        def execute(start, path, **kwargs):
            measured[0] = path[-1][0]
            calls.append((start, path, kwargs))
            return True, 0.5

        backend._execute_joint_path = execute
        backend._current_joint_positions = lambda: (measured[0],)

        outcome = backend._move_to_named_configuration_direct("ready")

        self.assertTrue(outcome.success)
        self.assertEqual(len(calls), 3)
        self.assertEqual([call[1][-1] for call in calls], [(0.2,), (0.4,), (0.6,)])
        self.assertAlmostEqual(outcome.execution_time_sec, 1.5)

    def test_named_home_stops_before_next_segment_on_endpoint_mismatch(self):
        class Arm:
            def set_start_state_to_current_state(self):
                pass

            def set_goal_state(self, **kwargs):
                pass

            def plan(self):
                states = [
                    SimpleNamespace(
                        get_joint_group_positions=lambda _group, value=value: (value,)
                    )
                    for value in (0.0, 0.2, 0.4)
                ]
                return SimpleNamespace(trajectory=states)

        backend = MoveItBackend.__new__(MoveItBackend)
        logger = self.Logger()
        backend.node = SimpleNamespace(get_logger=lambda: logger)
        backend._arm = Arm()
        backend.planning_group = "panda_arm"
        backend._arm_joint_names = ("panda_joint1",)
        backend.home_joint_trajectory_velocity_rad_per_sec = 0.10
        backend.home_joint_trajectory_segment_duration_sec = 3.0
        backend.home_joint_tolerance_rad = 0.03
        backend.maximum_joint_trajectory_points = 512
        backend.maximum_joint_trajectory_travel_rad = 40.0
        backend.maximum_joint_trajectory_duration_sec = 60.0
        backend.minimum_joint_limit_margin_rad = 0.01
        backend.minimum_joint_waypoint_duration_sec = 0.05
        backend._wait_until_arm_settled = lambda: True
        calls = []
        backend._execute_joint_path = lambda start, path, **kwargs: (
            calls.append((start, path, kwargs)) or (True, 0.5)
        )
        backend._current_joint_positions = lambda: (0.0,)

        outcome = backend._move_to_named_configuration_direct("ready")

        self.assertFalse(outcome.success)
        self.assertEqual(len(calls), 1)
        self.assertIn("did not reach its verified joint endpoint", logger.errors[-1])

    def test_stationary_home_plan_succeeds_only_when_live_joints_confirm_home(self):
        class Arm:
            def set_start_state_to_current_state(self):
                pass

            def set_goal_state(self, **kwargs):
                pass

            def plan(self):
                return SimpleNamespace(trajectory=())

        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend._arm = Arm()
        backend.home_joint_positions_rad = (
            0.0,
            -0.785,
            0.0,
            -2.356,
            0.0,
            1.571,
            0.785,
        )
        backend.home_joint_tolerance_rad = 0.03
        backend._wait_until_arm_settled = lambda: True
        backend._current_joint_positions = lambda: (
            0.001,
            -0.786,
            0.0,
            -2.355,
            0.0,
            1.570,
            0.786,
        )

        outcome = backend._move_to_named_configuration_direct("ready")

        self.assertTrue(outcome.success)
        self.assertEqual(outcome.execution_time_sec, 0.0)

    def test_stationary_home_plan_fails_when_live_joints_are_not_home(self):
        class Arm:
            def set_start_state_to_current_state(self):
                pass

            def set_goal_state(self, **kwargs):
                pass

            def plan(self):
                return SimpleNamespace(trajectory=())

        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend._arm = Arm()
        backend.home_joint_positions_rad = (0.0,) * 7
        backend.home_joint_tolerance_rad = 0.03
        backend._wait_until_arm_settled = lambda: True
        backend._current_joint_positions = lambda: (0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0)

        outcome = backend._move_to_named_configuration_direct("ready")

        self.assertFalse(outcome.success)

    def test_direct_joint_path_waits_for_action_server_before_goal(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        logger = self.Logger()
        backend.node = SimpleNamespace(get_logger=lambda: logger)
        backend.request_timeout_sec = 5.0
        backend.maximum_joint_trajectory_points = 512
        backend.maximum_joint_trajectory_travel_rad = 40.0
        backend.maximum_joint_trajectory_duration_sec = 60.0
        backend.joint_trajectory_velocity_rad_per_sec = 0.30
        backend.home_joint_trajectory_velocity_rad_per_sec = 0.10
        backend.minimum_joint_waypoint_duration_sec = 0.05
        backend._arm_action_probe = SimpleNamespace(
            wait_for_server=lambda timeout_sec: False
        )

        succeeded, execution_time = backend._execute_joint_path((0.0,), ((0.1,),))

        self.assertFalse(succeeded)
        self.assertEqual(execution_time, 0.0)
        self.assertIn(
            "Panda arm trajectory action server became unavailable",
            logger.errors,
        )

    def test_direct_joint_path_rejects_stale_planned_start_before_goal(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        logger = self.Logger()
        backend.node = SimpleNamespace(get_logger=lambda: logger)
        backend._arm_joint_names = ("panda_joint1",)
        backend._gripper_state_lock = threading.Lock()
        backend._latest_arm_snapshot = ArmStateSnapshot(
            sequence=1,
            positions=(0.20,),
            feedback_json='{"acquisition_stamp_ns":1}',
            receipt_monotonic_ns=1,
        )
        backend.joint_trajectory_start_tolerance_rad = 0.05
        backend._arm_action_probe = SimpleNamespace(
            wait_for_server=lambda timeout_sec: self.fail(
                "stale planned start reached the controller"
            )
        )

        succeeded, execution_time = backend._execute_joint_path(
            (0.0,), ((0.1,),)
        )

        self.assertFalse(succeeded)
        self.assertEqual(execution_time, 0.0)
        self.assertIn("planned start differs from fresh live joints", logger.errors[-1])

    def test_settle_gate_rejects_slow_drift_across_complete_window(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        logger = self.Logger()
        backend.node = SimpleNamespace(get_logger=lambda: logger)
        backend.settle_timeout_sec = 0.8
        backend.settle_window_sec = 0.4
        backend.settle_sample_period_sec = 0.05
        backend.settle_delta_rad = 0.002
        backend.settle_stable_samples = 3
        backend._arm_joint_names = ("panda_joint1",)
        samples = iter(
            (index, (0.001 * index,))
            for index in range(1, 30)
        )
        backend._gripper_state_lock = threading.Lock()

        def next_sample():
            sequence, positions = next(samples)
            stamp_ns = sequence * 100_000_000
            return ArmStateSnapshot(
                sequence=sequence,
                positions=positions,
                feedback_json=(
                    '{"acquisition_stamp_ns":' + str(stamp_ns) + '}'
                ),
                receipt_monotonic_ns=int(__import__("time").monotonic_ns()),
            )

        backend._latest_live_arm_snapshot = next_sample
        clock = iter(index * 0.05 for index in range(100))

        with patch(
            "strawberry_manipulation.moveit_backend.time.monotonic",
            side_effect=lambda: next(clock),
        ), patch("strawberry_manipulation.moveit_backend.time.sleep"):
            self.assertFalse(backend._wait_until_arm_settled())

        self.assertIn("did not settle", logger.errors[-1])

    def test_settle_gate_accepts_fresh_samples_stable_for_complete_window(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = SimpleNamespace(get_logger=lambda: self.Logger())
        backend.node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=backend._latest_complete_stamp_ns)
        )
        backend.settle_timeout_sec = 1.0
        backend.settle_window_sec = 0.4
        backend.settle_sample_period_sec = 0.05
        backend.settle_delta_rad = 0.002
        backend.settle_stable_samples = 3
        backend._arm_joint_names = ("panda_joint1",)
        samples = iter(
            (index, (0.5 + (0.0001 if index % 2 else 0.0),))
            for index in range(1, 30)
        )
        backend._gripper_state_lock = threading.Lock()

        def next_sample():
            sequence, positions = next(samples)
            stamp_ns = sequence * 100_000_000
            backend._latest_complete_stamp_ns = stamp_ns
            return ArmStateSnapshot(
                sequence=sequence,
                positions=positions,
                feedback_json=(
                    '{"acquisition_stamp_ns":' + str(stamp_ns) + '}'
                ),
                receipt_monotonic_ns=int(__import__("time").monotonic_ns()),
            )

        backend._latest_live_arm_snapshot = next_sample
        clock = iter(index * 0.05 for index in range(100))

        with patch(
            "strawberry_manipulation.moveit_backend.time.monotonic",
            side_effect=lambda: next(clock),
        ), patch("strawberry_manipulation.moveit_backend.time.sleep"):
            self.assertTrue(backend._wait_until_arm_settled())

    def test_joint_path_rejects_excessive_nominal_duration_before_goal(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        logger = self.Logger()
        backend.node = SimpleNamespace(get_logger=lambda: logger)
        backend.maximum_joint_trajectory_points = 512
        backend.maximum_joint_trajectory_travel_rad = 40.0
        backend.maximum_joint_trajectory_duration_sec = 1.0
        backend.joint_trajectory_velocity_rad_per_sec = 0.30
        backend.minimum_joint_waypoint_duration_sec = 0.05
        backend.request_timeout_sec = 5.0
        backend._arm_action_probe = SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=lambda goal: self.fail(
                "unsafe trajectory reached the controller"
            ),
        )

        succeeded, execution_time = backend._execute_joint_path(
            (0.0,), ((0.3,), (0.6,), (0.9,), (1.2,), (1.5,))
        )

        self.assertFalse(succeeded)
        self.assertEqual(execution_time, 0.0)
        self.assertTrue(any("duration" in message for message in logger.errors))

    def test_joint_path_rejects_excessive_cumulative_travel_before_goal(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        logger = self.Logger()
        backend.node = SimpleNamespace(get_logger=lambda: logger)
        backend.maximum_joint_trajectory_points = 512
        backend.maximum_joint_trajectory_travel_rad = 1.0
        backend.maximum_joint_trajectory_duration_sec = 60.0
        backend.joint_trajectory_velocity_rad_per_sec = 0.30
        backend.minimum_joint_waypoint_duration_sec = 0.05
        backend.request_timeout_sec = 5.0
        backend._arm_action_probe = SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=lambda goal: self.fail(
                "unsafe trajectory reached the controller"
            ),
        )

        succeeded, execution_time = backend._execute_joint_path(
            (0.0,), ((0.6,), (0.0,), (0.6,))
        )

        self.assertFalse(succeeded)
        self.assertEqual(execution_time, 0.0)
        self.assertTrue(any("travel" in message for message in logger.errors))

    def test_dense_joint_path_timing_keeps_velocity_bound_without_long_dwell(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.joint_trajectory_velocity_rad_per_sec = 0.30
        backend.minimum_joint_waypoint_duration_sec = 0.05

        duration = backend._joint_path_nominal_duration(
            ((0.0,), (0.003,), (0.006,), (0.009,))
        )
        velocity_limited_duration = backend._joint_path_nominal_duration(
            ((0.0,), (0.06,))
        )

        self.assertAlmostEqual(duration, 0.15)
        self.assertAlmostEqual(velocity_limited_duration, 0.20)

    def test_joint_path_edges_above_controller_tolerance_are_subdivided(self):
        """v9 evidence: a 0.05+ rad single-edge step aborted the controller.

        The align-above-bin segment near the bin-edge singularity produced a
        joint5 edge larger than the 0.05 rad path tolerance; the simulation
        lagged one sample and the goal aborted with code -4. Edges must be
        subdivided so no single command step can reach the tolerance bound.
        """
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.max_joint_edge_step_rad = 0.02

        subdivided = backend._subdivide_joint_path_by_step(
            ((0.0,), (0.045,), (0.047,))
        )

        self.assertEqual(subdivided[0], (0.0,))
        self.assertEqual(subdivided[-1], (0.047,))
        for left, right in zip(subdivided, subdivided[1:]):
            maximum_step = max(
                abs(a - b) for a, b in zip(right, left)
            )
            self.assertLessEqual(
                maximum_step, 0.02 + 1e-12,
                "subdivided path still contains an edge above the cap",
            )

    def test_joint_path_edges_within_tolerance_are_unchanged(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.max_joint_edge_step_rad = 0.02

        original = ((0.0, 1.0), (0.01, 0.99))
        self.assertEqual(
            backend._subdivide_joint_path_by_step(original), original
        )

    def test_joint_edge_step_cap_is_validated(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        with self.assertRaises(ValueError):
            backend._subdivide_joint_path_by_step(((0.0,), (0.01,)), None)
        backend.max_joint_edge_step_rad = 0.0
        with self.assertRaises(ValueError):
            backend._subdivide_joint_path_by_step(((0.0,), (0.01,)))

    def test_joint_path_timing_honors_each_joint_velocity_limit(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.minimum_joint_waypoint_duration_sec = 0.05

        duration = backend._joint_path_nominal_duration(
            ((0.0, 0.0), (0.08, 0.04)),
            joint_velocity_limits_rad_per_sec=(0.08, 0.02),
        )

        self.assertAlmostEqual(duration, 2.0)
        with self.assertRaisesRegex(ValueError, "scalar or per-joint"):
            backend._joint_path_nominal_duration(
                ((0.0, 0.0), (0.08, 0.04)),
                velocity_rad_per_sec=0.08,
                joint_velocity_limits_rad_per_sec=(0.08, 0.02),
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
        backend._dense_pose_waypoints = lambda current, requested: dense_starts.append(
            current
        ) or (requested,)
        backend._solve_cartesian_joint_path = lambda waypoints: (
            (0.0,),
            ((0.1,),),
            False,
            0.01,
        )
        executions = []
        backend._execute_joint_path = lambda initial, path: executions.append(
            (initial, path)
        ) or (True, 0.02)
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

        backend._planning_scene_monitor = SimpleNamespace(read_only=lambda: ReadOnly())
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

        backend._planning_scene_monitor = SimpleNamespace(read_only=lambda: ReadOnly())
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
        self.assertAlmostEqual(midpoint.qz, -(2**-0.5))
        self.assertAlmostEqual(midpoint.qw, 2**-0.5)
        self.assertAlmostEqual(
            MoveItBackend._orientation_distance(start, target), math.pi
        )


if __name__ == "__main__":
    unittest.main()
