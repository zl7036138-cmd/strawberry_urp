import pathlib
import sys
import unittest
from types import SimpleNamespace

import numpy as np


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_localization.core import (  # noqa: E402
    CameraIntrinsics,
    LocalizationError,
    SensorFrameCache,
)
from strawberry_localization.node import (  # noqa: E402
    joint_samples_are_stationary,
    localization_retry_is_fresh,
    select_ripe_detection,
    select_sensor_frames,
    shutdown_executor_and_wait,
    validate_sensor_qos_depth,
    validate_selection_roi,
)
from strawberry_localization.generalized_node import (  # noqa: E402
    newest_pending_detection,
    select_candidate_detections,
    select_stable_ripe_track,
    split_stamp_seconds,
)


class LocalizationNodeInputSelectionTests(unittest.TestCase):
    def test_track_last_seen_stamp_conversion_is_normalized(self):
        self.assertEqual(split_stamp_seconds(12.25), (12, 250_000_000))
        self.assertEqual(split_stamp_seconds(1.9999999996), (2, 0))
        with self.assertRaises(ValueError):
            split_stamp_seconds(-0.1)

    @staticmethod
    def detection(
        target_id: int,
        confidence: float,
        center_xy: tuple[int, int],
        maturity: int = 1,
    ):
        x, y = center_xy
        return SimpleNamespace(
            target_id=target_id,
            confidence=confidence,
            maturity=maturity,
            RIPE=1,
            UNRIPE=2,
            bbox=SimpleNamespace(
                x_offset=x - 5,
                y_offset=y - 5,
                width=10,
                height=10,
            ),
        )

    @staticmethod
    def populated_cache() -> SensorFrameCache:
        cache = SensorFrameCache(capacity=8, retention_sec=2.0)
        intrinsics = CameraIntrinsics(320.0, 320.0, 2.0, 2.0)
        for stamp, value in ((10.0, 1.0), (10.1, 2.0)):
            cache.add_depth(
                stamp_s=stamp,
                frame_id="camera_optical",
                image=np.full((4, 4), value, dtype=np.float32),
            )
            cache.add_camera_info(
                stamp_s=stamp,
                frame_id="camera_optical",
                width=4,
                height=4,
                intrinsics=intrinsics,
                payload=f"camera info {stamp}",
            )
        return cache

    def test_delayed_detection_uses_its_old_sensor_frame(self) -> None:
        matched = select_sensor_frames(
            self.populated_cache(),
            detection_stamp_s=10.0,
            detection_frame="camera_optical",
            now_stamp_s=10.2,
            sync_tolerance_sec=0.02,
            stale_after_sec=0.5,
        )

        self.assertAlmostEqual(float(matched.depth.image[0, 0]), 1.0)
        self.assertEqual(matched.camera_info.payload, "camera info 10.0")

    def test_selection_prunes_frames_outside_cache_horizon(self) -> None:
        cache = self.populated_cache()

        with self.assertRaises(LocalizationError):
            select_sensor_frames(
                cache,
                detection_stamp_s=10.0,
                detection_frame="camera_optical",
                now_stamp_s=12.2,
                sync_tolerance_sec=0.02,
                stale_after_sec=3.0,
            )

        self.assertEqual(cache.depth_count, 0)
        self.assertEqual(cache.camera_info_count, 0)

    def test_deferred_retry_stays_inside_existing_freshness_gate(self) -> None:
        common = {
            "detection_stamp_s": 10.0,
            "stale_after_sec": 0.5,
            "sync_tolerance_sec": 0.05,
        }
        self.assertTrue(
            localization_retry_is_fresh(now_stamp_s=10.4, **common)
        )
        self.assertFalse(
            localization_retry_is_fresh(now_stamp_s=10.51, **common)
        )
        self.assertTrue(
            localization_retry_is_fresh(now_stamp_s=9.96, **common)
        )
        self.assertFalse(
            localization_retry_is_fresh(now_stamp_s=9.94, **common)
        )

    def test_sensor_qos_depth_is_positive_and_bounded(self) -> None:
        self.assertEqual(5, validate_sensor_qos_depth(5))
        self.assertEqual(30, validate_sensor_qos_depth(30))
        for value in (True, 0, -1, 121):
            with self.assertRaises(ValueError):
                validate_sensor_qos_depth(value)

    def test_stationary_sync_bound_spans_one_15hz_depth_period(self) -> None:
        cache = SensorFrameCache(capacity=4, retention_sec=2.0)
        intrinsics = CameraIntrinsics(320.0, 320.0, 2.0, 2.0)
        cache.add_depth(
            stamp_s=10.0,
            frame_id="camera_optical",
            image=np.ones((4, 4), dtype=np.float32),
        )
        cache.add_camera_info(
            stamp_s=10.065,
            frame_id="camera_optical",
            width=4,
            height=4,
            intrinsics=intrinsics,
        )
        arguments = {
            "cache": cache,
            "detection_stamp_s": 10.065,
            "detection_frame": "camera_optical",
            "now_stamp_s": 10.1,
            "stale_after_sec": 0.5,
        }

        with self.assertRaises(LocalizationError):
            select_sensor_frames(sync_tolerance_sec=0.05, **arguments)
        matched = select_sensor_frames(
            sync_tolerance_sec=0.075,
            **arguments,
        )

        self.assertAlmostEqual(matched.depth.stamp_s, 10.0)
        self.assertAlmostEqual(matched.camera_info.stamp_s, 10.065)

    def test_stationary_sync_bound_covers_observed_99ms_bridge_hole(self) -> None:
        cache = SensorFrameCache(capacity=4, retention_sec=2.0)
        intrinsics = CameraIntrinsics(320.0, 320.0, 2.0, 2.0)
        cache.add_depth(
            stamp_s=10.0,
            frame_id="camera_optical",
            image=np.ones((4, 4), dtype=np.float32),
        )
        cache.add_camera_info(
            stamp_s=10.099,
            frame_id="camera_optical",
            width=4,
            height=4,
            intrinsics=intrinsics,
        )
        arguments = {
            "cache": cache,
            "detection_stamp_s": 10.099,
            "detection_frame": "camera_optical",
            "now_stamp_s": 10.15,
            "stale_after_sec": 0.5,
        }

        with self.assertRaises(LocalizationError):
            select_sensor_frames(sync_tolerance_sec=0.075, **arguments)
        matched = select_sensor_frames(
            sync_tolerance_sec=0.105,
            **arguments,
        )

        self.assertAlmostEqual(matched.depth.stamp_s, 10.0)

    def test_stationary_joint_window_is_bounded_and_requires_all_samples(self):
        stable = [
            (0.1, -0.2),
            (0.1005, -0.2004),
            (0.1002, -0.1998),
            (0.1001, -0.2001),
            (0.1003, -0.2002),
        ]
        self.assertTrue(
            joint_samples_are_stationary(
                stable, minimum_samples=5, maximum_delta_rad=0.002
            )
        )
        self.assertFalse(
            joint_samples_are_stationary(
                stable[:4], minimum_samples=5, maximum_delta_rad=0.002
            )
        )
        moving = list(stable)
        moving[-1] = (0.11, -0.2)
        self.assertFalse(
            joint_samples_are_stationary(
                moving, minimum_samples=5, maximum_delta_rad=0.002
            )
        )

    def test_attention_roi_selects_lower_confidence_candidate_inside_region(self):
        upper = self.detection(1, 0.90, (180, 80))
        lower = self.detection(2, 0.80, (500, 390))
        selected = select_ripe_detection(
            [upper, lower],
            confidence_threshold=0.58,
            selection_roi_xyxy_px=(320, 240, 640, 480),
        )
        self.assertIs(selected, lower)

    def test_attention_roi_is_opt_in_and_fails_closed_when_empty(self):
        upper = self.detection(1, 0.90, (180, 80))
        lower = self.detection(2, 0.80, (500, 390))
        self.assertIs(
            select_ripe_detection(
                [upper, lower],
                confidence_threshold=0.58,
            ),
            upper,
        )
        self.assertIsNone(
            select_ripe_detection(
                [upper],
                confidence_threshold=0.58,
                selection_roi_xyxy_px=(320, 240, 640, 480),
            )
        )
        with self.assertRaises(ValueError):
            validate_selection_roi((320, 240, 320, 480))

    def test_multi_target_selection_keeps_ripe_and_unripe_candidates(self):
        detections = [
            self.detection(3, 0.81, (500, 390), maturity=2),
            self.detection(1, 0.92, (180, 80), maturity=1),
            self.detection(2, 0.40, (320, 220), maturity=1),
        ]
        selected = select_candidate_detections(
            detections,
            confidence_threshold=0.60,
        )
        self.assertEqual([item.target_id for item in selected], [1, 3])

    def test_generalized_retry_keeps_only_the_newest_deferred_frame(self):
        values = {(1, 10): ("old", 2), (2, 0): ("new", 0)}
        self.assertEqual(newest_pending_detection(values), ("new", 0))
        self.assertIsNone(newest_pending_detection({}))

    def test_wrist_hint_selects_current_fruit_instead_of_lowest_sigma_neighbour(self):
        current = SimpleNamespace(
            track_id=4,
            maturity=1,
            position=(0.308, 0.165, 0.549),
            confidence=0.91,
            sigma_m=0.024,
        )
        neighbour = SimpleNamespace(
            track_id=3,
            maturity=1,
            position=(0.487, 0.258, 0.540),
            confidence=0.93,
            sigma_m=0.021,
        )

        selected = select_stable_ripe_track(
            (current, neighbour),
            target_hint_position=(0.308, 0.165, 0.549),
            target_hint_max_distance_m=0.05,
            require_target_hint=True,
        )

        self.assertIs(selected, current)

    def test_required_wrist_hint_fails_closed_when_missing_or_inconsistent(self):
        track = SimpleNamespace(
            track_id=1,
            maturity=1,
            position=(0.45, -0.10, 0.55),
            confidence=0.90,
            sigma_m=0.006,
        )
        self.assertIsNone(
            select_stable_ripe_track((track,), require_target_hint=True)
        )
        self.assertIsNone(
            select_stable_ripe_track(
                (track,),
                target_hint_position=(0.60, 0.20, 0.55),
                target_hint_max_distance_m=0.05,
                require_target_hint=True,
            )
        )

    def test_geometry_layer_estimator_is_explicitly_opt_in(self) -> None:
        source = (
            PACKAGE_ROOT / "strawberry_localization" / "node.py"
        ).read_text(encoding="utf-8")
        development_config = (
            PACKAGE_ROOT / "config" / "localization_geometry_layer_v1.yaml"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'self.declare_parameter("depth_estimator_mode", "center_median")',
            source,
        )
        self.assertIn("geometry_expected_depth_tolerance_m", source)
        self.assertIn("geometry_ambiguity_min_support_ratio", source)
        self.assertIn("geometry_bbox_quantization_margin_px", source)
        self.assertIn("depth_estimator_mode: geometry_layer", development_config)

    def test_geometry_layer_runtime_runner_is_no_motion_only(self) -> None:
        runner = (
            PACKAGE_ROOT.parents[2]
            / "scripts"
            / "run_field_v3_geometry_layer_shadow.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("localization_geometry_layer_v1.yaml", runner)
        self.assertIn("start_manipulation:=false", runner)
        self.assertIn("start_orchestrator:=false", runner)
        self.assertIn("enable_attachment:=false", runner)
        self.assertIn("enable_pose_control:=false", runner)
        self.assertNotIn("pick_and_place_server", runner)

    def test_runtime_drains_executor_before_context_shutdown(self):
        source = (
            PACKAGE_ROOT
            / "strawberry_localization"
            / "node.py"
        ).read_text(encoding="utf-8")
        self.assertIn("signal_handler_options=SignalHandlerOptions.NO", source)
        self.assertIn("signal.signal(signal.SIGTERM, _request_stop)", source)
        self.assertIn("node.prepare_shutdown()", source)
        self.assertLess(
            source.rindex("shutdown_executor_and_wait("),
            source.rindex("rclpy.try_shutdown()"),
        )

    def test_legacy_executor_workers_and_failed_tasks_are_drained(self):
        class Task:
            def __init__(self):
                self.result_calls = 0

            def done(self):
                return True

            def result(self):
                self.result_calls += 1
                raise RuntimeError("expected shutdown task failure")

        class WorkerPool:
            def __init__(self):
                self.wait_values = []

            def shutdown(self, wait):
                self.wait_values.append(wait)

        class Executor:
            def __init__(self):
                self.task = Task()
                self._futures = [self.task]
                self._executor = WorkerPool()

            def shutdown(self, timeout_sec=None):
                self.timeout_sec = timeout_sec
                return True

        executor = Executor()
        self.assertTrue(shutdown_executor_and_wait(executor, 2.0))
        self.assertEqual(executor.timeout_sec, 2.0)
        self.assertEqual(executor._executor.wait_values, [True])
        self.assertEqual(executor.task.result_calls, 1)
        self.assertEqual(executor._futures, [])


if __name__ == "__main__":
    unittest.main()
