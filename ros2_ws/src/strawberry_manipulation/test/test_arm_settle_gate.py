"""Adversarial tests for physical arm-stop confirmation (no ROS motion).

Only the ROS transport and wall clock are simulated. The production callback
and settling gate are called directly; a cached planning scene is not evidence
that physical joint samples are arriving.
"""

import pathlib
import sys
import threading
import unittest
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import patch


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.moveit_backend import (  # noqa: E402
    ArmStateSnapshot,
    MoveItBackend,
)


class FakeClock:
    def __init__(self, on_sleep=None):
        self.now = 0.0
        self.on_sleep = on_sleep

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        if self.on_sleep is not None:
            self.on_sleep(self.now)


class ArmSettleGateTests(unittest.TestCase):
    def make_backend(self):
        backend = MoveItBackend.__new__(MoveItBackend)
        backend._arm_joint_names = tuple(f"panda_joint{i}" for i in range(1, 8))
        backend.gripper_joints = ("panda_finger_joint1", "panda_finger_joint2")
        backend._gripper_state_lock = threading.Lock()
        backend._latest_gripper_positions_m = dict.fromkeys(backend.gripper_joints)
        backend._latest_arm_positions_rad = dict.fromkeys(backend._arm_joint_names)
        backend._arm_state_sequence = 0
        backend.planning_group = "panda_arm"
        backend.settle_timeout_sec = 1.5
        backend.settle_window_sec = 0.5
        backend.settle_sample_period_sec = 0.05
        backend.settle_delta_rad = 0.002
        backend.settle_stable_samples = 3
        errors = []
        backend.node = SimpleNamespace(
            get_logger=lambda: SimpleNamespace(error=errors.append)
        )
        backend.node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=backend._latest_complete_stamp_ns)
        )
        backend._latest_complete_stamp_ns = 1_000_000_000
        backend._planning_scene_monitor = SimpleNamespace(
            read_only=lambda: nullcontext(
                SimpleNamespace(
                    current_state=SimpleNamespace(
                        get_joint_group_positions=lambda group: (
                            0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785
                        )
                    )
                )
            )
        )
        return backend, errors

    def run_gate(self, backend, clock):
        with (
            patch(
                "strawberry_manipulation.moveit_backend.time.monotonic",
                clock.monotonic,
            ),
            patch(
                "strawberry_manipulation.moveit_backend.time.monotonic_ns",
                lambda: 1_000_000_000 + round(clock.now * 1_000_000_000),
            ),
            patch("strawberry_manipulation.moveit_backend.time.sleep", clock.sleep),
        ):
            return backend._wait_until_arm_settled()

    def emit_positions(self, backend, positions=None):
        ns = 1_000_000_000 + (backend._arm_state_sequence + 1) * 50_000_000
        backend._latest_complete_stamp_ns = ns
        backend._on_gripper_joint_state(
            SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(sec=ns // 1_000_000_000,
                                                             nanosec=ns % 1_000_000_000)),
                name=list(backend._arm_joint_names),
                position=list(positions or (
                    0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785
                )),
            )
        )

    def test_replayed_acquisition_stamp_cannot_prove_stop(self):
        backend, _ = self.make_backend()
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0), frame_id="base"),
            name=list(backend._arm_joint_names),
            position=[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            velocity=[0.0] * 7,
        )
        clock = FakeClock(lambda now: backend._on_gripper_joint_state(message))
        self.assertFalse(self.run_gate(backend, clock))

    def test_old_complete_feedback_cannot_prove_stop(self):
        backend, _ = self.make_backend()
        clock = FakeClock(lambda now: self.emit_positions(backend))
        backend._evidence_ros_now_ns = lambda: 100_000_000_000
        backend._on_gripper_joint_state(
            SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
                name=list(backend._arm_joint_names),
                position=[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
                velocity=[0.0] * 7,
            )
        )
        self.assertFalse(self.run_gate(backend, clock))

    def test_partial_feedback_cannot_extend_complete_arm_snapshot(self):
        backend, _ = self.make_backend()
        self.emit_positions(backend)
        initial_sequence = backend._arm_state_sequence
        stamps = [2_000_000_000]

        def deliver(_now):
            stamps[0] += 50_000_000
            backend._evidence_ros_now_ns = lambda: stamps[0]
            backend._on_gripper_joint_state(
                SimpleNamespace(
                    header=SimpleNamespace(stamp=SimpleNamespace(
                        sec=stamps[0] // 1_000_000_000,
                        nanosec=stamps[0] % 1_000_000_000)),
                    name=["panda_finger_joint1"],
                    position=[0.04],
                )
            )

        self.assertFalse(self.run_gate(backend, FakeClock(deliver)))
        self.assertEqual(backend._arm_state_sequence, initial_sequence)

    def test_long_feedback_gap_cannot_count_as_a_stable_window(self):
        backend, _ = self.make_backend()
        self.emit_positions(backend)
        arrivals = [0.05, 1.0]

        def deliver(now):
            if arrivals and now >= arrivals[0]:
                arrivals.pop(0)
                self.emit_positions(backend)

        clock = FakeClock(deliver)
        self.assertFalse(self.run_gate(backend, clock))
        self.assertEqual(arrivals, [])

    def test_nonfinite_joint_sample_cannot_prove_stop(self):
        backend, _ = self.make_backend()
        positions = [0.0, -0.785, 0.0, -2.356, 0.0, float("nan"), 0.785]
        clock = FakeClock(lambda now: self.emit_positions(backend, positions))

        self.assertFalse(self.run_gate(backend, clock))

    def test_fresh_stable_callback_stream_can_prove_stop(self):
        backend, _ = self.make_backend()
        clock = FakeClock(lambda now: self.emit_positions(backend))

        self.assertTrue(self.run_gate(backend, clock))
        self.assertGreaterEqual(clock.now, backend.settle_window_sec)
        self.assertLess(clock.now, backend.settle_timeout_sec)

    def test_one_cached_live_sample_cannot_prove_stop(self):
        backend, _ = self.make_backend()
        self.emit_positions(backend)

        self.assertFalse(self.run_gate(backend, FakeClock()))

    def test_partial_messages_cannot_extend_arm_sample_sequence(self):
        backend, _ = self.make_backend()
        self.emit_positions(backend)
        clock = FakeClock(lambda now: backend._on_gripper_joint_state(
            SimpleNamespace(name=["panda_joint6"], position=[1.571])
        ))

        self.assertFalse(self.run_gate(backend, clock))
        self.assertEqual(backend._arm_state_sequence, 1)

    def test_malformed_complete_arm_message_cannot_advance_snapshot(self):
        backend, _ = self.make_backend()
        self.emit_positions(backend)
        original_sequence = backend._arm_state_sequence
        original_feedback = backend._latest_arm_feedback
        backend._on_gripper_joint_state(
            SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0)),
                name=list(backend._arm_joint_names) + ["panda_joint1"],
                position=[0.0] * 8,
                velocity=[0.0] * 8,
            )
        )
        self.assertEqual(backend._arm_state_sequence, original_sequence)
        self.assertIs(backend._latest_arm_feedback, original_feedback)

    def test_snapshot_reader_returns_positions_and_timestamps_from_one_sample(self):
        backend, _ = self.make_backend()
        self.emit_positions(backend)
        snapshot = backend._latest_live_arm_snapshot()
        self.assertEqual(snapshot.sequence, backend._arm_state_sequence)
        self.assertEqual(
            snapshot.positions, tuple(backend._latest_arm_positions_rad.values())
        )
        self.assertEqual(dict(snapshot.feedback), backend._latest_arm_feedback)
        self.assertEqual(
            snapshot.receipt_monotonic_ns,
            backend._latest_arm_complete_monotonic_ns,
        )

    def test_snapshot_feedback_remains_json_serializable(self):
        import json
        backend, _ = self.make_backend()
        self.emit_positions(backend)
        json.dumps(backend._latest_live_arm_snapshot().feedback, allow_nan=False)

    def test_slow_drift_in_callback_stream_does_not_prove_stop(self):
        backend, _ = self.make_backend()
        positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        def deliver(now):
            positions[5] = 1.571 + now * 0.01
            self.emit_positions(backend, positions)

        self.assertFalse(self.run_gate(backend, FakeClock(deliver)))

    def test_gap_restarts_window_before_recovered_stream_can_pass(self):
        backend, _ = self.make_backend()
        self.emit_positions(backend)

        def deliver(now):
            if now <= 0.05 or now >= 0.75:
                self.emit_positions(backend)

        clock = FakeClock(deliver)
        self.assertTrue(self.run_gate(backend, clock))
        self.assertGreaterEqual(clock.now, 0.75 + backend.settle_window_sec)

    def test_invalid_dimensions_and_infinity_are_rejected(self):
        invalid_positions = [
            (),
            (0.0,) * 6,
            (0.0,) * 8,
            (0.0, -0.785, 0.0, -2.356, 0.0, float("inf"), 0.785),
        ]
        for positions in invalid_positions:
            with self.subTest(positions=positions):
                backend, _ = self.make_backend()
                backend._latest_arm_snapshot = ArmStateSnapshot(
                    sequence=1,
                    positions=positions,
                    feedback_json='{"acquisition_stamp_ns":1000000000}',
                    receipt_monotonic_ns=1_000_000_000,
                )
                self.assertFalse(self.run_gate(backend, FakeClock()))

    def test_repeated_or_reversed_sequences_cannot_prove_stop(self):
        for direction in (0, -1):
            with self.subTest(direction=direction):
                backend, _ = self.make_backend()
                sequence = [100]

                def snapshot():
                    sequence[0] += direction
                    stamp_ns = 1_000_000_000 + sequence[0] * 1_000_000
                    return ArmStateSnapshot(
                        sequence=sequence[0],
                        positions=(0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785),
                        feedback_json=(
                            '{"acquisition_stamp_ns":' + str(stamp_ns) + '}'
                        ),
                        receipt_monotonic_ns=1_000_000_000,
                    )

                backend._latest_live_arm_snapshot = snapshot
                backend._evidence_ros_now_ns = lambda: 1_100_000_000
                self.assertFalse(self.run_gate(backend, FakeClock()))

    def test_cached_planning_scene_without_joint_messages_cannot_prove_stop(self):
        backend, errors = self.make_backend()
        clock = FakeClock()

        self.assertFalse(self.run_gate(backend, clock))
        self.assertGreaterEqual(clock.now, backend.settle_timeout_sec)
        self.assertTrue(errors)


if __name__ == "__main__":
    unittest.main()
