"""Passive telemetry contracts; fixtures are observations, never ROS commands."""
import importlib
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
NAMES = tuple(f"panda_joint{i}" for i in range(1, 8))


def production():
    spec = importlib.util.find_spec("strawberry_manipulation.arm_telemetry")
    assert spec is not None, "production arm_telemetry serializer is missing"
    return importlib.import_module("strawberry_manipulation.arm_telemetry")


def stamp(ns):
    return NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def point(positions):
    return NS(positions=positions, velocities=[], accelerations=[], effort=[],
              time_from_start=stamp(250_000_000))


def controller(ns=1_000_000_000):
    return NS(header=NS(stamp=stamp(ns), frame_id="panda_link0"),
              joint_names=list(reversed(NAMES)),
              reference=point([i / 10 for i in range(7)]),
              feedback=point([i / 10 - 0.01 for i in range(7)]),
              error=point([0.01] * 7), output=point([]),
              multi_dof_joint_names=[], speed_scaling_factor=1.0)


def joints(ns=1_000_000_000, velocity=None, positions=None):
    return NS(header=NS(stamp=stamp(ns), frame_id="panda_link0"),
              name=list(reversed(NAMES)),
              position=[0.0] * 7 if positions is None else positions,
              velocity=[0.0] * 7 if velocity is None else velocity, effort=[])


class SerializationTests(unittest.TestCase):
    def test_jazzy_fields_preserve_raw_arrays_and_named_alignment(self):
        api = production()
        message = controller()
        result = api.serialize_controller_state(message)
        self.assertEqual(result["stamp_ns"], 1_000_000_000)
        self.assertEqual(result["raw"]["reference"]["positions"], message.reference.positions)
        self.assertEqual(result["raw"]["feedback"]["positions"], message.feedback.positions)
        self.assertEqual(result["raw"]["error"]["positions"], [0.01] * 7)
        self.assertEqual(result["field_roles"], {"desired": "reference", "actual": "feedback"})
        self.assertEqual(result["aligned"]["panda_joint7"]["desired_position_rad"], 0.0)
        self.assertEqual(result["aligned"]["panda_joint1"]["desired_position_rad"], 0.6)
        self.assertEqual(result["raw"]["reference"]["time_from_start"]["nanosec"], 250_000_000)
        self.assertEqual(result["issues"], [])


class StopEvidenceTests(unittest.TestCase):
    def test_independent_stop_requires_advancing_source_time_and_velocity(self):
        api = production()
        self.assertTrue(hasattr(api, "StopEvidence"), "independent stop analyzer is missing")
        from strawberry_manipulation.motion_evidence import serialize_joint_feedback
        monitor = api.StopEvidence(NAMES, home_positions=[0.0] * 7)
        result = None
        for i in range(12):
            ns = 1_000_000_000 + i * 50_000_000
            row = serialize_joint_feedback(joints(ns), receipt_monotonic_ns=ns,
                                           received_ros_ns=ns)
            result = monitor.observe(row, sample_id=f"test-{i}")
        self.assertEqual(result["stop_status"], "PASS")
        self.assertEqual(result["home_status"], "PASS")
        self.assertGreaterEqual(result["source_duration_ns"], 500_000_000)
        self.assertGreaterEqual(result["receipt_duration_ns"], 500_000_000)
        self.assertGreaterEqual(len(result["sample_ids"]), 3)


class StopAdversarialTests(unittest.TestCase):
    def observe(self, monitor, ns, *, received=None, now=None, message=None):
        from strawberry_manipulation.motion_evidence import serialize_joint_feedback
        return monitor.observe(serialize_joint_feedback(
            joints(ns) if message is None else message,
            receipt_monotonic_ns=ns if received is None else received,
            received_ros_ns=ns if now is None else now), sample_id=f"sample-{ns}")

    def test_replayed_and_paused_source_clock_never_pass(self):
        monitor = production().StopEvidence(NAMES)
        for i in range(20):
            result = self.observe(monitor, 1_000_000_000, received=2_000_000_000 + i * 50_000_000)
            self.assertNotEqual(result["stop_status"], "PASS")
        self.assertIn("DUPLICATE_ACQUISITION_STAMP", result["coverage_issues"])

    def test_unavailable_effort_is_retained_but_not_required_for_stop(self):
        monitor = production().StopEvidence(NAMES)
        for i in range(12):
            ns = 1_000_000_000 + i * 50_000_000
            message = joints(ns)
            message.effort = [float("nan")] * 7
            result = self.observe(monitor, ns, message=message)
        self.assertEqual(result["stop_status"], "PASS")

    def test_missing_velocity_and_nonfinite_are_not_inferred_from_position(self):
        for velocity in ([], [float("nan")] * 7, [float("inf")] * 7, [0.021] * 7):
            with self.subTest(velocity=velocity):
                monitor = production().StopEvidence(NAMES)
                for i in range(12):
                    ns = 1_000_000_000 + i * 50_000_000
                    result = self.observe(monitor, ns, message=joints(ns, velocity=velocity))
                self.assertNotEqual(result["stop_status"], "PASS")

    def test_late_future_and_regressing_acquisition_never_pass(self):
        for offset in (-50_000_000, 300_000_000):
            monitor = production().StopEvidence(NAMES)
            for i in range(12):
                ns = 1_000_000_000 + i * 50_000_000
                result = self.observe(monitor, ns, now=ns + offset)
            self.assertNotEqual(result["stop_status"], "PASS")
        monitor = production().StopEvidence(NAMES)
        self.observe(monitor, 2_000_000_000)
        self.observe(monitor, 1_000_000_000)
        for i in range(20):
            result = self.observe(monitor, 3_000_000_000 + i * 50_000_000)
        self.assertNotEqual(result["stop_status"], "PASS")
        self.assertTrue(monitor.clock_reset)

    def test_source_burst_cannot_substitute_for_receipt_window(self):
        monitor = production().StopEvidence(NAMES)
        for i in range(12):
            result = self.observe(monitor, 1_000_000_000 + i * 50_000_000,
                                  received=1_000_000_000 + i * 1_000_000)
        self.assertNotEqual(result["stop_status"], "PASS")

    def test_stop_is_not_home_and_expires(self):
        monitor = production().StopEvidence(NAMES, home_positions=[0.5] * 7)
        for i in range(12):
            result = self.observe(monitor, 1_000_000_000 + i * 50_000_000)
        self.assertEqual(result["stop_status"], "PASS")
        self.assertEqual(result["home_status"], "FAIL")
        expired = monitor.current(now_monotonic_ns=3_000_000_000, now_ros_ns=1_550_000_000)
        self.assertNotEqual(expired["stop_status"], "PASS")

    def test_gap_resets_window_and_is_not_erased_by_later_stop(self):
        monitor = production().StopEvidence(NAMES)
        for ns in (1_000_000_000, 1_050_000_000, 1_900_000_000):
            result = self.observe(monitor, ns)
        self.assertNotEqual(result["stop_status"], "PASS")
        for i in range(12):
            result = self.observe(monitor, 1_950_000_000 + i * 50_000_000)
        self.assertEqual(result["stop_status"], "PASS")
        self.assertIn("SAMPLE_GAP", result["coverage_issues"])
        self.assertFalse(result["full_interval_safety_claim"])


    def test_command_overflow_json_is_retained_not_crash(self):
        import tempfile
        api = production()
        with tempfile.TemporaryDirectory() as tmp:
            recorder = api.TelemetryRecorder(Path(tmp) / "run", run_id="r", scenario_id="s")
            recorder.command('{"a": 1e309}', received_ns=1, ros_now_ns=2)
            summary = recorder.close(now_ns=3, ros_now_ns=4)
            self.assertEqual(summary["invalid_command_count"], 1)
            rows = (Path(tmp) / "run" / "samples.jsonl").read_text()
            self.assertIn("1e309", rows)
            self.assertIn("MALFORMED_COMMAND_EVENT", rows)

    def test_missing_command_preparation_flags_orphan_acceptance(self):
        import tempfile
        import json
        api = production()
        with tempfile.TemporaryDirectory() as tmp:
            recorder = api.TelemetryRecorder(Path(tmp) / "run", run_id="r", scenario_id="s")
            event = {"run_id": "r", "scenario_id": "s", "producer_id": "p",
                     "producer_seq": 2, "event_type": "ACTION_ACCEPTED", "command_id": "p/cmd/x"}
            recorder.command(json.dumps(event), received_ns=1, ros_now_ns=2)
            summary = recorder.close(now_ns=3, ros_now_ns=4)
            self.assertGreaterEqual(summary["invalid_command_count"], 1)
            rows_text = (Path(tmp) / "run" / "samples.jsonl").read_text()
            self.assertTrue("COMMAND_PREPARATION_MISSING" in rows_text or "COMMAND_SEQUENCE_DISCONTINUITY" in rows_text)

    def test_invalid_preparation_cannot_authorize_later_action(self):
        import tempfile
        import json
        api = production()
        with tempfile.TemporaryDirectory() as tmp:
            recorder = api.TelemetryRecorder(Path(tmp) / "run", run_id="r", scenario_id="s")
            command_id = "p/command/x"
            bad = {"schema_version": 1, "run_id": "wrong", "scenario_id": "s",
                   "producer_id": "p", "producer_seq": 1,
                   "event_type": "COMMAND_PREPARED", "command_id": command_id}
            accepted = {"schema_version": 1, "run_id": "r", "scenario_id": "s",
                        "producer_id": "p", "producer_seq": 2,
                        "event_type": "ACTION_ACCEPTED", "command_id": command_id}
            recorder.command(json.dumps(bad), received_ns=1, ros_now_ns=2)
            self.assertEqual(recorder.command_sequences, {})
            self.assertNotIn(command_id, recorder.prepared_commands)
            recorder.command(json.dumps(accepted), received_ns=2, ros_now_ns=3)
            summary = recorder.close(now_ns=3, ros_now_ns=4)
            self.assertEqual(summary["invalid_command_count"], 2)
            text = (Path(tmp) / "run" / "samples.jsonl").read_text()
            self.assertIn("COMMAND_PREPARATION_MISSING", text)

    def test_cross_producer_command_reuse_is_rejected(self):
        import tempfile
        import json
        api = production()
        with tempfile.TemporaryDirectory() as tmp:
            recorder = api.TelemetryRecorder(Path(tmp) / "run", run_id="r", scenario_id="s")
            command_id = "p1/command/x"
            prepared = {"schema_version": 1, "run_id": "r", "scenario_id": "s",
                        "producer_id": "p1", "producer_seq": 1,
                        "event_type": "COMMAND_PREPARED", "command_id": command_id}
            accepted = {"schema_version": 1, "run_id": "r", "scenario_id": "s",
                        "producer_id": "p2", "producer_seq": 1,
                        "event_type": "ACTION_ACCEPTED", "command_id": command_id}
            recorder.command(json.dumps(prepared), received_ns=1, ros_now_ns=2)
            recorder.command(json.dumps(accepted), received_ns=2, ros_now_ns=3)
            summary = recorder.close(now_ns=3, ros_now_ns=4)
            self.assertEqual(summary["invalid_command_count"], 1)
            self.assertIn(
                "COMMAND_PRODUCER_MISMATCH",
                (Path(tmp) / "run" / "samples.jsonl").read_text(),
            )

    def test_duplicate_preparation_cannot_rebind_command_owner(self):
        import tempfile
        import json
        api = production()
        with tempfile.TemporaryDirectory() as tmp:
            recorder = api.TelemetryRecorder(Path(tmp) / "run", run_id="r", scenario_id="s")
            command_id = "p1/command/x"
            original = {"schema_version": 1, "run_id": "r", "scenario_id": "s",
                        "producer_id": "p1", "producer_seq": 1,
                        "event_type": "COMMAND_PREPARED", "command_id": command_id}
            hijack = {"schema_version": 1, "run_id": "r", "scenario_id": "s",
                       "producer_id": "p2", "producer_seq": 1,
                       "event_type": "COMMAND_PREPARED", "command_id": command_id}
            action = {"schema_version": 1, "run_id": "r", "scenario_id": "s",
                      "producer_id": "p2", "producer_seq": 1,
                      "event_type": "ACTION_ACCEPTED", "command_id": command_id}
            recorder.command(json.dumps(original), received_ns=1, ros_now_ns=2)
            recorder.command(json.dumps(hijack), received_ns=2, ros_now_ns=3)
            self.assertEqual(
                recorder.prepared_commands[command_id]["producer"], "p1"
            )
            recorder.command(json.dumps(action), received_ns=3, ros_now_ns=4)
            summary = recorder.close(now_ns=4, ros_now_ns=5)
            self.assertEqual(summary["invalid_command_count"], 2)
            text = (Path(tmp) / "run" / "samples.jsonl").read_text()
            self.assertIn("COMMAND_ID_ALREADY_PREPARED", text)
            self.assertIn("COMMAND_PRODUCER_MISMATCH", text)

    def test_preparation_command_id_must_use_producer_namespace(self):
        import tempfile
        import json
        api = production()
        with tempfile.TemporaryDirectory() as tmp:
            recorder = api.TelemetryRecorder(Path(tmp) / "run", run_id="r", scenario_id="s")
            event = {"schema_version": 1, "run_id": "r", "scenario_id": "s",
                     "producer_id": "p1", "producer_seq": 1,
                     "event_type": "COMMAND_PREPARED",
                     "command_id": "other/command/x"}
            recorder.command(json.dumps(event), received_ns=1, ros_now_ns=2)
            summary = recorder.close(now_ns=2, ros_now_ns=3)
            self.assertEqual(summary["invalid_command_count"], 1)
            self.assertEqual(recorder.prepared_commands, {})

class RecorderTests(unittest.TestCase):
    def test_recording_preserves_payload_sequence_and_invalid_samples(self):
        import tempfile
        import json
        api = production()
        self.assertTrue(hasattr(api, "TelemetryRecorder"), "recorder is missing")
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "run"
            recorder = api.TelemetryRecorder(out, run_id="test-run", scenario_id="test-scene")
            for i in range(12):
                ns = 1_000_000_000 + i * 50_000_000
                recorder.controller(controller(ns), received_ns=ns, ros_now_ns=ns)
                recorder.joints(joints(ns), received_ns=ns, ros_now_ns=ns)
            recorder.joints(joints(ns), received_ns=ns + 1, ros_now_ns=ns)
            summary = recorder.close(now_ns=ns + 2, ros_now_ns=ns)
            rows = [json.loads(line) for line in (out / "samples.jsonl").read_text().splitlines()]
            self.assertEqual([r["producer_seq"] for r in rows], list(range(1, len(rows) + 1)))
            self.assertEqual(summary["controller_sample_count"], 12)
            self.assertEqual(summary["joint_sample_count"], 13)
            self.assertIn("panda_joint7", summary["tracking_peak_errors_rad"])
            self.assertNotEqual(summary["independent_stop"]["stop_status"], "PASS")
            self.assertIn("DUPLICATE_ACQUISITION_STAMP", rows[-2]["issues"])
            self.assertEqual(rows[1]["payload"]["raw"]["reference"]["positions"], controller().reference.positions)
            self.assertIsNone(rows[1]["event_time_monotonic_ns"])
            self.assertEqual(rows[1]["received_time_monotonic_ns"], 1_000_000_000)
            with self.assertRaises(FileExistsError):
                api.TelemetryRecorder(out, run_id="again", scenario_id="test")

    def test_no_samples_and_malformed_command_cannot_be_valid_stop(self):
        import tempfile
        api = production()
        self.assertTrue(hasattr(api, "TelemetryRecorder"), "recorder is missing")
        with tempfile.TemporaryDirectory() as tmp:
            recorder = api.TelemetryRecorder(Path(tmp) / "run", run_id="test", scenario_id="test")
            recorder.command("bad json", received_ns=3, ros_now_ns=4)
            result = recorder.close(now_ns=5, ros_now_ns=6)
            self.assertEqual(result["independent_stop"]["stop_status"], "INDETERMINATE")
            self.assertEqual(result["invalid_command_count"], 1)
            self.assertFalse(result["full_interval_safety_claim"])


if __name__ == "__main__":
    unittest.main()
