import copy
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "empty_arm_probe", ROOT / "scripts/probe_empty_arm_command_chain.py"
)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class ReplayBoundsTests(unittest.TestCase):
    def trajectory(self):
        q = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        return {"joint_names": [f"panda_joint{i}" for i in range(1, 8)],
                "points": [{"positions": copy.copy(q), "time_from_start_ns": t}
                           for t in (0, 100_000_000, 200_000_000)]}

    def test_valid_bounded_empty_arm_command(self):
        probe.validate_trajectory(self.trajectory())

    def test_rejects_wrong_joint_identity_and_nonfinite_position(self):
        data = self.trajectory()
        data["joint_names"][4] = "panda_finger_joint1"
        with self.assertRaises(ValueError): probe.validate_trajectory(data)
        data = self.trajectory()
        data["points"][0]["positions"][4] = float("nan")
        with self.assertRaises(ValueError): probe.validate_trajectory(data)

    def test_rejects_unbounded_or_regressing_command_time(self):
        for stamp in (0, 11_000_000_000):
            data = self.trajectory()
            data["points"][1]["time_from_start_ns"] = stamp
            with self.assertRaises(ValueError): probe.validate_trajectory(data)

    def test_rejects_joint_limit_and_excessive_travel(self):
        data = self.trajectory()
        data["points"][1]["positions"][4] = 3.0
        with self.assertRaises(ValueError): probe.validate_trajectory(data)
        data = self.trajectory()
        data["points"][1]["positions"][4] = 1.1
        with self.assertRaises(ValueError): probe.validate_trajectory(data)

    def test_observer_has_no_command_component_writes(self):
        text = (ROOT / "tools/gazebo_command_audit/CommandAudit.cc").read_text()
        for mutation in ("CreateComponent(", "SetData(", "RemoveComponent("):
            self.assertNotIn(mutation, text)
        self.assertIn("ConfigurePriority() override { return -1; }", text)
        self.assertIn("strawberry::CommandAudit::ISystemConfigurePriority", text)


class StopEvidenceTests(unittest.TestCase):
    def frames(self):
        return [{"sim_sec": t, "receipt_monotonic_sec": t,
                 "actual": [0.0]*7, "reference": [0.0]*7, "velocities": [0.0]*7}
                for t in (10.0, 10.2, 10.5)]

    def test_stationary_fresh_complete_feedback(self):
        result = probe.final_feedback_metrics(self.frames(), [0.0]*7, 10.6)
        self.assertTrue(result["final_stop_observed"])
        self.assertEqual(result["final_endpoint_error_rad"], 0.0)

    def test_empty_or_malformed_feedback_cannot_prove_stop(self):
        self.assertFalse(probe.final_feedback_metrics([], [0.0]*7, 10.6)["final_stop_observed"])
        frames = self.frames()
        frames[-1]["actual"] = [0.0]
        self.assertFalse(probe.final_feedback_metrics(frames, [0.0]*7, 10.6)["final_stop_observed"])

    def test_stale_duplicate_and_backward_clock_withhold_stop(self):
        self.assertFalse(probe.final_feedback_metrics(self.frames(), [0.0]*7, 12)["final_stop_observed"])
        for stamp in (10.0, 9.0):
            frames = self.frames()
            frames[1]["sim_sec"] = stamp
            self.assertFalse(probe.final_feedback_metrics(frames, [0.0]*7, 10.6)["final_stop_observed"])

    def test_motion_drift_and_nonfinite_feedback_withhold_stop(self):
        for field, value in (("velocities", 0.03), ("actual", 0.003), ("actual", float("nan"))):
            frames = self.frames()
            frames[1][field][4] = value
            self.assertFalse(probe.final_feedback_metrics(frames, [0.0]*7, 10.6)["final_stop_observed"])

    def test_stop_does_not_imply_correct_endpoint(self):
        result = probe.final_feedback_metrics(self.frames(), [0.1]*7, 10.6)
        self.assertTrue(result["final_stop_observed"])
        self.assertGreater(result["final_endpoint_error_rad"], 0.05)


if __name__ == "__main__": unittest.main()
