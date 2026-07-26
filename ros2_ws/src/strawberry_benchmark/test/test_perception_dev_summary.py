import json
from pathlib import Path
import tempfile
import unittest

from strawberry_benchmark.perception_dev_summary import (
    summarize_repeated_dev_gate,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
MANIFEST_PATH = (
    REPOSITORY_ROOT / "config" / "p3_perception_repeated_dev_gate_v1.json"
)


def _write_trial(output: Path, label: str, mode: str) -> None:
    positive = mode == "positive"
    payload = {
        "success": True,
        "manipulation_success": positive,
        "safe_no_pick": not positive,
        "fruit_picked": positive,
        "unsafe_control_attempt": False,
        "state_machine_complete": True,
        "position_configuration_applied": True,
        "parked_model_configuration_applied": True,
        "settled_window_detection_frames": 10,
        "settled_window_detection_count": 10 if positive else 0,
        "settled_window_ripe_detection_count": 10 if positive else 0,
        "settled_window_total_control_samples": 10 if positive else 0,
        "planning_time_sec": 0.1 if positive else None,
        "execution_time_sec": 1.0 if positive else None,
        "failure_code": None,
        "failure_message": "",
        "final_status": {
            "state": "DONE",
            "outcome": "SUCCESS" if positive else "NO_PICK",
            "target_source": "perception",
            "control_target_topic": "/strawberry/target_pose",
            "shadow_target_topic": "/strawberry/shadow/target_pose",
            "control_target_id": 1 if positive else None,
            "state_history": [],
        },
    }
    (output / f"{label}.json").write_text(
        json.dumps(payload), encoding="utf-8"
    )
    (output / f"{label}_launch.log").write_text("clean shutdown\n", encoding="utf-8")


def _write_complete_fixture(output: Path) -> None:
    for mode in ("positive", "negative"):
        for index in range(1, 11):
            _write_trial(output, f"{mode}_{index:02d}", mode)


class PerceptionRepeatedDevSummaryTests(unittest.TestCase):
    def test_complete_fixture_passes(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            _write_complete_fixture(output)
            result = summarize_repeated_dev_gate(
                output, MANIFEST_PATH, REPOSITORY_ROOT
            )
            self.assertTrue(result["development_gate_passed"])
            self.assertEqual(10, result["positive_successes"])
            self.assertEqual(10, result["negative_no_picks"])
            self.assertEqual(0, result["negative_control_attempts"])

    def test_negative_control_attempt_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            _write_complete_fixture(output)
            path = output / "negative_01.json"
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["success"] = False
            payload["safe_no_pick"] = False
            payload["unsafe_control_attempt"] = True
            payload["final_status"]["outcome"] = "FAILED"
            payload["final_status"]["control_target_id"] = 2
            path.write_text(json.dumps(payload), encoding="utf-8")
            result = summarize_repeated_dev_gate(
                output, MANIFEST_PATH, REPOSITORY_ROOT
            )
            self.assertFalse(result["development_gate_passed"])
            self.assertEqual(1, result["negative_control_attempts"])
            self.assertEqual(0.9, result["negative_no_pick_rate"])

    def test_missing_result_is_infrastructure_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            _write_complete_fixture(output)
            (output / "positive_01.json").unlink()
            result = summarize_repeated_dev_gate(
                output, MANIFEST_PATH, REPOSITORY_ROOT
            )
            self.assertFalse(result["infrastructure_passed"])
            self.assertFalse(result["development_gate_passed"])


if __name__ == "__main__":
    unittest.main()
