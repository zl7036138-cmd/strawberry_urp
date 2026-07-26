import importlib.util
import json
import pathlib
import tempfile
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[4] / "scripts" / "summarize_t60_oracle_gate.py"
SPEC = importlib.util.spec_from_file_location("summarize_t60_oracle_gate", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class T60SummaryTests(unittest.TestCase):
    def write_trial(self, root: pathlib.Path, **overrides) -> None:
        payload = {
            "success": True,
            "source_isolated": True,
            "state_machine_complete": True,
            "planning_time_sec": 0.2,
            **overrides,
        }
        (root / "trial_01.json").write_text(json.dumps(payload), encoding="utf-8")
        (root / "trial_01_launch.log").write_text("clean shutdown", encoding="utf-8")

    def test_green_trial_passes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self.write_trial(root)
            result = MODULE.summarize(root, 1)
            self.assertTrue(result["gate_passed"])
            self.assertFalse(result["held_out_test_consumed"])

    def test_routing_leak_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self.write_trial(root, source_isolated=False)
            result = MODULE.summarize(root, 1)
            self.assertFalse(result["gate_passed"])


if __name__ == "__main__":
    unittest.main()
