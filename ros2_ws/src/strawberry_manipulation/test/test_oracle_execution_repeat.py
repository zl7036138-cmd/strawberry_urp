import importlib.util
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
SCRIPT = (
    REPOSITORY_ROOT
    / "scripts"
    / "summarize_blender_v2_oracle_execution_repeat.py"
)
spec = importlib.util.spec_from_file_location(
    "_oracle_execution_repeat_summary", SCRIPT
)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class OracleExecutionRepeatTests(unittest.TestCase):
    def test_requires_all_five_fixed_trials(self):
        records = [{"passed": True} for _ in range(5)]
        self.assertTrue(module.aggregate_passes(records, 5))
        records[3]["passed"] = False
        self.assertFalse(module.aggregate_passes(records, 5))
        self.assertFalse(
            module.aggregate_passes([{"passed": True}] * 4, 5)
        )

    def test_runner_starts_and_stops_one_fresh_world_per_trial(self):
        runner = (
            REPOSITORY_ROOT
            / "scripts"
            / "run_blender_v2_oracle_execution_repeat_5.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('trial_count=5', runner)
        self.assertIn(
            'for trial_index in $(seq 1 "${trial_count}")', runner
        )
        self.assertIn("shutdown_launch", runner)
        self.assertIn(
            "capture_blender_v2_execution_initial_state.py", runner
        )
        self.assertIn("test_oracle_pick_and_place.py", runner)
        self.assertIn("start_perception:=false", runner)
        self.assertIn("start_orchestrator:=false", runner)
        self.assertIn("enable_pose_control:=false", runner)
        self.assertIn("enable_attachment:=true", runner)
        self.assertNotIn("retry", runner.lower())


if __name__ == "__main__":
    unittest.main()
