from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]


class OracleExecutionRepeatV2Tests(unittest.TestCase):
    def test_domain_filter_removes_only_cli_self_node(self):
        runner = (
            REPOSITORY_ROOT
            / "scripts"
            / "run_blender_v2_oracle_execution_repeat_5_v2.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("grep -Ev '^/_ros2cli_[0-9]+$'", runner)
        self.assertIn('if [[ -n "${discovered}" ]]', runner)
        self.assertNotIn("grep -Ev '/_ros2cli_", runner)

    def test_v2_repeat_keeps_fixed_five_trial_matrix(self):
        runner = (
            REPOSITORY_ROOT
            / "scripts"
            / "run_blender_v2_oracle_execution_repeat_5_v2.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("trial_count=5", runner)
        self.assertIn(
            'for trial_index in $(seq 1 "${trial_count}")', runner
        )
        self.assertIn("camera_mount:=dual", runner)
        self.assertIn("start_perception:=false", runner)
        self.assertIn("start_orchestrator:=false", runner)
        self.assertIn("enable_attachment:=true", runner)
        self.assertIn("enable_pose_control:=false", runner)
        self.assertNotIn("retry", runner.lower())


if __name__ == "__main__":
    unittest.main()
