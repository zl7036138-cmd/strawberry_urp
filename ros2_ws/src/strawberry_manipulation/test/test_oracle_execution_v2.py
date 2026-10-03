import importlib.util
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]


def load_script(name, filename):
    path = REPOSITORY_ROOT / "scripts" / filename
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


probe = load_script(
    "_execution_initial_state_probe",
    "capture_blender_v2_execution_initial_state.py",
)


class OracleExecutionV2Tests(unittest.TestCase):
    def test_complete_ready_snapshot_passes(self):
        violations, error = probe.evaluate_snapshot(
            probe.READY_JOINTS,
            {
                "panda_finger_joint1": 0.04,
                "panda_finger_joint2": 0.04,
            },
            False,
            10,
            required_samples=10,
            maximum_ready_error_rad=0.02,
        )
        self.assertEqual(violations, [])
        self.assertEqual(error, 0.0)

    def test_incomplete_or_attached_snapshot_fails(self):
        violations, _ = probe.evaluate_snapshot(
            {},
            {},
            True,
            0,
            required_samples=10,
            maximum_ready_error_rad=0.02,
        )
        self.assertIn("initial arm state incomplete", violations)
        self.assertIn("initial gripper state incomplete", violations)
        self.assertIn("target is not confirmed detached", violations)
        self.assertIn("insufficient complete joint samples", violations)

    def test_v2_runner_preserves_motion_parameters_and_adds_preflight(self):
        runner = (
            REPOSITORY_ROOT
            / "scripts"
            / "run_blender_v2_oracle_execution_round_trip_v2.sh"
        ).read_text(encoding="utf-8")
        for expected in (
            "capture_blender_v2_execution_initial_state.py",
            "--required-samples 10",
            "--maximum-ready-error-rad 0.02",
            "camera_mount:=dual",
            "start_perception:=false",
            "start_oracle_provider:=false",
            "start_manipulation:=true",
            "start_orchestrator:=false",
            "enable_attachment:=true",
            "enable_pose_control:=false",
            "--target-topic /strawberry/ground_truth/fruit_1/pose",
            "--place-x 0.35",
            "--place-y -0.45",
            "--place-z 0.45",
        ):
            self.assertIn(expected, runner)
        self.assertIn("refusing to overwrite frozen output", runner)
        self.assertNotIn("for trial_", runner)


if __name__ == "__main__":
    unittest.main()
