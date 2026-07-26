from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest

from strawberry_benchmark.cli import main
from strawberry_benchmark.io import write_trial_results_jsonl
from strawberry_benchmark.models import (
    Maturity,
    ScenarioKind,
    TrialMode,
    TrialResult,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]


class CliTests(unittest.TestCase):
    def test_generate_scenarios_command(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "scenarios.jsonl"
            stdout = io.StringIO()
            with redirect_stdout(stdout):
                return_code = main(
                    [
                        "generate-scenarios",
                        "--config",
                        str(REPOSITORY_ROOT / "config" / "benchmark.yaml"),
                        "--output",
                        str(output),
                    ]
                )
            summary = json.loads(stdout.getvalue())
            lines = output.read_text(encoding="utf-8").splitlines()
        self.assertEqual(0, return_code)
        self.assertEqual(135, summary["positive_trials"])
        self.assertEqual(30, summary["negative_trials"])
        self.assertEqual(165, summary["total_trials"])
        self.assertEqual(165, len(lines))

    def test_evaluate_command_passes_complete_log(self):
        common = {
            "occlusion": "none",
            "lighting": "nominal",
            "position": "center",
            "seed": 20260710,
            "detection_confidence": 0.95,
            "localization_error_mm": 5.0,
            "planning_time_sec": 1.0,
            "pick_attempted": True,
            "fruit_picked": True,
            "success": True,
        }
        results = [
            TrialResult(
                trial_id="oracle-positive",
                mode=TrialMode.ORACLE,
                scenario_kind=ScenarioKind.POSITIVE,
                expected_maturity=Maturity.RIPE,
                predicted_maturity=None,
                **common,
            ),
            TrialResult(
                trial_id="e2e-positive",
                mode=TrialMode.END_TO_END,
                scenario_kind=ScenarioKind.POSITIVE,
                expected_maturity=Maturity.RIPE,
                predicted_maturity=Maturity.RIPE,
                **common,
            ),
            TrialResult(
                trial_id="e2e-negative",
                mode=TrialMode.END_TO_END,
                scenario_kind=ScenarioKind.NEGATIVE,
                expected_maturity=Maturity.UNRIPE,
                predicted_maturity=Maturity.UNRIPE,
                **{
                    **common,
                    "pick_attempted": False,
                    "fruit_picked": False,
                    "localization_error_mm": None,
                    "planning_time_sec": None,
                },
            ),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "results.jsonl"
            write_trial_results_jsonl(results, path)
            stdout = io.StringIO()
            with redirect_stdout(stdout):
                return_code = main(
                    [
                        "evaluate",
                        "--results",
                        str(path),
                        "--project-config",
                        str(REPOSITORY_ROOT / "config" / "project.yaml"),
                    ]
                )
        payload = json.loads(stdout.getvalue())
        self.assertEqual(0, return_code)
        self.assertEqual("PASS", payload["acceptance"]["status"])


if __name__ == "__main__":
    unittest.main()
