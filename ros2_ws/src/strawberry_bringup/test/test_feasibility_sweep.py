import json
from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.feasibility_sweep import (  # noqa: E402
    scenario_rows,
    select_runtime_scenarios,
    validate_development_matrix,
)


class FeasibilitySweepTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads(
            (
                ROOT / "config" / "generalized_runtime_development_matrix_v1.json"
            ).read_text(encoding="utf-8")
        )
        formal = json.loads(
            (ROOT / "config" / "generalized_harvest_matrix_v1.json").read_text(
                encoding="utf-8"
            )
        )
        cls.formal_seeds = [row["seed"] for row in formal["scenarios"]]

    def test_frozen_schedule_is_balanced_and_formal_disjoint(self):
        validate_development_matrix(self.config, formal_seeds=self.formal_seeds)
        rows = scenario_rows(self.config, split="discovery")

        self.assertEqual(len(rows), 18)
        self.assertEqual({row["plant_count"] for row in rows}, {2, 3})
        self.assertEqual({row["position_band"] for row in rows}, {"near", "middle", "far"})
        self.assertEqual({row["occlusion"] for row in rows}, {"none", "partial", "heavy"})
        self.assertFalse({row["seed"] for row in rows} & set(self.formal_seeds))

    def test_qualification_batches_advance_by_one_hundred(self):
        first = scenario_rows(self.config, split="qualification", batch_index=0)
        second = scenario_rows(self.config, split="qualification", batch_index=1)

        self.assertEqual(first[0]["seed"], 46001)
        self.assertEqual(second[0]["seed"], 46101)
        self.assertFalse(
            {row["seed"] for row in first} & {row["seed"] for row in second}
        )

    def test_first_five_eligible_are_selected_without_score_cherry_pick(self):
        rows = scenario_rows(self.config, split="qualification")
        eligible_indices = {0, 2, 4, 6, 8, 10}
        receipts = {
            row["scenario_id"]: {
                "eligible_for_multi_fruit_runtime": index in eligible_indices,
                "runtime_truth_use": False,
                "trajectory_execution_allowed": False,
            }
            for index, row in enumerate(rows)
        }

        selected = select_runtime_scenarios(rows, receipts, count=5)

        self.assertEqual(
            selected,
            tuple(rows[index]["scenario_id"] for index in (0, 2, 4, 6, 8)),
        )

    def test_truth_using_receipt_is_never_eligible(self):
        rows = scenario_rows(self.config, split="qualification")
        receipts = {
            rows[0]["scenario_id"]: {
                "eligible_for_multi_fruit_runtime": True,
                "runtime_truth_use": True,
                "trajectory_execution_allowed": False,
            }
        }
        self.assertEqual(select_runtime_scenarios(rows, receipts), ())

    def test_qualification_tools_freeze_code_and_materialized_scenes(self):
        sweep = (ROOT / "scripts/run_generalized_feasibility_sweep.py").read_text(
            encoding="utf-8"
        )
        runtime = (
            ROOT / "scripts/run_generalized_qualification_batch.py"
        ).read_text(encoding="utf-8")

        self.assertIn('options.split == "qualification" and git_status', sweep)
        self.assertIn('("scene", scene_path)', sweep)
        self.assertIn('("world", world_path)', sweep)
        self.assertIn('if _git(("status", "--porcelain"))', runtime)
        self.assertIn('or len(selected) != 5', runtime)
        self.assertIn("summarize_runtime_gate(scores)", runtime)


if __name__ == "__main__":
    unittest.main()
