import json
from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.feasibility_sweep import (  # noqa: E402
    observability_resolution_matches,
    parse_base_camera_resolution,
    observability_receipt_is_eligible,
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
        cls.config_v2 = json.loads(
            (
                ROOT / "config" / "generalized_runtime_development_matrix_v2.json"
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

    def test_v2_discovery_and_qualification_advance_without_reusing_v1_seeds(self):
        validate_development_matrix(self.config_v2, formal_seeds=self.formal_seeds)
        discovery = scenario_rows(self.config_v2, split="discovery", batch_index=0)
        next_discovery = scenario_rows(
            self.config_v2, split="discovery", batch_index=1
        )
        qualification = scenario_rows(
            self.config_v2, split="qualification", batch_index=0
        )
        next_qualification = scenario_rows(
            self.config_v2, split="qualification", batch_index=1
        )

        self.assertEqual(discovery[0]["seed"], 45101)
        self.assertEqual(next_discovery[0]["seed"], 45201)
        self.assertEqual(qualification[0]["seed"], 46101)
        self.assertEqual(next_qualification[0]["seed"], 46201)
        self.assertTrue(
            all(row["layout_contract"] == "multi_pick_v2" for row in discovery)
        )
        self.assertFalse(
            {row["seed"] for row in discovery}
            & {row["seed"] for row in qualification}
        )

    def test_v2_generation_contract_never_claims_moveit_feasibility(self):
        generation = self.config_v2["generation_contract"]
        self.assertEqual(generation["minimum_primary_ripe_by_construction"], 2)
        self.assertFalse(generation["moveit_feasibility_guaranteed_by_generator"])
        self.assertFalse(generation["runtime_truth_use"])
        eligibility = self.config_v2["zero_motion_eligibility"]
        self.assertEqual(eligibility["minimum_depth_visible_ripe_truth"], 2)
        self.assertEqual(
            eligibility["minimum_correctly_localized_ripe_truth"], 2
        )
        self.assertEqual(eligibility["maximum_false_ripe_localizations"], 0)

    def test_base_camera_resolution_is_restricted_and_receipt_bound(self):
        self.assertEqual(parse_base_camera_resolution("320x240"), (320, 240))
        self.assertEqual(parse_base_camera_resolution("640x480"), (640, 480))
        with self.assertRaisesRegex(ValueError, "must be one of"):
            parse_base_camera_resolution("800x600")
        self.assertTrue(
            observability_resolution_matches(
                {"image_shape_hw": [480, 640]}, "640x480"
            )
        )
        self.assertFalse(
            observability_resolution_matches(
                {"image_shape_hw": [240, 320]}, "640x480"
            )
        )
        self.assertFalse(
            observability_resolution_matches(
                {"image_shape_hw": ["bad", 640]}, "640x480"
            )
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

    def test_v2_selection_requires_two_visible_correctly_localized_ripe_targets(self):
        rows = scenario_rows(self.config_v2, split="qualification")
        feasibility = {
            row["scenario_id"]: {
                "eligible_for_multi_fruit_runtime": True,
                "runtime_truth_use": False,
                "trajectory_execution_allowed": False,
            }
            for row in rows[:2]
        }

        def observability(localized, accepted=None, true=None):
            accepted = localized if accepted is None else accepted
            true = localized if true is None else true
            return {
                "schema_version": 3,
                "kind": "generalized_rgbd_frame_diagnostic",
                "runtime_truth_use": False,
                "commands_published": 0,
                "observability_metrics": {
                    "visible_ripe_truth_count": 2,
                    "localized_ripe_truth_count": localized,
                    "accepted_ripe_prediction_count": accepted,
                    "true_ripe_prediction_count": true,
                },
            }

        observations = {
            rows[0]["scenario_id"]: observability(1),
            rows[1]["scenario_id"]: observability(2),
        }
        self.assertEqual(
            select_runtime_scenarios(
                rows,
                feasibility,
                observability_receipts=observations,
                count=1,
            ),
            (rows[1]["scenario_id"],),
        )
        self.assertFalse(observability_receipt_is_eligible(observability(2, 3, 2)))
        self.assertFalse(observability_receipt_is_eligible(observability(3, 3, 3)))
        self.assertFalse(observability_receipt_is_eligible(observability(2, -1, -1)))

    def test_qualification_tools_freeze_code_and_materialized_scenes(self):
        sweep = (ROOT / "scripts/run_generalized_feasibility_sweep.py").read_text(
            encoding="utf-8"
        )
        runtime = (
            ROOT / "scripts/run_generalized_qualification_batch.py"
        ).read_text(encoding="utf-8")

        self.assertIn('options.split == "qualification" and git_status', sweep)
        self.assertIn(
            'default=ROOT / "config" / "generalized_runtime_development_matrix_v2.json"',
            sweep,
        )
        self.assertIn('("scene", scene_path)', sweep)
        self.assertIn('("world", world_path)', sweep)
        self.assertIn('str(row["layout_contract"])', sweep)
        self.assertIn('"STRAWBERRY_BASE_CAMERA_RESOLUTION"', sweep)
        self.assertIn('"base_camera_resolution": options.base_camera_resolution', sweep)
        self.assertIn('if _git(("status", "--porcelain"))', runtime)
        self.assertIn('or len(selected) != 5', runtime)
        self.assertIn("summarize_runtime_gate(scores)", runtime)
        self.assertIn(
            'environment["STRAWBERRY_BASE_CAMERA_RESOLUTION"] = base_camera_resolution',
            runtime,
        )
        self.assertIn(
            'row.get("camera_resolution_matches_requested") is True', runtime
        )
        self.assertIn("_verify_receipt(binding, expected_path)", runtime)
        self.assertIn(
            '"config/generalized_runtime_development_matrix_v2.json"', runtime
        )

        development_probe = (
            ROOT / "scripts/run_generalized_development_probe.sh"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'base_camera_resolution="${STRAWBERRY_BASE_CAMERA_RESOLUTION:-320x240}"',
            development_probe,
        )
        self.assertIn(
            'base_camera_resolution:="${base_camera_resolution}"', development_probe
        )


if __name__ == "__main__":
    unittest.main()
