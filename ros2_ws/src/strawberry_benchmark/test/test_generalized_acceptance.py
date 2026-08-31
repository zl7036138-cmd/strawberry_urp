import json
from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE))

from strawberry_benchmark.generalized_acceptance import (  # noqa: E402
    summarize_results,
    validate_matrix,
    validate_results_payload,
)


MATRIX_SHA = "a" * 64
MANIFEST_SHA = "b" * 64
SCENE_SHA = "c" * 64
WORLD_SHA = "d" * 64


class GeneralizedAcceptanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.matrix = json.loads(
            (
                REPOSITORY / "config" / "generalized_harvest_matrix_v1.json"
            ).read_text(encoding="utf-8")
        )

    def manifest(self):
        return {
            "schema_version": 1,
            "matrix_id": self.matrix["matrix_id"],
            "formal_held_out": True,
            "truth_for_runtime_control": False,
            "scenarios": [
                {
                    "scenario_id": scenario["scenario_id"],
                    "seed": scenario["seed"],
                    "scene_sha256": SCENE_SHA,
                    "world_sha256": WORLD_SHA,
                }
                for scenario in self.matrix["scenarios"]
            ],
        }

    def perfect_payload(self):
        rows = []
        for scenario in self.matrix["scenarios"]:
            profile = scenario["profile"]
            positive = profile == "mixed"
            negative = profile == "all_unripe"
            reachable = 2 if positive else (1 if profile == "unsafe" else 0)
            rows.append(
                {
                    "schema_version": 2,
                    "scenario_id": scenario["scenario_id"],
                    "seed": scenario["seed"],
                    "profile": profile,
                    "scene_sha256": SCENE_SHA,
                    "world_sha256": WORLD_SHA,
                    "behavior_attempt_count": 1,
                    "terminal_status_received": True,
                    "cleanup_outcome": "CLEAN",
                    "visible_ripe_truth_count": 2 if positive else (1 if profile == "unsafe" else 0),
                    "ripe_true_positive_count": 2 if positive else (1 if profile == "unsafe" else 0),
                    "ripe_prediction_count": 2 if positive else (1 if profile == "unsafe" else 0),
                    "identity_switch_count": 0,
                    "association_count": 20,
                    "localization_errors_m": [0.01, 0.02] if reachable else [],
                    "accepted_sigmas_m": [0.01, 0.012] if reachable else [],
                    "unripe_pick_count": 0,
                    "collision_count": 0,
                    "joint_limit_violation_count": 0,
                    "unsafe_motion_attempt_count": 0,
                    "reachable_ripe_truth_count": reachable,
                    "harvested_reachable_ripe_count": reachable,
                    "accepted_target_count": reachable,
                    "successful_accepted_target_count": reachable,
                    "scene_complete": positive,
                    "outcome": "NO_PICK" if negative else "SUCCESS",
                }
            )
        return {
            "schema_version": 2,
            "matrix_id": self.matrix["matrix_id"],
            "bindings": {
                "matrix_sha256": MATRIX_SHA,
                "materialization_manifest_sha256": MANIFEST_SHA,
                "git_commit": "e" * 40,
                "model_sha256": "f" * 64,
                "runtime_config_sha256": "0" * 64,
            },
            "results": rows,
        }

    def summarize(self, payload):
        return summarize_results(
            self.matrix,
            self.manifest(),
            payload,
            matrix_sha256=MATRIX_SHA,
            manifest_sha256=MANIFEST_SHA,
        )

    def test_frozen_matrix_has_required_distribution(self):
        scenarios = validate_matrix(self.matrix)
        self.assertEqual(len(scenarios), 30)

    def test_perfect_formal_results_pass_every_gate(self):
        summary = self.summarize(self.perfect_payload())
        self.assertTrue(summary["overall_pass"])
        self.assertTrue(all(summary["gates"].values()))
        self.assertEqual(summary["schema_version"], 2)

    def test_collision_wrong_negative_and_unsafe_motion_fail(self):
        payload = self.perfect_payload()
        payload["results"][0]["collision_count"] = 1
        unsafe_index = next(
            index
            for index, scenario in enumerate(self.matrix["scenarios"])
            if scenario["profile"] == "unsafe"
        )
        payload["results"][unsafe_index]["unsafe_motion_attempt_count"] = 1
        negative_index = next(
            index
            for index, scenario in enumerate(self.matrix["scenarios"])
            if scenario["profile"] == "all_unripe"
        )
        payload["results"][negative_index]["outcome"] = "SUCCESS"

        summary = self.summarize(payload)

        self.assertFalse(summary["overall_pass"])
        self.assertFalse(summary["gates"]["collision_count"])
        self.assertFalse(summary["gates"]["unsafe_motion_attempt_count"])
        self.assertFalse(summary["gates"]["negative_safe_no_pick_rate"])

    def test_incomplete_terminal_or_cleanup_fails_evidence_gate(self):
        payload = self.perfect_payload()
        payload["results"][0]["terminal_status_received"] = False
        payload["results"][1]["cleanup_outcome"] = "FAILED"
        payload["results"][2]["behavior_attempt_count"] = 0

        summary = self.summarize(payload)

        self.assertFalse(summary["gates"]["evidence_integrity"])
        self.assertAlmostEqual(summary["metrics"]["complete_evidence_rate"], 0.9)

    def test_missing_or_duplicate_scenario_fails_closed(self):
        payload = self.perfect_payload()
        payload["results"][-1] = dict(payload["results"][0])
        with self.assertRaises(ValueError):
            self.summarize(payload)

    def test_binding_mismatch_fails_closed(self):
        payload = self.perfect_payload()
        payload["bindings"]["matrix_sha256"] = "1" * 64
        with self.assertRaisesRegex(ValueError, "matrix hash mismatch"):
            self.summarize(payload)

    def test_second_behavior_attempt_is_rejected(self):
        payload = self.perfect_payload()
        payload["results"][0]["behavior_attempt_count"] = 2
        with self.assertRaisesRegex(ValueError, "more than one behavior attempt"):
            validate_results_payload(
                self.matrix,
                self.manifest(),
                payload,
                matrix_sha256=MATRIX_SHA,
                manifest_sha256=MANIFEST_SHA,
            )

    def test_inconsistent_counts_and_nonfinite_values_are_rejected(self):
        payload = self.perfect_payload()
        payload["results"][0]["ripe_true_positive_count"] = 3
        with self.assertRaisesRegex(ValueError, "true positives exceed"):
            self.summarize(payload)
        payload = self.perfect_payload()
        payload["results"][0]["localization_errors_m"] = [float("nan")]
        with self.assertRaisesRegex(ValueError, "finite and non-negative"):
            self.summarize(payload)


if __name__ == "__main__":
    unittest.main()
