import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE))

from strawberry_benchmark.generalized_acceptance import (  # noqa: E402
    summarize_results,
    validate_matrix,
    validate_results_payload,
)
from strawberry_benchmark.run_identity import (  # noqa: E402
    build_run_identity,
    sha256_file,
)


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
        cls.temporary = tempfile.TemporaryDirectory()
        cls.identity_root = Path(cls.temporary.name)
        (cls.identity_root / ".gitignore").write_text(
            "results.json\n", encoding="utf-8"
        )
        cls.manifest_payload = {
            "schema_version": 1,
            "matrix_id": cls.matrix["matrix_id"],
            "formal_held_out": True,
            "truth_for_runtime_control": False,
            "scenarios": [
                {
                    "scenario_id": scenario["scenario_id"],
                    "seed": scenario["seed"],
                    "scene_sha256": SCENE_SHA,
                    "world_sha256": WORLD_SHA,
                }
                for scenario in cls.matrix["scenarios"]
            ],
        }
        for name in (
            "model.bin",
            "configuration.json",
            "environment.json",
            "runner.py",
            "scorer.py",
        ):
            (cls.identity_root / name).write_text(f"{name}\n", encoding="utf-8")
        (cls.identity_root / "scene_manifest.json").write_text(
            json.dumps(cls.manifest_payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        (cls.identity_root / "protocol.json").write_text(
            json.dumps(cls.matrix, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        subprocess.run(["git", "init"], cwd=cls.identity_root, check=True)
        subprocess.run(
            ["git", "config", "user.name", "Acceptance Test"],
            cwd=cls.identity_root,
            check=True,
        )
        subprocess.run(
            ["git", "config", "user.email", "acceptance@example.invalid"],
            cwd=cls.identity_root,
            check=True,
        )
        subprocess.run(["git", "add", "."], cwd=cls.identity_root, check=True)
        subprocess.run(
            ["git", "commit", "-m", "fixture"],
            cwd=cls.identity_root,
            check=True,
        )
        cls.git_commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=cls.identity_root,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        cls.model_sha = sha256_file(cls.identity_root / "model.bin")
        cls.configuration_sha = sha256_file(
            cls.identity_root / "configuration.json"
        )
        cls.matrix_sha = sha256_file(cls.identity_root / "protocol.json")
        cls.manifest_sha = sha256_file(
            cls.identity_root / "scene_manifest.json"
        )

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def manifest(self):
        return self.manifest_payload

    def required_files(self):
        return {
            "scene_or_resource": self.identity_root / "scene_manifest.json",
            "scorer": self.identity_root / "scorer.py",
            "protocol": self.identity_root / "protocol.json",
            "result": self.identity_root / "results.json",
        }

    def perfect_payload(self):
        rows = []
        for scenario in self.matrix["scenarios"]:
            profile = scenario["profile"]
            positive = profile == "mixed"
            negative = profile == "all_unripe"
            visible = 2 if positive else (0 if negative else 1)
            reachable = 2 if positive else 0
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
                    "visible_ripe_truth_count": visible,
                    "ripe_true_positive_count": visible,
                    "ripe_prediction_count": visible,
                    "identity_switch_count": 0,
                    "association_count": 20,
                    "localization_errors_m": [0.01] * visible,
                    "accepted_sigmas_m": [0.01] * reachable,
                    "unripe_pick_count": 0,
                    "collision_count": 0,
                    "joint_limit_violation_count": 0,
                    "unsafe_motion_attempt_count": 0,
                    "reachable_ripe_truth_count": reachable,
                    "harvested_reachable_ripe_count": reachable,
                    "accepted_target_count": reachable,
                    "successful_accepted_target_count": reachable,
                    "scene_complete": positive,
                    "outcome": "SUCCESS" if positive else "NO_PICK",
                }
            )
        return {
            "schema_version": 2,
            "matrix_id": self.matrix["matrix_id"],
            "bindings": {
                "matrix_sha256": self.matrix_sha,
                "materialization_manifest_sha256": self.manifest_sha,
                "git_commit": self.git_commit,
                "model_sha256": self.model_sha,
                "runtime_config_sha256": self.configuration_sha,
            },
            "results": rows,
        }

    def identity_for(self, payload):
        result = self.identity_root / "results.json"
        result.write_text(json.dumps(payload), encoding="utf-8")
        return build_run_identity(
            repository_root=self.identity_root,
            run_id="generalized-formal-fixture",
            run_type="BEHAVIOR",
            purpose="validate generalized acceptance evidence",
            tree_state="clean",
            bindings={
                "model": self.identity_root / "model.bin",
                "configuration": self.identity_root / "configuration.json",
                "environment": self.identity_root / "environment.json",
                "scene_or_resource": self.identity_root / "scene_manifest.json",
                "runner": self.identity_root / "runner.py",
                "scorer": self.identity_root / "scorer.py",
                "protocol": self.identity_root / "protocol.json",
                "result": result,
            },
        )

    def summarize(self, payload, *, run_identity=None):
        if run_identity is None:
            run_identity = self.identity_for(payload)
        return summarize_results(
            self.matrix,
            self.manifest(),
            payload,
            matrix_sha256=self.matrix_sha,
            manifest_sha256=self.manifest_sha,
            run_identity=run_identity,
            repository_root=self.identity_root,
            required_identity_files=self.required_files(),
        )

    def test_frozen_matrix_has_required_distribution(self):
        scenarios = validate_matrix(self.matrix)
        self.assertEqual(len(scenarios), 30)

    def test_coherent_aggregate_metrics_remain_indeterminate_without_events(self):
        summary = self.summarize(self.perfect_payload())
        self.assertFalse(summary["overall_pass"])
        self.assertEqual(summary["overall_status"], "INDETERMINATE")
        self.assertTrue(
            all(
                passed
                for name, passed in summary["raw_threshold_checks"].items()
                if name != "evidence_integrity"
            )
        )
        self.assertTrue(
            all(
                status == "INDETERMINATE"
                for status in summary["gate_statuses"].values()
            )
        )
        self.assertEqual(summary["schema_version"], 3)

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
        payload["results"][negative_index]["outcome"] = "FAILED"

        summary = self.summarize(payload)

        self.assertFalse(summary["overall_pass"])
        self.assertFalse(summary["gates"]["collision_count"])
        self.assertFalse(summary["gates"]["unsafe_motion_attempt_count"])
        self.assertFalse(summary["gates"]["negative_safe_no_pick_rate"])
        self.assertEqual(summary["gate_statuses"]["collision_count"], "FAIL")
        self.assertEqual(summary["overall_status"], "FAIL")

    def test_incomplete_terminal_or_cleanup_fails_evidence_gate(self):
        payload = self.perfect_payload()
        payload["results"][0]["terminal_status_received"] = False
        payload["results"][1]["cleanup_outcome"] = "FAILED"
        payload["results"][2]["behavior_attempt_count"] = 0

        summary = self.summarize(payload)

        self.assertFalse(summary["gates"]["evidence_integrity"])
        self.assertAlmostEqual(
            summary["metrics"]["terminal_cleanup_receipt_rate"], 0.9
        )
        self.assertEqual(summary["metrics"]["complete_evidence_rate"], 0.0)
        self.assertEqual(
            summary["gate_statuses"]["evidence_integrity"], "INDETERMINATE"
        )

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
                matrix_sha256=self.matrix_sha,
                manifest_sha256=self.manifest_sha,
                run_identity=self.identity_for(payload),
                repository_root=self.identity_root,
                required_identity_files=self.required_files(),
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

    def test_contradictory_outcome_and_scene_completion_are_rejected(self):
        for outcome, scene_complete in (("FAILED", True), ("SUCCESS", False)):
            with self.subTest(outcome=outcome, scene_complete=scene_complete):
                payload = self.perfect_payload()
                payload["results"][0]["outcome"] = outcome
                payload["results"][0]["scene_complete"] = scene_complete
                with self.assertRaisesRegex(
                    ValueError, "outcome.*scene_complete|scene_complete.*outcome"
                ):
                    self.summarize(payload)

    def test_successful_accepted_count_must_match_harvested_count(self):
        payload = self.perfect_payload()
        payload["results"][0]["successful_accepted_target_count"] = 0
        with self.assertRaisesRegex(
            ValueError, "successful accepted.*harvested"
        ):
            self.summarize(payload)

    def test_localization_and_sigma_samples_cannot_be_selectively_omitted(self):
        for field in ("localization_errors_m", "accepted_sigmas_m"):
            with self.subTest(field=field):
                payload = self.perfect_payload()
                for row in payload["results"]:
                    row[field] = []
                payload["results"][0][field] = [0.001]
                with self.assertRaisesRegex(ValueError, "sample count"):
                    self.summarize(payload)

    def test_bare_or_syntactically_valid_unverified_identity_is_rejected(self):
        payload = self.perfect_payload()
        with self.assertRaisesRegex(ValueError, "verified run identity"):
            summarize_results(
                self.matrix,
                self.manifest(),
                payload,
                matrix_sha256=self.matrix_sha,
                manifest_sha256=self.manifest_sha,
            )

        for target in (
            "model",
            "configuration",
            "scene_or_resource",
            "scorer",
            "protocol",
            "result",
            "git",
        ):
            with self.subTest(target=target):
                identity = self.identity_for(payload)
                if target == "git":
                    identity["source"]["commit"] = "1" * 40
                else:
                    identity["bindings"][target]["sha256"] = "1" * 64
                with self.assertRaisesRegex(
                    ValueError, "run identity.*verify|identity.*binding"
                ):
                    self.summarize(payload, run_identity=identity)

    def test_exact_bound_files_are_mandatory(self):
        payload = self.perfect_payload()
        identity = self.identity_for(payload)
        with self.assertRaisesRegex(ValueError, "exact scene, scorer, protocol"):
            summarize_results(
                self.matrix,
                self.manifest(),
                payload,
                matrix_sha256=self.matrix_sha,
                manifest_sha256=self.manifest_sha,
                run_identity=identity,
                repository_root=self.identity_root,
                required_identity_files=None,
            )

    def test_bound_result_cannot_be_swapped_after_identity_capture(self):
        payload = self.perfect_payload()
        identity = self.identity_for(payload)
        changed = self.perfect_payload()
        changed["results"][0]["collision_count"] = 1
        with self.assertRaisesRegex(ValueError, "bound result content"):
            summarize_results(
                self.matrix,
                self.manifest(),
                changed,
                matrix_sha256=self.matrix_sha,
                manifest_sha256=self.manifest_sha,
                run_identity=identity,
                repository_root=self.identity_root,
                required_identity_files=self.required_files(),
            )


if __name__ == "__main__":
    unittest.main()
