import importlib.util
import json
import pathlib
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "scripts" / "summarize_t60_position_diagnostic.py"
MANIFEST = ROOT / "config" / "t60_oracle_shadow_position_diagnostic.json"
CLEARANCE_MANIFEST = (
    ROOT / "config" / "t60_oracle_shadow_multifruit_clearance_diagnostic.json"
)
SPEC = importlib.util.spec_from_file_location("summarize_t60_position_diagnostic", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class T60PositionDiagnosticSummaryTests(unittest.TestCase):
    def _write_trials(self, root: pathlib.Path, **overrides) -> None:
        manifest = MODULE.load_oracle_shadow_diagnostic_manifest(MANIFEST)
        for scenario in manifest.scenarios:
            payload = {
                "success": True,
                "source_isolated": True,
                "state_machine_complete": True,
                "position_configuration_applied": True,
                "configured_truth_position_m": list(scenario.target_position_m),
                "configured_control_position_m": list(scenario.target_position_m),
                "shadow_evidence_present": True,
                "shadow_detection_frames": 10,
                "shadow_detection_count": 20,
                "shadow_ripe_detection_count": 10,
                "shadow_target_observations": 9,
                "planning_time_sec": 0.1,
                "execution_time_sec": 1.0,
                "final_status": {"state": "DONE", "outcome": "SUCCESS"},
                **overrides,
            }
            (root / f"{scenario.scenario_id}.json").write_text(
                json.dumps(payload), encoding="utf-8"
            )
            (root / f"{scenario.scenario_id}_launch.log").write_text(
                "clean shutdown", encoding="utf-8"
            )

    def test_complete_diagnostic_never_claims_formal_acceptance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self._write_trials(root)
            result = MODULE.summarize(root, MANIFEST, base_domain_id=220)
        self.assertTrue(result["diagnostic_complete"])
        self.assertEqual(1.0, result["oracle_motion_success_rate"])
        self.assertEqual(1.0, result["shadow_target_scenario_rate"])
        self.assertFalse(result["formal_acceptance"])
        self.assertFalse(result["may_close_p3_gate"])
        self.assertFalse(result["held_out_test_consumed"])

    def test_motion_failure_is_a_valid_diagnostic_observation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self._write_trials(
                root,
                success=False,
                state_machine_complete=False,
                final_status={"state": "FAILED", "outcome": "FAILED"},
            )
            result = MODULE.summarize(root, MANIFEST)
        self.assertTrue(result["diagnostic_complete"])
        self.assertEqual(0.0, result["oracle_motion_success_rate"])

    def test_routing_leak_invalidates_diagnostic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self._write_trials(root, source_isolated=False)
            result = MODULE.summarize(root, MANIFEST)
        self.assertFalse(result["diagnostic_complete"])

    def test_client_setup_error_is_not_misattributed_to_collision(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self._write_trials(
                root,
                success=False,
                position_configuration_applied=False,
                error="Gazebo set_pose timed out",
            )
            for launch_path in root.glob("*_launch.log"):
                launch_path.write_text(
                    "MoveIt collision_object monitor started", encoding="utf-8"
                )
            result = MODULE.summarize(root, MANIFEST)
        self.assertFalse(result["diagnostic_complete"])
        self.assertEqual(
            {"INFRASTRUCTURE_FAILURE": 5}, result["failure_attribution_counts"]
        )
        self.assertTrue(
            all(not trial["collision_objects"] for trial in result["trials"])
        )

    def test_log_attribution_separates_neighbor_collision_from_ik(self):
        collision_log = (
            "Found a contact between 'strawberry_fruit_2' (type 'Object') "
            "and 'panda_leftfinger' (type 'Robot link')"
        ).lower()
        cause, objects = MODULE._diagnostic_failure_attribution(
            collision_log, motion_success=False
        )
        self.assertEqual("NON_TARGET_FRUIT_COLLISION", cause)
        self.assertEqual(["strawberry_fruit_2"], objects)
        cause, objects = MODULE._diagnostic_failure_attribution(
            "MoveIt IK failed at Cartesian waypoint 2/3".lower(),
            motion_success=False,
        )
        self.assertEqual("CARTESIAN_IK_FAILURE", cause)
        self.assertEqual([], objects)

    def test_clearance_summary_records_frozen_surface_gap_range(self):
        manifest = MODULE.load_oracle_shadow_diagnostic_manifest(CLEARANCE_MANIFEST)
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            for scenario in manifest.scenarios:
                payload = {
                    "success": True,
                    "source_isolated": True,
                    "state_machine_complete": True,
                    "position_configuration_applied": True,
                    "parked_model_configuration_applied": True,
                    "configured_truth_position_m": list(scenario.target_position_m),
                    "configured_control_position_m": list(scenario.target_position_m),
                    "configured_parked_model_poses": [
                        {
                            "model_name": model.model_name,
                            "target_id": model.target_id,
                            "position_m": list(model.position_m),
                        }
                        for model in manifest.parked_models
                    ],
                    "shadow_evidence_present": True,
                    "shadow_detection_frames": 10,
                    "shadow_detection_count": 20,
                    "shadow_ripe_detection_count": 10,
                    "shadow_target_observations": 9,
                    "final_status": {"state": "DONE", "outcome": "SUCCESS"},
                }
                (root / f"{scenario.scenario_id}.json").write_text(
                    json.dumps(payload), encoding="utf-8"
                )
                (root / f"{scenario.scenario_id}_launch.log").write_text(
                    "clean shutdown", encoding="utf-8"
                )
            result = MODULE.summarize(root, CLEARANCE_MANIFEST)
        self.assertTrue(result["diagnostic_complete"])
        self.assertAlmostEqual(
            0.03,
            result["requested_surface_clearance_range_m"]["minimum"],
            places=9,
        )
        self.assertGreater(
            result["requested_surface_clearance_range_m"]["maximum"], 0.13
        )


if __name__ == "__main__":
    unittest.main()
