import pathlib
import sys
import tempfile
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_bringup.candidate_execution_probe import (  # noqa: E402
    build_candidate_execution_payload,
    candidate_execution_probe_exit_code,
)
from strawberry_bringup.candidate_qualification_probe import StableRipeTrack  # noqa: E402


def event(event_type, timestamp, payload):
    return {
        "event_type": event_type,
        "event_time_monotonic_ns": timestamp,
        "payload": payload,
    }


def valid_events(target_id=7):
    return [
        event("ADAPTIVE_CANDIDATE_EXECUTION_STARTED", 10, {
            "target_id": target_id,
            "live_payload_state": "EMPTY",
            "controller_commands_before_authorization": 0,
            "gripper_commands_before_authorization": 0,
            "physical_attach_before_authorization": 0,
        }),
        event("CANDIDATE_QUALIFICATION_STARTED", 20, {
            "target_id": target_id,
            "mode": "PRE_EXECUTION_QUALIFICATION",
            "live_payload_state": "EMPTY",
            "controller_commands_during_evaluation": 0,
            "gripper_commands_during_evaluation": 0,
            "physical_attach_during_evaluation": 0,
        }),
        event("CANDIDATE_EVALUATION_RESULT", 30, {
            "target_id": target_id, "candidate_id": "G00",
            "geometry_fingerprint": "fp0", "result": "APPROACH_FAILED",
            "feasible": False,
        }),
        event("CANDIDATE_EVALUATION_RESULT", 40, {
            "target_id": target_id, "candidate_id": "G02",
            "geometry_fingerprint": "fp2", "result": "FEASIBLE", "feasible": True,
        }),
        event("CANDIDATE_QUALIFICATION_RESULT", 50, {
            "target_id": target_id, "mode": "PRE_EXECUTION_QUALIFICATION",
            "status": "PLAN_ONLY_CERTIFIED", "feasible": True,
            "scene_isolated": True, "selected_candidate_id": "G02",
            "geometry_fingerprint": "fp2", "certificate_fingerprint": "cert2",
            "scene_signature_before": "scene2", "execution_dispatched": False,
        }),
        event("TARGET_COLLISION_RESTORED", 60, {
            "target_id": target_id, "restored": True,
        }),
        event("ADAPTIVE_CANDIDATE_QUALIFICATION_CLEANUP", 70, {
            "target_id": target_id, "target_collision_restored": True,
            "execution_dispatched": False,
        }),
        event("AUTHORIZED_CANDIDATE_EXECUTION_DISPATCHED", 80, {
            "target_id": target_id, "candidate_id": "G02",
            "geometry_fingerprint": "fp2", "certificate_fingerprint": "cert2",
            "scene_signature": "scene2", "execution_dispatched": True,
        }),
        event("COMMAND_PREPARED", 90, {"command_id": "arm-1"}),
        event("AUTHORIZED_CANDIDATE_EXECUTION_RESULT", 100, {
            "target_id": target_id, "candidate_id": "G02",
            "geometry_fingerprint": "fp2", "certificate_fingerprint": "cert2",
            "success": True,
        }),
    ]


class CandidateExecutionProbeTests(unittest.TestCase):
    def test_runner_enables_only_the_bounded_adaptive_action(self):
        runner = (
            PACKAGE_ROOT.parents[2]
            / "scripts"
            / "run_generalized_candidate_execution.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("harvest_control_enabled:=false", runner)
        self.assertIn("adaptive_candidate_execution_enabled:=true", runner)
        self.assertIn("STRAWBERRY_DEVELOPMENT_CANDIDATE_CHALLENGE_OBSTACLE_SPEC", runner)
        self.assertIn("generalized_candidate_execution_probe", runner)
        self.assertIn("--target-id", runner)
        self.assertNotIn("/strawberry/run_harvest", runner)

    def test_builds_valid_nonformal_physical_challenge_receipt(self):
        target = StableRipeTrack(7, (0.4, 0.1, 0.5), 0.9, 0.005, 6, 1.0)
        with tempfile.TemporaryDirectory() as temporary:
            world = pathlib.Path(temporary) / "challenge.sdf"
            world.write_text("<sdf version='1.10'/>", encoding="utf-8")
            payload = build_candidate_execution_payload(
                outcome="ACTION_SUCCEEDED",
                target=target,
                action_result={"success": True, "action_status": 4},
                feedback=[{"stage": "AUTHORIZED_G02", "progress": 0.08}],
                motion_events=valid_events(),
                elapsed_wall_sec=12.5,
                scene_config_file=pathlib.Path("scene.yaml"),
                world_file=world,
                challenge_obstacle_spec="0.2871 0.1120 0.6091 0.002 0.002 0.002",
                git_commit="deadbeef",
                place_position_m=(0.35, -0.45, 0.45),
                timeouts_sec={"action": 480.0},
            )

        self.assertTrue(payload["receipt_validation"]["passed"])
        self.assertFalse(payload["runtime_truth_use"])
        self.assertTrue(payload["trajectory_execution_allowed"])
        self.assertTrue(payload["world_file_sha256"])
        self.assertEqual(candidate_execution_probe_exit_code(payload), 0)

    def test_no_target_is_not_a_successful_execution(self):
        with tempfile.TemporaryDirectory() as temporary:
            world = pathlib.Path(temporary) / "challenge.sdf"
            world.write_text("<sdf version='1.10'/>", encoding="utf-8")
            payload = build_candidate_execution_payload(
                outcome="NO_STABLE_RIPE_TARGET",
                target=None,
                action_result=None,
                feedback=[],
                motion_events=[],
                elapsed_wall_sec=5.0,
                scene_config_file=pathlib.Path("scene.yaml"),
                world_file=world,
                challenge_obstacle_spec="0.2871 0.1120 0.6091 0.002 0.002 0.002",
                git_commit="deadbeef",
                place_position_m=(0.35, -0.45, 0.45),
                timeouts_sec={"action": 480.0},
            )

        self.assertFalse(payload["receipt_validation"]["passed"])
        self.assertEqual(candidate_execution_probe_exit_code(payload), 1)


if __name__ == "__main__":
    unittest.main()
