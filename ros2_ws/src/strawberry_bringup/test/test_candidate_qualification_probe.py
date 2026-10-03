from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.candidate_qualification_probe import (  # noqa: E402
    StableRipeTrack,
    build_candidate_qualification_payload,
    candidate_qualification_probe_exit_code,
    place_position_from_mapping,
    stable_ripe_tracks,
)


def record(identity: int, *, confidence=0.9, sigma=0.01, observations=4, maturity=1):
    return {
        "track_id": identity,
        "maturity": maturity,
        "position": [0.40 + 0.01 * identity, 0.0, 0.62],
        "confidence": confidence,
        "sigma_m": sigma,
        "observation_count": observations,
        "last_seen_sec": 10.0,
    }


def valid_motion_events(target_id: int):
    candidate_ids = ["G00", "G01"]
    fingerprints = ["fp00", "fp01"]
    return [
        {
            "event_time_monotonic_ns": 10,
            "event_type": "CANDIDATE_QUALIFICATION_STARTED",
            "payload": {
                "target_id": target_id,
                "mode": "PLAN_ONLY",
                "live_payload_state": "EMPTY",
                "candidate_ids": candidate_ids,
                "candidate_geometry_fingerprints": fingerprints,
                "controller_commands_during_evaluation": 0,
                "gripper_commands_during_evaluation": 0,
                "physical_attach_during_evaluation": 0,
            },
        },
        {
            "event_time_monotonic_ns": 20,
            "event_type": "CANDIDATE_EVALUATION_RESULT",
            "payload": {
                "target_id": target_id,
                "candidate_id": "G00",
                "geometry_fingerprint": "fp00",
                "feasible": False,
            },
        },
        {
            "event_time_monotonic_ns": 30,
            "event_type": "CANDIDATE_EVALUATION_RESULT",
            "payload": {
                "target_id": target_id,
                "candidate_id": "G01",
                "geometry_fingerprint": "fp01",
                "feasible": True,
            },
        },
        {
            "event_time_monotonic_ns": 40,
            "event_type": "CANDIDATE_QUALIFICATION_RESULT",
            "payload": {
                "target_id": target_id,
                "mode": "PLAN_ONLY",
                "live_payload_state": "EMPTY",
                "status": "PLAN_ONLY_CERTIFIED",
                "feasible": True,
                "scene_isolated": True,
                "selected_candidate_id": "G01",
                "geometry_fingerprint": "fp01",
                "certificate_fingerprint": "certificate",
                "execution_dispatched": False,
                "controller_commands_during_evaluation": 0,
                "gripper_commands_during_evaluation": 0,
                "physical_attach_during_evaluation": 0,
            },
        },
        {
            "event_time_monotonic_ns": 50,
            "event_type": "TARGET_COLLISION_RESTORED",
            "payload": {"target_id": target_id, "restored": True},
        },
        {
            "event_time_monotonic_ns": 60,
            "event_type": "CANDIDATE_QUALIFICATION_CLEANUP",
            "payload": {
                "target_id": target_id,
                "target_collision_restored": True,
                "execution_dispatched": False,
            },
        },
    ]


class CandidateQualificationProbeTests(unittest.TestCase):
    def test_perception_selection_is_ripe_stable_and_deterministic(self):
        selected = stable_ripe_tracks(
            (
                record(3, confidence=0.99, maturity=2),
                record(2, confidence=0.8, sigma=0.012),
                record(1, confidence=0.8, sigma=0.010),
                record(4, observations=2),
            )
        )

        self.assertEqual([item.track_id for item in selected], [1, 2])
        self.assertEqual(
            [
                item.track_id
                for item in stable_ripe_tracks(
                    (record(1), record(2)), requested_target_id=2
                )
            ],
            [2],
        )

    def test_scene_place_position_is_finite_and_explicit(self):
        self.assertEqual(
            place_position_from_mapping(
                {"bin": {"place_pose_m": [0.35, -0.45, 0.45]}}
            ),
            (0.35, -0.45, 0.45),
        )
        with self.assertRaises(ValueError):
            place_position_from_mapping({"bin": {"place_pose_m": [0.35, 0.45]}})

    def test_complete_receipt_requires_evidence_and_never_allows_execution(self):
        target = StableRipeTrack(7, (0.45, 0.0, 0.62), 0.9, 0.01, 4, 10.0)
        payload = build_candidate_qualification_payload(
            outcome="PLAN_ONLY_CERTIFIED",
            target=target,
            response={"status": "PLAN_ONLY_CERTIFIED", "feasible": True},
            motion_events=valid_motion_events(target.track_id),
            elapsed_wall_sec=8.0,
            scene_config_file=Path("scene.yaml"),
            place_position_m=(0.35, -0.45, 0.45),
            timeouts_sec={"hard": 180.0},
        )

        self.assertTrue(payload["receipt_validation"]["passed"])
        self.assertFalse(payload["trajectory_execution_allowed"])
        self.assertFalse(payload["execution_dispatched"])
        self.assertEqual(candidate_qualification_probe_exit_code(payload), 0)

    def test_invalid_evidence_cannot_pass_a_nominally_feasible_response(self):
        target = StableRipeTrack(7, (0.45, 0.0, 0.62), 0.9, 0.01, 4, 10.0)
        events = valid_motion_events(target.track_id)
        events[4]["event_type"] = "TARGET_COLLISION_RESTORE_FAILED"
        payload = build_candidate_qualification_payload(
            outcome="PLAN_ONLY_CERTIFIED",
            target=target,
            response={"status": "PLAN_ONLY_CERTIFIED", "feasible": True},
            motion_events=events,
            elapsed_wall_sec=8.0,
            scene_config_file=Path("scene.yaml"),
            place_position_m=(0.35, -0.45, 0.45),
            timeouts_sec={"hard": 180.0},
        )

        self.assertFalse(payload["receipt_validation"]["passed"])
        self.assertEqual(candidate_qualification_probe_exit_code(payload), 4)

    def test_runner_keeps_harvest_control_disabled(self):
        runner = (
            PACKAGE.parents[2] / "scripts" / "run_generalized_candidate_qualification.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("harvest_control_enabled:=false", runner)
        self.assertIn("generalized_candidate_qualification_probe", runner)
        self.assertIn("generalized_truth_isolation_audit", runner)
        self.assertIn(
            "STRAWBERRY_DEVELOPMENT_CANDIDATE_CHALLENGE_OBSTACLE_SPEC", runner
        )
        self.assertIn(
            "development_candidate_challenge_obstacle_spec", runner
        )
        self.assertIn("STRAWBERRY_CANDIDATE_QUALIFICATION_WORLD_FILE", runner)
        self.assertNotIn("/strawberry/run_harvest", runner)


if __name__ == "__main__":
    unittest.main()
