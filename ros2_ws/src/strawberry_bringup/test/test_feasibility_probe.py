from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.feasibility_probe import (  # noqa: E402
    FreshMessageRetryGate,
    build_feasibility_payload,
    probe_exit_code,
    tracked_snapshot_priority,
)
from strawberry_bringup.target_selector import harvest_candidates_from_records  # noqa: E402


LIMITS = {
    "confidence_threshold": 0.20,
    "maximum_sigma_m": 0.015,
    "minimum_observations": 3,
    "maximum_age_sec": 0.50,
    "minimum_clearance_m": 0.02,
    "excluded_track_ids": set(),
}


def record(identity: int, maturity: int = 1):
    return {
        "track_id": identity,
        "maturity": maturity,
        "position": [0.45 + 0.12 * (identity - 1), 0.0, 0.64],
        "confidence": 0.9,
        "sigma_m": 0.01,
        "observation_count": 4,
        "last_seen_sec": 10.0,
    }


class FeasibilityProbeTests(unittest.TestCase):
    def test_snapshot_priority_keeps_richer_safe_inventory(self):
        rich = tracked_snapshot_priority(
            ranked_count=2, candidate_count=4, observations=20, sequence=5
        )
        transient_empty = tracked_snapshot_priority(
            ranked_count=0, candidate_count=0, observations=0, sequence=6
        )
        equally_rich_newer = tracked_snapshot_priority(
            ranked_count=2, candidate_count=4, observations=20, sequence=7
        )

        self.assertGreater(rich, transient_empty)
        self.assertGreater(equally_rich_newer, rich)

    def test_transient_retry_waits_for_a_new_perception_snapshot(self):
        gate = FreshMessageRetryGate(maximum_recoveries=3, wait_timeout_sec=5.0)

        self.assertTrue(gate.defer(message_sequence=8, now_sec=10.0))
        self.assertEqual(
            gate.decision(message_sequence=8, now_sec=10.5), "WAIT"
        )
        self.assertEqual(
            gate.decision(message_sequence=9, now_sec=10.6), "REFRESH"
        )
        self.assertEqual(
            gate.decision(message_sequence=9, now_sec=10.7), "READY"
        )

    def test_transient_retry_is_bounded_by_timeout_and_count(self):
        gate = FreshMessageRetryGate(maximum_recoveries=1, wait_timeout_sec=2.0)

        self.assertTrue(gate.defer(message_sequence=3, now_sec=4.0))
        self.assertEqual(
            gate.decision(message_sequence=3, now_sec=6.0), "EXHAUSTED"
        )
        self.assertFalse(gate.defer(message_sequence=3, now_sec=6.1))

    def test_runner_is_zero_motion_and_truth_audited(self):
        runner = (PACKAGE.parents[2] / "scripts" / "run_generalized_feasibility_probe.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn("harvest_control_enabled:=false", runner)
        self.assertIn("generalized_truth_isolation_audit", runner)
        self.assertIn("generalized_feasibility_probe", runner)
        self.assertNotIn("/strawberry/run_harvest", runner)

    def candidates(self, records):
        return harvest_candidates_from_records(
            records,
            fruit_radius_m=0.026,
            bin_bounds=(0.14, 0.56, -0.68, -0.22, 0.25, 0.55),
            static_obstacle_margin_m=0.05,
        )

    def test_two_moveit_feasible_ripe_tracks_are_eligible(self):
        records = (record(1), record(2), record(3, maturity=2))
        payload = build_feasibility_payload(
            outcome="COMPLETED",
            records=records,
            candidates=self.candidates(records),
            evaluations=[
                {"track_id": 1, "feasible": True},
                {"track_id": 2, "feasible": True},
            ],
            now_sec=10.0,
            selection_limits=LIMITS,
            elapsed_sec=20.0,
            minimum_feasible_ripe=2,
        )

        self.assertTrue(payload["eligible_for_multi_fruit_runtime"])
        self.assertFalse(payload["runtime_truth_use"])
        self.assertFalse(payload["trajectory_execution_allowed"])
        self.assertEqual(payload["transient_recoveries"], [])
        self.assertEqual(probe_exit_code(payload), 0)

    def test_one_feasible_track_is_not_eligible(self):
        records = (record(1), record(2))
        payload = build_feasibility_payload(
            outcome="COMPLETED",
            records=records,
            candidates=self.candidates(records),
            evaluations=[
                {"track_id": 1, "feasible": False},
                {"track_id": 2, "feasible": True},
            ],
            now_sec=10.0,
            selection_limits=LIMITS,
            elapsed_sec=20.0,
            minimum_feasible_ripe=2,
        )

        self.assertFalse(payload["eligible_for_multi_fruit_runtime"])
        self.assertEqual(probe_exit_code(payload), 1)

    def test_timeout_never_passes_even_with_partial_results(self):
        records = (record(1), record(2))
        payload = build_feasibility_payload(
            outcome="HARD_TIMEOUT",
            records=records,
            candidates=self.candidates(records),
            evaluations=[
                {"track_id": 1, "feasible": True},
                {"track_id": 2, "feasible": True},
            ],
            now_sec=10.0,
            selection_limits=LIMITS,
            elapsed_sec=240.0,
            minimum_feasible_ripe=2,
        )

        self.assertFalse(payload["eligible_for_multi_fruit_runtime"])
        self.assertEqual(probe_exit_code(payload), 3)


if __name__ == "__main__":
    unittest.main()
