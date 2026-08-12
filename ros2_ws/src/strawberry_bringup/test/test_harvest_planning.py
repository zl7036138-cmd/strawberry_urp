from pathlib import Path
import math
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.harvest_planning import (  # noqa: E402
    HarvestCandidate,
    ViewAssessment,
    fuse_position_estimates,
    generate_dynamic_views,
    hand_pose_for_optical_view,
    rank_dynamic_views,
    rank_safe_targets,
    select_dynamic_view,
    target_rejection_reasons,
    wrist_refinement_rejection_reason,
    wrist_refinement_diagnostics,
)


def candidate(identity, **overrides):
    values = dict(
        track_id=identity,
        maturity=1,
        position=(0.50, 0.0, 0.55),
        confidence=0.90,
        sigma_m=0.010,
        observation_count=4,
        last_seen_sec=10.0,
        clearance_m=0.05,
        joint_travel_rad=1.0,
        pregrasp_feasible=True,
        grasp_feasible=True,
        retreat_feasible=True,
    )
    values.update(overrides)
    return HarvestCandidate(**values)


class HarvestPlanningTests(unittest.TestCase):
    def test_filter_is_fail_closed_for_every_safety_gate(self):
        rows = [
            candidate(1),
            candidate(2, maturity=2),
            candidate(3, sigma_m=0.02),
            candidate(4, observation_count=2),
            candidate(5, last_seen_sec=9.0),
            candidate(6, clearance_m=0.01),
            candidate(7, grasp_feasible=False),
        ]
        ranked = rank_safe_targets(rows, now_sec=10.1)
        self.assertEqual([row.track_id for row in ranked], [1])

    def test_rank_prioritizes_clearance_then_uncertainty_and_confidence(self):
        rows = [
            candidate(1, clearance_m=0.04, sigma_m=0.005),
            candidate(2, clearance_m=0.06, sigma_m=0.014),
            candidate(3, clearance_m=0.06, sigma_m=0.008, confidence=0.80),
            candidate(4, clearance_m=0.06, sigma_m=0.008, confidence=0.95),
        ]
        self.assertEqual(
            [row.track_id for row in rank_safe_targets(rows, now_sec=10.1)],
            [4, 3, 2, 1],
        )

    def test_rejection_reasons_report_every_failed_gate(self):
        row = candidate(
            3,
            maturity=2,
            confidence=0.40,
            sigma_m=0.020,
            observation_count=2,
            last_seen_sec=9.0,
            clearance_m=0.01,
            grasp_feasible=False,
        )

        self.assertEqual(
            target_rejection_reasons(
                row,
                now_sec=10.1,
                excluded_track_ids={3},
            ),
            (
                "EXCLUDED",
                "NOT_RIPE",
                "LOW_CONFIDENCE",
                "HIGH_UNCERTAINTY",
                "INSUFFICIENT_OBSERVATIONS",
                "STALE_OR_FUTURE_DATA",
                "LOW_CLEARANCE",
                "OUTSIDE_CONSERVATIVE_REACH",
            ),
        )

    def test_dynamic_view_bank_is_finite_normalized_and_points_at_target(self):
        views = generate_dynamic_views(4, (0.50, 0.0, 0.55))
        self.assertEqual(len(views), 12)
        for view in views:
            self.assertAlmostEqual(
                math.sqrt(sum(value * value for value in view.quaternion_xyzw)), 1.0
            )
            self.assertGreater(view.position[2], 0.55)

    def test_optical_view_is_converted_to_the_urdf_hand_frame(self):
        view = generate_dynamic_views(4, (0.50, 0.0, 0.55))[0]
        hand = hand_pose_for_optical_view(view)
        self.assertEqual(hand.target_id, view.target_id)
        self.assertAlmostEqual(
            math.sqrt(sum(value * value for value in hand.quaternion_xyzw)), 1.0
        )
        self.assertNotEqual(hand.position, view.position)

    def test_view_selection_rejects_unsafe_and_is_deterministic(self):
        views = generate_dynamic_views(
            1,
            (0.50, 0.0, 0.55),
            distances_m=(0.22,),
            elevations_deg=(15,),
            azimuths_deg=(-35, 0, 35),
        )

        def evaluator(view):
            feasible = view.azimuth_rad >= 0.0
            return ViewAssessment(feasible, feasible, True, 0.04, abs(view.azimuth_rad))

        selected = select_dynamic_view(views, evaluator)
        self.assertIsNotNone(selected)
        self.assertAlmostEqual(selected[0].azimuth_rad, 0.0)
        ranked = rank_dynamic_views(views, evaluator)
        self.assertEqual(len(ranked), 2)
        self.assertAlmostEqual(ranked[0][0].azimuth_rad, 0.0)
        self.assertGreater(ranked[1][0].azimuth_rad, 0.0)

    def test_wrist_fusion_accounts_for_calibrated_systematic_error(self):
        fused, sigma = fuse_position_estimates(
            (0.38, -0.26, 0.55),
            (0.384, -0.278, 0.556),
            base_sigma_m=0.010,
            wrist_sigma_m=0.005,
            wrist_systematic_sigma_m=0.030,
        )
        self.assertLess(math.dist(fused, (0.38, -0.26, 0.55)), 0.003)
        self.assertLess(sigma, 0.010)

    def test_wrist_confirmation_requires_matching_base_track_identity(self):
        common = {
            "expected_target_id": 2,
            "correction_m": 0.008,
            "confidence": 0.90,
            "sigma_m": 0.007,
            "maximum_correction_m": 0.05,
            "minimum_confidence": 0.60,
            "maximum_sigma_m": 0.015,
        }
        self.assertEqual(
            "TARGET_ID_MISMATCH",
            wrist_refinement_rejection_reason(observed_target_id=5, **common),
        )
        self.assertIsNone(
            wrist_refinement_rejection_reason(observed_target_id=2, **common)
        )

    def test_wrist_diagnostics_preserve_measurements_and_thresholds(self):
        diagnostics = wrist_refinement_diagnostics(
            expected_target_id=5,
            observed_target_id=5,
            correction_m=0.012,
            confidence=0.91,
            sigma_m=0.018,
            maximum_correction_m=0.05,
            minimum_confidence=0.60,
            maximum_sigma_m=0.015,
        )

        self.assertEqual(diagnostics["gate_result"], "HIGH_UNCERTAINTY")
        self.assertEqual(diagnostics["expected_target_id"], 5)
        self.assertAlmostEqual(diagnostics["sigma_m"], 0.018)
        self.assertAlmostEqual(diagnostics["maximum_sigma_m"], 0.015)


if __name__ == "__main__":
    unittest.main()
