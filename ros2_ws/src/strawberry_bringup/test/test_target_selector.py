from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from strawberry_bringup.harvest_planning import (  # noqa: E402
    HarvestCandidate,
    generate_dynamic_views,
)
from strawberry_bringup.target_selector import (  # noqa: E402
    MoveItEvaluation,
    MoveItSelectionBatch,
    apply_moveit_evaluation,
    axis_aligned_box_clearance,
    evaluation_failure_is_transient,
    evaluation_matches_position,
    expanded_bin_bounds,
    harvest_candidates_from_records,
    inside_conservative_reach,
    inside_observation_reach,
    moveit_priority_group,
    register_completed_track,
)


def candidate(identity=1, **overrides):
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


class TargetSelectorWorkspaceTests(unittest.TestCase):
    @staticmethod
    def selection_limits():
        return {
            "confidence_threshold": 0.60,
            "maximum_sigma_m": 0.015,
            "minimum_observations": 3,
            "maximum_age_sec": 0.50,
            "minimum_clearance_m": 0.02,
            "excluded_track_ids": set(),
        }

    def test_async_batch_rejects_first_and_selects_second(self):
        first = candidate(1)
        second = candidate(2)
        batch = MoveItSelectionBatch(
            epoch=7,
            now_sec=10.0,
            candidates=(first, second),
            ranked=(first, second),
            messages={1: object(), 2: object()},
            selection_limits=self.selection_limits(),
        )

        step, row = batch.next_step(7)
        self.assertEqual((step, row.track_id), ("EVALUATE", 1))
        self.assertTrue(
            batch.record(
                7,
                row,
                MoveItEvaluation(row.position, False, True, 0.2, 1.1, "blocked"),
                cached=False,
            )
        )
        step, row = batch.next_step(7)
        self.assertEqual((step, row.track_id), ("EVALUATE", 2))
        self.assertTrue(
            batch.record(
                7,
                row,
                MoveItEvaluation(row.position, True, False, 0.3, 1.3, "feasible"),
                cached=False,
            )
        )
        self.assertEqual(batch.next_step(7), ("SELECT", None))
        self.assertEqual(batch.selected().track_id, 2)
        self.assertEqual(batch.evaluated_track_ids, [1, 2])
        self.assertEqual(batch.moveit_rejections[0]["track_id"], 1)

    def test_async_batch_all_infeasible_finishes_no_pick(self):
        rows = (candidate(1), candidate(2))
        batch = MoveItSelectionBatch(
            epoch=2,
            now_sec=10.0,
            candidates=rows,
            ranked=rows,
            messages={1: object(), 2: object()},
            selection_limits=self.selection_limits(),
        )
        for expected in rows:
            step, row = batch.next_step(2)
            self.assertEqual((step, row.track_id), ("EVALUATE", expected.track_id))
            self.assertTrue(
                batch.record(
                    2,
                    row,
                    MoveItEvaluation(row.position, False, True, 0.1, 0.8, "blocked"),
                    cached=False,
                )
            )
        self.assertEqual(batch.next_step(2), ("NO_PICK", None))
        self.assertIsNone(batch.selected())

    def test_async_batch_rejects_stale_epoch_and_identity(self):
        row = candidate(3)
        batch = MoveItSelectionBatch(
            epoch=4,
            now_sec=10.0,
            candidates=(row,),
            ranked=(row,),
            messages={3: object()},
            selection_limits=self.selection_limits(),
        )
        self.assertEqual(batch.next_step(4)[0], "EVALUATE")
        result = MoveItEvaluation(row.position, True, False, 0.1, 0.4, "ok")
        self.assertFalse(batch.record(3, row, result, cached=False))
        self.assertFalse(batch.record(4, candidate(8), result, cached=False))
        batch.invalidate()
        self.assertFalse(batch.record(4, row, result, cached=False))
        self.assertEqual(batch.next_step(4), ("STALE", None))

    def test_shared_candidate_builder_matches_selector_clearance_contract(self):
        records = [
            {
                "track_id": 1,
                "maturity": 1,
                "position": (0.50, 0.0, 0.55),
                "confidence": 0.9,
                "sigma_m": 0.01,
                "observation_count": 4,
                "last_seen_sec": 10.0,
            },
            {
                "track_id": 2,
                "maturity": 2,
                "position": (0.60, 0.0, 0.55),
                "confidence": 0.8,
                "sigma_m": 0.01,
                "observation_count": 4,
                "last_seen_sec": 10.0,
            },
        ]
        built = harvest_candidates_from_records(
            records,
            fruit_radius_m=0.026,
            bin_bounds=(0.14, 0.56, -0.68, -0.22, 0.25, 0.55),
            static_obstacle_margin_m=0.05,
        )

        self.assertEqual([row.track_id for row in built], [1, 2])
        self.assertEqual([row.maturity for row in built], [1, 2])
        self.assertTrue(all(row.clearance_m >= 0.0 for row in built))

    def test_moveit_result_replaces_geometric_path_assumption(self):
        evaluation = MoveItEvaluation(
            position=(0.50, 0.0, 0.55),
            feasible=False,
            collision=True,
            planning_time_sec=0.3,
            joint_travel_rad=1.7,
            message="all bounded pregrasp IK or collision checks failed",
        )

        assessed = apply_moveit_evaluation(candidate(), evaluation)

        self.assertFalse(assessed.path_feasible)
        self.assertAlmostEqual(assessed.joint_travel_rad, 1.7)

    def test_moveit_priority_group_defers_joint_travel_until_after_feasibility(self):
        short = candidate(1, joint_travel_rad=0.1)
        long = candidate(2, joint_travel_rad=2.0)

        self.assertEqual(moveit_priority_group(short), moveit_priority_group(long))

    def test_cached_path_proof_is_bounded_by_target_pose_drift(self):
        evaluation = MoveItEvaluation(
            position=(0.50, 0.0, 0.55),
            feasible=True,
            collision=False,
            planning_time_sec=0.4,
            joint_travel_rad=1.1,
            message="connected and feasible",
        )

        self.assertTrue(
            evaluation_matches_position(
                evaluation,
                (0.503, 0.0, 0.55),
                maximum_drift_m=0.005,
            )
        )
        self.assertFalse(
            evaluation_matches_position(
                evaluation,
                (0.506, 0.0, 0.55),
                maximum_drift_m=0.005,
            )
        )

    def test_backend_busy_is_deferred_instead_of_cached_as_unreachable(self):
        self.assertTrue(evaluation_failure_is_transient("motion backend is busy"))
        self.assertFalse(
            evaluation_failure_is_transient(
                "all bounded pregrasp IK or collision checks failed"
            )
        )

    def test_selector_contract_calls_moveit_before_publishing_selection(self):
        source = (PACKAGE / "strawberry_bringup" / "target_selector.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("from strawberry_interfaces.srv import EvaluateTarget", source)
        self.assertIn("self._evaluation_client.call_async(request)", source)
        self.assertIn("def _finish_evaluation_with_selection", source)
        self.assertIn('"MOVEIT_PATH_INFEASIBLE"', source)
        self.assertIn('"position_m": list(selected.position)', source)

    def test_new_completion_is_recorded_once_without_cache_replay(self):
        excluded = set()

        self.assertTrue(register_completed_track(excluded, 4))
        self.assertEqual(excluded, {4})
        self.assertFalse(register_completed_track(excluded, 4))

    def test_invalid_completion_identity_is_not_recorded(self):
        excluded = set()

        self.assertFalse(register_completed_track(excluded, 0))
        self.assertEqual(excluded, set())

    def test_valid_fruit_and_nearer_wrist_view_use_distinct_bounds(self):
        fruit = (0.50, 0.0, 0.55)
        forward_view = generate_dynamic_views(
            1,
            fruit,
            distances_m=(0.22,),
            azimuths_deg=(0.0,),
            elevations_deg=(15.0,),
        )[0]

        self.assertTrue(inside_conservative_reach(fruit))
        self.assertFalse(inside_conservative_reach(forward_view.position))
        self.assertTrue(inside_observation_reach(forward_view.position))

    def test_observation_prefilter_rejects_clearly_impossible_pose(self):
        self.assertFalse(inside_observation_reach((-0.40, 0.0, 1.20)))

    def test_bin_clearance_rejects_target_at_wall_and_keeps_distant_target(self):
        bounds = expanded_bin_bounds(
            (0.19, 0.51, -0.63, -0.27, 0.28, 0.55)
        )
        self.assertLess(
            axis_aligned_box_clearance((0.3818, -0.2608, 0.5493), bounds),
            0.02,
        )
        self.assertGreater(
            axis_aligned_box_clearance((0.5086, -0.1065, 0.5457), bounds),
            0.10,
        )

    def test_bin_outer_bounds_cover_the_physical_back_wall(self):
        bounds = expanded_bin_bounds(
            (0.19, 0.51, -0.63, -0.27, 0.28, 0.55)
        )
        for actual, expected in zip(
            bounds, (0.14, 0.56, -0.68, -0.22, 0.25, 0.55)
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertLess(
            axis_aligned_box_clearance((0.31, -0.215, 0.541), bounds),
            0.01,
        )


if __name__ == "__main__":
    unittest.main()
