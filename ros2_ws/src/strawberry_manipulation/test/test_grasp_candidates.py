from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.core import DEFAULT_GRASP_QUATERNION, Pose  # noqa: E402
from strawberry_manipulation.grasp_candidates import (  # noqa: E402
    TILT_SEQUENCE_DEGREES,
    generate_grasp_candidates,
)


class GraspCandidateTests(unittest.TestCase):
    def setUp(self):
        self.target = Pose(0.42, -0.08, 0.54)

    def test_frozen_count_order_and_unique_ids(self):
        candidates = generate_grasp_candidates(self.target)
        self.assertEqual(len(candidates), 15)
        self.assertEqual(
            tuple(candidate.candidate_id for candidate in candidates),
            tuple(f"G{index:02d}" for index in range(15)),
        )
        self.assertEqual(TILT_SEQUENCE_DEGREES[0], (0, 0))
        self.assertEqual((candidates[0].tilt_x_rad, candidates[0].tilt_y_rad), (0.0, 0.0))

    def test_output_is_deterministic_normalized_and_fingerprinted(self):
        first = generate_grasp_candidates(self.target)
        second = generate_grasp_candidates(self.target)
        self.assertEqual(first, second)
        self.assertEqual(len({candidate.geometry_fingerprint for candidate in first}), 15)
        for candidate in first:
            pose = candidate.grasp_pose
            self.assertAlmostEqual(
                math.sqrt(pose.qx**2 + pose.qy**2 + pose.qz**2 + pose.qw**2), 1.0
            )

    def test_nominal_preserves_nominal_orientation_and_fruit_center(self):
        candidate = generate_grasp_candidates(self.target)[0]
        self.assertEqual(
            (candidate.grasp_pose.qx, candidate.grasp_pose.qy,
             candidate.grasp_pose.qz, candidate.grasp_pose.qw),
            DEFAULT_GRASP_QUATERNION,
        )
        # Reconstruct fruit centre by moving back along the candidate's own axis.
        axis = (
            candidate.grasp_pose.x - self.target.x,
            candidate.grasp_pose.y - self.target.y,
            candidate.grasp_pose.z - self.target.z,
        )
        self.assertAlmostEqual(math.sqrt(sum(value * value for value in axis)), 0.1054)

    def test_pregrasp_and_escape_follow_each_candidates_own_axis(self):
        for candidate in generate_grasp_candidates(self.target):
            pre_delta = tuple(
                left - right for left, right in zip(
                    (candidate.pregrasp_pose.x, candidate.pregrasp_pose.y, candidate.pregrasp_pose.z),
                    (candidate.grasp_pose.x, candidate.grasp_pose.y, candidate.grasp_pose.z),
                )
            )
            escape_delta = tuple(
                left - right for left, right in zip(
                    (candidate.escape_pose.x, candidate.escape_pose.y, candidate.escape_pose.z),
                    (candidate.grasp_pose.x, candidate.grasp_pose.y, candidate.grasp_pose.z),
                )
            )
            self.assertAlmostEqual(math.sqrt(sum(value * value for value in pre_delta)), 0.15)
            self.assertAlmostEqual(math.sqrt(sum(value * value for value in escape_delta)), 0.08)
            self.assertAlmostEqual(
                pre_delta[0] * escape_delta[1] - pre_delta[1] * escape_delta[0], 0.0,
                places=8,
            )

    def test_invalid_target_and_offsets_fail_closed(self):
        with self.assertRaisesRegex(ValueError, "target position"):
            generate_grasp_candidates(Pose(float("nan"), 0.0, 0.0))
        with self.assertRaisesRegex(ValueError, "offsets"):
            generate_grasp_candidates(self.target, escape_offset_m=0.0)

    def test_candidate_fingerprint_cannot_be_reused_for_changed_geometry(self):
        candidate = generate_grasp_candidates(self.target)[2]
        with self.assertRaisesRegex(ValueError, "fingerprint"):
            replace(candidate, tilt_x_rad=candidate.tilt_x_rad + 0.01)


if __name__ == "__main__":
    unittest.main()
