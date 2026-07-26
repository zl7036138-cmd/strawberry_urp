import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.handoff_shadow import (  # noqa: E402
    ARM_JOINT_NAMES,
    summarize_handoff_shadow,
    target_clock_is_coherent,
)


EXPECTED_COLLISIONS = [
    "work_table",
    "strawberry_planter",
    "strawberry_plant_crown",
    "collection_bin",
    "strawberry_fruit_1",
    "strawberry_fruit_2",
    "strawberry_fruit_3",
]


class HandoffShadowTests(unittest.TestCase):
    @staticmethod
    def target_sample(target_id=1, age_sec=0.02):
        return {
            "target_id": target_id,
            "frame_id": "panda_link0",
            "age_sec": age_sec,
            "position_xyz_m": [0.5, 0.0, 0.7],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "detection_confidence": 0.8,
            "position_sigma_m": 0.01,
        }

    @staticmethod
    def joint_sample(offset=0.0):
        return {
            name: (index * 0.1) + offset
            for index, name in enumerate(ARM_JOINT_NAMES)
        }

    def summarize(self, **overrides):
        inputs = {
            "expected_target_id": 1,
            "target_samples": [self.target_sample() for _ in range(15)],
            "joint_samples": [self.joint_sample() for _ in range(10)],
            "expected_collision_ids": EXPECTED_COLLISIONS,
            "observed_collision_ids": EXPECTED_COLLISIONS,
            "observed_collision_frames": {
                object_id: "panda_link0"
                for object_id in EXPECTED_COLLISIONS
            },
            "moveit_state_delta_rad": 0.0,
        }
        inputs.update(overrides)
        return summarize_handoff_shadow(**inputs)

    def test_complete_handoff_stops_before_pick(self):
        result = self.summarize()
        self.assertTrue(result["handoff_passed"])
        self.assertFalse(result["pick_authorized"])
        self.assertFalse(result["pick_action_called"])
        self.assertFalse(result["trajectory_command_sent"])
        self.assertTrue(
            result["collision_scene"]["selected_fruit_collision_retained"]
        )
        self.assertEqual(
            result["state_history"][-1], "STOP_BEFORE_PICK_ACTION"
        )

    def test_wrong_identity_and_stale_pose_fail_closed(self):
        samples = [self.target_sample() for _ in range(14)]
        samples.append(self.target_sample(target_id=3, age_sec=0.7))
        result = self.summarize(target_samples=samples)
        self.assertFalse(result["handoff_passed"])
        self.assertIn(3, result["observed_target_ids"])
        self.assertTrue(
            any("stale" in violation for violation in result["violations"])
        )

    def test_motion_and_missing_target_collision_fail_closed(self):
        samples = [self.joint_sample() for _ in range(9)]
        samples.append(self.joint_sample(offset=0.01))
        observed = [
            value
            for value in EXPECTED_COLLISIONS
            if value != "strawberry_fruit_1"
        ]
        result = self.summarize(
            joint_samples=samples,
            observed_collision_ids=observed,
            observed_collision_frames={
                object_id: "panda_link0" for object_id in observed
            },
        )
        self.assertFalse(result["handoff_passed"])
        self.assertFalse(
            result["collision_scene"]["selected_fruit_collision_retained"]
        )
        self.assertTrue(
            any("arm moved" in violation for violation in result["violations"])
        )

    def test_control_activity_is_always_a_violation(self):
        result = self.summarize(pick_action_called=True)
        self.assertFalse(result["handoff_passed"])
        self.assertFalse(result["pick_authorized"])

    def test_moveit_fixed_world_frame_is_explicitly_allowed(self):
        result = self.summarize(
            observed_collision_frames={
                object_id: "world" for object_id in EXPECTED_COLLISIONS
            },
            allowed_collision_frame_ids=("panda_link0", "world"),
        )
        self.assertTrue(result["handoff_passed"])
        self.assertEqual(
            result["collision_scene"]["allowed_frame_ids"],
            ["panda_link0", "world"],
        )

    def test_runtime_source_does_not_create_control_endpoints(self):
        source = (
            PACKAGE_ROOT
            / "strawberry_manipulation"
            / "handoff_shadow.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("ActionClient", source)
        self.assertNotIn("create_publisher(", source)
        self.assertNotIn("moveit.execute(", source)
        self.assertIn("enable_trajectory_execution=False", source)

    def test_clock_readiness_rejects_only_large_startup_skew(self):
        self.assertFalse(
            target_clock_is_coherent(
                receipt_stamp_sec=0.0,
                acquisition_stamp_sec=39.8,
                maximum_startup_skew_sec=0.5,
            )
        )
        self.assertFalse(
            target_clock_is_coherent(
                receipt_stamp_sec=0.4,
                acquisition_stamp_sec=39.8,
                maximum_startup_skew_sec=0.5,
            )
        )
        self.assertTrue(
            target_clock_is_coherent(
                receipt_stamp_sec=39.85,
                acquisition_stamp_sec=39.8,
                maximum_startup_skew_sec=0.5,
            )
        )


if __name__ == "__main__":
    unittest.main()
