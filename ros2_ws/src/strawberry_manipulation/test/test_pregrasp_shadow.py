import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.handoff_shadow import ARM_JOINT_NAMES  # noqa: E402
from strawberry_manipulation.pregrasp_shadow import (  # noqa: E402
    summarize_pregrasp_shadow,
)


COLLISIONS = [
    "work_table",
    "strawberry_planter",
    "strawberry_plant_crown",
    "collection_bin",
    "strawberry_fruit_1",
    "strawberry_fruit_2",
    "strawberry_fruit_3",
]


class PregraspShadowTests(unittest.TestCase):
    @staticmethod
    def target_sample():
        return {
            "target_id": 1,
            "frame_id": "panda_link0",
            "age_sec": 0.05,
        }

    @staticmethod
    def joint_sample(offset=0.0):
        return {
            name: index * 0.1 + offset
            for index, name in enumerate(ARM_JOINT_NAMES)
        }

    def summarize(self, **overrides):
        values = {
            "handoff": {
                "handoff_passed": True,
                "pick_authorized": False,
                "expected_target_id": 1,
            },
            "target_samples": [self.target_sample() for _ in range(15)],
            "joint_samples": [self.joint_sample() for _ in range(10)],
            "planning_attempts": [
                {
                    "attempt": 1,
                    "success": True,
                    "planning_time_sec": 0.03,
                    "trajectory_waypoint_count": 20,
                    "endpoint_position_error_m": 0.001,
                    "endpoint_orientation_error_rad": 0.01,
                }
            ],
            "expected_collision_ids": COLLISIONS,
            "collision_ids_before": COLLISIONS,
            "collision_ids_after": COLLISIONS,
            "plan_completion_target_age_sec": 0.12,
            "moveit_state_delta_rad": 0.0,
        }
        values.update(overrides)
        return summarize_pregrasp_shadow(**values)

    def test_valid_plan_is_discarded_without_control(self):
        result = self.summarize()
        self.assertTrue(result["planning_passed"])
        self.assertTrue(result["trajectory_generated"])
        self.assertTrue(result["trajectory_discarded"])
        self.assertFalse(result["trajectory_executed"])
        self.assertFalse(result["pick_authorized"])
        self.assertEqual(result["control_command_count"], 0)
        self.assertTrue(
            result["collision_scene"]["selected_fruit_collision_retained"]
        )

    def test_stale_target_or_moving_arm_fails_closed(self):
        samples = [self.target_sample() for _ in range(15)]
        samples[-1]["age_sec"] = 0.7
        joints = [self.joint_sample() for _ in range(9)]
        joints.append(self.joint_sample(offset=0.01))
        result = self.summarize(
            target_samples=samples,
            joint_samples=joints,
        )
        self.assertFalse(result["planning_passed"])
        self.assertTrue(
            any("stale" in value for value in result["violations"])
        )
        self.assertTrue(
            any("arm moved" in value for value in result["violations"])
        )

    def test_missing_target_collision_or_execution_fails_closed(self):
        collisions = [
            value for value in COLLISIONS if value != "strawberry_fruit_1"
        ]
        result = self.summarize(
            collision_ids_after=collisions,
            trajectory_executed=True,
        )
        self.assertFalse(result["planning_passed"])
        self.assertFalse(result["trajectory_discarded"])
        self.assertGreater(result["control_command_count"], 0)

    def test_bad_endpoint_is_rejected(self):
        result = self.summarize(
            planning_attempts=[
                {
                    "attempt": 1,
                    "success": True,
                    "trajectory_waypoint_count": 2,
                    "endpoint_position_error_m": 0.02,
                    "endpoint_orientation_error_rad": 0.01,
                }
            ]
        )
        self.assertFalse(result["planning_passed"])

    def test_runtime_source_has_no_control_endpoint_or_execute_call(self):
        source = (
            PACKAGE_ROOT
            / "strawberry_manipulation"
            / "pregrasp_shadow.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("ActionClient", source)
        self.assertNotIn("create_publisher(", source)
        self.assertNotIn("moveit.execute(", source)
        self.assertNotIn(".execute(", source)
        self.assertIn("enable_trajectory_execution=False", source)


if __name__ == "__main__":
    unittest.main()
