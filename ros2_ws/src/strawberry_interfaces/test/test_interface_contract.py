# Copyright 2026 Strawberry URP Team
# SPDX-License-Identifier: Apache-2.0

"""Offline structural checks for the frozen Strawberry URP ROS contract.

These tests intentionally require only the Python standard library so the
interface schema can be checked before ROS 2 is installed.
"""

from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def schema_lines(relative_path: str) -> list[str]:
    """Return non-empty, comment-free interface declaration lines."""
    text = (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")
    return [
        line.strip()
        for line in text.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


class InterfaceContractTest(unittest.TestCase):
    def test_detection_schema_and_maturity_values(self) -> None:
        self.assertEqual(
            schema_lines("msg/StrawberryDetection.msg"),
            [
                "uint8 UNKNOWN=0",
                "uint8 RIPE=1",
                "uint8 UNRIPE=2",
                "uint32 target_id",
                "uint8 maturity",
                "float32 confidence",
                "sensor_msgs/RegionOfInterest bbox",
            ],
        )

    def test_detection_array_has_one_shared_header(self) -> None:
        self.assertEqual(
            schema_lines("msg/StrawberryDetectionArray.msg"),
            [
                "std_msgs/Header header",
                "strawberry_interfaces/StrawberryDetection[] detections",
            ],
        )

    def test_target_pose_schema(self) -> None:
        self.assertEqual(
            schema_lines("msg/TargetPose.msg"),
            [
                "std_msgs/Header header",
                "uint32 target_id",
                "geometry_msgs/Pose pose",
                "float32 detection_confidence",
                "float32 position_sigma_m",
            ],
        )

    def test_tracked_target_schema_carries_stable_identity_and_quality(self) -> None:
        self.assertEqual(
            schema_lines("msg/TrackedTarget.msg"),
            [
                "std_msgs/Header header",
                "uint32 track_id",
                "uint32 source_detection_id",
                "uint8 UNKNOWN=0",
                "uint8 RIPE=1",
                "uint8 UNRIPE=2",
                "uint8 maturity",
                "geometry_msgs/Pose pose",
                "float32 detection_confidence",
                "float32 position_sigma_m",
                "uint32 observation_count",
            ],
        )
        self.assertEqual(
            schema_lines("msg/TrackedTargetArray.msg"),
            [
                "std_msgs/Header header",
                "strawberry_interfaces/TrackedTarget[] targets",
            ],
        )

    def test_observation_plan_contains_a_bounded_runtime_pose_bank(self) -> None:
        self.assertEqual(
            schema_lines("msg/ObservationPlan.msg"),
            [
                "std_msgs/Header header",
                "uint32 target_id",
                "geometry_msgs/Pose[] hand_poses",
            ],
        )

    def test_action_goal_result_and_feedback(self) -> None:
        sections: list[list[str]] = [[]]
        for line in schema_lines("action/PickAndPlace.action"):
            if line == "---":
                sections.append([])
            else:
                sections[-1].append(line)

        self.assertEqual(len(sections), 3)
        self.assertEqual(
            sections[0],
            [
                "uint32 target_id",
                "geometry_msgs/PoseStamped target_pose",
                "geometry_msgs/PoseStamped place_pose",
            ],
        )
        self.assertEqual(
            sections[1][-5:],
            [
                "bool success",
                "uint8 failure_code",
                "float32 planning_time_sec",
                "float32 execution_time_sec",
                "string message",
            ],
        )
        self.assertEqual(sections[2], ["string stage", "float32 progress"])

    def test_observation_motion_service_is_bounded_and_typed(self) -> None:
        self.assertEqual(
            schema_lines("srv/MoveToObservation.srv"),
            [
                "uint32 target_id",
                "geometry_msgs/PoseStamped target_pose",
                "geometry_msgs/PoseStamped observation_pose",
                "---",
                "bool success",
                "float32 planning_time_sec",
                "float32 execution_time_sec",
                "string message",
            ],
        )
        self.assertEqual(
            schema_lines("srv/EvaluateTarget.srv"),
            [
                "uint32 target_id",
                "geometry_msgs/PoseStamped target_pose",
                "---",
                "bool feasible",
                "bool collision",
                "float32 planning_time_sec",
                "float32 joint_travel_rad",
                "string message",
            ],
        )

    def test_failure_codes_are_stable_and_contiguous(self) -> None:
        expected = {
            "NONE": 0,
            "NO_TARGET": 1,
            "LOW_CONFIDENCE": 2,
            "DEPTH_INVALID": 3,
            "TF_TIMEOUT": 4,
            "UNREACHABLE": 5,
            "PLANNING_FAILED": 6,
            "COLLISION": 7,
            "GRASP_FAILED": 8,
            "PLACE_FAILED": 9,
            "STALE_DATA": 10,
        }
        declarations = schema_lines("action/PickAndPlace.action")
        actual = {}
        for declaration in declarations:
            match = re.fullmatch(r"uint8 ([A-Z_]+)=(\d+)", declaration)
            if match:
                actual[match.group(1)] = int(match.group(2))
        self.assertEqual(actual, expected)

    def test_manifest_declares_interface_package_and_dependencies(self) -> None:
        root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
        self.assertEqual(root.findtext("name"), "strawberry_interfaces")
        self.assertEqual(root.findtext("version"), "0.1.0")
        self.assertEqual(
            root.findtext("member_of_group"), "rosidl_interface_packages"
        )

        declared_dependencies = {
            node.text
            for tag in ("build_depend", "depend", "exec_depend")
            for node in root.findall(tag)
        }
        self.assertTrue(
            {
                "geometry_msgs",
                "rosidl_default_generators",
                "rosidl_default_runtime",
                "sensor_msgs",
                "std_msgs",
            }.issubset(declared_dependencies)
        )

    def test_cmake_generates_every_interface(self) -> None:
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        for interface in (
            "msg/StrawberryDetection.msg",
            "msg/StrawberryDetectionArray.msg",
            "msg/TargetPose.msg",
            "msg/TrackedTarget.msg",
            "msg/TrackedTargetArray.msg",
            "msg/ObservationPlan.msg",
            "srv/MoveToObservation.srv",
            "srv/EvaluateTarget.srv",
            "action/PickAndPlace.action",
        ):
            self.assertIn(f'"{interface}"', cmake)
        self.assertIn("rosidl_generate_interfaces(${PROJECT_NAME}", cmake)


if __name__ == "__main__":
    unittest.main()
