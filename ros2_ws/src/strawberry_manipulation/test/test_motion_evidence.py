"""Exercise production command evidence without ROS or a physical controller."""
import pathlib
import sys
import unittest
from types import SimpleNamespace as NS

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.motion_evidence import (  # noqa: E402
    MotionEvidence,
    serialize_joint_feedback,
    serialize_trajectory,
)


def stamp(sec=1, nanosec=5):
    return NS(sec=sec, nanosec=nanosec)


class MotionEvidenceTests(unittest.TestCase):
    def test_records_complete_trajectory_without_mutating_command(self):
        points = [NS(positions=[0.1, 0.2], velocities=[], accelerations=[],
                     effort=[], time_from_start=stamp(0, 50000000))]
        trajectory = NS(header=NS(stamp=stamp(0, 0), frame_id="base"),
                        joint_names=["panda_joint7", "panda_joint1"], points=points)
        record = serialize_trajectory(trajectory)
        self.assertEqual(record["joint_names"], ["panda_joint7", "panda_joint1"])
        self.assertEqual(record["points"][0]["positions"], [0.1, 0.2])
        self.assertEqual(record["points"][0]["time_from_start_ns"], 50000000)
        self.assertEqual(record["points"][0]["velocities"], [])
        self.assertEqual(points[0].positions, [0.1, 0.2])
        points[0].positions[0] = 8.0
        self.assertEqual(record["points"][0]["positions"], [0.1, 0.2])

    def test_command_identity_and_causal_clock_are_preserved(self):
        rows = []
        evidence = MotionEvidence(rows.append, run_id="unit-run", scenario_id="unit-scene",
                                  producer_id="unit-producer", clock_ns=lambda: 500,
                                  ros_now_ns=lambda: 42)
        command = evidence.start_command("DIRECT_ARM_TRAJECTORY", {"joint_names": ["panda_joint7"]})
        evidence.emit("ACTION_ACCEPTED", {"goal_uuid": "abc"}, command_id=command)
        self.assertEqual([r["producer_seq"] for r in rows], [1, 2])
        self.assertEqual(rows[0]["event_time_monotonic_ns"], 500)
        self.assertEqual(rows[0]["ros_time_ns"], 42)
        self.assertEqual(rows[1]["command_id"], command)
        self.assertNotEqual(rows[0]["event_id"], rows[1]["event_id"])
        self.assertIsNone(rows[0]["operation_id"])
        self.assertFalse(rows[0]["physical_stop_claim"])

    def test_evidence_identity_requires_nonempty_run_and_scenario(self):
        for run_id, scenario_id in ((None, None), ("run", None), (None, "scene"), ("", "scene")):
            with self.subTest(run_id=run_id, scenario_id=scenario_id):
                with self.assertRaises(ValueError):
                    MotionEvidence([], run_id=run_id, scenario_id=scenario_id)

    def test_joint_feedback_retains_acquisition_and_receipt_separately(self):
        message = NS(header=NS(stamp=stamp(), frame_id="base"),
                     name=["panda_joint7", "panda_joint1"],
                     position=[0.1, 0.2], velocity=[0.3, 0.4], effort=[])
        result = serialize_joint_feedback(message, receipt_monotonic_ns=700,
                                          received_ros_ns=1000000100)
        self.assertEqual(result["acquisition_stamp_ns"], 1000000005)
        self.assertEqual(result["received_monotonic_ns"], 700)
        self.assertEqual(result["received_ros_ns"], 1000000100)
        self.assertEqual(result["velocities"], [0.3, 0.4])
        self.assertEqual(result["joint_names"], message.name)
        self.assertEqual(result["invalid_fields"], [])

    def test_missing_and_nonfinite_fields_are_explicit_json_not_nan(self):
        import json
        message = NS(name=["panda_joint7"], position=[float("nan")], velocity=[])
        result = serialize_joint_feedback(message, receipt_monotonic_ns=7,
                                          received_ros_ns=None)
        self.assertIsNone(result["acquisition_stamp_ns"])
        self.assertEqual(result["positions"], [None])
        self.assertIn("positions[0]:NONFINITE", result["invalid_fields"])
        self.assertIn("acquisition_stamp_ns:MISSING", result["invalid_fields"])
        json.dumps(result, allow_nan=False)


class BackendEvidenceTests(unittest.TestCase):
    def backend(self):
        import threading
        from strawberry_manipulation.moveit_backend import MoveItBackend
        rows = []
        backend = MoveItBackend.__new__(MoveItBackend)
        backend.node = NS(get_logger=lambda: NS(info=lambda *_: None, error=lambda *_: None,
                                               warning=lambda *_: None),
                          get_clock=lambda: NS(now=lambda: NS(nanoseconds=1000000100)))
        backend._motion_evidence = MotionEvidence(
            rows.append,
            run_id="unit-run",
            scenario_id="unit-scene",
            clock_ns=lambda: 700,
            ros_now_ns=lambda: 1000000100,
        )
        backend._gripper_state_lock = threading.Lock()
        backend.gripper_joints = ()
        backend._latest_gripper_positions_m = {}
        backend._arm_joint_names = ("panda_joint1", "panda_joint7")
        backend._latest_arm_positions_rad = dict.fromkeys(backend._arm_joint_names)
        backend._arm_state_sequence = 0
        return backend, rows

    def test_backend_keeps_source_stamp_and_velocity_from_real_callback(self):
        backend, _ = self.backend()
        message = NS(header=NS(stamp=stamp(), frame_id="base"),
                     name=["panda_joint7", "panda_joint1"],
                     position=[0.2, 0.1], velocity=[0.02, 0.01], effort=[])
        backend._on_gripper_joint_state(message)
        self.assertTrue(hasattr(backend, "_latest_arm_feedback"))
        sample = backend._latest_arm_feedback
        self.assertEqual(sample["acquisition_stamp_ns"], 1000000005)
        self.assertEqual(sample["velocities"], [0.02, 0.01])
        self.assertIsInstance(sample["received_monotonic_ns"], int)

    def test_moveit_execution_route_serializes_its_time_parameterized_plan(self):
        backend, rows = self.backend()
        trajectory = NS(header=NS(stamp=stamp(0, 0), frame_id="base"),
                        joint_names=["panda_joint1", "panda_joint7"],
                        points=[NS(positions=[0.1, 0.2], velocities=[0.02, 0.03],
                                   accelerations=[0.0, 0.0], effort=[],
                                   time_from_start=stamp(1, 0))])
        robot_trajectory = NS(get_robot_trajectory_msg=lambda: NS(joint_trajectory=trajectory))
        backend._wait_until_arm_settled = lambda: True
        backend._arm = NS(set_start_state_to_current_state=lambda: None,
                          set_goal_state=lambda **k: None,
                          plan=lambda: NS(trajectory=robot_trajectory))
        backend._moveit = NS(execute=lambda *a, **k: True)
        result = backend._plan_and_execute(configuration="ready")
        self.assertTrue(result.success)
        self.assertTrue(rows, "MoveIt execution route has no evidence")
        self.assertEqual(rows[0]["payload"]["trajectory"], serialize_trajectory(trajectory))
        self.assertEqual(rows[0]["payload"]["route"], "MOVEIT_EXECUTE")
        self.assertEqual(rows[-1]["event_type"], "MOVEIT_EXECUTION_RETURNED")
        self.assertIsNone(rows[-1]["payload"]["action_accepted"])

    def test_direct_command_emits_exact_trajectory_acceptance_and_terminal(self):
        backend, rows = self.backend()
        backend._latest_live_arm_positions = lambda: None
        backend.joint_trajectory_velocity_rad_per_sec = 0.3
        backend.minimum_joint_waypoint_duration_sec = 0.05
        backend._joint_path_within_limit_margin = lambda *a, **k: True
        backend._joint_path_within_safety_limits = lambda *a, **k: True
        backend.request_timeout_sec = 1.0
        backend.trajectory_timeout_margin_sec = 1.0
        backend._FollowJointTrajectory = NS(Goal=lambda: NS(trajectory=NS(
            header=NS(stamp=stamp(0, 0), frame_id=""), joint_names=[], points=[])),
            Result=NS(SUCCESSFUL=0))
        backend._JointTrajectoryPoint = lambda: NS(positions=[], velocities=[],
            accelerations=[], effort=[], time_from_start=stamp(0, 0))
        backend._Duration = lambda seconds: NS(to_msg=lambda: stamp(
            int(seconds), round((seconds % 1) * 1000000000)))
        sent = []
        handle = NS(accepted=True, goal_id=NS(uuid=list(range(16))),
                    get_result_async=lambda: "result-future")
        backend._arm_action_probe = NS(wait_for_server=lambda **kw: True,
            send_goal_async=lambda goal: sent.append(goal) or handle)
        backend._wait_future = lambda future, timeout: future
        from strawberry_manipulation.moveit_backend import ArmTrajectoryWaitResult
        backend._wait_arm_trajectory_result = lambda *a: ArmTrajectoryWaitResult(
            "TERMINAL", result=NS(status=4, result=NS(error_code=0, error_string=""))
        )
        success, _ = backend._execute_joint_path((0.0, 0.0), ((0.03, 0.06),))
        self.assertTrue(success)
        events = {r["event_type"]: r for r in rows}
        self.assertIn("COMMAND_PREPARED", events)
        self.assertIn("ACTION_ACCEPTED", events)
        self.assertIn("ACTION_TERMINAL", events)
        expected = serialize_trajectory(sent[0].trajectory)
        self.assertEqual(events["COMMAND_PREPARED"]["payload"]["trajectory"], expected)
        self.assertEqual(len({r["command_id"] for r in rows}), 1)
        self.assertFalse(events["ACTION_TERMINAL"]["physical_stop_claim"])
    def test_direct_command_classifies_nonterminal_wait_outcomes(self):
        from strawberry_manipulation.moveit_backend import ArmTrajectoryWaitResult

        cases = (
            ("LIVE_JOINT_LIMIT_ABORT", "ACTION_LIVE_JOINT_LIMIT_ABORT"),
            ("RESULT_FUTURE_ERROR", "ACTION_RESULT_FUTURE_ERROR"),
            ("WALL_TIMEOUT", "ACTION_RESULT_TIMEOUT"),
        )
        for kind, expected_event in cases:
            with self.subTest(kind=kind):
                backend, rows = self.backend()
                backend._latest_live_arm_positions = lambda: None
                backend.joint_trajectory_velocity_rad_per_sec = 0.3
                backend.minimum_joint_waypoint_duration_sec = 0.05
                backend._joint_path_within_limit_margin = lambda *a, **k: True
                backend._joint_path_within_safety_limits = lambda *a, **k: True
                backend.request_timeout_sec = 1.0
                backend.trajectory_timeout_margin_sec = 1.0
                backend._FollowJointTrajectory = NS(
                    Goal=lambda: NS(
                        trajectory=NS(
                            header=NS(stamp=stamp(0, 0), frame_id=""),
                            joint_names=[],
                            points=[],
                        )
                    ),
                    Result=NS(SUCCESSFUL=0),
                )
                backend._JointTrajectoryPoint = lambda: NS(
                    positions=[], velocities=[], accelerations=[], effort=[],
                    time_from_start=stamp(0, 0),
                )
                backend._Duration = lambda seconds: NS(
                    to_msg=lambda: stamp(
                        int(seconds), round((seconds % 1) * 1_000_000_000)
                    )
                )
                handle = NS(
                    accepted=True,
                    goal_id=NS(uuid=list(range(16))),
                    get_result_async=lambda: "result-future",
                    cancel_goal_async=lambda: NS(return_code=0),
                )
                backend._arm_action_probe = NS(
                    wait_for_server=lambda **kw: True,
                    send_goal_async=lambda goal: handle,
                )
                backend._wait_future = lambda future, timeout: future
                backend._wait_arm_trajectory_result = lambda *a: (
                    ArmTrajectoryWaitResult(kind, detail="diagnostic")
                )

                success, _ = backend._execute_joint_path(
                    (0.0, 0.0), ((0.03, 0.06),)
                )

                self.assertFalse(success)
                event_types = [row["event_type"] for row in rows]
                self.assertIn(expected_event, event_types)
                self.assertEqual(
                    next(
                        row for row in rows
                        if row["event_type"] == "CANCEL_RESPONSE"
                    )["payload"]["reason"],
                    kind,
                )
                if kind != "WALL_TIMEOUT":
                    self.assertNotIn("ACTION_RESULT_TIMEOUT", event_types)


if __name__ == "__main__":
    unittest.main()
