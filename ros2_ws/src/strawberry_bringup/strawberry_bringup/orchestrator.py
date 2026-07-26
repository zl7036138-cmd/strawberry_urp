"""ROS 2 trial orchestrator using the frozen PickAndPlace action."""

from __future__ import annotations

import json

from .core import Event, State, TrialStateMachine, validate_target_routing


def main(args=None) -> None:  # pragma: no cover - exercised in ROS integration
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from strawberry_interfaces.action import PickAndPlace
        from strawberry_interfaces.msg import TargetPose
        from strawberry_interfaces.msg import StrawberryDetectionArray
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    class TrialOrchestrator(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_trial_orchestrator")
            self.declare_parameter("acquisition_timeout_sec", 3.0)
            self.declare_parameter("place_x", 0.35)
            self.declare_parameter("place_y", -0.45)
            self.declare_parameter("place_z", 0.45)
            self.declare_parameter("target_source", "oracle")
            self.declare_parameter(
                "control_target_topic", "/strawberry/oracle/target_pose"
            )
            self.declare_parameter("shadow_enabled", False)
            self.declare_parameter(
                "shadow_target_topic", "/strawberry/shadow/target_pose"
            )
            self.declare_parameter(
                "shadow_detections_topic", "/strawberry/shadow/detections"
            )
            try:
                self._routing = validate_target_routing(
                    source=self.get_parameter("target_source").value,
                    control_topic=self.get_parameter("control_target_topic").value,
                    shadow_enabled=self.get_parameter("shadow_enabled").value,
                    shadow_topic=self.get_parameter("shadow_target_topic").value,
                )
            except ValueError as exc:
                raise RuntimeError(f"invalid target routing: {exc}") from exc
            self._machine = TrialStateMachine(sensor_retry_limit=3)
            self._active = False
            self._timer = None
            self._trial_sequence = 0
            self._control_target_id = None
            self._shadow_observations = 0
            self._shadow_latest_target_id = None
            self._shadow_detection_frames = 0
            self._shadow_detection_count = 0
            self._shadow_ripe_detection_count = 0
            self._planning_time_sec = None
            self._execution_time_sec = None
            self._failure_code = None
            self._failure_message = ""
            self._status = self.create_publisher(String, "/strawberry/trial_status", 10)
            self._action = ActionClient(
                self, PickAndPlace, "/strawberry/pick_and_place"
            )
            self.create_subscription(
                TargetPose,
                self._routing.control_topic,
                self._on_control_target,
                10,
            )
            if self._routing.shadow_enabled:
                self.create_subscription(
                    TargetPose,
                    self._routing.shadow_topic,
                    self._on_shadow_target,
                    10,
                )
                self.create_subscription(
                    StrawberryDetectionArray,
                    str(self.get_parameter("shadow_detections_topic").value),
                    self._on_shadow_detections,
                    10,
                )
            self.create_service(Trigger, "/strawberry/run_trial", self._run_trial)

        def _publish(self, outcome: str = "") -> None:
            message = String()
            message.data = json.dumps(
                {
                    "schema_version": 2,
                    "trial_sequence": self._trial_sequence,
                    "state": self._machine.state.value,
                    "outcome": outcome,
                    "sensor_retries": self._machine.sensor_retries,
                    "target_source": self._routing.source.value,
                    "control_target_topic": self._routing.control_topic,
                    "control_target_id": self._control_target_id,
                    "shadow_enabled": self._routing.shadow_enabled,
                    "shadow_target_topic": self._routing.shadow_topic,
                    "shadow_observations": self._shadow_observations,
                    "shadow_latest_target_id": self._shadow_latest_target_id,
                    "shadow_detection_frames": self._shadow_detection_frames,
                    "shadow_detection_count": self._shadow_detection_count,
                    "shadow_ripe_detection_count": (
                        self._shadow_ripe_detection_count
                    ),
                    "planning_time_sec": self._planning_time_sec,
                    "execution_time_sec": self._execution_time_sec,
                    "failure_code": self._failure_code,
                    "failure_message": self._failure_message,
                    "state_history": [
                        {
                            "previous": record.previous.value,
                            "event": record.event.value,
                            "current": record.current.value,
                            "reason": record.reason,
                        }
                        for record in self._machine.history
                    ],
                },
                separators=(",", ":"),
            )
            self._status.publish(message)

        def _run_trial(self, request, response):
            if self._active:
                response.success = False
                response.message = "a trial is already active"
                return response
            if not self._action.server_is_ready():
                response.success = False
                response.message = "pick action is not ready"
                return response
            if self._machine.terminal:
                self._machine = TrialStateMachine(sensor_retry_limit=3)
            self._trial_sequence += 1
            self._control_target_id = None
            self._shadow_observations = 0
            self._shadow_latest_target_id = None
            self._shadow_detection_frames = 0
            self._shadow_detection_count = 0
            self._shadow_ripe_detection_count = 0
            self._planning_time_sec = None
            self._execution_time_sec = None
            self._failure_code = None
            self._failure_message = ""
            self._machine.apply(Event.START)
            self._machine.apply(Event.READY)
            self._active = True
            self._publish()
            timeout = float(self.get_parameter("acquisition_timeout_sec").value)
            self._timer = self.create_timer(timeout, self._on_timeout)
            response.success = True
            response.message = "trial started"
            return response

        def _on_timeout(self) -> None:
            if self._timer is not None:
                self._timer.cancel()
            if not self._active:
                return
            # No localized ripe target means safe no-pick, not robot failure.
            self._machine.apply(Event.NO_PICK, "no ripe target before timeout")
            self._active = False
            self._publish("NO_PICK")

        def _on_shadow_target(self, target) -> None:
            if not self._active:
                return
            self._shadow_observations += 1
            self._shadow_latest_target_id = int(target.target_id)

        def _on_shadow_detections(self, message) -> None:
            if not self._active:
                return
            self._shadow_detection_frames += 1
            self._shadow_detection_count += len(message.detections)
            self._shadow_ripe_detection_count += sum(
                item.maturity == item.RIPE for item in message.detections
            )

        def _on_control_target(self, target) -> None:
            if not self._active or self._machine.state is not State.ACQUIRE:
                return
            if int(target.target_id) <= 0:
                self.get_logger().warning("rejecting non-positive control target ID")
                return
            if self._timer is not None:
                self._timer.cancel()
            self._control_target_id = int(target.target_id)
            self._machine.apply(Event.FRAME)
            self._machine.apply(Event.DETECTIONS)
            self._machine.apply(Event.TARGET)
            self._machine.apply(Event.TARGET)
            goal = PickAndPlace.Goal()
            goal.target_id = self._control_target_id
            goal.target_pose = PoseStamped()
            goal.target_pose.header = target.header
            goal.target_pose.pose = target.pose
            goal.place_pose = PoseStamped()
            goal.place_pose.header.frame_id = "panda_link0"
            goal.place_pose.pose.position.x = float(self.get_parameter("place_x").value)
            goal.place_pose.pose.position.y = float(self.get_parameter("place_y").value)
            goal.place_pose.pose.position.z = float(self.get_parameter("place_z").value)
            goal.place_pose.pose.orientation.w = 1.0
            if not self._action.wait_for_server(timeout_sec=1.0):
                self._machine.apply(Event.ERROR, "pick action unavailable")
                self._active = False
                self._publish("ACTION_UNAVAILABLE")
                return
            future = self._action.send_goal_async(goal, feedback_callback=self._feedback)
            future.add_done_callback(self._goal_response)

        def _feedback(self, feedback_message) -> None:
            mapping = {
                "APPROACH": (State.PLAN, Event.PLANNED),
                "GRASP": (State.APPROACH, Event.APPROACHED),
                "RETREAT": (State.GRASP, Event.GRASPED),
                "PLACE": (State.RETREAT, Event.RETREATED),
                "VERIFY": (State.PLACE, Event.PLACED),
            }
            stage = feedback_message.feedback.stage
            expected = mapping.get(stage)
            if expected and self._machine.state is expected[0]:
                self._machine.apply(expected[1])
                self._publish()

        def _goal_response(self, future) -> None:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._machine.apply(Event.ERROR, "pick goal rejected")
                self._active = False
                self._publish("GOAL_REJECTED")
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(self._result)

        def _result(self, future) -> None:
            result = future.result().result
            self._planning_time_sec = float(result.planning_time_sec)
            self._execution_time_sec = float(result.execution_time_sec)
            if result.success:
                if self._machine.state is State.VERIFY:
                    self._machine.apply(Event.VERIFIED)
                else:
                    # Feedback can be dropped; terminal result remains authoritative.
                    self._machine.state = State.DONE
                outcome = "SUCCESS"
            else:
                self._failure_code = int(result.failure_code)
                self._failure_message = str(result.message)
                self._machine.apply(Event.ERROR, result.message)
                outcome = f"FAILURE_{result.failure_code}"
            self._active = False
            self._publish(outcome)

    rclpy.init(args=args)
    node = TrialOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
