"""ROS 2 continuous multi-fruit harvest orchestrator."""

from __future__ import annotations

import json
import math
import time

from .harvest_planning import (
    fuse_position_estimates,
    rank_distinct_view_indices,
    wrist_refinement_diagnostics,
    wrist_refinement_rejection_reason,
)
from .harvest_sequence import HarvestSequence, HarvestState


def main(args=None) -> None:  # pragma: no cover - ROS integration
    try:
        import rclpy
        from geometry_msgs.msg import Pose, PoseStamped
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String, UInt32
        from std_srvs.srv import Trigger
        from strawberry_interfaces.action import PickAndPlace
        from strawberry_interfaces.msg import ObservationPlan, TargetPose, TrackedTargetArray
        from strawberry_interfaces.srv import EvaluateTarget, MoveToObservation
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    class HarvestOrchestrator(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_harvest_orchestrator")
            self.declare_parameter("scan_timeout_sec", 3.0)
            self.declare_parameter("wrist_confirmation_timeout_sec", 3.0)
            self.declare_parameter("wrist_max_correction_m", 0.05)
            self.declare_parameter("wrist_min_confidence", 0.60)
            self.declare_parameter("wrist_max_sigma_m", 0.015)
            self.declare_parameter("wrist_systematic_sigma_m", 0.030)
            self.declare_parameter("minimum_reobservation_baseline_m", 0.04)
            self.declare_parameter("max_targets", 12)
            self.declare_parameter("place_x", 0.35)
            self.declare_parameter("place_y", -0.45)
            self.declare_parameter("place_z", 0.45)
            self.declare_parameter("require_wrist_confirmation", True)
            self._group = ReentrantCallbackGroup()
            self._sequence = HarvestSequence(max_targets=int(self.get_parameter("max_targets").value))
            self._started_monotonic = None
            self._scan_timer = None
            self._confirmation_timer = None
            self._recovery_home_in_progress = False
            self._latest_view_by_stamp = {}
            self._latest_plan_by_stamp = {}
            self._current_target = None
            self._current_goal_pose = None
            self._current_view = None
            self._current_views = []
            self._current_view_index = 0
            self._current_is_reobservation = False
            self._last_observation_position_by_target = {}
            self._distinct_reobservation_targets = set()
            self._last_candidate_ids = set()
            self._last_wrist_rejection_reason = None
            self._status = self.create_publisher(String, "/strawberry/harvest_status", 10)
            self._completed = self.create_publisher(UInt32, "/strawberry/completed_track_id", 10)
            self._wrist_target_hint = self.create_publisher(
                TargetPose, "/strawberry/wrist/target_hint", 10
            )
            self._pick_client = ActionClient(self, PickAndPlace, "/strawberry/pick_and_place", callback_group=self._group)
            self._observation_client = self.create_client(
                MoveToObservation, "/strawberry/move_to_observation", callback_group=self._group
            )
            self._evaluation_client = self.create_client(
                EvaluateTarget, "/strawberry/evaluate_target", callback_group=self._group
            )
            self._home_client = self.create_client(
                Trigger, "/strawberry/move_home", callback_group=self._group
            )
            self.create_subscription(PoseStamped, "/strawberry/wrist_observation_pose", self._on_view, 10, callback_group=self._group)
            self.create_subscription(ObservationPlan, "/strawberry/wrist_observation_plan", self._on_view_plan, 10, callback_group=self._group)
            self.create_subscription(TargetPose, "/strawberry/target_pose", self._on_selected, 10, callback_group=self._group)
            self.create_subscription(TargetPose, "/strawberry/wrist/target_pose", self._on_wrist_target, 10, callback_group=self._group)
            self.create_subscription(TrackedTargetArray, "/strawberry/tracked_targets", self._on_tracks, 10, callback_group=self._group)
            self.create_service(Trigger, "/strawberry/run_harvest", self._run, callback_group=self._group)

        @staticmethod
        def _stamp_key(stamp):
            return int(stamp.sec), int(stamp.nanosec)

        def _elapsed(self) -> float:
            return 0.0 if self._started_monotonic is None else time.monotonic() - self._started_monotonic

        def _publish(
            self,
            outcome: str = "",
            diagnostics: dict | None = None,
        ) -> None:
            message = String()
            message.data = json.dumps(
                {
                    "schema_version": 1,
                    "state": self._sequence.state.value,
                    "outcome": outcome,
                    "current_target_id": self._sequence.current_target_id,
                    "retry_target_id": self._sequence.retry_target_id,
                    "harvested_target_ids": list(self._sequence.harvested),
                    "skipped_targets": dict(self._sequence.skipped),
                    "failures": list(self._sequence.failures),
                    "remaining_candidate_count": len(self._last_candidate_ids - self._sequence.completed_ids),
                    "elapsed_sec": self._elapsed(),
                    "diagnostics": {} if diagnostics is None else diagnostics,
                    "state_history": [event.__dict__ | {"state": event.state.value} for event in self._sequence.history],
                },
                separators=(",", ":"),
            )
            self._status.publish(message)

        def _cancel_timer(self, name: str) -> None:
            timer = getattr(self, name)
            if timer is not None:
                timer.cancel()
                setattr(self, name, None)

        def _arm_scan_timer(self) -> None:
            self._cancel_timer("_scan_timer")
            self._scan_timer = self.create_timer(float(self.get_parameter("scan_timeout_sec").value), self._scan_timeout, callback_group=self._group)

        def _run(self, request, response):
            del request
            if self._sequence.state not in {HarvestState.IDLE, HarvestState.DONE}:
                response.success = False
                response.message = "a harvest sequence is already active"
                return response
            if (
                not self._pick_client.server_is_ready()
                or not self._observation_client.service_is_ready()
                or not self._evaluation_client.service_is_ready()
                or not self._home_client.service_is_ready()
            ):
                response.success = False
                response.message = "pick or observation motion server is unavailable"
                return response
            self._sequence.start()
            self._started_monotonic = time.monotonic()
            self._current_target = None
            self._current_goal_pose = None
            self._current_view = None
            self._current_views = []
            self._current_view_index = 0
            self._current_is_reobservation = False
            self._last_observation_position_by_target = {}
            self._distinct_reobservation_targets = set()
            self._last_wrist_rejection_reason = None
            self._recovery_home_in_progress = False
            self._arm_scan_timer()
            self._publish("STARTED")
            response.success = True
            response.message = "continuous harvest started"
            return response

        def _on_tracks(self, message) -> None:
            self._last_candidate_ids = {int(item.track_id) for item in message.targets if int(item.maturity) == int(item.RIPE)}

        def _on_view(self, message) -> None:
            self._latest_view_by_stamp[self._stamp_key(message.header.stamp)] = message
            if len(self._latest_view_by_stamp) > 20:
                oldest = next(iter(self._latest_view_by_stamp))
                self._latest_view_by_stamp.pop(oldest, None)

        def _on_view_plan(self, message) -> None:
            key = self._stamp_key(message.header.stamp)
            self._latest_plan_by_stamp[key] = message
            if len(self._latest_plan_by_stamp) > 20:
                oldest = next(iter(self._latest_plan_by_stamp))
                self._latest_plan_by_stamp.pop(oldest, None)

        def _on_selected(self, target) -> None:
            if (
                self._sequence.state is not HarvestState.SCANNING
                or self._recovery_home_in_progress
            ):
                return
            target_id = int(target.target_id)
            if target_id in self._sequence.completed_ids:
                return
            if self._sequence.retry_target_id is not None and target_id != self._sequence.retry_target_id:
                return
            stamp_key = self._stamp_key(target.header.stamp)
            view_plan = self._latest_plan_by_stamp.get(stamp_key)
            fallback_view = self._latest_view_by_stamp.get(stamp_key)
            if view_plan is not None and int(view_plan.target_id) != target_id:
                return
            views = []
            if view_plan is not None:
                for pose in view_plan.hand_poses:
                    stamped = PoseStamped()
                    stamped.header = view_plan.header
                    stamped.pose = pose
                    views.append(stamped)
            elif fallback_view is not None:
                views.append(fallback_view)
            is_reobservation = target_id in self._distinct_reobservation_targets
            previous_position = self._last_observation_position_by_target.get(target_id)
            if is_reobservation and previous_position is not None:
                ranked_indices = rank_distinct_view_indices(
                    [
                        (
                            float(view.pose.position.x),
                            float(view.pose.position.y),
                            float(view.pose.position.z),
                        )
                        for view in views
                    ],
                    previous_position,
                    minimum_baseline_m=float(
                        self.get_parameter("minimum_reobservation_baseline_m").value
                    ),
                )
                views = [views[index] for index in ranked_indices]
            if not views and not is_reobservation:
                return
            self._cancel_timer("_scan_timer")
            try:
                self._sequence.select(target_id)
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                self._arm_scan_timer()
                return
            self._current_target = target
            self._current_goal_pose = target.pose
            self._current_views = views
            self._current_view_index = 0
            self._current_is_reobservation = is_reobservation
            self._current_view = views[0] if views else None
            if not views:
                minimum_baseline_m = float(
                    self.get_parameter("minimum_reobservation_baseline_m").value
                )
                self._sequence.observation_result(
                    False,
                    "no safe observation view provides the required distinct "
                    f"baseline of {minimum_baseline_m:.3f} m",
                )
                self._publish(
                    "REOBSERVATION_UNAVAILABLE",
                    diagnostics={
                        "minimum_reobservation_baseline_m": minimum_baseline_m,
                        "target_id": target_id,
                    },
                )
                self._after_attempt_failure()
                return
            evaluation = EvaluateTarget.Request()
            evaluation.target_id = target_id
            evaluation.target_pose = PoseStamped()
            evaluation.target_pose.header = target.header
            evaluation.target_pose.pose = target.pose
            future = self._evaluation_client.call_async(evaluation)
            future.add_done_callback(self._evaluation_result)
            self._publish(
                "TARGET_FEASIBILITY_REOBSERVATION"
                if is_reobservation
                else "TARGET_FEASIBILITY",
                diagnostics={
                    "observation_view_count": len(views),
                    "reobservation": is_reobservation,
                },
            )

        def _evaluation_result(self, future) -> None:
            try:
                result = future.result()
                feasible = result is not None and bool(result.feasible)
                detail = result.message if result is not None else "target evaluation service failed"
            except Exception as exc:
                feasible, detail = False, str(exc)
            if not feasible:
                self._sequence.observation_result(False, detail)
                self._after_attempt_failure()
                return
            self._try_current_observation_view()

        def _try_current_observation_view(self) -> None:
            if self._current_target is None or self._current_view_index >= len(self._current_views):
                self._sequence.observation_result(False, "no collision-free wrist observation view")
                self._after_attempt_failure()
                return
            self._current_view = self._current_views[self._current_view_index]
            diagnostics = {
                "view_index": self._current_view_index,
                "view_count": len(self._current_views),
            }
            if self._current_is_reobservation:
                target_id = int(self._current_target.target_id)
                previous_position = self._last_observation_position_by_target.get(
                    target_id
                )
                if previous_position is not None:
                    current = self._current_view.pose.position
                    diagnostics["view_baseline_m"] = math.dist(
                        previous_position,
                        (float(current.x), float(current.y), float(current.z)),
                    )
                    diagnostics["minimum_reobservation_baseline_m"] = float(
                        self.get_parameter("minimum_reobservation_baseline_m").value
                    )
            self._wrist_target_hint.publish(self._current_target)
            request = MoveToObservation.Request()
            request.target_id = int(self._current_target.target_id)
            request.target_pose = PoseStamped()
            request.target_pose.header = self._current_target.header
            request.target_pose.pose = self._current_goal_pose
            request.observation_pose = self._current_view
            future = self._observation_client.call_async(request)
            future.add_done_callback(self._observation_result)
            prefix = "REOBSERVATION_MOTION" if self._current_is_reobservation else "OBSERVATION_MOTION"
            self._publish(
                f"{prefix}_{self._current_view_index + 1}_OF_{len(self._current_views)}",
                diagnostics=diagnostics,
            )

        def _observation_result(self, future) -> None:
            try:
                result = future.result()
                success = result is not None and bool(result.success)
                detail = "observation reached" if success else (result.message if result else "observation service failed")
                execution_time = 0.0 if result is None else float(result.execution_time_sec)
            except Exception as exc:
                success, detail, execution_time = False, str(exc), 0.0
            if not success:
                # Planning/IK/collision rejection is a zero-motion outcome, so
                # the next bounded candidate can be checked safely.  Once an
                # execution actually started, fail closed and do not move on.
                if execution_time <= 1e-6 and self._current_view_index + 1 < len(self._current_views):
                    self._current_view_index += 1
                    self._try_current_observation_view()
                    return
                self._sequence.observation_result(False, detail)
                self._after_attempt_failure()
                return
            if self._current_view is not None:
                position = self._current_view.pose.position
                self._last_observation_position_by_target[
                    int(self._current_target.target_id)
                ] = (
                    float(position.x),
                    float(position.y),
                    float(position.z),
                )
            self._sequence.observation_result(True, detail)
            if not bool(self.get_parameter("require_wrist_confirmation").value):
                self._sequence.confirmation_result(True, "wrist confirmation disabled")
                self._send_pick()
                return
            self._wrist_target_hint.publish(self._current_target)
            self._confirmation_timer = self.create_timer(
                float(self.get_parameter("wrist_confirmation_timeout_sec").value),
                self._confirmation_timeout,
                callback_group=self._group,
            )
            self._publish("WAITING_WRIST_CONFIRMATION")

        def _on_wrist_target(self, refinement) -> None:
            if self._sequence.state is not HarvestState.CONFIRMING or self._current_target is None:
                return
            current = self._current_target.pose.position
            refined = refinement.pose.position
            correction = math.dist((current.x, current.y, current.z), (refined.x, refined.y, refined.z))
            rejection_reason = wrist_refinement_rejection_reason(
                expected_target_id=int(self._current_target.target_id),
                observed_target_id=int(refinement.target_id),
                correction_m=correction,
                confidence=float(refinement.detection_confidence),
                sigma_m=float(refinement.position_sigma_m),
                maximum_correction_m=float(
                    self.get_parameter("wrist_max_correction_m").value
                ),
                minimum_confidence=float(
                    self.get_parameter("wrist_min_confidence").value
                ),
                maximum_sigma_m=float(
                    self.get_parameter("wrist_max_sigma_m").value
                ),
            )
            diagnostics = wrist_refinement_diagnostics(
                expected_target_id=int(self._current_target.target_id),
                observed_target_id=int(refinement.target_id),
                correction_m=correction,
                confidence=float(refinement.detection_confidence),
                sigma_m=float(refinement.position_sigma_m),
                maximum_correction_m=float(
                    self.get_parameter("wrist_max_correction_m").value
                ),
                minimum_confidence=float(
                    self.get_parameter("wrist_min_confidence").value
                ),
                maximum_sigma_m=float(
                    self.get_parameter("wrist_max_sigma_m").value
                ),
            )
            if rejection_reason is not None:
                if rejection_reason != self._last_wrist_rejection_reason:
                    self._last_wrist_rejection_reason = rejection_reason
                    self._publish(
                        f"WRIST_CONFIRMATION_REJECTED_{rejection_reason}",
                        diagnostics=diagnostics,
                    )
                return
            self._last_wrist_rejection_reason = None
            fused_position, fused_sigma = fuse_position_estimates(
                (current.x, current.y, current.z),
                (refined.x, refined.y, refined.z),
                base_sigma_m=float(self._current_target.position_sigma_m),
                wrist_sigma_m=float(refinement.position_sigma_m),
                wrist_systematic_sigma_m=float(
                    self.get_parameter("wrist_systematic_sigma_m").value
                ),
            )
            self._cancel_timer("_confirmation_timer")
            fused_pose = Pose()
            fused_pose.position.x, fused_pose.position.y, fused_pose.position.z = fused_position
            fused_pose.orientation = refinement.pose.orientation
            self._current_goal_pose = fused_pose
            fused_correction = math.dist(
                (current.x, current.y, current.z), fused_position
            )
            self._sequence.confirmation_result(
                True,
                "wrist evidence fused; "
                f"base=({current.x:.4f},{current.y:.4f},{current.z:.4f}); "
                f"wrist=({refined.x:.4f},{refined.y:.4f},{refined.z:.4f}); "
                "fused=("
                f"{fused_position[0]:.4f},{fused_position[1]:.4f},"
                f"{fused_position[2]:.4f}); "
                f"raw_correction={correction:.4f} m; "
                f"fused_correction={fused_correction:.4f} m; "
                f"fused_sigma={fused_sigma:.4f} m",
            )
            self._publish(
                "WRIST_CONFIRMATION_ACCEPTED",
                diagnostics=diagnostics
                | {
                    "fused_correction_m": fused_correction,
                    "fused_sigma_m": fused_sigma,
                },
            )
            self._request_final_pick_feasibility()

        def _request_final_pick_feasibility(self) -> None:
            evaluation = EvaluateTarget.Request()
            evaluation.target_id = int(self._current_target.target_id)
            evaluation.target_pose = PoseStamped()
            evaluation.target_pose.header = self._current_target.header
            evaluation.target_pose.pose = self._current_goal_pose
            future = self._evaluation_client.call_async(evaluation)
            future.add_done_callback(self._final_pick_feasibility_result)
            self._publish("FINAL_PICK_FEASIBILITY")

        def _final_pick_feasibility_result(self, future) -> None:
            try:
                result = future.result()
                feasible = result is not None and bool(result.feasible)
                detail = (
                    result.message
                    if result is not None
                    else "final target evaluation service failed"
                )
                collision = bool(result.collision) if result is not None else False
            except Exception as exc:
                feasible, detail, collision = False, str(exc), False
            if feasible:
                self._send_pick()
                return
            failure_detail = f"final connected pick feasibility failed: {detail}"
            if self._begin_cached_distinct_reobservation(
                "FINAL_PICK_FEASIBILITY_FAILED",
                failure_detail,
                failure_code=7 if collision else 6,
            ):
                return
            if self._sequence.state is not HarvestState.PICKING:
                self._after_attempt_failure()
                return
            self._sequence.pick_result(False, failure_detail, 7 if collision else 6)
            self._after_attempt_failure()

        def _confirmation_timeout(self) -> None:
            self._cancel_timer("_confirmation_timer")
            if self._sequence.state is not HarvestState.CONFIRMING:
                return
            if self._begin_cached_distinct_reobservation(
                "CONFIRMATION_FAILED",
                "wrist confirmation timeout",
            ):
                return
            if self._sequence.state is not HarvestState.CONFIRMING:
                self._after_attempt_failure()
                return
            self._sequence.confirmation_result(False, "wrist confirmation timeout")
            self._after_attempt_failure()

        def _begin_cached_distinct_reobservation(
            self,
            outcome: str,
            detail: str,
            *,
            failure_code: int | None = None,
        ) -> bool:
            if self._current_target is None or self._current_view is None:
                return False
            previous = self._last_observation_position_by_target.get(
                int(self._current_target.target_id)
            )
            if previous is None:
                return False
            minimum_baseline_m = float(
                self.get_parameter("minimum_reobservation_baseline_m").value
            )
            ranked_indices = rank_distinct_view_indices(
                [
                    (
                        float(view.pose.position.x),
                        float(view.pose.position.y),
                        float(view.pose.position.z),
                    )
                    for view in self._current_views
                ],
                previous,
                minimum_baseline_m=minimum_baseline_m,
            )
            if not ranked_indices:
                return False
            if not self._sequence.begin_bounded_reobservation(
                outcome,
                detail,
                failure_code=failure_code,
            ):
                return False
            self._current_views = [
                self._current_views[index] for index in ranked_indices
            ]
            self._current_view_index = 0
            self._current_view = self._current_views[0]
            self._current_is_reobservation = True
            self._last_wrist_rejection_reason = None
            self._publish(
                "CACHED_DISTINCT_REOBSERVATION",
                diagnostics={
                    "target_id": int(self._current_target.target_id),
                    "remaining_view_count": len(self._current_views),
                    "minimum_reobservation_baseline_m": minimum_baseline_m,
                    "trigger": outcome,
                },
            )
            self._try_current_observation_view()
            return True

        def _send_pick(self) -> None:
            goal = PickAndPlace.Goal()
            goal.target_id = int(self._current_target.target_id)
            goal.target_pose = PoseStamped()
            goal.target_pose.header = self._current_target.header
            goal.target_pose.pose = self._current_goal_pose
            goal.place_pose = PoseStamped()
            goal.place_pose.header.frame_id = "panda_link0"
            goal.place_pose.pose.position.x = float(self.get_parameter("place_x").value)
            goal.place_pose.pose.position.y = float(self.get_parameter("place_y").value)
            goal.place_pose.pose.position.z = float(self.get_parameter("place_z").value)
            goal.place_pose.pose.orientation.w = 1.0
            future = self._pick_client.send_goal_async(
                goal,
                feedback_callback=self._pick_feedback,
            )
            future.add_done_callback(self._goal_response)
            self._publish("PICK_SENT")

        def _pick_feedback(self, message) -> None:
            if self._sequence.state is not HarvestState.PICKING:
                return
            feedback = message.feedback
            stage = str(feedback.stage).strip().upper() or "UNKNOWN"
            progress = float(feedback.progress)
            if not math.isfinite(progress):
                self._publish(f"PICK_STAGE_{stage}")
                return
            bounded_progress = min(max(progress, 0.0), 1.0)
            self._publish(
                f"PICK_STAGE_{stage}_{bounded_progress:.2f}"
            )

        def _goal_response(self, future) -> None:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._sequence.pick_result(False, "pick goal rejected", 6)
                self._after_attempt_failure()
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(self._pick_result)

        def _pick_result(self, future) -> None:
            result = future.result().result
            target_id = self._sequence.current_target_id
            self._sequence.pick_result(bool(result.success), str(result.message), int(result.failure_code))
            if result.success:
                self._distinct_reobservation_targets.discard(int(target_id))
                self._last_observation_position_by_target.pop(int(target_id), None)
                completed = UInt32()
                completed.data = int(target_id)
                self._completed.publish(completed)
                self._current_target = None
                self._current_goal_pose = None
                self._current_view = None
                self._current_views = []
                self._current_view_index = 0
                self._current_is_reobservation = False
                self._publish("TARGET_HARVESTED")
                if not self._sequence.terminal:
                    self._arm_scan_timer()
            else:
                self._after_attempt_failure()

        def _after_attempt_failure(self) -> None:
            target_id = None
            if self._sequence.history:
                target_id = self._sequence.history[-1].target_id
            if target_id in self._sequence.skipped:
                self._distinct_reobservation_targets.discard(int(target_id))
                self._last_observation_position_by_target.pop(int(target_id), None)
                completed = UInt32()
                completed.data = int(target_id)
                self._completed.publish(completed)
            elif (
                target_id is not None
                and self._sequence.retry_target_id == target_id
                and target_id in self._last_observation_position_by_target
            ):
                self._distinct_reobservation_targets.add(int(target_id))
            self._current_target = None
            self._current_goal_pose = None
            self._current_view = None
            self._current_views = []
            self._current_view_index = 0
            self._current_is_reobservation = False
            self._publish("RETRY" if self._sequence.retry_target_id else "TARGET_SKIPPED")
            self._request_recovery_home()

        def _request_recovery_home(self) -> None:
            if self._recovery_home_in_progress:
                return
            self._cancel_timer("_scan_timer")
            self._recovery_home_in_progress = True
            self._publish("RETURNING_TO_GLOBAL_SCAN_HOME")
            if not self._home_client.service_is_ready():
                self._finish_after_recovery_home_failure(
                    "recovery home service is unavailable"
                )
                return
            future = self._home_client.call_async(Trigger.Request())
            future.add_done_callback(self._recovery_home_result)

        def _recovery_home_result(self, future) -> None:
            try:
                result = future.result()
                success = result is not None and bool(result.success)
                detail = (
                    result.message
                    if result is not None
                    else "recovery home service failed"
                )
            except Exception as exc:
                success, detail = False, str(exc)
            self._recovery_home_in_progress = False
            if not success:
                self._finish_after_recovery_home_failure(detail)
                return
            self._publish("GLOBAL_SCAN_HOME_REACHED")
            if not self._sequence.terminal:
                self._arm_scan_timer()

        def _finish_after_recovery_home_failure(self, detail: str) -> None:
            self._recovery_home_in_progress = False
            if not self._sequence.terminal:
                self._sequence.finish(
                    f"unsafe to continue after recovery-home failure: {detail}"
                )
            self._publish(
                "RECOVERY_HOME_FAILED",
                diagnostics={"message": detail},
            )

        def _scan_timeout(self) -> None:
            self._cancel_timer("_scan_timer")
            if self._sequence.state is not HarvestState.SCANNING:
                return
            if self._sequence.retry_target_id is not None:
                target_id = self._sequence.retry_unavailable(
                    "retry target was not safely observable before timeout"
                )
                self._distinct_reobservation_targets.discard(int(target_id))
                self._last_observation_position_by_target.pop(int(target_id), None)
                completed = UInt32()
                completed.data = int(target_id)
                self._completed.publish(completed)
                self._publish("TARGET_SKIPPED")
                self._arm_scan_timer()
                return
            outcome = self._sequence.finish("no additional safe target before timeout")
            self._publish(outcome)

    rclpy.init(args=args)
    node = HarvestOrchestrator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown(timeout_sec=10.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
