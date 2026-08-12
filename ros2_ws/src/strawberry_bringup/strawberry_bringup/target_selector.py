"""ROS 2 generalized target selector without oracle or fixed ROI dependencies."""

from __future__ import annotations

import json
import math

from .harvest_planning import (
    HarvestCandidate,
    ViewAssessment,
    generate_dynamic_views,
    hand_pose_for_optical_view,
    rank_dynamic_views,
    rank_safe_targets,
    target_rejection_reasons,
)


REACH_BOUNDS = (0.30, 0.72, -0.34, 0.34, 0.48, 0.66)
OBSERVATION_REACH_BOUNDS = (0.12, 0.78, -0.55, 0.55, 0.30, 0.90)


def geometric_clearance(position, neighbours, fruit_radius_m: float) -> float:
    distances = [
        math.dist(position, neighbour) - 2.0 * fruit_radius_m
        for neighbour in neighbours
    ]
    return max(0.0, min(distances)) if distances else 1.0


def axis_aligned_box_clearance(position, bounds) -> float:
    """Euclidean point clearance from an axis-aligned forbidden volume."""

    if len(position) != 3 or len(bounds) != 6:
        raise ValueError("position and bounds must contain three and six values")
    min_x, max_x, min_y, max_y, min_z, max_z = (float(value) for value in bounds)
    if min_x >= max_x or min_y >= max_y or min_z >= max_z:
        raise ValueError("axis-aligned bounds are invalid")
    x, y, z = (float(value) for value in position)
    deltas = (
        max(min_x - x, 0.0, x - max_x),
        max(min_y - y, 0.0, y - max_y),
        max(min_z - z, 0.0, z - max_z),
    )
    return math.sqrt(sum(value * value for value in deltas))


def expanded_bin_bounds(interior_bounds) -> tuple[float, ...]:
    """Convert the manifest's interior volume into a conservative outer box."""

    if len(interior_bounds) != 6:
        raise ValueError("bin interior bounds must contain six values")
    min_x, max_x, min_y, max_y, min_z, max_z = (
        float(value) for value in interior_bounds
    )
    return (
        min_x - 0.05,
        max_x + 0.05,
        min_y - 0.05,
        max_y + 0.05,
        min_z - 0.03,
        max_z,
    )


def inside_conservative_reach(position) -> bool:
    min_x, max_x, min_y, max_y, min_z, max_z = REACH_BOUNDS
    x, y, z = position
    return min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z


def inside_observation_reach(position) -> bool:
    """Reject only wrist views that are certainly outside Panda's workspace.

    Observation poses naturally sit between the arm base and the fruit, so the
    narrower fruit-center bounds must not be reused here. MoveIt remains the
    authority for IK, joint-limit, and collision feasibility.
    """

    min_x, max_x, min_y, max_y, min_z, max_z = OBSERVATION_REACH_BOUNDS
    x, y, z = position
    return min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z


def main(args=None) -> None:  # pragma: no cover - exercised in ROS integration
    try:
        import rclpy
        from geometry_msgs.msg import Pose, PoseStamped
        from rclpy.node import Node
        from std_msgs.msg import String, UInt32
        from strawberry_interfaces.msg import (
            ObservationPlan,
            TargetPose,
            TrackedTargetArray,
        )
        from strawberry_sim.core import load_scene_config
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    class TargetSelector(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_target_selector")
            self.declare_parameter(
                "tracked_targets_topic", "/strawberry/tracked_targets"
            )
            self.declare_parameter("selected_target_topic", "/strawberry/target_pose")
            self.declare_parameter(
                "observation_pose_topic", "/strawberry/wrist_observation_pose"
            )
            self.declare_parameter(
                "observation_plan_topic", "/strawberry/wrist_observation_plan"
            )
            self.declare_parameter("status_topic", "/strawberry/selection_status")
            self.declare_parameter("confidence_threshold", 0.60)
            self.declare_parameter("maximum_sigma_m", 0.015)
            self.declare_parameter("minimum_observations", 3)
            self.declare_parameter("maximum_age_sec", 0.50)
            self.declare_parameter("minimum_clearance_m", 0.02)
            self.declare_parameter("fruit_radius_m", 0.026)
            self.declare_parameter("static_obstacle_margin_m", 0.05)
            self.declare_parameter("scene_config_file", "")
            scene_path = str(self.get_parameter("scene_config_file").value)
            if not scene_path:
                raise RuntimeError(
                    "scene_config_file is required for static-obstacle clearance"
                )
            scene = load_scene_config(scene_path)
            bounds = scene.bin_bounds
            self._bin_bounds = expanded_bin_bounds(
                (
                    bounds.min_x,
                    bounds.max_x,
                    bounds.min_y,
                    bounds.max_y,
                    bounds.min_z,
                    bounds.max_z,
                )
            )
            self._excluded = set()
            self._target_publisher = self.create_publisher(
                TargetPose, str(self.get_parameter("selected_target_topic").value), 10
            )
            self._view_publisher = self.create_publisher(
                PoseStamped, str(self.get_parameter("observation_pose_topic").value), 10
            )
            self._view_plan_publisher = self.create_publisher(
                ObservationPlan,
                str(self.get_parameter("observation_plan_topic").value),
                10,
            )
            self._status_publisher = self.create_publisher(
                String, str(self.get_parameter("status_topic").value), 10
            )
            self.create_subscription(
                TrackedTargetArray,
                str(self.get_parameter("tracked_targets_topic").value),
                self._on_targets,
                10,
            )
            self.create_subscription(
                UInt32,
                "/strawberry/completed_track_id",
                self._on_completed_track,
                10,
            )

        def _on_completed_track(self, message) -> None:
            if int(message.data) > 0:
                self._excluded.add(int(message.data))

        @staticmethod
        def _seconds(stamp) -> float:
            return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

        def _on_targets(self, message) -> None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            radius = float(self.get_parameter("fruit_radius_m").value)
            positions = {
                int(item.track_id): (
                    float(item.pose.position.x),
                    float(item.pose.position.y),
                    float(item.pose.position.z),
                )
                for item in message.targets
            }
            candidates = []
            for item in message.targets:
                track_id = int(item.track_id)
                position = positions[track_id]
                neighbours = [
                    value
                    for other_id, value in positions.items()
                    if other_id != track_id
                ]
                reachable = inside_conservative_reach(position)
                clearance = min(
                    geometric_clearance(position, neighbours, radius),
                    max(
                        0.0,
                        axis_aligned_box_clearance(position, self._bin_bounds)
                        - float(self.get_parameter("static_obstacle_margin_m").value),
                    ),
                )
                candidates.append(
                    HarvestCandidate(
                        track_id=track_id,
                        maturity=int(item.maturity),
                        position=position,
                        confidence=float(item.detection_confidence),
                        sigma_m=float(item.position_sigma_m),
                        observation_count=int(item.observation_count),
                        last_seen_sec=self._seconds(item.header.stamp),
                        clearance_m=clearance,
                        joint_travel_rad=math.dist((0.0, 0.0, 0.55), position),
                        pregrasp_feasible=reachable,
                        grasp_feasible=reachable,
                        retreat_feasible=reachable,
                    )
                )
            selection_limits = {
                "confidence_threshold": float(
                    self.get_parameter("confidence_threshold").value
                ),
                "maximum_sigma_m": float(self.get_parameter("maximum_sigma_m").value),
                "minimum_observations": int(
                    self.get_parameter("minimum_observations").value
                ),
                "maximum_age_sec": float(self.get_parameter("maximum_age_sec").value),
                "minimum_clearance_m": float(
                    self.get_parameter("minimum_clearance_m").value
                ),
                "excluded_track_ids": self._excluded,
            }
            ranked = rank_safe_targets(
                candidates,
                now_sec=now_sec,
                **selection_limits,
            )
            status = String()
            if not ranked:
                status.data = json.dumps(
                    {
                        "schema_version": 1,
                        "outcome": "NO_PICK",
                        "candidate_count": len(candidates),
                        "rejections": [
                            {
                                "track_id": candidate.track_id,
                                "reasons": list(
                                    target_rejection_reasons(
                                        candidate,
                                        now_sec=now_sec,
                                        **selection_limits,
                                    )
                                ),
                                "confidence": candidate.confidence,
                                "sigma_m": candidate.sigma_m,
                                "observation_count": candidate.observation_count,
                                "age_sec": now_sec - candidate.last_seen_sec,
                                "clearance_m": candidate.clearance_m,
                            }
                            for candidate in candidates
                        ],
                    },
                    separators=(",", ":"),
                )
                self._status_publisher.publish(status)
                return
            selected = ranked[0]

            def assess_view(view):
                reachable = inside_observation_reach(view.position)
                return ViewAssessment(
                    ik_reachable=reachable,
                    collision_free=reachable,
                    target_in_view=True,
                    clearance_m=selected.clearance_m,
                    joint_travel_rad=math.dist((0.0, 0.0, 0.55), view.position),
                )

            ranked_views = rank_dynamic_views(
                generate_dynamic_views(selected.track_id, selected.position),
                assess_view,
            )
            if not ranked_views:
                status.data = json.dumps(
                    {
                        "schema_version": 1,
                        "outcome": "NO_PICK",
                        "reason": "NO_SAFE_WRIST_VIEW",
                    }
                )
                self._status_publisher.publish(status)
                return
            view, assessment = ranked_views[0]
            hand_views = tuple(
                hand_pose_for_optical_view(candidate_view)
                for candidate_view, _ in ranked_views
            )
            hand_view = hand_views[0]
            target = TargetPose()
            target.header = message.header
            target.target_id = selected.track_id
            target.pose.position.x, target.pose.position.y, target.pose.position.z = (
                selected.position
            )
            target.pose.orientation.w = 1.0
            target.detection_confidence = selected.confidence
            target.position_sigma_m = selected.sigma_m
            view_message = PoseStamped()
            view_message.header = message.header
            (
                view_message.pose.position.x,
                view_message.pose.position.y,
                view_message.pose.position.z,
            ) = hand_view.position
            (
                view_message.pose.orientation.x,
                view_message.pose.orientation.y,
                view_message.pose.orientation.z,
                view_message.pose.orientation.w,
            ) = hand_view.quaternion_xyzw
            view_plan = ObservationPlan()
            view_plan.header = message.header
            view_plan.target_id = selected.track_id
            for candidate in hand_views:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = candidate.position
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ) = candidate.quaternion_xyzw
                view_plan.hand_poses.append(pose)
            # Publish the full bounded bank first.  The orchestrator asks the
            # MoveIt service to check candidates one by one without motion on
            # failed plans, and moves only to the first genuinely safe view.
            self._view_plan_publisher.publish(view_plan)
            self._view_publisher.publish(view_message)
            # Publish the target after its stamp-matched observation pose so a
            # batch orchestrator can consume the pair without a fixed preset.
            self._target_publisher.publish(target)
            status.data = json.dumps(
                {
                    "schema_version": 1,
                    "outcome": "TARGET_SELECTED",
                    "track_id": selected.track_id,
                    "clearance_m": selected.clearance_m,
                    "view_joint_travel_rad": assessment.joint_travel_rad,
                    "observation_candidate_count": len(hand_views),
                },
                separators=(",", ":"),
            )
            self._status_publisher.publish(status)

    rclpy.init(args=args)
    node = TargetSelector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
