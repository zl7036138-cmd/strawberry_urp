"""Publish deterministic fruit ground truth from Gazebo PosePublisher topics."""

from __future__ import annotations

from functools import partial
import json
import signal

from .core import Pose3D, load_scene_config, normalize_entity_name


def main(args=None) -> None:  # pragma: no cover - exercised in ROS integration
    try:
        import rclpy
        from geometry_msgs.msg import Pose, PoseArray, PoseStamped
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    def to_pose(message) -> Pose3D:
        return Pose3D(
            x=float(message.translation.x),
            y=float(message.translation.y),
            z=float(message.translation.z),
            qx=float(message.rotation.x),
            qy=float(message.rotation.y),
            qz=float(message.rotation.z),
            qw=float(message.rotation.w),
        )

    def fill_pose(source: Pose3D) -> Pose:
        message = Pose()
        message.position.x = source.x
        message.position.y = source.y
        message.position.z = source.z
        message.orientation.x = source.qx
        message.orientation.y = source.qy
        message.orientation.z = source.qz
        message.orientation.w = source.qw
        return message

    class GroundTruthPublisher(Node):
        def __init__(self) -> None:
            super().__init__("ground_truth_publisher")
            self.declare_parameter("scene_config_file", "")
            self.declare_parameter("publish_rate_hz", 30.0)
            path = str(self.get_parameter("scene_config_file").value)
            if not path:
                raise RuntimeError("scene_config_file parameter is required")
            self._scene = load_scene_config(path)
            self._latest: dict[int, tuple[Pose3D, object]] = {}

            catalog_qos = QoSProfile(depth=1)
            catalog_qos.reliability = ReliabilityPolicy.RELIABLE
            catalog_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._catalog_publisher = self.create_publisher(
                String, "/strawberry/ground_truth/catalog", catalog_qos
            )
            self._array_publisher = self.create_publisher(
                PoseArray,
                "/strawberry/ground_truth/poses",
                qos_profile_sensor_data,
            )
            self._pose_publishers = {
                fruit.target_id: self.create_publisher(
                    PoseStamped,
                    fruit.ground_truth_pose_topic,
                    qos_profile_sensor_data,
                )
                for fruit in self._scene.ordered_fruits
            }
            self._pose_subscriptions = [
                self.create_subscription(
                    TFMessage,
                    fruit.pose_tf_topic,
                    partial(self._on_pose_tf, fruit),
                    qos_profile_sensor_data,
                )
                for fruit in self._scene.ordered_fruits
            ]
            rate = float(self.get_parameter("publish_rate_hz").value)
            if rate <= 0.0:
                raise RuntimeError("publish_rate_hz must be positive")
            self._timer = self.create_timer(1.0 / rate, self._publish_array)
            self._publish_catalog()

        def _publish_catalog(self) -> None:
            message = String()
            message.data = json.dumps(
                {
                    "schema_version": 1,
                    "frame_id": self._scene.base_frame,
                    "pose_array_order": [
                        fruit.target_id for fruit in self._scene.ordered_fruits
                    ],
                    "fruits": [
                        {
                            "target_id": fruit.target_id,
                            "model_name": fruit.model_name,
                            "maturity": fruit.maturity,
                            "pose_topic": fruit.ground_truth_pose_topic,
                        }
                        for fruit in self._scene.ordered_fruits
                    ],
                },
                separators=(",", ":"),
                sort_keys=True,
            )
            self._catalog_publisher.publish(message)

        def _on_pose_tf(self, fruit, message) -> None:
            if not message.transforms:
                return
            # A model-local PosePublisher normally emits exactly one model
            # transform. Prefer the expected model name, then use that sole
            # transform. Never guess when an unexpected multi-transform packet
            # arrives.
            expected = fruit.model_name
            matches = [
                transform
                for transform in message.transforms
                if normalize_entity_name(transform.child_frame_id) == expected
            ]
            if len(matches) == 1:
                transform = matches[0]
            elif len(message.transforms) == 1:
                transform = message.transforms[0]
            else:
                self.get_logger().error(
                    f"ambiguous PosePublisher packet for target {fruit.target_id}"
                )
                return

            stamp = transform.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                stamp = self.get_clock().now().to_msg()
            pose = to_pose(transform.transform)
            self._latest[fruit.target_id] = (pose, stamp)

            output = PoseStamped()
            output.header.stamp = stamp
            output.header.frame_id = self._scene.base_frame
            output.pose = fill_pose(pose)
            self._pose_publishers[fruit.target_id].publish(output)

        def _publish_array(self) -> None:
            # PoseArray has no identity field. Publish it only after every fruit
            # has a live Gazebo sample, preserving the catalog's fixed order.
            if len(self._latest) != len(self._scene.fruits):
                return
            message = PoseArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self._scene.base_frame
            message.poses = [
                fill_pose(self._latest[fruit.target_id][0])
                for fruit in self._scene.ordered_fruits
            ]
            self._array_publisher.publish(message)

    rclpy.init(args=args)
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
