"""Publish one explicitly labelled oracle TargetPose from simulation truth."""

from __future__ import annotations

from .core import parse_oracle_catalog, select_oracle_pose_index


def main(args=None) -> None:  # pragma: no cover - exercised in ROS integration
    try:
        import rclpy
        from geometry_msgs.msg import PoseArray
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from std_msgs.msg import String
        from strawberry_interfaces.msg import TargetPose
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    class OracleTargetProvider(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_oracle_target_provider")
            self.declare_parameter(
                "output_topic", "/strawberry/oracle/target_pose"
            )
            self.declare_parameter("oracle_target_id", 1)
            output_topic = str(self.get_parameter("output_topic").value)
            if not output_topic.startswith("/"):
                raise RuntimeError("output_topic must be an absolute ROS topic")
            requested_id = int(self.get_parameter("oracle_target_id").value)
            if requested_id < 0:
                raise RuntimeError("oracle_target_id cannot be negative")
            self._requested_id = requested_id
            self._catalog = None
            self._publisher = self.create_publisher(TargetPose, output_topic, 10)

            catalog_qos = QoSProfile(depth=1)
            catalog_qos.reliability = ReliabilityPolicy.RELIABLE
            catalog_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                String,
                "/strawberry/ground_truth/catalog",
                self._on_catalog,
                catalog_qos,
            )
            self.create_subscription(
                PoseArray,
                "/strawberry/ground_truth/poses",
                self._on_poses,
                qos_profile_sensor_data,
            )

        def _on_catalog(self, message) -> None:
            try:
                self._catalog = parse_oracle_catalog(message.data)
            except ValueError as exc:
                self._catalog = None
                self.get_logger().error(str(exc))

        def _on_poses(self, message) -> None:
            if self._catalog is None:
                return
            if message.header.frame_id != self._catalog.frame_id:
                self.get_logger().error("oracle PoseArray frame differs from catalog")
                return
            try:
                index = select_oracle_pose_index(
                    self._catalog,
                    len(message.poses),
                    self._requested_id,
                )
            except ValueError as exc:
                self.get_logger().error(str(exc))
                return
            if index is None:
                return
            fruit = self._catalog.fruits[index]
            result = TargetPose()
            result.header = message.header
            result.target_id = fruit.target_id
            result.pose = message.poses[index]
            result.detection_confidence = 1.0
            result.position_sigma_m = 1.0e-6
            self._publisher.publish(result)

    rclpy.init(args=args)
    node = OracleTargetProvider()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
