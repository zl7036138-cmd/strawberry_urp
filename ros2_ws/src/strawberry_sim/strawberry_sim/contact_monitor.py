"""Convert Gazebo contact arrays into per-fruit dual-finger Boolean topics."""

from __future__ import annotations

from functools import partial
import signal

from .core import (
    fruit_models_contacting_entity,
    fruit_models_in_contacts,
    load_scene_config,
)


def main(args=None) -> None:  # pragma: no cover - exercised in ROS integration
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from ros_gz_interfaces.msg import Contacts
        from std_msgs.msg import Bool
    except ImportError as exc:
        raise RuntimeError(
            "contact monitor requires ROS 2 and ros_gz_interfaces"
        ) from exc

    class ContactMonitor(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_contact_monitor")
            self.declare_parameter("scene_config_file", "")
            config_path = str(self.get_parameter("scene_config_file").value)
            if not config_path:
                raise RuntimeError("scene_config_file parameter is required")
            scene = load_scene_config(config_path)
            self._models = {
                fruit.target_id: fruit.model_name for fruit in scene.ordered_fruits
            }
            self._contact_publishers = {
                (side, target_id): self.create_publisher(
                    Bool,
                    f"/strawberry/sim/fruit_{target_id}/{side}_contact",
                    qos_profile_sensor_data,
                )
                for side in ("left", "right")
                for target_id in self._models
            }
            self._bin_contact_publishers = {
                target_id: self.create_publisher(
                    Bool,
                    f"/strawberry/sim/fruit_{target_id}/bin_contact",
                    qos_profile_sensor_data,
                )
                for target_id in self._models
            }
            self._contact_subscriptions = [
                self.create_subscription(
                    Contacts,
                    f"/strawberry/sim/gripper/{side}_contacts",
                    partial(self._on_contacts, side),
                    qos_profile_sensor_data,
                )
                for side in ("left", "right")
            ]
            self._contact_subscriptions.append(
                self.create_subscription(
                    Contacts,
                    "/strawberry/sim/fruit_contacts",
                    self._on_fruit_contacts,
                    qos_profile_sensor_data,
                )
            )

        def _on_contacts(self, side: str, message) -> None:
            pairs = [
                (contact.collision1.name, contact.collision2.name)
                for contact in message.contacts
            ]
            active = fruit_models_in_contacts(pairs, self._models)
            for target_id in self._models:
                output = Bool()
                output.data = target_id in active
                self._contact_publishers[(side, target_id)].publish(output)

        def _on_fruit_contacts(self, message) -> None:
            pairs = [
                (contact.collision1.name, contact.collision2.name)
                for contact in message.contacts
            ]
            active = fruit_models_contacting_entity(
                pairs,
                self._models,
                "floor_collision",
            )
            # Every fruit owns a sensor but all sensors share this Gazebo
            # topic. A message about fruit B is therefore not negative
            # evidence for fruit A. Publish positive floor-contact events
            # only; the consumer's freshness timer expires missing events.
            for target_id in active:
                output = Bool()
                output.data = True
                self._bin_contact_publishers[target_id].publish(output)

    rclpy.init(args=args)
    node = ContactMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
