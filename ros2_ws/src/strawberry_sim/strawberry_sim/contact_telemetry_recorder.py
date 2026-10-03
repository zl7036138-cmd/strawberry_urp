"""Record /strawberry/sim/fruit_contacts (gz contact sensor data) to JSONL.

Development telemetry only. Appends one line per Contacts message:
{stamp_sec (gz sim time), contacts: [{c1, c2, n_points, force_norm, pos_x, pos_y, pos_z}]}
Exclusive-claim output file (never truncates existing evidence).
"""
from __future__ import annotations

import json
import math
import signal


def main(args=None) -> None:  # pragma: no cover - exercised in ROS integration
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from ros_gz_interfaces.msg import Contacts
    except ImportError as exc:
        raise RuntimeError("contact telemetry recorder requires ROS 2 + ros_gz_interfaces") from exc
    class ContactRecorder(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_contact_telemetry")
            self.declare_parameter("output_file", "")
            output = str(self.get_parameter("output_file").value)
            if not output:
                raise RuntimeError("output_file parameter is required")
            self._stream = open(output, "x", encoding="utf-8")  # exclusive: refuse overwrite
            self._count = 0
            self.create_subscription(
                Contacts,
                "/strawberry/sim/fruit_contacts",
                self._on_contacts,
                qos_profile_sensor_data,
            )

        def _on_contacts(self, msg: Contacts) -> None:
            row = {
                "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                "contacts": [],
            }
            for c in msg.contacts:
                if c.wrenches:
                    w = c.wrenches[0]
                    # JointWrench: body1_wrench / body2_wrench (each a Wrench)
                    force = 0.0
                    for body in ("body1_wrench", "body2_wrench"):
                        bw = getattr(w, body, None)
                        if bw is not None:
                            f = bw.force
                            force += math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
                else:
                    force = 0.0
                pos = (
                    c.positions[0].x if c.positions else 0.0,
                    c.positions[0].y if c.positions else 0.0,
                    c.positions[0].z if c.positions else 0.0,
                )
                row["contacts"].append(
                    {
                        "c1": c.collision1.name,
                        "c2": c.collision2.name,
                        "n_points": len(c.positions),
                        "force_norm": force,
                        "pos": pos,
                    }
                )
            self._stream.write(json.dumps(row, separators=(",", ":")) + "\n")
            self._count += 1
            if self._count % 200 == 0:
                self._stream.flush()

        def close(self) -> None:
            self._stream.flush()
            self._stream.close()

    rclpy.init(args=args)
    node = ContactRecorder()
    stop = [False]

    def _sig(signum, frame):
        del signum, frame
        stop[0] = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    try:
        while not stop[0]:
            rclpy.spin_once(node, timeout_sec=0.5)
    except ExternalShutdownException:
        pass
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
