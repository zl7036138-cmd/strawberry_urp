#!/usr/bin/env python3
"""Configure and verify all fruit poses for one no-motion P4 scenario."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-pose", nargs=5, action="append", required=True,
                        metavar=("MODEL", "ID", "X", "Y", "Z"))
    parser.add_argument("--target-id", type=int, required=True)
    parser.add_argument("--maturity", choices=("RIPE", "UNRIPE"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--settled-camera-frames", type=int, default=10)
    return parser.parse_args(argv)


def main(args=None) -> int:  # pragma: no cover - ROS integration
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.utilities import remove_ros_args
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
    from sensor_msgs.msg import Image

    options = _parse_args(remove_ros_args(args=args)[1:])
    if options.output.exists():
        raise ValueError(f"refusing to overwrite scene receipt: {options.output}")
    if len(options.model_pose) != 3:
        raise ValueError("exactly three fruit model poses are required")
    models = []
    for raw in options.model_pose:
        name, identity_raw, x_raw, y_raw, z_raw = raw
        identity = int(identity_raw)
        position = (float(x_raw), float(y_raw), float(z_raw))
        if not all(math.isfinite(value) for value in position):
            raise ValueError("model poses must be finite")
        models.append((name, identity, position))
    if {item[0] for item in models} != {"strawberry_1", "strawberry_2", "strawberry_3"} or {item[1] for item in models} != {1, 2, 3}:
        raise ValueError("model identities must be strawberry_1..3 / IDs 1..3")
    if options.target_id not in {item[1] for item in models}:
        raise ValueError("target ID is absent from model poses")

    class SceneNode(Node):
        def __init__(self):
            super().__init__("p4_qualification_scene_configurator")
            self.set_parameters([Parameter("use_sim_time", value=True)])
            self.truth = {}
            self.image_count = 0
            self.client = self.create_client(SetEntityPose, "/world/strawberry_orchard/set_pose")
            for identity in (1, 2, 3):
                self.create_subscription(
                    PoseStamped, f"/strawberry/ground_truth/fruit_{identity}/pose",
                    lambda message, selected=identity: self.truth.__setitem__(selected, message),
                    qos_profile_sensor_data,
                )
            self.create_subscription(
                Image, "/camera/color/image_raw", self._image, qos_profile_sensor_data
            )

        def _image(self, _message):
            self.image_count += 1

        def wait_for(self, predicate, timeout, description):
            deadline = time.monotonic() + timeout
            while rclpy.ok() and time.monotonic() < deadline:
                if predicate():
                    return
                rclpy.spin_once(self, timeout_sec=0.05)
            if not predicate():
                raise RuntimeError(f"timed out waiting for {description}")

        def set_pose(self, name, position):
            for attempt in (1, 2):
                request = SetEntityPose.Request()
                request.entity.name = name
                request.entity.type = Entity.MODEL
                request.pose.position.x, request.pose.position.y, request.pose.position.z = position
                request.pose.orientation.w = 1.0
                future = self.client.call_async(request)
                try:
                    self.wait_for(future.done, 5.0, f"set_pose for {name}")
                except RuntimeError:
                    remove = getattr(self.client, "remove_pending_request", None)
                    if callable(remove):
                        remove(future)
                    if attempt == 2:
                        raise
                    continue
                if future.exception() is not None or not future.result().success:
                    raise RuntimeError(f"Gazebo rejected set_pose for {name}")
                return attempt
            raise AssertionError("unreachable")

        def xyz(self, identity):
            p = self.truth[identity].pose.position
            return (float(p.x), float(p.y), float(p.z))

    rclpy.init(args=args)
    node = SceneNode()
    attempts = []
    try:
        node.wait_for(lambda: len(node.truth) == 3 and node.image_count > 0 and node.client.service_is_ready(), options.timeout_sec, "truth, camera, and pose service")
        for name, identity, position in models:
            attempts.append({"model_name": name, "target_id": identity, "attempts": node.set_pose(name, position)})
        node.wait_for(
            lambda: all(
                identity in node.truth and math.dist(node.xyz(identity), position) <= 0.001
                for _, identity, position in models
            ),
            options.timeout_sec,
            "configured fruit poses",
        )
        start = node.image_count
        node.wait_for(lambda: node.image_count >= start + options.settled_camera_frames, options.timeout_sec, "settled camera frames")
        payload = {
            "schema_version": 1,
            "kind": "p4_qualification_scene_configuration",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "maturity": options.maturity,
            "target_id": options.target_id,
            "robot_motion_started": False,
            "manipulation_started": False,
            "orchestrator_started": False,
            "settled_camera_frames": options.settled_camera_frames,
            "pose_configuration_attempts": attempts,
            "configured_models": [
                {"model_name": name, "target_id": identity, "position_m": list(node.xyz(identity))}
                for name, identity, _ in models
            ],
        }
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"configured": True, "maturity": options.maturity, "target_id": options.target_id}, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
