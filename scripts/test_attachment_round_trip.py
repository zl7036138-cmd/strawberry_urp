#!/usr/bin/env python3
"""Exercise the guarded DetachableJoint attach/detach round trip in simulation."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def call_trigger(
    node: Node,
    client,
    *,
    timeout_sec: float,
    contact_publishers=(),
):
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + timeout_sec
    contact_message = Bool(data=True)
    while not future.done():
        for publisher in contact_publishers:
            publisher.publish(contact_message)
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.monotonic() >= deadline:
            raise TimeoutError("simulation Trigger service timed out")
    return future.result()


def wait_until_ready(node: Node, detach_client, timeout_sec: float):
    deadline = time.monotonic() + timeout_sec
    last_message = "service not called"
    while time.monotonic() < deadline:
        response = call_trigger(node, detach_client, timeout_sec=2.0)
        last_message = response.message
        if response.success:
            return response
        time.sleep(0.1)
    raise RuntimeError(f"attachment backend did not become ready: {last_message}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-id", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    args = parser.parse_args()
    if args.target_id <= 0 or args.timeout_sec <= 0.0:
        parser.error("target-id and timeout-sec must be positive")

    prefix = f"/strawberry/sim/fruit_{args.target_id}"
    rclpy.init()
    node = Node("attachment_round_trip_test")
    left = node.create_publisher(Bool, f"{prefix}/left_contact", 10)
    right = node.create_publisher(Bool, f"{prefix}/right_contact", 10)
    attach = node.create_client(Trigger, f"{prefix}/attach")
    detach = node.create_client(Trigger, f"{prefix}/detach")
    report: dict[str, object] = {"target_id": args.target_id}

    try:
        if not attach.wait_for_service(timeout_sec=args.timeout_sec):
            raise RuntimeError("attach service is unavailable")
        if not detach.wait_for_service(timeout_sec=args.timeout_sec):
            raise RuntimeError("detach service is unavailable")

        ready = wait_until_ready(node, detach, args.timeout_sec)
        report["initial_detach"] = ready.message

        attached = call_trigger(
            node,
            attach,
            timeout_sec=args.timeout_sec,
            contact_publishers=(left, right),
        )
        report["attach"] = attached.message
        if not attached.success:
            raise RuntimeError(f"attach failed: {attached.message}")

        detached = call_trigger(node, detach, timeout_sec=args.timeout_sec)
        report["detach"] = detached.message
        if not detached.success:
            raise RuntimeError(f"detach failed: {detached.message}")

        report["success"] = True
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
