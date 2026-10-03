"""Audit a live generalized ROS graph for forbidden truth subscriptions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Mapping, Sequence


TRUTH_TOPIC_PREFIX = "/strawberry/ground_truth"
DEFAULT_ALLOWED_NODES = frozenset({"/attachment_manager"})


def normalized_node_name(name: str, namespace: str = "/") -> str:
    namespace = "/" + str(namespace).strip("/") if str(namespace).strip("/") else ""
    return f"{namespace}/{str(name).strip('/')}"


def audit_truth_subscriptions(
    subscriptions: Mapping[str, Sequence[str]],
    *,
    allowed_nodes: Sequence[str] = tuple(DEFAULT_ALLOWED_NODES),
) -> tuple[dict[str, object], ...]:
    """Return every truth subscription owned by a non-allowlisted node."""

    allowed = {normalized_node_name(value) for value in allowed_nodes}
    violations = []
    for node_name, topics in subscriptions.items():
        normalized = normalized_node_name(node_name)
        for topic in topics:
            topic_name = "/" + str(topic).strip("/")
            if topic_name.startswith(TRUTH_TOPIC_PREFIX) and normalized not in allowed:
                violations.append({"node": normalized, "topic": topic_name})
    return tuple(sorted(violations, key=lambda row: (row["node"], row["topic"])))


def build_audit_payload(
    subscriptions: Mapping[str, Sequence[str]],
    *,
    allowed_nodes: Sequence[str],
    required_nodes: Sequence[str],
) -> dict[str, object]:
    normalized_subscriptions = {
        normalized_node_name(node): sorted({"/" + topic.strip("/") for topic in topics})
        for node, topics in subscriptions.items()
    }
    required = sorted({normalized_node_name(node) for node in required_nodes})
    missing = [node for node in required if node not in normalized_subscriptions]
    violations = audit_truth_subscriptions(
        normalized_subscriptions, allowed_nodes=allowed_nodes
    )
    return {
        "schema_version": 1,
        "scope": "GENERALIZED_RUNTIME_TRUTH_ISOLATION_AUDIT",
        "truth_topic_prefix": TRUTH_TOPIC_PREFIX,
        "allowed_nodes": sorted(
            {normalized_node_name(node) for node in allowed_nodes}
        ),
        "required_nodes": required,
        "missing_required_nodes": missing,
        "subscriptions": normalized_subscriptions,
        "violations": list(violations),
        "overall_pass": not missing and not violations,
    }


def main(argv=None) -> int:  # pragma: no cover - exercised in ROS integration
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--allowed-node", action="append", default=list(DEFAULT_ALLOWED_NODES)
    )
    parser.add_argument("--required-node", action="append", default=[])
    options = parser.parse_args(argv)
    if options.output.exists():
        raise FileExistsError(options.output)
    if options.timeout <= 0.0:
        raise ValueError("timeout must be positive")

    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    rclpy.init()
    node = Node("generalized_truth_isolation_auditor")
    deadline = time.monotonic() + options.timeout
    payload = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            subscriptions = {}
            for name, namespace in node.get_node_names_and_namespaces():
                full_name = normalized_node_name(name, namespace)
                try:
                    entries = node.get_subscriber_names_and_types_by_node(
                        name, namespace
                    )
                except RuntimeError:
                    continue
                subscriptions[full_name] = [topic for topic, _types in entries]
            payload = build_audit_payload(
                subscriptions,
                allowed_nodes=options.allowed_node,
                required_nodes=options.required_node,
            )
            if not payload["missing_required_nodes"]:
                break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if payload is None:
        payload = build_audit_payload(
            {},
            allowed_nodes=options.allowed_node,
            required_nodes=options.required_node,
        )
    options.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = options.output.with_suffix(options.output.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(options.output)
    print(json.dumps({"overall_pass": payload["overall_pass"], "output": str(options.output)}))
    return 0 if payload["overall_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
