"""Bounded recorder for one non-formal generalized harvest run."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Mapping


TERMINAL_STATE = "DONE"


def event_progress_signature(event: Mapping[str, object]) -> tuple[object, ...]:
    """Return fields that prove the batch or current action made progress."""

    return (
        event.get("state"),
        event.get("outcome"),
        event.get("current_target_id"),
        json.dumps(event.get("harvested_target_ids", []), sort_keys=True),
        json.dumps(event.get("skipped_targets", {}), sort_keys=True),
        json.dumps(event.get("failures", []), sort_keys=True),
    )


@dataclass
class ProbeWatchdog:
    """Combine an inactivity timeout with a non-extendable hard deadline."""

    started_monotonic: float
    startup_timeout_sec: float
    idle_timeout_sec: float
    hard_timeout_sec: float
    last_progress_monotonic: float | None = None
    last_signature: tuple[object, ...] | None = None

    def __post_init__(self) -> None:
        values = (
            self.started_monotonic,
            self.startup_timeout_sec,
            self.idle_timeout_sec,
            self.hard_timeout_sec,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("probe watchdog values must be finite")
        if self.startup_timeout_sec <= 0.0 or self.idle_timeout_sec <= 0.0:
            raise ValueError("probe inactivity timeouts must be positive")
        if self.hard_timeout_sec < max(
            self.startup_timeout_sec, self.idle_timeout_sec
        ):
            raise ValueError("hard timeout must cover all inactivity timeouts")

    def observe(self, event: Mapping[str, object], now_monotonic: float) -> bool:
        """Record one new meaningful status signature."""

        if not math.isfinite(now_monotonic):
            raise ValueError("probe observation time must be finite")
        signature = event_progress_signature(event)
        if signature == self.last_signature:
            return False
        self.last_signature = signature
        self.last_progress_monotonic = now_monotonic
        return True

    def timeout_outcome(self, now_monotonic: float) -> str | None:
        """Return a stable timeout code, giving the hard limit precedence."""

        elapsed = now_monotonic - self.started_monotonic
        if elapsed >= self.hard_timeout_sec:
            return "HARD_TIMEOUT"
        if self.last_progress_monotonic is None:
            if elapsed >= self.startup_timeout_sec:
                return "STARTUP_TIMEOUT"
            return None
        if now_monotonic - self.last_progress_monotonic >= self.idle_timeout_sec:
            return "INACTIVITY_TIMEOUT"
        return None


def recorder_exit_code(outcome: str) -> int:
    """Map terminal probe outcomes to a shell-visible status."""

    if outcome in {"SUCCESS", "PARTIAL_SUCCESS", "NO_PICK"}:
        return 0
    if outcome == "START_REJECTED":
        return 2
    if outcome in {"STARTUP_TIMEOUT", "INACTIVITY_TIMEOUT", "HARD_TIMEOUT"}:
        return 3
    return 4


def build_probe_payload(
    *,
    outcome: str,
    events: list[dict],
    selection_events: list[dict],
    ground_truth_score_events: list[dict],
    elapsed_sec: float,
    startup_timeout_sec: float,
    idle_timeout_sec: float,
    hard_timeout_sec: float,
) -> dict:
    """Build the auditable development receipt."""

    return {
        "schema_version": 3,
        "scope": "GENERALIZED_DEVELOPMENT_RUNTIME_PROBE",
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "recorder_outcome": outcome,
        "terminal_status_received": bool(
            events and events[-1].get("state") == TERMINAL_STATE
        ),
        "elapsed_wall_sec": float(elapsed_sec),
        "timeouts_sec": {
            "startup": float(startup_timeout_sec),
            "inactivity": float(idle_timeout_sec),
            "hard": float(hard_timeout_sec),
        },
        "events": events,
        "selection_events": selection_events,
        "ground_truth_score_events": ground_truth_score_events,
        "ground_truth_score_events_used_for_control": False,
    }


def main(argv=None) -> int:  # pragma: no cover - exercised in ROS integration
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--startup-timeout", type=float, default=120.0)
    parser.add_argument("--idle-timeout", type=float, default=180.0)
    parser.add_argument("--hard-timeout", type=float, default=900.0)
    options = parser.parse_args(argv)
    if options.output.exists():
        raise FileExistsError(options.output)

    try:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    class Recorder(Node):
        def __init__(self) -> None:
            super().__init__("generalized_development_probe_recorder")
            self.started_monotonic = time.monotonic()
            self.watchdog = ProbeWatchdog(
                self.started_monotonic,
                options.startup_timeout,
                options.idle_timeout,
                options.hard_timeout,
            )
            self.events: list[dict] = []
            self.selection_events: list[dict] = []
            self.ground_truth_score_events: list[dict] = []
            self.client = self.create_client(
                Trigger, "/strawberry/run_harvest"
            )
            self.create_subscription(
                String, "/strawberry/harvest_status", self.on_status, 10
            )
            self.create_subscription(
                String, "/strawberry/selection_status", self.on_selection, 10
            )
            self.create_subscription(
                String,
                "/strawberry/ground_truth/harvest_events",
                self.on_ground_truth_score_event,
                10,
            )
            self.timer = self.create_timer(0.1, self.tick)
            self.called = False
            self.done = False
            self.outcome = "RECORDER_STOPPED"

        def tick(self) -> None:
            if self.done:
                return
            timeout = self.watchdog.timeout_outcome(time.monotonic())
            if timeout is not None:
                self.finish(timeout)
                return
            if not self.called and self.client.service_is_ready():
                self.called = True
                future = self.client.call_async(Trigger.Request())
                future.add_done_callback(self.on_started)

        def on_started(self, future) -> None:
            try:
                response = future.result()
            except Exception:
                response = None
            if response is None or not response.success:
                self.finish("START_REJECTED")

        def on_status(self, message) -> None:
            try:
                event = json.loads(message.data)
            except (json.JSONDecodeError, TypeError):
                return
            if not isinstance(event, dict):
                return
            self.events.append(event)
            self.watchdog.observe(event, time.monotonic())
            if event.get("state") == TERMINAL_STATE:
                self.finish(str(event.get("outcome", "DONE")))

        def on_selection(self, message) -> None:
            try:
                event = json.loads(message.data)
            except (json.JSONDecodeError, TypeError):
                return
            if not isinstance(event, dict):
                return
            self.selection_events.append(
                {
                    "received_wall_offset_sec": (
                        time.monotonic() - self.started_monotonic
                    ),
                    **event,
                }
            )

        def on_ground_truth_score_event(self, message) -> None:
            try:
                event = json.loads(message.data)
            except (json.JSONDecodeError, TypeError):
                return
            if not isinstance(event, dict):
                return
            self.ground_truth_score_events.append(
                {
                    "received_wall_offset_sec": (
                        time.monotonic() - self.started_monotonic
                    ),
                    **event,
                }
            )

        def finish(self, outcome: str) -> None:
            if self.done:
                return
            self.done = True
            self.outcome = outcome
            payload = build_probe_payload(
                outcome=outcome,
                events=self.events,
                selection_events=self.selection_events,
                ground_truth_score_events=self.ground_truth_score_events,
                elapsed_sec=time.monotonic() - self.started_monotonic,
                startup_timeout_sec=options.startup_timeout,
                idle_timeout_sec=options.idle_timeout,
                hard_timeout_sec=options.hard_timeout,
            )
            temporary = options.output.with_suffix(options.output.suffix + ".tmp")
            temporary.write_text(
                json.dumps(payload, indent=2) + "\n", encoding="utf-8"
            )
            temporary.replace(options.output)

    rclpy.init()
    node = Recorder()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.2)
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return recorder_exit_code(node.outcome)


if __name__ == "__main__":
    raise SystemExit(main())
