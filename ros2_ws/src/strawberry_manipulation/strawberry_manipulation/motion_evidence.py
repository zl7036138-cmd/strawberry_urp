"""Diagnostic command evidence with explicit source and clock boundaries.

This module does not grant operation ownership or physical-stop safety. A
command_id identifies a send attempt only, never a harvest operation/attempt.
"""
from __future__ import annotations

import math
import threading
import time
import uuid
from typing import Callable


def stamp_ns(stamp) -> int | None:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nsec = getattr(stamp, "nanosec", None)
    if (not isinstance(sec, int) or isinstance(sec, bool)
            or not isinstance(nsec, int) or isinstance(nsec, bool)
            or sec < 0 or not 0 <= nsec < 1_000_000_000):
        return None
    return sec * 1_000_000_000 + nsec


def _numbers(values, name, invalid):
    result = []
    for index, value in enumerate(values):
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError):
            number = math.nan
        if not math.isfinite(number):
            invalid.append(f"{name}[{index}]:NONFINITE")
            result.append(None)
        else:
            result.append(number)
    return result


def serialize_joint_feedback(message, *, receipt_monotonic_ns, received_ros_ns):
    """Retain source stamp, frame, all values and invalidity (never fill gaps)."""
    header = getattr(message, "header", None)
    acquisition = stamp_ns(getattr(header, "stamp", None))
    invalid = []
    if acquisition is None:
        invalid.append("acquisition_stamp_ns:MISSING")
    names = [str(name) for name in getattr(message, "name", ())]
    result = {
        "source_topic": "/joint_states",
        "acquisition_clock": "ROS_TIME",
        "acquisition_stamp_ns": acquisition,
        "received_monotonic_ns": receipt_monotonic_ns,
        "received_ros_ns": received_ros_ns,
        "frame_id": getattr(header, "frame_id", None),
        "joint_names": names,
        "positions": _numbers(getattr(message, "position", ()), "positions", invalid),
        "velocities": _numbers(getattr(message, "velocity", ()), "velocities", invalid),
        "effort": _numbers(getattr(message, "effort", ()), "effort", invalid),
        "invalid_fields": invalid,
    }
    if len(names) != len(set(names)):
        invalid.append("joint_names:DUPLICATE")
    for field in ("positions", "velocities"):
        if len(result[field]) != len(names):
            invalid.append(f"{field}:INCOMPLETE")
    return result


def serialize_trajectory(trajectory):
    invalid = []
    points = []
    for index, point in enumerate(trajectory.points):
        data = {key: _numbers(getattr(point, key, ()), f"points[{index}].{key}", invalid)
                for key in ("positions", "velocities", "accelerations", "effort")}
        data["time_from_start_ns"] = stamp_ns(point.time_from_start)
        if data["time_from_start_ns"] is None:
            invalid.append(f"points[{index}].time_from_start_ns:INVALID")
        points.append(data)
    header = getattr(trajectory, "header", None)
    return {"joint_names": list(trajectory.joint_names),
            "header_stamp_ns": stamp_ns(getattr(header, "stamp", None)),
            "frame_id": getattr(header, "frame_id", None),
            "points": points, "invalid_fields": invalid}


class MotionEvidence:
    """Serialize concurrent emissions; transport failures do not alter motion."""
    def __init__(self, sink: Callable, *, run_id=None, scenario_id=None,
                 producer_id=None, clock_ns=time.monotonic_ns, ros_now_ns=lambda: None):
        if not isinstance(run_id, str) or not run_id.strip():
            raise ValueError("motion evidence run_id is required")
        if not isinstance(scenario_id, str) or not scenario_id.strip():
            raise ValueError("motion evidence scenario_id is required")
        self.sink = sink
        self.run_id = run_id.strip()
        self.scenario_id = scenario_id.strip()
        self.producer_id = producer_id or f"motion-backend/{uuid.uuid4().hex}"
        self.clock_ns = clock_ns
        self.ros_now_ns = ros_now_ns
        self.sequence = 0
        self._lock = threading.Lock()

    def emit(self, event_type, payload, *, command_id=None):
        with self._lock:
            self.sequence += 1
            event = {
                "schema_version": 1,
                "scope": "MOTION_DIAGNOSTIC_NOT_OPERATION_OWNERSHIP",
                "run_id": self.run_id, "scenario_id": self.scenario_id,
                "producer_id": self.producer_id, "producer_seq": self.sequence,
                "event_id": f"{self.producer_id}/{self.sequence}",
                "event_type": event_type,
                "event_time_monotonic_ns": self.clock_ns(),
                "ros_time_ns": self.ros_now_ns(),
                "command_id": command_id,
                "operation_id": None, "attempt_id": None, "motion_epoch_id": None,
                "physical_stop_claim": False,
                "payload": payload,
            }
            self.sink(event)
            return event

    def start_command(self, route, payload):
        command_id = f"{self.producer_id}/command/{uuid.uuid4().hex}"
        self.emit("COMMAND_PREPARED", {"route": route, **payload}, command_id=command_id)
        return command_id
