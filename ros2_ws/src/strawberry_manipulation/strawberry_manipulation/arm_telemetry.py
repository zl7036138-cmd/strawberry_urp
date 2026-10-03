"""Passive named controller samples; no control or scoring-truth dependencies."""
from __future__ import annotations

import json
import math
import time
import hashlib
from pathlib import Path
from collections import Counter, deque

from .motion_evidence import _numbers, stamp_ns, serialize_joint_feedback


class StopEvidence:
    """Independent, finite-window joint-state evidence; not an operation gate.

    Source ROS time and receiver monotonic time are never subtracted from each
    other. Stop evidence expires; historical quiet windows are not current stop.
    Coverage errors remain counted even when a later clean window is observed.
    """
    def __init__(self, joint_names, *, home_positions=None,
                 duration_sec=0.5, velocity_limit=0.02, position_span=0.002,
                 max_source_gap_sec=0.2, max_receipt_gap_sec=1.0,
                 home_tolerance=0.05, minimum_samples=3):
        self.names = tuple(joint_names)
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError("joint names must be distinct and nonempty")
        self.home = tuple(home_positions) if home_positions is not None else None
        values = (duration_sec, velocity_limit, position_span, max_source_gap_sec,
                  max_receipt_gap_sec, home_tolerance)
        if any(not math.isfinite(x) or x <= 0 for x in values):
            raise ValueError("stop thresholds must be finite and positive")
        if self.home is not None and (len(self.home) != len(self.names)
                                     or not all(math.isfinite(x) for x in self.home)):
            raise ValueError("home joint vector must match the joint names")
        if minimum_samples < 2:
            raise ValueError("stop confirmation requires multiple samples")
        self.duration_ns = round(duration_sec * 1e9)
        self.source_gap_ns = round(max_source_gap_sec * 1e9)
        self.receipt_gap_ns = round(max_receipt_gap_sec * 1e9)
        self.velocity_limit = velocity_limit
        self.position_span = position_span
        self.home_tolerance = home_tolerance
        self.minimum_samples = minimum_samples
        self.window = deque()
        self.last_source = None
        self.last_receipt = None
        self.clock_reset = False
        self.issues = Counter()
        self.last = self._empty("NO_SAMPLES")

    def _empty(self, reason):
        return {"stop_status": "INDETERMINATE", "home_status": "INDETERMINATE",
                "reason": reason, "source_duration_ns": 0, "receipt_duration_ns": 0,
                "sample_ids": [], "samples": [], "thresholds": {
                    "duration_ns": self.duration_ns, "velocity_limit_rad_s": self.velocity_limit,
                    "position_span_rad": self.position_span,
                    "max_source_gap_ns": self.source_gap_ns,
                    "max_receipt_gap_ns": self.receipt_gap_ns,
                    "home_tolerance_rad": self.home_tolerance,
                    "minimum_samples": self.minimum_samples},
                "coverage_issues": dict(self.issues), "full_interval_safety_claim": False}

    def _reject(self, reason):
        self.issues[reason] += 1
        self.window.clear()
        self.last = self._empty(reason)
        return self.last

    def observe(self, sample, *, sample_id):
        source = sample.get("acquisition_stamp_ns")
        receipt = sample.get("received_monotonic_ns")
        ros_now = sample.get("received_ros_ns")
        if any(not isinstance(x, int) or isinstance(x, bool) or x < 0
               for x in (source, receipt, ros_now)):
            return self._reject("INVALID_TIME")
        if self.clock_reset:
            return self._reject("CLOCK_RESET_REQUIRES_NEW_EPOCH")
        if self.last_source is not None and source < self.last_source:
            self.clock_reset = True
            return self._reject("ACQUISITION_CLOCK_REGRESSION")
        if self.last_source is not None and source == self.last_source:
            return self._reject("DUPLICATE_ACQUISITION_STAMP")
        if self.last_receipt is not None and receipt <= self.last_receipt:
            return self._reject("RECEIPT_CLOCK_REGRESSION")
        previous_source, previous_receipt = self.last_source, self.last_receipt
        self.last_source, self.last_receipt = source, receipt
        if source > ros_now or ros_now - source > self.source_gap_ns:
            return self._reject("STALE_OR_FUTURE_ACQUISITION")
        names = sample.get("joint_names", [])
        p, v = sample.get("positions", []), sample.get("velocities", [])
        if (len(names) != len(set(names)) or not set(self.names).issubset(names)
                or len(p) != len(names) or len(v) != len(names)
                or any(issue.startswith(("positions", "velocities"))
                       for issue in sample.get("invalid_fields", ()))):
            return self._reject("INCOMPLETE_OR_INVALID_JOINT_SAMPLE")
        indexes = [names.index(name) for name in self.names]
        positions = [p[i] for i in indexes]
        velocities = [v[i] for i in indexes]
        if any(not isinstance(x, (float, int)) or not math.isfinite(x)
               for x in positions + velocities):
            return self._reject("NONFINITE_JOINT_SAMPLE")
        if previous_source is not None and (
                source - previous_source > self.source_gap_ns
                or receipt - previous_receipt > self.receipt_gap_ns):
            self._reject("SAMPLE_GAP")
        if max(abs(x) for x in velocities) > self.velocity_limit:
            return self._reject("VELOCITY_EXCEEDS_STOP_BOUND")
        if len(self.window) >= 4096:
            self._reject("STOP_WINDOW_BUFFER_LIMIT")
        self.window.append({"sample_id": sample_id, "acquisition_stamp_ns": source,
                            "received_monotonic_ns": receipt,
                            "positions_rad": positions, "velocities_rad_s": velocities})
        # Keep the shortest suffix that spans the complete source AND receipt interval.
        while len(self.window) > self.minimum_samples and (
                source - self.window[1]["acquisition_stamp_ns"] >= self.duration_ns
                and receipt - self.window[1]["received_monotonic_ns"] >= self.duration_ns):
            self.window.popleft()
        spans = [max(s["positions_rad"][i] for s in self.window)
                 - min(s["positions_rad"][i] for s in self.window)
                 for i in range(len(self.names))]
        if max(spans) > self.position_span:
            newest = self.window[-1]
            self._reject("POSITION_DRIFT")
            self.window.append(newest)
            return self.last
        result = self._empty("OBSERVING_STABLE_WINDOW")
        result.update(source_duration_ns=source - self.window[0]["acquisition_stamp_ns"],
                      receipt_duration_ns=receipt - self.window[0]["received_monotonic_ns"],
                      sample_ids=[s["sample_id"] for s in self.window],
                      samples=list(self.window), joint_names=list(self.names),
                      position_span_rad=spans,
                      maximum_abs_velocity_rad_s=max(abs(x) for s in self.window
                                                    for x in s["velocities_rad_s"]))
        if (len(self.window) >= self.minimum_samples
                and result["source_duration_ns"] >= self.duration_ns
                and result["receipt_duration_ns"] >= self.duration_ns):
            result.update(stop_status="PASS", reason="INDEPENDENT_JOINT_WINDOW_OBSERVED")
            if self.home is not None:
                errors = [max(abs(s["positions_rad"][i] - target) for s in self.window)
                          for i, target in enumerate(self.home)]
                result["home_errors_rad"] = errors
                result["home_status"] = "PASS" if max(errors) <= self.home_tolerance else "FAIL"
        self.last = result
        return result

    def current(self, *, now_monotonic_ns, now_ros_ns):
        if (self.last_receipt is None or self.last_source is None
                or now_ros_ns is None or now_monotonic_ns < self.last_receipt
                or now_ros_ns < self.last_source
                or now_monotonic_ns - self.last_receipt > self.receipt_gap_ns
                or now_ros_ns - self.last_source > self.source_gap_ns):
            return self._empty("FINAL_SAMPLE_NOT_FRESH")
        return self.last


def _json_data_is_finite(value):
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_json_data_is_finite(v) for v in value.values())
    if isinstance(value, (list, tuple)):
        return all(_json_data_is_finite(item) for item in value)
    return True

def serialize_controller_state(message):
    """Keep Jazzy reference/feedback and legacy desired/actual explicitly named."""
    issues = []
    header = getattr(message, "header", None)
    names = list(getattr(message, "joint_names", ()))
    stamp = stamp_ns(getattr(header, "stamp", None))
    if stamp is None:
        issues.append("MISSING_ACQUISITION_STAMP")
    if not names or len(names) != len(set(names)):
        issues.append("INVALID_JOINT_NAMES")
    desired = "reference" if hasattr(message, "reference") else "desired"
    actual = "feedback" if hasattr(message, "feedback") else "actual"
    raw = {}
    for field in (desired, actual, "error", "output"):
        point = getattr(message, field, None)
        if point is None:
            raw[field] = None
            if field != "output":
                issues.append(f"MISSING_{field}")
            continue
        duration = getattr(point, "time_from_start", None)
        raw[field] = {
            key: _numbers(getattr(point, key, ()), f"{field}.{key}", issues)
            for key in ("positions", "velocities", "accelerations", "effort")
        }
        raw[field]["time_from_start"] = {
            "sec": getattr(duration, "sec", None),
            "nanosec": getattr(duration, "nanosec", None),
        }
        if field != "output" and len(raw[field]["positions"]) != len(names):
            issues.append(f"INCOMPLETE_{field}_POSITIONS")
    aligned = {}
    if not issues:
        for i, name in enumerate(names):
            reference = raw[desired]["positions"][i]
            feedback = raw[actual]["positions"][i]
            aligned[name] = {
                "desired_position_rad": reference,
                "actual_position_rad": feedback,
                "reported_error_rad": raw["error"]["positions"][i],
                "computed_error_rad": reference - feedback,
            }
    return {"stamp_ns": stamp, "frame_id": getattr(header, "frame_id", None),
            "joint_names": names, "raw": raw, "aligned": aligned,
            "field_roles": {"desired": desired, "actual": actual}, "issues": issues}


class TelemetryRecorder:
    """Append-only passive recorder; current stop is distinct from coverage."""
    def __init__(self, output, *, run_id, scenario_id, home_positions=None, **stop_options):
        if not run_id or not scenario_id:
            raise ValueError("run_id and scenario_id are required")
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.stream = (self.output / "samples.jsonl").open("x", encoding="utf-8", newline="\n")
        self.run_id, self.scenario_id = run_id, scenario_id
        self.seq = 0
        self.counts = Counter()
        self.peak = {}
        self.controller_stamps = None
        self.command_sequences = {}
        self.prepared_commands = {}
        self.closed = False
        self.stop = StopEvidence(tuple(f"panda_joint{i}" for i in range(1, 8)),
                                 home_positions=home_positions, **stop_options)
        self._record("RECORDER_START", {"topics": ["/joint_states",
            "/panda_arm_controller/controller_state", "/clock", "/strawberry/motion_evidence"],
            "acquisition_clock": "ROS_TIME", "receipt_clock": "MONOTONIC",
            "action_clients": [], "control_publishers": [],
            "independence": "Independent joint-state subscriber; no controller success boolean is used for stop.",
            "stop_thresholds": self.stop.last["thresholds"]},
            time.monotonic_ns(), None)

    def _record(self, event_type, payload, received_ns, ros_now_ns, issues=()):
        if self.closed:
            raise RuntimeError("recorder is closed")
        self.seq += 1
        row = {"schema_version": 1, "run_id": self.run_id, "scenario_id": self.scenario_id,
               "event_id": f"{self.run_id}/recorder/{self.seq}",
               "producer_id": "passive-arm-telemetry", "producer_seq": self.seq,
               "event_type": event_type, "received_time_monotonic_ns": received_ns,
               "recorder_ros_time_ns": ros_now_ns,
               "event_time_monotonic_ns": None,
               "event_time_monotonic_status": "NOT_PROVIDED_BY_ROS_SENSOR",
               "payload": payload, "issues": list(issues)}
        try:
            line = json.dumps(row, allow_nan=False, separators=(",", ":"))
        except (ValueError, TypeError):
            # Preserve the malformed payload as raw bytes; never drop the record.
            fallback = dict(row)
            fallback["payload"] = {"raw_repr": repr(payload)}
            fallback["issues"] = list(issues) + ["PAYLOAD_SERIALIZATION_FAILED"]
            line = json.dumps(fallback, allow_nan=False, separators=(",", ":"))
        self.stream.write(line + "\n")
        self.stream.flush()
        return row

    def controller(self, message, *, received_ns, ros_now_ns):
        data = serialize_controller_state(message)
        stamp = data["stamp_ns"]
        issues = list(data["issues"])
        if stamp is not None and self.controller_stamps is not None:
            if stamp <= self.controller_stamps:
                issues.append("CONTROLLER_STAMP_DUPLICATE_OR_REGRESSING")
            elif stamp - self.controller_stamps > self.stop.source_gap_ns:
                issues.append("CONTROLLER_SAMPLE_GAP")
        if stamp is not None:
            self.controller_stamps = stamp
        self.counts["controller_sample_count"] += 1
        row = self._record("CONTROLLER_STATE", data, received_ns, ros_now_ns, issues)
        for issue in issues:
            self.counts["controller_issue_" + issue] += 1
        if not issues:
            for name, state in data["aligned"].items():
                error = abs(state["computed_error_rad"])
                old = self.peak.get(name)
                if old is None or error > old["absolute_error_rad"]:
                    self.peak[name] = {"absolute_error_rad": error,
                        "sample_id": row["event_id"], "acquisition_stamp_ns": stamp,
                        "desired_rad": state["desired_position_rad"],
                        "actual_rad": state["actual_position_rad"],
                        "reported_error_rad": state["reported_error_rad"]}

    def joints(self, message, *, received_ns, ros_now_ns):
        data = serialize_joint_feedback(message, receipt_monotonic_ns=received_ns,
                                        received_ros_ns=ros_now_ns)
        sample_id = f"{self.run_id}/recorder/{self.seq + 1}"
        result = self.stop.observe(data, sample_id=sample_id)
        self.counts["joint_sample_count"] += 1
        issues = list(data["invalid_fields"])
        if result["reason"] not in {"OBSERVING_STABLE_WINDOW", "INDEPENDENT_JOINT_WINDOW_OBSERVED"}:
            issues.append(result["reason"])
        self._record("JOINT_STATE", data, received_ns, ros_now_ns, issues)

    def clock(self, message, *, received_ns, ros_now_ns):
        stamp = stamp_ns(getattr(message, "clock", None))
        self.counts["clock_sample_count"] += 1
        self._record("CLOCK", {"stamp_ns": stamp}, received_ns, ros_now_ns,
                     ["INVALID_CLOCK"] if stamp is None else [])

    def command(self, text, *, received_ns, ros_now_ns):
        issues = []
        try:
            data = json.loads(text, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
            if not isinstance(data, dict):
                raise ValueError("command event must be an object")
            if not _json_data_is_finite(data):
                raise ValueError("command event contains non-finite values")
        except (TypeError, ValueError, OverflowError):
            data = None
            issues.append("MALFORMED_COMMAND_EVENT")
        if data is not None:
            if data.get("schema_version") != 1:
                issues.append("COMMAND_SCHEMA_UNSUPPORTED")
            if data.get("run_id") != self.run_id or data.get("scenario_id") != self.scenario_id:
                issues.append("COMMAND_IDENTITY_UNBOUND_OR_MISMATCH")
            producer, seq = data.get("producer_id"), data.get("producer_seq")
            producer_valid = isinstance(producer, str) and bool(producer.strip())
            sequence_valid = isinstance(seq, int) and not isinstance(seq, bool) and seq > 0
            previous = None
            if not producer_valid or not sequence_valid:
                issues.append("COMMAND_PRODUCER_IDENTITY_MISSING")
            else:
                previous = self.command_sequences.get(producer)
                if previous is None and seq != 1:
                    issues.append("COMMAND_SEQUENCE_DISCONTINUITY")
                elif previous is not None and seq != previous + 1:
                    issues.append("COMMAND_SEQUENCE_DISCONTINUITY")
            event_type = data.get("event_type")
            command_id = data.get("command_id")
            command_valid = isinstance(command_id, str) and bool(command_id.strip())
            command_namespace_valid = (
                command_valid
                and producer_valid
                and command_id.startswith(f"{producer}/command/")
                and len(command_id) > len(f"{producer}/command/")
            )
            if event_type == "COMMAND_PREPARED":
                if not command_valid:
                    issues.append("COMMAND_PREPARATION_IDENTITY_MISSING")
                else:
                    if not command_namespace_valid:
                        issues.append("COMMAND_ID_PRODUCER_NAMESPACE_MISMATCH")
                    if command_id in self.prepared_commands:
                        issues.append("COMMAND_ID_ALREADY_PREPARED")
                if not issues:
                    self.prepared_commands[command_id] = {
                        "seq": seq,
                        "producer": producer,
                    }
            elif event_type in ("ACTION_ACCEPTED", "ACTION_TERMINAL", "ACTION_REJECTED",
                                "ACTION_ACCEPTANCE_UNKNOWN", "ACTION_RESULT_TIMEOUT",
                                "ACTION_LIVE_JOINT_LIMIT_ABORT", "ACTION_RESULT_FUTURE_ERROR",
                                "CANCEL_RESPONSE"):
                if not command_valid:
                    issues.append("ACTION_EVENT_COMMAND_IDENTITY_MISSING")
                else:
                    preparation = self.prepared_commands.get(command_id)
                    if preparation is None:
                        issues.append("COMMAND_PREPARATION_MISSING")
                    elif preparation["producer"] != producer:
                        issues.append("COMMAND_PRODUCER_MISMATCH")
                    elif not sequence_valid or seq <= preparation["seq"]:
                        issues.append("COMMAND_CAUSAL_ORDER_INVALID")
            if not issues and producer_valid and sequence_valid:
                self.command_sequences[producer] = seq
        self.counts["command_event_count"] += 1
        if issues:
            self.counts["invalid_command_count"] += 1
        self._record("BACKEND_COMMAND_EVENT", {"raw_text": text, "event": data},
                     received_ns, ros_now_ns, issues)

    def close(self, *, now_ns, ros_now_ns):
        final_stop = self.stop.current(now_monotonic_ns=now_ns, now_ros_ns=ros_now_ns)
        self._record("RECORDER_END", {"independent_stop": final_stop}, now_ns, ros_now_ns)
        self.stream.close()
        self.closed = True
        raw = self.output / "samples.jsonl"
        digest = hashlib.sha256()
        with raw.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        summary = {"schema_version": 1, "scope": "PASSIVE_ARM_DIAGNOSTIC",
                   "run_id": self.run_id, "scenario_id": self.scenario_id,
                   "event_count": self.seq, **self.counts,
                   "invalid_command_count": self.counts["invalid_command_count"],
                   "tracking_peak_errors_rad": self.peak,
                   "independent_stop": final_stop,
                   "full_interval_safety_claim": False, "formal_acceptance": False,
                   "operation_ownership_proved": False,
                   "samples_sha256": digest.hexdigest(), "samples_file": "samples.jsonl"}
        with (self.output / "summary.json").open("x", encoding="utf-8") as stream:
            json.dump(summary, stream, allow_nan=False, indent=2)
        return summary
