"""Offline contact-window summaries; never a collision-free/safety certificate."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


def summarize(rows, event_time, before=0.5, after=0.1):
    if not all(math.isfinite(v) for v in (event_time, before, after)) or min(before, after) < 0:
        raise ValueError("finite event time and nonnegative window required")
    pairs = {}
    coverage = []
    last = None
    for row in rows:
        phase = row.get("phase")
        if phase not in ("CONTACTS", "CONTACT_COVERAGE"):
            continue
        stamp = float(row["sim_time_sec"])
        if not math.isfinite(stamp) or (last is not None and stamp < last):
            raise ValueError("contact timestamps must be finite and nondecreasing")
        last = stamp
        if not event_time-before <= stamp <= event_time+after:
            continue
        if phase == "CONTACT_COVERAGE":
            count = row["source_count"]
            if not isinstance(count, int) or isinstance(count, bool) or count < 0:
                raise ValueError("invalid contact source count")
            coverage.append((stamp, count))
            continue
        for contact in row["contacts"].get("contact", []):
            identities = []
            for key in ("collision1", "collision2"):
                entity = contact[key]
                identities.append((str(entity.get("id", "")), str(entity.get("name", ""))))
            key = tuple(sorted(identities))
            depths = [float(value) for value in contact.get("depth", [])]
            if not all(math.isfinite(value) for value in depths):
                raise ValueError("nonfinite contact depth")
            item = pairs.setdefault(key, {"collisions": [dict(id=i, name=n) for i, n in key],
                "first_sim_sec": stamp, "last_sim_sec": stamp, "sample_count": 0,
                "max_depth_m": None, "source_entities": []})
            item["last_sim_sec"] = stamp
            item["sample_count"] += 1
            source = row["source_entity"]
            if source not in item["source_entities"]:
                item["source_entities"].append(source)
            if depths:
                item["max_depth_m"] = max(depths + ([] if item["max_depth_m"] is None else [item["max_depth_m"]]))
    return {"event_sim_sec": event_time, "window_before_sec": before, "window_after_sec": after,
        "coverage_sample_count": len(coverage),
        "minimum_observed_source_count": min((count for _, count in coverage), default=None),
        "maximum_coverage_gap_sec": max((b[0]-a[0] for a, b in zip(coverage, coverage[1:])), default=None),
        "pairs": list(pairs.values()), "collision_free_proven": False,
        "coverage_scope": "EXISTING_CONTACT_SENSOR_DATA_ONLY", "causal_attribution": "UNDETERMINED"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ecm", type=Path, required=True)
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("refusing to overwrite output")
    events = []
    with args.samples.open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("event_type") == "BACKEND_COMMAND_EVENT":
                event = row["payload"]["event"]
                if event["event_type"] == "TRAJECTORY_RECOVERY_REQUIRED":
                    events.append(event)
    windows = []
    for event in events:
        with args.ecm.open(encoding="utf-8") as stream:
            window = summarize((json.loads(line) for line in stream), event["ros_time_ns"]/1e9)
        window["command_id"] = event["command_id"]
        windows.append(window)
    result = {"schema_version": 1, "scope": "OFFLINE_CONTACT_DIAGNOSTIC", "formal_acceptance": False,
        "input_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in (args.ecm, args.samples)}, "recovery_windows": windows}
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
