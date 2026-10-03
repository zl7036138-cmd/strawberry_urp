"""Cleared-world load fixture only; not contact or harvesting evidence."""
import math
import xml.etree.ElementTree as ET


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
            [-sp, cp*sr, cp*cr, xyz[2]], [0, 0, 0, 1]]


def link_transform(robot, positions, target):
    joints = robot.findall("joint")
    children = {j.find("child").get("link") for j in joints}
    poses = {link.get("name"): transform() for link in robot.findall("link") if link.get("name") not in children}
    for _ in range(len(joints) + 1):
        if target in poses: return poses[target]
        for joint in joints:
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent not in poses or child in poses: continue
            origin = joint.find("origin")
            vec = lambda key: tuple(map(float, origin.get(key, "0 0 0").split())) if origin is not None else (0, 0, 0)
            offset = transform(vec("xyz"), vec("rpy"))
            q = positions.get(joint.get("name"), 0)
            axis = joint.find("axis")
            v = tuple(map(float, axis.get("xyz", "1 0 0").split())) if axis is not None else (1, 0, 0)
            if joint.get("type") in ("revolute", "continuous"):
                length = math.sqrt(sum(x*x for x in v))
                if length == 0: raise ValueError("Zero joint axis")
                v = tuple(x/length for x in v)
                x, y, z = v; c, s = math.cos(q), math.sin(q)
                skew = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
                motion = transform()
                for i in range(3):
                    for j in range(3): motion[i][j] = (c if i == j else 0) + (1-c)*v[i]*v[j] + s*skew[i][j]
                offset = multiply(offset, motion)
            elif joint.get("type") == "prismatic":
                offset = multiply(offset, transform(tuple(q*x for x in v)))
            elif joint.get("type") != "fixed": raise ValueError("Unsupported fixture joint")
            poses[child] = multiply(poses[parent], offset)
    raise ValueError("Missing or disconnected fixture link")


def add_payload_fixture(world, robot, positions, attachment_plugin):
    frame = multiply(link_transform(robot, positions, "panda_link7"), transform((0, 0, 0.16)))
    # SDF pose uses the same roll-pitch-yaw convention as URDF origins.
    pitch = math.atan2(-frame[2][0], math.hypot(frame[0][0], frame[1][0]))
    roll, yaw = math.atan2(frame[2][1], frame[2][2]), math.atan2(frame[1][0], frame[0][0])
    model = ET.SubElement(world, "model", name="diagnostic_payload")
    ET.SubElement(model, "pose").text = " ".join(map(str, [frame[i][3] for i in range(3)] + [roll, pitch, yaw]))
    link = ET.SubElement(model, "link", name="fruit_link")
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass").text = "0.030"
    inertia = ET.SubElement(inertial, "inertia")
    for key in ("ixx", "iyy", "izz"): ET.SubElement(inertia, key).text = "0.0000081"
    # No geometry: isolate topology/load from grasp and bin contact.
    anchor = ET.SubElement(world, "model", name="diagnostic_stem")
    ET.SubElement(anchor, "link", name="anchor")
    joint = ET.SubElement(anchor, "joint", name="world_fixed", type="fixed")
    ET.SubElement(joint, "parent").text = "world"
    ET.SubElement(joint, "child").text = "anchor"
    def plugin(parent, filename, name, values):
        item = ET.SubElement(parent, "plugin", filename=filename, name=name)
        for key, value in values.items(): ET.SubElement(item, key).text = value
    plugin(anchor, "gz-sim-detachable-joint-system", "gz::sim::systems::DetachableJoint",
           {"parent_link": "anchor", "child_model": "diagnostic_payload", "child_link": "fruit_link",
            "attach_topic": "/diagnostic/stem/attach", "detach_topic": "/diagnostic/stem/detach", "output_topic": "/diagnostic/stem/state"})
    plugin(world, str(attachment_plugin), "strawberry::LazyDetachableJoint",
           {"parent_model": "panda", "parent_link": "panda_link7", "child_model": "diagnostic_payload", "child_link": "fruit_link",
            "attach_topic": "/diagnostic/gripper/attach", "detach_topic": "/diagnostic/gripper/detach", "output_topic": "/diagnostic/gripper/state"})


def transfer_fixture(state, ecm_path):
    """Scenario assembly only. Never use this to claim a physical grasp."""
    import subprocess
    import time
    events = []
    with ecm_path.open() as stream:
        def await_count(expected):
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                position = stream.tell()
                line = stream.readline()
                if not line or not line.endswith("\n"):
                    stream.seek(position); time.sleep(0.02); continue
                import json
                row = json.loads(line)
                if row.get("phase") == "GRAPH":
                    events.append(row)
                    if row["multi_supported_child_count"]: raise RuntimeError("Fixture double support")
                    if row["detachable_joint_count"] == expected: return
            raise RuntimeError("Fixture topology confirmation timeout")
        def request(topic):
            subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", ""], check=True, timeout=5)
        await_count(1)
        request("/diagnostic/stem/detach"); await_count(0)
        request("/diagnostic/gripper/attach"); await_count(1)
        if state == "released":
            request("/diagnostic/gripper/detach"); await_count(0)
    return events


def motion_metrics(rows, events, state):
    if len(events) < 3 or [e["detachable_joint_count"] for e in events[:3]] != [1, 0, 1]:
        return {"physical_fixture_status": "INDETERMINATE"}
    start = events[2]["sim_time_sec"] + 0.1
    stop = events[3]["sim_time_sec"] if state == "released" and len(events) > 3 else math.inf
    frames = [r for r in rows if r.get("phase") == "FIXTURE_POSE" and start <= r["sim_time_sec"] < stop]
    if len(frames) < 3: return {"physical_fixture_status": "INDETERMINATE"}
    if any(not math.isfinite(r["sim_time_sec"]) for r in frames) or any(b["sim_time_sec"] <= a["sim_time_sec"] for a,b in zip(frames,frames[1:])):
        return {"physical_fixture_status": "INDETERMINATE"}
    keys = ("parent_xyz", "child_xyz", "relative_xyz", "relative_quat_wxyz")
    if any(len(r[k]) != (4 if "quat" in k else 3) or not all(math.isfinite(x) for x in r[k]) for r in frames for k in keys):
        return {"physical_fixture_status": "INDETERMINATE"}
    distance = lambda a, b: math.sqrt(sum((x-y)**2 for x,y in zip(a, b)))
    drift = max(distance(r["relative_xyz"], frames[0]["relative_xyz"]) for r in frames)
    travel = max(distance(r["parent_xyz"], frames[0]["parent_xyz"]) for r in frames)
    angles = []
    for row in frames:
        q, q0 = row["relative_quat_wxyz"], frames[0]["relative_quat_wxyz"]
        norm = math.sqrt(sum(x*x for x in q) * sum(x*x for x in q0))
        if norm <= 0: return {"physical_fixture_status": "INDETERMINATE"}
        angles.append(2 * math.acos(min(1.0, abs(sum(x*y for x,y in zip(q,q0)) / norm))))
    result = {"physical_fixture_status": "CARRY_OBSERVED" if travel >= 0.005 and drift <= 0.002 and max(angles) <= 0.02 else "INDETERMINATE",
              "parent_translation_span_m": travel, "relative_translation_drift_m": drift,
              "relative_rotation_drift_rad": max(angles), "contact_grasp_proven": False}
    if state == "released" and len(events) > 3:
        after = [r for r in rows if r.get("phase") == "FIXTURE_POSE" and r["sim_time_sec"] >= stop + 0.1]
        valid = len(after) >= 3 and all(len(r["child_xyz"]) == 3 and all(math.isfinite(x) for x in r["child_xyz"]) for r in after)
        valid = valid and all(b["sim_time_sec"] > a["sim_time_sec"] for a,b in zip(after,after[1:]))
        result["release_fall_observed"] = valid and after[0]["child_xyz"][2] - after[-1]["child_xyz"][2] >= 0.05
        # The arm is deliberately held still during this scenario initialization.
        result["physical_fixture_status"] = "RELEASE_FALL_OBSERVED" if result["release_fall_observed"] else "INDETERMINATE"
    return result
