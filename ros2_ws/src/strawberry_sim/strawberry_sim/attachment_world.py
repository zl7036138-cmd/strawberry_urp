"""Development lazy-backend fixture; no scene coordinates or control truth."""
import xml.etree.ElementTree as ET


def add_external_stem_support(root, fruit_count):
    if type(fruit_count) is not int or not 1 <= fruit_count <= 9:
        raise ValueError("Invalid attachment capacity")
    world = root.find("world")
    if world is None or world.find("model[@name='strawberry_stem_support']") is not None:
        raise ValueError("World missing or external stem support already present")
    model = ET.SubElement(world, "model", name="strawberry_stem_support")
    # A movable skeleton with a fixed world root, like the Panda base, but
    # independent of the arm. No geometry, pose reset, or whole-arm self-collision.
    ET.SubElement(model, "static").text = "false"
    link = ET.SubElement(model, "link", name="stem_support_link")
    ET.SubElement(link, "gravity").text = "false"
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass").text = "1.0"
    inertia = ET.SubElement(inertial, "inertia")
    for field in ("ixx", "iyy", "izz"):
        ET.SubElement(inertia, field).text = "0.01"
    joint = ET.SubElement(model, "joint", name="stem_support_to_world", type="fixed")
    ET.SubElement(joint, "parent").text = "world"
    ET.SubElement(joint, "child").text = "stem_support_link"
    for target_id in range(1, fruit_count + 1):
        plugin = ET.SubElement(model, "plugin", filename="gz-sim-detachable-joint-system",
                               name="gz::sim::systems::DetachableJoint")
        for tag, value in {
            "parent_link": "stem_support_link", "child_model": f"strawberry_{target_id}",
            "child_link": "fruit_link", "attach_topic": f"/strawberry/sim/fruit_{target_id}/stem_attach",
            "detach_topic": f"/strawberry/sim/fruit_{target_id}/stem_detach",
            "output_topic": f"/strawberry/sim/fruit_{target_id}/stem_attached",
        }.items():
            ET.SubElement(plugin, tag).text = value
