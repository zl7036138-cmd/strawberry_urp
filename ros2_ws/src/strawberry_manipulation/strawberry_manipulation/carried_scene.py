"""MoveIt attached-body lifecycle for the perception-selected carried fruit."""
from __future__ import annotations
import math
from .scene_geometry import SpherePrimitive


def carried_id(track_id):
    if not isinstance(track_id, int) or isinstance(track_id, bool) or track_id <= 0:
        raise ValueError("positive tracked identity required")
    return f"strawberry_carried_fruit_{track_id}"


def set_carried_fruit(monitor, link_name, track_id, *, center_m=None, radius_m=.026):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from shape_msgs.msg import SolidPrimitive
    object_id = carried_id(track_id)
    if not link_name:
        raise ValueError("attachment link required")
    attached = AttachedCollisionObject()
    attached.link_name = link_name
    attached.object.id = object_id
    attached.object.header.frame_id = link_name
    attached.object.operation = CollisionObject.REMOVE if center_m is None else CollisionObject.ADD
    if center_m is not None:
        if (len(center_m) != 3 or not all(math.isfinite(v) for v in center_m)
                or not math.isfinite(radius_m) or radius_m <= 0):
            raise ValueError("finite carried sphere required")
        sphere = SpherePrimitive(tuple(center_m), radius_m)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [sphere.radius_m]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = sphere.center_m
        pose.orientation.w = 1.
        attached.object.primitives = [primitive]
        attached.object.primitive_poses = [pose]
        # Legitimate grasp contact only. No camera/arm/environment exemptions.
        attached.touch_links = [link_name, "panda_leftfinger", "panda_rightfinger"]
    with monitor.read_write() as scene:
        result = scene.process_attached_collision_object(attached)
        if result is False:
            raise RuntimeError("MoveIt refused carried-fruit attachment update")
        if center_m is None:
            # MoveIt detach can leave a world object; release removes this
            # planning-only proxy, never another tracked obstacle.
            removal = CollisionObject()
            removal.id = object_id
            removal.header.frame_id = link_name
            removal.operation = CollisionObject.REMOVE
            scene.apply_collision_object(removal)
        scene.current_state.update()
        if bool(scene.knows_frame_transform(object_id)) != (center_m is not None):
            raise RuntimeError("MoveIt carried-fruit lifecycle was not retained")
    return object_id
