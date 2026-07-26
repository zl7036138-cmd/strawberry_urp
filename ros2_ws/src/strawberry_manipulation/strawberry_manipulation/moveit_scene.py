"""Build and install tracked collision geometry in a MoveItPy scene."""

from __future__ import annotations

from .scene_geometry import (
    FRUIT_COLLISION_RADIUS_M,
    STATIC_COLLISION_OBJECTS,
    SpherePrimitive,
    fruit_collision_id,
)


def build_static_collision_messages(base_frame: str):
    """Create ROS collision messages without importing ROS at module load."""

    if not base_frame:
        raise ValueError("base frame must be non-empty")
    try:
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
    except ImportError as exc:  # pragma: no cover - ROS integration only
        raise RuntimeError(
            "static planning scene requires geometry_msgs, moveit_msgs, and shape_msgs"
        ) from exc

    messages = []
    for specification in STATIC_COLLISION_OBJECTS:
        collision_object = CollisionObject()
        collision_object.header.frame_id = base_frame
        collision_object.id = specification.object_id
        collision_object.operation = CollisionObject.ADD
        for box in specification.boxes:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(box.size_m)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = box.center_m
            pose.orientation.w = 1.0
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)
        messages.append(collision_object)
    return messages


def apply_static_collision_scene(planning_scene_monitor, base_frame: str) -> tuple[str, ...]:
    """Apply table and bin primitives under one planning-scene write lock."""

    messages = build_static_collision_messages(base_frame)
    with planning_scene_monitor.read_write() as scene:
        for collision_object in messages:
            scene.apply_collision_object(collision_object)
            if not scene.knows_frame_transform(collision_object.id):
                raise RuntimeError(
                    f"MoveIt did not retain collision object {collision_object.id}"
                )
        scene.current_state.update()
    return tuple(collision_object.id for collision_object in messages)


def build_fruit_collision_message(
    base_frame: str,
    target_id: int,
    sphere: SpherePrimitive,
):
    """Create one replace-by-ID spherical fruit collision object."""

    if not base_frame:
        raise ValueError("base frame must be non-empty")
    try:
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
    except ImportError as exc:  # pragma: no cover - ROS integration only
        raise RuntimeError(
            "fruit planning scene requires geometry_msgs, moveit_msgs, and shape_msgs"
        ) from exc

    collision_object = CollisionObject()
    collision_object.header.frame_id = base_frame
    collision_object.id = fruit_collision_id(target_id)
    collision_object.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [float(sphere.radius_m)]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = sphere.center_m
    pose.orientation.w = 1.0
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(pose)
    return collision_object


def build_fruit_removal_message(base_frame: str, target_id: int):
    """Create an explicit removal message for one target fruit."""

    if not base_frame:
        raise ValueError("base frame must be non-empty")
    try:
        from moveit_msgs.msg import CollisionObject
    except ImportError as exc:  # pragma: no cover - ROS integration only
        raise RuntimeError("fruit removal requires moveit_msgs") from exc
    collision_object = CollisionObject()
    collision_object.header.frame_id = base_frame
    collision_object.id = fruit_collision_id(target_id)
    collision_object.operation = CollisionObject.REMOVE
    return collision_object


def apply_fruit_collision_scene(
    planning_scene_monitor,
    base_frame: str,
    centers_by_target_id,
    radius_m: float = FRUIT_COLLISION_RADIUS_M,
) -> tuple[str, ...]:
    """Add every known fruit under one planning-scene write lock."""

    messages = [
        build_fruit_collision_message(
            base_frame,
            target_id,
            SpherePrimitive(tuple(center), radius_m),
        )
        for target_id, center in sorted(centers_by_target_id.items())
    ]
    with planning_scene_monitor.read_write() as scene:
        for collision_object in messages:
            scene.apply_collision_object(collision_object)
            if not scene.knows_frame_transform(collision_object.id):
                raise RuntimeError(
                    f"MoveIt did not retain collision object {collision_object.id}"
                )
        scene.current_state.update()
    return tuple(collision_object.id for collision_object in messages)


def set_target_fruit_collision(
    planning_scene_monitor,
    base_frame: str,
    target_id: int,
    *,
    center_m=None,
    radius_m: float = FRUIT_COLLISION_RADIUS_M,
) -> str:
    """Add/update a target sphere, or remove it when ``center_m`` is absent."""

    if center_m is None:
        collision_object = build_fruit_removal_message(base_frame, target_id)
    else:
        collision_object = build_fruit_collision_message(
            base_frame,
            target_id,
            SpherePrimitive(tuple(center_m), radius_m),
        )
    with planning_scene_monitor.read_write() as scene:
        scene.apply_collision_object(collision_object)
        known_after_apply = scene.knows_frame_transform(collision_object.id)
        if center_m is None and known_after_apply:
            raise RuntimeError(
                f"MoveIt did not remove collision object {collision_object.id}"
            )
        if center_m is not None and not known_after_apply:
            raise RuntimeError(
                f"MoveIt did not retain collision object {collision_object.id}"
            )
        scene.current_state.update()
    return collision_object.id
