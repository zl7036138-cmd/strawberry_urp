"""Vision-only carried-fruit transforms and conservative transit clearance."""
from __future__ import annotations
import math
from .core import Pose


def rotate(point, pose: Pose, *, inverse=False):
    values = (*point, pose.qx, pose.qy, pose.qz, pose.qw)
    if len(point) != 3 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError("finite point and quaternion required")
    q = pose.normalized()
    x, y, z = q.qx, q.qy, q.qz
    if inverse:
        x, y, z = -x, -y, -z
    w = q.qw
    a, b, c = point
    return ((1-2*(y*y+z*z))*a + 2*(x*y-z*w)*b + 2*(x*z+y*w)*c,
            2*(x*y+z*w)*a + (1-2*(x*x+z*z))*b + 2*(y*z-x*w)*c,
            2*(x*z-y*w)*a + 2*(y*z+x*w)*b + (1-2*(x*x+y*y))*c)


def local_fruit_center(hand: Pose, visual_center):
    if len(visual_center) != 3 or not all(math.isfinite(v) for v in visual_center):
        raise ValueError("finite visual xyz triple required")
    if not all(math.isfinite(v) for v in (hand.x, hand.y, hand.z)):
        raise ValueError("finite hand position required")
    return rotate(tuple(a-b for a, b in zip(visual_center, (hand.x, hand.y, hand.z))), hand, inverse=True)


def transit_hand_height(nominal_z, target_orientation, local_center, scene_centers, radius, uncertainty):
    """Keep both fruit envelopes apart before the bin-only descent.

    All centres come from perception, including unripe fruit. This does not
    certify unobserved space or replace MoveIt attached-body collision checks.
    The selected fruit's original centre may remain in this conservative set.
    """
    values = (nominal_z, radius, uncertainty)
    if not all(math.isfinite(v) for v in values) or radius <= 0 or uncertainty < 0:
        raise ValueError("finite height, positive radius and nonnegative uncertainty required")
    centers = tuple(tuple(center) for center in scene_centers)
    if not centers or any(len(c) != 3 or not all(math.isfinite(v) for v in c) for c in centers):
        raise ValueError("nonempty finite perceived scene required")
    offset_z = rotate(local_center, target_orientation)[2]
    return max(nominal_z, max(c[2] for c in centers) + 2*(radius+uncertainty) - offset_z)
