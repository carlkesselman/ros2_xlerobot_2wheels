"""Frames and the head aiming solver; plain numpy, no ROS, so it can be tested on its own."""
import math
import xml.etree.ElementTree as ET

import numpy as np


def quat_to_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def transform_to_matrix(tf):
    """geometry_msgs/TransformStamped -> 4x4."""
    t, q = tf.transform.translation, tf.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def pose_to_matrix(pose):
    """geometry_msgs/Pose -> 4x4."""
    p, q = pose.position, pose.orientation
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def matrix_to_quat(R):
    """(x, y, z, w) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    return x, y, z, w


def yaw_of_matrix(R):
    return math.atan2(R[1, 0], R[0, 0])


def yaw_quat(yaw):
    """(x, y, z, w) of a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def joint_info(urdf_xml, names):
    """{joint: {"axis": unit vector in the child frame, "child": link, "lower": rad, "upper": rad}} from a URDF string."""
    root = ET.fromstring(urdf_xml)
    out = {}
    for j in root.findall("joint"):                  # top level only: ros2_control blocks repeat joint names
        name = j.get("name")
        if name not in names or j.find("child") is None:
            continue
        axis = j.find("axis")
        a = np.array([float(v) for v in axis.get("xyz").split()]) if axis is not None else np.array([1.0, 0, 0])
        lim = j.find("limit")
        lower = float(lim.get("lower", -math.pi)) if lim is not None else -math.pi
        upper = float(lim.get("upper", math.pi)) if lim is not None else math.pi
        out[name] = {"axis": a / np.linalg.norm(a), "child": j.find("child").get("link"), "lower": lower, "upper": upper}
    missing = set(names) - set(out)
    if missing:
        raise ValueError(f"joints not in the robot description: {', '.join(sorted(missing))}")
    return out


def rpy_matrix(r, p, y):
    """URDF rpy (fixed axes x, y, z) -> 3x3."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def chain(urdf_xml, base, tip):
    """The joints from link `base` down to link `tip`: [(name, type, origin 4x4, axis)], base first. For forward
    kinematics without a MoveIt round trip per evaluation (an IK solve takes thousands)."""
    root = ET.fromstring(urdf_xml)
    by_child = {j.find("child").get("link"): j for j in root.findall("joint") if j.find("child") is not None}
    out, link = [], tip
    while link != base:
        j = by_child.get(link)
        if j is None:
            raise ValueError(f"no joint chain from {base} to {tip}")
        o = j.find("origin")
        T = np.eye(4)
        if o is not None:
            T[:3, :3] = rpy_matrix(*[float(v) for v in o.get("rpy", "0 0 0").split()])
            T[:3, 3] = [float(v) for v in o.get("xyz", "0 0 0").split()]
        axis = j.find("axis")
        a = np.array([float(v) for v in axis.get("xyz").split()]) if axis is not None else np.array([1.0, 0, 0])
        out.append((j.get("name"), j.get("type"), T, a))
        link = j.find("parent").get("link")
    return out[::-1]


def chain_fk(ch, q):
    """4x4 of the chain's tip link in its base link, for joint values q {name: value} (missing joints at 0)."""
    T = np.eye(4)
    for name, kind, origin, axis in ch:
        T = T @ origin
        if kind in ("revolute", "continuous"):
            M = np.eye(4)
            M[:3, :3] = axis_angle(axis, q.get(name, 0.0))
            T = T @ M
        elif kind == "prismatic":
            M = np.eye(4)
            M[:3, 3] = np.asarray(axis, float) * q.get(name, 0.0)
            T = T @ M
    return T


def aim_step(cam_origin, cam_axis, target, joint_axes, max_step=0.6):
    """Joint changes (one per axis) that turn the camera's optical axis towards the target, linearised:
    rotating about a joint axis `a` by dq moves the viewing direction by (a x c) dq. All vectors in one frame.
    Returns (dq array, angle error in radians before the step)."""
    c = cam_axis / np.linalg.norm(cam_axis)
    d = target - cam_origin
    d = d / np.linalg.norm(d)
    err = math.acos(max(-1.0, min(1.0, float(c @ d))))
    J = np.stack([np.cross(a, c) for a in joint_axes], axis=1)
    dq, *_ = np.linalg.lstsq(J, d - c * float(c @ d), rcond=None)
    # d - c(c.d) is the component of the target direction across the view; scale it to the true angle
    across = np.linalg.norm(d - c * float(c @ d))
    if across > 1e-9:
        dq = dq * (err / across)
    n = np.max(np.abs(dq))
    if n > max_step:
        dq = dq * (max_step / n)
    return dq, err
