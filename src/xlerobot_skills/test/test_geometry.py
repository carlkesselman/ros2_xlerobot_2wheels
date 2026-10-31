import math

import numpy as np

from xlerobot_skills.geometry import aim_step, joint_info


def rot(axis, q):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


def camera(pan, tilt):
    """A pan-tilt head: pan about z at the origin, tilt about the panned y axis 0.1 m up; camera looks along x."""
    Rp = rot([0, 0, 1], pan)
    Rt = Rp @ rot([0, 1, 0], tilt)
    origin = np.array([0, 0, 0.1]) + Rt @ np.array([0.03, 0, 0.02])
    return origin, Rt @ np.array([1.0, 0, 0]), [Rp @ np.array([0, 0, 1.0]), Rp @ np.array([0, 1.0, 0])]


def test_aim_converges():
    target = np.array([0.4, -0.25, -0.3])
    q = np.zeros(2)
    for _ in range(4):
        o, c, axes = camera(*q)
        dq, err = aim_step(o, c, target, axes)
        q = q + dq
    o, c, _ = camera(*q)
    d = (target - o) / np.linalg.norm(target - o)
    assert math.degrees(math.acos(c @ d)) < 0.5


def test_joint_info():
    urdf = """<robot name="r"><joint name="head_pan_joint" type="revolute"><axis xyz="0 0 -1"/>
              <child link="head_pan_link"/><limit lower="-1.5" upper="1.5" effort="1" velocity="1"/></joint></robot>"""
    j = joint_info(urdf, ["head_pan_joint"])["head_pan_joint"]
    assert np.allclose(j["axis"], [0, 0, -1]) and j["lower"] == -1.5
