"""A cube's pose from its outline in one wrist camera frame (no ROS, so it can be tested on saved frames).

The wrist camera looks past the claw at the object, so part of the object is often behind the claw. The claw does
not move in the image (camera and claw are on the same link), so a fixed mask per camera marks it (an occluder mask,
tools/wrist_occluder.py). The fit moves a cube of known size, resting flat (yaw about the vertical of the base frame),
so that its projected outline runs through the mask's outline where the object meets the background, and stays
within the mask or the claw everywhere else. The cube's size fixes its distance from the camera.
"""
import cv2
import numpy as np
from scipy.optimize import least_squares

_UNIT = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float) / 2.0


def _rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def project_cube(T_cam_base, centre, yaw, size, K, D):
    """Image points of the cube's eight corners."""
    pts = (_rotz(yaw) @ (_UNIT * size).T).T + centre
    cam = (T_cam_base[:3, :3] @ pts.T).T + T_cam_base[:3, 3]
    uv, _ = cv2.projectPoints(cam.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D if np.any(D) else None)
    return uv.reshape(-1, 2)


def _perimeter_samples(poly, n):
    hv = poly.reshape(-1, 2).astype(float)
    seg = np.roll(hv, -1, 0) - hv
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(seg, axis=1))])
    t = np.linspace(0.0, cum[-1], n, endpoint=False)
    i = np.clip(np.searchsorted(cum, t, side="right") - 1, 0, len(hv) - 1)
    f = (t - cum[i]) / np.maximum(cum[i + 1] - cum[i], 1e-9)
    return hv[i] + seg[i] * f[:, None]


def fit_cube(mask, occluder, T_base_cam, K, D, size, prior, fixed_yaw=None):
    """(centre in base, yaw, rms px, visible share) or None. mask, occluder: bool images; prior: centre guess (base).
    With fixed_yaw only x and y are fitted (height from the prior): enough when the claw hides much of the object."""
    h, w = mask.shape
    m8 = mask.astype(np.uint8)
    occ = cv2.dilate(occluder.astype(np.uint8), np.ones((5, 5), np.uint8)) if occluder is not None else np.zeros_like(m8)
    cs, _ = cv2.findContours(m8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cs:
        return None
    c = max(cs, key=cv2.contourArea).reshape(-1, 2)
    # outline points where the object meets the background: not against the claw, not at the image border
    ring = cv2.dilate(m8, np.ones((7, 7), np.uint8)) - m8
    against = cv2.dilate(occ & ring, np.ones((5, 5), np.uint8))
    keep = np.array([not against[y, x] and 3 < x < w - 4 and 3 < y < h - 4 for x, y in c])
    edge = c[keep].astype(np.float32)[::2]
    if len(edge) < 12:
        return None
    allowed = ((m8 > 0) | (occ > 0)).astype(np.uint8)
    outside = cv2.distanceTransform(1 - allowed, cv2.DIST_L2, 3)        # pixels from the mask or the claw
    T_cam_base = np.linalg.inv(T_base_cam)

    def outline(q):
        return cv2.convexHull(project_cube(T_cam_base, q[:3], q[3], size, K, D).astype(np.float32))

    def resid(q):
        hull = outline(q)
        r1 = [cv2.pointPolygonTest(hull, (float(u), float(v)), True) for u, v in edge]
        s = _perimeter_samples(hull, 48)
        r2 = outside[np.clip(s[:, 1], 0, h - 1).astype(int), np.clip(s[:, 0], 0, w - 1).astype(int)]
        return np.concatenate([np.asarray(r1, float), r2])

    if fixed_yaw is not None:
        def resid_xy(p):
            return resid([p[0], p[1], prior[2], fixed_yaw])
        r = least_squares(resid_xy, list(prior[:2]), x_scale=[0.005, 0.005], loss="soft_l1", f_scale=2.0)
        q = np.array([r.x[0], r.x[1], prior[2], fixed_yaw])
    else:
        best = None
        for yaw0 in np.radians([0.0, 22.5, 45.0, 67.5]):               # a cube repeats every 90 degrees
            r = least_squares(resid, [*prior, yaw0], x_scale=[0.005, 0.005, 0.005, 0.2], loss="soft_l1", f_scale=2.0)
            if best is None or r.cost < best.cost:
                best = r
        q = best.x
    rms = float(np.sqrt(np.mean(resid(q)[:len(edge)] ** 2)))
    s = _perimeter_samples(outline(q), 96)
    inside = (s[:, 0] >= 0) & (s[:, 0] < w) & (s[:, 1] >= 0) & (s[:, 1] < h)
    seen = np.zeros(len(s), bool)
    xi, yi = np.clip(s[:, 0], 0, w - 1).astype(int), np.clip(s[:, 1], 0, h - 1).astype(int)
    seen[inside] = ~(occ[yi[inside], xi[inside]] > 0)
    return q[:3], float(q[3] % (np.pi / 2)), rms, float(seen.mean())
