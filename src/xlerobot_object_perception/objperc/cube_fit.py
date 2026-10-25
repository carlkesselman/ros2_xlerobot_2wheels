"""Pose of a cube of known size from its silhouette: the outline of the cube standing on a plane is fitted to a mask.

Shared by the object perception node and tools/cube_pose_eval.py. `T` maps plane coordinates (millimetres, z up from the plane)
to the camera frame; `K` and `D` are the camera matrix and distortion.
"""
import math

import cv2
import numpy as np
from scipy.optimize import minimize


def project(T, K, pts, D=None):
    p = (T[:3, :3] @ np.asarray(pts, float).T).T + T[:3, 3]
    if D is None or not np.any(D):
        uv = (K @ p.T).T
        return uv[:, :2] / uv[:, 2:3]
    uv, _ = cv2.projectPoints(p.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, np.asarray(D, float))
    return uv.reshape(-1, 2)


def cube_corners(x, y, yaw, size=28.0):
    c, s = math.cos(yaw), math.sin(yaw)
    h = size / 2
    sq = [(-h, -h), (h, -h), (h, h), (-h, h)]
    return np.array([(x + c * a - s * b, y + s * a + c * b, z) for z in (0.0, size) for a, b in sq])


def fit_cube(mask, T, K, guess_xy, D=None, size=28.0):
    """x, y (plane mm), yaw (rad, modulo 90 degrees) and the overlap (1 = perfect) of the cube whose silhouette best covers the mask."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 80:
        return None
    x0, x1, y0, y1 = max(xs.min() - 12, 0), xs.max() + 12, max(ys.min() - 12, 0), ys.max() + 12
    S = 4                                                     # sub-pixel by rasterising 4x finer
    roi = cv2.resize(mask[y0:y1, x0:x1].astype(np.uint8), None, fx=S, fy=S, interpolation=cv2.INTER_NEAREST) > 0

    def cost(p):
        uv = (project(T, K, cube_corners(p[0], p[1], p[2], size), D) - [x0, y0]) * S
        hull = cv2.convexHull(uv.astype(np.float32)).astype(np.int32)
        r = np.zeros_like(roi, np.uint8)
        cv2.fillConvexPoly(r, hull, 1)
        inter = np.logical_and(r > 0, roi).sum()
        return 1.0 - inter / max(1, np.logical_or(r > 0, roi).sum())

    # a coarse look at the yaw first (the cost is cheap to evaluate once), then the simplex only from the two best starting yaws
    coarse = sorted(np.radians(np.arange(0.0, 90.0, 7.5)), key=lambda t: cost([guess_xy[0], guess_xy[1], t]))[:2]
    best = None
    for yaw0 in coarse:
        res = minimize(cost, [guess_xy[0], guess_xy[1], yaw0], method="Nelder-Mead",
                       options={"initial_simplex": np.array([[guess_xy[0], guess_xy[1], yaw0], [guess_xy[0] + 6, guess_xy[1], yaw0],
                                                             [guess_xy[0], guess_xy[1] + 6, yaw0], [guess_xy[0], guess_xy[1], yaw0 + 0.2]]),
                                "xatol": 0.02, "fatol": 1e-5, "maxiter": 400})
        if best is None or res.fun < best.fun:
            best = res
    return best.x[0], best.x[1], (best.x[2] + math.pi / 4) % (math.pi / 2) - math.pi / 4, 1.0 - best.fun
