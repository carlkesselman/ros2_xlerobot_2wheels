#!/usr/bin/env python3
"""Experiment 1 of ADR-0005: how well does the head camera see a cube's pose, with no learned model?

Run on the Jetson (needs the stack's head camera):

  python3 tools/cube_pose_eval.py [--sheet ~/placement_sheet.json] [--scale 0.998] [--frames 20]

The printed placement sheet (tools/generate_placement_sheet.py) gives the table plane and a frame to
measure in: its markers are fitted with a pose, scaled by the measured print scale. For each footprint you
put the cube on, the yellow mask is cut out of the colour image and the outline of a 28 mm cube on that plane
(its silhouette, the convex hull of the 8 corners) is fitted to it by position and yaw. The result is
compared with where the footprint is. Every frame is a separate estimate, so the spread over the frames is the
repeatability and the difference to the footprint is the error. A CSV of every frame goes to ~/pose_eval/.

Keys at the prompt: Enter = measure the footprint shown, s = skip it, q = stop and print the summary.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

_venv = os.path.expanduser("~/venvs/xlerobot-mcp/bin/python")
if os.path.exists(_venv) and os.path.realpath(sys.prefix) != os.path.realpath(os.path.dirname(os.path.dirname(_venv))):
    os.execv(_venv, [_venv] + sys.argv)

import cv2  # noqa: E402
import cv2.aruco as aruco  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from scipy.optimize import minimize  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402

CUBE_MM = 28.0
YELLOW_LO, YELLOW_HI = (20, 90, 90), (40, 255, 255)


def sheet_pose(img, sheet, scale, K):
    """Pose of the sheet in the camera (4x4, millimetres), from its markers; None if too few agree."""
    dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    mc, mi, _ = aruco.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), dic)
    if mi is None or len(mi) < 4:
        return None, 0
    by_id = {m["id"]: np.array(m["corners_mm"], np.float32) * scale for m in sheet["markers"]}
    keep = [k for k, i in enumerate(mi.ravel()) if int(i) in by_id]
    if len(keep) < 4:
        return None, 0
    src = np.array([by_id[int(mi[k][0])].mean(0) for k in keep], np.float32)
    dst = np.array([mc[k].reshape(-1, 2).mean(0) for k in keep], np.float32)
    side = float(np.mean([np.linalg.norm(mc[k][0][0] - mc[k][0][1]) for k in keep]))
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 0.3 * side)       # drops duplicates of the same ids elsewhere
    if H is None or int(mask.sum()) < 4:
        return None, 0
    good = [keep[k] for k in range(len(keep)) if mask[k]]
    obj = np.concatenate([np.c_[by_id[int(mi[k][0])], np.zeros(4)] for k in good]).astype(np.float32)
    px = np.concatenate([mc[k].reshape(-1, 2) for k in good]).astype(np.float32)
    n, rvs, tvs, errs = cv2.solvePnPGeneric(obj, px, K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE)
    if not n:
        return None, 0
    b = int(np.argmin([float(e) for e in errs]))
    rv, tv = cv2.solvePnPRefineLM(obj, px, K, np.zeros(5), rvs[b], tvs[b])
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rv)[0]
    T[:3, 3] = tv.ravel()
    return T, len(good)


def project(T, K, pts):
    p = (T[:3, :3] @ np.asarray(pts).T).T + T[:3, 3]
    uv = (K @ p.T).T
    return uv[:, :2] / uv[:, 2:3]


def cube_corners(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    h = CUBE_MM / 2
    sq = [(-h, -h), (h, -h), (h, h), (-h, h)]
    pts = [(x + c * a - s * b, y + s * a + c * b, z) for z in (0.0, CUBE_MM) for a, b in sq]
    return np.array(pts)


def fit_cube(mask, T, K, guess_xy):
    """x, y (sheet mm) and yaw (rad, modulo 90 degrees) of the cube whose silhouette best covers the mask."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 80:
        return None
    x0, x1, y0, y1 = max(xs.min() - 12, 0), xs.max() + 12, max(ys.min() - 12, 0), ys.max() + 12
    S = 4                                                     # sub-pixel by rasterising 4x finer
    roi = cv2.resize(mask[y0:y1, x0:x1].astype(np.uint8), None, fx=S, fy=S, interpolation=cv2.INTER_NEAREST) > 0

    def cost(p):
        uv = (project(T, K, cube_corners(p[0], p[1], p[2])) - [x0, y0]) * S
        hull = cv2.convexHull(uv.astype(np.float32)).astype(np.int32)
        r = np.zeros_like(roi, np.uint8)
        cv2.fillConvexPoly(r, hull, 1)
        inter = np.logical_and(r > 0, roi).sum()
        return 1.0 - inter / max(1, np.logical_or(r > 0, roi).sum())

    best = None
    for yaw0 in np.radians((0, 22.5, 45, 67.5)):
        res = minimize(cost, [guess_xy[0], guess_xy[1], yaw0], method="Nelder-Mead",
                       options={"initial_simplex": np.array([[guess_xy[0], guess_xy[1], yaw0], [guess_xy[0] + 6, guess_xy[1], yaw0],
                                                             [guess_xy[0], guess_xy[1] + 6, yaw0], [guess_xy[0], guess_xy[1], yaw0 + 0.2]]),
                                "xatol": 0.02, "fatol": 1e-5, "maxiter": 400})
        if best is None or res.fun < best.fun:
            best = res
    return best.x[0], best.x[1], (best.x[2] + math.pi / 4) % (math.pi / 2) - math.pi / 4, 1.0 - best.fun


def yellow_mask(img):
    m = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), YELLOW_LO, YELLOW_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    return lab == k


class Cam:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("cube_pose_eval")
        self.frames, self.K = [], None
        self.node.create_subscription(Image, "/cameras/head/image_raw", self._img, qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo, "/cameras/head/camera_info", self._info, qos_profile_sensor_data)

    def _info(self, m):
        self.K = np.array(m.k).reshape(3, 3)

    def _img(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        self.frames.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, a[:, :, ::-1].copy() if m.encoding == "rgb8" else a.copy()))
        del self.frames[:-3]

    def next_frames(self, n, timeout=30.0):
        out, last, t0 = [], 0.0, time.time()
        while len(out) < n and time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.frames and self.frames[-1][0] > last + 0.05 and self.K is not None:
                last = self.frames[-1][0]
                out.append(self.frames[-1][1])
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sheet", default=os.path.expanduser("~/placement_sheet.json"))
    ap.add_argument("--scale", type=float, default=1.0, help="measured print scale of the sheet")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--ids", nargs="*", help="footprint ids to measure, default every cube footprint")
    a = ap.parse_args()
    sheet = json.load(open(a.sheet))
    prints = {f["id"]: f for f in sheet["footprints"] if f["kind"] == "cube"}
    ids = a.ids or sorted(prints)
    cam = Cam()
    out_dir = os.path.expanduser("~/pose_eval")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, time.strftime("cube_%Y%m%d_%H%M%S.csv"))
    rows, summary = [], []
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["footprint", "frame", "x_mm", "y_mm", "yaw_deg", "iou", "dx_mm", "dy_mm", "dyaw_deg", "markers"])
        for fid in ids:
            fp = prints[fid]
            cmd = input(f"\nplace the cube on {fid} (brackets, yaw {fp['yaw_deg']:g} deg), then Enter  [s skip, q quit] ").strip().lower()
            if cmd == "q":
                break
            if cmd == "s":
                continue
            print("  taking frames...", flush=True)
            frames = cam.next_frames(a.frames, timeout=10.0)
            if not frames:
                print("  no frames from /cameras/head/image_raw (is the head camera running?)")
                continue
            res = []
            for i, img in enumerate(frames):
                T, nm = sheet_pose(img, sheet, a.scale, cam.K)
                m = yellow_mask(img)
                if T is None or m is None:
                    continue
                truth = np.array(fp["center_mm"]) * a.scale
                z = project(T, cam.K, [(truth[0], truth[1], CUBE_MM / 2)])[0]
                near = np.zeros_like(m)
                r = 90
                near[int(max(z[1] - r, 0)):int(z[1] + r), int(max(z[0] - r, 0)):int(z[0] + r)] = True
                mm = m & near
                r = fit_cube(mm, T, cam.K, truth)
                if r is None:
                    continue
                x, y, yaw, iou = r
                dyaw = math.degrees((yaw - math.radians(fp["yaw_deg"]) + math.pi / 4) % (math.pi / 2) - math.pi / 4)
                res.append((x, y, math.degrees(yaw), iou, x - truth[0], y - truth[1], dyaw, nm))
                w.writerow([fid, i, f"{x:.2f}", f"{y:.2f}", f"{math.degrees(yaw):.2f}", f"{iou:.3f}", f"{x - truth[0]:.2f}",
                            f"{y - truth[1]:.2f}", f"{dyaw:.2f}", nm])
            if not res:
                print("  no usable frame (sheet markers or the cube not found)")
                continue
            r = np.array(res)
            err = float(np.hypot(r[:, 4].mean(), r[:, 5].mean()))
            print(f"  {len(r)}/{len(frames)} frames, silhouette IoU {r[:, 3].mean():.3f}: error to the footprint "
                  f"dx {r[:, 4].mean():+.2f} dy {r[:, 5].mean():+.2f} mm ({err:.2f} mm), dyaw {r[:, 6].mean():+.2f} deg | "
                  f"repeatability sigma x {r[:, 0].std():.2f} y {r[:, 1].std():.2f} mm, yaw {r[:, 2].std():.2f} deg")
            summary.append((fid, err, abs(r[:, 6].mean()), r[:, 0].std(), r[:, 1].std(), r[:, 2].std()))
    if summary:
        s = np.array([x[1:] for x in summary])
        print(f"\n{len(summary)} placements; horizontal error max {s[:, 0].max():.2f} mm, mean {s[:, 0].mean():.2f} mm; "
              f"yaw error max {s[:, 1].max():.2f} deg; repeatability sigma xy up to {max(s[:, 2].max(), s[:, 3].max()):.2f} mm, "
              f"yaw up to {s[:, 4].max():.2f} deg\nper-frame results: {path}")
    cam.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
