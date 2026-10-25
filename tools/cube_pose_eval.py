#!/usr/bin/env python3
"""Experiment 1 of ADR-0005: how well does the head camera see a cube's pose, with no learned model?

Run on the Jetson (needs the stack's head camera):

  python3 tools/cube_pose_eval.py [--sheet ~/placement_sheet.json] [--scale 0.998] [--frames 20]

The printed placement sheet (tools/generate_placement_sheet.py) gives the table plane and a frame to
measure in: its markers are fitted with a pose, scaled by the measured print scale. For each footprint you
put the cube on, the yellow mask is cut out of the colour image and the outline of a 28 mm cube on that plane
(its silhouette, the convex hull of the 8 corners) is fitted to it by position and yaw. The result is
compared with where the footprint is. Every frame is a separate estimate, so the spread over the frames is the
repeatability and the difference to the footprint is the error. A CSV of every frame goes to ~/pose_eval/, and
the frames themselves are saved next to it (colour, the nearest depth frame, the sheet pose, the camera
intrinsics and the truth) so any other model can be scored on the same data later: --no-save turns that off.

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
if __name__ == "__main__" and os.path.exists(_venv) and os.path.realpath(sys.prefix) != os.path.realpath(os.path.dirname(os.path.dirname(_venv))):
    os.execv(_venv, [_venv] + sys.argv)

import cv2  # noqa: E402
import cv2.aruco as aruco  # noqa: E402
import numpy as np  # noqa: E402
from scipy.optimize import minimize  # noqa: E402

try:                                    # the ROS side is only needed to record; scoring saved frames works without it
    import rclpy  # noqa: E402
    from rclpy.qos import qos_profile_sensor_data  # noqa: E402
    from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
except ImportError:
    rclpy = None

CUBE_MM = 28.0
YELLOW_LO, YELLOW_HI = (20, 90, 90), (40, 255, 255)


def sheet_pose(img, sheet, scale, K, D=None):
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
    good = keep
    ids_seen = [int(mi[k][0]) for k in keep]
    if len(set(ids_seen)) < len(ids_seen):          # the same id twice: another sheet or board is in view, keep what fits
        side = float(np.mean([np.linalg.norm(mc[k][0][0] - mc[k][0][1]) for k in keep]))
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 0.3 * side)
        if H is None or int(mask.sum()) < 4:
            return None, 0
        good = [keep[k] for k in range(len(keep)) if mask[k]]
    obj = np.concatenate([np.c_[by_id[int(mi[k][0])], np.zeros(4)] for k in good]).astype(np.float32)
    px = np.concatenate([mc[k].reshape(-1, 2) for k in good]).astype(np.float32)
    D = np.zeros(5) if D is None else np.asarray(D, float)
    n, rvs, tvs, errs = cv2.solvePnPGeneric(obj, px, K, D, flags=cv2.SOLVEPNP_IPPE)
    if not n:
        return None, 0
    span = np.ptp(obj[:, :2], axis=0)
    if span[0] < 100 or span[1] < 60:                # a sheet seen only along one edge gives a poor plane
        return None, 0
    if len(good) < len(sheet["markers"]) and not getattr(sheet_pose, "warned", False):
        print(f"  warning: only {len(good)} of {len(sheet['markers'])} sheet markers are visible; the sheet's tilt is poorly determined and positions away from the visible markers can be off by 1-3 mm (move the sheet or tilt the head so all of them are in view)")
        sheet_pose.warned = True
    b = int(np.argmin([float(np.ravel(e)[0]) for e in errs]))
    rv, tv = cv2.solvePnPRefineLM(obj, px, K, D, rvs[b], tvs[b])
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rv)[0]
    T[:3, 3] = tv.ravel()
    return T, len(good)


def project(T, K, pts, D=None):
    p = (T[:3, :3] @ np.asarray(pts, float).T).T + T[:3, 3]
    if D is None or not np.any(D):
        uv = (K @ p.T).T
        return uv[:, :2] / uv[:, 2:3]
    uv, _ = cv2.projectPoints(p.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, np.asarray(D, float))
    return uv.reshape(-1, 2)


def cube_corners(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    h = CUBE_MM / 2
    sq = [(-h, -h), (h, -h), (h, h), (-h, h)]
    pts = [(x + c * a - s * b, y + s * a + c * b, z) for z in (0.0, CUBE_MM) for a, b in sq]
    return np.array(pts)


def fit_cube(mask, T, K, guess_xy, D=None):
    """x, y (sheet mm) and yaw (rad, modulo 90 degrees) of the cube whose silhouette best covers the mask."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 80:
        return None
    x0, x1, y0, y1 = max(xs.min() - 12, 0), xs.max() + 12, max(ys.min() - 12, 0), ys.max() + 12
    S = 4                                                     # sub-pixel by rasterising 4x finer
    roi = cv2.resize(mask[y0:y1, x0:x1].astype(np.uint8), None, fx=S, fy=S, interpolation=cv2.INTER_NEAREST) > 0

    def cost(p):
        uv = (project(T, K, cube_corners(p[0], p[1], p[2]), D) - [x0, y0]) * S
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


def yellow_map(img, mask):
    """How yellow each pixel is, 0 (paper or mat) to 1 (the cube), scaled with the cube's own colour and the
    colour just around it, so the cube's edge sits where the colour is half way between the two."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hue_ok = ((hsv[:, :, 0] >= 18) & (hsv[:, :, 0] <= 45)).astype(np.float32)
    y = hsv[:, :, 1].astype(np.float32) * hue_ok / 255.0
    m8 = mask.astype(np.uint8)
    core = cv2.erode(m8, np.ones((7, 7), np.uint8)) > 0
    ring = (cv2.dilate(m8, np.ones((25, 25), np.uint8)) > 0) & ~(cv2.dilate(m8, np.ones((9, 9), np.uint8)) > 0)
    if core.sum() < 20 or ring.sum() < 50:
        return None
    hi, lo = float(np.median(y[core])), float(np.median(y[ring]))
    if hi - lo < 0.2:
        return None
    return np.clip((y - lo) / (hi - lo), 0.0, 1.0)


def fit_cube_soft(img, mask, T, K, guess_xy, D=None):
    """Like fit_cube, but scores the outline against the graded yellow map (anti-aliased rendering)."""
    ys, xs = np.nonzero(mask)
    if len(xs) < 80:
        return None
    pmap = yellow_map(img, mask)
    if pmap is None:
        return None
    x0, x1, y0, y1 = max(xs.min() - 12, 0), xs.max() + 12, max(ys.min() - 12, 0), ys.max() + 12
    S = 4
    roi = pmap[y0:y1, x0:x1]
    h, w = roi.shape

    def cost(p):
        uv = (project(T, K, cube_corners(p[0], p[1], p[2]), D) - [x0, y0]) * S
        hull = cv2.convexHull(uv.astype(np.float32)).astype(np.int32)
        r = np.zeros((h * S, w * S), np.uint8)
        cv2.fillConvexPoly(r, hull, 1)
        cov = r.reshape(h, S, w, S).mean(axis=(1, 3))
        return float(((cov - roi) ** 2).mean())

    best = None
    for yaw0 in np.radians((0, 22.5, 45, 67.5)):
        res = minimize(cost, [guess_xy[0], guess_xy[1], yaw0], method="Nelder-Mead",
                       options={"initial_simplex": np.array([[guess_xy[0], guess_xy[1], yaw0], [guess_xy[0] + 6, guess_xy[1], yaw0],
                                                             [guess_xy[0], guess_xy[1] + 6, yaw0], [guess_xy[0], guess_xy[1], yaw0 + 0.2]]),
                                "xatol": 0.02, "fatol": 1e-9, "maxiter": 400})
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
        self.frames, self.K, self.D, self.depths = [], None, np.zeros(5), []
        self.node.create_subscription(Image, "/cameras/head/image_raw", self._img, qos_profile_sensor_data)
        self.node.create_subscription(Image, "/cameras/head/depth/image_raw", self._depth, qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo, "/cameras/head/camera_info", self._info, qos_profile_sensor_data)

    def _info(self, m):
        self.K = np.array(m.k).reshape(3, 3)
        self.D = np.array(m.d, float)

    def _img(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        self.frames.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, a[:, :, ::-1].copy() if m.encoding == "rgb8" else a.copy()))
        del self.frames[:-3]

    def _depth(self, m):
        a = np.frombuffer(m.data, np.uint16 if m.encoding == "16UC1" else np.float32).reshape(m.height, m.width)
        self.depths.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, a.copy(), m.encoding))
        del self.depths[:-6]

    def nearest_depth(self, stamp):
        if not self.depths:
            return None
        d = min(self.depths, key=lambda x: abs(x[0] - stamp))
        return d if abs(d[0] - stamp) < 0.08 else None

    def next_frames(self, n, timeout=30.0):
        out, last, t0 = [], 0.0, time.time()
        while len(out) < n and time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.frames and self.frames[-1][0] > last + 0.05 and self.K is not None:
                last = self.frames[-1][0]
                out.append(self.frames[-1])
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sheet", default=os.path.expanduser("~/placement_sheet.json"))
    ap.add_argument("--scale", type=float, default=1.0, help="measured print scale of the sheet")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--ids", nargs="*", help="footprint ids to measure, default every cube footprint")
    ap.add_argument("--no-distortion", dest="distortion", action="store_false", help="ignore the lens distortion the camera reports")
    ap.add_argument("--no-save", dest="save", action="store_false", help="do not keep the frames")
    a = ap.parse_args()
    sheet = json.load(open(a.sheet))
    prints = {f["id"]: f for f in sheet["footprints"] if f["kind"] == "cube"}
    ids = a.ids or sorted(prints)
    cam = Cam()
    out_dir = os.path.expanduser("~/pose_eval")
    os.makedirs(out_dir, exist_ok=True)
    stamp_s = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(out_dir, f"cube_{stamp_s}.csv")
    sess_dir = os.path.join(out_dir, f"cube_{stamp_s}")
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
            keep_dir = None
            if a.save:
                keep_dir = os.path.join(sess_dir, fid)
                os.makedirs(keep_dir, exist_ok=True)
            meta_frames = []
            for i, (stamp, img) in enumerate(frames):
                T, nm = sheet_pose(img, sheet, a.scale, cam.K, cam.D if a.distortion else None)
                m = yellow_mask(img)
                if T is None or m is None:
                    continue
                truth = np.array(fp["center_mm"]) * a.scale
                z = project(T, cam.K, [(truth[0], truth[1], CUBE_MM / 2)], cam.D if a.distortion else None)[0]
                near = np.zeros_like(m)
                r = 90
                near[int(max(z[1] - r, 0)):int(z[1] + r), int(max(z[0] - r, 0)):int(z[0] + r)] = True
                mm = m & near
                r = fit_cube(mm, T, cam.K, truth, cam.D if a.distortion else None)
                if r is None:
                    continue
                x, y, yaw, iou = r
                if keep_dir:
                    cv2.imwrite(os.path.join(keep_dir, f"color_{i:02d}.png"), img)
                    d = cam.nearest_depth(stamp)
                    if d is not None:
                        cv2.imwrite(os.path.join(keep_dir, f"depth_{i:02d}.png"), d[1] if d[2] == "16UC1" else (d[1] * 1000).astype(np.uint16))
                    meta_frames.append({"index": i, "stamp": stamp, "depth_dt": None if d is None else d[0] - stamp,
                                        "T_camera_from_sheet": T.tolist(), "markers": nm, "fit": [x, y, math.degrees(yaw), iou]})
                dyaw = math.degrees((yaw - math.radians(fp["yaw_deg"]) + math.pi / 4) % (math.pi / 2) - math.pi / 4)
                res.append((x, y, math.degrees(yaw), iou, x - truth[0], y - truth[1], dyaw, nm))
                w.writerow([fid, i, f"{x:.2f}", f"{y:.2f}", f"{math.degrees(yaw):.2f}", f"{iou:.3f}", f"{x - truth[0]:.2f}",
                            f"{y - truth[1]:.2f}", f"{dyaw:.2f}", nm])
            if keep_dir:
                with open(os.path.join(keep_dir, "meta.json"), "w") as mf:
                    json.dump({"footprint": fp, "scale": a.scale, "K": cam.K.tolist(), "D": None if not a.distortion or cam.D is None else list(map(float, cam.D)), "sheet_file": a.sheet,
                               "cube_mm": CUBE_MM, "frames": meta_frames}, mf, indent=1)
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
