#!/usr/bin/env python3
"""How far the claw tip really lands from where it was sent, over the placement sheet (issue #12).

  python3 tools/arm_accuracy.py [--arms right left] [--height 0.03] [--ids cube2 brick3 ...]

Needs the stack, move_group, the skill server and both wrist cameras, with the placement sheet
(tools/generate_placement_sheet.py) flat on the table where the head camera sees its markers and the arms reach its
footprints, and nothing standing on it.

The head camera fits the sheet once. For each footprint on an arm's side, the arm puts its claw tip `--height` above
the footprint's centre (move_arm with tool_point, wrist camera on top). The wrist camera then fits the sheet's
markers as well, which gives the hand's real pose in the head camera's frame: comparing it with the pose the joint
encoders and the model predict gives the claw tip's real position, so the error covers the whole chain from head
perception to arm motion, without finding the tip in an image. Per target it prints the real tip's offset from the
target (dx forward, dy left, dz up, mm) and the model's own residual; a CSV, the wrist frames and the measured poses go
to ~/arm_accuracy/<time>/.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from xlerobot_skill_msgs.action import MoveArm
from xlerobot_skills.geometry import transform_to_matrix

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cube_pose_eval as cpe  # noqa: E402

TIP = {"left": [0.0, -0.111, 0.0], "right": [0.0, -0.132, 0.0]}     # as config/skills.yaml of xlerobot_skills


class Camera:
    def __init__(self, node, topic):
        self.frame, self.K, self.D, self.frame_id = None, None, None, None
        node.create_subscription(Image, f"{topic}/image_raw", self._img, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f"{topic}/camera_info", self._info, qos_profile_sensor_data)

    def _img(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        self.frame = (time.monotonic(), a[:, :, ::-1].copy() if m.encoding == "rgb8" else a.copy())
        self.frame_id = m.header.frame_id

    def _info(self, m):
        self.K, self.D = np.array(m.k).reshape(3, 3), np.array(m.d, float)
        self.frame_id = self.frame_id or m.header.frame_id


def spin_for(node, seconds):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.02)


def fresh(node, cam, after, timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.02)
        if cam.frame is not None and cam.frame[0] > after and cam.K is not None:
            return cam.frame[1]
    return None


def lookup(tfb, target, source):
    return transform_to_matrix(tfb.lookup_transform(target, source, Time(), timeout=Duration(seconds=2.0)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arms", nargs="+", default=["right", "left"])
    ap.add_argument("--height", type=float, default=0.03, help="claw tip above the footprint centre, m")
    ap.add_argument("--ids", nargs="*", help="footprint ids, default every footprint on the arm's side")
    ap.add_argument("--sheet", default=os.path.expanduser("~/placement_sheet.json"))
    ap.add_argument("--scale", type=float, default=0.998)
    a = ap.parse_args()
    sheet = json.load(open(a.sheet))
    out = os.path.expanduser(time.strftime("~/arm_accuracy/%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("arm_accuracy")
    tfb = Buffer()
    TransformListener(tfb, node)
    head = Camera(node, "/cameras/head")
    wrist = {arm: Camera(node, f"/cameras/{arm}") for arm in a.arms}
    move = ActionClient(node, MoveArm, "/skills/move_arm")
    spin_for(node, 1.5)
    if not move.wait_for_server(timeout_sec=5.0):
        sys.exit("no /skills/move_arm (is the skill server running?)")

    def run(goal, timeout=120.0):
        f = move.send_goal_async(goal)
        while not f.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        h = f.result()
        r = h.get_result_async()
        t0 = time.monotonic()
        while not r.done() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
        return r.result().result.result if r.done() else None

    toggle = node.create_client(SetBool, "/cameras/head/toggle_color")

    def head_frame():
        """A fresh head frame. The Gemini driver's colour output sometimes stops after a subscriber leaves (frames
        still arrive from the camera); switching the colour stream off and on restarts it."""
        for attempt in range(3):
            img = fresh(node, head, time.monotonic(), timeout=3.0)
            if img is not None:
                return img
            if toggle.wait_for_service(timeout_sec=2.0):
                for on in (False, True):
                    f = toggle.call_async(SetBool.Request(data=on))
                    while not f.done():
                        rclpy.spin_once(node, timeout_sec=0.05)
                    spin_for(node, 2.0)
                print("  head colour stream restarted")
        return None

    for arm in ("left", "right"):                          # arms out of the head camera's view of the sheet
        r = run(MoveArm.Goal(arm=arm, named_position=f"{arm}_rest"))
        if r is None or r.code != 0:
            sys.exit(f"{arm} arm did not go to rest ({r.message if r else 'timeout'})")

    # the sheet from the head camera: best of a few frames
    best = None
    for _ in range(5):
        img = head_frame()
        if img is None:
            sys.exit("no head camera frames")
        T, n = cpe.sheet_pose(img, sheet, a.scale, head.K, head.D)
        if T is not None and (best is None or n > best[1]):
            best = (T, n, img)
    if best is None:
        sys.exit("the head camera does not see the sheet's markers")
    T_head_sheet, n_head, img = best
    cv2.imwrite(os.path.join(out, "head.jpg"), img)
    T_base_head = lookup(tfb, "base_footprint", head.frame_id)
    T_base_sheet = T_base_head.copy()
    T_base_sheet[:3, :3] = T_base_head[:3, :3] @ T_head_sheet[:3, :3]
    T_base_sheet[:3, 3] = T_base_head[:3, :3] @ (T_head_sheet[:3, 3] / 1000.0) + T_base_head[:3, 3]
    print(f"sheet from the head camera: {n_head} markers, centre at {np.round(T_base_sheet[:3, 3], 3).tolist()} m")

    rows = []
    for arm in a.arms:
        side = 1.0 if arm == "left" else -1.0
        targets = []
        for f in sheet["footprints"]:
            c = np.array(f["center_mm"]) * a.scale / 1000.0
            p = (T_base_sheet @ [c[0], c[1], 0.0, 1.0])[:3]
            if (a.ids and f["id"] in a.ids) or (not a.ids and side * p[1] > 0.0):
                targets.append((f["id"], p))
        for fid, p in sorted(targets, key=lambda t: (-t[1][0], -side * t[1][1])):
            goal = MoveArm.Goal(arm=arm, tool_point=True, table_z=float(p[2]))
            goal.point.header.frame_id = "base_footprint"
            goal.point.point.x, goal.point.point.y, goal.point.point.z = float(p[0]), float(p[1]), float(p[2] + a.height)
            res = run(goal)
            if res is None or res.code != 0:
                print(f"{arm} {fid}: move failed ({res.code if res else 'timeout'}: {res.message if res else ''})")
                continue
            spin_for(node, 0.8)
            target = p + [0.0, 0.0, a.height]
            meas = []
            for k in range(3):
                img = fresh(node, wrist[arm], time.monotonic())
                if img is None:
                    break
                Tw, nw = cpe.sheet_pose(img, sheet, a.scale, wrist[arm].K, wrist[arm].D, min_span=(20, 0), min_markers=1)
                cv2.imwrite(os.path.join(out, f"{arm}_{fid}_{k}.jpg"), img)
                if Tw is None:
                    continue
                T_wcam_sheet = Tw.copy()
                T_wcam_sheet[:3, 3] /= 1000.0
                meas.append((T_base_sheet @ np.linalg.inv(T_wcam_sheet), nw))
            if not meas:
                print(f"{arm} {fid}: the wrist camera does not see enough of the sheet")
                continue
            T_model_cam = lookup(tfb, "base_footprint", wrist[arm].frame_id)
            T_model_link = lookup(tfb, "base_footprint", f"{arm}_arm_gripper_link")
            tip_model = (T_model_link @ [*TIP[arm], 1.0])[:3]
            tips, angles, axes = [], [], []
            for T_meas_cam, _ in meas:
                E = T_meas_cam @ np.linalg.inv(T_model_cam)               # where the hand really is, relative to the model
                tips.append((E @ [*tip_model, 1.0])[:3])
                c = (np.trace(E[:3, :3]) - 1.0) / 2.0
                angles.append(math.degrees(math.acos(max(-1.0, min(1.0, c)))))
                Re = E[:3, :3]                                            # rotation vector, base axes, degrees
                v = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]]) / 2.0
                axes.append(np.degrees(v / max(np.linalg.norm(v), 1e-9) * math.radians(angles[-1])))
            tip_real = np.median(tips, axis=0)
            err = (tip_real - target) * 1000.0
            res_model = (tip_model - target) * 1000.0
            spread = float(np.max(np.std(tips, axis=0)) * 1000.0) if len(tips) > 1 else float("nan")
            print(f"{arm} {fid:6s} real tip off by dx {err[0]:+5.1f} dy {err[1]:+5.1f} dz {err[2]:+5.1f} mm "
                  f"({np.linalg.norm(err[:2]):.1f} horizontal) | model {np.linalg.norm(res_model):.1f} mm | "
                  f"hand turned {np.median(angles):.1f} deg (about x {np.median(axes, axis=0)[0]:+.1f}, y {np.median(axes, axis=0)[1]:+.1f}, "
                  f"z {np.median(axes, axis=0)[2]:+.1f}) | {meas[0][1]} markers, spread {spread:.1f} mm")
            rows.append([arm, fid, *np.round(target, 4), *np.round(err, 1), *np.round(res_model, 1),
                         round(float(np.median(angles)), 2), *np.round(np.median(axes, axis=0), 2), meas[0][1], round(spread, 1)])
        rest = MoveArm.Goal(arm=arm, named_position=f"{arm}_rest")
        run(rest)
    with open(os.path.join(out, "accuracy.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["arm", "footprint", "x", "y", "z", "dx_mm", "dy_mm", "dz_mm", "model_dx", "model_dy", "model_dz",
                    "hand_angle_deg", "rot_x_deg", "rot_y_deg", "rot_z_deg", "markers", "spread_mm"])
        w.writerows(rows)
    for arm in a.arms:
        e = np.array([r[5:8] for r in rows if r[0] == arm], float)
        if len(e):
            h = np.hypot(e[:, 0], e[:, 1])
            print(f"{arm}: {len(e)} targets, horizontal error mean {h.mean():.1f} max {h.max():.1f} mm, "
                  f"mean offset dx {e[:, 0].mean():+.1f} dy {e[:, 1].mean():+.1f} dz {e[:, 2].mean():+.1f} mm")
    print("results:", out)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
