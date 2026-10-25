#!/usr/bin/env python3
"""Check the object pose service against the printed placement sheet (run on the Jetson with the stack and the service up).

  python3 tools/perception_check.py [--sheet ~/placement_sheet.json] [--scale 0.998] [--repeat 3] [--prompt "a cube"]

For each repeat it calls /perception/detect, finds the sheet in a fresh colour frame, expresses the returned pose in the sheet
frame through TF and prints its distance to the closest cube footprint, so a cube put on a footprint gives the pose error.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.time import Time  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

import cube_pose_eval as cpe  # noqa: E402
from xlerobot_object_perception.srv import DetectObjects  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sheet", default=os.path.expanduser("~/placement_sheet.json"))
    ap.add_argument("--scale", type=float, default=0.998)
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--prompt", default="a cube")
    a = ap.parse_args()
    sheet = json.load(open(a.sheet))
    cams = cpe.Cam()
    node = cams.node
    tfb = Buffer()
    TransformListener(tfb, node)
    cli = node.create_client(DetectObjects, "/perception/detect")
    if not cli.wait_for_service(timeout_sec=5.0):
        sys.exit("no /perception/detect service (is object_perception running?)")
    for _ in range(20):                                                   # let TF fill
        rclpy.spin_once(node, timeout_sec=0.1)
    feet = [f for f in sheet["footprints"] if f["kind"] == "cube"]
    for i in range(a.repeat):
        req = DetectObjects.Request()
        req.prompt, req.label = a.prompt, "cube"
        fut = cli.call_async(req)
        while not fut.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        res = fut.result()
        frames = cams.next_frames(1, timeout=5.0)
        if not frames or not res.detections.detections:
            print(f"[{i}] no detection, codes {list(res.codes)}")
            continue
        stamp, img = frames[0]
        T, nm = cpe.sheet_pose(img, sheet, a.scale, cams.K, cams.D)
        if T is None:
            print(f"[{i}] sheet not found")
            continue
        tf = tfb.lookup_transform(cams.frame_id, "base_footprint", Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        Tcb = np.eye(4)
        q, t = tf.transform.rotation, tf.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        Tcb[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                       [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                       [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        Tcb[:3, 3] = [t.x, t.y, t.z]
        Tcb[:3, 3] *= 1000.0                                                  # the sheet pose is in millimetres
        Tsc = np.linalg.inv(T)
        for d, h in zip(res.detections.detections, res.heights_m):
            p = d.bbox.center.position
            ps = (Tsc @ Tcb @ [p.x * 1000, p.y * 1000, p.z * 1000, 1.0])[:3] / a.scale             # sheet frame, mm
            yaw_b = 2 * math.atan2(d.bbox.center.orientation.z, d.bbox.center.orientation.w)
            R_sb = (Tsc @ Tcb)[:3, :3]
            yaw_s = math.degrees(yaw_b + math.atan2(R_sb[1, 0], R_sb[0, 0]))
            near = min(feet, key=lambda f: math.hypot(f["center_mm"][0] - ps[0], f["center_mm"][1] - ps[1]))
            dx, dy = ps[0] - near["center_mm"][0], ps[1] - near["center_mm"][1]
            dyaw = (yaw_s - near["yaw_deg"] + 45) % 90 - 45
            print(f"[{i}] {d.id}: nearest {near['id']}  dx {dx:+.2f} dy {dy:+.2f} mm ({math.hypot(dx, dy):.2f}), dyaw {dyaw:+.1f} deg, "
                  f"height above sheet {ps[2] - 14:+.1f} mm, depth height {h * 1000:.1f} mm, score {d.results[0].hypothesis.score:.2f}, "
                  f"{res.latency_ms:.0f} ms, codes {list(res.codes)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
