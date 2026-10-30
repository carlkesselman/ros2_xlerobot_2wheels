#!/usr/bin/env python3
"""Save a burst of the head camera's colour frames for a lens calibration (run on the Jetson).

  python3 tools/head_record.py [--seconds 40] [--every 0.3]

Move the 7x5 ChArUco board slowly through many poses while it records: near and far (30-80 cm), flat and
tilted, and above all across the whole image, including the bottom and the corners (where the lens
distortion is largest and where objects are measured). Frames go to ~/head_calib/calib_<time>/. Fit them on
the laptop with: python3 tools/side_cam.py calibrate --from <folder> --name head_color
"""
import argparse
import os
import sys
import time

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--every", type=float, default=0.3)
    a = ap.parse_args()
    out = os.path.expanduser(time.strftime("~/head_calib/calib_%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("head_record")
    last = [None]
    node.create_subscription(Image, "/cameras/head/image_raw", lambda m: last.__setitem__(0, m), qos_profile_sensor_data)
    print(f"recording {a.seconds:g} s to {out}: move the board through the whole image, including the bottom and the corners")
    t0, n, t_next = time.time(), 0, 0.0
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        m = last[0]
        if m is not None and time.time() >= t_next:
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            cv2.imwrite(os.path.join(out, f"{n:04d}.png"), img[:, :, ::-1] if m.encoding == "rgb8" else img)
            n += 1
            last[0] = None
            t_next = time.time() + a.every
    print(f"{n} frames in {out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
