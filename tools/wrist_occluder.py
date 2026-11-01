#!/usr/bin/env python3
"""The claw's mask in a wrist camera's image, for /perception/locate_in_wrist (the align skill).

  python3 tools/wrist_occluder.py --arm right                      # live: a frame every 3 s while you move the arm
  python3 tools/wrist_occluder.py --arm right --from DIR           # from saved frames (*.jpg, *.png)

The camera and the claw sit on the same link, so the claw stays at the same pixels while everything else moves. Open
the gripper (the align skill works with it open) and move the hand over a coloured or dark surface, not white paper
(white paper and the white claw both saturate and look the same), at a few spots and heights, by teleop or with
goto_position. The mask is the pixels that stay the same in every frame and reach the bottom of the image. Writes
src/xlerobot_object_perception/config/wrist_occluder_<arm>.png (rebuild the package to install it) and a preview
in the current directory.
"""
import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def grab_live(arm, n, interval):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("wrist_occluder")
    latest = {}

    def on_img(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        latest["img"] = a[:, :, ::-1].copy() if m.encoding == "rgb8" else a.copy()
    node.create_subscription(Image, f"/cameras/{arm}/image_raw", on_img, qos_profile_sensor_data)
    frames = []
    for k in range(n):
        print(f"frame {k + 1}/{n} in {interval:.0f} s: move the hand", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < interval:
            rclpy.spin_once(node, timeout_sec=0.05)
        if "img" in latest:
            frames.append(latest["img"])
    node.destroy_node()
    rclpy.shutdown()
    return frames


def occluder(frames, tol=30.0, share=0.8, min_area=1500):
    stack = np.stack([cv2.GaussianBlur(f, (5, 5), 0).astype(np.float32) for f in frames])
    # near its median colour in most frames (auto-exposure moves the claw's brightness a little between frames)
    close = (np.abs(stack - np.median(stack, axis=0)).max(axis=3) < tol).mean(axis=0)
    still = (close >= share).astype(np.uint8)
    still = cv2.morphologyEx(still, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(still)
    h = still.shape[0]
    mask = np.zeros_like(still)
    for i in range(1, n):
        if st[i, cv2.CC_STAT_TOP] + st[i, cv2.CC_STAT_HEIGHT] >= h - 2 and st[i, cv2.CC_STAT_AREA] >= min_area:
            mask[lab == i] = 1
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return cv2.dilate(mask, np.ones((3, 3), np.uint8))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["left", "right"], required=True)
    ap.add_argument("--from", dest="src", help="folder of saved wrist frames instead of the live camera")
    ap.add_argument("--frames", type=int, default=8)
    ap.add_argument("--interval", type=float, default=3.0)
    ap.add_argument("--out", default=os.path.join(REPO, "src", "xlerobot_object_perception", "config"))
    a = ap.parse_args()
    if a.src:
        frames = [cv2.imread(p) for p in sorted(glob.glob(os.path.join(a.src, "*.jpg")) + glob.glob(os.path.join(a.src, "*.png")))]
    else:
        frames = grab_live(a.arm, a.frames, a.interval)
    if len(frames) < 4:
        sys.exit(f"need at least 4 frames, have {len(frames)}")
    mask = occluder(frames)
    path = os.path.join(a.out, f"wrist_occluder_{a.arm}.png")
    cv2.imwrite(path, mask * 255)
    preview = frames[-1].copy()
    preview[mask > 0] = (0.5 * preview[mask > 0] + [120, 0, 0]).astype(np.uint8)
    cv2.imwrite(f"wrist_occluder_{a.arm}_preview.jpg", preview)
    print(f"{path}: {int(mask.sum())} px ({100.0 * mask.mean():.0f}% of the image); check wrist_occluder_{a.arm}_preview.jpg")


if __name__ == "__main__":
    main()
