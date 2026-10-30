#!/usr/bin/env python3
"""Measure the head depth camera's scale against the 7x5 ChArUco board.

  python3 tools/head_record.py --depth --seconds 60        # on the robot: hold the board flat and still at 35-80 cm
  python3 tools/depth_scale.py ~/head_calib/calib_<time>    # anywhere: fit

For every colour/depth pair, the board's pose from the colour image (camera intrinsics from the factory calibration the
driver publishes, see --intrinsics) gives the true distance of every pixel on the board; the registered depth image gives
what the depth camera reports there. The result is the ratio depth / true per distance band and overall. A constant ratio
means a scale error: set depth_scale = 1 / ratio in src/xlerobot_object_perception/config/object_perception.yaml.
"""
import argparse
import glob
import json
import os
import sys

import cv2
import cv2.aruco as aruco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import side_cam  # noqa: E402

# The Gemini 2 colour camera at 1280x720 as /cameras/head/camera_info reports it (rational polynomial model). A ChArUco
# lens calibration of this camera agreed with it (focal length 690.4 +- 3 px against 690.8).
GEMINI2_COLOR = {"K": [[690.8104248046875, 0, 638.4581298828125], [0, 690.4276123046875, 360.6810302734375], [0, 0, 1]],
                 "D": [53.997039794921875, -110.29363250732422, -0.000273539568297565, 0.00021029447088949382,
                       123.43366241455078, 53.815738677978516, -109.88721466064453, 123.76702117919922]}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("--intrinsics", help="JSON file with K and D of the colour camera (default: the Gemini 2's)")
    ap.add_argument("--min-corners", type=int, default=15)
    a = ap.parse_args()
    cam = json.load(open(a.intrinsics)) if a.intrinsics else GEMINI2_COLOR
    K, D = np.array(cam["K"], float), np.array(cam["D"], float)
    board = side_cam.make_board("7x5")
    b = side_cam.BOARDS["7x5"]
    W, H = b["squares_x"] * b["square_length"], b["squares_y"] * b["square_length"]
    dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    rows = []
    for f in sorted(glob.glob(os.path.join(a.folder, "[0-9]*.png"))):
        df = os.path.join(a.folder, "depth_" + os.path.basename(f))
        if not os.path.exists(df):
            continue
        r = side_cam.charuco_frame(cv2.imread(f), dic, board)
        if r is None or len(r[1]) < a.min_corners:
            continue
        cc, ci = r[:2]
        obj = np.array(board.chessboardCorners)[ci.ravel()].astype(np.float32)
        _, rv, tv = cv2.solvePnP(obj, cc, K, D, flags=cv2.SOLVEPNP_IPPE)
        rv, tv = cv2.solvePnPRefineLM(obj, cc, K, D, rv, tv)
        n, p0 = cv2.Rodrigues(rv)[0][:, 2], tv.ravel()
        m = 0.01                                                     # stay 1 cm inside the board's edge
        q, _ = cv2.projectPoints(np.array([[m, m, 0], [W - m, m, 0], [W - m, H - m, 0], [m, H - m, 0]], np.float32), rv, tv, K, D)
        depth = cv2.imread(df, cv2.IMREAD_UNCHANGED).astype(np.float64) / 1000.0
        mask = np.zeros(depth.shape, np.uint8)
        cv2.fillConvexPoly(mask, q.reshape(-1, 2).astype(np.int32), 1)
        ys, xs = np.nonzero((mask > 0) & (depth > 0.1))
        if len(xs) < 500:
            continue
        und = cv2.undistortPoints(np.stack([xs, ys], 1).reshape(-1, 1, 2).astype(np.float64), K, D).reshape(-1, 2)
        z_true = (p0 @ n) / (np.c_[und, np.ones(len(und))] @ n)
        ratio = depth[ys, xs] / z_true
        rows.append((float(np.median(z_true)), float(np.median(ratio)), float(np.std(ratio))))
    if len(rows) < 5:
        sys.exit(f"only {len(rows)} usable frames (board with {a.min_corners}+ corners and a matching depth frame)")
    r = np.array(rows)
    print(f"{len(r)} frames, board at {r[:, 0].min():.2f}-{r[:, 0].max():.2f} m")
    print("distance   frames  depth/true  spread")
    for lo in np.arange(0.3, 1.2, 0.1):
        s = r[(r[:, 0] >= lo) & (r[:, 0] < lo + 0.1)]
        if len(s):
            print(f"{lo:.1f}-{lo + 0.1:.1f} m  {len(s):6d}  {np.median(s[:, 1]):.4f}      {s[:, 1].std():.4f}")
    ratio = float(np.median(r[:, 1]))
    k, c = np.linalg.lstsq(np.c_[r[:, 0], np.ones(len(r))], r[:, 0] * (r[:, 1] - 1), rcond=None)[0]
    print(f"overall depth/true {ratio:.4f}; as a line, depth - true = {k * 100:.2f}% of the distance {c * 1000:+.1f} mm")
    print(f"depth_scale: {1.0 / ratio:.4f}")


if __name__ == "__main__":
    main()
