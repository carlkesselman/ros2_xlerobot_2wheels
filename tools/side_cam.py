#!/usr/bin/env python3
"""The side (measurement) camera on the laptop: a plain USB webcam on /dev/videoN, used to read claw tip
heights and gaps off the image and as ground truth for the perception tests (ADR-0005).

  python3 tools/side_cam.py snap [--out DIR]        save one frame (exposure, focus and white balance locked)
  python3 tools/side_cam.py calibrate               lens calibration with the 7x5 ChArUco board; hold the board
                                                    in front of the camera at many angles, Space takes a view,
                                                    q finishes and fits
  python3 tools/side_cam.py info                    what the camera reports

Frames are 1920x1080 MJPG. Exposure, focus and white balance are switched to manual at the values the
camera had when it opened, so frames taken minutes apart are comparable. The lens calibration is written to
~/.ros/side_cam.yaml. Runs on the laptop, not on the Jetson.
"""
import argparse
import os
import pathlib
import select
import sys
import termios
import time
import tty

_venv = os.path.expanduser("~/venvs/xlerobot-mcp/bin/python")
if os.path.exists(_venv) and os.path.realpath(sys.prefix) != os.path.realpath(os.path.dirname(os.path.dirname(_venv))):
    os.execv(_venv, [_venv] + sys.argv)

import cv2  # noqa: E402
import cv2.aruco as aruco  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

CALIB = pathlib.Path.home() / ".ros" / "side_cam.yaml"
BOARD = dict(squares_x=7, squares_y=5, square_length=0.03386, marker_length=0.02562)   # the board printed for the arm calibration


def open_camera(dev, lock=True):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f"cannot open /dev/video{dev}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    for _ in range(20):                        # let the automatic controls settle before they are locked
        cap.read()
    if lock:
        exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # V4L2: 1 is manual
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        for _ in range(5):
            cap.read()
    return cap


def grab(cap, n=3):
    frame = None
    for _ in range(n):
        ok, f = cap.read()
        if ok:
            frame = f
    if frame is None:
        sys.exit("no frame from the camera")
    return frame


def cmd_info(a):
    cap = open_camera(a.dev, lock=False)
    f = grab(cap)
    print(f"/dev/video{a.dev}: {f.shape[1]}x{f.shape[0]}, fps {cap.get(cv2.CAP_PROP_FPS):.0f}")
    for name in ("AUTO_EXPOSURE", "EXPOSURE", "AUTOFOCUS", "FOCUS", "AUTO_WB", "WB_TEMPERATURE", "BRIGHTNESS", "GAIN"):
        print(f"  {name.lower():<15}{cap.get(getattr(cv2, 'CAP_PROP_' + name))}")
    print("lens calibration:", CALIB if CALIB.exists() else "none yet")


def cmd_snap(a):
    cap = open_camera(a.dev)
    f = grab(cap)
    out = pathlib.Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    path = out / time.strftime("side_%Y%m%d_%H%M%S.png")
    cv2.imwrite(str(path), f)
    print(path)


def cmd_calibrate(a):
    dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    board = aruco.CharucoBoard_create(BOARD["squares_x"], BOARD["squares_y"], BOARD["square_length"],
                                      BOARD["marker_length"], dic)
    cap = open_camera(a.dev)
    views, size = [], None
    if not sys.stdin.isatty():
        sys.exit("needs a terminal")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("hold the board in view at many angles and distances (also tilted, also near the corners); "
          "Space takes a view, q finishes")
    try:
        while True:
            f = grab(cap, 1)
            size = f.shape[1::-1]
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            mc, mi, _ = aruco.detectMarkers(g, dic)
            n = 0
            if mi is not None and len(mi) > 0:
                n, cc, ci = aruco.interpolateCornersCharuco(mc, mi, g, board)
                n = 0 if n is None else n
            print(f"\rcorners in view: {n:3d}  views taken: {len(views):2d}   ", end="", flush=True)
            if select.select([sys.stdin], [], [], 0.15)[0]:
                k = sys.stdin.read(1)
                if k in ("q", "Q"):
                    break
                if k == " " and n >= 12:
                    views.append((cc, ci))
                    print(f"\nview {len(views)} saved ({n} corners)")
                elif k == " ":
                    print("\nnot enough corners in view, move the board closer or turn it towards the camera")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    if len(views) < 8:
        sys.exit(f"\nneed at least 8 views, got {len(views)}")
    # Start from a plain webcam guess and keep the model simple (square pixels, no third distortion term):
    # with free parameters and a few poor views the fit runs away to nonsense focal lengths.
    f0 = 0.62 * size[0]
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST)
    K0 = np.array([[f0, 0, size[0] / 2], [0, f0, size[1] / 2], [0, 0, 1.0]])
    keep = list(range(len(views)))
    while True:
        rms, K, D, rvecs, tvecs = aruco.calibrateCameraCharuco(
            [views[i][0] for i in keep], [views[i][1] for i in keep], board, size, K0.copy(), np.zeros(5), flags=flags)
        errs = []
        for i, rv, tv in zip(keep, rvecs, tvecs):
            obj = board.chessboardCorners[views[i][1].ravel()]
            pr, _ = cv2.projectPoints(obj, rv, tv, K, D)
            errs.append(float(np.sqrt(((pr.reshape(-1, 2) - views[i][0].reshape(-1, 2)) ** 2).sum(1).mean())))
        worst = int(np.argmax(errs))
        if errs[worst] < max(1.0, 2.5 * rms) or len(keep) <= 8:
            break
        print(f"dropping view {keep[worst] + 1} (error {errs[worst]:.1f} px)")
        del keep[worst]
    print("per-view error (px):", [round(e, 2) for e in errs])
    views = [views[i] for i in keep]
    print(f"\nfx {K[0, 0]:.1f} fy {K[1, 1]:.1f} cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}, reprojection error {rms:.2f} px "
          f"over {len(views)} views, saved to {CALIB}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["snap", "calibrate", "info"])
    ap.add_argument("--dev", type=int, default=0, help="N of /dev/videoN")
    ap.add_argument("--out", default="~/side_cam")
    a = ap.parse_args()
    {"snap": cmd_snap, "calibrate": cmd_calibrate, "info": cmd_info}[a.command](a)


if __name__ == "__main__":
    main()
