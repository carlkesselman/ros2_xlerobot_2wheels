#!/usr/bin/env python3
"""The side (measurement) camera on the laptop: a plain USB webcam on /dev/videoN, used to read claw tip
heights and gaps off the image and as ground truth for the perception tests (ADR-0005).

  python3 tools/side_cam.py snap [--out DIR]        save one frame (exposure, focus and white balance locked)
  python3 tools/side_cam.py calibrate               lens calibration with the 7x5 ChArUco board; hold the board
                                                    in front of the camera at many angles, Space takes a view,
                                                    q finishes and fits
  python3 tools/side_cam.py record [--seconds 30]   save a burst of frames while you move the board (no key presses)
  python3 tools/side_cam.py calibrate --from DIR    fit the lens from recorded frames (sharp, varied views only)
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
# The two ChArUco boards in use (DICT_4X4_250): the 7x5 printed for the arm calibration (square measured on the
# print) and the older 11x8 one. Which one is in front of the camera is told from the marker IDs it shows.
BOARDS = {"7x5": dict(squares_x=7, squares_y=5, square_length=0.03386, marker_length=0.02562),
          "11x8": dict(squares_x=11, squares_y=8, square_length=0.015, marker_length=0.011)}
BOARD = BOARDS["7x5"]


def make_board(name):
    b = BOARDS[name]
    return aruco.CharucoBoard_create(b["squares_x"], b["squares_y"], b["square_length"], b["marker_length"],
                                     aruco.getPredefinedDictionary(aruco.DICT_4X4_250))


def pick_board(frames):
    """7x5 if no marker ID above 16 shows up, otherwise 11x8."""
    dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    top = 0
    for f in frames:
        _, mi, _ = aruco.detectMarkers(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), dic)
        if mi is not None and len(mi):
            top = max(top, int(mi.max()))
    return "7x5" if top <= 16 else "11x8"


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


def cmd_record(a):
    cap = open_camera(a.dev)
    out = pathlib.Path(a.out).expanduser() / time.strftime("calib_%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    print(f"recording {a.seconds:g} s to {out}: move the board slowly through many angles and distances, hold each pose a moment")
    t0, n = time.time(), 0
    while time.time() - t0 < a.seconds:
        f = grab(cap, 1)
        cv2.imwrite(str(out / f"{n:04d}.jpg"), f, [cv2.IMWRITE_JPEG_QUALITY, 95])
        n += 1
        time.sleep(0.25)
    print(f"{n} frames in {out}")


def charuco_frame(f, dic, board):
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    mc, mi, _ = aruco.detectMarkers(g, dic)
    if mi is None or len(mi) == 0:
        return None
    n, cc, ci = aruco.interpolateCornersCharuco(mc, mi, g, board)
    if n is None or n < 12:
        return None
    sharp = cv2.Laplacian(g, cv2.CV_64F).var()
    return cc, ci, sharp


def cmd_calibrate_offline(a):
    dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    files = sorted(pathlib.Path(a.from_dir).expanduser().glob("*.jpg"))
    global BOARD
    bname = pick_board([cv2.imread(str(fp)) for fp in files[::max(1, len(files) // 25)]])
    BOARD = BOARDS[bname]
    board = make_board(bname)
    print(f"board in the frames: {bname} ({BOARD['squares_x']}x{BOARD['squares_y']}, square {BOARD['square_length'] * 1000:g} mm)")
    found = []
    for fp in files:
        f = cv2.imread(str(fp))
        r = charuco_frame(f, dic, board)
        if r:
            found.append((fp.name, *r))
    print(f"{len(found)} of {len(files)} frames have the board with 12 or more corners")
    if len(found) < 8:
        sys.exit("not enough usable frames")
    size = (f.shape[1], f.shape[0])
    sharp_min = np.percentile([x[3] for x in found], 30)
    found = [x for x in found if x[3] >= sharp_min]
    # pick varied views: farthest-point sampling on (centre x, centre y, spread)
    feats = np.array([[x[1].reshape(-1, 2)[:, 0].mean() / size[0], x[1].reshape(-1, 2)[:, 1].mean() / size[1],
                       np.ptp(x[1].reshape(-1, 2), axis=0).mean() / size[0], len(x[2]) / 24.0] for x in found])
    order = [0]
    while len(order) < min(a.views, len(found)):
        d = np.min(np.linalg.norm(feats[:, None] - feats[order][None], axis=2), axis=1)
        order.append(int(np.argmax(d)))
    views = [(found[i][1], found[i][2]) for i in order]
    print(f"using {len(views)} varied sharp views")
    best = None
    for name, flags in (("simple", cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST),
                        ("with k3", cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST),
                        ("rational", cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_ASPECT_RATIO)):
        for f0 in (0.45 * size[0], 0.62 * size[0], 0.9 * size[0]):
            K0 = np.array([[f0, 0, size[0] / 2], [0, f0, size[1] / 2], [0, 0, 1.0]])
            try:
                rms, K, D, rv, tv = aruco.calibrateCameraCharuco([v[0] for v in views], [v[1] for v in views], board, size,
                                                                 K0, np.zeros(8 if "rational" in name else 5), flags=flags)
            except cv2.error as e:
                print(f"  {name}, f0 {f0:.0f}: failed ({e.code})")
                continue
            print(f"  {name}, start f {f0:.0f}: rms {rms:.2f} px, fx {K[0, 0]:.0f}, cx {K[0, 2]:.0f}, cy {K[1, 2]:.0f}")
            if best is None or rms < best[0]:
                best = (rms, K, D, name)
    rms, K, D, name = best
    CALIB.parent.mkdir(parents=True, exist_ok=True)
    CALIB.write_text(yaml.safe_dump({"image_size": list(size), "camera_matrix": K.tolist(), "distortion": D.ravel().tolist(),
                                     "rms_px": float(rms), "views": len(views), "model": name, "board": BOARD}))
    print(f"best: {name}, rms {rms:.2f} px, fx {K[0, 0]:.1f}, cx {K[0, 2]:.1f}, cy {K[1, 2]:.1f}, saved to {CALIB}")


def cmd_calibrate(a):
    if a.from_dir:
        return cmd_calibrate_offline(a)
    # Interactive: every frame taken with Space is saved, then the same offline fit runs on the folder, so
    # a bad fit can be repeated or tuned later without taking the views again.
    dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    cap = open_camera(a.dev)
    out = pathlib.Path(a.out).expanduser() / time.strftime("calib_%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    if not sys.stdin.isatty():
        sys.exit("needs a terminal (or use: record, then calibrate --from DIR)")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    taken = 0
    print(f"hold the board in view at many angles and distances; Space saves a frame to {out}, q finishes and fits")
    try:
        while True:
            f = grab(cap, 1)
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            _, mi, _ = aruco.detectMarkers(g, dic)
            print(f"\rmarkers in view: {0 if mi is None else len(mi):3d}  frames saved: {taken:2d}   ", end="", flush=True)
            if select.select([sys.stdin], [], [], 0.15)[0]:
                k = sys.stdin.read(1)
                if k in ("q", "Q"):
                    break
                if k == " ":
                    cv2.imwrite(str(out / f"{taken:04d}.jpg"), f, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    taken += 1
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print()
    a.from_dir = str(out)
    return cmd_calibrate_offline(a)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["snap", "calibrate", "record", "info"])
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--from", dest="from_dir", help="directory of recorded frames for calibrate")
    ap.add_argument("--views", type=int, default=30)
    ap.add_argument("--dev", type=int, default=0, help="N of /dev/videoN")
    ap.add_argument("--out", default="~/side_cam")
    a = ap.parse_args()
    {"snap": cmd_snap, "calibrate": cmd_calibrate, "record": cmd_record, "info": cmd_info}[a.command](a)


if __name__ == "__main__":
    main()
