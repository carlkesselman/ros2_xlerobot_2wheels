#!/usr/bin/env python3
"""Arm / wrist-camera calibration against one fixed ChArUco board.

  python3 tools/calibrate_arm_camera.py left            # you press Space for each sample
  python3 tools/calibrate_arm_camera.py left --auto     # takes a sample whenever the board is steady

The head camera measures the board's pose once. The board then stays put while you hold the arm
(torque off) so its wrist camera sees the board from many different poses. In the default mode you
hold the arm at a pose and press Space (or Enter): it waits for a fresh frame, checks the arm was
still while that frame was taken and that the board is detected, and saves it with the joint angles.
Keys: Space/Enter take a sample, u drop the last one, q finish and fit. Every saved view has the joint
angles from the moment the image was captured (wrist frames arrive about 0.6 s late), and a least-squares fit finds the five joint zero
offsets and the wrist camera's real mounting pose that make all views agree with the head's
measurement (it minimises the corner reprojection error, in pixels).

Needs: stack running with cameras and the arm, the board flat and rigid and not moved during the
run, visible to the head camera at the start. The head camera's pose is trusted (IMU and board
checked), so the arm and its camera come out in the head camera's frame.

Torque on the selected arm's servo bus is cut for the capture (bus1 also holds the head, so it droops,
and bus2 the wheels, so the base can roll) and restored at the end, also on Ctrl-C. Hold the arm.

Board defaults match tools/calibrate_camera_charuco.py (11x8, 15 mm squares, 11 mm markers,
DICT_4X4_250). The wrist camera sees 11 mm markers only within about 20 cm.
"""
import argparse
import atexit
import json
import math
import select
import pathlib
import signal
import sys
import termios
import threading
import time
import tty
import xml.etree.ElementTree as ET

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, CompressedImage, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import motion_recorder  # noqa: E402  (torque cut/restore helper)

ARM_BUS = {"left": "xlerobot_bus1", "right": "xlerobot_bus2"}   # bus1 also has the head, bus2 the wheels
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
OUT_DIR = pathlib.Path.home() / ".ros" / "arm_camera_calibration"


def rpy_matrix(r, p, y):
    return R.from_euler("xyz", [r, p, y]).as_matrix()


def transform(rot, trans):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = rot, trans
    return m


class Chain:
    """Forward kinematics base_footprint -> a frame, from the live robot_description."""

    def __init__(self, urdf, target):
        root = ET.fromstring(urdf)
        parent_of = {}
        for j in root.findall("joint"):
            parent_of[j.find("child").get("link")] = j
        path, link = [], target
        while link in parent_of:
            path.append(parent_of[link])
            link = parent_of[link].find("parent").get("link")
        self.joints = []
        for j in reversed(path):
            o = j.find("origin")
            xyz = [float(v) for v in o.get("xyz", "0 0 0").split()] if o is not None else [0, 0, 0]
            rpy = [float(v) for v in o.get("rpy", "0 0 0").split()] if o is not None else [0, 0, 0]
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None
            self.joints.append((j.get("name"), j.get("type"), transform(rpy_matrix(*rpy), xyz), axis))
        self.target = target

    def fk(self, q):
        m = np.eye(4)
        for name, kind, origin, axis in self.joints:
            m = m @ origin
            if kind in ("revolute", "continuous") and name in q:
                m = m @ transform(R.from_rotvec(axis * q[name]).as_matrix(), [0, 0, 0])
        return m


class Capture:
    def __init__(self, arm):
        rclpy.init()
        self.node = rclpy.create_node("calibrate_arm_camera")
        self.arm = arm
        self.q = {}
        self.history = []
        self.frames = {}
        self.info = {}
        self.urdf = None
        self.buf = Buffer()
        TransformListener(self.buf, self.node, spin_thread=False)
        n = self.node
        n.create_subscription(JointState, "/joint_states", self._joints, 10)
        n.create_subscription(String, "/robot_description", self._urdf,
                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                         reliability=ReliabilityPolicy.RELIABLE))
        # the Gemini publishes its colour stream under color/, the wrist cameras directly
        for cam, topic in (("head", "/cameras/head/color/image_raw/compressed"),
                           ("head", "/cameras/head/image_raw/compressed"),
                           (arm, f"/cameras/{arm}/image_raw/compressed")):
            n.create_subscription(CompressedImage, topic, lambda m, c=cam: self._frame(c, m), qos_profile_sensor_data)
        for cam in ("head", arm):
            n.create_subscription(CameraInfo, f"/cameras/{cam}/camera_info", lambda m, c=cam: self.info.__setitem__(c, m),
                                  qos_profile_sensor_data)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(n)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def _joints(self, m):
        self.q.update(zip(m.name, m.position))
        self.history.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, dict(self.q)))
        del self.history[:-400]

    def _urdf(self, m):
        self.urdf = m.data

    def _frame(self, cam, m):
        self.frames[cam] = (time.monotonic(), m)

    def wait_for(self, what, timeout=15.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if what():
                return True
            time.sleep(0.1)
        return False

    def joints_at(self, stamp, before=0.5, after=0.2, eps=0.007):
        """Mean joint angles around an image's capture time, None unless the arm was still then.
        The wrist camera's frames arrive about 0.6 s after they were taken, so what the arm is
        doing now says nothing about what the image shows."""
        rows = [h for h in self.history if stamp - before <= h[0] <= stamp + after]
        if len(rows) < 5 or self.history[-1][0] < stamp + after:
            return None
        names = [f"{self.arm}_arm_{j}_joint" for j in JOINTS]
        vals = np.array([[r[1][n] for n in names] for r in rows])
        if (vals.max(0) - vals.min(0)).max() > eps:
            return None
        return vals.mean(0)


def make_board(a):
    dic = aruco.getPredefinedDictionary(getattr(aruco, a.dict))
    return aruco.CharucoBoard_create(a.squares_x, a.squares_y, a.square_length, a.marker_length, dic), dic


def detect(msg, board, dic, K, D):
    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
    params = aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    mc, mi, _ = aruco.detectMarkers(img, dic, parameters=params)
    if mi is None or len(mi) == 0:
        return None
    n, cc, ci = aruco.interpolateCornersCharuco(mc, mi, img, board)
    if n is None or n < 8:
        return None
    return ci.flatten(), cc.reshape(-1, 2)


def board_pose(ids, px, board, K, D, min_inliers=8):
    obj = board.chessboardCorners[ids].astype(np.float32)
    ok, rv, tv, inl = cv2.solvePnPRansac(obj, px.astype(np.float32), K, D, reprojectionError=3.0,
                                         iterationsCount=500, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inl is None or len(inl) < min_inliers:
        return None
    i = inl.ravel()
    ok, rv, tv = cv2.solvePnP(obj[i], px[i].astype(np.float32), K, D, rv, tv, True)
    proj, _ = cv2.projectPoints(obj[i], rv, tv, K, D)
    rms = float(np.sqrt(((proj.reshape(-1, 2) - px[i]) ** 2).sum(1).mean()))
    return transform(cv2.Rodrigues(rv)[0], tv.ravel()), rms, len(i)


def camera_matrices(info):
    return np.array(info.k).reshape(3, 3), np.array(info.d)


def head_board_in_base(cap, board, dic):
    frame_id = cap.info["head"].header.frame_id
    K, D = camera_matrices(cap.info["head"])
    poses = []
    for _ in range(15):
        t, m = cap.frames["head"]
        det = detect(m, board, dic, K, D)
        if det is not None:
            p = board_pose(det[0], det[1], board, K, D, min_inliers=6)
            if p is not None and p[1] < 1.5:
                poses.append(p[0])
        time.sleep(0.2)
    if len(poses) < 4:
        sys.exit("head camera does not see the board well enough (need it in view, well lit)")
    rots = R.from_matrix([p[:3, :3] for p in poses]).mean().as_matrix()
    trans = np.mean([p[:3, 3] for p in poses], axis=0)
    spread = np.std([p[:3, 3] for p in poses], axis=0) * 1000
    tilt = [np.degrees(np.arccos(np.clip(p[:3, 2] @ rots[:, 2], -1, 1))) for p in poses]
    tf = cap.buf.lookup_transform("base_footprint", frame_id, rclpy.time.Time())
    q, p = tf.transform.rotation, tf.transform.translation
    T_base_head = transform(R.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [p.x, p.y, p.z])
    T = T_base_head @ transform(rots, trans)
    print(f"board from head camera: {len(poses)} views, position scatter {spread.round(1)} mm, "
          f"normal scatter {np.std(tilt):.2f} deg, "
          f"origin in base {T[:3, 3].round(3)}, normal {T[:3, 2].round(3)}")
    return T


def collect(cap, a, board, dic, T_bb):
    K, D = camera_matrices(cap.info[a.arm])
    samples, last_msg, last_stamp = [], 0.0, 0.0
    names = [f"{a.arm}_arm_{j}_joint" for j in JOINTS]
    print("move the arm so its camera sees the board, hold it still, then move on. Ctrl-C finishes.")
    while len(samples) < a.samples:
        try:
            time.sleep(0.25)
        except KeyboardInterrupt:
            break
        if a.arm not in cap.frames:
            continue
        t, m = cap.frames[a.arm]
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if stamp <= last_stamp or time.monotonic() - t > 0.5:
            continue
        q = cap.joints_at(stamp, eps=a.still_eps)
        if q is None:
            continue
        last_stamp = stamp
        det = detect(m, board, dic, K, D)
        if det is None or len(det[0]) < a.min_corners:
            if time.time() - last_msg > 2:
                print("  board not seen well enough")
                last_msg = time.time()
            continue
        pose = board_pose(det[0], det[1], board, K, D)
        if pose is None or pose[1] > a.max_rms:
            continue
        if samples and min(np.abs(q - np.array(s["q"])).max() for s in samples) < a.min_move:
            if time.time() - last_msg > 2:
                print("  too close to an earlier pose, move further")
                last_msg = time.time()
            continue
        samples.append({"q": q.tolist(), "ids": det[0].tolist(), "px": det[1].tolist()})
        dist = float(np.linalg.norm(pose[0][:3, 3]))
        print(f"  sample {len(samples)}/{a.samples}: {len(det[0])} corners, board {dist * 100:.0f} cm away, "
              f"pnp rms {pose[1]:.2f} px")
    return samples



class Keys:
    """Single key presses from the terminal, without waiting for Enter."""

    def __enter__(self):
        if not sys.stdin.isatty():
            sys.exit("manual mode needs a terminal, run it yourself (or use --auto)")
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get(self, timeout):
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None


def collect_manual(cap, a, board, dic):
    K, D = camera_matrices(cap.info[a.arm])
    samples = []
    print("hold the arm so its camera sees the board, keep it STILL, press Space (or Enter) to take a sample.")
    print("u = drop the last sample, q = finish and fit. Need at least 6, 12 or more is better.\n")
    with Keys() as keys:
        while True:
            try:
                key = keys.get(0.5)
            except KeyboardInterrupt:
                break
            if key in ("q", "Q", "\x03"):
                break
            if key in ("u", "U") and samples:
                samples.pop()
                print(f"\ndropped, {len(samples)} samples")
                continue
            if key not in (" ", "\n", "\r"):
                if a.arm in cap.frames:
                    det = detect(cap.frames[a.arm][1], board, dic, K, D)
                    n = 0 if det is None else len(det[0])
                    print(f"\rboard corners in view: {n:3d}  samples: {len(samples):2d}   (live view is ~0.6 s old)   ",
                          end="", flush=True)
                continue
            # a sample: wait for a frame taken after the key press, with the arm still around it
            t_press = cap.node.get_clock().now().nanoseconds * 1e-9
            print("\n  hold still...", end="", flush=True)
            result, deadline = None, time.time() + 6.0
            while time.time() < deadline and result is None:
                time.sleep(0.1)
                t, m = cap.frames.get(a.arm, (0, None))
                if m is None:
                    continue
                stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                if stamp < t_press + 0.1:
                    continue
                q = cap.joints_at(stamp, before=0.3, eps=a.still_eps)
                if q is None:
                    if cap.history and cap.history[-1][0] >= stamp + 0.2:
                        result = ("the arm moved around the frame, hold it still", None)
                    continue
                det = detect(m, board, dic, K, D)
                if det is None or len(det[0]) < a.min_corners:
                    result = (f"only {0 if det is None else len(det[0])} corners seen (need {a.min_corners})", None)
                    break
                pose = board_pose(det[0], det[1], board, K, D)
                if pose is None or pose[1] > a.max_rms:
                    result = ("board pose unreliable (blur, glare or too oblique)", None)
                    break
                result = ("ok", {"q": q.tolist(), "ids": det[0].tolist(), "px": det[1].tolist(),
                                 "_info": f"{len(det[0])} corners, board {100 * np.linalg.norm(pose[0][:3, 3]):.0f} cm, "
                                          f"rms {pose[1]:.2f} px"})
            if result is None:
                print(" no new frame, is the camera running?")
            elif result[1] is None:
                print(f" NOT saved: {result[0]}")
            else:
                info = result[1].pop("_info")
                near = samples and min(np.abs(np.array(result[1]["q"]) - np.array(s["q"])).max() for s in samples)
                samples.append(result[1])
                print(f" saved sample {len(samples)}: {info}"
                      + (" (similar to an earlier pose, vary it more)" if samples[:-1] and near < a.min_move else ""))
    print()
    return samples


def fit(cap, a, samples, board, T_bb):
    chain = Chain(cap.urdf, f"{a.arm}_arm_wrist_camera_optical_frame")
    K, D = camera_matrices(cap.info[a.arm])
    names = [f"{a.arm}_arm_{j}_joint" for j in JOINTS]
    obj = [board.chessboardCorners[np.array(s["ids"])].astype(float) for s in samples]
    px = [np.array(s["px"]) for s in samples]

    def residuals(x):
        out = []
        dX = transform(R.from_rotvec(x[5:8]).as_matrix(), x[8:11])
        for s, o, p in zip(samples, obj, px):
            q = {n: s["q"][i] + x[i] for i, n in enumerate(names)}
            T_cam = chain.fk(q) @ dX
            T_cb = np.linalg.inv(T_cam) @ T_bb
            pc = (T_cb[:3, :3] @ o.T).T + T_cb[:3, 3]
            if (pc[:, 2] <= 0.01).any():
                out.append(np.full(len(o) * 2, 100.0))
                continue
            proj, _ = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
            out.append((proj.reshape(-1, 2) - p).ravel())
        return np.concatenate(out)

    x0 = np.zeros(11)
    r0 = residuals(x0)
    rms0 = math.sqrt(np.mean(r0 ** 2) * 2)
    sol = least_squares(residuals, x0, loss="huber", f_scale=2.0, x_scale=[0.05] * 5 + [0.05] * 3 + [0.01] * 3)
    r = residuals(sol.x)
    rms = math.sqrt(np.mean(r ** 2) * 2)
    dof = max(1, len(r) - 11)
    cov = np.linalg.pinv(sol.jac.T @ sol.jac) * (np.sum(r ** 2) / dof)
    sd = np.sqrt(np.diag(cov))
    print(f"\nreprojection error per corner: {rms0:.1f} px before, {rms:.1f} px after the fit "
          f"({len(samples)} samples, {len(r) // 2} corners)")
    print("joint zero offsets (add to the reported angle to get the true angle), +/- 1 sigma:")
    for i, n in enumerate(JOINTS):
        print(f"  {n:<14} {math.degrees(sol.x[i]):+6.2f} deg  +/- {math.degrees(sd[i]):.2f}")
    rv = sol.x[5:8]
    print("camera mount correction (applied after the URDF camera frame): "
          f"rotation {np.degrees(rv).round(2)} deg +/- {np.degrees(sd[5:8]).round(2)}, "
          f"translation {(sol.x[8:11] * 1000).round(1)} mm +/- {(sd[8:11] * 1000).round(1)}")
    per = []
    s0 = 0
    for s, o in zip(samples, obj):
        n = len(o) * 2
        per.append(math.sqrt(np.mean(r[s0:s0 + n] ** 2) * 2))
        s0 += n
    print("per-sample rms after fit (px):", np.round(per, 1).tolist())
    return {"x": sol.x.tolist(), "sd": sd.tolist(), "rms_px": rms, "rms_before_px": rms0}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("arm", choices=["left", "right"])
    ap.add_argument("--samples", type=int, default=12)
    ap.add_argument("--min-corners", type=int, default=12)
    ap.add_argument("--still-eps", type=float, default=0.007, help="rad, largest joint movement allowed around a frame")
    ap.add_argument("--max-rms", type=float, default=1.5, help="px, PnP reprojection error a view may have")
    ap.add_argument("--min-move", type=float, default=0.12, help="rad, how far a new pose must differ from every saved one")
    ap.add_argument("--squares-x", type=int, default=11)
    ap.add_argument("--squares-y", type=int, default=8)
    ap.add_argument("--square-length", type=float, default=0.015)
    ap.add_argument("--marker-length", type=float, default=0.011)
    ap.add_argument("--dict", default="DICT_4X4_250")
    ap.add_argument("--fit-only", type=pathlib.Path, nargs="+",
                    help="refit saved capture file(s) from the same board position, no robot motion")
    ap.add_argument("--no-torque-cut", action="store_true", help="the arm is moved some other way")
    ap.add_argument("--auto", action="store_true", help="take samples automatically instead of on a key press")
    ap.add_argument("--yes", action="store_true", help="no Enter prompt, 10 s countdown before torque is cut")
    a = ap.parse_args()
    # a background launch ignores SIGINT, and SIGTERM would skip the torque restore
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(1))

    board, dic = make_board(a)
    cap = Capture(a.arm)
    if not cap.wait_for(lambda: cap.urdf and a.arm in cap.info and "head" in cap.info and cap.q):
        sys.exit("no robot_description / camera_info / joint_states, is the stack running with cameras?")
    time.sleep(1.0)

    if a.fit_only:
        runs = [json.loads(f.read_text()) for f in a.fit_only]
        T_bb = np.array(runs[0]["T_base_board"])
        samples = []
        for f, d in zip(a.fit_only, runs):
            moved = np.linalg.norm(np.array(d["T_base_board"])[:3, 3] - T_bb[:3, 3]) * 1000
            if moved > 5:
                print(f"warning: {f.name} measured the board {moved:.0f} mm from the first file, did it move?")
            samples += d["samples"]
        print(f"{len(samples)} samples from {len(runs)} file(s)")
    else:
        if not cap.wait_for(lambda: "head" in cap.frames and a.arm in cap.frames, timeout=40):
            sys.exit("no camera frames")
        T_bb = head_board_in_base(cap, board, dic)
        torque_cut = [False]
        bus = ARM_BUS[a.arm]

        def restore():
            if torque_cut[0]:
                err = motion_recorder.set_torque_cut(cap.node, False, buses=[bus])
                print("torque restored" if not err else f"TORQUE NOT RESTORED ({err}), run tools/estop.py --release")
                torque_cut[0] = False
        atexit.register(restore)
        if not a.no_torque_cut:
            extra = "the head" if a.arm == "left" else "the wheels"
            print(f"\nTorque will be cut on the {a.arm} arm's servo bus ({bus}): the arm and {extra} go limp. "
                  "HOLD THE ARM.")
            if a.yes:
                for sec in range(10, 0, -1):
                    print(f"  cutting torque in {sec} s", flush=True)
                    time.sleep(1)
            else:
                input("press Enter when you hold it... ")
            err = motion_recorder.set_torque_cut(cap.node, True, buses=[bus])
            if err:
                motion_recorder.set_torque_cut(cap.node, False, buses=[bus])
                sys.exit(f"could not cut torque: {err}")
            torque_cut[0] = True
        try:
            samples = collect(cap, a, board, dic, T_bb) if a.auto else collect_manual(cap, a, board, dic)
        except KeyboardInterrupt:
            samples = []
            print("\nstopped")
        finally:
            restore()
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        path = OUT_DIR / f"{a.arm}_{time.strftime('%Y%m%d_%H%M%S')}.json"
        if samples:
            path.write_text(json.dumps({"arm": a.arm, "T_base_board": T_bb.tolist(), "samples": samples}))
            print(f"saved {len(samples)} samples to {path}")

    if len(samples) < 6:
        sys.exit("need at least 6 samples to fit")
    fit(cap, a, samples, board, T_bb)


if __name__ == "__main__":
    main()
