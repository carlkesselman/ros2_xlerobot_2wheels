#!/usr/bin/env python3
"""On-demand object pose service (ADR-0005). The camera is subscribed only while a request runs; nothing else uses the CPU or GPU."""
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from objperc.cube_fit import fit_cube
from objperc.pipeline import ENGINES, PROMPTS, TrtEvitSam, TrtOwlv2
from xlerobot_object_perception.srv import DetectObjects

DEFAULT_SIZE_MM = 28.0
MERGE_RADIUS_MM = 15.0


def _stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def _mat(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, [t.x, t.y, t.z]
    return T


class ObjectPerception(Node):
    def __init__(self):
        super().__init__("object_perception")
        P = self.declare_parameter
        self.image_topic = P("image_topic", "/cameras/head/image_raw").value
        self.depth_topic = P("depth_topic", "/cameras/head/depth/image_raw").value
        self.info_topic = P("camera_info_topic", "/cameras/head/camera_info").value
        self.target = P("target_frame", "base_footprint").value
        self.default_frames = int(P("frames", 3).value)
        self.default_score = float(P("min_score", 0.15).value)
        self.max_objects = int(P("max_objects", 6).value)
        self.ring_m = float(P("table_search_m", 0.12).value)
        self.engines_dir = P("engines_dir", "").value or ENGINES
        self.prompts_file = P("prompts_file", "").value or PROMPTS
        self.sam_variant = P("sam_variant", "l0").value
        self.depth_scale = float(P("depth_scale", 1.0).value)          # measured with tools/depth_scale.py
        self.prof = {}
        self.det = self.sam = None
        self.table_z = None                                # last good table height, base_footprint, metres
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Detection3DArray, "/perception/detections", qos)
        self._lock = threading.Lock()                      # one request at a time: the GPU and the camera subscriptions are shared
        self._sub_group = MutuallyExclusiveCallbackGroup()
        self.create_service(DetectObjects, "/perception/detect", self._on_detect, callback_group=ReentrantCallbackGroup())
        self.get_logger().info("object_perception ready (models load on the first request)")

    # ---- camera ----
    def _grab(self, n, timeout=5.0):
        """n colour frames at least 0.15 s apart, each with the depth frame closest in time, and the camera info."""
        frames, depths, info = [], [], {}
        last = [0.0]

        def on_img(m):
            t = _stamp(m)
            if t - last[0] < 0.15 or len(frames) >= n or not depths:        # wait for the first depth frame
                return
            last[0] = t
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            frames.append((t, m.header.frame_id, a[:, :, ::-1].copy() if m.encoding == "rgb8" else a.copy()))

        def on_depth(m):
            a = np.frombuffer(m.data, np.uint16 if m.encoding == "16UC1" else np.float32).reshape(m.height, m.width)
            scale = (0.001 if m.encoding == "16UC1" else 1.0) * self.depth_scale
            depths.append((_stamp(m), a.astype(np.float32) * scale))
            del depths[:-8]

        def on_info(m):
            info["K"], info["D"] = np.array(m.k).reshape(3, 3), np.array(m.d, float)

        subs = [self.create_subscription(Image, self.image_topic, on_img, qos_profile_sensor_data, callback_group=self._sub_group),
                self.create_subscription(Image, self.depth_topic, on_depth, qos_profile_sensor_data, callback_group=self._sub_group),
                self.create_subscription(CameraInfo, self.info_topic, on_info, qos_profile_sensor_data, callback_group=self._sub_group)]
        try:
            t0 = time.time()
            while (len(frames) < n or "K" not in info) and time.time() - t0 < timeout:
                time.sleep(0.02)
        finally:
            for s in subs:
                self.destroy_subscription(s)
        out = []
        for t, fid, img in frames:
            d = min(depths, key=lambda x: abs(x[0] - t)) if depths else None
            out.append((t, fid, img, d[1] if d is not None and abs(d[0] - t) < 0.1 else None))
        return out, info

    def _t(self, name, t0):
        self.prof[name] = self.prof.get(name, 0.0) + (time.time() - t0) * 1000.0

    # ---- one frame ----
    def _pixel_ray(self, uv, K, D):
        p = cv2.undistortPoints(np.array([[uv]], np.float64), K, D if np.any(D) else None)[0, 0]
        return np.array([p[0], p[1], 1.0])

    def _points(self, depth, mask, K, D):
        ys, xs = np.nonzero(mask & (depth > 0.1) & (depth < 3.0))
        if len(xs) == 0:
            return np.zeros((0, 3))
        und = cv2.undistortPoints(np.stack([xs, ys], 1).reshape(-1, 1, 2).astype(np.float64), K, D if np.any(D) else None).reshape(-1, 2)
        z = depth[ys, xs].astype(np.float64)
        return np.stack([und[:, 0] * z, und[:, 1] * z, z], 1)

    def _frame(self, stamp, frame_id, img, depth, K, D, thr, size_mm, codes):
        try:
            tf = self.tf_buf.lookup_transform(self.target, frame_id, Time(seconds=int(stamp), nanoseconds=int((stamp % 1) * 1e9)),
                                              timeout=Duration(seconds=0.3))
        except Exception as e:                              # noqa: BLE001
            codes.append("STALE_TF")
            self.get_logger().warn(f"no transform {self.target} <- {frame_id}: {e}")
            return []
        T_base_cam = _mat(tf)
        t0 = time.time()
        boxes = self.det(img, None, thr)
        self._t("detector", t0)
        h, w = img.shape[:2]
        boxes = [b for b in boxes if 0.3 < (b[3] - b[1]) / max(b[4] - b[2], 1) < 3.3 and (b[3] - b[1]) < 0.3 * w][:self.max_objects]
        if not boxes:
            codes.append("NO_DET")
            return []
        out = []
        for score, x0, y0, x1, y1 in boxes:
            t0 = time.time()
            mask = self.sam(img, (x0, y0, x1, y1))
            self._t("mask", t0)
            t0 = time.time()
            if mask.sum() < 80:
                codes.append("NO_MASK")
                continue
            # table height: the depth in a ring around the object, in base_footprint
            m8 = mask.astype(np.uint8)
            r_in, r_out = 8, int(0.12 * (x1 - x0) * 8)
            ring = (cv2.dilate(m8, np.ones((2 * r_out + 1,) * 2, np.uint8)) > 0) & ~(cv2.dilate(m8, np.ones((2 * r_in + 1,) * 2, np.uint8)) > 0)
            z_tab = self.table_z
            if depth is not None:
                pr = self._points(depth, ring, K, D)
                if len(pr) > 200:
                    z_tab = float(np.median((T_base_cam[:3, :3] @ pr.T).T[:, 2] + T_base_cam[2, 3]))
                    self.table_z = z_tab
            if z_tab is None:
                codes.append("FEW_DEPTH")
                continue
            height = None
            if depth is not None:
                pt = self._points(depth, cv2.erode(m8, np.ones((5, 5), np.uint8)) > 0, K, D)
                if len(pt) > 20:
                    height = float(np.median((T_base_cam[:3, :3] @ pt.T).T[:, 2] + T_base_cam[2, 3]) - z_tab)
            # plane frame = base_footprint axes with the origin on the table; millimetres
            T_cam_base = np.linalg.inv(T_base_cam)
            T_cam_plane = T_cam_base.copy()
            T_cam_plane[:3, 3] = T_cam_base[:3, :3] @ [0, 0, z_tab] + T_cam_base[:3, 3]
            T_cam_plane[:3, 3] *= 1000.0
            ray = T_base_cam[:3, :3] @ self._pixel_ray(((x0 + x1) / 2, (y0 + y1) / 2), K, D)
            c = T_base_cam[:3, 3]
            if ray[2] >= -1e-6:
                continue
            guess = c[:2] + ray[:2] * ((z_tab + size_mm / 2000.0 - c[2]) / ray[2])
            self._t("depth", t0)
            t0 = time.time()
            r = fit_cube(mask, T_cam_plane, K, guess * 1000.0, D, size_mm)
            self._t("fit", t0)
            if r is None or r[3] < 0.8:
                codes.append("NO_MASK")
                continue
            x, y, yaw, iou = r
            if height is not None and abs(height - size_mm / 1000.0) > 0.010:
                codes.append("SIZE_OOR")
            out.append({"x": x / 1000.0, "y": y / 1000.0, "z": z_tab + size_mm / 2000.0, "yaw": yaw, "iou": iou, "score": score,
                        "height": height if height is not None else float("nan")})
        return out

    # ---- service ----
    def _on_detect(self, req, res):
        t_start = time.time()
        if not self._lock.acquire(blocking=False):          # never queue: a waiting request holds an executor thread
            res.codes = ["BUSY"]
            return res
        try:
            try:
                self._load(req.prompt)
            except Exception as e:                          # noqa: BLE001
                res.codes = ["BAD_PROMPT"]
                self.get_logger().error(f"cannot load models for {req.prompt!r}: {e}")
                return res
            n = req.frames or self.default_frames
            thr = req.min_score or self.default_score
            size_mm = req.size_mm or DEFAULT_SIZE_MM
            label = req.label or req.prompt
            self.prof = {}
            t0 = time.time()
            frames, info = self._grab(n)
            self._t("grab", t0)
            codes = []
            if not frames:
                res.codes = ["NO_FRAMES"]
                return res
            per_frame = [self._frame(t, fid, img, d, info["K"], info["D"], thr, size_mm, codes) for t, fid, img, d in frames]
            merged = self._merge(per_frame, len(frames))
            msg = Detection3DArray()
            msg.header.frame_id = self.target
            msg.header.stamp = self.get_clock().now().to_msg() if frames[-1][0] <= 0 else self._to_stamp(frames[-1][0])
            for i, m in enumerate(merged):
                d = Detection3D()
                d.header = msg.header
                d.id = f"{label}_{i}"
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id, hyp.hypothesis.score = label, float(m["score"])
                hyp.pose.pose = self._pose(m)
                cov = np.zeros(36)
                cov[0] = cov[7] = max(m["std_xy"], 0.001) ** 2
                cov[14] = 0.002 ** 2
                cov[35] = max(m["std_yaw"], math.radians(1.0)) ** 2
                hyp.pose.covariance = cov.tolist()
                d.results.append(hyp)
                d.bbox.center = hyp.pose.pose
                d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = size_mm / 1000.0
                msg.detections.append(d)
                res.heights_m.append(float(m["height"]))
            res.success = bool(merged)
            res.detections = msg
            res.codes = sorted(set(codes))
            self.pub.publish(msg)
        finally:
            self._lock.release()
        res.latency_ms = float((time.time() - t_start) * 1000.0)
        self.get_logger().info(f"{req.prompt!r}: {len(merged)} object(s) from {len(frames)} frame(s) in {res.latency_ms:.0f} ms {res.codes} | ms: " + ", ".join(f"{k} {v:.0f}" for k, v in self.prof.items()))
        return res

    @staticmethod
    def _to_stamp(t):
        from builtin_interfaces.msg import Time as TimeMsg
        return TimeMsg(sec=int(t), nanosec=int((t % 1) * 1e9))

    @staticmethod
    def _pose(m):
        p = Pose()
        p.position.x, p.position.y, p.position.z = m["x"], m["y"], m["z"]
        p.orientation.z, p.orientation.w = math.sin(m["yaw"] / 2), math.cos(m["yaw"] / 2)
        return p

    @staticmethod
    def _merge(per_frame, n_frames):
        """Group per-frame objects that sit within MERGE_RADIUS_MM of each other, keep groups seen in at least half the frames."""
        groups = []
        for fi, objs in enumerate(per_frame):
            for o in objs:
                for g in groups:
                    if math.hypot(g[0]["x"] - o["x"], g[0]["y"] - o["y"]) * 1000 < MERGE_RADIUS_MM and g[-1]["fi"] != fi:
                        g.append({**o, "fi": fi})
                        break
                else:
                    groups.append([{**o, "fi": fi}])
        out = []
        for g in groups:
            if len(g) * 2 < n_frames:
                continue
            xs, ys = np.array([o["x"] for o in g]), np.array([o["y"] for o in g])
            ang = np.array([o["yaw"] for o in g]) * 4.0                     # a cube repeats every 90 degrees
            mean_ang = math.atan2(np.sin(ang).mean(), np.cos(ang).mean()) / 4.0
            dev = np.angle(np.exp(1j * (ang - mean_ang * 4.0))) / 4.0
            out.append({"x": float(np.median(xs)), "y": float(np.median(ys)), "z": float(np.median([o["z"] for o in g])),
                        "yaw": mean_ang, "score": float(np.mean([o["score"] for o in g])),
                        "height": float(np.nanmedian([o["height"] for o in g])) if not all(math.isnan(o["height"]) for o in g) else float("nan"),
                        "std_xy": float(max(xs.std(), ys.std())), "std_yaw": float(dev.std())})
        return sorted(out, key=lambda o: -o["score"])

    def _load(self, prompt):
        if self.det is None:
            self.get_logger().info("loading engines")
            self.det = TrtOwlv2(None, os.path.join(self.engines_dir, "owlv2_det.engine"), self.prompts_file)
            self.sam = TrtEvitSam(self.sam_variant, self.engines_dir)
        self.det.set_prompt(prompt)


def main():
    rclpy.init()
    node = ObjectPerception()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
