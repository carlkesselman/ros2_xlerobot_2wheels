"""The robot as LeRobot sees it, over ROS: shared by the recorder and the policy node.

One observation is the newest JPEG from each camera (decoded to RGB) plus /joint_states in the layout's joint order.
One action is the joint targets last commanded on the arm and gripper controllers' JointTrajectory topics, the ones
the leader arms stream while recording, in the same order. The policy node sends its action chunks to the same
topics.

Everything is timed by arrival on this machine. usb_cam 0.8.1 stamps the wrist frames 0-1 s early, so header stamps
are not used.
"""
import io
import math
import os
import signal
import threading
import time
from dataclasses import dataclass

import numpy as np
import rclpy
import yaml
from PIL import Image
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def share_file(*parts):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(parts[0]), *parts[1:])


@dataclass
class Layout:
    """config/robot.yaml: which topics make an observation and an action."""
    fps: int
    cameras: dict            # name -> CompressedImage topic
    image_size: tuple        # (height, width)
    joints: list             # /joint_states names, the order of observation.state and action
    controllers: dict        # JointTrajectory topic -> its joints
    stale_s: float

    @classmethod
    def load(cls, path=None):
        path = path or share_file("xlerobot_lerobot", "config", "robot.yaml")
        with open(path) as f:
            c = yaml.safe_load(f)
        layout = cls(fps=int(c["fps"]), cameras=dict(c["cameras"]), image_size=tuple(c["image_size"]),
                     joints=list(c["joints"]), controllers={k: list(v) for k, v in c["controllers"].items()},
                     stale_s=float(c["stale_s"]))
        commanded = [j for js in layout.controllers.values() for j in js]
        missing = [j for j in layout.joints if j not in commanded]
        if missing:
            raise ValueError(f"{path}: no controller takes {missing}")
        return layout

    @property
    def names(self):
        """The joints as the dataset names them."""
        return [j.removesuffix("_joint") for j in self.joints]

    def image_key(self, camera):
        return f"observation.images.{camera}"

    def dataset_features(self):
        """LeRobotDataset features for this layout."""
        h, w = self.image_size
        vec = {"dtype": "float32", "shape": (len(self.joints),), "names": self.names}
        features = {"observation.state": dict(vec), "action": dict(vec)}
        for cam in self.cameras:
            features[self.image_key(cam)] = {"dtype": "video", "shape": (h, w, 3),
                                             "names": ["height", "width", "channels"]}
        return features


def arm_joint_limits(margin_scale=1.0):
    """{joint name: (lo, hi)} for the SO-101 arm joints, the same limits ros2_control.xacro and leader_arms use."""
    with open(share_file("xlerobot_description", "config", "robot_dimensions.yaml")) as f:
        dims = yaml.safe_load(f)["xlerobot"]["arms"]
    margin = dims["limit_margin"] * margin_scale
    limits = {}
    for side in ("left", "right"):
        for j in ARM_JOINTS:
            jd = dims["joints"][j]
            m = -jd["direction"]
            lo = min(m * jd["lower"], m * jd["upper"]) - margin
            hi = max(m * jd["lower"], m * jd["upper"]) + margin
            limits[f"{side}_arm_{j}_joint"] = (lo, hi)
    return limits


class RosRobot(Node):
    """Subscribes to the layout's topics and keeps the newest of each; publishes action chunks."""

    def __init__(self, layout, name="lerobot", publish_actions=False):
        super().__init__(name)
        self.layout = layout
        self.lock = threading.Lock()
        self.frames = {}          # camera -> (arrival, jpeg bytes)
        self.positions = {}       # joint -> (arrival, position)
        self.commands = {}        # joint -> (arrival, commanded position)
        self.resized = set()
        image_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        for cam, topic in layout.cameras.items():
            self.create_subscription(CompressedImage, topic, lambda m, c=cam: self._on_frame(c, m), image_qos)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        # Commands are always watched: the recorder's actions, and the policy node's check that nobody else (the
        # leader arms) is driving the arms before it does.
        self.publishers_ = {}
        for topic in layout.controllers:
            self.create_subscription(JointTrajectory, topic, self._on_command, 10)
            if publish_actions:
                self.publishers_[topic] = self.create_publisher(JointTrajectory, topic, 10)
        self.executor_ = None
        self.spin_thread = None

    # -- subscriptions

    def _on_frame(self, cam, msg):
        with self.lock:
            self.frames[cam] = (time.monotonic(), bytes(msg.data))

    def _on_joint_states(self, msg):
        now = time.monotonic()
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                self.positions[n] = (now, p)

    def _on_command(self, msg):
        if not msg.points:
            return
        now = time.monotonic()
        point = msg.points[-1]
        with self.lock:
            for n, p in zip(msg.joint_names, point.positions):
                self.commands[n] = (now, p)

    # -- spinning

    def spin_in_background(self):
        # One executor thread. The callbacks only store the newest message, and every rclpy executor thread competes
        # for the GIL with the caller's loop: with a 4-thread MultiThreadedExecutor, SmolVLA's chunk inference took
        # 1.8 s instead of 0.19 s on the M5 Pro (2026-11-14).
        self.executor_ = SingleThreadedExecutor()
        self.executor_.add_node(self)
        self.spin_thread = threading.Thread(target=self.executor_.spin, daemon=True)
        self.spin_thread.start()

    def close(self):
        if self.executor_ is not None:
            self.executor_.shutdown()
        self.destroy_node()

    # -- sampling

    def _vector(self, table, what, now):
        out, problems = np.zeros(len(self.layout.joints), np.float32), []
        for i, j in enumerate(self.layout.joints):
            entry = table.get(j)
            if entry is None:
                problems.append(f"no {what} for {j}")
            elif now - entry[0] > self.layout.stale_s:
                problems.append(f"{what} for {j} is {now - entry[0]:.1f} s old")
            else:
                out[i] = entry[1]
        return out, problems

    def state(self):
        """(observation.state, problems)."""
        with self.lock:
            return self._vector(self.positions, "joint state", time.monotonic())

    def action(self):
        """(commanded joint targets, problems): what the leader arms last sent."""
        with self.lock:
            return self._vector(self.commands, "command", time.monotonic())

    def observation(self):
        """({feature: value}, problems). Images are HxWx3 uint8 RGB, the state float32."""
        now = time.monotonic()
        with self.lock:
            frames = dict(self.frames)
            state, problems = self._vector(self.positions, "joint state", now)
        obs = {"observation.state": state}
        h, w = self.layout.image_size
        for cam in self.layout.cameras:
            entry = frames.get(cam)
            if entry is None:
                problems.append(f"no {cam} frame")
                continue
            if now - entry[0] > self.layout.stale_s:
                problems.append(f"{cam} frame is {now - entry[0]:.1f} s old")
                continue
            img = Image.open(io.BytesIO(entry[1])).convert("RGB")
            if img.size != (w, h):
                if cam not in self.resized:
                    self.resized.add(cam)
                    self.get_logger().warn(f"{cam} frames are {img.size[0]}x{img.size[1]}, resizing to {w}x{h}")
                img = img.resize((w, h), Image.BILINEAR)
            obs[self.layout.image_key(cam)] = np.array(img)   # writable: torch.from_numpy warns otherwise
        return obs, problems

    def ages(self):
        """{stream: seconds since its last message}, for status lines."""
        now = time.monotonic()
        with self.lock:
            out = {cam: now - self.frames[cam][0] for cam in self.frames}
            if self.positions:
                out["joint_states"] = now - max(t for t, _ in self.positions.values())
            if self.commands:
                out["commands"] = now - max(t for t, _ in self.commands.values())
        return out

    # -- commanding

    def send_chunk(self, actions, dt):
        """Send actions (T x joints, layout order) as one trajectory per controller, the first point dt from now.

        A header stamp of zero makes the controller start it on arrival and replace whatever it was running.
        """
        index = {j: i for i, j in enumerate(self.layout.joints)}
        for topic, joints in self.layout.controllers.items():
            msg = JointTrajectory()
            msg.joint_names = list(joints)
            for k, row in enumerate(actions):
                pt = JointTrajectoryPoint()
                pt.positions = [float(row[index[j]]) for j in joints]
                t = (k + 1) * dt
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int(round((t - math.floor(t)) * 1e9))
                msg.points.append(pt)
            self.publishers_[topic].publish(msg)

    def stop_motion(self):
        """An empty trajectory makes a joint_trajectory_controller drop its goal and hold where it is."""
        for topic, joints in self.layout.controllers.items():
            msg = JointTrajectory()
            msg.joint_names = list(joints)
            self.publishers_[topic].publish(msg)


def _terminate(signum, frame):
    # Once: `timeout` and pixi can both deliver a SIGTERM, and a second one would interrupt the cleanup (stopping the
    # arms, finishing the dataset). A process stuck after that needs kill -9.
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise KeyboardInterrupt


def init_ros():
    """rclpy without its own signal handlers: with them, SIGTERM only shuts ROS down and the tool's loop runs on (seen
    2026-11-14: a --live policy node survived kill). Ctrl-C and SIGTERM both raise KeyboardInterrupt in the main
    thread instead, so the tools' cleanup (stop the arms, finish the dataset) runs."""
    if not rclpy.ok():
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        signal.signal(signal.SIGTERM, _terminate)


def shutdown_ros():
    if rclpy.ok():
        rclpy.shutdown()
