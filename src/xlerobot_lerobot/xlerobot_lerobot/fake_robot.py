"""Fake robot topics, to try record.py and policy.py without the robot: the layout's cameras (a moving test pattern,
JPEG), /joint_states and leader-arm commands, all following slow sine waves.

    ROS_DOMAIN_ID=42 pixi run tools/lerobot.sh fake_robot
    ROS_DOMAIN_ID=42 pixi run tools/lerobot.sh record --repo-id test/fake --task "wave" --num-episodes 2

It publishes on the real arm controllers' command topics, so it refuses to run in the robot's ROS domain (0, or
ROS_DOMAIN_ID unset) unless --any-domain is given. --no-leader leaves the commands out (a disengaged leader).
"""
import argparse
import io
import math
import os
import time

import numpy as np
import rclpy
from PIL import Image, ImageDraw
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .robot_io import Layout


class FakeRobot(Node):
    def __init__(self, layout, leader):
        super().__init__("lerobot_fake_robot")
        self.layout, self.leader = layout, leader
        self.t0 = time.monotonic()
        self.cams = {c: self.create_publisher(CompressedImage, t, 2) for c, t in layout.cameras.items()}
        self.js = self.create_publisher(JointState, "/joint_states", 10)
        self.cmd = {t: self.create_publisher(JointTrajectory, t, 10) for t in layout.controllers}
        self.create_timer(1.0 / layout.fps, self.publish_frames)
        self.create_timer(0.02, self.publish_joints)

    def angle(self, i, t):
        return 0.3 * math.sin(2 * math.pi * 0.2 * t + i * 0.5)

    def publish_frames(self):
        t = time.monotonic() - self.t0
        h, w = self.layout.image_size
        for k, (cam, pub) in enumerate(self.cams.items()):
            img = Image.fromarray(np.full((h, w, 3), (40 + 60 * k, 80, 120), np.uint8))
            d = ImageDraw.Draw(img)
            x = int((w - 80) * (0.5 + 0.5 * math.sin(t + k)))
            d.rectangle([x, h // 2 - 40, x + 80, h // 2 + 40], fill=(240, 200, 40))
            d.text((10, 10), f"{cam} {t:7.2f}", fill=(255, 255, 255))
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=70)
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "rgb8; jpeg compressed bgr8"
            msg.data = buf.getvalue()
            pub.publish(msg)

    def publish_joints(self):
        t = time.monotonic() - self.t0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.layout.joints)
        msg.position = [self.angle(i, t) for i in range(len(msg.name))]
        self.js.publish(msg)
        if not self.leader:
            return
        index = {j: i for i, j in enumerate(self.layout.joints)}
        for topic, joints in self.layout.controllers.items():
            m = JointTrajectory()
            m.joint_names = list(joints)
            pt = JointTrajectoryPoint()
            pt.positions = [self.angle(index[j], t + 0.1) for j in joints]   # the leader is a little ahead
            pt.time_from_start.nanosec = 40_000_000
            m.points = [pt]
            self.cmd[topic].publish(m)


def main(argv=None):
    p = argparse.ArgumentParser(prog="lerobot.sh fake_robot", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=None)
    p.add_argument("--no-leader", action="store_true", help="publish no leader commands")
    p.add_argument("--any-domain", action="store_true", help="run even in the robot's ROS domain")
    a = p.parse_args(argv)
    if os.environ.get("ROS_DOMAIN_ID", "0") in ("", "0") and not a.any_domain:
        raise SystemExit("refusing to fake the robot in ROS domain 0, where the real one is: "
                         "set ROS_DOMAIN_ID to something else (e.g. 42)")
    rclpy.init()
    node = FakeRobot(Layout.load(a.config), leader=not a.no_leader)
    print(f"fake robot on ROS domain {os.environ.get('ROS_DOMAIN_ID')}: "
          f"{len(node.cams)} cameras, /joint_states, leader commands {'on' if node.leader else 'off'}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
