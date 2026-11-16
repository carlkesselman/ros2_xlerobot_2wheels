#!/usr/bin/env python3
"""Gamepad d-pad -> head pan and tilt, through head_controller.

Hold a d-pad direction and the head turns at pan_speed / tilt_speed; let go and it stops where it
is. Left/right pans, up/down tilts. A single-point trajectory goes to
/head_controller/joint_trajectory on every tick while a direction is held. Nothing is sent while the
d-pad is idle, so it never fights another source of head commands (the MCP server, a skill).

The d-pad is a hat, which joy_node reports as axes 6 (left/right, +1 = left) and 7 (up/down,
+1 = up): on Linux (arm_servo_teleop.yaml) and on the Mac too, where joy_node gives the 8BitDo
8 axes and 12 buttons (checked 2026-11-14). game_controller_node reports it as buttons instead:
up 11, down 12, left 13, right 14. With joy_layout sdl those buttons are read when the message
has them (more than 14 buttons); otherwise the axes are.

Each press starts from the head's measured position (/joint_states), so the target never drifts
from where the head is. The target is clamped to the joints' limits from /robot_description, so
holding a direction against a limit doesn't wind it up. The driver and controller_manager enforce
the same limits again downstream.

Starting is bringup with_head:=true on the robot (that spawns head_controller) and this node next
to joy_node, which teleop.launch.py does.
"""
import xml.etree.ElementTree as ET

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HeadTeleop(Node):
    def __init__(self):
        super().__init__("head_teleop")
        p = self.declare_parameter
        p("joy_layout", "linux")
        p("dpad_buttons", [11, 12, 13, 14])   # sdl: up, down, left, right
        p("dpad_axes", [6, 7])                # linux: left/right (+1 left), up/down (+1 up)
        p("pan_joint", "head_pan_joint")
        p("tilt_joint", "head_tilt_joint")
        p("pan_speed", 0.6)                   # rad/s while held; the joints' limit is 0.8
        p("tilt_speed", 0.4)
        # The model's sign conventions, checked with the head camera on the cart 2026-11-14:
        # positive pan turns the head left, positive tilt looks up. -1 flips a direction if a
        # build has it the other way.
        p("pan_left_sign", 1.0)
        p("tilt_up_sign", 1.0)
        p("rate_hz", 30.0)
        p("joy_timeout_s", 0.5)               # joy_node repeats at 20 Hz while anything is held
        gp = lambda name: self.get_parameter(name).value  # noqa: E731

        layout = gp("joy_layout")
        if layout not in ("sdl", "linux", "ps"):   # linux and ps both have the d-pad as a hat
            raise ValueError(f"joy_layout must be sdl, linux or ps, not {layout!r}")
        self.layout = layout
        self.buttons = list(gp("dpad_buttons"))
        self.axes = list(gp("dpad_axes"))
        self.joints = (gp("pan_joint"), gp("tilt_joint"))
        self.speed = (gp("pan_speed"), gp("tilt_speed"))
        self.signs = (gp("pan_left_sign"), gp("tilt_up_sign"))
        self.period = 1.0 / gp("rate_hz")
        self.joy_timeout = gp("joy_timeout_s")

        self.measured = {}
        self.limits = {}
        self.joy = None
        self.joy_time = None
        self.target = None    # [pan, tilt] while a direction is held, None when idle

        self.pub = self.create_publisher(JointTrajectory, "/head_controller/joint_trajectory", 10)
        self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.create_timer(self.period, self.tick)
        self.get_logger().info(
            f"head teleop up ({layout} d-pad): left/right pan, up/down tilt")

    def on_joy(self, msg):
        self.joy = msg
        self.joy_time = self.get_clock().now()

    def on_joint_states(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joints:
                self.measured[name] = pos

    def on_description(self, msg):
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as e:
            self.get_logger().warn(f"robot_description does not parse: {e}")
            return
        for joint in root.iter("joint"):
            limit = joint.find("limit")
            if joint.get("name") in self.joints and limit is not None:
                self.limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
        missing = [j for j in self.joints if j not in self.limits]
        if missing:
            self.get_logger().warn(f"no limits for {missing} in robot_description (bringup with_head:=true?)")
        else:
            self.get_logger().info("head limits: " + ", ".join(
                f"{j} {lo:+.2f}..{hi:+.2f}" for j, (lo, hi) in self.limits.items()))

    def dpad(self):
        """(right, up), each -1, 0 or +1."""
        msg = self.joy
        if msg is None or (self.get_clock().now() - self.joy_time).nanoseconds * 1e-9 > self.joy_timeout:
            return 0, 0
        # Buttons only where the message is long enough to have them: joy_node gives this pad
        # 12 buttons, so an index of 11 there is some other button, not "up".
        if self.layout == "sdl" and len(msg.buttons) > max(self.buttons):
            return self.dpad_from_buttons(msg)
        a = lambda i: msg.axes[i] if i < len(msg.axes) else 0.0  # noqa: E731
        lr, ud = a(self.axes[0]), a(self.axes[1])
        return (-1 if lr > 0.5 else 1 if lr < -0.5 else 0), (1 if ud > 0.5 else -1 if ud < -0.5 else 0)

    def dpad_from_buttons(self, msg):
        b = lambda i: 1 if msg.buttons[i] else 0  # noqa: E731
        up, down, left, right = (b(i) for i in self.buttons)
        return right - left, up - down

    def tick(self):
        right, up = self.dpad()
        if right == 0 and up == 0:
            self.target = None
            return
        if self.target is None:
            if not all(j in self.measured for j in self.joints):
                self.get_logger().warn("no head joint states yet (bringup with_head:=true?)",
                                       throttle_duration_sec=5.0)
                return
            self.target = [self.measured[j] for j in self.joints]
        self.target[0] -= self.signs[0] * right * self.speed[0] * self.period   # right: away from left
        self.target[1] += self.signs[1] * up * self.speed[1] * self.period
        for i, joint in enumerate(self.joints):
            if joint in self.limits:
                lo, hi = self.limits[joint]
                self.target[i] = min(max(self.target[i], lo), hi)
        msg = JointTrajectory()
        msg.joint_names = list(self.joints)
        point = JointTrajectoryPoint()
        point.positions = list(self.target)
        point.time_from_start.nanosec = int(2 * self.period * 1e9)   # two ticks: no stall between them
        msg.points = [point]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HeadTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
