#!/usr/bin/env python3
"""Presence/heartbeat watchdog for agent-issued (MCP) motion commands.

teleop's own deadman (joystick.yaml's enable_button, physically RB on this gamepad as of
2026-10-03) only guards commands sent
while a human is actively driving with the stick. Once an agent is the one issuing motion
commands, nothing requires a human to still be watching - this closes that gap: a human holds or
repeatedly taps a *different* button (config'd in mcp_presence.yaml, default button 4, Y on the
gamepad this was tested against - confirmed via `ros2 topic echo /joy`, not a generic-Xbox-layout
guess; deliberately not the drive-enable button, the two concerns don't overlap, just swapped
which physical button each one uses 2026-10-03 so drive could move to RB) and
/mcp_presence_ok stays true only while that's recent. MCP motion tools must check this before
accepting a goal and keep checking it while one is in flight, same as any other watchdog here.

Same stale-after-timeout latch shape as safety_supervisor.py's check_watchdog - not merged into
that node since this watches operator attention, not servo health, and is only relevant when
something is actually driving the robot via MCP.
"""
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class McpPresenceGuard(Node):
    def __init__(self):
        super().__init__("mcp_presence_guard")
        self.declare_parameter("presence_button", 5)
        self.declare_parameter("timeout_sec", 5.0)
        # Development switch: False means MCP motion does not wait for the button. It is False only
        # until this node restarts (the default is True), so it cannot stay off by accident.
        # ros2 param set /mcp_presence_guard presence_required false
        self.declare_parameter("presence_required", True)
        self.button = self.get_parameter("presence_button").value
        self.timeout = self.get_parameter("timeout_sec").value

        self.presence_pub = self.create_publisher(Bool, "/mcp_presence_ok", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.bypass_pub = self.create_publisher(Bool, "/mcp_presence_bypass", latched)
        self.bypass_pub.publish(Bool(data=False))
        self.add_on_set_parameters_callback(self.on_params)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.last_press_time = None
        self.ok_latched = False
        self.create_timer(0.1, self.check_watchdog)
        self.get_logger().info(
            f"mcp presence guard up, button {self.button}, timeout {self.timeout:.1f}s")

    def on_params(self, params):
        for p in params:
            if p.name == "presence_required":
                self.bypass_pub.publish(Bool(data=not p.value))
                if p.value:
                    self.get_logger().info("presence required again")
                else:
                    self.get_logger().warning("DEVELOPMENT MODE: MCP motion does not wait for the presence button")
        return SetParametersResult(successful=True)

    def on_joy(self, msg: Joy):
        if self.button < len(msg.buttons) and msg.buttons[self.button]:
            self.last_press_time = self.get_clock().now()

    def check_watchdog(self):
        if not self.get_parameter("presence_required").value:
            fresh = True
        elif self.last_press_time is None:
            fresh = False
        else:
            age = (self.get_clock().now() - self.last_press_time).nanoseconds / 1e9
            fresh = age <= self.timeout
        if fresh and not self.ok_latched:
            self.get_logger().info("presence confirmed")
        elif not fresh and self.ok_latched:
            self.get_logger().warning("presence lost - no button press within timeout")
        self.ok_latched = fresh
        self.presence_pub.publish(Bool(data=fresh))


def main():
    rclpy.init()
    node = McpPresenceGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
