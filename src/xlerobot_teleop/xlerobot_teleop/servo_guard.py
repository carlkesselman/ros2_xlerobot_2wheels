#!/usr/bin/env python3
"""The robot's own guard against stalled and overheating servos. Runs on the robot with bringup.

A position servo that can't reach its command pushes as hard as it can for as long as it is commanded. On
2026-11-16 the right wrist_flex did that against an obstruction for almost two hours, reached 85 C and broke, the
joint's third servo. Its own protections didn't stop it: the stall stayed under its overload threshold, and it went
past its 70 C limit. leader_arms has a guard of its own, but only while the leader arms drive. This one watches the
servos whatever commands them: the leader arms, the lerobot policy, MoveIt, the skills, head_teleop.

Every commander goes through the trajectory controllers, so this reads each one's controller_state (the command it
is following and the measured position) and /dynamic_joint_states (each servo's temperature):

  stall   a joint further than stall_error from its command (the command clamped to the joint's limits from
          /robot_description, since ros2_control clamps it there too), not getting any closer, for stall_s.
          Not for grippers: gripping is a deliberate stall.
  hot     any servo, grippers too, at temp_stop.
  either  hold: one controller switch swaps the trajectory controller for its hold twin (<group>_hold, a
          forward_command_controller loaded inactive by bringup, controllers.yaml), which is then sent the measured
          positions. Nothing pushes, the joints stay stiff where they are, and no commander can drive them: they all
          talk to the trajectory controller, now inactive. (Sending the trajectory controller a hold and then
          deactivating it doesn't work: a commander streaming at 50 Hz overwrites the hold before it goes.) Without
          a hold twin the trajectory controller is only deactivated, and the temperature check is the backstop.
  cut     a servo still heating to temp_cut: every controller on its bus held as above, then torque off on the
          whole bus (/emergency_stop_<bus>), the last resort. The arm goes limp.

reset (std_msgs/String "reset" on ~/command, a topic so it also works from rosie, whose Fast DDS drops service
requests from elroy's) swaps the trajectory controllers back; they start from the hold's command, so nothing moves.
For a bus it cut, the holds are first sent where the limp arm is now and torque comes back after that, so the arm
doesn't jump to where it was. Refused while a servo is at temp_resume or more. leader_arms sends it when the leader
arms engage. An emergency stop someone else set is left alone.

~/status (std_msgs/String, JSON, latched): what is held or cut and why, and the servos at temp_warn or more. The
station viewer shows it.
"""
import json
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from control_msgs.msg import DynamicJointState, JointTrajectoryControllerState
from controller_manager_msgs.srv import SwitchController
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool

CONTROLLERS = ["left_arm_controller", "right_arm_controller", "left_gripper_controller", "right_gripper_controller",
               "head_controller"]


def short(joint):
    """right_arm_wrist_flex_joint -> right wrist_flex, head_pan_joint -> head pan"""
    name = joint.removesuffix("_joint")
    side, sep, rest = name.partition("_arm_")
    if sep:
        return f"{side} {rest}"
    return name.replace("_", " ", 1)


class ServoGuard(Node):
    def __init__(self):
        super().__init__("servo_guard")
        p = self.declare_parameter
        p("controllers", CONTROLLERS)
        p("stall_error", 0.15)      # rad
        p("stall_s", 3.0)           # longer than leader_arms' 2 s, so its gentler disengage comes first
        p("temp_warn", 55.0)        # C: in the status
        p("temp_resume", 60.0)      # C: no reset while a servo is this hot
        p("temp_stop", 65.0)        # C: hold and deactivate
        p("temp_cut", 70.0)         # C: torque off on the bus
        p("rate_hz", 10.0)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.stall_error, self.stall_s = gp("stall_error"), gp("stall_s")
        self.temp_warn, self.temp_resume, self.temp_stop, self.temp_cut = (
            gp(n) for n in ("temp_warn", "temp_resume", "temp_stop", "temp_cut"))

        self.ctrl = {}                # controller -> {"raw", "joints", "ref", "fb", "at", "hold", "hold_pub"}
        for c in gp("controllers"):
            hold = c.removesuffix("_controller") + "_hold"
            self.ctrl[c] = {"raw": None, "joints": [], "ref": {}, "fb": {}, "at": 0.0, "hold": hold,
                            "hold_pub": self.create_publisher(Float64MultiArray, f"/{hold}/commands", 10)}
            # 100 Hz each: kept serialized, the newest decoded at rate_hz
            self.create_subscription(JointTrajectoryControllerState, f"/{c}/controller_state",
                                     lambda data, c=c: self.ctrl[c].__setitem__("raw", data), 10, raw=True)
        self.dynamic_raw = None
        self.create_subscription(DynamicJointState, "/dynamic_joint_states",
                                 lambda data: setattr(self, "dynamic_raw", data), 10, raw=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.create_subscription(String, "~/command", self.on_command, 10)
        self.status_pub = self.create_publisher(String, "~/status", latched)
        self.switch = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.estop = {}               # bus -> SetBool client

        self.limits = {}              # joint -> (lower, upper)
        self.bus_of = {}              # joint -> ros2_control hardware component (xlerobot_bus1, ...)
        self.temps = {}               # joint -> C
        self.positions = {}           # joint -> rad, measured (also while its controller is inactive)
        self.temps_at = 0.0
        self.stall_since = {}         # joint -> (since, position then)
        self.held = {}                # controller -> why
        self.via_hold = set()         # held controllers whose hold twin is active (else only deactivated)
        self.cut = {}                 # bus -> why
        self.seq = 0
        self.last_status = None
        self.create_timer(1.0 / gp("rate_hz"), self.check)
        self.publish_status()
        self.get_logger().info(
            f"servo guard up: stall {self.stall_error} rad for {self.stall_s} s, hold at {self.temp_stop:.0f} C, "
            f"cut torque at {self.temp_cut:.0f} C; watching {', '.join(self.ctrl)}")

    # -- inputs

    def on_description(self, msg):
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as e:
            self.get_logger().warn(f"robot_description does not parse: {e}")
            return
        for joint in root.iter("joint"):
            limit = joint.find("limit")
            if joint.get("type") in ("revolute", "prismatic") and limit is not None:
                self.limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
        for hw in root.iter("ros2_control"):
            for joint in hw.findall("joint"):
                self.bus_of[joint.get("name")] = hw.get("name")
        for bus in set(self.bus_of.values()):
            if bus not in self.estop:
                self.estop[bus] = self.create_client(SetBool, f"/emergency_stop_{bus}")

    def on_command(self, msg):
        if msg.data.strip() == "reset":
            self.reset()
        else:
            self.get_logger().warn(f"unknown command {msg.data!r} (reset)")

    def decode(self, now):
        for c, s in self.ctrl.items():
            if s["raw"] is None:
                continue
            m = deserialize_message(s["raw"], JointTrajectoryControllerState)
            s["raw"] = None
            s["joints"] = list(m.joint_names)
            s["ref"] = dict(zip(m.joint_names, m.reference.positions))
            s["fb"] = dict(zip(m.joint_names, m.feedback.positions))
            s["at"] = now
        if self.dynamic_raw is not None:
            m = deserialize_message(self.dynamic_raw, DynamicJointState)
            self.dynamic_raw = None
            for name, iv in zip(m.joint_names, m.interface_values):
                values = dict(zip(iv.interface_names, iv.values))
                if "temperature" in values:
                    self.temps[name] = values["temperature"]
                if "position" in values:
                    self.positions[name] = values["position"]
            self.temps_at = now
        elif now - self.temps_at > 5.0:
            self.temps = {}           # none for a while: don't act on old readings

    # -- the checks

    def check(self):
        now = time.monotonic()
        self.decode(now)
        for c, s in self.ctrl.items():
            if c in self.held or now - s["at"] > 1.0:
                for j in s["joints"]:
                    self.stall_since.pop(j, None)
                continue
            for j in s["joints"]:
                if "gripper" in j or j not in s["ref"] or j not in s["fb"]:
                    continue
                lo, hi = self.limits.get(j, (-math.inf, math.inf))
                command, measured = min(max(s["ref"][j], lo), hi), s["fb"][j]
                if abs(command - measured) <= self.stall_error:
                    self.stall_since.pop(j, None)
                    continue
                since, where = self.stall_since.get(j, (now, measured))
                if abs(measured - where) > self.stall_error / 2:   # still moving: start over from here
                    since, where = now, measured
                self.stall_since[j] = (since, where)
                if now - since >= self.stall_s:
                    self.hold(c, f"{short(j)} stuck {math.degrees(abs(command - measured)):.0f} deg short of its "
                                 f"command for {self.stall_s:.0f} s")
                    break
        for j, t in sorted(self.temps.items(), key=lambda kv: -kv[1]):
            bus, c = self.bus_of.get(j), self.controller_of(j)
            if t >= self.temp_cut and bus and bus not in self.cut:
                self.cut_bus(bus, f"{short(j)} at {t:.0f} C")
            elif t >= self.temp_stop and c and c not in self.held:
                self.hold(c, f"{short(j)} at {t:.0f} C")
        self.publish_status()

    def controller_of(self, joint):
        return next((c for c, s in self.ctrl.items() if joint in s["joints"]), None)

    # -- the actions

    def send_hold(self, controller, positions):
        """Command the controller's hold twin to these positions (joint -> rad), in the controller's joint order.
        False if one of its joints is missing."""
        s = self.ctrl[controller]
        if not s["joints"] or not all(j in positions for j in s["joints"]):
            return False
        s["hold_pub"].publish(Float64MultiArray(data=[float(positions[j]) for j in s["joints"]]))
        return True

    def after(self, seconds, fn):
        timer = None

        def once():
            timer.cancel()
            fn()
        timer = self.create_timer(seconds, once)

    def hold(self, controller, why):
        self.held[controller] = why
        self.seq += 1
        s = self.ctrl[controller]
        where = dict(s["fb"] or self.positions)    # where it is now, before anything else moves
        self.get_logger().error(f"{controller}: {why} - holding where it is ({s['hold']})")

        def holding():
            self.via_hold.add(controller)
            # a couple of times, in case the first is missed while the hold controller starts
            for delay in (0.0, 0.1, 0.3):
                self.after(delay or 0.01, lambda: self.send_hold(controller, where))

        def no_twin():
            self.get_logger().error(f"{controller}: no {s['hold']} to swap in, only deactivating it (a commander "
                                    "may have left it pushing; the temperature check stays the backstop)")
            self.switch_controllers(deactivate=[controller])
        self.switch_controllers(activate=[s["hold"]], deactivate=[controller], then=holding, otherwise=no_twin,
                                strict=True)

    def controllers_on(self, buses):
        return [c for c, s in self.ctrl.items() if any(self.bus_of.get(j) in buses for j in s["joints"])]

    def cut_bus(self, bus, why):
        self.cut[bus] = why
        self.seq += 1
        for c in self.controllers_on([bus]):
            if c not in self.held:
                self.hold(c, f"{why} (cutting {bus})")
        self.get_logger().error(f"{bus}: {why} - cutting its torque (emergency stop)")
        client = self.estop.get(bus)
        if client is None or not client.service_is_ready():
            self.get_logger().error(f"no /emergency_stop_{bus} service: can't cut its torque")
            return
        client.call_async(SetBool.Request(data=True))

    def switch_controllers(self, activate=(), deactivate=(), then=None, otherwise=None, strict=False):
        if not self.switch.service_is_ready():
            self.get_logger().error("no /controller_manager/switch_controller service")
            if otherwise:
                otherwise()
            return
        req = SwitchController.Request(
            activate_controllers=list(activate), deactivate_controllers=list(deactivate),
            strictness=SwitchController.Request.STRICT if strict else SwitchController.Request.BEST_EFFORT)
        req.timeout.sec = 2
        fut = self.switch.call_async(req)

        def done(f):
            r = f.result()
            what = ", ".join([f"activated {c}" for c in activate] + [f"deactivated {c}" for c in deactivate])
            if r is None or not r.ok:
                self.get_logger().error(f"switch_controller failed ({what}): {r.message if r else 'no answer'}")
                if otherwise:
                    otherwise()
            else:
                self.get_logger().info(what)
                if then:
                    then()
        fut.add_done_callback(done)

    def reset(self):
        hot = {j: t for j, t in self.temps.items() if t >= self.temp_resume}
        if hot:
            j = max(hot, key=hot.get)
            self.get_logger().warn(f"reset refused: {short(j)} is at {hot[j]:.0f} C, let it cool below "
                                   f"{self.temp_resume:.0f} C")
            return
        if not self.held and not self.cut:
            return
        held, cut = list(self.held), list(self.cut)
        via_hold = [c for c in held if c in self.via_hold]
        self.get_logger().info(f"reset: back to {held or 'nothing'}, torque back on {cut or 'nothing'}")
        self.held, self.via_hold, self.stall_since = {}, set(), {}

        def swap_back():
            # the trajectory controllers start from the hold's command, where the arm is: nothing moves
            self.switch_controllers(activate=held, deactivate=[self.ctrl[c]["hold"] for c in via_hold],
                                    then=self.publish_status)

        def release():
            for bus in cut:
                client = self.estop.get(bus)
                if client is not None and client.service_is_ready():
                    client.call_async(SetBool.Request(data=False))
                self.cut.pop(bus, None)
            self.after(0.3, swap_back)

        if cut:
            # The limp arm has moved since it was held: send the holds where it is now, and give torque back once
            # that has reached the hardware, so it doesn't jump back to where it was.
            for c in self.controllers_on(cut):
                if c in via_hold and not self.send_hold(c, self.positions):
                    self.get_logger().warn(f"{c}: no measured position for all its joints")
            self.after(0.4, release)
        elif held:
            swap_back()
        self.publish_status()

    def publish_status(self):
        status = {
            "held": self.held,
            "cut": self.cut,
            "seq": self.seq,
            "hot": {short(j): round(t) for j, t in sorted(self.temps.items()) if t >= self.temp_warn},
        }
        if status != self.last_status:
            self.status_pub.publish(String(data=json.dumps(status)))
            self.last_status = status


def main():
    rclpy.init()
    node = ServoGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
