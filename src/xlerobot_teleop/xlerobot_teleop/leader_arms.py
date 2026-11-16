#!/usr/bin/env python3
"""SO-101 leader arms -> follower arm controllers, for the operator station (the Mac, or rosie).

Reads both leader arms over their Feetech bus boards (50 Hz) and streams their pose to the follower's
joint_trajectory_controllers (left/right_arm_controller and left/right_gripper_controller) as
single-point trajectories. Nothing else on the cart changes: bringup with_arms:=true already
starts those controllers.

Mapping, per joint, in servo steps (both arms calibrated the lerobot way):
  degrees (default)  follower = follower_mid + (leader - leader_mid), clamped to the follower's
                     range: the follower copies the leader's angle 1:1.
  range              follower = follower_min + (leader - leader_min) / leader_span * follower_span,
                     what lerobot does with use_degrees False (Carl's lerobot setup).
  The gripper is always range to range, as in lerobot.
The follower step then becomes the model's joint angle the way sts_hardware_interface reports it:
  angle = (zero_step - step) * 2 pi / 4096          (zero_step from robot_dimensions.yaml)

Leader ranges come from each servo's EEPROM (Min/Max_Position_Limit, which lerobot's calibration
writes), so they are the arms' current calibration whatever JSON files lie around. Follower
ranges come from the follower's lerobot calibration JSON (the cart's xlerobot.json), which this
node needs a copy of.

Safety: starts DISENGAGED and publishes nothing. ~/engage (std_srvs/SetBool) or a tap of
engage_button on /joy toggles it; ~/engaged (std_msgs/Bool, latched) says which. On engage the command blends from where the follower is
(/joint_states) to the leader over ramp_s, so a leader held somewhere else doesn't snap the arm
across; sts_hardware_interface also caps every joint at its position_rate_limit. If a leader
stops answering, that arm stops streaming and its controller holds the last point.

The leaders are only read, never written: their torque stays as it is (off).
"""
import glob
import json
import math
import os
import time

import rclpy
import serial
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
ALL_JOINTS = ARM_JOINTS + ("gripper",)
IDS = {j: i + 1 for i, j in enumerate(ALL_JOINTS)}
STEPS_PER_RAD = 4096 / (2 * math.pi)
BAUD = 1_000_000
MIN_POSITION_LIMIT, MAX_POSITION_LIMIT, PRESENT_POSITION = 9, 11, 56


def sign_magnitude(value, sign_bit):
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if value & (1 << sign_bit) else magnitude


def map_step(leader, lcal, fcal, mode):
    """Leader servo step -> follower servo step. cal = (range_min, range_max)."""
    lmin, lmax = lcal
    fmin, fmax = fcal
    if mode == "range":
        frac = (leader - lmin) / max(lmax - lmin, 1)
        step = fmin + frac * (fmax - fmin)
    else:
        step = (fmin + fmax) / 2 + (leader - (lmin + lmax) / 2)
    return min(max(step, fmin), fmax)


def step_to_angle(step, zero_step):
    return (zero_step - step) / STEPS_PER_RAD


class LeaderBus:
    """Feetech STS protocol 0, read-only: limits once, then Present_Position by sync read."""

    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.02)

    def close(self):
        self.ser.close()

    def _packet(self, motor_id, instruction, params):
        body = bytes([motor_id, len(params) + 2, instruction]) + bytes(params)
        return b"\xff\xff" + body + bytes([~sum(body) & 0xFF])

    def _reply(self, motor_id):
        head = self.ser.read(4)
        if len(head) < 4 or head[:2] != b"\xff\xff" or head[2] != motor_id:
            return None
        rest = self.ser.read(head[3])
        if len(rest) < head[3] or (~sum(head[2:] + rest[:-1]) & 0xFF) != rest[-1]:
            return None
        return rest[1:-1]

    def read(self, motor_id, addr, size, retries=3):
        for _ in range(retries):
            self.ser.reset_input_buffer()
            self.ser.write(self._packet(motor_id, 0x02, [addr, size]))
            data = self._reply(motor_id)
            if data is not None and len(data) >= size:
                return int.from_bytes(data[:size], "little")
        raise IOError(f"servo {motor_id} did not answer")

    def positions(self, ids):
        """Sync read of Present_Position. Returns {id: step}, missing ids left out."""
        self.ser.reset_input_buffer()
        self.ser.write(self._packet(0xFE, 0x82, [PRESENT_POSITION, 2] + list(ids)))
        out = {}
        for motor_id in ids:
            data = self._reply(motor_id)
            if data is None or len(data) < 2:
                break
            out[motor_id] = sign_magnitude(int.from_bytes(data[:2], "little"), 15)
        return out


def find_port(serial_hint):
    """A /dev/cu.* (macOS) or /dev/serial/by-id path whose USB serial number contains the hint."""
    if os.path.exists(serial_hint):
        return serial_hint
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.serial_number and serial_hint.lower() in p.serial_number.lower():
            return p.device.replace("/dev/tty.", "/dev/cu.")
    for p in glob.glob("/dev/serial/by-id/*"):
        if serial_hint in p:
            return p
    return None


class LeaderArms(Node):
    def __init__(self):
        super().__init__("leader_arms")
        p = self.declare_parameter
        p("arms", ["left", "right"])
        p("left_serial", "5B90149222")    # docs/platform.md: left leader bus board
        p("right_serial", "5A7A017922")
        p("follower_calibration",
          os.path.expanduser("~/.cache/huggingface/lerobot/calibration/robots/xlerobot/xlerobot.json"))
        p("mapping", "degrees")
        p("rate_hz", 50.0)
        p("ramp_s", 2.0)
        p("engage_button", -1)            # /joy button that toggles engage, -1 = service only
        gp = self.get_parameter
        self.mode = gp("mapping").value
        if self.mode not in ("degrees", "range"):
            raise ValueError("mapping must be degrees or range")
        self.period = 1.0 / gp("rate_hz").value
        self.ramp_s = gp("ramp_s").value
        self.engage_button = gp("engage_button").value

        dims_file = os.path.join(get_package_share_directory("xlerobot_description"),
                                 "config", "robot_dimensions.yaml")
        dims = yaml.safe_load(open(dims_file))["xlerobot"]["arms"]
        margin = dims["limit_margin"]
        follower = json.load(open(gp("follower_calibration").value))

        self.arms = {}
        for side in gp("arms").value:
            port = find_port(gp(f"{side}_serial").value)
            if port is None:
                raise RuntimeError(f"{side} leader: no serial port with serial {gp(f'{side}_serial').value}")
            bus = LeaderBus(port)
            arm = {"bus": bus, "joints": {}, "failures": 0}
            for j in ALL_JOINTS:
                jd = dims["joints"][j]
                m = -jd["direction"]   # the model's limits, as in ros2_control.xacro
                lo = min(m * jd["lower"], m * jd["upper"]) - margin
                hi = max(m * jd["lower"], m * jd["upper"]) + margin
                lcal = (bus.read(IDS[j], MIN_POSITION_LIMIT, 2), bus.read(IDS[j], MAX_POSITION_LIMIT, 2))
                fc = follower[f"{side}_arm_{j}"]
                arm["joints"][j] = {
                    "name": f"{side}_arm_{j}_joint", "lcal": lcal,
                    "fcal": (fc["range_min"], fc["range_max"]),
                    "zero": jd["zero_step"][side], "lo": lo, "hi": hi,
                }
                span = lcal[1] - lcal[0]
                self.get_logger().info(
                    f"{side} {j:13s} leader {lcal[0]:4d}..{lcal[1]:4d}  follower {fc['range_min']:4d}..{fc['range_max']:4d}"
                    f"  zero {jd['zero_step'][side]}")
                if span < 400:
                    self.get_logger().warn(
                        f"{side} leader {j} range is only {span} steps: recalibrate it, the follower "
                        "will swing its whole range for a tiny leader motion")
            arm["arm_pub"] = self.create_publisher(JointTrajectory, f"/{side}_arm_controller/joint_trajectory", 10)
            arm["grip_pub"] = self.create_publisher(JointTrajectory, f"/{side}_gripper_controller/joint_trajectory", 10)
            self.arms[side] = arm
            self.get_logger().info(f"{side} leader on {port}")

        self.follower_state = {}
        self.engaged = False
        self.engaged_at = 0.0
        self.start_pose = {}
        self.last_button = 0
        # ~/engaged, latched: whether the leaders are driving the arms (the station viewer shows it)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.engaged_pub = self.create_publisher(Bool, "~/engaged", latched)
        self.engaged_pub.publish(Bool(data=False))
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_service(SetBool, "~/engage", self.on_engage)
        self.create_timer(self.period, self.tick)
        self.get_logger().info(
            f"leader arms up ({self.mode} mapping), DISENGAGED - call "
            f"`ros2 service call /leader_arms/engage std_srvs/srv/SetBool \"{{data: true}}\"`"
            + (f" or tap /joy button {self.engage_button}" if self.engage_button >= 0 else ""))

    def on_joint_states(self, msg):
        self.follower_state.update(zip(msg.name, msg.position))

    def on_joy(self, msg):
        if self.engage_button < 0 or self.engage_button >= len(msg.buttons):
            return
        b = msg.buttons[self.engage_button]
        if b and not self.last_button:
            self.set_engaged(not self.engaged)
        self.last_button = b

    def on_engage(self, req, resp):
        ok, why = self.set_engaged(req.data)
        resp.success, resp.message = ok, why
        return resp

    def set_engaged(self, on):
        if on and not self.engaged:
            names = [j["name"] for a in self.arms.values() for j in a["joints"].values()]
            missing = [n for n in names if n not in self.follower_state]
            if missing:
                why = f"no follower state for {missing[0]} (is bringup with_arms:=true running?)"
                self.get_logger().error(why)
                return False, why
            self.start_pose = {n: self.follower_state[n] for n in names}
            self.engaged_at = time.monotonic()
        self.engaged = on
        self.engaged_pub.publish(Bool(data=on))
        why = "engaged" if on else "disengaged, follower holds its last command"
        self.get_logger().info(why)
        return True, why

    def tick(self):
        if not self.engaged:
            return
        blend = min((time.monotonic() - self.engaged_at) / self.ramp_s, 1.0) if self.ramp_s > 0 else 1.0
        stamp_ns = int(self.period * 2.0 * 1e9)   # reach each pose in two periods: no stall between messages
        for side, arm in self.arms.items():
            try:
                pos = arm["bus"].positions([IDS[j] for j in ALL_JOINTS])
            except (OSError, serial.SerialException) as e:
                pos = {}
                self.get_logger().error(f"{side} leader: {e}", throttle_duration_sec=2.0)
            if len(pos) < len(ALL_JOINTS):
                arm["failures"] += 1
                if arm["failures"] in (3, 30):
                    self.get_logger().warn(f"{side} leader: {arm['failures']} incomplete reads, not streaming")
                continue
            arm["failures"] = 0
            target = {}
            for j, jc in arm["joints"].items():
                mode = "range" if j == "gripper" else self.mode
                step = map_step(pos[IDS[j]], jc["lcal"], jc["fcal"], mode)
                angle = min(max(step_to_angle(step, jc["zero"]), jc["lo"]), jc["hi"])
                start = self.start_pose[jc["name"]]
                target[j] = start + blend * (angle - start)
            for pub, joints in ((arm["arm_pub"], ARM_JOINTS), (arm["grip_pub"], ("gripper",))):
                msg = JointTrajectory()
                msg.joint_names = [arm["joints"][j]["name"] for j in joints]
                pt = JointTrajectoryPoint()
                pt.positions = [float(target[j]) for j in joints]
                pt.time_from_start.nanosec = stamp_ns
                msg.points = [pt]
                pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, "engaged_pub") and rclpy.ok():
            self.engaged_pub.publish(Bool(data=False))
        for arm in self.arms.values():
            arm["bus"].close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LeaderArms()
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
