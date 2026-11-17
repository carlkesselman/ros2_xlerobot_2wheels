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

The guard: a servo that can't reach its command pushes at full effort until it does, and on 2026-11-16 the right
wrist_flex did that against an obstruction for almost two hours, reached 85 C and shut down (the joint's second
servo). So, while engaged:
  - stall: an arm joint (not a gripper: gripping is a deliberate stall) further than stall_error from its command
    for stall_s disengages;
  - heat: any arm servo, grippers too, at temp_stop disengages (servo temperatures from /dynamic_joint_states,
    read twice a second), and engaging is refused while one is at temp_resume or more;
  - commands stay inside the model's own limits, not the limit_margin wider ones ros2_control enforces (the margin
    is there so a measured rest pose a little past a limit doesn't stop the controller, not to be driven into);
  - elbow + wrist_flex stays at or above robot_dimensions.yaml's elbow_plus_wrist_min (+ margin): with the arm
    folded the wrist bending back meets the upper arm, 22-26 degrees before its end (measured 2026-11-16), and the
    limit eases off one for one as the elbow opens. A folded leader with its wrist bent back would otherwise stall
    the follower's wrist there.
On any disengage, a joint further than hold_error from its command (stuck, or still catching up) is told to hold
where it is, so nothing keeps pushing; the others keep their last command, so gravity sag doesn't turn into a
droop. Grippers keep theirs (an object stays gripped) unless it is a gripper that overheated.
~/status (std_msgs/String, JSON, latched) has the last fault and the servos at temp_warn or more; the station viewer
shows both. The robot has a guard of its own for every commander (servo_guard, started by bringup), which locks a
stalled controller out; engaging sends it "reset" (/servo_guard/command) to bring them back.

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
from control_msgs.msg import DynamicJointState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String
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
        # the guard (see the module docstring)
        p("stall_error", 0.15)            # rad
        p("stall_s", 2.0)
        p("temp_warn", 55.0)              # C: shown on the station viewer
        p("temp_resume", 60.0)            # C: no engaging while a servo is this hot
        p("temp_stop", 65.0)              # C: disengage (the servos' own cutoff is 70)
        p("hold_error", 0.05)             # rad
        gp = self.get_parameter
        self.mode = gp("mapping").value
        if self.mode not in ("degrees", "range"):
            raise ValueError("mapping must be degrees or range")
        self.period = 1.0 / gp("rate_hz").value
        self.ramp_s = gp("ramp_s").value
        self.engage_button = gp("engage_button").value
        self.stall_error, self.stall_s = gp("stall_error").value, gp("stall_s").value
        self.temp_warn, self.temp_resume, self.temp_stop = (gp(n).value for n in ("temp_warn", "temp_resume", "temp_stop"))
        self.hold_error = gp("hold_error").value

        dims_file = os.path.join(get_package_share_directory("xlerobot_description"),
                                 "config", "robot_dimensions.yaml")
        dims = yaml.safe_load(open(dims_file))["xlerobot"]["arms"]
        follower = json.load(open(gp("follower_calibration").value))
        # With the arm folded the wrist meets the upper arm bending back: keep elbow + wrist above this
        # (robot_dimensions.yaml elbow_plus_wrist_min, measured 2026-11-16, plus its margin)
        self.elbow_plus_wrist_min = dims["elbow_plus_wrist_min"] + dims["elbow_plus_wrist_margin"]

        self.arms = {}
        for side in gp("arms").value:
            port = find_port(gp(f"{side}_serial").value)
            if port is None:
                raise RuntimeError(f"{side} leader: no serial port with serial {gp(f'{side}_serial').value}")
            bus = LeaderBus(port)
            arm = {"bus": bus, "joints": {}, "failures": 0}
            for j in ALL_JOINTS:
                jd = dims["joints"][j]
                m = -jd["direction"]   # the model's limits, as in ros2_control.xacro but without limit_margin
                lo = min(m * jd["lower"], m * jd["upper"])
                hi = max(m * jd["lower"], m * jd["upper"])
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
        self.last_target = {}         # joint name -> the last position sent
        self.stall_since = {}         # joint name -> when it went further than stall_error from its command
        self.temps = {}               # joint name -> servo temperature, C
        self.temps_at = 0.0
        self.dynamic_raw = None       # the newest /dynamic_joint_states, still serialized
        self.fault = ""               # why the guard last disengaged, "" after a clean engage
        self.fault_seq = 0
        self.last_status = None
        # ~/engaged, latched: whether the leaders are driving the arms (the station viewer shows it)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.engaged_pub = self.create_publisher(Bool, "~/engaged", latched)
        self.engaged_pub.publish(Bool(data=False))
        self.status_pub = self.create_publisher(String, "~/status", latched)
        # the robot's own guard (servo_guard, with bringup): engaging resets it, so controllers it locked out after a
        # stall come back. A topic, not a service, so it gets from rosie to elroy.
        self.guard_pub = self.create_publisher(String, "/servo_guard/command", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        # 100 Hz and large: kept serialized here, decoded twice a second in check_temps
        self.create_subscription(DynamicJointState, "/dynamic_joint_states", self.on_dynamic, 10, raw=True)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_service(SetBool, "~/engage", self.on_engage)
        self.create_timer(self.period, self.tick)
        self.create_timer(0.5, self.check_temps)
        self.publish_status()
        self.get_logger().info(
            f"leader arms up ({self.mode} mapping), DISENGAGED - call "
            f"`ros2 service call /leader_arms/engage std_srvs/srv/SetBool \"{{data: true}}\"`"
            + (f" or tap /joy button {self.engage_button}" if self.engage_button >= 0 else ""))

    def on_joint_states(self, msg):
        self.follower_state.update(zip(msg.name, msg.position))

    def on_dynamic(self, data):
        self.dynamic_raw = data

    @staticmethod
    def short(name):
        """right_arm_wrist_flex_joint -> right wrist_flex"""
        side, _, rest = name.partition("_arm_")
        return f"{side} {rest.removesuffix('_joint')}"

    def check_temps(self):
        data, self.dynamic_raw = self.dynamic_raw, None
        now = time.monotonic()
        if data is not None:
            msg = deserialize_message(data, DynamicJointState)
            ours = {j["name"] for a in self.arms.values() for j in a["joints"].values()}
            for name, iv in zip(msg.joint_names, msg.interface_values):
                if name in ours and "temperature" in iv.interface_names:
                    self.temps[name] = iv.values[iv.interface_names.index("temperature")]
            self.temps_at = now
        elif now - self.temps_at > 5.0:
            self.temps = {}           # none for a while (bringup down): don't act on old readings
        over = [n for n, t in self.temps.items() if t >= self.temp_stop]
        if over and self.engaged:
            name = max(over, key=self.temps.get)
            relax = {name.split("_arm_")[0]} if name.endswith("_gripper_joint") else set()
            self.trip(f"{self.short(name)} at {self.temps[name]:.0f} C (the guard stops at {self.temp_stop:.0f} C)",
                      relax_grippers=relax)
        self.publish_status()

    def publish_status(self):
        status = {
            "engaged": self.engaged,
            "fault": self.fault,
            "fault_seq": self.fault_seq,
            "hot": {self.short(n): round(t) for n, t in sorted(self.temps.items()) if t >= self.temp_warn},
        }
        if status != self.last_status:
            self.status_pub.publish(String(data=json.dumps(status)))
            self.last_status = status

    def trip(self, why, relax_grippers=()):
        self.fault = why
        self.fault_seq += 1
        self.get_logger().error(f"guard: {why} - disengaging")
        self.set_engaged(False, relax_grippers=relax_grippers)

    def hold_where_stuck(self, relax_grippers=()):
        """Joints further than hold_error from their last command hold where they are measured instead."""
        for side, arm in self.arms.items():
            groups = [(arm["arm_pub"], ARM_JOINTS)]
            if side in relax_grippers:
                groups.append((arm["grip_pub"], ("gripper",)))
            for pub, joints in groups:
                names = [arm["joints"][j]["name"] for j in joints]
                held, positions = [], []
                for n in names:
                    last, measured = self.last_target.get(n), self.follower_state.get(n)
                    if last is None or measured is None:
                        break
                    if abs(last - measured) > self.hold_error or joints == ("gripper",):
                        held.append(self.short(n))
                        last = measured
                    positions.append(float(last))
                if len(positions) < len(names) or not held:
                    continue
                msg = JointTrajectory()
                msg.joint_names = names
                pt = JointTrajectoryPoint()
                pt.positions = positions
                pt.time_from_start.nanosec = 200_000_000
                msg.points = [pt]
                pub.publish(msg)
                for n, pos in zip(names, positions):
                    self.last_target[n] = pos
                self.get_logger().info(f"holding where they are: {', '.join(held)}")

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

    def set_engaged(self, on, relax_grippers=()):
        if on and not self.engaged:
            names = [j["name"] for a in self.arms.values() for j in a["joints"].values()]
            missing = [n for n in names if n not in self.follower_state]
            if missing:
                why = f"no follower state for {missing[0]} (is bringup with_arms:=true running?)"
                self.get_logger().error(why)
                return False, why
            hot = [n for n, t in self.temps.items() if t >= self.temp_resume]
            if hot:
                name = max(hot, key=self.temps.get)
                why = (f"not engaging: {self.short(name)} is at {self.temps[name]:.0f} C, "
                       f"let it cool below {self.temp_resume:.0f} C")
                self.get_logger().warn(why)
                return False, why
            self.guard_pub.publish(String(data="reset"))
            self.start_pose = {n: self.follower_state[n] for n in names}
            self.engaged_at = time.monotonic()
            self.stall_since = {}
            self.fault = ""
        was = self.engaged
        self.engaged = on
        self.engaged_pub.publish(Bool(data=on))
        if was and not on:
            self.hold_where_stuck(relax_grippers)
        self.publish_status()
        why = "engaged" if on else "disengaged"
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
            if self.stalled(side, arm, blend):
                return
            target = {}
            for j, jc in arm["joints"].items():
                mode = "range" if j == "gripper" else self.mode
                step = map_step(pos[IDS[j]], jc["lcal"], jc["fcal"], mode)
                angle = min(max(step_to_angle(step, jc["zero"]), jc["lo"]), jc["hi"])
                start = self.start_pose[jc["name"]]
                target[j] = start + blend * (angle - start)
            # folded, the wrist bending back meets the upper arm: lift it to where it clears
            target["wrist_flex"] = max(target["wrist_flex"], self.elbow_plus_wrist_min - target["elbow_flex"])
            for j, jc in arm["joints"].items():
                self.last_target[jc["name"]] = target[j]
            for pub, joints in ((arm["arm_pub"], ARM_JOINTS), (arm["grip_pub"], ("gripper",))):
                msg = JointTrajectory()
                msg.joint_names = [arm["joints"][j]["name"] for j in joints]
                pt = JointTrajectoryPoint()
                pt.positions = [float(target[j]) for j in joints]
                pt.time_from_start.nanosec = stamp_ns
                msg.points = [pt]
                pub.publish(msg)

    def stalled(self, side, arm, blend):
        """True, after disengaging, if one of the arm's joints has stayed further than stall_error from the last
        command sent, without getting any closer, for stall_s. A joint that trails a fast leader keeps moving, so it
        isn't stuck. Not during the engage blend, when the whole arm may be on its way somewhere."""
        if blend < 1.0:
            self.stall_since = {}
            return False
        now = time.monotonic()
        for j in ARM_JOINTS:
            name = arm["joints"][j]["name"]
            last, measured = self.last_target.get(name), self.follower_state.get(name)
            if last is None or measured is None or abs(last - measured) <= self.stall_error:
                self.stall_since.pop(name, None)
                continue
            since, where = self.stall_since.get(name, (now, measured))
            if abs(measured - where) > self.stall_error / 2:   # still moving: start over from here
                since, where = now, measured
            self.stall_since[name] = (since, where)
            if now - since >= self.stall_s:
                self.trip(f"{side} {j} stuck {math.degrees(abs(last - measured)):.0f} deg short of its command "
                          f"for {self.stall_s:.0f} s")
                return True
        return False

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
