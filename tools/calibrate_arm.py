#!/usr/bin/env python3
"""Calibrate one SO-101 arm, or the head pan/tilt, the way lerobot does, and save the result in lerobot's format.

  status     read-only: positions, stored offsets and limits, torque, and how they compare to the file
  torque-off write torque_enable=0 to every servo of the part (--part all: both buses, arms, head and
             wheels) and read it back. For after the stack was stopped: the ROS hardware interface
             leaves the servos holding when its process is killed. The arms go limp, hold them.
  calibrate  middle-of-range pose, then a sweep of every joint (lerobot's `calibrate()` for
             so101_follower): writes Homing_Offset and the position limits into the servos and
             <calibration dir>/robots/so101_follower/<id>.json

The head (pan 7, tilt 8, on bus 1 with the left arm) uses the same procedure and file schema, with
the "middle" being the head looking straight ahead and level. lerobot has no robot type for it, the
file goes to robots/xlerobot_head/.

Talks to the servo bus directly, so the ROS stack must be stopped (the tool refuses if another
process holds the port). Only the servo IDs of the chosen part are touched, so the rest of the
bus (the other part, the wheels 9, 10) is left alone. Torque stays off.

The file is the same one `lerobot-calibrate --robot.type=so101_follower --robot.id=<id>` writes,
so lerobot picks it up from the same place (HF_LEROBOT_HOME, default ~/.cache/huggingface/lerobot).

Usage:
  python3 tools/calibrate_arm.py status    --arm left
  python3 tools/calibrate_arm.py torque-off --part all
  python3 tools/calibrate_arm.py calibrate --arm left
  python3 tools/calibrate_arm.py calibrate --arm right --port /dev/xle-bus2
  python3 tools/calibrate_arm.py calibrate --arm head
"""

import argparse
import glob
import json
import os
import pathlib
import sys
import threading
import time

import serial

SO101_JOINTS = {  # name -> servo ID, same names and order as lerobot's SO101Follower
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

# part -> port, joints, joint stored as a full turn instead of swept (lerobot does that for the
# wrist roll), lerobot robot type (the directory). Left arm + head on bus 1, right arm + wheels on
# bus 2 (xlerobot.ros2_control.xacro).
PARTS = {
    "left": ("/dev/xle-bus1", SO101_JOINTS, "wrist_roll", "so101_follower"),
    "right": ("/dev/xle-bus2", SO101_JOINTS, "wrist_roll", "so101_follower"),
    "head": ("/dev/xle-bus1", {"head_pan": 7, "head_tilt": 8}, None, "xlerobot_head"),
    "wheels": ("/dev/xle-bus2", {"left_wheel": 9, "right_wheel": 10}, None, "xlerobot_wheels"),
}
BAUD = 1_000_000
MAX_RES = 4095  # STS3215, 12 bit
HALF_TURN = MAX_RES // 2  # 2047

# STS3215 control table (address, size)
MIN_POSITION_LIMIT = (9, 2)
MAX_POSITION_LIMIT = (11, 2)
HOMING_OFFSET = (31, 2)  # sign-magnitude, sign bit 11
OPERATING_MODE = (33, 1)
TORQUE_ENABLE = (40, 1)
LOCK = (55, 1)
PRESENT_POSITION = (56, 2)  # sign-magnitude, sign bit 15
PRESENT_VOLTAGE = (62, 1)
PRESENT_TEMPERATURE = (63, 1)

INST_PING, INST_READ, INST_WRITE = 0x01, 0x02, 0x03


class BusError(RuntimeError):
    pass


def encode_sign_magnitude(value, sign_bit):
    if abs(value) > (1 << sign_bit) - 1:
        raise ValueError(f"{value} does not fit in {sign_bit} bits")
    return ((1 if value < 0 else 0) << sign_bit) | abs(value)


def decode_sign_magnitude(value, sign_bit):
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if value & (1 << sign_bit) else magnitude


class Bus:
    """Feetech STS protocol 0 over a serial port, just what calibration needs."""

    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.05)

    def close(self):
        self.ser.close()

    def _txrx(self, motor_id, instruction, params=b"", retries=3):
        length = len(params) + 2
        body = bytes([motor_id, length, instruction]) + bytes(params)
        packet = b"\xff\xff" + body + bytes([~sum(body) & 0xFF])
        for _ in range(retries):
            self.ser.reset_input_buffer()
            self.ser.write(packet)
            reply = self._read_reply(motor_id)
            if reply is not None:
                return reply[1]
        raise BusError(f"servo {motor_id} did not answer")

    def _read_reply(self, motor_id):
        head = self.ser.read(4)  # ff ff id len
        if len(head) < 4 or head[:2] != b"\xff\xff" or head[2] != motor_id:
            return None
        rest = self.ser.read(head[3])  # error, params..., checksum
        if len(rest) < head[3]:
            return None
        body = head[2:] + rest[:-1]
        if (~sum(body) & 0xFF) != rest[-1]:
            return None
        return rest[0], rest[1:-1]

    def ping(self, motor_id):
        try:
            self._txrx(motor_id, INST_PING)
            return True
        except BusError:
            return False

    def read(self, motor_id, reg, sign_bit=None):
        addr, size = reg
        data = self._txrx(motor_id, INST_READ, bytes([addr, size]))
        value = int.from_bytes(data[:size], "little")
        return decode_sign_magnitude(value, sign_bit) if sign_bit else value

    def write(self, motor_id, reg, value, sign_bit=None):
        addr, size = reg
        if sign_bit:
            value = encode_sign_magnitude(value, sign_bit)
        self._txrx(motor_id, INST_WRITE, bytes([addr]) + value.to_bytes(size, "little"))

    def position(self, motor_id):
        return self.read(motor_id, PRESENT_POSITION, sign_bit=15)


def port_holders(port):
    """PIDs that have the (resolved) serial device open."""
    target = os.path.realpath(port)
    holders = []
    for fd in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            if os.path.realpath(fd) == target:
                pid = int(fd.split("/")[2])
                cmd = pathlib.Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ")
                holders.append((pid, cmd.decode(errors="replace")[:100]))
        except (OSError, ValueError):
            continue
    return [h for h in holders if h[0] != os.getpid()]


def calibration_path(robot_id, calibration_dir, robot_type):
    if calibration_dir:
        directory = pathlib.Path(calibration_dir)
    else:
        home = pathlib.Path(os.environ.get("HF_LEROBOT_HOME", "~/.cache/huggingface/lerobot"))
        directory = home / "calibration" / "robots" / robot_type
    return directory.expanduser() / f"{robot_id}.json"


def open_bus(port, joints):
    holders = port_holders(port)
    if holders:
        lines = "\n".join(f"  pid {pid}: {cmd}" for pid, cmd in holders)
        sys.exit(f"{port} is open by another process, stop it first (the ROS bringup?):\n{lines}")
    bus = Bus(port)
    missing = [name for name, i in joints.items() if not bus.ping(i)]
    if missing:
        bus.close()
        sys.exit(f"no answer from {', '.join(missing)} on {port}. Wrong --port, or arm power off?")
    return bus


def cmd_status(args):
    path = calibration_path(args.id, args.calibration_dir, args.robot_type)
    saved = json.loads(path.read_text()) if path.is_file() else None
    bus = open_bus(args.port, args.joints)
    print(f"{args.port}, calibration file: {path} ({'found' if saved else 'not found'})")
    print(f"{'joint':<14}{'id':>3}{'pos':>6}{'deg':>8}{'offset':>8}{'min':>6}{'max':>6}"
          f"{'V':>6}{'C':>4}{'torque':>7}   file")
    for name, i in args.joints.items():
        pos = bus.position(i)
        offset = bus.read(i, HOMING_OFFSET, sign_bit=11)
        lo, hi = bus.read(i, MIN_POSITION_LIMIT), bus.read(i, MAX_POSITION_LIMIT)
        volts = bus.read(i, PRESENT_VOLTAGE) / 10
        temp = bus.read(i, PRESENT_TEMPERATURE)
        if saved is None:
            match = "-"
        else:
            c = saved.get(name, {})
            same = (c.get("homing_offset"), c.get("range_min"), c.get("range_max")) == (offset, lo, hi)
            match = "same as servo" if same else "DIFFERS from servo"
        torque = "on" if bus.read(i, TORQUE_ENABLE) else "off"
        print(f"{name:<14}{i:>3}{pos:>6}{(pos - HALF_TURN) * 360 / 4096:>8.1f}{offset:>8}"
              f"{lo:>6}{hi:>6}{volts:>6.1f}{temp:>4}{torque:>7}   {match}")
    print("deg is relative to the calibrated middle (step 2047)")
    bus.close()


def cmd_torque_off(args):
    parts = list(PARTS) if args.part == "all" else [args.part]
    failed = False
    for part in parts:
        port, joints = (args.port if args.part == part and args.port else PARTS[part][0]), PARTS[part][1]
        bus = open_bus(port, joints)
        for name, i in joints.items():
            bus.write(i, TORQUE_ENABLE, 0)
            state = bus.read(i, TORQUE_ENABLE)
            failed |= state != 0
            print(f"{part:<7}{name:<14}id {i:>2}  torque {'OFF' if state == 0 else 'STILL ON'}")
        bus.close()
    if failed:
        sys.exit("some servos still hold torque")


def wait_for_enter():
    pressed = threading.Event()
    threading.Thread(target=lambda: (sys.stdin.readline(), pressed.set()), daemon=True).start()
    return pressed


def cmd_calibrate(args):
    path = calibration_path(args.id, args.calibration_dir, args.robot_type)
    bus = open_bus(args.port, args.joints)
    ids = list(args.joints.values())

    for i in ids:
        bus.write(i, TORQUE_ENABLE, 0)
        bus.write(i, LOCK, 0)
        bus.write(i, OPERATING_MODE, 0)
    print(f"Torque is off on the {args.part} (servo IDs {ids[0]}-{ids[-1]} on {args.port}).")

    if args.part == "head":
        input("Put the head looking straight ahead and level, then press ENTER... ")
    else:
        input(f"Move the {args.part} arm to the middle of its range of motion and press ENTER... ")

    # reset, then make the present position exactly a half turn, as lerobot's set_half_turn_homings
    for i in ids:
        bus.write(i, HOMING_OFFSET, 0, sign_bit=11)
        bus.write(i, MIN_POSITION_LIMIT, 0)
        bus.write(i, MAX_POSITION_LIMIT, MAX_RES)
    time.sleep(0.05)
    offsets = {}
    for name, i in args.joints.items():
        offsets[name] = bus.position(i) - HALF_TURN
        bus.write(i, HOMING_OFFSET, offsets[name], sign_bit=11)
    time.sleep(0.05)
    for name, i in args.joints.items():
        pos = bus.position(i)
        if abs(pos - HALF_TURN) > 20:  # about 2 degrees, the arm is held by hand
            sys.exit(f"{name} reads {pos} after homing, expected about {HALF_TURN}. Nothing saved.")

    print("Move all joints sequentially through their entire ranges of motion.\n"
          "Recording positions. Press ENTER to stop...")
    swept = [n for n in args.joints if n != args.full_turn]
    start = {n: bus.position(args.joints[n]) for n in swept}
    mins, maxes = dict(start), dict(start)
    done = wait_for_enter()
    first = True
    while not done.is_set():
        pos = {n: bus.position(args.joints[n]) for n in swept}
        for n in swept:
            mins[n], maxes[n] = min(mins[n], pos[n]), max(maxes[n], pos[n])
        if not first:
            print(f"\033[{len(swept) + 2}A", end="")
        first = False
        print(f"{'NAME':<15}| {'MIN':>6} | {'POS':>6} | {'MAX':>6}\033[K")
        for n in swept:
            print(f"{n:<15}| {mins[n]:>6} | {pos[n]:>6} | {maxes[n]:>6}\033[K")
        print("\033[K", end="")
        time.sleep(0.02)
    if args.full_turn:
        mins[args.full_turn], maxes[args.full_turn] = 0, MAX_RES

    still = [n for n in swept if mins[n] == maxes[n]]
    if still:
        sys.exit(f"{', '.join(still)} did not move. Nothing saved, the offsets stay written, run again.")

    calibration = {
        name: {
            "id": i,
            "drive_mode": 0,
            "homing_offset": offsets[name],
            "range_min": mins[name],
            "range_max": maxes[name],
        }
        for name, i in args.joints.items()
    }
    for name, i in args.joints.items():
        bus.write(i, MIN_POSITION_LIMIT, mins[name])
        bus.write(i, MAX_POSITION_LIMIT, maxes[name])
        bus.write(i, LOCK, 1)
    bus.close()

    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_file():
        path.replace(path.with_suffix(".json.bak"))
    path.write_text(json.dumps(calibration, indent=4) + "\n")
    print(f"Calibration saved to {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["status", "calibrate", "torque-off"])
    parser.add_argument("--part", "--arm", dest="part", choices=list(PARTS) + ["all"], required=True,
                        help="left or right arm, the head, the wheels, or all (torque-off only)")
    parser.add_argument("--port", help="default: /dev/xle-bus1 for left arm and head, /dev/xle-bus2 for right")
    parser.add_argument("--id", help="lerobot robot id, the file name; default xlerobot_<part>_arm, xlerobot_head")
    parser.add_argument("--calibration-dir",
                        help="default: $HF_LEROBOT_HOME/calibration/robots/so101_follower (xlerobot_head for the head)")
    args = parser.parse_args()
    if args.command == "torque-off":
        return cmd_torque_off(args)
    if args.part == "all":
        sys.exit("--part all only works with torque-off")
    port, args.joints, args.full_turn, args.robot_type = PARTS[args.part]
    args.port = args.port or port
    args.id = args.id or ("xlerobot_head" if args.part == "head" else f"xlerobot_{args.part}_arm")
    {"status": cmd_status, "calibrate": cmd_calibrate}[args.command](args)


if __name__ == "__main__":
    main()
