#!/usr/bin/env python3
# Device detection and udev rule generation ported from RoboCrew
# (github.com/Grigorij-Dudnik/RoboCrew, robocrew_setup_usb_modules.py and
# robocrew_generate_udev_rules.py), MIT licensed:
#
#   MIT License
#   Copyright (c) 2025 Grigorij Dudnik
#   Permission is hereby granted, free of charge, to any person obtaining a
#   copy of this software and associated documentation files (the
#   "Software"), to deal in the Software without restriction, including
#   without limitation the rights to use, copy, modify, merge, publish,
#   distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject to
#   the following conditions:
#   The above copyright notice and this permission notice shall be included
#   in all copies or substantial portions of the Software.
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
#   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
#   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
#   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
#   CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
#   TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# The WiFi-priority feature in the original robocrew_setup_usb_modules.py
# is not carried over, it has nothing to do with USB device naming.
"""Assigns stable /dev names to the XLeRobot's USB serial buses and cameras,
so /dev/ttyACM0 and /dev/video0 swapping on reboot or re-plug stops mattering.
See README.md's bus layout section.

Aliases: xle-bus1, xle-bus2 (servo buses), xle-cam-head, xle-cam-left,
xle-cam-right (cameras, optional). Anything else can be added by name.

Rules match a device's serial number where it has a unique one, so the device
keeps its name in any USB port. Devices without a usable serial, or sharing
one with an identical device (two of the same webcam), are bound to their USB
port instead, so moving those cables swaps their names.

Usage:
    python3 tools/xlerobot_setup_usb_modules.py --list
    python3 tools/xlerobot_setup_usb_modules.py --assign xle-bus2=/dev/ttyACM1 \\
        --assign xle-cam-left=/dev/video2 [--dry-run]
    python3 tools/xlerobot_setup_usb_modules.py --identify-cameras [ALIAS ...]
    python3 tools/xlerobot_setup_usb_modules.py            # interactive

--assign names devices that are already connected, no unplugging. --dry-run
prints the rules and writes nothing. --identify-cameras asks you to cover each
camera's lens in turn and picks the node whose image gets darker (needs
OpenCV). The interactive mode asks to disconnect everything, then connect each
device in turn.

Rules are merged into /etc/udev/rules.d/99-xlerobot.rules (an alias that is
assigned again replaces its old rule, the others are kept) and udev is
reloaded. Writing needs root, the script re-runs itself with sudo.
"""

import argparse
import glob
import os
import re
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

MODE = os.environ.get("MODE", "0666")
GROUP = os.environ.get("GROUP", "dialout")
RULES_PATH = os.environ.get("RULES_PATH", "/etc/udev/rules.d/99-xlerobot.rules")

DEFAULT_ALIASES = ["xle-bus1", "xle-bus2"]
# factory default serials that many identical devices share (a bare CP2102 reports 0001), a
# rule on one of these would match every such device, so they are bound by USB port instead
GENERIC_SERIALS = {"00000000", "0", "0000", "0001", "12345678"}
CAMERA_ALIASES = ["xle-cam-head", "xle-cam-left", "xle-cam-right"]


def udevadm(args):
    try:
        out = subprocess.run(
            ["udevadm", *args],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except FileNotFoundError as exc:
        raise SystemExit("udevadm binary is required to run this script") from exc
    return out.stdout.strip()


def get_props(node):
    props = {}
    for line in udevadm(["info", "-q", "property", "-n", node]).splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            props[key] = value
    return props


def get_camera_index(kernel):
    idx_file = Path("/sys/class/video4linux") / kernel / "index"
    if idx_file.is_file():
        return idx_file.read_text(encoding="ascii", errors="ignore").strip()
    return None


def device_info(real_device, subsystem):
    sysfs_path = udevadm(["info", "-q", "path", "-n", real_device])
    if not sysfs_path:
        return None
    props = get_props(real_device)
    vendor_id = props.get("ID_VENDOR_ID", "").lower()
    if not vendor_id:
        return None
    product_id = props.get("ID_MODEL_ID", "").lower()

    phys_dir = os.path.dirname(os.path.dirname(sysfs_path))
    phys_path = os.path.basename(phys_dir).split(":", 1)[0]
    serial_short = props.get("ID_SERIAL_SHORT")
    return {
        "kernel": os.path.basename(real_device),
        "serial": serial_short or props.get("ID_SERIAL") or None,
        "serial_is_short": bool(serial_short),
        "vendor": vendor_id,
        "product": product_id,
        "id_path": props.get("ID_PATH") or phys_path,
        "phys": phys_path,
        "subsystem": subsystem,
    }


def scan(pattern, subsystem, devices, camera_ids):
    for device in sorted(glob.glob(pattern)):
        real_device = os.path.realpath(device)
        dev = device_info(real_device, subsystem)
        if dev is None:
            continue
        cam_key = f'{dev["vendor"]}:{dev["product"]}'

        if subsystem == "video4linux":
            camera_ids.add(cam_key)
            idx = get_camera_index(os.path.basename(real_device))
            if idx and idx != "0":
                continue
        elif cam_key in camera_ids:
            continue

        # several by-path symlinks can point at the same node
        if any(d["subsystem"] == subsystem and d["kernel"] == dev["kernel"] for d in devices):
            continue
        devices.append(dev)


def capture_devices():
    devices = []
    camera_ids = set()
    scan("/dev/v4l/by-path/*", "video4linux", devices, camera_ids)
    scan("/dev/serial/by-path/*", "tty", devices, camera_ids)
    scan("/dev/snd/controlC*", "sound", devices, camera_ids)
    return devices


def serial_counts(devices):
    return Counter(dev["serial"] for dev in devices if dev.get("serial"))


def device_key(dev):
    return (dev["subsystem"], dev["kernel"], dev["phys"], dev.get("serial"))


def wait_for_device(known):
    print("Waiting for newly connected device...")
    while True:
        for dev in capture_devices():
            key = device_key(dev)
            if key not in known:
                return dev, key
        time.sleep(1)


def build_rule(dev, alias, counts):
    rule = f'SUBSYSTEM=="{dev["subsystem"]}"'
    if dev["subsystem"] == "video4linux":
        rule += ', ATTR{index}=="0"'
    serial = dev.get("serial")
    if serial and serial not in GENERIC_SERIALS and counts.get(serial, 0) == 1:
        if dev.get("serial_is_short"):
            rule += (f', ATTRS{{idVendor}}=="{dev["vendor"]}"'
                     f', ATTRS{{idProduct}}=="{dev["product"]}"'
                     f', ATTRS{{serial}}=="{serial}"')
        else:
            rule += f', ENV{{ID_SERIAL}}=="{serial}"'
    else:
        rule += f', ENV{{ID_PATH}}=="{dev.get("id_path") or dev["phys"]}"'
    rule += f', MODE="{MODE}", GROUP="{GROUP}", SYMLINK+="{alias}"'
    return rule


def alias_of(rule):
    match = re.search(r'SYMLINK\+="([^"]+)"', rule)
    return match.group(1) if match else None


def ensure_root():
    if os.geteuid() == 0:
        return
    print("Writing udev rules needs root, requesting sudo...")
    cmd = ["sudo", "-E", sys.executable, *sys.argv]
    os.execvp("sudo", cmd)


def write_rules(assignments, dry_run):
    counts = serial_counts(capture_devices())
    new_rules = {a["alias"]: build_rule(a["device"], a["alias"], counts) for a in assignments}
    if dry_run:
        print("\n".join(new_rules.values()))
        return

    ensure_root()
    existing = []
    if Path(RULES_PATH).is_file():
        existing = Path(RULES_PATH).read_text(encoding="ascii").splitlines()
    kept = [line for line in existing if line.strip() and alias_of(line) not in new_rules]
    with open(RULES_PATH, "w", encoding="ascii") as fh:
        fh.write("\n\n".join(kept + list(new_rules.values())) + "\n")
    print(f"Saved {len(new_rules)} rule(s) to {RULES_PATH}, kept {len(kept)} existing.")
    subprocess.run(["udevadm", "control", "--reload-rules"], check=False)
    # --action=add: a plain trigger sends "change" events, and on the Jetson the ID_PATH rules
    # did not create their symlinks on a change, only on add (99-cameras-elroy.rules notes)
    subprocess.run(["udevadm", "trigger", "--action=add", "--subsystem-match=tty",
                    "--subsystem-match=video4linux"], check=False)
    print("Done. /dev/" + " and /dev/".join(new_rules) + " should now be stable across reboots.")


def list_devices():
    counts = serial_counts(capture_devices())
    for dev in capture_devices():
        serial = dev.get("serial") or "-"
        unique = serial != "-" and serial not in GENERIC_SERIALS and counts[serial] == 1
        binding = "serial" if unique else "USB port"  # the same test as build_rule
        print(f'/dev/{dev["kernel"]:<10} {dev["subsystem"]:<11} {dev["vendor"]}:{dev["product"]}  '
              f'serial={serial}  port={dev["id_path"]}  (rule binds by {binding})')


def assign_from_args(pairs):
    assignments = []
    for pair in pairs:
        alias, _, node = pair.partition("=")
        if not alias or not node:
            raise SystemExit(f"--assign expects ALIAS=/dev/node, got '{pair}'")
        real = os.path.realpath(node)
        subsystem = get_props(real).get("SUBSYSTEM")
        dev = device_info(real, subsystem) if subsystem else None
        if dev is None:
            raise SystemExit(f"cannot read device attributes for {node}")
        assignments.append({"alias": alias, "device": dev})
    return assignments


def brightness(node, frames=8, warmup=5):
    import cv2

    cap = cv2.VideoCapture(node, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    values = []
    for i in range(warmup + frames):
        ok, frame = cap.read()
        if ok and i >= warmup:
            values.append(float(frame.mean()))
    cap.release()
    return sum(values) / len(values) if values else None


def identify_cameras(aliases):
    try:
        import cv2  # noqa: F401
    except ImportError as exc:
        raise SystemExit("--identify-cameras needs OpenCV (python3-opencv)") from exc

    cameras = {f'/dev/{d["kernel"]}': d for d in capture_devices() if d["subsystem"] == "video4linux"}
    baseline = {node: brightness(node) for node in cameras}
    baseline = {node: b for node, b in baseline.items() if b is not None}
    if not baseline:
        raise SystemExit("no readable cameras found")
    print("Cameras:", ", ".join(sorted(baseline)))

    assignments = []
    for alias in aliases:
        resp = input(f"Cover the lens of '{alias}' with your hand, then press Enter (or 's' to skip): ")
        if resp.strip().lower() == "s":
            continue
        drops = {}
        for node, base in baseline.items():
            if any(a["device"] is cameras[node] for a in assignments):
                continue
            now = brightness(node)
            if now is not None:
                drops[node] = (base - now) / max(base, 1.0)
        ranked = sorted(drops.items(), key=lambda kv: kv[1], reverse=True)
        if not ranked or ranked[0][1] < 0.3 or (len(ranked) > 1 and ranked[1][1] > 0.3):
            print(f"Could not tell which camera is '{alias}' (drops: "
                  + ", ".join(f"{n} {d:.0%}" for n, d in ranked) + "), skipping.")
            continue
        node, drop = ranked[0]
        print(f"'{alias}' is {node} (image got {drop:.0%} darker)")
        assignments.append({"alias": alias, "device": cameras[node]})
    return assignments


def interactive():
    ensure_root()
    input("Disconnect all XLeRobot USB buses and cameras, then press Enter to continue.")

    known = {device_key(dev) for dev in capture_devices()}
    assignments = []

    for alias in DEFAULT_ALIASES + CAMERA_ALIASES:
        optional = alias in CAMERA_ALIASES
        resp = input(f"Connect the USB cable for '{alias}' and press Enter (or 's' to skip"
                     f"{', optional' if optional else ''}): ").strip().lower()
        if resp == "s":
            continue
        dev, key = wait_for_device(known)
        assignments.append({"alias": alias, "device": dev})
        known.add(key)

    while True:
        resp = input("Type 'a' to add another device (lidar, mic), or press Enter to finish: ").strip().lower()
        if resp != "a":
            break
        alias = input("Enter alias for the new device: ").strip()
        if not alias:
            print("Alias cannot be empty. Skipping.")
            continue
        dev, key = wait_for_device(known)
        assignments.append({"alias": alias, "device": dev})
        known.add(key)

    if not assignments:
        print("No devices registered, nothing to write.")
        return
    write_rules(assignments, dry_run=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--list", action="store_true", help="show connected devices and how a rule would bind them")
    parser.add_argument("--assign", action="append", default=[], metavar="ALIAS=/dev/node",
                        help="name an already connected device, repeatable")
    parser.add_argument("--identify-cameras", nargs="*", metavar="ALIAS",
                        help=f"find cameras by covering their lens (default: {' '.join(CAMERA_ALIASES)})")
    parser.add_argument("--dry-run", action="store_true", help="print the rules, write nothing")
    args = parser.parse_args()

    if args.list:
        list_devices()
    elif args.identify_cameras is not None:
        assignments = identify_cameras(args.identify_cameras or CAMERA_ALIASES)
        if assignments:
            write_rules(assignments, args.dry_run)
    elif args.assign:
        write_rules(assign_from_args(args.assign), args.dry_run)
    else:
        interactive()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
