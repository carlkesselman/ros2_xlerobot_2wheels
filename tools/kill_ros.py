#!/usr/bin/env python3
"""Find and kill every process belonging to this robot's ROS2 stack - the "ps aux | grep | kill"
cleanup this session kept doing by hand, several times, on both the Jetson and this laptop.

  python3 tools/kill_ros.py              # list what it would kill, ask to confirm
  python3 tools/kill_ros.py --yes        # kill without asking
  python3 tools/kill_ros.py --dry-run    # just list, never kill, never asks
  python3 tools/kill_ros.py --clean-shm  # also delete stale FastDDS shared-memory files
  python3 tools/kill_ros.py --usb-reset  # also reset the Gemini (2bc5:0670) on USB
  python3 tools/kill_ros.py --usb-reset 2bc5:0670 1234:5678   # reset any VID:PID, repeatable

Run it on whichever machine has the stuck processes - it only ever looks at local processes, no
SSH/multi-host orchestration. Matches every process this project's launch files start: the
bringup stack (robot_state_publisher, ros2_control_node, controller spawners), cameras, lidar,
MoveIt's move_group, the Nav2/SLAM stack, and teleop/safety (joy_node, twist_mux,
safety_supervisor, mcp_presence_guard). Deliberately does NOT match interactive tools you want to
keep running across a cleanup - RViz2, the Rerun viewer, your editor/terminal, ssh sessions, this
script's own `ros2 launch` parent shells are matched (so the whole tree goes), but a bare `ros2`
CLI command (topic echo, node list, ...) is not, since those patterns are intentionally specific
executable names, not a broad "ros2" substring match.

SIGTERM first, give it a few seconds, SIGKILL anything still alive after that - same two-step
this session used by hand every time, because a few of these (ros2_control_node especially) don't
always die cleanly on the first signal.

--clean-shm also closes rviz2 (see CLEAN_SHM_EXTRA) and removes this user's /dev/shm/fastrtps_* and sem.fastrtps_* files, which accumulate
when processes are killed and can break FastDDS's shared-memory transport (symptom:
"RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed"). It only runs once nothing from this
stack is left, and anything else on the machine still using ROS 2 (rviz, a ros2 CLI command)
loses shared memory until it is restarted.

--usb-reset runs `usbreset` on the given devices after the kill, using `sudo -n` when it is not
root. It re-enumerates the device, it does not cut its power.
"""
import argparse
import os
import signal
import subprocess
import sys
import time

# Executable basenames / cmdline substrings, not regexes - matched against each process's full
# command line. One entry per node this project's launch files can start; add to this list
# instead of using a broader pattern if a new launch file starts something new.
PATTERNS = [
    "ros2 launch",  # the launch parent itself, so the whole tree it spawned goes with it
    "component_container",  # composable-node host: orbbec_camera (Gemini) and ldlidar
    "ros2cli.daemon",  # one per ROS_DOMAIN_ID ever used, restarts on demand and caches discovery
    "robot_state_publisher",
    "ros2_control_node",
    "controller_manager/spawner",
    "usb_cam_node_exe",
    "move_group",
    "ldlidar_container",
    "ldlidar_lifecycle_manager",
    "laser_filters/scan_to_scan_filter_chain",  # the LD19's filter chain, robot and MuJoCo twin
    "sim/mujoco/run_mujoco.py",        # the MuJoCo twin (python, or mjpython with its viewer)
    "nav2_map_server",
    "nav2_amcl",
    "nav2_controller",
    "nav2_planner",
    "nav2_behaviors",
    "nav2_bt_navigator",
    "nav2_velocity_smoother",
    "nav2_lifecycle_manager",
    "async_slam_toolbox_node",
    "joy/joy_node",
    "teleop_twist_joy",
    "twist_mux/twist_mux",
    "xlerobot_teleop/safety_supervisor",
    "xlerobot_teleop/mcp_presence_guard",
    "objperc.node",                    # object perception service (run by its venv's python)
    "xlerobot_skills/skill_server",
    "xlerobot_executive/executive",
]

# Deleting the shared-memory files under a ROS process that stays alive breaks that process's
# transport (RTPS_TRANSPORT_SHM errors, nodes that cannot find each other), and the next start then
# fails in odd ways. So --clean-shm also closes viewers like RViz that this script otherwise leaves.
CLEAN_SHM_EXTRA = ["rviz2"]
CLEAN_SHM_EXTRA_ON = False


def find_matches():
    patterns = PATTERNS + (CLEAN_SHM_EXTRA if CLEAN_SHM_EXTRA_ON else [])
    out = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True, check=True).stdout
    my_pid = os.getpid()
    matches = []
    for line in out.splitlines()[1:]:
        line = line.strip()
        if not line:
            continue
        pid_str, _, cmd = line.partition(" ")
        try:
            pid = int(pid_str)
        except ValueError:
            continue
        if pid == my_pid or "kill_ros.py" in cmd:
            continue  # never target this script's own process
        if cmd.startswith("ssh ") or "nohup bash -c" in cmd:
            continue  # a wrapper that just MENTIONS a pattern in a remote/quoted command string,
            # not the local process itself - confirmed false-positive testing this script:
            # matched this script's own `ssh ... "ros2 launch ..."` call on the machine running
            # the ssh command, not on the machine actually running ros2 launch.
        if any(p in cmd for p in patterns):
            matches.append((pid, cmd))
    return matches


DEFAULT_USB_RESET = "2bc5:0670"  # Orbbec Gemini 2


def _shm_in_use():
    """FastDDS shared-memory files some live process still maps or holds open (the TUI, the MCP server ...)."""
    used = set()
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            with open(f"/proc/{pid}/maps") as f:
                used.update(line.split(None, 5)[5].strip() for line in f if "/dev/shm/" in line and len(line.split(None, 5)) == 6)
            fd_dir = f"/proc/{pid}/fd"
            for fd in os.listdir(fd_dir):
                t = os.readlink(os.path.join(fd_dir, fd))
                if t.startswith("/dev/shm/"):
                    used.add(t)
        except OSError:
            continue
    return used


def _shm_owner(f):
    """The segment or port file a FastDDS file belongs to: fastrtps_X_el and sem.fastrtps_X_mutex go with fastrtps_X."""
    d, b = os.path.split(f)
    return os.path.join(d, b.removeprefix("sem.").removesuffix("_el").removesuffix("_mutex"))


def clean_shm(dry_run):
    import glob
    uid = os.getuid()
    used = {_shm_owner(f) for f in _shm_in_use()}
    files = [f for pat in ("/dev/shm/fastrtps_*", "/dev/shm/sem.fastrtps_*") for f in glob.glob(pat)
             if os.stat(f).st_uid == uid and _shm_owner(f) not in used]
    print(f"{len(files)} stale FastDDS shared-memory file(s) owned by this user ({len(used)} in use by running processes, kept)")
    if dry_run:
        return
    for f in files:
        try:
            os.remove(f)
        except OSError as e:
            print(f"  could not remove {f}: {e}")


def usb_reset(devices, dry_run):
    for dev in devices:
        cmd = ["usbreset", dev] if os.geteuid() == 0 else ["sudo", "-n", "usbreset", dev]
        print("usb reset:", " ".join(cmd))
        if dry_run:
            continue
        r = subprocess.run(cmd, capture_output=True, text=True)
        print("  " + (r.stdout.strip() or r.stderr.strip() or f"exit {r.returncode}"))


def kill_matches(matches):
    pids = [pid for pid, _ in matches]
    print(f"\nsending SIGTERM to {len(pids)} process(es)...")
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    time.sleep(3)

    still_alive = []
    for pid in pids:
        try:
            os.kill(pid, 0)  # signal 0: check it exists, don't actually signal it
            still_alive.append(pid)
        except ProcessLookupError:
            pass

    if still_alive:
        print(f"{len(still_alive)} still alive after SIGTERM, sending SIGKILL: {still_alive}")
        for pid in still_alive:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--yes", action="store_true", help="kill without asking for confirmation")
    parser.add_argument("--dry-run", action="store_true", help="list matches, never kill, never asks")
    parser.add_argument("--clean-shm", action="store_true",
                        help="after killing, delete this user's stale FastDDS /dev/shm files")
    parser.add_argument("--usb-reset", nargs="*", metavar="VID:PID",
                        help=f"after killing, usbreset these devices (default {DEFAULT_USB_RESET})")
    args = parser.parse_args()
    global CLEAN_SHM_EXTRA_ON
    CLEAN_SHM_EXTRA_ON = args.clean_shm
    usb_devices = None if args.usb_reset is None else (args.usb_reset or [DEFAULT_USB_RESET])

    matches = find_matches()
    if matches:
        print(f"found {len(matches)} matching process(es):")
        for pid, cmd in matches:
            print(f"  {pid:>7}  {cmd[:110]}")
    else:
        print("nothing matching this project's ROS2 stack found running.")

    if not args.dry_run and matches:
        if not args.yes:
            answer = input(f"\nkill all {len(matches)}? [y/N] ").strip().lower()
            if answer != "y":
                print("not killing anything.")
                return
        kill_matches(matches)
        remaining = find_matches()
        if remaining:
            print(f"WARNING: {len(remaining)} process(es) still matched after cleanup:")
            for pid, cmd in remaining:
                print(f"  {pid:>7}  {cmd[:110]}")
            sys.exit(1)
        print("done, nothing matching left running.")

    if args.clean_shm:
        if find_matches() and not args.dry_run:
            print("not cleaning shared memory: stack processes are still running.")
        else:
            clean_shm(args.dry_run)
    if usb_devices:
        usb_reset(usb_devices, args.dry_run)


if __name__ == "__main__":
    main()
