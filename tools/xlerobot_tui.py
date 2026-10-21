#!/usr/bin/env python3
"""Console UI to start, stop and check the robot's ROS 2 stack.

  python3 tools/xlerobot_tui.py                  # watch + control the Jetson stack
  python3 tools/xlerobot_tui.py --host jetson --profile sensing

Run it in a shell where ROS 2 Jazzy is sourced (the same shell you run the other tools from).
Needs `textual` (pip install textual); the venv at ~/venvs/xlerobot-mcp has it.

The table shows every component the chosen profile should have running, with its topic rate,
message age and stamp skew (camera stamps 160 s behind the clock emptied the octomap once), the
node and controller state, and a one-line detail. Rows are live: no row is "ok" because it was
once. Start and stop run on the Jetson over ssh (`--host`), the monitoring reads the ROS graph
directly, so it works from any machine on the robot's network.

Motions: r records hand-guided (torque off on both buses, move the arms and head by hand, wheels
are not recorded), R records while you drive with torque on (arms/head by teleop plus the
joystick's wheel commands), y plays one back, z stops playback, m lists recordings. Playback sends the wheels through twist_mux below the
joystick, so touching the stick or e-stop wins. The header line shows the Jetson's CPU, memory,
GPU and temperature, read over the same ssh connection.

Keys: s start stack, x stop stack (kill_ros --clean-shm), X stop and then cut torque on every servo, t cut
torque only (stack must be stopped; the arms go limp and the head droops, hold them; it asks for "yes"), g presence guard on/off, e e-stop,
u release e-stop, c clear octomap, v validate, p next profile, l show/hide log, q quit.
Heavy topics (images, clouds) are never subscribed: a standing remote raw-image subscriber
stalled the Gemini driver, so cameras are judged by their camera_info stream.
"""
import argparse
import collections
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import DataTable, Footer, Header, Input, RichLog, Static

import motion_recorder

REPO_ON_JETSON = "Developer/ros2_xlerobot_2wheels"
TOOLS = Path(__file__).resolve().parent
BEST_EFFORT = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)


@dataclass
class Topic:
    name: str
    type: str
    min_hz: float
    max_skew: float | None = None   # seconds between header stamp and this machine's clock


@dataclass
class Component:
    name: str
    group: str
    nodes: list = field(default_factory=list)          # each entry: a name or a tuple of alternatives
    topics: list = field(default_factory=list)
    controllers: list = field(default_factory=list)
    optional: bool = False                             # never reported as a failure, "off" when absent


COMPONENTS = [
    Component("ros2_control", "base", nodes=["/controller_manager", "/sts_hardware_interface_node_xlerobot_bus1",
                                              "/sts_hardware_interface_node_xlerobot_bus2"],
              topics=[Topic("/joint_states", "sensor_msgs/msg/JointState", 20, 2.0)],
              controllers=["joint_state_broadcaster"]),
    Component("wheels", "base", topics=[Topic("/diff_drive_controller/odom", "nav_msgs/msg/Odometry", 5, 2.0)],
              controllers=["diff_drive_controller"]),
    Component("safety supervisor", "base", nodes=["/xlerobot_safety_supervisor"]),
    Component("joystick teleop", "base", nodes=["/joy_node", "/teleop_twist_joy_node"]),
    Component("left arm", "arms", controllers=["left_arm_controller", "left_gripper_controller"]),
    Component("right arm", "arms", controllers=["right_arm_controller", "right_gripper_controller"]),
    Component("head pan/tilt", "arms", controllers=["head_controller"]),
    Component("head camera", "cameras", nodes=["/cameras/head/camera_container"],
              topics=[Topic("/cameras/head/camera_info", "sensor_msgs/msg/CameraInfo", 15, 2.0)]),
    Component("head depth", "cameras", nodes=["/cameras/head/depth_points_low"],
              topics=[Topic("/cameras/head/depth/camera_info", "sensor_msgs/msg/CameraInfo", 10, 2.0)]),
    Component("left wrist cam", "cameras", nodes=["/cameras/left/usb_cam"],
              topics=[Topic("/cameras/left/camera_info", "sensor_msgs/msg/CameraInfo", 5, 2.0)]),
    Component("right wrist cam", "cameras", nodes=["/cameras/right/usb_cam"],
              topics=[Topic("/cameras/right/camera_info", "sensor_msgs/msg/CameraInfo", 5, 2.0)]),
    Component("lidar", "lidar", nodes=["/ldlidar_node"],
              topics=[Topic("/ldlidar_node/scan", "sensor_msgs/msg/LaserScan", 5, 2.0)]),
    Component("navigation", "nav", nodes=["/planner_server", "/controller_server", "/bt_navigator",
                                          "/behavior_server", "/velocity_smoother",
                                          "/lifecycle_manager_navigation"]),
    Component("MoveIt", "nav", nodes=["/move_group"]),
    Component("presence guard", "presence", nodes=["/mcp_presence_guard"],
              topics=[Topic("/mcp_presence_ok", "std_msgs/msg/Bool", 0.0)], optional=True),
    Component("MCP bridge", "mcp", nodes=["/mcp_bridge"], optional=True),
]

# profile -> (groups that should be running, full_stack.launch.py arguments)
PROFILES = {
    "base": ({"base", "nav"}, ""),
    "arms": ({"base", "nav", "arms"}, "with_arms:=true with_head:=true"),
    "sensing": ({"base", "nav", "arms", "cameras", "lidar"},
                "with_arms:=true with_head:=true cameras:=true sensor:=gemini lidar:=true"),
}

OK, WARN, FAIL, OFF = "ok", "warn", "FAIL", "off"
STYLE = {OK: "green", WARN: "yellow", FAIL: "bold red", OFF: "dim"}


class Monitor:
    """rclpy node in a background thread: topic rates/ages, node list, controller states."""

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("xlerobot_tui")
        self.arrivals = {}                  # topic -> deque of arrival times (monotonic)
        self.skew = {}                      # topic -> latest (now - header.stamp) seconds
        self.value = {}                     # topic -> latest Bool value
        self.nodes = set()
        self.joint_positions = {}
        self.controllers = {}               # name -> state, None until the service answered
        self.controllers_ok = False
        self.rosout = collections.deque(maxlen=200)
        self.lock = threading.Lock()
        for c in COMPONENTS:
            for t in c.topics:
                self._subscribe(t)
        self.node.create_subscription(Bool, "/e_stop", lambda m: self.value.__setitem__("/e_stop", m.data), 10)
        self.node.create_subscription(Log, "/rosout", self._on_rosout, QoSProfile(depth=50))
        from controller_manager_msgs.srv import ListControllers
        self.list_ctrl = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self._ListControllers = ListControllers
        self.node.create_timer(1.0, self._poll)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _subscribe(self, t):
        self.arrivals[t.name] = collections.deque(maxlen=400)

        def cb(msg, t=t):
            now = time.monotonic()
            self.arrivals[t.name].append(now)
            if t.name == "/joint_states":
                self.joint_positions.update(zip(msg.name, msg.position))
            if t.type == "std_msgs/msg/Bool":
                self.value[t.name] = msg.data
            if t.max_skew is not None and hasattr(msg, "header"):
                s = msg.header.stamp
                self.skew[t.name] = self.node.get_clock().now().nanoseconds * 1e-9 - (s.sec + s.nanosec * 1e-9)
        self.node.create_subscription(get_message(t.type), t.name, cb, BEST_EFFORT)

    def _on_rosout(self, m):
        if m.level >= 40:   # ERROR and FATAL only, WARN floods
            self.rosout.append((time.strftime("%H:%M:%S"), m.name, m.msg))

    def _poll(self):
        names = {ns.rstrip("/") + "/" + n if ns != "/" else "/" + n
                 for n, ns in self.node.get_node_names_and_namespaces()}
        self.nodes = names
        if self.list_ctrl.service_is_ready():
            fut = self.list_ctrl.call_async(self._ListControllers.Request())
            fut.add_done_callback(self._on_controllers)
        else:
            self.controllers_ok = False

    def _on_controllers(self, fut):
        r = fut.result()
        if r is not None:
            self.controllers = {c.name: c.state for c in r.controller}
            self.controllers_ok = True

    def rate(self, name, window=3.0):
        d = self.arrivals.get(name)
        if not d:
            return 0.0, None
        now = time.monotonic()
        n = sum(1 for x in list(d) if now - x <= window)
        return n / window, now - d[-1]

    def has_node(self, entry):
        alts = entry if isinstance(entry, tuple) else (entry,)
        return any(a in self.nodes for a in alts)

    def check(self, c):
        """-> (state, detail, hz) for one component, judged live."""
        problems, notes, hz_show = [], [], None
        for entry in c.nodes:
            if not self.has_node(entry):
                problems.append(f"no node {entry if isinstance(entry, str) else entry[0]}")
        for ctl in c.controllers:
            st = self.controllers.get(ctl)
            if not self.controllers_ok:
                problems.append("controller_manager not answering")
                break
            if st != "active":
                problems.append(f"{ctl} {st or 'missing'}")
        for t in c.topics:
            hz, age = self.rate(t.name)
            hz_show = hz if hz_show is None else min(hz_show, hz)
            if t.min_hz <= 0:                           # event topic, value only
                if t.name in self.value:
                    notes.append(f"{t.name.split('/')[-1]}={self.value[t.name]}")
                continue
            if age is None:
                problems.append(f"no data on {t.name}")
            elif age > max(1.5, 3.0 / t.min_hz):
                problems.append(f"{t.name.split('/')[-1]} stale {age:.1f}s")
            elif hz < t.min_hz * 0.7:
                problems.append(f"{hz:.1f} Hz < {t.min_hz:g}")
            sk = self.skew.get(t.name)
            if t.max_skew is not None and sk is not None and abs(sk) > t.max_skew:
                problems.append(f"stamp {sk:+.1f}s off clock")
        if problems:
            return FAIL, "; ".join(problems[:2]) + (f" (+{len(problems) - 2})" if len(problems) > 2 else ""), hz_show
        return OK, ", ".join(notes), hz_show


STATS_SCRIPT = r"""
while :; do
  awk '/^cpu /{print "cpu", $2+$3+$4+$7+$8, $2+$3+$4+$5+$6+$7+$8}' /proc/stat
  awk '/MemTotal/{t=$2} /MemAvailable/{a=$2} END{print "mem", t, a}' /proc/meminfo
  for g in /sys/devices/platform/bus@0/17000000.gpu/load /sys/devices/platform/gpu.0/load; do
    [ -r $g ] && { echo gpu $(cat $g); break; }; done
  echo temp $(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1)
  echo end
  sleep 1
done
"""


class Stats:
    """Jetson load over one long-lived ssh: CPU %, memory, GPU %, hottest thermal zone."""

    def __init__(self, host):
        self.host = host
        self.text = "Jetson: connecting..."
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                p = subprocess.Popen(["ssh", "-o", "ServerAliveInterval=5", self.host, "bash -s"],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                p.stdin.write(STATS_SCRIPT)
                p.stdin.close()
                prev, cur = None, {}
                for line in p.stdout:
                    k, *v = line.split()
                    if k != "end":
                        cur[k] = [float(x) for x in v]
                        continue
                    busy, total = cur["cpu"]
                    cpu = 0.0 if prev is None or total == prev[1] else 100 * (busy - prev[0]) / (total - prev[1])
                    prev = (busy, total)
                    mt, ma = cur["mem"]
                    parts = [f"CPU {cpu:3.0f}%", f"mem {(mt - ma) / 1048576:.1f}/{mt / 1048576:.1f} GB"]
                    if cur.get("gpu"):
                        parts.append(f"GPU {cur['gpu'][0] / 10:3.0f}%")
                    if cur.get("temp"):
                        parts.append(f"{cur['temp'][0] / 1000:.0f} C")
                    self.text = f"{self.host}:  " + "   ".join(parts)
                    cur = {}
            except Exception:  # noqa: BLE001 - keep retrying, the link may just be down
                pass
            self.text = f"{self.host}: no stats (ssh down?)"
            time.sleep(3)


class TuiApp(App):
    CSS = "DataTable {height: 1fr;} RichLog {height: 12; border-top: solid $primary;} #stats {height: 1; padding: 0 1;} .hidden {display: none;}"
    TITLE = "XLeRobot stack"
    BINDINGS = [
        Binding("s", "start", "start"), Binding("x", "stop", "stop"), Binding("X", "stop_limp", "stop + limp"),
        Binding("t", "limp", "torque off"),
        Binding("g", "presence", "presence"), Binding("e", "estop", "E-STOP"),
        Binding("u", "release", "release"), Binding("c", "clear_octomap", "clear octomap"),
        Binding("v", "validate", "validate"), Binding("p", "profile", "profile"),
        Binding("r", "record_hand", "record by hand"), Binding("R", "record_drive", "record driving"), Binding("y", "play", "play"),
        Binding("z", "stop_play", "stop play"), Binding("m", "list_motions", "motions"),
        Binding("l", "toggle_log", "log"), Binding("q", "quit", "quit"),
    ]

    def __init__(self, host, profile):
        super().__init__()
        self.host, self.profile = host, profile
        self.mon = None
        self.busy = False
        self._rosout_seen = 0

    def compose(self) -> ComposeResult:
        yield Header()
        yield Static(id="stats")
        yield DataTable(cursor_type="none")
        yield RichLog(wrap=True, markup=True)
        yield Input(id="prompt", placeholder="", classes="hidden")
        yield Footer()

    def on_mount(self):
        t = self.query_one(DataTable)
        t.add_columns("component", "group", "state", "Hz", "detail")
        self.mon = Monitor()
        self.stats = Stats(self.host)
        self.recorder = None
        self.player = motion_recorder.Player(self.mon.node, self.say_any)
        self.prompt_mode = None
        self.set_interval(0.5, self.refresh_table)
        self.refresh_title()
        self.say(f"host [b]{self.host}[/b], profile [b]{self.profile}[/b]. s starts the stack, v validates.")

    def on_unmount(self):
        if self.recorder is not None and self.recorder.hand_guided:
            self.recorder.stop(self.recorder.name)      # puts torque back
        if self.mon:
            self.mon.executor.shutdown()
            self.mon.node.destroy_node()
            rclpy.try_shutdown()

    def refresh_title(self):
        self.sub_title = f"{self.host}  profile: {self.profile}  ({', '.join(sorted(PROFILES[self.profile][0]))})"

    def say_any(self, text):
        if threading.get_ident() == self._thread_id:
            self.say(text)
        else:
            self.call_from_thread(self.say, text)

    def say(self, text):
        self.query_one(RichLog).write(f"[dim]{time.strftime('%H:%M:%S')}[/dim] {text}")

    def refresh_table(self):
        self.query_one("#stats", Static).update(self.stats.text)
        expected = PROFILES[self.profile][0]
        t = self.query_one(DataTable)
        t.clear()
        for c in COMPONENTS:
            state, detail, hz = self.mon.check(c)
            wanted = c.group in expected and not c.optional
            if not wanted and state == FAIL:
                state, detail = OFF, ""
            elif not wanted and state == OK and not c.optional:
                detail = (detail + " " if detail else "") + "(running, not in profile)"
            elif c.optional and state == FAIL:
                state, detail = OFF, ""
            hz_s = "" if hz is None else f"{hz:5.1f}"
            t.add_row(c.name, c.group, f"[{STYLE[state]}]{state}[/]", hz_s, detail)
        if self.mon.value.get("/e_stop") is True:
            self.sub_title = "E-STOP ACTIVE"
        elif self.recorder is not None:
            self.sub_title = f"RECORDING {self.recorder.name}" + (" (torque off)" if self.recorder.hand_guided else "")
        else:
            self.refresh_title()
        while self._rosout_seen < len(self.mon.rosout):
            ts, name, msg = self.mon.rosout[self._rosout_seen]
            self._rosout_seen += 1
            self.say(f"[red]{name}[/red] {msg[:200]}")

    # --- remote control -------------------------------------------------------------------
    def remote(self, script, label):
        if self.busy:
            self.say("another action is still running")
            return
        self.busy = True
        self.say(f"[b]{label}[/b]")

        def run():
            try:
                p = subprocess.Popen(["ssh", self.host, "bash -s"], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                p.stdin.write(script)
                p.stdin.close()
                for line in p.stdout:
                    if "RTPS" in line or "BrokenPipe" in line:
                        continue
                    self.call_from_thread(self.say, line.rstrip())
                p.wait()
                self.call_from_thread(self.say, f"{label}: exit {p.returncode}")
            finally:
                self.busy = False
        threading.Thread(target=run, daemon=True).start()

    def prelude(self):
        return f"cd ~/{REPO_ON_JETSON}\nsource /opt/ros/jazzy/setup.bash\nsource install/setup.bash\n"

    def action_start(self):
        args = PROFILES[self.profile][1]
        self.remote(self.prelude() + "python3 tools/kill_ros.py --yes --clean-shm | tail -1\n"
                    f"setsid nohup ros2 launch xlerobot_bringup full_stack.launch.py {args} "
                    "> /tmp/full_stack.log 2>&1 &\necho started, log /tmp/full_stack.log on the Jetson\n",
                    f"start {self.profile}")

    def action_stop(self):
        self.remote(self.prelude() + "python3 tools/kill_ros.py --yes --clean-shm | tail -3\n", "stop stack")

    LIMP = "python3 tools/calibrate_arm.py torque-off --part all\n"

    def action_stop_limp(self):
        self.ask("stop_limp", 'type "yes": stop the stack AND cut servo torque, arms go limp, hold them')

    def action_limp(self):
        self.ask("limp", 'type "yes": cut servo torque, arms go limp, hold them (stack must be stopped)')

    def action_presence(self):
        up = "/mcp_presence_guard" in self.mon.nodes
        if up:
            self.remote("pkill -f mcp_presence.launch.py; pkill -f mcp_presence_guard; echo stopped\n", "presence guard off")
        else:
            self.remote(self.prelude() + "setsid nohup ros2 launch xlerobot_teleop mcp_presence.launch.py "
                        "> /tmp/presence.log 2>&1 &\necho started\n", "presence guard on")

    def local(self, argv, label):
        self.say(f"[b]{label}[/b]")

        def run():
            r = subprocess.run(argv, capture_output=True, text=True)
            out = (r.stdout + r.stderr).strip().splitlines()
            for line in out[-6:]:
                self.call_from_thread(self.say, line)
        threading.Thread(target=run, daemon=True).start()

    def action_estop(self):
        self.local([sys.executable, str(TOOLS / "estop.py")], "E-STOP")

    def action_release(self):
        self.say("[yellow]hold the arms before releasing[/yellow]")
        self.local([sys.executable, str(TOOLS / "estop.py"), "--release"], "release e-stop")

    # --- motion record / playback ---------------------------------------------------------
    def ask(self, mode, text):
        self.prompt_mode = mode
        box = self.query_one("#prompt", Input)
        box.placeholder, box.value = text, ""
        box.remove_class("hidden")
        box.focus()

    def on_key(self, event):
        if event.key == "escape" and self.prompt_mode:
            self.close_prompt()

    def close_prompt(self):
        self.prompt_mode = None
        box = self.query_one("#prompt", Input)
        box.add_class("hidden")
        self.query_one(DataTable).focus()

    def on_input_submitted(self, ev):
        name, mode = ev.value.strip(), self.prompt_mode
        self.close_prompt()
        if not name:
            return
        if mode in ("limp", "stop_limp"):
            if name.lower() != "yes":
                self.say("cancelled")
                return
            if mode == "limp":
                self.remote(self.prelude() + self.LIMP, "torque off")
            else:
                self.remote(self.prelude() + "python3 tools/kill_ros.py --yes --clean-shm | tail -2\nsleep 2\n" + self.LIMP,
                            "stop stack and cut torque")
            return
        if mode in ("record_hand", "record_drive"):
            threading.Thread(target=self.begin_recording, args=(name, mode == "record_hand"), daemon=True).start()
        elif mode == "play":
            threading.Thread(target=self.start_playback, args=(name,), daemon=True).start()

    def begin_recording(self, name, hand):
        r = motion_recorder.Recorder(self.mon.node, hand_guided=hand)
        r.name = name
        err = r.start()
        if err:
            self.call_from_thread(self.say, f"[red]cannot start: {err}[/red]")
            return
        self.recorder = r
        self.call_from_thread(self.say, f"[red]recording '{name}'[/red]" + (
            ": torque is OFF, arms and head are limp, hold them. Press r to stop (torque comes back)"
            if hand else ": drive now, press r to stop"))

    def stop_recording(self):
        r, self.recorder = self.recorder, None

        def finish():
            result = r.stop(r.name)
            self.call_from_thread(self.say, f"saved motion '{r.name}': {result}" if result
                                  else "nothing recorded (no /joint_states?)")
        threading.Thread(target=finish, daemon=True).start()

    def action_record_hand(self):
        if self.recorder is not None:
            self.stop_recording()
        else:
            self.ask("record_hand", "name, Enter CUTS TORQUE on arms/head/wheels, support the arms (Esc cancels)")

    def action_record_drive(self):
        if self.recorder is not None:
            self.stop_recording()
        else:
            self.ask("record_drive", "name, Enter starts recording while you drive (Esc cancels)")

    def action_play(self):
        names = motion_recorder.list_motions()
        self.ask("play", "motion to play: " + (", ".join(n[7:] for n in names) or "none recorded yet"))

    def start_playback(self, name):
        if not motion_recorder.motion_path(name).exists():
            self.call_from_thread(self.say, f"no motion '{name}'")
            return
        if self.mon.value.get("/e_stop") is True:
            self.call_from_thread(self.say, "[red]e-stop is active, release it first[/red]")
            return
        self.call_from_thread(self.say, "[yellow]clear the area and support the arms; e = e-stop, z = stop playback[/yellow]")
        cur = dict(self.mon.joint_positions)
        self.player.play(name, cur)

    def action_stop_play(self):
        self.player.stop()
        self.say("playback stopped")

    def action_list_motions(self):
        self.say("motions: " + (", ".join(n[7:] for n in motion_recorder.list_motions()) or "none"))

    def action_clear_octomap(self):
        from std_srvs.srv import Empty
        c = self.mon.node.create_client(Empty, "/clear_octomap")
        if not c.service_is_ready():
            self.say("/clear_octomap not available (move_group down?)")
            return
        c.call_async(Empty.Request())
        self.say("octomap cleared")

    def action_profile(self):
        names = list(PROFILES)
        self.profile = names[(names.index(self.profile) + 1) % len(names)]
        self.refresh_title()
        self.say(f"profile {self.profile}")

    def action_toggle_log(self):
        log = self.query_one(RichLog)
        log.display = not log.display

    def action_validate(self):
        threading.Thread(target=self.validate, daemon=True).start()

    def validate(self):
        m = self.mon
        say = lambda s: self.call_from_thread(self.say, s)
        results = []

        def rec(name, ok, detail=""):
            results.append(ok)
            say(f"  [{'green' if ok else 'red'}]{'PASS' if ok else 'FAIL'}[/] {name} {detail}")
        say("[b]validate[/b]")
        expected = PROFILES[self.profile][0]
        bad = []
        for c in COMPONENTS:
            if c.group in expected and not c.optional:
                s, d, _ = m.check(c)
                if s != OK:
                    bad.append(f"{c.name}: {d}")
        rec("every component in the profile is live", not bad, "; ".join(bad[:3]))
        if m.value.get("/e_stop") is True:
            rec("no e-stop active", False)
        guard = m.value.get("/mcp_presence_ok")
        say(f"  info presence guard: {'not running' if '/mcp_presence_guard' not in m.nodes else f'ok={guard}'}")
        from tf2_ros import Buffer, TransformListener
        buf = Buffer()
        TransformListener(buf, m.node, spin_thread=False)
        time.sleep(3.0)
        frames = ["head_color_optical_frame", "left_arm_gripper_link", "right_arm_gripper_link", "ldlidar_link"]
        for f in frames:
            try:
                buf.lookup_transform("base_footprint", f, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5))
                rec(f"TF base_footprint -> {f}", True)
            except Exception as e:  # noqa: BLE001 - report whatever tf says
                rec(f"TF base_footprint -> {f}", False, type(e).__name__)
        if "/move_group" in m.nodes:
            from moveit_msgs.srv import GetPlanningScene
            cl = m.node.create_client(GetPlanningScene, "/get_planning_scene")
            if cl.wait_for_service(timeout_sec=2.0):
                req = GetPlanningScene.Request()
                req.components.components = 32      # OCTOMAP
                fut = cl.call_async(req)
                t0 = time.time()
                while not fut.done() and time.time() - t0 < 5:
                    time.sleep(0.05)
                if fut.done():
                    size = len(fut.result().scene.world.octomap.octomap.data)
                    rec("octomap has data", size > 0, f"({size} bytes)")
                else:
                    rec("octomap has data", False, "(no answer)")
        say(f"validate: {sum(results)}/{len(results)} passed")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="jetson", help="ssh host that runs the stack")
    ap.add_argument("--profile", default="sensing", choices=list(PROFILES))
    a = ap.parse_args()
    TuiApp(a.host, a.profile).run()


if __name__ == "__main__":
    main()
