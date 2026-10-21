"""Record a driven/teleoperated motion (arms, head and wheels together) and play it back.

Used by tools/xlerobot_tui.py. Unlike tools/record_trajectory.py nothing loses torque: you drive
with the joystick (and arm teleop) as usual while this samples /joint_states and the joystick's
base command, and the file can be replayed later with the same timing.

Wheels: the recording is `cmd_vel_joy` (what the joystick asked for). Playback publishes to
`cmd_vel_nav` at 20 Hz, which twist_mux ranks below the joystick and below the e-stop lock, so a
touch on the stick or an e-stop stops the base at once. Arms and head are sent as one
FollowJointTrajectory goal per controller, all starting together. Playback is open loop and does
no collision checking: it resends what was done, so run it from the same start pose and with the
same clear area.
"""
import threading
import time

import yaml
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

import position_store

GROUPS = ["left_arm", "left_gripper", "right_arm", "right_gripper", "head"]
JOINTS = [j for g in GROUPS for j in position_store.GROUP_JOINTS[g]]
JOINT_RATE = 20.0
BASE_TOPIC_IN = "/cmd_vel_joy"
BASE_TOPIC_OUT = "/cmd_vel_nav"
MOVE_EPS = 0.03          # rad, a group that moved less than this in the recording is not replayed
MAX_JOINT_VEL = 0.9      # rad/s, under the hardware cap of 1.25 x the 0.8 planning limit (robot_dimensions.yaml). The hardware
                         # interface caps commands at 1.25x this, so a faster recording just lags and is clamped
LEAD_SPEED = 0.4         # rad/s used to get from the current pose to the recording's first pose


def list_motions():
    return sorted(p.stem for p in position_store.TRAJECTORY_DIR.glob("motion_*.yaml"))


def motion_path(name):
    return position_store.TRAJECTORY_DIR / f"motion_{name}.yaml"


BUSES = ["xlerobot_bus1", "xlerobot_bus2"]


def set_torque_cut(node, cut, timeout=5.0, buses=None):
    """Cut (True) or restore (False) torque on both servo buses, same service tools/estop.py uses.
    Bus 2 also carries the wheels. Returns an error string or None. Needs the node's executor to
    be spinning, it only waits on the futures."""
    futures = {}
    for bus in buses or BUSES:
        client = node.create_client(SetBool, f"/emergency_stop_{bus}")
        if not client.wait_for_service(timeout_sec=timeout):
            return f"/emergency_stop_{bus} not available"
        futures[bus] = client.call_async(SetBool.Request(data=cut))
    end = time.monotonic() + timeout
    for bus, fut in futures.items():
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.02)
        r = fut.result() if fut.done() else None
        if r is None or not r.success:
            return f"{bus}: {'no answer' if r is None else r.message}"
    return None


class Recorder:
    def __init__(self, node, hand_guided=False):
        self.node = node
        self.hand_guided = hand_guided
        self.positions = {}
        self.samples = []
        self.base = []
        self.twist = (0.0, 0.0, 0.0)
        self.t0 = None
        self.subs = []
        self.timer = None

    def start(self):
        if self.hand_guided:
            err = set_torque_cut(self.node, True)
            if err:
                set_torque_cut(self.node, False)
                return err
        self.t0 = time.monotonic()
        self.samples, self.base = [], []
        self.subs = [
            self.node.create_subscription(JointState, "/joint_states", self._joints, 10),
            self.node.create_subscription(TwistStamped, BASE_TOPIC_IN, self._twist, 10),
        ]
        self.timer = self.node.create_timer(1.0 / JOINT_RATE, self._tick)
        return None

    def _joints(self, m):
        self.positions.update(zip(m.name, m.position))

    def _twist(self, m):
        t = m.twist
        self.twist = (t.linear.x, t.linear.y, t.angular.z)
        self.base.append({"t": round(time.monotonic() - self.t0, 3), "v": [round(x, 4) for x in self.twist]})

    def _tick(self):
        if all(j in self.positions for j in JOINTS):
            self.samples.append({"time_from_start": round(time.monotonic() - self.t0, 3),
                                 "positions": [round(float(self.positions[j]), 5) for j in JOINTS]})

    def stop(self, name):
        for s in self.subs:
            self.node.destroy_subscription(s)
        self.node.destroy_timer(self.timer)
        if self.hand_guided:
            err = set_torque_cut(self.node, False)
            if err:
                return f"TORQUE NOT RESTORED ({err}), run tools/estop.py --release"
        if not self.samples:
            return None
        position_store.TRAJECTORY_DIR.mkdir(parents=True, exist_ok=True)
        data = {"joints": JOINTS, "points": self.samples, "base_topic": BASE_TOPIC_IN, "base": self.base}
        with open(motion_path(name), "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        return f"{len(self.samples)} joint samples, {len(self.base)} base commands, {self.samples[-1]['time_from_start']:.1f} s"


class Player:
    """Plays a recording on a worker thread. `log` is called with progress strings."""

    def __init__(self, node, log):
        self.node = node
        self.log = log
        self.cancel = threading.Event()
        self.handles = []
        self.pub = node.create_publisher(TwistStamped, BASE_TOPIC_OUT, QoSProfile(depth=1))
        self.clients = {}
        self.thread = None
        self.slowdown, self.vmax = 1.0, 0.0

    def plan(self, name, current):
        """-> (parts, times, lead, base, moved) without sending anything."""
        with open(motion_path(name)) as f:
            data = yaml.safe_load(f)
        joints, points = data["joints"], data["points"]
        times = [p["time_from_start"] for p in points]
        parts = []
        for g in GROUPS:
            gj = position_store.GROUP_JOINTS[g]
            idx = [joints.index(j) for j in gj]
            traj = [[p["positions"][i] for i in idx] for p in points]
            span = max(max(r[k] for r in traj) - min(r[k] for r in traj) for k in range(len(gj)))
            start_gap = max(abs(current.get(j, traj[0][k]) - traj[0][k]) for k, j in enumerate(gj))
            if span >= MOVE_EPS:
                parts.append((g, gj, traj, start_gap))
        lead = max([1.0] + [gap / LEAD_SPEED + 0.5 for *_, gap in parts])
        base = data.get("base") or []
        # Hand-guided motions are quicker than the joint limits allow. Stretch the whole recording
        # in time (arms, head and wheels together, wheel speed scaled down so the distance driven
        # stays the same) instead of letting the controller clamp it and drift out of sync.
        window = max(1, round(0.25 * JOINT_RATE))
        vmax = 0.0
        for _, _, traj, _ in parts:
            for i in range(len(traj) - window):
                dt = times[i + window] - times[i]
                if dt > 0:
                    vmax = max(vmax, max(abs(a - b) for a, b in zip(traj[i + window], traj[i])) / dt)
        self.slowdown = max(1.0, vmax / MAX_JOINT_VEL)
        self.vmax = vmax
        times = [t * self.slowdown for t in times]
        base = [{"t": b["t"] * self.slowdown, "v": [x / self.slowdown for x in b["v"]]} for b in base]
        return parts, times, lead, base, [p[0] for p in parts]

    def play(self, name, current, dry_run=False):
        parts, times, lead, base, moved = self.plan(name, current)
        duration = times[-1]
        self.log(f"'{name}': {duration:.1f} s, moves {', '.join(moved) or 'nothing'}, "
                 f"{len(base)} base commands, lead-in {lead:.1f} s"
                 + (f", peak {self.vmax:.1f} rad/s is over the {MAX_JOINT_VEL} limit so played {self.slowdown:.1f}x slower"
                    if self.slowdown > 1 else ""))
        if dry_run:
            return
        self.cancel.clear()
        self.thread = threading.Thread(target=self._run, args=(parts, times, lead, base, current), daemon=True)
        self.thread.start()

    def _client(self, group):
        ctl = position_store.GROUP_CONTROLLER[group]
        if ctl not in self.clients:
            self.clients[ctl] = ActionClient(self.node, FollowJointTrajectory, f"/{ctl}/follow_joint_trajectory")
        return self.clients[ctl]

    def _run(self, parts, times, lead, base, current):
        duration = times[-1]
        sends = []
        for group, gj, traj, _ in parts:
            client = self._client(group)
            if not client.wait_for_server(timeout_sec=5.0):
                self.log(f"[red]{group}: controller action server not available, aborting[/red]")
                return
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = gj
            here = [current.get(j, traj[0][k]) for k, j in enumerate(gj)]
            goal.trajectory.points.append(JointTrajectoryPoint(positions=here, time_from_start=Duration(seconds=0.0).to_msg()))
            goal.trajectory.points.append(JointTrajectoryPoint(positions=traj[0], time_from_start=Duration(seconds=lead).to_msg()))
            for p, q in zip(times, traj):
                goal.trajectory.points.append(JointTrajectoryPoint(positions=q, time_from_start=Duration(seconds=lead + p).to_msg()))
            sends.append((group, client.send_goal_async(goal)))
        self.handles = []
        for group, fut in sends:
            while not fut.done() and not self.cancel.is_set():
                time.sleep(0.02)
            h = fut.result() if fut.done() else None
            if h is None or not h.accepted:
                self.log(f"[red]{group}: goal rejected[/red]")
                self.stop()
                return
            self.handles.append(h)
        self.log("playing")
        t0 = time.monotonic()
        i = 0
        total = lead + duration
        while time.monotonic() - t0 < total + 0.3 and not self.cancel.is_set():
            t = time.monotonic() - t0 - lead
            while i + 1 < len(base) and base[i + 1]["t"] <= t:
                i += 1
            v = base[i]["v"] if base and t >= base[0]["t"] and t <= base[-1]["t"] + 0.3 else (0.0, 0.0, 0.0)
            self._publish(v)
            time.sleep(0.05)
        self._publish((0.0, 0.0, 0.0))
        self.log("playback cancelled" if self.cancel.is_set() else "playback finished")

    def _publish(self, v):
        m = TwistStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = "base_footprint"
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = v
        self.pub.publish(m)

    def stop(self):
        self.cancel.set()
        for h in self.handles:
            h.cancel_goal_async()
        self._publish((0.0, 0.0, 0.0))

