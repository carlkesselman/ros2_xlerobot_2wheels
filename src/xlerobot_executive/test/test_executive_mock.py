"""The executive against mock hardware: plan validation, events, failures handled by the plan, pause and resume, abort,
the planner lease and safety stops.

  source install/setup.bash && source tools/ros_env.sh
  python3 -m pytest src/xlerobot_executive/test/test_executive_mock.py -v

Same setup as xlerobot_skills' test_skills_mock.py (mock hardware, move_group, skill server, presence guard in
development mode, scripted perception) plus the executive with a 10 s lease. About three minutes, no robot needed;
XLEROBOT_SKIP_MOCK=1 skips it.
"""
import json
import os
import random
import signal
import subprocess
import sys
import time

import pytest

if os.environ.get("XLEROBOT_SKIP_MOCK"):
    pytest.skip("XLEROBOT_SKIP_MOCK is set", allow_module_level=True)

os.environ["ROS_DOMAIN_ID"] = str(random.randint(150, 220))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from xlerobot_skill_msgs.srv import ExecutionControl, SubmitPlan, WaitForEvent  # noqa: E402

from ament_index_python.packages import get_package_prefix  # noqa: E402

FAKE = os.path.join(get_package_prefix("xlerobot_skills"), "..", "..", "src", "xlerobot_skills", "test", "fake_perception.py")
if not os.path.exists(FAKE):
    FAKE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "xlerobot_skills", "test", "fake_perception.py")
CUBE = [0.241, -0.053, 0.7666]
LAUNCHES = [
    ["ros2", "launch", "xlerobot_bringup", "bringup.launch.py", "use_mock_hardware:=true", "with_arms:=true",
     "with_head:=true", "teleop:=false"],
    ["ros2", "launch", "xlerobot_moveit_config", "move_group.launch.py"],
    ["ros2", "launch", "xlerobot_skills", "skills.launch.py"],
    ["ros2", "launch", "xlerobot_teleop", "mcp_presence.launch.py"],
    [sys.executable, FAKE],
    ["ros2", "run", "xlerobot_executive", "executive", "--ros-args", "-p", "lease_s:=10.0", "-p", "max_hold_s:=30.0"],
]


class Planner:
    def __init__(self):
        self.node = rclpy.create_node("planner_test")
        self.submit_c = self.node.create_client(SubmitPlan, "/executive/submit_plan")
        self.wait_c = self.node.create_client(WaitForEvent, "/executive/wait_for_event")
        self.control_c = self.node.create_client(ExecutionControl, "/executive/control")
        self.params = self.node.create_client(SetParameters, "/mcp_presence_guard/set_parameters")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene_pub = self.node.create_publisher(String, "/fake_perception/scene", latched)
        self.seq = 0

    def call(self, client, req, timeout=60.0):
        f = client.call_async(req)
        t0 = time.monotonic()
        while not f.done() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert f.done(), "no answer"
        return f.result()

    def submit(self, plan):
        return self.call(self.submit_c, SubmitPlan.Request(plan_json=json.dumps(plan)))

    def events(self, max_wait=5.0):
        r = self.call(self.wait_c, WaitForEvent.Request(after_seq=self.seq, max_wait_s=max_wait), max_wait + 10)
        self.seq = r.last_seq
        return [json.loads(e) for e in r.events_json], json.loads(r.state_json)

    def until(self, kinds, timeout=180.0):
        """All events up to and including the first one whose type is in kinds."""
        seen, t0 = [], time.monotonic()
        while time.monotonic() - t0 < timeout:
            evs, _ = self.events(5.0)
            for e in evs:
                seen.append(e)
                if e["type"] in kinds:
                    return seen
        raise AssertionError(f"none of {kinds} within {timeout} s; saw {[e['type'] for e in seen]}")

    def control(self, cmd):
        return self.call(self.control_c, ExecutionControl.Request(command=cmd))

    def presence_required(self, on):
        self.call(self.params, SetParameters.Request(parameters=[Parameter("presence_required", value=on).to_parameter_msg()]))
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.5:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def scene(self, objects=(), codes=()):
        self.scene_pub.publish(String(data=json.dumps({"objects": [list(o) for o in objects], "codes": list(codes)})))
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.5:
            rclpy.spin_once(self.node, timeout_sec=0.05)


@pytest.fixture(scope="module")
def planner():
    def start(cmds):
        return [subprocess.Popen(c, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True) for c in cmds]
    procs = start(LAUNCHES[:1])
    time.sleep(8.0)
    procs += start(LAUNCHES[1:])
    rclpy.init()
    p = Planner()
    try:
        for c in (p.submit_c, p.wait_c, p.control_c, p.params):
            assert c.wait_for_service(timeout_sec=60.0), f"{c.srv_name} not up"
        p.presence_required(False)
        time.sleep(15.0)                                   # move_group and the skill server's action clients
        p.events(0.0)
        yield p
    finally:
        p.node.destroy_node()
        rclpy.try_shutdown()
        for pr in procs:
            try:
                os.killpg(pr.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(3.0)
        for pr in procs:
            try:
                os.killpg(pr.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def plan(*steps, objects=None):
    return {"schema_version": 1, "task": "test", "objects": objects or {}, "steps": list(steps)}


def arm(name, pos, **kw):
    side = "left" if "left" in pos else "right"
    return {"id": name, "skill": "move_arm", "params": {"arm": side, "named_position": pos, **kw}}


def test_invalid_plan_is_refused(planner):
    r = planner.submit(plan({"id": "x", "skill": "dance"}))
    assert not r.accepted and any("unknown skill" in e for e in r.errors)
    r = planner.submit(plan(arm("a", "right_rest"), arm("b", "no_such_pose")))
    assert not r.accepted and any("no saved position" in e for e in r.errors)


def test_simple_plan_succeeds(planner):
    r = planner.submit(plan({"id": "look", "skill": "look_at", "params": {"named_position": "head_look_down"}},
                            arm("out", "left_pose_b"), arm("back", "left_rest",
                                                          ) | {"expect": [{"joint_near": {"position": "left_rest", "tol": 0.05}}]}))
    assert r.accepted, r.errors
    seen = planner.until({"plan_succeeded", "plan_failed", "plan_aborted"})
    assert seen[-1]["type"] == "plan_succeeded", seen[-1]
    assert [e["step_id"] for e in seen if e["type"] == "step_succeeded"] == ["look", "out", "back"]


def test_detect_binds_and_approach(planner):
    planner.scene([CUBE])
    r = planner.submit(plan(
        {"id": "find", "skill": "detect", "params": {"prompt": "a cube", "bind": "cube"}},
        {"id": "reach", "skill": "approach", "params": {"arm": "right", "object": "cube"}, "expect": [{"detected": "cube"}]},
        arm("back", "right_rest"),
        objects={"cube": {"prompt": "a cube", "size_m": 0.028}}))
    assert r.accepted, r.errors
    seen = planner.until({"plan_succeeded", "plan_failed", "plan_aborted"})
    assert seen[-1]["type"] == "plan_succeeded", seen[-1]
    _, state = planner.events(0.0)
    assert abs(state["objects"]["cube"]["pose"][0] - CUBE[0]) < 0.01


def test_retry_then_fail(planner):
    planner.scene([])
    r = planner.submit(plan({"id": "find", "skill": "detect", "params": {"prompt": "a cube"},
                             "on_failure": {"codes": ["NOT_FOUND"], "action": "retry", "max_retries": 2}}))
    assert r.accepted, r.errors
    seen = planner.until({"plan_succeeded", "plan_failed", "plan_aborted"})
    fails = [e for e in seen if e["type"] == "step_failed"]
    assert [e["then"] for e in fails] == ["retry", "retry", "stop"]
    assert seen[-1]["type"] == "plan_failed" and seen[-1]["code"] == "NOT_FOUND"


def test_goto(planner):
    planner.scene([])
    r = planner.submit(plan({"id": "look", "skill": "look_at", "params": {"named_position": "head_forward"}},
                            {"id": "find", "skill": "detect", "params": {"prompt": "a cube"},
                             "on_failure": {"action": "goto", "step": "look", "max_retries": 1}}))
    assert r.accepted, r.errors
    seen = planner.until({"plan_succeeded", "plan_failed", "plan_aborted"})
    assert [e["step_id"] for e in seen if e["type"] == "step_started"] == ["look", "find", "look", "find"]
    assert seen[-1]["type"] == "plan_failed"


def test_pause_resume(planner):
    r = planner.submit(plan(arm("go", "cube_right_standoff", scaling=0.05), arm("back", "right_rest")))
    assert r.accepted, r.errors
    planner.until({"step_started"})
    time.sleep(3.0)
    assert planner.control("pause").ok
    planner.until({"paused"})
    _, state = planner.events(0.0)
    assert state["status"] == "paused"
    assert planner.control("resume").ok
    seen = planner.until({"plan_succeeded", "plan_failed", "plan_aborted"})
    assert seen[-1]["type"] == "plan_succeeded", seen[-1]
    assert any(e["type"] == "resumed" for e in seen)


def test_abort_and_one_plan_at_a_time(planner):
    r = planner.submit(plan(arm("go", "cube_right_standoff", scaling=0.05), arm("back", "right_rest")))
    assert r.accepted, r.errors
    planner.until({"step_started"})
    again = planner.submit(plan(arm("x", "right_rest")))
    assert not again.accepted and "abort it first" in again.errors[0]
    assert planner.control("abort").ok
    seen = planner.until({"plan_succeeded", "plan_failed", "plan_aborted"})
    assert seen[-1]["type"] == "plan_aborted"


def test_safety_stop_aborts(planner):
    r = planner.submit(plan(arm("start", "right_rest"), arm("go", "cube_right_standoff", scaling=0.05)))
    assert r.accepted, r.errors
    planner.until({"step_succeeded"})
    time.sleep(3.0)
    planner.presence_required(True)
    try:
        seen = planner.until({"plan_succeeded", "plan_failed", "plan_aborted"})
        assert seen[-1]["type"] == "plan_aborted" and "safety" in seen[-1]["reason"]
    finally:
        planner.presence_required(False)
    planner.submit(plan(arm("back", "right_rest")))
    planner.until({"plan_succeeded", "plan_failed", "plan_aborted"})


def test_lease_expiry_retreats(planner):
    r = planner.submit(plan(arm("go", "cube_right_standoff", scaling=0.02), arm("back", "right_rest")))
    assert r.accepted, r.errors
    time.sleep(16.0)                                       # no wait_for_event: the 10 s lease lapses
    seen = planner.until({"plan_succeeded", "plan_failed", "plan_aborted"}, timeout=120.0)
    assert seen[-1]["type"] == "plan_aborted" and "lease" in seen[-1]["reason"], seen[-1]
