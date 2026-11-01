"""The skills against mock hardware: every result code each skill can produce, cancel and the safety stop.

  source install/setup.bash && source tools/ros_env.sh
  python3 -m pytest src/xlerobot_skills/test/test_skills_mock.py -v

Brings up the robot on ros2_control mock hardware (arms, head, base), move_group, the skill server, the presence guard
(in development mode) and a scripted perception service, on a ROS domain of its own and localhost-only discovery, so
it does not touch a running robot. About two minutes. Set XLEROBOT_SKIP_MOCK=1 to skip.
"""
import json
import math
import os
import random
import signal
import subprocess
import sys
import time

import pytest

if os.environ.get("XLEROBOT_SKIP_MOCK"):
    pytest.skip("XLEROBOT_SKIP_MOCK is set", allow_module_level=True)

DOMAIN = str(random.randint(150, 220))
os.environ["ROS_DOMAIN_ID"] = DOMAIN                       # before rclpy loads
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from xlerobot_skill_msgs.action import Align, Approach, Detect, LookAt, MoveArm, Touch  # noqa: E402
from xlerobot_skill_msgs.msg import SkillResult as R  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CUBE = (0.241, -0.053, 0.7666)                            # in reach of the right arm
LAUNCHES = [
    ["ros2", "launch", "xlerobot_bringup", "bringup.launch.py", "use_mock_hardware:=true", "with_arms:=true",
     "with_head:=true", "teleop:=false"],
    ["ros2", "launch", "xlerobot_moveit_config", "move_group.launch.py"],
    ["ros2", "launch", "xlerobot_skills", "skills.launch.py"],
    ["ros2", "launch", "xlerobot_teleop", "mcp_presence.launch.py"],
    [sys.executable, os.path.join(HERE, "fake_perception.py")],
]


class Robot:
    def __init__(self):
        self.node = rclpy.create_node("skills_test")
        self.joints = {}
        self.node.create_subscription(JointState, "/joint_states", lambda m: self.joints.update(zip(m.name, m.position)), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene_pub = self.node.create_publisher(String, "/fake_perception/scene", latched)
        self.params = self.node.create_client(SetParameters, "/mcp_presence_guard/set_parameters")
        self.clients = {n: ActionClient(self.node, t, f"/skills/{n}") for n, t in
                        (("move_arm", MoveArm), ("look_at", LookAt), ("detect", Detect), ("approach", Approach),
                         ("touch", Touch), ("align", Align))}

    def spin(self, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def until(self, cond, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if cond():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return False

    def presence_required(self, on):
        req = SetParameters.Request(parameters=[Parameter("presence_required", value=on).to_parameter_msg()])
        f = self.params.call_async(req)
        self.until(f.done, 10.0)
        self.spin(0.5)

    def scene(self, objects=(), codes=(), wrist_offset=(0.0, 0.0, 0.0)):
        self.scene_pub.publish(String(data=json.dumps({"objects": [list(o) for o in objects], "codes": list(codes),
                                                       "wrist_offset": list(wrist_offset)})))
        self.spin(0.5)

    def send(self, name, goal):
        f = self.clients[name].send_goal_async(goal)
        assert self.until(f.done, 15.0), f"{name}: goal not accepted"
        return f.result()

    def result(self, handle, timeout=120.0):
        f = handle.get_result_async()
        assert self.until(f.done, timeout), "no result"
        return f.result().result

    def run(self, name, goal, timeout=120.0):
        return self.result(self.send(name, goal), timeout)


@pytest.fixture(scope="module")
def robot():
    procs = [subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
             for cmd in LAUNCHES[:1]]
    time.sleep(8.0)                                        # controllers before move_group
    procs += [subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
              for cmd in LAUNCHES[1:]]
    rclpy.init()
    r = Robot()
    try:
        assert all(c.wait_for_server(timeout_sec=60.0) for c in r.clients.values()), "skill server not up"
        assert r.params.wait_for_service(timeout_sec=30.0), "presence guard not up"
        r.presence_required(False)
        assert r.until(lambda: len(r.joints) >= 12, 30.0), "no joint states"
        r.spin(10.0)                                       # move_group loads its scene
        yield r
    finally:
        r.node.destroy_node()
        rclpy.try_shutdown()
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(3.0)
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def point(x, y, z, frame="base_footprint"):
    from geometry_msgs.msg import PointStamped
    p = PointStamped()
    p.header.frame_id = frame
    p.point.x, p.point.y, p.point.z = float(x), float(y), float(z)
    return p


def obj(x, y, z, size=0.028):
    from xlerobot_skill_msgs.msg import ObjectRef
    o = ObjectRef(prompt="a cube", size_m=size)
    o.pose.header.frame_id = "base_footprint"
    o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = float(x), float(y), float(z)
    o.pose.pose.orientation.w = 1.0
    return o


# ---- move_arm ----
def test_move_arm_named(robot):
    res = robot.run("move_arm", MoveArm.Goal(arm="left", named_position="left_pose_b"))
    assert res.result.code == R.SUCCEEDED, res.result.message


def test_move_arm_unknown_name(robot):
    assert robot.run("move_arm", MoveArm.Goal(arm="left", named_position="no_such_pose")).result.code == R.NOT_FOUND


def test_move_arm_other_arms_position(robot):
    assert robot.run("move_arm", MoveArm.Goal(arm="right", named_position="left_pose_b")).result.code == R.FAILED


def test_move_arm_unreachable(robot):
    res = robot.run("move_arm", MoveArm.Goal(arm="left", point=point(1.5, 0.2, 0.8)))
    assert res.result.code == R.NOT_REACHABLE, res.result.message


def test_move_arm_rest(robot):
    for arm in ("left", "right"):
        assert robot.run("move_arm", MoveArm.Goal(arm=arm, named_position=f"{arm}_rest")).result.code == R.SUCCEEDED


# ---- look_at ----
def test_look_at_named(robot):
    assert robot.run("look_at", LookAt.Goal(named_position="head_look_down")).result.code == R.SUCCEEDED


def test_look_at_point(robot):
    res = robot.run("look_at", LookAt.Goal(target=point(0.5, 0.25, 0.75)))
    assert res.result.code == R.SUCCEEDED, res.result.message
    assert res.error_deg < 2.0


def test_look_at_too_close(robot):
    res = robot.run("look_at", LookAt.Goal(target=point(0.12, 0.0, 0.75)))
    assert res.result.code == R.NOT_REACHABLE, res.result.message


# ---- detect ----
def test_detect_one(robot):
    robot.scene([CUBE])
    res = robot.run("detect", Detect.Goal(prompt="a cube"))
    assert res.result.code == R.SUCCEEDED and len(res.detections.detections) == 1


def test_detect_none(robot):
    robot.scene([])
    assert robot.run("detect", Detect.Goal(prompt="a cube")).result.code == R.NOT_FOUND


def test_detect_busy(robot):
    robot.scene([], ["BUSY"])
    assert robot.run("detect", Detect.Goal(prompt="a cube")).result.code == R.FAILED


# ---- approach ----
def test_approach_not_found(robot):
    robot.scene([CUBE])
    res = robot.run("approach", Approach.Goal(arm="right", object=obj(CUBE[0], CUBE[1] + 0.2, CUBE[2])))
    assert res.result.code == R.NOT_FOUND, res.result.message


def test_approach_ambiguous(robot):
    robot.scene([CUBE, (CUBE[0], CUBE[1] + 0.03, CUBE[2])])
    res = robot.run("approach", Approach.Goal(arm="right", object=obj(CUBE[0], CUBE[1] + 0.015, CUBE[2])))
    assert res.result.code == R.AMBIGUOUS, res.result.message


def test_approach_and_touch(robot):
    robot.scene([CUBE])
    res = robot.run("approach", Approach.Goal(arm="right", object=obj(*CUBE)))
    assert res.result.code == R.SUCCEEDED, res.result.message
    # mock joints follow every command, so nothing is ever felt: touch must stop at its travel limit
    res = robot.run("touch", Touch.Goal(arm="right", object=obj(*CUBE)), timeout=180.0)
    assert res.result.code == R.NOT_FOUND, res.result.message
    assert math.isnan(res.contact_z)


def test_align_corrects_the_head_cameras_error(robot):
    """The wrist camera sees the cube 8 mm from where the head camera put it: align moves the tip over it."""
    robot.scene([CUBE], wrist_offset=(0.006, -0.005, 0.0))
    res = robot.run("approach", Approach.Goal(arm="right", object=obj(*CUBE)))
    assert res.result.code == R.SUCCEEDED, res.result.message
    res = robot.run("align", Align.Goal(arm="right", object=obj(*CUBE)), timeout=180.0)
    assert res.result.code == R.SUCCEEDED, res.result.message
    assert res.offset_m < 0.002, res.result.message
    p = res.object_pose.pose.position
    assert abs(p.x - (CUBE[0] + 0.006)) < 0.001 and abs(p.y - (CUBE[1] - 0.005)) < 0.001
    # a cube the wrist camera cannot see is LOST_OBJECT, not a guess
    robot.scene([])
    res = robot.run("align", Align.Goal(arm="right", object=obj(*CUBE)), timeout=60.0)
    assert res.result.code == R.LOST_OBJECT, res.result.message
    robot.scene([CUBE])


# ---- cancel and safety ----
def _right_arm(robot):
    return [robot.joints[f"right_arm_{j}_joint"] for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")]


def test_cancel_is_preempted(robot):
    robot.run("move_arm", MoveArm.Goal(arm="right", named_position="right_rest"))
    h = robot.send("move_arm", MoveArm.Goal(arm="right", named_position="cube_right_standoff", scaling=0.05))
    robot.spin(3.0)
    f = h.cancel_goal_async()
    robot.until(f.done, 10.0)
    res = robot.result(h)
    assert res.result.code == R.PREEMPTED, res.result.message


def test_safety_stop_holds(robot):
    robot.run("move_arm", MoveArm.Goal(arm="right", named_position="right_rest"))
    h = robot.send("move_arm", MoveArm.Goal(arm="right", named_position="cube_right_standoff", scaling=0.05))
    robot.spin(3.0)
    robot.presence_required(True)                          # no presence button in a test: the heartbeat drops
    try:
        res = robot.result(h)
        assert res.result.code == R.SAFETY_STOP, res.result.message
        robot.spin(1.0)
        a = _right_arm(robot)
        robot.spin(2.0)
        b = _right_arm(robot)
        assert max(abs(x - y) for x, y in zip(a, b)) < 0.01, "the arm kept moving after the safety stop"
        res = robot.run("move_arm", MoveArm.Goal(arm="right", named_position="right_rest"))
        assert res.result.code == R.SAFETY_STOP                 # nothing moves without presence
    finally:
        robot.presence_required(False)


def test_one_motion_per_bus(robot):
    robot.run("move_arm", MoveArm.Goal(arm="right", named_position="right_rest"))
    h = robot.send("move_arm", MoveArm.Goal(arm="right", named_position="cube_right_standoff", scaling=0.05))
    robot.spin(1.0)
    second = robot.run("move_arm", MoveArm.Goal(arm="right", named_position="right_rest"))
    assert second.result.code == R.FAILED and "in use" in second.result.message, second.result.message
    robot.result(h)
    robot.run("move_arm", MoveArm.Goal(arm="right", named_position="right_rest"))
