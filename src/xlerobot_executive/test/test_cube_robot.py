"""On the robot: find a cube on the table, approach it, touch it and come back, through the executive.

  XLEROBOT_ROBOT=1 python3 -m pytest src/xlerobot_executive/test/test_cube_robot.py -v

Needs the full stack with object_perception, skills and executive (the TUI's sensing profile), the presence guard
(development mode, or the button held), a cube of about 28 mm within reach of the arm (XLEROBOT_ARM, default right)
on the table in front of the robot, and nothing else in the arm's way. XLEROBOT_CUBE="x,y,z" (map) picks the cube when
several are found; otherwise the one nearest the arm's side is used. About two minutes; skipped without XLEROBOT_ROBOT.
"""
import json
import os
import re
import time

import pytest

import rclpy
from rclpy.action import ActionClient
from xlerobot_skill_msgs.action import Detect
from xlerobot_skill_msgs.srv import SubmitPlan, WaitForEvent

# a module-level pytest.skip here would end the collection of the whole directory under ROS's pytest plugins
pytestmark = pytest.mark.skipif(not os.environ.get("XLEROBOT_ROBOT"), reason="on-robot test: set XLEROBOT_ROBOT=1")

ARM = os.environ.get("XLEROBOT_ARM", "right")
FINAL = {"plan_succeeded", "plan_failed", "plan_aborted"}


class Planner:
    def __init__(self):
        self.node = rclpy.create_node("cube_robot_test")
        self.submit_c = self.node.create_client(SubmitPlan, "/executive/submit_plan")
        self.wait_c = self.node.create_client(WaitForEvent, "/executive/wait_for_event")
        self.detect_a = ActionClient(self.node, Detect, "/skills/detect")
        self.seq = 0

    def wait(self, future, timeout):
        t0 = time.monotonic()
        while not future.done() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert future.done(), "no answer"
        return future.result()

    def detect(self, prompt="a cube"):
        handle = self.wait(self.detect_a.send_goal_async(Detect.Goal(prompt=prompt, label="cube", frames=3)), 15.0)
        assert handle.accepted
        return self.wait(handle.get_result_async(), 60.0).result

    def run(self, plan, timeout=180.0):
        self.seq = self.events(0.0)[1]
        r = self.wait(self.submit_c.call_async(SubmitPlan.Request(plan_json=json.dumps(plan))), 30.0)
        assert r.accepted, r.errors
        seen, t0 = [], time.monotonic()
        while time.monotonic() - t0 < timeout:
            evs, self.seq = self.events(20.0)
            seen += evs
            if any(e["type"] in FINAL for e in evs):
                return seen
        raise AssertionError(f"plan did not finish within {timeout} s; saw {[e['type'] for e in seen]}")

    def events(self, max_wait):
        r = self.wait(self.wait_c.call_async(WaitForEvent.Request(after_seq=self.seq, max_wait_s=max_wait)), max_wait + 10)
        return [json.loads(e) for e in r.events_json], r.last_seq


@pytest.fixture(scope="module")
def planner():
    rclpy.init()
    p = Planner()
    try:
        assert p.submit_c.wait_for_service(timeout_sec=10.0), "executive not running (executive:=true)"
        assert p.detect_a.wait_for_server(timeout_sec=10.0), "skill server not running (skills:=true)"
        yield p
    finally:
        p.node.destroy_node()
        rclpy.try_shutdown()


@pytest.fixture(scope="module")
def cube(planner):
    planner.run({"schema_version": 1, "task": "look down", "steps": [
        {"id": "rest", "skill": "move_arm", "params": {"arm": ARM, "named_position": f"{ARM}_rest"}},
        {"id": "look", "skill": "look_at", "params": {"named_position": "head_look_down"}}]})
    res = planner.detect()
    poses = [[d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z] for d in res.detections.detections]
    assert poses, f"no cube found: {res.result.message}"
    if os.environ.get("XLEROBOT_CUBE"):
        want = [float(v) for v in os.environ["XLEROBOT_CUBE"].split(",")]
        return min(poses, key=lambda p: sum((a - b) ** 2 for a, b in zip(p[:2], want[:2])))
    side = -1.0 if ARM == "right" else 1.0
    return min(poses, key=lambda p: (p[0] - 0.24) ** 2 + (p[1] - side * 0.05) ** 2)


def touch_plan(pose, stay=False):
    return {"schema_version": 1, "task": "touch the cube",
            "objects": {"cube": {"prompt": "a cube", "pose": pose, "frame": "map", "size_m": 0.028}},
            "steps": [
                {"id": "find", "skill": "detect", "params": {"prompt": "a cube", "label": "cube", "bind": "cube"},
                 "on_failure": {"codes": ["NOT_FOUND"], "action": "retry", "max_retries": 2}},
                {"id": "reach", "skill": "approach", "params": {"arm": ARM, "object": "cube"},
                 "on_failure": {"codes": ["NOT_REACHABLE"], "action": "retry", "max_retries": 2},
                 "on_pause": f"retreat:{ARM}_rest"},
                {"id": "touch", "skill": "touch", "params": {"arm": ARM, "object": "cube", "stay": stay},
                 "on_failure": {"action": "goto", "step": "back", "max_retries": 1}},
                {"id": "back", "skill": "move_arm", "params": {"arm": ARM, "named_position": f"{ARM}_rest"}}]}


def nearest(planner, pose):
    res = planner.detect()
    poses = [[d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z] for d in res.detections.detections]
    assert poses, res.result.message
    return min(poses, key=lambda p: (p[0] - pose[0]) ** 2 + (p[1] - pose[1]) ** 2)


def touch_once(planner, pose):
    """Touch the cube and come back; the contact height (mm from the expected top) and how far the cube moved (m)."""
    before = nearest(planner, pose)
    seen = planner.run(touch_plan(before))
    failed = [e for e in seen if e["type"] == "step_failed" and e["step_id"] == "touch"]
    assert not failed, failed[0]["message"]
    assert seen[-1]["type"] == "plan_succeeded", seen[-1]
    msg = next(e["message"] for e in seen if e["type"] == "step_succeeded" and e["step_id"] == "touch")
    m = re.search(r"contact ([+-]\d+) mm from the expected top, (\d+) mm beside its centre, sensed by (\w+)", msg)
    assert m, msg
    after = nearest(planner, before)
    moved = ((after[0] - before[0]) ** 2 + (after[1] - before[1]) ** 2) ** 0.5
    return int(m.group(1)), moved, msg


def test_touch_cube(planner, cube):
    height, moved, msg = touch_once(planner, cube)
    # a touch on top leaves the cube where it was; detection repeats to a few millimetres
    assert moved < 0.008, f"the cube moved {moved * 1000:.0f} mm: {msg}"


def test_touch_cube_again_lands_at_the_same_height(planner, cube):
    runs = [touch_once(planner, cube) for _ in range(2)]
    assert all(moved < 0.008 for _, moved, _ in runs), runs
    assert abs(runs[0][0] - runs[1][0]) <= 6, runs


def test_touch_nothing_is_not_found(planner, cube):
    """A made-up object 4 cm above the cube's height beside it: the descent ends at its travel limit in free air."""
    side = -1.0 if ARM == "right" else 1.0
    ghost = [cube[0], cube[1] - side * 0.06, cube[2] + 0.04]
    seen = planner.run({"schema_version": 1, "task": "touch nothing",
                        "objects": {"ghost": {"prompt": "a cube", "pose": ghost, "frame": "map", "size_m": 0.028}},
                        "steps": [
                            # approach would re-detect the object, so go above it by coordinates
                            {"id": "above", "skill": "move_arm", "params": {
                                "arm": ARM, "point": [ghost[0], ghost[1], ghost[2] + 0.064], "frame": "map", "tool_point": True},
                             "on_failure": {"codes": ["NOT_REACHABLE"], "action": "retry", "max_retries": 2}},
                            {"id": "touch", "skill": "touch", "params": {"arm": ARM, "object": "ghost"},
                             "on_failure": {"codes": ["NOT_FOUND"], "action": "goto", "step": "back", "max_retries": 1}},
                            {"id": "back", "skill": "move_arm", "params": {"arm": ARM, "named_position": f"{ARM}_rest"}}]})
    fails = [e for e in seen if e["type"] == "step_failed" and e["step_id"] == "touch"]
    assert [e["code"] for e in fails] == ["NOT_FOUND"], fails
    assert seen[-1]["type"] == "plan_succeeded", seen[-1]
