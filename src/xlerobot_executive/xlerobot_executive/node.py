#!/usr/bin/env python3
"""The plan executive (ADR-0006): runs a planner's plan as a sequence of skills.

  ros2 launch xlerobot_executive executive.launch.py

Services:
  /executive/submit_plan     (xlerobot_skill_msgs/srv/SubmitPlan)       validate and start a plan (schema in plan.py)
  /executive/wait_for_event  (xlerobot_skill_msgs/srv/WaitForEvent)     long poll for events; renews the planner's lease
  /executive/control         (xlerobot_skill_msgs/srv/ExecutionControl) pause, resume, abort

The planner holds a lease: while a plan runs or is paused, it must call wait_for_event at least every `lease_s`
seconds. Without it (the agent ended or lost its connection) the executive stops the current step, sends the arms to
their retreat positions and aborts the plan. A paused plan is given up the same way after `max_hold_s`. A safety stop
from a skill, or the safety supervisor's e-stop, aborts the plan and is never retried.
"""
import json
import os
import threading
import time
import uuid

import numpy as np
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from xlerobot_skill_msgs.action import Approach, Detect, LookAt, MoveArm, NavigateTo, Touch
from xlerobot_skill_msgs.msg import ObjectRef
from xlerobot_skill_msgs.srv import ExecutionControl, SubmitPlan, WaitForEvent

from . import plan as P

ACTIONS = {"navigate_to": NavigateTo, "look_at": LookAt, "detect": Detect, "move_arm": MoveArm, "approach": Approach,
           "touch": Touch}
CODE_NAMES = P.CODES                                  # SkillResult code number -> name (same order as the message)


class Abort(Exception):
    pass


class Executive(Node):
    def __init__(self):
        super().__init__("executive")
        Pm = self.declare_parameter
        self.lease_s = float(Pm("lease_s", 60.0).value)
        self.max_hold_s = float(Pm("max_hold_s", 120.0).value)
        self.retreat = list(Pm("retreat_positions", ["left_rest", "right_rest"]).value)
        self.positions_file = Pm("positions_file", "").value or self._share("saved_positions.yaml")
        self.waypoints_file = self._share("saved_waypoints.yaml")
        self.cb = ReentrantCallbackGroup()
        self.actions = {n: ActionClient(self, t, f"/skills/{n}", callback_group=self.cb) for n, t in ACTIONS.items()}
        self.create_service(SubmitPlan, "/executive/submit_plan", self._on_submit, callback_group=self.cb)
        self.create_service(WaitForEvent, "/executive/wait_for_event", self._on_wait, callback_group=self.cb)
        self.create_service(ExecutionControl, "/executive/control", self._on_control, callback_group=self.cb)
        self.create_subscription(Bool, "/e_stop", self._on_estop, 10, callback_group=self.cb)
        self.lock = threading.Condition()
        self.events, self.seq = [], 0
        self.status, self.plan, self.step_index, self.attempt = "idle", None, None, 0
        self.last_contact = time.monotonic()
        self.paused_at = None
        self.request = None                          # "pause", "abort", "give_up" while running
        self.skill_handle = None                           # the running skill goal
        self.create_timer(1.0, self._watchdog, callback_group=self.cb)
        self.get_logger().info("executive ready: /executive/{submit_plan, wait_for_event, control}")

    @staticmethod
    def _share(name):
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("xlerobot_bringup"), "config", name)

    def _names(self, path, key):
        try:
            with open(path) as f:
                return set((yaml.safe_load(f) or {}).get(key, {}) or {})
        except OSError:
            return set()

    # ---- events and state ----
    def _event(self, kind, **details):
        with self.lock:
            self.seq += 1
            step = self._step()
            ev = {"seq": self.seq, "time": time.time(), "type": kind, "plan_id": self.plan and self.plan["plan_id"],
                  "step_id": step and step["id"], "attempt": self.attempt, **details}
            self.events.append(ev)
            del self.events[:-500]
            self.lock.notify_all()
        self.get_logger().info(f"event {kind} {json.dumps(details)[:200]}")
        return ev

    def _step(self):
        if self.plan is None or self.step_index is None or self.step_index >= len(self.plan["steps"]):
            return None
        return self.plan["steps"][self.step_index]

    def _state(self):
        step = self._step()
        return {"status": self.status, "plan_id": self.plan and self.plan["plan_id"], "task": self.plan and self.plan.get("task"),
                "step_id": step and step["id"], "step_index": self.step_index,
                "steps": [s["id"] for s in self.plan["steps"]] if self.plan else [], "attempt": self.attempt,
                "lease_left_s": round(self.lease_s - (time.monotonic() - self.last_contact), 1),
                "objects": self.plan.get("objects", {}) if self.plan else {}}

    # ---- services ----
    def _on_submit(self, req, res):
        try:
            p = json.loads(req.plan_json)
        except json.JSONDecodeError as e:
            res.errors = [f"not JSON: {e}"]
            return res
        with self.lock:
            if self.status in ("running", "paused"):
                res.errors = [f"a plan is {self.status} ({self.plan['plan_id']}); abort it first"]
                return res
        errors = P.validate(p, self._names(self.positions_file, "positions"), self._names(self.waypoints_file, "waypoints"))
        if errors:
            res.errors = errors
            return res
        p["plan_id"] = p.get("plan_id") or uuid.uuid4().hex[:8]
        p.setdefault("objects", {})
        with self.lock:
            self.plan, self.step_index, self.attempt, self.status, self.request = p, 0, 0, "running", None
            self.last_contact = time.monotonic()
        self._event("plan_accepted", task=p.get("task", ""), steps=[s["id"] for s in p["steps"]])
        threading.Thread(target=self._run, daemon=True).start()
        res.accepted, res.plan_id = True, p["plan_id"]
        return res

    def _on_wait(self, req, res):
        deadline = time.monotonic() + max(0.0, min(float(req.max_wait_s), 30.0))
        with self.lock:
            self.last_contact = time.monotonic()
            while True:
                new = [e for e in self.events if e["seq"] > req.after_seq]
                if new or time.monotonic() >= deadline:
                    break
                self.lock.wait(timeout=deadline - time.monotonic())
            res.events_json = [json.dumps(e) for e in new]
            res.last_seq = self.seq
            res.state_json = json.dumps(self._state())
            self.last_contact = time.monotonic()
        return res

    def _on_control(self, req, res):
        cmd = req.command.strip().lower()
        with self.lock:
            self.last_contact = time.monotonic()
            if cmd == "pause" and self.status == "running":
                self.request = "pause"
            elif cmd == "resume" and self.status == "paused":
                self.request = "resume"
            elif cmd == "abort" and self.status in ("running", "paused"):
                self.request = "abort"
            else:
                res.ok, res.message = False, f"cannot {cmd} while {self.status}"
                return res
            self.lock.notify_all()
        if cmd in ("pause", "abort"):
            self._cancel_skill()
        res.ok, res.message = True, f"{cmd} requested"
        return res

    def _on_estop(self, m):
        if m.data and self.status in ("running", "paused"):
            with self.lock:
                self.request = "estop"
                self.lock.notify_all()
            self._cancel_skill()

    def _watchdog(self):
        with self.lock:
            if self.status not in ("running", "paused") or self.request in ("give_up", "abort", "estop"):
                return
            lease = time.monotonic() - self.last_contact > self.lease_s
            hold = self.status == "paused" and self.paused_at and time.monotonic() - self.paused_at > self.max_hold_s
            if not (lease or hold):
                return
            self.request = "give_up"
            self.give_up_reason = "lease expired: no planner contact" if lease else "paused too long"
            self.lock.notify_all()
        self._cancel_skill()

    # ---- running ----
    def _cancel_skill(self):
        h = self.skill_handle
        if h is not None:
            h.cancel_goal_async()

    def _wait_future(self, future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda f: done.set())
        if not done.wait(timeout):
            return None
        return future.result()

    def _call(self, skill, goal, timeout=300.0):
        """(code name, message, result) of one skill goal."""
        client = self.actions[skill]
        if not client.wait_for_server(timeout_sec=5.0):
            return "FAILED", f"/skills/{skill} is not available", None
        h = self._wait_future(client.send_goal_async(goal), 15.0)
        if h is None or not h.accepted:
            return "FAILED", f"/skills/{skill} refused the goal", None
        self.skill_handle = h
        try:
            r = self._wait_future(h.get_result_async(), timeout)
            if r is None:
                self._wait_future(h.cancel_goal_async(), 5.0)
                return "TIMEOUT", f"no result within {timeout:.0f} s", None
            res = r.result
            return CODE_NAMES[res.result.code], res.result.message, res
        finally:
            self.skill_handle = None

    def _ref(self, name):
        o = self.plan["objects"][name]
        ref = ObjectRef(prompt=o["prompt"], size_m=float(o.get("size_m", 0.0)), gate_m=float(o.get("gate_m", 0.0)))
        ref.pose.header.frame_id = o.get("frame", "map")
        ref.pose.pose.position.x, ref.pose.pose.position.y, ref.pose.pose.position.z = (float(v) for v in o["pose"])
        ref.pose.pose.orientation.w = 1.0
        return ref

    def _point(self, target, xyz, frame):
        target.header.frame_id = frame
        target.point.x, target.point.y, target.point.z = (float(v) for v in xyz)

    def _goal(self, skill, params):
        g = ACTIONS[skill].Goal()
        if skill == "navigate_to":
            g.waypoint = params.get("waypoint", "")
            if "pose" in params:
                x, y, yaw = params["pose"]
                g.pose.header.frame_id = params.get("frame", "map")
                g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
                g.pose.pose.orientation.z, g.pose.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
            g.timeout_s = float(params.get("timeout_s", 0.0))
        elif skill == "look_at":
            g.named_position = params.get("named_position", "")
            if "target" in params:
                self._point(g.target, params["target"], params.get("frame", "base_footprint"))
            if "object" in params:
                o = self.plan["objects"][params["object"]]
                self._point(g.target, o["pose"], o.get("frame", "map"))
            g.tolerance_deg = float(params.get("tolerance_deg", 0.0))
        elif skill == "detect":
            g.prompt, g.label, g.size_m = params["prompt"], params.get("label", ""), float(params.get("size_m", 0.0))
        elif skill == "move_arm":
            g.arm, g.named_position = params["arm"], params.get("named_position", "")
            if "point" in params:
                self._point(g.point, params["point"], params.get("frame", "base_footprint"))
            g.tool_point, g.table_z = bool(params.get("tool_point", False)), float(params.get("table_z", 0.0))
            g.scaling, g.timeout_s = float(params.get("scaling", 0.0)), float(params.get("timeout_s", 0.0))
        elif skill == "approach":
            g.arm, g.object = params["arm"], self._ref(params["object"])
            g.height_m, g.scaling = float(params.get("height_m", 0.0)), float(params.get("scaling", 0.0))
        elif skill == "touch":
            g.arm, g.object = params["arm"], self._ref(params["object"])
            g.max_travel_m, g.step_m = float(params.get("max_travel_m", 0.0)), float(params.get("step_m", 0.0))
            g.stay = bool(params.get("stay", False))
        return g

    def _bind(self, name, result):
        """After detect: the object binding takes the detection nearest to its previous pose (the best one without)."""
        dets = result.detections.detections
        if not dets:
            return
        o = self.plan["objects"][name]
        pts = [np.array([d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z]) for d in dets]
        if "pose" in o and o.get("frame", "map") == result.detections.header.frame_id:
            k = int(np.argmin([np.linalg.norm(p[:2] - np.array(o["pose"][:2])) for p in pts]))
        else:
            k = int(np.argmax([d.results[0].hypothesis.score if d.results else 0.0 for d in dets]))
        o["pose"], o["frame"] = [round(float(v), 4) for v in pts[k]], result.detections.header.frame_id
        if not o.get("size_m"):
            o["size_m"] = float(dets[k].bbox.size.x)

    def _joints(self, timeout=3.0):
        got = {}
        ev = threading.Event()

        def cb(m):
            got.update(zip(m.name, m.position))
            ev.set()
        sub = self.create_subscription(JointState, "/joint_states", cb, 10, callback_group=self.cb)
        ev.wait(timeout)
        self.destroy_subscription(sub)
        return got

    def _check(self, checks):
        """(ok, why not) for a step's pre or expect checks."""
        for c in checks or []:
            (name, arg), = c.items()
            if name == "joint_near":
                try:
                    with open(self.positions_file) as f:
                        pos = (yaml.safe_load(f) or {})["positions"][arg["position"]]["joints"]
                except (OSError, KeyError) as e:
                    return False, f"joint_near: {e}"
                now = self._joints()
                off = max(abs(now.get(j, 1e9) - v) for j, v in pos.items())
                if off > float(arg.get("tol", 0.05)):
                    return False, f"joints {off:.3f} rad from {arg['position']}"
            elif name == "detected":
                code, msg, res = self._call("detect", self._goal("detect", {"prompt": self.plan["objects"][arg]["prompt"]}), 60.0)
                if code != "SUCCEEDED":
                    return False, f"detected({arg}): {msg}"
                o = self.plan["objects"][arg]
                if "pose" in o and o.get("frame", "map") == res.detections.header.frame_id:
                    gate = float(o.get("gate_m", 0.05) or 0.05)
                    near = [d for d in res.detections.detections
                            if np.hypot(d.bbox.center.position.x - o["pose"][0], d.bbox.center.position.y - o["pose"][1]) <= gate]
                    if not near:
                        return False, f"detected({arg}): nothing within {gate * 100:.0f} cm of its last pose"
        return True, ""

    def _requested(self):
        with self.lock:
            return self.request

    def _pause(self, step):
        """Apply the step's on_pause, then wait for resume, abort or the hold limit."""
        how = step.get("on_pause", "hold")
        if how.startswith("retreat:"):
            pos = how[8:]
            self._call("move_arm", self._goal("move_arm", {"arm": "left" if pos.startswith("left") else "right",
                                                            "named_position": pos}), 120.0)
        with self.lock:
            self.status, self.paused_at = "paused", time.monotonic()
        self._event("paused", on_pause=how)
        with self.lock:
            while self.request not in ("resume", "abort", "give_up", "estop"):
                self.lock.wait(timeout=1.0)
            req = self.request
            self.request = None if req == "resume" else req
            self.paused_at = None
            if req == "resume":
                self.status = "running"
        if req == "resume":
            self._event("resumed")
            return
        raise Abort(req)

    def _give_up(self, why):
        for pos in self.retreat:
            self._call("move_arm", self._goal("move_arm", {"arm": "left" if pos.startswith("left") else "right",
                                                            "named_position": pos}), 120.0)
        self._finish("aborted", reason=why)

    def _finish(self, status, **details):
        with self.lock:
            self.status = status
        self._event(f"plan_{status}", **details)

    def _run(self):
        steps = self.plan["steps"]
        ids = [s["id"] for s in steps]
        budget = {}                                  # step id -> retries and gotos used
        try:
            while self.step_index < len(steps):
                step = steps[self.step_index]
                if self._requested() == "pause":
                    self._pause(step)
                if self._requested() in ("abort", "give_up", "estop"):
                    raise Abort(self._requested())
                self._event("step_started", skill=step["skill"])
                ok, why = self._check(step.get("pre"))
                if ok:
                    code, msg, res = self._call(step["skill"], self._goal(step["skill"], step.get("params", {})),
                                                float(step.get("timeout_s", 300.0)))
                else:
                    code, msg, res = "FAILED", f"pre check failed: {why}", None
                req = self._requested()
                if code == "PREEMPTED" and req == "pause":
                    self._pause(step)                # re-run the same step after resume (pre is checked again)
                    continue
                if req in ("abort", "give_up", "estop"):
                    raise Abort(req)
                if code == "SUCCEEDED":
                    if step["skill"] == "detect" and "bind" in step.get("params", {}):
                        self._bind(step["params"]["bind"], res)
                    ok, why = self._check(step.get("expect"))
                    if ok:
                        self._event("step_succeeded", message=msg)
                        self.step_index, self.attempt = self.step_index + 1, 0
                        continue
                    code, msg = "FAILED", f"expect check failed: {why}"
                if code == "SAFETY_STOP":
                    self._event("step_failed", code=code, message=msg)
                    self._finish("aborted", reason=f"safety stop: {msg}")
                    return
                action, f = P.handles(step, code)
                used = budget.get(step["id"], 0)
                if action in ("retry", "goto") and used < int(f.get("max_retries", 1)):
                    budget[step["id"]] = used + 1
                    self._event("step_failed", code=code, message=msg, then=action, retries_left=int(f.get("max_retries", 1)) - used - 1)
                    if action == "goto":
                        self.step_index = ids.index(f["step"])
                    self.attempt += 1
                    continue
                self._event("step_failed", code=code, message=msg, then="stop")
                self._finish("failed", step_id=step["id"], code=code, message=msg)
                return
            self._finish("succeeded")
        except Abort as a:
            why = str(a)
            if why == "give_up":
                self._give_up(getattr(self, "give_up_reason", "given up"))
            else:
                self._finish("aborted", reason="e-stop" if why == "estop" else "aborted by the planner")
        except Exception as e:                       # noqa: BLE001
            self.get_logger().error(f"plan crashed: {e!r}")
            self._finish("failed", code="FAILED", message=f"executive error: {e}")


def main():
    rclpy.init()
    node = Executive()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
