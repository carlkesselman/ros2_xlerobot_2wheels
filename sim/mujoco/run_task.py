#!/usr/bin/env python3
"""Run a plan on the executive and follow it to the end, printing each event: the planner's side of ADR-0006,
from a file instead of a language model.

    python3 sim/mujoco/run_task.py sim/mujoco/tasks/table_to_crate.json

Needs the executive (mujoco_twin.launch.py skills:=true) and, for navigate_to steps, Nav2. Exit code 0 when the
plan succeeds. Ctrl-C aborts the plan.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from xlerobot_skill_msgs.srv import ExecutionControl, SubmitPlan, WaitForEvent


def call(node, client, req, timeout=40.0):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"{client.srv_name} is not available")
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done():
        raise RuntimeError(f"{client.srv_name} did not answer")
    return fut.result()


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    plan = open(sys.argv[1]).read()
    rclpy.init()
    node = Node("run_task")
    submit = node.create_client(SubmitPlan, "/executive/submit_plan")
    wait = node.create_client(WaitForEvent, "/executive/wait_for_event")
    control = node.create_client(ExecutionControl, "/executive/control")
    res = call(node, submit, SubmitPlan.Request(plan_json=plan))
    if not res.accepted:
        print("refused:\n  " + "\n  ".join(res.errors))
        return 2
    print(f"plan {res.plan_id} accepted: {json.loads(plan).get('task', '')}")
    t0, seq, status = time.monotonic(), 0, None
    try:
        while status is None:
            r = call(node, wait, WaitForEvent.Request(after_seq=seq, max_wait_s=20.0))
            seq = r.last_seq
            for e in map(json.loads, r.events_json):
                details = {k: v for k, v in e.items() if k not in ("seq", "time", "type", "plan_id", "step_id", "attempt")}
                print(f"{time.monotonic() - t0:6.1f} s  {e['type']:15s} {e.get('step_id') or '':20s} "
                      f"{json.dumps(details) if details else ''}")
                if e["type"] in ("plan_succeeded", "plan_failed", "plan_aborted"):
                    status = e["type"]
    except KeyboardInterrupt:
        call(node, control, ExecutionControl.Request(command="abort"))
        print("aborted")
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if status == "plan_succeeded" else 1


if __name__ == "__main__":
    sys.exit(main())
