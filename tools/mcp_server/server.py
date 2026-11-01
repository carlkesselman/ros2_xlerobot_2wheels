#!/usr/bin/env python3
"""MCP server exposing XleRobot navigation, both arms, the gripper and cameras.

Run via .mcp.json (see README.md), or standalone for a quick check:
  python3 -m mcp_server.server

Needs ROS sourced (rclpy, cv_bridge, moveit_msgs, nav2_msgs, ...) plus the `mcp` pip package -
see README.md for the venv setup. Needs bringup running (with_arms:=true cameras:=true),
move_group, navigation.launch.py localization:=true, and mcp_presence.launch.py (see
xlerobot_teleop) for any motion tool to work.

Tool surface: read-only tools (list_positions, list_waypoints, get_joint_states, get_robot_pose,
get_status, capture_image, recall_objects) work with no presence button held, as do
remember_object/forget_object (they only read cached sensor/TF state and write a YAML file,
they don't move anything). Every motion tool (goto_position, move_to_pose, move_gripper,
play_sequence, goto_waypoint, save_position, save_waypoint) requires /mcp_presence_ok and is
cancelled + the arm's bus e-stopped (or the base stopped) if presence lapses mid-motion.
estop/release_estop are never gated - always callable, including to recover from a
PresenceLost error.

Every tool takes an optional `reason` argument and every call is appended to
logs/mcp_calls.jsonl (tool, arguments, reason, outcome, timing) via call_log.py - see
replay_log.py to play a captured log back against a live server.
"""
import functools
import os
import time

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError

import bridge
import call_log
from bridge import MoveError, PresenceLost

mcp = MCPServer("xlerobot")


def catch_bridge_errors(fn):
    """MCPServer treats a bare exception as a crash (UnexpectedToolError, message withheld from
    the model, logged at ERROR) - these are anticipated operational failures (bad presence
    state, a rejected plan, an unknown saved name, a bad argument), not bugs, so re-raise them
    as ToolError: the model sees the real message, no traceback, logged at INFO. LookupError
    covers both position_store.NotFoundError/waypoint_store.NotFoundError and a plain KeyError
    from an invalid group/arm name."""
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except (MoveError, PresenceLost, ValueError, LookupError) as e:
            raise ToolError(str(e)) from e
    return wrapper


def logged(fn):
    """Append every call (success or failure) to logs/mcp_calls.jsonl via call_log.py - the
    calling agent's own `reason` argument goes in verbatim, so the log doubles as a record of
    why each command was issued, not just what it was. See replay_log.py to replay one."""
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        reason = kwargs.get("reason", "")
        log_args = {k: v for k, v in kwargs.items() if k != "reason"}
        t0 = time.monotonic()
        try:
            result = fn(*args, **kwargs)
            call_log.record(fn.__name__, log_args, reason, ok=True,
                             duration_s=time.monotonic() - t0, result=result)
            return result
        except BaseException as e:
            call_log.record(fn.__name__, log_args, reason, ok=False,
                             duration_s=time.monotonic() - t0, error=str(e))
            raise
    return wrapper


@mcp.tool()
@logged
@catch_bridge_errors
def list_positions(reason: str = "") -> dict:
    """List saved named arm/gripper/head joint positions (src/xlerobot_bringup/config/saved_positions.yaml)."""
    return bridge.get_bridge().list_positions()


@mcp.tool()
@logged
@catch_bridge_errors
def list_waypoints(reason: str = "") -> dict:
    """List saved named base waypoints, map-frame x/y/yaw (src/xlerobot_bringup/config/saved_waypoints.yaml)."""
    return bridge.get_bridge().list_waypoints()


@mcp.tool()
@logged
@catch_bridge_errors
def get_joint_states(group: str = "", reason: str = "") -> dict:
    """Current joint positions (radians). group is a MoveIt group name (left_arm, right_arm,
    left_gripper, right_gripper, head, both_arms) or omitted for every cached joint."""
    return bridge.get_bridge().get_joint_states(group or None)


@mcp.tool()
@logged
@catch_bridge_errors
def get_robot_pose(reason: str = "") -> dict | None:
    """Current localized base pose (map frame), from /amcl_pose. None if not localized yet."""
    return bridge.get_bridge().get_robot_pose()


@mcp.tool()
@logged
@catch_bridge_errors
def get_status(reason: str = "") -> dict:
    """Liveness summary: presence heartbeat, joint_states freshness, localization, which cameras
    have a publisher (cameras_live says one is advertising, not that frames are fresh; use
    capture_image for that). NOT a torque-state check - see tools/calibrate_arm.py status for that."""
    return bridge.get_bridge().get_status()


@mcp.tool()
@logged
@catch_bridge_errors
def capture_image(camera: str, reason: str = "") -> dict:
    """Grab the latest frame from a camera (head, left, right - the arm wrist cams) and save it
    as a PNG under captures/. Returns path/width/height plus spatial context: the camera's own
    pose in map frame (camera_pose_map, from TF - None with a pose_note if not localized yet)
    and the robot's base pose (robot_pose_map). No presence needed - this doesn't move
    anything."""
    output = bridge.CAPTURE_DIR / f"{camera}_{int(time.time())}.png"
    return bridge.get_bridge().capture_image(camera, output)


@mcp.tool()
@logged
@catch_bridge_errors
def goto_position(name: str, scaling: float = 0.0, reason: str = "") -> str:
    """Plan and execute a move to a saved named joint position (collision-aware, via MoveIt).
    scaling 0 uses joint_limits.yaml's default. Requires the presence heartbeat."""
    return bridge.get_bridge().goto_position(name, scaling)


@mcp.tool()
@logged
@catch_bridge_errors
def move_to_pose(arm: str, x: float, y: float, z: float, frame: str = "base_footprint",
                  scaling: float = 0.0, reason: str = "") -> str:
    """Move an arm's gripper to a Cartesian point (position-only - the SO-101 arms' IK has no
    orientation control, gripper orientation follows whatever the IK solution lands on). arm is
    'left' or 'right'. Requires the presence heartbeat."""
    return bridge.get_bridge().move_to_pose(arm, x, y, z, frame, scaling)


@mcp.tool()
@logged
@catch_bridge_errors
def move_gripper(arm: str, action: str, reason: str = "") -> str:
    """Open or close an arm's gripper. arm is 'left' or 'right', action is 'open' or 'close'.
    Requires the presence heartbeat."""
    return bridge.get_bridge().move_gripper(arm, action)


@mcp.tool()
@logged
@catch_bridge_errors
def play_sequence(names: list[str], loop: str = "1", scaling: float = 0.0, reason: str = "") -> str:
    """Move through several saved named positions in order. loop is a repeat count or 'forever'
    (use with care - only cancellable via presence loss or estop). Requires the presence heartbeat."""
    return bridge.get_bridge().play_sequence(names, loop, scaling)


@mcp.tool()
@logged
@catch_bridge_errors
def goto_waypoint(name: str, timeout: float = 120.0, reason: str = "") -> str:
    """Navigate to a saved named base waypoint via Nav2. Requires navigation.launch.py running
    with localization:=true, and the presence heartbeat."""
    return bridge.get_bridge().goto_waypoint(name, timeout)


@mcp.tool()
@logged
@catch_bridge_errors
def save_position(name: str, group: str, reason: str = "") -> str:
    """Save the arm/gripper/head's current joint positions under a name, for later goto_position/
    play_sequence calls. Requires the presence heartbeat (it's not motion, but teaching a position
    you'll later replay deserves the same "someone's watching" guarantee)."""
    return bridge.get_bridge().save_position(name, group)


@mcp.tool()
@logged
@catch_bridge_errors
def save_waypoint(name: str, reason: str = "") -> str:
    """Save the robot's current localized pose as a named waypoint, for later goto_waypoint
    calls. Requires /amcl_pose and the presence heartbeat."""
    return bridge.get_bridge().save_waypoint(name)


@mcp.tool()
@logged
@catch_bridge_errors
def remember_object(label: str, description: str, camera: str, pixel_x: int | None = None,
                     pixel_y: int | None = None, plane_height_m: float | None = None,
                     reason: str = "") -> dict:
    """Capture a frame from `camera` and record it as an observation of `label` (creates the
    entry if new, appends if `label` already exists - same label = same object, this is agent-
    driven identity, no automatic re-identification). pixel_x/pixel_y default to the image
    center. Returns the updated entry, including a best-effort 3D position estimate: a single
    observation assumes a horizontal plane at plane_height_m (default ~0.75m, override for
    floor/shelf items); a second observation from a different-enough viewpoint triangulates a
    real depth instead - check the returned provenance/confidence fields either way, a low-
    confidence plane-assumption estimate is still returned, never withheld. No presence needed -
    this doesn't move anything, same as capture_image/get_status."""
    return bridge.get_bridge().remember_object(label, description, camera, pixel_x, pixel_y, plane_height_m)


@mcp.tool()
@logged
@catch_bridge_errors
def recall_objects(label: str = "", reason: str = "") -> dict:
    """List remembered objects (or one by label) with their position estimate, provenance/
    confidence, and full observation history. No presence needed."""
    return bridge.get_bridge().recall_objects(label or None)


@mcp.tool()
@logged
@catch_bridge_errors
def forget_object(label: str, reason: str = "") -> dict:
    """Delete a remembered object. This is agent working memory, not curated config - plain
    deletion, no confirmation/undo. Does not delete the referenced image files under captures/.
    No presence needed."""
    return bridge.get_bridge().forget_object(label)


# -- plans (ADR-0006): the planner submits a plan and follows its events --------------------------

@mcp.tool()
@logged
@catch_bridge_errors
def submit_plan(plan: dict, reason: str = "") -> dict:
    """Run a task as a plan on the robot's executive. `plan` is a JSON object (schema version 1):
    {"schema_version": 1, "task": "...", "objects": {name: {"prompt", "pose": [x, y, z], "frame", "size_m"}},
     "steps": [{"id", "skill", "params", "pre", "expect", "on_failure", "on_pause", "timeout_s"}, ...]}.
    Skills: navigate_to, look_at, detect (params.bind sets an object's pose), move_arm, approach, touch. Checks:
    {"detected": object}, {"joint_near": {"position", "tol"}}. on_failure: {"codes": [...], "action":
    "retry"|"goto"|"abort", "max_retries": 1-5, "step"}. Full rules: src/xlerobot_executive/README.md.
    Returns accepted, plan_id, errors (every problem when refused) and last_seq: then call wait_for_event with
    after_seq=last_seq, at least once a minute while the plan runs (otherwise the robot retreats and aborts).
    Requires the presence heartbeat."""
    return bridge.get_bridge().submit_plan(plan)


@mcp.tool()
@logged
@catch_bridge_errors
def wait_for_event(after_seq: int = 0, max_wait_s: float = 25.0, reason: str = "") -> dict:
    """Events of the running plan after `after_seq` (step_started, step_succeeded, step_failed with code and message,
    paused, resumed, plan_succeeded / plan_failed / plan_aborted), waiting up to max_wait_s (at most 30) for one,
    plus the executive's state and last_seq for the next call. Keeps the plan's lease alive."""
    return bridge.get_bridge().wait_for_event(after_seq, max_wait_s)


@mcp.tool()
@logged
@catch_bridge_errors
def get_execution_state(reason: str = "") -> dict:
    """The executive's state now (status, plan, current step, objects) and last_seq, without waiting."""
    return bridge.get_bridge().wait_for_event(0, 0.0) | {"events": []}


@mcp.tool()
@logged
@catch_bridge_errors
def control_plan(command: str, reason: str = "") -> str:
    """pause, resume or abort the running plan. Pausing holds the arm (or retreats it, per the step's on_pause);
    resume re-runs the paused step (requires the presence heartbeat)."""
    return bridge.get_bridge().control_plan(command)


@mcp.tool()
@logged
@catch_bridge_errors
def describe_scene(prompt: str, label: str = "", reason: str = "") -> dict:
    """Find objects for a text prompt (e.g. "a cube") with the head camera and return them numbered: id, label,
    score, pose [x, y, z] and frame (map when localised), size. `image` is a head camera frame with the same
    numbers drawn on the objects, to look at when choosing which one a plan should use; put the chosen object's
    pose into the plan's objects. No presence needed."""
    return bridge.get_bridge().describe_scene(prompt, label)


@mcp.tool()
@logged
@catch_bridge_errors
def estop(reason: str = "") -> dict:
    """Emergency-stop every servo bus (arms, head, wheels lose torque/command). Never gated -
    always callable. Follow up with tools/calibrate_arm.py status (stack stopped) to verify."""
    return bridge.get_bridge().estop(active=True)


@mcp.tool()
@logged
@catch_bridge_errors
def release_estop(reason: str = "") -> dict:
    """Release e-stop on every servo bus - joints hold where they are, then move back toward
    their last commanded trajectory at the speed cap. Never gated - always callable."""
    return bridge.get_bridge().estop(active=False)


if __name__ == "__main__":
    try:
        mcp.run()
    finally:
        # See bridge.shutdown()'s docstring: a clean rclpy shutdown here still isn't enough to
        # avoid an abort during CPython's own interpreter teardown, confirmed reproducible - skip
        # that teardown entirely once ROS is properly torn down. Every tool result was already
        # delivered to the client before this point regardless of how mcp.run() exited.
        bridge.shutdown()
        os._exit(0)
