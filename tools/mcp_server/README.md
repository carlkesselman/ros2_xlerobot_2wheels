# XleRobot MCP server

Exposes navigation, both arms, the gripper and the USB cameras as MCP tools, so an agent can
drive the robot directly instead of a human running `tools/*.py` scripts by hand. See
`server.py`'s docstring for the full tool list and what each one needs running.

Scope: orchestration, control and perception, including spatial memory (`remember_object`/
`recall_objects`/`forget_object` - best-effort object positions without a depth sensor, via
plane-assumption or multi-observation triangulation, see `spatial_memory_math.py`).

## Setup

Needs ROS sourced (for `rclpy`, `cv_bridge`, `moveit_msgs`, `nav2_msgs`) plus the `mcp` pip package, which the tools venv
has; see [docs/tools.md](../../docs/tools.md#python-environment).

Register in `.mcp.json` at the repo root (gitignored - it only ever holds a machine-specific
absolute path):

```json
{
  "mcpServers": {
    "xlerobot": {
      "command": "/absolute/path/to/ros2_xlerobot_2wheels/tools/mcp_server/run.sh"
    }
  }
}
```

`run.sh` sources ROS and the venv itself before running `server.py` - confirmed by testing, MCP
stdio clients only pass an allow-listed subset of environment variables to the server process
(`PATH`, `HOME`, ... not `PYTHONPATH`/`AMENT_PREFIX_PATH`/`LD_LIBRARY_PATH`), so `rclpy` is not
visible otherwise, regardless of what's sourced in the shell that started Claude Code. It activates
`$XLEROBOT_VENV` (`~/venvs/xlerobot` by default); edit the ROS `source` line if ROS is installed elsewhere, and make sure
`ROS_DOMAIN_ID` (if you use one) matches the rest of the bringup - add an `export` for it in
`run.sh` alongside the other `source` lines if so.

## Before any motion tool works

1. Bringup with arms and cameras: `ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true with_head:=true cameras:=true`
   (`teleop:=true` is the default, so this already starts joy_node/twist_mux/safety_supervisor;
   pass `teleop:=false` to skip it)
2. `ros2 launch xlerobot_moveit_config move_group.launch.py`
3. `ros2 launch xlerobot_navigation navigation.launch.py localization:=true` (needed for
   `goto_waypoint`/`save_waypoint`, not for arm-only tools)
4. `ros2 launch xlerobot_teleop mcp_presence.launch.py` (needs `/joy` already up from step 1)

Read-only tools (`list_positions`, `list_waypoints`, `get_joint_states`, `get_robot_pose`,
`get_status`, `capture_image`, `recall_objects`) work without step 4 - they don't move
anything, and neither do `remember_object`/`forget_object` (same reasoning, they only read
cached sensor/TF state and write a YAML file).

## Presence heartbeat

Every motion tool checks `/mcp_presence_ok` before accepting a goal and keeps checking it while
one is in flight - see `xlerobot_teleop/mcp_presence_guard.py` and
`xlerobot_teleop/config/mcp_presence.yaml`. Default: hold or tap RB (button 7 - confirmed against
the actual gamepad via `ros2 topic echo /joy`, not a generic-layout guess; if you swap
controllers, re-check the same way) at least once every 5 seconds. A lapse mid-motion cancels the
goal and, for an arm move, e-stops that arm's bus; for navigation, stops the base and cancels the
Nav2 goal.

`estop`/`release_estop` are never gated - always callable, including to recover from a
`PresenceLost` error surfaced by another tool.

## Audit log and replay

Every tool call - success or failure - is appended to `logs/mcp_calls.jsonl` (gitignored,
runtime data) via `call_log.py`: timestamp, tool name, arguments, the calling agent's own
`reason` argument (every tool takes one, optional but always worth filling in), outcome, and
timing. This is the answer to "what did the agent actually do, and why" for reviewing a session
after the fact.

Replay a captured log against a live server:

```bash
python3 tools/mcp_server/replay_log.py logs/mcp_calls.jsonl              # replay everything
python3 tools/mcp_server/replay_log.py logs/mcp_calls.jsonl --dry-run    # just print, don't call
python3 tools/mcp_server/replay_log.py logs/mcp_calls.jsonl --limit 5    # first 5 entries only
```

This replays the *commands*, not their prior outcomes, through the exact same tool surface -
every real safety gate (presence, estop) applies exactly as it did the first time, so a motion
call that succeeded live will be correctly refused on replay if nothing is holding the presence
button now.

## Verification

- Read-only tools first, no presence button held - `get_status`, `list_positions`,
  `capture_image` should all just work.
- A motion tool (e.g. `goto_position` to `rest`) should be refused with a `PresenceLost` error
  with no button held, and succeed with it held.
- Staged presence-loss test (arm supported by hand first): start a `goto_position` move, release
  the presence button mid-motion, confirm the goal cancels and the bus e-stops.
- After any estop, verify with `python3 tools/calibrate_arm.py status --arm <left|right|head>`
  (stack stopped) - `get_status` is a liveness summary, not a torque-state check.
- `capture_image` while localized: confirm `camera_pose_map`/`robot_pose_map` populate. Stop
  `navigation.launch.py` (delocalize on purpose) and confirm a graceful `None` + `pose_note`,
  no exception, no hang.
- `remember_object` on a real object once: expect `"single-observation-plane-assumption"`
  provenance. Move or pan the camera and `remember_object` the same label again: expect
  `"triangulated"` with a plausible residual. `recall_objects` (all, one by label, and a typo'd
  label - the last should raise a `ToolError`). `forget_object`, then call it again on the same
  label - should raise `ToolError`.
