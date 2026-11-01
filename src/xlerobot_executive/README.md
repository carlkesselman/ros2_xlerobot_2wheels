# xlerobot_executive

Runs a planner's plan as a sequence of skills ([ADR-0006](../../docs/decisions/0006-plan-execute-watch.md)): validates
it, runs the steps with their checks and failure handling, and reports every step as an event. The planner (a language
model through MCP, or a script) submits a plan and then follows the events. The MCP server's `submit_plan`,
`wait_for_event`, `get_execution_state`, `control_plan` and `describe_scene` tools are thin clients of these services.

```bash
ros2 launch xlerobot_executive executive.launch.py         # needs the skill server (xlerobot_skills)
```

| Service | Type | |
|---|---|---|
| `/executive/submit_plan` | `xlerobot_skill_msgs/srv/SubmitPlan` | validate and start a plan; the errors if it is refused |
| `/executive/wait_for_event` | `xlerobot_skill_msgs/srv/WaitForEvent` | long poll (at most 30 s) for events after a sequence number, and the state; renews the lease |
| `/executive/control` | `xlerobot_skill_msgs/srv/ExecutionControl` | `pause`, `resume`, `abort` |

## Plans

JSON, schema version 1 (full rules in `xlerobot_executive/plan.py`):

```json
{
  "schema_version": 1,
  "task": "touch the yellow cube",
  "objects": {"cube": {"prompt": "a cube", "size_m": 0.028}},
  "steps": [
    {"id": "look", "skill": "look_at", "params": {"named_position": "head_look_down"}},
    {"id": "find", "skill": "detect", "params": {"prompt": "a cube", "bind": "cube"},
     "on_failure": {"codes": ["NOT_FOUND"], "action": "retry", "max_retries": 2}},
    {"id": "reach", "skill": "approach", "params": {"arm": "right", "object": "cube"},
     "expect": [{"detected": "cube"}], "on_pause": "retreat:right_rest"},
    {"id": "touch", "skill": "touch", "params": {"arm": "right", "object": "cube"}},
    {"id": "back", "skill": "move_arm", "params": {"arm": "right", "named_position": "right_rest"}}
  ]
}
```

- **Skills**: `navigate_to`, `look_at`, `detect`, `move_arm`, `approach`, `touch`, `align` with their parameters (see
  the skill server's README). Objects are named in `objects`; `detect` with `bind` sets an object's pose from the
  detection nearest to its previous pose, `align` sets it from the wrist camera's measurement, and later steps refer to
  the object by name.
- **Checks**: `pre` before a step and `expect` after it, from a fixed list: `{"detected": object}`,
  `{"joint_near": {"position": saved position, "tol": rad}}`. A failed check is a failure with code `FAILED`.
- **Failures**: `on_failure` with `codes` (empty = any), `action` `retry`, `goto` (a step id) or `abort` (default),
  and `max_retries` (1-5). A failure the step does not handle stops the plan and is reported. `SAFETY_STOP` and
  `PREEMPTED` can never be handled by a plan: a safety stop aborts it.
- **Pause**: `on_pause` per step: `hold` (default), `retreat:<saved position>`, `stop_base`. A pause cancels the
  running skill (the arm holds), applies `on_pause` and waits; `resume` runs the step again from its `pre` checks.

Validation refuses a plan with unknown skills or parameters, missing objects, unknown saved positions or waypoints,
unbounded retries, or a handled safety code, and lists every problem. One plan runs at a time.

## Events and the lease

Events are JSON objects with `seq`, `time`, `type`, `plan_id`, `step_id`, `attempt` and details: `plan_accepted`,
`step_started`, `step_succeeded`, `step_failed` (with `code`, `message` and `then`: `retry`, `goto` or `stop`),
`paused`, `resumed`, and one final `plan_succeeded`, `plan_failed` or `plan_aborted` (with `reason`).

While a plan runs or is paused, the planner must call `wait_for_event` at least every `lease_s` (60 s). If it does not
(the agent ended or lost its connection), the executive stops the current step, sends the arms to
`retreat_positions` and aborts the plan. A paused plan is given up the same way after `max_hold_s` (120 s). The
safety supervisor's `/e_stop` aborts a plan at once.

## Tests

```bash
python3 -m pytest src/xlerobot_executive/test/ -v
```

`test_plan.py` checks validation; `test_executive_mock.py` runs plans on mock hardware with the skill server and a
scripted perception service (success, retries, goto, pause and resume, abort, one plan at a time, the safety stop, the
lease), about three minutes.

On the robot, with the full stack (`object_perception`, `skills` and `executive` on), the presence guard, and a cube of
about 28 mm on the table within reach of the right arm:

```bash
XLEROBOT_ROBOT=1 python3 -m pytest src/xlerobot_executive/test/test_cube_robot.py -v
```

`test_cube_robot.py` finds the cube, approaches and touches it three times through plans (contact sensed, the same
height each time, the cube not moved by the touch, checked by detecting it again), aligns the claw tip over it with the
wrist camera (within 3 mm by that camera), and touches a made-up object in the air, which must end `NOT_FOUND`. About three minutes; `XLEROBOT_ARM=left` uses the left arm, `XLEROBOT_CUBE="x,y,z"`
picks one of several cubes.
