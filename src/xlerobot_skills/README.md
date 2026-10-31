# xlerobot_skills

The robot's skills as ROS 2 actions with one result contract ([ADR-0006](../../docs/decisions/0006-plan-execute-watch.md)).
Every action returns `xlerobot_skill_msgs/SkillResult` (`SUCCEEDED`, `TIMEOUT`, `NOT_REACHABLE`, `NOT_FOUND`,
`AMBIGUOUS`, `LOST_OBJECT`, `COLLISION`, `SAFETY_STOP`, `PREEMPTED`, `FAILED`) and reports its phase in the feedback.

```bash
ros2 launch xlerobot_skills skills.launch.py
ros2 action send_goal --feedback /skills/look_at xlerobot_skill_msgs/action/LookAt "{named_position: head_look_down}"
```

| Action | What it does |
|---|---|
| `/skills/navigate_to` | Drives to a saved waypoint or a pose with Nav2. |
| `/skills/look_at` | Moves the head to a saved position, or aims the head camera at a point (closed on TF). |
| `/skills/detect` | Finds objects for a text prompt with the object perception service; poses in `map` when localised. |
| `/skills/move_arm` | Moves an arm to a saved position, puts its gripper link at a point, or (`tool_point`) its claw tip at a point with the wrist camera on top. |
| `/skills/approach` | Re-detects an object near a reference pose (`NOT_FOUND`, `AMBIGUOUS` instead of guessing) and puts the claw tip above it. |
| `/skills/touch` | From above an object, moves the claw tip straight down until contact (a stall or a load jump), then backs up. |

Motions need the operator presence heartbeat (`/mcp_presence_ok`). Losing it, or the safety supervisor's `/e_stop`,
stops a motion with `SAFETY_STOP`: the arm holds where it is with torque on. Two motions on one servo bus at once are
refused. Requires the stack; `move_arm`, `approach` and `touch` need move_group, `detect` and `approach` the object
perception service, `navigate_to` Nav2 with localisation. Tool points, the table margin and the touch thresholds are in
`config/skills.yaml`.

## Tests

```bash
source install/setup.bash && source tools/ros_env.sh
python3 -m pytest src/xlerobot_skills/test/ -v
```

`test_skills_mock.py` brings up the robot on ros2_control mock hardware, move_group, the skill server, the presence
guard and a scripted perception service (`test/fake_perception.py`) on a ROS domain of its own, then checks every
skill's result codes, cancel, the safety stop and the one-motion-per-bus rule (about two minutes, no robot needed;
`XLEROBOT_SKIP_MOCK=1` skips it). `test_geometry.py` covers the aiming and frame maths.
