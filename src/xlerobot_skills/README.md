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
| `/skills/touch` | From above an object, moves the claw tip down over its centre until contact, then backs up (see below). |

Motions need the operator presence heartbeat (`/mcp_presence_ok`). Losing it, or the safety supervisor's `/e_stop`,
stops a motion with `SAFETY_STOP`: the arm holds where it is with torque on. Two motions on one servo bus at once are
refused. Requires the stack; `move_arm`, `approach` and `touch` need move_group, `detect` and `approach` the object
perception service, `navigate_to` Nav2 with localisation. Tool points, the table margin and the touch thresholds are in
`config/skills.yaml`.

## Touch

`touch` first settles the claw tip over the object's centre at the starting height (approach leaves it up to 15 mm off,
enough to land on a cube's edge), then steps down 3 mm at a time on the measured tip, correcting for the arm's sag and
keeping it over the centre. Contact is a stall: from 15 mm above the expected top, the tip falls 3 mm behind the
commanded steps over three steps and moves less than 1.7 mm in each of the last two (the servos stick and slip on
small steps in free air, so shortfalls and catch-ups are summed with their sign); or a load jump on the arm's joints, a guard for hard hits. Results:

- `SUCCEEDED` with the contact height relative to the expected top and the distance from the object's centre;
- `COLLISION` for contact more than 15 mm above the expected top (something is in the way);
- `LOST_OBJECT` for contact more than 18 mm below it (the claw slid off the side and pushed the object);
- `NOT_FOUND` with no contact down to 25 mm below the expected top.

On the reference robot, touches on a 28 mm cube report contact 8-15 mm below the top that perception expects (11 mm on
average) and leave the cube where it was: the right arm reaches about a centimetre lower than the head camera places
things (issue #16), and the contact height is the one to trust. The claw lands within
about 8 mm of the centre, the head-to-arm calibration error.

## Tests

```bash
source install/setup.bash && source tools/ros_env.sh
python3 -m pytest src/xlerobot_skills/test/ -v
```

`test_skills_mock.py` brings up the robot on ros2_control mock hardware, move_group, the skill server, the presence
guard and a scripted perception service (`test/fake_perception.py`) on a ROS domain of its own, then checks every
skill's result codes, cancel, the safety stop and the one-motion-per-bus rule (about two minutes, no robot needed;
`XLEROBOT_SKIP_MOCK=1` skips it). `test_geometry.py` covers the aiming and frame maths. The on-robot test of
detect, approach and touch is `xlerobot_executive`'s `test_cube_robot.py`.
