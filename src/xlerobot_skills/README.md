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
| `/skills/align` | From above an object, measures it with the wrist camera and moves the claw tip over its centre (see below). |

Motions need the operator presence heartbeat (`/mcp_presence_ok`). Losing it, or the safety supervisor's `/e_stop`,
stops a motion with `SAFETY_STOP`: the arm holds where it is with torque on. Two motions on one servo bus at once are
refused. Requires the stack; `move_arm`, `approach` and `touch` need move_group, `detect` and `approach` the object
perception service, `navigate_to` Nav2 with localisation. Tool points, the table margin and the touch thresholds are in
`config/skills.yaml`.

## Align

After `approach` the claw tip is above the object to within the head camera's and the arm's errors, about a centimetre.
`align` opens the gripper (the moving jaw would hide the object) and uses the wrist camera
(`/perception/locate_in_wrist`) in two stages:

1. From beside the object: over it the claw hides most of it, so the tip is shifted `align_view_offset_m` (2 cm) toward
   the image's left, where the object shows whole, and the object's centre, height and yaw are measured there. The tip
   moves over that centre at the same height and hand orientation, the encoders keeping it there, and the object is
   measured again from beside it, until within `align_tolerance_m` (2 mm).
2. Over the object (`align_check_over`): the hand turns a little between the two poses (servo play the encoders do not
   see), so the object is measured once more with the tip where it ends up. The claw hides part of it, so only x and y
   are fitted, with the height and yaw from stage 1; small moves correct what is left.

With `height_m` the tip is first lowered to that gap above the object and works there. The result is the remaining
offset by the wrist camera and the measured object pose; a plan's executive binds that pose to the object, so a
following `touch` centres on it.

On the reference robot (right arm, cube about 25 cm ahead): 0.6-1.2 mm by the wrist camera; a camera above the table
puts the real tip 2.0-2.1 mm from the centre over three runs. That needs the claw tip right in the camera's frame:
`tip_offset_right` was measured this way (align with `height_m` about 12 mm, then the real tip from a camera above,
corrected for the parallax of its height above the cube). Measure yours the same way.

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

After `approach` alone the claw lands within about a centimetre of the centre (the head-to-arm calibration error) and
the contact can be a centimetre off the height the head camera expects (issue #16): the contact height is the one to
trust. After `align`, `touch` centres on the wrist camera's measurement and its contact comes within a few millimetres
of the expected top.

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
