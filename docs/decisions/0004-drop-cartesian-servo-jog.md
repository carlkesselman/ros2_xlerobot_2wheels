# ADR-0004: Drop MoveIt Servo's Cartesian (TWIST) jogging, keep JOINT_JOG only

- Status: Accepted
- Date: 2026-10-03

## Context

`arm_servo_teleop` drove each SO-101 arm (5 non-gripper DOF: shoulder_pan, shoulder_lift,
elbow_flex, wrist_flex, wrist_roll) through `moveit_servo`'s `servo_node` in two modes:
`JOINT_JOG` (each stick axis maps straight to one joint's velocity) and `TWIST` (each stick
axis maps to Cartesian linear velocity, IK solved by `pick_ik`, position-only).

On the real arms `TWIST` refused to move, reporting `DECELERATE_FOR_SINGULARITY` and `HALT_FOR_SINGULARITY` for
ordinary stick input, while `JOINT_JOG` worked throughout.

Servo's singularity gate compares the condition number of the arm's Jacobian against two thresholds. Raising them by
three orders of magnitude did not help: with `elbow_flex` at about -3.14 rad the arm is fully extended, a real rank
loss, not a scaling artefact of its short links. That pose is the arms' saved `rest` position, so Cartesian jogging
from rest, the obvious way to use it, starts at the singularity. Similar reports on other small arms are open upstream
(moveit2 #3411, moveit #3350).

Isaac ROS cuMotion is a motion planner, not a jogging controller, and its optimisation-based IK can jump to a
discontinuous configuration at a rank-loss pose, which is worse for live teleop than Servo stopping.

## Decision

Drop `TWIST`/Cartesian jogging entirely. `arm_servo_teleop` now only ever drives `JOINT_JOG`
(`mode_button` and the trigger-driven Z axis are removed, along with the `TwistStamped`
publishers). `servo.yaml` and `servo.launch.py` keep the Cartesian-related fields and topic
(Servo expects them at startup regardless of command type) but they're unused.

A real fix would mean avoiding near-singular poses (a dedicated "ready" pose, or falling back to `JOINT_JOG` near
one) or damping the Jacobian ourselves: real complexity for a mode whose only advantage is a more intuitive stick
mapping.

## Consequences

- `arm_servo_teleop` is simpler: no mode state, no mode toggle, no per-arm TWIST publisher.
- Operators get one jogging mode, not two - direct joint control only, no Cartesian stick
  mapping.
- Cartesian input later (a SpaceMouse or a leader arm sending pose deltas through `move_group`'s plan and execute
  instead of Servo's continuous jogging) is not ruled out.
