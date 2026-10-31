# ADR-0001: ros2_control talks to the servo bus directly, lerobot is not in the control path

- Status: Accepted
- Date: 2026-09-20

## Context

Upstream XLeRobot controls the robot from Python via `lerobot`. Shipped topology is a ZMQ
host on the robot (`xlerobot_2wheels_host.py`) talking to a client, commands on port 5555,
observations on 5556.

Two ways to put ROS 2 on top:

**A. Bridge.** Keep the lerobot host, write a ROS 2 node that wraps its ZMQ sockets,
translate `/cmd_vel` into `x.vel`/`theta.vel` and observations into `/joint_states` and
`/odom`. This is what `robotmindio/cleanroom` does for the related LeKiwi platform.

**B. Native.** A `ros2_control` SystemInterface talks Feetech directly. lerobot is not in
the loop.

## Decision

B. `adityakamath/sts_hardware_interface`, one per serial bus, under a normal
`controller_manager`.

## Why

1. **Rate.** The lerobot host runs at `max_loop_freq_hz = 30`, and the source comment warns
   that going higher causes jitter. Nav2's controller server wants 20 Hz minimum, prefers 50.
   30 Hz shared with observation serialisation and camera capture is thin.

2. **Odometry.** The ZMQ observation only has `x.vel` and `theta.vel`, body velocities already
   derived from `Present_Velocity`. Integrating a velocity report to get pose is worse than
   differencing wheel position, and `Present_Velocity` on STS servos is noisiest at low
   speed, exactly where docking and goal approach happen. Native keeps access to raw
   per-joint position.

3. **Ecosystem.** `diff_drive_controller`, `joint_trajectory_controller`, MoveIt 2, cuMotion
   all assume `ros2_control`. Option A means reimplementing odometry integration, TF, command
   timeouts and trajectory interpolation by hand.

4. **Dependency direction.** With B, lerobot becomes an optional client of ROS topics instead
   of a required server underneath everything. Matches huggingface/lerobot#4368, a generic
   config driven plugin over standard `JointState` / `JointTrajectory` / `Twist` / `Odometry`
   instead of per-robot bridges.

5. **Hardware churn.** Upstream says 0.4.0 is the last Feetech/SO101 chassis version,
   brushless is next. Behind `ros2_control`, that swap replaces one plugin, not the stack.

## Consequences

- Upstream's teleop scripts, VR interface and dataset recording don't run against this stack unchanged; LeRobot comes
  back as a client of ROS topics, not as the layer underneath.
- The stack depends on a third-party hardware interface. It runs both buses under one `controller_manager`, with the
  arms in position mode and the wheels in velocity mode on the same bus; fixes made here go upstream as PRs.
- Gained: standard odometry and controllers, MoveIt and Isaac ROS compatibility, and per-joint temperature and current
  for the safety supervisor.

## Revisit if

A LeRobot policy has to drive the robot before a topic-level integration exists; a bridge for the base only would be a
legitimate stopgap.
