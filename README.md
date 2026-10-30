# ros2_xlerobot_2wheels

ROS 2 Jazzy stack for [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) **0.4.0**, the
servo two-wheel differential drive version. Built for the NVIDIA Isaac stack on Jetson,
but runs on any PC with a decent NVIDIA GPU.

This repo reuses the XLeRobot hardware platform - CAD, motor conventions, kinematics - under
`ros2_control` instead of the LeRobot-integrated Python control path, so Nav2, MoveIt 2 and
Isaac ROS run on it natively. Upstream stays the hardware project and the imitation-learning
path; the two don't depend on each other.

---

## Status

Running on real hardware:

- [x] Base drive, odometry, SLAM/AMCL on a saved map, Nav2 point-to-point navigation
- [x] Both SO-101 arms and the head - URDF, calibration, `ros2_control` (torqued and
  commandable by default, `torque_off:=true` for states-only), MoveIt 2 planning and
  execution, checked against mock hardware, a fake servo bus, the Isaac Sim twin, and the
  real robot
- [x] Head camera: `sensor:=usbcam` for a plain webcam (the default), `sensor:=gemini` for
  an Orbbec Gemini 2's color/depth/point cloud/IMU, see
  [ADR-0003](docs/decisions/0003-perception-sensor.md)
- [x] Dual-stick arm teleop, joystick base teleop, safety supervisor watching servo
  temperature/current
- [x] Hand-guided trajectory record/replay, including synchronized multi-arm trajectories
- [x] MCP server for agent control - saved positions/waypoints, spatial memory, camera
  capture with pose context
- [x] Rerun bridge - robot model, odometry, camera video and depth, MCP call log

Not yet:

- [ ] Tag-based AMCL re-localization (ArUco home marker) - built, not tested end to end on
  hardware
- [ ] CI/CD - no automated build or test on this repo
- [ ] Real obstacle avoidance for MoveIt 2 - plans against self-collision only, nothing yet
  feeds the depth camera into an octomap
- [ ] cuVSLAM / nvblox adoption - unblocked by the Gemini 2 and the JetPack 7.2 reflash, not started
- [ ] [Open issues](https://github.com/tfoldi/ros2_xlerobot_2wheels/issues) - the servo
  torque-disable race on shutdown (#4) is the one to know about before leaving a bringup
  unattended

See [ARCHITECTURE.md](ARCHITECTURE.md) and [§ Phases](ARCHITECTURE.md#8-phases) for the
full picture, [docs/arms-bringup.md](docs/arms-bringup.md) for the arm procedure.

---

## What this is for

A standard ROS 2 autonomy stack on the XLeRobot two-wheel base:

- `ros2_control` hardware interface for both Feetech buses, 16 joints
- `diff_drive_controller`, SLAM and Nav2 for the base
- Both SO-101 arms and the head: calibration, model, controllers, MoveIt 2
- Isaac ROS cuVSLAM / nvblox / cuMotion where they earn their place
- MCP server for agent-based orchestration

## Target hardware

| | |
|---|---|
| Robot | XLeRobot 0.4.0, two-wheel servo base, IKEA RÅSKOG chassis |
| Actuators | 16x Feetech STS3215 on 2 half-duplex TTL buses |
| Compute | Jetson Orin NX (JetPack 7.2) on the robot. Any x86-64 machine with an NVIDIA GPU works too, for building, Isaac Sim, or running the stack without a robot attached. |
| ROS | Jazzy Jalisco |
| Sensors | LDROBOT LD19 lidar for mapping, plain USB webcam for the head camera (default) - an Orbbec Gemini 2 is a drop-in optional upgrade for depth/point cloud/IMU, see [ADR-0003](docs/decisions/0003-perception-sensor.md) |

Only Jazzy is supported, since Isaac ROS 4.0 only supports that.

---

## Layout

This repo is a colcon workspace, clone and build in place.

```
src/xlerobot_description/   xacro, meshes, measured dimensions, rviz config
src/xlerobot_bringup/       controller configs and top level launch
src/xlerobot_moveit_config/ MoveIt 2 for both arms: groups, named states, self-collision matrix
src/xlerobot_teleop/        joystick, twist mux, safety supervisor
src/xlerobot_navigation/    slam_toolbox, Nav2, robot_localization params
src/xlerobot_perception/    sensor agnostic camera and lidar bringup
sim/                        Isaac Sim twin: scene builder, headless/GUI launch
tools/                      calibration, bus diagnostics, e-stop, fake servo bus for dev
docs/tools.md               what every tool in tools/ does
docs/arms-bringup.md        the arm bring-up procedure, step by step
docs/sim-twin.md            the Isaac Sim twin, setup and known limitations
docs/decisions/             architecture decision records
xlerobot.repos              community deps, pulled with vcs
```

## Getting started

```bash
git clone <this-repo> ~/Developer/ros2_xlerobot_2wheels
cd ~/Developer/ros2_xlerobot_2wheels

# community deps (sts_hardware_interface, etc.)
vcs import src < xlerobot.repos

rosdep install --from-paths src --ignore-src -r -y

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The Python tools in `tools/` (the TUI, calibration, the MCP server) need a venv on top of this; see
[docs/tools.md](docs/tools.md#python-environment).

View the model with no hardware attached:

```bash
ros2 launch xlerobot_description display.launch.py
```

Bring up the base on real hardware (wheels only, arms and head off, nothing draws torque
outside the wheels):

```bash
ros2 launch xlerobot_bringup bringup.launch.py cameras:=true lidar:=true
```

Add the arms and head (state-only, torque off, safe to bring up and move by hand - see
[docs/arms-bringup.md](docs/arms-bringup.md) before ever setting `torque_off:=false`):

```bash
ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true with_head:=true
```

Add `sensor:=gemini` to either command for an Orbbec Gemini 2's color/depth/point cloud/IMU
instead of the default plain webcam - see [ADR-0003](docs/decisions/0003-perception-sensor.md).

Same stack against mock hardware or the Isaac Sim twin, no robot needed:

```bash
ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true use_mock_hardware:=true torque_off:=false
ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true use_sim:=true      # see docs/sim-twin.md
ros2 launch xlerobot_moveit_config move_group.launch.py rviz:=true               # MoveIt 2, either way
```

---

## Before you plug anything in

Three things bit during bring-up on this robot, worth checking before you trust anything
downstream:

**1. Upstream's wheel radius is wrong, measure yours.**
`config_xlerobot_2wheels.py` has `wheel_radius = 0.05`; the 0.4.0 build notes say 5 inch
walker wheels (r ≈ 0.0635), and this robot's own drive wheel measured 0.0625 m outer
diameter - neither number is a safe default. It goes in
`src/xlerobot_description/config/robot_dimensions.yaml`, and everything else (odometry,
Nav2's velocity limits) is scaled off it, so get it right before tuning anything else.
See [ARCHITECTURE §1.4](ARCHITECTURE.md#14-numbers-that-are-probably-wrong).

**2. Neither upstream URDF has wheels.**
Both the ManiSkill and Isaac Sim URDFs put the base on a virtual planar joint
(`root_x_axis_joint` / `root_y_axis_joint` / `root_z_rotation_joint`), a sim trick for a free
floating base, no wheel links, still the 0.3.0 three omni wheel geometry.
`base_2wheels.xacro` here is a real URDF with real wheel joints instead, which
`diff_drive_controller` needs.

**3. One wheel motor is mounted mirrored - check which one on your build, don't assume.**
On this robot it's the right motor: commanding it forward with a raw velocity controller
spun it backward. The fix is a single negative `right_wheel_radius_multiplier` in
`diff_drive_controller`'s config (`src/xlerobot_bringup/config/controllers.yaml`), which
flips both the outgoing command and the odometry reading consistently. Applied exactly
once, there; don't also negate it in the URDF axis or the hardware interface.

---

## Bus layout

Two buses, and the split crosses subsystems, that's the main constraint here.

| Bus | IDs | Joints | Mode |
|-----|-----|--------|------|
| 1 | 1-6 | left arm | position |
| 1 | 7-8 | head pan, head tilt | position |
| 2 | 1-6 | right arm | position |
| 2 | 9-10 | left wheel, right wheel | **velocity** |

So bus 2 runs position and velocity mode servos at the same time on one UART, and wheel
traffic competes with arm traffic. `sts_hardware_interface` supports per-joint operating
modes, which is why it was picked.

`/dev/ttyACM0` and `ttyACM1` swap on reboot. Pin them with:

```bash
sudo python3 tools/xlerobot_setup_usb_modules.py
```

Prompts to disconnect both buses, then connect each in turn, and writes
`/etc/udev/rules.d/99-xlerobot.rules` for `/dev/xle-bus1` and `/dev/xle-bus2`
itself, matched by USB physical port and serial rather than by hand. Also
scans for cameras and microphones, so the same tool covers a depth camera's
device name later (type `a` at the prompt to add one). Device detection and
udev rule generation ported from
[RoboCrew](https://github.com/Grigorij-Dudnik/RoboCrew)'s
`robocrew_setup_usb_modules` (MIT licensed).

To check which cable is which or whether a servo answers, `tools/calibrate_arm.py status
--arm left|right|head` reads every joint's position, temperature and voltage on that bus
(read-only, stack must be stopped first). See [docs/tools.md](docs/tools.md) for every tool
in `tools/`.

---

## Relationship to upstream

| Taken from XLeRobot | How |
|---|---|
| Meshes and CAD | From the Isaac Sim URDF bundle and STEP files |
| Motor IDs, bus split, mirrored wheel sign | Read from `xlerobot_2wheels.py`, restated in [ARCHITECTURE §1](ARCHITECTURE.md#1-hardware) |
| Nominal dimensions | Starting point only, treated as calibration targets |
| Head camera mount point | Kept, plain USB webcam by default - swappable for an Orbbec Gemini 2, see [ADR-0003](docs/decisions/0003-perception-sensor.md) |

Not taken: the ZMQ host/client control path, the `lerobot` runtime dependency, the virtual
planar joint URDF.

---

## Community dependencies

Full table with licenses and reasoning in
[ARCHITECTURE §3](ARCHITECTURE.md#3-what-comes-from-the-community). The one that matters most:

- [`adityakamath/sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface)
  (Apache-2.0), `ros2_control` interface for Feetech STS servos, CI tested on Jazzy, per-joint
  operating modes, and per-joint temperature/current/voltage that the safety supervisor needs.
  `xlerobot.repos` pulls it from [`tfoldi/sts_hardware_interface`](https://github.com/tfoldi/sts_hardware_interface)
  `main` instead of upstream directly - the canonical source for fixes made here, kept merged
  with upstream `main`, each fix also going back as its own PR.

Worth reading before starting: [`robotmindio/cleanroom`](https://github.com/robotmindio/cleanroom),
a Jazzy stack for the closely related LeKiwi platform. Uses the ZMQ bridge approach this repo
rejected, but the Nav2 tuning and safety patterns are directly useful.

## License

Apache-2.0
