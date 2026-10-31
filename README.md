# ros2_xlerobot_2wheels

ROS 2 Jazzy stack for [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) **0.4.0**, the servo two-wheel
differential drive version, on a Jetson Orin (or any PC with an NVIDIA GPU).

It runs the XLeRobot hardware (CAD, motor conventions, kinematics) under `ros2_control` instead of LeRobot's Python
control path, so Nav2, MoveIt 2 and Isaac ROS run on it natively. Upstream stays the hardware project and the
imitation-learning path; the two don't depend on each other.

## Status

Running on the real robot:

- [x] Base: drive, odometry, SLAM and AMCL on a saved map, Nav2 point-to-point navigation
- [x] Both SO-101 arms and the head: model, servo and camera calibration, `ros2_control`, MoveIt 2 planning and
  execution with the head depth camera feeding an octomap
- [x] Head camera: Orbbec Gemini 2 colour, depth, point cloud and IMU (`sensor:=gemini`), or a plain webcam (the default),
  see [ADR-0003](docs/decisions/0003-perception-sensor.md); colour cameras on both wrists
- [x] Object poses on demand: open-vocabulary detector, segmentation and a pose fit on TensorRT, about 2 mm on a cube,
  see [ADR-0005](docs/decisions/0005-object-perception.md)
- [x] Teleop: joystick base, dual-stick arms, safety supervisor watching servo temperature and current
- [x] Hand-guided record and replay, also of both arms together
- [x] Console UI to start, stop and check the stack (`tools/xlerobot_tui.py`)
- [x] MCP server for agent control, and a Rerun bridge for timeline debugging

Not yet:

- [ ] Grasping: the wrist-camera closed loop for the last centimetre
- [ ] Task execution: a planner writing behaviour trees, an executive on the robot and a visual watcher, see
  [ADR-0006](docs/decisions/0006-plan-execute-watch.md) (proposed)
- [ ] Tag-based AMCL re-localization (ArUco home marker): built, not tested end to end
- [ ] cuVSLAM / nvblox
- [ ] CI
- [ ] [Open issues](https://github.com/tfoldi/ros2_xlerobot_2wheels/issues), notably the servo torque-disable race on
  shutdown (#4)

The design is in [ARCHITECTURE.md](ARCHITECTURE.md) and [docs/decisions/](docs/decisions/).

## Hardware

| | |
|---|---|
| Robot | XLeRobot 0.4.0, two-wheel servo base, IKEA RÅSKOG chassis |
| Actuators | 16 Feetech STS3215 on two half-duplex TTL buses |
| Compute | Jetson Orin NX, JetPack 7.2. Any x86-64 machine with an NVIDIA GPU works too, for building, Isaac Sim, or running without a robot. |
| Sensors | LDROBOT LD19 lidar; Orbbec Gemini 2 head camera (or a USB webcam); USB colour cameras on the wrists |
| ROS | Jazzy only, as Isaac ROS 4 supports only Jazzy |

## Layout

The repo is a colcon workspace; clone and build in place.

```
src/xlerobot_description/         xacro, meshes, measured dimensions and calibration results
src/xlerobot_bringup/             controller configs and the top-level launch files
src/xlerobot_moveit_config/       MoveIt 2 for both arms and the head
src/xlerobot_teleop/              joystick, twist mux, safety supervisor
src/xlerobot_navigation/          slam_toolbox, Nav2, robot_localization
src/xlerobot_perception/          camera and lidar bringup
src/xlerobot_object_perception/   on-demand object pose service
sim/                              Isaac Sim twin
tools/                            TUI, calibration, diagnostics, MCP server, see docs/tools.md
docs/setup.md                     setting up and calibrating a robot, step by step
docs/jetson.md                    preparing the Jetson for the full stack (DDS, network, USB, power)
docs/arms-bringup.md              bringing up the arms safely
docs/sim-twin.md                  the Isaac Sim twin
docs/decisions/                   architecture decision records
```

## Getting started

```bash
git clone https://github.com/tfoldi/ros2_xlerobot_2wheels ~/Developer/ros2_xlerobot_2wheels
cd ~/Developer/ros2_xlerobot_2wheels
vcs import src < xlerobot.repos                 # community dependencies (sts_hardware_interface, ...)
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The tools in `tools/` need a Python venv on top of this ([docs/tools.md](docs/tools.md#python-environment)). A new
robot then needs its devices named and its servos, dimensions and cameras calibrated: [docs/setup.md](docs/setup.md).
The robot's computer needs a few system settings first: [docs/jetson.md](docs/jetson.md).

Without hardware:

```bash
ros2 launch xlerobot_description display.launch.py                                   # the model
ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true use_mock_hardware:=true torque_off:=false
ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true use_sim:=true         # Isaac Sim, docs/sim-twin.md
ros2 launch xlerobot_moveit_config move_group.launch.py rviz:=true
```

On the robot, `python3 tools/xlerobot_tui.py` starts the whole stack and shows what is running. By hand:

```bash
ros2 launch xlerobot_bringup bringup.launch.py cameras:=true lidar:=true sensor:=gemini   # base and cameras
ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true with_head:=true             # plus arms and head
```

Read [docs/arms-bringup.md](docs/arms-bringup.md) before the arms' first start.

## Before you plug anything in

**Upstream's wheel radius is wrong; measure yours.** `config_xlerobot_2wheels.py` says `wheel_radius = 0.05`, the
0.4.0 build notes say 5 inch wheels, and this robot's measured 0.0625 m. It goes in
`src/xlerobot_description/config/robot_dimensions.yaml`, and odometry and Nav2's limits scale from it.

**Neither upstream URDF has wheels.** Both put the base on a virtual planar joint, a simulation trick.
`base_2wheels.xacro` here has real wheel joints, which `diff_drive_controller` needs.

**One wheel motor is mounted mirrored; check which on your build.** Here it is the right one. The fix is a single
negative `right_wheel_radius_multiplier` in `src/xlerobot_bringup/config/controllers.yaml`, which flips the command and
the odometry together. Apply it there only, not also in the URDF or the hardware interface.

## Bus layout

| Bus | IDs | Joints | Mode |
|-----|-----|--------|------|
| 1 | 1-6 | left arm | position |
| 1 | 7-8 | head pan, head tilt | position |
| 2 | 1-6 | right arm | position |
| 2 | 9-10 | left wheel, right wheel | **velocity** |

Bus 2 runs position and velocity servos on one UART, so wheel traffic competes with arm traffic;
`sts_hardware_interface` supports per-joint operating modes, which is why it was picked. The two USB adapters swap
names on reboot; `tools/xlerobot_setup_usb_modules.py` pins them (device detection ported from
[RoboCrew](https://github.com/Grigorij-Dudnik/RoboCrew), MIT). `tools/calibrate_arm.py status --arm left|right|head`
checks which cable is which and whether every servo answers (with the stack stopped).

## Relationship to upstream

| Taken from XLeRobot | How |
|---|---|
| Meshes and CAD | the Isaac Sim URDF bundle and STEP files |
| Motor IDs, bus split, mirrored wheel | `xlerobot_2wheels.py`, restated in [ARCHITECTURE §1](ARCHITECTURE.md#1-hardware) |
| Nominal dimensions | starting points only, replaced by measurements |

Not taken: the ZMQ host/client control path, the `lerobot` runtime dependency, the virtual planar joint URDF.

## Community dependencies

Full list with licenses in [ARCHITECTURE §3](ARCHITECTURE.md#3-what-comes-from-the-community). The main one is
[`adityakamath/sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface) (Apache-2.0), the
`ros2_control` interface for Feetech STS servos with per-joint operating modes and the temperature, current and
voltage readings the safety supervisor uses. `xlerobot.repos` pulls it from
[`tfoldi/sts_hardware_interface`](https://github.com/tfoldi/sts_hardware_interface) `main`, which carries the fixes made
here (each also sent upstream as a PR).

[`robotmindio/cleanroom`](https://github.com/robotmindio/cleanroom), a Jazzy stack for the related LeKiwi, is worth a
read for its Nav2 tuning and safety patterns.

## License

Apache-2.0
