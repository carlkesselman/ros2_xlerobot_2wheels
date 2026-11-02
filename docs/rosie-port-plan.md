# Porting Carl's cart to this stack (branch `rosie`)

Draft plan, 2026-11-01. Sources: `../XLeRobot` branch `rosie-config` (`config/README.md`, `RUNNING.md`,
`ORIN-SETUP.md`, `cart-remote.yaml`, the udev rules, `software/src/robots/xlerobot_2wheels_odrive/`,
`hardware/lidar_bracket/README.md`) and `../lerobot` branch `feature/xlerobot` (`FORK.md`). Nothing here is
built or tested yet.

## 1. How the two robots differ

The full inventory, wiring, power and measurement list for Carl's cart is [platform.md](platform.md).

| | this repo (reference robot) | Carl's cart (`rosie-config`) |
|---|---|---|
| Cart computer | Jetson Orin NX, JetPack 7.2 | **elroy**, Jetson Orin Nano, JetPack 7, `wlP1p1s0` wifi |
| Operator station | gamepad on the robot | **rosie**, Raspberry Pi 5: leader arms + gamepad (8BitDo SF30 Pro), ZMQ to the cart |
| Base | 2 × STS3215 wheels, ids 9/10 on bus 2 | **2 × brushless hub wheels on an ODESC** (ODrive v3, fw 0.5.1), USB/libusb `1209:0d32`; axis0 = right, axis1 = left; `invert_left_wheel: true` |
| Bus 2 | right arm 1-6 + wheels 9-10 | **right arm 1-6 only** |
| Wheel radius / track | 0.0625 / 0.55 m | **0.0824 / 0.527 m** (measured 2026-10-21, radius from the 51.8 cm circumference 2026-11-01) |
| Mass | 8 kg (TODO) | **17.2 kg**, complete |
| Speed / accel | 0.25 m/s, 0.3 m/s² | **1.0 m/s, 1.0 m/s²** (validated on rug and hardwood, 2026-10-25) |
| Base tuning | none (servo) | ODESC gains 5.2 / 16.3, encoder bandwidth 50, current_lim 10.5 A (saved on the board); feed-forward friction 0.36 Nm, inertia 0.419 Nm per turn/s²; board watchdog 0.5 s; VEL_RAMP on the board |
| Head camera | Orbbec Gemini 2 (`sensor:=gemini`) | **Intel RealSense D415**, serial `836112060112`, 640×480×30, own USB root port. **No IMU**; rolling shutter |
| Wrist cameras | USB | Innomaker U20CAM, MJPG 640×480×30, `buffer_size 2`, positional udev names `/dev/cam_left`, `/dev/cam_right` |
| Lidar | LD19 under the second basket, scan plane 0.387 m, yaw +90° | **LD19 low on the cart frame**, centreline over the drive axle, in the gap between the bottom and middle baskets, scan plane ~0.30-0.35 m (to measure); the four corner posts block thin slices (§2.4) |
| Serial buses | `/dev/xle-bus1`, `/dev/xle-bus2` | by-id `…5A7A058116` (left + head), `…5B91028673` (right) |
| Arm calibration | ROS repo yaml + lerobot-format JSON | lerobot JSONs on elroy (`so_follower/xlerobot_arms_{left,right}.json`); `head_goal_velocity 600`, `head_acceleration 40`, `arm_acceleration 100` |

Same as the reference robot: the SO-101 arms, the head pan/tilt on ids 7/8 of bus 1, the LD19 driver, Jazzy.

## 2. What to change

### 2.1 Base: an ODrive `ros2_control` hardware interface (the main piece of work)

`diff_drive_controller` stays. The wheels move off `sts_hardware_interface` onto a third `<ros2_control>`
component, `xlerobot_base`, with two velocity-command joints.

Driver: **port [Factor-Robotics/odrive_ros2_control](https://github.com/Factor-Robotics/odrive_ros2_control)
to Jazzy** (C++, native USB protocol, supports fw 0.5.1, which is what the ODESC runs; officially only Foxy/Humble
and probably unmaintained). Pull it with `xlerobot.repos` from a fork, like `sts_hardware_interface`. Then add
what `odrive_base.py` learned on this cart:

- velocity control with `INPUT_MODE_VEL_RAMP`, ramp rate = `max_accel / wheel_circumference`
- the board watchdog (0.5 s), turned on at the first command and fed from `write()`
- feed-forward torque (`friction_ff_nm`, `inertia_ff`), which needs passthrough mode plus our own ramp
- error words read and logged before clearing; refuse to arm if the brake resistor is disarmed
- position state from `encoder.pos_estimate` (turns → rad), so odometry differences positions; that avoids
  the STS wrap problem altogether (§4.1 doesn't apply to this base)

The fallback if the port goes badly: a small Python node built from `odrive_base.py`, publishing `/odom` itself.
Rejected for now because it bypasses `diff_drive_controller` and `ros2_control`, which ADR-0001 is built on.

Wheel signs: per ARCHITECTURE §1.2, apply the inversion in exactly one place. Here that is
`left_wheel_radius_multiplier: -1.0` (matching `invert_left_wheel: true`), not also in the driver.

### 2.2 Description and dimensions

- `robot_dimensions.yaml`: wheel radius, track, mass, limits from §1. Re-measure axle x, caster positions, body
  size and the head/arm mount heights on this cart; the reference robot's measured values don't carry over.
- `base_2wheels.xacro`: 165 mm hub wheels.
- `bus2`: remove the wheel joints; right arm only.
- Arm and head calibration: run `tools/calibrate_arm.py` on elroy. It writes the same lerobot JSONs the cart
  already has, so the existing files can seed it, but `zero_step`, `model_offset`, `mount_correction`,
  `frame_correction` and `wrist_camera` are per robot and must be re-measured
  (`tools/calibrate_arm_camera.py`, `tools/arm_accuracy.py`).
- Carry over the cart's servo profile: arm Acceleration 100 (limits inrush on the first command; both buses once
  went silent from that), head Goal_Velocity 600 / Acceleration 40.

### 2.3 Head camera: RealSense

- Make `sensor:=realsense` real, the way `gemini` is: `realsense2_camera` (`ros-jazzy-realsense2-camera`),
  `serial_no: '836112060112'`, 640×480×30, aligned depth, point cloud. The D415 has no IMU, so
  `enable_gyro`/`enable_accel` go **off** (the current `realsense.yaml` turns them on).
- D415 traits that matter: about 65°×40° depth field of view (narrower than the D435's 87°), good depth from about
  0.3 m, a rolling shutter (smears while the head or the base moves; fine for the octomap and object poses taken
  standing still, weaker for cuVSLAM).
- URDF: `camera_link` on the head tilt link. Upstream ships gimbal holders for the D435 and D455 only, and the
  D415 body (99×23×20 mm) is a different size, so measure the real mount rather than taking upstream's numbers.
- Point MoveIt's octomap and the object perception at the RealSense topics.
- udev: librealsense `99-realsense-libusb.rules`; keep the camera on its own USB root port.

### 2.4 LD19, low on the cart frame

Decided 2026-11-01: the LD19 moves off the head spacer (which scanned at about 1.1 m, missed tables and chair
seats, was crossed by a raised arm and lost the rear ~90° to the gimbal base) to a low mount, the same placement as
the reference robot.

- Mount: on the cart's centreline, as close above the drive axle as the frame allows, in the open gap between the
  bottom basket's rim and the underside of the middle basket. Scan plane about 0.30-0.35 m (reference robot:
  0.387 m, hung upside down from the middle basket). Nothing in that gap (battery, ODESC, cabling) may cross the
  scan plane. **To measure on the cart:** the gap height, the scan plane height, x/y relative to the drive axle,
  and the yaw of the LD19's forward arrow.
- `robot_dimensions.yaml` lidar block: those measurements (`x`, `y`, `mount_z`, `scan_below_mount`, `yaw`, plus
  whether it's upside down, which flips roll by π). `lidar.xacro` keeps `ldlidar_link` on `base_link`, as now.
- Corner posts: done with a laser_filters box filter (`config/laser_filters.yaml`, started by `lidar.launch.py`),
  `/ldlidar_node/scan` → `/scan`, which Nav2, slam_toolbox and amcl now read. The driver's angle crop is one sector
  and would have blinded the whole rear. `range_min: 0.30` stays.
- `/dev/ldlidar` udev name (`tools/xlerobot_setup_usb_modules.py`).
- The D415 depth still goes into the local costmap for what's above the scan plane (tabletops, counter edges,
  shelves), as a second obstacle source.
- The head spacer (`XLeRobot/hardware/lidar_bracket/`) is no longer used; the head goes back to M3x12 screws and
  sits 6 mm lower, so remeasure the head height before calibrating the camera mount.

### 2.5 Wrist cameras, buses, udev

- Port `99-cameras-elroy.rules` (`/dev/cam_left`, `/dev/cam_right`) and `99-odrive-elroy.rules`.
- `cameras.yaml`: MJPG 640×480×30 for both wrists.
- Bus ports: either keep `/dev/xle-bus1/2` from the setup tool, or point the launch defaults at the by-id
  paths.
- Also carry over the ModemManager ignore rule for `1a86` and the `dialout` group.

### 2.6 Nav2, EKF, controllers

- `controllers.yaml`: new radius/separation, `linear.x.max_velocity` 1.0 (Nav2 lower to start), acceleration
  1.0, `cmd_vel_timeout` 0.25 s (inside the board's 0.5 s watchdog).
- Nav2 and the velocity smoother: retune for a 17 kg cart at up to 1 m/s and a 0.527 m track.
- EKF: the D415 has no IMU. To begin with, wheel odometry alone with slam_toolbox correcting `map→odom`; the
  encoder odometry on the hub motors should be far better than on the servo base. If yaw drift during turns in
  place turns out to matter, add a separate IMU on the base (e.g. a BNO085) and fuse it in `ekf_node`.

### 2.7 Machines and teleop

- elroy runs everything robot-side. JetPack 7 + Jazzy as in `docs/jetson.md`; also bring over the wifi
  power-saving fixes (`iw`, `nmcli`, the `rtl8822ce` modprobe file) from `ORIN-SETUP.md`.
- rosie (the Pi) as the operator station, see open question 2.
- `tools/cyclonedds.xml`: the peers are elroy and rosie (and the Mac for RViz).

## 3. Order of work

1. Done 2026-11-01: description + dimensions for this cart, ODrive component with **mock hardware**, RViz. (Runs on the Mac,
   in the pixi Jazzy environment.)
2. ODrive driver port; on elroy, drive from the gamepad; check `/odom` against a tape (`calibrate_wheel_odometry.py`).
3. LD19 on the low mount + slam_toolbox map.
4. RealSense head camera, then the octomap and the depth costmap layer.
5. Arms and head: calibration, MoveIt.
6. Nav2 tuning, then the remote operator station.

## 4. Open questions

1. ~~Which RealSense model?~~ D415 (2026-11-01): no IMU, see §2.3 and §2.6.
2. The leader arms on rosie: keep teleop with leader arms in ROS 2 (a leader-arm node on the Pi publishing
   joint commands over DDS)? Or keep lerobot for leader-arm teleop and recording, with ROS 2 doing navigation
   and MoveIt? The two can't drive the same buses at once.
3. ODrive driver: port Factor-Robotics (recommended), or write our own?
4. ~~Lidar height?~~ Low mount, about 0.30-0.35 m, as on the reference robot (2026-11-01, §2.4).
