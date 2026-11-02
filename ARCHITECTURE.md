# Architecture

ROS 2 Jazzy stack for XLeRobot 0.4.0, the servo two-wheel version.

This repo does not fork [XLeRobot](https://github.com/Vector-Wangel/XLeRobot). It reuses it:
meshes and dimensions from the CAD, motor IDs and kinematics from the `lerobot` plugin.
Upstream stays the hardware and imitation learning project. This is the ROS 2 / Isaac
autonomy layer next to it.

---

## 1. Hardware

All of this was read out of the upstream sources, not assumed. Some of the numbers look
wrong and I say so where they do.

### 1.1 Motors and buses

16 joints, all Feetech STS3215, on two half-duplex TTL buses. The split is per cable run,
not per subsystem, and that is the main constraint on the whole design.

| Bus | Port (upstream default) | ID | Joint | Mode |
|-----|-------------------------|----|-------|------|
| 1 | `/dev/ttyACM0` | 1 | `left_arm_shoulder_pan` | position |
| 1 | | 2 | `left_arm_shoulder_lift` | position |
| 1 | | 3 | `left_arm_elbow_flex` | position |
| 1 | | 4 | `left_arm_wrist_flex` | position |
| 1 | | 5 | `left_arm_wrist_roll` | position |
| 1 | | 6 | `left_arm_gripper` | position |
| 1 | | 7 | `head_pan` (`head_motor_1`) | position |
| 1 | | 8 | `head_tilt` (`head_motor_2`) | position |
| 2 | `/dev/ttyACM1` | 1 | `right_arm_shoulder_pan` | position |
| 2 | | 2 | `right_arm_shoulder_lift` | position |
| 2 | | 3 | `right_arm_elbow_flex` | position |
| 2 | | 4 | `right_arm_wrist_flex` | position |
| 2 | | 5 | `right_arm_wrist_roll` | position |
| 2 | | 6 | `right_arm_gripper` | position |
| 2 | | **9** | `left_wheel` | **velocity** |
| 2 | | **10** | `right_wheel` | **velocity** |

From `XLeRobot/software/src/robots/xlerobot_2wheels/xlerobot_2wheels.py`.

Two things follow from this:

1. Bus 2 has to run mixed modes at the same time, six position servos and two velocity
   servos on one UART. Any driver that assumes one mode per bus is useless here.
2. The wheels share a bus with an arm. Same half-duplex line, so arm traffic and wheel
   traffic compete. Base latency is tied to arm latency. Better to budget the bus up front
   (§4.3) than to find it during a Nav2 run.

### 1.2 Wheel signs

Upstream's model:

```
v_left  = (v - ω·L/2) / r          left raw command is negated
v_right = (v + ω·L/2) / r
```

The negation is physical, not a frame choice: upstream's own robot has its left motor mounted
mirrored, and without correcting for it `x.vel` turns the robot and `theta.vel` drives it forward.

This build is not the same wheel: confirmed on the real robot by driving each wheel
individually through a raw `velocity_controllers/JointGroupVelocityController` (bypassing
`diff_drive_controller`) and watching which way it turned. Same command sign, opposite result
from upstream - **this robot's right motor is the mirrored one**, not the left. Don't assume
either side without testing; it's a mechanical fact of the specific build, not something you
can derive from the URDF or upstream's docs.

The sign goes in `diff_drive_controller`'s `right_wheel_radius_multiplier: -1.0`
(`controllers.yaml`), not the `ros2_control` joint config - `sts_hardware_interface` has no
per-joint invert param (checked its source), so this is the only layer that can apply it. A
negative radius flips both the outgoing command and the odometry reading for that wheel
consistently. The URDF axis stays honest either way; don't apply the correction twice.

### 1.3 Raw units

```
steps_per_deg = 4096 / 360
raw = round(deg_per_sec * steps_per_deg)      clamped to int16
```

Upstream clamps at `max_raw = 3000` ticks/s, so 264 °/s, 4.6 rad/s. With the wheel radius
that gives roughly **0.23 to 0.29 m/s** depending on which radius is right (§1.4). Nav2 and
the velocity smoother must stay under this.

### 1.4 Numbers that are probably wrong

`config_xlerobot_2wheels.py` ships:

```python
wheel_radius: float = 0.05   # m
wheelbase:    float = 0.25   # m
```

The first one looks like a placeholder and the 0.4.0 build notes contradict it. The dual
wheel assembly page says **5 inch universal walker wheels**, so 127 mm diameter, **r ≈ 0.0635**.
If 0.05 is used, every reported distance is off by about **27%**. SLAM, Nav2 goal tolerances
and anything learned from `x.vel` will be consistently wrong and it won't be obvious.

`wheelbase` is nominal too. Note the name: the code uses it as **track width**, the lateral
distance between the driven wheels, which is what `diff_drive_controller` calls
`wheel_separation`. Not wheelbase in the car sense.

So both are calibration results, not constants. They live in
`xlerobot_description/config/robot_dimensions.yaml` with a procedure in §6.

`wheel_radius` is now a direct measurement (0.0625 m, 125 mm wheel diameter) and passed a
1 m straight-line odometry check on the real robot (reported vs. tape measured, within
~1 cm) after the position-unwrap fix in §4.1/§7 R1 - no scale correction needed at that
distance. Still worth the full 5 m+ run from §6 for a tighter number; a 1 m check has more
relative measurement error to hide behind.

### 1.5 Chassis

IKEA RÅSKOG cart, two driven wheels on their own axle bar at the front and two passive
casters at the rear, in the stock mounting place but larger than stock to level the cart -
see `base_2wheels.xacro`. Neither caster has an encoder. In the URDF each is a fixed link with a low friction
sphere, not a steerable joint, because a joint would only publish invented numbers.

The thing is tall and top heavy, two 6-DOF arms and a head above a narrow track. CoM height
is a real limit on acceleration.

---

## 2. Why this shape

### 2.1 Stack

```
                      ┌─────────────────────────────────────────────┐
                      │  Nav2 (bt_navigator, planner, controller)    │
   autonomy           │  slam_toolbox  |  isaac_ros_visual_slam      │
                      │  nvblox (optional, costmap layer)            │
                      └───────────────┬─────────────────────────────┘
                                      │ /cmd_vel (TwistStamped)
                      ┌───────────────▼─────────────────────────────┐
                      │  ros2_control controller_manager             │
   control            │   diff_drive_controller  → /odom + odom→base │
                      │   joint_trajectory_controller ×2 (arms)      │
                      │   head position controller                   │
                      │   joint_state_broadcaster                    │
                      └───────┬──────────────────────┬──────────────┘
                              │                      │
                  ┌───────────▼──────────┐ ┌─────────▼────────────┐
   hardware       │ sts_hardware_interface│ │ sts_hardware_interface│
                  │   port=/dev/xle-bus1  │ │   port=/dev/xle-bus2  │
                  │   8 joints, position  │ │   6 pos + 2 velocity  │
                  └───────────┬──────────┘ └─────────┬────────────┘
                              │ Feetech TTL          │ Feetech TTL
                         ┌────▼─────┐           ┌────▼─────┐
                         │ IDs 1-8  │           │ IDs 1-6, │
                         │          │           │   9, 10  │
                         └──────────┘           └──────────┘
```

### 2.2 The main decision

`ros2_control` talks to the servo bus directly. `lerobot` is not in the control path.

The other option was wrapping upstream's `xlerobot_2wheels_host` ZMQ server in a ROS node,
like the LeKiwi `cleanroom` stack does. I went the other way, for these reasons:

1. **Rate.** The upstream host runs at `max_loop_freq_hz = 30` and the source warns that
   raising it causes jitter. Nav2's controller server wants 20 Hz minimum and is happier at
   50. 30 Hz shared with observation serialisation is not much to build on.
2. **Odometry.** The ZMQ path only gives `x.vel` and `theta.vel`, already integrated from
   `Present_Velocity`. Integrating a velocity report is worse than differencing wheel
   positions, and `Present_Velocity` is noisiest at low speed, which is where docking and
   goal approach happen. See §4.1.
3. **Ecosystem.** `diff_drive_controller`, `joint_trajectory_controller`, MoveIt 2 and
   cuMotion all assume `ros2_control`. Avoiding it means writing odometry, TF, command
   timeouts and trajectory interpolation by hand.
4. **Direction.** This way `lerobot` becomes an optional client of ROS topics instead of a
   server under everything. Policies and recording still work, they just talk to controllers.
   That also matches huggingface/lerobot#4368, which proposes a generic config driven plugin
   over standard `JointState` / `JointTrajectory` / `Twist` / `Odometry` instead of one
   bridge per robot.

[ADR-0001](docs/decisions/0001-ros2-control-over-lerobot-bridge.md).

The cost is real: upstream's teleop scripts, VR interface and recording examples don't work
against this without changes. Getting them back is phase 5.

### 2.3 Jazzy, and the JetPack gap on the actual board

Isaac ROS 4.0 dropped Humble and moved to Jazzy, current release is 4.6.0 (Aug 2026). Since
the point of the Jetson is cuVSLAM / nvblox / cuMotion, NVIDIA picks the distro, not us.
Jazzy on Ubuntu 24.04, both on the dev box and everything non-GPU on the Jetson.

The Orin NX was on JetPack 6.2 (L4T 36.4.3, Ubuntu 22.04 base, CUDA 12.6) when this gap was
first written up - current Isaac ROS needed JetPack 7.2, which had only just added Orin NX
support in June 2026. **Resolved by reflashing to JetPack 7.2** (L4T 39.2.0, Ubuntu 24.04
base, CUDA 13.2, TensorRT 10.16) - the board now matches the dev box's distro and CUDA
generation, so Isaac ROS 4.x packages are no longer blocked on the OS underneath them.
Phase 4 (cuVSLAM/nvblox/cuMotion adoption itself) is still a separate, not-yet-started
decision - this only removed the platform-level blocker. Full reasoning in
[ADR-0002](docs/decisions/0002-jazzy-and-isaac-ros.md).

---

## 3. What comes from the community

As of September 2026, licenses included.

| Need | Package | License | Verdict |
|------|---------|---------|---------|
| Feetech STS bus → `ros2_control` | [`adityakamath/sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface) | Apache-2.0 | **Use it.** The critical dependency. CI tested on Jazzy and Kilted. Per-joint `operating_mode` (0 position, 1 velocity, 2 PWM) set in the URDF, so bus 2's mixed mode case works natively. Exports position, velocity, effort, voltage, temperature, current and moving status per joint, so we get a real safety supervisor almost for free. Position and velocity modes are tested on hardware. |
| Diff drive + odometry | `diff_drive_controller` | Apache-2.0 | Use as is. |
| Arm trajectories | `joint_trajectory_controller` | Apache-2.0 | Use as is. |
| SO-101 description and MoveIt | [`adityakamath/so_arm_ros2`](https://github.com/adityakamath/so_arm_ros2) | Apache-2.0 | **Reference only.** Same author as the hardware interface so conventions match. Jazzy/Kilted, SO100 and SO101 with gripper. But it assumes one arm owns one bus, and here two arms share buses with a head and wheels. Take the joint limits, mesh conventions and MoveIt setup, rewrite the xacro. Author marks it work in progress with AI generated content, so read before copying. |
| Alt SO-ARM description | [`ros-physical-ai/ros2_so_arm`](https://github.com/ros-physical-ai/ros2_so_arm) | BSD-3 | Cross check for joint limits and MoveIt config. Uses `feetech_ros2_driver`. |
| Alt Feetech driver | [`JafarAbdi/feetech_ros2_driver`](https://github.com/JafarAbdi/feetech_ros2_driver) | check | Fallback if `sts_hardware_interface` fails the multi-bus test (§7, R2). |
| 2D SLAM | `slam_toolbox` | LGPL-2.1 | Use. Primary mapping until VSLAM is proven. |
| Navigation | `nav2` | Apache-2.0 | Use. |
| Visual SLAM | [`isaac_ros_visual_slam`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) | NVIDIA | Use if we get a suitable camera (§5). Stereo or RGB-D only, no monocular. Optional IMU fusion. |
| 3D recon → costmap | `isaac_ros_nvblox` | NVIDIA | Phase 4. Feeds Nav2 through a costmap plugin. |
| Lidar → scan | `pointcloud_to_laserscan` | BSD | If the VLP-16 is used. |
| VLP-16 driver | `velodyne` (ros2 branch) | BSD | If used. |
| Orbbec camera | [`OrbbecSDK_ROS2`](https://github.com/orbbec/OrbbecSDK_ROS2) | Apache-2.0 | Conditional, see §5. |
| Similar robot | [`robotmindio/cleanroom`](https://github.com/robotmindio/cleanroom) | Apache-2.0 | **Read it, don't import it.** Closest thing out there: Jazzy, Feetech driven LeKiwi, Nav2, RTAB-Map. Uses the ZMQ bridge we rejected so the driver layer doesn't transfer, but the Nav2 tuning, safety supervisor and RTAB-Map config are useful. Their finding that lidar ICP beat visual loop closure in feature poor corridors is worth knowing. |
| USB device naming | [`Grigorij-Dudnik/RoboCrew`](https://github.com/Grigorij-Dudnik/RoboCrew) | MIT | **Ported.** `tools/xlerobot_setup_usb_modules.py` is RoboCrew's `robocrew_setup_usb_modules` device-detection and udev rule generation, adapted for the two Feetech buses (`xle-bus1`/`xle-bus2`) instead of RoboCrew's own camera/mic/arm layout. |

### 3.1 What we have to write

Nothing out there covers these, and they are the actual work:

1. **A URDF with wheels.** Neither upstream URDF has any. Both
   `simulation/Maniskill/assets/xlerobot/xlerobot.urdf` and the Isaac Sim one hang
   `base_link` on a virtual planar joint (`root_x_axis_joint` prismatic →
   `root_y_axis_joint` prismatic → `root_z_rotation_joint` continuous). That is a free
   floating base sim trick. No wheel links, no caster, and it still has the 0.3.0 three
   omni wheel geometry. `diff_drive_controller` can't work with it and neither can a
   physics sim of the real drivetrain. Prerequisite for everything else.
   Also the inherited `base_link` inertial says `mass = 70.1 kg`, which is ManiSkill's
   loaded cart, not the robot. Don't carry that over.
2. **Two-bus mixed-mode `ros2_control` config**, plus whatever shim
   `sts_hardware_interface` turns out to need.
3. **Multi-turn wheel position accumulation**, see §4.1. Sharpest risk here.
4. **Calibration tooling**, given §1.4.
5. **Safety supervisor** on the per-joint temperature and current interfaces.
6. Bringup, Nav2 tuning and sensor integration for this machine.

---

## 4. Details that matter

### 4.1 Odometry and the wrap problem

`diff_drive_controller` wants wheel position in radians and differences it. STS3215 reports
`Present_Position` as 12 bit, 0 to 4095, and it wraps. In velocity mode the wheel turns
continuously, so it wraps about every 0.39 m at r = 0.0625 (measured, see §1.4).

Fixed in the `tfoldi/sts_hardware_interface` fork, which `xlerobot.repos` pulls into
`src/sts_hardware_interface` (upstream PR: adityakamath/sts_hardware_interface#21). Building
against plain upstream `main` brings the wrap back: every wheel revolution shows up as a
~-7 rad/s spike in odometry yaw rate. `read()` unwraps `MODE_VELOCITY` joints by accumulating the
wrap-safe delta between consecutive single-turn readings (if the raw angle jumps by more
than π between cycles, treat it as a wrap and fold it back into [-π, π] before adding it to
a running total) instead of reporting the raw 2π-periodic angle directly.
`MODE_SERVO`/`MODE_PWM` joints are untouched - they have real position limits inside one
revolution already, where the old fmod-then-clamp behavior is correct.

Confirmed on real hardware, not just from reading source: driving the base past the old
~0.39 m wrap boundary shows `/joint_states` position climbing smoothly with no discontinuity,
and a 1 m odometry-tracked drive (§1.4) matched a tape measurement.

Safety margin against aliasing: the unwrap assumes no more than half a revolution happens
between consecutive `read()` cycles. At `wheel_max_angular_velocity` (4.6 rad/s) and the
100 Hz control loop, one cycle covers ~0.046 rad - nowhere near the π that would make a real
revolution look like standing still.

Still open: a dropped/failed SyncRead is already safe (`read()` skips that motor for the
cycle and keeps its last known state, doesn't feed anything into the unwrap). What isn't
guarded is a transaction that succeeds (right byte count, no comm error) but carries a
corrupted payload from electrical noise - unlike the old raw-only behavior, which
self-healed back to the true single-turn value every cycle, a bad delta here permanently
offsets the running total, since it accumulates. A max-plausible-delta gate derived from
commanded velocity would catch that; not yet built.

Either way wheel odometry here is a prior, not truth:

- Two driven wheels and an unsensed caster on a tall cart. Caster scrub during in place
  rotation is unmodelled error, and that is exactly what Nav2 does to align to goals.
- No suspension. Grout lines and carpet edges cause slip.
- No IMU in the base as shipped.

So yaw will drift. The plan is fusion from the start: `ekf_node` (robot_localization) with
wheel odometry plus an IMU, and SLAM correcting `map→odom`. An IMU is close to mandatory for
a usable `odom` frame, either the one in a D435i/D455 or a separate part. Budget for it.

### 4.2 Frames

Plain REP-105:

```
map → odom → base_footprint → base_link → {left_wheel, right_wheel, caster,
                                            torso → {left_arm_*, right_arm_*,
                                                     head_pan → head_tilt → camera_*}}
```

`base_footprint` is the ground projection and what Nav2 plans for.
`odom→base_footprint` from `diff_drive_controller`, `map→odom` from slam_toolbox or cuVSLAM.
Exactly one of those two at a time, never both.

Camera optical frames follow REP-103. Upstream's URDF already gets the head camera frames
right, so keep those names.

### 4.3 Bus budget

Half duplex TTL at 1 Mbps. A sync read of N motors is one packet out, N back.
`sts_hardware_interface`'s `SYNC_READ_FEEDBACK_LEN` is 15 bytes (registers 56 through 70),
so for an 8-motor bus (bus 1 and bus 2 both carry 8):

```
SyncRead request + 8 responses   ~184 bytes  ~1.84 ms
SyncWrite                        ~56 bytes   ~0.56 ms
round trip                       ~2.4 ms     -> ~400 Hz on the wire alone
```

So the servo protocol itself supports well above the 100 Hz `controller_manager` is set to.
That number is not bus-limited: it is set by the host's real-time loop budget, not the wire.
`controller_manager` runs its update loop on a regular (non-RT) desktop or Jetson kernel by
default, and 200 Hz (a 5 ms budget) overruns under ordinary scheduling jitter, particularly
with Isaac Sim's own GPU/CPU load competing for the same cores. 100 Hz (10 ms budget) has
headroom against that regardless of which hardware interface is loaded underneath (real bus,
mock, or the Isaac Sim topic bridge). Raising it again needs an RT kernel or isolated cores,
not just a bus-budget argument.

Separately, the USB-serial adapter itself has its own ceiling on the real bus: FTDI/CH340/
CP2102 chips commonly default their Linux `latency_timer` to 16 ms per USB transaction, which
caps effective round-trip rate around 60 Hz regardless of what the servo protocol or the host
loop allow. Check `/sys/bus/usb-serial/devices/*/latency_timer` on the real hardware before
trusting either number.

With arms idle bus 2 only carries the two wheels. With an arm streaming a trajectory at the
same rate, arm and wheel traffic interleave on the sync read/write and the wheel update rate
drops, proportionally to how many joints share the bus, not by much else.

If measurement shows this matters: run the two buses in **separate** `controller_manager`
instances at different rates, base fast and arms slower, instead of one manager stuck at the
lowest common rate. That is why the hardware interfaces are two independent `<ros2_control>`
blocks from the start, it keeps the option open.

Still needs a real bench measurement. The wire-protocol math and the latency_timer caveat are
both reasoning from source and from how these USB-serial chips generally behave, not from a
scope on the actual bus. The `sts_hardware_interface` diagnostics node plus `controller_manager`
timing stats should answer it for real once there's hardware to run it on.

### 4.4 Safety

STS3215 is a hobby servo with no external thermal cutout, driving a top heavy cart with two
arms on it. The per-joint temperature and current interfaces are there, so a supervisor node
should:

- stop the base and hold arm position over a temperature threshold,
- detect stall (high current, zero velocity, non-zero command) and release torque,
- enforce a command watchdog, upstream uses 500 ms, `cmd_vel_timeout` should be tighter,
  around 250 ms,
- clamp commanded acceleration to what the CoM tolerates.

**Don't build any of this around `sts_hardware_interface`'s own `/emergency_stop` service.**
Upstream `main` opens one internal node per bus instance with the hardcoded name
`sts_hardware_interface_node`, and each advertises `/emergency_stop` on that same absolute
name. Two servers under one service name means a call only reaches one of them, not both, so
it looks like a global e-stop and isn't one. The fork adds an opt-in `instance_id` hardware
parameter that suffixes node and service names (upstream PR #22, not yet tested with two
buses on hardware); `xlerobot.ros2_control.xacro` sets it to each component's name, giving
`/emergency_stop_xlerobot_bus1` and `_bus2`. Each instance's e-stop still only stops its own
bus, so it is still not a global e-stop. With two buses that's a stop-half-the-robot bug, not a cosmetic one.

Use `controller_manager`'s own `switch_controller` to deactivate everything instead, or just
rely on `cmd_vel_timeout` plus zero-command on watchdog trip, both of which act uniformly
across every loaded hardware component. Worth an upstream issue on the node-naming collision
too, small fix (name the internal node from the hardware component's own `info_.name`, e.g.
"bus1"/"bus2").

---

## 5. Perception, still open

`xlerobot_perception` picks the sensor by launch argument, because the sensor question is
not settled.

**On hand:**

- **Orbbec Gemini 2**, USB `2bc5:0670`, serial AY6G65301J0, firmware 1.4.92. Originally
  misidentified here as the original (non-2) Gemini - corrected 2026-10-05, confirmed from
  the device's own firmware response, not inferred. Fully supported by the official
  `ros-jazzy-orbbec-camera` apt package, `gemini2.launch.py`, no patching needed. Live-tested:
  color/depth/IR/point cloud/IMU all streaming real data. `sensor:=gemini` is a supported,
  tested option, not the stack's own default (`sensor:=usbcam`, a plain webcam, is - see
  [ADR-0003](docs/decisions/0003-perception-sensor.md)).
- **Velodyne VLP-16.** Definitely works. `velodyne` → `pointcloud_to_laserscan` →
  `slam_toolbox` → Nav2 is the most predictable path and needs no NVIDIA stack. Costs
  ~830 g and ~8 W on a battery powered servo cart, plus bulk, plus no cuVSLAM.
- **RealSense D435i / D455**, not owned. Easiest path to the Isaac stack: tested with
  cuVSLAM, has the IMU, and upstream XLeRobot already ships head gimbal mounts for exactly
  these two (`Gimbal_Pitch_Holder_d435.stl`, `Gimbal_Pitch_Holder_d455.stl`), with a
  commented out `RealSenseCameraConfig` head camera already in the lerobot config.

With the Gemini 2 confirmed working as a supported option, cuVSLAM and nvblox are unblocked
(no longer gated on "once there is a camera for it") - still separate, not yet started,
adoption decisions. [ADR-0003](docs/decisions/0003-perception-sensor.md).

**Real obstacle avoidance for MoveIt 2 is also unblocked, and also not started.**
`xlerobot_moveit_config` has no `sensors_3d.yaml` and no occupancy map monitor - the arms
plan against self-collision geometry only, blind to anything actually in front of them.
This doesn't need Isaac/nvblox at all: stock MoveIt 2's own `PointCloudOctomapUpdater`
(off `depth/points`) or `DepthImageOctomapUpdater` (off `depth/image_raw`) can feed
`move_group`'s planning scene directly from the Gemini 2 today. Octomap resolution and
update rate need picking with the Jetson's CPU budget in mind, same spirit as the ~8Hz
throttle on the depth stream into `tools/rerun_bridge.py`.

---

## 6. Calibration

Three numbers decide whether anything downstream means anything. All of them are data.

| What | How | Where |
|------|-----|-------|
| `wheel_radius` | Drive a measured straight line, compare reported to tape measured, scale. Repeat on the real floor surface, longer runs first (§1.4 - a 1 m check has more relative measurement error to hide behind). | `controllers.yaml` |
| `wheel_separation_multiplier` | Rotate in place N full turns, compare reported yaw to actual, scale. Or UMBmark for a tighter number. | `controllers.yaml` |
| per-wheel scale | UMBmark square path, corrects unequal effective radii | `controllers.yaml` |

Do this **before** tuning Nav2, otherwise the Nav2 gains are just compensating a scale error.

`tools/calibrate_wheel_odometry.py` drives the motion itself and does the arithmetic - it
doesn't edit the YAML, so apply the number it prints by hand and relaunch bringup (these are
read once at controller activation, a live `ros2 param set` doesn't take effect):

```
python3 tools/calibrate_wheel_odometry.py straight-line --distance 1.0
python3 tools/calibrate_wheel_odometry.py rotate-in-place --turns 0.5
```

Run with bringup already up, a clear straight-line path or room to spin in place, and a tape
measure or protractor. It stops automatically once `/diff_drive_controller/odom` reports the
commanded target, then prompts for the actual measurement and prints the corrected value to
apply, using the current value from `controllers.yaml` as the baseline each time - so it's
safe to run again after applying a suggestion, to converge over a couple of iterations.

---

## 7. Risks

| # | Risk | Impact | What to do |
|---|------|--------|------------|
| R1 | ~~`sts_hardware_interface` doesn't unwrap multi-turn wheel position~~ Fixed locally, confirmed on hardware (§4.1) | No unwrap gate against corrupted-but-valid reads yet | Add a max-plausible-delta check, see §4.1 "Still open". |
| R2 | Upstream: both hardware instances share a hardcoded internal node name and collide on `/emergency_stop`. Fork: per-instance names, but each e-stop covers its own bus only | One instance's e-stop doesn't reach the other bus | Don't build the safety supervisor around this service; use `controller_manager`'s `switch_controller` instead, see §4.4. |
| R3 | Bus 2 contention, arms starve wheel updates | Odometry jitter while manipulating | Instrument from day one. Split controller managers by rate. |
| R4 | ~~`wheel_radius` really is 0.0635 not 0.05~~ Measured (0.0625) and spot-checked at 1 m (§1.4) | Only the full 5 m+ calibration run from §6 is still outstanding | Run the §6 procedure before trusting odometry past a few meters. |
| R5 | ~~Gemini unusable for cuVSLAM (§5)~~ Resolved 2026-10-05: it's a Gemini 2 (misidentified before), confirmed working, color/depth/IR/IMU all live | cuVSLAM/nvblox adoption itself is still a separate, not-yet-started decision | n/a - lidar + slam_toolbox remains the navigation baseline either way, Isaac is an upgrade, not a dependency. |
| R6 | Servo thermal/stall limits under long navigation | Shutdown mid mission, maybe damage | `safety_supervisor.py` already subscribes to `/dynamic_joint_states` and defines `TEMPERATURE_LIMIT_C`/`STALL_CURRENT_THRESHOLD`, but `on_dynamic_joint_states` doesn't check them yet - the threshold logic itself is an unwritten `# TODO`, not just untuned. Still open. |
| R7 | Top heavy cart tips under hard acceleration | Broken hardware | Conservative accel limits in `diff_drive_controller` and the velocity smoother from the start. |
| R8 | Upstream says 0.4.0 is the last Feetech / SO101 chassis version | Platform moves to brushless | Keep the hardware behind `ros2_control`, then a brushless base swaps one plugin instead of the stack. The ODrive variant already exists upstream. |
| R9 | ~~Orin NX is on JetPack 6.2, current Isaac ROS needs 7.2 (§2.3)~~ Resolved: reflashed to JetPack 7.2 (L4T 39.2.0), confirmed on the actual board | cuVSLAM/nvblox/cuMotion adoption itself is still a separate, not-yet-started decision | n/a - phase 4 can start whenever it's prioritized, no OS-level blocker left, see ADR-0002. |
| R10 | `diff_drive_controller` can stop driving the wheels while `controller_manager` still reports every controller `active` and logs no error anywhere. Confirmed not a hardware/power/protocol fault: writing the servo bus directly (same registers, same broadcast sync-write `sts_hardware_interface` itself uses) moves the wheels correctly while the ROS side stays silently stuck | Base ignores drive commands with nothing visibly wrong in `ros2 control list_controllers`, `/diagnostics`, or the hardware interface's own logs | A full bringup restart (fresh `controller_manager` process) clears it. Root cause not isolated yet; suspect bus 2 contention or a stuck internal state in a long-running process, see R3. If it recurs, instrument `write()`'s actual per-cycle `hw_cmd_velocity_` for the wheel joints rather than re-checking hardware. |

---

## 8. Phases

**Phase 1, description and a wheel that turns.** Real URDF with wheel joints, measured
dimensions, RViz. Two-bus `ros2_control` config. Settle R1 and R2. `diff_drive_controller`
publishing `/odom`. Joystick teleop. Safety supervisor.
Done when: it drives from a gamepad and `/odom` is within 5% over 10 m.

**Phase 2, mapping and navigation.** VLP-16 (or Gemini) bringup, EKF, `slam_toolbox`, Nav2
tuned for a 0.25 m track diff drive robot at ~0.25 m/s.
Done when: autonomous point to point in a mapped room.

**Phase 3, arms.** Both arms on `joint_trajectory_controller`, MoveIt 2, head pan/tilt,
self collision and arm vs base geometry.
Done when: MoveIt plans and executes on both arms. Steps and risks in
[docs/arms-bringup.md](docs/arms-bringup.md).

**Phase 4, Isaac.** cuVSLAM replacing or fusing with slam_toolbox, nvblox costmap layer,
cuMotion for the arms. First decision here is JetPack: reflash the Orin NX to 7.2 for Jazzy,
or run Isaac ROS 3.2 on Humble in a separate container, see ADR-0002.
Done when: phase 2 and 3 parity on the Jetson.

**Phase 5, lerobot comes back.** A ROS 2 node exposing this stack as a `lerobot` robot so
policies and dataset recording work over ROS topics. Follow huggingface/lerobot#4368
conventions instead of a custom bridge.

**Phase 6, the room in simulation (later).** A reconstruction of the mapped room for the
Isaac Sim twin ([sim-twin](docs/sim-twin.md)), so the simulated robot works in the room
the real one does. Rendered as an NVIDIA NuRec Gaussian splat scene in Omniverse.
- **Geometry from mapping.** Walls and obstacles come from the saved `slam_toolbox` map (the
  occupied cells, extruded from the lidar plane) and the floor is the ground plane. The map
  frame starts at the robot's start mark, so the scene lines up with the map with no
  extra alignment. Splats are not collidable, this simple geometry is what the physics and
  the simulated lidar see.
- **Appearance from camera views.** Splats are trained from the head and wrist camera video
  recorded during mapping runs (the H.264 streams in the rosbag), with each frame's pose
  from the SLAM `map` to camera TF and the metric scale from the lidar map.
- **Needs first:** calibrated cameras (intrinsics, lens distortion, the fixed transforms
  are in the URDF), timestamps that line up with TF, and enough camera coverage of the
  room, which the fixed head camera and the arm cameras give little of.
- **Open:** how well a splat trained on this low resolution USB video renders, and which
  parts of the NuRec pipeline run on the workstation GPU rather than the Orin.
Done when: the simulated robot localises and navigates in the reconstructed room with the
same map.

Phases 1 and 2 are the commitment. The rest depends on what phase 1 finds.

---

## 9. Layout

```
ros2_xlerobot_2wheels/          # the repo is a colcon workspace
├── src/
│   ├── xlerobot_description/   # xacro, meshes, dimensions, rviz
│   ├── xlerobot_bringup/       # controller configs, top level launch
│   ├── xlerobot_teleop/        # joystick, twist mux, safety supervisor
│   ├── xlerobot_navigation/    # slam_toolbox, Nav2, EKF params
│   └── xlerobot_perception/    # sensor agnostic camera and lidar bringup
├── tools/                      # calibration and bus diagnostics
├── docs/decisions/             # ADRs
└── xlerobot.repos              # community deps via vcs
```
