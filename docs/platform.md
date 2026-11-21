# Carl's XLeRobot: the platform

What the robot is made of, how it is connected and powered, and every measurement the stack depends on, with its
status. Standard XLeRobot 0.4.0 except where this says otherwise. Branch `rosie`; the porting plan is
[rosie-port-plan.md](rosie-port-plan.md).

Sources: `../XLeRobot` (branch `rosie-config`: `config/README.md`, `ORIN-SETUP.md`, `RUNNING.md`, `cart-remote.yaml`,
`operator-leader-host.yaml`, the udev rules, `servo_tools/hw.py`, `odrive_check.py`), the lerobot fork's `FORK.md`, and
Carl (2026-11-01). All the questions in §7 are answered; §6 lists the measurements still to take.

## 1. Machines

```
        OPERATOR STATION                                   CART
  rosie, Raspberry Pi 5                         elroy, Jetson Orin Nano
  ─────────────────────                         ───────────────────────
  2 SO-101 leader arms         leader actions    2 SO-101 follower arms + wrist cams
  8BitDo SF30 Pro gamepad      ─────────────>    2-DOF head + RealSense D415
  Rerun viewer                 <─────────────    2 brushless hub wheels on an ODESC
                                  video          LD19 lidar
                       wifi, network "homenet"
```

| | rosie | elroy | HomeBoy (Mac mini) |
|---|---|---|---|
| Role | operator station: leaders, gamepad, viewer | the robot: motors, sensors, ROS stack | development, build (pixi), RViz, viewer; leader arms and gamepad; LeRobot datasets and policies ([lerobot.md](lerobot.md)) |
| Hardware | Raspberry Pi 5 | Jetson Orin Nano Super developer kit, 8 GB | Mac mini, M5 Pro (15-core CPU, 16-core GPU), 48 GB |
| OS | Raspberry Pi OS | JetPack 7 (Ubuntu 24.04, CUDA 13) | macOS 27 |
| Network | wifi `wlan0`, connection `preconfigured`, 192.168.1.100 | wifi `wlP1p1s0` (RTL8822CE), connection `homenet` | |
| Must-do | wifi power saving off | wifi power saving off, plus the `rtl8822ce` modprobe options (`ORIN-SETUP.md`) | |

**Code on elroy.** `~/Developer/ros2_xlerobot_2wheels` on elroy is a git checkout of `rosie`, tracking GitHub
(`carlkesselman/ros2_xlerobot_2wheels`). Change code on the Mac, commit and push, then on elroy:

```bash
cd ~/Developer/ros2_xlerobot_2wheels && git pull --ff-only
colcon build --symlink-install --packages-select <packages with C++ changes or new files>
```

elroy is built with `--symlink-install`, so changed Python, launch and config files take effect at the next launch
without a build. A package with C++ changes, or with files added to its install, needs the build. Don't edit on elroy;
`git status` there should stay clean. The vcs-imported dependencies in `src/` (`xlerobot.repos`) are not tracked here.
elroy's `sts_hardware_interface` carries the local `servo_acceleration` patch (copy in
`tools/sts_servo_acceleration.patch`), so re-importing it needs the patch reapplied.

## 2. Operator station (rosie)

| Item | Detail |
|---|---|
| Left leader arm | SO-101 leader, Feetech bus board `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B90149222-if00` |
| Right leader arm | SO-101 leader, Feetech bus board `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A68009991-if00` (replaced 2026-11-21, was `5A7A017922`) |
| Gamepad | 8BitDo SF30 Pro, wired, PlayStation-compatible mode (`054c:05c4`, 6 axes, 13 buttons). Left stick Y forward/back, right stick X turn, D-pad head, R1/L2 speed up/down. Re-probe if it power-cycles into another mode |
| Leader calibration | lerobot JSONs on rosie (`~/.cache/huggingface/lerobot/calibration/`) |
| Power | mains: the Pi 5 on a standard Raspberry Pi USB-C power supply; the leader arms on their own DC power supply. Nothing at the operator station is on the cart's batteries |

## 3. Cart (elroy)

### 3.1 Compute

Jetson Orin Nano developer kit, powered from the Solix through a USB-C PD to DC barrel cable set to 20 V (the
developer kit takes 9-20 V on the barrel; 20 V is the top of that range). Four USB-A ports in two vertical stacks
(§4).

### 3.2 Arms and head (Feetech STS3215, 12 V)

| Bus | Board (by-id serial) | IDs | Joints |
|---|---|---|---|
| 1 | `usb-1a86_USB_Single_Serial_5A7A058116-if00` | 1-6, 7-8 | left arm; head pan (`head_motor_1`) and tilt (`head_motor_2`) |
| 2 | `usb-1a86_USB_Single_Serial_5B91028673-if00` | 1-6 | right arm |

- 14 servos, 1 Mbps half-duplex TTL. The head depends on bus 1: a missing head servo stops the left arm too.
- Servo profile from the lerobot setup: arm Acceleration 100 (limits inrush; both buses once went silent from the
  first all-joints command), head Goal_Velocity 600 / Acceleration 40.
- Calibration: lerobot JSONs on elroy, `so_follower/xlerobot_arms_{left,right}.json`.
- Udev: ModemManager must ignore `1a86` (`98-feetech-no-mm.rules`); the user is in `dialout`.

### 3.3 Cameras

| Camera | Model | Address | Mode |
|---|---|---|---|
| Head | Intel RealSense **D415** (no IMU, rolling shutter, ~65°×40° depth FOV) | serial `836112060112` | 640×480×30 colour; depth for ROS |
| Left wrist | Innomaker U20CAM-1080p-S1 | `/dev/cam_left`, udev on `ID_PATH` `platform-3610000.usb-usb-0:2.2.2:1.0` | MJPG 640×480×30, `buffer_size 2` |
| Right wrist | Innomaker U20CAM-1080p-S1 | `/dev/cam_right`, `ID_PATH` `…usb-0:2.2.4:1.0` | MJPG 640×480×30, `buffer_size 2` |

Both Innomakers report serial `SN0001`, so they are told apart only by hub port: moving a camera to another hub port
breaks its name silently. The D415 needs librealsense's `99-realsense-libusb.rules` and the `video`/`plugdev` groups.

### 3.4 Base

| Item | Detail |
|---|---|
| Motors | two L-faster 6.5 in brushless hub motor wheels (hoverboard replacement: 350 W, 36 V rated, 800 rpm, 3 kg each), 15 pole pairs, Hall sensors, KV ~22 (800 rpm / 36 V agrees; about 440 rpm, 3.8 m/s no-load on the 20 V battery), torque constants 0.371 (right) / 0.379 (left) Nm/A from back-EMF |
| Controller | ODESC (ODrive v3 clone), firmware 0.5.1, USB `1209:0d32` (`0d31` = DFU), libusb, udev `99-odrive.rules` |
| Wiring | M0 = axis0 = **right** wheel, M1 = axis1 = **left** wheel; the left motor is the mirrored one |
| Tuning (saved on the board) | vel gain 5.2, integrator 16.3, encoder bandwidth 50, current limit 10.5 A |
| Bus limits (`odrive-check.sh --apply`) | brake resistor 2 Ω / 50 W, `dc_max_positive_current` 10 A, no regen into the supply, undervoltage trip 16.5 V, overvoltage trip 26 V |
| Power | 20 V (5S) 4 Ah drill battery through an XH-M609 low-voltage cutoff relay, see §5 |
| Casters | two passive casters at the rear in the stock RÅSKOG mounting place, larger than stock to level the cart; the drive wheels are at the front, no front casters |
| Chassis | IKEA RÅSKOG cart, 17.2 kg complete |

### 3.5 Lidar

LDROBOT LD19 (Waveshare D500 developer kit), USB serial adapter, 230400 baud, `/dev/ldlidar`. Mounted low on the cart:
centreline, over the drive axle, in the gap between the bottom basket's rim and the middle basket's underside, scan
plane 0.375 m (built and measured 2026-11-13). The head spacer in `XLeRobot/hardware/lidar_bracket/` is retired.

## 4. USB

```
elroy (Orin Nano)
├── USB-C ──── RealSense D415           (root port 2-2, 5000M; the FUSB301 puts it in host mode)
├── internal hub port 4 ── ODESC          (USB 2, libusb, serial 387734503135)
├── internal hub port 2 ── powered 4-port hub ─┬─ bus 1 board (left arm + head)   (hub port 1)
│                                              ├─ left wrist camera    (hub port 2 -> /dev/cam_left)
│                                              ├─ bus 2 board (right arm)        (hub port 3)
│                                              └─ right wrist camera   (hub port 4 -> /dev/cam_right)
└── internal hub port 3 ── LD19 lidar adapter (/dev/ldlidar)
```

Measured after the rebuild, 2026-11-13: wrist cameras at `ID_PATH` `…usb-0:2.2.2` (left, identified by covering it)
and `…usb-0:2.2.4` (right); `/etc/udev/rules.d/99-cameras.rules` updated to match. `/dev/ldlidar` is in
`/etc/udev/rules.d/99-xlerobot.rules` (`tools/xlerobot_setup_usb_modules.py --assign ldlidar=/dev/ttyUSB0`), bound to
`ID_PATH` `…usb-0:2.3` because the CP2102's serial is the factory `0001`: moving the lidar to another port loses the
name. The ODESC (internal hub port 4) needs no name, the odrive package finds it by USB id.

- The hub is full; internal hub port 1 is the cart's only free port.
- The D415 is on elroy's USB-C port, apart from the hub and its two wrist cameras (it was on a USB-A port, "stack A",
  until 2026-11-13). If it fails to show up there, replug it: the FUSB301 only switches the port on when it sees
  the attach. That follows XLeRobot's
  `config/ORIN-SETUP.md` §3: before the cart's cameras moved to the Orin they ran on a Pi, where a wrist camera on the
  same USB host controller as the RealSense stopped delivering frames once the RealSense streamed. Stack A's other
  device, the ODESC, carries little data. `ORIN-SETUP.md` measured the
  RealSense at `ID_PATH` `…usb-0:1.3` and the hub's cameras under `…usb-0:2.1.x`.
- The two bus boards are CH340s (`1a86`). Check their `latency_timer` (ARCHITECTURE §4.3).

## 5. Power

```
Anker SOLIX C300 DC (288 Wh LiFePO4, 300 W total, 600 W surge)
├── 12 V car socket (120 W, 10 A) ── 12 V car adapter (switch) ── servo supply: both follower bus boards (14 STS3215)
├── USB-C PD ── USB-C to DC cable, 20 V ── Orin Nano barrel jack
└── USB-C1 ── powered 4-port USB hub (wrist cameras, both bus boards' USB side)

20 V drill battery, 5S, 4 Ah (~72 Wh), switch on its connector ── XH-M609 cutoff relay (16.0 V cut, 10 A) ── ODESC ── 2 hub motors
                                                                                    └── 2 Ω / 50 W brake resistor
```

The C300 DC's figures are from retailer listings (not checked against Anker's own sheet): 288 Wh, 300 W
continuous with 600 W surge, two 140 W USB-C PD ports, a 120 W car socket. The car socket's 10 A is the limit that
matters: it carries all 14 servos, and the arm Acceleration of 100 (§3.2) exists because twelve arm servos starting at
once from rest tripped the supply, cutting both buses on a run's first command. Keep that profile; a servo
supply with more current would be the fix if the arms ever need to move harder.

Rough run time, to be measured (§6.5): the Solix carries the Orin (up to ~25 W), the hub, cameras and lidar
(~5-10 W) and the servos (a few W holding, tens of W moving), on the order of 50-80 W, so about 3-4 h. The drill
battery (72 Wh) carries only the base: 1-2 h of mixed driving at a guess.

The base runs from its own battery, separate from the Solix. The ODESC's bus limits are set for it
(`odrive-check.sh --apply`): undervoltage trip at 16.5 V, just above the relay's 16.0 V cut, so the board faults
cleanly before the relay drops it; overvoltage trip at 26 V (the pack is 21 V full); at most 10 A drawn, the relay's
rating; no regen current back into the pack, with the brake resistor taking braking energy instead.

Notes:
- **Grounds.** The Orin, the bus boards and the ODESC share ground through USB, and the ODESC runs from its own pack,
  so USB ties the drill battery's negative to the Orin's (and the Solix's) ground. Large motor currents can then
  return partly through the USB cable, the classic cause of ODrive USB disconnects. ODrive recommends a USB
  isolator. Worth fitting if the board ever drops off USB while driving.
- **Cut-offs: two switches** (2026-11-01).
  - The switch on the drill battery's connector cuts the ODESC and so the motors, which coast (at 1 m/s their
    back-EMF is only ~5 V, KV ~22).
  - The switch on the 12 V car plug adapter cuts the servo supply, so both arms and the head go limp. Arms that are
    raised fall, so lower them first unless it's an emergency.

  The Orin, the hub and the cameras stay up on either switch, so the stack can be inspected afterwards. In
  software: `diff_drive_controller`'s 0.25 s command timeout, the ODESC's own 0.5 s watchdog (from 1 m/s the cart
  stops within ~0.6 m), and for the arms, torque off on disconnect; none of these helps if the Orin or the ROS
  stack hangs with a velocity latched, which is what the switches are for. Before Nav2 drives the cart unattended,
  check the battery switch can be reached and hit quickly from beside a moving cart; if not, a latching mushroom
  switch in the same line does the same job.

## 6. Measurements

Every number the stack depends on. "Cart" = measured on this cart; "vendor" = spec sheet; "ref" = the reference
robot's value standing in; "—" = no value yet. Where: `dims` = `robot_dimensions.yaml`, `ctrl` = `controllers.yaml`.

### 6.1 Base and chassis

| Measurement | Value | Status | Where | How |
|---|---|---|---|---|
| Wheel radius | 0.0824 m | cart, tyre circumference 51.8 cm ÷ 2π, 2026-11-01 (6.5 in wheel) | dims, ctrl | then `calibrate_wheel_odometry.py straight-line`, 5 m+ on the real floor |
| Track (contact patch to contact patch) | 0.527 m | cart, 2026-10-21 | dims, ctrl | then `rotate-in-place` for `wheel_separation_multiplier` |
| Per-wheel radius scale | 1.0 | — | dims, ctrl | UMBmark square |
| Wheel width (tyre) | 0.046 m | cart, 2026-11-01 | dims | calipers |
| Wheel mass (motor + tyre) | 3.0 kg | vendor (L-faster 6.5 in hub motor wheel: 350 W, 36 V rated, 800 rpm) | dims | scale |
| Axle height above floor | 0.085 m | cart, about, 2026-11-01 | dims | ruler; only places the model, wheel_radius comes from the circumference |
| Base pitch (front deck vs rear) | 0.0 rad | cart, level: front and rear heights within 1-2 mm (under 0.005 rad), 2026-11-01 | dims | ruler: deck height at front and rear, over their distance |
| Caster diameter | 0.0745 m, trail = radius | cart, 2026-11-01 | dims, nav2 footprint | calipers |
| Caster positions (swivel screws: x, y) | -0.325, ±0.20 m | cart, rear pair, along the floor, about, 2026-11-01 (axle clamp 18 mm ahead of the front rail + 35 cm RÅSKOG frame gives about 0.35) | dims, nav2 footprint | from the drive axle line and the centreline |
| Cart body: length, width, height | **0.295**, **0.425**, **0.780** m | length, width and height (floor to the top rim) measured 2026-11-01, width across the outside of the vertical side rails (the top basket alone is 0.405) | dims | tape, outside of the rim |
| Overall width (outer edges of the drive wheels) | 0.573 m (0.527 track + 0.046 tyre) | cart, 2026-11-01: nothing else is wider | nav2 footprint (±0.29) | — |
| Neck overhang past the top basket's rear rim | about 0.07 m by tape; 0.054 m in the Fusion model, the box ending 0.370 m behind the axle, inside the nav2 footprint (-0.41) | cart, 2026-11-01: arms and neck are on the rear (caster-side) edge of the top basket, the arms reach forward over it | head and arm mounts | horizontal, outside of the rim to the neck's edge |
| Top basket rims from the axle line (front, rear) | 0.021 / 0.316 m behind | cart, front rim 20-22 mm behind, 2026-11-01; rear from the 295 mm length | dims (`body_x_offset`, mesh placement) | tape; the reference notes had 22 mm ahead / 260 mm behind |
| Total mass | 17.2 kg | cart | dims (`body_mass` 9.2 = 17.2 less the other links' 7.97) | scale |
| CoM height | 0.35 m | — | dims (tip-over limit) | tilt test, or estimate from component heights |
| Footprint polygon for Nav2 | derived | — | `nav2_params.yaml` | from the dimensions above |
| Torque constants | 0.371 / 0.379 Nm/A | cart, 2026-10-25 | ODESC | `base-characterize.sh blocks` |
| Pole pairs | 15 (Hall cpr 90 = 6 × 15, 5.75 mm of travel per count) | cart, from cpr 90 in XLeRobot's config/cart_selftest.py; not yet read off the board | ODESC | `odrive-check.sh` summary, or `--halls` and one wheel turn |
| Speed and acceleration limits | 1.0 m/s, 1.0 m/s² | cart, 2026-10-25 | dims, ctrl | `base-characterize.sh step --candidates`; repeat a brake test with the arms up before raising |

### 6.2 Lidar

| Measurement | Value | Status | How |
|---|---|---|---|
| x, y from the drive axle / centreline | 0, 0 | cart, 2026-11-13: over the axle, centred (bracket: Fusion "Lidar Base") | plumb line from the LD19 axis to the axle |
| Mounting surface height (the LD19's base) | 0.402 m (scan plane 0.375, head bottom 0.367) | cart, 2026-11-13: lower plate's bottom 15 7/8 in less the 1.2 mm boss plate; LD19's bottom 14 1/2 in (0.368) agrees | tape, floor to the lower plate and to the LD19's bottom |
| Scan plane to mounting surface | 0.027 m | LD19 base to scan plane (the 6 mm plate is now below the mounting surface, not in this value) | |
| Upside down | true | cart, 2026-11-13 | as mounted |
| Yaw (forward arrow vs robot front) | +90° | cart, 2026-11-13: arrow to the robot's left | checked in RViz against a wall ahead |
| Cart frame removed from the scan | two boxes in base_link, x -0.26..-0.10, y 0.16..0.28 and -0.28..-0.16; range_min 0.12 | cart, 2026-11-13: in the scan plane the only part of the cart is the side frames' uprights (a pair of tubes per side at x about -0.135 and -0.180, y ±0.20), 0.21-0.34 m away at ±120..±138°; no corner posts and nothing else under 0.35 m. `config/laser_filters.yaml` drops them on the way from `/ldlidar_node/scan` to `/scan` | 15 s of raw LD19 packets read from `/dev/ttyUSB0` and decoded, robot frame from the mount above; then RViz: `/scan` against the raw scan |

### 6.3 Head and D415

| Measurement | Value | Status | How |
|---|---|---|---|
| Mast (pan axis) x from the axle line | 0.328 m behind | cart, 2026-11-02: Fusion "XLeRobot 0.3 Upper Body" (with servos and D415), pan horn 22.4 mm behind the shoulders' backs, which butt against the inside of the top basket's rear wall (0.3058, the neck box's slot over the rim); the box behind the neck reaches 63 mm past the rim (tape about 70) | tape |
| Pan servo horn face above floor | 1.118 m | cart, CAD (Fusion, the neck box's slot on the rim); tape 1.118 | tape |
| Tilt axis from the pan axis (forward, up) | 0.031 m, 0.052 m | cart, Fusion (tilt servo horn) | CAD |
| D415 front glass centre: height, ahead of the tilt axis, left of the pan axis | 1.196 m, 0.034 m, 0.003 m | cart, CAD, head level; tape 1.200 high | tape |
| Pan/tilt zero steps and directions | ref values | ref | `tools/calibrate_arm.py --arm head` |
| Pan/tilt mechanical limits | upstream URDF | ref | move to the stops |
| D415 intrinsics and depth scale | factory | vendor | check: `tools/depth_scale.py`, ChArUco |
| D415 yaw against the lidar | — | — | depth cloud against the scan on a far wall (ARCHITECTURE §5 method) |

### 6.4 Arms and wrist cameras

| Measurement | Value | Status | How |
|---|---|---|---|
| Arm mount x, y (pan axes), z (base bottom), yaw | **0.054 behind, ±0.1356, 0.772 m, forward (`shoulders: front_corners`, Fusion "XLeRobot 0.4 Upper Body Split Shoulders"), built and measured 2026-11-13: horn to axle 54 mm both arms, horn to horn 10 11/16 in, platform 1/4 in under the rim (CAD 0.0593, 0.759)**. The rear shoulders are retired | rear values: cart, CAD (Fusion upper body), 2026-11-02: pan axes 67 mm ahead of the shoulders' backs against the inside of the rear wall, 265.7 mm apart; z from the neck box's slot resting on the rim (tape 0.765); yaw ref. The reference robot's mount and frame corrections are reset | CAD |
| Servo calibration (`zero_step` per joint) | cart values in `robot_dimensions.yaml` | cart, 2026-11-13: each arm held stretched straight forward, read raw; gripper = range_min + 244. Right wrist_flex is a new servo, recalibrated the same day (`servo.sh recal`). The left wrist_flex's calibrated middle is about 16° off straight, so its full down travel maps to -2.08 rad, past the model's -1.66 limit | `tools/calibrate_arm.py`, arms held straight |
| Model offsets, frame and mount corrections | ref values | ref | `tools/calibrate_arm_camera.py` (ChArUco, 20 poses per arm), `tools/arm_accuracy.py --fit` |
| Wrist camera extrinsics | ref values | ref | `calibrate_arm_camera.py` |
| Wrist camera intrinsics | — | — | `tools/calibrate_camera_charuco.py` |
| Rest and folded poses | ref values | ref | `calibrate_arm.py status` with the arms parked |

### 6.5 Electrical

| Measurement | Status | How |
|---|---|---|
| Servo voltage under load, per servo | — | `XLeRobot/config/servo.sh power` |
| Bus board `latency_timer` | — | `/sys/bus/usb-serial/devices/*/latency_timer` |
| ODESC bus voltage at rest and under acceleration (sag on the drill battery) | — | `odrive-check.sh --watch` |
| Run time on the Solix, and on the drill battery | — | Solix: its display's watts and remaining time during a session; drill battery: `odrive-check.sh --watch` voltage over a session |

## 7. Open questions

1. ~~What powers the ODESC?~~ A 20 V 4 Ah drill battery (2026-11-01), §5.
2. ~~What powers the hub, the Pi 5 and the leader arms?~~ Hub: the Solix's USB-C1. Pi 5: a standard Raspberry Pi power supply. Leader arms: their own DC supply (2026-11-01).
3. ~~Which Solix?~~ Anker SOLIX C300 DC (2026-11-01), §5.
4. ~~Emergency stop or main switch?~~ Two switches: the drill battery connector's cuts the base, the 12 V car plug adapter's cuts the servos (2026-11-01), §5.
5. ~~Other parts?~~ None (2026-11-01): the list above is complete.
