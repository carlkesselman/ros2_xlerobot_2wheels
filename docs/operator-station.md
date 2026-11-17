# The operator station on rosie

For recording demonstrations in a room away from the Mac. The leader arms, the gamepad and a small monitor plug into
**rosie** (the Raspberry Pi 5, [platform.md](platform.md) §1-2). The monitor shows the cameras and what the leader arms
and the recorder are doing, so nobody has to be at the Mac. The recorder and the policy still run on the Mac
([lerobot.md](lerobot.md)), which can sit in another room. Rerun stays on the Mac too, for debugging.

| Machine | Runs |
|---|---|
| elroy | bringup: controllers, cameras, the arms' and head's servos |
| rosie | `tools/station.sh`: joy_node, teleop_twist_joy (base), head_teleop (d-pad), leader_arms, station_viewer |
| Mac | `tools/lerobot.sh record` (or `policy`); Rerun when needed |

All three talk over wifi with the default Fast DDS, domain 0, on the same subnet. No peer configuration needed. The
station viewer takes the cameras' 10 fps viewer copies, about 1.3 MB/s for all three. The recorder on the Mac pulls the
30 fps JPEG streams, about 4 MB/s. So the training room needs good wifi whichever machine the leader arms are on.

## Hardware on rosie

- **Leader arms:** both Feetech bus boards (left `5B90149222`, right `5A7A017922`) on rosie's USB, with their own DC
  supply. Their servos run torque off and draw little.
- **Gamepad:** the 8BitDo, wired, in PlayStation mode (`054c:05c4`).
- **Monitor:** the Pi 5's video outputs are micro-HDMI.
- rosie's user is already in `dialout` (serial) and `input` (gamepad), and wifi power saving is off (checked
  2026-11-15).

## Setup, once

rosie runs Debian 13, which has no ROS packages. ROS Jazzy comes from RoboStack through pixi, like on the Mac:
the `rosie` environment in `pixi.toml` (ros-base, joy, teleop_twist_joy, pyserial, pygame, the build tools). It lives in
the checkout's `.pixi`, so nothing is installed system-wide.

```bash
ssh rosie
curl -fsSL https://pixi.sh/install.sh | PIXI_VERSION=v0.84.0 PIXI_NO_PATH_UPDATE=1 bash   # ~/.pixi/bin/pixi
git clone -b rosie https://github.com/carlkesselman/ros2_xlerobot_2wheels.git ~/Developer/ros2_xlerobot_2wheels
cd ~/Developer/ros2_xlerobot_2wheels && tools/station.sh build
```

`leader_arms` also needs the follower's calibration, the same file the Mac uses. Copy it again after any follower
recalibration:

```bash
scp ~/.cache/huggingface/lerobot/calibration/robots/xlerobot/xlerobot.json \
    rosie:.cache/huggingface/lerobot/calibration/robots/xlerobot/xlerobot.json
```

Like elroy, rosie runs the committed and pushed code: change it on the Mac, push, then on rosie run `git pull --ff-only`.
Python, launch and config changes take effect at the next start (`--symlink-install`); new files need
`tools/station.sh build`.

## A recording session

1. **elroy:** bringup with the arms, head and cameras, and without its own gamepad nodes:

   ```bash
   ros2 launch xlerobot_bringup bringup.launch.py teleop_joy:=false with_arms:=true with_head:=true cameras:=true \
     sensor:=realsense arm_max_velocity:=4.5 \
     bus1_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058116-if00 \
     bus2_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B91028673-if00
   ```

   Keep `arm_max_velocity:=4.5`. Without it the arms keep the URDF's 0.8 rad/s limit, which ros2_control enforces,
   and the follower trails every quick leader move. elroy's log then fills with "Command of at least one joint is out
   of limits".

   Recording doesn't move the cart, so the base's pack can stay off: add `with_base:=false`. That leaves the
   ODrive, diff_drive_controller, twist_mux and the safety supervisor out. Without it, bringup won't start while the
   ODrive is off USB, because ros2_control refuses hardware that isn't there. R + left stick then drives nothing,
   and the wheel links have no transforms (their joints have no states), which only shows in RViz.

2. **rosie:** `tools/station.sh`, from rosie's own terminal or over ssh (`ssh rosie Developer/ros2_xlerobot_2wheels/tools/station.sh`).
   The viewer opens full screen on rosie's monitor either way.
3. **Mac**, at the Mac or over ssh: `pixi run tools/lerobot.sh record --repo-id ... --task "..."`.
4. **At the station:**

| 8BitDo | Viewer keys | Does |
|---|---|---|
| X (top face button) | | leader arms on / off (they blend in from where the follower is over 2 s) |
| start, tap | right arrow, Enter | keep the episode and end it now; during the reset, end the reset |
| select | left arrow, r | throw the episode away and record it again |
| start, hold 1.5 s | q | keep the episode and stop recording |
| R (right shoulder) held + left stick | | drive the base |
| d-pad | | point the head |
| | f | full screen on / off |

The 8BitDo's letters are Nintendo-style, and PlayStation mode maps the buttons by position: X is the top face button,
Y the left one. On a real PlayStation pad the same buttons are triangle, options and share.

The recorder's own terminal keys still work as well.

## The station viewer

`station_viewer` (in `xlerobot_teleop`) draws the head camera large and the two wrist cameras beside it (under it on
a squarer screen). The bar underneath shows:

- **ARMS ON / ARMS OFF**, from `/leader_arms/engaged`. It says "no leader arms" when leader_arms isn't running.
  **ARMS STOPPED** (red) means the guard disengaged the arms, and the reason follows in red. It also flashes over
  the head image for a few seconds. Servos at 55 °C or more are listed in amber ("hot: right wrist_flex 58°C"). See
  [The guard](#the-guard).
- **The recorder**, from `/recorder/status`:
  - `REC ep 3 12.4/30 s saved 2/20` while recording;
  - `PAUSED` with the reason (a stale camera or a disengaged leader; the pause doesn't use up episode time);
  - `RESET 8 s` between episodes;
  - `WAITING` while the robot's streams aren't all there yet;
  - `STOPPED`;
  - `recorder off` when none has run, or `recorder silent` when it stopped answering.
- The recorder's spoken prompts ("Recording episode 3", "Reset the scene") flash over the head image. The Mac's
  speech is in another room.
- A camera whose frames stop gets a red frame and the age of its last frame.
- Over the head image are the head's pan and tilt in degrees, with a scale for each between the joint's limits. Pan
  runs along the bottom, with turned fully left at the left end; tilt runs down the right edge, with up at the top.
  It reads `/joint_states` and the limits from `/robot_description`, and says "no joint states" when they stop.

The buttons and keys publish `/recorder/command` (`keep`, `redo`, `stop`), which `record.py` treats like its own
keys.

Options, as `station.launch.py` arguments through `tools/station.sh`:

- `head_topic:=/cameras/head/color/image_raw/compressed`: the head at its full 30 fps, for smoother pointing (about
  1 MB/s more).
- `leader_arms:=false`, `viewer:=false`: leave a part out.
- `fullscreen:=false`: a window.
- `joy_layout:=...`: another gamepad mode (`ps` on rosie, `sdl` on the Mac, `linux` for joystick.yaml's pad).

To check the viewer without looking at it, the `snapshot_path` parameter writes a PNG of the window every 2 s.

The same launch runs on the Mac (`tools/station.sh` there uses pixi's default environment, in a window), for
training next to the Mac with the leader arms on a long USB cable.

## The guard

A position servo that can't reach its command pushes as hard as it can, for as long as it is commanded. On
2026-11-16 the follower's right wrist_flex was sent to +1.76 rad, folded, but jammed at +1.51. It pushed there for
almost two hours, including while the leaders were disengaged, reached 85 °C and shut down. That joint had already
lost a servo on 2026-11-13. leader_arms now watches for this while the leaders are engaged:

| Check | Does |
|---|---|
| An arm joint (not a gripper) more than 0.15 rad from its command for 2 s, without getting closer | disengages: "right wrist_flex stuck 14 deg short of its command for 2 s" |
| Any arm servo, grippers too, at 65 °C (the servos' own cutoff is 70 °C) | disengages |
| A servo at 60 °C or more | engaging is refused until it cools |
| Leader beyond the follower's limits | the command stops at the model's own limits, not the 0.1 rad wider ones ros2_control allows |
| Arm folded, wrist bent back (down) | the wrist command is lifted to keep elbow + wrist above a limit that depends on the wrist roll: -4.33 rad near normal roll, -3.76 rolled near 180°, where the wrist camera faces the upper arm and touches first. Folded, the gripper meets the upper arm 22–26° before the wrist's end at normal roll, about 30° sooner rolled. The limit eases off one for one as the elbow opens (robot_dimensions.yaml `elbow_plus_wrist_min_by_roll`, measured 2026-11-16) |

Any disengage, by X or by the guard, tells joints more than 0.05 rad from their command to hold where they are.
Nothing keeps pushing after that. The other joints keep their last command. Grippers keep theirs, so an object stays
gripped, unless it is a gripper that overheated. The temperatures come from `/dynamic_joint_states` (bringup's
joint_state_broadcaster). The thresholds are leader_arms parameters (`stall_error`, `stall_s`, `temp_warn`,
`temp_resume`, `temp_stop`, `hold_error`).

A trip pauses the recording like any disengage. Look at the arm before engaging again: something is in the way, or
the joint can't reach where the leader asks.

### The robot's own guard

leader_arms' guard only acts while the leaders drive. On the robot, bringup also starts **servo_guard**
(xlerobot_teleop). It watches every arm, gripper and head servo whatever commands it: the leader arms, the lerobot
policy, MoveIt, the skills, the d-pad. It reads each trajectory controller's `controller_state` (command and measured
position) and the servo temperatures.

| Check | Does |
|---|---|
| A joint (not a gripper) more than 0.15 rad from its command, within the joint's limits, for 3 s without getting closer | **hold**: the controller is swapped for its hold twin, which keeps the joints stiff where they are |
| Any servo at 65 °C | hold its controller |
| Any servo at 70 °C | every controller on its bus held, then **that bus's torque cut** (the arm goes limp) |

The hold twins (`left_arm_hold`, `right_arm_hold`, `left_gripper_hold`, `right_gripper_hold`, `head_hold`) are
forward_command_controllers that bringup loads inactive (controllers.yaml). While one is in, no commander reaches
those joints: they all talk to the trajectory controller.

The station viewer shows a hold or a cut in red ("robot guard holds right arm: ..."), flashed over the head image too.
To bring the joints back, **engage the leaders (X)**: leader_arms sends servo_guard `reset`. Or from any machine:

```bash
ros2 topic pub --once /servo_guard/command std_msgs/msg/String "{data: reset}"
```

The reset is a topic, not a service, so it also gets from rosie to elroy. It's refused while a servo is 60 °C or
more. After a cut, the reset first sets the holds to where the limp arm is now and gives torque back after that, so
the arm doesn't jump back up. An emergency stop set by hand (`/emergency_stop_xlerobot_bus2`) is left alone.
`servo_guard:=false` on bringup turns it off.

## If something is off

- **No window over ssh:** `station.sh` points SDL at the desktop session's Wayland socket
  (`/run/user/1000/wayland-0`) and at `DISPLAY=:0` for XWayland. rosie's desktop (labwc) must be logged in, which it
  is at boot (lightdm autologin).
- **"no leader arms":** leader_arms didn't start. Its log says why: usually a bus board missing (a serial number not
  found) or no `xlerobot.json`.
- **Buttons do nothing:** the pad came up in another mode. `ros2 topic echo /joy` on rosie shows its buttons, and
  `joystick_ps.yaml` and station.launch.py's `ENGAGE_BUTTON` say which numbers are used.
- **A camera stays red:** the camera on elroy, or the wifi. Measure the frame age on elroy before blaming the network.
  A wrist camera that drops off USB or stalls turns red for a few seconds: its usb_cam node exits and is started again.
- **A service call to rosie gets no answer** (`ros2 service call /leader_arms/engage ...` from elroy): rosie's Fast DDS
  (RoboStack) and elroy's (apt) disagree on small fixed-size messages. leader_arms logs `Change payload size of '24'
  bytes is larger than the history payload size of '11' bytes`. Call it on rosie. Topics with variable-size
  messages (joint states, trajectories, images, `/joy`, the recorder's strings) are not affected.
