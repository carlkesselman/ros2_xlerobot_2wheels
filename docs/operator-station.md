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
     sensor:=realsense bus1_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058116-if00 \
     bus2_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B91028673-if00
   ```

2. **rosie:** `tools/station.sh`, from rosie's own terminal or over ssh (`ssh rosie Developer/ros2_xlerobot_2wheels/tools/station.sh`).
   The viewer opens full screen on rosie's monitor either way.
3. **Mac**, at the Mac or over ssh: `pixi run tools/lerobot.sh record --repo-id ... --task "..."`.
4. **At the station:**

| Gamepad (PlayStation mode) | Viewer keys | Does |
|---|---|---|
| triangle | | leader arms on / off (they blend in from where the follower is over 2 s) |
| options, tap | right arrow, Enter | keep the episode and end it now; during the reset, end the reset |
| share | left arrow, r | throw the episode away and record it again |
| options, hold 1.5 s | q | keep the episode and stop recording |
| R1 held + left stick | | drive the base |
| d-pad | | point the head |
| | f | full screen on / off |

The recorder's own terminal keys still work as well.

## The station viewer

`station_viewer` (in `xlerobot_teleop`) draws the head camera large and the two wrist cameras beside it (under it on
a squarer screen). The bar underneath shows:

- **ARMS ON / ARMS OFF**, from `/leader_arms/engaged`. It says "no leader arms" when leader_arms isn't running.
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

## If something is off

- **No window over ssh:** `station.sh` points SDL at the desktop session's Wayland socket
  (`/run/user/1000/wayland-0`) and at `DISPLAY=:0` for XWayland. rosie's desktop (labwc) must be logged in, which it
  is at boot (lightdm autologin).
- **"no leader arms":** leader_arms didn't start. Its log says why: usually a bus board missing (a serial number not
  found) or no `xlerobot.json`.
- **Buttons do nothing:** the pad came up in another mode. `ros2 topic echo /joy` on rosie shows its buttons, and
  `joystick_ps.yaml` and station.launch.py's `ENGAGE_BUTTON` say which numbers are used.
- **A camera stays red:** the camera on elroy, or the wifi. Measure the frame age on elroy before blaming the network.
