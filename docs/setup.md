# Setting up and calibrating a robot

From a built workspace to a robot whose model matches the hardware. The order matters: each step trusts the ones
before it. Every result is a file, listed in [Where the results live](#where-the-results-live), so a step can be redone
on its own.

## 1. Software

Prepare the robot's computer ([docs/jetson.md](jetson.md): DDS buffers, Wi-Fi power saving, USB), build the workspace
([README](../README.md#getting-started)) and create the Python venv for the tools
([docs/tools.md](tools.md#python-environment)).

## 2. Device names

```bash
sudo python3 tools/xlerobot_setup_usb_modules.py
```

Pins the two servo buses to `/dev/xle-bus1` and `/dev/xle-bus2` and the cameras to stable names (udev rules matched
by USB port and serial). Once per robot, again after moving a cable to another port.

## 3. Servos

With the stack stopped, for each part:

```bash
python3 tools/calibrate_arm.py calibrate --arm left     # then right, then head
```

lerobot's procedure: the part in the middle of its range (the head looking straight ahead and level), then a sweep of
every joint. Writes the homing offsets and limits into the servos and lerobot's calibration file. `status --arm left`
reads positions, limits and torque back. Example files: [calibration-examples/](calibration-examples/).

## 4. Measured dimensions and wheels

Measure the wheel radius and the mount positions and put them in
`src/xlerobot_description/config/robot_dimensions.yaml`; upstream's numbers are wrong for this build (see the
[README](../README.md#before-you-plug-anything-in)). Then, with the base running:

```bash
python3 tools/calibrate_wheel_odometry.py straight-line --distance 1.0
python3 tools/calibrate_wheel_odometry.py rotate-in-place --turns 0.5
```

Each one drives, asks for the distance or angle you measured, and suggests the `wheel_radius` or
`wheel_separation_multiplier` correction for `src/xlerobot_bringup/config/controllers.yaml`.

## 5. Calibration boards

Two ChArUco boards (DICT_4X4_250), made with `tools/generate_charuco.py`. Print without scaling, then measure a square
with a ruler and pass the measured size to the tools: a printer's few percent end up directly in every distance.

| Board | Used for |
|---|---|
| 11x8 squares of 15 mm, 11 mm markers | wrist camera lenses, which focus close (the tools' default) |
| 7x5 squares of about 34 mm, about 26 mm markers | everything the head camera sees from 0.4-0.8 m: the arm calibration, the head lens and depth checks |

The board must be flat and rigid (glued to card or foam board).

## 6. Wrist camera lenses

```bash
python3 tools/calibrate_camera_charuco.py left     # then right
```

Move the 11x8 board in front of the camera at many angles and distances while it collects samples. Writes
`~/.ros/camera_info/<camera>.yaml`, which the camera driver loads at start.

## 7. Head camera

The Gemini 2 publishes its factory calibration (`/cameras/head/camera_info`), and it is used as is: a ChArUco lens
calibration of this camera (`tools/head_record.py`, then `tools/side_cam.py calibrate --from <folder>`) agreed with it
to within the fit's uncertainty.

Its mount on the head is `head_camera_joint` in `src/xlerobot_description/urdf/sensors.xacro`.
`tools/calibrate_head_pose.py` fits the floor plane in the depth cloud and prints the height, pitch and roll that make
the model agree with it; x, y and yaw cannot be seen from a plane.

## 8. Arms against the head camera

The head camera is the reference: the arms, their joint zeros and their wrist cameras are fitted to what it sees.
Lay the 7x5 board flat where the head camera and both wrist cameras can see it, then for each arm:

```bash
python3 tools/calibrate_arm_camera.py left --squares-x 7 --squares-y 5 --square-length 0.03386 --marker-length 0.02562
```

The head camera measures the board once; then the arm's torque is cut and you hold it in about 20 different poses where
its wrist camera sees the board, pressing Space at each (`--auto` takes them when the arm is still). The fit prints the
joint zero offsets, the wrist camera's mounting pose and the reprojection error. Copy them into
`robot_dimensions.yaml` under the arms: `model_offset` per joint, `wrist_camera`, and `frame_correction` (the arms'
frame relative to the body), rebuild, and check a few saved positions against the real arm. Expect about 1 cm
agreement: the servos sag and have backlash, so the last centimetre of a grasp comes from the wrist camera.

## 9. Head depth scale

```bash
python3 tools/head_record.py --depth --seconds 60     # on the robot; hold the 7x5 board flat and still at 35-80 cm
python3 tools/depth_scale.py ~/head_calib/calib_<time>
```

Compares the depth with the board's distance from the colour image, per distance band. A ratio that is the same at
every distance is a scale error: put the printed `depth_scale` into
`src/xlerobot_object_perception/config/object_perception.yaml`. This robot's camera reads 0.8% too far. The point
cloud the octomap uses is not corrected; for obstacles that is well within what matters.

## 10. Object perception

The detector and mask models are exported to ONNX on a machine with torch and built into TensorRT engines on the
robot; see [src/xlerobot_object_perception/README.md](../src/xlerobot_object_perception/README.md). To check poses
end to end, print the placement sheet (`tools/generate_placement_sheet.py`, PDF at actual size, measure its scale),
put a cube on a footprint and run `tools/cube_pose_eval.py` (colour baseline) or `tools/perception_check.py` (the
service). Tilt the head so all seven sheet markers are visible; with the bottom ones out of view the sheet's tilt is
poorly determined.

## Where the results live

| Step | Result |
|---|---|
| Device names | `/etc/udev/rules.d/99-xlerobot.rules` |
| Servos | in the servos, and `~/.cache/huggingface/lerobot/calibration/robots/{so101_follower,xlerobot_head}/` |
| Dimensions, joint offsets, wrist camera mounts, arms frame | `src/xlerobot_description/config/robot_dimensions.yaml` |
| Head camera mount | `head_camera_joint` in `src/xlerobot_description/urdf/sensors.xacro` |
| Wheel odometry | `src/xlerobot_bringup/config/controllers.yaml` |
| Wrist camera lenses | `~/.ros/camera_info/` |
| Head depth scale | `src/xlerobot_object_perception/config/object_perception.yaml` |
| Perception models | `~/.cache/xlerobot/` |
