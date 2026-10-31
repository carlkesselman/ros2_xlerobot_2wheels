# ADR-0003: Head camera - Orbbec Gemini 2

- Status: Accepted
- Date: 2026-09-20

## Context

Object poses, octomap obstacle avoidance and, later, visual SLAM need a depth camera on the head. Navigation already
runs on a 2D lidar and is not part of this decision. Candidates:

- **Orbbec Gemini 2** (on hand): RGB-D with an IMU, supported by the vendor's `ros-jazzy-orbbec-camera` package.
- **Velodyne VLP-16** (on hand): predictable with `slam_toolbox` and Nav2, but about 830 g and 8 W on a top-heavy
  battery-powered cart, and no input for cuVSLAM.
- **RealSense D435i / D455** (not on hand): the easiest route to the Isaac stack, and upstream XLeRobot ships head
  mounts for both.

## Decision

The Orbbec Gemini 2, through the vendor's ROS 2 package (`gemini2.launch.py`, no source build), selected with
`sensor:=gemini`. It gives colour at 1280x720 and about 24 Hz, depth registered to colour at about 20 Hz, a point cloud
and the IMU. A plain USB webcam (`sensor:=usbcam`) stays the stack's default, since most builds have no depth camera.

The driver publishes its own extrinsic TF tree rooted at `<camera_name>_link`; the URDF places only that link on the
head, and the launch file sets `enable_publish_extrinsic:=true` (the vendor default publishes nothing on `/tf`). The
camera needs the vendor's udev rule once per machine (see `cameras.launch.py`). Its mount pose, intrinsics and depth
scale are calibrated as in [docs/setup.md](../setup.md).

## Consequences

- MoveIt plans against an octomap built from the head depth cloud, and object poses use depth (ADR-0005).
- cuVSLAM and nvblox have a camera to run on; adopting them is a separate decision.
- The lidar remains Nav2's obstacle source.
