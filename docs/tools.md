# Tools

Every script in `tools/`, grouped by what it's for. Each one also has its own usage in its
docstring (`--help` where it takes args) - this is the index, not a replacement for reading
the script itself.

## Python environment

The tools are plain Python scripts on top of ROS 2 Jazzy. ROS's own Python packages (`rclpy`, `cv_bridge`, the message
types), OpenCV, SciPy and, on a Jetson, TensorRT come from the system; the few packages Ubuntu does not ship (or ships too
old) go into a venv that also sees the system packages. Ubuntu 24.04 refuses `pip install` into the system Python, and
forcing it can replace the NumPy that ROS was built against.

On the robot (and on any machine that runs the TUI, the MCP server or the calibration tools):

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages ~/venvs/xlerobot
source ~/venvs/xlerobot/bin/activate
pip install -r tools/requirements.txt
```

Then run tools with that venv active, for example:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash && source ~/venvs/xlerobot/bin/activate
python3 tools/xlerobot_tui.py
```

The MCP server's `run.sh` and the object perception launch file activate the venv themselves; they look for it in
`$XLEROBOT_VENV` and fall back to `~/venvs/xlerobot`, so set that variable if you keep it elsewhere.

Two tools need a venv of their own because their packages want NumPy 2, which breaks `cv_bridge`:

| Venv | Requirements | For |
|---|---|---|
| `~/venvs/xlerobot-rerun` (no `--system-site-packages`) | `tools/requirements-rerun.txt` | `rerun_bridge.py`; keep `rerun-sdk` at the viewer's version |
| `~/venvs/xlerobot-models` (no `--system-site-packages`) | `tools/perception/requirements.txt` | exporting and benchmarking the perception models (`tools/perception/export_models.py`, `segment_eval.py`), on a machine with an NVIDIA GPU |

Downloaded model weights, exported ONNX files and TensorRT engines live in `~/.cache/xlerobot`.

## Setup

| Tool | What it does |
|---|---|
| `xlerobot_setup_usb_modules.py` | Pins `/dev/xle-bus1`/`xle-bus2` and camera/microphone device names via udev rules, matched by USB physical port and serial - run once per machine. See the main [README's bus layout section](../README.md#bus-layout). |

## Running the robot

| Tool | What it does |
|---|---|
| `xlerobot_tui.py` | Console UI on the robot: starts and stops the stack, shows every component's state, topic rates, CPU, memory and GPU, toggles the presence guard and e-stop, records and plays back motions. |
| `motion_recorder.py` | The TUI's motion record and playback (hand-guided with torque off, or driven by teleop); playback slows to safe joint speeds and sends the wheels through twist_mux below the joystick. |

## Process management

| Tool | What it does |
|---|---|
| `kill_ros.py` | Finds and kills every process belonging to this project's ROS 2 stack, on whichever machine it's run on. `--yes` to skip the confirmation, `--dry-run` to just list. |
| `isaac_sim_sleep_guard.sh` | Masks systemd's sleep targets for the duration of an Isaac Sim session - suspending while the Isaac Sim process is up can hard-hang the machine, see the script's own header for why. `disable` before starting Isaac Sim, `enable` after. |

## Servo bus and calibration

The order to run the calibrations in, and where each result goes, is in [setup.md](setup.md).

| Tool | What it does |
|---|---|
| `calibrate_arm.py` | `status` reads positions/temperature/voltage for an arm or the head, read-only. `calibrate` runs lerobot's own procedure (middle pose, then a sweep) and writes the result in lerobot's calibration file format. Its `Bus` class (raw Feetech protocol 0 over serial) is also what several raw-protocol diagnostics below import directly. |
| `fake_sts_bus.py` | Fakes two Feetech servo buses on ptys, so `sts_hardware_interface` (the real driver) can run against them with no hardware attached - for mock-hardware bringup. Logs every torque/goal write with the resulting position. |
| `estop.py` | Emergency stop (and `--release`) across every bus at once, calling `sts_hardware_interface`'s per-bus `/emergency_stop_<component>` service on each. |
| `calibrate_wheel_odometry.py` | Drives a measured calibration motion (`straight-line` or `rotate-in-place`) and suggests the `wheel_radius`/`wheel_separation_multiplier` correction to apply by hand in `controllers.yaml`. See [ARCHITECTURE §6](../ARCHITECTURE.md#6-calibration). |

## Joint and controller diagnostics

| Tool | What it does |
|---|---|
| `single_joint_test.py` | Moves exactly one joint of a controller by a delta, holding every other joint of that controller at its current position. |
| `characterize_joint.py` | Sends one move at a known speed, logs `/joint_states` through it and a hold period after, and reports whether the joint is still converging (speed/timing problem) or settled at a persistent offset (steady-state/gravity/compliance problem). |
| `moveit_tracking_diag.py` | Plans a single-joint move through MoveIt (plan only), then sends that exact trajectory straight to the controller's `FollowJointTrajectory` action (bypassing `move_group`'s own executor) and logs desired/actual/error feedback at full rate. |
| `acc_test.py`, `matrix_test.py`, `repeat_test.py`, `speed600_test.py`, `stream_test.py`, `syncwrite_test.py` | Raw Feetech-protocol probes (bypass ROS entirely, via `calibrate_arm.py`'s `Bus` class) built to isolate why a streamed trajectory tracked a commanded position more slowly than a one-shot move to the same target. See [`SERVO_STREAMING_DIAGNOSTICS.md`](../tools/SERVO_STREAMING_DIAGNOSTICS.md) for what each one isolates and how to read the results. |

## MoveIt 2 and arm motion

| Tool | What it does |
|---|---|
| `moveit_smoke_test.py` | End-to-end MoveIt check against whatever's running (mock, the Isaac twin, or the real robot): plans and executes to the rest pose and a nearby pose, moves the gripper, checks IK against FK, and confirms an in-collision goal is refused. |
| `head_moveit_test.py` | Quick plan+execute check for the `head` MoveIt group. |
| `generate_moveit_collisions.py` | Generates the self-collision matrix (adjacent/default/always/never link pairs) the way the Setup Assistant does, without its GUI - queries `move_group`'s `/check_state_validity` over random poses. |
| `move_to_pose.py` | Plans and executes a Cartesian move of an arm's gripper to an `(x, y, z)` point (position-only IK), for targets that come from vision rather than a fixed joint configuration. |
| `goto_position.py`, `save_position.py`, `list_positions.py`, `playback_sequence.py` | Named-position workflow: save the current joint configuration of a MoveIt group under a name, list what's saved, move to one, or move through several in sequence. Shared storage in `position_store.py` (`src/xlerobot_bringup/config/saved_positions.yaml`, curated and tracked in git). |

## Navigation

| Tool | What it does |
|---|---|
| `nav_forward.py` | Sends Nav2 a goal a fixed distance straight ahead of the robot's current pose and watches it through, with a travel and timeout limit. |
| `goto_waypoint.py`, `save_waypoint.py`, `list_waypoints.py` | Named-waypoint workflow: save the robot's current AMCL-localized map pose under a name, list what's saved, or send Nav2 to one and watch it complete. Shared storage in `waypoint_store.py` (`src/xlerobot_bringup/config/saved_waypoints.yaml`). |
| `home_marker_calibrate.py`, `home_marker_localize.py`, `generate_home_marker.py`, `home_marker_common.py` | Tag-based AMCL cold-start seeding: print the generated ArUco marker and mount it somewhere fixed, run `home_marker_calibrate.py` once (well-localized) to save its map-frame pose, then `home_marker_localize.py` at every future cold start to publish `/initialpose` from wherever the robot can see the marker - no manual "2D Pose Estimate" needed. Reads the head camera's own live `camera_info` for its frame, so this works the same regardless of which `sensor:=` is in use. |

## Trajectory record and replay

| Tool | What it does |
|---|---|
| `record_trajectory.py` | Hand-guided (kinesthetic teaching) recording: cuts torque on the relevant bus(es), streams `/joint_states` while the arm is moved by hand, and saves the sampled trajectory on Ctrl-C. Supports single groups (`left_arm`, `right_arm`, `head`, a gripper) and the composite `both_arms` group for synchronized two-arm recordings. |
| `replay_trajectory.py` | Replays a recorded trajectory as `FollowJointTrajectory` goal(s) - one goal per controller, sent concurrently for a composite group like `both_arms` so multi-arm motions stay in sync. No collision checking; it's resending exactly what was recorded. |

## Camera and perception

| Tool | What it does |
|---|---|
| `capture_image.py` | Grabs a single frame from a camera's `image_raw` topic and saves it as a PNG under `captures/` (gitignored). |
| `calibrate_camera_charuco.py` | Live ChArUco calibration against a camera's real-time feed - move the board in front of the camera while it collects samples. Can watch a camera over the network from another machine, since it's just a ROS 2 subscription. |
| `calibrate_arm_camera.py` | Calibrates an arm against a fixed ChArUco board with the head camera as the reference: fits the arm's joint zero offsets, its wrist camera's mount and the arms' frame. See [setup.md](setup.md#8-arms-against-the-head-camera). |
| `calibrate_head_pose.py` | Fits the floor plane in the head depth cloud and prints the head camera mount height, pitch and roll that make the model agree. |
| `head_record.py` | Saves head camera frames (with `--depth` also the matching depth frames) while you move a board, for a lens calibration or a depth check. |
| `depth_scale.py` | The head depth camera's scale against the 7x5 board, per distance, from a `head_record.py --depth` folder. See [setup.md](setup.md#9-head-depth-scale). |
| `side_cam.py` | A plain USB webcam beside the robot as an independent measurement: locked exposure capture and ChArUco lens calibration (`calibrate --from` also fits any recorded folder, e.g. the head camera's). |
| `generate_charuco.py` | Writes a ChArUco board PNG to print at actual size. |
| `calibrate_from_bag.py` | Same calibration math as above, but reading images from a recorded rosbag instead of a live feed, so the dataset is fixed once and the math can be iterated on without re-recording. |

## Object perception

| Tool | What it does |
|---|---|
| `generate_placement_sheet.py` | A printable sheet (PDF at actual size) with markers around the edge and cube and brick footprints at known positions and yaws: ground truth for object poses. |
| `cube_pose_eval.py` | Measures the head camera's cube pose on the placement sheet with a colour threshold (no learned model), and saves every frame for comparing models later. |
| `segment_eval.py` | Scores detectors and mask models (OWLv2, OWL-ViT, Grounding DINO, YOLO-World; SAM 2, MobileSAM, EfficientViT-SAM, also on TensorRT) on saved frames through the same pose fit. Needs `perception/requirements.txt`. |
| `perception_check.py` | Calls the object pose service while the head camera sees the placement sheet and prints the error to the nearest footprint. |
| `wrist_occluder.py` | The claw's mask in a wrist camera's image (the claw stays at the same pixels while the scene moves), for the wrist camera pose fit. |
| `perception/export_models.py`, `perception/build_engines.sh` | Export the models to ONNX (with torch) and build TensorRT engines on the robot, see [the package README](../src/xlerobot_object_perception/README.md). |

## Spatial memory

| Tool | What it does |
|---|---|
| `spatial_memory_math.py` | Pure math (no ROS node needed) for turning a `(pixel, camera pose)` observation into a 3D point estimate - plane intersection for a single observation, triangulation for two or more from different viewpoints. Used by the MCP server's `remember_object`. |
| `spatial_memory_store.py` | Load/save for the agent's own remembered-object cache (`captures/spatial_memory.yaml`, gitignored - working memory, not curated config). |

## Visualization

| Tool | What it does |
|---|---|
| `rerun_bridge.py` | Streams the robot model, odometry, velocities, Nav2 goal/plan, lidar scan, the SLAM map, camera video and the head camera's depth (back-projected into a 3D point cloud on the viewer side) into [Rerun](https://rerun.io/) for timeline debugging - complements RViz/Foxglove rather than replacing them. Also has a Logs tab combining `/rosout` with the MCP server's own call log. |

## MCP server

`tools/mcp_server/` is its own thing - an MCP server exposing the robot to LLM agents (saved
positions/waypoints, spatial memory, camera capture with pose context, navigation and arm
motion tools). See [`tools/mcp_server/README.md`](../tools/mcp_server/README.md).
