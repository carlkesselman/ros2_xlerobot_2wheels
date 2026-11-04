# Isaac Sim simulation support

An Isaac Sim scene of the base, the head and the two SO-101 arms, wired to the real `ros2_control`
stack over ROS 2 topics, for development and testing without hardware
attached.

Isaac Sim needs an NVIDIA RTX GPU on Linux or Windows. On the Mac (and anywhere without one) the
MuJoCo twin below does the same job, plus a simulated LD19 for Nav2; see
[MuJoCo twin](#mujoco-twin-mac) at the end.

## Scope

Supports: Nav2, `diff_drive_controller` kinematics and odometry
integration, the safety supervisor, and launch/configuration testing
against simulated physical response.

Does not support: reproducing R1, the STS3215 encoder wrap issue
(ARCHITECTURE §4.1). That is real-servo register behavior; a physics
simulation has no reason to reproduce it. Isaac Sim wraps continuous joint
position on its own convention (see `sum_wrapped_joint_states` below),
which is a different problem from R1 and does not substitute for hardware
validation of it.

The arms are in the scene when the import URDF has them (`with_arms:=true`, below). They start in
`arms.rest_pose` of `robot_dimensions.yaml`, the folded pose, and take the same arm controllers as the
robot (`left_arm_controller`, `left_gripper_controller`, ...), so MoveIt plans and executes against the
twin exactly as against the robot, see "MoveIt against the twin".

## Requirements

- Isaac Sim 6.1 or later (pip package, `isaacsim[all,extscache]`), own
  Python 3.12 virtual environment.
- ROS 2 Jazzy on the host.
- [`joint_state_topic_hardware_interface`](https://github.com/ros-controls/topic_based_hardware_interfaces)
  (see `xlerobot.repos`).

## Installation

```bash
python3 -m venv ~/venvs/isaacsim
source ~/venvs/isaacsim/bin/activate
pip install "isaacsim[all,extscache]" --extra-index-url https://pypi.nvidia.com
```

Isaac Sim's ROS 2 bridge extension requires `ROS_DISTRO`,
`RMW_IMPLEMENTATION`, and `LD_LIBRARY_PATH` to be set in the environment
before the process starts; setting them after launch does not take effect.

```bash
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/venvs/isaacsim/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib
```

`OMNI_KIT_ACCEPT_EULA=YES` is required for non-interactive use and is set
in the venv's `activate` script.

Isaac Sim on Ubuntu 24.04 uses its own bundled ROS 2 Jazzy libraries by
default when nothing else is sourced. This matches the host distro, so no
library-path changes or workspace rebuilds against Isaac Sim's Python are
needed. Do not source `/opt/ros/jazzy/setup.bash` in the same shell that
launches Isaac Sim; let it use its own internal libraries. DDS handles
transport regardless of which Jazzy build either side links against.

## Integration approach

There is no first-party `ros2_control` hardware interface for Isaac Sim.
The supported pattern is
[`topic_based_hardware_interfaces`](https://github.com/ros-controls/topic_based_hardware_interfaces)
(Apache-2.0, official `ros-controls` package, Jazzy supported), specifically
`joint_state_topic_hardware_interface/JointStateTopicSystem` — a
`hardware_interface::SystemInterface` that reads and writes plain
`sensor_msgs/JointState` on two topics, matching Isaac Sim's ROS 2 bridge
OmniGraph nodes.

`cm_topic_hardware_component` (same upstream repo) is a different tool for
replaying `controller_manager`'s own introspection topic from a bag file
and is not used here.

With this hardware interface, the same `controller_manager`,
`diff_drive_controller`, and Nav2 configuration run unchanged; only the
hardware plugin and the wheel signs differ from the physical-robot
configuration. `controllers.yaml` flips the left wheel for the cart's mirrored
motor; the twin has none, so `bringup.launch.py` loads
`xlerobot_bringup/config/controllers_sim.yaml` after it under `use_sim:=true`,
setting both `*_wheel_radius_multiplier` back to 1.0. Without it a forward
`cmd_vel` spins the twin in place while its odometry reports a straight line.

The cart's chassis mass is on `body_link`, which has the collision box, not on
`base_link`: the importer keeps each link a separate rigid body
(`merge_fixed_joints=False`), and PhysX gives a body that has a collider but no
mass one from the collider's volume at water density, about 98 kg for the cart
box. `base_link` keeps a token 0.1 kg so the articulation root isn't given
PhysX's default 1 kg. The two wrist camera links still have no `<inertial>`;
PhysX makes their mass from their small camera meshes, a few tens of grams.

Selected by `use_sim:=true` in `xlerobot.urdf.xacro`, which loads
`xlerobot.ros2_control_sim.xacro` — two hardware components, one per
command-interface type (`xlerobot_sim_wheels` for the velocity-commanded
wheels, `xlerobot_sim_position` for the position-commanded head/arms), not
one combined component and not the real robot's bus split (ARCHITECTURE
§1.1, which is about physical wiring and does not apply here).

The split exists because `joint_state_topic_hardware_interface`'s write()
only appends to a `sensor_msgs/JointState` command message's
position/velocity/effort arrays for the command interfaces a joint
actually declares, while `name` always lists every joint in the component.
A component mixing velocity- and position-commanded joints produces a
message whose arrays are shorter than `name` and misaligned; Isaac Sim's
`ROS2SubscribeJointState` correctly refuses that ("size of position,
velocity and effort arrays must match the number of actuators") instead of
guessing which value belongs to which joint. Keeping each component
homogeneous keeps its arrays always either full-length or empty.

Topic names: commands are split, `/isaac_wheel_commands` and
`/isaac_position_commands`; state stays shared on `/isaac_joint_states`,
since `JointStateTopicSystem::read()` looks up each incoming name against
its own joints and silently skips ones it doesn't own, so one combined
state message covering every joint is safe to share. These must match
across the `ros2_control` xacro and the Isaac Sim Action Graph.

`sum_wrapped_joint_states` is enabled, since Isaac Sim reports continuous
joint position wrapped to a ±2π range. This is unrelated to the STS3215's
0–4095 step wrap (R1); the two use different unwrapping conventions and
are handled independently.

## Scene construction

`sim/build_isaac_scene.py` imports the robot description and builds a ROS 2
bridge Action Graph:

- `ROS2SubscribeJointState` (`/isaac_wheel_commands`) → its own `IsaacArticulationController` (wheels)
- `ROS2SubscribeJointState` (`/isaac_position_commands`) → its own `IsaacArticulationController` (head/arms)
- Both `IsaacArticulationController` instances drive the same articulation, on disjoint joint sets
- `ROS2PublishJointState` (`/isaac_joint_states`) ← articulation state, shared across both
- `ROS2PublishClock` (`/clock`)

Each joint's PhysX drive gets an explicit stiffness/damping/max_force set
by `configure_joint_drives()` after import: the URDF importer creates a
drive with zero gain for any joint whose source URDF has no
`<dynamics damping="..."/>`, and a zero-gain drive applies no corrective
torque at all regardless of the commanded position or velocity — a command
can reach `/isaac_wheel_commands` with the exact right value and the wheel
still won't turn. The gain values are picked to make closed-loop control
visibly work, not tuned against the real servo's torque curve.

The articulation controller and joint-state publisher target
`/World/xlerobot/Geometry/base_footprint/base_link`, where the URDF
importer applies `PhysicsArticulationRootAPI` — not the top-level
reference prim, which does not carry the articulation schema. The
importer's output also gates its physics representation behind a
`Physics` USD variant set (`physics`/`physx`/`mujoco`/`none`); the
`physics` variant is selected explicitly, since it is not selected by
default when the asset is referenced. The script asserts
`prim.HasAPI(UsdPhysics.ArticulationRootAPI)` after setup and fails
loudly if the articulation root is not present.

Ground-truth odometry (published directly from the chassis pose, as
opposed to being derived from joint states) is intentionally not wired
into the scene: it would duplicate what `diff_drive_controller` is meant
to produce, and mixing the two would make it unclear which source is
authoritative.

## Layout

| Path | Purpose |
|---|---|
| `sim/xlerobot_import.urdf` | Generated. Plain URDF for the Isaac Sim importer, rendered from the xacro with `use_ros2_control:=false` and `with_arms:=true`. Gitignored. |
| `sim/build_isaac_scene.py` | Imports the URDF, drives the wheels by velocity and every other revolute joint by position (from the URDF), sets joint drive gains and the arms' start pose, builds the ROS 2 bridge graph, saves the stage. `--test-frames N` presses Play and steps N frames first, to catch graph errors before saving. `--screenshot PATH` captures a viewport image after building, for checking mesh placement without opening the GUI. |
| `sim/output/` | Generated. `xlerobot_twin.usd` is the scene to open in Isaac Sim. Gitignored. |
| `sim/launch_gui.py` | Opens a given USD stage in the interactive Isaac Sim GUI (not headless), waits for a human to press Play. Defaults to `sim/output/xlerobot_twin.usd`. |
| `sim/run_headless.py` | The headless equivalent: opens the stage, presses Play itself, and stays running so a `ros2_control` bringup elsewhere can drive it. No display needed - the form meant for CI or the Jetson. |

Regenerate the import URDF after xacro changes:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
xacro src/xlerobot_description/urdf/xlerobot.urdf.xacro \
  use_ros2_control:=false with_arms:=true sensor:=none \
  > sim/xlerobot_import.urdf
```

Then rebuild the scene:

```bash
source ~/venvs/isaacsim/bin/activate
python sim/build_isaac_scene.py
```

Topic names are duplicated between `xlerobot.ros2_control_sim.xacro` and
`build_isaac_scene.py` rather than derived from one source; keep them in
sync manually when either changes.

## Viewing a scene interactively

```bash
source ~/venvs/isaacsim/bin/activate
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/venvs/isaacsim/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib
python sim/launch_gui.py [path/to/stage.usd]
```

Requires a display. Opens the given stage, or `sim/output/xlerobot_twin.usd`
by default, and stays open until the window is closed.

`base_2wheels.xacro`'s `base_link` visual uses `raskogbody.stl`, from
upstream's own Isaac Sim asset bundle (`simulation/Isaac_sim/xlerobot.zip`
in the XLeRobot repo, Apache-2.0) — the same IKEA RÅSKOG cart this robot is
built on. Its placement offset is derived from upstream's own reference
URDF numbers (see the comment above that visual block in the xacro for the
derivation), not measured against this specific robot, so it should be
visually checked against the real proportions rather than trusted outright.
Deliberately excludes upstream's `raskogwheel1/2.stl` sub-meshes: those
depict the stock cart's original four casters at their stock 0.45m track,
narrower than this robot's actual 0.55m custom drive-wheel track, and would
draw a second, wrong-width pair of wheels overlapping the real ones. The
drive wheels and caster are still primitive geometry (cylinder/sphere),
sized to the real measured dimensions in `robot_dimensions.yaml` but not
using a mesh — `hardware/ongoing_upgrades/035wheel.stl` in the XLeRobot
repo is the closest candidate and has not been wired in; unlike the body
mesh it has no existing reference transform to derive a placement from.

## Running headless

```bash
source ~/venvs/isaacsim/bin/activate
python sim/run_headless.py [path/to/stage.usd]
```

No display needed. Opens the stage, presses Play, and stays running so a
`ros2_control` bringup (`use_sim:=true`) elsewhere has something live to
talk to. Stop with Ctrl+C or plain `kill <pid>`, not `-9` — see
`launch_gui.py`'s docstring for why a flag checked in the main loop is used
for shutdown instead of a signal handler that raises.

Suspending the machine mid-session (lid close, idle timeout) risks a hard
hang requiring a manual power-cycle: this driver's `nvidia_uvm` kernel
module has a NULL-pointer bug in its GPU-unregister path
(`uvm_hmm_unregister_gpu`) that a CUDA process exiting can trigger — clean
exits included, not just kills — leaving the kernel tainted
("Fixing recursive fault but reboot is needed!"). Suspending on top of
that tainted state is what turns a recoverable oops into an unrecoverable
one. `tools/isaac_sim_sleep_guard.sh disable` masks the systemd sleep
targets for the duration of a session; `... enable` restores them
afterward.

## Known limitations

- Joint drive gains (`configure_joint_drives()` in `build_isaac_scene.py`)
  are picked to make closed-loop control visibly work, not tuned against
  the real servo's torque curve. Revisit once that matters.
- `ROS2PublishJointState.inputs:targetPrim` is a deprecated input on
  current Isaac Sim releases in favor of a separate `IsaacReadJointState`
  node. It remains functional; migrating is not required but is preferred
  for new work on this graph.
- The drive wheels and caster are still primitive geometry, not a mesh —
  see the note above `## Running headless`. `base_link`'s cart body mesh
  placement is derived from upstream's reference numbers, not measured
  against this specific robot.
- The arm drive gains (`ARM_GAINS`) are a first guess: the simulated arms hold the rest pose with
  about 0.02 rad of gravity sag, and nothing is tuned to the real servos.
- **A real trajectory move (not just holding a pose) reliably aborts with `PATH_TOLERANCE_VIOLATED`**
  when the Isaac Sim GUI is open and its viewport is actually rendering: the joint_trajectory_controller
  in `xlerobot_bringup` paces itself against wall-clock time, and physics stepping under GUI render load
  does not track 1:1 with it, so the arm falls too far behind where the trajectory expects it. Planning,
  IK/FK and the collision-refusal check are unaffected - only closed-loop trajectory execution against the
  live twin. Tried `use_sim_time:=true` on `bringup.launch.py` (2026-09-27, wired but off by default,
  opt in per run) expecting the controllers to pace off the twin's own `/clock` instead: made it worse,
  not better - `controller_manager`'s own controller activation started timing out before anything could
  run, meaning the twin's `/clock` isn't steady/fast enough for that handshake with the GUI open. Not
  reproduced running the twin headless in earlier sessions. Root cause open; the real robot doesn't have
  this problem (real time is real time), so arm controller testing moved there instead of chasing this
  further in sim.

## MoveIt against the twin

```bash
# terminal 1, the twin (Isaac's own environment, see above)
python sim/run_headless.py
# terminal 2
ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true use_sim:=true
ros2 launch xlerobot_moveit_config move_group.launch.py rviz:=true
python3 tools/moveit_smoke_test.py      # plans and executes on both arms, checks IK
```

Use one `ROS_DOMAIN_ID` for all of them. `tools/moveit_smoke_test.py` also runs against the mock
hardware and the robot, it only needs the controllers and `move_group`.

## Omniverse MCP (optional)

[`serionist/omniverse-mcp`](https://github.com/serionist/omniverse-mcp)
provides interactive scene inspection and scripting against a running
Isaac Sim instance, for iterating on a scene once it exists (not for
constructing it from scratch — `sim/build_isaac_scene.py` is the
reproducible path for that).

Setup:

1. Clone and install in its own virtual environment, then register it as
   an MCP server (see the project's own README for the exact
   `.mcp.json` entry).
2. Add its `extension/` directory as an Isaac Sim extension search path,
   and enable + autoload "Omniverse MCP Bridge" under Window > Extensions
   in the Isaac Sim UI. This is a one-time step; the extension autoloads
   on subsequent launches.
3. A running MCP client needs to reload its server configuration to pick
   up a newly registered server.

## TODO

- Wire a real mesh for the drive wheels (`hardware/ongoing_upgrades/035wheel.stl`
  in the XLeRobot repo is the closest candidate) and the caster.
- Verify `base_link`'s cart body mesh placement against the real robot,
  not just upstream's reference numbers.
- Tune joint drive gains against the real servo's torque curve.
- Reference real meshes for the head (`head_pantilt.xacro` is still
  placeholder geometry).
- Extend the scene to include the arms once phase 3 starts.
- Migrate off the deprecated `ROS2PublishJointState.inputs:targetPrim`.

## MuJoCo twin (Mac)

The same idea on MuJoCo, which runs natively on Apple Silicon: `sim/mujoco/`. MuJoCo's own Python
bindings run the physics, and `ros2_control` talks to it over joint-state topics exactly as with
Isaac. `use_sim:=true sim_engine:=mujoco` selects it; the controllers, `controllers_sim.yaml` and
Nav2 are unchanged.

```bash
pixi run build
source install/setup.zsh && source tools/ros_env.sh       # Cyclone DDS, as on the robot
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST            # not ros_env.sh's SUBNET: Nav2 hangs at startup with it on the Mac
ros2 launch sim/mujoco/mujoco_twin.launch.py              # viewer:=true opens MuJoCo's window
ros2 launch xlerobot_navigation navigation.launch.py rviz:=true
python sim/mujoco/grasp_check.py                          # the gripper holding a cube, no ROS
```

| File | Purpose |
|---|---|
| `sim/mujoco/world.py` | The world: the xacro as a plain URDF, loaded by MuJoCo, plus a free base, actuators, frictionless casters and a room with crates, a table and a 4 cm cube. `python sim/mujoco/world.py` saves it as MJCF in `sim/output/mujoco/`. |
| `sim/mujoco/run_mujoco.py` | Runs it in real time: `/mujoco_wheel_commands` and `/mujoco_position_commands` in, `/mujoco_joint_states` out, `/ldlidar_node/scan` as the LD19's driver publishes it (455 bins, 10 Hz, `ldlidar_link`), `/mujoco/ground_truth` for checking odometry against. No `/clock`, nothing needs `use_sim_time`. |
| `sim/mujoco/mujoco_twin.launch.py` | The twin, bringup with arms and head, and the LD19's `laser_filters` chain. |
| `sim/mujoco/grasp_check.py` | Closes the left gripper on the cube, lets go of it, swings the arm and reports how far the cube slipped. |
| `sim/mujoco/skills_sim.yaml` | The skill server's settings for the twin, over `xlerobot_skills/config/skills.yaml`: the model's claw length. |
| `sim/mujoco/tasks/table_to_crate.json` | A plan for the executive: cube from the table into the top basket, drive to `crate_2`, cube onto it. |
| `sim/mujoco/run_task.py` | Submits a plan to the executive and prints its events to the end; exit code 0 when it succeeds. |

The plugin is PickNik's `topic_based_ros2_control` (RoboStack has no macOS build of
`mujoco_ros2_control` or of `joint_state_topic_hardware_interface`); see the comment in
`xlerobot.ros2_control_sim.xacro` for its two differences.

What MuJoCo needed beyond the URDF, all in `world.py`:

- The drive wheels collide as spheres of the wheel radius. As the URDF's cylinders they touched
  the floor at the rim edges, hopped off it a third of the time turning in place, and the cart
  turned 28% less than its wheels.
- Wheel friction 2.0 and jaw contacts stiffer than MuJoCo's default (`solref` 0.005): MuJoCo's soft
  contacts otherwise let a loaded rolling wheel creep (15-28%) and a squeezed cube creep out of the
  jaws. Neither number is a measurement of the real tyres or jaws.
- The casters are frictionless with contact priority, or the floor's friction wins the pair and
  they drag.
- `body_link`'s collision box starts at `CART_FLOOR_CLEARANCE` (0.08 m, TODO measure) instead of
  the floor, or the cart stands on it with the wheels in the air.

Checked on the M5 Pro Mac mini (2026-11-03): odometry against ground truth 0.2 cm after a 0.9 m
reverse and a half turn; every one of the 455 scan points within 0.1 mm of the wall or box it hit,
through TF (so the upside-down mount is right); Nav2 from the start to the table around the crates
in 19 s, 4.5 cm from the goal; the gripper holding the cube through the arm's swing (6 mm seated,
0.4 mm more in 5 s). The physics runs about 100x faster than real time.

MuJoCo's viewer shows geom groups 0-2 by default and the robot is in group 3 (so the lidar's rays
skip it); `run_mujoco.py` turns group 3 on, and the `3` key toggles it.

### Pick and place

`skills:=true` adds the arm stack: MoveIt's `move_group`, the skill server and the plan executive.
The twin then answers `/perception/detect` from its own state instead of a camera (what the prompt
names, within 1.5 m, in `base_footprint`: "a cube", "a box" for the crates, "a table") and publishes
the operator presence heartbeat the skill server waits for. Nav2 still goes on top.

```bash
ros2 launch sim/mujoco/mujoco_twin.launch.py viewer:=true skills:=true
ros2 launch xlerobot_navigation navigation.launch.py rviz:=true          # second terminal
python3 sim/mujoco/run_task.py sim/mujoco/tasks/table_to_crate.json     # third
```

The skills it uses are `grasp` and `place` (`xlerobot_skills`), and `navigate_to`'s
`final_straight_m`. What shaped them:

- **Reach.** The arms are mounted at the back of the top basket (0.24 m behind the axle) and reach
  only 6-10 cm past the cart's front at table height, and the table top (0.72 m) is lower than the
  basket rim (0.78 m), so the cart cannot nose under it. To the side the left arm reaches 0.42 m
  with the claw straight down at table height, 0.40 m at the crate's top, and 0.48 / 0.45 m with the
  claw leaning 25 deg outward. So the cart parks alongside what it picks from or places on, and
  `grasp`/`place` lean the claw out (up to `grasp_max_lean_deg`) only when straight down is out of
  reach. Nav2's goal tolerance is 5 cm for this.
- **Parking alongside a solid box.** Turning in place swings the cart's rear corners through 0.5 m,
  so at a parking spot a few centimetres from a crate Nav2's controller (rightly) refuses the final
  turn. `final_straight_m` parks from a staging pose that far back along the goal heading: turn
  there, then Nav2's `drive_on_heading` straight in. The table did not need it (the lidar sees only
  its legs), the crate did.
- **The top basket** is a collision model of its own now (`body_link`: a solid box to the basket's
  floor, four walls to the rim, `top_basket_depth` 0.09 m, measured), so MoveIt lets the arms
  reach into it and the twin's cube lands on its floor.
- **Grasping** comes down along the claw onto the object with the jaws square to it (the wrist roll),
  closes, and judges holding by where the gripper stopped. Before coming down it corrects for the
  arm's sag under gravity, read from the joint encoders (7 mm short at first, 1 mm after).

Checked on the M5 Pro Mac mini (2026-11-03), headless, the whole plan in 130 s: Nav2 to the table,
grasp (claw 17 deg from vertical), drop into the basket, arm to rest, Nav2 to `crate_2` with 1 m
straight in, grasp from the basket (1 deg), place on the crate (25 deg); the cube ended on the crate's
top (its centre 2 cm above it) 5 cm in from the edge. The crate parking ended 6 cm farther out than
asked (a 2 deg heading error over the straight 1 m), which used the whole 25 deg lean: the margin
there is thin. Not in the twin yet: cameras (detection is the world's truth), the head, two-arm
coordination, and MoveIt knowing about the table or the held cube.
