"""Base bringup: robot_state_publisher + controller_manager + spawners.
The base is the ODrive (USB), so a base-only bringup needs no servo bus; bus1
(left arm + head) and bus2 (right arm) come in with with_head/with_arms. Point
bus1_port/bus2_port at your udev symlinks, not the raw ttyACM device, those
numbers swap on reboot. Until the ODrive hardware interface exists (step 2 of
docs/rosie-port-plan.md), run with use_mock_hardware:=true.

use_sim:=true switches to the Isaac Sim twin's hardware interface instead
of the real bus (or sts_hardware_interface's mock mode), see docs/sim-twin.md.
bus1_port/bus2_port are ignored in that case. The Isaac Sim scene needs to
be playing (Play pressed in the GUI, or app_utils.play() in a script) for
its side of the ROS 2 bridge to actually publish/subscribe anything.

teleop:=true also brings up xlerobot_teleop's joystick/twist_mux/safety
supervisor stack (xlerobot_teleop/launch/teleop.launch.py). On by default,
since every real bringup needs the safety watchdog and a human-override
path; pass teleop:=false for a states-only/no-joystick bringup (e.g. CI,
or a headless box with no controller plugged in). The watchdog engages
e_stop until this bringup's own joint_state_broadcaster is actually
publishing, so there's no real ordering issue bringing both up together
this way, unlike running teleop.launch.py standalone before bringup is up.

record:=true records every topic to bags/<timestamp> (gitignored) for
offline debugging, camera video as H.264 packets only (raw and the other
image transports are left out, they are far too big and would make the
cameras encode them). foxglove:=true starts foxglove_bridge on the default
port 8765 for live inspection from Foxglove Studio. Both off by default.

cameras:=true starts the USB cameras from config/cameras.yaml, see
cameras.launch.py. Off by default, a camera that is not connected is skipped.

lidar:=true starts the LD19 (/dev/ldlidar), see lidar.launch.py. Off by default.
"""

import datetime
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _find_port_holders(port_path):
    """PIDs (with cmdline) that currently have port_path open, or [] if free.

    sts_hardware_interface's SCS::syncReadBegin isn't safe against a second
    process opening the same serial port mid-configure - confirmed on this
    robot: it doesn't error, it segfaults controller_manager (heap
    corruption inside the vendor SDK's read buffer setup). That's exactly
    what happens if a previous bringup hasn't fully released the port yet
    (e.g. Ctrl-C then an immediate relaunch) - the old process is still
    dying when the new one opens the same device. Catch that here, before
    ros2_control_node ever starts, instead of after it crashes.
    """
    if not os.path.exists(port_path):
        return []
    real_path = os.path.realpath(port_path)
    holders = []
    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        pid = fd_dir.split("/")[2]
        try:
            for fd in os.listdir(fd_dir):
                if os.path.realpath(os.path.join(fd_dir, fd)) == real_path:
                    try:
                        with open(f"/proc/{pid}/cmdline", "rb") as f:
                            cmdline = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
                    except OSError:
                        cmdline = "?"
                    holders.append((pid, cmdline))
                    break
        except OSError:
            continue  # process exited while we were scanning it, not a real holder
    return holders


def _check_serial_ports_free(context, *args, **kwargs):
    use_sim = LaunchConfiguration("use_sim").perform(context) == "true"
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context) == "true"
    if use_sim or use_mock_hardware:
        return []  # no real serial port gets opened in either mode

    # The wheels are on the ODrive (USB, libusb), not on a servo bus, so a base-only
    # bringup opens no serial port at all.
    with_head = LaunchConfiguration("with_head").perform(context) == "true"
    with_arms = LaunchConfiguration("with_arms").perform(context) == "true"
    ports = []
    if with_head or with_arms:
        ports.append(LaunchConfiguration("bus1_port").perform(context))
    if with_arms:
        ports.append(LaunchConfiguration("bus2_port").perform(context))

    problems = []
    for port in ports:
        for pid, cmdline in _find_port_holders(port):
            problems.append(f"  {port} is already open by pid {pid}: {cmdline}")

    if problems:
        raise RuntimeError(
            "Refusing to start bringup: a real servo bus port is still held open by "
            "another process (a previous bringup that hasn't fully exited yet is the "
            "usual cause - wait a couple seconds after Ctrl-C before relaunching):\n"
            + "\n".join(problems)
        )
    return []


def generate_launch_description():
    description_share = get_package_share_directory("xlerobot_description")
    bringup_share = get_package_share_directory("xlerobot_bringup")
    teleop_share = get_package_share_directory("xlerobot_teleop")

    xacro_file = os.path.join(description_share, "urdf", "xlerobot.urdf.xacro")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers.yaml")
    controllers_sim_yaml = os.path.join(bringup_share, "config", "controllers_sim.yaml")
    teleop_launch_file = os.path.join(teleop_share, "launch", "teleop.launch.py")
    cameras_launch_file = os.path.join(bringup_share, "launch", "cameras.launch.py")
    lidar_launch_file = os.path.join(bringup_share, "launch", "lidar.launch.py")

    bus1_port = LaunchConfiguration("bus1_port")
    bus2_port = LaunchConfiguration("bus2_port")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim = LaunchConfiguration("use_sim")
    # Off by default even under use_sim:=true, opt in with use_sim_time:=true. Tried tying it
    # to use_sim (2026-09-27): the idea was that the Isaac twin publishes /clock, and without
    # use_sim_time the controllers pace themselves against the wall clock while the twin's
    # physics stepping - especially with its GUI actually rendering the viewport - does not
    # track 1:1 with it, so a joint_trajectory_controller path-tolerance check sees that as a
    # tracking error and aborts. Tested worse, not better: with use_sim_time:=true,
    # controller_manager's own controller activation started timing out
    # ("Switch controller timed out after 5 seconds!") before anything could even run, so the
    # twin's /clock is not steady/fast enough for that handshake, at least with the GUI open.
    # Root cause of the trajectory aborts is still open, see docs/sim-twin.md.
    use_sim_time = LaunchConfiguration("use_sim_time")
    teleop = LaunchConfiguration("teleop")
    with_head = LaunchConfiguration("with_head")
    with_arms = LaunchConfiguration("with_arms")
    torque_off = LaunchConfiguration("torque_off")
    arm_controllers = PythonExpression([
        "'", with_arms, "' == 'true' and ('", torque_off, "' == 'false' or '", use_sim, "' == 'true')",
    ])
    head_controller_condition = PythonExpression([
        "'", with_head, "' == 'true' and ('", torque_off, "' == 'false' or '", use_sim, "' == 'true')",
    ])
    # The twin's overrides of controllers.yaml (the wheel signs), loaded after it. Without
    # use_sim the second file is controllers.yaml again, which changes nothing.
    controllers_override_yaml = PythonExpression([
        "'", controllers_sim_yaml, "' if '", use_sim, "' == 'true' else '", controllers_yaml, "'",
    ])
    record = LaunchConfiguration("record")
    foxglove = LaunchConfiguration("foxglove")
    cameras = LaunchConfiguration("cameras")
    lidar = LaunchConfiguration("lidar")
    sensor = LaunchConfiguration("sensor")

    # Relative to wherever `ros2 launch` is run from - this repo's own convention (see
    # tools/*.py) is always the repo root, where bags/ is gitignored.
    bag_path = os.path.join("bags", datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " use_ros2_control:=true",
            " use_mock_hardware:=", use_mock_hardware,
            " use_sim:=", use_sim,
            " sim_engine:=", LaunchConfiguration("sim_engine"),
            " bus1_port:=", bus1_port,
            " bus2_port:=", bus2_port,
            " with_arms:=", with_arms,
            " torque_off:=", torque_off,
            " with_head:=", with_head,
            " sensor:=", sensor,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("bus1_port", default_value="/dev/xle-bus1"),
        DeclareLaunchArgument("bus2_port", default_value="/dev/xle-bus2"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("use_sim", default_value="false"),
        # isaac | mujoco, which twin use_sim:=true talks to (xlerobot.urdf.xacro)
        DeclareLaunchArgument("sim_engine", default_value="isaac"),
        # see the module-level use_sim_time comment above
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("teleop", default_value="true"),
        # off by default: bus1 (head pan/tilt) isn't wired on a base-only
        # robot, see xlerobot.urdf.xacro. Flip on once the head is plugged in.
        DeclareLaunchArgument("with_head", default_value="false"),
        # off by default for the same reason: both SO-101 arms are on bus1/bus2, and
        # with_arms:=true adds their joints to ros2_control (real buses, or use
        # use_mock_hardware:=true to see the model). Controllers come with phase 3.
        DeclareLaunchArgument("with_arms", default_value="false"),
        # Torqued and commandable by default now that both arms and the head are calibrated,
        # live-tested, and MoveIt2-integrated. Pass torque_off:=true for a states-only bringup.
        DeclareLaunchArgument("torque_off", default_value="false"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="false"),
        DeclareLaunchArgument("cameras", default_value="false"),
        # usbcam | gemini, see xlerobot.urdf.xacro - usbcam is a plain webcam, the common
        # case; gemini (Orbbec Gemini 2) adds depth/point cloud/IMU for builds that have
        # one. Also selects which driver cameras.launch.py brings up for "head" - see its
        # own docstring.
        DeclareLaunchArgument("sensor", default_value="usbcam"),
        DeclareLaunchArgument("lidar", default_value="false"),

        # Must run before the Node actions below: OpaqueFunction is resolved
        # synchronously at this point in the list, so a raised error here
        # aborts the launch before ros2_control_node ever spawns and touches
        # the port. See _check_serial_ports_free's docstring.
        OpaqueFunction(function=_check_serial_ports_free),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}, controllers_yaml,
                        controllers_override_yaml],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller"],
        ),
        # Arm controllers need command interfaces on the arm joints: not in the read-only
        # bringup (torque_off:=true), always there in the sim. Each hardware component
        # brings the joints up holding where they are, so these start holding too.
        *[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[name],
                condition=IfCondition(arm_controllers),
            )
            for name in ("left_arm_controller", "right_arm_controller",
                         "left_gripper_controller", "right_gripper_controller")
        ],
        # Same condition shape as the arm controllers above, gated on with_head instead.
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["head_controller"],
            condition=IfCondition(head_controller_condition),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch_file),
            condition=IfCondition(teleop),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cameras_launch_file),
            launch_arguments={"sensor": sensor}.items(),
            condition=IfCondition(cameras),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_file),
            condition=IfCondition(lidar),
        ),

        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-a", "-o", bag_path,
                 "--exclude-regex", r".*/image_raw(/compressed|/compressedDepth|/theora|/zstd)?"],
            condition=IfCondition(record),
            output="screen",
        ),
        Node(
            package="foxglove_bridge", executable="foxglove_bridge",
            condition=IfCondition(foxglove),
        ),
    ])
