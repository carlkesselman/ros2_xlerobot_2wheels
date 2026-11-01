"""Everything in one command: bringup (+ teleop, cameras, lidar), move_group, and navigation.

Replaces running bringup.launch.py, move_group.launch.py, and navigation.launch.py by hand in
three (or more) separate terminals/SSH sessions - the same arguments, just in one place. Nothing
new: this is purely IncludeLaunchDescription of the existing launch files, same as how
bringup.launch.py itself already folds in teleop/cameras/lidar.

  ros2 launch xlerobot_bringup full_stack.launch.py with_arms:=true with_head:=true \
      cameras:=true lidar:=true slam:=true

Add sensor:=gemini for an Orbbec Gemini 2 head camera, arm_teleop:=true for the dual-stick arm
teleop (needs with_arms:=true and torque_off:=false, the default). For tasks: object_perception:=true (the on-demand
object pose service, needs its TensorRT engines and the tools venv), skills:=true (the skill actions, needs move_group)
and executive:=true (runs plans through the skills, needs skills:=true); see ADR-0006.

move_group and navigation start 40 s after bringup, not simultaneously with it - lidar
and the controllers need to be up first (on the loaded Jetson the controllers take 25-40 s; Nav2's
costmaps block their configure step on the odom transform and the lifecycle manager then hangs). Starting navigation before the lidar is
actually publishing scans is a real failure mode, not a hypothetical one: Nav2's own lifecycle
bringup times out waiting for the map transform and tears the whole navigation launch back down
again, which then also looks like slam_toolbox "crashed" even though it exited cleanly as a side
effect - see project memory from 2026-10-02 if this happens again despite the delay (it shouldn't,
but the delay is a mitigation, not a guarantee against a sufficiently slow lidar enumeration).

slam:=true and localization:=true can't both be true (navigation.launch.py itself refuses that
combination - they'd both try to publish map -> odom). Neither is on by default, matching
navigation.launch.py's own defaults, so a plain full_stack.launch.py with no args still comes up
with the stock identity map -> odom publisher and no AMCL/SLAM running, same as running the three
launch files separately with no extra arguments.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    bringup_share = get_package_share_directory("xlerobot_bringup")
    moveit_share = get_package_share_directory("xlerobot_moveit_config")
    nav_share = get_package_share_directory("xlerobot_navigation")

    bringup_launch_file = os.path.join(bringup_share, "launch", "bringup.launch.py")
    move_group_launch_file = os.path.join(moveit_share, "launch", "move_group.launch.py")
    navigation_launch_file = os.path.join(nav_share, "launch", "navigation.launch.py")
    arm_teleop_launch_file = os.path.join(
        get_package_share_directory("xlerobot_teleop"), "launch", "arm_servo_teleop.launch.py")

    def task_launch(package, launch_file):
        try:
            return os.path.join(get_package_share_directory(package), "launch", launch_file)
        except Exception:                                     # noqa: BLE001  (package not built: argument unusable)
            return None
    task_launches = [(arg, task_launch(pkg, f)) for arg, pkg, f in (
        ("object_perception", "xlerobot_object_perception", "object_perception.launch.py"),
        ("skills", "xlerobot_skills", "skills.launch.py"),
        ("executive", "xlerobot_executive", "executive.launch.py"))]

    # Same names/defaults as the underlying launch files, so this is a drop-in replacement for
    # running them separately - nothing new to learn, just fewer terminals.
    bringup_args = {
        "bus1_port": LaunchConfiguration("bus1_port"),
        "bus2_port": LaunchConfiguration("bus2_port"),
        "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
        "use_sim": LaunchConfiguration("use_sim"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "teleop": LaunchConfiguration("teleop"),
        "with_head": LaunchConfiguration("with_head"),
        "with_arms": LaunchConfiguration("with_arms"),
        "torque_off": LaunchConfiguration("torque_off"),
        "cameras": LaunchConfiguration("cameras"),
        "sensor": LaunchConfiguration("sensor"),
        "lidar": LaunchConfiguration("lidar"),
        "record": LaunchConfiguration("record"),
        "foxglove": LaunchConfiguration("foxglove"),
    }
    move_group_args = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }
    navigation_args = {
        "rviz": LaunchConfiguration("rviz"),
        "slam": LaunchConfiguration("slam"),
        "localization": LaunchConfiguration("localization"),
        "map": LaunchConfiguration("map"),
    }

    return LaunchDescription([
        DeclareLaunchArgument("bus1_port", default_value="/dev/xle-bus1"),
        DeclareLaunchArgument("bus2_port", default_value="/dev/xle-bus2"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("use_sim", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("teleop", default_value="true"),
        DeclareLaunchArgument("with_head", default_value="false"),
        DeclareLaunchArgument("with_arms", default_value="false"),
        DeclareLaunchArgument("torque_off", default_value="false"),
        DeclareLaunchArgument("cameras", default_value="false"),
        DeclareLaunchArgument("sensor", default_value="usbcam"),
        DeclareLaunchArgument("arm_teleop", default_value="false"),
        DeclareLaunchArgument("lidar", default_value="false"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("object_perception", default_value="false"),
        DeclareLaunchArgument("skills", default_value="false"),
        DeclareLaunchArgument("executive", default_value="false"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(nav_share, "maps", "room1.yaml")),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_file),
            launch_arguments=bringup_args.items(),
        ),

        # Give controllers/cameras/lidar a few seconds head start - see the module docstring for
        # why starting navigation concurrently with (rather than after) the lidar is a real
        # failure mode, not just tidiness.
        TimerAction(
            period=40.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(move_group_launch_file),
                    launch_arguments=move_group_args.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_launch_file),
                    launch_arguments=navigation_args.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(arm_teleop_launch_file),
                    condition=IfCondition(LaunchConfiguration("arm_teleop")),
                ),
            ],
        ),
        # the task layer after move_group has had time to load
        TimerAction(
            period=55.0,
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(path), condition=IfCondition(LaunchConfiguration(arg)))
                     for arg, path in task_launches if path],
        ),
    ])
