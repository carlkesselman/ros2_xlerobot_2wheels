"""MoveIt 2 for the two arms: move_group and, with rviz:=true, RViz with the MotionPlanning display.

Run it next to the bringup, it uses the controllers and joint states of that one:

  ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true torque_off:=false      # the robot
  ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true use_sim:=true          # the Isaac twin
  ros2 launch xlerobot_bringup bringup.launch.py with_arms:=true use_mock_hardware:=true torque_off:=false
  ros2 launch xlerobot_moveit_config move_group.launch.py [rviz:=true] [use_sim_time:=true]

The robot description here is only used for kinematics and collision, it does not have to match
torque_off or the hardware, but it must have the arms.

RViz's renderer (OGRE, over GLX) needs an X11 window, but on a Wayland session Qt defaults to
its native Wayland backend, which crashes RViz on startup ("Unable to create the rendering
window after 100 tries"). rviz_qt_platform:=xcb (the default) forces Qt onto XWayland instead,
harmless on a plain X11 session too. Set it to the empty string to leave QT_QPA_PLATFORM alone.

Same reasoning for rviz_qt_scale_1x (default true): forces Qt's own HiDPI autoscaling off
(QT_AUTO_SCREEN_SCALE_FACTOR=0, QT_SCALE_FACTOR=1, QT_ENABLE_HIGHDPI_SCALING=0), so it renders
one RViz pixel per screen pixel instead of guessing a scale from the monitor. This is
independent of rviz_qt_platform: under XWayland the compositor can still upscale the whole
X11 surface to match a fractional display scale set in the desktop's own display settings
(that's compositor output scaling, not a Qt setting, and no env var here controls it) -
lower the display's scale there if RViz still looks oversized with this on.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    description_share = get_package_share_directory("xlerobot_description")
    config_share = get_package_share_directory("xlerobot_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("xlerobot", package_name="xlerobot_moveit_config")
        .robot_description(
            file_path=os.path.join(description_share, "urdf", "xlerobot.urdf.xacro"),
            # no ros2_control block: MoveIt needs kinematics and collision only, and the base's
            # hardware choice (real, mock, sim) would otherwise have to be passed here too
            mappings={"with_arms": "true", "with_head": "true", "use_ros2_control": "false"},
        )
        .robot_description_semantic(
            file_path="config/xlerobot.srdf.xacro",
            mappings={"disabled_collisions": LaunchConfiguration("disabled_collisions").perform(context)},
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "use_sim_time": use_sim_time,
                    "octomap_frame": "odom",
                    "octomap_resolution": 0.05,
                    "max_range": 1.2,
                },
            ],
            condition=IfCondition(LaunchConfiguration("move_group")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", os.path.join(config_share, "config", "moveit.rviz")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {"use_sim_time": use_sim_time},
            ],
            # see the module docstring on rviz_qt_platform / rviz_qt_scale_1x
            additional_env=dict(
                ({"QT_QPA_PLATFORM": LaunchConfiguration("rviz_qt_platform")}
                 if LaunchConfiguration("rviz_qt_platform").perform(context) else {}),
                **({"QT_AUTO_SCREEN_SCALE_FACTOR": "0", "QT_SCALE_FACTOR": "1",
                    "QT_ENABLE_HIGHDPI_SCALING": "0"}
                   if LaunchConfiguration("rviz_qt_scale_1x").perform(context) == "true" else {}),
            ),
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        # false runs only RViz, for a machine that is not the one running move_group
        DeclareLaunchArgument("move_group", default_value="true"),
        DeclareLaunchArgument("rviz_qt_platform", default_value="xcb"),
        DeclareLaunchArgument("rviz_qt_scale_1x", default_value="true"),
        # false starts without the self-collision matrix, only for generating it
        DeclareLaunchArgument("disabled_collisions", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
