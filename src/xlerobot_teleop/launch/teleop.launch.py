"""Joystick teleop with twist_mux in front of the base controller, plus the
safety supervisor. Nav2's cmd_vel_nav plugs into the same mux once phase 2
exists, so the joystick keeps override priority over autonomy from day one
instead of being bolted on after the fact.

The stack has two halves that can run on different machines:

  with_joy:=true   joy_node + teleop_twist_joy, wherever the gamepad is plugged in
  with_base:=true  twist_mux + safety supervisor, next to the controllers

Both default to true (everything on one machine). For the Mac operator station:

  Mac:    ros2 launch xlerobot_teleop teleop.launch.py with_base:=false
  elroy:  ros2 launch xlerobot_bringup bringup.launch.py teleop_joy:=false

joy_layout picks the button numbering: "linux" (joystick.yaml as confirmed on the robot),
"sdl" (macOS, where joy_node opens the pad as an SDL GameController, joystick_sdl.yaml) or
"ps" (the 8BitDo in PlayStation mode on Linux evdev, as on rosie: joystick_ps.yaml).
It defaults to sdl on macOS and linux everywhere else. split_stick:=true moves turning to the
right stick (joystick_split.yaml; its axis numbers are the linux layout's).

For the operator station with leader arms and the viewer, station.launch.py includes this one.
"""

import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _joy_nodes(context):
    pkg_share = get_package_share_directory("xlerobot_teleop")
    params = [os.path.join(pkg_share, "config", "joystick.yaml")]
    layout = LaunchConfiguration("joy_layout").perform(context)
    if layout in ("sdl", "ps"):
        params.append(os.path.join(pkg_share, "config", f"joystick_{layout}.yaml"))
    elif layout != "linux":
        raise RuntimeError(f"joy_layout must be linux, sdl or ps, not {layout!r}")
    if LaunchConfiguration("split_stick").perform(context).lower() == "true":
        params.append(os.path.join(pkg_share, "config", "joystick_split.yaml"))
    return [
        Node(package="joy", executable="joy_node"),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            parameters=params,
            remappings=[("cmd_vel", "cmd_vel_joy")],
        ),
        # d-pad -> head pan/tilt (head_controller, bringup with_head:=true). Sends only while a
        # direction is held, so it is harmless on a robot without the head.
        Node(
            package="xlerobot_teleop",
            executable="head_teleop",
            parameters=[{"joy_layout": layout}],
            output="screen",
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("xlerobot_teleop")
    twist_mux_yaml = os.path.join(pkg_share, "config", "twist_mux.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("with_joy", default_value="true"),
        DeclareLaunchArgument("with_base", default_value="true"),
        # true: turn with the right stick instead of the left (joystick_split.yaml)
        DeclareLaunchArgument("split_stick", default_value="false"),
        DeclareLaunchArgument(
            "joy_layout", default_value="sdl" if platform.system() == "Darwin" else "linux"),

        GroupAction(
            [OpaqueFunction(function=_joy_nodes)],
            condition=IfCondition(LaunchConfiguration("with_joy")),
        ),
        GroupAction(
            [
                Node(
                    package="twist_mux",
                    executable="twist_mux",
                    parameters=[twist_mux_yaml],
                    remappings=[("cmd_vel_out", "diff_drive_controller/cmd_vel")],
                ),
                Node(
                    package="xlerobot_teleop",
                    executable="safety_supervisor",
                    output="screen",
                ),
            ],
            condition=IfCondition(LaunchConfiguration("with_base")),
        ),
    ])
