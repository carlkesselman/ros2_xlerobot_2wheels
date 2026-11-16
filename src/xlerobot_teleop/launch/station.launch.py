"""The operator station: gamepad, leader arms and the station viewer, on one machine (docs/operator-station.md).

On rosie (the Pi 5), through tools/station.sh:

    tools/station.sh                        everything, viewer full screen on rosie's monitor
    tools/station.sh leader_arms:=false     gamepad and viewer only

The same launch runs on the Mac (pixi run bash -c 'source install/setup.bash && ros2 launch xlerobot_teleop
station.launch.py'), with the viewer in a window.

What it starts:
  teleop.launch.py with_base:=false   joy_node, teleop_twist_joy (base, RB held), head_teleop (d-pad)
  leader_arms                         the SO-101 leaders -> the arm controllers; engage_button toggles it
  station_viewer                      cameras + leader and recorder status; recorder keys and buttons

The robot side is bringup on elroy (teleop_joy:=false with_arms:=true with_head:=true cameras:=true); the recorder
runs on the Mac (tools/lerobot.sh record) and takes its keep / redo / stop from the station.
"""

import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Leader-arm engage button per joy_layout: the top face button on every layout (Y on an Xbox-style pad; in PlayStation
# mode triangle, which is X on the 8BitDo's Nintendo-style letters).
# station_viewer prints its name and has the recorder buttons (RECORDER_BUTTONS).
ENGAGE_BUTTON = {"sdl": 3, "linux": 4, "ps": 2}


def _station(context):
    layout = LaunchConfiguration("joy_layout").perform(context)
    if layout not in ENGAGE_BUTTON:
        raise RuntimeError(f"joy_layout must be one of {sorted(ENGAGE_BUTTON)}, not {layout!r}")
    engage = ENGAGE_BUTTON[layout]
    truth = lambda name: LaunchConfiguration(name).perform(context).lower() == "true"  # noqa: E731
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("xlerobot_teleop"), "launch", "teleop.launch.py")),
            launch_arguments={"with_base": "false", "joy_layout": layout,
                              "split_stick": LaunchConfiguration("split_stick")}.items(),
        ),
    ]
    if truth("leader_arms"):
        actions.append(Node(
            package="xlerobot_teleop", executable="leader_arms", output="screen",
            parameters=[{"engage_button": engage}],
        ))
    if truth("viewer"):
        actions.append(Node(
            package="xlerobot_teleop", executable="station_viewer", output="screen",
            parameters=[{
                "joy_layout": layout,
                "fullscreen": truth("fullscreen"),
                "head_topic": LaunchConfiguration("head_topic").perform(context),
            }],
        ))
    return actions


def generate_launch_description():
    mac = platform.system() == "Darwin"
    return LaunchDescription([
        # sdl on the Mac; ps on rosie, where the 8BitDo runs in PlayStation mode (054c:05c4) on evdev
        DeclareLaunchArgument("joy_layout", default_value="sdl" if mac else "ps"),
        DeclareLaunchArgument("leader_arms", default_value="true"),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("fullscreen", default_value="false" if mac else "true"),
        # the 10 fps viewer copy; /cameras/head/color/image_raw/compressed is the 30 fps stream
        DeclareLaunchArgument("head_topic", default_value="/cameras/head/viewer/compressed"),
        DeclareLaunchArgument("split_stick", default_value="false"),
        OpaqueFunction(function=_station),
    ])
