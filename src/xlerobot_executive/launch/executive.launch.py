"""The plan executive (ADR-0006). Needs the skill server."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("xlerobot_executive"), "config", "executive.yaml")
    return LaunchDescription([
        Node(package="xlerobot_executive", executable="executive", name="executive", output="screen", parameters=[cfg]),
    ])
