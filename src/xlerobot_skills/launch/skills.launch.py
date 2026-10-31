"""The skill action servers (ADR-0006). Needs the stack; move_arm and approach need move_group, navigate_to needs Nav2,
detect and approach need the object perception service."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("xlerobot_skills"), "config", "skills.yaml")
    return LaunchDescription([
        Node(package="xlerobot_skills", executable="skill_server", name="skill_server", output="screen", parameters=[cfg]),
    ])
