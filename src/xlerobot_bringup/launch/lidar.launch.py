"""LD19 lidar: the driver's lifecycle node in a component container, and a Nav2
lifecycle manager that configures and activates it. Publishes /ldlidar_node/scan
in ldlidar_link, the frame xlerobot_description places on the robot.

A laser_filters chain (config/laser_filters.yaml) takes that scan, drops the
cart's own posts and publishes /scan, which Nav2, slam_toolbox and amcl read.
It needs base_link -> ldlidar_link from robot_state_publisher.

Not the driver's own ldlidar_bringup.launch.py: that one starts a second
robot_state_publisher with its own little URDF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("xlerobot_bringup"), "config", "ldlidar.yaml")
    config = LaunchConfiguration("lidar_config")
    filter_config = os.path.join(
        get_package_share_directory("xlerobot_bringup"), "config", "laser_filters.yaml")

    container = ComposableNodeContainer(
        name="ldlidar_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=[
            ComposableNode(
                package="ldlidar_component",
                plugin="ldlidar::LdLidarComponent",
                name="ldlidar_node",
                parameters=[config],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="ldlidar_lifecycle_manager",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["ldlidar_node"],
            "bond_timeout": 5.0,
            "attempt_respawn_reconnection": True,
            "bond_respawn_max_duration": 10.0,
        }],
    )
    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        parameters=[filter_config],
        remappings=[("scan", "/ldlidar_node/scan"), ("scan_filtered", "/scan")],
    )
    return LaunchDescription([
        DeclareLaunchArgument("lidar_config", default_value=default_config),
        container,
        lifecycle_manager,
        scan_filter,
    ])
