"""The MuJoCo twin and the stack against it, in one launch:

    ros2 launch sim/mujoco/mujoco_twin.launch.py [viewer:=true] [teleop:=false] [skills:=false]

- sim/mujoco/run_mujoco.py, the physics (with MuJoCo's viewer under viewer:=true, via mjpython)
- xlerobot_bringup's bringup.launch.py with use_sim:=true sim_engine:=mujoco, arms and head
- the LD19's laser_filters chain, /ldlidar_node/scan to /scan, as lidar.launch.py runs it on the
  robot (its driver is the twin here)

- with skills:=true, the arm stack: MoveIt's move_group, the skill server (xlerobot_skills, with
  sim/mujoco/skills_sim.yaml on top of its config) and the plan executive; the twin then answers
  /perception/detect from its own truth and publishes the operator presence heartbeat

Nav2 goes on top as on the robot: ros2 launch xlerobot_navigation navigation.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    bringup_share = get_package_share_directory("xlerobot_bringup")
    viewer = LaunchConfiguration("viewer")
    skills = LaunchConfiguration("skills")

    twin = ExecuteProcess(
        cmd=[PythonExpression(["'mjpython' if '", viewer, "' == 'true' else 'python3'"]),
             os.path.join(HERE, "run_mujoco.py"),
             PythonExpression(["'--viewer' if '", viewer, "' == 'true' else ''"]),
             PythonExpression(["'--presence' if '", skills, "' == 'true' else ''"])],
        output="screen",
    )
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "bringup.launch.py")),
        launch_arguments={
            "use_sim": "true",
            "sim_engine": "mujoco",
            "with_arms": "true",
            "with_head": "true",
            "teleop": LaunchConfiguration("teleop"),
        }.items(),
    )
    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        parameters=[os.path.join(bringup_share, "config", "laser_filters.yaml")],
        remappings=[("scan", "/ldlidar_node/scan"), ("scan_filtered", "/scan")],
    )
    arm_stack = GroupAction(condition=IfCondition(skills), actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("xlerobot_moveit_config"), "launch", "move_group.launch.py"))),
        Node(package="xlerobot_skills", executable="skill_server", name="skill_server", output="screen",
             parameters=[os.path.join(get_package_share_directory("xlerobot_skills"), "config", "skills.yaml"),
                         os.path.join(HERE, "skills_sim.yaml")]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("xlerobot_executive"), "launch", "executive.launch.py"))),
    ])
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="false"),
        DeclareLaunchArgument("teleop", default_value="false"),
        DeclareLaunchArgument("skills", default_value="false"),
        twin,
        bringup,
        scan_filter,
        arm_stack,
    ])
