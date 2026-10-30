"""On-demand object pose service. Runs with the tools venv's Python ($XLEROBOT_VENV, ~/venvs/xlerobot by default), which has
cuda-python next to ROS's and JetPack's Python packages (docs/tools.md#python-environment)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("xlerobot_object_perception"), "config", "object_perception.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("python", default_value=os.path.join(os.environ.get("XLEROBOT_VENV", os.path.expanduser("~/venvs/xlerobot")), "bin", "python"),
                              description="Python with tensorrt and cuda-python (and ROS's packages visible)"),
        ExecuteProcess(cmd=[LaunchConfiguration("python"), "-m", "objperc.node", "--ros-args", "--params-file", cfg],
                       name="object_perception", output="screen"),
    ])
