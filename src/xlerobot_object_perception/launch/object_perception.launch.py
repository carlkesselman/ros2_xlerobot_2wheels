"""On-demand object pose service. Runs in the perception venv, which has tensorrt and cuda-python next to ROS's Python packages."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("xlerobot_object_perception"), "config", "object_perception.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("python", default_value=os.environ.get("XLEROBOT_PERCEPTION_PYTHON",
                                                                       os.path.expanduser("~/venvs/xlerobot-perception/bin/python")),
                              description="Python with tensorrt and cuda-python (and ROS's packages visible)"),
        ExecuteProcess(cmd=[LaunchConfiguration("python"), "-m", "objperc.node", "--ros-args", "--params-file", cfg],
                       name="object_perception", output="screen"),
    ])
