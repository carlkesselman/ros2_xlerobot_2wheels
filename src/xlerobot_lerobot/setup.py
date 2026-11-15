from setuptools import find_packages, setup

package_name = "xlerobot_lerobot"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/robot.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Carl Kesselman",
    maintainer_email="carl@isi.edu",
    description="LeRobot datasets and policies (SmolVLA) over the ROS 2 stack, for the operator station",
    license="Apache-2.0",
    # No console_scripts: these run on the LeRobot venv's Python, not the ROS one (tools/lerobot.sh).
)
