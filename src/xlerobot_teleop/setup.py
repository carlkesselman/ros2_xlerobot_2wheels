from setuptools import find_packages, setup

package_name = "xlerobot_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/teleop.launch.py", "launch/mcp_presence.launch.py", "launch/arm_servo_teleop.launch.py",
            "launch/station.launch.py",
        ]),
        ("share/" + package_name + "/config", [
            "config/joystick.yaml", "config/joystick_sdl.yaml", "config/joystick_ps.yaml",
            "config/joystick_split.yaml", "config/twist_mux.yaml", "config/mcp_presence.yaml",
            "config/arm_servo_teleop.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tamas Foldi",
    maintainer_email="tfoldi@xsi.hu",
    description="Joystick teleop and safety supervisor for XLeRobot 2-wheel base",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_supervisor = xlerobot_teleop.safety_supervisor:main",
            "mcp_presence_guard = xlerobot_teleop.mcp_presence_guard:main",
            "arm_servo_teleop = xlerobot_teleop.arm_servo_teleop:main",
            "leader_arms = xlerobot_teleop.leader_arms:main",
            "head_teleop = xlerobot_teleop.head_teleop:main",
            "station_viewer = xlerobot_teleop.station_viewer:main",
        ],
    },
)
