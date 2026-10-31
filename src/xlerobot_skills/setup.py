from setuptools import find_packages, setup

package_name = "xlerobot_skills"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/skills.launch.py"]),
        ("share/" + package_name + "/config", ["config/skills.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tamas Foldi",
    maintainer_email="tfoldi@xsi.hu",
    description="The robot's skills as ROS 2 actions (ADR-0006)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["skill_server = xlerobot_skills.server:main"]},
)
