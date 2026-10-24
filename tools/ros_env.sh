# Keep all ROS 2 traffic on one machine (ROS_LOCALHOST_ONLY is deprecated in Jazzy and overrides the range, so it is not set). Source this after /opt/ros/jazzy/setup.bash (that file
# sets ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET when it is not already set, and SUBNET lets nodes find
# each other over Wi-Fi: slow discovery, lost image frames, late joystick/presence messages).
# The whole stack, the TUI, the MCP server and the calibration tools run on the Jetson; the laptop
# only talks to it over ssh (MCP, terminals) and gRPC (Rerun).
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
