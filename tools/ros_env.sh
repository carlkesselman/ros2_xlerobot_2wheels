# ROS 2 discovery for the robot (source after /opt/ros/jazzy/setup.bash). The whole stack, the TUI,
# the MCP server and the calibration tools run on the Jetson, the laptop only talks to it over ssh
# (MCP, terminals) and gRPC (Rerun), so the laptop sets ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# (~/.zshenv) and never sees the Jetson's DDS traffic.
# On the Jetson the range must stay SUBNET: with LOCALHOST only about 20 participants are found
# (21 of ~41 nodes visible, Nav2's lifecycle manager never saw its servers). Not
# ROS_LOCALHOST_ONLY, it is deprecated and overrides the range.
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
