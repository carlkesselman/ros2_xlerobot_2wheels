# ROS 2 settings for the robot (source after /opt/ros/jazzy/setup.bash). The whole stack, the TUI, the MCP server and the
# tools run on the Jetson; the laptop only talks to it over ssh (MCP, terminals) and gRPC (Rerun), so the laptop sets
# ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST (~/.profile) and never sees the Jetson's DDS traffic.
#
# Cyclone DDS, not the default Fast DDS: Fast DDS's shared-memory transport loses its discovery ports under load (a new
# process judges a busy owner's port dead and recreates it, after which new processes such as the TUI see topics but no
# nodes), and with LOCALHOST discovery it found only about 20 participants. Cyclone has no shared memory by default; images
# stay inside their composable containers (intra-process, later NITROS), so loopback UDP carries little.
# Every process must use the same RMW, so interactive shells on the Jetson should source this file too.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
