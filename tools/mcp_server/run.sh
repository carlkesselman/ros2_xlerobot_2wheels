#!/bin/bash
# Launcher for .mcp.json: MCP stdio clients only pass an allow-listed subset of environment
# variables to the server process (PATH, HOME, ... - not PYTHONPATH/AMENT_PREFIX_PATH/
# LD_LIBRARY_PATH), so rclpy and friends are NOT visible unless ROS is sourced here explicitly.
# The tools venv is $XLEROBOT_VENV, <repo>/.venv by default (tools/setup_venv.sh, docs/tools.md#python-environment).
set -e
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/../../install/setup.bash" 2>/dev/null || true
source "$(dirname "$0")/../ros_env.sh"
source "${XLEROBOT_VENV:-$(dirname "$0")/../../.venv}/bin/activate"
exec python3 "$(dirname "$0")/server.py"
