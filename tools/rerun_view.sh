#!/bin/bash
# Rerun view of the robot from the operator station (the Mac): the three cameras, the lidar scan
# and the robot model, via tools/rerun_bridge.py. The bridge runs here and subscribes over the
# network: camera video arrives as H.264 (about 2 Mbit/s per camera) and is not re-encoded.
#
#   pixi run tools/rerun_view.sh [rerun_bridge.py args]
#
# Needs, once: tools/setup_venv.sh rerun (~/venvs/xlerobot-rerun). On the robot: bringup, plus
# cameras:=true lidar:=true (or cameras.launch.py / lidar.launch.py beside it).
# rclpy comes from the pixi environment, rerun-sdk from the venv; the viewer is the venv's own
# `rerun`, which matches the SDK version.
set -eo pipefail
cd "$(dirname "$0")/.."
venv="$HOME/venvs/xlerobot-rerun"
if [ ! -x "$venv/bin/python" ]; then
    echo "no $venv - run tools/setup_venv.sh rerun first" >&2
    exit 1
fi
source install/setup.bash
export PATH="$venv/bin:$PATH"   # rr.init(spawn=True) starts the viewer found on PATH
exec "$venv/bin/python" tools/rerun_bridge.py "$@"
