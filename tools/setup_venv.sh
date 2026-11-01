#!/bin/bash
# Create or update a Python environment for tools/ with uv (pyproject.toml at the repo root).
#
#   tools/setup_venv.sh            the tools venv, <repo>/.venv
#   tools/setup_venv.sh rerun      ~/venvs/xlerobot-rerun, for rerun_bridge.py
#   tools/setup_venv.sh models     ~/venvs/xlerobot-models, perception model export (NVIDIA GPU machine)
#
# The tools venv is built on the ROS install's own Python with --system-site-packages, so rclpy, cv_bridge, the message
# types and (on the robot) OpenCV, SciPy and TensorRT are visible in it. That Python is the pixi environment's on a Mac
# (run this inside `pixi shell`, or as `pixi run venv`) and /usr/bin/python3 with apt ROS Jazzy on the robot. A bare
# `uv sync` would instead build .venv on a uv-managed Python that cannot see rclpy.
#
# rerun and models want NumPy 2 and must not see the system packages; they get plain venvs. rclpy still reaches them
# through the sourced ROS install's PYTHONPATH.
set -euo pipefail
cd "$(dirname "$0")/.."

which_env="${1:-tools}"

if [ -n "${CONDA_PREFIX:-}" ] && [ -x "$CONDA_PREFIX/bin/python" ]; then
    ros_python="$CONDA_PREFIX/bin/python"       # pixi
else
    ros_python=/usr/bin/python3                 # apt ROS Jazzy
fi

case "$which_env" in
    tools)
        venv="${XLEROBOT_VENV:-$PWD/.venv}"
        if [ ! -x "$venv/bin/python" ] || ! grep -q "include-system-site-packages = true" "$venv/pyvenv.cfg"; then
            uv venv --clear --python "$ros_python" --system-site-packages "$venv"
        fi
        UV_PROJECT_ENVIRONMENT="$venv" uv sync --group tools
        "$venv/bin/python" -c "import rclpy" 2>/dev/null \
            || echo "warning: rclpy is not visible in $venv - source ROS (or use pixi shell) before running the tools"
        ;;
    rerun|models)
        venv="$HOME/venvs/xlerobot-$which_env"
        [ -x "$venv/bin/python" ] || uv venv --python 3.12 "$venv"
        UV_PROJECT_ENVIRONMENT="$venv" uv sync --no-default-groups --group "$which_env"
        ;;
    *)
        echo "usage: $0 [tools|rerun|models]" >&2
        exit 2
        ;;
esac
echo "ready: $venv"
