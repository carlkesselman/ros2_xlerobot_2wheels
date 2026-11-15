#!/bin/bash
# LeRobot on the operator station (the Mac), over ROS: src/xlerobot_lerobot, docs/lerobot.md.
#
#   pixi run tools/lerobot.sh record --repo-id <user>/<name> --task "..."     leader-arm demos -> LeRobot dataset
#   pixi run tools/lerobot.sh policy --policy <model> --task "..." [--live]   run a policy (dry run by default)
#   ROS_DOMAIN_ID=42 pixi run tools/lerobot.sh fake_robot                    fake topics, to test without the robot
#
# Each takes --help. Needs, once: tools/setup_venv.sh lerobot (~/venvs/xlerobot-lerobot) and pixi run build.
# The modules run on the venv's Python, rclpy comes from the pixi environment.
set -eo pipefail
cd "$(dirname "$0")/.."
venv="$HOME/venvs/xlerobot-lerobot"
if [ ! -x "$venv/bin/python" ]; then
    echo "no $venv - run tools/setup_venv.sh lerobot first" >&2
    exit 1
fi
if [ ! -d install/xlerobot_lerobot ]; then
    echo "xlerobot_lerobot is not built - run pixi run build first" >&2
    exit 1
fi
source install/setup.bash
# The venv's packages go first. pixi's site-packages are on PYTHONPATH (rclpy lives there) and carry their own NumPy
# and Pillow, which would otherwise shadow the versions torch and lerobot were installed with.
export PYTHONPATH="$venv/lib/python3.12/site-packages:$PYTHONPATH"
export SVT_LOG="${SVT_LOG:-1}"   # SVT-AV1 (the dataset's video encoder): errors only, not a config dump per video
cmd="${1:-}"
case "$cmd" in
    record|policy|fake_robot)
        shift
        exec "$venv/bin/python" -m "xlerobot_lerobot.$cmd" "$@"
        ;;
    *)
        echo "usage: $0 record|policy|fake_robot [args]   (each takes --help)" >&2
        exit 2
        ;;
esac
