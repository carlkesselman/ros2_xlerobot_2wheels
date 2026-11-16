#!/bin/bash
# The operator station: gamepad, leader arms and the station viewer (docs/operator-station.md).
#
#   tools/station.sh                       on rosie, from its own terminal or over ssh; the viewer goes full screen
#   tools/station.sh leader_arms:=false    any station.launch.py argument passes through
#   tools/station.sh build                 (re)build the station's packages first, then exit
#
# On rosie (Linux, aarch64) it uses pixi's rosie environment, on the Mac the default one. Over ssh the viewer still
# opens on rosie's monitor: the desktop session's Wayland socket (and X display, for XWayland) are filled in when the
# shell has none.
set -eo pipefail
cd "$(dirname "$0")/.."

env=default
if [ "$(uname -s)" = Linux ]; then
    env=rosie
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
    if [ -z "$WAYLAND_DISPLAY" ] && [ -z "$DISPLAY" ]; then
        sock=$(cd "$XDG_RUNTIME_DIR" 2>/dev/null && ls -1 wayland-? 2>/dev/null | head -1 || true)
        [ -n "$sock" ] && export WAYLAND_DISPLAY="$sock"
        export DISPLAY=:0
    fi
fi

if [ "${1:-}" = build ] || [ ! -d install/xlerobot_teleop ]; then
    pixi run -e "$env" build
    [ "${1:-}" = build ] && exit 0
fi

exec pixi run -e "$env" bash -c 'source install/setup.bash && exec ros2 launch xlerobot_teleop station.launch.py "$@"' \
    station "$@"
