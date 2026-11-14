#!/usr/bin/env bash
# The sudo half of preparing the robot's computer (docs/jetson.md): ROS 2 Jazzy from the ROS apt
# repository, the build tools, and the system settings the stack needs. Run once, on the robot,
# as your own user (it calls sudo itself and asks for the password once):
#
#   bash tools/install_ros_jazzy.sh
#
# Safe to run again; every step checks or overwrites its own file. The workspace build itself
# (vcs import, rosdep install, colcon build) needs no sudo beyond rosdep's apt step and is not here.
set -euo pipefail

if [ "$(. /etc/os-release && echo "$VERSION_CODENAME")" != "noble" ]; then
  echo "Jazzy's apt packages are for Ubuntu 24.04 (noble); this is $(. /etc/os-release && echo "$PRETTY_NAME")" >&2
  exit 1
fi

sudo -v   # ask for the password up front

echo "== ROS 2 apt repository"
sudo apt-get update
sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | cut -d'"' -f4)
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.noble_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  rm -f /tmp/ros2-apt-source.deb
fi
sudo apt-get update

echo "== ROS 2 Jazzy and build tools"
# ros-base, not desktop: the robot is headless. rosdep installs everything else the packages declare.
sudo apt-get install -y \
  ros-jazzy-ros-base ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-jazzy-rmw-cyclonedds-cpp \
  libusb-1.0-0-dev
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init

echo "== DDS socket buffers (docs/jetson.md section 3)"
printf "net.core.rmem_max=67108864\nnet.core.wmem_max=67108864\n" | sudo tee /etc/sysctl.d/60-ros-dds.conf >/dev/null
sudo sysctl --system >/dev/null

echo "== Wi-Fi power saving off (section 4)"
printf "[connection]\nwifi.powersave = 2\n" | sudo tee /etc/NetworkManager/conf.d/99-wifi-powersave-off.conf >/dev/null
sudo systemctl reload NetworkManager

echo "== Real-time priority for the control loop (section 6)"
printf "@realtime soft rtprio 99\n@realtime hard rtprio 99\n@realtime soft memlock unlimited\n@realtime hard memlock unlimited\n" \
  | sudo tee /etc/security/limits.d/realtime.conf >/dev/null
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"

echo
echo "Done. Log out and in again for the realtime group. Then, without sudo:"
echo "  rosdep update"
