# Preparing the Jetson

Everything the robot's computer needs beyond a stock JetPack install to run the whole stack (ros2_control, Nav2,
MoveIt with an octomap, three cameras, the object perception models) at once. Each item says why; the symptom it
prevents is usually confusing enough to cost an afternoon.

## 1. System

- **JetPack 7.2** (L4T R39, Ubuntu 24.04, CUDA 13, TensorRT 10.16) on the Orin NX, see
  [ADR-0002](decisions/0002-jazzy-and-isaac-ros.md).
- **Power mode**: a high mode for the whole stack. Check with `nvpmodel -q`, set with `sudo nvpmodel -m <id>`; the modes
  are listed in `/etc/nvpmodel.conf`. The full stack has been run in the Orin NX 16 GB's 40 W mode.
- **Fan**: `FAN_DEFAULT_PROFILE` in `/etc/nvfancontrol.conf` (`quiet` or `cool`), then
  `sudo systemctl restart nvfancontrol`. Under the full stack expect about 65-70 °C.
- `jetson-stats` (`jtop`) to watch CPU, GPU and temperatures, optional.

## 2. ROS 2

ROS 2 Jazzy from the ROS apt repository, then the workspace's own dependencies with `rosdep` as in the
[README](../README.md#getting-started). Besides those, the robot uses:

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-orbbec-camera ros-jazzy-usb-cam \
                 ros-jazzy-navigation2 ros-jazzy-slam-toolbox ros-jazzy-moveit
```

## 3. DDS: Cyclone with large socket buffers

Every ROS process on the robot uses the settings in `tools/ros_env.sh`: Cyclone DDS
(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`), discovery on the subnet, and `tools/cyclonedds.xml`. The TUI, the MCP
server and the stack start script load it themselves; for your own shells, add to `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Developer/ros2_xlerobot_2wheels/install/setup.bash
source ~/Developer/ros2_xlerobot_2wheels/tools/ros_env.sh
```

Camera images travel between processes as UDP over loopback, a 1280x720 colour frame as about 2,000 fragments. With
the kernel's default receive buffer (208 KB) a reader that falls behind for a moment loses fragments, and best-effort
delivery drops the whole frame: slow readers (Python nodes, tools) receive few or no images while the camera driver
keeps publishing. `tools/cyclonedds.xml` asks for 16 MB; the kernel has to allow it:

```bash
printf "net.core.rmem_max=67108864\nnet.core.wmem_max=67108864\n" | sudo tee /etc/sysctl.d/60-ros-dds.conf
sudo sysctl --system
```

Fast DDS (Jazzy's default) is not used: under load its shared-memory transport recreated discovery ports of busy
processes, after which new processes saw topics but no nodes, and with localhost-only discovery it found only about
20 participants.

## 4. Network

Turn off Wi-Fi power saving. With it on, every packet waits for the radio to wake: pings of 40-300 ms, slow SSH,
late DDS messages.

```bash
printf "[connection]\nwifi.powersave = 2\n" | sudo tee /etc/NetworkManager/conf.d/99-wifi-powersave-off.conf
sudo systemctl reload NetworkManager
```

Do not subscribe to the raw camera images from another machine over Wi-Fi (RViz, rqt image views, a remote tool):
pulling 2.7 MB frames over the air stalls the camera driver's output for every subscriber. Look at the cameras
through Rerun (`tools/rerun_bridge.py`, which runs on the robot) or compressed topics.

## 5. USB

- **Device names**: `sudo python3 tools/xlerobot_setup_usb_modules.py` writes `/etc/udev/rules.d/99-xlerobot.rules`
  for `/dev/xle-bus1`, `/dev/xle-bus2`, the cameras and the lidar ([docs/setup.md](setup.md#2-device-names)). The
  cameras are matched by the USB port, so moving a camera to another port needs the tool again.
- **Orbbec camera permissions**:
  `sudo cp /opt/ros/jazzy/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`.
- **Power**: three cameras and two servo adapters exceed what the carrier board's ports supply; a camera that drops
  out now and then is starved of power, not broken. Put the cameras on a hub with its own power supply.
- **Permissions**: the udev rules give the robot's devices mode 0666, so no group membership is needed for them.
- `tools/kill_ros.py --usb-reset` resets the head camera on the USB bus with `usbreset`, which needs sudo without a
  password for that command.

## 6. Python and the perception models

The tools' venv: [docs/tools.md](tools.md#python-environment). The object perception service's TensorRT engines are
built on the robot from ONNX files exported on a machine with torch:
[src/xlerobot_object_perception/README.md](../src/xlerobot_object_perception/README.md).

## Checks

```bash
sysctl net.core.rmem_max                              # 67108864
iw dev wlP1p1s0 get power_save                        # off (interface name may differ)
echo $RMW_IMPLEMENTATION $CYCLONEDDS_URI              # rmw_cyclonedds_cpp file://.../cyclonedds.xml
ros2 topic hz /cameras/head/image_raw                 # about 30 Hz with the stack running
ls -l /dev/xle-*                                      # bus1, bus2, the cameras
```
