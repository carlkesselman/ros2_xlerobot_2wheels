"""Cameras, from config/cameras.yaml. Included by bringup.launch.py with cameras:=true, or run
standalone, optionally with config:=<other cameras.yaml> for a machine with different devices.

left/right (arm wrist cameras) are always plain USB webcams: one usb_cam node per camera whose
device exists, in the namespace cameras/<name>. image_transport's ffmpeg plugin encodes on
<name>/image_raw/ffmpeg while something subscribes to it.

head depends on the sensor:=usbcam|gemini arg (default usbcam, matches xlerobot.urdf.xacro's own
default - see ADR-0003):
  - usbcam: same plain usb_cam node as left/right, from cameras.yaml's head entry.
  - gemini: the usb_cam entry for head is skipped entirely. Instead, the official
    orbbec_camera package's own gemini2.launch.py is included under an outer
    PushRosNamespace("cameras") with camera_name:="head" - gemini2.launch.py uses camera_name
    for both its own PushRosNamespace *and* literally as the composable node's name, and ROS 2
    node names can't contain "/" (confirmed by testing: passing camera_name:="cameras/head"
    directly throws "Couldn't parse remap rule" on the node's own __node:= argument) - two
    stacked namespace pushes (outer "cameras", inner "head" from the included file) gets the
    same cameras/head/... topic layout everything else uses, while keeping the node's own name
    a plain valid "head". Its color/image_raw and color/camera_info are remapped to
    cameras/head/image_raw and cameras/head/camera_info specifically, so existing flat-topic
    consumers (the MCP server's capture_image/remember_object, tools/rerun_bridge.py) keep
    working unchanged - depth/ir/point cloud are new capabilities nobody consumes yet, left at
    their native nested names (cameras/head/depth/image_raw etc.), clearer than remapping those
    too.

    The driver publishes its own calibrated extrinsic TF tree rooted at a link named
    "<camera_name>_link" - NOT a fixed "camera_link" (an earlier standalone test left
    camera_name at its own default, "camera", which happened to render as "camera_link" and
    looked like a hardcoded constant; it isn't - confirmed live once camera_name:="head" was
    actually passed here, which renames the whole tree to head_link -> head_depth_frame ->
    head_{color,ir}_frame -> head_{color,ir}_optical_frame, etc). xlerobot_description/urdf/
    sensors.xacro's gemini branch places a link named "head_link" at the physical mount
    point to match, so this tree attaches to the robot's own TF chain instead of floating
    disconnected from it. That tree is only actually published when
    enable_publish_extrinsic:=true is passed - the vendor launch file defaults it to false,
    in which case the node still advertises /tf and /tf_static but never sends anything on
    them (confirmed live: only robot_state_publisher's static transforms showed up without
    this).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ComposableNode


def _camera_nodes(context, *args, **kwargs):
    with open(LaunchConfiguration("config").perform(context)) as f:
        config = yaml.safe_load(f)
    d = config["defaults"]
    sensor = LaunchConfiguration("sensor").perform(context)

    actions = []
    for name, camera in config["cameras"].items():
        if name == "head" and sensor == "gemini":
            orbbec_share = get_package_share_directory("orbbec_camera")
            gemini2_launch_file = os.path.join(orbbec_share, "launch", "gemini2.launch.py")
            actions.append(GroupAction([
                PushRosNamespace("cameras"),
                SetRemap("color/image_raw", "image_raw"),
                SetRemap("color/camera_info", "camera_info"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(gemini2_launch_file),
                    launch_arguments={
                        "camera_name": "head",
                        # Default false - without it the driver only advertises /tf and
                        # /tf_static but never actually publishes the per-stream extrinsic
                        # tree (camera_depth_frame -> camera_color_frame ->
                        # camera_color_optical_frame etc.), confirmed live: robot_state_publisher's
                        # static transforms showed up fine, the driver's own tree did not, until
                        # this was set.
                        "enable_publish_extrinsic": "true",
                        # The driver's own cloud is 1280x720 (about 920k points) and costs most of
                        # a core; MoveIt's octomap gets the decimated one below instead.
                        "enable_point_cloud": "false",
                        # Stamp frames with the host clock. The default, "global", maps the
                        # device clock to the host's and had drifted 160 s behind, which made
                        # MoveIt drop every cloud as older than its TF buffer.
                        "time_domain": "system",
                    }.items(),
                ),
                # depth/points_low: the depth image shrunk 4x (nearest, so no depth blending at
                # edges) and turned into a cloud, about 58k points, for the MoveIt octomap.
                ComposableNodeContainer(
                    package="rclcpp_components",
                    executable="component_container",
                    name="depth_cloud_container",
                    namespace="/cameras/head",
                    composable_node_descriptions=[
                        ComposableNode(
                            package="image_proc",
                            plugin="image_proc::ResizeNode",
                            name="depth_resize",
                            namespace="/cameras/head",
                            parameters=[{"scale_height": 0.25, "scale_width": 0.25, "interpolation": 0}],
                            remappings=[
                                ("image/image_raw", "depth/image_raw"),
                                ("image/camera_info", "depth/camera_info"),
                                ("resize/image_raw", "depth_low/image_raw"),
                                ("resize/camera_info", "depth_low/camera_info"),
                            ],
                        ),
                        ComposableNode(
                            package="depth_image_proc",
                            plugin="depth_image_proc::PointCloudXyzNode",
                            name="depth_points_low",
                            namespace="/cameras/head",
                            remappings=[
                                ("image_rect", "depth_low/image_raw"),
                                ("camera_info", "depth_low/camera_info"),
                                ("points", "depth/points_low"),
                            ],
                        ),
                    ],
                ),
            ]))
            continue

        device = camera["device"]
        if not os.path.exists(device):
            print(f"[cameras.launch] {name}: {device} not found, skipping")
            continue
        cfg = {**d, **camera}
        actions.append(Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            namespace=f"cameras/{name}",
            parameters=[{
                "video_device": device,
                "image_width": cfg["width"],
                "image_height": cfg["height"],
                "framerate": float(cfg["fps"]),
                "pixel_format": cfg["pixel_format"],
                "camera_name": name,
                # usb_cam_node_exe's actual parameter is "frame_id", not "camera_frame_id" -
                # confirmed via `ros2 param describe`: the wrong key was being silently ignored
                # (ROS2 doesn't error on an unused parameter dict entry), so every camera_info
                # header read usb_cam's own hardcoded default ("default_cam") instead of the real
                # URDF optical frame name below, regardless of what cameras.yaml said.
                "frame_id": cfg.get("frame_id", f"{name}_camera_optical_frame"),
                "camera_info_url": "",
                "brightness": int(cfg.get("brightness", 0)),
                "image_raw.ffmpeg.encoder": cfg["encoder"],
                "image_raw.ffmpeg.encoder_av_options": cfg["encoder_options"],
                "image_raw.ffmpeg.bit_rate": cfg["bit_rate"],
                "image_raw.ffmpeg.gop_size": cfg["gop_size"],
                "image_raw.ffmpeg.max_b_frames": 0,
            }],
        ))
    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("xlerobot_bringup"), "config", "cameras.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("sensor", default_value="usbcam"),
        OpaqueFunction(function=_camera_nodes),
    ])
