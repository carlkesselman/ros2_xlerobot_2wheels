"""Cameras, from config/cameras.yaml. Included by bringup.launch.py with cameras:=true, or run
standalone, optionally with config:=<other cameras.yaml> for a machine with different devices.

left/right (arm wrist cameras) are always plain USB webcams: one usb_cam node per camera whose
device exists, in the namespace cameras/<name>. image_transport's ffmpeg plugin encodes on
<name>/image_raw/ffmpeg while something subscribes to it.

head depends on the sensor:=usbcam|gemini|realsense arg (realsense: _realsense_head below, colour
plus aligned depth from realsense2_camera; default usbcam, matches xlerobot.urdf.xacro's own
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


def _viewer_throttle(name, cfg, source="image_raw/compressed"):
    """The remote viewer's copy of a camera's JPEG stream, see cameras.yaml viewer_fps. The
    throttle keeps the JPEG encoder running at the full rate (it always subscribes), but only
    viewer_fps frames a second leave the robot. source is relative to cameras/<name>."""
    viewer_fps = float(cfg.get("viewer_fps", 0))
    if viewer_fps <= 0:
        return []
    return [Node(
        package="topic_tools",
        executable="throttle",
        name="viewer_throttle",
        namespace=f"cameras/{name}",
        arguments=["messages", source, str(viewer_fps), "viewer/compressed"],
    )]


def _realsense_head(cfg):
    """The head RealSense through realsense2_camera: colour, and depth aligned to it.

    The node is /cameras/head. Its colour comes out at cameras/head/image_raw and camera_info,
    remapped from color/*, so every consumer of the usb_cam layout keeps working, the viewer
    copy included, except that the driver's transports (compressed, ffmpeg, ...) stay at
    color/image_raw/*: the remap moves only the raw topic. Aligned depth stays at aligned_depth_to_color/*: the unaligned depth stream's
    depth/camera_info has different intrinsics, so remapping onto depth/* would mix the two.

    With camera_name head the driver publishes its TF tree rooted at head_link (head_link ->
    head_color_frame -> head_color_optical_frame, ...). xlerobot_description's realsense
    branch places head_link on the head, as for the Gemini.

    depth_viewer/image_raw: the aligned depth shrunk by depth_viewer_scale (nearest, so no
    blending at edges) and throttled to depth_viewer_fps, for the remote viewer. Raw 640x480
    depth at 30 fps is about 18 MB/s; at half size and 5 fps it is about 0.75 MB/s. Its intrinsics
    are depth_small/camera_info (full rate, small): image_transport names a camera's info topic
    after its image topic, so the resize node's info follows the depth_small/image_raw remap."""
    head = "/cameras/head"
    nodes = [Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="cameras",
        name="head",
        parameters=[{
            "camera_name": "head",
            "serial_no": str(cfg.get("serial_no", "")),
            "rgb_camera.color_profile": cfg["color_profile"],
            "depth_module.depth_profile": cfg["depth_profile"],
            "enable_color": True,
            "enable_depth": True,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": False,        # the D415 has no IMU
            "enable_accel": False,
            "align_depth.enable": True,
            "pointcloud.enable": False,  # the viewer back-projects depth itself
            "publish_tf": True,
            "tf_publish_rate": 0.0,      # static
            # image_transport names its parameters after the topic before remapping
            ".head.color.image_raw.compressed.jpeg_quality": int(cfg.get("jpeg_quality", 95)),
        }],
        remappings=[(f"{head}/color/image_raw", f"{head}/image_raw"),
                    (f"{head}/color/camera_info", f"{head}/camera_info")],
    )]
    scale = float(cfg.get("depth_viewer_scale", 0.5))
    depth_fps = float(cfg.get("depth_viewer_fps", 0))
    if depth_fps > 0:
        nodes.append(ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container",
            name="depth_viewer_container",
            namespace=head,
            composable_node_descriptions=[ComposableNode(
                package="image_proc",
                plugin="image_proc::ResizeNode",
                name="depth_viewer_resize",
                namespace=head,
                parameters=[{"scale_height": scale, "scale_width": scale, "interpolation": 0}],
                remappings=[
                    ("image/image_raw", "aligned_depth_to_color/image_raw"),
                    ("image/camera_info", "aligned_depth_to_color/camera_info"),
                    ("resize/image_raw", "depth_small/image_raw"),   # info: depth_small/camera_info
                ],
            )],
        ))
        nodes.append(Node(
            package="topic_tools",
            executable="throttle",
            name="depth_viewer_throttle",
            namespace=head,
            arguments=["messages", "depth_small/image_raw", str(depth_fps), "depth_viewer/image_raw"],
        ))
    return nodes + _viewer_throttle("head", cfg, source="color/image_raw/compressed")


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

        if name == "head" and sensor == "realsense":
            actions += _realsense_head({**d, **camera, **camera.get("realsense", {})})
            continue

        # device is a path or a list of them: the first one that exists is used, so one file
        # serves machines whose udev rules name the same camera differently. A list entry can
        # also be a mapping, {path: ..., <setting>: ...}, whose settings apply only to that device
        # (a different camera behind the same name, say, with its own pixel_format).
        candidates = camera["device"] if isinstance(camera["device"], list) else [camera["device"]]
        candidates = [c if isinstance(c, dict) else {"path": c} for c in candidates]
        found = next((c for c in candidates if os.path.exists(c["path"])), None)
        if found is None:
            print(f"[cameras.launch] {name}: none of {[c['path'] for c in candidates]} found, skipping")
            continue
        # usb_cam follows a symlink itself but takes its target as relative to /dev, so a link
        # outside /dev (/dev/v4l/by-path/..., target ../../video8) becomes /dev/../../video8.
        device = os.path.realpath(found["path"])
        cfg = {**d, **camera, **{k: v for k, v in found.items() if k != "path"}}
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
                # compressed_image_transport declares its parameters with a leading dot here
                ".image_raw.compressed.jpeg_quality": int(cfg.get("jpeg_quality", 95)),
            }],
        ))
        actions += _viewer_throttle(name, cfg)
    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("xlerobot_bringup"), "config", "cameras.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("sensor", default_value="usbcam"),
        OpaqueFunction(function=_camera_nodes),
    ])
