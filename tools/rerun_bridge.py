#!/usr/bin/env python3
"""Streams the robot, its odometry, velocities, Nav2 goal and plan, and camera
video into Rerun for timeline debugging. Complements RViz/Foxglove rather than
replacing them: scrub commanded vs actual velocity next to the robot moving in
the odom frame, with the camera views alongside.

The model comes from /robot_description (the same URDF robot_state_publisher
serves), placed in the odom frame from odometry, wheels turned from
/joint_states. The odom path is drawn in the 3D view and in a top-down view.

Nav2: /goal_pose (what RViz's 2D Goal Pose publishes), /plan and /local_plan
are drawn in both views. There is no localisation yet, map -> odom is an
identity transform, so map poses are drawn as odom poses.

The slam_toolbox map (/map) is drawn as points in the map frame, with map -> odom from
/tf, so the robot, the scan and the plan sit in the map as SLAM sees it. The map frame
starts at the robot's start pose.

With amcl running, the particle cloud (/particle_cloud) is drawn in the map frame too.

The LD19 scan (/ldlidar_node/scan) is drawn as points in its own frame, coloured
by beam angle from red at scan angle 0, next to arrows for the scan's zero direction
and the robot's front, so the mounting yaw can be checked by eye.

Camera video is taken from the ffmpeg_image_transport topics
(<camera>/image_raw/ffmpeg, H.264 or H.265 packets) and logged as an encoded
video stream, so nothing is decoded or re-encoded here. Cameras are found
while running, so they can start after the bridge.

The head camera's depth stream (<camera>/depth/image_raw, 16-bit, hardware-registered to
color) is logged alongside its color video and camera_info at the same entity, so the
viewer back-projects it into a colored 3D point cloud on its own - no point cloud topic
is subscribed to or parsed here.

The Logs tab is two side-by-side panels sharing the same ros_time timeline: ROS2
Log (/rosout, every node's log messages) and MCP Calls (tools/mcp_server's own
call_log.py audit trail, logs/mcp_calls.jsonl, tailed from disk - the MCP server
has no ROS dependency, so this is a file read, not a subscription). Each MCP
entry shows the call, the agent's stated reason, and the result/error together,
so "what was the agent doing, and what did the stack say back" reads next to
"what was ROS2 saying at that same moment" without digging through two tools.

Usage (bringup already running):
    python3 tools/rerun_bridge.py
    python3 tools/rerun_bridge.py --save run.rrd    # record to a file, no viewer
    python3 tools/rerun_bridge.py --connect rerun+http://laptop:9876/proxy
        # run on the robot, show in a viewer started elsewhere with `rerun`
"""

import argparse
import colorsys
import datetime
import json
import math
import os
import pathlib
import signal
import tempfile

import rclpy
import rerun as rr
import rerun.blueprint as rrb
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rerun.blueprint.components import PlayState
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

try:
    from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
except ImportError:
    FFMPEGPacket = None

ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
MAP_FRAME = "map"
LIDAR_FRAME = "ldlidar_link"
LIDAR_ENTITIES = ("lidar/scan", "lidar/zero")
MAP_ENTITIES = ("world/map/occupied", "world/map/free", "world/particles")
MAP_TF_PERIOD = 0.2  # s, slam_toolbox publishes map -> odom at 50 Hz
JOINT_TRANSFORM_PERIOD = 1.0 / 30  # s, /joint_states arrives at 100 Hz, 30 Hz is smooth enough
CAMERAS = ("head", "left", "right")
DEPTH_CAMERAS = ("head",)  # left/right are plain 2D wrist webcams, no depth stream
DEPTH_LOG_PERIOD = 1.0 / 8  # s, matches the driver's own depth/points rate - raw depth/image_raw
# arrives at ~20-26Hz uncompressed (1280x720 16UC1, no ffmpeg transport for it), logging every
# frame was the same mistake that caused the 90s joint-state latency bug below - throttle it.
DEPTH_METER = 1000.0  # Orbbec depth is millimeters (OpenNI/RealSense convention), not measured
FFMPEG_TYPE = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket"
COMPRESSED_TYPE = "sensor_msgs/msg/CompressedImage"
AV_PKT_FLAG_KEY = 1

# rcl_interfaces/msg/Log.level values (rclpy.logging.LoggingSeverity) -> rr.TextLogLevel
ROSOUT_LEVELS = {
    10: rr.TextLogLevel.DEBUG, 20: rr.TextLogLevel.INFO,
    30: rr.TextLogLevel.WARN, 40: rr.TextLogLevel.ERROR, 50: rr.TextLogLevel.CRITICAL,
}
# matches mcp_server/call_log.py's own LOG_PATH (repo root)/logs/mcp_calls.jsonl
MCP_LOG_PATH = pathlib.Path(__file__).resolve().parent.parent / "logs" / "mcp_calls.jsonl"
MCP_LOG_POLL_PERIOD = 0.5  # s

# 3D entities that hold world (odom frame) coordinates
WORLD_ENTITIES = ("world/odom_path", "world/goal", "world/plan", "world/local_plan")
# a new path point is only logged after this much movement
PATH_MIN_STEP = 0.02  # m
PATH_MIN_TURN = 0.05  # rad
PLAN_Z = 0.02  # lift plan/path lines a little off the floor grid

# Okabe-Ito colours: measured in strong colours, commands in lighter ones.
# linear velocity in blues and green, angular in oranges and purple.
VELOCITY_SERIES = {
    "velocity/odom_linear": ("odom vx", (0, 114, 178)),
    "velocity/odom_angular": ("odom wz", (213, 94, 0)),
    "velocity/cmd_linear": ("cmd vx", (86, 180, 233)),
    "velocity/cmd_angular": ("cmd wz", (230, 159, 0)),
    "velocity/nav_linear": ("nav vx", (0, 158, 115)),
    "velocity/nav_angular": ("nav wz", (204, 121, 167)),
}
GOAL_COLOR = (0, 158, 115)
PLAN_COLOR = (0, 158, 115)
LOCAL_PLAN_COLOR = (230, 159, 0)


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec / 1e9


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def robot_view(**eye):
    return rrb.Spatial3DView(
        name="Robot", origin="/",
        contents=[
            "/xlerobot/**", "/transforms/**", "/world/**", "/lidar/**", "/cameras/**",
            "- /xlerobot/collision_geometries/**"],
        spatial_information=rrb.SpatialInformation(target_frame=ODOM_FRAME),
        eye_controls=rrb.EyeControls3D(**eye))


def camera_views(cameras):
    return [rrb.Spatial2DView(name=f"{name} camera", origin=f"/cameras/{name}") for name in cameras]


def blueprint(cameras):
    # Tabs, not everything at once: the robot+cameras view is what you want live and
    # by default, the rest (odometry/map/lidar diagnostics) is there when needed but
    # doesn't compete for screen space the rest of the time.
    main_tab = rrb.Horizontal(
        # from the front left and above, the whole robot with both arms and the
        # head in view, +z up
        robot_view(position=(1.25, 0.95, 1.3), look_target=(-0.1, 0.0, 0.75), eye_up=(0.0, 0.0, 1.0)),
        rrb.Vertical(
            *([rrb.Horizontal(*camera_views(cameras))] if cameras else []),
            rrb.TimeSeriesView(name="Velocities", origin="/velocity"),
            rrb.TextLogView(name="MCP Calls", origin="/logs/mcp"),
            row_shares=[3, 1, 1.3] if cameras else [1, 1],
        ),
        column_shares=[3, 2],
        name="Main",
    )

    tabs = [main_tab]

    if cameras:
        tabs.append(rrb.Horizontal(*camera_views(cameras), name="Cameras"))

    if "head" in cameras:
        # the head depth cloud against the robot model, from the side and from above in
        # base_link, to check the head camera's mounted pose (tools/calibrate_head_pose.py)
        tabs.append(rrb.Horizontal(
            *[rrb.Spatial3DView(
                name=name, origin="/",
                contents=["/xlerobot/**", "/transforms/**", "/cameras/head",
                          "- /xlerobot/collision_geometries/**"],
                spatial_information=rrb.SpatialInformation(target_frame="base_link"),
                eye_controls=rrb.EyeControls3D(
                    position=position, look_target=(0.3, 0.0, 0.6), eye_up=eye_up))
              for name, position, eye_up in (
                  ("Head depth, side", (0.3, -2.8, 0.7), (0.0, 0.0, 1.0)),
                  ("Head depth, top", (0.3, 0.0, 3.0), (1.0, 0.0, 0.0)))],
            name="Head depth check",
        ))

    tabs.append(rrb.Horizontal(
        rrb.Vertical(
            # from above with the robot's front (+x) up, the scan is in the robot's frame
            rrb.Spatial3DView(
                name="Lidar (front up)", origin="/",
                contents=[
                    "/xlerobot/**", "/transforms/**", "/lidar/**", "/world/front",
                    "- /xlerobot/collision_geometries/**"],
                spatial_information=rrb.SpatialInformation(target_frame="base_link"),
                eye_controls=rrb.EyeControls3D(
                    position=(0.0, 0.0, 2.6), look_target=(0.0, 0.0, 0.0), eye_up=(1.0, 0.0, 0.0))),
            rrb.Spatial3DView(
                name="Map (start heading up)", origin="/",
                contents=[
                    "/xlerobot/**", "/transforms/**", "/lidar/**", "/world/**",
                    "- /xlerobot/collision_geometries/**"],
                spatial_information=rrb.SpatialInformation(target_frame=MAP_FRAME),
                eye_controls=rrb.EyeControls3D(
                    position=(0.0, 0.0, 8.0), look_target=(0.0, 0.0, 0.0), eye_up=(1.0, 0.0, 0.0))),
            row_shares=[1, 1],
        ),
        rrb.Vertical(
            rrb.Spatial2DView(
                name="Odometry (top view)", origin="/odom",
                contents=["/odom/path", "/odom/pose", "/odom/goal", "/odom/plan"]),
            rrb.TimeSeriesView(name="Yaw", origin="/odom/yaw"),
            row_shares=[2, 1],
        ),
        column_shares=[1, 1],
        name="Odometry & Map",
    ))

    tabs.append(rrb.Horizontal(
        rrb.TextLogView(name="ROS2 Log", origin="/logs/rosout"),
        rrb.TextLogView(name="MCP Calls", origin="/logs/mcp"),
        name="Logs",
    ))

    return rrb.Blueprint(
        rrb.Tabs(*tabs, active_tab=0),
        # follow the live edge, otherwise a fresh viewer sits at the start of the
        # recording, before the first odom transform, and draws no robot
        rrb.TimePanel(timeline="ros_time", play_state=PlayState.Following),
    )


def log_static_setup():
    for entity in WORLD_ENTITIES:
        rr.log(entity, rr.CoordinateFrame(ODOM_FRAME), static=True)
    for entity in MAP_ENTITIES:
        rr.log(entity, rr.CoordinateFrame(MAP_FRAME), static=True)
    for entity in LIDAR_ENTITIES:
        rr.log(entity, rr.CoordinateFrame(LIDAR_FRAME), static=True)
    rr.log("world/front", rr.CoordinateFrame("base_link"), static=True)
    rr.log("world/front", rr.Arrows3D(
        origins=[[0.0, 0.0, 0.0]], vectors=[[0.6, 0.0, 0.0]],
        colors=[(255, 255, 255)], radii=0.01, labels=["robot front (+x)"]), static=True)
    rr.log("lidar/zero", rr.Arrows3D(
        origins=[[0.0, 0.0, 0.0]], vectors=[[0.4, 0.0, 0.0]],
        colors=[(213, 94, 0)], radii=0.008, labels=["scan angle 0"]), static=True)
    for entity, (name, color) in VELOCITY_SERIES.items():
        rr.log(entity, rr.SeriesLines(colors=[color], names=[name], widths=[2.0]), static=True)


class RerunBridge(Node):
    def __init__(self, video="jpeg"):
        super().__init__("rerun_bridge")
        self.path = []
        self.joints = {}
        self.last_joint_transform_time = 0.0
        self.create_subscription(Log, "/rosout", self.on_rosout, QoSProfile(depth=100))
        # start at the current end, not 0 - call_log.py's audit log is append-only and keeps
        # every past session, so reading from the start on each bridge restart would replay
        # old calls under their real (old) timestamps, dragging the recording's whole time
        # range back to whenever the log began instead of showing only what's new.
        self.mcp_log_pos = MCP_LOG_PATH.stat().st_size if MCP_LOG_PATH.is_file() else 0
        self.create_timer(MCP_LOG_POLL_PERIOD, self.poll_mcp_log)
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self.on_odom, 20)
        self.create_subscription(
            TwistStamped, "/diff_drive_controller/cmd_vel", self.on_cmd_vel_final, 20)
        self.create_subscription(TwistStamped, "/cmd_vel_nav", self.on_cmd_vel_nav, 20)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 20)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_subscription(LaserScan, "/ldlidar_node/scan", self.on_scan, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(
            ParticleCloud, "/particle_cloud", self.on_particles,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.map_tf_time = 0.0
        self.create_subscription(TFMessage, "/tf", lambda m: self.on_tf(m, False), QoSProfile(depth=100))
        self.create_subscription(
            TFMessage, "/tf_static", lambda m: self.on_tf(m, True),
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(Path, "/local_plan", self.on_local_plan, 10)
        # robot_state_publisher latches the description, so match its durability.
        self.create_subscription(
            String, "/robot_description", self.on_robot_description,
            QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE))
        self.video_subs = {}
        self.video_started = set()
        self.video_lags = {}       # path -> arrival minus stamp of the first packets, seconds
        self.video_restamp = {}    # path -> True: place packets at arrival, not at their stamp
        self.video_mode = video
        if video == "jpeg" or FFMPEGPacket is not None:
            self.create_timer(2.0, self.find_cameras)

        # Camera intrinsics (rr.Pinhole), so the 3D view draws a frustum at each camera's real
        # mounted pose and projects the live video stream through it - logged once per camera, as
        # soon as its intrinsics are real (camera_info publishes an all-zero placeholder before
        # calibration, which would draw a degenerate frustum - skipped below until fx/fy are
        # non-zero). The camera's optical frame (msg.header.frame_id, e.g.
        # head_camera_optical_frame) already has a static Transform3D from the URDF
        # (UrdfTree.log_urdf_to_recording logs one per joint, fixed or not, at /tf_static) - giving
        # Pinhole that same name as its own child_frame would make /cameras/<name> a SECOND static
        # definition of that already-owned frame ("Two entities define the same child frame").
        # CoordinateFrame doesn't help either: it relabels an entity's own coordinate values, it
        # doesn't give the entity a resolvable pose. What actually works: set Pinhole's
        # parent_frame to the real URDF frame and leave child_frame at its default (the entity's
        # own implicit "tf#/cameras/<name>") - a brand new, uniquely-named frame, parented to the
        # real one. No identity ever gets redefined, and any number of children can share one
        # parent, so nothing conflicts.
        self.pinhole_logged = set()
        for name in CAMERAS:
            self.create_subscription(
                CameraInfo, f"cameras/{name}/camera_info",
                lambda msg, name=name: self.on_camera_info(msg, name), qos_profile_sensor_data)

        # Depth, logged to the same entity as that camera's Pinhole/video (both are
        # head_color_optical_frame - depth is hardware-registered to color by default, confirmed
        # live via camera_info). rr.DepthImage under a Pinhole ancestor back-projects into a
        # colored 3D point cloud automatically on the viewer side - already covers "point cloud"
        # for free, without this bridge parsing depth/points (PointCloud2, ~920k XYZ points/frame)
        # off the Jetson itself.
        self.last_depth_log_time = {}
        for name in DEPTH_CAMERAS:
            self.create_subscription(
                Image, f"cameras/{name}/depth/image_raw",
                lambda msg, name=name: self.on_depth_image(msg, name), qos_profile_sensor_data)

    def on_camera_info(self, msg: CameraInfo, name):
        if name in self.pinhole_logged:
            return
        fx, fy = msg.k[0], msg.k[4]
        if fx <= 0.0 or fy <= 0.0:
            return  # uncalibrated placeholder (all-zeros) - nothing real to draw yet
        rr.log(
            f"cameras/{name}",
            rr.Pinhole(
                resolution=[msg.width, msg.height], focal_length=[fx, fy],
                principal_point=[msg.k[2], msg.k[5]], camera_xyz=rr.ViewCoordinates.RDF,
                parent_frame=msg.header.frame_id),
            static=True)
        self.pinhole_logged.add(name)
        self.get_logger().info(f"logged camera frustum for {name} (fx={fx:.1f}, fy={fy:.1f})")

    def on_depth_image(self, msg: Image, name):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_depth_log_time.get(name, 0.0) < DEPTH_LOG_PERIOD:
            return
        self.last_depth_log_time[name] = now
        if msg.encoding != "16UC1":
            self.get_logger().warning(f"cameras/{name}/depth: unexpected encoding {msg.encoding!r}, skipping")
            return
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        self.set_time(msg.header)
        rr.log(f"cameras/{name}", rr.DepthImage(depth, meter=DEPTH_METER))

    def on_rosout(self, msg: Log):
        stamp = msg.stamp.sec + msg.stamp.nanosec / 1e9
        rr.set_time("ros_time", timestamp=stamp if stamp > 0.0 else self.get_clock().now().nanoseconds / 1e9)
        rr.log(
            f"logs/rosout/{msg.name}",
            rr.TextLog(msg.msg, level=ROSOUT_LEVELS.get(msg.level, rr.TextLogLevel.INFO)))

    def poll_mcp_log(self):
        """Tail logs/mcp_calls.jsonl (mcp_server/call_log.py's audit trail) for new lines. A
        plain file read, not a subscription - the MCP server has no ROS dependency, and this
        stays true if it ever runs on a different machine than the bridge (it won't find the
        file then, and the Logs tab just has nothing from that source - no hard dependency)."""
        if not MCP_LOG_PATH.is_file():
            return
        with open(MCP_LOG_PATH) as f:
            f.seek(self.mcp_log_pos)
            new_lines = f.readlines()
            self.mcp_log_pos = f.tell()
        for line in new_lines:
            try:
                entry = json.loads(line)
            except ValueError:
                continue
            stamp = datetime.datetime.fromisoformat(entry["time"]).timestamp()
            rr.set_time("ros_time", timestamp=stamp)
            ok = entry.get("ok")
            level = rr.TextLogLevel.INFO if ok else rr.TextLogLevel.ERROR
            # one line each for the call, why it was made, and what came back, so the three
            # read as distinct fields rather than one run-on line - this is the whole point of
            # call_log.py's record() (tool/args/reason/result/error), so the Logs tab should
            # show it the same way, not flatten it.
            text = (
                f"CALL {entry['tool']}({entry.get('args')})\n"
                f"  reason: {entry.get('reason')}\n"
                f"  {'result' if ok else 'ERROR'}: {entry.get('result') if ok else entry.get('error')}"
                f"  ({entry.get('duration_s')}s)"
            )
            rr.log(f"logs/mcp/{entry['tool']}", rr.TextLog(text, level=level))

    def set_time(self, header):
        stamp = stamp_seconds(header)
        if stamp <= 0.0:  # a publisher that leaves the stamp empty
            stamp = self.get_clock().now().nanoseconds / 1e9
        rr.set_time("ros_time", timestamp=stamp)

    def find_cameras(self):
        for topic, types in self.get_topic_names_and_types():
            if "/depth/" in topic:
                # the orbbec driver exposes an ffmpeg_image_transport encoding of depth too, but
                # squeezing 16-bit depth through an 8-bit video codec isn't a valid video - depth
                # is logged properly (raw 16UC1) by on_depth_image instead.
                continue
            if topic in self.video_subs:
                continue
            if self.video_mode == "jpeg":
                # cameras.launch.py's throttled JPEG copy, <camera>/viewer/compressed
                if topic.endswith("/viewer/compressed") and COMPRESSED_TYPE in types:
                    path = topic.strip("/").removesuffix("/viewer/compressed")
                    self.video_subs[topic] = self.create_subscription(
                        CompressedImage, topic, lambda msg, path=path: self.on_jpeg(msg, path),
                        qos_profile_sensor_data)
                    self.get_logger().info(f"logging JPEG frames from {topic} to {path}")
                continue
            if FFMPEGPacket is not None and FFMPEG_TYPE in types:
                # /cameras/left/image_raw/ffmpeg -> cameras/left
                path = topic.strip("/").removesuffix("/image_raw/ffmpeg").removesuffix("/ffmpeg")
                self.video_subs[topic] = self.create_subscription(
                    FFMPEGPacket, topic, lambda msg, path=path: self.on_video(msg, path),
                    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
                self.get_logger().info(f"logging video from {topic} to {path}")

    def on_video(self, msg, path):
        keyframe = bool(msg.flags & AV_PKT_FLAG_KEY)
        if path not in self.video_started:
            # a stream can only start on a keyframe, it carries the codec headers
            if not keyframe:
                return
            # encoding is "codec;source_pixel_format;...", the first token is the codec
            hevc = msg.encoding.split(";")[0].lower() in ("hevc", "h265")
            rr.log(
                path, rr.VideoStream(codec=rr.VideoCodec.H265 if hevc else rr.VideoCodec.H264),
                static=True)
            self.video_started.add(path)
        self.set_video_time(msg.header, path)
        rr.log(path, rr.VideoStream.from_fields(sample=bytes(msg.data), is_keyframe=keyframe))

    def on_jpeg(self, msg, path):
        self.set_video_time(msg.header, path)
        rr.log(path, rr.EncodedImage(contents=bytes(msg.data), media_type="image/jpeg"))

    def set_video_time(self, header, path):
        """Like set_time, but a stream whose stamps run well behind their arrival is placed at
        arrival instead. usb_cam 0.8.1 stamps every frame early by a fixed 0-1 s, set when the
        node starts: its epoch shift (utils.hpp, get_epoch_time_shift_us) adds the start time's
        microseconds divided by 1000. On the timeline those frames sit behind the scan and the
        joints, so the viewer, following the newest time, waits for video that never comes and
        shows the last frame faded with a spinner. Decided per stream on its first 30 packets."""
        now = self.get_clock().now().nanoseconds / 1e9
        restamp = self.video_restamp.get(path)
        if restamp is None:
            lags = self.video_lags.setdefault(path, [])
            lags.append(now - stamp_seconds(header))
            # Until decided, the stamp: switching to arrival later only moves samples forward,
            # and a video stream's samples must not go back in time.
            restamp = False
            if len(lags) == 30:
                lag = sorted(lags)[15]
                restamp = self.video_restamp[path] = lag > 0.25
                if restamp:
                    self.get_logger().warn(
                        f"{path}: stamps run {lag * 1000:.0f} ms behind arrival (usb_cam's "
                        f"start-time offset?), placing its video at arrival time")
        if restamp:
            rr.set_time("ros_time", timestamp=now)
        else:
            self.set_time(header)

    def on_robot_description(self, msg: String):
        if self.joints:
            return
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "robot.urdf")
            with open(path, "w") as f:
                f.write(msg.data)
            tree = rr.urdf.UrdfTree.from_file_path(path)
        tree.log_urdf_to_recording()
        self.joints = {j.name: j for j in tree.joints() if j.joint_type != "fixed"}
        self.get_logger().info(f"logged robot model, {len(self.joints)} movable joints")

    def on_odom(self, msg: Odometry):
        self.set_time(msg.header)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q)
        self.log_path_point(p.x, p.y, yaw)
        rr.log("odom/pose", rr.Points2D([[p.x, p.y]], colors=[(213, 94, 0)], radii=0.02))
        rr.log("odom/yaw", rr.Scalars(yaw))
        rr.log("velocity/odom_linear", rr.Scalars(msg.twist.twist.linear.x))
        rr.log("velocity/odom_angular", rr.Scalars(msg.twist.twist.angular.z))
        rr.log(
            "transforms/base",
            rr.Transform3D(
                translation=[p.x, p.y, p.z],
                rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                parent_frame=ODOM_FRAME, child_frame=BASE_FRAME))

    def log_path_point(self, x, y, yaw):
        """The full path is re-sent whenever it grows, so only add a point after real movement."""
        if self.path:
            lx, ly, lyaw = self.path[-1]
            turn = abs(math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw)))
            if math.hypot(x - lx, y - ly) < PATH_MIN_STEP and turn < PATH_MIN_TURN:
                return
        self.path.append((x, y, yaw))
        points = [(px, py) for px, py, _ in self.path]
        rr.log("odom/path", rr.LineStrips2D([points], colors=[(0, 114, 178)]))
        rr.log(
            "world/odom_path",
            rr.LineStrips3D([[(px, py, PLAN_Z) for px, py in points]], colors=[(0, 114, 178)], radii=0.01))

    def on_tf(self, msg: TFMessage, static):
        for t in msg.transforms:
            if t.header.frame_id != MAP_FRAME or t.child_frame_id != ODOM_FRAME:
                continue
            if not static:
                now = self.get_clock().now().nanoseconds / 1e9
                if now - self.map_tf_time < MAP_TF_PERIOD:
                    continue
                self.map_tf_time = now
                self.set_time(t.header)
            v, q = t.transform.translation, t.transform.rotation
            rr.log(
                "transforms/map_to_odom",
                rr.Transform3D(
                    translation=[v.x, v.y, v.z], rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                    parent_frame=MAP_FRAME, child_frame=ODOM_FRAME),
                static=static)

    def on_map(self, msg: OccupancyGrid):
        self.set_time(msg.header)
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        def cells(mask, stride):
            rows, cols = np.nonzero(mask)
            keep = (rows % stride == 0) & (cols % stride == 0)
            rows, cols = rows[keep], cols[keep]
            return np.column_stack([ox + (cols + 0.5) * res, oy + (rows + 0.5) * res, np.zeros(len(rows))])

        rr.log("world/map/occupied", rr.Points3D(cells(grid >= 65, 1), colors=[(240, 240, 240)], radii=res / 2))
        rr.log("world/map/free", rr.Points3D(cells(grid == 0, 2), colors=[(45, 70, 85)], radii=0.6 * res))

    def on_particles(self, msg: ParticleCloud):
        self.set_time(msg.header)
        points = [(p.pose.position.x, p.pose.position.y, 0.05) for p in msg.particles]
        rr.log("world/particles", rr.Points3D(points, colors=[(230, 159, 0)], radii=0.015))

    def on_scan(self, msg: LaserScan):
        self.set_time(msg.header)
        points, colors = [], []
        n = len(msg.ranges)
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):  # also drops inf and nan
                continue
            a = msg.angle_min + i * msg.angle_increment
            points.append((r * math.cos(a), r * math.sin(a), 0.0))
            colors.append(tuple(int(255 * c) for c in colorsys.hsv_to_rgb(0.8 * i / n, 1.0, 1.0)))
        rr.log("lidar/scan", rr.Points3D(points, colors=colors, radii=0.012))

    def on_goal(self, msg: PoseStamped):
        self.set_time(msg.header)
        p = msg.pose.position
        yaw = yaw_from_quat(msg.pose.orientation)
        rr.log("odom/goal", rr.Points2D([[p.x, p.y]], colors=[GOAL_COLOR], radii=0.012))
        rr.log("world/goal", rr.Arrows3D(
            origins=[[p.x, p.y, PLAN_Z]],
            vectors=[[0.4 * math.cos(yaw), 0.4 * math.sin(yaw), 0.0]],
            colors=[GOAL_COLOR], radii=0.02, labels=["goal"]))
        self.get_logger().info(f"goal ({p.x:.2f}, {p.y:.2f}) heading {math.degrees(yaw):.0f} deg")

    def on_plan(self, msg: Path):
        self.log_path_msg(msg, "world/plan", "odom/plan", PLAN_COLOR, 0.012)

    def on_local_plan(self, msg: Path):
        self.log_path_msg(msg, "world/local_plan", None, LOCAL_PLAN_COLOR, 0.008)

    def log_path_msg(self, msg: Path, entity_3d, entity_2d, color, radius):
        if not msg.poses:
            return
        self.set_time(msg.header)
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        rr.log(entity_3d, rr.LineStrips3D(
            [[(x, y, PLAN_Z) for x, y in points]], colors=[color], radii=radius))
        if entity_2d:
            rr.log(entity_2d, rr.LineStrips2D([points], colors=[color], radii=0.01))

    def on_cmd_vel_final(self, msg: TwistStamped):
        self.set_time(msg.header)
        rr.log("velocity/cmd_linear", rr.Scalars(msg.twist.linear.x))
        rr.log("velocity/cmd_angular", rr.Scalars(msg.twist.angular.z))

    def on_cmd_vel_nav(self, msg: TwistStamped):
        self.set_time(msg.header)
        rr.log("velocity/nav_linear", rr.Scalars(msg.twist.linear.x))
        rr.log("velocity/nav_angular", rr.Scalars(msg.twist.angular.z))

    def on_joint_states(self, msg: JointState):
        self.set_time(msg.header)
        for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
            if name in ("left_wheel_joint", "right_wheel_joint"):
                rr.log(f"wheels/{name}/velocity", rr.Scalars(velocity))
        # the 16 movable joints each need their own Transform3D log call (one rr.log() per
        # joint, there's no batched multi-entity send), and /joint_states arrives at 100 Hz -
        # unthrottled that's ~1600 log calls/s from this callback alone, which was the actual
        # cause of a measured 90s+ end-to-end latency (SDK-side gRPC sink and the viewer's own
        # ingest both backed up under the volume, confirmed via the viewer's own latency
        # breakdown - not network, both machines' clocks and ROS2 transport measured at ~70ms).
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_joint_transform_time < JOINT_TRANSFORM_PERIOD:
            return
        self.last_joint_transform_time = now
        for name, position in zip(msg.name, msg.position):
            if name in self.joints:
                rr.log(f"transforms/{name}", self.joints[name].compute_transform(position))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--save", metavar="FILE.rrd", help="record to a file instead of opening the viewer")
    parser.add_argument("--connect", metavar="URL",
                        help="send to a running viewer, e.g. rerun+http://laptop:9876/proxy")
    parser.add_argument("--cameras", default=",".join(CAMERAS),
                        help="camera views in the layout, comma separated, empty for none")
    parser.add_argument("--video", choices=("jpeg", "h264"), default="jpeg",
                        help="jpeg (default): each camera's <camera>/viewer/compressed, JPEG frames "
                             "throttled on the robot (cameras.yaml viewer_fps), every frame stands "
                             "alone. h264: <camera>/image_raw/ffmpeg at the full rate, about a third "
                             "of the bandwidth, but the viewer's decoder lags a live stream and "
                             "shows it faded with a spinner much of the time")
    args, ros_args = parser.parse_known_args()

    # so a backgrounded run (SIGINT ignored) still flushes the recording on kill
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    rr.init("xlerobot", spawn=args.save is None and args.connect is None)
    if args.save:
        rr.save(args.save)
    elif args.connect:
        rr.connect_grpc(args.connect)
    rr.send_blueprint(blueprint([name for name in args.cameras.split(",") if name]))
    log_static_setup()
    rclpy.init(args=ros_args)
    node = RerunBridge(video=args.video)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # rclpy's own signal handler may have shut the context down already
            rclpy.shutdown()


if __name__ == "__main__":
    main()
