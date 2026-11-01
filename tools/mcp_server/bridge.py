"""XleRobotBridge: one long-lived rclpy node backing every MCP tool in server.py.

Unlike tools/*.py, which each do rclpy.init()/shutdown() around a single action, this node is
created once and spun continuously in a background thread for the life of the MCP server
process. Tool calls run on FastMCP's own thread/event loop, not the executor's spin thread, so
they can't use rclpy.spin_until_future_complete (that would fight the background spin for the
same node) - instead every blocking call here uses a Future.add_done_callback() + threading.Event
pair: the callback (which runs on the executor thread) sets the event, the calling thread just
waits on it. See _wait_for() below.

Motion calls are serialized through motion_lock - the robot only does one thing at a time by
design, not a limitation worth working around.
"""
import datetime
import json
import math
import pathlib
import sys
import threading
import time

TOOLS_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS_DIR))

# Matches tools/capture_image.py's own convention (repo_root/captures/, gitignored - see
# .gitignore, ephemeral like /trajectories/, not curated).
CAPTURE_DIR = TOOLS_DIR.parent / "captures"

import cv2
import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, PlanningOptions
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from xlerobot_skill_msgs.action import Detect
from xlerobot_skill_msgs.srv import ExecutionControl, SubmitPlan, WaitForEvent
from tf2_ros import Buffer, TransformException, TransformListener

import position_store
import spatial_memory_math
import spatial_memory_store
import waypoint_store
from goto_position import MoveError, joint_goal_request
from move_to_pose import position_goal_request

CAMERAS = ["head", "left", "right"]
ESTOP_BUSES = ["xlerobot_bus1", "xlerobot_bus2"]
NAV_TERMINAL = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_CANCELED: "canceled",
                GoalStatus.STATUS_ABORTED: "aborted"}

# Camera name -> optical frame id, loaded once from the same config cameras.launch.py uses, so
# this never drifts from a second hardcoded copy.
_CAMERAS_YAML = TOOLS_DIR.parent / "src" / "xlerobot_bringup" / "config" / "cameras.yaml"
with open(_CAMERAS_YAML) as _f:
    CAMERA_FRAME_IDS = {name: c["frame_id"] for name, c in yaml.safe_load(_f)["cameras"].items()}


class PresenceLost(RuntimeError):
    """Raised when a motion-class tool is called, or a motion in flight is cancelled, because
    /mcp_presence_ok is false - see xlerobot_teleop/mcp_presence_guard.py."""


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class XleRobotBridge(Node):
    def __init__(self):
        super().__init__("mcp_bridge")
        self.cv_bridge = CvBridge()
        self.motion_lock = threading.Lock()

        # cached state, updated by subscription callbacks on the executor thread
        self.joint_positions = {}
        self.last_joint_state_time = None
        self.amcl_pose = None  # (x, y, yaw)
        self.presence_ok = False
        self.nav_status = None
        self.camera_info = {c: None for c in CAMERAS}  # last camera_info seen by capture_image

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        # AMCL publishes /amcl_pose with TRANSIENT_LOCAL durability (confirmed via `ros2 topic
        # info /amcl_pose --verbose`) - the default subscription QoS is VOLATILE, which is
        # incompatible and silently receives nothing (no error, no warning, just an empty
        # self.amcl_pose forever). Found by testing: get_robot_pose/save_waypoint/goto_waypoint's
        # localization-wait-loop would always time out even with AMCL fully up and localized.
        amcl_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, amcl_qos)
        self.create_subscription(Bool, "/mcp_presence_ok", self._on_presence, 10)
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self._on_nav_status, 10)

        self.move_group = ActionClient(self, MoveGroup, "/move_action")
        self.nav_action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.nav_cancel_client = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.cmd_pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.estop_clients = {b: self.create_client(SetBool, f"/emergency_stop_{b}") for b in ESTOP_BUSES}
        # the plan executive and the skills (ADR-0006)
        self.submit_client = self.create_client(SubmitPlan, "/executive/submit_plan")
        self.wait_client = self.create_client(WaitForEvent, "/executive/wait_for_event")
        self.control_client = self.create_client(ExecutionControl, "/executive/control")
        self.detect_action = ActionClient(self, Detect, "/skills/detect")

    # -- subscription callbacks (executor thread) --------------------------------------------

    def _on_joint_states(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))
        self.last_joint_state_time = time.monotonic()

    def _on_amcl_pose(self, msg):
        p = msg.pose.pose.position
        self.amcl_pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def _on_presence(self, msg):
        self.presence_ok = msg.data

    def _on_nav_status(self, msg):
        if msg.status_list:
            self.nav_status = msg.status_list[-1].status

    # -- generic async-future -> sync-call bridge (calling thread) ---------------------------

    def _wait_for(self, future, timeout):
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        if not event.wait(timeout=timeout):
            return None
        return future.result()

    def _require_presence(self):
        if not self.presence_ok:
            raise PresenceLost(
                "no presence heartbeat - hold/tap the presence button (see mcp_presence.yaml) "
                "before commanding motion")

    # -- estop (never presence-gated - always callable, including to recover from PresenceLost) --

    def estop(self, buses=None, active=True):
        buses = buses or ESTOP_BUSES
        results = {}
        futures = {}
        for bus in buses:
            client = self.estop_clients[bus]
            if not client.wait_for_service(timeout_sec=3.0):
                results[bus] = "service not available"
                continue
            futures[bus] = client.call_async(SetBool.Request(data=active))
        for bus, future in futures.items():
            result = self._wait_for(future, timeout=5.0)
            results[bus] = result.message if result else "no answer"
        return results

    def stop_base(self):
        if self.nav_cancel_client.wait_for_service(timeout_sec=1.0):
            self.nav_cancel_client.call_async(CancelGoal.Request())
        for _ in range(5):
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)

    # -- MoveGroup goals (goto_position, move_to_pose, move_gripper, play_sequence) ----------

    def _run_move_group(self, request, group, timeout=60.0):
        self._require_presence()
        with self.motion_lock:
            # server_is_ready() alone is an instant, no-wait check - right after the bridge
            # starts (before this client has discovered move_action at all) it false-negatives
            # even though move_group is genuinely up, since discovery takes a moment. Bounded
            # wait instead - only blocks the calling (tool) thread via a plain sleep loop, safe
            # to call alongside the background executor spinning this node (confirmed: rclpy's
            # own wait_for_server() does no spinning of its own).
            if not self.move_group.wait_for_server(timeout_sec=5.0):
                raise MoveError("/move_action not available - is move_group running?")
            options = PlanningOptions(plan_only=False)
            goal_msg = MoveGroup.Goal(request=request, planning_options=options)
            send_future = self.move_group.send_goal_async(goal_msg)
            handle = self._wait_for(send_future, timeout=15.0)
            if handle is None or not handle.accepted:
                raise MoveError("goal not accepted")

            result_future = handle.get_result_async()
            result_event = threading.Event()
            result_future.add_done_callback(lambda f: result_event.set())
            deadline = time.monotonic() + timeout
            while not result_event.is_set():
                if not self.presence_ok:
                    handle.cancel_goal_async()
                    result_event.wait(timeout=2.0)
                    self.estop(position_store.GROUP_BUSES.get(group, ESTOP_BUSES))
                    raise PresenceLost("presence lost mid-motion - goal cancelled, bus e-stopped")
                if time.monotonic() > deadline:
                    handle.cancel_goal_async()
                    raise MoveError(f"timed out after {timeout:.0f}s, goal cancelled")
                result_event.wait(timeout=0.1)

            result = result_future.result()
            code = result.result.error_code.val
            if code != MoveItErrorCodes.SUCCESS:
                raise MoveError(f"MoveIt error code {code}")
            return True

    def goto_position(self, name, scaling=0.0):
        entry = position_store.get(name)
        request = joint_goal_request(entry["group"], entry["joints"], scaling)
        self._run_move_group(request, entry["group"])
        return f"reached '{name}' ({entry['group']})"

    def move_to_pose(self, arm, x, y, z, frame="base_footprint", scaling=0.0):
        group = f"{arm}_arm"
        tip_link = position_store.GROUP_TIP_LINK[group]
        request = position_goal_request(group, tip_link, frame, (x, y, z), scaling)
        self._run_move_group(request, group)
        return f"{group} gripper landed near ({x:.3f}, {y:.3f}, {z:.3f}) in {frame}"

    def move_gripper(self, arm, action, scaling=0.0):
        if action not in ("open", "close"):
            raise ValueError("action must be 'open' or 'close'")
        group = f"{arm}_gripper"
        joint = position_store.GROUP_JOINTS[group][0]
        value = position_store.GRIPPER_OPEN if action == "open" else position_store.GRIPPER_CLOSED
        request = joint_goal_request(group, {joint: value}, scaling)
        self._run_move_group(request, group)
        return f"{arm} gripper {action}d"

    def play_sequence(self, names, loop="1", scaling=0.0):
        entries = [(name, position_store.get(name)) for name in names]
        forever = loop == "forever"
        count = 0 if forever else int(loop)
        if not forever and count < 1:
            raise ValueError("loop must be a positive integer or 'forever'")
        iteration = 0
        while forever or iteration < count:
            iteration += 1
            for name, entry in entries:
                request = joint_goal_request(entry["group"], entry["joints"], scaling)
                self._run_move_group(request, entry["group"])
        return f"played {' -> '.join(names)} x{iteration}"

    def save_position(self, name, group):
        joints = position_store.GROUP_JOINTS[group]
        deadline = time.time() + 5
        while not all(j in self.joint_positions for j in joints) and time.time() < deadline:
            time.sleep(0.1)
        missing = [j for j in joints if j not in self.joint_positions]
        if missing:
            raise MoveError(f"no /joint_states for: {', '.join(missing)}")
        positions = position_store.load()
        positions[name] = {
            "group": group,
            "joints": {j: round(float(self.joint_positions[j]), 5) for j in joints},
        }
        position_store.save(positions)
        return f"saved '{name}' ({group})"

    # -- navigation (goto_waypoint) -----------------------------------------------------------

    def goto_waypoint(self, name, timeout=120.0):
        self._require_presence()
        w = waypoint_store.get(name)
        with self.motion_lock:
            if not self.nav_action.wait_for_server(timeout_sec=10.0):
                raise MoveError("Nav2 is not up (no navigate_to_pose action server)")

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = w["x"], w["y"]
            goal.pose.orientation.z = math.sin(w["yaw"] / 2)
            goal.pose.orientation.w = math.cos(w["yaw"] / 2)
            self.nav_status = None
            self.goal_pub.publish(goal)

            t0 = time.monotonic()
            active = False
            while True:
                if not self.presence_ok:
                    self.stop_base()
                    raise PresenceLost("presence lost mid-navigation - goal cancelled")
                active = active or self.nav_status in (
                    GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
                if active and self.nav_status in NAV_TERMINAL:
                    reason = NAV_TERMINAL[self.nav_status]
                    break
                if not active and time.monotonic() - t0 > 5.0:
                    self.stop_base()
                    raise MoveError("Nav2 did not accept the goal within 5s")
                if time.monotonic() - t0 > timeout:
                    self.stop_base()
                    raise MoveError(f"timed out after {timeout:.0f}s, goal cancelled")
                time.sleep(0.05)
            if reason != "succeeded":
                raise MoveError(f"navigation {reason}")
            return f"arrived at '{name}'"

    def save_waypoint(self, name):
        deadline = time.time() + 5
        while self.amcl_pose is None and time.time() < deadline:
            time.sleep(0.1)
        if self.amcl_pose is None:
            raise MoveError("no /amcl_pose - is navigation.launch.py running with localization:=true?")
        x, y, yaw = self.amcl_pose
        waypoints = waypoint_store.load()
        waypoints[name] = {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)}
        waypoint_store.save(waypoints)
        return f"saved '{name}': x={x:.3f} y={y:.3f} yaw={yaw:.3f} rad"

    # -- read-only introspection (never presence-gated) --------------------------------------

    def list_positions(self):
        return position_store.load()

    def list_waypoints(self):
        return waypoint_store.load()

    def get_joint_states(self, group=None):
        joints = position_store.GROUP_JOINTS[group] if group else list(self.joint_positions)
        return {j: self.joint_positions.get(j) for j in joints}

    def get_robot_pose(self):
        if self.amcl_pose is None:
            return None
        x, y, yaw = self.amcl_pose
        return {"x": x, "y": y, "yaw": yaw, "frame": "map"}

    def _camera_pose_in_map(self, frame_id):
        """(pose_dict, None) or (None, error_str). Never raises: TF lookup failure (not
        localized yet, frame not in the tree yet) is an expected, common state right after
        bringup, not a bug. A tilting head camera needs the full 6DOF pose, not a yaw-only
        reduction the way the planar base pose is."""
        try:
            t = self.tf_buffer.lookup_transform("map", frame_id, rclpy.time.Time())
        except TransformException as e:
            return None, str(e)
        tr, q = t.transform.translation, t.transform.rotation
        return {"x": tr.x, "y": tr.y, "z": tr.z, "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}, None

    def get_presence(self):
        return self.presence_ok

    def get_status(self):
        """Best-effort liveness summary, NOT a torque-state check - sts_hardware_interface's own
        bus is owned by the running hardware interface while bringup is up, so there's no way to
        read the real torque-enable register from here without a serial-port conflict. The only
        ground truth for "is torque actually off" is tools/calibrate_arm.py status with the stack
        stopped - always use that after any estop, per standing practice, regardless of what this
        tool reports."""
        fresh = (self.last_joint_state_time is not None
                 and time.monotonic() - self.last_joint_state_time < 1.0)
        return {
            "presence_ok": self.presence_ok,
            "joint_states_fresh": fresh,
            "localized": self.amcl_pose is not None,
            "cameras_live": {c: self.count_publishers(f"cameras/{c}/image_raw") > 0 for c in CAMERAS},
        }

    def _compressed_topic(self, camera):
        # The Gemini driver publishes its JPEG stream under color/, the usb_cam nodes directly.
        candidates = [f"cameras/{camera}/image_raw/compressed", f"cameras/{camera}/color/image_raw/compressed"]
        for topic in candidates:
            if self.count_publishers(topic) > 0:
                return topic
        return candidates[0]

    def _grab_frame(self, camera, timeout=5.0):
        """One fresh (frame, camera_info) pair, or (None, None). Subscribes only for the duration
        of the call, to the JPEG stream: a standing subscription to the raw video pulled every
        frame over the network and stalled the Gemini driver, and a raw 1280x720 frame sent
        best-effort over Wi-Fi is thousands of UDP fragments, so losing any one drops the frame."""
        got = {}
        image_arrived = threading.Event()

        def on_image(msg):
            if "image" not in got:
                got["image"] = msg
                image_arrived.set()

        def on_info(msg):
            got.setdefault("info", msg)

        subs = [
            self.create_subscription(CompressedImage, self._compressed_topic(camera), on_image, 1),
            self.create_subscription(CameraInfo, f"cameras/{camera}/camera_info", on_info, qos_profile_sensor_data),
        ]
        try:
            if not image_arrived.wait(timeout):
                return None, None
            info_deadline = time.time() + 1.0
            while "info" not in got and time.time() < info_deadline:
                time.sleep(0.05)
            frame = cv2.imdecode(np.frombuffer(got["image"].data, np.uint8), cv2.IMREAD_COLOR)
            return frame, got.get("info")
        finally:
            for sub in subs:
                self.destroy_subscription(sub)

    def capture_image(self, camera, output_path):
        if camera not in CAMERAS:
            raise ValueError(f"camera must be one of {CAMERAS}")
        frame, camera_info = self._grab_frame(camera)
        if frame is None:
            raise MoveError(f"no frame from cameras/{camera} - is bringup running with cameras:=true?")
        self.camera_info[camera] = camera_info
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_path), frame)

        # Read live, not CAMERA_FRAME_IDS's static cameras.yaml value - the real frame_id
        # depends on which sensor:= is actually running (usbcam vs gemini vs a future
        # realsense), and camera_info's own header already carries it. Falls back to the
        # static value only in the unlikely case camera_info hasn't arrived yet even though
        # a frame has.
        frame_id = camera_info.header.frame_id if camera_info is not None else CAMERA_FRAME_IDS[camera]
        camera_pose_map, tf_error = self._camera_pose_in_map(frame_id)
        result = {
            "path": str(output_path),
            "width": frame.shape[1],
            "height": frame.shape[0],
            "camera": camera,
            "frame_id": frame_id,
            "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds"),
            "camera_pose_map": camera_pose_map,  # {x,y,z,qx,qy,qz,qw} in map frame, or None
            "robot_pose_map": self.get_robot_pose(),
        }
        if camera_pose_map is None:
            result["pose_note"] = f"camera pose unavailable (not localized yet?): {tf_error}"
        return result

    # -- plans (the executive, ADR-0006) -------------------------------------------------------

    def _service(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=3.0):
            raise MoveError(f"{client.srv_name} is not available - is the executive running (executive:=true)?")
        result = self._wait_for(client.call_async(request), timeout)
        if result is None:
            raise MoveError(f"{client.srv_name} did not answer within {timeout:.0f}s")
        return result

    def submit_plan(self, plan):
        self._require_presence()
        r = self._service(self.submit_client, SubmitPlan.Request(plan_json=json.dumps(plan)), 30.0)
        state = self.wait_for_event(after_seq=0, max_wait_s=0.0)
        return {"accepted": r.accepted, "plan_id": r.plan_id, "errors": list(r.errors),
                "last_seq": state["last_seq"] if r.accepted else None}

    def wait_for_event(self, after_seq=0, max_wait_s=25.0):
        max_wait_s = max(0.0, min(float(max_wait_s), 30.0))
        r = self._service(self.wait_client, WaitForEvent.Request(after_seq=int(after_seq), max_wait_s=max_wait_s),
                          max_wait_s + 10.0)
        return {"events": [json.loads(e) for e in r.events_json], "last_seq": r.last_seq, "state": json.loads(r.state_json)}

    def control_plan(self, command):
        if command == "resume":
            self._require_presence()
        r = self._service(self.control_client, ExecutionControl.Request(command=command), 15.0)
        if not r.ok:
            raise MoveError(r.message)
        return r.message

    def describe_scene(self, prompt, label="", camera="head"):
        """Detect objects for a prompt and draw them numbered on a fresh frame of the camera."""
        if not self.detect_action.wait_for_server(timeout_sec=3.0):
            raise MoveError("/skills/detect is not available - is the skill server running (skills:=true)?")
        handle = self._wait_for(self.detect_action.send_goal_async(Detect.Goal(prompt=prompt, label=label, frames=3)), 15.0)
        if handle is None or not handle.accepted:
            raise MoveError("detect refused the goal")
        res = self._wait_for(handle.get_result_async(), 60.0)
        if res is None:
            raise MoveError("detect did not finish within 60s")
        result = res.result
        frame, info = self._grab_frame(camera)
        dets = result.detections.detections
        objects = []
        for k, d in enumerate(dets, 1):
            p, sz = d.bbox.center.position, d.bbox.size
            objects.append({"id": k, "label": d.results[0].hypothesis.class_id if d.results else label or prompt,
                            "score": round(float(d.results[0].hypothesis.score), 2) if d.results else None,
                            "pose": [round(p.x, 4), round(p.y, 4), round(p.z, 4)], "frame": result.detections.header.frame_id,
                            "size_m": [round(sz.x, 4), round(sz.y, 4), round(sz.z, 4)]})
        out = {"objects": objects, "skill_result": result.result.message, "image": None}
        if frame is not None and info is not None and dets:
            try:
                tf = self.tf_buffer.lookup_transform(info.header.frame_id, result.detections.header.frame_id, rclpy.time.Time(),
                                                     timeout=rclpy.duration.Duration(seconds=1.0))
                t, q = tf.transform.translation, tf.transform.rotation
                x, y, z, w = q.x, q.y, q.z, q.w
                R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                              [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                              [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
                K, D = np.array(info.k).reshape(3, 3), np.array(info.d, float)
                for o in objects:
                    c, hs = np.array(o["pose"]), np.array(o["size_m"]) / 2.0
                    corners = np.array([[c[0] + sx * hs[0], c[1] + sy * hs[1], c[2] + sz * hs[2]]
                                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
                    cam = (R @ corners.T).T + [t.x, t.y, t.z]
                    if np.any(cam[:, 2] <= 0.01):
                        continue
                    uv, _ = cv2.projectPoints(cam.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D if np.any(D) else None)
                    hull = cv2.convexHull(uv.reshape(-1, 2).astype(np.float32)).astype(np.int32)
                    cv2.polylines(frame, [hull], True, (0, 0, 255), 2)
                    x0, y0 = hull.reshape(-1, 2).min(axis=0)
                    cv2.putText(frame, str(o["id"]), (int(x0), int(y0) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            except Exception as e:                            # noqa: BLE001  (the list is still useful without marks)
                out["image_note"] = f"marks not drawn: {e}"
        if frame is not None:
            path = CAPTURE_DIR / f"scene_{int(time.time())}.jpg"
            path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(path), frame)
            out["image"] = str(path)
        return out

    # -- spatial memory (remember_object/recall_objects/forget_object) -----------------------
    # Not presence-gated: this only reads cached sensor/TF state and writes a YAML file, it
    # doesn't move anything - same risk class as capture_image/get_status. The precedent that IS
    # gated (save_position/save_waypoint) is gated because that data later drives a motion
    # command directly; a wrong spatial-memory estimate is already covered at the actual motion
    # boundary (move_to_pose/goto_waypoint), gating the memory write would just duplicate that
    # protection for no benefit.

    def remember_object(self, label, description, camera, pixel_x=None, pixel_y=None,
                         plane_height_m=None):
        if camera not in CAMERAS:
            raise ValueError(f"camera must be one of {CAMERAS}")
        output_path = CAPTURE_DIR / f"{camera}_{int(time.time())}.png"
        capture = self.capture_image(camera, output_path)

        px = capture["width"] // 2 if pixel_x is None else pixel_x
        py = capture["height"] // 2 if pixel_y is None else pixel_y
        if not (0 <= px < capture["width"] and 0 <= py < capture["height"]):
            raise ValueError(f"pixel ({px},{py}) outside image bounds {capture['width']}x{capture['height']}")

        camera_info = self.camera_info[camera]
        ray = (spatial_memory_math.pixel_to_ray_camera_frame(camera_info, px, py)
               if camera_info is not None else None)

        observation = {
            "camera": camera, "frame_id": capture["frame_id"], "image_path": capture["path"],
            "timestamp": capture["timestamp"],
            "image_size": {"width": capture["width"], "height": capture["height"]},
            "pixel": {"x": px, "y": py},
            "camera_pose_map": capture["camera_pose_map"],
            "robot_pose_map": capture["robot_pose_map"],
            "ray_camera_frame": None if ray is None else {"x": float(ray[0]), "y": float(ray[1]), "z": float(ray[2])},
            "pose_available": capture["camera_pose_map"] is not None,
            "ray_available": ray is not None,
        }

        objects = spatial_memory_store.load()
        entry = objects.setdefault(label, {"created_at": observation["timestamp"], "observations": []})
        entry["description"] = description  # last-write-wins, this is working memory not curated config
        entry["updated_at"] = observation["timestamp"]
        entry["observations"].append(observation)
        entry["position_estimate"] = spatial_memory_math.estimate_position(
            entry["observations"],
            plane_height_m or spatial_memory_math.DEFAULT_PLANE_HEIGHT_M)
        objects[label] = entry
        spatial_memory_store.save(objects)
        return entry

    def recall_objects(self, label=None):
        if label is not None:
            return {label: spatial_memory_store.get(label)}
        return spatial_memory_store.load()

    def forget_object(self, label):
        removed = spatial_memory_store.delete(label)
        return {"forgot": label, "observations_removed": len(removed["observations"])}


_bridge = None
_executor = None
_init_lock = threading.Lock()


def get_bridge():
    """Lazily start rclpy + the bridge node + its background spin thread, once per process.
    FastMCP dispatches tool calls on worker threads, so two calls can race in here on the very
    first request - the lock makes the double-init (rclpy.init() twice) impossible rather than
    just unlikely.

    The brief sleep after starting the spin thread is a one-time cost, paid only by whichever
    tool call happens to be first: DDS discovery of the rest of the ROS graph (bringup's
    /joint_states, /mcp_presence_ok, ...) isn't instant, and a caller landing before it settles
    would see empty caches - not a hang, just a wrong-looking "nothing is alive yet" answer from
    the very first call. 1.0s was enough on a laptop with nothing else competing for CPU; on a
    Jetson Orin NX also running cameras + mock hardware + move_group, discovery sometimes took
    longer than that (confirmed live: joint_states came through by 1s but /mcp_presence_ok
    hadn't yet) - 2.5s gave consistent results there. Same one-time cost either way, paid once
    per server process lifetime."""
    global _bridge, _executor
    with _init_lock:
        if _bridge is None:
            rclpy.init()
            _bridge = XleRobotBridge()
            _executor = rclpy.executors.SingleThreadedExecutor()
            _executor.add_node(_bridge)
            threading.Thread(target=_executor.spin, daemon=True).start()
            time.sleep(2.5)
        return _bridge


def shutdown():
    """Call once, right before process exit (see server.py) - not needed if get_bridge() was
    never called (nothing to shut down). Tears down the executor/node/rclpy context cleanly, but
    a clean rclpy.shutdown() alone is NOT enough to avoid a crash on exit here: some rmw_fastrtps
    background thread doesn't finish joining before CPython's own interpreter teardown starts,
    which aborts the process ("terminate called without an active exception") even after this
    function returns successfully - confirmed reproducible with a textbook-correct explicit
    shutdown sequence, not a bug in this cleanup. Caller must follow this with os._exit() to skip
    that teardown entirely; every tool result is already delivered by this point regardless, so
    nothing is lost by skipping it."""
    if _bridge is None:
        return
    _executor.shutdown()
    _bridge.destroy_node()
    rclpy.shutdown()
