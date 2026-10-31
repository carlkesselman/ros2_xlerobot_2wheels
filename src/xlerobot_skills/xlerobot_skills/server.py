#!/usr/bin/env python3
"""The robot's skills as ROS 2 actions with one result contract (ADR-0006).

  ros2 launch xlerobot_skills skills.launch.py
  ros2 action send_goal /skills/look_at xlerobot_skill_msgs/action/LookAt "{named_position: head_look_down}" --feedback

Actions under /skills: navigate_to, look_at, detect, move_arm, approach. Every motion checks the operator presence
heartbeat (/mcp_presence_ok) and the safety supervisor's /e_stop while it runs: either one stops it, e-stops the servo
bus it uses and returns SAFETY_STOP. A cancel returns PREEMPTED. Two motions on one servo bus (or two base motions)
at once are refused.
"""
import math
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, TwistStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningOptions, PlanningScene, PlanningSceneComponents, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from vision_msgs.msg import Detection3DArray
from xlerobot_object_perception.srv import DetectObjects
from xlerobot_skill_msgs.action import Approach, Detect, LookAt, MoveArm, NavigateTo
from xlerobot_skill_msgs.msg import SkillResult

from . import geometry as geo
from . import motion

R = SkillResult
HEAD_JOINTS = ["head_pan_joint", "head_tilt_joint"]
HEAD_SPEED = 0.8              # rad/s for head moves sent straight to the head controller
BASE_FRAME = "base_footprint"


class SkillError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code, self.message = code, message


class SkillServer(Node):
    def __init__(self):
        super().__init__("skill_server")
        P = self.declare_parameter
        self.positions_file = P("positions_file", "").value or motion.default_store("saved_positions.yaml")
        self.waypoints_file = P("waypoints_file", "").value or motion.default_store("saved_waypoints.yaml")
        self.camera_frame = P("head_camera_frame", "").value
        self.look_tol = math.radians(float(P("look_at_tolerance_deg", 2.0).value))
        self.approach_height = float(P("approach_height_m", 0.05).value)
        self.gate = float(P("association_gate_m", 0.05).value)
        self.nav_timeout = float(P("nav_timeout_s", 120.0).value)
        self.arm_timeout = float(P("arm_timeout_s", 60.0).value)
        self.default_scaling = float(P("default_scaling", 1.0).value)
        self.table_box = bool(P("table_box", True).value)
        self.table_margin = float(P("table_margin_m", 0.025).value)
        self.correct_tol = float(P("tip_correction_tolerance_m", 0.005).value)
        self.correct_steps = int(P("tip_correction_steps", 2).value)
        self.camera_up_roll = {a: float(P(f"camera_up_wrist_roll_{a}", 1.5708).value) for a in ("left", "right")}
        self.roll_tol = float(P("camera_up_tolerance", 0.6).value)
        self.tip = {a: [float(v) for v in P(f"tip_offset_{a}", [0.0, 0.0, 0.0]).value] for a in ("left", "right")}

        self.joints = {}
        self.presence_ok = False
        self.estop_active = False
        self.urdf = None
        self.head = None
        self.resources = {}               # bus or "base" -> skill name holding it
        self.res_lock = threading.Lock()

        state = MutuallyExclusiveCallbackGroup()
        self.cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10, callback_group=state)
        self.create_subscription(Bool, "/mcp_presence_ok", lambda m: setattr(self, "presence_ok", m.data), 10, callback_group=state)
        self.create_subscription(Bool, "/e_stop", lambda m: setattr(self, "estop_active", m.data), 10, callback_group=state)
        self.create_subscription(String, "/robot_description", self._on_urdf, latched, callback_group=state)
        if not self.camera_frame:
            self.create_subscription(CameraInfo, "/cameras/head/camera_info", self._on_camera_info,
                                     qos_profile_sensor_data, callback_group=state)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.move_group = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb)
        self.execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory", callback_group=self.cb)
        self.validity = self.create_client(GetStateValidity, "/check_state_validity", callback_group=self.cb)
        self.ik = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb)
        self.fk = self.create_client(GetPositionFK, "/compute_fk", callback_group=self.cb)
        self.get_scene = self.create_client(GetPlanningScene, "/get_planning_scene", callback_group=self.cb)
        self.apply_scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene", callback_group=self.cb)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose", callback_group=self.cb)
        self.head_ctl = ActionClient(self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory",
                                     callback_group=self.cb)
        self.perception = self.create_client(DetectObjects, "/perception/detect", callback_group=self.cb)
        self.estop_clients = {b: self.create_client(SetBool, f"/emergency_stop_{b}", callback_group=self.cb)
                              for b in ("xlerobot_bus1", "xlerobot_bus2")}
        self.cmd_pub = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)

        for name, action, fn in (("navigate_to", NavigateTo, self.navigate_to), ("look_at", LookAt, self.look_at),
                                 ("detect", Detect, self.detect), ("move_arm", MoveArm, self.move_arm),
                                 ("approach", Approach, self.approach)):
            ActionServer(self, action, f"/skills/{name}", execute_callback=self._wrap(name, action, fn),
                         goal_callback=lambda g: GoalResponse.ACCEPT,
                         cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=self.cb)
        self.get_logger().info("skills ready: /skills/{navigate_to, look_at, detect, move_arm, approach}")

    # ---- state ----
    def _on_joints(self, m):
        self.joints.update(zip(m.name, m.position))

    def _on_urdf(self, m):
        self.urdf = m.data
        try:
            self.head = geo.joint_info(m.data, HEAD_JOINTS)
        except ValueError as e:
            self.get_logger().warning(f"no head in the robot description: {e}")

    def _on_camera_info(self, m):
        self.camera_frame = self.camera_frame or m.header.frame_id

    def _safe(self):
        if self.estop_active:
            return "the safety supervisor's e-stop is active"
        if not self.presence_ok:
            return "no operator presence heartbeat (/mcp_presence_ok)"
        return None

    # ---- plumbing ----
    def _wrap(self, name, action, fn):
        def execute(goal_handle):
            result = action.Result()
            try:
                fn(goal_handle, result)
                result.result.code, result.result.message = R.SUCCEEDED, result.result.message or "done"
            except SkillError as e:
                result.result.code, result.result.message = e.code, e.message
            except Exception as e:                              # noqa: BLE001
                self.get_logger().error(f"{name}: {e!r}")
                result.result.code, result.result.message = R.FAILED, f"{type(e).__name__}: {e}"
            if result.result.code == R.SUCCEEDED:
                goal_handle.succeed()
            elif result.result.code == R.PREEMPTED and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self.get_logger().info(f"{name}: {result.result.code} {result.result.message}")
            return result
        return execute

    def _claim(self, skill, resources):
        with self.res_lock:
            busy = [r for r in resources if r in self.resources]
            if busy:
                raise SkillError(R.FAILED, f"{', '.join(busy)} in use by {self.resources[busy[0]]}")
            for r in resources:
                self.resources[r] = skill

    def _release(self, resources):
        with self.res_lock:
            for r in resources:
                self.resources.pop(r, None)

    def _wait(self, future, timeout, goal_handle=None, guard=True, on_stop=None):
        """Wait for a future while watching cancel, safety and the timeout. on_stop() runs before raising."""
        done = threading.Event()
        future.add_done_callback(lambda f: done.set())
        deadline = time.monotonic() + timeout
        while not done.wait(0.05):
            if goal_handle is not None and goal_handle.is_cancel_requested:
                on_stop and on_stop(False)
                raise SkillError(R.PREEMPTED, "cancelled")
            unsafe = guard and self._safe()
            if unsafe:
                on_stop and on_stop(True)
                raise SkillError(R.SAFETY_STOP, unsafe)
            if time.monotonic() > deadline:
                on_stop and on_stop(False)
                raise SkillError(R.TIMEOUT, f"no result within {timeout:.0f} s")
        return future.result()

    def _estop(self, buses):
        for b in buses:
            c = self.estop_clients.get(b)
            if c is not None and c.service_is_ready():
                c.call_async(SetBool.Request(data=True))

    def _stop_base(self):
        for _ in range(5):
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)

    def _lookup(self, target, source, timeout=1.0):
        try:
            return geo.transform_to_matrix(self.tf.lookup_transform(target, source, Time(), timeout=Duration(seconds=timeout)))
        except Exception as e:                                  # noqa: BLE001
            raise SkillError(R.FAILED, f"no transform {target} <- {source}: {e}")

    def _point_in(self, frame, ps):
        """PointStamped or PoseStamped -> numpy point in `frame` (latest transform)."""
        p = ps.point if isinstance(ps, PointStamped) else ps.pose.position
        return (self._lookup(frame, ps.header.frame_id) @ [p.x, p.y, p.z, 1.0])[:3]

    def _require_safe(self):
        unsafe = self._safe()
        if unsafe:
            raise SkillError(R.SAFETY_STOP, unsafe)

    def _send(self, client, goal, what, feedback=None, timeout=10.0):
        if not client.wait_for_server(timeout_sec=5.0):
            raise SkillError(R.FAILED, f"{what} is not available")
        handle = self._wait(client.send_goal_async(goal, feedback_callback=feedback), timeout, guard=False)
        if handle is None or not handle.accepted:
            raise SkillError(R.FAILED, f"{what} refused the goal")
        return handle

    # ---- MoveIt ----
    def _move_group(self, skill, goal_handle, request, group, timeout=None, feedback=None):
        """Plan, then execute: a planning failure is NOT_REACHABLE, a failure while moving is reported as such."""
        if request.max_velocity_scaling_factor <= 0.0:      # MoveIt's time parameterisation takes 0 as full speed
            request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = self.default_scaling
        bus = motion.GROUP_BUS[group]
        self._require_safe()
        self._claim(skill, [bus])
        try:
            handle = self._send(self.move_group, MoveGroup.Goal(request=request, planning_options=PlanningOptions(plan_only=True)),
                                "move_group (/move_action)")
            res = self._wait(handle.get_result_async(), 30.0, goal_handle, on_stop=lambda unsafe: handle.cancel_goal_async())
            if res.result.error_code.val != 1:
                contacts = self._start_state_contacts(group)
                if contacts:
                    raise SkillError(R.COLLISION, f"the start state is in collision: {contacts}")
                code = motion.moveit_code(res.result.error_code.val)
                raise SkillError(R.NOT_REACHABLE if code == R.FAILED else code,
                                 f"no motion plan (MoveIt error code {res.result.error_code.val})")
            if feedback is not None:
                feedback("moving")
            self._require_safe()
            handle = self._send(self.execute, ExecuteTrajectory.Goal(trajectory=res.result.planned_trajectory),
                                "move_group (/execute_trajectory)")

            def stop(unsafe):
                handle.cancel_goal_async()
                if unsafe:
                    self._estop([bus])
            res = self._wait(handle.get_result_async(), timeout or self.arm_timeout, goal_handle, on_stop=stop)
            if res.result.error_code.val != 1:
                raise SkillError(motion.moveit_code(res.result.error_code.val),
                                 f"failed while moving (MoveIt error code {res.result.error_code.val})")
        finally:
            self._release([bus])

    OCTOMAP = "<octomap>"

    def _scene(self, scene):
        if not (self.get_scene.wait_for_service(timeout_sec=2.0) and self.apply_scene.wait_for_service(timeout_sec=2.0)):
            raise SkillError(R.FAILED, "move_group's planning scene services are not available")
        res = self._wait(self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)), 5.0, guard=False)
        if not res.success:
            raise SkillError(R.FAILED, "move_group refused the planning scene change")

    def _near_table(self, arm, table_z, allow):
        """For a move to an object on the table (approach, touch): the jaws may touch the octomap, which at 5 cm voxels
        plus padding covers the target and the table under it; everything else, the wrist camera included, keeps
        avoiding it. The table also becomes a box at the height perception measured plus a margin for the arm's sag.
        Undone with allow=False."""
        req = GetPlanningScene.Request(components=PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))
        acm = self._wait(self.get_scene.call_async(req), 5.0, guard=False).scene.allowed_collision_matrix
        names = list(acm.entry_names)
        if self.OCTOMAP not in names:
            for e in acm.entry_values:
                e.enabled.append(False)
            names.append(self.OCTOMAP)
            acm.entry_names = names
            acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(names)))
        o = names.index(self.OCTOMAP)
        for i, link in enumerate(names):
            if link in (f"{arm}_arm_gripper_link", f"{arm}_arm_moving_jaw_link"):
                acm.entry_values[i].enabled[o] = allow
                acm.entry_values[o].enabled[i] = allow
        scene = PlanningScene(is_diff=True)
        scene.allowed_collision_matrix = acm
        table = CollisionObject(id="skill_table")
        table.header.frame_id = BASE_FRAME
        if allow and self.table_box:
            table.operation = CollisionObject.ADD
            table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.8, 1.2, 0.04])]
            table.primitive_poses = [Pose(position=Point(x=0.15 + 0.4, y=0.0, z=float(table_z) + self.table_margin - 0.02))]
        else:
            table.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [table]
        self._scene(scene)

    def _tip_in_base(self, arm):
        T = self._lookup(BASE_FRAME, motion.GROUP_TIP_LINK[f"{arm}_arm"])
        return (T @ [*self.tip[arm], 1.0])[:3]

    def _state(self, overrides=None):
        st = RobotState()
        js = dict(self.joints)
        js.update(overrides or {})
        st.joint_state.name = list(js)
        st.joint_state.position = [float(v) for v in js.values()]
        return st

    def _ik(self, arm, link_pos, R, seed, collisions=True):
        """Joints putting the gripper link at `link_pos` (position-only IK, seeded), or None."""
        group = f"{arm}_arm"
        req = GetPositionIK.Request()
        req.ik_request.group_name = group
        req.ik_request.ik_link_name = motion.GROUP_TIP_LINK[group]
        req.ik_request.robot_state = self._state(seed)
        req.ik_request.avoid_collisions = collisions
        req.ik_request.timeout = DurationMsg(sec=0, nanosec=300_000_000)
        ps = req.ik_request.pose_stamped
        ps.header.frame_id = BASE_FRAME
        ps.pose.position = Point(x=float(link_pos[0]), y=float(link_pos[1]), z=float(link_pos[2]))
        q = geo.matrix_to_quat(R)
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        res = self._wait(self.ik.call_async(req), 5.0, guard=False)
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return {j: sol[j] for j in motion.GROUP_JOINTS[group] if j in sol}

    def _fk(self, arm, joints):
        """4x4 of the gripper link (base frame) for these arm joints."""
        req = GetPositionFK.Request(fk_link_names=[motion.GROUP_TIP_LINK[f"{arm}_arm"]], robot_state=self._state(joints))
        req.header.frame_id = BASE_FRAME
        res = self._wait(self.fk.call_async(req), 5.0, guard=False)
        if res.error_code.val != 1 or not res.pose_stamped:
            raise SkillError(R.FAILED, f"forward kinematics failed ({res.error_code.val})")
        return geo.pose_to_matrix(res.pose_stamped[0].pose)

    def _solve_tip(self, arm, target, seed_overrides=None, collisions=True, iterations=8, tol=0.002):
        """Joints putting the tool point at `target`. Position-only IK can only place the gripper link, and where the
        link must be depends on the hand's orientation, which IK chooses: so alternate IK for the link and forward
        kinematics for the orientation until the tool point lands within `tol`. Returns (joints, error m) or (None, None)."""
        if not (self.ik.wait_for_service(timeout_sec=2.0) and self.fk.wait_for_service(timeout_sec=2.0)):
            raise SkillError(R.FAILED, "move_group's IK and FK services are not available")
        tip = np.asarray(self.tip[arm])
        seed = {j: self.joints[j] for j in motion.GROUP_JOINTS[f"{arm}_arm"] if j in self.joints}
        seed.update(seed_overrides or {})
        T = self._fk(arm, seed)
        best, best_err = None, None
        for _ in range(iterations):
            joints = self._ik(arm, np.asarray(target) - T[:3, :3] @ tip, T[:3, :3], seed, collisions)
            if joints is None:
                break
            joints.update(seed_overrides or {})                  # held joints (the wrist roll) stay where asked
            T = self._fk(arm, joints)
            err = float(np.linalg.norm(T[:3, :3] @ tip + T[:3, 3] - np.asarray(target)))
            if best_err is None or err < best_err:
                best, best_err = joints, err
            if err <= tol:
                break
            seed = joints
        return best, best_err

    def _move_tip(self, skill, goal_handle, arm, target, scaling, feedback=None, joints=None):
        """Put the arm's tool point at `target` (base frame): solve the joints (see _solve_tip), plan to them, then read
        the tool point back through TF (the arm stops short under gravity, which the encoders see) and re-solve from
        there for the remaining error."""
        group = f"{arm}_arm"
        overrides = {j: v for j, (v, _) in (joints or {}).items()}
        sol, err = self._solve_tip(arm, target, overrides)
        if sol is None or err > 0.01:
            free, _ = self._solve_tip(arm, target, overrides, collisions=False)
            if free is not None:
                contacts = self._contacts(group, free)
                raise SkillError(R.NOT_REACHABLE, f"reachable, but the pose is in collision: {contacts or 'unknown contact'}")
            raise SkillError(R.NOT_REACHABLE, "out of reach for the tool point")
        self._move_group(skill, goal_handle, motion.joint_goal_request(group, sol, scaling), group, feedback=feedback)
        goal = np.asarray(target, float)
        e = None
        for step in range(self.correct_steps + 1):
            time.sleep(0.4)                                      # let the servos settle and TF catch up
            e = np.asarray(target) - self._tip_in_base(arm)
            self.get_logger().info(f"{skill}: tool point {np.round(e * 1000).astype(int).tolist()} mm from the goal after move {step + 1}")
            if np.linalg.norm(e) <= self.correct_tol or step == self.correct_steps:
                break
            goal = goal + e
            sol, _ = self._solve_tip(arm, goal, overrides)       # seeded with where the arm is now
            if sol is None:
                break
            if feedback is not None:
                feedback("correcting")
            self._move_group(skill, goal_handle, motion.joint_goal_request(group, sol, scaling), group)
        return float(np.linalg.norm(e))

    def _contacts(self, group, joints):
        if not self.validity.wait_for_service(timeout_sec=1.0):
            return ""
        req = GetStateValidity.Request(group_name=group, robot_state=self._state(joints))
        res = self._wait(self.validity.call_async(req), 5.0, guard=False)
        return "" if res.valid else ", ".join(sorted({f"{c.contact_body_1} - {c.contact_body_2}" for c in res.contacts}))

    def _start_state_contacts(self, group):
        """'a - b, c - d' when MoveIt finds the current state in collision, else ''. MoveIt reports a refused start state
        only as a generic failure, so this tells the planner why."""
        if not self.validity.wait_for_service(timeout_sec=1.0):
            return ""
        req = GetStateValidity.Request(group_name=group, robot_state=RobotState())
        req.robot_state.joint_state.name = list(self.joints)
        req.robot_state.joint_state.position = [float(v) for v in self.joints.values()]
        try:
            res = self._wait(self.validity.call_async(req), 5.0, guard=False)
        except SkillError:
            return ""
        if res.valid:
            return ""
        return ", ".join(sorted({f"{c.contact_body_1} - {c.contact_body_2}" for c in res.contacts}))

    # ---- skills ----
    def navigate_to(self, goal_handle, result):
        g = goal_handle.request
        pose = PoseStamped()
        if g.waypoint:
            w = motion.load_waypoints(self.waypoints_file).get(g.waypoint)
            if w is None:
                raise SkillError(R.NOT_FOUND, f"no waypoint '{g.waypoint}'")
            pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y = float(w["x"]), float(w["y"])
            q = geo.yaw_quat(float(w["yaw"]))
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        else:
            pose = g.pose
            if not pose.header.frame_id:
                raise SkillError(R.FAILED, "neither a waypoint nor a pose with a frame")
        pose.header.stamp = self.get_clock().now().to_msg()
        self._require_safe()
        self._claim("navigate_to", ["base"])
        fb = NavigateTo.Feedback()
        try:
            fb.phase = "waiting_for_nav2"
            goal_handle.publish_feedback(fb)

            def on_fb(m):
                fb.phase = "recovering" if m.feedback.number_of_recoveries else "driving"
                fb.distance_remaining_m = float(m.feedback.distance_remaining)
                goal_handle.publish_feedback(fb)
            handle = self._send(self.nav, NavigateToPose.Goal(pose=pose), "Nav2 (/navigate_to_pose)", feedback=on_fb)

            def stop(unsafe):
                handle.cancel_goal_async()
                self._stop_base()
            res = self._wait(handle.get_result_async(), g.timeout_s or self.nav_timeout, goal_handle, on_stop=stop)
            if res.status != GoalStatus.STATUS_SUCCEEDED:
                err = getattr(res.result, "error_msg", "") or getattr(res.result, "error_code", "")
                raise SkillError(R.PREEMPTED if res.status == GoalStatus.STATUS_CANCELED else R.NOT_REACHABLE,
                                 f"Nav2 ended with status {res.status} {err}".strip())
            result.result.message = f"arrived at {g.waypoint or 'the pose'}"
        finally:
            self._release(["base"])

    def _head_to(self, goal_handle, q, timeout=10.0):
        missing = [j for j in HEAD_JOINTS if j not in self.joints]
        if missing:
            raise SkillError(R.FAILED, f"no joint state for {', '.join(missing)}")
        now = np.array([self.joints[j] for j in HEAD_JOINTS])
        t = max(0.4, float(np.max(np.abs(np.asarray(q) - now))) / HEAD_SPEED)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = HEAD_JOINTS
        goal.trajectory.points = [JointTrajectoryPoint(positions=[float(v) for v in q],
                                                       time_from_start=DurationMsg(sec=int(t), nanosec=int((t % 1) * 1e9)))]
        handle = self._send(self.head_ctl, goal, "the head controller")

        def stop(unsafe):
            handle.cancel_goal_async()
            if unsafe:
                self._estop([motion.GROUP_BUS["head"]])
        res = self._wait(handle.get_result_async(), t + timeout, goal_handle, on_stop=stop)
        if res.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise SkillError(R.FAILED, f"head controller error {res.result.error_code} {res.result.error_string}")
        time.sleep(0.3)                                          # let /joint_states and TF catch up

    def look_at(self, goal_handle, result):
        g = goal_handle.request
        self._require_safe()
        bus = motion.GROUP_BUS["head"]
        self._claim("look_at", [bus])
        fb = LookAt.Feedback()
        try:
            if g.named_position:
                entry = motion.load_positions(self.positions_file).get(g.named_position)
                if entry is None or entry.get("group") != "head":
                    raise SkillError(R.NOT_FOUND, f"no head position '{g.named_position}'")
                fb.phase = "moving"
                goal_handle.publish_feedback(fb)
                self._head_to(goal_handle, [entry["joints"][j] for j in HEAD_JOINTS])
                result.result.message = f"head at '{g.named_position}'"
                return
            if self.head is None or not self.camera_frame:
                raise SkillError(R.FAILED, "no robot description or head camera frame yet")
            tol = math.radians(g.tolerance_deg) if g.tolerance_deg > 0 else self.look_tol
            target = self._point_in(BASE_FRAME, g.target)
            lim = np.array([[self.head[j]["lower"], self.head[j]["upper"]] for j in HEAD_JOINTS])
            err = math.pi
            for _ in range(5):
                Tc = self._lookup(BASE_FRAME, self.camera_frame)
                axes = [self._lookup(BASE_FRAME, self.head[j]["child"])[:3, :3] @ self.head[j]["axis"] for j in HEAD_JOINTS]
                dq, err = geo.aim_step(Tc[:3, 3], Tc[:3, 2], target, axes)
                fb.phase, fb.error_deg = "aiming", math.degrees(err)
                goal_handle.publish_feedback(fb)
                if err < tol:
                    break
                q = np.array([self.joints[j] for j in HEAD_JOINTS]) + dq
                clipped = np.clip(q, lim[:, 0], lim[:, 1])
                if np.allclose(clipped, [self.joints[j] for j in HEAD_JOINTS], atol=1e-3):
                    break                                        # at a limit, nothing left to do
                self._head_to(goal_handle, clipped)
            result.error_deg = math.degrees(err)
            if err >= tol:
                raise SkillError(R.NOT_REACHABLE, f"the head points {math.degrees(err):.1f} deg off the target (at its limits)")
            result.result.message = f"looking at the target, {math.degrees(err):.1f} deg off"
        finally:
            self._release([bus])

    def _detect(self, goal_handle, prompt, label, size_m, frames, timeout=30.0):
        if not self.perception.wait_for_service(timeout_sec=3.0):
            raise SkillError(R.FAILED, "the object perception service (/perception/detect) is not running")
        req = DetectObjects.Request(prompt=prompt, label=label or prompt, size_mm=float(size_m) * 1000.0, frames=int(frames))
        res = self._wait(self.perception.call_async(req), timeout, goal_handle, guard=False)
        codes = list(res.codes)
        if not res.detections.detections:
            if {"BUSY", "BAD_PROMPT", "NO_FRAMES", "STALE_TF"} & set(codes):
                raise SkillError(R.FAILED, f"perception: {', '.join(codes)}")
            raise SkillError(R.NOT_FOUND, f"no '{prompt}' found ({', '.join(codes) or 'no detections'})")
        return res

    def detect(self, goal_handle, result):
        g = goal_handle.request
        fb = Detect.Feedback(phase="detecting")
        goal_handle.publish_feedback(fb)
        res = self._detect(goal_handle, g.prompt, g.label, g.size_m, g.frames)
        out = Detection3DArray()
        out.header = res.detections.header
        try:                                                     # map when localised
            T = self._lookup("map", res.detections.header.frame_id, timeout=0.2)
            out.header.frame_id = "map"
        except SkillError:
            T = None
        for d in res.detections.detections:
            if T is not None:
                M = T @ geo.pose_to_matrix(d.bbox.center)
                p = d.bbox.center.position
                p.x, p.y, p.z = (float(v) for v in M[:3, 3])
                yaw = geo.yaw_of_matrix(M[:3, :3])
                o = d.bbox.center.orientation
                o.x, o.y, o.z, o.w = geo.yaw_quat(yaw)
                if d.results:
                    d.results[0].pose.pose = d.bbox.center
                d.header.frame_id = "map"
            out.detections.append(d)
        result.detections = out
        result.heights_m = list(res.heights_m)
        result.result.message = f"{len(out.detections)} found, poses in {out.header.frame_id}"

    def move_arm(self, goal_handle, result):
        g = goal_handle.request
        if g.arm not in ("left", "right"):
            raise SkillError(R.FAILED, "arm must be left or right")
        fb = MoveArm.Feedback(phase="planning")
        goal_handle.publish_feedback(fb)
        if g.named_position:
            entry = motion.load_positions(self.positions_file).get(g.named_position)
            if entry is None:
                raise SkillError(R.NOT_FOUND, f"no saved position '{g.named_position}'")
            group = entry["group"]
            if not group.startswith(g.arm):
                raise SkillError(R.FAILED, f"'{g.named_position}' belongs to {group}, not the {g.arm} arm")
            request = motion.joint_goal_request(group, entry["joints"], g.scaling)
        else:
            group = f"{g.arm}_arm"
            if not g.point.header.frame_id:
                raise SkillError(R.FAILED, "neither a named position nor a point with a frame")
            request = motion.position_goal_request(group, BASE_FRAME, self._point_in(BASE_FRAME, g.point), g.scaling)
        def moving(phase):
            fb.phase = phase
            goal_handle.publish_feedback(fb)
        self._move_group("move_arm", goal_handle, request, group, g.timeout_s or None, feedback=moving)
        result.result.message = f"{group} at {g.named_position or 'the point'}"

    def approach(self, goal_handle, result):
        g = goal_handle.request
        if g.arm not in ("left", "right"):
            raise SkillError(R.FAILED, "arm must be left or right")
        ref = g.object
        fb = Approach.Feedback(phase="detecting")
        goal_handle.publish_feedback(fb)
        expected = self._point_in(BASE_FRAME, ref.pose)
        res = self._detect(goal_handle, ref.prompt, ref.prompt, ref.size_m, 0)
        gate = ref.gate_m or self.gate
        found = []
        for d in res.detections.detections:
            T = self._lookup(BASE_FRAME, d.header.frame_id or res.detections.header.frame_id)
            M = T @ geo.pose_to_matrix(d.bbox.center)
            found.append((float(np.linalg.norm(M[:2, 3] - expected[:2])), M, d))
        near = sorted([f for f in found if f[0] <= gate], key=lambda f: f[0])
        if not near:
            closest = min(f[0] for f in found)
            raise SkillError(R.NOT_FOUND, f"no '{ref.prompt}' within {gate * 100:.0f} cm of the reference (closest {closest * 100:.0f} cm)")
        if len(near) > 1:
            raise SkillError(R.AMBIGUOUS, f"{len(near)} '{ref.prompt}' within {gate * 100:.0f} cm of the reference")
        _, M, d = near[0]
        top = M[2, 3] + d.bbox.size.z / 2.0
        target = (M[0, 3], M[1, 3], top + (g.height_m or self.approach_height))
        fb.phase = "planning"
        goal_handle.publish_feedback(fb)

        def moving(phase):
            fb.phase = phase
            goal_handle.publish_feedback(fb)
        table_z = M[2, 3] - d.bbox.size.z / 2.0
        self._near_table(g.arm, table_z, True)
        try:
            roll = {f"{g.arm}_arm_wrist_roll_joint": (self.camera_up_roll[g.arm], self.roll_tol)}
            residual = self._move_tip("approach", goal_handle, g.arm, target, g.scaling, feedback=moving, joints=roll)
        finally:
            self._near_table(g.arm, table_z, False)
        out = PoseStamped()
        out.header.frame_id = ref.pose.header.frame_id or BASE_FRAME
        Mo = self._lookup(out.header.frame_id, BASE_FRAME) @ M
        out.pose.position = Point(x=float(Mo[0, 3]), y=float(Mo[1, 3]), z=float(Mo[2, 3]))
        o = out.pose.orientation
        o.x, o.y, o.z, o.w = geo.yaw_quat(geo.yaw_of_matrix(Mo[:3, :3]))
        result.object_pose = out
        above = (self._lookup(BASE_FRAME, f"{g.arm}_arm_wrist_camera_link")[2, 3]
                 - self._lookup(BASE_FRAME, motion.GROUP_TIP_LINK[f"{g.arm}_arm"])[2, 3])
        result.result.message = (f"{g.arm} claw tip {target[2] - top:.3f} m above the object, "
                                 f"{residual * 1000:.0f} mm from the goal by the joint encoders, wrist camera "
                                 f"{abs(above) * 100:.0f} cm {'above' if above > 0 else 'below'} the hand")


def main():
    rclpy.init()
    node = SkillServer()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
