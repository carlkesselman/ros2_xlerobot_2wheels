"""The MuJoCo twin: runs the world from world.py in real time and stands in for the robot's
hardware over ROS 2 topics, like the Isaac Sim twin (docs/sim-twin.md), but on the Mac.

    python sim/mujoco/run_mujoco.py              headless
    mjpython sim/mujoco/run_mujoco.py --viewer   with MuJoCo's viewer (macOS needs mjpython for it)

Then, in another shell, bringup against it:

    ros2 launch xlerobot_bringup bringup.launch.py use_sim:=true sim_engine:=mujoco with_arms:=true with_head:=true

or both at once with sim/mujoco/mujoco_twin.launch.py.

Topics, matching xlerobot.ros2_control_sim.xacro's engine:=mujoco:
  /mujoco_wheel_commands      in   sensor_msgs/JointState, velocity, the two wheels
  /mujoco_position_commands   in   sensor_msgs/JointState, position, head and arms
  /mujoco_joint_states        out  every joint: position, velocity, effort (the actuator's force)
  /ldlidar_node/scan          out  the LD19's scan, what its driver publishes: 455 bins, 10 Hz,
                                   counter-clockwise in ldlidar_link; laser_filters makes /scan
  /mujoco/ground_truth        out  nav_msgs/Odometry, base_footprint in the world, for checking
                                   odometry and Nav2 against; nothing in the stack reads it
  /perception/detect          srv  xlerobot_object_perception/DetectObjects from the world's truth, in
                                   place of the camera pipeline (no camera in the twin yet): what
                                   matches the prompt within DETECT_RANGE of the robot, in base_footprint
  /mcp_presence_ok            out  with --presence, always true: the skill server moves only while
                                   an operator's heartbeat says someone is watching; the twin stands in
No /clock: the twin keeps to the wall clock, so nothing needs use_sim_time.
"""

import argparse
import math
import os
import sys
import threading
import time

import mujoco
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose
from xlerobot_object_perception.srv import DetectObjects

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world  # noqa: E402

JOINT_STATE_PERIOD = 0.005     # s, twice controller_manager's 100 Hz
SCAN_PERIOD = 0.1              # s, the LD19 at 10 Hz

# The LD19 as ldlidar.yaml configures its driver.
SCAN_BINS = 455
SCAN_RANGE_MIN = 0.30
SCAN_RANGE_MAX = 12.0
# Rays see the environment (geom groups 0-2), not the robot (world.ROBOT_GROUP).
SCAN_GEOMGROUP = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)

# Ground-truth detection: prompt word -> what in the world it finds. Boxes are geoms (crates, the table
# top), cubes are free bodies. Only what is this close to the robot (horizontally) is reported.
DETECT_RANGE = 1.5
DETECT_GEOMS = {"box": ("crate_",), "crate": ("crate_",), "table": ("table_top",)}
DETECT_BODIES = {"cube": ("cube",), "block": ("cube",)}


class MujocoTwin(Node):
    def __init__(self, model, data, lock, presence=False):
        super().__init__("mujoco_twin")
        self.m, self.d, self.lock = model, data, lock
        self.joints = [model.joint(j).name for j in range(model.njnt)
                       if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
        self.qpos_adr = {n: model.joint(n).qposadr[0] for n in self.joints}
        self.dof_adr = {n: model.joint(n).dofadr[0] for n in self.joints}
        self.actuator = {model.actuator(i).name: i for i in range(model.nu)}

        self.create_subscription(JointState, "/mujoco_wheel_commands", self.on_wheels, 10)
        self.create_subscription(JointState, "/mujoco_position_commands", self.on_positions, 10)
        self.state_pub = self.create_publisher(JointState, "/mujoco_joint_states", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/ldlidar_node/scan", qos_profile_sensor_data)
        self.truth_pub = self.create_publisher(Odometry, "/mujoco/ground_truth", 10)
        self.create_service(DetectObjects, "/perception/detect", self.on_detect)
        if presence:
            pub = self.create_publisher(Bool, "/mcp_presence_ok", 10)
            self.create_timer(0.2, lambda: pub.publish(Bool(data=True)))

        self.lidar = model.body("ldlidar_link").id
        self.base = model.body("base_footprint").id
        angles = np.arange(SCAN_BINS) * (2 * math.pi / SCAN_BINS)
        self.scan_dirs_local = np.stack([np.cos(angles), np.sin(angles), np.zeros(SCAN_BINS)], axis=1)
        self.scan_geomid = np.zeros(SCAN_BINS, dtype=np.int32)
        self.scan_dist = np.zeros(SCAN_BINS)

    def _set(self, msg, values, what):
        if len(values) != len(msg.name):
            self.get_logger().warn(f"{what} command: {len(msg.name)} names, {len(values)} values, ignored",
                                   throttle_duration_sec=5.0)
            return
        with self.lock:
            for name, v in zip(msg.name, values):
                a = self.actuator.get(name)
                if a is not None and math.isfinite(v):
                    self.d.ctrl[a] = v

    def on_wheels(self, msg):
        self._set(msg, msg.velocity, "wheel")

    def on_positions(self, msg):
        self._set(msg, msg.position, "position")

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joints
        with self.lock:
            msg.position = [float(self.d.qpos[self.qpos_adr[n]]) for n in self.joints]
            msg.velocity = [float(self.d.qvel[self.dof_adr[n]]) for n in self.joints]
            msg.effort = [float(self.d.actuator_force[self.actuator[n]]) if n in self.actuator else 0.0
                          for n in self.joints]
            pos, quat, vel = self.d.xpos[self.base].copy(), self.d.xquat[self.base].copy(), self.d.qvel[:6].copy()
        self.state_pub.publish(msg)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "mujoco_world"
        odom.child_frame_id = "base_footprint"
        p = odom.pose.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = map(float, quat)
        t = odom.twist.twist
        t.linear.x, t.linear.y, t.linear.z = map(float, vel[:3])      # world frame
        t.angular.x, t.angular.y, t.angular.z = map(float, vel[3:6])
        self.truth_pub.publish(odom)

    def on_detect(self, req, res):
        """Everything in the world the prompt names, within DETECT_RANGE of the robot: its centre, yaw and size in
        base_footprint, as the camera pipeline would report it. A stand-in for perception, exact."""
        words = req.prompt.lower()
        geoms = [p for w, ps in DETECT_GEOMS.items() if w in words for p in ps]
        bodies = [p for w, ps in DETECT_BODIES.items() if w in words for p in ps]
        res.detections.header.frame_id = "base_footprint"
        res.detections.header.stamp = self.get_clock().now().to_msg()
        if not (geoms or bodies):
            res.codes = ["BAD_PROMPT"]
            return res
        found = []
        with self.lock:
            Tb = np.eye(4)
            Tb[:3, :3], Tb[:3, 3] = self.d.xmat[self.base].reshape(3, 3), self.d.xpos[self.base]
            for g in range(self.m.ngeom):
                name = self.m.geom(g).name
                if name and any(name.startswith(p) for p in geoms):
                    found.append((name, self.d.geom_xpos[g].copy(), self.d.geom_xmat[g].reshape(3, 3).copy(),
                                  2 * self.m.geom_size[g].copy()))
            for b in range(self.m.nbody):
                name = self.m.body(b).name
                if name and any(name.startswith(p) for p in bodies):
                    size = 2 * self.m.geom_size[self.m.body_geomadr[b]].copy()
                    found.append((name, self.d.xpos[b].copy(), self.d.xmat[b].reshape(3, 3).copy(), size))
        Tinv = np.linalg.inv(Tb)
        label = req.label or req.prompt
        for name, pos, R, size in found:
            if np.hypot(*(pos[:2] - Tb[:2, 3])) > DETECT_RANGE:
                continue
            p = Tinv[:3, :3] @ pos + Tinv[:3, 3]
            Rr = Tinv[:3, :3] @ R
            yaw = math.atan2(Rr[1, 0], Rr[0, 0])
            d = Detection3D()
            d.header = res.detections.header
            d.id = name
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = label, 1.0
            h.pose.pose.position.x, h.pose.pose.position.y, h.pose.pose.position.z = map(float, p)
            h.pose.pose.orientation.z, h.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            d.results.append(h)
            d.bbox.center = h.pose.pose
            d.bbox.size.x, d.bbox.size.y, d.bbox.size.z = map(float, size)
            res.detections.detections.append(d)
            res.heights_m.append(float(size[2]))
        res.success = bool(res.detections.detections)
        res.codes = [] if res.success else ["NO_DET"]
        return res

    def publish_scan(self):
        with self.lock:
            origin = self.d.xpos[self.lidar].copy()
            R = self.d.xmat[self.lidar].reshape(3, 3).copy()
            dirs = (self.scan_dirs_local @ R.T).ravel()
            mujoco.mj_multiRay(self.m, self.d, origin, dirs, SCAN_GEOMGROUP, True, -1,
                               self.scan_geomid, self.scan_dist, None, SCAN_BINS, SCAN_RANGE_MAX)
        ranges = np.where((self.scan_geomid >= 0) & (self.scan_dist >= SCAN_RANGE_MIN)
                          & (self.scan_dist <= SCAN_RANGE_MAX), self.scan_dist, np.inf)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ldlidar_link"
        msg.angle_min = 0.0
        msg.angle_increment = 2 * math.pi / SCAN_BINS
        msg.angle_max = msg.angle_increment * (SCAN_BINS - 1)
        msg.time_increment = SCAN_PERIOD / SCAN_BINS
        msg.scan_time = SCAN_PERIOD
        msg.range_min = SCAN_RANGE_MIN
        msg.range_max = SCAN_RANGE_MAX
        msg.ranges = ranges.astype(float).tolist()
        self.scan_pub.publish(msg)


def make_world():
    spec, dims = world.build_spec(with_arms=True)
    model = spec.compile()
    data = mujoco.MjData(model)
    actuator = {model.actuator(i).name: i for i in range(model.nu)}
    for name, v in world.rest_qpos(model, dims).items():
        data.qpos[model.joint(name).qposadr[0]] = v
        data.ctrl[actuator[name]] = v
    mujoco.mj_forward(model, data)
    return model, data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--viewer", action="store_true", help="open MuJoCo's viewer (run with mjpython on macOS)")
    ap.add_argument("--presence", action="store_true", help="publish /mcp_presence_ok true, for the skill server")
    # known args only: the launch file passes an empty argument when the viewer is off
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    model, data = make_world()
    lock = threading.Lock()
    rclpy.init()
    node = MujocoTwin(model, data, lock, presence=args.presence)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    viewer = None
    if args.viewer:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
        # The viewer shows geom groups 0-2 by default; the robot is in world.ROBOT_GROUP (3).
        with viewer.lock():
            viewer.opt.geomgroup[world.ROBOT_GROUP] = 1

    dt = model.opt.timestep
    start = time.monotonic()
    sim_t = 0.0
    next_state = next_scan = next_view = 0.0
    behind_warned = 0.0
    node.get_logger().info(f"MuJoCo twin running, {model.nu} actuators, {len(node.joints)} joints")
    try:
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            with lock:
                mujoco.mj_step(model, data)
            sim_t += dt
            if sim_t >= next_state:
                node.publish_joint_states()
                next_state += JOINT_STATE_PERIOD
            if sim_t >= next_scan:
                node.publish_scan()
                next_scan += SCAN_PERIOD
            if viewer is not None and sim_t >= next_view:
                with lock:
                    viewer.sync()
                next_view += 1 / 60
            ahead = sim_t - (time.monotonic() - start)
            if ahead > 0:
                time.sleep(ahead)
            elif ahead < -0.5 and time.monotonic() - behind_warned > 10:
                node.get_logger().warn(f"twin is {-ahead:.2f} s behind real time")
                behind_warned = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
