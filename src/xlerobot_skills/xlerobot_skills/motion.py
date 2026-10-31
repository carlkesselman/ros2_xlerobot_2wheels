"""MoveIt requests, MoveIt error codes as skill results, and the saved positions and waypoints."""
import os

import yaml
from geometry_msgs.msg import Point, Pose, Vector3
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             PositionConstraint)
from shape_msgs.msg import SolidPrimitive
from xlerobot_skill_msgs.msg import SkillResult

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GROUP_JOINTS = {f"{a}_arm": [f"{a}_arm_{j}_joint" for j in ARM_JOINTS] for a in ("left", "right")}
GROUP_JOINTS.update({f"{a}_gripper": [f"{a}_arm_gripper_joint"] for a in ("left", "right")})
GROUP_JOINTS["head"] = ["head_pan_joint", "head_tilt_joint"]
GROUP_TIP_LINK = {"left_arm": "left_arm_gripper_link", "right_arm": "right_arm_gripper_link"}
# Servo buses, for the e-stop on a safety stop and for keeping two motions off one bus.
GROUP_BUS = {"left_arm": "xlerobot_bus1", "left_gripper": "xlerobot_bus1", "head": "xlerobot_bus1",
             "right_arm": "xlerobot_bus2", "right_gripper": "xlerobot_bus2"}
POSITION_TOLERANCE_M = 0.01      # a point goal still needs a region; a zero-radius sphere is degenerate for some planners

_REACH = {MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.INVALID_MOTION_PLAN, MoveItErrorCodes.NO_IK_SOLUTION,
          MoveItErrorCodes.GOAL_IN_COLLISION, MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED,
          MoveItErrorCodes.START_STATE_IN_COLLISION, MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS,
          MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS, MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS}


def moveit_code(val):
    """MoveIt error code -> skill result code."""
    if val == MoveItErrorCodes.SUCCESS:
        return SkillResult.SUCCEEDED
    if val in _REACH:
        return SkillResult.NOT_REACHABLE
    if val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return SkillResult.COLLISION
    if val == MoveItErrorCodes.TIMED_OUT:
        return SkillResult.TIMEOUT
    if val == MoveItErrorCodes.PREEMPTED:
        return SkillResult.PREEMPTED
    return SkillResult.FAILED


def _request(group, scaling):
    r = MotionPlanRequest()
    r.group_name = group
    r.num_planning_attempts = 5
    r.allowed_planning_time = 10.0
    r.max_velocity_scaling_factor = float(scaling)
    r.max_acceleration_scaling_factor = float(scaling)
    return r


def joint_goal_request(group, goal, scaling):
    r = _request(group, scaling)
    c = Constraints()
    for joint, value in goal.items():
        c.joint_constraints.append(JointConstraint(joint_name=joint, position=float(value), tolerance_above=0.005,
                                                   tolerance_below=0.005, weight=1.0))
    r.goal_constraints.append(c)
    return r


def position_goal_request(group, frame, point, scaling, tip=(0.0, 0.0, 0.0), joints=None):
    """Put a point of the gripper link (`tip`, in that link's frame) at `point`. The arm has five joints, so the hand's
    orientation is free except for `joints`: {joint: (position, tolerance)}, e.g. the wrist roll."""
    r = _request(group, scaling)
    pc = PositionConstraint()
    pc.header.frame_id = frame
    pc.link_name = GROUP_TIP_LINK[group]
    pc.target_point_offset = Vector3(x=float(tip[0]), y=float(tip[1]), z=float(tip[2]))
    pc.constraint_region = BoundingVolume(
        primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[POSITION_TOLERANCE_M])],
        primitive_poses=[Pose(position=Point(x=float(point[0]), y=float(point[1]), z=float(point[2])))])
    pc.weight = 1.0
    jc = [JointConstraint(joint_name=j, position=float(v), tolerance_above=float(t), tolerance_below=float(t), weight=1.0)
          for j, (v, t) in (joints or {}).items()]
    r.goal_constraints.append(Constraints(position_constraints=[pc], joint_constraints=jc))
    return r


def default_store(name):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("xlerobot_bringup"), "config", name)


def load_positions(path):
    with open(path) as f:
        return (yaml.safe_load(f) or {}).get("positions", {})


def load_waypoints(path):
    with open(path) as f:
        return (yaml.safe_load(f) or {}).get("waypoints", {})
