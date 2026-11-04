"""The MuJoCo twin's world: the robot from the xacro, in a room with a table and a cube.

The robot comes from the same xacro as everything else, rendered as a plain URDF
(use_ros2_control:=false, like the Isaac import) and loaded by MuJoCo's own URDF importer; what
MuJoCo needs on top of it is added here with MjSpec, not kept in a second model file:

- a free joint under base_footprint, so the cart can drive
- actuators: velocity for the two wheels (diff_drive_controller), position for the head and arm
  joints, with the URDF's effort limits as force limits
- frictionless rear casters: the URDF has them as fixed spheres (no encoder, no swivel), so with
  friction they would drag across the floor instead of rolling
- the world: floor, walls, a few boxes for the lidar and Nav2 to see, a table and a cube

    python sim/mujoco/world.py [--save sim/output/mujoco/xlerobot_world.xml] [--with-arms]

saves the compiled world as MJCF, for `mujoco-simulate` or a look at what got built.
"""

import argparse
import os
import subprocess
import sys

import mujoco
import yaml

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DESCRIPTION = os.path.join(REPO_ROOT, "src", "xlerobot_description")
DIMS = os.path.join(DESCRIPTION, "config", "robot_dimensions.yaml")

# Geom groups. The lidar's rays see groups 0-2 only, so the robot's own geoms (group 3) never
# block it; the real LD19 sees the cart's posts and laser_filters removes them, nothing to remove
# here. MuJoCo's viewer hides group 3 by default; run_mujoco.py turns it on (key 3 toggles it).
ENV_GROUP = 0
ROBOT_GROUP = 3

# STS3215 position servos: kp in N*m/rad, damping ratio, URDF effort limit as the force limit.
# A first guess like the Isaac twin's ARM_GAINS, not fitted to the real servos.
SERVO_KP = 30.0
GRIPPER_KP = 15.0
SERVO_DAMPRATIO = 1.0
# Reflected rotor inertia of a 1:345 geared servo, keeps the stiff, light links stable at a 2 ms step.
SERVO_ARMATURE = 0.01
# Hub motor wheels under ODrive velocity control: torque = kv * velocity error, up to WHEEL_TORQUE.
# The ODrive holds speed tightly, so kv is high.
WHEEL_KV = 20.0
WHEEL_TORQUE = 12.0

# Floor to the underside of the cart's frame, TODO measure; only the collision box uses it.
CART_FLOOR_CLEARANCE = 0.08

# Where things are in the room (world frame, the robot starts at the origin facing +x).
TABLE_POS = (2.0, 1.2)
TABLE_SIZE = (0.30, 0.45, 0.72)       # half x, half y, height
CUBE_SIZE = 0.04
CUBE_MASS = 0.03
CUBE_EDGE_INSET = 0.06


def render_urdf(with_arms):
    """The xacro as a plain URDF, as for the Isaac import (docs/sim-twin.md)."""
    xacro = os.path.join(DESCRIPTION, "urdf", "xlerobot.urdf.xacro")
    out = subprocess.run(
        ["xacro", xacro, "use_ros2_control:=false", f"with_arms:={'true' if with_arms else 'false'}",
         "sensor:=none", "lidar:=ld19"],
        check=True, capture_output=True, text=True)
    return out.stdout


def load_robot(urdf):
    # package:// URIs to files, and MuJoCo's URDF options: keep every link as its own body
    # (fusestatic off, the lidar needs ldlidar_link), collision geometry only.
    urdf = urdf.replace("package://xlerobot_description/", DESCRIPTION + "/")
    head, sep, rest = urdf.partition('<robot name="xlerobot">')
    if not sep:
        raise RuntimeError("unexpected URDF: no <robot name=\"xlerobot\">")
    urdf = (head + sep + '<mujoco><compiler fusestatic="false" discardvisual="true" strippath="false"'
            ' balanceinertia="true"/></mujoco>' + rest)
    return mujoco.MjSpec.from_string(urdf)


def build_spec(with_arms=True):
    spec = load_robot(render_urdf(with_arms))
    dims = yaml.safe_load(open(DIMS))["xlerobot"]

    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    # Elliptic friction cones and a high impratio keep a grasped object from creeping out of the
    # jaws, MuJoCo's own advice for manipulation.
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10.0

    for g in spec.geoms:
        g.group = ROBOT_GROUP
    # body_link's solid collision box runs from the floor to the top basket's floor, so the cart
    # would stand on it with the wheels in the air. Isaac never sees it (collision_from_visuals uses
    # the cart mesh); here its bottom moves up to the cart's ground clearance. The basket's walls
    # (the other boxes, all well off the floor) stay as they are.
    for g in spec.body("body_link").geoms:
        if g.type == mujoco.mjtGeom.mjGEOM_BOX and g.pos[2] - g.size[2] < 0.01:
            top = g.pos[2] + g.size[2]
            g.size = [g.size[0], g.size[1], (top - CART_FLOOR_CLEARANCE) / 2]
            g.pos = [g.pos[0], g.pos[1], (top + CART_FLOOR_CLEARANCE) / 2]
    for b in spec.bodies:
        if b.name.endswith("caster_link"):
            for g in b.geoms:
                g.condim = 1          # normal force only, no friction
                g.priority = 1        # or the floor's condim 3 wins the pair and they drag
        if b.name.endswith("_wheel_link"):
            for g in b.geoms:
                # A sphere of the wheel's radius, not the URDF's cylinder: MuJoCo touches a
                # cylinder lying on its side at the two rim edges, and turning in place the
                # wheels hopped off the floor a third of the time and the cart turned 28% less
                # than its wheels. The tyre is rounded anyway.
                g.type = mujoco.mjtGeom.mjGEOM_SPHERE
                g.size = [g.size[0], 0, 0]
                # Not the tyres' real grip: MuJoCo's soft contacts let a rolling wheel creep in
                # proportion to its load over its friction, about 15-28% at 1.2 with tangential
                # forces at a sixth of the limit, nothing at 2.0. Odometry tests need the latter.
                g.friction = [2.0, 0.01, 0.0002]
        if "jaw" in b.name or b.name.endswith("gripper_link"):
            for g in b.geoms:
                g.condim = 4          # torsional friction too, holds a cube against spinning
                g.friction = [1.5, 0.02, 0.002]
                # Stiffer than MuJoCo's default 0.02 s: with it a squeezed cube crept out of the
                # jaws (15 mm while the arm moved), with this it seats ~4 mm and stays.
                g.solref = [0.005, 1.0]

    base = spec.body("base_footprint")
    base.add_freejoint(name="base_free")
    # base_footprint is on the floor; the casters touch it, the wheels are 2.6 mm up (axle_height
    # over wheel_radius in robot_dimensions.yaml) until the cart settles nose down onto them.
    base.pos = [0, 0, 0.001]

    for j in spec.joints:
        if j.type != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        if j.name.endswith("_wheel_joint"):
            a = spec.add_actuator(name=j.name, target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
            a.set_to_velocity(kv=WHEEL_KV)
            a.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
            a.forcerange = [-WHEEL_TORQUE, WHEEL_TORQUE]
        else:
            j.armature = SERVO_ARMATURE
            kp = GRIPPER_KP if j.name.endswith("gripper_joint") else SERVO_KP
            a = spec.add_actuator(name=j.name, target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
            a.set_to_position(kp=kp, dampratio=SERVO_DAMPRATIO, inheritrange=1.0)
            effort = 3.35
            a.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
            a.forcerange = [-effort, effort]

    add_world(spec)
    return spec, dims


def add_world(spec):
    w = spec.worldbody
    spec.visual.headlight.ambient = [0.4, 0.4, 0.4]
    w.add_light(pos=[0, 0, 4], dir=[0, 0, -1], diffuse=[0.6, 0.6, 0.6])
    w.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[6, 6, 0.1],
               rgba=[0.75, 0.75, 0.72, 1], group=ENV_GROUP, friction=[1.0, 0.01, 0.0001])

    # A 6 x 5 m room, walls 1 m high, the robot starts 1.5 m from the west wall.
    def box(name, center, half, rgba=(0.6, 0.6, 0.65, 1)):
        w.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(center), size=list(half),
                   rgba=list(rgba), group=ENV_GROUP)
    x0, x1, y0, y1, t, h = -1.5, 4.5, -2.5, 2.5, 0.05, 0.5
    box("wall_west", (x0 - t, 0, h), (t, (y1 - y0) / 2 + 2 * t, h))
    box("wall_east", (x1 + t, 0, h), (t, (y1 - y0) / 2 + 2 * t, h))
    box("wall_south", ((x0 + x1) / 2, y0 - t, h), ((x1 - x0) / 2, t, h))
    box("wall_north", ((x0 + x1) / 2, y1 + t, h), ((x1 - x0) / 2, t, h))
    # obstacles between the start and the table
    box("crate_1", (1.0, -0.4, 0.25), (0.25, 0.25, 0.25), (0.55, 0.4, 0.25, 1))
    box("crate_2", (2.8, -1.2, 0.3), (0.3, 0.2, 0.3), (0.55, 0.4, 0.25, 1))
    box("pillar", (0.4, 1.8, 0.5), (0.1, 0.1, 0.5), (0.4, 0.4, 0.45, 1))

    tx, ty = TABLE_POS
    hx, hy, th = TABLE_SIZE
    box("table_top", (tx, ty, th - 0.015), (hx, hy, 0.015), (0.7, 0.55, 0.35, 1))
    for i, (dx, dy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        box(f"table_leg_{i}", (tx + dx * (hx - 0.03), ty + dy * (hy - 0.03), (th - 0.03) / 2),
            (0.025, 0.025, (th - 0.03) / 2), (0.5, 0.38, 0.25, 1))

    # The cube 6 cm in from the table's near long edge (x = tx - hx): the arms reach about 40 cm out to
    # the side with the claw straight down, so the cart parks alongside the table to pick it up.
    cube = w.add_body(name="cube", pos=[tx - hx + CUBE_EDGE_INSET, ty, th + CUBE_SIZE / 2 + 0.001])
    cube.add_freejoint(name="cube_free")
    cube.add_geom(name="cube", type=mujoco.mjtGeom.mjGEOM_BOX, size=[CUBE_SIZE / 2] * 3,
                  mass=CUBE_MASS, rgba=[0.85, 0.15, 0.15, 1], group=ENV_GROUP, condim=4,
                  friction=[1.0, 0.02, 0.002])


def rest_qpos(model, dims):
    """qpos with the arms in arms.rest_pose and the actuator targets matching, as the Isaac twin starts."""
    rest = dims["arms"]["rest_pose"]
    targets = {}
    for side in ("left", "right"):
        for k, v in rest.items():
            targets[f"{side}_arm_{k}_joint"] = v
    return targets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default=os.path.join(REPO_ROOT, "sim", "output", "mujoco", "xlerobot_world.xml"))
    ap.add_argument("--no-arms", action="store_true")
    args = ap.parse_args()
    spec, _ = build_spec(with_arms=not args.no_arms)
    model = spec.compile()
    os.makedirs(os.path.dirname(args.save), exist_ok=True)
    with open(args.save, "w") as f:
        f.write(spec.to_xml())
    print(f"{args.save}: {model.nbody} bodies, {model.njnt} joints, {model.nu} actuators, "
          f"{sum(model.body_mass):.2f} kg in all")


if __name__ == "__main__":
    sys.exit(main())
