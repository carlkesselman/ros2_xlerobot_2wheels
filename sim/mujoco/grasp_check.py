"""Does a gripper hold the cube? A MuJoCo-only check of the twin's grasp physics, no ROS:

1. the left arm settles in a pose with its gripper open (-0.4)
2. the cube is put between the jaws, centred on their surfaces, and held still there while the
   gripper servo closes on it
3. let go: only the jaws' friction holds the cube
4. the arm lifts and swings the cube around; the cube's slip in the gripper's frame is the result

    python sim/mujoco/grasp_check.py [--viewer]   (mjpython for the viewer on macOS)
"""

import argparse
import os
import sys
import time

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world  # noqa: E402

ARM = "left_arm"
POSE = {"shoulder_pan": 0.0, "shoulder_lift": 1.2, "elbow_flex": -1.2, "wrist_flex": 1.0, "wrist_roll": 0.0}
OPEN, CLOSED = -0.4, 0.35
# from the fingertips back toward the hinge, where the jaws are flat
INSET = 0.015
HELD_SLIP = 0.01            # m the cube may move in the gripper and still count as held


def mesh_verts(m, d, body):
    out = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == m.body(body).id and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            out.append(d.geom_xpos[g] + v @ d.geom_xmat[g].reshape(3, 3).T)
    return np.vstack(out)


def place_cube(m, d):
    """Put the cube between the open jaws, a face toward each, centred on their surfaces. Returns
    the cube's qpos (position, quaternion) and the clearance on each side."""
    jid = m.joint(f"{ARM}_gripper_joint").id
    piv, ax = d.xanchor[jid].copy(), d.xaxis[jid].copy()

    def radial(p):
        r = p - piv
        return np.linalg.norm(r - (r @ ax) * ax)
    # first guess from the fingertips: the moving jaw's farthest point from the hinge, and the
    # fixed jaw's point farthest out in the same direction
    mv = mesh_verts(m, d, f"{ARM}_moving_jaw_link")
    fx = mesh_verts(m, d, f"{ARM}_gripper_link")
    tip_m = mv[np.argmax([radial(p) for p in mv])]
    out = (tip_m - piv) / np.linalg.norm(tip_m - piv)
    cand = fx[(fx - piv) @ out > 0.6 * np.linalg.norm(tip_m - piv)]
    tip_f = cand[np.argmax((cand - piv) @ out)]
    centre = (tip_m + tip_f) / 2
    centre -= INSET * (centre - piv) / np.linalg.norm(centre - piv)
    x = (tip_f - tip_m) / np.linalg.norm(tip_f - tip_m)           # closing direction
    z = ax - (ax @ x) * x
    z /= np.linalg.norm(z)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.stack([x, np.cross(z, x), z], axis=1).ravel())

    # then centre it on the jaws' actual surfaces: slide toward the fixed jaw until the clearance
    # to both is the same
    adr = m.joint("cube_free").qposadr[0]
    cube = m.geom("cube").id
    ft = np.zeros(6)

    def nearest(body):
        best = (1e9, None)
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == m.body(body).id:
                dist = mujoco.mj_geomDistance(m, d, g, cube, 0.3, ft)
                if dist < best[0]:
                    best = (dist, ft[:3].copy())
        return best
    for _ in range(6):
        d.qpos[adr:adr + 3], d.qpos[adr + 3:adr + 7] = centre, quat
        d.qvel[m.joint("cube_free").dofadr[0]:][:6] = 0
        mujoco.mj_forward(m, d)
        df, pf = nearest(f"{ARM}_gripper_link")
        dm, _ = nearest(f"{ARM}_moving_jaw_link")
        centre = centre + (df - dm) / 2 * (pf - centre) / np.linalg.norm(pf - centre)
    return np.concatenate([centre, quat]), (df + dm) / 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--viewer", action="store_true")
    args = ap.parse_args()

    spec, _ = world.build_spec(with_arms=True)
    m = spec.compile()
    d = mujoco.MjData(m)
    act = {m.actuator(i).name: i for i in range(m.nu)}
    for k, v in POSE.items():
        d.qpos[m.joint(f"{ARM}_{k}_joint").qposadr[0]] = v
        d.ctrl[act[f"{ARM}_{k}_joint"]] = v
    d.qpos[m.joint(f"{ARM}_gripper_joint").qposadr[0]] = OPEN
    d.ctrl[act[f"{ARM}_gripper_joint"]] = OPEN
    for k, v in world.rest_qpos(m, world.yaml.safe_load(open(world.DIMS))["xlerobot"]).items():
        if not k.startswith(ARM):
            d.qpos[m.joint(k).qposadr[0]] = v
            d.ctrl[act[k]] = v
    mujoco.mj_forward(m, d)

    viewer = None
    if args.viewer:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(m, d)
        with viewer.lock():
            viewer.opt.geomgroup[world.ROBOT_GROUP] = 1      # hidden by default (groups 0-2 only)

    grip = m.body(f"{ARM}_gripper_link").id
    cube_id = m.body("cube").id
    qadr, vadr = m.joint("cube_free").qposadr[0], m.joint("cube_free").dofadr[0]
    hold = None                 # cube qpos to pin it at, while the jaws close

    def cube_in_gripper():
        R = d.xmat[grip].reshape(3, 3)
        return R.T @ (d.xpos[cube_id] - d.xpos[grip])

    def run(sec, ctrl=None):
        n = int(sec / m.opt.timestep)
        start = {k: d.ctrl[act[k]] for k in (ctrl or {})}
        for i in range(n):
            for k, v in (ctrl or {}).items():
                d.ctrl[act[k]] = start[k] + (v - start[k]) * min(1.0, (i + 1) / (0.8 * n))
            mujoco.mj_step(m, d)
            if hold is not None:
                d.qpos[qadr:qadr + 7] = hold
                d.qvel[vadr:vadr + 6] = 0
            if viewer is not None and i % 16 == 0:
                viewer.sync()
                time.sleep(0.016)

    run(1.0)                    # the arm settles under gravity, cube still on the table
    hold, clearance = place_cube(m, d)
    print(f"cube {world.CUBE_SIZE*1000:.0f} mm, {world.CUBE_MASS*1000:.0f} g, between the open jaws with "
          f"{clearance*1000:.1f} mm clearance each side")
    run(1.5, {f"{ARM}_gripper_joint": CLOSED})
    g = d.qpos[m.joint(f"{ARM}_gripper_joint").qposadr[0]]
    print(f"gripper stopped on the cube at {g:+.3f} rad (commanded {CLOSED}), squeezing with "
          f"{d.actuator_force[act[f'{ARM}_gripper_joint']]:+.2f} N*m of its 3.35")
    hold = None                 # let go: friction only from here
    p0 = cube_in_gripper()
    z0 = d.xpos[cube_id][2]
    run(1.0)
    print(f"let go, 1 s on friction alone: cube moved {np.linalg.norm(cube_in_gripper()-p0)*1000:.1f} mm in the "
          f"gripper, {(z0 - d.xpos[cube_id][2])*1000:+.1f} mm down")
    run(1.5, {f"{ARM}_shoulder_lift_joint": POSE["shoulder_lift"] - 0.5, f"{ARM}_wrist_flex_joint": POSE["wrist_flex"] - 0.4})
    run(1.5, {f"{ARM}_shoulder_pan_joint": 0.8})
    run(1.5, {f"{ARM}_shoulder_pan_joint": -0.8, f"{ARM}_elbow_flex_joint": POSE["elbow_flex"] + 0.4})
    run(1.0)
    after_moves = cube_in_gripper()
    print(f"carried {(d.xpos[cube_id][2]-z0)*100:+.1f} cm in height while the arm lifted and swung through 1.6 rad: "
          f"cube moved {np.linalg.norm(after_moves - p0)*1000:.1f} mm in the gripper")
    run(5.0)
    creep = np.linalg.norm(cube_in_gripper() - after_moves)
    slip = np.linalg.norm(cube_in_gripper() - p0)
    held = slip < HELD_SLIP
    print(f"then 5 s holding still: {creep*1000:.2f} mm more -> {'HELD' if held else 'DROPPED / SLIPPED'} "
          f"({slip*1000:.1f} mm in all)")
    if viewer is not None:
        while viewer.is_running():
            time.sleep(0.1)
    return 0 if held else 1


if __name__ == "__main__":
    sys.exit(main())
