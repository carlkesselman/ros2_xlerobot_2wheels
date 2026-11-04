#!/usr/bin/env python3
"""Compare arm mount layouts on the top basket by reachable workspace.

Takes the URDF generated from xlerobot.urdf.xacro (with_arms:=true), moves the two
arm mount joints to each candidate layout, writes that variant's URDF, and samples
the SO-101 joint space to see where the grasp point can get to:

  - surfaces in front of the cart (table, counter, low table, floor), grasp point
    3 cm above the surface, gripper pointing down within 30 degrees
  - side reach at table height
  - drop points 5 cm above the top basket's rim, gripper within 45 degrees of down

Collisions are approximate: points along each link are kept out of the cart body
(the open top basket excepted), the drive wheels, the floor and the neck box.
Arm-arm collisions and self collisions are ignored. Position reach only.

  pixi run python tools/arm_layout_reach.py <generated.urdf> <out_dir>
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# ---- cart geometry, base_link frame (x forward from the drive axle, z up from the floor) ----
RIM_Z = 0.78                 # top basket rim
BASKET_FLOOR_Z = 0.69        # rim less top_basket_depth
FRONT_RIM_X = -0.021         # outside of the front rim
REAR_RIM_X = -0.316
INNER_FRONT_X = -0.027       # inside of the front wall (5.9 mm wall)
INNER_REAR_X = -0.3058
INNER_HALF_W = 0.1965        # top basket 0.405 outside, less the wall
BODY_HALF_W = 0.2125         # outside of the vertical side rails
WHEEL_FRONT_X = 0.0824       # tyre front, the first thing to touch a cabinet base
WHEEL_Y = (0.2405, 0.2865)   # 0.527/2 -+ tyre half width
NECK = dict(x=(-0.37, -0.29), y=(-0.06, 0.06), z=(0.0, 1.16))  # assumed neck box + mast, TODO measure
CAM = np.array([-0.263, 0.0, 1.196])  # D415 glass, head level (platform.md 6.3)

# ---- SO-101 bits (meshes/upstream) ----
PAN_OFF = 0.0452                         # pan axis from the arm base origin, base frame -y
BASE_FOOT = (-0.0555, 0.0555, -0.071, 0.0249)  # Base.stl x min/max, y min/max
GRASP_Y = -0.085                         # grasp point along gripper_link -y (fixed jaw tip -0.106)
CLEAR = 0.005                            # base footprint to basket wall


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_mat(*rpy)
    T[:3, 3] = xyz
    return T


def axis_rot(axis, q):
    """Batch of 4x4 rotations about a unit axis, q shape (N,)."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    s, c = np.sin(q)[:, None, None], np.cos(q)[:, None, None]
    R = np.eye(3) + s * K + (1 - c) * (K @ K)
    T = np.zeros((len(q), 4, 4))
    T[:, :3, :3] = R
    T[:, 3, 3] = 1
    return T


class Urdf:
    def __init__(self, path):
        self.tree = ET.parse(path)
        self.joints = {}
        for j in self.tree.getroot().iter('joint'):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0').split() if o is not None else [0, 0, 0])]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0').split() if o is not None else [0, 0, 0])]
            ax = j.find('axis')
            lim = j.find('limit')
            self.joints[j.get('name')] = dict(
                el=j, type=j.get('type'), parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                T=tf(xyz, rpy), axis=[float(v) for v in ax.get('xyz').split()] if ax is not None else None,
                lim=(float(lim.get('lower')), float(lim.get('upper'))) if lim is not None else None)
        self.by_child = {j['child']: n for n, j in self.joints.items()}

    def set_origin(self, name, xyz, rpy):
        j = self.joints[name]
        j['T'] = tf(xyz, rpy)
        j['el'].find('origin').set('xyz', ' '.join(f'{v:.5f}' for v in xyz))
        j['el'].find('origin').set('rpy', ' '.join(f'{v:.8f}' for v in rpy))

    def fixed_chain(self, root, link):
        T = np.eye(4)
        while link != root:
            n = self.by_child[link]
            j = self.joints[n]
            assert j['type'] == 'fixed', n
            T = j['T'] @ T
            link = j['parent']
        return T

    def write(self, path):
        self.tree.write(path, xml_declaration=True, encoding='utf-8')


def arm_fk(u, prefix, T0, q):
    """q: (N,5) pan, lift, elbow, wflex, roll. Returns list of (N,3) points along the arm and the
    grasp point / pointing direction."""
    names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
    N = len(q)
    T = np.broadcast_to(T0, (N, 4, 4)).copy()
    pts = []
    for i, n in enumerate(names):
        j = u.joints[f'{prefix}_{n}_joint']
        T = T @ j['T']
        pts.append(T[:, :3, 3].copy())
        T = T @ axis_rot(j['axis'], q[:, i])
    tip = T @ np.array([0, -0.106, 0, 1])
    grasp = T @ np.array([0, GRASP_Y, 0, 1])
    pts.append(tip[:, :3])
    d = -T[:, :3, 1]  # pointing direction (gripper -y)
    # midpoints so long links can't pass through an obstacle between their joints
    chain = pts[1:]
    mids = [(a + b) / 2 for a, b in zip(chain[:-1], chain[1:])]
    mids += [a + (b - a) * f for a, b in zip(chain[:-1], chain[1:]) for f in (0.25, 0.75)]
    return pts[1:] + mids, grasp[:, :3], d


def in_box(p, x, y, z, m=0.01):
    return ((p[:, 0] > x[0] - m) & (p[:, 0] < x[1] + m) & (p[:, 1] > y[0] - m) & (p[:, 1] < y[1] + m)
            & (p[:, 2] > z[0] - m) & (p[:, 2] < z[1] + m))


def collides(p):
    body = in_box(p, (REAR_RIM_X, FRONT_RIM_X), (-BODY_HALF_W, BODY_HALF_W), (0.0, RIM_Z), m=0.005)
    basket = in_box(p, (INNER_REAR_X, INNER_FRONT_X), (-INNER_HALF_W, INNER_HALF_W), (BASKET_FLOOR_Z, 2.0), m=-0.01)
    c = body & ~basket
    c |= p[:, 2] < 0.01
    for s in (1, -1):
        c |= in_box(p, (-WHEEL_FRONT_X, WHEEL_FRONT_X), sorted((s * WHEEL_Y[0], s * WHEEL_Y[1])), (0, 2 * WHEEL_FRONT_X))
    c |= in_box(p, NECK['x'], NECK['y'], NECK['z'])
    return c


def sample_arm(u, prefix, T0, n=(49, 41, 41, 31), seed=0):
    lims = [u.joints[f'{prefix}_{k}_joint']['lim'] for k in ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex')]
    grids = [np.linspace(lo, hi, k) for (lo, hi), k in zip(lims, n)]
    Q = np.stack(np.meshgrid(*grids, indexing='ij'), -1).reshape(-1, 4)
    rng = np.random.default_rng(seed)
    step = np.array([(hi - lo) / (k - 1) for (lo, hi), k in zip(lims, n)])
    Q = Q + rng.uniform(-0.5, 0.5, Q.shape) * step  # jitter off the lattice
    Q = np.clip(Q, [l[0] for l in lims], [l[1] for l in lims])
    Q = np.hstack([Q, np.zeros((len(Q), 1))])
    out_g, out_d = [], []
    for k in range(0, len(Q), 200_000):
        pts, g, d = arm_fk(u, prefix, T0, Q[k:k + 200_000])
        bad = np.zeros(len(g), bool)
        for p in pts:
            bad |= collides(p)
        out_g.append(g[~bad])
        out_d.append(d[~bad])
    return np.vstack(out_g), np.vstack(out_d)


# ---------------------------------------------------------------------------------------------
def place(u, layout):
    """Move both arm mount joints to the layout. Works in base_link (x forward, left +y); the
    mount joints live in the body frame (base_link turned 180 degrees about z), where an arm
    whose stretched zero pose points forward has yaw -pi/2. Toe-in turns each arm toward the
    centreline; front layouts push each base into its front corner of the basket."""
    toe = math.radians(layout.get('toe_in', 0))
    z = layout.get('z', 0.7603)
    fx0, fx1, fy0, fy1 = BASE_FOOT
    corners = np.array([[x, y + PAN_OFF] for x in (fx0, fx1) for y in (fy0, fy1)])  # about the pan axis
    pans = {}
    for side, sgn in (('left', 1), ('right', -1)):
        phi = -sgn * toe                      # base_link heading of the stretched arm
        yaw = phi - math.pi / 2               # body frame yaw of the arm base
        c, s_ = math.cos(yaw), math.sin(yaw)
        body = corners @ np.array([[c, s_], [-s_, c]])
        base = -body
        if layout['kind'] == 'current':
            px, py = -0.239, sgn * 0.1329
        else:
            px = INNER_FRONT_X - CLEAR - base[:, 0].max()
            py = sgn * (INNER_HALF_W - CLEAR - (base[:, 1].max() if sgn > 0 else -base[:, 1].min()))
        xb, yb = -px, -py                     # pan axis in the body frame
        xyz = (xb - PAN_OFF * math.sin(yaw), yb + PAN_OFF * math.cos(yaw), z)
        u.set_origin(f'{side}_arm_mount_joint', xyz, (0, 0, yaw))
        pans[side] = (px, py)
    return pans['left'], z


LAYOUTS = [
    dict(name='A_current', kind='current', label='A  current (rear, facing forward)'),
    dict(name='B_front', kind='front', toe_in=0, label='B  front corners, facing forward'),
    dict(name='C_front_in30', kind='front', toe_in=30, label='C  front corners, toed in 30°'),
    dict(name='D_front_in45', kind='front', toe_in=45, label='D  front corners, toed in 45°'),
    dict(name='E_front_out20', kind='front', toe_in=-20, label='E  front corners, toed out 20°'),
    dict(name='F_front_low', kind='front', toe_in=30, z=0.695, label='F  as C, base on the basket floor (6.5 cm lower)'),
]

SURFACES = [('table', 0.75), ('counter', 0.91), ('low table', 0.45), ('floor', 0.0)]
BIN = 0.02


def occupancy(g, d, z, down_cos, tol=0.02):
    m = (np.abs(g[:, 2] - z) < tol) & (-d[:, 2] >= down_cos)
    ij = np.floor(g[m, :2] / BIN).astype(int)
    return set(map(tuple, ij))


def main():
    src, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    results = []
    for L in LAYOUTS:
        u = Urdf(src)
        (px, py), z = place(u, L)
        u.write(out / f'{L["name"]}.urdf')
        arms = {}
        for side in ('left', 'right'):
            T0 = u.fixed_chain('base_link', f'{side}_arm_base_link')
            # sanity: pan axis where we meant it
            pan = (T0 @ u.joints[f'{side}_arm_shoulder_pan_joint']['T'])[:3, 3]
            arms[side] = dict(pan=pan, g_d=sample_arm(u, f'{side}_arm', T0))
            # zero pose direction
            g0 = arm_fk(u, f'{side}_arm', T0, np.zeros((1, 5)))[1][0]
            arms[side]['zero_dir'] = math.degrees(math.atan2(g0[1] - pan[1], g0[0] - pan[0]))
        r = dict(layout=L, arms={s: dict(pan=a['pan'].round(4).tolist(), zero_dir=round(a['zero_dir'], 1)) for s, a in arms.items()})
        both = {s: a['g_d'] for s, a in arms.items()}
        down30, down45 = math.cos(math.radians(30)), math.cos(math.radians(45))
        maps = {}
        for sname, h in SURFACES:
            zz = h + 0.03
            occ = {s: occupancy(*both[s], zz, down30) for s in both}
            union, inter = occ['left'] | occ['right'], occ['left'] & occ['right']
            stop = WHEEL_FRONT_X if sname in ('counter', 'floor', 'low table') else FRONT_RIM_X
            if sname == 'low table':
                stop = WHEEL_FRONT_X  # low table: wheels/rim may pass over or not, conservative
            front = [c for c in union if (c[0] + 0.5) * BIN > stop and abs((c[1] + 0.5) * BIN) <= BODY_HALF_W]
            front_both = [c for c in inter if (c[0] + 0.5) * BIN > stop and abs((c[1] + 0.5) * BIN) <= BODY_HALF_W]
            xs = [(c[0] + 1) * BIN for c in union if abs((c[1] + 0.5) * BIN) <= BODY_HALF_W]
            ys = [abs((c[1] + 0.5) * BIN) for c in union if -0.25 < (c[0] + 0.5) * BIN < 0.05]
            # any orientation, for the record
            occ_any = set()
            for s in both:
                occ_any |= occupancy(*both[s], zz, -1.0)
            xs_any = [(c[0] + 1) * BIN for c in occ_any if abs((c[1] + 0.5) * BIN) <= BODY_HALF_W]
            r[sname] = dict(
                height=h, stop=stop,
                max_x_down=round(max(xs), 3) if xs else None,
                past_stop_down=round(max(xs) - stop, 3) if xs else None,
                area_past_stop_cm2=len(front) * (BIN * 100) ** 2,
                area_both_past_stop_cm2=len(front_both) * (BIN * 100) ** 2,
                max_x_any=round(max(xs_any), 3) if xs_any else None,
                side_y_down=round(max(ys) + BIN / 2, 3) if ys else None)
            maps[sname] = dict(left=occ['left'], right=occ['right'])
        # basket drop points
        zz = RIM_Z + 0.05
        occ_s = {s: occupancy(*both[s], zz, down45) for s in both}
        occ = occ_s['left'] | occ_s['right']
        cells = [(i, j) for i in range(int(math.floor(INNER_REAR_X / BIN)) + 1, int(math.floor(INNER_FRONT_X / BIN)))
                 for j in range(int(math.floor(-INNER_HALF_W / BIN)) + 1, int(math.floor(INNER_HALF_W / BIN)))]
        hit = [c for c in cells if c in occ]
        r['basket_cover'] = round(len(hit) / len(cells), 3)
        maps['basket'] = dict(left=occ_s['left'], right=occ_s['right'], cells=cells)
        r['basket_both'] = round(len([c for c in cells if c in occ_s['left'] and c in occ_s['right']]) / len(cells), 3)
        # camera range to the middle of the table workspace in front of the rim
        tbl = maps['table']['left'] | maps['table']['right']
        fr = np.array([((c[0] + 0.5) * BIN, (c[1] + 0.5) * BIN, 0.78) for c in tbl if (c[0] + 0.5) * BIN > FRONT_RIM_X and abs((c[1] + 0.5) * BIN) < BODY_HALF_W])
        if len(fr):
            v = fr - CAM
            rng = np.linalg.norm(v, axis=1)
            dep = np.degrees(np.arctan2(-v[:, 2], np.hypot(v[:, 0], v[:, 1])))
            r['cam'] = dict(range_min=round(rng.min(), 3), range_max=round(rng.max(), 3), depression_min=round(dep.min(), 1), depression_max=round(dep.max(), 1))
        r['pan_axis'] = (round(px, 4), round(py, 4)); r['mount_z'] = z
        r['_maps'] = maps
        results.append(r)
        print(L['label'], {k: v for k, v in r.items() if k not in ('_maps', 'layout')}, flush=True)
    np.save(out / 'results.npy', np.array(results, dtype=object), allow_pickle=True)


if __name__ == '__main__':
    main()
