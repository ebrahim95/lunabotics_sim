#!/usr/bin/env python3
"""
Bucket digging test in MuJoCo.

Builds a MuJoCo scene from the repaired robot model: the excavator ladder's
rollers and the regolith bin at their CAD positions (robot standing on the
ground), the 10 CAD buckets running round the belt loop, and a pit of
regolith particles under the ladder. Then it runs the belt and counts how
many particles the buckets lift and drop in the bin.

MuJoCo collides with the convex hull of a mesh, which would turn the hollow
CAD bucket and bin into solid lumps. So each is split into its walls: every
pair of parallel outer/inner faces in the STL becomes one convex slab.

The robot's motors are force-limited like the real ones:

  * Lift: the ladder slides on its rails on a joint driven by a position
    actuator capped at the two PA-14 actuators' 311 N, lowered at their
    2"/s. Gravity on the 8 kg ladder and the regolith pushing back on the
    buckets both load it; past 311 N the ladder stops going down.
  * Belt: one belt-travel coordinate driven by the TSINY motor model
    (6 N*m at the drive roller, 50 rpm, self-locking worm gear). Each bucket
    rides the ladder on its own joints, held to its place on the loop by
    stiff servos; the force those servos need along the belt is the load
    the motor feels. Past 6 N*m the belt slows, and it stalls rather than
    runs backwards.

The chassis is still moved by script (--advance); the wheels aren't
modelled. Particles are spheres with rolling friction, so a pile stays a
pile.

Usage (from the repo root):
    .venv-mujoco/bin/python mujoco_dig/dig_test.py            # headless, prints counts
    .venv-mujoco/bin/python mujoco_dig/dig_test.py --viewer   # watch it
"""
import argparse
import os
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO, "src", "lunabotics_model")
sys.path.insert(0, os.path.join(PKG, "scripts"))
from fix_kinematics import (  # noqa: E402
    LIFT_EFFORT, LIFT_LOWER, LIFT_UPPER, LIFT_VELOCITY, ROLLER_EFFORT, ROLLER_VELOCITY,
    load_stl_vertices, loop_pose, origin_to_T, world_poses)

ROLLER_RADIUS = 0.047
ROLLER_LENGTH = 0.26
MAX_WALL = 0.015  # thickest wall to look for when splitting a hollow mesh
# Pit footprint relative to the ladder's bottom shaft, in base_link axes
# (x forward, y left; m): the strip the 0.205 m wide buckets sweep. Every
# particle costs sim speed, so keep it to where the buckets reach.
PIT_FORWARD = (-0.06, 0.24)
PIT_LEFT = (-0.13, 0.17)
# The regolith fills a pit this deep, level with the surface the robot
# stands on, so lowering the ladder digs into a bed rather than a heap.
PIT_DEPTH = 0.10
GROUND_HALF_SIZE = 3.0  # m, extent of the surface around the pit
# Servo gains. The lift servo stands in for the actuators' lead screws; the
# bucket servos for the belt holding each bucket at its place on the loop.
LIFT_KP, LIFT_KV = 50000.0, 3000.0      # N/m, N*s/m
BUCKET_KP, BUCKET_KV = 20000.0, 150.0   # N/m, N*s/m along / across the ladder
BUCKET_ROT_KP, BUCKET_ROT_KV = 30.0, 0.3  # N*m/rad, N*m*s/rad about the shafts
# Belt drive model: speed loop gain, and the moving mass of belt + buckets
# as seen along the belt (sets how fast the belt speeds up and slows down).
BELT_GAIN = 50.0    # N*m per (m/s) of belt speed error
BELT_MASS = 3.0     # kg
LOAD_FILTER = 0.02  # s, smoothing of the belt load the motor sees
# Contact groups: buckets and rollers touch only the regolith (they'd
# otherwise snag on the pit edges and the bin, which they clear on the robot).
REGOLITH_BITS = 'contype="1" conaffinity="3"'
DIGGER_BITS = 'contype="2" conaffinity="0"'


def shell_slabs(tris, max_thickness=MAX_WALL):
    """Split a hollow, thin-walled mesh into convex wall slabs.

    Groups the triangles by plane and pairs each face with the parallel,
    opposite-facing face at most max_thickness away (the other side of the
    same wall). Returns one vertex array per wall; MuJoCo hulls each one.
    Unpaired faces (rims around openings, small facets) are skipped.
    """
    planes = {}
    for t in tris:
        n = np.cross(t[1] - t[0], t[2] - t[0])
        if np.linalg.norm(n) < 1e-12:
            continue
        n = n / np.linalg.norm(n)
        k = (tuple(np.round(n, 3)), round(float(n @ t[0]), 4))
        planes.setdefault(k, []).append(t)
    slabs, used = [], set()
    for (n, d), faces in planes.items():
        if (n, d) in used:
            continue
        best = None
        for (m, e), other in planes.items():
            if (m, e) in used or (m, e) == (n, d):
                continue
            if np.allclose(m, -np.array(n), atol=2e-3) and abs(d + e) <= max_thickness:
                if best is None or abs(d + e) < abs(best[0][1] + d):
                    best = ((m, e), other)
        if best:
            used |= {(n, d), best[0]}
            slabs.append(np.unique(np.round(np.vstack(faces + best[1]), 6), axis=0))
    return slabs


def load_geometry():
    """Everything the scene needs, from the repaired URDF and belt config."""
    robot = ET.parse(os.path.join(PKG, "urdf", "assembly_1_fixed.urdf")).getroot()
    links = {l.get("name"): l for l in robot.findall("link")}
    W = world_poses(robot)
    with open(os.path.join(PKG, "config", "belt_animator.yaml")) as f:
        belt = yaml.safe_load(f)["belt_animator"]["ros__parameters"]

    # Ladder frame (same construction as fix_kinematics.add_belt_track).
    bottom, top = W["rod_1"][:3, 3], W["rod"][:3, 3]
    eu = (top - bottom) / np.linalg.norm(top - bottom)
    ew = W["rod_1"][:3, 2] - (W["rod_1"][:3, 2] @ eu) * eu
    ew /= np.linalg.norm(ew)
    L = np.eye(4)
    L[:3, :3] = np.column_stack([eu, np.cross(ew, eu), ew])
    L[:3, 3] = bottom

    def mesh_tris(link):
        """Triangles of a link's visual mesh, in the link frame."""
        vis = links[link].find("visual")
        T = origin_to_T(vis.find("origin"))
        name = vis.find("geometry/mesh").get("filename").rsplit("/", 1)[-1]
        V = load_stl_vertices(os.path.join(PKG, "meshes", name))
        return ((T[:3, :3] @ V.T).T + T[:3, 3]).reshape(-1, 3, 3)

    # Bucket walls in the belt-mount frame (x along the belt, y away from it,
    # z along the shafts). At zero joint values a bucket's link frame is the
    # ladder frame, so its link-frame mesh is already in mount coordinates.
    b0 = belt["buckets"][1]
    bucket_tris = mesh_tris(b0)
    to_mount = np.linalg.inv(L) @ W[b0]
    bucket_tris = np.einsum("ij,ntj->nti", to_mount[:3, :3], bucket_tris) + to_mount[:3, 3]

    # Bin walls in the robot frame.
    bin_tris = mesh_tris("bucket")
    bin_tris = np.einsum("ij,ntj->nti", W["bucket"][:3, :3], bin_tris) + W["bucket"][:3, 3]
    bin_pts = bin_tris.reshape(-1, 3)

    # Every other CAD part, for display only, split by whether it rides the
    # excavator lift (the subtree below excavator_lift) or stays on the chassis.
    children = {}
    for j in robot.findall("joint"):
        children.setdefault(j.find("parent").get("link"), []).append(j)
    lift_child = robot.find("joint[@name='excavator_lift']").find("child").get("link")
    on_ladder, stack = set(), [lift_child]
    while stack:
        n = stack.pop()
        on_ladder.add(n)
        stack += [j.find("child").get("link") for j in children.get(n, [])]
    def mass(name):
        m = links[name].find("inertial/mass")
        return float(m.get("value")) if m is not None else 0.0
    bucket_mass = mass(belt["buckets"][0])
    ladder_mass = sum(mass(n) for n in on_ladder) - len(belt["buckets"]) * bucket_mass
    visuals = []
    for name, link in links.items():
        if name in belt["buckets"]:
            continue  # buckets are drawn by their own moving bodies
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            color = vis.find("material/color")
            visuals.append(dict(
                file=os.path.join(PKG, "meshes", mesh.get("filename").rsplit("/", 1)[-1]),
                T=W[name] @ origin_to_T(vis.find("origin")),
                rgba=color.get("rgba") if color is not None else "0.7 0.7 0.7 1",
                ladder=name in on_ladder))

    return dict(L=L, belt=belt, visuals=visuals, bucket_walls=shell_slabs(bucket_tris),
                bin_walls=shell_slabs(bin_tris), bin=(bin_pts.min(0), bin_pts.max(0)),
                rollers=[W["rod"][:3, 3], W["rod_1"][:3, 3]], shaft_axis=ew,
                pit_centre=W["rod_1"][:2, 3], bucket_mass=bucket_mass, ladder_mass=ladder_mass)


def quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.flatten())
    return q


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def build_xml(g, radius, layers, seed, lift_force, pit_depth=PIT_DEPTH):
    # The chassis (bin, frame) is a mocap body moved by script, so --advance
    # can drive the robot forward. The ladder hangs off it on the lift joint,
    # and the buckets hang off the ladder on their belt joints.
    L = g["L"]
    Linv = np.linalg.inv(L)

    def in_ladder(T):
        """A robot-frame pose, expressed in the ladder frame."""
        return Linv @ T

    chassis, ladder = [], []
    # Rollers: cylinders on the shafts.
    ax = g["shaft_axis"]
    Rcyl = np.column_stack([np.cross(ax, [0, 0, 1.0]), np.zeros(3), ax])
    Rcyl[:, 0] /= np.linalg.norm(Rcyl[:, 0])
    Rcyl[:, 1] = np.cross(Rcyl[:, 2], Rcyl[:, 0])
    for i, c in enumerate(g["rollers"]):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = Rcyl, c
        T = in_ladder(T)
        ladder.append(f'<geom name="roller_{i}" type="cylinder" size="{ROLLER_RADIUS} {ROLLER_LENGTH / 2}" '
                      f'pos="{fmt(T[:3, 3])}" quat="{fmt(quat(T[:3, :3]))}" {DIGGER_BITS} rgba="0 0 0 0"/>')

    # Bin: the CAD hopper's walls.
    assets = []
    for i, w in enumerate(g["bin_walls"]):
        assets.append(f'<mesh name="bin_wall_{i}" vertex="{fmt(w.flatten())}"/>')
        chassis.append(f'<geom name="bin_wall_{i}" type="mesh" mesh="bin_wall_{i}" rgba="0 0 0 0"/>')

    # The rest of the robot: CAD meshes, display only (no collisions, no mass).
    for i, v in enumerate(g["visuals"]):
        assets.append(f'<mesh name="cad_{i}" file="{v["file"]}" inertia="shell"/>')
        T = in_ladder(v["T"]) if v["ladder"] else v["T"]
        geom = (f'<geom type="mesh" mesh="cad_{i}" contype="0" conaffinity="0" group="1" mass="0" '
                f'pos="{fmt(T[:3, 3])}" quat="{fmt(quat(T[:3, :3]))}" rgba="{v["rgba"]}"/>')
        (ladder if v["ladder"] else chassis).append(geom)

    # Buckets: the CAD bucket's walls, on three joints in the ladder frame
    # (x along the ladder, y across it, z along the shafts), like the
    # bucket_N_belt_u / _v / _rot joints in the URDF.
    for i, w in enumerate(g["bucket_walls"]):
        assets.append(f'<mesh name="bucket_wall_{i}" vertex="{fmt(w.flatten())}"/>')
    pts = np.vstack(g["bucket_walls"])
    lo, hi = pts.min(0), pts.max(0)
    size = hi - lo
    m = g["bucket_mass"]
    inertia = m / 12 * np.array([size[1] ** 2 + size[2] ** 2, size[0] ** 2 + size[2] ** 2, size[0] ** 2 + size[1] ** 2])
    actuators = []
    buckets = []
    for b in g["belt"]["buckets"]:
        geoms = "".join(f'<geom type="mesh" mesh="bucket_wall_{i}" mass="0" {DIGGER_BITS} rgba="0.9 0.55 0.1 1"/>'
                        for i in range(len(g["bucket_walls"])))
        buckets.append(
            f'<body name="{b}"><inertial pos="{fmt((lo + hi) / 2)}" mass="{m:.4g}" diaginertia="{fmt(inertia)}"/>'
            f'<joint name="{b}_u" type="slide" axis="1 0 0"/><joint name="{b}_v" type="slide" axis="0 1 0"/>'
            f'<joint name="{b}_rot" type="hinge" axis="0 0 1"/>{geoms}</body>')
        actuators += [f'<position name="{b}_u" joint="{b}_u" kp="{BUCKET_KP}" kv="{BUCKET_KV}"/>',
                      f'<position name="{b}_v" joint="{b}_v" kp="{BUCKET_KP}" kv="{BUCKET_KV}"/>',
                      f'<position name="{b}_rot" joint="{b}_rot" kp="{BUCKET_ROT_KP}" kv="{BUCKET_ROT_KV}"/>']

    # Ladder body: frame L, sliding along its own x axis (up the rails).
    ladder_com = (np.array([0.0, 0.0, 0.0]) + Linv[:3, :3] @ (g["rollers"][0] - g["rollers"][1])) / 2
    lift_joint = (f'<joint name="lift" type="slide" axis="1 0 0" range="{LIFT_LOWER} {LIFT_UPPER}" '
                  f'limited="true" damping="50"/>')
    ladder_body = (f'<body name="ladder" pos="{fmt(L[:3, 3])}" quat="{fmt(quat(L[:3, :3]))}">'
                   f'<inertial pos="{fmt(ladder_com)}" mass="{g["ladder_mass"]:.4g}" diaginertia="0.3 0.3 0.05"/>'
                   f'{lift_joint}{"".join(ladder)}{"".join(buckets)}</body>')
    actuators.insert(0, f'<position name="lift" joint="lift" kp="{LIFT_KP}" kv="{LIFT_KV}" '
                        f'forcelimited="true" forcerange="{-lift_force} {lift_force}"/>')
    # base_link sits on the ground, so the robot's frame is the world frame at start.
    parts = [f'<body name="chassis" mocap="true">{"".join(chassis)}{ladder_body}</body>']

    # Ground: the surface the robot stands on (z = 0) with the pit cut out
    # of it. Four slabs around the pit, from the pit floor up to the surface;
    # their inner faces are the pit walls. The plane below is the pit floor.
    cx, cy = g["pit_centre"]
    x0, x1 = cx + PIT_FORWARD[0], cx + PIT_FORWARD[1]
    y0, y1 = cy + PIT_LEFT[0], cy + PIT_LEFT[1]
    S = GROUND_HALF_SIZE
    zc, hz = -pit_depth / 2, pit_depth / 2
    for name, (ax0, ax1, ay0, ay1) in {
        "west": (-S, x0, -S, S), "east": (x1, S, -S, S),
        "south": (x0, x1, -S, y0), "north": (x0, x1, y1, S),
    }.items():
        parts.append(f'<geom name="ground_{name}" type="box" '
                     f'size="{(ax1 - ax0) / 2:.4g} {(ay1 - ay0) / 2:.4g} {hz:.4g}" '
                     f'pos="{(ax0 + ax1) / 2:.4g} {(ay0 + ay1) / 2:.4g} {zc:.4g}" rgba="0.72 0.72 0.72 1"/>')

    # Regolith: spheres filling the pit, dropped in loosely and left to settle.
    rng = np.random.default_rng(seed)
    pitch = 2 * radius * 1.05
    xs = np.arange(x0 + radius, x1 - radius, pitch)
    ys = np.arange(y0 + radius, y1 - radius, pitch)
    n = 0
    for k in range(layers):
        for x in xs:
            for y in ys:
                # small sideways jitter only, so particles never start overlapping
                p = np.array([x, y, -pit_depth + radius + k * pitch + 0.002]) + np.r_[rng.uniform(-0.02, 0.02, 2) * radius, 0]
                parts.append(f'<body name="regolith_{n}" pos="{fmt(p)}"><freejoint/>'
                             f'<geom type="sphere" size="{radius}" class="regolith"/></body>')
                n += 1

    xml = f"""<mujoco model="bucket_dig_test">
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" solver="CG"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <default>
    <geom friction="0.8 0.01 0.001" solref="0.004 1"/>
    <default class="regolith">
      <geom condim="6" density="1500" friction="0.9 0.02 0.002" rgba="0.55 0.5 0.45 1" {REGOLITH_BITS}/>
    </default>
  </default>
  <asset>{''.join(assets)}</asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="pit_floor" type="plane" size="5 5 0.1" pos="0 0 {-pit_depth}" rgba="0.5 0.47 0.43 1"/>
    {''.join(parts)}
  </worldbody>
  <actuator>{''.join(actuators)}</actuator>
</mujoco>"""
    return xml, n


class Robot:
    """Drives the chassis by script; runs the lift and belt through their
    force-limited motors."""

    def __init__(self, model, g, belt_torque):
        b = g["belt"]
        self.length, self.radius = b["shaft_distance"], b["pitch_radius"]
        self.start = np.array(b["start_positions"])
        self.buckets = b["buckets"]
        self.act = {name: model.actuator(name).id for name in
                    ["lift"] + [f"{k}_{j}" for k in self.buckets for j in ("u", "v", "rot")]}
        self.chassis = model.body("chassis").mocapid[0]
        self.lift_qpos = model.jnt_qposadr[model.joint("lift").id]
        # Forward = toward the excavator, perpendicular to the wheel axles.
        self.forward = np.cross(g["shaft_axis"], [0, 0, 1.0])
        self.forward /= np.linalg.norm(self.forward)
        self.origin = np.zeros(3)
        self.belt_torque = belt_torque
        self.s = 0.0           # belt travel (m)
        self.v = 0.0           # belt speed (m/s)
        self.load = 0.0        # torque the belt needs at the drive roller (N*m), filtered
        self.motor = 0.0       # torque the belt motor gives (N*m)
        self.lift_target = 0.0
        # Running totals for the summary.
        self.peak_lift, self.peak_belt, self.limited_time, self.stalled_time = 0.0, 0.0, 0.0, 0.0

    def set_targets(self, data):
        """Point the bucket servos at their places on the loop at travel s."""
        for k, s0 in zip(self.buckets, self.start):
            u, v, th = loop_pose(s0 + self.s, self.length, self.radius)
            data.ctrl[self.act[f"{k}_u"]] = u
            data.ctrl[self.act[f"{k}_v"]] = v
            data.ctrl[self.act[f"{k}_rot"]] = th

    def belt_load(self, data):
        """Torque at the drive roller needed to keep the buckets moving:
        the bucket servo forces along the belt (work per metre of travel)
        times the roller radius."""
        F, h = 0.0, 1e-4
        for k, s0 in zip(self.buckets, self.start):
            d = (np.array(loop_pose(s0 + self.s + h, self.length, self.radius))
                 - np.array(loop_pose(s0 + self.s - h, self.length, self.radius))) / (2 * h)
            f = np.array([data.actuator_force[self.act[f"{k}_{j}"]] for j in ("u", "v", "rot")])
            F += f @ d
        return F * self.radius

    def step(self, model, data, belt_speed, advance, lift):
        dt = model.opt.timestep
        data.mocap_pos[self.chassis] = self.origin + advance * self.forward
        # Lift: the target moves no faster than the actuators (2"/s).
        step = LIFT_VELOCITY * dt
        self.lift_target += np.clip(lift - self.lift_target, -step, step)
        data.ctrl[self.act["lift"]] = self.lift_target
        # Belt: speed loop on the motor, capped at its torque; the worm gear
        # can't be back-driven, so the belt never runs against the motor.
        a = dt / LOAD_FILTER
        self.load += a * (self.belt_load(data) - self.load)
        v_cmd = np.clip(belt_speed, -ROLLER_VELOCITY * self.radius, ROLLER_VELOCITY * self.radius)
        self.motor = float(np.clip(self.load + BELT_GAIN * (v_cmd - self.v), -self.belt_torque, self.belt_torque))
        self.v += (self.motor - self.load) / self.radius / BELT_MASS * dt
        if v_cmd >= 0:
            self.v = min(max(self.v, 0.0), v_cmd)
        else:
            self.v = max(min(self.v, 0.0), v_cmd)
        self.s += self.v * dt
        self.set_targets(data)
        mujoco.mj_step(model, data)
        self.peak_lift = max(self.peak_lift, abs(self.lift_force(data)))
        self.peak_belt = max(self.peak_belt, abs(self.motor))
        if v_cmd != 0 and abs(self.motor) >= self.belt_torque * 0.999:
            self.limited_time += dt
            if self.v == 0.0:
                self.stalled_time += dt

    def lift_pos(self, data):
        return float(data.qpos[self.lift_qpos])

    def lift_force(self, data):
        return float(data.actuator_force[self.act["lift"]])


def counts(model, data, g, n, pile_top, robot):
    lo, hi = g["bin"]
    P = np.array([data.xpos[model.body(f"regolith_{i}").id] for i in range(n)])
    local = P - data.mocap_pos[robot.chassis]  # chassis only translates
    in_bin = np.all((local >= lo) & (local <= hi), axis=1)
    lifted = (P[:, 2] > pile_top + 0.05) & ~in_bin
    return int(in_bin.sum()), int(lifted.sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--viewer", action="store_true", help="open the MuJoCo viewer")
    ap.add_argument("--duration", type=float, default=30.0, help="seconds of belt running")
    ap.add_argument("--speed", type=float, default=0.15, help="belt speed in m/s (+ = dig direction)")
    ap.add_argument("--lift", type=float, default=-0.12, help="dig depth: ladder position on its rails (m, + = up)")
    ap.add_argument("--start-lift", type=float, default=0.0, help="ladder position while the pile settles (m)")
    ap.add_argument("--plunge", type=float, default=0.05, help="speed the ladder is lowered into the pile (m/s); "
                    f"the lift can't go faster than the PA-14's {LIFT_VELOCITY:.4f}")
    ap.add_argument("--lift-force", type=float, default=LIFT_EFFORT,
                    help=f"lift actuators' combined force limit (N; default {LIFT_EFFORT:.0f}, 2 x PA-14-6-35)")
    ap.add_argument("--belt-torque", type=float, default=ROLLER_EFFORT,
                    help=f"belt motor torque limit at the drive roller (N*m; default {ROLLER_EFFORT:.0f}, TSINY)")
    ap.add_argument("--advance", type=float, default=0.005, help="robot creep speed into the pile (m/s)")
    ap.add_argument("--radius", type=float, default=0.012, help="regolith particle radius (m)")
    ap.add_argument("--layers", type=int, default=4, help="layers of particles in the pile")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--save-xml", help="also write the generated MJCF here")
    args = ap.parse_args()

    g = load_geometry()
    xml, n = build_xml(g, args.radius, args.layers, args.seed, args.lift_force)
    if args.save_xml:
        with open(args.save_xml, "w") as f:
            f.write(xml)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    belt = Robot(model, g, args.belt_torque)
    belt.lift_target = args.start_lift
    data.qpos[belt.lift_qpos] = args.start_lift
    data.ctrl[belt.act["lift"]] = args.start_lift
    belt.set_targets(data)
    for k, s0 in zip(belt.buckets, belt.start):  # start the buckets on their targets
        for j, val in zip(("u", "v", "rot"), loop_pose(s0, belt.length, belt.radius)):
            data.qpos[model.jnt_qposadr[model.joint(f"{k}_{j}").id]] = val
    mujoco.mj_forward(model, data)

    settle = 1.0  # let the pile settle before the belt starts
    pile_top = args.radius * 2 * args.layers - PIT_DEPTH
    print(f"{n} particles of {args.radius * 200:.1f} cm, belt {args.speed:+.2f} m/s, "
          f"lift {args.lift:+.3f} m, advance {args.advance * 1000:.0f} mm/s, {args.duration:.0f} s; "
          f"limits: lift {args.lift_force:.0f} N, belt {args.belt_torque:.1f} N*m", flush=True)

    def step_once():
        t = data.time
        running = max(0.0, t - settle)
        # Plunge: lower the ladder from start_lift into the pile once the belt runs.
        lift = max(args.lift, args.start_lift - running * args.plunge)
        belt.step(model, data, args.speed if running > 0 else 0.0, running * args.advance, lift)

    report_every, next_report = 2.0, settle
    wall0 = time.time()
    end = settle + args.duration
    best_bin = 0

    def report():
        nonlocal best_bin
        in_bin, lifted = counts(model, data, g, n, pile_top, belt)
        best_bin = max(best_bin, in_bin)
        rtf = data.time / max(time.time() - wall0, 1e-6)
        print(f"t={data.time:5.1f}s  in bin {in_bin:4d}  lifted {lifted:4d}  "
              f"ladder {belt.lift_pos(data):+.3f} m, lift {belt.lift_force(data):+5.0f} N  "
              f"belt {belt.v:.3f} m/s, {belt.motor:+5.2f} N*m  (sim speed {rtf:.2f}x real time)", flush=True)

    if args.viewer:
        from mujoco import viewer
        with viewer.launch_passive(model, data) as v:
            while v.is_running() and data.time < end:
                t0 = time.time()
                for _ in range(10):
                    step_once()
                v.sync()
                if data.time >= next_report:
                    report()
                    next_report += report_every
                time.sleep(max(0.0, 10 * model.opt.timestep - (time.time() - t0)))
            in_bin, lifted = counts(model, data, g, n, pile_top, belt)
            status = ("done" if data.time >= end
                      else f"stopped early at t={data.time:.1f}s (window closed)")
            print(f"{status}: {in_bin} of {n} particles in the bin (max {max(best_bin, in_bin)}), "
                  f"{lifted} still on buckets", flush=True)
            print(motor_summary(belt, data), flush=True)
            if data.time >= end:
                print("close the window to exit", flush=True)
            while v.is_running():  # leave the final state on screen
                v.sync()
                time.sleep(0.05)
        # The viewer's OpenGL teardown can segfault on Linux at interpreter exit.
        os._exit(0)
    else:
        while data.time < end:
            step_once()
            if data.time >= next_report:
                report()
                next_report += report_every
    in_bin, lifted = counts(model, data, g, n, pile_top, belt)
    print(f"done: {in_bin} of {n} particles in the bin (max {max(best_bin, in_bin)}), {lifted} still on buckets", flush=True)
    print(motor_summary(belt, data), flush=True)


def motor_summary(belt, data):
    return (f"motors: ladder reached {belt.lift_pos(data):+.3f} m, lift force peak {belt.peak_lift:.0f} N; "
            f"belt torque peak {belt.peak_belt:.2f} N*m, at its limit for {belt.limited_time:.1f} s "
            f"(stalled {belt.stalled_time:.1f} s), belt travelled {belt.s:.2f} m")


if __name__ == "__main__":
    main()
