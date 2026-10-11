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

The buckets follow the belt path exactly (MuJoCo mocap bodies), like a
chain drive that is strong enough to never stall. Particles are spheres
with rolling friction, so a pile stays a pile.

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
    load_stl_vertices, loop_pose, origin_to_T, rot_w, world_poses)

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
                pit_centre=W["rod_1"][:2, 3])


def quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.flatten())
    return q


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def build_xml(g, radius, layers, seed, pit_depth=PIT_DEPTH):
    # Ladder, rollers and bin are built in the robot frame and carried by one
    # mocap body ("chassis"), so --advance can drive the whole robot forward.
    L = g["L"]

    chassis = []
    # Rollers: cylinders on the shafts.
    ax = g["shaft_axis"]
    Rcyl = np.column_stack([np.cross(ax, [0, 0, 1.0]), np.zeros(3), ax])
    Rcyl[:, 0] /= np.linalg.norm(Rcyl[:, 0])
    Rcyl[:, 1] = np.cross(Rcyl[:, 2], Rcyl[:, 0])
    ladder = []  # rollers ride the lift, so they get their own body
    for i, c in enumerate(g["rollers"]):
        ladder.append(f'<geom name="roller_{i}" type="cylinder" size="{ROLLER_RADIUS} {ROLLER_LENGTH / 2}" '
                      f'pos="{fmt(c)}" quat="{fmt(quat(Rcyl))}" rgba="0 0 0 0"/>')

    # Bin: the CAD hopper's walls.
    assets = []
    for i, w in enumerate(g["bin_walls"]):
        assets.append(f'<mesh name="bin_wall_{i}" vertex="{fmt(w.flatten())}"/>')
        chassis.append(f'<geom name="bin_wall_{i}" type="mesh" mesh="bin_wall_{i}" rgba="0 0 0 0"/>')

    # The rest of the robot: CAD meshes, display only (no collisions, no mass).
    for i, v in enumerate(g["visuals"]):
        assets.append(f'<mesh name="cad_{i}" file="{v["file"]}" inertia="shell"/>')
        geom = (f'<geom type="mesh" mesh="cad_{i}" contype="0" conaffinity="0" group="1" '
                f'pos="{fmt(v["T"][:3, 3])}" quat="{fmt(quat(v["T"][:3, :3]))}" rgba="{v["rgba"]}"/>')
        (ladder if v["ladder"] else chassis).append(geom)
    # base_link sits on the ground, so the robot's frame is the world frame at start.
    parts = [f'<body name="chassis" mocap="true">{"".join(chassis)}</body>',
             f'<body name="ladder" mocap="true">{"".join(ladder)}</body>']

    # Buckets: mocap bodies built from the CAD bucket's walls.
    for i, w in enumerate(g["bucket_walls"]):
        assets.append(f'<mesh name="bucket_wall_{i}" vertex="{fmt(w.flatten())}"/>')
    for b in g["belt"]["buckets"]:
        geoms = "".join(f'<geom type="mesh" mesh="bucket_wall_{i}" rgba="0.9 0.55 0.1 1"/>'
                        for i in range(len(g["bucket_walls"])))
        parts.append(f'<body name="{b}" mocap="true">{geoms}</body>')

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
      <geom condim="6" density="1500" friction="0.9 0.02 0.002" rgba="0.55 0.5 0.45 1"/>
    </default>
  </default>
  <asset>{''.join(assets)}</asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="pit_floor" type="plane" size="5 5 0.1" pos="0 0 {-pit_depth}" rgba="0.5 0.47 0.43 1"/>
    {''.join(parts)}
  </worldbody>
</mujoco>"""
    return xml, n


class Robot:
    """Drives the chassis forward and the buckets round the belt loop."""

    def __init__(self, model, g):
        self.L = g["L"]
        b = g["belt"]
        self.length, self.radius = b["shaft_distance"], b["pitch_radius"]
        self.start = np.array(b["start_positions"])
        self.ids = [model.body(name).mocapid[0] for name in b["buckets"]]
        self.chassis = model.body("chassis").mocapid[0]
        self.ladder = model.body("ladder").mocapid[0]
        # Forward = toward the excavator, perpendicular to the wheel axles.
        self.forward = np.cross(g["shaft_axis"], [0, 0, 1.0])
        self.forward /= np.linalg.norm(self.forward)
        self.origin = np.zeros(3)

    def set(self, data, travel, advance, lift):
        base = self.origin + advance * self.forward
        ladder = base + lift * self.L[:3, 0]  # the lift slides the ladder along its own u axis
        data.mocap_pos[self.chassis] = base
        data.mocap_pos[self.ladder] = ladder
        for mid, s0 in zip(self.ids, self.start):
            u, v, th = loop_pose(s0 + travel, self.length, self.radius)
            T = self.L @ np.block([[rot_w(th), np.array([[u], [v], [0.0]])], [np.zeros((1, 3)), np.ones((1, 1))]])
            data.mocap_pos[mid] = T[:3, 3] + ladder
            data.mocap_quat[mid] = quat(T[:3, :3])


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
    ap.add_argument("--plunge", type=float, default=0.05, help="speed the ladder is lowered into the pile (m/s); the PA-14 lift actuator does 0.05")
    ap.add_argument("--advance", type=float, default=0.005, help="robot creep speed into the pile (m/s)")
    ap.add_argument("--radius", type=float, default=0.012, help="regolith particle radius (m)")
    ap.add_argument("--layers", type=int, default=4, help="layers of particles in the pile")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--save-xml", help="also write the generated MJCF here")
    args = ap.parse_args()

    g = load_geometry()
    xml, n = build_xml(g, args.radius, args.layers, args.seed)
    if args.save_xml:
        with open(args.save_xml, "w") as f:
            f.write(xml)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    belt = Robot(model, g)
    belt.set(data, 0.0, 0.0, args.start_lift)
    mujoco.mj_forward(model, data)

    settle = 1.0  # let the pile settle before the belt starts
    pile_top = args.radius * 2 * args.layers - PIT_DEPTH
    print(f"{n} particles of {args.radius * 200:.1f} cm, belt {args.speed:+.2f} m/s, "
          f"lift {args.lift:+.3f} m, advance {args.advance * 1000:.0f} mm/s, {args.duration:.0f} s", flush=True)

    def step_once():
        t = data.time
        running = max(0.0, t - settle)
        # Plunge: lower the ladder from start_lift into the pile once the belt runs.
        lift = max(args.lift, args.start_lift - running * args.plunge)
        belt.set(data, running * args.speed, running * args.advance, lift)
        mujoco.mj_step(model, data)

    report_every, next_report = 2.0, settle
    wall0 = time.time()
    end = settle + args.duration
    best_bin = 0

    def report():
        nonlocal best_bin
        in_bin, lifted = counts(model, data, g, n, pile_top, belt)
        best_bin = max(best_bin, in_bin)
        rtf = data.time / max(time.time() - wall0, 1e-6)
        print(f"t={data.time:5.1f}s  in bin {in_bin:4d}  lifted {lifted:4d}  (sim speed {rtf:.2f}x real time)", flush=True)

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


if __name__ == "__main__":
    main()
