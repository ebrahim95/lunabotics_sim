#!/usr/bin/env python3
"""
Repairs the kinematic tree of the Onshape-exported URDF so the wheels and
the excavator move the way the real robot does.

What the raw export gets wrong:
  * All four wheels are fastened to flange_1, so only revolute_1_3 turns
    anything and it spins every wheel about that one flange's axis. The
    other three wheel joints turn empty flanges. The two sides also have
    opposite axis signs.
  * The chassis frame is not attached to `root` at all -- Onshape hung the
    excavator off root (hanging_node_to_root_joint -> bucket_1) and the
    whole frame dangles below it through belt -> roller -> rod -> slider.
    The belt joint (cylindrical_1_1) therefore spins the chassis around
    the excavator instead of turning a roller.
  * Closed loops are exported as massless *_loop_closure duplicate links
    that nothing in ROS/Gazebo can close.
  * <mimic> tags chain the wheel joints together with per-side signs.
  * The buckets are fastened rigidly to the belt, so Gazebo merges them into
    the ladder and nothing can move them around the loop.
  * No link has <collision> geometry, so the robot falls through the ground.
  * The tree is rooted at a motor part ~5 degrees off the chassis, so the
    robot's frame isn't square to the robot (see add_base_frame).
    Collisions are generated from the meshes (see COLLISION_SHAPES).

What this script produces (every part keeps its exact pose relative to the
others; the whole robot is re-based on a ROS-style base_link):
  base_link  (x forward, y left, z up; origin on the ground between the wheels)
   '- root
   |- motors -> continuous revolute_1* -> flange_k -> wheel   (one wheel per flange)
   '- 30___30___700_1 (chassis frame) ... 30___60___400_3 -> slider_bracket
        '- prismatic excavator_lift -> ladder (side plates, both shafts, belt, buckets)
             |- continuous drive_roller_joint -> roller_1   (top roller, driven)
             |- continuous idler_roller_joint -> roller     (bottom roller, passive)
             '- per bucket: <b>_belt_u (prismatic, along ladder)
                            -> <b>_belt_v (prismatic, across ladder)
                            -> <b>_belt_rot (continuous, about the shafts) -> bucket
                belt_animator.py sets these from the drive roller angle so the
                buckets run round the belt loop.

The bucket links are re-framed so joint value 0 is the bottom shaft centre;
their CAD pose sits at the joint values written to config/belt_animator.yaml
(as each bucket's start position along the loop).
Usage:
    ros2 run lunabotics_model fix_kinematics.py \
        --input urdf/assembly_1.urdf \
        --output urdf/assembly_1_fixed.urdf \
        --config-dir config

Run this on every fresh Onshape export, then feed the output to
add_ros2_control.py.
"""
import argparse
import math
import os
import struct
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

# ---- Tune these to match the real hardware ------------------------------
# Excavator lift: two Progressive Automations PA-14-6-35 linear actuators,
# one each side of the ladder, driven together. Each: 12 V, 6" stroke,
# 35 lb force, 2.00"/s no load (1.38"/s full load); assumed to push
# straight along the ladder rails. Modelled as one joint with their
# combined force. The CAD pose (buckets just clear of
# the ground) is taken as fully retracted, so the stroke lowers the ladder.
# Positive = ladder slides up the rails (away from the ground).
LIFT_LOWER = -6 * 0.0254         # m, full 6" stroke down from the CAD pose
LIFT_UPPER = 0.0                 # m, retracted (CAD pose)
LIFT_ACTUATORS = 2
LIFT_EFFORT = LIFT_ACTUATORS * 35 * 4.448222   # N (2 x 35 lb)
LIFT_VELOCITY = 2.0 * 0.0254     # m/s (2.00"/s, no load)
# Belt drive: TSINY TS-30GZ6287-R-LS 12 V worm-gear motor, 50 rpm version,
# ~6 N*m rated (seller listings), assumed direct on the drive roller (1:1).
ROLLER_EFFORT = 6.0                      # N*m
ROLLER_VELOCITY = 50 * 2 * math.pi / 60  # rad/s (50 rpm -> ~0.25 m/s belt speed)
ROLLER_DAMPING = 0.01
BUCKET_EFFORT = 200.0   # N / N*m holding each bucket on its track
BUCKET_VELOCITY = 5.0   # m/s, rad/s
# Drive gearmotors: 57BLR70-24-02-HG50 (StepperOnline datasheet): 24 V
# 172 W brushless motor + 50:1 planetary gearbox, rated 17.63 N*m at
# 70 rpm, gearbox max permissible 20 N*m (40 N*m momentary), 1.49 kg.
WHEEL_EFFORT = 20.0                      # N*m, gearbox continuous limit
WHEEL_VELOCITY = 70 * 2 * math.pi / 60   # rad/s, rated speed (70 rpm)
DRIVE_MOTOR_MASS = 1.49                  # kg each, datasheet
DRIVE_MOTOR_INBOARD = 0.10               # m, motor body sits this far inboard of its wheel flange
# Whole-robot mass. The CAD export has no materials on most parts (motors
# weigh micrograms, wheels 9 kg each), so masses are rebuilt to this total:
# drive motors from the datasheet, the frame and excavator parts from CAD,
# the assumptions below for parts the CAD leaves massless, and the wheels
# take whatever is left. Edit the assumptions if you know better.
TOTAL_MASS = 50.0                        # kg, measured
ELECTRONICS_MASS = 5.0                   # kg, ASSUMED: battery + electronics box
BIN_MASS = 1.5                           # kg, ASSUMED: regolith bin
WHEEL_JOINTS = ["revolute_1", "revolute_1_1", "revolute_1_2", "revolute_1_3"]
WHEELS = ["wheel", "wheel_1", "wheel_mirrored", "wheel_mirrored_1"]
# -------------------------------------------------------------------------

# Collision shape per mesh file. Anything not listed gets a box fitted to
# the mesh's bounding box, which suits the aluminium extrusions, brackets
# and motor parts (their meshes are axis-aligned in their own frames).
#   ("cylinder", axis)  -- axis 0/1/2 = mesh x/y/z; radius/length from the mesh
#   ("mesh",)           -- the visual mesh itself (only for low-poly concave
#                          parts where the shape matters, e.g. scoops)
#   None                -- no collision
COLLISION_SHAPES = {
    "wheel.stl": ("cylinder", 1),
    "wheel_Mirrored.stl": ("cylinder", 1),
    "roller.stl": ("cylinder", 2),
    "rod.stl": ("cylinder", 2),
    "bucket_1.stl": ("mesh",),   # excavator bucket, 36 triangles
    "bucket.stl": ("mesh",),     # regolith bin, 24 triangles
    # The belt band is static in the sim (only the buckets travel), so a
    # solid belt would drag on regolith the real moving belt wouldn't.
    "belt.stl": None,
    # Hidden inside the wheel hub, and Gazebo's WheelSlip needs each wheel
    # link (wheel merged into its flange) to have exactly one collision.
    "flange.stl": None,
}

# Dummy links Onshape adds for closed loops / cylindrical mates. They have
# no geometry and only exist to carry the bogus joints removed below.
DROP_LINKS = {"cylindrical_1", "cylindrical_1_1"}

# Raw-export joints that encode the wrong structure.
DROP_JOINTS = {
    "hanging_node_to_root_joint",  # root -> bucket_1 (excavator hung off root)
    "fastened_15", "fastened_15_1", "fastened_15_2", "fastened_15_3",  # all wheels on flange_1
    "fastened_4_1",  # belt -> roller (rigid; roller must spin)
    "fastened_3_1",  # belt -> roller_1 (rigid; roller must spin)
    "slider_1_1",    # second slider bracket rides the ladder instead of the frame
}

# (name, type, parent, child, axis_in_world or None). Origins are computed
# from the current world poses so nothing moves.
def new_joints(W):
    shaft_axis = W["rod"][:3, 2]  # rod's z-axis runs along the roller shafts
    return [
        ("chassis_mount", "fixed", "root", "30___30___700_1", None),
        ("ladder_bottom_shaft", "fixed", "rod", "rod_1", None),
        ("belt_mount", "fixed", "rod", "belt", None),
        ("slider_bracket_1_mount", "fixed", "30___60___400_3", "slider_bracket_1", None),
        ("drive_roller_joint", "continuous", "rod", "roller_1", shaft_axis),
        ("idler_roller_joint", "continuous", "rod_1", "roller", shaft_axis),
    ]

RENAME = {"slider_1": "excavator_lift"}


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def R_to_rpy(R):
    p = np.arcsin(-np.clip(R[2, 0], -1.0, 1.0))
    if abs(np.cos(p)) > 1e-9:
        r = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        r = 0.0
        y = np.arctan2(-R[0, 1], R[1, 1])
    return r, p, y


def origin_to_T(el):
    T = np.eye(4)
    if el is not None:
        T[:3, :3] = rpy_to_R(*map(float, el.get("rpy", "0 0 0").split()))
        T[:3, 3] = list(map(float, el.get("xyz", "0 0 0").split()))
    return T


def fmt(v):
    return " ".join(f"{x:.9g}" for x in v)


def set_origin(joint, T):
    o = joint.find("origin")
    if o is None:
        o = ET.SubElement(joint, "origin")
    o.set("xyz", fmt(T[:3, 3]))
    o.set("rpy", fmt(R_to_rpy(T[:3, :3])))


def set_axis(joint, a):
    ax = joint.find("axis")
    if ax is None:
        ax = ET.SubElement(joint, "axis")
    a = a / np.linalg.norm(a)
    a = np.where(np.abs(a) < 1e-5, 0.0, a)  # CAD rounding noise
    ax.set("xyz", fmt(a / np.linalg.norm(a)))


def set_limit(joint, **kw):
    lim = joint.find("limit")
    if lim is None:
        lim = ET.SubElement(joint, "limit")
    for k, v in kw.items():
        lim.set(k, f"{v:.9g}")


def joint_motion(joint, q):
    """Transform produced by moving `joint` to position q (in its own frame)."""
    M = np.eye(4)
    if not q or joint.get("type") == "fixed":
        return M
    a = np.array(list(map(float, joint.find("axis").get("xyz").split())))
    if joint.get("type") == "prismatic":
        M[:3, 3] = a * q
    else:  # Rodrigues
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        M[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K
    return M


def tree_root(robot):
    """The one link that is nobody's child."""
    child_links = {j.find("child").get("link") for j in robot.findall("joint")}
    roots = [l.get("name") for l in robot.findall("link") if l.get("name") not in child_links]
    if len(roots) != 1:
        raise RuntimeError(f"Expected one root link, found {roots}")
    return roots[0]


def world_poses(robot, q=None):
    """Pose of every link frame in the root link's frame (joints at q, default 0)."""
    q = q or {}
    children = {}
    for j in robot.findall("joint"):
        children.setdefault(j.find("parent").get("link"), []).append(j)
    root = tree_root(robot)
    W = {root: np.eye(4)}
    stack = [root]
    while stack:
        p = stack.pop()
        for j in children.get(p, []):
            c = j.find("child").get("link")
            W[c] = W[p] @ origin_to_T(j.find("origin")) @ joint_motion(j, q.get(j.get("name")))
            stack.append(c)
    return W


def com_world(link, W):
    i = link.find("inertial")
    T = W[link.get("name")]
    return (T @ origin_to_T(i.find("origin")) if i is not None else T)[:3, 3]


MESH_ROOT = None  # package root, set by fix() for mesh lookups


def fix(robot, mesh_root):
    global MESH_ROOT
    MESH_ROOT = mesh_root
    W = world_poses(robot)
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = {j.get("name"): j for j in robot.findall("joint")}

    # Axis of every movable joint, in world, before we touch anything.
    axis_world = {}
    for n, j in joints.items():
        if j.get("type") != "fixed" and j.find("axis") is not None:
            a = np.array(list(map(float, j.find("axis").get("xyz").split())))
            axis_world[n] = W[j.find("child").get("link")][:3, :3] @ a

    # --- drop dummies and wrong joints ---
    drop_links = DROP_LINKS | {n for n in links if "loop_closure" in n}
    for n in drop_links:
        robot.remove(links.pop(n))
    for n, j in list(joints.items()):
        if (n in DROP_JOINTS or j.find("parent").get("link") in drop_links
                or j.find("child").get("link") in drop_links):
            robot.remove(joints.pop(n))

    # Onshape couples the wheel joints (and the sliders) with <mimic> tags.
    # Each joint is commanded on its own through ros2_control, the signs
    # assume the old per-side axes, and some point at joints dropped above.
    for j in joints.values():
        for m in j.findall("mimic"):
            j.remove(m)

    # --- wheels: each wheel onto the flange its own joint drives ---
    flange_of = {j: joints[j].find("child").get("link") for j in WHEEL_JOINTS}
    wheel_mounts = []
    for wheel in WHEELS:
        c = com_world(links[wheel], W)
        jn = min(WHEEL_JOINTS, key=lambda j: np.linalg.norm(W[flange_of[j]][:3, 3] - c))
        wheel_mounts.append((f"{wheel}_mount", "fixed", flange_of[jn], wheel, None))
    if len({m[2] for m in wheel_mounts}) != len(WHEELS):
        raise RuntimeError(f"Wheel/flange pairing is not one-to-one: {wheel_mounts}")

    # Same sign on every wheel axis so +velocity drives every wheel forward.
    ref = axis_world[WHEEL_JOINTS[0]]
    for jn in WHEEL_JOINTS:
        if axis_world[jn] @ ref < 0:
            axis_world[jn] = -axis_world[jn]

    for jn in WHEEL_JOINTS:
        set_limit(joints[jn], effort=WHEEL_EFFORT, velocity=WHEEL_VELOCITY)

    for name, jtype, parent, child, axis in new_joints(W) + wheel_mounts:
        j = ET.SubElement(robot, "joint", name=name, type=jtype)
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=child)
        joints[name] = j
        if axis is not None:
            axis_world[name] = axis

    # --- re-orient every joint by walking out from root ---
    adj = {}
    for n, j in joints.items():
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        adj.setdefault(p, []).append((n, c))
        adj.setdefault(c, []).append((n, p))

    if len(joints) != len(links) - 1:
        raise RuntimeError(f"{len(links)} links but {len(joints)} joints -- not a tree")

    seen, queue = {"root"}, deque(["root"])
    while queue:
        parent = queue.popleft()
        for n, child in adj.get(parent, []):
            if child in seen:
                continue
            seen.add(child)
            queue.append(child)
            j = joints[n]
            j.find("parent").set("link", parent)
            j.find("child").set("link", child)
            # The child link's frame stays where it is, so the joint frame
            # (== child frame) and all geometry are untouched.
            set_origin(j, np.linalg.inv(W[parent]) @ W[child])
            if n in axis_world:
                set_axis(j, W[child][:3, :3].T @ axis_world[n])

    missing = set(links) - seen
    if missing:
        raise RuntimeError(f"Links not reachable from root: {sorted(missing)}")

    # Reversing slider_1 (bracket is now the parent) flips the motion sense.
    # Re-point the axis so +travel always moves the ladder up the rails.
    lift = joints["slider_1"]
    up = axis_world["slider_1"] if axis_world["slider_1"][2] > 0 else -axis_world["slider_1"]
    set_axis(lift, W[lift.find("child").get("link")][:3, :3].T @ up)
    set_limit(lift, lower=LIFT_LOWER, upper=LIFT_UPPER, effort=LIFT_EFFORT, velocity=LIFT_VELOCITY)

    for n in ("drive_roller_joint", "idler_roller_joint"):
        set_limit(joints[n], effort=ROLLER_EFFORT, velocity=ROLLER_VELOCITY)
        ET.SubElement(joints[n], "dynamics", damping=f"{ROLLER_DAMPING}")

    for old, new in RENAME.items():
        joints[old].set("name", new)

    # Sanity check: forward kinematics of the new tree must match the old one.
    W_new = world_poses(robot)
    worst = max(np.abs(W_new[n] - W[n]).max() for n in links)
    if worst > 1e-6:
        raise RuntimeError(f"Link poses moved by up to {worst:g} -- refusing to write")

    belt = add_belt_track(robot, W)
    add_collisions(robot, mesh_root)
    add_base_frame(robot)
    set_masses(robot)

    # Keep joints grouped and ordered for readability.
    for j in sorted(robot.findall("joint"), key=lambda j: j.get("name")):
        robot.remove(j)
        robot.append(j)
    return sorted(drop_links), belt


def add_base_frame(robot):
    """Add `base_link` as the new root: ROS-style chassis frame.

    Onshape roots the tree at a motor part that sits ~5 degrees off the
    chassis, so `root` is not square to the robot. base_link is: x forward
    (toward the excavator, perpendicular to the wheel axles), y left, z up,
    origin on the ground centred between the four wheels. `root` hangs off it
    on a fixed joint, so every part keeps its pose relative to the others.
    """
    W = world_poses(robot)
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = {j.get("name"): j for j in robot.findall("joint")}

    centres, radius = [], None
    for wheel in WHEELS:
        col = links[wheel].find("collision")
        centres.append((W[wheel] @ origin_to_T(col.find("origin")))[:3, 3])
        radius = float(col.find("geometry/cylinder").get("radius"))
    centres = np.array(centres)

    # Wheel axle direction (all wheel joints share it after fix()).
    j = joints[WHEEL_JOINTS[0]]
    axle = W[j.find("child").get("link")][:3, :3] @ np.array(list(map(float, j.find("axis").get("xyz").split())))
    up = np.array([0.0, 0.0, 1.0])
    axle = axle - (axle @ up) * up
    axle /= np.linalg.norm(axle)
    forward = np.cross(axle, up)  # + wheel speed drives this way (toward the excavator)
    left = np.cross(up, forward)

    B = np.eye(4)
    B[:3, :3] = np.column_stack([forward, left, up])
    B[:3, 3] = [centres[:, 0].mean(), centres[:, 1].mean(), centres[:, 2].min() - radius]

    base = ET.Element("link", name="base_link")
    robot.insert(0, base)
    jb = ET.SubElement(robot, "joint", name="base_link_to_root", type="fixed")
    ET.SubElement(jb, "parent", link="base_link")
    ET.SubElement(jb, "child", link="root")
    set_origin(jb, np.linalg.inv(B))

    # Relative poses must be unchanged: W_new = inv(B) @ W_old for every link.
    W_new = world_poses(robot)
    worst = max(np.abs(W_new[n] - np.linalg.inv(B) @ W[n]).max() for n in W)
    if worst > 1e-6:
        raise RuntimeError(f"Adding base_link moved parts by up to {worst:g}")
    yaw = np.degrees(np.arctan2(forward[1], forward[0]))
    print(f"base_link: root was {abs(yaw + 90):.2f} deg off square; "
          f"wheels touch the ground at base_link z = 0")


def set_inertial(link, mass, com, inertia):
    """Replace a link's <inertial> (com and inertia in the link frame)."""
    old = link.find("inertial")
    if old is not None:
        link.remove(old)
    el = ET.Element("inertial")
    ET.SubElement(el, "origin", xyz=fmt(com), rpy="0 0 0")
    ET.SubElement(el, "mass", value=f"{mass:.6g}")
    I = inertia
    ET.SubElement(el, "inertia", ixx=f"{I[0, 0]:.6g}", ixy=f"{I[0, 1]:.6g}", ixz=f"{I[0, 2]:.6g}",
                  iyy=f"{I[1, 1]:.6g}", iyz=f"{I[1, 2]:.6g}", izz=f"{I[2, 2]:.6g}")
    link.insert(0, el)


def box_inertia(mass, size):
    x, y, z = size
    return np.diag([y * y + z * z, x * x + z * z, x * x + y * y]) * mass / 12


def set_masses(robot):
    """Rebuild masses to the real robot's (see TOTAL_MASS and friends)."""
    W = world_poses(robot)  # base_link frame
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = {j.get("name"): j for j in robot.findall("joint")}
    mass = lambda n: float(links[n].find("inertial/mass").get("value")) if links[n].find("inertial") is not None else 0.0

    def mesh_box(name):
        """Centre and size of a link's visual mesh, in the link frame."""
        vis = links[name].find("visual")
        T = origin_to_T(vis.find("origin"))
        rel = vis.find("geometry/mesh").get("filename").split("://", 1)[-1].split("/", 1)[-1]
        V = load_stl_vertices(os.path.join(MESH_ROOT, rel))
        P = (T[:3, :3] @ V.T).T + T[:3, 3]
        return (P.min(0) + P.max(0)) / 2, P.max(0) - P.min(0)

    # Drive motors: four point masses on the chassis (root link), just
    # inboard of each wheel flange.
    to_root = np.linalg.inv(W["root"])
    pts = []
    for jn in WHEEL_JOINTS:
        p = W[joints[jn].find("child").get("link")][:3, 3].copy()
        p[1] -= math.copysign(DRIVE_MOTOR_INBOARD, p[1])
        pts.append((to_root @ np.append(p, 1))[:3])
    pts = np.array(pts)
    m_motors = DRIVE_MOTOR_MASS * len(pts)
    com = pts.mean(0)
    I = sum(DRIVE_MOTOR_MASS * ((r @ r) * np.eye(3) - np.outer(r, r)) for r in pts - com)
    set_inertial(links["root"], m_motors, com, I + 1e-4 * np.eye(3))

    for name, m in (("electronics", ELECTRONICS_MASS), ("bucket", BIN_MASS)):
        c, size = mesh_box(name)
        set_inertial(links[name], m, c, box_inertia(m, size))

    # Wheels take the rest; scale each one's CAD mass and inertia.
    wheel_links = [l for l in links if l in WHEELS]
    others = sum(mass(n) for n in links if n not in wheel_links)
    wheel_total = TOTAL_MASS - others
    if wheel_total <= 0:
        raise RuntimeError(f"Parts other than the wheels already weigh {others:.1f} kg > TOTAL_MASS")
    for n in wheel_links:
        inertial = links[n].find("inertial")
        k = (wheel_total / len(wheel_links)) / mass(n)
        inertial.find("mass").set("value", f"{mass(n) * k:.6g}")
        el = inertial.find("inertia")
        for a in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            el.set(a, f"{float(el.get(a)) * k:.6g}")
    total = sum(mass(n) for n in links)
    print(f"masses: total {total:.2f} kg (drive motors {m_motors:.2f}, electronics {ELECTRONICS_MASS}, "
          f"bin {BIN_MASS}, wheels {wheel_total / len(wheel_links):.2f} kg each, rest from CAD)")


def load_stl_vertices(path):
    """Vertices of a binary or ASCII STL as an (N, 3) array."""
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * n == len(data):
            tris = np.frombuffer(data[84:], dtype=np.dtype(
                [("n", "<3f4"), ("v", "<9f4"), ("attr", "<u2")]))
            return tris["v"].reshape(-1, 3).astype(float)
    verts = [list(map(float, line.split()[1:4]))
             for line in data.decode(errors="ignore").splitlines()
             if line.strip().startswith("vertex")]
    return np.array(verts)


def add_collisions(robot, mesh_root):
    """Give every visual mesh a collision shape (see COLLISION_SHAPES)."""
    cache = {}
    for link in robot.findall("link"):
        if link.find("collision") is not None:
            continue  # keep anything a newer export already provides
        for i, vis in enumerate(link.findall("visual")):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            uri = mesh.get("filename")
            fname = uri.rsplit("/", 1)[-1]
            shape = COLLISION_SHAPES.get(fname, ("box",))
            if shape is None:
                continue
            if fname not in cache:
                rel = uri.split("://", 1)[-1].split("/", 1)[-1]  # strip package://<pkg>/
                cache[fname] = load_stl_vertices(os.path.join(mesh_root, rel))
            V = cache[fname] * np.array(list(map(float, mesh.get("scale", "1 1 1").split())))
            lo, hi = V.min(0), V.max(0)

            local = np.eye(4)
            local[:3, 3] = (lo + hi) / 2
            col = ET.Element("collision", name=f"{link.get('name')}_collision_{i}")
            geom = ET.SubElement(col, "geometry")
            if shape[0] == "cylinder":
                axis = shape[1]
                radial = [k for k in range(3) if k != axis]
                c = (lo + hi) / 2
                radius = np.sqrt(((V[:, radial] - c[radial]) ** 2).sum(1)).max()
                if axis == 0:
                    local[:3, :3] = rpy_to_R(0, np.pi / 2, 0)
                elif axis == 1:
                    local[:3, :3] = rpy_to_R(np.pi / 2, 0, 0)
                ET.SubElement(geom, "cylinder", radius=f"{radius:.6g}",
                              length=f"{hi[axis] - lo[axis]:.6g}")
            elif shape[0] == "mesh":
                local = np.eye(4)
                ET.SubElement(geom, "mesh", filename=uri, scale=mesh.get("scale", "1 1 1"))
            else:
                ET.SubElement(geom, "box", size=fmt(hi - lo))
            T = origin_to_T(vis.find("origin")) @ local
            col.insert(0, ET.Element("origin", xyz=fmt(T[:3, 3]), rpy=fmt(R_to_rpy(T[:3, :3]))))
            # URDF wants <collision> after <visual>; keep each next to its visual.
            link.insert(list(link).index(vis) + 1, col)


def rot_w(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def loop_pose(s, length, radius):
    """(u, v, theta) at arc length s round the belt loop, in the ladder frame.

    Ladder frame: origin on the bottom shaft, u toward the top shaft, w along
    the shafts, v = w x u. s = 0 is at (0, +r); s grows up the +v side, round
    the top shaft, down the -v side and round the bottom shaft. theta is the
    bucket's rotation about w: 0 on the +v side, -pi on the -v side.
    Kept in sync with belt_animator.py.
    """
    total = 2 * length + 2 * np.pi * radius
    laps, s = divmod(s, total)
    if s < length:
        u, v, th = s, radius, 0.0
    elif s < length + np.pi * radius:
        a = (s - length) / radius
        u, v, th = length + radius * np.sin(a), radius * np.cos(a), -a
    elif s < 2 * length + np.pi * radius:
        u, v, th = length - (s - length - np.pi * radius), -radius, -np.pi
    else:
        a = (s - 2 * length - np.pi * radius) / radius
        u, v, th = -radius * np.sin(a), -radius * np.cos(a), -np.pi - a
    return u, v, th - 2 * np.pi * laps


def loop_arclength(u, v, length, radius):
    """Inverse of loop_pose for a point on one of the straight runs."""
    if not -1e-3 <= u <= length + 1e-3:
        raise RuntimeError(f"Bucket at u={u:.4f} is not on a straight run of the belt")
    return u if v > 0 else 2 * length + np.pi * radius - u


def add_belt_track(robot, W):
    """Replace each bucket's fixed belt mount with a u/v/rot joint chain."""
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = {j.get("name"): j for j in robot.findall("joint")}

    mounts = {}
    for n, j in joints.items():
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        if p == "belt" and c.startswith("bucket"):
            mounts[c] = j
    buckets = sorted(mounts, key=lambda b: int(b.rsplit("_", 1)[1]))
    for b in buckets:
        if any(j.find("parent").get("link") == b for j in joints.values()):
            raise RuntimeError(f"{b} has child links; it can't ride the belt on its own")

    # Ladder frame L, in world.
    bottom, top = W["rod_1"][:3, 3], W["rod"][:3, 3]
    length = np.linalg.norm(top - bottom)
    eu = (top - bottom) / length
    ew = W["rod_1"][:3, 2]
    ew = ew - (ew @ eu) * eu
    ew /= np.linalg.norm(ew)
    ev = np.cross(ew, eu)
    L = np.eye(4)
    L[:3, :3] = np.column_stack([eu, ev, ew])
    L[:3, 3] = bottom
    Linv = np.linalg.inv(L)

    # Every bucket is the same part. Find (1) the fixed angle between a
    # bucket's x-axis and its outward belt normal, and (2) where on the part
    # it is attached to the belt, from the buckets whose Onshape link frame
    # *is* their belt mate (frame origin on the belt plane).
    pose = {b: Linv @ W[b] for b in buckets}
    com = {b: (Linv @ np.append(com_world(links[b], W), 1))[:3] for b in buckets}
    side = {b: 1.0 if com[b][1] > 0 else -1.0 for b in buckets}
    tilt = {b: np.arctan2(pose[b][1, 0], pose[b][0, 0]) - side[b] * np.pi / 2 for b in buckets}
    tilt0 = tilt[buckets[0]]
    if any(abs(np.angle(np.exp(1j * (t - tilt0)))) > 1e-3 for t in tilt.values()):
        raise RuntimeError("Buckets are not all mounted at the same angle to the belt")
    theta = {b: 0.0 if side[b] > 0 else -np.pi for b in buckets}
    mated = [b for b in buckets if abs(pose[b][2, 3]) < 1e-4]
    if not mated:
        raise RuntimeError("No bucket frame lies on the belt plane to locate the belt mate")
    offsets = [rot_w(-theta[b]) @ (com[b] - pose[b][:3, 3]) for b in mated]
    com_offset = np.mean(offsets, axis=0)  # COM relative to belt mate, theta = 0
    if np.abs(np.array(offsets) - com_offset).max() > 1e-4:
        raise RuntimeError("Bucket belt mates disagree -- are they all the same part?")

    attach = {b: com[b] - rot_w(theta[b]) @ com_offset for b in buckets}
    radius = float(np.mean([abs(a[1]) for a in attach.values()]))
    if max(abs(abs(a[1]) - radius) for a in attach.values()) > 1e-3 or \
            max(abs(a[2]) for a in attach.values()) > 1e-3:
        raise RuntimeError("Bucket belt mates do not lie on one belt loop")

    start = {b: loop_arclength(attach[b][0], attach[b][1], length, radius) for b in buckets}
    mesh_world = {b: W[b] @ origin_to_T(links[b].find("visual/origin")) for b in buckets}
    q0 = {}
    for b in buckets:
        u, v, th = loop_pose(start[b], length, radius)
        q0.update({f"{b}_belt_u": u, f"{b}_belt_v": v, f"{b}_belt_rot": th})
        A = L @ np.block([[rot_w(th), np.array([[u], [v], [0.0]])], [np.zeros((1, 3)), np.ones((1, 1))]])
        # New link frame is A at these joint values; keep the geometry where it is.
        G = np.linalg.inv(A) @ W[b]
        for tag in ("inertial", "visual", "collision"):
            for el in links[b].findall(tag):
                o = el.find("origin")
                if o is None:
                    o = ET.SubElement(el, "origin")
                T = G @ origin_to_T(o)
                o.set("xyz", fmt(T[:3, 3]))
                o.set("rpy", fmt(R_to_rpy(T[:3, :3])))

        robot.remove(mounts[b])
        chain = [
            (f"{b}_belt_u", "prismatic", "rod", f"{b}_belt_u_link", np.linalg.inv(W["rod"]) @ L,
             (1, 0, 0), dict(lower=-radius - 0.01, upper=length + radius + 0.01)),
            (f"{b}_belt_v", "prismatic", f"{b}_belt_u_link", f"{b}_belt_v_link", np.eye(4),
             (0, 1, 0), dict(lower=-radius - 0.01, upper=radius + 0.01)),
            (f"{b}_belt_rot", "continuous", f"{b}_belt_v_link", b, np.eye(4), (0, 0, 1), {}),
        ]
        for name, jtype, parent, child, origin, axis, lim in chain:
            if child != b:
                # Gazebo drops massless links, so give the carriers a token mass.
                link = ET.SubElement(robot, "link", name=child)
                inertial = ET.SubElement(link, "inertial")
                ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
                ET.SubElement(inertial, "mass", value="0.001")
                ET.SubElement(inertial, "inertia", ixx="1e-07", ixy="0", ixz="0",
                              iyy="1e-07", iyz="0", izz="1e-07")
            j = ET.SubElement(robot, "joint", name=name, type=jtype)
            ET.SubElement(j, "parent", link=parent)
            ET.SubElement(j, "child", link=child)
            set_origin(j, origin)
            set_axis(j, np.array(axis, dtype=float))
            set_limit(j, effort=BUCKET_EFFORT, velocity=BUCKET_VELOCITY, **lim)

    # At their start positions every bucket mesh must sit exactly at its CAD pose.
    W_new = world_poses(robot, q0)
    worst = max(
        np.abs(W_new[b] @ origin_to_T(links[b].find("visual/origin")) - mesh_world[b]).max()
        for b in buckets)
    if worst > 1e-6:
        raise RuntimeError(f"Buckets moved by up to {worst:g} at their start positions")

    # Drive roller turning +theta moves the +v side of the belt toward -u
    # (down the ladder) when its axis is +w, i.e. s decreases.
    drive = joints["drive_roller_joint"]
    drive_axis = W[drive.find("child").get("link")][:3, :3] @ np.array(
        list(map(float, drive.find("axis").get("xyz").split())))
    return dict(buckets=buckets, start=start, length=length, radius=radius,
                direction=-1.0 if drive_axis @ ew > 0 else 1.0)


def write_belt_configs(belt, config_dir):
    header = "# Generated by fix_kinematics.py from the CAD bucket layout. Do not hand-edit.\n"
    names = belt["buckets"]
    joints = [f"{b}_belt_{axis}" for b in names for axis in ("u", "v", "rot")]

    def yaml_list(items, fmt_item):
        return "[" + ", ".join(fmt_item(i) for i in items) + "]"

    with open(os.path.join(config_dir, "belt_animator.yaml"), "w") as f:
        f.write(header)
        f.write("belt_animator:\n  ros__parameters:\n")
        f.write(f"    shaft_distance: {belt['length']:.9g}   # m, bottom to top shaft\n")
        f.write(f"    pitch_radius: {belt['radius']:.9g}   # m, shaft axis to bucket mount\n")
        f.write(f"    belt_direction: {belt['direction']:.1f}   # sign of belt travel per +roller turn\n")
        f.write("    drive_joint: drive_roller_joint\n")
        f.write(f"    buckets: {yaml_list(names, str)}\n")
        f.write(f"    start_positions: {yaml_list(names, lambda b: f"{belt['start'][b]:.9g}")}\n")

    with open(os.path.join(config_dir, "bucket_controller.yaml"), "w") as f:
        f.write(header)
        f.write("bucket_position_controller:\n  ros__parameters:\n")
        f.write("    interface_name: position\n    joints:\n")
        for j in joints:
            f.write(f"      - {j}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", required=True, help="Path to raw Onshape-exported URDF")
    parser.add_argument("--output", required=True, help="Path to write the repaired URDF")
    parser.add_argument("--config-dir", default=None,
                        help="Where to write belt_animator.yaml and bucket_controller.yaml "
                             "(default: ../config next to --output)")
    args = parser.parse_args()
    config_dir = args.config_dir or os.path.join(os.path.dirname(os.path.abspath(args.output)), "..", "config")

    tree = ET.parse(args.input)
    robot = tree.getroot()
    dropped_links, belt = fix(robot, os.path.dirname(os.path.dirname(os.path.abspath(args.input))))

    ET.indent(tree, space="    ")
    with open(args.output, "wb") as f:
        f.write(b"<?xml version='1.0' encoding='utf-8'?>\n")
        f.write(b"<!-- Generated by fix_kinematics.py from an Onshape export. Do not hand-edit;"
                b" re-run the script instead. -->\n")
        tree.write(f, encoding="utf-8", xml_declaration=False)
        f.write(b"\n")
    write_belt_configs(belt, config_dir)

    print(f"Wrote repaired URDF to {args.output}")
    print(f"Wrote belt_animator.yaml and bucket_controller.yaml to {os.path.normpath(config_dir)}")
    print(f"Removed dummy links: {dropped_links}")
    print("Wheels now mounted one per flange; chassis frame mounted to root;")
    print("excavator ladder on prismatic 'excavator_lift' with "
          "'drive_roller_joint' and 'idler_roller_joint';")
    print(f"{len(belt['buckets'])} buckets on belt track joints "
          f"(loop {belt['length']:.3f} m x r={belt['radius']:.4f} m).")


if __name__ == "__main__":
    main()
