"""
Step 2: build the MuJoCo scene. Turns the geometry from geometry.py into an
MJCF (MuJoCo XML) model:

  * chassis: a mocap body (moved by script) carrying the bin and frame
  * ladder: a body on the "lift" slide joint, with a force-limited
    position actuator (the two lift actuators)
  * buckets: one body each on the ladder, with three joints (along, across,
    about the shafts) and a stiff position servo on each
  * ground: the surface with the pit cut out of it
  * regolith: spheres filling the pit
"""
import mujoco
import numpy as np

from settings import (BUCKET_KP, BUCKET_KV, BUCKET_ROT_KP, BUCKET_ROT_KV, DIGGER_BITS,
                      GROUND_HALF_SIZE, LIFT_KP, LIFT_KV, PIT_DEPTH, PIT_FORWARD, PIT_LEFT,
                      REGOLITH_BITS, ROLLER_LENGTH, ROLLER_RADIUS)
from fix_kinematics import LIFT_LOWER, LIFT_UPPER  # noqa: E402 (path set by settings)


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
