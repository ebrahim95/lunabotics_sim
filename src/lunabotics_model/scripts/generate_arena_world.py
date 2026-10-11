#!/usr/bin/env python3
"""
Writes a Gazebo world of the NASA Lunabotics Artemis Arena (Kennedy Space
Center), filled with Black Point-1 (BP-1) lunar regolith simulant.

Layout (Lunabotics 2024 guidebook, Drawing 2 "Artemis Arena Layout 1";
2026 guidebook for berm sizes). Origin at the starting-zone corner,
x along the 6.88 m length, y along the 5 m width:

    +-----------------------------+---------------------+  y = 5
    |  obstacle zone (~3.88 m)    |  excavation zone    |
    |  >= 3 rocks, >= 3 craters   |  (~3 x 3 m, dig     |
    |                     [col]   |   here, >= 1 crater)|  y = 2
    +--------+                    +---------------------+
    | start  |                    |  construction zone  |
    | 2 x 2 m|                    |  berm target 2.2x0.9|
    +--------+--------------------+---------------------+  y = 0
    x = 0                       3.88                  6.88

BP-1 cannot be simulated as a granular soil in Gazebo, so it is modelled
as the floor's surface: a terrain mesh with crater depressions and ~2 cm
gravel roughness, dark basalt colour, and a friction coefficient from
BP-1's internal friction angle. Measured BP-1 properties (Rahmatian &
Metzger; NASA ARES): crushed basalt, rough and angular grains, specific
gravity 2.81, bulk density 1.27-1.75 g/cm^3, peak internal friction angle
39-51 deg, negligible cohesion. The arena holds ~45 cm of it.

Usage:
    ros2 run lunabotics_model generate_arena_world.py --output-dir worlds
    ros2 run lunabotics_model generate_arena_world.py --output-dir worlds --seed 7   # random layout

Then launch with:
    ros2 launch lunabotics_model gazebo.launch.py world:=artemis_arena
"""
import argparse
import math
import os
import struct

import numpy as np

# ---- Arena (m) ------------------------------------------------------------
LENGTH, WIDTH = 6.88, 5.0
OBSTACLE_END = 3.88              # obstacle zone is x < 3.88
CONSTRUCTION_TOP = 2.0           # construction zone is y < 2 beyond the obstacle zone
START_SIZE = 2.0                 # starting zone: x, y < 2 (no rocks or craters)
BERM_TARGET = (5.58, 0.6, 2.2, 0.9)   # centre x, centre y, length, width
COLUMN = (3.61, 2.61, 0.45)      # centre x, centre y, side (measured from the drawing)
COLUMN_HEIGHT = 1.5
WALL_HEIGHT, WALL_THICKNESS = 0.5, 0.1

# ---- BP-1 -------------------------------------------------------------------
BP1_FRICTION = 0.8      # ~tan(39 deg): BP-1's internal friction angle, loose state
BP1_COLOUR = "0.23 0.22 0.21"   # dark grey crushed basalt
GRAVEL_ROUGHNESS = 0.005        # m, surface bumps from the ~2 cm gravel
MESH_SPACING = 0.06             # m between terrain mesh vertices (finer = slower sim)

# ---- Obstacles: the guidebook drawing's example layout ----------------------
# Craters: (x, y, diameter, depth); <= 40-50 cm wide and deep.
CRATERS = [(1.08, 4.19, 0.45, 0.20), (2.95, 4.52, 0.45, 0.18),
           (3.00, 1.22, 0.45, 0.22), (6.40, 4.38, 0.45, 0.20)]
# Rocks: (x, y, diameter, height above the surface); 30-40 cm across.
ROCKS = [(1.92, 4.44, 0.35, 0.20), (1.03, 2.96, 0.32, 0.15), (2.92, 2.96, 0.38, 0.25),
         (3.57, 1.95, 0.30, 0.15), (4.91, 4.16, 0.35, 0.20), (6.43, 3.14, 0.32, 0.18),
         (4.84, 2.65, 0.36, 0.22)]


def random_layout(seed):
    """Obstacles placed the way the judges do: random, inside their zones."""
    rng = np.random.default_rng(seed)
    placed = []

    def free(x, y, d):
        if abs(x - COLUMN[0]) < COLUMN[2] / 2 + d / 2 + 0.1 and abs(y - COLUMN[1]) < COLUMN[2] / 2 + d / 2 + 0.1:
            return False
        return all(math.hypot(x - px, y - py) > (d + pd) / 2 + 0.15 for px, py, pd in placed)

    def place(n, zone, make):
        out = []
        while len(out) < n:
            x, y = rng.uniform(*zone[0]), rng.uniform(*zone[1])
            if x < START_SIZE + 0.3 and y < START_SIZE + 0.3:
                continue  # nothing in (or hugging) the starting zone
            item = make(x, y)
            if free(x, y, item[2]):
                placed.append((x, y, item[2]))
                out.append(item)
        return out

    obstacle_zone = ((0.3, OBSTACLE_END - 0.3), (0.3, WIDTH - 0.3))
    excavation_zone = ((OBSTACLE_END + 0.3, LENGTH - 0.3), (CONSTRUCTION_TOP + 0.3, WIDTH - 0.3))
    crater = lambda x, y: (x, y, rng.uniform(0.40, 0.50), rng.uniform(0.15, 0.30))
    rock = lambda x, y: (x, y, rng.uniform(0.30, 0.40), rng.uniform(0.12, 0.30))
    craters = place(3, obstacle_zone, crater) + place(1, excavation_zone, crater)
    rocks = place(3, obstacle_zone, rock) + place(2, excavation_zone, rock)
    return craters, rocks


def terrain(craters, seed):
    """Height grid of the arena floor: gravel roughness plus crater bowls."""
    rng = np.random.default_rng(seed)
    xs = np.linspace(0, LENGTH, int(round(LENGTH / MESH_SPACING)) + 1)
    ys = np.linspace(0, WIDTH, int(round(WIDTH / MESH_SPACING)) + 1)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    Z = rng.normal(0, GRAVEL_ROUGHNESS / 2, X.shape)
    Z = np.clip(Z, -GRAVEL_ROUGHNESS, GRAVEL_ROUGHNESS)
    for cx, cy, d, depth in craters:
        r = np.hypot(X - cx, Y - cy) / (d / 2)
        bowl = -depth * np.clip(1 - r ** 2, 0, None) ** 1.5             # smooth bowl
        rim = np.where((r >= 0.8) & (r < 1.3), 0.02 * np.cos((r - 1.05) / 0.25 * math.pi / 2) ** 2, 0.0)
        Z = Z + bowl + rim
    # Keep the edges flat so the floor meets the walls cleanly.
    Z[0, :] = Z[-1, :] = Z[:, 0] = Z[:, -1] = 0.0
    return X, Y, Z


def write_stl(path, X, Y, Z):
    """Binary STL of the height grid (two triangles per cell, normals up)."""
    nx, ny = X.shape
    P = np.stack([X, Y, Z], axis=-1)
    a, b = P[:-1, :-1].reshape(-1, 3), P[1:, :-1].reshape(-1, 3)
    c, d = P[1:, 1:].reshape(-1, 3), P[:-1, 1:].reshape(-1, 3)
    tris = np.concatenate([np.stack([a, b, c], 1), np.stack([a, c, d], 1)])
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("attr", "<u2")]))
    rec["n"] = n
    rec["v"] = tris.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(b"Lunabotics Artemis Arena floor, BP-1".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())
    return len(tris)


def box(name, x, y, z, sx, sy, sz, rgba, collide=True):
    col = (f'<collision name="c"><geometry><box><size>{sx:.4g} {sy:.4g} {sz:.4g}</size></box></geometry></collision>'
           if collide else "")
    return (f'<model name="{name}"><static>true</static><pose>{x:.4g} {y:.4g} {z:.4g} 0 0 0</pose>'
            f'<link name="link">{col}<visual name="v"><geometry><box><size>{sx:.4g} {sy:.4g} {sz:.4g}</size></box></geometry>'
            f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>'
            f'{"" if collide else "<cast_shadows>false</cast_shadows>"}</visual></link></model>')


def outline(name, x0, y0, x1, y1, rgba, w=0.03):
    """Zone boundary painted on the floor (display only)."""
    z = GRAVEL_ROUGHNESS + 0.002
    return "".join([
        box(f"{name}_s", (x0 + x1) / 2, y0, z, x1 - x0, w, 0.002, rgba, False),
        box(f"{name}_n", (x0 + x1) / 2, y1, z, x1 - x0, w, 0.002, rgba, False),
        box(f"{name}_w", x0, (y0 + y1) / 2, z, w, y1 - y0, 0.002, rgba, False),
        box(f"{name}_e", x1, (y0 + y1) / 2, z, w, y1 - y0, 0.002, rgba, False),
    ])


def world_sdf(floor_uri, rocks):
    rgba = BP1_COLOUR + " 1"
    surface = (f"<surface><friction><ode><mu>{BP1_FRICTION}</mu><mu2>{BP1_FRICTION}</mu2></ode>"
               f"</friction></surface>")
    parts = [f"""
    <model name="bp1_floor"><static>true</static><link name="link">
      <collision name="c"><geometry><mesh><uri>{floor_uri}</uri></mesh></geometry>{surface}</collision>
      <visual name="v"><geometry><mesh><uri>{floor_uri}</uri></mesh></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.05 0.05 0.05 1</specular></material>
      </visual></link></model>"""]
    # Arena walls (outside the 6.88 x 5 m interior).
    t, h = WALL_THICKNESS, WALL_HEIGHT
    wall = "0.55 0.55 0.6 1"
    parts += [
        box("wall_south", LENGTH / 2, -t / 2, h / 2, LENGTH + 2 * t, t, h, wall),
        box("wall_north", LENGTH / 2, WIDTH + t / 2, h / 2, LENGTH + 2 * t, t, h, wall),
        box("wall_west", -t / 2, WIDTH / 2, h / 2, t, WIDTH, h, wall),
        box("wall_east", LENGTH + t / 2, WIDTH / 2, h / 2, t, WIDTH, h, wall),
        box("column", COLUMN[0], COLUMN[1], COLUMN_HEIGHT / 2, COLUMN[2], COLUMN[2], COLUMN_HEIGHT, "0.7 0.7 0.72 1"),
    ]
    # Rocks: spheres sunk into the BP-1 so they stand at the given height.
    for i, (x, y, d, height) in enumerate(rocks):
        r = d / 2
        shade = 0.30 + 0.05 * (i % 3)
        parts.append(
            f'<model name="rock_{i}"><static>true</static><pose>{x:.4g} {y:.4g} {height - r:.4g} 0 0 0</pose>'
            f'<link name="link"><collision name="c"><geometry><sphere><radius>{r:.4g}</radius></sphere></geometry>{surface}</collision>'
            f'<visual name="v"><geometry><sphere><radius>{r:.4g}</radius></sphere></geometry>'
            f'<material><ambient>{shade:.2f} {shade:.2f} {shade - 0.02:.2f} 1</ambient><diffuse>{shade:.2f} {shade:.2f} {shade - 0.02:.2f} 1</diffuse></material>'
            f'</visual></link></model>')
    # Zone outlines.
    bx, by, bl, bw = BERM_TARGET
    parts += [
        outline("zone_start", 0.02, 0.02, START_SIZE, START_SIZE, "0.2 0.8 0.3 1"),
        outline("zone_excavation", OBSTACLE_END, CONSTRUCTION_TOP, LENGTH - 0.02, WIDTH - 0.02, "0.25 0.5 0.95 1"),
        outline("zone_construction", OBSTACLE_END, 0.02, LENGTH - 0.02, CONSTRUCTION_TOP, "0.95 0.65 0.15 1"),
        outline("berm_target", bx - bl / 2, by - bw / 2, bx + bl / 2, by + bw / 2, "0.9 0.15 0.15 1"),
    ]
    return f"""<?xml version="1.0" ?>
<!-- Generated by generate_arena_world.py. Do not hand-edit; re-run the script. -->
<sdf version="1.9">
  <world name="artemis_arena">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.75 0.78 0.82 1</background></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
    {"".join(parts)}
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--output-dir", required=True, help="the package's worlds/ directory")
    ap.add_argument("--seed", type=int, default=None,
                    help="random obstacle layout (default: the guidebook drawing's layout)")
    args = ap.parse_args()

    craters, rocks = (CRATERS, ROCKS) if args.seed is None else random_layout(args.seed)
    os.makedirs(os.path.join(args.output_dir, "meshes"), exist_ok=True)
    X, Y, Z = terrain(craters, args.seed or 0)
    ntri = write_stl(os.path.join(args.output_dir, "meshes", "artemis_arena_floor.stl"), X, Y, Z)
    with open(os.path.join(args.output_dir, "artemis_arena.world.sdf"), "w") as f:
        f.write(world_sdf("package://lunabotics_model/worlds/meshes/artemis_arena_floor.stl", rocks))
    print(f"Wrote Artemis Arena ({LENGTH} x {WIDTH} m, BP-1 floor {ntri} triangles): "
          f"{len(craters)} craters, {len(rocks)} rocks, "
          f"{'guidebook layout' if args.seed is None else f'random layout seed {args.seed}'}")


if __name__ == "__main__":
    main()
