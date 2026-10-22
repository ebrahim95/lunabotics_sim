"""
Step 1: read the robot. Loads the repaired URDF, the CAD meshes and the belt
layout, and works out everything the scene needs: the ladder frame, the
bucket and bin walls, the rollers, where the pit goes, and the masses.

MuJoCo collides with the convex hull of a mesh, which would turn the hollow
CAD bucket and bin into solid lumps. So shell_slabs() splits each into its
walls: every pair of parallel outer/inner faces in the STL becomes one
convex slab.
"""
import os
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from settings import MAX_WALL, PKG
from fix_kinematics import load_stl_vertices, origin_to_T, world_poses  # noqa: E402 (path set by settings)


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
