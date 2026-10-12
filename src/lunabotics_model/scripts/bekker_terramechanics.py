"""
Bekker-Wong terramechanics for Gazebo: soft-soil forces on the drive wheels.

A Gazebo Python system (loaded by gz-sim-python-system-loader-system from the
robot's <gazebo> block in assembly_1_sim_bekker.urdf). The wheels don't
collide with the arena floor in that model; instead, every physics step,
this computes for each wheel:

  * sinkage z: how far the wheel's lowest point is below the soil surface
  * support (Bekker pressure-sinkage, rigid wheel):
        W = (3 - n)/3 * (k_c + b k_phi) * sqrt(D) * z^((2n + 1)/2)
  * traction (Janosi-Hanamoto shear) from the wheel's slip ratio s:
        H = (A c + W tan(phi)) * (1 - K/(s L) * (1 - exp(-s L / K)))
  * compaction (rolling) resistance from the rut the wheel makes:
        R = b (k_c/b + k_phi) z^(n + 1) / (n + 1)
  * sideways shear resisting lateral sliding, built up with lateral slip
    through the same Janosi curve (so skid-steer wheels can scrub sideways)
  * the matching torque about the wheel, so the motors feel the load

and applies them as a world wrench on the wheel link. A = b L is the contact
patch, L = sqrt(D z) its length, D = 2r, b the wheel width.

Soil: BP-1 (Black Point-1) friction angle and cohesion; Bekker pressure-
sinkage constants and K are standard lunar-regolith values, as no BP-1
values were found. All in SOIL below.

Wheel geometry comes from config/terramechanics.yaml (written by
add_ros2_control.py); the soil surface from worlds/<world>_height.npz
(written by generate_arena_world.py), or flat at z = 0 if there isn't one.
"""
import math
import os
import time

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from gz.math import Vector3d
from gz.sim import Link, Model, World, world_entity

SOIL = {
    "n": 1.0,            # sinkage exponent (lunar regolith)
    "k_c": 1400.0,       # N/m^(n+1), cohesive modulus (lunar regolith)
    "k_phi": 820000.0,   # N/m^(n+2), frictional modulus (lunar regolith)
    "c": 100.0,          # Pa, cohesion (BP-1: negligible)
    "phi_deg": 39.0,     # deg, internal friction angle (BP-1, loose)
    "K": 0.018,          # m, shear deformation modulus (lunar regolith)
}
VERTICAL_DAMPING = 300.0  # N*s/m per wheel, numerical damping on sinkage rate
SLIP_V_EPS = 0.05         # m/s, slip-ratio regularisation near standstill
LATERAL_V_EPS = 0.02      # m/s, smoothing of the sideways shear direction
REPORT_PERIOD = 2.0       # s of sim time between debug reports (LUNA_TERRA_DEBUG=1)

UP = np.array([0.0, 0.0, 1.0])


def vec(v):
    return np.array([v.x(), v.y(), v.z()])


class Terrain:
    """Soil surface height: arena heightmap (bilinear) or flat ground."""

    def __init__(self, path):
        self.grid = None
        if path and os.path.isfile(path):
            d = np.load(path)
            self.xs, self.ys, self.z = d["xs"], d["ys"], d["z"]
            self.grid = True

    def height(self, x, y):
        if self.grid is None:
            return 0.0
        xs, ys = self.xs, self.ys
        if not (xs[0] <= x <= xs[-1] and ys[0] <= y <= ys[-1]):
            return 0.0
        i = min(int((x - xs[0]) / (xs[1] - xs[0])), len(xs) - 2)
        j = min(int((y - ys[0]) / (ys[1] - ys[0])), len(ys) - 2)
        tx = (x - xs[i]) / (xs[i + 1] - xs[i])
        ty = (y - ys[j]) / (ys[j + 1] - ys[j])
        z = self.z
        return ((1 - tx) * (1 - ty) * z[i, j] + tx * (1 - ty) * z[i + 1, j]
                + (1 - tx) * ty * z[i, j + 1] + tx * ty * z[i + 1, j + 1])


def wheel_forces(r, b, z, vx, vy, spin, soil, vz):
    """Soil forces on one wheel: (support W, shear traction H, compaction
    resistance R, sideways F_side, slip s).

    vx/vy: wheel centre velocity forward/sideways; spin: axle rate (rad/s,
    + rolls forward); vz: vertical velocity of the wheel centre.
    """
    n, kc, kphi, c, K = soil["n"], soil["k_c"], soil["k_phi"], soil["c"], soil["K"]
    tan_phi = math.tan(math.radians(soil["phi_deg"]))
    D = 2 * r
    z = min(z, r)
    W = (3 - n) / 3 * (kc + b * kphi) * math.sqrt(D) * z ** ((2 * n + 1) / 2)
    W = max(0.0, W - VERTICAL_DAMPING * vz)  # soil only pushes, never pulls
    L = math.sqrt(D * z)
    H_max = b * L * c + W * tan_phi

    # Slip ratio: + when the wheel spins faster than it travels (driving).
    v_wheel = spin * r
    s = (v_wheel - vx) / max(abs(v_wheel), abs(vx), SLIP_V_EPS)
    s = max(-1.0, min(1.0, s))
    x = abs(s) * L / K
    shear = (1 - (1 - math.exp(-x)) / x) if x > 1e-9 else 0.0
    H = math.copysign(H_max * shear, s)

    R = b * (kc / b + kphi) * z ** (n + 1) / (n + 1) * math.tanh(vx / SLIP_V_EPS)

    # Sideways: lateral slip = sideways speed relative to how fast the wheel
    # surface is moving; shear builds along the same Janosi curve.
    s_y = vy / max(abs(v_wheel), abs(vx), abs(vy), LATERAL_V_EPS)
    x_y = abs(s_y) * L / K
    shear_y = (1 - (1 - math.exp(-x_y)) / x_y) if x_y > 1e-9 else 0.0
    F_side = -math.copysign(H_max * shear_y, vy)
    return W, H, R, F_side, s


class BekkerTerramechanics:
    def configure(self, entity, sdf, ecm, event_mgr):
        share = get_package_share_directory("lunabotics_model")
        with open(os.path.join(share, "config", "terramechanics.yaml")) as f:
            cfg = yaml.safe_load(f)
        model = Model(entity)
        self.wheels = []
        for w in cfg["wheels"]:
            link = Link(model.link_by_name(ecm, w["link"]))
            link.enable_velocity_checks(ecm, True)
            self.wheels.append(dict(link=link, centre=np.array(w["centre"]),
                                    axle=np.array(w["axle"]), r=w["radius"], b=w["width"]))
        self.world_name = None
        self.terrain = None
        self.share = share
        self.debug = os.environ.get("LUNA_TERRA_DEBUG") == "1"
        self.next_report = 0.0
        self.stats = []
        print(f"[bekker_terramechanics] {len(self.wheels)} wheels on soft soil: {SOIL}", flush=True)

    def pre_update(self, info, ecm):
        if info.paused:
            return
        if self.terrain is None:
            name = World(world_entity(ecm)).name(ecm)
            path = os.path.join(self.share, "worlds", f"{name}_height.npz")
            self.terrain = Terrain(path)
            print(f"[bekker_terramechanics] world '{name}': "
                  f"{'heightmap ' + path if self.terrain.grid else 'flat soil at z = 0'}", flush=True)

        for w in self.wheels:
            link = w["link"]
            pose = link.world_pose(ecm)
            v_lin = link.world_linear_velocity(ecm)
            v_ang = link.world_angular_velocity(ecm)
            if pose is None or v_lin is None or v_ang is None:
                continue
            rot = pose.rot()
            origin = vec(pose.pos())
            centre = origin + vec(rot.rotate_vector(Vector3d(*w["centre"])))
            axle = vec(rot.rotate_vector(Vector3d(*w["axle"])))
            omega = vec(v_ang)
            v_centre = vec(v_lin) + np.cross(omega, centre - origin)

            # Lowest point of the wheel rim (direction "down" within the wheel plane).
            down = -UP + (UP @ axle) * axle
            down /= np.linalg.norm(down)
            bottom = centre + w["r"] * down
            z = self.terrain.height(bottom[0], bottom[1]) - bottom[2]
            if z <= 0:
                continue

            forward = np.cross(axle, UP)
            forward /= np.linalg.norm(forward)
            side = np.cross(UP, forward)
            W, H, R, Fy, s = wheel_forces(w["r"], w["b"], z, v_centre @ forward, v_centre @ side,
                                           omega @ axle, SOIL, v_centre @ UP)
            # Shear (traction, sideways) acts at the contact patch, so it
            # twists the wheel and the motor must supply H*r. Soil pressure
            # (support W, compaction resistance R) acts on the rim towards
            # the axle, so it pushes the wheel without twisting it.
            shear = H * forward + Fy * side
            pressure = W * UP - R * forward
            F = shear + pressure
            com = vec(link.world_inertial_pose(ecm).pos())
            torque = np.cross(bottom - com, shear) + np.cross(centre - com, pressure)
            Fx = H - R
            link.add_world_wrench(ecm, Vector3d(*F), Vector3d(*torque))
            if self.debug:
                self.stats.append((z, s, W, Fx))

        if self.debug:
            t = info.sim_time.total_seconds() if hasattr(info.sim_time, "total_seconds") else time.time()
            if t >= self.next_report and self.stats:
                a = np.array(self.stats)
                print(f"[bekker_terramechanics] t={t:6.1f}s sinkage {a[:, 0].mean() * 1000:5.1f} mm  "
                      f"slip {a[:, 1].mean() * 100:+5.1f}%  load {a[:, 2].mean():5.1f} N  "
                      f"traction {a[:, 3].mean():+6.1f} N (per wheel, avg)", flush=True)
                self.stats = []
                self.next_report = t + REPORT_PERIOD

    def reset(self, info, ecm):
        self.next_report = 0.0


def get_system():
    return BekkerTerramechanics()
