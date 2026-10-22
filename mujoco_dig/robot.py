"""
Step 3: run the robot. Each physics step, Robot.step():

  * moves the chassis forward by script (--advance)
  * moves the lift target toward the commanded ladder position, no faster
    than the actuators' 2"/s; the force-limited actuator in the scene does
    the rest
  * runs the belt motor model: measures the load the buckets put on the belt
    (belt_load), gives torque up to the motor's limit, works out the new belt
    speed (the worm gear can't be back-driven), and moves the bucket servos'
    targets along the loop
"""
import mujoco
import numpy as np

from settings import BELT_GAIN, BELT_MASS, LOAD_FILTER
from fix_kinematics import LIFT_VELOCITY, ROLLER_VELOCITY, loop_pose  # noqa: E402


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
