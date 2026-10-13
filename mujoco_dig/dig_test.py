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

The code is split by job:

    settings.py   paths and every tunable number
    geometry.py   step 1: read the robot (URDF, CAD meshes, belt layout)
    scene.py      step 2: build the MuJoCo scene (robot, pit, regolith)
    robot.py      step 3: run the robot (chassis, lift, belt motor)
    measure.py    step 4: count regolith, summarise motor loads
    dig_test.py   this file: options, start-up, the run loop, the viewer

Usage (from the repo root):
    .venv-mujoco/bin/python mujoco_dig/dig_test.py            # headless, prints counts
    .venv-mujoco/bin/python mujoco_dig/dig_test.py --viewer   # watch it
"""
import argparse
import os
import time

import mujoco

from settings import PIT_DEPTH
from fix_kinematics import LIFT_EFFORT, LIFT_VELOCITY, ROLLER_EFFORT, loop_pose  # noqa: E402
from geometry import load_geometry
from measure import counts, motor_summary
from robot import Robot
from scene import build_xml


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


if __name__ == "__main__":
    main()
