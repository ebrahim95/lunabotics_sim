"""
Step 4: measure. Counts the regolith (in the bin, or riding the buckets) and
summarises how hard the motors worked.
"""
import numpy as np


def counts(model, data, g, n, pile_top, robot):
    lo, hi = g["bin"]
    P = np.array([data.xpos[model.body(f"regolith_{i}").id] for i in range(n)])
    local = P - data.mocap_pos[robot.chassis]  # chassis only translates
    in_bin = np.all((local >= lo) & (local <= hi), axis=1)
    lifted = (P[:, 2] > pile_top + 0.05) & ~in_bin
    return int(in_bin.sum()), int(lifted.sum())


def motor_summary(belt, data):
    return (f"motors: ladder reached {belt.lift_pos(data):+.3f} m, lift force peak {belt.peak_lift:.0f} N; "
            f"belt torque peak {belt.peak_belt:.2f} N*m, at its limit for {belt.limited_time:.1f} s "
            f"(stalled {belt.stalled_time:.1f} s), belt travelled {belt.s:.2f} m")
