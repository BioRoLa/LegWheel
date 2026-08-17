"""Does a SOFTER virtual spring buy the trot enough torque headroom?

trot_fixed_point.py sweeps speed at the shipped design stiffness k_rel = 18 and
finds no feasible trot. Stiffness is the axis it never varied, and it is the one
place a torque win could still be hiding: peak GRF falls with k, so a softer
template should demand less per leg.

The reason to be sceptical before running it -- and the reason this reports the
whole grid rather than a verdict line -- is that softening does not only lower
force. It lengthens stance, which pushes SLIP duty toward the 0.67-0.88 grazing
band, and it deepens compression, which pushes theta toward its workspace limit.
The realistic outcome is that the torque win is eaten by the guards, and a grid
showing WHICH guard each cell fails is worth more than "no".

Do NOT read across from the pronk's falsified softer-spring run (k_radial 8941
-> 4900 made saturation WORSE, see the weekly log). That failed because the
robot left its designed range and the force came from what it met there -- an
implementation effect. This is a template-level question and the two are not
the same claim.

Guards are the same as trot_fixed_point.py and for the same reason: the filter
has to be on the SLIP duty and the apex, because duty_leg < 0.5 cannot catch a
grazing gait (half of 0.785 still passes).

Run:
    uv run python examples/gslip/trot_stiffness_sweep.py
"""

import numpy as np

from legwheel.models import slip_rf
from legwheel.planners import gslip_to_corgi as g2c
from trot_fixed_point import (
    G,
    MASS,
    MAX_DUTY,
    MIN_APEX_MM,
    MOTOR_TORQUE_LIMIT,
    NOMINAL_THETA_DEG,
    PRONK_STANCE_LEGS,
    TORQUE_EROSION,
    TROT_STANCE_LEGS,
    apex_mm,
    leg_loads,
    solve_fixed_point,
)

# The band corgi_stability_sweep.py already used, so this sits on ground the
# stability work has covered rather than inventing a new range.
K_REL_SWEEP = (7.0, 10.0, 12.0, 15.0, 18.0, 22.0, 27.0, 34.0, 45.0)

# Narrower than trot_fixed_point.py's speed sweep: the point here is stiffness,
# and the speeds that matter are the operating range plus the design point.
V_TILDE_SWEEP = (0.5, 0.7, 1.05, 1.2, 1.4)

# Reproduce this before trusting any new cell -- it is the published S14 number.
CHECK_CELL = (18.0, 1.2, 30.03)


def params_for(k_rel, leg_map):
    hip_to_arc = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))
    return slip_rf.SlipRfParams(
        m=MASS, l0=hip_to_arc + leg_map.leg.foot_radius,
        k=k_rel * MASS * G / hip_to_arc, r=leg_map.leg.foot_radius,
    )


def cell(p, v, leg_map):
    """(tau_trot, label) for one (k_rel, v) cell. label says what bound it."""
    fp = solve_fixed_point(p, v)
    if fp is None:
        return None, "graze"          # no non-grazing fixed point at all
    res, traj, _, tau = leg_loads(p, v, fp, leg_map, TROT_STANCE_LEGS)
    r = traj.guard_report()
    if not (r["theta_ok"] and r["beta_ok"] and r["stays_on_foot_arc"]):
        which = ("theta" if not r["theta_ok"]
                 else "beta" if not r["beta_ok"] else "arc")
        return tau, which
    if 0.5 * fp.duty_factor >= 0.5:
        return tau, "duty"
    if tau * TORQUE_EROSION > MOTOR_TORQUE_LIMIT:
        return tau, "torque"
    return tau, "OK"


def main() -> None:
    leg_map = g2c.LegLengthMap()

    print()
    print("=" * 78)
    print("TROT FEASIBILITY over STIFFNESS x SPEED")
    print("=" * 78)
    print(f"  erosion x{TORQUE_EROSION:.2f}, limit {MOTOR_TORQUE_LIMIT} N.m, "
          f"grazing filter duty <= {MAX_DUTY}, apex >= {MIN_APEX_MM:.0f} mm")
    print(f"  a trot needs eroded tau <= {MOTOR_TORQUE_LIMIT} N.m, i.e. "
          f"template tau <= {MOTOR_TORQUE_LIMIT/TORQUE_EROSION:.2f} N.m")
    print()

    # --- regression against the published number ---------------------------
    k_chk, v_chk, tau_chk = CHECK_CELL
    p = params_for(k_chk, leg_map)
    tau, _ = cell(p, v_chk * np.sqrt(G * p.l0), leg_map)
    ok = tau is not None and abs(tau - tau_chk) < 0.05
    print(f"  regression  k_rel {k_chk:.0f}, v~ {v_chk}: "
          f"tau = {tau:.2f} N.m, expected {tau_chk:.2f}  "
          f"{'PASS' if ok else 'FAIL -- do not trust the grid below'}")
    if not ok:
        return
    print()

    hdr = "  ".join(f"{vt:>12.2f}" for vt in V_TILDE_SWEEP)
    print(f"{'k_rel':>6}  {hdr}      (tau trot N.m / what bound it)")
    best = None
    for k_rel in K_REL_SWEEP:
        p = params_for(k_rel, leg_map)
        cells = []
        for vt in V_TILDE_SWEEP:
            tau, label = cell(p, vt * np.sqrt(G * p.l0), leg_map)
            if tau is None:
                cells.append(f"{'-- graze --':>12}")
                continue
            cells.append(f"{tau:8.2f} {label:<4}")
            if label == "OK" and (best is None or tau < best[0]):
                best = (tau, k_rel, vt)
        print(f"{k_rel:6.0f}  " + "  ".join(cells))

    print()
    print("  label = the FIRST guard the cell fails, not the only one")
    print("    graze  no non-grazing fixed point exists at this (k_rel, v~)")
    print("    theta/beta/arc  workspace guard")
    print("    torque  fixed point is fine, eroded demand exceeds 35 N.m")
    print()
    print("=" * 78)
    if best is None:
        print("  NO (k_rel, v~) CELL IS FEASIBLE.")
        print()
        print("  Softening does lower the template's torque, but not nearly")
        print("  enough and not for free -- the low-stiffness cells lose their")
        print("  fixed points to the grazing filter before the torque falls far")
        print("  enough to matter. Stiffness is not the escape.")
    else:
        tau, k_rel, vt = best
        print(f"  FEASIBLE: k_rel {k_rel:.0f}, v~ {vt:.2f}, "
              f"tau {tau:.2f} N.m -> {tau*TORQUE_EROSION:.1f} eroded")
        print()
        print("  Check this against the pronk before believing it: a stiffness")
        print("  the trot can afford still has to be a stiffness the LEG can")
        print("  realise, and k_radial is set per-leg at k/2 for a trot.")
    print("=" * 78)
    print()


if __name__ == "__main__":
    main()
