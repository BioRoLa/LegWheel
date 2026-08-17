"""Was the trot's duty guard a physics limit, or an inherited artifact filter?

trot_fixed_point.py and trot_stiffness_sweep.py both filter with MAX_DUTY = 0.55
and MIN_APEX_MM = 10. Those values came from S5's stability sweep, where their
job was to EXCLUDE ARTIFACTS: near-grazing solutions with 0.24-3.2 mm of apex
clearance and duty 0.67-0.88, which the sweep was falsely counting as stable
because their flight phase had vanished.

That is a sound reason to reject those solutions as EVIDENCE. It is not, by
itself, a reason to reject a higher duty as a DESIGN. The two uses of the same
constant have never been separated, and this script separates them.

WHY IT SHOULD MATTER

Peak ground reaction falls with stance duration for a fixed impulse -- the body
has to be turned around either way, and a longer stance does it more gently. So
a trot at duty 0.60 should demand meaningfully less peak torque than one at
0.42, on an axis S31's stiffness grid could not see because the filter removed
those cells before they were costed.

The apex guard is KEPT AT 10 mm and should stay there. It is the one that
genuinely excludes artifacts: on a 145 mm foot radius a 3 mm hop is
indistinguishable from continuous contact, and no amount of design intent makes
it a running gait. Only the duty ceiling is relaxed here.

WHAT IT COSTS, AND WHY THAT IS NOT THIS SCRIPT'S CALL

A duty-0.6 trot has both diagonal pairs on the ground for more of the cycle and
drifts toward a running walk. That weakens the "this is a running template"
framing the thesis rests on. It is a real cost, it is a framing cost rather than
a physics one, and it should be paid deliberately if it is paid -- so this
script reports duty and apex on every cell rather than collapsing them into a
pass/fail.

CEILINGS

Reported against four, because the number to beat has moved twice:

    35.0   the legacy simulator clamp -- NOT a hardware limit, no provenance
    29.5   HT-04 stall at 6:1, the real absolute ceiling (S32)
    25.0   usable at running speed at 6:1 (S32) -- the honest one for 6:1
    37.0   usable at running speed at 9:1, if the leg gearbox were changed

Run:
    .venv/bin/python -u examples/gslip/trot_duty_sweep.py
"""

import numpy as np

import trot_fixed_point as tfp
from legwheel.models import slip_rf
from legwheel.planners import gslip_to_corgi as g2c
from trot_fixed_point import (
    G,
    MASS,
    NOMINAL_THETA_DEG,
    TROT_STANCE_LEGS,
    apex_mm,
    leg_loads,
)

K_REL_SWEEP = (7.0, 10.0, 12.0, 15.0, 18.0, 22.0, 27.0, 34.0, 45.0)
V_TILDE_SWEEP = (0.5, 0.7, 1.05, 1.2, 1.4)

# The relaxation under test. 0.55 is the inherited value; 0.70 admits the
# longer-stance branch while still refusing the 0.67-0.88 grazing band's upper
# half outright.
DUTY_CEILINGS = (0.55, 0.70)

CEILINGS = (
    ("35.0 legacy sim clamp", 35.0),
    ("29.5 stall, 6:1", 29.5),
    ("25.0 usable, 6:1", 25.0),
    ("37.0 usable, 9:1", 37.0),
)

# Reproduce before trusting anything below -- the published S31 number.
CHECK_CELL = (18.0, 1.2, 30.03)


def params_for(k_rel, leg_map):
    hip_to_arc = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))
    return slip_rf.SlipRfParams(
        m=MASS, l0=hip_to_arc + leg_map.leg.foot_radius,
        k=k_rel * MASS * G / hip_to_arc, r=leg_map.leg.foot_radius,
    )


def cell(p, v, leg_map):
    """(tau, duty, apex, workspace_ok) or None if no fixed point survives."""
    fp = tfp.solve_fixed_point(p, v)
    if fp is None:
        return None
    res, traj, _, tau = leg_loads(p, v, fp, leg_map, TROT_STANCE_LEGS)
    r = traj.guard_report()
    ws = r["theta_ok"] and r["beta_ok"] and r["stays_on_foot_arc"]
    return tau, fp.duty_factor, apex_mm(res), ws


def run_grid(leg_map, duty_max):
    """The whole grid at one duty ceiling. Returns {(k,v): cell}."""
    original = tfp.MAX_DUTY
    tfp.MAX_DUTY = duty_max
    try:
        out = {}
        for k_rel in K_REL_SWEEP:
            p = params_for(k_rel, leg_map)
            for vt in V_TILDE_SWEEP:
                out[(k_rel, vt)] = cell(p, vt * np.sqrt(G * p.l0), leg_map)
        return out
    finally:
        tfp.MAX_DUTY = original


def main() -> None:
    leg_map = g2c.LegLengthMap()

    print()
    print("=" * 78)
    print("TROT: IS THE DUTY GUARD LOAD-BEARING?")
    print("=" * 78)
    print(f"  apex guard held at {tfp.MIN_APEX_MM:.0f} mm throughout -- only the")
    print("  duty ceiling is varied. All torques below are TEMPLATE torques,")
    print("  before any erosion.")
    print()

    # --- regression -------------------------------------------------------
    k_chk, v_chk, tau_chk = CHECK_CELL
    p = params_for(k_chk, leg_map)
    got = cell(p, v_chk * np.sqrt(G * p.l0), leg_map)
    ok = got is not None and abs(got[0] - tau_chk) < 0.05
    print(f"  regression  k_rel {k_chk:.0f}, v~ {v_chk} at the INHERITED guard: "
          f"tau = {got[0]:.2f}, expected {tau_chk:.2f}  "
          f"{'PASS' if ok else 'FAIL -- stop'}")
    if not ok:
        return

    results = {}
    for duty_max in DUTY_CEILINGS:
        print()
        print("-" * 78)
        print(f"  MAX_DUTY = {duty_max:.2f}")
        print("-" * 78)
        grid = run_grid(leg_map, duty_max)
        results[duty_max] = grid

        hdr = "  ".join(f"{vt:>16.2f}" for vt in V_TILDE_SWEEP)
        print(f"{'k_rel':>6}  {hdr}")
        print(f"{'':>6}  " + "  ".join(f"{'tau  duty  apex':>16}"
                                       for _ in V_TILDE_SWEEP))
        for k_rel in K_REL_SWEEP:
            cells = []
            for vt in V_TILDE_SWEEP:
                c = grid[(k_rel, vt)]
                if c is None:
                    cells.append(f"{'-- graze --':>16}")
                    continue
                tau, duty, apex, ws = c
                flag = "" if ws else "*"
                cells.append(f"{tau:6.2f} {duty:5.3f} {apex:4.0f}{flag:<1}")
            print(f"{k_rel:6.0f}  " + "  ".join(cells))
        print("    * = fails a workspace guard (theta / beta / foot arc)")

        best = min((v + (k,) for k, v in grid.items()
                    if v is not None and v[3]), default=None)
        if best is None:
            print("    no cell with a valid workspace at this duty ceiling")
            continue
        tau, duty, apex, _, key = best
        print()
        print(f"    minimum template torque: {tau:.2f} N.m "
              f"at k_rel {key[0]:.0f}, v~ {key[1]:.2f} "
              f"(duty {duty:.3f}, apex {apex:.0f} mm)")

    # --- what the relaxation bought --------------------------------------
    print()
    print("=" * 78)
    print("WHAT THE RELAXATION BOUGHT")
    print("=" * 78)
    mins = {}
    for duty_max, grid in results.items():
        vals = [(v[0], v[1], v[2], k) for k, v in grid.items()
                if v is not None and v[3]]
        mins[duty_max] = min(vals) if vals else None

    for duty_max in DUTY_CEILINGS:
        m = mins[duty_max]
        if m is None:
            print(f"  MAX_DUTY {duty_max:.2f}: no valid cell")
            continue
        tau, duty, apex, key = m
        print(f"  MAX_DUTY {duty_max:.2f}: min tau {tau:6.2f} N.m  "
              f"(k_rel {key[0]:.0f}, v~ {key[1]:.2f}, duty {duty:.3f}, "
              f"apex {apex:.0f} mm)")

    lo, hi = DUTY_CEILINGS[0], DUTY_CEILINGS[-1]
    if mins[lo] and mins[hi]:
        delta = mins[lo][0] - mins[hi][0]
        pct = 100.0 * delta / mins[lo][0]
        verb = "LOWERS" if delta > 0 else "RAISES"
        print()
        print(f"  relaxing {lo:.2f} -> {hi:.2f} {verb} the torque floor by "
              f"{abs(delta):.2f} N.m ({abs(pct):.1f}%)")

    # --- feasibility against every ceiling that has been quoted -----------
    print()
    print("=" * 78)
    print("FEASIBILITY OF THE BEST CELL, AND THE EROSION IT TOLERATES")
    print("=" * 78)
    for duty_max in DUTY_CEILINGS:
        m = mins[duty_max]
        if m is None:
            continue
        tau = m[0]
        print()
        print(f"  MAX_DUTY {duty_max:.2f}, template tau {tau:.2f} N.m:")
        for label, ceil in CEILINGS:
            fits = "fits" if tau <= ceil else "EXCEEDS"
            print(f"    vs {label:<24} {fits:<8} "
                  f"max tolerable erosion {ceil/tau:5.2f}x")

    print()
    print("  Measured erosion is 5-8x (S28, unclipped) with a 2.33x lower bound")
    print("  (S25). Nothing on this grid tolerates that, at either duty ceiling.")
    print("  The trot's answer is therefore NOT in the template -- it is in")
    print("  whatever makes the robot demand 5-8x what the model predicts, and")
    print("  that has never been decomposed.")
    print("=" * 78)
    print()


if __name__ == "__main__":
    main()
