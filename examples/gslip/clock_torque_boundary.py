"""Damping, not stiffness: where is the clock-torque stability boundary, and
what does crossing it cost in the controller's own units?

REWRITTEN 2026-08-22. The previous version of this file inherited three of the
five defects S149 found in clock_torque_gate (beta-window pinning, min|slope|
branch selection, no v-closure). It never produced a number that was recorded.
This version solves at a HELD beta with a v-closure screen, via
clock_torque_regime_switch.solve_held_beta.

WHY DAMPING

S149's arm-for-arm check (kt_arms_vs_plant.py) found that at v~0.70, with
b_tangential fixed at its shipped 30, EVERY arm of the banked k_tangential
sweep is unstable in the constant leg-frame regime:

    k_tangential   150     300     600(shipped)   1200
    model |slope|  1.2010  1.1852  1.1540         1.0936

Doubling the shipped stiffness moves the slope by 0.06 and does not cross 1.
Stiffness alone cannot get there. S141's grid already implied this -- its
k_c 40 was unstable at d_c 2.0 and stable at d_c 6.0 -- but the implication was
never tested, because S141's affordability gate rejected the row on a torque
number that S149 showed to be a search artefact.

So the question this file asks is narrow and decision-relevant:

    holding k_tangential at values the plant can actually be given, how much
    b_tangential does it take to cross |slope| = 1, and what does that cost?

Everything is reported in the CONTROLLER's units, because the answer is meant
to be typed into a launch file.

TORQUE IS REPORTED, NOT GATED (Alex's call). The per-motor column already
applies the x0.5 leg-axis split S149 found missing from S141, and the erosion
factor spans 1.1x-8x by configuration (S25/S28/S64) so it is a scale, not a
verdict.

Run:
    uv run python examples/gslip/clock_torque_boundary.py
    uv run python examples/gslip/clock_torque_boundary.py --selftest
"""

from __future__ import annotations

import sys

import numpy as np

from clock_torque_regime_switch import (
    B_TANGENTIAL, EROSION, K_TANGENTIAL, STALL, baseline, params,
    solve_held_beta, to_model,
)

V_TILDES = (0.70, 0.34)

# Gains the plant can actually be given. b_tangential 30 is shipped; 74 is the
# b_tangential that S141's stable d_c 6.0 maps to.
K_TS = (600.0, 1200.0, 2400.0)
B_TS = (30.0, 45.0, 60.0, 74.0, 90.0, 120.0, 160.0)


def selftest() -> int:
    print("SELF-TEST")
    fails = 0
    p = params()

    # 1. the shipped cell must reproduce S149's constant leg-frame number
    v, fp0 = baseline(p, 0.70)
    g = to_model(K_TANGENTIAL, B_TANGENTIAL)
    r = solve_held_beta(p, v, fp0, g, g, np.inf)
    if r is None:
        print("  FAIL: no admitted root at the shipped gains"); fails += 1
    else:
        print("  shipped (k_t %.0f, b_t %.0f) -> slope %+.4f  (S149: +1.1540)"
              % (K_TANGENTIAL, B_TANGENTIAL, r["slope"]))
        if abs(r["slope"] - 1.1540) > 0.01:
            print("  FAIL: does not reproduce S149's constant leg-frame slope")
            fails += 1
        print("  v closure %+.3f%% (screen is %.1f%%)"
              % (100 * (r["v_ratio"] - 1), 100 * 0.02))

    # 2. more damping must not make it LESS stable at the shipped stiffness --
    #    if it does, the premise of this whole file is wrong and it should say so
    g2 = to_model(K_TANGENTIAL, 120.0)
    r2 = solve_held_beta(p, v, fp0, g2, g2, np.inf)
    if r2 is not None and r is not None:
        print("  b_t 30 -> %+.4f ; b_t 120 -> %+.4f" % (r["slope"], r2["slope"]))
        if abs(r2["slope"]) >= abs(r["slope"]):
            print("  NOTE: damping does NOT help here. The file's premise fails;")
            print("        report that rather than hunting for a crossing.")

    print("\n%s" % ("SELF-TEST PASSED" if not fails else "SELF-TEST FAILED (%d)" % fails))
    return 1 if fails else 0


def main() -> None:
    print(__doc__.split("Run:")[0].rstrip())
    print()
    p = params()
    for vt in V_TILDES:
        v, fp0 = baseline(p, vt)
        print("=" * 94)
        print("v~ %.2f   baseline beta* %.2f deg (HELD), conservative slope %+.4f"
              % (vt, np.rad2deg(fp0.beta), fp0.slope))
        print("=" * 94)
        print("  %9s %9s %8s %8s %10s %9s %11s %s"
              % ("k_tang", "b_tang", "k_c", "d_c", "slope", "v drift",
                 "tau/motor", ""))
        crossings, n_stable, n_cells = [], 0, 0
        for kt in K_TS:
            prev = None
            for bt in B_TS:
                k_c, d_c = to_model(kt, bt)
                r = solve_held_beta(p, v, fp0, (k_c, d_c), (k_c, d_c), np.inf)
                if r is None:
                    print("  %9.0f %9.0f %8.2f %8.2f %10s %9s %11s  no admitted root"
                          % (kt, bt, k_c, d_c, "--", "--", "--"))
                    continue
                st = abs(r["slope"]) < 1.0
                n_cells += 1
                n_stable += int(st)
                mark = "  STABLE" if st else ""
                if kt == K_TANGENTIAL and bt == B_TANGENTIAL:
                    mark += "   <== SHIPPED"
                print("  %9.0f %9.0f %8.2f %8.2f %+10.4f %8.2f%% %11.1f%s"
                      % (kt, bt, k_c, d_c, r["slope"],
                         100 * (r["v_ratio"] - 1), r["tau_motor"], mark))
                if prev is not None and not prev[1] and st:
                    crossings.append((kt, prev[0], bt, r["tau_motor"]))
                prev = (bt, st)
            print()
        if crossings:
            print("  |slope| = 1 is crossed between:")
            for kt, b0, b1, tq in crossings:
                print("      k_tangential %.0f : b_tangential %.0f -> %.0f,"
                      " at %.1f N.m/motor (stall %.1f)" % (kt, b0, b1, tq, STALL))
        elif n_cells and n_stable == n_cells:
            # "No crossing" has two opposite meanings and the first version of
            # this message conflated them. Everything already stable is not a
            # failure to find a boundary -- it is the boundary being off-grid
            # on the good side.
            print("  NO CROSSING because EVERY cell is already stable -- the")
            print("  boundary lies below this grid at this speed, not above it.")
        elif n_cells and n_stable == 0:
            print("  NO CROSSING because NO cell is stable. Neither lever gets")
            print("  there within gains the plant can be given at this speed.")
        else:
            print("  No monotone crossing found along any b_tangential column,")
            print("  though %d of %d cells are stable -- read the table."
                  % (n_stable, n_cells))
        print()
    print("=" * 94)
    print("tau/motor = orbit's own peak x%.2f erosion x0.5 leg-axis split." % EROSION)
    print("Reported, not gated. Erosion is configuration-dependent (S149 sec 3).")
    print("=" * 94)


if __name__ == "__main__":
    raise SystemExit(selftest() if "--selftest" in sys.argv else main())
