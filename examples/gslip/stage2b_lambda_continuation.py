"""Where does the Ackermann cambered family actually END in lambda?

Open Issue #23 / log S185. Offline, no simulator.

WHY. Adopting the "measured" radius law left lam_in = 15 deg with no periodic
gait (least-squares residual 1.1e-02) while 5 and 10 closed, and under "torus"
all three closed. That was ONE solve from ONE cold seed at 15 deg -- which is
not evidence that the family ends, only that least_squares did not find it from
there. Stage 4's hardware matrix runs to 20 deg, so the difference matters.

METHOD. Numerical continuation instead of cold seeds: solve at lam_in = 0, then
step lambda up in small increments, each solve SEEDED FROM THE PREVIOUS
SOLUTION. A family that genuinely terminates will stop converging no matter how
small the step; a family that merely has a bad basin from a cold seed will walk
straight through the angle that failed.

Reported per step: convergence, the residual, and the fixed point itself, so a
fold (the solution turning back on itself) is visible as a state that stops
moving monotonically rather than only as a failure.

Run:
    uv run python examples/gslip/stage2b_lambda_continuation.py
    LEGWHEEL_RADIUS_LAW=torus uv run python .../stage2b_lambda_continuation.py
"""

from __future__ import annotations

import re

import numpy as np

from legwheel.models import cambered_return_map as crm
from legwheel.models import coronal_bip as bip
from legwheel.models.gslip import GSlipFailure

V_OP = 1.19
BETA0 = np.deg2rad(80.75)
SEED_X = [V_OP * np.cos(np.deg2rad(40.74)), 0.0, 0.32, 0.0, 0.0]

STEP_DEG = 0.5           # continuation step
LAM_MAX_DEG = 30.0       # walk well past Stage 4's 20
RESID_OK = 1e-4          # what counts as converged


def solve_straight(p):
    """The straight fixed point, solved EXACTLY as stage2b_clocked_torque does:
    solve_periodic's defaults, i.e. free (h, beta) at fixed speed."""
    return crm.solve_periodic(p, SEED_X, [BETA0, 0.0, 0.0])


def solve_at(p, lam_in, x_seed, u_seed, ride):
    """One cambered solve at lam_in, seeded from the previous solution.

    free_x / free_u MATCH stage2b_clocked_torque.py: vy, h, rho, drho and beta
    are solved, vx is HELD (the operating point is a speed, not an outcome).

    ⚠ Two ways to get this wrong, both of which produced a bogus "the family
    terminates below 0.5 deg" on earlier runs of this script, and both of which
    S45's own recorded gaits at 5/10/15 deg immediately contradicted:
      * using solve_periodic's DEFAULTS for the cambered solve (frees h and
        beta only -- too few unknowns to absorb a lean); and
      * anchoring on a straight solve that itself used the cambered free set,
        which lands on h = 0.3246 instead of the recorded 0.3260.
    `ride` is the ANCHOR's apex height, so lam_out comes from the same
    Ackermann condition the recorded numbers used.
    """
    _, lam_out = crm.ackermann_pair(lam_in, ride)
    u0 = [u_seed[0], lam_in, lam_out]
    try:
        x_star, u_star = crm.solve_periodic(p, x_seed, u0,
                                            free_x=(1, 2, 3, 4), free_u=(0,))
    except GSlipFailure as e:
        m = re.search(r"residual ([0-9.eE+-]+)", str(e))
        return None, None, float(m.group(1)) if m else float("inf"), str(e)
    resid = float(np.linalg.norm(x_star - crm.apex_map(p, x_star, u_star)))
    return x_star, u_star, resid, ""


def main() -> None:
    law = bip.RADIUS_LAW_DEFAULT
    print(f"lambda continuation of the Ackermann family -- radius_law = {law}")
    print(f"  step {STEP_DEG} deg, converged if |x - f(x,u)| < {RESID_OK:g}")
    print()
    print(f"  {'lam_in':>7} {'lam_out':>8} {'resid':>10} {'beta*':>8} "
          f"{'vx*':>8} {'vy*':>9} {'h*':>7} {'rho*':>8}  status")

    p = crm.PairParams()
    # Anchor at lambda = 0, where both radius laws agree and the answer is the
    # sagittal fixed point -- if THIS does not close, nothing downstream means
    # anything.
    x, u = solve_straight(p)
    RIDE = float(x[2])
    r = float(np.linalg.norm(x - crm.apex_map(p, x, u)))
    print(f"  {0.0:7.2f} {0.0:8.2f} {r:10.2e} {np.rad2deg(u[0]):8.2f} "
          f"{x[0]:8.4f} {x[1]:+9.4f} {x[2]:7.4f} {np.rad2deg(x[3]):+8.3f}  anchor")

    last_ok = 0.0
    lam = STEP_DEG
    while lam <= LAM_MAX_DEG + 1e-9:
        xs, us, r, err = solve_at(p, np.deg2rad(lam), x, u, RIDE)
        ok = xs is not None and r <= RESID_OK
        status = "ok" if ok else f"FAIL {err[:28]}"
        if xs is not None:
            print(f"  {lam:7.2f} {np.rad2deg(us[2]):8.2f} {r:10.2e} "
                  f"{np.rad2deg(us[0]):8.2f} {xs[0]:8.4f} {xs[1]:+9.4f} "
                  f"{xs[2]:7.4f} {np.rad2deg(xs[3]):+8.3f}  {status}")
        else:
            print(f"  {lam:7.2f} {'--':>8} {r:10.2e} {'--':>8} {'--':>8} "
                  f"{'--':>9} {'--':>7} {'--':>8}  {status}")
        if not ok:
            print()
            print(f"  FAMILY TERMINATES between {last_ok:.2f} and {lam:.2f} deg "
                  f"under radius_law={law}.")
            # Refine once at a tenth of the step, to distinguish a real end
            # from a step-size artefact.
            fine = last_ok + STEP_DEG / 10.0
            xf, uf, rf, _ = solve_at(p, np.deg2rad(fine), x, u, RIDE)
            okf = xf is not None and rf <= RESID_OK
            print(f"  refine at {fine:.2f} deg (step/10): "
                  f"{'converges, resid %.2e' % rf if okf else 'also fails'}")
            if okf:
                print("  -> the boundary is sharp at this resolution, not a "
                      "step-size artefact.")
            return
        x, u, last_ok = xs, us, lam
        lam += STEP_DEG

    print()
    print(f"  family reaches {LAM_MAX_DEG:.1f} deg without terminating "
          f"under radius_law={law}.")


if __name__ == "__main__":
    main()
