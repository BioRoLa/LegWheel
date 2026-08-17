"""Does actuated roll stabilization rescue the cambered pronk? The stage gate.

Stage 2b Modules 3+4, on the 3D cambered-pair return map. Three questions, in
the order the stage asks them:

1. EXISTENCE (Stage 2a's gate, on the 3D model): do periodic gaits exist for
   Ackermann camber pairs at the operating speed? Fixed points are solved with
   least squares on ||x - f(x, u)||, x = [vx, vy, h, rho, drho], E = I (the
   pronk repeats every apex; Sovukluk's leg-swap E is for alternating gaits).
2. PASSIVE FAILURE (Chang Fig. 13, on our own model): steps-to-fail over a
   (rho, drho) apex perturbation grid with no actuation.
3. RESCUE (the lab's own fix, ours through the ABAD): the same grid with
   in-stance PD roll torque, and with apex-to-apex deadbeat on top. Chang's
   Table 1 shape -- 0.59% -> 100% surviving 7 steps -- is the comparison.

The ABAD torque budget is reported because section 37 showed the joint already
works near its 44.25 N.m ceiling in the running gait: a rescue that needs more
torque than the joint has is not a rescue.

Run:
    uv run python examples/gslip/stage2b_clocked_torque.py
"""

from __future__ import annotations

import numpy as np

from legwheel.models import cambered_return_map as crm
from legwheel.models.cambered_return_map import PairParams, RollPD
from legwheel.models.gslip import GSlipFailure

V_OP = 1.19                      # ~ v~0.70 operating point
BETA0 = np.deg2rad(80.75)        # the v~0.70 sagittal fixed point's beta*
SEED = [V_OP * np.cos(np.deg2rad(40.74)), 0.0, 0.32, 0.0, 0.0]

MAX_STEPS = 12
# Two regimes, because the answer differs and both matter. NEAR is the gate
# question (is the fixed point stabilizable at all); FAR is the basin question
# (what disturbance the actuation can absorb inside the ABAD torque budget).
GRIDS = {
    "NEAR": (np.deg2rad([-3.0, -1.5, 1.5, 3.0]), [-0.15, -0.08, 0.08, 0.15]),
    "FAR": (np.deg2rad([-12.0, -6.0, 6.0, 12.0]), [-0.6, -0.3, 0.3, 0.6]),
}


def survival_table(grid: np.ndarray, rho_grid, drho_grid) -> str:
    head = "rho / drho"
    lines = [f"{head:>10} " + " ".join(f"{d:>6.2f}" for d in drho_grid)]
    for r, row in zip(rho_grid, grid):
        lines.append(f"{np.rad2deg(r):>9.1f}d " +
                     " ".join(f"{int(n):>6d}" for n in row))
    return "\n".join(lines)


def main() -> None:
    print(__doc__.split("Run:")[0].rstrip())
    print()
    p = PairParams()

    # -- 1: the straight fixed point, then the cambered family --------------
    x_star, u_star = crm.solve_periodic(p, SEED, [BETA0, 0.0, 0.0])
    print(f"straight fixed point: vx {x_star[0]:.4f} m/s, apex h "
          f"{x_star[2]:.4f} m, beta* {np.rad2deg(u_star[0]):.2f} deg")

    print()
    print("Ackermann cambered family (lam_in, lam_out solved as a pair):")
    ride = float(np.sqrt(max(p.k_side, 1.0)) * 0.0 + x_star[2])  # apex height
    for lam_in_deg in (5.0, 10.0, 15.0):
        lam_in, lam_out = crm.ackermann_pair(np.deg2rad(lam_in_deg), ride)
        try:
            xc, uc = crm.solve_periodic(
                p, x_star, [u_star[0], lam_in, lam_out],
                free_x=(1, 2, 3, 4), free_u=(0,))
            print(f"  lam_in {lam_in_deg:4.1f}d (out {np.rad2deg(lam_out):4.2f}d)"
                  f" -> EXISTS: vy* {xc[1]:+.4f} m/s, rho* "
                  f"{np.rad2deg(xc[3]):+.3f} deg, beta* {np.rad2deg(uc[0]):.2f} deg")
        except GSlipFailure as e:
            print(f"  lam_in {lam_in_deg:4.1f}d -> {e}")

    # -- 2 and 3: survival grids, near then far -----------------------------
    jx, ju = crm.jacobians(p, x_star, u_star)
    k_db = crm.deadbeat_gain(jx, ju)
    for regime, (rho_g, drho_g) in GRIDS.items():
        print()
        print(f"=== {regime} regime: steps-to-fail, max {MAX_STEPS} strides ===")
        passive = crm.perturbation_grid(p, x_star, u_star, rho_g, drho_g,
                                        max_steps=MAX_STEPS)
        print("PASSIVE (no actuation):")
        print(survival_table(passive, rho_g, drho_g))

        ctrl = RollPD()
        pd_grid = crm.perturbation_grid(p, x_star, u_star, rho_g, drho_g,
                                        ctrl=ctrl, max_steps=MAX_STEPS)
        print(f"IN-STANCE ROLL PD (kp {ctrl.kp:.0f}, kd {ctrl.kd:.0f}, "
              f"clamp {ctrl.tau_max:.0f} N.m) -- peak demanded "
              f"{ctrl.peak_used:.1f} N.m:")
        print(survival_table(pd_grid, rho_g, drho_g))

        ctrl2 = RollPD()
        both = crm.perturbation_grid(p, x_star, u_star, rho_g, drho_g,
                                     ctrl=ctrl2, gain=k_db,
                                     max_steps=MAX_STEPS)
        print(f"ROLL PD + APEX DEADBEAT (beta + differential camber) -- peak "
              f"demanded {ctrl2.peak_used:.1f} N.m:")
        print(survival_table(both, rho_g, drho_g))

        for name, g in (("passive", passive), ("roll PD", pd_grid),
                        ("PD+deadbeat", both)):
            pct = 100.0 * np.count_nonzero(g >= MAX_STEPS) / g.size
            print(f"  {name:>12}: {pct:5.1f}% survive {MAX_STEPS} strides "
                  f"(mean {g.mean():.1f})")


if __name__ == "__main__":
    main()
