"""Which roll gains buy the biggest basin INSIDE the ABAD torque budget?

Section 45 left two open ends, both with machinery: the FAR-regime basin
boundary, and a budget-constrained gain design. Its gains (kp 400, kd 31,
clamp 40) were a first guess -- kd critical against J_roll, clamp sized from
the 44.25 N.m ceiling -- and the gate run demanded 42.6-53.3 N.m near the
orbit against ~40 practical, with FAR cells starving the clamp so badly that
two corners did WORSE than passive. This script answers, in order:

1. SWEEP: (kp, kd/kd_crit, tau_max) over a merged perturbation grid, PD +
   apex deadbeat (section 45: the deadbeat is what closes the loop; PD alone
   rescues nothing). Ranked by survival, then by peak demand among the cells
   that survived -- a cell that survives while demanding 3x its clamp is being
   carried by the deadbeat, not the PD.
2. BOUNDARY: r(theta) of the connected basin along 16 rays, for the section 45
   default and the sweep's best configs. This is where the FAR regime's edge
   actually sits.
3. PATHOLOGY: the clipped-PD-worse-than-passive cells, checked for the winner.
4. RATE: the deadbeat's per-stride camber adjustment against the ABAD speed
   limit (motor_rate_budget: 220 rpm at 9:1 ~= 2.56 rad/s at the joint) --
   the torque budget is enforced by the clamp, the SPEED budget is only
   measured here.
5. CAUDAL BIAS: the Timeline's cheap extra dimension, read as a deliberate
   touchdown-angle offset beta* + dbeta held while the deadbeat trims about
   it. (Interpretation flag: "caudal shift" in Speed-Dependent Turning
   Strategies is a footfall-target shift; beta is this model's only sagittal
   touchdown input, so dbeta is its image here.)

Seed note: this file inherits stage2b_clocked_torque's canonical operating
point (V_OP 1.19 m/s along the leg plane, apex seed vx = V_OP*cos(40.74 deg),
h 0.32, BETA0 80.75 deg -> beta* 80.91). tests/test_cambered_return_map.py
solves a DIFFERENT orbit (vx = 1.19 directly, beta0 71.75): same model, other
operating point. The section 45 numbers, and everything here, are the former.

Run:
    uv run python examples/gslip/stage2b_budget_gain_sweep.py [--quick]
"""

from __future__ import annotations

import sys

import numpy as np

from legwheel.models import cambered_return_map as crm
from legwheel.models.cambered_return_map import PairParams, RollPD

V_OP = 1.19
BETA0 = np.deg2rad(80.75)
SEED = [V_OP * np.cos(np.deg2rad(40.74)), 0.0, 0.32, 0.0, 0.0]

MAX_STEPS = 12
J_ROLL = PairParams().j_roll
ABAD_JOINT_SPEED = 2.56          # rad/s at the joint, 220 rpm / 9:1 gearbox

# Merged grid: NEAR (+-1.5..3 deg) sat inside it, FAR's inner ring (+-6 deg,
# +-0.3 rad/s) bounds it, and unlike the section 45 grids it samples the
# rho = 0 row/column -- the gate grids never checked the unperturbed point.
RHO_GRID = np.deg2rad([-6.0, -3.0, 0.0, 3.0, 6.0])
DRHO_GRID = np.array([-0.3, -0.15, 0.0, 0.15, 0.3])

KP_VALS = [100.0, 200.0, 400.0, 700.0, 1000.0]
KD_RATIOS = [0.5, 0.8, 1.0, 1.3, 1.6]     # x kd_crit = 2 sqrt(kp J_roll)
TAU_VALS = [30.0, 40.0]

N_RAYS = 16
RAY_RHO_SCALE = np.deg2rad(6.0)           # r = 1 is the FAR inner ring
RAY_DRHO_SCALE = 0.3

# Physical input range for the deadbeat: beta inside solve_periodic's own
# validity band, camber inside +-30 deg (side_geometry is first-order in the
# lean; past that the model, not the joint, is the limit). Without this the
# linear gain commanded up to 1524 deg of camber per stride on the first
# sweep -- wrapping through the trig and "surviving" on impossible inputs.
U_LO = np.array([np.deg2rad(40.0), -np.deg2rad(30.0), -np.deg2rad(30.0)])
U_HI = np.array([np.deg2rad(89.0), +np.deg2rad(30.0), +np.deg2rad(30.0)])
U_LIMITS = (U_LO, U_HI)


def kd_crit(kp: float) -> float:
    return 2.0 * np.sqrt(kp * J_ROLL)


def sweep_configs(quick: bool):
    kps = [200.0, 400.0, 700.0] if quick else KP_VALS
    ratios = [0.8, 1.0, 1.3] if quick else KD_RATIOS
    taus = [40.0] if quick else TAU_VALS
    for tau in taus:
        for kp in kps:
            for ratio in ratios:
                yield RollPD(kp=kp, kd=ratio * kd_crit(kp), tau_max=tau)


def main(argv: list[str]) -> None:
    quick = "--quick" in argv
    print(__doc__.split("Run:")[0].rstrip())
    print()
    p = PairParams()

    # -- 1: fixed point and deadbeat, once ---------------------------------
    x_star, u_star = crm.solve_periodic(p, SEED, [BETA0, 0.0, 0.0])
    print(f"fixed point: vx {x_star[0]:.4f} m/s, h* {x_star[2]:.4f} m, "
          f"beta* {np.rad2deg(u_star[0]):.2f} deg  (section 45: 80.91)")
    jx, ju = crm.jacobians(p, x_star, u_star)
    k_db = crm.deadbeat_gain(jx, ju)
    t_flight = 2.0 * np.sqrt(2.0 * x_star[2] / p.g)   # ballistic upper bound

    # Invariant: input clamping must not change the section 45 NEAR verdict
    # (its deadbeat commands were small), or the clamp itself is suspect.
    near = crm.basin_scan(p, x_star, u_star,
                          np.deg2rad([-3.0, -1.5, 1.5, 3.0]),
                          [-0.15, -0.08, 0.08, 0.15],
                          ctrl_proto=RollPD(), gain=k_db,
                          max_steps=MAX_STEPS, u_limits=U_LIMITS)
    print(f"NEAR invariant with input clamp: "
          f"{100 * near.survival_fraction:.1f}% survive "
          f"(section 45: 100%), peak {near.peak_max:.1f} N.m, "
          f"max dlam {np.rad2deg(near.dlam.max()):.2f} deg")

    # -- 2: the (kp, kd, tau_max) sweep, PD + deadbeat ----------------------
    print()
    print(f"=== SWEEP: PD + deadbeat on the merged 5x5 grid "
          f"(rho +-6 deg, drho +-0.3), max {MAX_STEPS} strides ===")
    print(f"{'kp':>6} {'kd':>7} {'kd/kdc':>6} {'clamp':>5} | "
          f"{'survive':>8} {'mean':>5} {'peak':>7} {'peak_srv':>8} {'dlam':>6}")
    results = []
    for proto in sweep_configs(quick):
        res = crm.basin_scan(p, x_star, u_star, RHO_GRID, DRHO_GRID,
                             ctrl_proto=proto, gain=k_db, max_steps=MAX_STEPS,
                             u_limits=U_LIMITS)
        results.append((proto, res))
        print(f"{proto.kp:6.0f} {proto.kd:7.1f} "
              f"{proto.kd / kd_crit(proto.kp):6.2f} {proto.tau_max:5.0f} | "
              f"{100 * res.survival_fraction:7.1f}% {res.steps.mean():5.1f} "
              f"{res.peak_max:7.1f} {res.peak_max_surviving:8.1f} "
              f"{np.rad2deg(res.dlam.max()):5.2f}d")

    # Rank: survival first, then LOWEST peak among surviving cells -- the
    # config that holds the same basin while asking the joint for less.
    ranked = sorted(results, key=lambda pr: (-pr[1].survival_fraction,
                                             pr[1].peak_max_surviving))
    best = ranked[0]
    print()
    print(f"best inside the budget: kp {best[0].kp:.0f}, kd {best[0].kd:.1f}, "
          f"clamp {best[0].tau_max:.0f} -> {100 * best[1].survival_fraction:.1f}% "
          f"survive, peak {best[1].peak_max_surviving:.1f} N.m on survivors")

    # -- 3: basin boundary rays --------------------------------------------
    print()
    print(f"=== BOUNDARY: r(theta), scales rho {np.rad2deg(RAY_RHO_SCALE):.0f} "
          f"deg / drho {RAY_DRHO_SCALE} rad/s per unit r (r=1 is FAR inner "
          f"ring, r_max 3) ===")
    angles = np.linspace(0.0, 2 * np.pi, N_RAYS, endpoint=False)
    boundary_set = [("section 45 default", RollPD())] + \
                   [(f"sweep #{i + 1}", pr[0]) for i, pr in
                    enumerate(ranked[:2])]
    for label, proto in boundary_set:
        r = crm.basin_radius(p, x_star, u_star, angles,
                             RAY_RHO_SCALE, RAY_DRHO_SCALE,
                             ctrl_proto=proto, gain=k_db,
                             max_steps=MAX_STEPS, u_limits=U_LIMITS)
        line = " ".join(f"{v:4.2f}" for v in r)
        print(f"{label:>18} (kp {proto.kp:.0f} kd {proto.kd:.0f} "
              f"clamp {proto.tau_max:.0f}): min {r.min():.2f} "
              f"mean {r.mean():.2f}\n{'':>20}r(theta) = {line}")

    # -- 4: the clipped-PD pathology ---------------------------------------
    print()
    print("=== PATHOLOGY: PD-only cells doing worse than passive ===")
    passive = crm.basin_scan(p, x_star, u_star, RHO_GRID, DRHO_GRID,
                             max_steps=MAX_STEPS)
    for label, proto in (("section 45 default", RollPD()), ("winner", best[0])):
        pd_only = crm.basin_scan(p, x_star, u_star, RHO_GRID, DRHO_GRID,
                                 ctrl_proto=proto, max_steps=MAX_STEPS)
        worse = pd_only.steps < passive.steps
        cells = [(f"{np.rad2deg(RHO_GRID[i]):+.0f}d/{DRHO_GRID[j]:+.2f}")
                 for i, j in zip(*np.nonzero(worse))]
        print(f"  {label}: {worse.sum()} of {worse.size} cells worse than "
              f"passive{' -> ' + ', '.join(cells) if cells else ''}")

    # -- 5: rate budget (measured, not enforced) ---------------------------
    print()
    print("=== RATE: deadbeat camber slew vs the ABAD speed limit ===")
    dlam_max = best[1].dlam.max()
    print(f"  largest single-stride camber adjustment {np.rad2deg(dlam_max):.2f}"
          f" deg; at {ABAD_JOINT_SPEED} rad/s the joint needs "
          f"{dlam_max / ABAD_JOINT_SPEED * 1e3:.0f} ms against ~"
          f"{t_flight * 1e3:.0f} ms of ballistic flight (upper bound on the "
          f"reorientation window)")

    # -- 6: caudal touchdown bias ------------------------------------------
    print()
    print("=== CAUDAL BIAS: beta* + dbeta held, deadbeat trimming about it ===")
    for dbeta_deg in (-2.0, 0.0, +2.0):
        u_b = u_star.copy()
        u_b[0] += np.deg2rad(dbeta_deg)
        res = crm.basin_scan(p, x_star, u_b, RHO_GRID, DRHO_GRID,
                             ctrl_proto=best[0], gain=k_db,
                             max_steps=MAX_STEPS, u_limits=U_LIMITS)
        print(f"  dbeta {dbeta_deg:+4.1f} deg: "
              f"{100 * res.survival_fraction:5.1f}% survive, "
              f"mean {res.steps.mean():.1f}, peak on survivors "
              f"{res.peak_max_surviving:.1f} N.m")


if __name__ == "__main__":
    main(sys.argv[1:])
