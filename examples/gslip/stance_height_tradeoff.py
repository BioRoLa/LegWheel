"""What does a taller nominal stance cost the G-SLIP template?

Phase 0 picked theta = 65.89 deg because it hits the paper's combined-indicator
optimum, r~ = 0.6303. Because the Corgi's foot radius (0.145 m) is a large
fraction of its leg, that optimum is inherently a crouch: hip 0.230 m, about
100 mm lower than the pose the trot CSV stands in, and too low to clear the
support block.

This sweeps nominal theta and reports what is actually given up by standing
taller: stiffness, fixed point, ground reaction and motor torque.

Run:
    uv run python examples/gslip/stance_height_tradeoff.py
"""

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.models.leg_model import LegModel
from legwheel.planners import gslip_to_corgi as g2c

MASS, G = 30.0, 9.81
K_REL = 18.0
V_TILDE = 1.2
N_LEGS = 4
MOTOR_LIMIT = 35.0
R_TILDE_OPT = 0.6303
BLOCK_CLEARANCE = 0.3309  # hip height the trot CSV stands at, which clears the block


def main() -> None:
    leg = LegModel()
    leg_map = g2c.LegLengthMap(leg)
    r = leg.foot_radius

    print()
    print("=" * 104)
    print(f"NOMINAL STANCE HEIGHT TRADE-OFF   (k_rel = {K_REL}, v~ = {V_TILDE}, m = {MASS} kg)")
    print("=" * 104)
    header = (f"{'theta':>7} {'l0[m]':>7} {'hip[m]':>7} {'r~':>7} {'k[N/m]':>8} "
              f"{'k_leg':>7} {'beta*':>7} {'alpha*':>7} {'|slope|':>8} "
              f"{'GRF/BW':>7} {'tau[Nm]':>8} {'torq%':>6} {'clears':>7}")
    print(header)
    print("-" * len(header))

    for theta_deg in (65.89, 75.0, 85.0, 95.0, 105.0, 115.0):
        theta = np.deg2rad(theta_deg)
        l0 = leg_map.length(theta)
        hip = l0 + r
        r_tilde = r / (l0 + r)
        k = K_REL * MASS * G / l0

        p = slip_rf.SlipRfParams(m=MASS, l0=hip, k=k, r=r)
        v = V_TILDE * np.sqrt(G * p.l0)

        best = None
        for beta_deg in np.arange(60.0, 85.01, 0.5):
            try:
                for fp in find_fixed_points(
                    p, v, np.deg2rad(beta_deg),
                    alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
                    n_samples=25, stride_fn=slip_rf.stride,
                ):
                    if best is None or abs(fp.slope) < abs(best.slope):
                        best = fp
            except Exception:
                continue

        if best is None:
            print(f"{theta_deg:7.2f} {l0:7.4f} {hip:7.4f} {r_tilde:7.4f} {k:8.0f} "
                  f"{k/N_LEGS:7.0f}   -- no fixed point found --")
            continue

        res = slip_rf.stride(p, v, best.alpha, best.beta)
        tau = (res["peak_grf_mag"] / N_LEGS) * leg_map.slope(theta) / 2.0
        clears = "yes" if hip >= BLOCK_CLEARANCE else "no"
        print(f"{theta_deg:7.2f} {l0:7.4f} {hip:7.4f} {r_tilde:7.4f} {k:8.0f} "
              f"{k/N_LEGS:7.0f} {np.rad2deg(best.beta):7.2f} "
              f"{np.rad2deg(best.alpha):7.2f} {abs(best.slope):8.4f} "
              f"{res['peak_grf_mag']/(MASS*G):7.2f} {tau:8.2f} "
              f"{100*tau/MOTOR_LIMIT:6.1f} {clears:>7}")

    print()
    print(f"Paper's combined-indicator optimum is r~ = {R_TILDE_OPT}; the trot CSV")
    print(f"stands at hip {BLOCK_CLEARANCE:.4f} m, which is what clears the support block.")
    print()


if __name__ == "__main__":
    main()
