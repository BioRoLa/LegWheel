"""Phase 2: pick the Corgi's target running speed from its hardware limits.

Sweeps speed, finds the SLIP-RF fixed point at each, and scores it against
every constraint the real robot imposes:

  * peak motor torque vs the 35 N.m limit
  * stance length vs the 0.2025 m foot-arc budget (Phase 0), beyond which
    contact leaves the foot arc and the telescoping reduction breaks
  * peak leg compression vs the theta workspace (17-160 deg)
  * peak ground reaction force in body weights

Motor torque follows from virtual work. The Corgi's motors are related to the
leg coordinates by (force_control.cpp)

    phi_R = beta + theta - 17deg,   phi_L = beta - theta + 17deg

so a purely radial motion at fixed beta has d(phi_R) = -d(phi_L) = d(theta),
and F_leg * dl/dtheta = tau_R - tau_L. Splitting evenly between the two
motors gives

    tau_motor = F_leg * (dl/dtheta) / 2

with dl/dtheta taken from the real linkage via LegModel.

Run:
    uv run python examples/gslip/corgi_speed_study.py
"""

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.gslip import GSlipFailure
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.models.leg_model import LegModel

MASS = 30.0
G = 9.81
FOOT_RADIUS = 0.145
NOMINAL_THETA_DEG = 65.89  # Phase 0: gives r~ = 0.6303, the paper's optimum
HIP_TO_ARC = 0.0850  # l0 - r at the nominal pose
FOOT_ARC_BUDGET = 0.2025  # r * 80 deg, max rolling stance on the foot arc
MOTOR_TORQUE_LIMIT = 35.0
N_LEGS = 4
K_REL_PAPER = 18.0


def leg_length_slope(theta_deg: float, delta_deg: float = 0.5) -> float:
    """dl/dtheta of the real linkage at a given theta (m/rad)."""
    leg = LegModel()
    out = []
    for t in (theta_deg - delta_deg, theta_deg + delta_deg):
        leg.forward(np.deg2rad(t), 0.0, vector=False)
        out.append(abs(complex(leg.O_r)))
    return (out[1] - out[0]) / np.deg2rad(2 * delta_deg)


def theta_for_leg_length(target: float) -> float:
    """Inverse of the hip-to-arc-center map, in degrees."""
    from scipy.optimize import brentq

    leg = LegModel()

    def f(theta_deg: float) -> float:
        leg.forward(np.deg2rad(theta_deg), 0.0, vector=False)
        return abs(complex(leg.O_r)) - target

    return float(brentq(f, 17.5, 160.0))


def corgi_params(k_rel: float = K_REL_PAPER) -> slip_rf.SlipRfParams:
    k = k_rel * MASS * G / HIP_TO_ARC
    return slip_rf.SlipRfParams(m=MASS, l0=HIP_TO_ARC + FOOT_RADIUS, k=k, r=FOOT_RADIUS)


def evaluate(p: slip_rf.SlipRfParams, fp, dl_dtheta: float) -> dict:
    res = slip_rf.stride(p, fp.v, fp.alpha, fp.beta)
    f_leg_peak = res["peak_grf_mag"] / N_LEGS
    tau = f_leg_peak * dl_dtheta / 2.0
    min_hip_to_arc = res["min_length"] - p.r
    return {
        "peak_grf_bw": res["peak_grf_mag"] / (MASS * G),
        "f_leg_peak": f_leg_peak,
        "tau_motor": tau,
        "torque_use": tau / MOTOR_TORQUE_LIMIT,
        "stance_length": res["stance_length"],
        "arc_use": abs(res["stance_length"]) / FOOT_ARC_BUDGET,
        "compression": res["peak_compression"],
        "min_hip_to_arc": min_hip_to_arc,
        "theta_min_deg": theta_for_leg_length(min_hip_to_arc)
        if min_hip_to_arc > 0.001
        else float("nan"),
        "duty": fp.duty_factor,
        "mean_speed": fp.mean_speed,
    }


def main() -> None:
    dl_dtheta = leg_length_slope(NOMINAL_THETA_DEG)
    p = corgi_params()
    v_scale = np.sqrt(G * p.l0)

    print()
    print("=" * 100)
    print("CORGI SLIP-RF PARAMETERS (from Phase 0)")
    print("=" * 100)
    print(f"  m = {MASS} kg, mg = {MASS*G:.1f} N")
    print(f"  foot radius r      = {p.r} m")
    print(f"  rest length l0     = {p.l0:.4f} m  (hip-to-arc {HIP_TO_ARC} + r)")
    print(f"  stiffness k        = {p.k:.0f} N/m at k_rel = {K_REL_PAPER}")
    print(f"  k per leg (pronk)  = {p.k/N_LEGS:.0f} N/m")
    print(f"  dl/dtheta at {NOMINAL_THETA_DEG} deg = {dl_dtheta:.5f} m/rad")
    print(f"  => tau_motor = F_leg * {dl_dtheta/2:.5f}")
    print(f"  => F_leg at the {MOTOR_TORQUE_LIMIT} N.m limit "
          f"= {MOTOR_TORQUE_LIMIT/(dl_dtheta/2):.0f} N per leg "
          f"= {4*MOTOR_TORQUE_LIMIT/(dl_dtheta/2)/(MASS*G):.1f} body weights total")
    print(f"  speed scale sqrt(g*l0) = {v_scale:.4f} m/s")
    print()

    print("=" * 100)
    print("FIXED POINTS vs SPEED   (beta swept; the best-conditioned fixed point per speed)")
    print("=" * 100)
    header = (f"{'v[m/s]':>7} {'v~':>5} {'beta':>6} {'alpha':>6} {'|slope|':>8} "
              f"{'GRF/BW':>7} {'tau[Nm]':>8} {'torq%':>6} {'stance[m]':>10} {'arc%':>6} "
              f"{'th_min':>7} {'duty':>6}")
    print(header)
    print("-" * len(header))

    for v_tilde in (0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0):
        v = v_tilde * v_scale
        best = None
        for beta_deg in range(45, 86, 5):
            try:
                fps = find_fixed_points(
                    p, v, np.deg2rad(beta_deg),
                    alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
                    n_samples=40, stride_fn=slip_rf.stride,
                )
            except Exception:
                continue
            for fp in fps:
                if best is None or abs(fp.slope) < abs(best.slope):
                    best = fp
        if best is None:
            print(f"{v:7.3f} {v_tilde:5.2f}   -- no fixed point found --")
            continue
        try:
            m = evaluate(p, best, dl_dtheta)
        except (GSlipFailure, ValueError):
            print(f"{v:7.3f} {v_tilde:5.2f}   -- stride failed at fixed point --")
            continue
        print(f"{v:7.3f} {v_tilde:5.2f} {np.rad2deg(best.beta):6.1f} "
              f"{np.rad2deg(best.alpha):6.2f} {abs(best.slope):8.4f} "
              f"{m['peak_grf_bw']:7.2f} {m['tau_motor']:8.2f} "
              f"{100*m['torque_use']:6.1f} {m['stance_length']:10.4f} "
              f"{100*m['arc_use']:6.1f} {m['theta_min_deg']:7.1f} {m['duty']:6.3f}")

    print()
    print("=" * 100)
    print("SENSITIVITY TO LEG STIFFNESS (k_rel), at v~ = 1.2")
    print("=" * 100)
    print(f"{'k_rel':>6} {'k[N/m]':>9} {'k_leg':>8} {'beta':>6} {'alpha':>6} "
          f"{'GRF/BW':>7} {'tau[Nm]':>8} {'torq%':>6} {'stance[m]':>10} {'arc%':>6} {'th_min':>7}")
    print("-" * 96)
    for k_rel in (7.0, 12.0, 18.0, 22.0, 27.0):
        pk = corgi_params(k_rel)
        v = 1.2 * np.sqrt(G * pk.l0)
        best = None
        for beta_deg in range(45, 86, 5):
            try:
                for fp in find_fixed_points(
                    pk, v, np.deg2rad(beta_deg),
                    alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
                    n_samples=40, stride_fn=slip_rf.stride,
                ):
                    if best is None or abs(fp.slope) < abs(best.slope):
                        best = fp
            except Exception:
                continue
        if best is None:
            print(f"{k_rel:6.1f} {pk.k:9.0f}   -- no fixed point --")
            continue
        m = evaluate(pk, best, dl_dtheta)
        print(f"{k_rel:6.1f} {pk.k:9.0f} {pk.k/N_LEGS:8.0f} "
              f"{np.rad2deg(best.beta):6.1f} {np.rad2deg(best.alpha):6.2f} "
              f"{m['peak_grf_bw']:7.2f} {m['tau_motor']:8.2f} {100*m['torque_use']:6.1f} "
              f"{m['stance_length']:10.4f} {100*m['arc_use']:6.1f} {m['theta_min_deg']:7.1f}")
    print()


if __name__ == "__main__":
    main()
