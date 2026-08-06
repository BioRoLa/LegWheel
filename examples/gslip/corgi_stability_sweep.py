"""Phase 2: locate the Corgi's self-stable running region.

The coarse 5-degree beta sweep in corgi_speed_study.py found only marginally
unstable fixed points (|dP/dalpha| between 1.11 and 1.31). Classic SLIP
analysis says stable fixed points exist but occupy a narrow window in the
(landing angle, stiffness) plane, so this sweeps beta finely enough to
resolve it.

Uses continuation: each beta seeds its fixed-point solve from the previous
beta's solution, which is both faster and better conditioned than bracketing
from scratch.

Run:
    uv run python examples/gslip/corgi_stability_sweep.py
"""

import numpy as np
from scipy.optimize import brentq

from legwheel.models import slip_rf
from legwheel.models.gslip import GSlipFailure

MASS, G = 30.0, 9.81
FOOT_RADIUS, HIP_TO_ARC = 0.145, 0.0850
V_TILDE = 1.2  # chosen target: ~1.8 m/s
N_LEGS = 4
MOTOR_TORQUE_LIMIT = 35.0
DL_DTHETA = 0.10324  # m/rad at the nominal pose (see corgi_speed_study.py)


def params(k_rel: float) -> slip_rf.SlipRfParams:
    return slip_rf.SlipRfParams(
        m=MASS, l0=HIP_TO_ARC + FOOT_RADIUS,
        k=k_rel * MASS * G / HIP_TO_ARC, r=FOOT_RADIUS,
    )


def residual(p, v, beta):
    def f(alpha: float) -> float:
        return slip_rf.stride(p, v, alpha, beta)["alpha"] - alpha

    return f


def solve_alpha(p, v, beta, seed, half_width=np.deg2rad(6.0)):
    """Fixed-point alpha near `seed`, by expanding a bracket then Brent."""
    f = residual(p, v, beta)
    try:
        f_seed = f(seed)
    except (GSlipFailure, ValueError, np.linalg.LinAlgError):
        return None
    for scale in (0.25, 0.5, 1.0, 2.0):
        lo, hi = seed - scale * half_width, seed + scale * half_width
        lo = max(lo, np.deg2rad(0.2))
        hi = min(hi, np.deg2rad(75.0))
        try:
            f_lo, f_hi = f(lo), f(hi)
        except (GSlipFailure, ValueError, np.linalg.LinAlgError):
            continue
        if np.sign(f_lo) != np.sign(f_hi):
            try:
                return float(brentq(f, lo, hi, xtol=1e-13, rtol=1e-13))
            except (GSlipFailure, ValueError):
                continue
    return None


def slope_at(p, v, beta, alpha, h=np.deg2rad(0.02)):
    """dP/dalpha by central difference, in a step large enough to beat solver noise."""
    try:
        hi = slip_rf.stride(p, v, alpha + h, beta)["alpha"]
        lo = slip_rf.stride(p, v, alpha - h, beta)["alpha"]
    except (GSlipFailure, ValueError, np.linalg.LinAlgError):
        return None
    return (hi - lo) / (2 * h)


def bootstrap(p, v, beta):
    """Find any fixed point at this beta by coarse bracketing."""
    f = residual(p, v, beta)
    grid = np.deg2rad(np.linspace(1.0, 60.0, 80))
    vals = []
    for a in grid:
        try:
            vals.append((float(a), f(float(a))))
        except (GSlipFailure, ValueError, np.linalg.LinAlgError):
            vals.append((float(a), np.nan))
    for (a0, r0), (a1, r1) in zip(vals, vals[1:]):
        if np.isnan(r0) or np.isnan(r1) or np.sign(r0) == np.sign(r1):
            continue
        try:
            return float(brentq(f, a0, a1, xtol=1e-13, rtol=1e-13))
        except (GSlipFailure, ValueError):
            continue
    return None


def sweep(k_rel: float, beta_deg_values: np.ndarray) -> list[dict]:
    p = params(k_rel)
    v = V_TILDE * np.sqrt(G * p.l0)
    rows: list[dict] = []
    seed = None
    for beta_deg in beta_deg_values:
        beta = np.deg2rad(beta_deg)
        alpha = solve_alpha(p, v, beta, seed) if seed is not None else None
        if alpha is None:
            alpha = bootstrap(p, v, beta)
        if alpha is None:
            seed = None
            continue
        seed = alpha
        s = slope_at(p, v, beta, alpha)
        if s is None:
            continue
        try:
            res = slip_rf.stride(p, v, alpha, beta)
        except (GSlipFailure, ValueError):
            continue
        rows.append({
            "beta_deg": float(beta_deg),
            "alpha_deg": float(np.rad2deg(alpha)),
            "slope": float(s),
            "tau": res["peak_grf_mag"] / N_LEGS * DL_DTHETA / 2,
            "grf_bw": res["peak_grf_mag"] / (MASS * G),
            "duty": res["stance_time"] / res["period"],
        })
    return rows


def stable_windows(rows: list[dict]) -> list[tuple[float, float]]:
    """Contiguous beta ranges where |dP/dalpha| < 1."""
    windows, start, prev = [], None, None
    for row in rows:
        stable = abs(row["slope"]) < 1.0
        if stable and start is None:
            start = row["beta_deg"]
        elif not stable and start is not None:
            windows.append((start, prev))
            start = None
        prev = row["beta_deg"]
    if start is not None:
        windows.append((start, prev))
    return windows


def main() -> None:
    beta_values = np.arange(35.0, 89.01, 0.25)
    print()
    print(f"Corgi SLIP-RF self-stability at v~ = {V_TILDE} "
          f"({V_TILDE * np.sqrt(G * (HIP_TO_ARC + FOOT_RADIUS)):.3f} m/s)")
    print(f"beta swept {beta_values[0]}-{beta_values[-1]} deg in "
          f"{beta_values[1]-beta_values[0]} deg steps")
    print()

    best_overall = None
    for k_rel in (7.0, 10.0, 12.0, 15.0, 18.0, 22.0, 27.0):
        rows = sweep(k_rel, beta_values)
        if not rows:
            print(f"k_rel={k_rel:5.1f}: no fixed points found")
            continue
        slopes = [abs(r["slope"]) for r in rows]
        best = rows[int(np.argmin(slopes))]
        windows = stable_windows(rows)
        wtxt = ", ".join(f"{a:.2f}-{b:.2f}" for a, b in windows) if windows else "none"
        print(f"k_rel={k_rel:5.1f} k={params(k_rel).k:7.0f} N/m | "
              f"{len(rows):3d} fixed points | min|slope|={min(slopes):.4f} "
              f"at beta={best['beta_deg']:.2f} alpha={best['alpha_deg']:.2f}")
        print(f"{'':14s}   stable windows (|slope|<1): {wtxt}")
        if windows:
            in_window = [r for r in rows if abs(r["slope"]) < 1.0]
            tau = [r["tau"] for r in in_window]
            print(f"{'':14s}   within window: tau {min(tau):.1f}-{max(tau):.1f} N.m "
                  f"({100*max(tau)/MOTOR_TORQUE_LIMIT:.0f}% of limit), "
                  f"duty {min(r['duty'] for r in in_window):.3f}-"
                  f"{max(r['duty'] for r in in_window):.3f}")
        if best_overall is None or abs(best["slope"]) < abs(best_overall[1]["slope"]):
            best_overall = (k_rel, best)

    print()
    if best_overall:
        k_rel, b = best_overall
        verdict = "STABLE" if abs(b["slope"]) < 1 else "unstable"
        print(f"Best-conditioned fixed point overall: k_rel={k_rel}, "
              f"beta={b['beta_deg']:.2f} deg, alpha={b['alpha_deg']:.2f} deg, "
              f"slope={b['slope']:+.4f} ({verdict})")
        print(f"  peak GRF {b['grf_bw']:.2f} BW, tau {b['tau']:.1f} N.m "
              f"({100*b['tau']/MOTOR_TORQUE_LIMIT:.0f}% of {MOTOR_TORQUE_LIMIT} N.m), "
              f"duty {b['duty']:.3f}")
    print()


if __name__ == "__main__":
    main()
