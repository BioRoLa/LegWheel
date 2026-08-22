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
from legwheel.planners import gslip_to_corgi as g2c

MASS, G = 30.0, 9.81
V_TILDE = 1.2  # chosen target: ~1.8 m/s
N_LEGS = 4
MOTOR_TORQUE_LIMIT = 35.0

# THE POSE. This sweep used to hardcode HIP_TO_ARC = 0.0850, which is the
# theta ~ 65.89 deg crouch that hits the paper's r~ = 0.6303 optimum -- and
# which stance_height_tradeoff.py REJECTED, because on this leg that optimum is
# a 0.230 m crouch, too low to clear the support block and needlessly harsh on
# the hardware. Every other script (export_pronk_csv, pronk_operating_point,
# stage2a_turning_envelope) derives the pose from the real linkage at
# theta = 100 deg. So this file's conclusions were drawn at a pose the project
# does not run, which the log flagged and nobody fixed.
#
# The default is now theta = 100 deg, the pose of record. `--pose old`
# reproduces the historical numbers for comparison; it is not the robot.
NOMINAL_THETA_DEG = 100.0
LEGACY_HIP_TO_ARC = 0.0850   # theta ~ 65.89 deg; the rejected crouch

_LEG_MAP = g2c.LegLengthMap()
FOOT_RADIUS = float(_LEG_MAP.leg.foot_radius)
HIP_TO_ARC = float(_LEG_MAP.length(np.deg2rad(NOMINAL_THETA_DEG)))
POSE_LABEL = f"theta={NOMINAL_THETA_DEG:.0f}deg (pose of record)"

# dl/dtheta at the pose, for the torque proxy. This was a hardcoded 0.10324,
# which is a POSE-DEPENDENT quantity -- carrying it across a pose change would
# have silently mispriced every torque in the table, so it is measured here.
def _dl_dtheta(theta_deg: float, h: float = 1e-4) -> float:
    t = np.deg2rad(theta_deg)
    return float((_LEG_MAP.length(t + h) - _LEG_MAP.length(t - h)) / (2 * h))

DL_DTHETA = _dl_dtheta(NOMINAL_THETA_DEG)


def set_pose(theta_deg: float | None = None, hip_to_arc: float | None = None) -> None:
    """Select the stance pose. Give a theta, or a raw hip-to-arc length."""
    global HIP_TO_ARC, DL_DTHETA, POSE_LABEL
    if (theta_deg is None) == (hip_to_arc is None):
        raise ValueError("give exactly one of theta_deg, hip_to_arc")
    if theta_deg is not None:
        HIP_TO_ARC = float(_LEG_MAP.length(np.deg2rad(theta_deg)))
        DL_DTHETA = _dl_dtheta(theta_deg)
        POSE_LABEL = f"theta={theta_deg:.2f}deg"
    else:
        HIP_TO_ARC = float(hip_to_arc)
        # Invert the length map to price the torque at the matching theta.
        lo, hi = 20.0, 160.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if _LEG_MAP.length(np.deg2rad(mid)) < HIP_TO_ARC:
                lo = mid
            else:
                hi = mid
        DL_DTHETA = _dl_dtheta(0.5 * (lo + hi))
        POSE_LABEL = (f"hip_to_arc={HIP_TO_ARC:.4f}m (theta~{0.5*(lo+hi):.2f}deg)"
                      + (" -- LEGACY, the rejected crouch"
                         if abs(HIP_TO_ARC - LEGACY_HIP_TO_ARC) < 1e-9 else ""))


def pose_banner() -> str:
    p = params(18.0)
    return (f"POSE {POSE_LABEL}   hip_to_arc {HIP_TO_ARC:.4f} m   l0 {p.l0:.4f} m   "
            f"r~ {FOOT_RADIUS / p.l0:.4f}   v_scale {np.sqrt(G * p.l0):.4f} m/s   "
            f"dl/dtheta {DL_DTHETA:.5f} m/rad")


def set_speed(v_tilde: float) -> None:
    """Override the dimensionless target speed for a sweep."""
    global V_TILDE
    V_TILDE = v_tilde


def params(k_rel: float) -> slip_rf.SlipRfParams:
    return slip_rf.SlipRfParams(
        m=MASS, l0=HIP_TO_ARC + FOOT_RADIUS,
        k=k_rel * MASS * G / HIP_TO_ARC, r=FOOT_RADIUS,
    )


def residual(p, v, beta):
    def f(alpha: float) -> float:
        return slip_rf.next_touchdown(p, v, alpha, beta)[1] - alpha

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
                return float(brentq(f, lo, hi, xtol=1e-10, rtol=1e-10))
            except (GSlipFailure, ValueError):
                continue
    return None


def slope_at(p, v, beta, alpha, h=np.deg2rad(0.02)):
    """dP/dalpha by central difference, in a step large enough to beat solver noise."""
    try:
        hi = slip_rf.next_touchdown(p, v, alpha + h, beta)[1]
        lo = slip_rf.next_touchdown(p, v, alpha - h, beta)[1]
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
            return float(brentq(f, a0, a1, xtol=1e-10, rtol=1e-10))
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


def main(k_rel_values=(7.0, 10.0, 12.0, 15.0, 18.0, 22.0, 27.0),
         beta_step: float = 0.25) -> None:
    beta_values = np.arange(35.0, 89.01, beta_step)
    print()
    print(pose_banner())
    print(f"Corgi SLIP-RF self-stability at v~ = {V_TILDE} "
          f"({V_TILDE * np.sqrt(G * (HIP_TO_ARC + FOOT_RADIUS)):.3f} m/s)")
    print(f"beta swept {beta_values[0]}-{beta_values[-1]} deg in "
          f"{beta_values[1]-beta_values[0]} deg steps")
    print()

    best_overall = None
    for k_rel in k_rel_values:
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


def speed_scan(v_tildes, k_rels, beta_step: float = 0.5) -> None:
    """min|dP/dalpha| over the (speed, stiffness) plane -- is it ever stable?"""
    beta_values = np.arange(35.0, 89.01, beta_step)
    print()
    print(pose_banner())
    print("min |dP/dalpha| over (speed, stiffness); values < 1 are self-stable")
    print(f"  speeds scanned: v~ {min(v_tildes)}-{max(v_tildes)}. 'No stable point'"
          f" means none AT THESE SPEEDS -- the default used to stop at 1.8 and the")
    print("  first self-stable gait sits above it, so state the ceiling with the result.")
    header = f"{'v~':>6} " + "".join(f"{f'k_rel={k:g}':>12}" for k in k_rels)
    print(header)
    print("-" * len(header))
    for v_t in v_tildes:
        set_speed(v_t)
        row = f"{v_t:6.2f} "
        for k_rel in k_rels:
            rows = sweep(k_rel, beta_values)
            if not rows:
                row += f"{'--':>12}"
                continue
            best = min(abs(r["slope"]) for r in rows)
            mark = "*" if best < 1.0 else " "
            row += f"{best:11.4f}{mark}"
        print(row, flush=True)
    print()


if __name__ == "__main__":
    import sys

    args = sys.argv[1:]
    # --pose old  reproduces the historical (rejected-crouch) numbers;
    # --pose <deg> picks any stance theta. Default is the pose of record.
    if len(args) >= 2 and args[0] == "--pose":
        if args[1] == "old":
            set_pose(hip_to_arc=LEGACY_HIP_TO_ARC)
        else:
            set_pose(theta_deg=float(args[1]))
        args = args[2:]

    if args and args[0] == "--scan":
        speed_scan(
            v_tildes=[float(a) for a in args[1:]]
            # Reaches past the torque envelope on purpose: the question is
            # whether a stable window exists at all, and where it sits
            # relative to the fence, not whether it is affordable.
            or (0.5, 0.7, 0.9, 1.1, 1.2, 1.4, 1.6, 1.8, 2.0, 2.25, 2.5),
            k_rels=(7.0, 15.0, 27.0, 45.0),
        )
    elif args and args[0] == "--v":
        set_speed(float(args[1]))
        ks = [float(a) for a in args[2:]] or None
        main(k_rel_values=ks) if ks else main()
    else:
        ks = [float(a) for a in args] or None
        main(k_rel_values=ks) if ks else main()
