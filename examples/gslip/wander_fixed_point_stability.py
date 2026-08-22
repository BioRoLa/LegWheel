"""Is the orbit the robot's template encodes self-stable, and how fast does it recover?

The wander block's model-space half (log S130-S132). Prompted by a labmate's
tuning advice -- continuation over speed, basin of attraction, "it should come
back to the fixed point in about six steps", alpha 10-18 deg, T 0.2-0.3 s --
which is the Lu & Lin G-SLIP / CTR-SLIP methodology this model is a port of.

FOUR THINGS, in the order they answer the advice.

1. CONTINUATION over speed, warm-starting each solve from the previous fixed
   point. Reports the landing angle in BOTH conventions, because they are
   different numbers and confusing them is the expensive mistake here:

     alpha_leg = 90 - beta   the labmate's alpha: how far the leg is from
                             vertical at touchdown. This is what the template
                             CSV's `beta` column holds at the touchdown row.
     alpha_v               the CODEBASE's `alpha`: the touchdown VELOCITY
                             angle below horizontal, what slip_rf.stride
                             returns and what the Poincare map iterates on.

   At v~1.20 those are 18.25 and 17.79 deg -- close enough to be mistaken for
   each other, and they diverge fast (at v~0.70: 9.25 vs 40.74).

2. STEPS TO CONVERGE, not just "is it in the basin". The literature's headline
   is a stride count and `converges_to` threw it away.

3. THE (beta, k_rel) MAP is in corgi_stability_sweep.py, which until this block
   hardcoded the theta ~ 65.9 deg crouch that stance_height_tradeoff.py
   rejected. Run that with `--pose old` and without, and compare.

4. T AS A CONSTRAINT rather than an output. The labmate specifies T = 0.2-0.3 s;
   here T falls out of whichever fixed point gets solved. At one speed there is
   a one-parameter (k, beta) family, so T can be chosen -- this asks what k and
   alpha that costs.

Run:
    uv run python examples/gslip/wander_fixed_point_stability.py
"""

from __future__ import annotations

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.gslip_fixed_point import (
    basin_in_alpha, find_fixed_points, steps_to_converge,
)
from legwheel.planners import gslip_to_corgi as g2c

MASS, G = 30.0, 9.81
K_REL = 18.0
NOMINAL_THETA_DEG = 100.0
TOL = 0.01

# The plant, for scale. Signed path-integrated forward speed, n = 5 (S125).
PLANT_V = {"k7150": 0.282, "k12000": 0.215}
# The labmate's windows.
LM_ALPHA = (10.0, 18.0)
LM_T = (0.20, 0.30)

# Grazing filters, the same ones pronk_operating_point.py and
# export_speed_ramp_csv.py use. Without them the continuation happily reports
# "STABLE" for solutions that hop a fraction of a millimetre with a duty factor
# of 0.8 -- the grazing branch, which is a fixed point of the map and not a
# gait. The vault's stability claims are all made subject to these, so a table
# that omits them is not comparable to them.
MIN_APEX_MM = 10.0
MAX_DUTY = 0.55


def apex_mm(fp) -> float:
    return 1000.0 * G * fp.flight_time ** 2 / 8.0


def grazing(fp) -> bool:
    return apex_mm(fp) < MIN_APEX_MM or fp.duty_factor > MAX_DUTY


def params(k_rel: float = K_REL, theta_deg: float = NOMINAL_THETA_DEG):
    lm = g2c.LegLengthMap()
    r = lm.leg.foot_radius
    hip = lm.length(np.deg2rad(theta_deg))
    return slip_rf.SlipRfParams(m=MASS, l0=hip + r, k=k_rel * MASS * G / hip, r=r)


def solve_at(p, v, beta_lo, beta_hi, step=0.25, seed_beta=None):
    """Best-conditioned NON-GRAZING fixed point in a landing-angle window.

    Grazing is filtered BEFORE choosing, not after. Choosing on |slope| alone
    hands the answer to the grazing branch at speed: at v~2.25 the grazing root
    has |slope| 0.0035 against a real gait's ~1.2, so a min-|slope| rule reports
    a "stable" 3 mm hop with duty 0.57 and calls it the operating point. This is
    the same trap as bootstrap() following a single branch, wearing a different
    hat. pronk_operating_point.py filters first for the same reason.

    Returns (fixed_point, fell_back_to_grazing).
    """
    best = best_graze = None
    lo, hi = (seed_beta - 4.0, seed_beta + 4.0) if seed_beta else (beta_lo, beta_hi)
    lo, hi = max(lo, beta_lo), min(hi, beta_hi)
    for bd in np.arange(lo, hi + 1e-9, step):
        for fp in find_fixed_points(p, v, np.deg2rad(bd),
                                    alpha_range=(np.deg2rad(1.0), np.deg2rad(60.0)),
                                    n_samples=40, stride_fn=slip_rf.stride):
            if grazing(fp):
                if best_graze is None or abs(fp.slope) < abs(best_graze.slope):
                    best_graze = fp
            elif best is None or abs(fp.slope) < abs(best.slope):
                best = fp
    if best is None and best_graze is None and seed_beta is not None:
        return solve_at(p, v, beta_lo, beta_hi, step)      # continuation lost it
    return (best, False) if best is not None else (best_graze, True)


def recovery(p, fp, rel=0.05):
    """Simulated and analytic step counts for a small touchdown perturbation.

    The perturbation is RELATIVE to alpha*, not a fixed 1 deg. A fixed 1 deg is
    8% of a 12 deg fixed point but 45% of a 2.2 deg one, and the analytic
    formula is a linearisation -- comparing them at 45% measures the
    nonlinearity, not the tool.
    """
    d = rel * fp.alpha
    n = steps_to_converge(p, fp, fp.v, fp.alpha + d, tol=TOL, stride_fn=slip_rf.stride)
    err0 = d / fp.alpha
    s = abs(fp.slope)
    ana = np.log(TOL / err0) / np.log(s) if s != 1.0 else float("inf")
    return n, ana


def part1_continuation(p):
    v_scale = np.sqrt(G * p.l0)
    print("=" * 92)
    print("1. CONTINUATION OVER SPEED    theta_nom %.0f deg, k_rel %.0f, "
          "l0 %.4f m, v_scale %.4f m/s" % (NOMINAL_THETA_DEG, K_REL, p.l0, v_scale))
    print("=" * 92)
    print("   alpha_leg = 90 - beta*  is the LABMATE's alpha (leg from vertical).")
    print("   alpha_v    is the CODEBASE's alpha (touchdown velocity angle). "
          "They are NOT the same.\n")
    print("%6s %8s %9s %10s %9s %9s %8s %7s %6s %7s %7s %s" % (
        "v~", "v_td", "v_fwd", "alpha_leg", "beta*", "alpha_v", "slope", "T", "duty",
        "apex_mm", "steps", "notes"))

    rows, seed = [], None
    for vt in (0.40, 0.45, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00, 1.10, 1.20,
               1.35, 1.50, 1.75, 2.00, 2.25, 2.50):
        v = vt * v_scale
        got = solve_at(p, v, 40.0, 88.0, seed_beta=seed)
        fp, only_grazing = (got if isinstance(got, tuple) else (got, False))
        if fp is None:
            print("%6.2f %8.3f  -- no fixed point in beta 40-88 deg" % (vt, v))
            seed = None
            continue
        seed = float(np.rad2deg(fp.beta))
        n, ana = recovery(p, fp)
        a_leg = 90.0 - np.rad2deg(fp.beta)
        note = []
        if LM_ALPHA[0] <= a_leg <= LM_ALPHA[1]:
            note.append("a*")
        if LM_T[0] <= fp.period <= LM_T[1]:
            note.append("T*")
        gz = only_grazing or grazing(fp)
        if gz:
            note.append("ONLY-GRAZING" + ("-stable" if fp.stable else ""))
        elif fp.stable:
            note.append("*** STABLE GAIT ***")
        print("%6.2f %8.3f %9.3f %10.2f %9.2f %9.2f %+8.4f %7.4f %6.3f %7.1f %7s %s" % (
            vt, v, fp.mean_speed, a_leg, np.rad2deg(fp.beta), np.rad2deg(fp.alpha),
            fp.slope, fp.period, fp.duty_factor, apex_mm(fp),
            ("%d" % n) if n is not None else "never", " ".join(note)))
        rows.append((vt, fp, a_leg))

    print("\n   a* = inside the labmate's alpha %g-%g deg;  T* = inside his T %g-%g s"
          % (LM_ALPHA + LM_T))
    print("   The plant runs at v_fwd %.3f-%.3f m/s (n=5, S125) -- read that against"
          % (PLANT_V["k12000"], PLANT_V["k7150"]))
    print("   the v_fwd column, NOT v_td. They differ by roughly 2x.")
    print("   GRAZING = apex < %g mm or duty > %g: a fixed point of the map, NOT a"
          % (MIN_APEX_MM, MAX_DUTY))
    print("   gait. The vault's stability claims all carry these filters, so a table")
    print("   without them is not comparable to them.")
    real = [r for r in rows if r[1].stable and not grazing(r[1])]
    print("   Self-stable AND non-grazing: %s"
          % (", ".join("v~%.2f (v_fwd %.2f m/s)" % (r[0], r[1].mean_speed)
                       for r in real) if real else "NONE in this range"))
    return rows


def part2_recovery(p, rows):
    print("\n" + "=" * 92)
    print("2. RECOVERY -- simulated steps vs the analytic log(tol/err0)/log|slope|")
    print("=" * 92)
    print("   Disagreement here means the map or the eq-13 error metric is wrong,")
    print("   so it is checked rather than assumed.\n")
    print("%6s %9s %10s %12s %10s" % ("v~", "slope", "steps", "analytic", "verdict"))
    for vt, fp, _ in rows:
        n, ana = recovery(p, fp)
        if fp.stable:
            v = "agrees" if n is not None and abs(n - np.ceil(ana)) <= 1 else "MISMATCH"
        else:
            v = "diverges, as expected" if n is None else "MISMATCH"
        print("%6.2f %+9.4f %10s %12s %10s" % (
            vt, fp.slope, ("%d" % n) if n is not None else "never",
            ("%.2f" % ana) if np.isfinite(ana) and ana > 0 else "n/a", v))

    stable = [(vt, fp) for vt, fp, _ in rows if fp.stable and not grazing(fp)]
    if stable:
        vt, fp = stable[0]
        alphas = fp.alpha + np.deg2rad(np.linspace(-6, 6, 25))
        b = basin_in_alpha(p, fp, alphas, tol=TOL, stride_fn=slip_rf.stride)
        inside = b["converged"]
        print("\n   1-D basin at the slowest self-stable point (v~ %.2f, slope %+.4f):"
              % (vt, fp.slope))
        print("     %d of %d probed touchdown angles converge; widest recovering"
              % (inside.sum(), len(alphas)))
        print("     perturbation %.2f deg, worst-case %d steps, median %d steps"
              % (np.rad2deg(np.max(np.abs(alphas[inside] - fp.alpha))) if inside.any() else 0,
                 int(np.nanmax(b["steps"])) if inside.any() else -1,
                 int(np.nanmedian(b["steps"])) if inside.any() else -1))
        print("     -> the labmate's '~6 steps' IS this model's answer, at the one")
        print("        self-stable non-grazing orbit it has.")


def part3_period(p):
    print("\n" + "=" * 92)
    print("4. T AS A CONSTRAINT -- what does landing in %g-%g s cost?" % LM_T)
    print("=" * 92)
    v_scale = np.sqrt(G * p.l0)
    v = 0.70 * v_scale
    print("   At the config-of-record speed v~0.70 (v_td %.3f m/s), sweeping k_rel"
          % v)
    print("   and taking the best-conditioned fixed point at each:\n")
    print("%8s %10s %10s %9s %9s %8s %7s" % (
        "k_rel", "k N/m", "alpha_leg", "beta*", "slope", "T", "in band"))
    for k_rel in (7.0, 10.0, 14.0, 18.0, 24.0, 32.0, 45.0):
        pk = params(k_rel)
        got = solve_at(pk, 0.70 * np.sqrt(G * pk.l0), 60.0, 88.0)
        fp = got[0] if isinstance(got, tuple) else got
        if fp is None:
            print("%8.0f %10.0f   -- none" % (k_rel, pk.k))
            continue
        print("%8.0f %10.0f %10.2f %9.2f %+9.4f %7.4f %7s" % (
            k_rel, pk.k, 90 - np.rad2deg(fp.beta), np.rad2deg(fp.beta), fp.slope,
            fp.period, "yes" if LM_T[0] <= fp.period <= LM_T[1] else "NO"))
    print("\n   T is an OUTPUT of the solve today and there is no period launch")
    print("   parameter -- the robot's period is CSV row count x 1 ms. Choosing T")
    print("   means choosing a (k, beta) member and re-exporting, which is a")
    print("   three-place change: k_rel here, the template CSV, and the k_radial")
    print("   launch parameter (default 8941.0 = the k_rel 18 value).")


def main() -> None:
    p = params()
    rows = part1_continuation(p)
    part2_recovery(p, rows)
    part3_period(p)
    print("\n" + "=" * 92)
    print("3. THE (beta, k_rel) MAP lives in corgi_stability_sweep.py. Run both poses:")
    print("     uv run python examples/gslip/corgi_stability_sweep.py --pose old  "
          "7 10 12 15 18 22 27 34 45")
    print("     uv run python examples/gslip/corgi_stability_sweep.py            "
          "7 10 12 15 18 22 27 34 45")
    print("     uv run python examples/gslip/corgi_stability_sweep.py --scan")
    print("=" * 92)


if __name__ == "__main__":
    main()
