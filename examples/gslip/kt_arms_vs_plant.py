"""Does the model's map slope track the plant's touchdown scatter, arm for arm?

A POST-HOC CONSISTENCY CHECK, NOT A REGISTERED TEST. Say so plainly:

  * The PLANT numbers were seen first. `touchdown_phase.py` gained a per-run
    within-leg sd(beta_TD) on 2026-08-22, and applying it to the banked
    k_tangential sweep (~/corgi_runs/kt_sweep, n = 2 per arm) showed touchdown
    scatter falling monotonically with k_tangential.
  * The MODEL's direction was already known from two points -- S149 measured
    the leg-frame regime (k_t 600) at +1.154 and the flight regime (k_t 7150
    equivalent) at +0.582 at v~0.70, so "stiffer is more stable" was in hand
    before this ran.
  * n = 4 arms and n = 2 runs per arm. A 4-point rank correlation gates
    nothing. This is corroboration between two independent lines of evidence,
    and it is reported as such.

WHAT MAKES IT WORTH RUNNING ANYWAY

The two lines are genuinely independent -- one is a Poincare map slope in a
reduced-order model, the other is the standard deviation of a measured leg
angle at contact in Webots -- and they are being compared at THE SAME FOUR
COMMANDED GAINS rather than in the abstract. If the model's slope and the
plant's scatter disagree in direction at matched arms, the model is not
describing this plant and the retune argument collapses. That is a real risk
being taken, even though it is not a registered gate.

THE CONVERSION

k_c = k_tangential * lever^2, lever = 0.2852 m (the controller's rigid J_fb --
S149 sec 1). b_tangential was NOT swept and stays at its shipped 30, so d_c is
constant across arms at 2.44.

Run:
    uv run python examples/gslip/kt_arms_vs_plant.py
"""

from __future__ import annotations

import numpy as np

from clock_torque_regime_switch import (
    B_TANGENTIAL, EROSION, MOTOR_SPLIT, STALL, baseline, params,
    solve_held_beta, to_model,
)

# The four arms of the banked sweep.
ARMS = (150.0, 300.0, 600.0, 1200.0)

# MEASURED, hardcoded from analyser output, never re-derived here (vault
# CLAUDE.md). touchdown_phase.py within-leg df-pooled sd(beta_TD), rad,
# ~/corgi_runs/kt_sweep, 2026-08-22. Stride counts in the second tuple.
PLANT_SD = {
    150.0: (0.13248, 0.20341),
    300.0: (0.11544, 0.18096),
    600.0: (0.08233, 0.08007),
    1200.0: (0.05314, 0.05630),
}
PLANT_N = {150.0: (348, 226), 300.0: (366, 302),
           600.0: (384, 390), 1200.0: (440, 443)}

V_TILDES = (0.34, 0.70)


def spearman(a, b):
    """Rank correlation, n is tiny so do it by hand rather than pull in scipy."""
    def rank(x):
        order = sorted(range(len(x)), key=lambda i: x[i])
        r = [0.0] * len(x)
        for pos, i in enumerate(order):
            r[i] = float(pos)
        return r
    ra, rb = rank(a), rank(b)
    ma, mb = np.mean(ra), np.mean(rb)
    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    den = np.sqrt(sum((x - ma) ** 2 for x in ra)
                  * sum((y - mb) ** 2 for y in rb))
    return float(num / den) if den else float("nan")


def main() -> None:
    print(__doc__.split("Run:")[0].rstrip())
    print()
    p = params()

    for vt in V_TILDES:
        v, fp0 = baseline(p, vt)
        print("=" * 88)
        print("v~ %.2f   baseline beta* %.2f deg (HELD), conservative slope %+.4f"
              % (vt, np.rad2deg(fp0.beta), fp0.slope))
        print("=" * 88)
        print("  %9s %8s %8s %10s %10s %11s %14s"
              % ("k_tang", "k_c", "d_c", "slope", "|slope|", "tau/motor", "plant sd(b_TD)"))
        slopes, sds, kept = [], [], []
        for kt in ARMS:
            k_c, d_c = to_model(kt, B_TANGENTIAL)
            r = solve_held_beta(p, v, fp0, (k_c, d_c), (k_c, d_c), np.inf)
            sd_runs = PLANT_SD[kt]
            sd_med = float(np.median(sd_runs))
            if r is None:
                print("  %9.0f %8.2f %8.2f %10s %10s %11s %14.5f  no admitted root"
                      % (kt, k_c, d_c, "--", "--", "--", sd_med))
                continue
            print("  %9.0f %8.2f %8.2f %+10.4f %10.4f %11.1f %14.5f  %s"
                  % (kt, k_c, d_c, r["slope"], abs(r["slope"]), r["tau_motor"],
                     sd_med, "stable" if abs(r["slope"]) < 1.0 else "UNSTABLE"))
            slopes.append(abs(r["slope"])); sds.append(sd_med); kept.append(kt)
        print()
        if len(kept) >= 3:
            rho = spearman(slopes, sds)
            print("  Spearman( model |slope| , plant sd(beta_TD) ) over %d arms = %+.3f"
                  % (len(kept), rho))
            print("  monotone in the model : %s" % _mono(slopes))
            print("  monotone in the plant : %s" % _mono(sds))
            print("  => %s" % ("SAME DIRECTION -- the model's stability ordering and the"
                               " plant's touchdown repeatability agree arm for arm."
                               if rho > 0.5 else
                               "DISAGREE -- the model does not order these arms as the"
                               " plant does. The retune argument does not survive this."))
        print()

    print("=" * 88)
    print("n = 4 arms, n = 2 runs per arm, plant data seen BEFORE the model ran.")
    print("Corroboration, not a gate. tau/motor applies the x%.1f leg-axis split and"
          % MOTOR_SPLIT)
    print("x%.2f erosion; per-motor stall is %.1f N.m." % (EROSION, STALL))
    print("=" * 88)


def _mono(xs):
    up = all(b >= a for a, b in zip(xs, xs[1:]))
    dn = all(b <= a for a, b in zip(xs, xs[1:]))
    return "yes, falling" if dn else ("yes, rising" if up else "NO")


if __name__ == "__main__":
    main()
