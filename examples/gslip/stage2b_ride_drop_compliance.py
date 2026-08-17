"""Can a single per-side compliance close the ride-drop gap? A falsifiable fit.

Section 47 closed Module 2's roll validation at 0.99 / 1.00 / 1.01 and left
ONE observable open: measured ride drops (6.4 / 8.9 / 13.3 mm) exceed the
rigid prediction (-0.1 / 4.1 / 10.3), ratios 0.02 / 0.46 / 0.77, blamed on
kp-90 position loops sagging under shifted load. Its stated options: add a
per-leg compliance to the statics, or re-run the rig at kp 500. The rig needs
the sim (other session's); this is the compliance half, run on the existing
dumps, as hypothesis testing rather than curve fitting:

1. RIGID BASELINE re-derived from the dumps (must reproduce section 47 or the
   pipeline is broken, not the physics).
2. IMPLIED STIFFNESS per run: the servo_k that closes THAT run's drop alone.
   If the mechanism is one linear compliance, three runs imply one number.
3. JOINT FIT: one shared servo_k across all runs; report drop AND roll ratios
   at the optimum. Roll is the validated observable -- a fit that buys drop
   by wrecking roll is rejected regardless of its residual.

Expectation, stated before the run (register the prediction): a symmetric
linear compliance converts an antisymmetric load shift into MORE ROLL, not
net drop -- the excess drop also SHRINKS with lambda (6.5 / 4.8 / 3.0 mm)
while any load-shift effect grows. Prediction: no single servo_k closes the
three ratios jointly. If that holds, the negative result is the deliverable
and the kp-500 rig re-run is the remaining path (needs Webots -- coordinate).

Run:
    uv run python examples/gslip/stage2b_ride_drop_compliance.py \
        [dump:lam ...]     (default: the three section 47 dumps in ~/camber_dumps)
"""

from __future__ import annotations

import os
import sys

import numpy as np
from scipy.optimize import brentq

from stage2b_coronal_statics import solve_pose_asym
from stage2b_lean_rig_analysis import analyse

DEFAULT_DUMPS = [
    (os.path.expanduser(f"~/camber_dumps/camber_lean{lam}_pl.npz"), float(lam))
    for lam in (10, 20, 30)
]
K_LO, K_HI = 1e3, 1e8          # N/m bracket for the implied-stiffness search


def drop_pred_mm(left_deg: float, right_deg: float,
                 servo_k: float | None) -> tuple[float, float]:
    """(drop_mm, roll_deg) predicted for per-side achieved leans."""
    z0, _ = solve_pose_asym(0.0, 0.0, servo_k=servo_k)
    z, rho = solve_pose_asym(np.deg2rad(left_deg), np.deg2rad(right_deg),
                             servo_k=servo_k)
    return (z0 - z) * 1e3, np.rad2deg(rho)


def main(argv) -> None:
    print(__doc__.split("Run:")[0].rstrip())
    print()
    if argv:
        runs = [(a.rsplit(":", 1)[0], float(a.rsplit(":", 1)[1]))
                for a in argv]
    else:
        runs = DEFAULT_DUMPS
    rows = []
    for path, lam in runs:
        if not os.path.exists(path):
            raise SystemExit(f"dump not found: {path}")
        rows.append(analyse(path, lam))

    # -- 1: rigid baseline --------------------------------------------------
    print("=== RIGID BASELINE (must reproduce section 47) ===")
    for r in rows:
        d, roll = drop_pred_mm(r["left"], r["right"], None)
        print(f"  lam {r['lam_cmd']:4.0f}d: roll ratio "
              f"{abs(roll / r['roll_meas']):.2f}, drop pred {d:6.2f} vs meas "
              f"{r['drop_meas']:6.2f} mm (ratio "
              f"{d / r['drop_meas']:5.2f})")

    # -- 2: implied stiffness per run ---------------------------------------
    print()
    print("=== IMPLIED servo_k PER RUN (one mechanism -> one number) ===")
    implied = []
    for r in rows:
        def gap(k, r=r):
            d, _ = drop_pred_mm(r["left"], r["right"], k)
            return d - r["drop_meas"]
        lo, hi = gap(K_LO), gap(K_HI)
        if lo * hi > 0:
            print(f"  lam {r['lam_cmd']:4.0f}d: NO servo_k in "
                  f"[{K_LO:.0e}, {K_HI:.0e}] closes this run "
                  f"(gap {lo:+.2f} mm soft, {hi:+.2f} mm stiff)")
            implied.append(float("nan"))
            continue
        k = brentq(gap, K_LO, K_HI, xtol=1.0)
        implied.append(k)
        _, roll = drop_pred_mm(r["left"], r["right"], k)
        print(f"  lam {r['lam_cmd']:4.0f}d: servo_k {k:10.0f} N/m "
              f"(roll ratio at that k: {abs(roll / r['roll_meas']):.2f})")

    # -- 3: joint fit -------------------------------------------------------
    print()
    print("=== JOINT FIT: one shared servo_k ===")
    ks = np.logspace(np.log10(K_LO), np.log10(K_HI), 121)
    best_k, best_sse = None, np.inf
    for k in ks:
        sse = 0.0
        for r in rows:
            d, _ = drop_pred_mm(r["left"], r["right"], k)
            sse += (d - r["drop_meas"]) ** 2
        if sse < best_sse:
            best_k, best_sse = k, sse
    print(f"  best shared servo_k {best_k:10.0f} N/m "
          f"(rms {np.sqrt(best_sse / len(rows)):.2f} mm):")
    ok = True
    for r in rows:
        d, roll = drop_pred_mm(r["left"], r["right"], best_k)
        rr = abs(roll / r["roll_meas"])
        dr = d / r["drop_meas"]
        ok &= (0.95 < rr < 1.05) and (0.8 < dr < 1.2)
        print(f"    lam {r['lam_cmd']:4.0f}d: drop ratio {dr:5.2f}, "
              f"roll ratio {rr:5.2f}")
    print()
    if ok:
        print("VERDICT: a single linear per-side compliance closes drop "
              "without moving roll -- adopt servo_k above.")
    else:
        print("VERDICT: no single linear per-side compliance closes the drop "
              "ratios while holding roll -- as registered above. The kp-500 "
              "rig re-run stays the remaining path (needs the sim).")


if __name__ == "__main__":
    main(sys.argv[1:])
