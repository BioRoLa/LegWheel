"""Export a CAMBERED gait template -- Stage 3 task 3's feedforward lean.

Log S193. Offline, no simulator.

WHAT THIS IS, AND WHAT IT DELIBERATELY IS NOT.

Stage 3 task 3 is "lean command -- feedforward from fixed point first". Reading
the controller settles what that can and cannot mean:

  * `row.gamma` IS applied -- gslip_pronk.cpp:2535 sums it with the open-loop
    Ackermann term and the roll/yaw feedback.
  * BUT all four legs index the SAME template array (2498), so a scalar gamma
    column is COMMON-MODE: uniform camber on all four legs. S33 measured that
    at 0.02% steering authority, and camber_apex_geometry says uniform camber
    cannot roll drill-free on four contacts. A gamma column STRUCTURALLY
    cannot express an Ackermann in/out pair.
  * `gamma_acker_in/out/dir` already IS the feedforward lean channel, built to
    be fed ackermann_pair() numbers, with "NO geometry lives here" in its own
    header (2233-2260).

So this does NOT write a gamma column -- it leaves it zero and the lean is
delivered through gamma_acker_* exactly as now. What the template contributes
is the THETA and BETA trajectory re-solved AT that lean, which is the part
currently taken from the uncambered v070 while the robot runs cambered.

The route is Stage 2a's rescaling reduction (cambered_params: g -> g/cos lam,
r -> r_eff(lam)), which stage2a_turning_envelope.py:213-222 already chains
end-to-end. NOT the 3D map: apex_map discards its solve_ivp solution, has no
trace option, and carries no joint coordinates, so a trajectory emitter there
would need a whole new hip->foot->(theta,beta,gamma) layer.

⚠ EXPECT A SMALL EFFECT, and that is the honest framing: 1/cos(10 deg) = 1.015.
This is a 1.5% rescaling, not a new gait. It is worth running because nobody
has checked whether the plant can see it, not because it is large.

Usage:
    export_cambered_template.py --check          # lam=0 must reproduce v070
    export_cambered_template.py --lam-deg 10 --v-tilde 0.70 --suffix _v070_lam10
"""
from __future__ import annotations

import argparse
from functools import partial
from pathlib import Path

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.models.slip_rf_cambered import cambered_params, cambered_stride
from legwheel.planners import gslip_template as tpl
from legwheel.planners import gslip_to_corgi as g2c

MASS, G, K_REL, N_LEGS = 30.0, 9.81, 18.0, 4
NOMINAL_THETA_DEG = 100.0
SHIPPED_V070 = Path(__file__).resolve().parents[2] / "output_data"


def base_params():
    leg_map = g2c.LegLengthMap()
    foot_radius = leg_map.leg.foot_radius
    hip_to_arc = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))
    p = slip_rf.SlipRfParams(m=MASS, l0=hip_to_arc + foot_radius,
                             k=K_REL * MASS * G / hip_to_arc, r=foot_radius)
    return p, leg_map


def solve(p_cell, v, stride_fn, beta_lo, beta_hi, step=0.25):
    best = None
    for beta_deg in np.arange(beta_lo, beta_hi + 1e-9, step):
        for fp in find_fixed_points(
                p_cell, v, np.deg2rad(beta_deg),
                alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
                n_samples=30, stride_fn=stride_fn):
            if best is None or abs(fp.slope) < abs(best.slope):
                best = fp
    return best


def build(lam_deg, v_tilde, beta_range):
    p, _ = base_params()
    v = v_tilde * np.sqrt(G * p.l0)
    lam = np.deg2rad(lam_deg)
    if abs(lam) < 1e-12:
        p_cell, stride_fn = p, slip_rf.stride
    else:
        p_cell = cambered_params(p, lam)
        stride_fn = partial(cambered_stride, lam=lam)
    fp = solve(p_cell, v, stride_fn, *beta_range)
    if fp is None:
        raise SystemExit(f"no fixed point at lam={lam_deg} v~={v_tilde} in "
                         f"beta {beta_range}")
    template = tpl.build_template(p_cell, v, fp.alpha, fp.beta)
    traj = g2c.map_template(
        template, n=g2c.samples_for_rate(template.period, rate_hz=1000.0))
    return p_cell, fp, template, traj


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--lam-deg", type=float, default=10.0)
    ap.add_argument("--v-tilde", type=float, default=0.70)
    ap.add_argument("--beta-range", type=float, nargs=2, default=(70.0, 85.0))
    ap.add_argument("--suffix", default="")
    ap.add_argument("--out")
    ap.add_argument("--check", action="store_true",
                    help="lam=0 must reproduce the shipped v070 exactly")
    a = ap.parse_args()

    if a.check:
        # KNOWN ANSWER. At lam = 0 the cambered reduction is the identity, so
        # this must land on the shipped template's own fixed point: beta* =
        # 80.75 deg, period 0.265233 s, 266 rows. If it does not, nothing
        # exported at any other lambda means anything.
        print("lam = 0 identity check against the shipped v070\n")
        _, fp0, t0, tr0 = build(0.0, a.v_tilde, tuple(a.beta_range))
        b0 = np.rad2deg(fp0.beta)
        n0 = g2c.samples_for_rate(t0.period, rate_hz=1000.0)
        ok = True
        for name, got, want, tol in (
                ("beta*  (deg)", b0, 80.75, 0.02),
                ("period (s)", t0.period, 0.265233, 2e-5),
                ("rows", float(n0), 266.0, 0.5),
                ("row0 beta (rad)", float(tr0.beta[0]), -0.161443, 2e-5),
                ("row0 theta (rad)", float(tr0.theta[0]), 1.745329, 2e-5)):
            good = abs(got - want) <= tol
            ok = ok and good
            print(f"  {'ok ' if good else 'FAIL'} {name:18} {got:12.6f} "
                  f"(want {want:.6f})")
        print(f"\n  CHECK {'PASS' if ok else 'FAIL'}")
        raise SystemExit(0 if ok else 1)

    p_cell, fp, template, traj = build(a.lam_deg, a.v_tilde,
                                       tuple(a.beta_range))
    # The lam = 0 reference, so the effect size is reported rather than assumed.
    _, fp0, t0, tr0 = build(0.0, a.v_tilde, tuple(a.beta_range))

    print(f"cambered template, lambda = {a.lam_deg} deg, v~ = {a.v_tilde}")
    print(f"  1/cos(lam)      {1.0/np.cos(np.deg2rad(a.lam_deg)):.4f}"
          f"   <- the whole size of the rescaling")
    print()
    print(f"  {'':16} {'lam=0':>12} {'cambered':>12} {'delta':>12}")
    rows = [
        ("beta* (deg)", np.rad2deg(fp0.beta), np.rad2deg(fp.beta)),
        ("alpha* (deg)", np.rad2deg(fp0.alpha), np.rad2deg(fp.alpha)),
        ("slope", fp0.slope, fp.slope),
        ("period (s)", t0.period, template.period),
        ("row0 theta (rad)", float(tr0.theta[0]), float(traj.theta[0])),
        ("row0 beta (rad)", float(tr0.beta[0]), float(traj.beta[0])),
        ("theta min (rad)", float(np.min(tr0.theta)), float(np.min(traj.theta))),
        ("beta sweep (rad)", float(tr0.beta.max() - tr0.beta.min()),
         float(traj.beta.max() - traj.beta.min())),
    ]
    for name, a0, a1 in rows:
        print(f"  {name:16} {a0:12.6f} {a1:12.6f} {a1-a0:+12.6f}")

    out = Path(a.out) if a.out else (
        SHIPPED_V070 / f"gslip_pronk_template{a.suffix}.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    g2c.to_template_csv(traj, out)
    print(f"\nwrote {out}")
    print("  gamma column is ZERO by design -- a scalar gamma is common-mode")
    print("  across all four legs and cannot express an Ackermann pair. The")
    print("  lean is delivered through gamma_acker_in/out, as now.")


if __name__ == "__main__":
    main()
