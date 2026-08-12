"""Does the exported template's beta sweep move the hip as far as the model says?

Section 29 raised, but did not establish, a possible SECOND fault: the template's
stance advance (0.174 m from L(theta)*beta_dot) implied a mean stance speed of
1.78 m/s while the same fixed point's flight seemed to imply ~1.47, and
horizontal velocity must be continuous across liftoff. That arithmetic crossed
two sources at different beta-search resolutions, so it was flagged as unverified.

This checks it properly, comparing like with like:

  MODEL     slip_rf.stride returns stance_length = x at liftoff, and
            stride_length = stance_length + vx_liftoff * t_flight. Those are
            continuous by construction, so the model cannot be internally
            inconsistent here -- the question is whether the TEMPLATE matches it.

  TEMPLATE  the exported CSV's beta(t) integrated through the rolling
            constraint: advance = integral of L(theta) * beta_dot over stance.

If those disagree, the exporter is writing a beta sweep that does not carry the
body as far as the fixed point it was solved from -- a generation fault, and a
different problem from the speed mismatch of section 29.

    uv run python examples/gslip/template_vs_model_stance.py
"""
import os

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.models.leg_model import LegModel
from legwheel.planners import gslip_to_corgi as g2c

MASS, G, K_REL = 30.0, 9.81, 18.0
NOMINAL_THETA_DEG = 100.0
CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "..", "output", "csv")

CASES = [
    ("v~1.20", 1.20, (70.0, 80.0), "gslip_pronk_template.csv"),
    ("v~0.70", 0.70, (74.0, 86.0), "gslip_pronk_template_v070.csv"),
    ("v~0.45", 0.45, (78.0, 88.0), "gslip_pronk_template_v045.csv"),
]


def main():
    lm = LegModel()
    leg_map = g2c.LegLengthMap()
    fr = leg_map.leg.foot_radius
    hip = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))
    p = slip_rf.SlipRfParams(m=MASS, l0=hip + fr, k=K_REL * MASS * G / hip, r=fr)
    vs = np.sqrt(G * p.l0)

    def L(theta):
        lm.forward(theta, 0.0, vector=True)
        return float(np.hypot(lm.O_r[0], lm.O_r[1])) + lm.foot_radius

    print(f"{'case':>8} {'model stance':>13} {'template roll':>14} {'ratio':>7} "
          f"{'vx_lo':>8} {'stance mean':>12} {'flight vx':>10} {'cont?':>7}")
    for name, vt, (blo, bhi), fn in CASES:
        v = vt * vs
        best = None
        for b in np.arange(blo, bhi + 1e-9, 0.25):
            for fp in find_fixed_points(
                p, v, np.deg2rad(b),
                alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
                n_samples=30, stride_fn=slip_rf.stride):
                if best is None or abs(fp.slope) < abs(best.slope):
                    best = fp
        if best is None:
            print(f"{name:>8}  no fixed point in {blo}-{bhi}")
            continue
        res = slip_rf.stride(p, v, best.alpha, best.beta)

        # Liftoff horizontal velocity: flight is ballistic, so
        # stride_length = stance_length + vx_lo * flight_time.
        vx_lo = (res["stride_length"] - res["stance_length"]) / res["flight_time"]
        stance_mean = res["stance_length"] / res["stance_time"]

        path = os.path.join(CSV, fn)
        if not os.path.exists(path):
            print(f"{name:>8}  template not found: {fn}")
            continue
        raw = np.genfromtxt(path, delimiter=",", names=True)
        t, th, be = raw["t"], raw["theta"], raw["beta"]
        st = raw["in_stance"] > 0.5
        dt = float(np.median(np.diff(t)))
        bdot = np.gradient(be, dt)
        roll = float(np.trapezoid((np.array([L(x) for x in th]) * bdot)[st],
                                  dx=dt))

        # vx must be continuous across liftoff: the body leaves stance at vx_lo
        # and keeps it through flight. Continuity is automatic in the model; the
        # real comparison is stance MEAN vs vx_lo, which differ legitimately
        # because the spring accelerates the body during stance.
        cont = "n/a"
        print(f"{name:>8} {res['stance_length']:13.4f} {roll:14.4f} "
              f"{roll/res['stance_length']:7.2f} {vx_lo:8.3f} "
              f"{stance_mean:12.3f} {vx_lo:10.3f} {cont:>7}")

    print()
    print("  model stance  = slip_rf stance_length, the body's x at liftoff")
    print("  template roll = integral L(theta)*beta_dot over the CSV's stance")
    print("  ratio 1.00 means the exported sweep carries the body exactly as")
    print("  far as the fixed point it was solved from.")
    print()
    print("  vx_lo is derived as (stride - stance)/flight_time, so flight is")
    print("  ballistic at vx_lo by construction -- continuity across liftoff is")
    print("  automatic in the model and is NOT the thing at issue. The stance")
    print("  MEAN legitimately differs from vx_lo because the spring")
    print("  accelerates the body through contact.")


if __name__ == "__main__":
    main()
