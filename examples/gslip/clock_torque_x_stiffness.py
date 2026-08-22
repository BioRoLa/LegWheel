"""Does a SOFTER SPRING make clock-torque stabilisation affordable?

The clock-torque gate (log S141) found the map IS stabilisable at the plant's
speed -- slope +1.2416 -> -0.0186 as k_c runs 0 -> 600 -- but the cheapest
stable setting costs 34.5 N.m eroded against S32's 29.5 N.m real stall.
**1.17x short**, which is close enough that a second lever might close it.

S134 supplies the candidate: every self-stable, torque-feasible gait it found
sat at k_rel 7-15 against the shipped 18, i.e. a spring up to 2.6x softer. A
softer spring lowers peak ground reaction and therefore torque, and it also
moves the map. Whether the two effects combine favourably is not obvious from
either result alone, so sweep them jointly.

Zero sim time.

Run:
    uv run python examples/gslip/clock_torque_x_stiffness.py
"""

from __future__ import annotations

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.gslip import GSlipFailure
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.planners import gslip_template as tpl
from legwheel.planners import gslip_to_corgi as g2c

MASS, G, THETA = 30.0, 9.81, 100.0
MIN_APEX_MM, MAX_DUTY = 10.0, 0.55
STALL, USABLE = 29.5, 25.0
EROSION = 35.0 / 15.02
VT = 0.34                       # the plant's operating point

K_RELS = (7.0, 10.0, 14.0, 18.0)
GAINS = ((15.0, 3.0), (20.0, 4.0), (25.0, 4.0), (30.0, 4.0), (40.0, 6.0))

LEG = g2c.LegLengthMap()
R = LEG.leg.foot_radius
HIP = LEG.length(np.deg2rad(THETA))


def params(k_rel):
    return slip_rf.SlipRfParams(m=MASS, l0=HIP + R, k=k_rel * MASS * G / HIP, r=R)


def baseline(p, v):
    best = None
    for bd in np.arange(60.0, 89.01, 0.25):
        for fp in find_fixed_points(p, v, np.deg2rad(bd),
                                    alpha_range=(np.deg2rad(1.0), np.deg2rad(75.0)),
                                    n_samples=40, stride_fn=slip_rf.stride):
            apex = 1000.0 * G * fp.flight_time ** 2 / 8.0
            if apex < MIN_APEX_MM or fp.duty_factor > MAX_DUTY:
                continue
            if best is None or abs(fp.slope) < abs(best.slope):
                best = fp
    return best


def leg_torque(p, v, fp, tau_fn):
    """Peak LEG-motor torque, priced as pronk_operating_point.py does.

    The clock torque acts on the leg angle and the leg motors also carry the
    spring's ground reaction. Both are reported: a stabiliser that fits on the
    hip axis but blows the leg budget is not affordable either.
    """
    res = slip_rf.stride(p, v, fp.alpha, fp.beta, tau_fn=tau_fn)
    t = tpl.build_template(p, v, fp.alpha, fp.beta)
    traj = g2c.map_template(t, n=g2c.samples_for_rate(t.period, 1000.0))
    tq = g2c.motor_torque_for(res["peak_grf_mag"] / 4.0,
                              float(traj.theta[int(np.argmin(traj.theta))]), LEG)
    return tq * EROSION


def evaluate(p, v, base, k_c, d_c):
    peak = [0.0]
    phi_td = p.phi_touchdown(base.beta)
    t_st = base.stance_time
    rate = (-phi_td - phi_td) / t_st

    def tau_fn(t, length, phi, dl, dphi):
        tt = min(t, t_st)
        tq = k_c * (phi_td + rate * tt - phi) + d_c * ((rate if t < t_st else 0.0) - dphi)
        peak[0] = max(peak[0], abs(tq))
        return np.array([0.0, tq])

    def stride_fn(pp, vv, aa, bb):
        return slip_rf.stride(pp, vv, aa, bb, tau_fn=tau_fn)

    best = None
    for bd in np.arange(np.rad2deg(base.beta) - 6, np.rad2deg(base.beta) + 6.01, 0.5):
        try:
            cands = find_fixed_points(p, v, np.deg2rad(bd),
                                      alpha_range=(np.deg2rad(1.0), np.deg2rad(75.0)),
                                      n_samples=30, stride_fn=stride_fn)
        except (GSlipFailure, ValueError, np.linalg.LinAlgError):
            continue
        for fp in cands:
            try:
                res = slip_rf.stride(p, v, fp.alpha, fp.beta, tau_fn=tau_fn)
            except (GSlipFailure, ValueError, np.linalg.LinAlgError):
                continue
            apex = 1000.0 * G * res["flight_time"] ** 2 / 8.0
            duty = res["stance_time"] / res["period"]
            if apex < MIN_APEX_MM or duty > MAX_DUTY:
                continue
            if best is None or abs(fp.slope) < abs(best.slope):
                best = fp
    if best is None:
        return None
    return best, peak[0] * EROSION, leg_torque(p, v, best, tau_fn)


def main() -> None:
    print(__doc__.split("Run:")[0].rstrip())
    print()
    hits = []
    for k_rel in K_RELS:
        p = params(k_rel)
        v = VT * np.sqrt(G * p.l0)
        base = baseline(p, v)
        print("=" * 80)
        print("k_rel %.0f  (k %.0f N/m total, %.0f per leg)  v~%.2f"
              % (k_rel, p.k, p.k / 4, VT))
        print("=" * 80)
        if base is None:
            print("  no non-grazing conservative fixed point\n")
            continue
        print("  conservative: beta* %.2f  slope %+.4f  duty %.3f"
              % (np.rad2deg(base.beta), base.slope, base.duty_factor))
        print("  %7s %6s %10s %12s %12s %s"
              % ("k_c", "d_c", "slope", "tau_hip", "tau_leg", "verdict"))
        for k_c, d_c in GAINS:
            out = evaluate(p, v, base, k_c, d_c)
            if out is None:
                print("  %7.1f %6.1f %10s %12s %12s no fixed point"
                      % (k_c, d_c, "--", "--", "--"))
                continue
            fp, t_hip, t_leg = out
            stable = abs(fp.slope) < 1.0
            worst = max(t_hip, t_leg)
            fits = worst <= STALL
            verdict = ("*** STABLE + FITS ***" if stable and fits else
                       "stable, over" if stable else
                       "unstable, fits" if fits else "unstable, over")
            if stable and fits:
                hits.append((k_rel, k_c, d_c, fp.slope, t_hip, t_leg))
            print("  %7.1f %6.1f %+10.4f %12.1f %12.1f %s"
                  % (k_c, d_c, fp.slope, t_hip, t_leg, verdict))
        print()

    print("=" * 80)
    if hits:
        print("STABLE AND AFFORDABLE (both axes inside %.1f N.m):" % STALL)
        for k_rel, k_c, d_c, sl, th, tl in sorted(hits, key=lambda h: max(h[4], h[5])):
            extra = "  <- also inside the %.0f usable line" % USABLE \
                if max(th, tl) <= USABLE else ""
            print("  k_rel %.0f, k_c %.1f, d_c %.1f -> slope %+.4f, hip %.1f, leg %.1f%s"
                  % (k_rel, k_c, d_c, sl, th, tl, extra))
        print()
        print("  ⇒ clock torque IS affordable at this speed, with a softer spring.")
        print("    That is a THREE-PLACE change (S133): k_rel in the model, the")
        print("    template CSV, and the k_radial launch parameter -- plus MODEL_NM")
        print("    in tau_demand_window.py. Do not move one without the others.")
    else:
        print("NOTHING is both stable and affordable at v~%.2f, at any stiffness" % VT)
        print("tried. Softening the spring does not close the 1.17x gap.")
    print("=" * 80)


if __name__ == "__main__":
    main()
