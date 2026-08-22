"""Can a CLOCK-TORQUE law make the map stable at the PLANT's operating point?

The deployment gate for the clocked-torque regulator (log S141), run before
any controller work -- the same discipline that stopped the Stage 2b deadbeat
in S135 before it cost a campaign.

WHAT "CLOCK TORQUE" MEANS HERE, PRECISELY

Lu & Lin eq 11 -- `gslip.stance_accel`'s own docstring -- says the clocked
torque model "passes tau_theta from the PD law". So CTR-SLIP's clock torque is
a PD to a CLOCK REFERENCE on the leg angle, not an open-loop torque waveform.

That matters, because the Corgi ALREADY does that: beta tracks a clocked
template through the leg-frame impedance law. So the honest question is not
"can we add clock torque" -- it is:

    does including the clock-torque PD in the MODEL make the return map
    stable at the speed the plant actually runs?

If yes, the plant's instability is a tuning problem and there is a gain to
find. If no, then clock torque is not a stabiliser at this operating point and
the controller work would be chasing something the model says is not there.

THE LAW

During stance, on the leg angle phi:

    tau_phi = k_c * (phi_ref(t) - phi) + d_c * (dphi_ref(t) - dphi)

phi_ref is the clock's reference: the touchdown-to-liftoff sweep the template
commands, played on stance time. The fixed point is re-solved WITH the law in
the dynamics, and the map slope is taken there.

WHAT IS SCORED

  GATE 1  a fixed point still exists with the law engaged
  GATE 2  |dP/dalpha| < 1 for some (k_c, d_c) at the plant's speed
  GATE 3  the peak tau_phi fits the leg's real ceiling -- S32's 29.5 N.m
          stall, Alex's line, not the legacy 35

Run:
    uv run python examples/gslip/clock_torque_gate.py
"""

from __future__ import annotations

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.gslip import GSlipFailure
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.planners import gslip_to_corgi as g2c

MASS, G, K_REL, THETA = 30.0, 9.81, 18.0, 100.0
MIN_APEX_MM, MAX_DUTY = 10.0, 0.55
STALL = 29.5              # N.m, S32 -- the line Alex named
EROSION = 35.0 / 15.02

# The plant. S133's continuation puts v_fwd 0.215-0.282 near v~0.34; the
# reference orbit the template encodes is v~0.70.
V_TILDES = (0.34, 0.70)

# Gains to sweep. k_c in N.m/rad on the leg angle; d_c critical-ish around it.
K_CS = (0.0, 5.0, 15.0, 40.0, 100.0, 250.0, 600.0)
D_RATIOS = (0.0, 0.05, 0.15)


def params():
    lm = g2c.LegLengthMap()
    r = lm.leg.foot_radius
    hip = lm.length(np.deg2rad(THETA))
    return slip_rf.SlipRfParams(m=MASS, l0=hip + r, k=K_REL * MASS * G / hip, r=r)


def baseline(p, v_tilde):
    """The conservative fixed point at this speed -- the reference and the seed."""
    v = v_tilde * np.sqrt(G * p.l0)
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
    return v, best


def make_tau(p, fp, k_c, d_c, peak):
    """Clock-torque PD to the template's own stance sweep.

    phi_ref runs linearly from the touchdown angle to its mirror over the
    nominal stance time -- which is what the exported template commands, a
    clocked sweep at constant rate. `peak` collects the largest |tau_phi| seen
    so the gate can price it.
    """
    phi_td = p.phi_touchdown(fp.beta)
    phi_lo = -phi_td                      # symmetric sweep at the fixed point
    t_st = fp.stance_time
    rate = (phi_lo - phi_td) / t_st

    def tau_fn(t, length, phi, dl, dphi):
        tt = min(t, t_st)
        phi_ref = phi_td + rate * tt
        dphi_ref = rate if t < t_st else 0.0
        tq = k_c * (phi_ref - phi) + d_c * (dphi_ref - dphi)
        peak[0] = max(peak[0], abs(tq))
        return np.array([0.0, tq])

    return tau_fn


def solve_with_law(p, v, fp0, k_c, d_c):
    """Re-solve the fixed point WITH the law in the dynamics."""
    peak = [0.0]
    tau_fn = make_tau(p, fp0, k_c, d_c, peak)

    def stride_fn(pp, vv, aa, bb):
        return slip_rf.stride(pp, vv, aa, bb, tau_fn=tau_fn)

    best = None
    lo, hi = np.rad2deg(fp0.beta) - 6.0, np.rad2deg(fp0.beta) + 6.0
    for bd in np.arange(lo, hi + 1e-9, 0.5):
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
    return best, peak[0]


def main() -> None:
    print(__doc__.split("Run:")[0].rstrip())
    print()
    p = params()
    for vt in V_TILDES:
        v, fp0 = baseline(p, vt)
        print("=" * 78)
        print("v~ %.2f  (v_td %.4f m/s)" % (vt, v))
        print("=" * 78)
        if fp0 is None:
            print("  no non-grazing conservative fixed point -- skipping\n")
            continue
        print("  conservative baseline: beta* %.2f deg, alpha* %.2f, slope %+.4f"
              % (np.rad2deg(fp0.beta), np.rad2deg(fp0.alpha), fp0.slope))
        print()
        print("  %8s %8s %10s %10s %12s %s"
              % ("k_c", "d_c", "beta*", "slope", "peak tau", "verdict"))
        found = []
        for k_c in K_CS:
            for dr in D_RATIOS:
                d_c = dr * k_c
                if k_c == 0.0 and dr != 0.0:
                    continue
                fp, peak = solve_with_law(p, v, fp0, k_c, d_c)
                if fp is None:
                    print("  %8.1f %8.2f %10s %10s %12s %s"
                          % (k_c, d_c, "--", "--", "--", "no fixed point"))
                    continue
                eroded = peak * EROSION
                stable = abs(fp.slope) < 1.0
                fits = eroded <= STALL
                verdict = ("STABLE + FITS" if stable and fits else
                           "stable, OVER budget" if stable else
                           "unstable")
                if stable and fits:
                    found.append((k_c, d_c, fp.slope, eroded))
                print("  %8.1f %8.2f %10.2f %+10.4f %12.1f %s"
                      % (k_c, d_c, np.rad2deg(fp.beta), fp.slope, eroded, verdict))
        print()
        if found:
            print("  ⇒ %d (k_c, d_c) settings are BOTH stable and inside %.1f N.m:"
                  % (len(found), STALL))
            for k_c, d_c, sl, tq in found:
                print("      k_c %.1f d_c %.2f -> slope %+.4f, tau %.1f N.m"
                      % (k_c, d_c, sl, tq))
        else:
            print("  ⇒ NO setting is both stable and affordable at this speed.")
        print()

    print("=" * 78)
    print("Peak tau is priced x%.2f erosion against S32's %.1f N.m real stall,"
          % (EROSION, STALL))
    print("which is Alex's named line -- not the legacy 35 N.m simulator clamp.")
    print("=" * 78)


if __name__ == "__main__":
    main()
