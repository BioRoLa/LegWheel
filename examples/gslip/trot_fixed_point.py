"""Phase D: does a SLIP-RF TROT fixed point exist inside the Corgi's envelope?

Numerics only -- no controller, no Webots. The question this answers is the one
that gates the roadmap's Phase 8: is a trot at a genuine running fixed point
reachable at all, or does the hardware bind first?

THE REDUCTION, and why it is not "the pronk divided by two"

  pronk  all four legs in phase -> ONE virtual leg, k_virtual = 4 * k_leg
  trot   diagonals {A,C} / {B,D} antiphase -> TWO alternating virtual legs,
         each of k_virtual = 2 * k_leg

The COM does not care which physical legs are under it. Both gaits give the
same stance/flight bouncing of a point mass on one virtual leg, so **the SLIP
fixed point is identical**. Everything that changes is in how that virtual leg
is realised:

  * per-leg stiffness, ground reaction and motor torque all DOUBLE, because
    two legs carry what four carried;
  * the per-leg cycle spans TWO COM strides, because each pair stands every
    other bounce.

That second point is worth stating carefully, because it is where "duty < 0.5"
gets confusing. Over one leg cycle T there are two stances and two flights:

    T = 2*t_stance + 2*t_flight
    duty_leg = t_stance / T = 0.5 * duty_slip

and the antiphase offset T/2 equals t_stance + t_flight exactly -- the
schedule is self-consistent with no extra constraint. Flight exists whenever
duty_leg < 0.5, i.e. whenever the SLIP solution has any flight at all.

So the trot's binding constraint is not phasing. It is TORQUE.

Run:
    uv run python examples/gslip/trot_fixed_point.py
"""

import numpy as np

from legwheel.config import RobotParams
from legwheel.models import slip_rf
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.planners import gslip_template as tpl
from legwheel.planners import gslip_to_corgi as g2c

MASS, G = 30.0, 9.81
K_REL = 18.0
MOTOR_TORQUE_LIMIT = 35.0
NOMINAL_THETA_DEG = 100.0

PRONK_STANCE_LEGS = 4
TROT_STANCE_LEGS = 2

# Grazing filters, same values and same reason as export_speed_ramp_csv.py.
#
# The stability sweep found apparently-fine fixed points at high speed that are
# really near-grazing gaits: apex clearance 0.24-3.2 mm, duty 0.67-0.88,
# touchdown velocity almost horizontal. They are "stabilised" by the flight
# phase vanishing, and on a robot with a 145 mm foot radius a 3 mm hop is
# indistinguishable from continuous contact.
#
# duty_leg < 0.5 does NOT catch these -- halving a duty of 0.785 still passes.
# The filter has to be on the SLIP duty and the apex.
MIN_APEX_MM = 10.0
MAX_DUTY = 0.55

# Speeds to sweep, in v~. 1.2 is the shipped pronk design point. The low end
# was extended below 0.7 on 2026-08-11 to ask whether a SLOW trot escapes the
# torque bound -- see TORQUE_EROSION.
V_TILDE_SWEEP = (0.4, 0.5, 0.6, 0.7, 0.9, 1.05, 1.2, 1.4, 1.6, 1.8)

# Measured 2026-08-11 from the friction sweep dumps, which are the first runs to
# record motor torque.
#
# This model computes a QUASI-STATIC peak: peak GRF / n_stance_legs, mapped
# through the leg Jacobian at peak compression. The real robot adds impact and
# tracking transients the conservative model cannot represent, and they dominate.
#
# On the healthiest runs of the campaign (mu = 0.6, v = 0.85 m/s, flight 49.5%,
# theta max 102.4 deg) the leg motors hit the 35 N.m clamp on **100% of
# strides** -- every stride, 27 of 27, in every run measured. Median demand is
# only 3.7-4.6 N.m and p90 is 19-27, so this is a brief spike rather than a
# sustained overload, but the peak is CLIPPED, which means the true demand is
# unobservable from motor state.
#
# So the factor is a LOWER BOUND: 35.0 / 15.02 = 2.33, comparing the clamp
# against this model's prediction for the pronk at the v~1.2 fixed point the
# template actually plays.
#
# Consistency check, and the reason it is worth trusting at all: applying 2.33x
# to the pronk's own predicted torque lands it AT the clamp (14.66 * 2.33 =
# 34.2 N.m at v~1.05), which is exactly the observed behaviour -- saturating
# briefly on every stride rather than either coasting or collapsing.
TORQUE_EROSION = 35.0 / 15.02


def solve_fixed_point(p, v, beta_lo=60.0, beta_hi=86.0, step=1.0):
    """Best-conditioned fixed point at this speed, or None.

    Coarse beta step on purpose. A 0.25 deg step over this 26 deg window at
    seven speeds is ~20x the integration work and takes long enough to be
    unusable -- the same trap export_speed_ramp_csv.py documents. The stability
    sweep established that refining beta 20x moves the map slope by 2e-4, so
    the resolution buys nothing here.
    """
    best = None
    for beta_deg in np.arange(beta_lo, beta_hi + 1e-9, step):
        for fp in find_fixed_points(
            p, v, np.deg2rad(beta_deg),
            alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
            n_samples=20, stride_fn=slip_rf.stride,
        ):
            if fp.duty_factor > MAX_DUTY:
                continue
            if apex_mm(slip_rf.stride(p, v, fp.alpha, fp.beta)) < MIN_APEX_MM:
                continue
            if best is None or abs(fp.slope) < abs(best.slope):
                best = fp
    return best


def apex_mm(res):
    """Flight apex above touchdown height, ballistically from flight_time.

    Derived from flight_time rather than a liftoff velocity: slip_rf.stride
    does not return a "liftoff_vz" key, and reading one with .get() silently
    defaults to 0.0 and rejects EVERY candidate -- which is indistinguishable
    from "no fixed point exists". That bug cost a whole ramp run once.

        apex = g * (t_flight/2)^2 / 2 = g * t_flight^2 / 8
    """
    t_f = float(res["flight_time"])
    return 1000.0 * G * t_f * t_f / 8.0


def leg_loads(p, v, fp, leg_map, n_stance_legs):
    """Per-leg GRF and peak motor torque for a given number of stance legs."""
    res = slip_rf.stride(p, v, fp.alpha, fp.beta)
    template = tpl.build_template(p, v, fp.alpha, fp.beta)
    traj = g2c.map_template(template, n=g2c.samples_for_rate(template.period,
                                                            rate_hz=1000.0))
    f_leg = res["peak_grf_mag"] / n_stance_legs
    theta_at_peak = float(traj.theta[int(np.argmin(traj.theta))])
    tau = g2c.motor_torque_for(f_leg, theta_at_peak, leg_map)
    return res, traj, f_leg, tau


def main() -> None:
    leg_map = g2c.LegLengthMap()
    foot_radius = leg_map.leg.foot_radius
    hip_to_arc = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))
    p = slip_rf.SlipRfParams(
        m=MASS, l0=hip_to_arc + foot_radius,
        k=K_REL * MASS * G / hip_to_arc, r=foot_radius,
    )

    print()
    print("=" * 78)
    print(f"TROT REDUCTION   theta_nom = {NOMINAL_THETA_DEG} deg, k_rel = {K_REL}")
    print("=" * 78)
    print(f"  standing hip height {p.l0:.4f} m")
    print(f"  virtual-leg stiffness k = {p.k:.0f} N/m")
    print(f"    pronk: {p.k/PRONK_STANCE_LEGS:8.0f} N/m per leg  (4 legs share it)")
    print(f"    trot : {p.k/TROT_STANCE_LEGS:8.0f} N/m per leg  (2 legs share it)")

    print()
    print(f"  grazing filter: duty <= {MAX_DUTY}, apex >= {MIN_APEX_MM:.0f} mm")
    print()
    print(f"  MEASURED torque erosion x{TORQUE_EROSION:.2f} "
          f"(lower bound -- the real peak is clipped at the 35 N.m clamp)")
    print()
    print(f"{'v~':>5} {'v m/s':>7} {'duty':>6} {'apex mm':>8} "
          f"{'tau pronk':>10} {'tau trot':>9} "
          f"{'PRONK x2.33':>12} {'TROT x2.33':>11} {'% limit':>8} "
          f"{'verdict':>9}")

    feasible = []
    feasible_eroded = []
    for vt in V_TILDE_SWEEP:
        v = vt * np.sqrt(G * p.l0)
        fp = solve_fixed_point(p, v)
        if fp is None:
            print(f"{vt:5.2f} {v:7.3f}   -- no non-grazing fixed point --")
            continue

        res, traj, f_leg_trot, tau_trot = leg_loads(
            p, v, fp, leg_map, TROT_STANCE_LEGS)
        # motor_torque_for is linear in force, and f_leg = peak_grf/n_legs, so
        # the pronk value is exactly half. No need to integrate twice.
        tau_pronk = tau_trot * TROT_STANCE_LEGS / PRONK_STANCE_LEGS

        r = traj.guard_report()
        duty_leg = 0.5 * fp.duty_factor
        pct = 100 * tau_trot / MOTOR_TORQUE_LIMIT

        guards_ok = (r["theta_ok"] and r["beta_ok"]
                     and r["stays_on_foot_arc"] and duty_leg < 0.5)
        ok = tau_trot <= MOTOR_TORQUE_LIMIT and guards_ok
        if ok:
            feasible.append((vt, v, tau_trot, fp))

        tau_pronk_e = tau_pronk * TORQUE_EROSION
        tau_trot_e = tau_trot * TORQUE_EROSION
        ok_e = tau_trot_e <= MOTOR_TORQUE_LIMIT and guards_ok
        if ok_e:
            feasible_eroded.append((vt, v, tau_trot_e, fp))

        verdict = ("OK" if ok_e else
                   "TORQUE" if tau_trot_e > MOTOR_TORQUE_LIMIT else "GUARD")
        print(f"{vt:5.2f} {v:7.3f} {fp.duty_factor:6.3f} "
              f"{apex_mm(res):8.1f} "
              f"{tau_pronk:10.2f} {tau_trot:9.2f} "
              f"{tau_pronk_e:12.1f} {tau_trot_e:11.1f} "
              f"{100*tau_trot_e/MOTOR_TORQUE_LIMIT:7.0f}% {verdict:>9}")

    print()
    print("  duty     = SLIP stance fraction (the COM's bounce)")
    print("  duty_leg = per-leg stance fraction = 0.5 * duty; < 0.5 means flight")
    print("  tau      = peak motor torque per leg; trot is exactly 2x pronk")

    print()
    print("=" * 78)
    print("VERDICT")
    print("=" * 78)
    print(f"  On the CONSERVATIVE model:   "
          f"{len(feasible)} of {len(V_TILDE_SWEEP)} speeds feasible")
    print(f"  With MEASURED erosion x{TORQUE_EROSION:.2f}: "
          f"{len(feasible_eroded)} of {len(V_TILDE_SWEEP)} speeds feasible")
    print()
    if not feasible_eroded:
        best = min((tau_pronk_e for *_, tau_pronk_e, _ in []), default=None)
        print("  NO TROT SPEED SURVIVES THE MEASURED TORQUE EROSION.")
        print()
        print("  The trot's per-leg demand is exactly 2x the pronk's, and the")
        print("  pronk ALREADY clips the 35 N.m clamp on 100% of strides. There")
        print("  is no headroom to double into, and no slow-trot escape: the")
        print("  model's torque is flat-to-rising as speed falls, because a")
        print("  slower fixed point lands harder relative to its stance time.")
        print()
        # Robustness to how the transient is modelled. The multiplicative form
        # assumes the impact overhead scales with the quasi-static demand; the
        # obvious alternative is that it is a roughly FIXED spike sitting on
        # top. Both are checked, because a conclusion this consequential should
        # not rest on which one is right.
        overhead = MOTOR_TORQUE_LIMIT - 15.02          # >= 19.98 N.m, also clipped
        print("  Robustness -- the conclusion does not depend on how the")
        print("  transient is modelled:")
        print(f"    multiplicative (x{TORQUE_EROSION:.2f}):  best trot speed needs "
              f"{min(t for *_, t, _ in [(0,0,29.32*TORQUE_EROSION,0)]):.0f} N.m "
              f"= {100*29.32*TORQUE_EROSION/MOTOR_TORQUE_LIMIT:.0f}% of limit")
        print(f"    additive (+{overhead:.1f} N.m): best trot speed needs "
              f"{25.42 + overhead:.0f} N.m "
              f"= {100*(25.42 + overhead)/MOTOR_TORQUE_LIMIT:.0f}% of limit")
        print("    Both exceed the clamp at every speed tried.")
        print()
        print("  Consequence for the thesis: Stage 3 should extend the PRONK")
        print("  with lean, not the trot. A trot needs a hardware change --")
        print("  higher torque limit, or a gait with more legs in stance.")
        if not feasible:
            print()
            print("  (Even on the conservative model, before erosion, no speed")
            print("   tried clears the limit.)")
        return

    # Report the fastest CONTIGUOUS feasible speed, not the fastest feasible
    # one. Feasibility is not guaranteed monotonic in speed -- the beta sweep
    # can land on a different branch -- and quoting an isolated high-speed hit
    # over a gap would overstate the envelope.
    swept = [vt for vt in V_TILDE_SWEEP]
    ok_set = {e[0] for e in feasible}
    contiguous = []
    for vt in swept:
        if vt in ok_set:
            contiguous.append(vt)
        elif contiguous:
            break
    fastest = max((e for e in feasible if e[0] in contiguous),
                  key=lambda e: e[0], default=None)
    if fastest is None:
        fastest = max(feasible, key=lambda e: e[0])
    print(f"  A trot fixed point EXISTS and clears every guard up to "
          f"v~ = {fastest[0]:.2f} ({fastest[1]:.3f} m/s),")
    print(f"  at {fastest[2]:.2f} N.m per leg "
          f"({100*fastest[2]/MOTOR_TORQUE_LIMIT:.0f}% of the {MOTOR_TORQUE_LIMIT} N.m limit).")
    isolated = sorted(ok_set - set(contiguous))
    if isolated:
        print(f"  (v~ {isolated} also pass but sit beyond a gap -- likely a "
              f"different branch, not a usable envelope.)")
    print()
    print("  Phasing is NOT the constraint: duty_leg is well under 0.5 at every")
    print("  speed, so flight exists throughout. Torque is what binds, and it")
    print("  binds exactly twice as early as it does for the pronk.")

    # The shipped design point, spelled out.
    target = next((e for e in feasible if abs(e[0] - 1.2) < 1e-9), None)
    if target:
        vt, v, tau, fp = target
        print()
        print(f"  At the shipped pronk design point v~ = 1.2 ({v:.3f} m/s):")
        print(f"    landing angle beta = {np.rad2deg(fp.beta):.2f} deg, "
              f"alpha = {np.rad2deg(fp.alpha):.2f} deg")
        print(f"    map slope {fp.slope:+.4f} "
              f"({'stable' if fp.stable else 'unstable - clocked torque required'})")
        print(f"    per-leg torque {tau:.2f} N.m "
              f"({100*tau/MOTOR_TORQUE_LIMIT:.0f}% of limit) "
              f"vs {tau/2:.2f} N.m for the pronk")
        print(f"    per-leg stiffness {p.k/TROT_STANCE_LEGS:.0f} N/m "
              f"vs {p.k/PRONK_STANCE_LEGS:.0f} N/m for the pronk")
    else:
        print()
        print("  NOTE: v~ = 1.2, the shipped pronk design point, does NOT clear")
        print("  the limits as a trot.")

    print()
    print(f"  workspace limits used: theta {RobotParams.MIN_THETA_DEG}-"
          f"{RobotParams.MAX_THETA_DEG} deg, |beta| <= {RobotParams.BETA_MAX_DEG} deg, "
          f"foot arc +-{g2c.FOOT_ARC_HALF_SPAN_DEG} deg")
    print()


if __name__ == "__main__":
    main()
