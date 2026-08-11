"""Phase 3: export the Corgi pronk template as a hardware-replayable CSV.

Builds the SLIP-RF fixed point at the chosen target speed, parametrizes it as
a stride template, maps it onto Corgi joint commands, checks it against every
hardware limit, and writes a 12-DOF CSV that corgi_csv_control can replay.

Replaying this open-loop in Webots validates the kinematic mapping and the
workspace guards with no controller in the loop -- the cheap checkpoint before
Phase 4 exists.

Run:
    uv run python examples/gslip/export_pronk_csv.py
"""

import argparse

import numpy as np

from legwheel.config import OUTPUT_CSV_DIR, RobotParams
from legwheel.models import slip_rf
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.planners import gslip_template as tpl
from legwheel.planners import gslip_to_corgi as g2c

MASS, G = 30.0, 9.81
K_REL = 18.0
V_TILDE = 1.2
MOTOR_TORQUE_LIMIT = 35.0
N_LEGS = 4

# Nominal stance, as leg extension. Phase 0 originally used 65.89 deg because
# that hits the paper's combined-indicator optimum r~ = 0.6303, but because the
# Corgi's foot radius is a large fraction of its leg, that optimum is a 0.230 m
# crouch -- too low to clear the support block, and needlessly harsh on the
# hardware. stance_height_tradeoff.py shows a taller stance costs only a slope
# penalty (1.154 -> 1.224 at 95 deg) while roughly halving peak ground reaction
# and motor torque and softening the spring, which matters because a softer
# virtual spring is far less corrupted by the 0.36-0.68 N.m joint friction.
NOMINAL_THETA_DEG = 100.0


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--v-tilde", type=float, default=V_TILDE,
                    help="dimensionless target speed (default 1.2, the shipped "
                         "design point)")
    ap.add_argument("--beta-range", type=float, nargs=2, default=(70.0, 80.0),
                    metavar=("LO", "HI"),
                    help="landing-angle search window in degrees. Default "
                         "70-80 reproduces the shipped template exactly. IT "
                         "DOES NOT COVER LOW SPEEDS: the fixed point at "
                         "v~0.42-0.50 sits at beta* = 83-84 deg, outside this "
                         "window, so exporting a slow template with the default "
                         "silently finds either nothing or a badly conditioned "
                         "point on the boundary.")
    ap.add_argument("--suffix", default="",
                    help="appended to the output filenames, so a new speed does "
                         "not clobber the shipped template")
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    v_tilde = args.v_tilde
    beta_lo, beta_hi = args.beta_range

    # Derive the stance geometry from the real linkage at the nominal pose,
    # rather than carrying hardcoded lengths that can drift out of step.
    leg_map = g2c.LegLengthMap()
    foot_radius = leg_map.leg.foot_radius
    hip_to_arc = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))

    p = slip_rf.SlipRfParams(
        m=MASS, l0=hip_to_arc + foot_radius,
        k=K_REL * MASS * G / hip_to_arc, r=foot_radius,
    )
    v = v_tilde * np.sqrt(G * p.l0)

    print()
    print("=" * 72)
    print(f"NOMINAL STANCE   theta = {NOMINAL_THETA_DEG} deg")
    print("=" * 72)
    print(f"  hip-to-arc-center l0 = {hip_to_arc:.4f} m")
    print(f"  standing hip height  = {p.l0:.4f} m")
    print(f"  r~ = {foot_radius/p.l0:.4f}   (paper optimum 0.6303)")
    print(f"  k  = {p.k:.0f} N/m total, {p.k/N_LEGS:.0f} N/m per leg")

    # Best-conditioned fixed point near the stability-sweep optimum.
    best = None
    for beta_deg in np.arange(beta_lo, beta_hi + 0.01, 0.25):
        for fp in find_fixed_points(
            p, v, np.deg2rad(beta_deg),
            alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
            n_samples=30, stride_fn=slip_rf.stride,
        ):
            if best is None or abs(fp.slope) < abs(best.slope):
                best = fp
    if best is None:
        raise SystemExit(
            f"no fixed point found at v~ = {v_tilde} in beta "
            f"{beta_lo}-{beta_hi} deg. The window is speed-dependent: beta* "
            f"rises as speed falls (71.75 deg at v~1.20, 83-84 deg at "
            f"v~0.42-0.50). Try --beta-range 78 88.")

    # A best fixed point sitting ON the search boundary means the true optimum
    # is outside it, and the exported template is not the best-conditioned one
    # at this speed. Silent when it happens, so it is checked.
    edge = min(abs(np.rad2deg(best.beta) - beta_lo),
               abs(np.rad2deg(best.beta) - beta_hi))
    if edge < 0.5:
        print(f"\n  WARNING: beta* = {np.rad2deg(best.beta):.2f} deg is on the "
              f"edge of the {beta_lo}-{beta_hi} search window.\n"
              f"  The real optimum is probably outside it. Widen --beta-range.")

    print()
    print("=" * 72)
    print(f"FIXED POINT   v = {v:.3f} m/s (v~ = {v_tilde}), k_rel = {K_REL}")
    print("=" * 72)
    print(f"  landing angle beta   = {np.rad2deg(best.beta):.2f} deg")
    print(f"  touchdown angle alpha= {np.rad2deg(best.alpha):.2f} deg")
    print(f"  map slope            = {best.slope:+.4f} "
          f"({'stable' if best.stable else 'UNSTABLE - needs clocked torque'})")
    print(f"  stance / flight      = {best.stance_time:.4f} / {best.flight_time:.4f} s")
    print(f"  period / duty        = {best.period:.4f} s / {best.duty_factor:.3f}")
    print(f"  stride length        = {best.stride_length:.4f} m")

    template = tpl.build_template(p, v, best.alpha, best.beta)
    err = tpl.tracking_error(template)
    print()
    print("QUINTIC FIT TO THE STANCE TRAJECTORY")
    print(f"  max angle error  = {np.rad2deg(err['max_angle_error']):.4f} deg")
    print(f"  max length error = {1000*err['max_length_error']:.4f} mm")

    # One CSV row per 1 kHz control tick, so replay runs at the right speed.
    n_samples = g2c.samples_for_rate(template.period, rate_hz=1000.0)
    traj = g2c.map_template(template, n=n_samples)
    report = traj.guard_report()
    print()
    print("=" * 72)
    print("CORGI JOINT COMMANDS")
    print("=" * 72)
    print(f"  theta   {report['theta_min_deg']:.2f} to {report['theta_max_deg']:.2f} deg"
          f"   (limits {RobotParams.MIN_THETA_DEG}-{RobotParams.MAX_THETA_DEG})"
          f"  {'OK' if report['theta_ok'] else 'VIOLATION'}")
    print(f"  |beta|  peaks at {report['beta_abs_max_deg']:.2f} deg"
          f"   (limit {RobotParams.BETA_MAX_DEG})"
          f"  {'OK' if report['beta_ok'] else 'VIOLATION'}")
    print(f"  contact reaches {report['arc_max_deg']:.2f} deg on the foot arc"
          f"   (half-span {g2c.FOOT_ARC_HALF_SPAN_DEG})"
          f"  {'OK' if report['stays_on_foot_arc'] else 'LEAVES ARC'}")

    traj.assert_feasible()

    res = slip_rf.stride(p, v, best.alpha, best.beta)
    f_leg = res["peak_grf_mag"] / N_LEGS
    theta_at_peak = float(traj.theta[int(np.argmin(traj.theta))])
    tau = g2c.motor_torque_for(f_leg, theta_at_peak, leg_map)
    print()
    print(f"  peak GRF        {res['peak_grf_mag']:.0f} N total "
          f"({res['peak_grf_mag']/(MASS*G):.2f} body weights), "
          f"{f_leg:.0f} N per leg")
    print(f"  peak motor torque {tau:.2f} N.m "
          f"({100*tau/MOTOR_TORQUE_LIMIT:.0f}% of {MOTOR_TORQUE_LIMIT} N.m)")

    out = OUTPUT_CSV_DIR / f"gslip_pronk{args.suffix}.csv"
    cycles = 5
    g2c.to_csv(traj, out, cycles=cycles)
    template_out = OUTPUT_CSV_DIR / f"gslip_pronk_template{args.suffix}.csv"
    g2c.to_template_csv(traj, template_out)
    dt_ms = 1000 * traj.period / (len(traj.t) - 1)
    print()
    print(f"  wrote {out}")
    print(f"    open-loop replay: {len(traj.t)} samples/stride x {cycles} cycles, "
          f"12 columns, {dt_ms:.3f} ms per row")
    print(f"  wrote {template_out}")
    print(f"    controller reference: one leg + stance flag, "
          f"{int(traj.in_stance.sum())} stance / "
          f"{int((~traj.in_stance).sum())} flight samples")

    print()
    print("  gslip_pronk_node parameters for this stance:")
    print(f"    k_radial:={p.k / N_LEGS:.0f}.0   (leg-frame radial spring, per leg)")
    print(f"    b_radial:={0.008 * p.k / N_LEGS:.0f}.0")
    print()


if __name__ == "__main__":
    main()
