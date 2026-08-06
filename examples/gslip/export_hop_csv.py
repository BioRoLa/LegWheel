"""Export an in-place vertical hop template for the Corgi.

Companion to export_pronk_csv.py. That one exports the RUNNING template,
solved at a fixed point assuming 2.035 m/s of forward travel -- conditions a
robot starting from rest never reaches, which is why its stance depth, duty
factor and energy balance did not match in Webots.

This exports a hop the robot can actually perform from standstill:
beta = 90 deg (leg vertical), alpha = 90 deg (velocity straight down), so
SLIP-RF reduces to a vertical hopper with phi ~ 0 throughout. The conservative
spring makes any energy periodic, so the design variable is hop height rather
than a fixed point to solve for.

Run:
    uv run python examples/gslip/export_hop_csv.py
"""

import numpy as np

from legwheel.config import OUTPUT_CSV_DIR, RobotParams
from legwheel.models import slip_rf
from legwheel.planners import gslip_template as tpl
from legwheel.planners import gslip_to_corgi as g2c

MASS, G = 30.0, 9.81
K_REL = 18.0
NOMINAL_THETA_DEG = 100.0
N_LEGS = 4
MOTOR_TORQUE_LIMIT = 35.0

# Apex above the touchdown height. 30 mm gives duty 0.416 -- close to the
# running template's 0.434, so the stance/flight machinery is exercised the
# same way -- at 44% of the torque limit and theta_min 83 deg.
APEX_MM = 30.0


def main() -> None:
    leg_map = g2c.LegLengthMap()
    foot_radius = leg_map.leg.foot_radius
    hip_to_arc = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))

    p = slip_rf.SlipRfParams(
        m=MASS, l0=hip_to_arc + foot_radius,
        k=K_REL * MASS * G / hip_to_arc, r=foot_radius,
    )

    beta = np.deg2rad(90.0)
    alpha = np.deg2rad(90.0)
    v = float(np.sqrt(2 * G * APEX_MM / 1000.0))

    res = slip_rf.stride(p, v, alpha, beta)

    print()
    print("=" * 72)
    print(f"IN-PLACE HOP   apex {APEX_MM:.0f} mm")
    print("=" * 72)
    print(f"  touchdown speed  {v:.3f} m/s (straight down)")
    print(f"  stance / flight  {res['stance_time']:.4f} / {res['flight_time']:.4f} s")
    print(f"  period / duty    {res['period']:.4f} s / "
          f"{res['stance_time']/res['period']:.3f}")
    print(f"  compression      {1000*res['peak_compression']:.1f} mm")
    print(f"  peak GRF         {res['peak_grf_mag']:.0f} N "
          f"({res['peak_grf_mag']/(MASS*G):.2f} body weights)")

    template = tpl.build_template(p, v, alpha, beta)
    n_samples = g2c.samples_for_rate(template.period, rate_hz=1000.0)
    traj = g2c.map_template(template, n=n_samples)
    report = traj.guard_report()

    print()
    print("CORGI JOINT COMMANDS")
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

    f_leg = res["peak_grf_mag"] / N_LEGS
    theta_at_peak = float(traj.theta[int(np.argmin(traj.theta))])
    tau = g2c.motor_torque_for(f_leg, theta_at_peak, leg_map)
    print(f"  peak motor torque {tau:.2f} N.m "
          f"({100*tau/MOTOR_TORQUE_LIMIT:.0f}% of {MOTOR_TORQUE_LIMIT} N.m)")

    out = OUTPUT_CSV_DIR / "gslip_hop.csv"
    template_out = OUTPUT_CSV_DIR / "gslip_hop_template.csv"
    g2c.to_csv(traj, out, cycles=5)
    g2c.to_template_csv(traj, template_out)

    print()
    print(f"  wrote {out}")
    print(f"  wrote {template_out}")
    print(f"    {len(traj.t)} samples/stride, "
          f"{1000*template.period/(len(traj.t)-1):.3f} ms per row, "
          f"{int(traj.in_stance.sum())} stance / {int((~traj.in_stance).sum())} flight")
    print()
    print("  gslip_pronk_node parameters unchanged: "
          f"k_radial:={p.k/N_LEGS:.0f}.0")
    print()


if __name__ == "__main__":
    main()
