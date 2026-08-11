"""Which fixed point is the robot ACTUALLY near?

The controller plays the v~1.20 template -- a fixed point solved for 2.035 m/s
-- and the robot runs at 0.70-0.85 m/s, about 40% of it. Every torque, flight
and duty prediction quoted against the design point therefore compares the robot
to a gait it has never performed.

This asks the other question: at the speeds the robot actually reaches, does a
non-grazing SLIP-RF fixed point exist at all, and if so what does it look like?

It matters for Stage 2a. The cambered template has to reduce to the planar one
at lambda = 0, and the planar fixed point it must reproduce is the one at the
operating speed -- not the design point. Solving the cambered family around a
fixed point the robot cannot reach would be solving the wrong problem.

    uv run python examples/gslip/pronk_operating_point.py
"""
import numpy as np

from legwheel.models import slip_rf
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.planners import gslip_template as tpl
from legwheel.planners import gslip_to_corgi as g2c

MASS, G = 30.0, 9.81
K_REL = 18.0
NOMINAL_THETA_DEG = 100.0
MOTOR_TORQUE_LIMIT = 35.0
TORQUE_EROSION = 35.0 / 15.02          # measured; see trot_fixed_point.py

# Grazing filters, same as trot_fixed_point.py and export_speed_ramp_csv.py.
MIN_APEX_MM = 10.0
MAX_DUTY = 0.55

# The measured envelope. Straight-line runs at mu = 0.6 gave 0.74-0.87 m/s with
# 31-52% flight; the campaign as a whole sits in 0.70-0.85.
MEASURED_V = (0.70, 0.85)
MEASURED_FLIGHT = (0.42, 0.50)


def apex_mm(res):
    t_f = float(res["flight_time"])
    return 1000.0 * G * t_f * t_f / 8.0


def solve(p, v, step=0.5):
    """Best-conditioned non-grazing fixed point at this speed, or None."""
    best = None
    for beta_deg in np.arange(60.0, 86.0 + 1e-9, step):
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


def main():
    leg_map = g2c.LegLengthMap()
    foot_radius = leg_map.leg.foot_radius
    hip_to_arc = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))
    p = slip_rf.SlipRfParams(
        m=MASS, l0=hip_to_arc + foot_radius,
        k=K_REL * MASS * G / hip_to_arc, r=foot_radius,
    )
    v_scale = np.sqrt(G * p.l0)

    print(f"l0 = {p.l0:.4f} m,  v = v~ * {v_scale:.4f} m/s")
    print(f"measured operating range {MEASURED_V[0]}-{MEASURED_V[1]} m/s "
          f"= v~ {MEASURED_V[0]/v_scale:.3f}-{MEASURED_V[1]/v_scale:.3f}")
    print(f"design point v~1.20 = {1.20*v_scale:.3f} m/s\n")

    print(f"{'v~':>6} {'v m/s':>7} {'beta*':>7} {'alpha*':>7} {'slope':>8} "
          f"{'duty':>6} {'flight%':>8} {'apex mm':>8} {'tau':>7} "
          f"{'tau x2.33':>10} {'status':>8}")

    lo = None
    for vt in np.arange(0.36, 0.72 + 1e-9, 0.02):
        v = vt * v_scale
        fp = solve(p, v)
        inrange = MEASURED_V[0] <= v <= MEASURED_V[1]
        if fp is None:
            print(f"{vt:6.2f} {v:7.3f} {'--':>7} {'--':>7} {'--':>8} "
                  f"{'--':>6} {'--':>8} {'--':>8} {'--':>7} {'--':>10} "
                  f"{'NONE' + ('  <<' if inrange else ''):>8}")
            continue
        if lo is None:
            lo = (vt, v)
        res = slip_rf.stride(p, v, fp.alpha, fp.beta)
        template = tpl.build_template(p, v, fp.alpha, fp.beta)
        traj = g2c.map_template(
            template, n=g2c.samples_for_rate(template.period, rate_hz=1000.0))
        f_leg = res["peak_grf_mag"] / 4.0            # pronk: four legs share
        tau = g2c.motor_torque_for(
            f_leg, float(traj.theta[int(np.argmin(traj.theta))]), leg_map)
        print(f"{vt:6.2f} {v:7.3f} {np.rad2deg(fp.beta):7.2f} "
              f"{np.rad2deg(fp.alpha):7.2f} {fp.slope:+8.4f} "
              f"{fp.duty_factor:6.3f} {100*(1-fp.duty_factor):7.1f}% "
              f"{apex_mm(res):8.1f} {tau:7.2f} {tau*TORQUE_EROSION:10.1f} "
              f"{'OK' + ('  <<' if inrange else ''):>8}")

    print()
    print("  '<<' marks speeds inside the MEASURED operating range.")
    print(f"  flight% is the model's; the robot measures "
          f"{100*MEASURED_FLIGHT[0]:.0f}-{100*MEASURED_FLIGHT[1]:.0f}%.")
    print(f"  tau x2.33 applies the measured torque erosion against the "
          f"{MOTOR_TORQUE_LIMIT:.0f} N.m clamp.")
    if lo:
        print()
        print(f"  Lowest speed with a non-grazing fixed point: "
              f"v~ {lo[0]:.2f} = {lo[1]:.3f} m/s")


if __name__ == "__main__":
    main()
