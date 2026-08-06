"""In-place vertical hopping template for the Corgi.

The running template is solved at a fixed point that assumes 2.035 m/s of
forward travel. A robot starting from rest never reaches those touchdown
conditions, so its compression, stance duration and energy balance bear no
relation to the model -- which is why tuning gains against it went nowhere.

A standing start needs its own template. Setting beta = 90 deg (leg vertical)
and alpha = 90 deg (velocity purely downward) reduces SLIP-RF to a vertical
hopper: phi stays at zero and only the spring length varies.

For a conservative spring this is periodic at ANY energy -- the model returns
to its apex exactly. So rather than root-finding a fixed point, the design
variable is simply hop height, and the question is which heights are
achievable within the workspace and torque budget.

Run:
    uv run python examples/gslip/hop_in_place.py
"""

import numpy as np

from legwheel.models import slip_rf
from legwheel.models.gslip import GSlipFailure
from legwheel.planners import gslip_to_corgi as g2c

MASS, G = 30.0, 9.81
K_REL = 18.0
NOMINAL_THETA_DEG = 100.0
N_LEGS = 4
MOTOR_LIMIT = 35.0
THETA_MIN_DEG = 17.0


def main() -> None:
    leg_map = g2c.LegLengthMap()
    r = leg_map.leg.foot_radius
    hip_to_arc = leg_map.length(np.deg2rad(NOMINAL_THETA_DEG))
    p = slip_rf.SlipRfParams(
        m=MASS, l0=hip_to_arc + r, k=K_REL * MASS * G / hip_to_arc, r=r
    )
    dl_dtheta = leg_map.slope(np.deg2rad(NOMINAL_THETA_DEG))

    print()
    print("=" * 92)
    print("IN-PLACE HOP  (beta = 90 deg, alpha = 90 deg -> pure vertical)")
    print("=" * 92)
    print(f"  m {MASS} kg   l0 {p.l0:.4f} m   r {p.r} m   k {p.k:.0f} N/m "
          f"({p.k/N_LEGS:.0f} per leg)")
    print(f"  static sag under body weight: "
          f"{1000*MASS*G/p.k:.1f} mm")
    print()

    beta = np.deg2rad(90.0)
    alpha = np.deg2rad(90.0)

    header = (f"{'apex[mm]':>9} {'v_td[m/s]':>10} {'stance[s]':>10} {'flight[s]':>10} "
              f"{'duty':>6} {'compress[mm]':>13} {'theta_min':>10} "
              f"{'GRF/BW':>7} {'tau[Nm]':>8} {'ok':>4}")
    print(header)
    print("-" * len(header))

    usable = []
    for apex_mm in (5, 10, 20, 30, 50, 80, 120):
        # Touchdown speed from a free fall through the apex height.
        v = float(np.sqrt(2 * G * apex_mm / 1000.0))
        try:
            res = slip_rf.stride(p, v, alpha, beta)
        except (GSlipFailure, ValueError) as exc:
            print(f"{apex_mm:9d} {v:10.3f}   -- {exc}")
            continue

        compress_mm = 1000.0 * res["peak_compression"]
        min_len = res["min_length"] - p.r          # hip-to-arc at max compression
        try:
            theta_min = np.rad2deg(leg_map.theta_for(min_len))
        except g2c.WorkspaceViolation:
            theta_min = float("nan")

        f_leg = res["peak_grf_mag"] / N_LEGS
        tau = f_leg * dl_dtheta / 2.0
        duty = res["stance_time"] / res["period"]
        ok = (theta_min > THETA_MIN_DEG) and (tau < MOTOR_LIMIT)
        if ok:
            usable.append((apex_mm, v, duty, tau))

        print(f"{apex_mm:9d} {v:10.3f} {res['stance_time']:10.4f} "
              f"{res['flight_time']:10.4f} {duty:6.3f} {compress_mm:13.1f} "
              f"{theta_min:10.2f} {res['peak_grf_mag']/(MASS*G):7.2f} "
              f"{tau:8.2f} {'yes' if ok else 'NO':>4}")

    print()
    if usable:
        print("Usable hop heights (workspace and torque both satisfied):")
        for apex_mm, v, duty, tau in usable:
            print(f"  {apex_mm:3d} mm : touchdown {v:.2f} m/s, duty {duty:.3f}, "
                  f"peak torque {tau:.1f} N.m ({100*tau/MOTOR_LIMIT:.0f}%)")
    else:
        print("No hop height satisfied both constraints.")
    print()


if __name__ == "__main__":
    main()
