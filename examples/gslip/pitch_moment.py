"""Does the TROT cancel the pronk's pitch moment? (Phase D follow-on.)

Motivation: measured in Webots, the pronk's legs disagree about contact state
32-45% of the time on every forward rung (1.9% during the hop), and when they
disagree they split FRONT/REAR. A pitch oscillation is the obvious suspect.
The hope was that a trot -- one front leg and one rear leg down at a time --
would put the fore/aft foot offsets on opposite sides of the COM and cancel it.

KEY POINT ABOUT WHERE THE MOMENT COMES FROM

A first estimate of "F * d", treating the ground reaction as VERTICAL acting at
a foot displaced by d, is wrong. The leg force acts along the leg axis, and the
component of position from hip to foot is antiparallel to it, so it contributes
no moment. Taking moments about the COM:

    tau = sum_i (r_foot_i - r_com) x F_i
        = sum_i (r_hip_i - r_com) x F_i        (the hip->foot part drops out)

With the leg tilted by phi and the force along it, F_i = F_i * (-sin phi, 0, cos phi):

    tau_y,i = dz * (-F_i sin phi)  -  x_i * (F_i cos phi)

so the moment depends on the HIP-to-COM offset, not on the foot offset directly.

Run:
    uv run python examples/gslip/pitch_moment.py
"""

import numpy as np

MASS, G = 30.0, 9.81
# Hip (module mount) positions in the body frame, from corgi_driver.py:481
# and the proto's module translations. A=FL, B=FR, C=RR, D=RL.
HIPS = {"A": (+0.255, +0.12), "B": (+0.255, -0.12),
        "C": (-0.255, -0.12), "D": (-0.255, +0.12)}
HIP_Z = 0.057166          # module mount height in the robot frame (proto)
PEAK_GRF_BW = 3.64        # from the v~1.2 fixed point
BETA_MAX_DEG = 18.25


def pitch_moment(stance_legs, phi, dz, total_force):
    """Net pitch moment about the COM, N.m. Force along the leg axis."""
    f = total_force / len(stance_legs)
    tau = 0.0
    for leg in stance_legs:
        x, _y = HIPS[leg]
        # tau_y = r_z * F_x - r_x * F_z, with F along the leg axis
        tau += dz * (-f * np.sin(phi)) - x * (f * np.cos(phi))
    return tau


def main() -> None:
    F = PEAK_GRF_BW * MASS * G
    print()
    print("=" * 74)
    print("PITCH MOMENT ABOUT THE COM   peak GRF "
          f"{F:.0f} N ({PEAK_GRF_BW} BW)")
    print("=" * 74)
    print(f"  hips at x = +-0.255 m, z = {HIP_Z:.4f} m")
    print("  moment = sum_i (r_hip_i - r_com) x F_i, force along the leg axis")

    # dz is the hip-to-COM vertical offset. It is not precisely known, so sweep
    # a plausible range rather than pretend to a single value.
    print()
    print(f"{'dz (m)':>8} {'beta':>7} {'PRONK tau':>11} {'TROT AC':>10} "
          f"{'TROT BD':>10} {'trot/pronk':>11}")
    for dz in (0.0, 0.02, 0.05, 0.10):
        for beta_deg in (0.0, BETA_MAX_DEG):
            phi = np.deg2rad(beta_deg)
            p = pitch_moment(["A", "B", "C", "D"], phi, dz, F)
            t1 = pitch_moment(["A", "C"], phi, dz, F)
            t2 = pitch_moment(["B", "D"], phi, dz, F)
            ratio = (t1 / p) if abs(p) > 1e-9 else float("nan")
            print(f"{dz:8.3f} {beta_deg:7.2f} {p:11.2f} {t1:10.2f} "
                  f"{t2:10.2f} {ratio:11.2f}")

    print()
    print("=" * 74)
    print("VERDICT")
    print("=" * 74)
    print("  sum of hip x over the stance legs:")
    for name, legs in (("pronk (A,B,C,D)", ["A", "B", "C", "D"]),
                       ("trot  (A,C)", ["A", "C"]),
                       ("trot  (B,D)", ["B", "D"])):
        sx = sum(HIPS[l][0] for l in legs)
        print(f"    {name:<18} sum x = {sx:+.3f} m")
    print()
    print("  Both gaits have sum(x) = 0, so the cos(phi) term cancels in BOTH.")
    print("  What remains is  tau = -F * dz * sin(phi)  -- identical for the")
    print("  pronk and for either trot pair, because it depends on the TOTAL")
    print("  force and the hip-to-COM height, not on how many legs share it.")
    print()
    print("  => THE TROT DOES NOT CANCEL THE PITCH MOMENT.")
    print()
    print("  It is also much smaller than a naive 'F * d' estimate: at dz = 0.05")
    print(f"  and beta = {BETA_MAX_DEG} deg it is "
          f"{abs(pitch_moment(['A','B','C','D'], np.deg2rad(BETA_MAX_DEG), 0.05, F)):.1f} N.m, "
          "not ~80 N.m.")
    print()
    print("  So a pure axial leg force does NOT explain the measured front/rear")
    print("  contact split. The remaining candidate is the TANGENTIAL stiffness")
    print("  k_t: force_control commands k_t e_t e_t^T, i.e. a force component")
    print("  PERPENDICULAR to the leg, which does not pass through the hip and")
    print("  therefore does carry a moment. That is where to look next.")
    print()


if __name__ == "__main__":
    main()
