"""Pitch moment from the FULL commanded impedance, including the tangential term.

Established in pitch_moment.py: a purely AXIAL leg force cannot explain the
measured front/rear contact split. Taking moments about the COM, the hip->foot
arm is antiparallel to an axial force and drops out, leaving only
tau = -F*dz*sin(phi) -- 6.7-33.6 N.m, and identical for the pronk and either
trot diagonal pair.

The tangential term is different. force_control commands

    K = k_r e_r e_r^T + k_lat e_y e_y^T + k_t e_t e_t^T

and a force along e_t is PERPENDICULAR to the leg, so the hip->foot arm does
NOT drop out: it contributes L * F_t directly, with L the hip-to-contact
distance (~0.29 m). That is a long lever, and it is the term that scales with
beta -- which matches the trigger, since the split appears exactly when beta
becomes non-zero.

Run:
    uv run python examples/gslip/pitch_moment_kt.py
"""

import numpy as np

MASS, G = 30.0, 9.81
HIPS = {"A": (+0.255, +0.12), "B": (+0.255, -0.12),
        "C": (-0.255, -0.12), "D": (-0.255, +0.12)}
HIP_Z = 0.057166
PEAK_GRF_BW = 3.64
BETA_MAX_DEG = 18.25
L_HIP_TO_CONTACT = 0.293      # standing hip height = hip-to-contact distance
K_TANGENTIAL = 1200.0         # N/m per leg, gslip_pronk.launch.py default


def moment(stance_legs, phi, dz, f_axial_total, f_tang_per_leg):
    """Net pitch moment about the COM, N.m, from axial + tangential force.

    Works in the sagittal plane. e_r points hip->contact, e_t is perpendicular
    to it. Vectors are (x, z); the pitch moment is the scalar cross product
    r_x * F_z - r_z * F_x taken with the sign convention used throughout.
    """
    e_r = np.array([np.sin(phi), -np.cos(phi)])   # hip -> contact, pointing down
    e_t = np.array([np.cos(phi), np.sin(phi)])    # perpendicular, in-plane
    f_ax = f_axial_total / len(stance_legs)

    tau = 0.0
    for leg in stance_legs:
        x, _y = HIPS[leg]
        r_hip = np.array([x, HIP_Z - dz])         # hip relative to COM
        r_foot = r_hip + L_HIP_TO_CONTACT * e_r

        # axial force pushes the body along -e_r (ground pushes up the leg)
        F = -f_ax * e_r + f_tang_per_leg * e_t
        tau += r_foot[0] * F[1] - r_foot[1] * F[0]
    return tau


def main() -> None:
    F = PEAK_GRF_BW * MASS * G
    dz = 0.05          # hip-to-COM offset; the mid case from pitch_moment.py
    phi = np.deg2rad(BETA_MAX_DEG)

    print()
    print("=" * 76)
    print("PITCH MOMENT WITH THE TANGENTIAL TERM")
    print("=" * 76)
    print(f"  peak GRF {F:.0f} N,  hip-to-contact L = {L_HIP_TO_CONTACT} m,"
          f"  k_t = {K_TANGENTIAL:.0f} N/m per leg")
    print(f"  beta = {BETA_MAX_DEG} deg,  hip-to-COM dz = {dz} m")

    print()
    print("  tangential force comes from tracking error perpendicular to the leg.")
    print("  A beta tracking error of d_beta gives roughly L*sin(d_beta) of it.")
    print()
    print(f"{'d_beta':>8} {'lateral err':>12} {'F_t/leg':>9} "
          f"{'PRONK tau':>11} {'TROT tau':>10} {'axial only':>11}")
    axial_only = moment(["A", "B", "C", "D"], phi, dz, F, 0.0)
    for dbeta_deg in (0.0, 2.0, 5.0, 10.0, 15.0):
        err = L_HIP_TO_CONTACT * np.sin(np.deg2rad(dbeta_deg))
        f_t = K_TANGENTIAL * err
        p = moment(["A", "B", "C", "D"], phi, dz, F, f_t)
        # a trot leg carries twice the load, so twice the deflection and force
        t = moment(["A", "C"], phi, dz, F, 2.0 * f_t)
        print(f"{dbeta_deg:8.1f} {err:12.4f} {f_t:9.1f} {p:11.2f} {t:10.2f} "
              f"{axial_only:11.2f}")

    print()
    print("  measured beta tracking shortfall at v~1.20: commanded 36 deg p2p,")
    print("  left legs reach 33 (92%), right legs 23-25 (63-70%) -- so d_beta")
    print("  of order 5-13 deg is realistic, and the phase lag is ~28 ms of a")
    print("  ~98 ms stance, another ~10 deg of instantaneous error.")

    print()
    print("=" * 76)
    print("SENSITIVITY TO k_tangential")
    print("=" * 76)
    print(f"{'k_t':>8} {'F_t/leg':>9} {'PRONK tau':>11}   (at d_beta = 10 deg)")
    err = L_HIP_TO_CONTACT * np.sin(np.deg2rad(10.0))
    for kt in (0.0, 300.0, 600.0, 1200.0, 2400.0):
        f_t = kt * err
        p = moment(["A", "B", "C", "D"], phi, dz, F, f_t)
        print(f"{kt:8.0f} {f_t:9.1f} {p:11.2f}")

    print()
    print("=" * 76)
    print("VERDICT")
    print("=" * 76)
    print("  The tangential term dominates: at a realistic 10 deg of beta")
    print("  tracking error it contributes far more pitch moment than the axial")
    print("  term, because it acts on the FULL hip-to-contact lever (0.29 m)")
    print("  rather than only the hip-to-COM offset (0.05 m).")
    print()
    print("  It is still NOT gait-dependent: a trot leg carries twice the load,")
    print("  so twice the deflection and twice the force, over half as many")
    print("  legs. The trot does not escape this either.")
    print()
    print("  BUT it IS controllable: the moment is linear in k_tangential, and")
    print("  k_t = 1200 N/m was never tuned against this effect. Lowering it is")
    print("  the first cheap, testable lever found for the leg desynchronisation.")
    print()


if __name__ == "__main__":
    main()
