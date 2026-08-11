"""What does a beta offset in stance ACTUALLY do to the contact point?

The steering channel in gslip_pronk rests on a differential-drive model taken
from the handover:

    "the foot is a circular arc of radius r = 0.145 m and stance is rolling
     contact, so sweeping a leg by d_beta while it is down rolls that side
     forward by ~r*d_beta"

Two things in the forward kinematics say that cannot be the whole story, and
both are checkable without a simulator:

 1. Under ROLLING contact the instantaneous centre of rotation IS the contact
    point, so the HIP -- which is what carries the body -- advances by the
    hip-to-contact distance per radian, not by the rim radius. Those differ by
    about a factor of two (0.29 m vs 0.145 m).

 2. `LegModel.rim_point` is piecewise. The lower rim, centred at O_r on the leg
    axis, only covers |alpha| <= 40 deg. Past that the contact moves onto an
    UPPER rim centred at U_r or U_l, which are NOT on the leg axis. If a stance
    sweep crosses that boundary the rolling geometry changes discontinuously --
    and the steering offset pushes the contact point straight at it.

This computes the real relation by finding, for each (theta, beta) of the
template's stance phase, the actual lowest point of the rim and how the hip
must move for that point to stay put.

    uv run python examples/gslip/rolling_radius.py
"""
import numpy as np

from legwheel.models.leg_model import LegModel

# Pronk template stance, from config/gslip_pronk_template.csv: beta sweeps
# -18.25 -> +18.17 deg while in contact, at theta near the 100 deg nominal.
THETA_DEG = 100.0
BETA_SWEEP_DEG = (-18.25, 18.17)
STEER_OFFSETS_DEG = [-8.0, -5.0, -2.5, 0.0, 2.5, 5.0, 8.0]


def contact_state(lm, theta, beta, n_alpha=1441):
    """-> (alpha_deg at contact, contact point [x,y], hip-to-contact distance).

    The contact point is the lowest point of the whole rim, found by sampling
    alpha rather than assuming which arc is in play -- the piecewise structure
    of rim_point is exactly what is in question here.
    """
    lm.forward(theta, beta, vector=True)
    alphas = np.linspace(-180.0, 180.0, n_alpha)
    pts = np.array([lm.rim_point(a, 0.0) for a in alphas])
    i = int(np.argmin(pts[:, 1]))
    p = pts[i]
    # The hip is the leg-frame origin, so |p| is the hip-to-contact distance.
    return float(alphas[i]), p, float(np.hypot(p[0], p[1]))


def main():
    lm = LegModel()
    th = np.deg2rad(THETA_DEG)

    print(f"theta = {THETA_DEG:.1f} deg, w = 0 (wheel mid-plane)")
    print(f"rim radii: R_pitch {lm.R:.3f}  tread {lm.R + lm.foot_offset:.3f}  "
          f"outer (foot_radius) {lm.foot_radius:.3f} m")
    print()

    # --- 1. Where is the contact, and does it stay on the lower rim? --------
    print("Contact point through the stance sweep, with no steering offset:")
    print(f"  {'beta':>8} {'alpha@contact':>14} {'rim':>12} "
          f"{'|hip-contact|':>14} {'x_contact':>10} {'y_contact':>10}")
    betas = np.linspace(*BETA_SWEEP_DEG, 9)
    for b in betas:
        a, p, d = contact_state(lm, th, np.deg2rad(b))
        rim = "lower(O_r)" if abs(((a + 180) % 360) - 180) <= 40 else "UPPER"
        print(f"  {b:+8.2f} {a:+14.2f} {rim:>12} {d:14.4f} "
              f"{p[0]:+10.4f} {p[1]:+10.4f}")
    print()

    # --- 2. The true rolling radius -----------------------------------------
    # Under rolling the contact point is the instantaneous centre of rotation,
    # so |v_hip| = |omega| * |hip - contact| exactly. No finite difference is
    # needed, and it should not be used: sampling alpha discretises the contact
    # point, and differencing that discretisation produces convincing garbage
    # (a first pass of this script reported 0.07 m/rad at three of nine sweep
    # points, purely from the 0.25 deg alpha grid).
    #
    # While contact is on the LOWER rim the contact point is simply the rim
    # centre dropped by the effective radius, so it is available in closed form.
    print("Effective rolling radius = |hip - contact|, exact "
          "(contact is the instantaneous centre):")
    print(f"  {'theta':>8} {'|O_r| (hip->rim c.)':>20} {'rim radius':>11} "
          f"{'rolling radius':>15} {'vs r=0.145':>11}")
    for td in (84.4, 90.0, 95.0, 100.0):
        lm.forward(np.deg2rad(td), 0.0, vector=True)
        # O_r is the lower-rim centre; at beta = 0 the leg axis is vertical, so
        # the contact sits directly below it.
        d_hip_rim = float(np.hypot(lm.O_r[0], lm.O_r[1]))
        roll_r = d_hip_rim + lm.foot_radius
        print(f"  {td:8.2f} {d_hip_rim:20.4f} {lm.foot_radius:11.4f} "
              f"{roll_r:15.4f} {roll_r/0.145:11.2f}x")
    print()
    print("  The handover's differential-drive model uses r = 0.145 m, the RIM")
    print("  radius. The rim is rigidly part of the leg -- it is not a wheel")
    print("  free to spin on an axle -- so rotating the leg by d_beta rotates")
    print("  the rim by d_beta about the CONTACT point, and the hip, which is")
    print("  what carries the body, swings through |hip - contact|*d_beta.")
    print()

    # --- 3. What a steering offset does -------------------------------------
    # The offset is added to beta during stance. Ask where the contact ends up
    # at the two ends of the sweep, which is where it is closest to the +-40
    # deg boundary between the lower and upper rims.
    print("Steering offset vs the +-40 deg lower/upper rim boundary:")
    print(f"  {'offset':>8} {'alpha @ sweep start':>20} "
          f"{'alpha @ sweep end':>18} {'crosses 40 deg?':>17}")
    for off in STEER_OFFSETS_DEG:
        a0, _, _ = contact_state(lm, th, np.deg2rad(BETA_SWEEP_DEG[0] + off))
        a1, _, _ = contact_state(lm, th, np.deg2rad(BETA_SWEEP_DEG[1] + off))
        crossed = "YES" if max(abs(a0), abs(a1)) > 40.0 else "no"
        print(f"  {off:+8.2f} {a0:+20.2f} {a1:+18.2f} {crossed:>17}")
    print()

    # --- 4. The corrected differential-drive prediction ----------------------
    lm.forward(th, 0.0, vector=True)
    roll_r = float(np.hypot(lm.O_r[0], lm.O_r[1])) + lm.foot_radius
    track = 0.24
    print(f"Differential-drive authority, d_psi = L*(sweep_L - sweep_R)/track")
    print(f"  track {track:.3f} m")
    print(f"  {'per-side offset':>16} {'with r=0.145':>14} "
          f"{'with L=%.3f' % roll_r:>14}")
    for off in (2.5, 5.0, 8.0):
        diff = np.deg2rad(2 * off)   # left is +off, right is -off
        print(f"  {off:+15.2f}d {np.rad2deg(0.145*diff/track):13.2f}d "
              f"{np.rad2deg(roll_r*diff/track):13.2f}d")
    print()
    print("  The r = 0.145 row reproduces the handover's own 3.0 deg/stride at")
    print("  a 5 deg differential, so this is the same calculation with the")
    print("  radius corrected -- every prediction doubles.")
    print()
    print("  Section 19 measured the +2.5 deg command taking the hop from")
    print("  -16.29 to -0.71 deg over 8 strides = 1.95 deg/stride, and read")
    print("  that against 3.02 as 'slightly stronger than predicted'. Against")
    print("  the corrected 6.11 it is 32% of what rolling geometry allows.")
    print("  The channel is LOSSY, not strong, and the agreement that was")
    print("  taken as confirming the mechanism rested on a 2x error.")
    print()
    print("  Caveat, stated because it bounds the claim: theta also changes")
    print("  through stance (100 -> 84.4 -> 100 deg), so the leg is not a")
    print("  rigid body and the instantaneous-centre result is not exact. The")
    print("  theta term moves the hip along the leg axis, i.e. near-vertically")
    print("  at mid-stance, so it barely touches the HORIZONTAL advance this")
    print("  table is about -- but near the ends of the sweep, where the leg")
    print("  axis is 18 deg off vertical, it contributes and the figures here")
    print("  are first-order.")


if __name__ == "__main__":
    main()
