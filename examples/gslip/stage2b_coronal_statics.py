"""Does the coronal contact geometry reproduce section 33's measured statics?

Stage 2b Module 2 validation, NO SIMULATOR. Section 33 leaned the robot in
wheeled mode (uniform camber, gamma = lambda * {+1,-1,-1,+1}) and measured, in
Webots, two things this project's sagittal template provably cannot produce
(section 42: ride height is a body-roll observable):

    lambda (cmd)   body roll     ride drop
        10 deg      -4.58 deg      2.00 mm
        20 deg      -9.16 deg      4.34 mm
        30 deg     -13.8  deg      6.45 mm
    and the mirror: +9.15 deg at lambda = -20.

THE MECHANISM BEING TESTED

Each wheel plane sits WHEEL_AXIAL_OFFSET = 91.7 mm outboard along the hip axis.
The uniform-camber pattern leans both wheels the same WORLD direction, so the
left wheel centre swings DOWN by ~a*sin(lambda) while the right swings UP -- and
a rigid body on two contacts must roll until both touch. Roll is therefore
first-order in lambda (the sin-scaling section 42 identified), and the ride
drop follows from the rolled geometry plus the Stage 0 hub-clearance model

    h(phi) = R_tread*cos(phi) - w_flat*sin(phi) + r_corner .

This is a pure 2-unknown geometry solve -- no forces, no dynamics, no free
parameters. Wheel mode is rigid, all radii are the wheel's, and every constant
is measured. If the numbers land, the coronal contact seam is validated against
the simulator's own statics before any cambered fixed point rides on it.

One measured correction applied: section 33 found the ABAD undershoots command
by 1.5 / 3.0 / 4.0 deg at 10 / 20 / 30, so the ACHIEVED lean drives the
geometry; both commanded and achieved columns are printed.

Run:
    uv run python examples/gslip/stage2b_coronal_statics.py
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import fsolve

# Stage 0 constants, wheeled mode (theta = 17 deg: every rim centre coincides
# with the hip, so the "leg" is just the wheel).
W_HIP = 0.120           # hip lateral offset (m)
A_OFF = 0.091675        # wheel plane outboard along the hip axis (m)
R_TREAD, W_FLAT, R_CORNER = 0.130, 0.005, 0.015

MEASURED = {  # lambda_cmd_deg: (achieved_deg, roll_deg, drop_mm)
    10.0: (8.5, 4.58, 2.00),
    20.0: (17.0, 9.16, 4.34),
    30.0: (26.0, 13.8, 6.45),
}


def hub_clearance(phi: float) -> float:
    """Wheel-centre height above ground at lean phi -- Stage 0's tread model."""
    return R_TREAD * np.cos(phi) - W_FLAT * np.sin(abs(phi)) + R_CORNER


def wheel_centres_body(lean: float):
    """(y, z) of each wheel centre in the body coronal frame at command `lean`.

    The uniform pattern leans both wheels the same world direction: the offset
    vector (a along the hip axis, outboard) rotates about the fore-aft axis, so
    one centre rises and the other falls -- the asymmetry that forces the roll.
    """
    c, s = np.cos(lean), np.sin(lean)
    left = (+W_HIP + A_OFF * c, +A_OFF * s)
    right = (-W_HIP - A_OFF * c, +A_OFF * s * -1.0)
    return left, right


def solve_pose_asym(lean_left: float, lean_right: float):
    """Body (z, rho) for PER-SIDE world-sense leans. 2x2, exact.

    This is the solve the symmetric version cannot do, and the reason the
    0.80-0.94 validation bracket exists: section 39 showed the achieved lean is
    a LEFT/RIGHT SPLIT (left pair outboard ~3.8 deg, right pair inboard ~1.9 at
    lambda = 30, kp 90), so feeding one number to both sides mis-states the
    geometry. Leans are world-sense: for the lr pattern, left = +mean(gamma_A,
    gamma_D), right = -mean(gamma_B, gamma_C).
    """
    c_l, s_l = np.cos(lean_left), np.sin(lean_left)
    c_r, s_r = np.cos(lean_right), np.sin(lean_right)
    pl = (+W_HIP + A_OFF * c_l, +A_OFF * s_l)
    pr = (-W_HIP - A_OFF * c_r, -A_OFF * s_r)

    def residual(x):
        z, rho = x
        cr, sr = np.cos(rho), np.sin(rho)
        return [
            z + pl[0] * sr + pl[1] * cr - hub_clearance(lean_left + rho),
            z + pr[0] * sr + pr[1] * cr - hub_clearance(lean_right + rho),
        ]

    z, rho = fsolve(residual, [hub_clearance(0.0), 0.0], full_output=False)
    return float(z), float(rho)


def solve_pose(lean: float):
    """Body (z, rho) at a uniform world-sense lean -- both sides equal."""
    return solve_pose_asym(lean, lean)


def main() -> None:
    print(__doc__.split("Run:")[0].rstrip())
    print()
    z0, _ = solve_pose(0.0)
    print(f"baseline CoM height (lambda = 0): {z0 * 1e3:8.2f} mm")
    print()
    hdr = (f"{'cmd':>5} {'achieved':>9} | {'roll pred':>9} {'roll meas':>9} "
           f"{'ratio':>6} | {'drop pred':>9} {'drop meas':>9} {'ratio':>6}")
    print(hdr)
    print("-" * len(hdr))
    for cmd, (ach, roll_meas, drop_meas) in MEASURED.items():
        z, rho = solve_pose(np.deg2rad(ach))
        roll_pred = abs(np.rad2deg(rho))
        drop_pred = (z0 - z) * 1e3
        print(f"{cmd:4.0f}d {ach:8.1f}d | {roll_pred:8.2f}d {roll_meas:8.2f}d "
              f"{roll_pred / roll_meas:6.2f} | {drop_pred:8.2f} {drop_meas:8.2f} "
              f"{drop_pred / drop_meas:6.2f}")

    # Mirror check: the measured roll flips sign to within 0.01 deg.
    z_p, rho_p = solve_pose(np.deg2rad(+17.0))
    z_m, rho_m = solve_pose(np.deg2rad(-17.0))
    print()
    print(f"mirror: rho(+17) = {np.rad2deg(rho_p):+.3f}, "
          f"rho(-17) = {np.rad2deg(rho_m):+.3f}  "
          f"(measured -9.16 / +9.15)")


if __name__ == "__main__":
    main()
