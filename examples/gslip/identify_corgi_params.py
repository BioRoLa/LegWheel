"""Phase 0 of the G-SLIP port: identify the Corgi's G-SLIP parameters.

Maps the Corgi leg-wheel onto the G-SLIP model of Lu & Lin 2024
(Bioinspir. Biomim. 19 026017) and reports the dimensionless parameters
alongside the paper's Table 2 (four special cases) and Table 4 (optima).

The Corgi's foot is three circular tread arcs sharing one effective contact
radius (see LegModel.rim_point): the foot arc about O_r and the upper arcs
about U_l / U_r. So the rolling radius r is a hardware constant and the only
free geometric quantity is l0, the hip-to-arc-center distance, which theta
controls.

Run:
    uv run python examples/gslip/identify_corgi_params.py
"""

import numpy as np

from legwheel.config import RobotParams
from legwheel.models.leg_model import LegModel

# Measured on scales, standing (kg): FL 7.6, RL 7.4, RR 7.7, FR 7.3
MASS_PER_CORNER = {"front_left": 7.6, "rear_left": 7.4, "rear_right": 7.7, "front_right": 7.3}
MASS_TOTAL = sum(MASS_PER_CORNER.values())
G = 9.81

# Relative leg stiffness index, k_rel = (F_peak/mg) / (dl/l0).
# Animals and dynamic robots span 7-27; the paper's optimized leg is 18.
K_REL_RANGE = (7.0, 27.0)
K_REL_PAPER = 18.0

N_LEGS = 4  # pronk: all four legs in phase -> one virtual leg


def arc_centers(leg: LegModel, theta: float, beta: float = 0.0) -> dict:
    """Hip-to-arc-center vectors for the three tread arcs, in the leg frame.

    The hip is the origin. Returns complex vectors so magnitude and bearing
    both fall out directly.
    """
    leg.forward(theta, beta, vector=False)
    return {
        "O_r (foot arc)": complex(leg.O_r),
        "U_l (upper L)": complex(leg.U_l),
        "U_r (upper R)": complex(leg.U_r),
    }


def report_geometry(leg: LegModel) -> None:
    r_eff = leg.foot_radius
    print("=" * 78)
    print("HIP -> ARC-CENTER GEOMETRY  (leg frame, beta = 0)")
    print("=" * 78)
    print(f"effective contact radius r_eff = {r_eff:.4f} m  (constant across all three arcs)")
    print(f"  = TIRE_TREAD_RADIUS {RobotParams.TIRE_TREAD_RADIUS} + TIRE_CORNER_RADIUS "
          f"{RobotParams.TIRE_CORNER_RADIUS}")
    print()
    header = f"{'theta[deg]':>10}"
    for name in ("O_r (foot arc)", "U_l (upper L)", "U_r (upper R)"):
        header += f" | {name + ' l0':>20} {'bearing':>9}"
    print(header)
    print("-" * len(header))

    for theta_deg in (17, 30, 45, 60, 75, 90, 110, 130, 150, 160):
        row = f"{theta_deg:10.0f}"
        for name, vec in arc_centers(leg, np.deg2rad(theta_deg)).items():
            row += f" | {abs(vec):20.4f} {np.rad2deg(np.angle(vec)):9.2f}"
        print(row)
    print()


def report_dimensionless(leg: LegModel) -> None:
    """Dimensionless G-SLIP parameters, paper eqs 15-16."""
    r_eff = leg.foot_radius
    print("=" * 78)
    print("DIMENSIONLESS PARAMETERS  (paper eqs 15-16)")
    print("=" * 78)
    print("  r~ = r / (l0 + r),  l0~ = l0 / (l0 + r)")
    print("  Paper Table 4 optimum (combined indicator): r~ = 0.6303")
    print("  Paper Table 2 special cases: SLIP/TSL r~ = 0, SLIP-RF/R-SLIP r~ = 0.5")
    print()
    print(f"{'theta[deg]':>10} {'arc':>16} {'l0[m]':>9} {'l0+r[m]':>9} {'r~':>8} {'l0~':>8}")
    print("-" * 66)
    for theta_deg in (17, 45, 60, 75, 90, 110, 130, 160):
        for name, vec in arc_centers(leg, np.deg2rad(theta_deg)).items():
            l0 = abs(vec)
            print(f"{theta_deg:10.0f} {name:>16} {l0:9.4f} {l0 + r_eff:9.4f} "
                  f"{r_eff / (l0 + r_eff):8.4f} {l0 / (l0 + r_eff):8.4f}")
    print()


def report_stiffness(leg: LegModel) -> None:
    """Virtual spring stiffness from the relative-leg-stiffness index.

    k_rel = (F_peak/mg) / (dl/l0) and F_peak = k*dl collapse to
        k_virtual = k_rel * m * g / l0
    """
    mg = MASS_TOTAL * G
    print("=" * 78)
    print("VIRTUAL SPRING STIFFNESS")
    print("=" * 78)
    print(f"m = {MASS_TOTAL:.1f} kg   mg = {mg:.1f} N   static load per leg = {mg / N_LEGS:.1f} N")
    print(f"k_virtual = k_rel * m * g / l0,   k_leg = k_virtual / {N_LEGS} (pronk)")
    print()
    print(f"{'theta[deg]':>10} {'arc':>16} {'l0[m]':>9} "
          f"{'k_virt@18':>12} {'k_leg@18':>11} {'k_leg range':>22}")
    print("-" * 84)
    for theta_deg in (45, 60, 75, 90, 110):
        for name, vec in arc_centers(leg, np.deg2rad(theta_deg)).items():
            l0 = abs(vec)
            k_virt = K_REL_PAPER * mg / l0
            lo = K_REL_RANGE[0] * mg / l0 / N_LEGS
            hi = K_REL_RANGE[1] * mg / l0 / N_LEGS
            print(f"{theta_deg:10.0f} {name:>16} {l0:9.4f} "
                  f"{k_virt:12.0f} {k_virt / N_LEGS:11.0f} {f'{lo:.0f} - {hi:.0f}':>22}")
    print()
    print("Reference: exp_sim_stay.cpp stands the robot with kx = ky = 2000 N/m.")
    print()


def report_findings(leg: LegModel) -> None:
    """Consequences of the geometry for the G-SLIP mapping."""
    from scipy.optimize import brentq

    r_eff = leg.foot_radius

    def l0_foot(theta: float) -> float:
        leg.forward(theta, 0.0, vector=False)
        return abs(complex(leg.O_r))

    print("=" * 78)
    print("FINDINGS")
    print("=" * 78)

    # 1. Is the foot-arc center always on the leg axis?
    bearings = []
    for theta_deg in np.linspace(20, 160, 30):
        leg.forward(np.deg2rad(theta_deg), 0.0, vector=False)
        bearings.append(np.rad2deg(np.angle(complex(leg.O_r))))
    spread = max(bearings) - min(bearings)
    print(f"1. Foot-arc (O_r) bearing over theta in [20,160]: "
          f"{min(bearings):.4f} to {max(bearings):.4f} deg (spread {spread:.2e})")
    print("   -> the foot-arc center lies exactly on the leg axis, so contact on the")
    print("      foot arc makes the Corgi a TELESCOPING leg with a rolling circular")
    print("      foot. That is the SLIP-RF special case, not R-SLIP.")
    print()

    # 2. Nominal theta that hits the paper's optimal dimensionless morphology
    r_tilde_target = 0.6303
    theta_star = brentq(
        lambda t: r_eff / (l0_foot(t) + r_eff) - r_tilde_target,
        np.deg2rad(20), np.deg2rad(160),
    )
    l0_star = l0_foot(theta_star)
    print(f"2. Paper's combined-indicator optimum is r~ = {r_tilde_target}.")
    print(f"   Corgi reaches it at theta = {np.rad2deg(theta_star):.2f} deg "
          f"(l0 = {l0_star:.4f} m, leg length l0+r = {l0_star + r_eff:.4f} m).")
    print("   -> the optimal dimensionless morphology is reachable by choosing")
    print("      nominal theta; no hardware change needed.")
    print()

    mg = MASS_TOTAL * G
    print(f"   At that pose, k_virtual = k_rel*mg/l0:")
    for k_rel, label in ((K_REL_RANGE[0], "lower"), (K_REL_PAPER, "paper"), (K_REL_RANGE[1], "upper")):
        k_v = k_rel * mg / l0_star
        print(f"     k_rel={k_rel:5.1f} ({label:5s}): k_virtual = {k_v:8.0f} N/m, "
              f"k_leg = {k_v / N_LEGS:8.0f} N/m")
    print()

    # 3. How much stance is available before the contact leaves the foot arc
    span_deg = 80.0  # rim_point(): foot arc covers alpha in [-40, +40]
    roll_len = r_eff * np.deg2rad(span_deg)
    print(f"3. Foot arc spans alpha in [-40, +40] deg ({span_deg:.0f} deg total).")
    print(f"   Max rolling stance length on the foot arc alone: "
          f"r*dalpha = {roll_len:.4f} m.")
    print("   -> keep the whole stance inside this budget and the contact radius")
    print("      never changes and the leg stays telescoping (exact SLIP-RF).")
    print("      Exceeding it rolls onto U_l/U_r, whose centers are off-axis, and")
    print("      the model degenerates to the general G-SLIP.")
    print()


def main() -> None:
    leg = LegModel()

    print()
    print(f"Corgi -> G-SLIP parameter identification")
    print(f"mass by corner (kg): {MASS_PER_CORNER}")
    front = MASS_PER_CORNER["front_left"] + MASS_PER_CORNER["front_right"]
    rear = MASS_PER_CORNER["rear_left"] + MASS_PER_CORNER["rear_right"]
    left = MASS_PER_CORNER["front_left"] + MASS_PER_CORNER["rear_left"]
    right = MASS_PER_CORNER["front_right"] + MASS_PER_CORNER["rear_right"]
    com_x = (rear - front) / MASS_TOTAL * RobotParams.WHEEL_BASE / 2.0
    print(f"total {MASS_TOTAL:.1f} kg | front {front:.1f} rear {rear:.1f} "
          f"| left {left:.1f} right {right:.1f}")
    print(f"COM offset from wheelbase midpoint: {com_x * 1000:+.1f} mm (positive = rearward)")
    print()

    report_geometry(leg)
    report_dimensionless(leg)
    report_stiffness(leg)
    report_findings(leg)


if __name__ == "__main__":
    main()
