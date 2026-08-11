"""How much does the flat spot in the current tread actually cost?

The wheel today is a flat tread band with rounded shoulders; a fully round
(toroidal) tread is planned. This compares the two, so the decision to test
camber steering before or after the retread rests on numbers.

Cross-sections, both on a 40 mm wide wheel with 145 mm outer radius:

  CURRENT   flat at rho = 145 for |w| <= 5 mm, then a 15 mm fillet centred at
            (w = +-5, rho = 130), reaching rho = 130 at the |w| = 20 mm edge.
  ROUND     a single 20 mm crown arc centred at (w = 0, rho = 125), i.e. a
            semicircular section across the full width.

Contact under camber lambda is found by tilting the section and taking the
lowest point. On the FLAT band the section is a straight line parallel to the
axis, so the moment it tilts the contact runs to the shoulder -- which is the
whole issue, and it is a discontinuity, not a small error.

    uv run python examples/gslip/crown_effect.py
"""
import numpy as np

R_OUT = 0.145        # outer radius, both profiles
HALF_W = 0.020       # WHEEL_THICKNESS / 2
R_FILLET = 0.015     # TIRE_CORNER_RADIUS
W_FLAT = HALF_W - R_FILLET          # 0.005: half-width of the flat band
R_CROWN = HALF_W                    # a fully round section on the same width
AXIAL_OFFSET = 0.091675             # WHEEL_AXIAL_OFFSET: leg plane -> wheel plane

LAMBDAS = [0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 20.0, 30.0]


def contact_current(lam):
    """-> (w, rho) of the contact for the flat-with-fillet section.

    For lambda > 0 the flat band cannot carry the contact: a straight segment
    tilted off horizontal touches at its endpoint. Contact therefore sits on the
    shoulder fillet, whose centre is at (W_FLAT, R_OUT - R_FILLET), and rides
    around it at angle lambda.
    """
    if abs(lam) < 1e-12:
        return 0.0, R_OUT           # whole flat band in contact; w is undefined
    s = np.sign(lam)
    a = abs(np.deg2rad(lam))
    return (s * (W_FLAT + R_FILLET * np.sin(a)),
            (R_OUT - R_FILLET) + R_FILLET * np.cos(a))


def contact_round(lam):
    """-> (w, rho) for a single crown arc centred on the mid-plane."""
    a = np.deg2rad(lam)
    return (R_CROWN * np.sin(a), (R_OUT - R_CROWN) + R_CROWN * np.cos(a))


def turn_radius(rho, lam):
    """Cone-apex turn radius for a wheel of effective rolling radius rho
    cambered by lambda: the contact circle is centred where the spin axis
    meets the ground.  R = rho*cos^2(lam)/sin(lam).
    """
    a = np.deg2rad(lam)
    return rho * np.cos(a) ** 2 / np.sin(a)


def main():
    print("Contact position and effective rolling radius vs camber")
    print(f"{'lambda':>8} | {'CURRENT w':>10} {'rho':>8} {'R_turn':>8}"
          f" | {'ROUND w':>9} {'rho':>8} {'R_turn':>8} | {'w gap':>7}")
    for lam in LAMBDAS:
        wc, rc = contact_current(lam)
        wr, rr = contact_round(lam)
        print(f"{lam:8.1f} | {1000*wc:10.2f} {1000*rc:8.2f} "
              f"{turn_radius(rc, lam):8.3f}"
              f" | {1000*wr:9.2f} {1000*rr:8.2f} {turn_radius(rr, lam):8.3f}"
              f" | {1000*(wc-wr):7.2f}")
    print("  w in mm, rho in mm, R_turn in m")
    print()

    print("The discontinuity at lambda = 0 (current tread only):")
    for eps in (0.01, 0.1, 1.0):
        wp, _ = contact_current(+eps)
        wm, _ = contact_current(-eps)
        print(f"  lambda = +-{eps:5.2f} deg -> contact at w = {1000*wp:+.2f} / "
              f"{1000*wm:+.2f} mm, a {1000*(wp-wm):.1f} mm jump")
    print("  A round tread has no such jump: w = 20*sin(lambda), through zero.")
    print()

    print("Does path curvature actually scale with the crown radius?")
    print("  The contact circle is centred where the spin axis meets the")
    print("  ground, so R_turn = rho*cos^2(lam)/sin(lam), and the crown enters")
    print("  ONLY through rho = (R_out - r) + r*cos(lam):")
    for lam in (10.0, 20.0, 30.0):
        _, rc = contact_current(lam)
        _, rr = contact_round(lam)
        print(f"    lambda {lam:4.1f} deg: rho {1000*rc:6.2f} vs {1000*rr:6.2f} mm "
              f"-> R_turn {turn_radius(rc, lam):.3f} vs "
              f"{turn_radius(rr, lam):.3f} m "
              f"({100*abs(turn_radius(rc,lam)/turn_radius(rr,lam)-1):.1f}% apart)")
    print()

    print("Crown-driven lateral shift vs the shift from swinging the wheel")
    print(f"plane about the ABAD axis (WHEEL_AXIAL_OFFSET = "
          f"{1000*AXIAL_OFFSET:.1f} mm):")
    print(f"  {'lambda':>8} {'crown shift':>12} {'axial swing':>13} {'ratio':>8}")
    for lam in (5.0, 10.0, 20.0):
        wr, _ = contact_round(lam)
        swing = AXIAL_OFFSET * np.sin(np.deg2rad(lam))
        print(f"  {lam:8.1f} {1000*wr:12.2f} {1000*swing:13.2f} "
              f"{swing/wr if wr else float('nan'):8.1f}x")


if __name__ == "__main__":
    main()
