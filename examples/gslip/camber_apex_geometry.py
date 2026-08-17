"""Can a rigid multi-wheel body camber-steer at all? Apex geometry, no simulation.

Written 2026-08-13, after the confirming run (implementation log section 33)
found uniform camber steering the robot by 0.02% of its geometric turn in
wheeled mode with all four wheels down. This asks whether that null was an
accident of the test or is forced by the geometry.

THE ARGUMENT

A cambered wheel rolls without drilling only along the circle centred where its
SPIN AXIS meets the ground -- the cone apex. In body frame, wheel i has its hub
at (x_i, y_i, h) and, cambered by lambda_i about the fore-aft axis, a spin axis
along (0, cos lambda_i, sin lambda_i). Following that axis down to z = 0:

    apex_i = ( x_i ,  y_i - h * cot(lambda_i) )

Note what the x-coordinate does: NOTHING. The spin axis has no fore-aft
component, so the apex sits at the hub's own x, whatever the camber.

A rigid body in the plane has ONE instantaneous centre of rotation. Every wheel
demands the ICR sit at its own apex. So drill-free rolling on a curve needs all
apexes to coincide, and:

  * FRONT AND REAR CAN NEVER AGREE. Their apexes are pinned to x = +0.255 and
    x = -0.255 by the hub positions. No camber distribution moves them. With all
    four wheels down and no toe freedom, curved rolling is geometrically
    impossible -- not resisted, impossible.

  * A SINGLE LATERAL PAIR CAN AGREE. Both apexes share the pair's x, so they
    coincide when the y-coordinates match, which needs the two wheels at
    DIFFERENT camber angles:

        cot(lambda_outer) - cot(lambda_inner) = track / h

    This is Ackermann steering geometry, expressed in camber. The inner wheel
    leans more, exactly as it runs on the tighter circle.

CONSEQUENCE FOR THE THESIS: uniform camber is the wrong command. It is not
merely weak on this platform, it is inconsistent with rolling on every wheel at
once. The testable version is a lateral pair under Ackermann camber.

Run:
    uv run python examples/gslip/camber_apex_geometry.py
"""
import math

# Stage 0 geometry. TRACK is the CONTACT track -- all four wheel planes sit
# outboard of the hips by WHEEL_AXIAL_OFFSET = 0.091675 m, so the feet are
# 0.4234 m apart, not the 0.240 m hip spacing.
TRACK = 0.4234
HALF_TRACK = TRACK / 2.0
WHEELBASE = 0.510
R_OUT, R_CORNER = 0.145, 0.015


def ride_height(lam_deg):
    """Hub height of a wheel cambered by lambda, metres. Stage 0's tread form."""
    a = math.radians(abs(lam_deg))
    return R_OUT * math.cos(a) - 0.005 * math.sin(a) + 0.0


def apex_y(y_hub, h, lam_deg):
    """-> where this wheel's spin axis crosses the ground, in body y."""
    a = math.radians(abs(lam_deg))
    if a < 1e-9:
        return math.copysign(float("inf"), 1.0)
    return y_hub - math.copysign(h / math.tan(a), lam_deg)


def ackermann_outer(lam_inner_deg, h):
    """-> the outer wheel's camber that puts both apexes at the same point."""
    cot_in = 1.0 / math.tan(math.radians(lam_inner_deg))
    cot_out = cot_in + TRACK / h
    return math.degrees(math.atan(1.0 / cot_out))


def main():
    print(__doc__.split("Run:")[0].rstrip())
    print()
    print("=" * 72)
    print("1. All four wheels down, uniform camber: where does each apex land?")
    print("=" * 72)
    print(f"{'lambda':>7} {'h':>8} {'apex y':>9} {'front x':>9} {'rear x':>8} "
          f"{'agree?':>8}")
    for lam in (5, 10, 20, 30):
        h = ride_height(lam)
        ay = apex_y(HALF_TRACK, h, lam)
        print(f"{lam:6.0f}d {h:8.4f} {ay:9.4f} {+WHEELBASE/2:+9.3f} "
              f"{-WHEELBASE/2:+8.3f} {'NO':>8}")
    print()
    print(f"  Front and rear apexes are {WHEELBASE:.3f} m apart in x at EVERY")
    print("  camber, because the apex inherits the hub's x. A rigid body has one")
    print("  ICR. Curved rolling with four wheels down is impossible.")
    print()
    print("  It is worse than that: straight-line motion puts the ICR at")
    print("  infinity, which needs lambda = 0. So at any non-zero camber EVERY")
    print("  contact drills, whatever the robot does. Camber buys loss, not yaw.")

    print()
    print("=" * 72)
    print("2. One lateral pair, Ackermann camber: the apexes CAN be made to meet")
    print("=" * 72)
    print(f"{'lam_in':>7} {'lam_out':>8} {'h':>8} {'apex y':>9} {'R_turn':>9} "
          f"{'check':>9}")
    for lam_in in (10, 15, 20, 25, 30):
        h = ride_height(lam_in)
        lam_out = ackermann_outer(lam_in, h)
        # BOTH wheels lean the same physical way -- that is what "uniform
        # camber" means, and the Ackermann fix changes the MAGNITUDES, not the
        # directions. So both apex offsets carry the same sign; giving the
        # inner wheel a negative lambda here mirrors it instead, which is a
        # different (and impossible) pose.
        a_in = apex_y(-HALF_TRACK, h, +lam_in)
        a_out = apex_y(+HALF_TRACK, h, +lam_out)
        print(f"{lam_in:6.0f}d {lam_out:7.2f}d {h:8.4f} {a_in:9.4f} "
              f"{abs(a_in):9.3f} {abs(a_in-a_out):9.2e}")
    print()
    print("  lam_in is the INNER (right) wheel, lam_out the OUTER (left).")
    print("  R_turn is the body centreline's radius about the shared apex.")
    print("  'check' is |apex_inner - apex_outer|; it should be ~0 by")
    print("  construction, and it is.")

    print()
    print("=" * 72)
    print("3. What the confirming run actually commanded, for comparison")
    print("=" * 72)
    for lam in (10, 20, 30):
        h = ride_height(lam)
        a_l = apex_y(+HALF_TRACK, h, lam)
        a_r = apex_y(-HALF_TRACK, h, lam)
        print(f"  lambda {lam:2.0f}d uniform: left wheel wants ICR at y = "
              f"{a_l:+.3f}, right at {a_r:+.3f}"
              f"  -- {abs(a_l-a_r):.3f} m apart, always the track width")
    print()
    print("  Uniform camber puts the two sides' apexes exactly one track width")
    print("  apart, at every angle. They never agree, so the pair fights itself")
    print("  before the front/rear problem is even reached.")


if __name__ == "__main__":
    main()
