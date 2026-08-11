"""How much does the crown radius actually change camber steering?

The tread in the model is a flat band with 15 mm shoulder fillets, because the
rubber tyre is not fitted yet -- on the robot or in the sim. The question is
what fitting a round tread would change.

The mechanism at stake is GEOMETRIC camber steering: a body of revolution
tilted by lambda and rolling without slip pivots about the point where its spin
axis meets the ground, so it traces a circle. That is the "rolling-consistent
contact" claim, and it is distinct from camber thrust (tyre mechanics, needing a
deformable patch), which a rigid rim cannot produce.

Computed here by brute force rather than by the algebra, because the algebra
gives a suspiciously clean answer (h = R_t*cos(lambda) + r_c) and it is worth
knowing whether that is real before drawing a conclusion from it.

    uv run python examples/gslip/camber_crown_sensitivity.py
"""
import numpy as np

# Profiles, all with the same 145 mm outer radius and 40 mm width.
#   R_t  radius of the tread arc CENTRE circle
#   r_c  crown / fillet radius (the tube minor radius)
#   w_t  lateral position of the tube centre (0 = a fully round tread)
PROFILES = [
    ("flat + 15 mm shoulder fillet (model today)", 0.130, 0.015, 0.005),
    ("fully round tread, r_c = 20 mm",             0.125, 0.020, 0.000),
    ("shallow crown, r_c = 40 mm",                 0.105, 0.040, 0.000),
    ("thin disc (no crown at all)",                0.145, 0.000, 0.000),
]

CAMBERS_DEG = [2.0, 5.0, 10.0, 15.0, 20.0]

# Stance travel per stride, for converting a turn radius into deg/stride.
# Two bounds, because the robot does not achieve the rolling prediction:
#   geometric  0.293 m rolling radius * 0.635 rad of sweep
#   measured   v * T * duty at the measured 0.72 m/s
STANCE_TRAVEL_GEOM = 0.293 * np.deg2rad(36.4)
STANCE_TRAVEL_MEAS = 0.72 * 0.2239 * 0.4356


def contact_and_height(R_t, r_c, w_t, lam):
    """-> (centre height, contact lateral offset, contact radius).

    Wheel frame: spin axis along y, rolling along x, camber tilts about x by
    lambda. A tread point at axial offset w and radius r from the axis, at
    azimuth psi, lands at world height

        z = w*sin(lam) + r*sin(psi)*cos(lam)

    relative to the centre, so the lowest azimuth is psi = -pi/2 and the height
    of the CENTRE above that contact is  r*cos(lam) - w*sin(lam). The contact is
    whichever tread point MAXIMISES that -- the point that props the wheel
    highest is the one touching the ground.

    (The first version of this minimised r*cos(lam) + w*sin(lam) instead, which
    is wrong in both the sign and the direction of the extremum. It pinned the
    contact to the tread edge for every profile and every camber angle -- a
    result that looked plausible in the table and was pure artefact.)
    """
    # Flat central band: constant radius, w from -w_t to +w_t.
    ws = [-w_t, w_t]
    rs = [R_t + r_c, R_t + r_c]
    if r_c > 0:
        # Shoulder fillets, phi from the band edge round to the rim face.
        phi = np.linspace(0.0, np.pi / 2, 20001)
        for sgn in (-1.0, 1.0):
            ws.extend(sgn * (w_t + r_c * np.sin(phi)))
            rs.extend(R_t + r_c * np.cos(phi))
    ws = np.asarray(ws)
    rs = np.asarray(rs)

    h = rs * np.cos(lam) - ws * np.sin(lam)
    i = int(np.argmax(h))
    return float(h[i]), float(ws[i]), float(rs[i])


def main():
    print(f"{'profile':<44} {'lambda':>7} {'h_centre':>9} {'w_contact':>10} "
          f"{'R_turn':>8} {'deg/stride':>20}")
    print(f"{'':<44} {'':>7} {'mm':>9} {'mm':>10} {'m':>8} "
          f"{'(geom | measured)':>20}")
    for name, R_t, r_c, w_t in PROFILES:
        for lam_deg in CAMBERS_DEG:
            lam = np.deg2rad(lam_deg)
            h, w_c, _ = contact_and_height(R_t, r_c, w_t, lam)
            # The spin axis, tilted by lambda, meets the ground at horizontal
            # distance h / tan(lambda) from the centre. That is the pivot the
            # rolling body turns about.
            R_turn = h / np.tan(lam)
            d_geom = np.rad2deg(STANCE_TRAVEL_GEOM / R_turn)
            d_meas = np.rad2deg(STANCE_TRAVEL_MEAS / R_turn)
            print(f"{name if lam_deg == CAMBERS_DEG[0] else '':<44} "
                  f"{lam_deg:6.1f}d {1000*h:9.2f} {1000*w_c:10.2f} "
                  f"{R_turn:8.3f} {d_geom:8.2f} | {d_meas:7.2f}")
        print()

    print("Spread across profiles at each camber angle (this is the answer):")
    print(f"  {'lambda':>7} {'R_turn min':>12} {'R_turn max':>12} {'spread':>9}")
    for lam_deg in CAMBERS_DEG:
        lam = np.deg2rad(lam_deg)
        rs = []
        for _, R_t, r_c, w_t in PROFILES:
            h, _, _ = contact_and_height(R_t, r_c, w_t, lam)
            rs.append(h / np.tan(lam))
        print(f"  {lam_deg:6.1f}d {min(rs):12.4f} {max(rs):12.4f} "
              f"{100*(max(rs)-min(rs))/min(rs):8.2f}%")


if __name__ == "__main__":
    main()
