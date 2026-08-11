"""How much of the turning scrub is FORCED by geometry, and therefore immune to friction?

The measurement says slip goes from 8% running straight to 32% while turning.
Before spending runs on raising the friction coefficient, it is worth knowing
how much of that extra 24 points friction could possibly fix.

The robot has no steering degree of freedom at the foot. Each leg rolls in its
own sagittal plane, fixed by beta; gamma cambers it but does not yaw it. On a
curved path, though, a foot at body coordinate (x, y) has velocity

    v_foot = (v - psi_dot*y,  psi_dot*x)

so the component psi_dot*x is perpendicular to the only direction the foot can
roll. It has nowhere to go but sideways. That is scrub the gait CREATES, not
scrub the surface fails to prevent, and raising mu cannot remove it -- it can
only convert sliding into binding force.

    uv run python examples/gslip/turn_scrub_geometry.py
"""
import numpy as np

WHEEL_BASE = 0.510      # front-rear wheel centres
BODY_WIDTH = 0.240      # hip to hip
STRIDE_S = 0.2239
DUTY = 0.436            # template stance fraction

# Legs A=FL, B=FR, C=RR, D=RL at (x, y), +x front, +y left.
LEGS = {"A(FL)": (+WHEEL_BASE / 2, +BODY_WIDTH / 2),
        "B(FR)": (+WHEEL_BASE / 2, -BODY_WIDTH / 2),
        "C(RR)": (-WHEEL_BASE / 2, -BODY_WIDTH / 2),
        "D(RL)": (-WHEEL_BASE / 2, +BODY_WIDTH / 2)}


def main():
    v, psi = 0.534, 0.288          # the run that held a circle
    stance_s = STRIDE_S * DUTY

    print(f"Measured operating point: v = {v:.3f} m/s, "
          f"psi_dot = {psi:.3f} rad/s  (R = {v/psi:.2f} m)")
    print(f"stance {1000*stance_s:.1f} ms per stride\n")
    print(f"  {'leg':>7} {'fwd v':>8} {'lateral v':>10} {'misalign':>9} "
          f"{'roll/stance':>12} {'SCRUB/stance':>13}")
    tot = 0.0
    for name, (x, y) in LEGS.items():
        vf = v - psi * y
        vl = psi * x
        ang = np.rad2deg(np.arctan2(vl, vf))
        roll = vf * stance_s
        scrub = abs(vl) * stance_s
        tot += scrub
        print(f"  {name:>7} {vf:8.3f} {vl:+10.3f} {ang:+8.2f}d "
              f"{1000*roll:11.1f}mm {1000*scrub:12.1f}mm")
    print(f"\n  lateral scrub is {100*abs(psi*WHEEL_BASE/2)/v:.1f}% of the "
          f"rolling distance, on every foot, every stance.")
    print("  Front and rear pairs scrub in OPPOSITE directions, so the four")
    print("  feet also fight each other about the yaw axis.")
    print()

    print("Scaling with commanded turn rate (front/rear legs, v held at 0.534):")
    print(f"  {'psi_dot':>9} {'R':>7} {'misalign':>9} {'scrub/stance':>13} "
          f"{'% of roll':>10}")
    for p in (0.10, 0.20, 0.288, 0.36, 0.50):
        vl = p * WHEEL_BASE / 2
        ang = np.rad2deg(np.arctan2(vl, v))
        print(f"  {p:9.3f} {v/p:7.2f} {ang:+8.2f}d "
              f"{1000*vl*stance_s:12.1f}mm {100*vl/v:9.1f}%")
    print()
    print("  The envelope measured in sim -- usable to about 0.29 rad/s, failing")
    print("  by 0.36 -- brackets the point where forced lateral scrub passes")
    print("  ~10% of the rolling distance. That is a geometric limit, and it")
    print("  does not move when the surface gets grippier.")
    print()

    print("What raising mu can and cannot do:")
    print("  CAN   hold the foot against LONGITUDINAL slip, which is the 8%")
    print("        measured running straight, and steady the contact -- the")
    print("        runs that slipped most also had the fewest clean contacts.")
    print("  CANNOT remove the lateral component above. With more grip the foot")
    print("        binds instead of sliding, and the reaction is fed back into")
    print("        a leg whose commanded steering differential is already only")
    print("        0-87% realised. Grip could make the turn WORSE while making")
    print("        the straight-line roll better.")
    print()
    print("  This is also the quantitative case for camber steering: camber is")
    print("  the only actuated freedom that can act on the lateral contact")
    print("  condition at all. Differential beta cannot -- it only commands")
    print("  more or less of a roll the foot is already constrained to.")


if __name__ == "__main__":
    main()
