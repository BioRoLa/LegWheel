"""Stage 0 deliverable: verify the contact geometry before any camber work.

Per the thesis timeline, Stage 0 owes:
  - R(alpha) per rim segment, and the alpha ranges over which each contacts
  - the crown radius r
  - sign conventions: positive lambda, hip/ABAD frame location, camber axis
  - plots of contact migration and R_eff vs lambda
  - inner/outer radius mismatch for R = 1-2 m

No simulation. Everything here is kinematics plus the proto's transform tree.

    uv run python examples/gslip/stage0_geometry.py
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from legwheel.models.leg_model import LegModel  # noqa: E402
from legwheel.config import RobotParams  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "stage0_figs")

# --- Frame facts, traced from corgi_sim/protos/CorgiRobotABAD.proto ---------
# Module solids sit at (+-0.255, +-0.12, 0.057166) in the robot frame
# (x fwd, y left, z up). Each carries a 120 deg rotation about a (1,1,1)-type
# axis, and each contains the leg frame at +-WHEEL_AXIAL_OFFSET along its own
# local x:
#
#   A(FL) axis (+1,+1,+1) -> local x maps to +y ;  offset +0.091675 -> y = +0.2117
#   B(FR) axis (+1,+1,+1) -> local x maps to +y ;  offset -0.091675 -> y = -0.2117
#   C(RR) axis (+1,-1,-1) -> local x maps to -y ;  offset +0.091675 -> y = -0.2117
#   D(RL) axis (+1,-1,-1) -> local x maps to -y ;  offset -0.091675 -> y = +0.2117
#
# Two sign flips that cancel. All four wheel planes end up OUTBOARD at the same
# |y|, which is the number a differential-drive model needs.
HALF_TRACK = RobotParams.BODY_WIDTH / 2 + RobotParams.WHEEL_AXIAL_OFFSET
TRACK = 2 * HALF_TRACK
WHEELBASE = RobotParams.WHEEL_BASE

R_TREAD = RobotParams.TIRE_TREAD_RADIUS      # 0.130 torus major
R_CORNER = RobotParams.TIRE_CORNER_RADIUS    # 0.015 torus minor
R_OUTER = RobotParams.WHEEL_RADIUS_OUTER     # 0.145
HALF_W = RobotParams.WHEEL_THICKNESS / 2     # 0.020
W_FLAT = HALF_W - R_CORNER                   # 0.005 flat half-width


def banner(s):
    print(f"\n{'='*72}\n{s}\n{'='*72}")


def rim_radius_profile(lm, theta_deg, n=721):
    """-> (alpha_deg, |P(alpha)| from the HIP, segment label)."""
    lm.forward(np.deg2rad(theta_deg), 0.0, vector=True)
    al = np.linspace(-180, 180, n)
    r, seg = [], []
    for a in al:
        p = lm.rim_point(a, 0.0)
        r.append(np.hypot(p[0], p[1]))
        m = ((a + 180) % 360) - 180
        seg.append("foot" if abs(m) <= 40 else "upper")
    return al, np.array(r), np.array(seg)


def contact_w_and_reff(lam_deg, crowned):
    """Contact lateral offset w and effective radius, current vs full crown."""
    a = np.deg2rad(np.abs(lam_deg))
    s = np.sign(lam_deg)
    if crowned:
        r_c = HALF_W                      # a full 20 mm crown on the same width
        return s * r_c * np.sin(a), (R_OUTER - r_c) + r_c * np.cos(a)
    # Current tread: flat band cannot hold contact off-vertical, so the contact
    # sits on the shoulder fillet and rides around it.
    return (s * (W_FLAT + R_CORNER * np.sin(a)),
            R_TREAD + R_CORNER * np.cos(a))


def main():
    os.makedirs(OUT, exist_ok=True)
    lm = LegModel()

    banner("1. Frames and track  (the number that was wrong)")
    print(f"  wheelbase                {WHEELBASE:.4f} m")
    print(f"  hip-to-hip (BODY_WIDTH)  {RobotParams.BODY_WIDTH:.4f} m")
    print(f"  wheel axial offset       {RobotParams.WHEEL_AXIAL_OFFSET:.6f} m "
          f"(outboard on all four -- see header)")
    print(f"  => CONTACT TRACK         {TRACK:.4f} m   "
          f"({TRACK/RobotParams.BODY_WIDTH:.2f}x the hip spacing)")
    print()
    print("  The differential-drive model used the HIP spacing, 0.24 m. Yaw per")
    print("  unit sweep differential goes as 1/track, so every prediction made")
    print(f"  with 0.24 m is {TRACK/RobotParams.BODY_WIDTH:.2f}x too large.")

    banner("2. R(alpha) per rim segment, and which segment ever contacts")
    print(f"  {'theta':>7} {'|O_r|':>8} {'R(0)':>8} {'R(+-40)':>9} "
          f"{'R min':>8} {'R max':>8}  segment in contact while running")
    for td in (17, 45, 84.4, 100, 130):
        al, r, seg = rim_radius_profile(lm, td)
        i0 = int(np.argmin(np.abs(al)))
        i40 = int(np.argmin(np.abs(al - 40)))
        lm.forward(np.deg2rad(td), 0.0, vector=True)
        note = ("perfect circle about the hip -- WHEEL MODE"
                if td < 20 else "foot rim only (|alpha| <= 40 deg)")
        print(f"  {td:7.1f} {np.hypot(*lm.O_r):8.4f} {r[i0]:8.4f} "
              f"{r[i40]:9.4f} {r.min():8.4f} {r.max():8.4f}  {note}")
    print()
    print("  At theta = 17 deg every rim centre coincides with the hip, so")
    print("  R(alpha) is constant: the leg-wheel really is a wheel, and 0.145 m")
    print("  is its radius. That is the ONLY pose where 0.145 is the rolling")
    print("  radius. Stage 1's wheeled-mode sweep lives here.")

    # Which alpha contacts, through the running stance sweep.
    banner("3. Contact alpha through the running stance sweep")
    betas = np.linspace(-18.25, 18.17, 9)
    lm.forward(np.deg2rad(100.0), 0.0, vector=True)
    print(f"  {'beta':>8} {'alpha at contact':>18}  (alpha = -beta)")
    for b in betas[::2]:
        lm.forward(np.deg2rad(100.0), np.deg2rad(b), vector=True)
        al = np.linspace(-180, 180, 1441)
        pts = np.array([lm.rim_point(a, 0.0) for a in al])
        ac = al[int(np.argmin(pts[:, 1]))]
        print(f"  {b:+8.2f} {ac:+18.2f}")
    print()
    print("  Contact alpha = -beta, so the running sweep spans |alpha| <= 18.3")
    print("  deg and NEVER leaves the foot rim. The upper rims are wheel-mode")
    print("  geometry only. Stage 1 must sweep alpha well past 40 deg to")
    print("  exercise them.")

    banner("4. Camber: contact migration and R_eff  (Stage 1 gate formula)")
    print(f"  {'lambda':>8} | {'CURRENT w':>10} {'R_eff':>8} {'h':>8}"
          f" | {'CROWNED w':>10} {'R_eff':>8} {'h':>8}")
    for lam in (0, 2, 5, 10, 15, 20, 30):
        wc, rc = contact_w_and_reff(lam, crowned=False)
        wr, rr = contact_w_and_reff(lam, crowned=True)
        a = np.deg2rad(lam)
        h_cur = R_TREAD * np.cos(a) - W_FLAT * np.sin(a) + R_CORNER
        h_crn = (R_OUTER - HALF_W) * np.cos(a) + HALF_W
        print(f"  {lam:8.1f} | {1000*wc:10.2f} {1000*rc:8.2f} {1000*h_cur:8.2f}"
              f" | {1000*wr:10.2f} {1000*rr:8.2f} {1000*h_crn:8.2f}")
    print("  (w, R_eff, h in mm)")
    print()
    print("  Stage 1's gate is written h = R cos(lambda) + r. That is the FULL")
    print("  TORUS form. The real tread is flat across the middle, so the")
    print("  contact sits on a shoulder fillet offset by w_t = 5 mm and the")
    print("  gate needs the extra term:")
    print()
    print("      h = R_tread*cos(lambda) - w_flat*sin(lambda) + r_corner")
    print()
    print(f"  The correction is {1000*(R_TREAD*np.cos(np.deg2rad(20)) + R_CORNER - (R_TREAD*np.cos(np.deg2rad(20)) - W_FLAT*np.sin(np.deg2rad(20)) + R_CORNER)):.2f} mm at lambda = 20 deg -- small, but it is")
    print("  a systematic bias, and Stage 1 is a validation stage.")

    banner("5. Inner/outer radius mismatch on a turn")
    print(f"  using the CONTACT track {TRACK:.4f} m (not the hip spacing)")
    print(f"  {'R':>6} {'inner':>8} {'outer':>8} {'ratio':>7} "
          f"{'mismatch':>9} {'per-side speed split':>21}")
    for R in (1.0, 1.25, 1.5, 1.75, 2.0):
        ri, ro = R - HALF_TRACK, R + HALF_TRACK
        print(f"  {R:6.2f} {ri:8.3f} {ro:8.3f} {ro/ri:7.3f} "
              f"{100*(ro-ri)/R:8.1f}% {100*(ro/ri-1):20.1f}%")
    print()
    print("  This is the asymmetry a trot's diagonal pair has to absorb, and it")
    print("  is what the differential channel must SUPPLY to hold a circle.")

    # ---------------- plots ----------------
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))

    ax = axes[0]
    for td in (17, 45, 100, 130):
        al, r, _ = rim_radius_profile(lm, td)
        ax.plot(al, 1000 * r, label=f"θ = {td}°")
    ax.axvspan(-40, 40, alpha=0.12, color="tab:green")
    ax.axvspan(-18.3, 18.3, alpha=0.25, color="tab:orange")
    ax.annotate("foot rim\n|α| ≤ 40°", xy=(0, 0), xytext=(70, 120),
                fontsize=8, color="tab:green")
    ax.set_xlabel("α  (deg)")
    ax.set_ylabel("|hip → rim point|  (mm)")
    ax.set_title("R(α) per rim segment\norange = running sweep")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    lams = np.linspace(0, 30, 200)
    wc = np.array([contact_w_and_reff(l, False)[0] for l in lams])
    wr = np.array([contact_w_and_reff(l, True)[0] for l in lams])
    ax.plot(lams, 1000 * wc, label="current tread (flat + fillet)")
    ax.plot(lams, 1000 * wr, "--", label="fully crowned (20 mm)")
    ax.axhline(1000 * W_FLAT, color="grey", lw=0.8)
    ax.axhline(1000 * HALF_W, color="red", lw=0.8, ls=":")
    ax.text(15, 1000 * HALF_W + 0.4, "wheel edge", color="red", fontsize=8)
    ax.set_xlabel("camber λ  (deg)")
    ax.set_ylabel("contact offset w  (mm)")
    ax.set_title("Contact migration under camber\n(10 mm jump through λ=0)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[2]
    Rs = np.linspace(1.0, 2.0, 100)
    ax.plot(Rs, 100 * ((Rs + HALF_TRACK) / (Rs - HALF_TRACK) - 1),
            label=f"contact track {TRACK:.3f} m")
    ax.plot(Rs, 100 * ((Rs + 0.12) / (Rs - 0.12) - 1), "--",
            label="hip spacing 0.240 m (wrong)")
    ax.set_xlabel("turn radius R  (m)")
    ax.set_ylabel("outer/inner path mismatch  (%)")
    ax.set_title("Inner/outer mismatch vs turn radius")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    p = os.path.join(OUT, "stage0_geometry.png")
    fig.savefig(p, dpi=150)
    print(f"\nplots written to {p}")


if __name__ == "__main__":
    main()
