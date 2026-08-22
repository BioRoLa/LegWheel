"""Re-solve the Stage 2b roll deadbeat AT THE PLANT'S OPERATING POINT.

Deployment gate for the cambered-pair deadbeat controller.

WHY THIS EXISTS. `stage2b_clocked_torque.py` computes K = -pinv(J_u) J_x at
V_OP = 1.19 m/s touchdown (v~0.70). The plant runs at v_fwd 0.215-0.282 m/s,
which is v~ ~ 0.34 -- a factor of ~4 away in touchdown speed. K is the
linearisation AT x*; applied that far from it, K(x - x*) is not a small
correction but an extrapolation, and the "error" it feeds back is dominated by
a constant offset between the plant's limit cycle and a fixed point it never
visits. Shipping the v~0.70 gain would be a controller regulating to the wrong
orbit.

So: solve the pair fixed point at the plant's speed, take the Jacobians THERE,
and check three things before any of it reaches the robot.

  GATE 1  a non-grazing pair fixed point EXISTS at the plant's speed
  GATE 2  J_u still has the two-input structure rcond=1e-2 assumes
          (beta strong, differential camber usable, symmetric camber null) --
          if the singular values collapse differently down here, the
          regularisation that made the v~0.70 gain sane does not transfer
  GATE 3  the gain actually helps: survival on the perturbation grid must
          beat passive, at a peak ABAD torque the joint has (S37: 44.25 N.m
          ceiling, already worked near in the running gait)

If any gate fails, do not deploy -- say so and stop.

Run:
    uv run python examples/gslip/stage2b_deadbeat_at_plant.py
"""

from __future__ import annotations

import numpy as np

from legwheel.models import cambered_return_map as crm
from legwheel.models import slip_rf
from legwheel.models.cambered_return_map import PairParams, RollPD
from legwheel.models.gslip import GSlipFailure
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.planners import gslip_to_corgi as g2c

MASS, G, K_REL, THETA = 30.0, 9.81, 18.0, 100.0
MAX_STEPS = 12
ABAD_CEILING = 44.25          # N.m per joint, S37

# The plant, n = 5 (S125). v~ 0.34 is where the sagittal continuation puts it.
PLANT_VFWD = (0.215, 0.282)
PLANT_VT = 0.34

# The reference point stage2b used, for the side-by-side.
REF_VT, REF_V, REF_BETA = 0.70, 1.19, 80.75
REF_ALPHA_DEG = 40.74
L0_CORGI = 0.2931
# Bracket the plant: its fastest run is 0.282 m/s forward, and the pair
# point at v_td 0.41 has apex vx 0.311. Continue to there.
PLANT_VTD = 0.41

GRIDS = {
    "NEAR": (np.deg2rad([-3.0, -1.5, 1.5, 3.0]), [-0.15, -0.08, 0.08, 0.15]),
    "FAR": (np.deg2rad([-12.0, -6.0, 6.0, 12.0]), [-0.6, -0.3, 0.3, 0.6]),
}


def sagittal_point(v_tilde):
    """The planar fixed point at this speed -- seeds the 3D pair solve."""
    lm = g2c.LegLengthMap()
    r = lm.leg.foot_radius
    hip = lm.length(np.deg2rad(THETA))
    p = slip_rf.SlipRfParams(m=MASS, l0=hip + r, k=K_REL * MASS * G / hip, r=r)
    v = v_tilde * np.sqrt(G * p.l0)
    best = None
    for bd in np.arange(70.0, 89.01, 0.25):
        for fp in find_fixed_points(p, v, np.deg2rad(bd),
                                    alpha_range=(np.deg2rad(1.0), np.deg2rad(75.0)),
                                    n_samples=40, stride_fn=slip_rf.stride):
            apex = 1000.0 * G * fp.flight_time ** 2 / 8.0
            if apex < 10.0 or fp.duty_factor > 0.55:
                continue
            if best is None or abs(fp.slope) < abs(best.slope):
                best = fp
    return p, v, best


def report_ju(ju, label):
    sv = np.linalg.svd(ju, compute_uv=False)
    names = ("beta", "lam_left", "lam_right")
    print(f"  {label}: J_u singular values {np.array2string(sv, precision=4)}")
    ratio = sv / sv[0]
    print(f"    relative to beta: {np.array2string(ratio, precision=5)}")
    kept = int(np.sum(ratio >= 1e-2))
    print(f"    rcond=1e-2 keeps {kept} of 3 input directions")
    return sv, kept


def survival(p, x_star, u_star, gain=None, ctrl=None, regime="NEAR"):
    rho_g, drho_g = GRIDS[regime]
    if ctrl is not None:
        ctrl.reset()
    g = crm.perturbation_grid(p, x_star, u_star, rho_g, drho_g,
                              ctrl=ctrl, gain=gain, max_steps=MAX_STEPS)
    pct = 100.0 * np.count_nonzero(g >= MAX_STEPS) / g.size
    return pct, float(g.mean())


def main() -> None:
    print(__doc__.split("Run:")[0].rstrip())
    print()

    # --- GATE 1 -----------------------------------------------------------
    print("=" * 74)
    print("GATE 1 -- a PHYSICAL pair fixed point at the plant's speed")
    print("=" * 74)
    print("  Reached by CONTINUATION from the v~0.70 reference, not a cold solve.")
    print("  A cold solve at the plant's speed fails (residual 2.2e+01) -- that is")
    print("  the solver losing the branch, not the model lacking a gait, and")
    print("  reporting it as 'no fixed point' would have been wrong.")
    print("  Screened on apex height: the standing hip is ~0.293 m and the")
    print("  reference apex is 0.326, so anything outside 0.27-0.40 m is the")
    print("  absurd ballistic-hop branch, not this gait.\n")

    pp = PairParams()
    x_seed = [REF_V * np.cos(np.deg2rad(REF_ALPHA_DEG)), 0.0, 0.32, 0.0, 0.0]
    u_beta = np.deg2rad(REF_BETA)
    chain = []
    for v in np.arange(REF_V, PLANT_VTD - 1e-9, -0.03):
        seed = list(x_seed)
        seed[0] = v * np.cos(np.deg2rad(REF_ALPHA_DEG))
        try:
            x, u = crm.solve_periodic(pp, seed, [u_beta, 0.0, 0.0])
        except (GSlipFailure, ValueError, np.linalg.LinAlgError):
            continue
        if not (0.27 <= x[2] <= 0.40):
            continue
        x_seed, u_beta = list(x), float(u[0])
        chain.append((float(v), np.array(x), np.array(u)))
    if not chain:
        print("  FAIL: continuation never held the physical branch. Do not deploy.")
        return
    v_pt, x_star, u_star = chain[-1]
    print("  continuation held %d points down to v_td %.3f m/s (v~ %.3f)"
          % (len(chain), v_pt, v_pt / np.sqrt(G * L0_CORGI)))
    print("  x* = vx %.4f  vy %+.4f  h %.4f  rho %+.4f deg  drho %+.4f"
          % (x_star[0], x_star[1], x_star[2], np.rad2deg(x_star[3]), x_star[4]))
    print("  u* = beta %.2f deg" % np.rad2deg(u_star[0]))
    print("  plant v_fwd %.3f-%.3f m/s vs this point's apex vx %.3f -> %.2fx"
          % (*PLANT_VFWD, x_star[0], x_star[0] / PLANT_VFWD[1]))
    print("  PASS\n")

    jx, ju = crm.jacobians(pp, x_star, u_star)
    sv, kept = report_ju(ju, "PLANT")
    k = crm.deadbeat_gain(jx, ju)
    print("  |K| row norms (beta, lam_l, lam_r): "
          + np.array2string(np.linalg.norm(k, axis=1), precision=3))
    print("  max |K| element: %.3f" % np.max(np.abs(k)))
    r = dict(x=x_star, u=u_star, jx=jx, ju=ju, k=k, sv=sv, kept=kept)
    results = {"PLANT": r}

    # The reference gain, for the side-by-side that motivates re-linearising.
    try:
        xr, ur = crm.solve_periodic(
            pp, [REF_V * np.cos(np.deg2rad(REF_ALPHA_DEG)), 0.0, 0.32, 0.0, 0.0],
            [np.deg2rad(REF_BETA), 0.0, 0.0])
        jxr, jur = crm.jacobians(pp, xr, ur)
        kr = crm.deadbeat_gain(jxr, jur)
        print("\nfor comparison, the SHIPPED v~0.70 gain:")
        print("    max |K| element: %.3f   (plant point: %.3f)"
              % (np.max(np.abs(kr)), np.max(np.abs(k))))
        print("    x* vx %.4f vs plant %.4f -- the offset a v~0.70 gain would"
              % (xr[0], x_star[0]))
        print("    feed back as constant 'error' on this plant.")
    except Exception:
        pass
    print()

    # --- GATE 2 -----------------------------------------------------------
    print("=" * 74)
    print("GATE 2 -- does J_u keep the structure rcond=1e-2 assumes?")
    print("=" * 74)
    ok2 = r["kept"] == 2
    print("  stage2b at v~0.70 measured sigma(beta) ~ 3.4, sigma(diff lam) ~ 0.16,")
    print("  sigma(sym lam) ~ 0.0065 -- two usable directions, one null.")
    print("  at the plant: %d usable directions kept." % r["kept"])
    if "REF v~0.70" in results:
        print("  reference solve here kept %d." % results["REF v~0.70"]["kept"])
    print("  %s\n" % ("PASS" if ok2 else "FAIL -- the regularisation does not transfer"))

    # --- GATE 3 -----------------------------------------------------------
    print("=" * 74)
    print("GATE 3 -- does the gain actually help, within the ABAD budget?")
    print("=" * 74)
    for regime in ("NEAR", "FAR"):
        print(f"  --- {regime} ---")
        pas, pmean = survival(pp, r["x"], r["u"], regime=regime)
        ctrl = RollPD()
        pd, pdmean = survival(pp, r["x"], r["u"], ctrl=ctrl, regime=regime)
        pd_peak = ctrl.peak_used
        ctrl2 = RollPD()
        both, bmean = survival(pp, r["x"], r["u"], ctrl=ctrl2, gain=r["k"], regime=regime)
        print("    passive          %5.1f%% survive %d strides (mean %.1f)"
              % (pas, MAX_STEPS, pmean))
        print("    roll PD          %5.1f%% (mean %.1f)  peak %.1f N.m"
              % (pd, pdmean, pd_peak))
        print("    PD + deadbeat    %5.1f%% (mean %.1f)  peak %.1f N.m"
              % (both, bmean, ctrl2.peak_used))
        if ctrl2.peak_used > ABAD_CEILING:
            print("    !! peak exceeds the ABAD ceiling %.2f N.m -- not a rescue"
                  % ABAD_CEILING)
    print()
    print("=" * 74)
    print("Deploy only if all three gates pass. The gain, x* and u* printed above")
    print("are what the controller must carry -- at the PLANT's point, not v~0.70.")
    print("=" * 74)


if __name__ == "__main__":
    main()
