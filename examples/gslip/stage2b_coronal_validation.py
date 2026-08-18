"""Does our BIP reproduce Chang 2022's qualitative results? The literature gate.

Chang, Hsu, Wu & Lin, "An analysis of the rolling dynamics of a hexapod
robot using a three-dimensional rolling template", Nonlinear Dyn 109:631-655
(2022). PDF read 2026-08-18 -- which retired two placeholders this file
carried from memory: RTL = ROLL TWO-LEG (two R-SLIP legs joined by a rigid
bar, not "roll-tap"), and Chang's gamma is NOT a static left/right stiffness
ratio -- it is k0/k1 with the constants EXCHANGED between sides at every
apex (Eq. 1-3, the tripod's alternating two-leg/one-leg sets; k_sum fixed,
gamma = 2 is the natural RHex value). Three phases here:

1. STATIC ASYMMETRY (ours, not Chang's): one-bounce closure and orbit search
   under a permanent one-sided ratio -- the Ackermann inner/outer case. The
   rolled fixed points this finds are the static-asymmetry cousins of the
   paper's gamma-phase orbits, NOT a divergence from the paper (section 59's
   first framing, written from a remembered abstract, got this wrong).
2. GAMMA-PHASE REPRODUCTION (Chang section 2.2): the two-step exchange map.
   Paper's shape to match: at gamma = 1 the pronk closes; for gamma != 1
   "the pronking orbit could not exist" and the two-step fixed points carry
   NONZERO ROLLING VELOCITY (his Fig. 6). Caveat stated up front: Chang's
   BIP pins the CoM laterally (z-only); ours frees y and tilts the legs, so
   vy* at the fixed point is a measured departure, reported not hidden.
3. ROLL INSTABILITY (his Fig. 13 result, on the BIP level): spectral radius
   of the apex map at the symmetric bounce, plus growth per bounce pinned
   vs sliding (1.40 vs 1.0044 at 2 cm -- v1 was absorbing the instability).

Still NOT here: the RTL-R-SLIP model itself (rolling half-circular legs,
torsional springs -- a different leg model from our linear-spring BIP; its
Table 1 clock-torque rescue, 0.59% -> 100% at gamma 1, alpha* 10, is the
shape section 45 already reproduced on the cambered pair).

Run:
    uv run python examples/gslip/stage2b_coronal_validation.py
"""

from __future__ import annotations

import numpy as np

from legwheel.models import coronal_bip as v1
from legwheel.models import coronal_return_map as v2
from legwheel.models.coronal_bip import CoronalParams
from legwheel.models.gslip import GSlipFailure

RATIOS = (0.85, 0.90, 0.95, 1.00, 1.05, 1.10, 1.20)
DROP = 0.02                     # apex height above equilibrium, m
N_GROWTH_BOUNCES = 6
RHO_SEED = 1e-6


def growth_per_bounce(p: CoronalParams, x0, n: int = N_GROWTH_BOUNCES) -> float:
    """Geometric-mean |rho| growth per apex map, section 44's statistic on the
    pinned model. NaN if fewer than 3 bounces complete."""
    x = np.asarray(x0, float).copy()
    rhos = [abs(x[2])]
    try:
        for _ in range(n):
            x = v2.apex_map(p, x)
            rhos.append(abs(x[2]))
    except GSlipFailure:
        pass
    rhos = [r for r in rhos if r > 0.0]
    if len(rhos) < 3:
        return float("nan")
    factors = np.array(rhos[1:]) / np.array(rhos[:-1])
    return float(np.exp(np.mean(np.log(factors))))


def main() -> None:
    print(__doc__.split("Run:")[0].rstrip())
    print()
    p = CoronalParams()
    z_eq = v1.equilibrium_height(p)
    x_sym = np.array([0.0, z_eq + DROP, 0.0, 0.0])
    print(f"equilibrium height {z_eq:.4f} m, apex seed h {x_sym[1]:.4f} m")

    # -- 1: STATIC asymmetry (ours -- the Ackermann inner/outer case) -------
    print()
    print("=== STATIC one-sided ratio (ours, not Chang's gamma) ===")
    print(f"{'ratio':>6} | {'1-bounce closure':>16} | {'orbit search':>32}")
    for ratio in RATIOS:
        pr = v2.with_stiffness_ratio(p, ratio)
        closure = float(np.linalg.norm(v2.apex_map(pr, x_sym) - x_sym))
        try:
            x_star = v2.solve_periodic(pr, x_sym, free=(0, 1, 2, 3),
                                       max_nfev=60)
            # A residual is not an orbit: hold it for three maps.
            drift, x = 0.0, x_star.copy()
            for _ in range(3):
                x = v2.apex_map(pr, x)
                drift = max(drift, float(np.linalg.norm(x - x_star)))
            verdict = (f"holds 3 maps (drift {drift:.1e}), "
                       f"rho* {np.rad2deg(x_star[2]):+.3f} deg"
                       if drift < 1e-4 else
                       f"solver point does NOT hold ({drift:.1e})")
        except GSlipFailure as e:
            verdict = str(e)
        print(f"{ratio:6.2f} | {closure:16.2e} | {verdict:>32}")

    # -- 2: Chang's gamma-phase reproduction (section 2.2 of the paper) -----
    print()
    print("=== GAMMA-PHASE (Chang Eq. 1-3): two-step exchange map ===")
    print("paper: pronk only at gamma 1; gamma != 1 fixed points carry "
          "NONZERO drho* (his Fig. 6)")
    print(f"{'gamma':>6} | {'pronk closure':>13} | "
          f"{'two-step fixed point':>44}")
    seed = x_sym
    for g in (1.0, 1.1, 1.25, 1.5, 1.75, 2.0):
        closure = float(np.linalg.norm(
            v2.apex_map_two_step(p, x_sym, g) - x_sym))
        try:
            # Continuation: seed each gamma from the previous fixed point --
            # Chang's own recipe steps parameters (his Fig. 4b), and the
            # gamma = 2 orbit is outside the symmetric seed's convergence.
            x_star = v2.solve_periodic_gamma(p, seed, g, max_nfev=60)
            seed = x_star
            drift, x = 0.0, x_star.copy()
            for _ in range(3):
                x = v2.apex_map_two_step(p, x, g)
                drift = max(drift, float(np.linalg.norm(x - x_star)))
            j2 = v2.jacobian_gamma(p, x_star, g)
            sr = float(np.max(np.abs(np.linalg.eigvals(j2))))
            verdict = (f"drho* {x_star[3]:+.4f} rad/s, vy* {x_star[0]:+.4f},"
                       f" |eig|max {sr:.3f}, holds {drift:.0e}"
                       if drift < 1e-4 else
                       f"solver point does NOT hold ({drift:.1e})")
        except GSlipFailure as e:
            verdict = str(e)
        print(f"{g:6.2f} | {closure:13.2e} | {verdict}")

    # -- 3: passive roll instability at ratio 1 -----------------------------
    print()
    print("=== ROLL INSTABILITY at ratio 1 (Chang Fig. 13's shape) ===")
    j = v2.jacobian(p, x_sym)
    eigs = np.abs(np.linalg.eigvals(j))
    print(f"apex-map |eigenvalues| {np.sort(eigs)[::-1].round(4)} "
          f"-> spectral radius {eigs.max():.4f}")

    print()
    print(f"{'drop':>6} {'v2 pinned':>10} {'v1 sliding':>10}   (section 44 v1:"
          f" 1.0044 at 2 cm, 1.327 at 5 cm)")
    for drop in (0.02, 0.05):
        x0 = np.array([0.0, z_eq + drop, RHO_SEED, 0.0])
        g2 = growth_per_bounce(p, x0)
        g1 = v1.roll_growth_per_bounce(p, drop=drop, rho0=RHO_SEED)
        print(f"{drop * 100:5.0f}cm {g2:10.4f} {g1:10.4f}")

    print()
    print("RTL-R-SLIP (roll two-leg, torsional-spring rolling legs): a "
          "different leg model; its Table 1 rescue shape (0.59% -> 100% at "
          "gamma 1, alpha* 10) is what section 45 reproduced on the cambered "
          "pair. Not re-derived here.")


if __name__ == "__main__":
    main()
