"""Does our BIP reproduce Chang 2022's qualitative results? The literature gate.

Before the group sees any Stage 2b coronal result, the model has to reproduce
what the lab's own paper published. Two claims are checkable without the PDF
open (from the results as recorded in the reading list and section 40/44):

1. EXISTENCE: the bounce-in-place pronk orbit exists at stiffness ratio 1 and
   only there -- an asymmetric pair pumps roll, so no symmetric-shape periodic
   orbit survives. Checked by (a) one-bounce closure of the symmetric bounce
   at each ratio and (b) an honest periodic-orbit search (all four apex
   components free) with a re-simulation check on whatever the solver returns
   -- a converged residual is not an orbit until three consecutive maps hold.
2. ROLL INSTABILITY (Fig. 13's shape): the pronk's apex-map Jacobian has
   spectral radius > 1, and a seeded roll grows bounce over bounce. Growth
   factors are printed next to section 44's v1 (sliding-contact) numbers --
   1.0044 per bounce at a 2 cm drop, 1.327 at 5 cm -- and the same drops on
   the pinned model, plus the v1 cross-check, so contact-model robustness of
   the qualitative conclusion is on record.

NOT here: Chang's RTL fixed points (roll-tap locomotion) and the exact gamma
convention. Both need the PDF (Tier 1 reading list) -- the mirror machinery
(E_MIRROR) is built and waiting, but coding "RTL" from a remembered abstract
is how section 40's SLTL mistake happened. Blocked deliberately.

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

    # -- 1: existence vs stiffness ratio ------------------------------------
    print()
    print("=== EXISTENCE: pronk orbit vs stiffness ratio (Chang: only at 1) ===")
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

    # -- 2: passive roll instability at ratio 1 -----------------------------
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
    print("RTL fixed points: BLOCKED on the Chang 2022 PDF (E_MIRROR is built;"
          " the definition is not going to be coded from memory).")


if __name__ == "__main__":
    main()
