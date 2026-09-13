"""Log s336: rerun the Stage 2a GEOMETRIC radius-law arm after rolling_radius
became R_t + r_c cos (the profile contact's radius), merge it with the cached
empirical arm (lambda-independent radius, physics unchanged), and re-score E3.

Writes examples/gslip/stage2a_figs/stage2a_grid_s336.npz; the s84 cache
stage2a_grid.npz is left untouched.

    .venv/bin/python examples/gslip/s336_radius_law_rerun.py
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import stage2a_turning_envelope as env  # noqa: E402

OLD = env.FIG_DIR / "stage2a_grid.npz"
NEW = env.FIG_DIR / "stage2a_grid_s336.npz"
GATES = (("stage 35 / 40", 35.0, 40.0),
         ("paper 29.5 / 29.5", 29.5, 29.5),
         ("9:1 44.25 / 29.5", 44.25, 29.5))


def feasible_set(cells, law, leg_limit, abad_limit):
    out = set()
    for c in cells:
        if c["law"] != law or not c["exists"]:
            continue
        if (c["psi"] <= env.PSI_DOT_MAX and c["tau_leg"] <= leg_limit
                and c["tau_abad"] <= abad_limit):
            out.add((round(float(c["v_td"]), 3), float(c["lam_deg"])))
    return out


def e3(cells, leg_limit, abad_limit):
    fe = feasible_set(cells, "empirical", leg_limit, abad_limit)
    fg = feasible_set(cells, "geometric", leg_limit, abad_limit)
    return len(fe ^ fg) / max(1, len(fe | fg)), len(fe), len(fg)


def main():
    old = list(np.load(OLD, allow_pickle=True)["cells"])
    t0 = time.time()
    geo = env.run_grid(env.V_GRID_FULL, env.LAM_GRID_DEG, 0.5,
                       laws=("geometric",), verbose=False)
    print(f"geometric arm: {len(geo)} cells in {time.time() - t0:.0f} s")
    merged = [c for c in old if c["law"] == "empirical"] + geo
    np.savez_compressed(NEW, cells=np.array(merged, dtype=object))
    print(f"wrote {NEW}")

    old_geo = [c for c in old if c["law"] == "geometric"]
    for name, leg, abad in GATES:
        d_old, ne, ng_old = e3(old, leg, abad)
        d_new, _, ng_new = e3(merged, leg, abad)
        a = feasible_set(old_geo, "geometric", leg, abad)
        b = feasible_set(geo, "geometric", leg, abad)
        print(f"{name}: E3 old {d_old:.1%} (emp {ne}, geo {ng_old}) -> new "
              f"{d_new:.1%} (geo {ng_new}); geometric set old vs new differs "
              f"in {len(a ^ b)} of {len(a | b)} cells")
    ex_old = {(round(float(c["v_td"]), 3), float(c["lam_deg"]))
              for c in old_geo if c["exists"]}
    ex_new = {(round(float(c["v_td"]), 3), float(c["lam_deg"]))
              for c in geo if c["exists"]}
    print(f"geometric existence cells old {len(ex_old)} new {len(ex_new)}; "
          f"changed: {sorted(ex_old ^ ex_new)}")
    beta_shift = []
    for c_new in geo:
        if not c_new["exists"]:
            continue
        match = [c for c in old_geo if c["exists"]
                 and round(float(c["v_td"]), 3) == round(float(c_new["v_td"]), 3)
                 and c["lam_deg"] == c_new["lam_deg"]]
        if match:
            beta_shift.append(abs(c_new["beta_deg"] - match[0]["beta_deg"]))
    if beta_shift:
        print(f"|beta* shift| old -> new geometric: max {max(beta_shift):.2f} deg, "
              f"median {np.median(beta_shift):.2f} deg over {len(beta_shift)} cells")
    env.check_predictions(merged)


if __name__ == "__main__":
    main()
