"""Log s335 reruns that need no dynamics: (A) old vs corrected lateral offset,
(B) the s190/s197 Webots contact-travel data re-scored against the corrected
contact point, (C) C4 re-evaluated on the cached Stage 2a grid."""
import json
import math

import numpy as np

from legwheel.models import contact_profile as cp
from legwheel.models.slip_rf_cambered import rolling_radius

D = 0.091675
R0_EMP = 0.14482
out = {}

# ---- A: geometry ---------------------------------------------------------------
rows = []
for deg in (1.0, 5.0, 10.0, 15.0, 20.0, 30.0):
    phi = math.radians(deg)
    y_new = cp.contact_point(phi, D).y
    y_leg_torus = D * math.cos(phi) + rolling_radius(phi) * math.sin(phi)
    y_leg_meas = D * math.cos(phi) + 0.145 * math.sin(phi)
    ideal = cp.TreadProfile(0.125, 0.020)
    ideal_excess = (D * math.cos(phi) + (0.125 + 0.020 * math.cos(phi)) * math.sin(phi)
                    - cp.contact_point(phi, D, ideal).y)
    rows.append({"deg": deg, "y_profile_mm": 1e3 * y_new,
                 "legacy_torus_minus_profile_mm": 1e3 * (y_leg_torus - y_new),
                 "legacy_measured_minus_profile_mm": 1e3 * (y_leg_meas - y_new),
                 "ideal_r20_excess_mm": 1e3 * ideal_excess,
                 "d_lat_tread_mm": 1e3 * cp.contact_point(phi, D).d_lat})
out["A_geometry"] = rows

# ---- B: s190 / s197 re-score ---------------------------------------------------
# Measured delta-by (mm, from lambda = 0) and the s190 `r sin` prediction,
# verbatim from diag/residual_decompose.py TABLE; achieved gamma per leg (deg)
# from the log s190 table (2 dp, so predictions reproduce to ~0.05 mm).
TABLE = {
    5: {"A": (3.90, 13.46), "B": (11.70, 12.68), "C": (11.90, 12.86), "D": (3.80, 13.21)},
    10: {"A": (15.70, 25.72), "B": (25.30, 26.79), "C": (25.80, 27.39), "D": (15.50, 25.32)},
    15: {"A": (25.65, 37.29), "B": (37.80, 40.71), "C": (38.30, 41.19), "D": (25.50, 36.93)},
    20: {"A": (35.20, 48.43), "B": (49.70, 53.48), "C": (50.40, 54.02), "D": (34.80, 47.82)},
    30: {"A": (50.75, 65.36), "B": (75.30, 80.15), "C": (75.90, 80.88), "D": (50.50, 64.77)},
}
GAMMA = {0: {"A": -0.27, "B": -0.01, "C": -0.27, "D": -0.00},
         5: {"A": 5.21, "B": -4.89, "C": -5.21, "D": 5.38},
         10: {"A": 10.57, "B": -10.08, "C": -10.54, "D": 10.69},
         15: {"A": 16.10, "B": -15.03, "C": -15.43, "D": 16.26},
         20: {"A": 22.06, "B": -19.44, "C": -19.85, "D": 22.10},
         30: {"A": 33.40, "B": -28.45, "C": -28.92, "D": 33.47}}
SY = {"A": +1.0, "B": -1.0, "C": -1.0, "D": +1.0}


def y_legacy(g):
    return D * math.cos(g) + 0.145 * math.sin(g)


def y_profile(g, ref_centred):
    if ref_centred and abs(math.degrees(g)) < 1.0:
        return D          # the lambda = 0 hold: flat band centred
    return cp.contact_point(g, D).y


def score(fn):
    res, sym, anti = [], [], []
    for lam, legs in TABLE.items():
        r = {}
        for leg, (meas, _) in legs.items():
            g1, g0 = math.radians(GAMMA[lam][leg]), math.radians(GAMMA[0][leg])
            pred = 1e3 * SY[leg] * (fn(g1) - fn(g0))
            r[leg] = pred - meas
            res.append(abs(pred - meas))
        ad, bc = 0.5 * (r["A"] + r["D"]), 0.5 * (r["B"] + r["C"])
        sym.append(0.5 * (ad + bc))
        anti.append(0.5 * (ad - bc))
    return {"mean_abs_res_mm": float(np.mean(res)), "sym_mm": sym, "anti_mm": anti,
            "mean_abs_sym_mm": float(np.mean(np.abs(sym)))}


# reproduction check: the legacy form must return s190's 7.25 mm
chk = []
for lam, legs in TABLE.items():
    for leg, (_, pred_tab) in legs.items():
        g1, g0 = math.radians(GAMMA[lam][leg]), math.radians(GAMMA[0][leg])
        chk.append(abs(1e3 * SY[leg] * (y_legacy(g1) - y_legacy(g0)) - pred_tab))
out["B_repro_max_abs_pred_diff_mm"] = float(max(chk))
out["B_legacy"] = score(y_legacy)
out["B_profile_ref_centred"] = score(lambda g: y_profile(g, True))
out["B_profile_ref_achieved_sign"] = score(lambda g: y_profile(g, False))
sl = np.array([math.sin(math.radians(l)) for l in TABLE])
for key in ("B_legacy", "B_profile_ref_centred"):
    b, a = np.polyfit(sl, np.array(out[key]["sym_mm"]), 1)
    out[key]["fit_a_mm"], out[key]["fit_b_mm"] = float(a), float(b)

# ---- C: C4 on the Stage 2a cache ----------------------------------------------
def load(p):
    return list(np.load(p, allow_pickle=True)["cells"])


cells = load("/mnt/c/Users/alexc/code/LegWheel/examples/gslip/stage2a_figs/stage2a_grid.npz")
fine = load("/mnt/c/Users/alexc/code/corgi-abad-icra2027/figures/stage2a_fine_sliver.npz")


def c4(cellset, leg_limit, abad_limit=29.5):
    rep = {}
    for law in ("empirical", "geometric"):
        sub = [c for c in cellset if c["law"] == law and c["exists"]]
        old_feas, new_feas, flips, new_all, old_all = [], [], 0, [], []
        for c in sub:
            lam = math.radians(c["lam_deg"])
            r_eff = rolling_radius(lam) if law == "geometric" else R0_EMP
            lever_old = D * math.cos(lam) + r_eff * math.sin(lam)
            f_leg = c["tau_abad"] / lever_old
            pt = cp.contact_point(lam, D)
            tau_plane = cp.abad_moment(pt, *cp.leg_plane_force(lam, f_leg))
            tau_vert = cp.abad_moment(pt, 0.0, f_leg)
            ok23 = c["psi"] <= 0.29 and c["tau_leg"] <= leg_limit
            feas_old = ok23 and c["tau_abad"] <= abad_limit
            feas_new = ok23 and tau_plane <= abad_limit and tau_vert <= abad_limit
            flips += feas_old != feas_new
            old_all.append(c["tau_abad"])
            new_all.append(max(tau_plane, tau_vert))
            if feas_old:
                old_feas.append(c["tau_abad"])
            if feas_new:
                new_feas.append((tau_plane, tau_vert, c["lam_deg"]))
        rep[law] = {
            "cells": len(sub), "verdict_flips": int(flips),
            "old_max_feasible": max(old_feas, default=None),
            "new_max_feasible_plane": max((t[0] for t in new_feas), default=None),
            "new_max_feasible_vertical": max((t[1] for t in new_feas), default=None),
            "max_feasible_lam_deg": max((t[2] for t in new_feas), default=None),
            "old_max_any": max(old_all), "new_max_any": max(new_all)}
    return rep


out["C_6to1"] = c4(cells, 29.5)
out["C_9to1"] = c4(cells, 44.25)
out["C_6to1_with_fine"] = c4(cells + fine, 29.5)
print(json.dumps(out, indent=1))
