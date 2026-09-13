"""Log s335: coronal fixed points, straight orbit + Ackermann family, both pair signs.

Reproduces stage2b_clocked_torque.py section 1 (the log s45/s184 existence
result) and adds the opposite-sign pair. Model side convention: lam > 0 moves
that side's contact OUTBOARD, so a same-sign pair is a SPLAY and an
opposite-sign pair is a coherent LEAN (log s274 s1). Prints JSON.

Run once per arm, e.g.
    LEGWHEEL_RADIUS_LAW=torus LEGWHEEL_AXIAL_DROP=0.0 \
    LEGWHEEL_CONTACT_LATERAL=profile .venv/bin/python examples/gslip/s335_fixed_points.py
"""
import json
import os
import time

import numpy as np

from legwheel.models import cambered_return_map as crm
from legwheel.models import coronal_bip as bip
from legwheel.models.gslip import GSlipFailure

V_OP = 1.19
BETA0 = np.deg2rad(80.75)
SEED = [V_OP * np.cos(np.deg2rad(40.74)), 0.0, 0.32, 0.0, 0.0]


def main():
    out = {"radius_law": bip.RADIUS_LAW_DEFAULT,
           "axial_drop": bip.AXIAL_DROP_DEFAULT,
           "lateral": bip.CONTACT_LATERAL_DEFAULT}
    p = crm.PairParams()
    t0 = time.time()
    x, u = crm.solve_periodic(p, SEED, [BETA0, 0.0, 0.0])
    out["straight"] = {"vx": float(x[0]), "h": float(x[2]),
                       "beta_deg": float(np.rad2deg(u[0]))}
    ride = float(x[2])
    fam = []
    signs = [int(s) for s in os.environ.get("PAIR_SIGNS", "1,-1").split(",")]
    for pair_sign in signs:
        for lam_in_deg in (5.0, 10.0, 15.0):
            lam_in, lam_out = crm.ackermann_pair(np.deg2rad(lam_in_deg), ride)
            u0 = [u[0], lam_in, pair_sign * lam_out]
            g_l = bip.side_geometry(lam_in)
            g_r = bip.side_geometry(pair_sign * lam_out)
            rec = {"pair": "same-sign" if pair_sign > 0 else "opposite-sign",
                   "lam_in": lam_in_deg, "lam_out": float(np.rad2deg(lam_out)),
                   "d_out_l_mm": 1e3 * g_l.d_out, "d_out_r_mm": 1e3 * g_r.d_out,
                   "l0_l_mm": 1e3 * g_l.l0, "l0_r_mm": 1e3 * g_r.l0}
            try:
                xc, uc = crm.solve_periodic(p, x, u0, free_x=(1, 2, 3, 4),
                                            free_u=(0,))
                rec.update(exists=True, vy=float(xc[1]), h=float(xc[2]),
                           rho_deg=float(np.rad2deg(xc[3])),
                           beta_deg=float(np.rad2deg(uc[0])))
            except GSlipFailure as e:
                rec.update(exists=False, err=str(e))
            fam.append(rec)
    out["family"] = fam
    out["secs"] = round(time.time() - t0, 1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
