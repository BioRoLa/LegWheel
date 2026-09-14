"""Scratch: why does the SLIP-RF pronk family end at v_td ~0.75?

Scan v_td 0.35-0.80 with (a) the production windows (beta 60-86, alpha 1-45)
and (b) widened windows (beta 60-89.5, alpha 1-89), recording every fixed
point with duty, apex, mean forward speed. No filters applied at record time.

Prints one JSON line per (k_rel, v_td) job; modes prod, wide, krel, floor, band
(8 worker processes). The scan_<mode>.jsonl files next to this script are the
saved scans of log s338 -- do not redirect over them. From the LegWheel root:

    .venv/bin/python examples/gslip/s338_lowspeed/lowspeed_scan.py prod > /tmp/scan_prod.jsonl

Moved 2026-09-14 from the session scratchpad; the only code changes are the
pathlib import and the sys.path entry (the LegWheel root, derived from this file).
"""
import json
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # LegWheel root
from legwheel.models import slip_rf
from legwheel.models.slip_rf import SlipRfParams
from legwheel.models.gslip_fixed_point import find_fixed_points
from legwheel.planners import gslip_to_corgi as g2c

MASS, G = 30.0, 9.81


def params(k_rel, theta_deg=100.0):
    lm = g2c.LegLengthMap()
    r = lm.leg.foot_radius
    h = lm.length(np.deg2rad(theta_deg))
    return SlipRfParams(m=MASS, l0=h + r, k=k_rel * MASS * G / h, r=r)


def scan(args):
    k_rel, v, beta_lo, beta_hi, a_lo, a_hi, step, ns = args
    p = params(k_rel)
    out = []
    t0 = time.time()
    for bdeg in np.arange(beta_lo, beta_hi + 1e-9, step):
        try:
            fps = find_fixed_points(p, v, np.deg2rad(bdeg),
                                    alpha_range=(np.deg2rad(a_lo), np.deg2rad(a_hi)),
                                    n_samples=ns, stride_fn=slip_rf.stride)
        except Exception as e:  # noqa
            continue
        for fp in fps:
            try:
                res = slip_rf.stride(p, v, fp.alpha, fp.beta)
            except Exception:
                continue
            tf = res["flight_time"]
            out.append(dict(beta=float(bdeg), alpha=float(np.rad2deg(fp.alpha)),
                            slope=fp.slope, duty=fp.duty_factor,
                            apex_mm=1000 * G * tf * tf / 8.0,
                            vfwd=fp.mean_speed, stance=fp.stance_time,
                            flight=tf, stride=fp.stride_length,
                            vx_td=v * np.cos(fp.alpha),
                            comp_mm=1000 * res["peak_compression"],
                            grf_bw=res["peak_grf_mag"] / (MASS * G)))
    return dict(k_rel=k_rel, v=v, win=[beta_lo, beta_hi, a_lo, a_hi],
                secs=time.time() - t0, fps=out)


if __name__ == "__main__":
    mode = sys.argv[1]
    jobs = []
    if mode == "prod":
        for v in (0.55, 0.65, 0.70, 0.75):
            jobs.append((18.0, v, 60.0, 86.0, 1.0, 45.0, 0.5, 20))
    elif mode == "wide":
        for v in (0.35, 0.45, 0.55, 0.65, 0.75):
            jobs.append((18.0, v, 60.0, 89.5, 1.0, 89.0, 0.5, 40))
    elif mode == "krel":
        for k in (7.0, 10.0):
            for v in (0.45, 0.60):
                jobs.append((k, v, 60.0, 89.5, 1.0, 89.0, 1.0, 40))
    elif mode == "floor":
        for v in (0.48, 0.50, 0.52):
            jobs.append((18.0, v, 84.0, 89.5, 30.0, 89.0, 0.25, 40))
    elif mode == "band":
        for v in (0.60, 0.70):
            jobs.append((18.0, v, 84.0, 89.75, 30.0, 89.0, 0.25, 50))
    with Pool(8) as pool:
        for r in pool.imap_unordered(scan, jobs):
            print(json.dumps(r), flush=True)
