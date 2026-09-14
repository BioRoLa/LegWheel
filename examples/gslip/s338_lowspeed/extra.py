"""s338 extras: production solver at the missed v_td 0.70 point; base params; cache identity.

Slow (solve_existence at beta step 0.25). From the LegWheel root:

    .venv/bin/python examples/gslip/s338_lowspeed/extra.py

Moved 2026-09-14 from the session scratchpad. Code changes: paths derived from this
file, and the scratch copy's part 4 (rendered lines per page of two scratch paper
builds, sB_base.txt / sB_edit.txt) is dropped -- paper-layout bookkeeping whose
inputs lived only in the scratchpad.
"""
import hashlib
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2]))  # LegWheel root (legwheel package)
sys.path.insert(0, str(HERE.parent))      # examples/gslip (stage2a_turning_envelope)
import stage2a_turning_envelope as env  # noqa: E402
from legwheel.models import slip_rf  # noqa: E402
from legwheel.models.gslip_fixed_point import find_fixed_points  # noqa: E402

LW = HERE.parents[2]
PAPER = LW.parent / "corgi-abad-icra2027"  # sibling checkout of the paper repo

p = env.base_params()
print(f"base params: m {p.m} l0 {p.l0:.5f} r {p.r:.5f} k {p.k:.2f} g {p.g}")
print("R_EMPIRICAL", env.R_EMPIRICAL)

# 1. production search (n_samples 20, alpha 1-45) at beta 84.0 / 84.25 / 84.5, v_td 0.70
for b in (84.0, 84.25, 84.5):
    fps = find_fixed_points(p, 0.70, np.deg2rad(b), alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
                            n_samples=20, stride_fn=slip_rf.stride)
    for fp in fps:
        ap = env.apex_mm(slip_rf.stride(p, 0.70, fp.alpha, fp.beta))
        print(f"  n20 prod-alpha beta {b:.2f}: alpha {np.rad2deg(fp.alpha):.2f} duty {fp.duty_factor:.4f} apex {ap:.1f} "
              f"vfwd {fp.mean_speed:.3f} slope {fp.slope:+.3f} pass {fp.duty_factor <= env.MAX_DUTY and ap >= env.MIN_APEX_MM}")
    if not fps:
        print(f"  n20 prod-alpha beta {b:.2f}: no fixed points")

# 2. solve_existence itself at 0.5 vs 0.25 step, v_td 0.70 (lambda 0, base params, slip_rf.stride)
for step in (0.5, 0.25):
    t0 = time.time()
    fp = env.solve_existence(p, 0.70, slip_rf.stride, step=step)
    if fp is None:
        print(f"solve_existence v_td 0.70 step {step}: None ({time.time()-t0:.0f}s)")
    else:
        print(f"solve_existence v_td 0.70 step {step}: beta {np.rad2deg(fp.beta):.2f} alpha {np.rad2deg(fp.alpha):.2f} "
              f"duty {fp.duty_factor:.4f} vfwd {fp.mean_speed:.3f} slope {fp.slope:+.3f} ({time.time()-t0:.0f}s)")

# 3. cache identity
for f in (PAPER / "figures" / "stage2a_grid.npz", LW / "examples" / "gslip" / "stage2a_figs" / "stage2a_grid.npz"):
    if f.exists():
        print(f, hashlib.sha256(f.read_bytes()).hexdigest()[:16], f.stat().st_size)
    else:
        print(f, "MISSING")
