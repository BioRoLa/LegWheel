"""Which forward speed does the production solver give at the v~0.70 touchdown speed? (s84 says 0.87, s26 says 0.726).

Resolved in log s339 s5m: the shipped v070 template (export_pronk_csv.py rule, beta step
0.25) is beta* 80.75 deg, T 0.265233 s, forward 0.892 m/s; 0.87 is solve_existence at
beta step 1.0; 0.726 is a pre-s30 (stride_length bug) value. From the LegWheel root:

    .venv/bin/python examples/gslip/s338_lowspeed/v070.py

Moved 2026-09-14 from the session scratchpad; the only code changes are the pathlib
import and sys.path.
"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2]))  # LegWheel root (legwheel package)
sys.path.insert(0, str(HERE.parent))      # examples/gslip (stage2a_turning_envelope)
import stage2a_turning_envelope as env  # noqa: E402
from legwheel.models import slip_rf  # noqa: E402
from legwheel.models.gslip_fixed_point import find_fixed_points  # noqa: E402

p = env.base_params()
v_td = 0.70 * np.sqrt(env.G * p.l0)
print(f"v_td(v~0.70) = {v_td:.4f}")
for step in (1.0, 0.5):
    fp = env.solve_existence(p, v_td, slip_rf.stride, step=step)
    print(f"solve_existence step {step}: beta {np.rad2deg(fp.beta):.2f} alpha {np.rad2deg(fp.alpha):.2f} duty {fp.duty_factor:.4f} "
          f"mean_speed {fp.mean_speed:.4f} slope {fp.slope:+.4f}")
# every passing FP in the production window at 0.5 step, to see the spread of forward speeds
rows = []
for b in np.arange(60.0, 86.0 + 1e-9, 0.5):
    for fp in find_fixed_points(p, v_td, np.deg2rad(b), alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
                                n_samples=20, stride_fn=slip_rf.stride):
        ap = env.apex_mm(slip_rf.stride(p, v_td, fp.alpha, fp.beta))
        if fp.duty_factor <= 0.55 and ap >= 10.0:
            rows.append((b, np.rad2deg(fp.alpha), fp.duty_factor, ap, fp.mean_speed, fp.slope))
print("passing FPs in production window at v~0.70 touchdown:", len(rows))
for r in rows:
    print("  beta %.1f alpha %.2f duty %.3f apex %.1f vfwd %.3f slope %+.3f" % r)
# near the vault beta* 80.75
for b in (80.75, 80.91):
    for fp in find_fixed_points(p, v_td, np.deg2rad(b), alpha_range=(np.deg2rad(1.0), np.deg2rad(45.0)),
                                n_samples=60, stride_fn=slip_rf.stride):
        print(f"beta {b}: alpha {np.rad2deg(fp.alpha):.2f} duty {fp.duty_factor:.4f} vfwd {fp.mean_speed:.4f} slope {fp.slope:+.4f}")
