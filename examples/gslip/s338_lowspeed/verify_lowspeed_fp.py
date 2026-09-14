"""Independent spot check: do non-grazing SLIP-RF pronk fixed points exist below the
paper's 0.53 m/s existence edge once the landing-angle window is widened?
Same parameters and filters as stage2a_turning_envelope (base_params, duty <= 0.55,
apex >= 10 mm); only alpha/beta search ranges change.

Re-solves. From the LegWheel root:

    .venv/bin/python examples/gslip/s338_lowspeed/verify_lowspeed_fp.py

Moved 2026-09-14 from the session scratchpad; the only code change is sys.path.
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
print(f"base params: m {p.m}, l0 {p.l0:.4f}, r {p.r:.4f}, k {p.k:.1f}")
for label, betas, alpha_hi in (("production window", (84.0, 85.0, 86.0), 45.0),
                               ("steep landings", (86.5, 87.5, 88.5, 89.5), 89.0)):
    for v_td in (0.60, 0.70):
        passes = []
        for b in betas:
            for fp in find_fixed_points(p, v_td, np.deg2rad(b),
                                        alpha_range=(np.deg2rad(1.0), np.deg2rad(alpha_hi)),
                                        n_samples=40, stride_fn=slip_rf.stride):
                apex = env.apex_mm(slip_rf.stride(p, v_td, fp.alpha, fp.beta))
                ok = fp.duty_factor <= env.MAX_DUTY and apex >= env.MIN_APEX_MM
                passes.append((b, np.rad2deg(fp.alpha), fp.mean_speed, fp.duty_factor, apex, fp.slope, ok))
        good = [x for x in passes if x[6]]
        print(f"[{label}] v_td {v_td:.2f}: {len(passes)} fixed points, {len(good)} non-grazing")
        for b, a, v, d, ap, s, ok in sorted(good, key=lambda x: x[2])[:6]:
            print(f"    beta {b:5.1f} alpha {a:5.1f} v_fwd {v:.3f} duty {d:.3f} apex {ap:5.1f} mm slope {s:+.3f}")
