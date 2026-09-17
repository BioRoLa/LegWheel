"""How fast the axle may fall behind the block.

For a leg that landed at the arc-start pose at hip x = x_land with the
axle at hold + rise, roll the stance forward (beta linear in hip x over the
flat stroke's advance) and, at each hip x, bisect the lowest legal axle
height with the block behind the leg.  Prints the envelope."""
import sys
sys.path.insert(0, ".")
import matplotlib; matplotlib.use("Agg")
import numpy as np
from dataclasses import replace
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import SharedTerrainSpec2D
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import nominal_stroke_2d, standing_stroke_2d
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import hybrid_posture_2d
from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import held_landing_pose_2d

h = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
rise = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
lands = [float(v) for v in sys.argv[3].split(",")] if len(sys.argv) > 3 else [1.362, 1.394]
spec = SharedTerrainSpec2D(height_m=h, top_length_m=0.40, x_start_m=1.0, arc_samples=121)
posture = hybrid_posture_2d()
scene = replace(posture, obstacle_xwh_m=(float(spec.x_start_m), float(spec.top_length_m), h))
hold = float(posture.hold_hip_z_m)
flat = nominal_stroke_2d(posture)
b0 = float(flat.frames[0].beta_rad); b1 = float(flat.end.beta_rad)
adv = float(flat.end.hip_xz_m[0]) - float(flat.frames[0].hip_xz_m[0])
print(f"hold {hold*1e3:.1f} beta {b0:.3f}->{b1:.3f} over {adv*1e3:.1f} mm")

def legal(x, beta, z):
    try:
        theta, hz = held_landing_pose_2d(posture, beta, 0.0, z)
    except ValueError:
        return False
    return standing_stroke_2d(scene, theta, beta, x, hz).success

for x_land in lands:
    line = []
    for dx in np.arange(0.0, 0.33, 0.02):
        x = x_land + dx
        beta = b0 + (b1 - b0) * min(1.0, dx / adv)
        top = hold + rise
        if not legal(x, beta, top):
            line.append(f"{x*1e3:.0f}:X"); continue
        lo, hi = hold, top          # lo may be illegal, hi legal
        if legal(x, beta, lo):
            line.append(f"{x*1e3:.0f}:0"); continue
        for _ in range(10):
            mid = 0.5 * (lo + hi)
            if legal(x, beta, mid): hi = mid
            else: lo = mid
        line.append(f"{x*1e3:.0f}:{(hi-hold)*1e3:.0f}")
    print(f"land {x_land*1e3:.0f}: " + " ".join(line))
