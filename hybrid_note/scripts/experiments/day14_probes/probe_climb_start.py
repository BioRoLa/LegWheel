"""How close to the face can the rear swinger take off for its climb?

Takeoff pose: the stance of a stroke that landed at the arc-start pose at
hip x = x_land (beta linear in hip x over the flat advance), at the held
height; UP flown to landings on the top at hold-50 (the rear axle's height
at the roll-up's end).  Prints ok / refusal per (takeoff, landing)."""
import sys
sys.path.insert(0, ".")
import matplotlib; matplotlib.use("Agg")
from dataclasses import replace
import numpy as np
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import SharedTerrainSpec2D
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import nominal_stroke_2d, RecoveryConfig2D
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import hybrid_posture_2d, hybrid_timing_2d
from hybrid_note.scripts.experiments.day12_world_registration_2d import swing_hip_advance_m
from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
    run_nominal_transition_2d, held_landing_pose_2d, TransitionKind2D)

h = 0.10
x_land = float(sys.argv[1]) if len(sys.argv) > 1 else 0.641
top_rise = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
spec = SharedTerrainSpec2D(height_m=h, top_length_m=0.40, x_start_m=1.0, arc_samples=121)
posture = hybrid_posture_2d(); timing = hybrid_timing_2d()
config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))
hold = float(posture.hold_hip_z_m); flat = nominal_stroke_2d(posture)
b0, b1 = float(flat.frames[0].beta_rad), float(flat.end.beta_rad)
adv = float(flat.end.hip_xz_m[0]) - float(flat.frames[0].hip_xz_m[0])
scene = replace(posture, obstacle_xwh_m=(float(spec.x_start_m), float(spec.top_length_m), h))
top_z = float(spec.top_z_m)
print(f"stroke landed at hip {x_land*1e3:.0f}; landing on the top at hold{top_rise*1e3:+.0f} above ground")
for x_take in (0.870, 0.884, 0.895, 0.905, 0.915, 0.925, 0.935, 0.945):
    beta = b0 + (b1 - b0) * min(1.0, max(0.0, (x_take - x_land) / adv))
    theta, hz = held_landing_pose_2d(posture, beta, 0.0, hold)
    line = []
    for x_l in (1.030, 1.040, 1.055, 1.070, 1.090):
        out = run_nominal_transition_2d(spec, scene, config, kind=TransitionKind2D.UP,
            takeoff_theta_rad=float(theta), takeoff_beta_rad=beta,
            takeoff_hip_xz_m=(x_take, hz), landing_hip_x_m=x_l,
            landing_hip_z_above_surface_m=hold + top_rise - top_z)
        line.append(f"{x_l*1e3:.0f}:{'ok' if out.success else str(out.refusal)[:22]}")
    print(f"takeoff hip {x_take*1e3:.0f} (face-{(1.0-x_take)*1e3:.0f}) beta {np.degrees(beta):5.1f}: " + "  ".join(line))
