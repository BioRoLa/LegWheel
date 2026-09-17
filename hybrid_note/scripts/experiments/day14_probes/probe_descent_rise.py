"""Descent geometry from the top's trailing edge at a reduced axle rise.

For each axle rise, take the arc-end stance pose at hip x = edge - short
(the fronts' liftoff in the planner's traces) with the axle at hold + rise,
and fly a DOWN to landings 0..80 mm ahead with the axle still at hold + rise.
Prints success / refusal per landing so the geometric limit is measured,
not guessed."""
import sys
sys.path.insert(0, ".")
import matplotlib; matplotlib.use("Agg")
from dataclasses import replace
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import SharedTerrainSpec2D
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import nominal_stroke_2d, RecoveryConfig2D
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import hybrid_posture_2d, hybrid_timing_2d
from hybrid_note.scripts.experiments.day12_world_registration_2d import swing_hip_advance_m
from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
    run_nominal_transition_2d, held_landing_pose_2d, TransitionKind2D)

h = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
rises = [float(v) for v in sys.argv[2].split(",")] if len(sys.argv) > 2 else [0.05, 0.06, 0.10]
shorts = [float(v) for v in sys.argv[3].split(",")] if len(sys.argv) > 3 else [0.006, 0.02, 0.04]
spec = SharedTerrainSpec2D(height_m=h, top_length_m=0.40, x_start_m=1.0, arc_samples=121)
posture = hybrid_posture_2d()
timing = hybrid_timing_2d()
config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))
hold = float(posture.hold_hip_z_m)
flat = nominal_stroke_2d(posture)
beta_end = float(flat.end.beta_rad)
top_z = float(spec.top_z_m)
edge = float(spec.x_start_m) + float(spec.top_length_m)
print(f"hold {hold*1e3:.1f} mm, arc-end beta {beta_end:.3f} rad, edge {edge*1e3:.0f} mm")
scene_posture = replace(posture, obstacle_xwh_m=(float(spec.x_start_m), float(spec.top_length_m), h))
for rise in rises:
    for short in shorts:
        hip_x = edge - short
        hip_z = hold + rise            # axle above ground level
        try:
            theta, hz = held_landing_pose_2d(posture, beta_end, 0.0, hip_z - top_z)
        except ValueError as e:
            print(f"rise {rise*1e3:.0f} short {short*1e3:.0f}: takeoff pose unreachable ({e})"); continue
        line = []
        for adv_mm in (0, 5, 10, 20, 30, 40, 60, 80):
            out = run_nominal_transition_2d(
                spec, scene_posture, config, kind=TransitionKind2D.DOWN,
                takeoff_theta_rad=float(theta), takeoff_beta_rad=beta_end,
                takeoff_hip_xz_m=(hip_x, top_z + hz),
                landing_hip_x_m=hip_x + adv_mm * 1e-3,
                landing_hip_z_above_surface_m=hip_z)
            if out.success:
                c = out.swing.end.contact_xz_m
                line.append(f"{adv_mm:+d}:ok(c{(c[0]-edge)*1e3:.0f})")
            else:
                line.append(f"{adv_mm:+d}:{str(out.refusal)[:28]}")
        print(f"rise {rise*1e3:.0f} short {short*1e3:.0f} theta {float(theta)*57.3:.0f}deg: " + " ".join(line))
