import sys; sys.path.insert(0, ".")
import matplotlib; matplotlib.use("Agg")
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import SharedTerrainSpec2D
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import RecoveryConfig2D, nominal_stroke_2d
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import hybrid_posture_2d, hybrid_timing_2d
from hybrid_note.scripts.experiments.day12_world_registration_2d import swing_hip_advance_m
from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import TransitionKind2D, run_nominal_transition_2d
posture = hybrid_posture_2d(); config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(hybrid_timing_2d(), posture))
stroke = nominal_stroke_2d(posture); end = stroke.end
hx, hz = float(end.hip_xz_m[0]), float(end.hip_xz_m[1])
spec = SharedTerrainSpec2D(height_m=0.04, top_length_m=0.40, x_start_m=hx + 0.6, arc_samples=121)
for adv in (0.0, 1e-4, 0.0575):
    out = run_nominal_transition_2d(spec, posture, config, kind=TransitionKind2D.RECOVERY,
        takeoff_theta_rad=float(end.theta_rad), takeoff_beta_rad=float(end.beta_rad),
        takeoff_hip_xz_m=(hx, hz), landing_hip_x_m=hx + adv)
    print(f"adv {adv*1e3:.1f} mm: success={out.success} refusal={out.refusal} frames={None if out.swing is None else len(out.swing.frames)} "
          f"end hip {None if out.swing is None else out.swing.end.hip_xz_m[0]*1e3:.4f}")
