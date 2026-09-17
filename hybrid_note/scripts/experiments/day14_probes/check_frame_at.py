import sys, glob, pickle, time
sys.path.insert(0, ".")
import matplotlib; matplotlib.use("Agg")
import numpy as np
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import SharedTerrainSpec2D
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import RecoveryConfig2D
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import hybrid_posture_2d, hybrid_timing_2d
from hybrid_note.scripts.experiments.day12_world_registration_2d import swing_hip_advance_m
from hybrid_note.scripts.experiments.day14_leg_terrain_rule_2d import SwingSwingRule2D, terrain_axle_profiles_2d
from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import truncate_stroke_2d
spec = SharedTerrainSpec2D(height_m=0.04, top_length_m=0.40, x_start_m=1.0, arc_samples=121)
posture = hybrid_posture_2d(); timing = hybrid_timing_2d()
config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))
t0 = time.perf_counter()
rule = SwingSwingRule2D(spec=spec, posture=posture, config=config,
                        axles=terrain_axle_profiles_2d(spec, float(posture.hold_hip_z_m)))
print(f"rule built in {time.perf_counter()-t0:.0f}s")
S = sys.argv[1]
for path in sorted(glob.glob(S + "/stroke_cache/*.pkl"))[:40]:
    with open(path, "rb") as h:
        s = pickle.load(h)
    if len(s.frames) < 10:
        continue
    m = len(s.frames) // 2; a, b = s.frames[m], s.frames[m + 1]
    for w in (0.0, 0.3, 0.5, 0.97):
        x = float(a.hip_xz_m[0]) + w * (float(b.hip_xz_m[0]) - float(a.hip_xz_m[0]))
        t1 = time.perf_counter()
        f = rule.frame_at_hip_x(s, x)
        dt = time.perf_counter() - t1
        cz = float(f.contact_xz_m[1]) - float(a.contact_xz_m[1])
        # contact x should advance ~ linearly too
        cx_lin = float(a.contact_xz_m[0]) + w * (float(b.contact_xz_m[0]) - float(a.contact_xz_m[0]))
        print(f"{path[-12:-4]} w={w:.2f} hip {x*1e3:8.3f} beta {np.rad2deg(f.beta_rad):8.3f} theta {np.rad2deg(f.theta_rad):7.3f} "
              f"z {f.hip_xz_m[1]*1e3:7.3f} contact dz {cz*1e6:7.2f} um  contact x vs linear {(float(f.contact_xz_m[0])-cx_lin)*1e6:8.1f} um "
              f"rim {f.rim} surf {f.surface_id} idx {f.index} coll {f.collision} [{dt*1e3:.0f} ms]")
    cut = truncate_stroke_2d(s, float(a.hip_xz_m[0]) + 0.002, frame_at=rule.frame_at_hip_x)
    print(f"   truncated: {len(cut.frames)} frames, end hip {cut.end.hip_xz_m[0]*1e3:.3f} stop {cut.stop_reason} success {cut.success}")
    break
