"""Scan LF's arc-end contact before the face; one planning pass per value, traced.

usage: scan_origins.py <passes> <contact_mm>...   (each value planned in turn)
"""
import sys, time
sys.path.insert(0, ".")
import matplotlib; matplotlib.use("Agg")
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import SharedTerrainSpec2D
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LegId
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import RecoveryConfig2D
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import hybrid_posture_2d, hybrid_timing_2d
from hybrid_note.scripts.experiments.day12_world_registration_2d import swing_hip_advance_m
from hybrid_note.scripts.experiments.day14_leg_terrain_rule_2d import (
    SwingSwingRule2D, origin_for_arc_end_contact_2d, plan_swing_swing_crossing_2d, terrain_axle_profiles_2d)
from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import PlannerRefusal2D, hybrid_clock_2d
passes = int(sys.argv[1]); contacts = [float(v) * 1e-3 for v in sys.argv[2:]]
import os
spec = SharedTerrainSpec2D(height_m=float(os.environ.get("DAY14_H_MM", "40")) * 1e-3, top_length_m=0.40, x_start_m=1.0, arc_samples=121)
posture = hybrid_posture_2d(); timing = hybrid_timing_2d()
config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))
probe = SwingSwingRule2D(spec=spec, posture=posture, config=config,
                         axles=terrain_axle_profiles_2d(spec, float(posture.hold_hip_z_m)))
clock, _, _ = hybrid_clock_2d(timing, posture, config)
for contact in contacts:
    origin = origin_for_arc_end_contact_2d(probe, clock, LegId.LF, contact)
    print(f"##### LF arc-end contact {contact * 1e3:.0f} mm -> origin {origin * 1e3:.1f} mm")
    t0 = time.perf_counter()
    try:
        run, plan, rule, used = plan_swing_swing_crossing_2d(
            spec, timing=timing, posture=posture, config=config, samples=61, passes=passes, origins=[origin])
        print(f"[{time.perf_counter()-t0:.0f}s] PLANNED contact {contact * 1e3:.0f} passes={used} feasible={run.feasible} "
              f"hard={[c.value for c in run.report.hard_failed_checks()]} advisory={[c.value for c in run.report.advisory_failed_checks()]}")
        for e in plan.swings:
            print(f"  {e.leg.value:2s} {e.kind:22s} body x {e.body_x_start_m*1e3:8.1f}..{e.body_x_end_m*1e3:8.1f}  {e.start_s:7.3f}..{e.end_s:7.3f} s  min {e.minimum_duration_s:.3f}")
        for c in plan.cuts: print("  cut", c)
        for z in plan.slowdowns: print("  slow", z)
    except PlannerRefusal2D as e:
        print(f"[{time.perf_counter()-t0:.0f}s] REFUSED contact {contact * 1e3:.0f}: {e}")
