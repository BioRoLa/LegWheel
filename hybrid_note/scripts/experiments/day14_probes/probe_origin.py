"""Plan the 40 mm crossing from one given origin, one pass, and say what happens."""
import sys, time
sys.path.insert(0, ".")
import matplotlib; matplotlib.use("Agg")
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import SharedTerrainSpec2D
from hybrid_note.scripts.experiments.day14_leg_terrain_rule_2d import plan_swing_swing_crossing_2d
from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import PlannerRefusal2D
origin = float(sys.argv[1]); passes = int(sys.argv[2]) if len(sys.argv) > 2 else 1
spec = SharedTerrainSpec2D(height_m=0.04, top_length_m=0.40, x_start_m=1.0, arc_samples=121)
t0 = time.perf_counter()
try:
    run, plan, rule, used = plan_swing_swing_crossing_2d(spec, samples=61, passes=passes, origins=[origin])
    print(f"[{time.perf_counter()-t0:.0f}s] PLANNED passes={used} feasible={run.feasible}")
    for e in plan.swings:
        print(f"  {e.leg.value:2s} {e.kind:22s} body x {e.body_x_start_m*1e3:8.1f}..{e.body_x_end_m*1e3:8.1f}  {e.start_s:7.3f}..{e.end_s:7.3f} s")
    for c in plan.cuts: print("  cut", c)
    for z in plan.slowdowns: print("  slow", z)
except PlannerRefusal2D as e:
    print(f"[{time.perf_counter()-t0:.0f}s] REFUSED: {e}")
