"""Day 12 Step 5 driver: the body trajectory the four legs jointly ask for.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step5_body_trajectory.csv``   samples, concessions, conflicts, summary
``day12_step5_body_trajectory.png``   body_z(t) with each concession's owner

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step5_driver.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import numpy as np  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_composer_2d import (  # noqa: E402
    compose_2d,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    StrategyId,
    decide_2d,
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (  # noqa: E402
    BODY_BASIS,
    BodyDriver,
    body_rows,
    body_trajectory_2d,
    plot_body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (  # noqa: E402
    LEG_ORDER,
    initialize_four_leg_state_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (  # noqa: E402
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (  # noqa: E402
    build_leg_plan_2d,
    plan_four_legs_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"
DAY10_11 = OUT.parent / "day10-11"
DAY6_7 = OUT.parent / "day6-7"

#: Day 10--11 Step 9's own cell for ``#4`` (see the Step 4 driver).
SWING_SWING_HEIGHT_M = 0.080


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    tables = load_tables_2d(DAY10_11, DAY6_7)

    # The nominal body height is Step 2's, not a new number.
    state = initialize_four_leg_state_2d(SharedTerrainSpec2D(
        height_m=0.04, top_length_m=0.40, x_start_m=1.00,
        obstacle_id="day12_platform"))
    nominal_body_z_m = float(state.body_position_world_m[2])
    print(f"nominal body_z   {nominal_body_z_m*1e3:.3f} mm   (Step 2)")
    print(f"basis            {BODY_BASIS}")

    top = next((round(0.02 + 0.005 * i, 4) for i in range(87)
                if decide_2d(SWING_SWING_HEIGHT_M, round(0.02 + 0.005 * i, 4),
                             tables).winner is StrategyId.SWING_SWING), None)
    composed = compose_2d(SWING_SWING_HEIGHT_M, top, tables,
                          strategy=StrategyId.SWING_SWING)
    print(f"\ncrossing         {composed.strategy.value}  "
          f"h={SWING_SWING_HEIGHT_M*1e3:.0f} mm  L_top={top*1e3:.0f} mm  "
          f"composed={composed.composed}")

    plans = {leg: build_leg_plan_2d(leg, composed) for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, walk_timing_2d())
    trajectory = body_trajectory_2d(four, nominal_body_z_m=nominal_body_z_m)

    write_rows_csv(OUT / "day12_step5_body_trajectory.csv",
                   body_rows(trajectory))

    summary = trajectory.as_dict()
    print(f"\nsamples          {summary['samples']}")
    print(f"body_z           {summary['body_z_min_mm']:.3f} -> "
          f"{summary['body_z_max_mm']:.3f} mm")
    print(f"body_z travel    {summary['body_z_travel_mm']:.3f} mm"
          f"   <- plan §12 requirement 8")
    print(f"largest step     {summary['max_body_z_step_mm']:.3f} mm "
          f"between neighbouring samples")
    print(f"body_y / rpy     {summary['body_y_mm']:.1f} mm / "
          f"({summary['body_roll_deg']:.1f}, {summary['body_pitch_deg']:.1f}, "
          f"{summary['body_yaw_deg']:.1f}) deg   <- unchanged, requirement 6")
    print(f"feasible samples {summary['feasible_samples']} of "
          f"{summary['samples']}")
    print(f"conflicts        {summary['conflict_count']}"
          f"   feasible = {summary['is_feasible']}")
    print(f"worst disagreement between two hard requirements  "
          f"{summary['max_disagreement_mm']:.3f} mm")
    print("\nwhy: the foot-rim stroke's hip height is an ARC")
    print("  Step 1 measured 202.161 mm at both ends, 219.448 mm in the middle")
    print("  -> travel 17.287 mm over one stroke.")
    print("  Walk's phase offsets put the three stance legs at three different")
    print("  points on that arc, so on a rigid body with rpy = 0 they demand")
    print("  three different body heights at the same instant.  All three are")
    print("  TRACK, and TRACK is hard.")
    print("  Plan §12 requirement 5: return infeasible, do NOT average.")
    for conflict in trajectory.conflicts[:3]:
        row = conflict.as_dict()
        print(f"    t={row['time_s']:.3f}s  {row['leg_a']} wants "
              f"{row['body_z_a_mm']:.3f} mm, {row['leg_b']} wants "
              f"{row['body_z_b_mm']:.3f} mm  -> "
              f"{row['disagreement_mm']:.3f} mm apart")

    counts = {driver: sum(1 for s in trajectory.samples if s.driver is driver)
              for driver in BodyDriver}
    print("\nwhat set body_z, sample by sample")
    for driver, count in counts.items():
        if count:
            print(f"  {driver.value:12s} {count:4d}")

    print("\nevery concession, named   (plan §12 acceptance)")
    for start, end, leg in trajectory.concession_intervals():
        rows = [s for s in trajectory.samples
                if start <= s.time_s <= end and s.driver_leg is leg]
        kinds = sorted({s.driver_segment_kind for s in rows})
        heights = [s.body_z_m for s in rows]
        print(f"  [{start:7.3f}, {end:7.3f}] s  {leg.value}  "
              f"{','.join(kinds):22s} body_z "
              f"{min(heights)*1e3:8.3f} -> {max(heights)*1e3:8.3f} mm")

    plot_body_trajectory_2d(
        trajectory, path=OUT / "day12_step5_body_trajectory.png")
    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
