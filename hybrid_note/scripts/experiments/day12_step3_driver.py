"""Day 12 Step 3 driver: the four-leg timing skeleton, as tables and a plot.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step3_schedule.csv``       timing, every scheduled segment, conflicts
``day12_step3_rate_demand.csv``    what the duty demands of the recovery rate
``day12_step3_timeline.png``       the four-leg timeline (plan §10 requirement 8)

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step3_driver.py
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

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (  # noqa: E402
    LEG_ORDER,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (  # noqa: E402
    cycle_segments_2d,
    run_nominal_cycles_2d,
)
from hybrid_note.scripts.experiments.day12_segment_contract_2d import (  # noqa: E402
    SegmentChain2D,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (  # noqa: E402
    GaitTiming2D,
    plot_timeline_2d,
    rotation_rate_demand_2d,
    schedule_chains_2d,
    schedule_rows,
    walk_timing_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"


def nominal_chains(cycles: int = 2) -> tuple[dict, object]:
    """One nominal chain per leg.

    The same chain is given to all four legs on purpose: on flat ground every
    leg runs the identical cycle, and only the **phase offset** distinguishes
    them.  The legs' different ``x`` placements are Step 2's (registration) and
    Step 4's (mapping sequences onto the terrain); Step 3 is timing alone.
    """

    generated = run_nominal_cycles_2d(cycles)
    segments, offset = [], 0
    for cycle in generated:
        pair = cycle_segments_2d(cycle, source_id="day12_step3",
                                 frame_offset=offset)
        segments.extend(pair)
        offset += sum(s.frames.frame_count for s in pair)
    chains = {
        leg: SegmentChain2D(leg_id=leg.value, segments=tuple(segments))
        for leg in LEG_ORDER
    }
    return chains, generated[0]


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    timing = walk_timing_2d()
    print(f"gait           {timing.gait_name}  (from GAIT_LIBRARY)")
    print(f"stance duty    {timing.stance_duty}")
    print(f"cycle period   {timing.cycle_period_s} s  "
          f"-> stance {timing.stance_duration_s} s / "
          f"swing {timing.swing_duration_s} s")
    print(f"duty alone allows {timing.max_simultaneous_airborne} airborne leg(s)")
    print("\nswing windows (fractions of one cycle)")
    for leg in sorted(LEG_ORDER, key=lambda l: timing.swing_window(l)[0]):
        lo, hi = timing.swing_window(leg)
        print(f"  {leg.value}  [{lo:.2f}, {hi:.2f})")
    order = sorted(LEG_ORDER, key=lambda l: timing.swing_window(l)[0])
    print(f"  swing order: {' -> '.join(l.value for l in order)}")

    chains, first_cycle = nominal_chains()

    demand_rows = []
    for period in (0.8, 2.4, 9.0):
        row = rotation_rate_demand_2d(
            first_cycle.stroke.rotation_rad, first_cycle.recovery.rotation_rad,
            walk_timing_2d(period),
        )
        demand_rows.append({"cycle_period_s": period, **row})
    write_rows_csv(OUT / "day12_step3_rate_demand.csv", demand_rows)

    demand = demand_rows[1]
    print("\nwhat 'one airborne leg' costs")
    print(f"  Step 1 rotations           "
          f"{demand['stroke_rotation_deg']:.2f} deg rolling + "
          f"{demand['recovery_rotation_deg']:.2f} deg airborne")
    print(f"  duty if time followed rotation  "
          f"{demand['rotation_proportional_duty']:.4f}"
          f"   -> would need "
          f"{int(np.ceil(4*(1-demand['rotation_proportional_duty'])))} legs airborne")
    print(f"  duty actually used              {demand['stance_duty']:.2f}")
    print(f"  rolling  beta rate         "
          f"{demand['stance_rate_deg_per_s']:9.2f} deg/s")
    print(f"  recovery beta rate         "
          f"{demand['swing_rate_deg_per_s']:9.2f} deg/s")
    print(f"  ratio                      "
          f"{demand['swing_to_stance_rate_ratio']:9.3f} x"
          f"   <- the price, in beta speed")
    ratios = {round(r["swing_to_stance_rate_ratio"], 9) for r in demand_rows}
    print(f"  ratio is the same at T = 0.8 / 2.4 / 9.0 s: {len(ratios) == 1}"
          f"   (nothing turns on the period Step 3 had to choose)")

    schedule = schedule_chains_2d(chains, timing)
    write_rows_csv(OUT / "day12_step3_schedule.csv", schedule_rows(schedule))

    lo, hi = schedule.covered_interval_s
    print(f"\nschedule       {len(schedule.scheduled)} segments over "
          f"[{schedule.start_s:.3f}, {schedule.end_s:.3f}] s")
    print(f"  covered interval           [{lo:.3f}, {hi:.3f}] s  "
          f"(all four legs scheduled)")
    print(f"  ragged ends                "
          f"{[tuple(round(v, 3) for v in r) for r in schedule.ragged_intervals_s]}")
    print(f"  max airborne legs          {schedule.max_airborne_count}")
    print(f"  one airborne at a time     {schedule.one_leg_airborne_at_a_time}")
    print(f"  every swing has 3 supports {schedule.every_swing_has_three_supports}")
    print(f"  conflicts                  {len(schedule.conflicts)}")
    for conflict in schedule.conflicts:
        print(f"    {conflict.as_dict()}")

    print(f"\n{'time':>7}  {'airborne':>9}  support")
    for t in np.linspace(lo, hi, 13)[:-1]:
        airborne = schedule.airborne_legs_at(float(t))
        support = schedule.support_legs_at(float(t))
        print(f"{t:7.3f}  {(airborne[0].value if airborne else '-'):>9}  "
              f"{', '.join(l.value for l in support)}")

    print("\nper-leg timeline")
    for leg in LEG_ORDER:
        print(f"  {leg.value}")
        for segment in schedule.segments_of(leg):
            print(f"    [{segment.start_s:7.3f}, {segment.end_s:7.3f}] "
                  f"{segment.mode.value:8s} {segment.segment_kind.value:16s} "
                  f"frames {segment.frame_count:3d}  "
                  f"duration {segment.duration_s:.3f} s (assigned)")

    plot_timeline_2d(schedule, path=OUT / "day12_step3_timeline.png")
    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
