"""Day 12 Step 4 driver: Day 10--11 crossings placed on the common timeline.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step4_debug_table.csv``    plan §11 requirement 8's table
``day12_step4_leg_plans.csv``      one row per leg plan, per composed cell
``day12_step4_timeline.png``       the four-leg timeline with the crossing on it

``#1 ROLL_ROLL`` re-runs Day 6--7's traversal and takes about three minutes, so
it is opt-in::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step4_driver.py
    python3 -u .../day12_step4_driver.py --with-roll-roll
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

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
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (  # noqa: E402
    LEG_ORDER,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (  # noqa: E402
    plot_timeline_2d,
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (  # noqa: E402
    TransitionPhase,
    build_leg_plan_2d,
    debug_rows,
    plan_four_legs_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"
DAY10_11 = OUT.parent / "day10-11"
DAY6_7 = OUT.parent / "day6-7"

#: Evaluation queries, not planner constants (plan §0.1).  These are the cells
#: **Day 10--11 Step 9 itself** composed at, quoted so that Step 4 runs the
#: crossings that rule picked and does not choose a strategy anywhere.
COMPOSE_HEIGHTS_M = {
    StrategyId.SWING_OVER: 0.060,
    StrategyId.SWING_SWING: 0.080,
}
#: Step 7's first open item: a top length the ``#5`` sweep never measured.
SWING_OVER_TOP_LENGTH_M = 0.075
ROLL_ROLL_HEIGHT_M = 0.140
#: A blocked pair, quoted from Step 9's own cell.  It stays in the run because
#: plan §11 requirement 5 is precisely about an unresolved crossing.
BLOCKED_CELL = (StrategyId.ROLL_SWING, 0.160, 0.350)


def _first_winning_top(tables, height_m: float, strategy: StrategyId):
    """The cell Day 10--11's own rule hands this strategy.  Step 9's helper."""

    for i in range(87):
        top = round(0.02 + 0.005 * i, 4)
        if decide_2d(height_m, top, tables).winner is strategy:
            return top
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--with-roll-roll", action="store_true",
                        help="also compose #1 (about three minutes)")
    args = parser.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    tables = load_tables_2d(DAY10_11, DAY6_7)

    cells = []
    for strategy, height_m in COMPOSE_HEIGHTS_M.items():
        top = _first_winning_top(tables, height_m, strategy)
        if strategy is StrategyId.SWING_OVER:
            top = SWING_OVER_TOP_LENGTH_M
        if top is None:
            print(f"{strategy.value} never wins at h = {height_m*1e3:.0f} mm")
            continue
        cells.append((strategy, height_m, top))
    cells.append(BLOCKED_CELL)
    if args.with_roll_roll:
        top = _first_winning_top(tables, ROLL_ROLL_HEIGHT_M, StrategyId.ROLL_ROLL)
        if top is not None:
            cells.append((StrategyId.ROLL_ROLL, ROLL_ROLL_HEIGHT_M, top))

    plan_rows = []
    headline = None
    for strategy, height_m, top_length_m in cells:
        composed = compose_2d(height_m, top_length_m, tables, strategy=strategy)
        print(f"\n{strategy.value}  h={height_m*1e3:.0f} mm  "
              f"L_top={top_length_m*1e3:.0f} mm")
        print(f"  composed {composed.composed}"
              f"   refusal: {composed.refusal}")

        plans = {leg: build_leg_plan_2d(leg, composed) for leg in LEG_ORDER}
        four = plan_four_legs_2d(plans, walk_timing_2d())

        example = plans[LEG_ORDER[0]]
        print(f"  ascent / descent           "
              f"{example.ascent_strategy} / {example.descent_strategy}"
              f"   separable = {example.phases_are_separable}")
        for phase in TransitionPhase:
            n = len(example.segments_in(phase))
            if n:
                print(f"    {phase.value:16s} {n} segment(s)")
        print(f"  unresolved transitions     {len(example.unresolved)}")
        print(f"  chain breaks               {len(example.breaks)}")
        print(f"  leg plan executable        {example.is_executable}")
        print(f"  scheduled segments         {len(four.schedule.scheduled)}")
        print(f"  max airborne legs          {four.schedule.max_airborne_count}")
        print(f"  timing conflicts           {len(four.schedule.conflicts)}")
        print(f"  airborne overruns          {len(four.airborne_overruns)}")
        for overrun in four.airborne_overruns[:1]:
            row = overrun.as_dict()
            print(f"    {row['segment_kinds']}")
            print(f"    window {row['window_s']:.3f} s must carry "
                  f"{row['planned_s']:.3f} s of already-planned motion"
                  f" plus {row['untimed_segments']} untimed segment(s)"
                  f"  -> {row['compression']:.3f} x compression")
        print(f"  whole plan executable      {four.is_executable}")

        row = {"strategy": strategy.value,
               "obstacle_mm": height_m * 1e3,
               "top_length_mm": top_length_m * 1e3,
               **four.as_dict()}
        plan_rows.append(row)
        if composed.composed and headline is None:
            headline = (strategy, four)

    write_rows_csv(OUT / "day12_step4_leg_plans.csv", plan_rows)

    if headline is None:
        print("\nno cell composed; no debug table to write.")
        return
    strategy, four = headline
    write_rows_csv(OUT / "day12_step4_debug_table.csv", debug_rows(four))
    plot_timeline_2d(four.schedule, path=OUT / "day12_step4_timeline.png")

    print(f"\ndebug table drawn from {strategy.value}")
    print(f"{'start':>8} {'end':>8}  {'leg':>4} {'kind':16s} "
          f"{'state':9s} {'phase':15s} body")
    for row in debug_rows(four):
        if row["row_kind"] != "segment" or row["leg"] != LEG_ORDER[0].value:
            continue
        print(f"{row['start_s']:8.3f} {row['end_s']:8.3f}  {row['leg']:>4} "
              f"{row['segment_kind']:16s} {row['contact_state']:9s} "
              f"{str(row['phase']):15s} {row['body_kind']}")

    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
