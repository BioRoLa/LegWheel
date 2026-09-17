"""Day 12 Step 10 driver: the same planner, four terrains.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step10_generalization.csv``   plan §17 requirement 7's comparison table
``day12_step10_size_literals.csv``    what the generalization gate found

The evaluation set lives **here**, in the driver, as queries.  Plan §0.1: the
planner must not contain it.

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step10_driver.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (  # noqa: E402
    comparison_rows,
    nominal_body_height_m,
    plan_terrain_2d,
    planner_size_literals,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"
DAY10_11 = OUT.parent / "day10-11"
DAY6_7 = OUT.parent / "day6-7"

#: Plan §17's Tests A-D.  **Evaluation queries, not planner constants.**
#: ``None`` is flat ground -- the absence of an obstacle, not a size of zero.
TERRAIN_QUERIES: tuple[tuple[str, float | None, float], ...] = (
    ("A  flat", None, 0.0),
    ("B  4 cm", 0.04, 0.40),
    ("C  10 cm", 0.10, 0.40),
    ("D  19 cm (challenge)", 0.19, 0.40),
)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    tables = load_tables_2d(DAY10_11, DAY6_7)

    print(f"nominal body height  {nominal_body_height_m() * 1e3:.4f} mm"
          f"   (solved from the leg, not from a platform)")

    runs = []
    for label, height_m, top_length_m in TERRAIN_QUERIES:
        terrain = None if height_m is None else SharedTerrainSpec2D(
            height_m=height_m, top_length_m=top_length_m, x_start_m=1.00,
            obstacle_id="day12_platform")
        started = time.perf_counter()
        run = plan_terrain_2d(terrain, tables)
        runs.append(run)
        row = run.as_dict()
        print(f"\n{label}   ({time.perf_counter() - started:.0f} s)")
        print(f"  planned                 {row['planned']}")
        print(f"  feasible                {row['feasible']}")
        print(f"  ascent / descent        {row['ascent_primitive']} / "
              f"{row['descent_primitive']}")
        print(f"  nominal recovery swings {row['nominal_recovery_swings']}")
        print(f"  terrain-transition swings {row['terrain_transition_swings']}")
        print(f"  max body lift           {row['max_body_lift_mm']}"
              f"   (usable body samples: {row['usable_body_samples']})")
        print(f"  min stability margin    {row['min_stability_margin_mm']}")
        if run.first_limiting_constraint is not None:
            first = run.first_limiting_constraint
            print(f"  first limiting constraint  [{first.stage.value}] "
                  f"{first.detail[:150]}")

    write_rows_csv(OUT / "day12_step10_generalization.csv",
                   comparison_rows(runs))

    print("\n" + "=" * 78)
    print("plan §17 requirement 7: the comparison table")
    print(f"{'terrain':>14} {'feas':>5} {'ascent':>10} {'descent':>10} "
          f"{'nom sw':>7} {'tt sw':>6} {'lift mm':>9} {'margin mm':>10}")
    print("  lift is n/a wherever Step 5 left too few usable body heights to "
          "measure one")
    for run in runs:
        row = run.as_dict()
        lift = row["max_body_lift_mm"]
        margin = row["min_stability_margin_mm"]
        print(f"{row['terrain']:>14} {str(row['feasible']):>5} "
              f"{str(row['ascent_primitive']):>10} "
              f"{str(row['descent_primitive']):>10} "
              f"{row['nominal_recovery_swings']:>7} "
              f"{row['terrain_transition_swings']:>6} "
              f"{'n/a' if lift is None else format(lift, '9.3f')} "
              f"{'n/a' if margin is None else format(margin, '10.3f')}")

    print("\n" + "=" * 78)
    print("the generalization gate: evaluation-size literals in planner modules")
    found = planner_size_literals()
    rows = []
    if not found:
        print("  none -- no planner module mentions an evaluation size")
    for name, lines in found.items():
        print(f"  {name}")
        for line in lines:
            print(f"    {line[:110]}")
            rows.append({"module": name, "line": line})
    write_rows_csv(OUT / "day12_step10_size_literals.csv",
                   rows or [{"module": "", "line": ""}])

    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
