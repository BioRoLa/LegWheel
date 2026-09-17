"""Day 12 problems B1/B2: does the decision rule ever choose rolling?

Outputs (into ``hybrid_note/notes/day12/``):

``day12_b1_decision_criterion.csv``   every ordering x tolerance, and the map
``day12_b1_decision_criterion.png``   rolling's share against the tolerance

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_b1_decision_criterion_driver.py

**The question.**  The audit found that at every evaluation size the winner is
``#4 SWING_UP + SWING_DOWN`` even though ``#1 ROLL_UP + ROLL_DOWN`` is
feasible -- so the Hybrid's rolling contact is never used to cross anything,
and on obstacles the gait is a Walk.  This driver measures why, and what each
alternative rule would do instead.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    DEFAULT_ORDER,
    Availability,
    StrategyId,
    decide_2d,
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"
DAY10_11 = OUT.parent / "day10-11"
DAY6_7 = OUT.parent / "day6-7"

#: ``DEFAULT_ORDER`` puts ``margin`` before ``roll_preference``, so even a tie
#: on ``body`` is intercepted by clearance and the preference never fires.
#: This is the order a tolerance has to be used with for it to mean anything.
BODY_ROLL_ORDER = ("feasible", "body", "roll_preference", "margin")

ORDERS = {"body,margin,roll (current)": DEFAULT_ORDER,
          "body,roll,margin": BODY_ROLL_ORDER}
TOLERANCES_M = (0.0, 0.005, 0.010, 0.015, 0.020, 0.030, 0.050)
HEIGHTS_M = np.arange(0.02, 0.205, 0.01)
TOPS_M = np.arange(0.20, 0.62, 0.02)
EVALUATION_M = (0.04, 0.10, 0.19)


def sweep(tables, order, tolerance_m: float) -> dict:
    roll = over = swing = solved = 0
    worst_penalty_mm = 0.0
    boundary = set()
    for height_m in HEIGHTS_M:
        for top_m in TOPS_M:
            decision = decide_2d(float(height_m), float(top_m), tables,
                                 order=order, body_tolerance_m=tolerance_m)
            if decision.winner is None:
                continue
            solved += 1
            boundary.add((round(float(height_m), 4), round(float(top_m), 4)))
            if decision.winner is StrategyId.ROLL_ROLL:
                roll += 1
            elif decision.winner is StrategyId.SWING_OVER:
                over += 1
            elif decision.winner is StrategyId.SWING_SWING:
                swing += 1
            usable = [c.body_deviation_m for c in decision.cells
                      if c.feasible and c.body_deviation_m is not None]
            won = next(c for c in decision.cells if c.strategy is decision.winner)
            if usable and won.body_deviation_m is not None:
                worst_penalty_mm = max(
                    worst_penalty_mm,
                    (won.body_deviation_m - min(usable)) * 1e3)
    return {"roll": roll, "swing_over": over, "swing_swing": swing,
            "solved": solved, "worst_body_penalty_mm": worst_penalty_mm,
            "solvable_cells": boundary}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    tables = load_tables_2d(
        DAY10_11, DAY6_7,
        swing_over_csv=DAY10_11 / "day10_11_step5_swing_over.csv")

    # ---- 1. the mechanism, at the evaluation sizes -------------------------
    print("1. what the current rule decides, and what was available")
    for height_m in EVALUATION_M:
        decision = decide_2d(height_m, 0.40, tables)
        print(f"\n   h = {height_m*1e3:.0f} mm, L_top = 400 mm   -> "
              f"{decision.winner.value if decision.winner else 'NO STRATEGY'}")
        for cell in decision.cells:
            deviation = cell.body_deviation_m
            print(f"     {cell.strategy.value:<24} {cell.availability.value:<14} "
                  f"body "
                  f"{'    n/a' if deviation is None else f'{deviation*1e3:7.2f} mm'}")

    # ---- 2. every rule -----------------------------------------------------
    print(f"\n2. rolling's share of the solvable map, by rule")
    print(f"   {'ordering':>26} {'tol mm':>7} {'ROLL':>6} {'OVER':>6} "
          f"{'SWING':>6} {'roll share':>11} {'worst body cost':>16}")
    rows: list[dict] = []
    reference: set | None = None
    for label, order in ORDERS.items():
        for tolerance_m in TOLERANCES_M:
            result = sweep(tables, order, tolerance_m)
            cells = result.pop("solvable_cells")
            if reference is None:
                reference = cells
            moved = len(reference ^ cells)
            share = result["roll"] / result["solved"] * 100.0
            print(f"   {label:>26} {tolerance_m*1e3:7.0f} {result['roll']:6d} "
                  f"{result['swing_over']:6d} {result['swing_swing']:6d} "
                  f"{share:10.1f}% {result['worst_body_penalty_mm']:15.2f} mm")
            rows.append({"row_kind": "rule", "ordering": label,
                         "body_tolerance_mm": tolerance_m * 1e3,
                         **result, "roll_share_percent": share,
                         "feasibility_cells_moved": moved})

    moved_total = {row["feasibility_cells_moved"] for row in rows}
    print(f"\n   cells whose SOLVABILITY changed, across every rule above: "
          f"{sorted(moved_total)}")
    print(f"   -- the rule decides who wins.  It moves no region boundary at "
          f"all, which is")
    print(f"      what the Day 10-11 spec's last completion criterion asked "
          f"about.")

    # ---- 3. the evaluation sizes under each rule ---------------------------
    print(f"\n3. the evaluation sizes, under each rule")
    print(f"   {'ordering':>26} {'tol mm':>7} " +
          " ".join(f"{f'h={h*1e3:.0f}':>26}" for h in EVALUATION_M))
    for label, order in ORDERS.items():
        for tolerance_m in (0.0, 0.010, 0.015, 0.020):
            cells = []
            for height_m in EVALUATION_M:
                decision = decide_2d(height_m, 0.40, tables, order=order,
                                     body_tolerance_m=tolerance_m)
                cells.append(f"{(decision.winner.value if decision.winner else 'NONE')[:26]:>26}")
                rows.append({"row_kind": "evaluation_size", "ordering": label,
                             "body_tolerance_mm": tolerance_m * 1e3,
                             "height_mm": height_m * 1e3, "top_length_mm": 400.0,
                             "winner": (decision.winner.value
                                        if decision.winner else None)})
            print(f"   {label:>26} {tolerance_m*1e3:7.0f} " + " ".join(cells))

    # ---- 4. the strategy the audit got wrong -------------------------------
    print(f"\n4. #5 SWING_OVER -- the audit called this 'never measured'")
    counts: dict[str, int] = {}
    feasible_at = []
    for height_m in HEIGHTS_M:
        for top_m in TOPS_M:
            decision = decide_2d(float(height_m), float(top_m), tables)
            cell = next(c for c in decision.cells
                        if c.strategy is StrategyId.SWING_OVER)
            counts[cell.availability.value] = counts.get(
                cell.availability.value, 0) + 1
            if cell.feasible:
                feasible_at.append((float(height_m), float(top_m)))
    print(f"   across the map: {counts}")
    if feasible_at:
        print(f"   FEASIBLE in {len(feasible_at)} cells: "
              f"h {min(h for h, _ in feasible_at)*1e3:.0f}-"
              f"{max(h for h, _ in feasible_at)*1e3:.0f} mm, "
              f"L_top {min(l for _, l in feasible_at)*1e3:.0f}-"
              f"{max(l for _, l in feasible_at)*1e3:.0f} mm")
        print(f"   -- so it is measured, it is feasible, and at body "
              f"deviation 0.00 mm it wins")
        print(f"      every cell it is available in.  The audit entry B3 was "
              f"wrong; see log 1.7.")
    rows.append({"row_kind": "swing_over_availability", **counts,
                 "feasible_cells": len(feasible_at)})

    # Day 10--11 trap 21: ``write_rows_csv`` takes the header from the first
    # row, so rows of different shapes must be squared off first.
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    write_rows_csv(OUT / "day12_b1_decision_criterion.csv",
                   [{key: row.get(key, "") for key in keys} for row in rows])

    fig, axis = plt.subplots(figsize=(7.2, 4.4))
    for label, order in ORDERS.items():
        shares = []
        for tolerance_m in TOLERANCES_M:
            result = sweep(tables, order, tolerance_m)
            shares.append(result["roll"] / result["solved"] * 100.0)
        axis.plot([t * 1e3 for t in TOLERANCES_M], shares, marker="o", lw=1.8,
                  label=label)
    axis.set_xlabel("body tolerance [mm]")
    axis.set_ylabel("cells won by rolling [% of solvable]")
    axis.set_title("how often the Hybrid actually rolls across an obstacle",
                   fontsize=10, loc="left")
    axis.legend(fontsize=8, frameon=False)
    axis.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(OUT / "day12_b1_decision_criterion.png", dpi=150)
    plt.close(fig)

    print(f"\nwrote -> {OUT / 'day12_b1_decision_criterion.csv'}")
    print(f"wrote -> {OUT / 'day12_b1_decision_criterion.png'}")


if __name__ == "__main__":
    main()
