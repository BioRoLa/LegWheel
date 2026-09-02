"""Day 10--11 Step 1 driver: prove the concession contract is lossless.

The claim Step 1 has to establish is narrow and checkable:

    every Day 8--9 showcase can be re-expressed as a SwingConcession2D
    with nothing left behind in the prose.

So this runs the showcases, converts them, rebuilds the ``adjustments`` strings
*from the converted fields*, and compares those against what the showcase
actually printed.  A reconstruction is used rather than a parser on purpose --
a parser proves only that the strings can be read, while a reconstruction
proves the fields alone are enough to produce them.

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step1_driver.py

The heights are Day 8--9 §28.1's, plus 150 mm -- the greedy-search hole
measured on 2026-08-29, which has to survive into the contract as an
infeasible cell with a ``REACH`` ceiling rather than disappearing.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

if __package__ in (None, ""):  # direct execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from hybrid_note.scripts.experiments.day10_11_concession_2d import (  # noqa: E402
    BindingCeiling,
    ConcessionSource,
    adjustment_strings_from_concession,
    swing_concession_from_showcase,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)
from hybrid_note.scripts.experiments.cartesian_swing_planner_2d import (  # noqa: E402
    swing_off_step_2d,
    swing_onto_step_2d,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"

#: Day 8--9 §28.1's ladder, with 150 mm inserted.
HEIGHTS_MM: tuple[int, ...] = (20, 60, 100, 120, 140, 150, 160, 200)
SHOWCASE_KWARGS = dict(sample_count=31, arc_samples=61)


def _one(task) -> dict:
    direction, height_mm = task
    started = time.time()
    runner = swing_onto_step_2d if direction == "onto" else swing_off_step_2d
    showcase = runner(height_mm / 1000.0, **SHOWCASE_KWARGS)
    concession = swing_concession_from_showcase(
        showcase, source=ConcessionSource.GREEDY_LADDER
    )
    rebuilt = adjustment_strings_from_concession(
        concession, original_duration_s=showcase.original_duration_s
    )
    row = {"height_mm": height_mm}
    row.update(concession.as_dict())
    row["showcase_adjustments"] = "; ".join(showcase.adjustments) or "none"
    row["rebuilt_adjustments"] = "; ".join(rebuilt) or "none"
    row["lossless"] = tuple(showcase.adjustments) == rebuilt
    row["seconds"] = round(time.time() - started, 2)
    return row


def main() -> None:
    parser = argparse.ArgumentParser(description="Day 10--11 Step 1 driver")
    parser.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    tasks = [
        (direction, height_mm)
        for direction in ("onto", "off")
        for height_mm in HEIGHTS_MM
    ]
    print(f"step 1: {len(tasks)} showcases on {args.workers} workers", flush=True)
    started = time.time()
    rows: list[dict] = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for row in pool.map(_one, tasks):
            rows.append(row)
            print(
                f"  {row['direction']:>4} {row['height_mm']:>4} mm  "
                f"feasible={str(row['feasible']):<5} "
                f"ceiling={row['binding_ceiling']:<8} "
                f"body={row['body_deviation_mm']} mm  "
                f"lossless={row['lossless']}  ({row['seconds']}s)",
                flush=True,
            )

    path = write_rows_csv(args.output_dir / "day10_11_step1_swing_concessions.csv", rows)
    lossless = sum(1 for row in rows if row["lossless"])
    ceilings = {}
    for row in rows:
        ceilings[row["binding_ceiling"]] = ceilings.get(row["binding_ceiling"], 0) + 1
    print(f"\nlossless: {lossless}/{len(rows)}", flush=True)
    print(f"binding ceilings: {ceilings}", flush=True)
    print(f"total {time.time() - started:.1f}s; wrote {path}", flush=True)

    if lossless != len(rows):
        broken = [row for row in rows if not row["lossless"]]
        print("\nNOT LOSSLESS -- the contract is incomplete:", flush=True)
        for row in broken:
            print(
                f"  {row['direction']} {row['height_mm']} mm\n"
                f"    showcase: {row['showcase_adjustments']}\n"
                f"    rebuilt : {row['rebuilt_adjustments']}",
                flush=True,
            )
        raise SystemExit(1)


if __name__ == "__main__":
    main()
