"""Day 10--11 Step 0 driver: produce the alignment and handoff CSVs.

Kept out of the notebook on purpose.  One full rolling traversal takes minutes,
so the handoff half of Step 0 is a batch job; the notebook reads what this
wrote and re-runs only the cheap standing checks live.  That split is the same
one Day 6--7 and Day 8--9 settled on -- notebooks show summaries and endpoints,
CSVs hold the full record.

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step0_driver.py

The traversals are independent, so they run in a process pool.  Everything
passed to a worker is a plain dataclass or float, which is what keeps them
picklable -- the same constraint ``right_up_left_down_sweep_2d`` works under.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

if __package__ in (None, ""):  # direct execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
    alignment_rows,
    check_standing_alignment_2d,
    handoff_rows,
    roll_exit_swing_start_2d,
    rolling_inputs_2d,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d import (  # noqa: E402
    STAGE_ROLL_DOWN,
    STAGE_ROLL_UP,
    STAGE_WHEEL_TRANSITION,
    TraversalConstraints2D,
    check_right_up_left_down_traversal,
)
from hybrid_note.scripts.experiments.right_up_left_down_sweep_2d import (  # noqa: E402
    seam_bridge_for_sampling_m,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"

#: Heights that straddle everything Day 6--7 measured: comfortably feasible,
#: the narrow theta window at 0.12, and the last feasible row before the
#: rolling ceiling at 0.16.
HEIGHTS_M: tuple[float, ...] = (0.06, 0.10, 0.12, 0.14)
THETAS_DEG: tuple[float, ...] = (40.0, 50.0, 60.0, 70.0, 85.0)
CLEARANCES_M: tuple[float, ...] = (0.02, 0.04, 0.08, 0.12)
HANDOFF_STAGES: tuple[str, ...] = (STAGE_ROLL_UP, STAGE_WHEEL_TRANSITION, STAGE_ROLL_DOWN)


def _handoff_task(task):
    """One cell: run the traversal once, then read every stage exit off it."""

    height_m, theta_deg, top_length_m, arc_samples, clearance_m = task
    spec = SharedTerrainSpec2D(
        height_m=height_m, top_length_m=top_length_m, arc_samples=arc_samples
    )
    theta = float(np.deg2rad(theta_deg))
    constraints = TraversalConstraints2D(
        max_seam_bridge_m=seam_bridge_for_sampling_m(arc_samples)
    )
    started = time.time()
    obstacle, initial_state = rolling_inputs_2d(spec, theta, clearance_m)
    result = check_right_up_left_down_traversal(
        obstacle, initial_state, theta_climb=theta, constraints=constraints
    )
    rows = []
    for stage in HANDOFF_STAGES:
        handoff = roll_exit_swing_start_2d(
            spec, theta, clearance_m=clearance_m, stage=stage,
            constraints=constraints, result=result,
        )
        row = {
            "height_mm": spec.height_m * 1e3,
            "top_length_m": spec.top_length_m,
            "theta_climb_deg": theta_deg,
            "approach_clearance_mm": clearance_m * 1e3,
        }
        row.update(handoff.as_dict())
        row["seconds"] = round(time.time() - started, 2)
        rows.append(row)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Day 10--11 Step 0 driver")
    parser.add_argument("--top-length-m", type=float, default=0.35)
    parser.add_argument("--arc-samples", type=int, default=121)
    parser.add_argument("--handoff-clearance-m", type=float, default=0.04)
    parser.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument("--skip-handoff", action="store_true")
    parser.add_argument("--skip-alignment", action="store_true",
                        help="reuse the alignment CSV already on disk")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    # ---- half 1: standing alignment (cheap, swept widely) ----------------
    started = time.time()
    alignments = []
    if args.skip_alignment:
        print("standing alignment: skipped, reusing the CSV on disk", flush=True)
    else:
        for height in HEIGHTS_M:
            spec = SharedTerrainSpec2D(
                height_m=height, top_length_m=args.top_length_m, arc_samples=args.arc_samples
            )
            for theta_deg in THETAS_DEG:
                theta = float(np.deg2rad(theta_deg))
                for clearance in CLEARANCES_M:
                    alignments.append(check_standing_alignment_2d(spec, theta, clearance))
        rows = alignment_rows(alignments)
        path = write_rows_csv(args.output_dir / "day10_11_step0_scene_alignment.csv", rows)
        agreed = sum(1 for item in alignments if item.explained_by_surface_offset)
        worst_clearance_error_um = max(
            abs(item.measured_clearance_m - item.clearance_m) * 1e6
            for item in alignments
            if item.measured_clearance_m is not None
        )
        print(
            f"standing alignment: {agreed}/{len(alignments)} cells agree to the 1 nm "
            f"surface offset; worst realised-clearance error "
            f"{worst_clearance_error_um:.3f} um  ({time.time() - started:.1f}s)",
            flush=True,
        )
        print(f"  wrote {path}", flush=True)

    if args.skip_handoff:
        return

    # ---- half 2: roll-exit handoff (expensive, swept narrowly) -----------
    tasks = [
        (height, theta_deg, args.top_length_m, args.arc_samples, args.handoff_clearance_m)
        for height in HEIGHTS_M
        for theta_deg in THETAS_DEG
    ]
    print(
        f"handoff: {len(tasks)} traversals on {args.workers} workers "
        f"(minutes each, so this is the long half)",
        flush=True,
    )
    started = time.time()
    handoff_rows_out: list[dict] = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for index, rows in enumerate(pool.map(_handoff_task, tasks), start=1):
            handoff_rows_out.extend(rows)
            head = rows[0]
            print(
                f"  [{index}/{len(tasks)}] h={head['height_mm']:.0f} mm "
                f"theta={head['theta_climb_deg']:.0f} deg -> "
                f"{'full' if head['feasible_traversal'] else (head['failure_stage'] or 'partial')} "
                f"({head['seconds']:.1f}s)",
                flush=True,
            )
    path = write_rows_csv(
        args.output_dir / "day10_11_step0_roll_exit_handoff.csv", handoff_rows_out
    )
    print(f"handoff done in {time.time() - started:.1f}s", flush=True)
    print(f"  wrote {path}", flush=True)


if __name__ == "__main__":
    main()
