"""Day 10--11 Step 2 driver: sweep the ascent map and draw it.

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step2_driver.py

Produces the CSV plus three figures, and prints the checks Step 2 has to pass:

1. the §26.5(6) *effect* reproduces -- at some height, a small approach
   clearance fails where a larger one works.  Note the spec also expected the
   turning point to sit near the wheel radius; the sweep refutes that, see
   ``WHEEL_RADIUS_M`` below;
2. feasibility is monotone in height at fixed clearance, i.e. the 150 mm
   greedy hole does not survive a grid search;
3. 200 mm has a definite answer at every clearance.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

if __package__ in (None, ""):  # direct execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day10_11_swing_sweep_2d import (  # noqa: E402
    DEFAULT_APPROACH_CLEARANCES_M,
    DEFAULT_HEIGHTS_MM,
    SwingGridSettings2D,
    run_swing_onto_grid_2d,
    swing_grid_rows,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"

#: The wheel's outer radius.  Day 8--9 §26.5(6) proposed it as the threshold on
#: this axis -- "the tyre is inside the step before the contact point has moved".
#: The sweep does **not** support that for this geometry: measured at h = 150 mm,
#: the contact-point-to-face distance is 176 mm at c = 20 mm and 319 mm at
#: c = 160 mm, so it never drops below 145 mm anywhere on the axis, yet cells
#: still fail at c = 20 mm.  The line is kept on the figures as the reference
#: the data refutes, not as a fitted threshold.
WHEEL_RADIUS_M = 0.145

#: Where the axis actually stops buying anything, read off the sweep.
SATURATION_CLEARANCE_M = 0.06


def _pivot(rows, value_key):
    heights = sorted({row["height_mm"] for row in rows})
    clearances = sorted({row["approach_clearance_mm"] for row in rows})
    grid = np.full((len(heights), len(clearances)), np.nan)
    for row in rows:
        i = heights.index(row["height_mm"])
        j = clearances.index(row["approach_clearance_mm"])
        value = row[value_key]
        grid[i, j] = np.nan if value is None else float(value)
    return np.array(heights), np.array(clearances), grid


def _plot_min_hip_lift(rows, path: Path) -> Path:
    heights, clearances, grid = _pivot(rows, "min_hip_lift_mm")
    fig, ax = plt.subplots(figsize=(8.2, 5.2))
    mesh = ax.imshow(grid, origin="lower", aspect="auto", cmap="viridis",
                     extent=(clearances[0] - 10, clearances[-1] + 10,
                             heights[0] - 10, heights[-1] + 10))
    for i, height in enumerate(heights):
        for j, clearance in enumerate(clearances):
            value = grid[i, j]
            ax.text(clearance, height,
                    "x" if np.isnan(value) else f"{value:.0f}",
                    ha="center", va="center", fontsize=7.5,
                    color="#dc2626" if np.isnan(value) else "white")
    ax.axvline(SATURATION_CLEARANCE_M * 1e3, color="#f97316", ls="-", lw=1.6,
               label=f"saturation, measured: {SATURATION_CLEARANCE_M * 1e3:.0f} mm")
    ax.axvline(WHEEL_RADIUS_M * 1e3, color="#94a3b8", ls=":", lw=1.4,
               label=f"wheel radius {WHEEL_RADIUS_M * 1e3:.0f} mm (refuted)")
    ax.set_xlabel("approach clearance [mm]")
    ax.set_ylabel("step height [mm]")
    ax.set_title("Step 2 - minimum hip lift a swing onto the step demands [mm]\n"
                 "x = infeasible at every (hip lift, lift-off) on the grid")
    ax.legend(fontsize=8, loc="upper left")
    fig.colorbar(mesh, ax=ax, label="min hip lift [mm]")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def _plot_binding_ceiling(rows, path: Path) -> Path:
    order = {"none": 0, "fit": 1, "reach": 2, "stance": 3}
    colours = ["#16a34a", "#f59e0b", "#dc2626", "#7c3aed"]
    heights = sorted({row["height_mm"] for row in rows})
    clearances = sorted({row["approach_clearance_mm"] for row in rows})
    grid = np.full((len(heights), len(clearances)), np.nan)
    for row in rows:
        grid[heights.index(row["height_mm"]),
             clearances.index(row["approach_clearance_mm"])] = order[row["binding_ceiling"]]

    fig, ax = plt.subplots(figsize=(8.2, 5.2))
    ax.imshow(grid, origin="lower", aspect="auto",
              cmap=matplotlib.colors.ListedColormap(colours), vmin=-0.5, vmax=3.5,
              extent=(clearances[0] - 10, clearances[-1] + 10,
                      heights[0] - 10, heights[-1] + 10))
    ax.axvline(SATURATION_CLEARANCE_M * 1e3, color="#0f172a", ls="-", lw=1.6)
    ax.set_xlabel("approach clearance [mm]")
    ax.set_ylabel("step height [mm]")
    ax.set_title("Step 2 - which ceiling binds\n"
                 "green feasible / amber fit / red reach / purple stance")
    handles = [plt.Line2D([], [], marker="s", ls="", color=colour, label=name)
               for name, colour in zip(order, colours)]
    ax.legend(handles=handles, fontsize=8, loc="upper left", ncol=2)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def _plot_min_liftoff(rows, path: Path) -> Path:
    """The knob that fixes a small approach, which is what names the mechanism.

    A lift-off rise raises the leg vertically at the *start* of the swing.  That
    it is the knob a tight approach needs -- 50 mm at c = 20 mm for every step
    from 60 mm up, and 0 mm from c = 60 mm on -- says the problem is the leg
    body sweeping into the face while it is still next to it, not the static
    contact-point geometry §26.5(6) proposed.
    """

    heights, clearances, grid = _pivot(rows, "min_liftoff_rise_mm")
    fig, ax = plt.subplots(figsize=(8.2, 5.2))
    mesh = ax.imshow(grid, origin="lower", aspect="auto", cmap="magma",
                     extent=(clearances[0] - 10, clearances[-1] + 10,
                             heights[0] - 10, heights[-1] + 10))
    for i, height in enumerate(heights):
        for j, clearance in enumerate(clearances):
            value = grid[i, j]
            ax.text(clearance, height, "x" if np.isnan(value) else f"{value:.0f}",
                    ha="center", va="center", fontsize=7.5,
                    color="#dc2626" if np.isnan(value) else "white")
    ax.axvline(SATURATION_CLEARANCE_M * 1e3, color="#38bdf8", ls="-", lw=1.6,
               label=f"saturation {SATURATION_CLEARANCE_M * 1e3:.0f} mm")
    ax.set_xlabel("approach clearance [mm]")
    ax.set_ylabel("step height [mm]")
    ax.set_title("Step 2 - minimum lift-off rise [mm]\n"
                 "the knob a tight approach needs, and it vanishes past saturation")
    ax.legend(fontsize=8, loc="upper right")
    fig.colorbar(mesh, ax=ax, label="min lift-off rise [mm]")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def _report_checks(rows) -> None:
    heights = sorted({row["height_mm"] for row in rows})
    clearances = sorted({row["approach_clearance_mm"] for row in rows})
    lookup = {(row["height_mm"], row["approach_clearance_mm"]): row for row in rows}

    print("\n--- check 1: does a larger approach clearance rescue a failing cell? ---",
          flush=True)
    rescued = 0
    for height in heights:
        feasible = [bool(lookup[(height, c)]["feasible"]) for c in clearances]
        if any(feasible) and not all(feasible):
            first = clearances[feasible.index(True)]
            failing = [c for c, ok in zip(clearances, feasible) if not ok]
            rescued += 1
            print(f"  h={height:>5.0f} mm: fails at {failing} mm, "
                  f"first works at c={first:.0f} mm", flush=True)
    if not rescued:
        print("  none -- the clearance axis never flipped a cell.", flush=True)

    print("\n--- check 2: is feasibility monotone in height at fixed clearance? ---",
          flush=True)
    holes = []
    for clearance in clearances:
        column = [bool(lookup[(h, clearance)]["feasible"]) for h in heights]
        for i in range(1, len(column) - 1):
            if not column[i] and column[i + 1]:
                holes.append((heights[i], clearance))
    if holes:
        print(f"  {len(holes)} hole(s): {holes}", flush=True)
    else:
        print("  no holes -- every infeasible height stays infeasible above it.",
              flush=True)
    for clearance in clearances:
        column = [bool(lookup[(h, clearance)]["feasible"]) for h in heights]
        ceiling = max((h for h, ok in zip(heights, column) if ok), default=None)
        print(f"  c={clearance:>5.0f} mm -> tallest feasible step "
              f"{'none' if ceiling is None else f'{ceiling:.0f} mm'}", flush=True)

    print("\n--- check 3: 200 mm ---", flush=True)
    for clearance in clearances:
        row = lookup[(200.0, clearance)]
        print(f"  c={clearance:>5.0f} mm -> feasible={bool(row['feasible'])} "
              f"ceiling={row['binding_ceiling']} failure={row['failure']}", flush=True)

    print("\n--- contract: no grid cell may carry the greedy signature ---", flush=True)
    flagged = [r for r in rows if r["greedy_backtrack_suspected"]]
    print(f"  greedy_backtrack_suspected: {len(flagged)} (must be 0)", flush=True)
    sources = {row["source"] for row in rows}
    print(f"  sources: {sources} (must be {{'grid_minimum'}})", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Day 10--11 Step 2 driver")
    parser.add_argument("--workers", type=int,
                        default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--plots-only", action="store_true",
                        help="redraw the figures and re-run the checks from the CSV")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    if args.plots_only:
        import csv as _csv
        with (args.output_dir / "day10_11_step2_swing_onto_sweep.csv").open() as handle:
            rows = []
            for raw in _csv.DictReader(handle):
                row = {}
                for key, value in raw.items():
                    if value == "":
                        row[key] = None
                    elif value in ("True", "False"):
                        row[key] = value == "True"
                    else:
                        try:
                            row[key] = float(value)
                        except ValueError:
                            row[key] = value
                rows.append(row)
        for name, fn in (("min_hip_lift_map", _plot_min_hip_lift),
                         ("binding_ceiling_map", _plot_binding_ceiling),
                         ("min_liftoff_map", _plot_min_liftoff)):
            print(f"  wrote {fn(rows, args.output_dir / f'day10_11_step2_{name}.png')}",
                  flush=True)
        _report_checks(rows)
        return

    settings = SwingGridSettings2D()
    total = len(DEFAULT_HEIGHTS_MM) * len(DEFAULT_APPROACH_CLEARANCES_M)
    print(f"step 2: {total} cells "
          f"({len(settings.hip_lift_ladder_m)} hip lifts x "
          f"{len(settings.liftoff_rise_ladder_m)} lift-offs each) "
          f"on {args.workers} workers", flush=True)

    started = time.time()
    cells = run_swing_onto_grid_2d(settings=settings, workers=args.workers)
    rows = swing_grid_rows(cells)
    path = write_rows_csv(args.output_dir / "day10_11_step2_swing_onto_sweep.csv", rows)
    print(f"\nswept in {time.time() - started:.1f}s; wrote {path}", flush=True)
    print(f"total generate_swing_2d calls: {sum(r['evaluations'] for r in rows)}",
          flush=True)

    print(f"  wrote {_plot_min_hip_lift(rows, args.output_dir / 'day10_11_step2_min_hip_lift_map.png')}",
          flush=True)
    print(f"  wrote {_plot_binding_ceiling(rows, args.output_dir / 'day10_11_step2_binding_ceiling_map.png')}",
          flush=True)
    print(f"  wrote {_plot_min_liftoff(rows, args.output_dir / 'day10_11_step2_min_liftoff_map.png')}",
          flush=True)

    _report_checks(rows)


if __name__ == "__main__":
    main()
