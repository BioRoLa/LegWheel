"""Day 10--11 Step 2 close-out: the three things the main map left open.

The main sweep answered one question -- what does a swing onto a step cost, as
a function of height and approach clearance -- while holding three things fixed
that it never justified holding fixed.  Each of them makes the map conditional
in a way Step 5 would inherit silently:

``ceiling``
    every row at ``c >= 60 mm`` was feasible up to 200 mm, which is the edge of
    the sweep and not a ceiling.  "The tallest step this leg can swing onto" is
    currently unknown, and it is one of the two numbers Step 5 needs from this
    side.

``landing``
    ``landing_distance_m = 0.16`` -- where the foot comes down on the top.  It
    is a genuine freedom of the ascent, so a cost quoted at one value is a cost
    quoted under an unexamined choice.

``theta``
    ``theta = 60 deg``.  Option (i) in the spec fixes it deliberately, but
    "fixed" and "does not matter" are different claims and only the first one
    has been made.

Each section sweeps narrowly and past the saturation clearance, so what varies
is the thing under test.

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step2_closeout_driver.py
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import replace
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
    SwingGridSettings2D,
    run_swing_cells_2d,
    swing_grid_rows,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"

#: Past the measured saturation of the approach axis, so the approach is not
#: what is being tested in any of these three sections.
SATURATED_CLEARANCE_M = 0.08

CEILING_HEIGHTS_MM: tuple[int, ...] = (200, 220, 240, 260, 280, 300)
CEILING_CLEARANCES_M: tuple[float, ...] = (0.06, 0.10, 0.16)

PROBE_HEIGHTS_MM: tuple[int, ...] = (100, 150, 200)
LANDING_DISTANCES_M: tuple[float, ...] = (0.10, 0.13, 0.16, 0.19, 0.22)
THETAS_DEG: tuple[float, ...] = (40.0, 50.0, 60.0, 70.0, 85.0)


def _section(title: str) -> None:
    print(f"\n{'=' * 72}\n{title}\n{'=' * 72}", flush=True)


def _feasible_heights(rows, key, value):
    return sorted(
        row["height_mm"] for row in rows
        if row[key] == value and bool(row["feasible"])
    )


def run_ceiling(settings: SwingGridSettings2D, workers: int) -> list[dict]:
    _section("A. where is the real height ceiling?")
    tasks = [
        (height_mm / 1000.0, clearance, settings)
        for height_mm in CEILING_HEIGHTS_MM
        for clearance in CEILING_CLEARANCES_M
    ]
    cells = run_swing_cells_2d(tasks, workers=workers, label="ceiling ")
    rows = swing_grid_rows(cells)
    for row in rows:
        row["section"] = "ceiling"

    print("\n  tallest feasible step per clearance:", flush=True)
    for clearance in CEILING_CLEARANCES_M:
        feasible = _feasible_heights(rows, "approach_clearance_mm", clearance * 1e3)
        tallest = max(feasible) if feasible else None
        print(f"    c={clearance * 1e3:>5.0f} mm -> "
              f"{'none in this range' if tallest is None else f'{tallest:.0f} mm'}"
              f"   (feasible: {[f'{h:.0f}' for h in feasible] or '-'})", flush=True)
    ceilings = {row["binding_ceiling"] for row in rows if not row["feasible"]}
    print(f"  ceilings seen among the failures: {ceilings or '(none failed)'}", flush=True)
    return rows


def run_landing(settings: SwingGridSettings2D, workers: int) -> list[dict]:
    _section("B. how much does the landing distance change the answer?")
    tasks = [
        (height_mm / 1000.0, SATURATED_CLEARANCE_M,
         replace(settings, landing_distance_m=landing))
        for height_mm in PROBE_HEIGHTS_MM
        for landing in LANDING_DISTANCES_M
    ]
    cells = run_swing_cells_2d(tasks, workers=workers, label="landing ")
    rows = swing_grid_rows(cells)
    for row in rows:
        row["section"] = "landing"

    print("\n  min hip lift [mm] by (height, landing distance):", flush=True)
    header = "    height | " + " ".join(f"{d:>6.2f}" for d in LANDING_DISTANCES_M)
    print(header, flush=True)
    for height_mm in PROBE_HEIGHTS_MM:
        cells_for_height = [
            next(r for r in rows
                 if r["height_mm"] == height_mm and r["landing_distance_m"] == landing)
            for landing in LANDING_DISTANCES_M
        ]
        cellstr = " ".join(
            "     x" if not r["feasible"] else f"{r['min_hip_lift_mm']:>6.0f}"
            for r in cells_for_height
        )
        print(f"    {height_mm:>6.0f} | {cellstr}", flush=True)
    return rows


def run_theta(settings: SwingGridSettings2D, workers: int) -> list[dict]:
    _section("C. how much does theta change the answer?")
    tasks = [
        (height_mm / 1000.0, SATURATED_CLEARANCE_M,
         replace(settings, theta_rad=float(np.deg2rad(theta_deg))))
        for height_mm in PROBE_HEIGHTS_MM
        for theta_deg in THETAS_DEG
    ]
    cells = run_swing_cells_2d(tasks, workers=workers, label="theta   ")
    rows = swing_grid_rows(cells)
    for row in rows:
        row["section"] = "theta"

    print("\n  min hip lift [mm] by (height, theta):", flush=True)
    print("    height | " + " ".join(f"{t:>6.0f}" for t in THETAS_DEG), flush=True)
    for height_mm in PROBE_HEIGHTS_MM:
        cells_for_height = [
            next(r for r in rows
                 if r["height_mm"] == height_mm and abs(r["theta_deg"] - theta) < 1e-6)
            for theta in THETAS_DEG
        ]
        cellstr = " ".join(
            "     x" if not r["feasible"] else f"{r['min_hip_lift_mm']:>6.0f}"
            for r in cells_for_height
        )
        print(f"    {height_mm:>6.0f} | {cellstr}", flush=True)
    return rows


def _plot_closeout(rows, path: Path) -> Path:
    landing = [r for r in rows if r["section"] == "landing"]
    theta = [r for r in rows if r["section"] == "theta"]
    ceiling = [r for r in rows if r["section"] == "ceiling"]

    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.4))

    ax = axes[0]
    # The failures are the point of this panel, and three clearances failing at
    # the same height would draw three markers on top of each other -- so each
    # series gets its own row, labelled with the ceiling that stopped it.
    rows = {clearance: -18.0 - 16.0 * index
            for index, clearance in enumerate(CEILING_CLEARANCES_M)}
    # The feasible values coincide exactly where two clearances agree, so the
    # series are drawn with decreasing marker size and different dashes; a
    # single fat line would hide the fact that c = 100 and c = 160 are equal.
    styles = [("o", "-", 11), ("s", "--", 7), ("^", ":", 4)]
    for clearance, (marker, dash, size) in zip(CEILING_CLEARANCES_M, styles):
        subset = sorted(
            (r for r in ceiling if r["approach_clearance_mm"] == clearance * 1e3),
            key=lambda r: r["height_mm"])
        heights = [r["height_mm"] for r in subset]
        values = [r["min_hip_lift_mm"] if r["feasible"] else np.nan for r in subset]
        line, = ax.plot(heights, values, marker=marker, ls=dash, ms=size,
                        label=f"c = {clearance * 1e3:.0f} mm")
        failed = [r for r in subset if not r["feasible"]]
        ax.plot([r["height_mm"] for r in failed], [rows[clearance]] * len(failed),
                "x", ms=8, color=line.get_color())
        for r in failed:
            ax.annotate(str(r["binding_ceiling"]), (r["height_mm"], rows[clearance]),
                        textcoords="offset points", xytext=(0, -11),
                        ha="center", fontsize=6, color=line.get_color())
    ax.axhline(0.0, color="#94a3b8", lw=0.8)
    ax.set_xlabel("step height [mm]")
    ax.set_ylabel("min hip lift [mm]")
    ax.set_title("A. beyond 200 mm: the ceiling is 240 mm and it is reach\n"
                 "markers below the axis = infeasible, one row per clearance")
    ax.set_ylim(rows[CEILING_CLEARANCES_M[-1]] - 16.0, None)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, loc="upper right")

    ax = axes[1]
    for height_mm in PROBE_HEIGHTS_MM:
        subset = sorted(
            (r for r in landing if r["height_mm"] == height_mm),
            key=lambda r: r["landing_distance_m"])
        ax.plot([r["landing_distance_m"] for r in subset],
                [r["min_hip_lift_mm"] if r["feasible"] else np.nan for r in subset],
                "o-", label=f"h = {height_mm} mm")
        for r in subset:
            if not r["feasible"]:
                ax.plot(r["landing_distance_m"], 0, "x", ms=9, color="#dc2626")
    ax.set_xlabel("landing distance [m]")
    ax.set_ylabel("min hip lift [mm]")
    ax.set_title("B. landing distance\n(main map used 0.16 m)")
    ax.axvline(0.16, color="#64748b", ls=":", lw=1.2)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)

    ax = axes[2]
    for height_mm in PROBE_HEIGHTS_MM:
        subset = sorted(
            (r for r in theta if r["height_mm"] == height_mm),
            key=lambda r: r["theta_deg"])
        ax.plot([r["theta_deg"] for r in subset],
                [r["min_hip_lift_mm"] if r["feasible"] else np.nan for r in subset],
                "o-", label=f"h = {height_mm} mm")
        for r in subset:
            if not r["feasible"]:
                ax.plot(r["theta_deg"], 0, "x", ms=9, color="#dc2626")
    ax.set_xlabel("theta [deg]")
    ax.set_ylabel("min hip lift [mm]")
    ax.set_title("C. theta\n(main map used 60 deg)")
    ax.axvline(60.0, color="#64748b", ls=":", lw=1.2)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)

    fig.suptitle("Step 2 close-out: the three axes the main map held fixed", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description="Day 10--11 Step 2 close-out")
    parser.add_argument("--workers", type=int,
                        default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--plots-only", action="store_true",
                        help="redraw from the CSV instead of re-running 987 s of sweep")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    if args.plots_only:
        import csv as _csv

        def _typed(row):
            out = {}
            for key, value in row.items():
                if value in ("", "None"):
                    out[key] = None
                elif value in ("True", "False"):
                    out[key] = value == "True"
                else:
                    try:
                        out[key] = float(value)
                    except ValueError:
                        out[key] = value
            return out

        with (args.output_dir / "day10_11_step2_closeout.csv").open(encoding="utf-8") as handle:
            rows = [_typed(row) for row in _csv.DictReader(handle)]
        print(f"  wrote {_plot_closeout(rows, args.output_dir / 'day10_11_step2_closeout.png')}",
              flush=True)
        return

    settings = SwingGridSettings2D()
    started = time.time()
    rows: list[dict] = []
    rows += run_ceiling(settings, args.workers)
    rows += run_landing(settings, args.workers)
    rows += run_theta(settings, args.workers)

    path = write_rows_csv(args.output_dir / "day10_11_step2_closeout.csv", rows)
    print(f"\nclose-out finished in {time.time() - started:.1f}s; wrote {path}",
          flush=True)
    print(f"  wrote {_plot_closeout(rows, args.output_dir / 'day10_11_step2_closeout.png')}",
          flush=True)
    print(f"  total generate_swing_2d calls: {sum(r['evaluations'] for r in rows)}",
          flush=True)

    flagged = sum(1 for r in rows if r["greedy_backtrack_suspected"])
    print(f"  greedy_backtrack_suspected: {flagged} (must be 0)", flush=True)


if __name__ == "__main__":
    main()
