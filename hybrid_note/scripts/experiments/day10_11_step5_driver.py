"""Day 10--11 Step 5: the decision map, Figure D, and the sensitivity checks.

Six sections.  Only A runs a planner -- ``SWING_OVER`` is the one candidate no
earlier step measured, and the completion criterion "every candidate has a
concession value" cannot be met without it.  B--F are pure functions of the
tables Steps 2/3/4 and Day 6--7 already wrote.

    python3 day10_11_step5_driver.py                # everything
    python3 day10_11_step5_driver.py --skip-over    # reuse the #5 sweep on disk
    python3 day10_11_step5_driver.py --plots-only   # just redraw
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    DEFAULT_ORDER,
    SWING_LANDING_DISTANCE_CLOSEOUT_M,
    SWING_LANDING_DISTANCE_M,
    Availability,
    StrategyId,
    decide_2d,
    load_tables_2d,
    swing_swing_cell_2d,
)
from hybrid_note.scripts.experiments.day10_11_swing_over_2d import (  # noqa: E402
    SwingOverSettings2D,
    run_swing_over_cells_2d,
    swing_over_rows,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"
DAY6_7_DIR = Path(__file__).resolve().parents[2] / "notes" / "day6-7"

#: Section A.  Heights that Step 2 / Step 3 also swept, so ``#5`` lands on the
#: same axis as ``#4``; 100--160 mm are included to find the ceiling rather
#: than to expect successes.
OVER_HEIGHTS_MM: tuple[int, ...] = (20, 40, 60, 80, 100, 120, 140, 160)

#: Short tops are where ``#5`` is supposed to matter, so the grid is dense
#: there and thins out past the point where ``#1`` and ``#4`` take over.
OVER_TOP_LENGTHS_M: tuple[float, ...] = (
    0.02, 0.05, 0.08, 0.11, 0.14, 0.17, 0.20, 0.24, 0.28, 0.35,
)

#: Sections B--F.  Heights every strategy has data at, so the map's regions are
#: boundaries in the physics rather than edges of an experiment.
MAP_HEIGHTS_MM: tuple[int, ...] = (40, 60, 80, 100, 120, 140, 160)

#: Fine enough that the ``theta_climb`` staircase (steps of ~6--9 mm in the
#: required top length) is visible rather than aliased away, and starting low
#: enough to contain ``#5``'s domain: the over-swing clears tops from 20 mm up,
#: so an axis beginning at 150 mm would hide the region that strategy exists
#: for.
MAP_TOP_LENGTHS_M: tuple[float, ...] = tuple(
    round(0.02 + 0.005 * i, 4) for i in range(87)
)

#: Section E.  Re-orderings that a reader might reasonably argue for.
ORDER_VARIANTS: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("default (body -> margin -> roll)", DEFAULT_ORDER),
    ("margin before body", ("feasible", "margin", "body", "roll_preference")),
    ("roll preference before body", ("feasible", "roll_preference", "body", "margin")),
)

STRATEGY_COLOURS = {
    StrategyId.ROLL_ROLL: "#2563eb",
    StrategyId.SWING_SWING: "#ea580c",
    StrategyId.SWING_OVER: "#16a34a",
}


# --------------------------------------------------------------------------
# A -- the one new measurement
# --------------------------------------------------------------------------


def section_a(output_dir: Path, workers: int) -> Path:
    print("\n=== A. SWING_OVER sweep (the candidate nobody had measured)", flush=True)
    settings = SwingOverSettings2D()
    tasks = [
        (mm / 1e3, top, settings)
        for mm in OVER_HEIGHTS_MM
        for top in OVER_TOP_LENGTHS_M
    ]
    print(f"  {len(tasks)} cells on {workers} workers "
          f"(theta {settings.theta_ladder_deg}, c {settings.clearance_ladder_m})",
          flush=True)
    started = time.perf_counter()
    path = output_dir / "day10_11_step5_swing_over.csv"

    def report(done: int, total: int, cell) -> None:
        # A cell here can take minutes and ``pool.map`` shows nothing until the
        # whole sweep ends, so progress is printed as results land.
        if done % 5 == 0 or done == total:
            print(f"    {done:3d}/{total}  "
                  f"{(time.perf_counter() - started) / 60:.1f} min elapsed, "
                  f"~{(time.perf_counter() - started) / done * (total - done) / 60:.1f} "
                  f"min left", flush=True)

    cells = run_swing_over_cells_2d(tasks, workers=workers, on_result=report)
    rows = swing_over_rows(cells)
    write_rows_csv(path, rows)

    feasible = [c for c in cells if c.feasible]
    print(f"  {len(feasible)} / {len(cells)} feasible "
          f"in {time.perf_counter() - started:.0f} s", flush=True)
    for mm in OVER_HEIGHTS_MM:
        row = [c for c in cells if abs(c.height_m * 1e3 - mm) < 1e-6]
        marks = "".join("o" if c.feasible else "." for c in row)
        best = [c.theta_deg for c in row if c.feasible]
        print(f"    h = {mm:4d} mm  {marks}   "
              + (f"theta {min(best):.0f}-{max(best):.0f} deg" if best else "none"),
              flush=True)
    if feasible:
        tallest = max(c.height_m for c in feasible)
        print(f"\n  tallest obstacle any over-swing clears: {tallest * 1e3:.0f} mm",
              flush=True)
        widest = max(c.top_length_m for c in feasible)
        print(f"  widest top any over-swing clears:       {widest * 1e3:.0f} mm",
              flush=True)
        strides = [c.stride_m for c in feasible if c.stride_m]
        print(f"  stride required: {min(strides) * 1e3:.0f} - "
              f"{max(strides) * 1e3:.0f} mm", flush=True)
    fails = {}
    for c in cells:
        if not c.feasible:
            fails[c.failure] = fails.get(c.failure, 0) + 1
    print(f"  failure modes: {fails}", flush=True)
    return path


# --------------------------------------------------------------------------
# B -- the decision map
# --------------------------------------------------------------------------


def _decisions(tables, order=DEFAULT_ORDER):
    return [
        decide_2d(mm / 1e3, top, tables, order=order)
        for mm in MAP_HEIGHTS_MM
        for top in MAP_TOP_LENGTHS_M
    ]


def section_b(tables) -> list[dict]:
    print("\n=== B. the decision map on (h, L_top)", flush=True)
    decisions = _decisions(tables)
    rows = [{"section": "decision", **d.as_dict()} for d in decisions]

    print("  winner by (h, L_top).  1 = #1 roll+roll, 4 = #4 swing+swing, "
          "5 = #5 over, . = none", flush=True)
    symbol = {StrategyId.ROLL_ROLL: "1", StrategyId.SWING_SWING: "4",
              StrategyId.SWING_OVER: "5", None: "."}
    tops = MAP_TOP_LENGTHS_M
    ruler = ["|" if i % 10 == 0 else " " for i in range(len(tops))]
    print("             " + "".join(ruler), flush=True)
    print("             " + "  ".join(
        f"{tops[i] * 1e3:.0f}".ljust(8) for i in range(0, len(tops), 10)
    ), flush=True)
    for mm in MAP_HEIGHTS_MM:
        line = "".join(
            symbol[d.winner] for d in decisions if abs(d.height_m * 1e3 - mm) < 1e-6
        )
        print(f"  h = {mm:4d} mm  {line}", flush=True)
    print(f"  (columns run L_top = {tops[0] * 1e3:.0f} -> {tops[-1] * 1e3:.0f} mm "
          f"in {(tops[1] - tops[0]) * 1e3:.0f} mm steps)", flush=True)
    return rows


# --------------------------------------------------------------------------
# C -- the overlap region and its cost gap
# --------------------------------------------------------------------------


def section_c(tables) -> list[dict]:
    print("\n=== C. where more than one strategy works, and what the choice costs",
          flush=True)
    decisions = _decisions(tables)
    rows = []
    overlap = [d for d in decisions if len(d.feasible_strategies) > 1]
    print(f"  {len(overlap)} / {len(decisions)} cells have more than one option.",
          flush=True)
    for mm in MAP_HEIGHTS_MM:
        at_h = [d for d in decisions if abs(d.height_m * 1e3 - mm) < 1e-6]
        multi = [d for d in at_h if len(d.feasible_strategies) > 1]
        gaps = [d.cost_gap_m for d in multi if d.cost_gap_m is not None]
        none = [d for d in at_h if not d.feasible_strategies]
        print(f"    h = {mm:4d} mm  multi {len(multi):3d}/{len(at_h)}  "
              f"no option {len(none):3d}  "
              f"gap {'--' if not gaps else f'{min(gaps) * 1e3:5.1f} - {max(gaps) * 1e3:5.1f} mm'}",
              flush=True)
    for d in decisions:
        for cell in d.cells:
            rows.append({
                "section": "strategy_cell",
                **cell.as_dict(),
                "winner_here": d.winner.value if d.winner else None,
            })
    ties = [d for d in decisions if d.decided_by_tie_break]
    print(f"\n  decided by tie-break (body cost did not separate them): {len(ties)}",
          flush=True)
    return rows


# --------------------------------------------------------------------------
# D -- the L_top slices
# --------------------------------------------------------------------------


def section_d(tables) -> list[dict]:
    """Spec task 6: does the strategy really shift toward swing as L_top shrinks?"""

    print("\n=== D. L_top slices: what changes as the top gets shorter", flush=True)
    rows = []
    for mm in MAP_HEIGHTS_MM:
        previous = None
        transitions = []
        for top in MAP_TOP_LENGTHS_M:
            d = decide_2d(mm / 1e3, top, tables)
            if d.winner is not previous:
                transitions.append((top, previous, d.winner))
                previous = d.winner
            winner_cell = d.winner_cell
            rows.append({
                "section": "slice",
                "obstacle_mm": mm,
                "top_length_m": top,
                "winner": d.winner.value if d.winner else None,
                "feasible_count": len(d.feasible_strategies),
                "body_deviation_mm": (
                    None if winner_cell is None or winner_cell.body_deviation_m is None
                    else winner_cell.body_deviation_m * 1e3
                ),
                "theta_climb_deg": dict(winner_cell.parameters).get("theta_climb_deg")
                if winner_cell else None,
            })
        text = " -> ".join(
            f"{'none' if w is None else w.value.split()[0]}@{t * 1e3:.0f}"
            for t, _, w in transitions
        )
        print(f"  h = {mm:4d} mm  {text}", flush=True)
    return rows


# --------------------------------------------------------------------------
# E -- lexicographic sensitivity
# --------------------------------------------------------------------------


def section_e(tables) -> list[dict]:
    print("\n=== E. does re-ordering the lexicographic rule move the boundaries?",
          flush=True)
    baseline = {(d.height_m, d.top_length_m): d.winner for d in _decisions(tables)}
    rows = []
    for label, order in ORDER_VARIANTS:
        changed = 0
        for d in _decisions(tables, order=order):
            key = (d.height_m, d.top_length_m)
            if baseline[key] is not d.winner:
                changed += 1
                rows.append({
                    "section": "order_sensitivity",
                    "order": label,
                    "obstacle_mm": d.height_m * 1e3,
                    "top_length_m": d.top_length_m,
                    "baseline_winner": (
                        None if baseline[key] is None else baseline[key].value
                    ),
                    "variant_winner": None if d.winner is None else d.winner.value,
                })
        total = len(MAP_HEIGHTS_MM) * len(MAP_TOP_LENGTHS_M)
        print(f"  {label:34s} {changed:4d} / {total} cells change", flush=True)
        if changed == 0:
            rows.append({
                "section": "order_sensitivity", "order": label,
                "obstacle_mm": None, "top_length_m": None,
                "baseline_winner": "unchanged", "variant_winner": "unchanged",
            })

    # -- and the other knob spec 6.2 names: the margin floor ----------------
    # The default trusts the planner's own verdict, because the tightest point
    # of a swing is normally the touchdown and its clearance is zero by
    # construction -- a handful of feasible plans report a few times 1e-7 m,
    # which is 1000x finer than the planner's own tolerance.  Imposing a floor
    # is therefore a *choice*, and this is what it costs.
    print("\n  margin floor (spec 6.2), which the default deliberately leaves off:",
          flush=True)
    for floor_mm in (0.0, 0.5, 1.0):
        lost = 0
        for mm in MAP_HEIGHTS_MM:
            for top in MAP_TOP_LENGTHS_M:
                base = decide_2d(mm / 1e3, top, tables)
                strict = decide_2d(mm / 1e3, top, tables,
                                   margin_floor_m=floor_mm / 1e3)
                if base.winner is not strict.winner:
                    lost += 1
        rows.append({
            "section": "margin_sensitivity",
            "order": f"margin floor {floor_mm:.1f} mm",
            "obstacle_mm": None, "top_length_m": None,
            "baseline_winner": f"{lost} cells change",
            "variant_winner": f"{lost} cells change",
        })
        print(f"    floor {floor_mm:4.1f} mm  {lost:4d} / {total} cells change",
              flush=True)
    return rows


# --------------------------------------------------------------------------
# F -- the monotonicity claim of spec 2.7
# --------------------------------------------------------------------------


def section_f(tables) -> list[dict]:
    """"The more swing-like the pair, the shorter the top it needs" -- true?"""

    print("\n=== F. spec 2.7's monotonicity claim", flush=True)
    rows = []
    for mm in MAP_HEIGHTS_MM:
        minima: dict[StrategyId, float | None] = {}
        for strategy in (StrategyId.ROLL_ROLL, StrategyId.SWING_SWING,
                         StrategyId.SWING_OVER):
            found = None
            for top in MAP_TOP_LENGTHS_M:
                d = decide_2d(mm / 1e3, top, tables)
                cell = next(c for c in d.cells if c.strategy is strategy)
                if cell.availability is Availability.FEASIBLE:
                    found = top
                    break
            minima[strategy] = found
        # #5 is the one strategy a LONGER top hurts, so its bound is a maximum.
        over_max = None
        for top in reversed(MAP_TOP_LENGTHS_M):
            d = decide_2d(mm / 1e3, top, tables)
            cell = next(c for c in d.cells if c.strategy is StrategyId.SWING_OVER)
            if cell.availability is Availability.FEASIBLE:
                over_max = top
                break
        rows.append({
            "section": "monotonicity",
            "obstacle_mm": mm,
            "roll_roll_min_top_m": minima[StrategyId.ROLL_ROLL],
            "swing_swing_min_top_m": minima[StrategyId.SWING_SWING],
            "swing_over_min_top_m": minima[StrategyId.SWING_OVER],
            "swing_over_max_top_m": over_max,
        })
        def _mm(value):
            return "--" if value is None else f"{value * 1e3:5.0f}"
        print(f"  h = {mm:4d} mm   #1 needs >= {_mm(minima[StrategyId.ROLL_ROLL])}"
              f"   #4 needs >= {_mm(minima[StrategyId.SWING_SWING])}"
              f"   #5 works {_mm(minima[StrategyId.SWING_OVER])} .. "
              f"{_mm(over_max)}", flush=True)

    holds = [
        r for r in rows
        if r["roll_roll_min_top_m"] is not None
        and r["swing_swing_min_top_m"] is not None
    ]
    shorter = sum(
        1 for r in holds
        if r["swing_swing_min_top_m"] < r["roll_roll_min_top_m"]
    )
    print(f"\n  '#4 needs a shorter top than #1' holds in "
          f"{shorter} / {len(holds)} heights.", flush=True)

    # -- sensitivity: #4's bound is set by where the ascent lands ------------
    # Step 2's main grid fixed the landing at 160 mm, but its close-out swept
    # 100--220 mm and found 100 mm feasible at h = 100 / 150 / 200 with no
    # change in hip lift.  That moves #4's minimum top length by 60 mm, which
    # is a region boundary, so it is reported rather than silently adopted.
    print(f"\n  sensitivity: the ascent's landing distance sets #4's bound.",
          flush=True)
    for mm in MAP_HEIGHTS_MM:
        bounds = {}
        for label, landing in (("as swept (160 mm)", SWING_LANDING_DISTANCE_M),
                               ("close-out (100 mm)",
                                SWING_LANDING_DISTANCE_CLOSEOUT_M)):
            found = None
            for top in MAP_TOP_LENGTHS_M:
                cell = swing_swing_cell_2d(mm / 1e3, top, tables,
                                           landing_distance_m=landing)
                if cell.availability is Availability.FEASIBLE:
                    found = top
                    break
            bounds[label] = found
        rows.append({
            "section": "landing_sensitivity",
            "obstacle_mm": mm,
            "swing_swing_min_top_as_swept_m": bounds["as swept (160 mm)"],
            "swing_swing_min_top_closeout_m": bounds["close-out (100 mm)"],
        })
        def _mm2(value):
            return "--" if value is None else f"{value * 1e3:5.0f}"
        print(f"    h = {mm:4d} mm   #4 needs >= {_mm2(bounds['as swept (160 mm)'])}"
              f" mm as swept, {_mm2(bounds['close-out (100 mm)'])} mm if the "
              "ascent lands closer", flush=True)
    print("    (close-out verified the 100 mm landing at h = 100 / 150 / 200 only)",
          flush=True)
    return rows


# --------------------------------------------------------------------------
# Figures
# --------------------------------------------------------------------------


def _grid(tables):
    decisions = _decisions(tables)
    lookup = {(round(d.height_m * 1e3), d.top_length_m): d for d in decisions}
    return decisions, lookup


def _plot_figure_d(tables, path: Path) -> Path:
    """Figure D: terrain geometry -> motion class."""

    _, lookup = _grid(tables)
    order = [StrategyId.ROLL_ROLL, StrategyId.SWING_SWING, StrategyId.SWING_OVER]
    index = {s: i for i, s in enumerate(order)}
    grid = np.full((len(MAP_HEIGHTS_MM), len(MAP_TOP_LENGTHS_M)), np.nan)
    multi = np.zeros_like(grid)
    for i, mm in enumerate(MAP_HEIGHTS_MM):
        for j, top in enumerate(MAP_TOP_LENGTHS_M):
            d = lookup[(mm, top)]
            if d.winner is not None:
                grid[i, j] = index[d.winner]
                multi[i, j] = len(d.feasible_strategies)

    fig, axes = plt.subplots(1, 2, figsize=(15.0, 5.0),
                             gridspec_kw={"width_ratios": [1.35, 1]})

    ax = axes[0]
    cmap = ListedColormap([STRATEGY_COLOURS[s] for s in order])
    ax.imshow(grid, aspect="auto", origin="lower", cmap=cmap, vmin=-0.5, vmax=2.5,
              extent=(MAP_TOP_LENGTHS_M[0] * 1e3, MAP_TOP_LENGTHS_M[-1] * 1e3,
                      -0.5, len(MAP_HEIGHTS_MM) - 0.5))
    # Hatch the cells where the choice is a preference, not a necessity.
    for i, mm in enumerate(MAP_HEIGHTS_MM):
        for j, top in enumerate(MAP_TOP_LENGTHS_M):
            d = lookup[(mm, top)]
            if len(d.feasible_strategies) > 1:
                ax.plot(top * 1e3, i, ".", ms=2.0, color="white", alpha=0.75)
            elif d.winner is None:
                ax.plot(top * 1e3, i, "x", ms=4, color="#dc2626")
    ax.set_yticks(range(len(MAP_HEIGHTS_MM)), [f"{m}" for m in MAP_HEIGHTS_MM])
    ax.set_xlabel("top length L_top [mm]")
    ax.set_ylabel("obstacle height h [mm]")
    ax.set_title("Figure D. terrain geometry -> motion class\n"
                 "white dots = more than one strategy works; red x = none does",
                 fontsize=10)
    ax.legend(handles=[Patch(facecolor=STRATEGY_COLOURS[s], label=s.value)
                       for s in order]
              + [Patch(facecolor="#ffffff", edgecolor="#94a3b8",
                       label="x  no strategy")],
              fontsize=7, loc="lower right", framealpha=0.92)

    # --- the cost of the choice, where there is one -----------------------
    ax = axes[1]
    for i, mm in enumerate(MAP_HEIGHTS_MM):
        xs, ys = [], []
        for top in MAP_TOP_LENGTHS_M:
            d = lookup[(mm, top)]
            if len(d.feasible_strategies) > 1 and d.cost_gap_m is not None:
                xs.append(top * 1e3)
                ys.append(d.cost_gap_m * 1e3)
        if xs:
            ax.plot(xs, ys, "-", lw=1.8, label=f"h = {mm} mm")
    ax.axhline(0.0, color="#64748b", lw=1)
    ax.set_xlabel("top length L_top [mm]")
    ax.set_ylabel("body deviation given up by the runner-up [mm]")
    ax.set_title("what the choice is worth\n"
                 "how much more the second-best strategy would demand",
                 fontsize=10)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7)

    fig.suptitle("Step 5: the decision map -- L_top decides the strategy, and "
                 "inside rolling it also decides theta_climb", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def _plot_cost_gap(tables, path: Path) -> Path:
    """Body cost of every strategy along L_top, at three heights."""

    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.6))
    for ax, mm in zip(axes, (60, 100, 140)):
        for strategy in (StrategyId.ROLL_ROLL, StrategyId.SWING_SWING,
                         StrategyId.SWING_OVER):
            xs, ys = [], []
            for top in MAP_TOP_LENGTHS_M:
                d = decide_2d(mm / 1e3, top, tables)
                cell = next(c for c in d.cells if c.strategy is strategy)
                xs.append(top * 1e3)
                ys.append(
                    cell.body_deviation_m * 1e3
                    if (cell.availability is Availability.FEASIBLE
                        and cell.body_deviation_m is not None)
                    else np.nan
                )
            ax.plot(xs, ys, "-", lw=2.0, color=STRATEGY_COLOURS[strategy],
                    label=strategy.value)
        ax.axhline(mm, color="#64748b", ls=":", lw=1.2,
                   label="h (the step itself)")
        ax.set_xlabel("top length L_top [mm]")
        ax.set_ylabel("body deviation [mm]")
        ax.set_title(f"h = {mm} mm", fontsize=10)
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=6.5, loc="upper right")

    fig.suptitle("Step 5: rolling's cost is a STAIRCASE in L_top -- a shorter "
                 "top forces a more extended climb, which costs body excursion",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def _plot_top_length_slice(tables, path: Path) -> Path:
    """Spec task 6's slice, plus the bounds each strategy imposes on L_top."""

    fig, axes = plt.subplots(1, 2, figsize=(14.0, 4.8))

    ax = axes[0]
    order = [StrategyId.ROLL_ROLL, StrategyId.SWING_SWING, StrategyId.SWING_OVER]
    for i, mm in enumerate(MAP_HEIGHTS_MM):
        for strategy in order:
            spans = []
            run_start = None
            for top in MAP_TOP_LENGTHS_M:
                d = decide_2d(mm / 1e3, top, tables)
                cell = next(c for c in d.cells if c.strategy is strategy)
                ok = cell.availability is Availability.FEASIBLE
                if ok and run_start is None:
                    run_start = top
                elif not ok and run_start is not None:
                    spans.append((run_start, top))
                    run_start = None
            if run_start is not None:
                spans.append((run_start, MAP_TOP_LENGTHS_M[-1]))
            offset = {StrategyId.ROLL_ROLL: -0.22, StrategyId.SWING_SWING: 0.0,
                      StrategyId.SWING_OVER: 0.22}[strategy]
            for low, high in spans:
                ax.plot([low * 1e3, high * 1e3], [i + offset] * 2, "-",
                        lw=5, color=STRATEGY_COLOURS[strategy], solid_capstyle="butt")
    ax.set_yticks(range(len(MAP_HEIGHTS_MM)), [f"{m}" for m in MAP_HEIGHTS_MM])
    ax.set_xlabel("top length L_top [mm]")
    ax.set_ylabel("obstacle height h [mm]")
    ax.set_title("where each strategy is available\n"
                 "#1 and #4 need L_top ABOVE a bound; #5 needs it BELOW one",
                 fontsize=10)
    ax.legend(handles=[Patch(facecolor=STRATEGY_COLOURS[s], label=s.value)
                       for s in order], fontsize=7, loc="lower right")
    ax.grid(True, alpha=0.2, axis="x")

    # --- the theta staircase, which is the mechanism behind the left panel --
    ax = axes[1]
    for mm in (60, 100, 140):
        xs, ys = [], []
        for top in MAP_TOP_LENGTHS_M:
            d = decide_2d(mm / 1e3, top, tables)
            cell = next(c for c in d.cells if c.strategy is StrategyId.ROLL_ROLL)
            xs.append(top * 1e3)
            ys.append(dict(cell.parameters).get("theta_climb_deg", np.nan)
                      if cell.availability is Availability.FEASIBLE else np.nan)
        ax.step(xs, ys, where="post", lw=2.0, label=f"h = {mm} mm")
    ax.set_xlabel("top length L_top [mm]")
    ax.set_ylabel("theta_climb chosen [deg]")
    ax.set_title("inside #1, L_top chooses theta_climb\n"
                 "shorter top -> more extended climb -> more body excursion",
                 fontsize=10)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7.5)

    fig.suptitle("Step 5: the top-length slice", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


# --------------------------------------------------------------------------


def _union(rows: list[dict]) -> list[dict]:
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    return [{key: row.get(key, "") for key in columns} for row in rows]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--workers", type=int,
                        default=max(1, (os.cpu_count() or 4) - 1))
    parser.add_argument("--skip-over", action="store_true",
                        help="reuse day10_11_step5_swing_over.csv instead of re-running")
    parser.add_argument("--plots-only", action="store_true")
    args = parser.parse_args()

    over_csv = args.output_dir / "day10_11_step5_swing_over.csv"
    if not (args.skip_over or args.plots_only):
        section_a(args.output_dir, args.workers)
    elif not over_csv.exists():
        print(f"  {over_csv.name} not found; run without --skip-over first.",
              flush=True)
        return 1

    tables = load_tables_2d(args.output_dir, DAY6_7_DIR, swing_over_csv=over_csv)

    figure_d = args.output_dir / "day10_11_step5_figure_d.png"
    cost_gap = args.output_dir / "day10_11_step5_cost_gap.png"
    slice_png = args.output_dir / "day10_11_step5_top_length_slice.png"

    if not args.plots_only:
        rows = (section_b(tables) + section_c(tables) + section_d(tables)
                + section_e(tables) + section_f(tables))
        path = args.output_dir / "day10_11_step5_decision_map.csv"
        write_rows_csv(path, _union(rows))
        print(f"\n  wrote {path.name} ({len(rows)} rows)", flush=True)

    print(f"  wrote {_plot_figure_d(tables, figure_d)}", flush=True)
    print(f"  wrote {_plot_cost_gap(tables, cost_gap)}", flush=True)
    print(f"  wrote {_plot_top_length_slice(tables, slice_png)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
