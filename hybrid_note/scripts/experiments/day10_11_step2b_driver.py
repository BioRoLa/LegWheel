"""Day 10--11 Step 2b: does ``SWING_UP + ROLL_DOWN`` exist?

Three sections, matching the three sub-questions in spec §2.8, in the order
that costs least to learn the most:

``A. the window``
    A static beta sweep at ``theta = 17 deg``.  Answers (b) "is the left rim
    reachable at all" and (c) "how close is the alpha seam", and it costs
    seconds rather than minutes.  If there were no window, sections B and C
    would be pointless, so it runs first.

``B. the landing``
    ``height x corner distance`` on the Day 6--7 obstacle.  Answers (a) "is
    the swing feasible and what does it cost the body", and -- separately --
    whether Step 9R can actually descend from where the swing put the leg.
    The two verdicts are kept apart: a valid swing to a pose the descent
    cannot use is not a strategy-#3 cell.

``C. the budget``
    ``height x top length``, landing at the shortest corner distance section B
    found usable.  This is the number the whole step exists for: the minimum
    top length ``SWING_UP + ROLL_DOWN`` needs, to be compared against the
    0.20--0.27 m that ``ROLL_UP + ROLL_DOWN`` spends on ``L_transition``.

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step2b_driver.py
"""

from __future__ import annotations

import argparse
import csv
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

from hybrid_note.scripts.experiments.day10_11_left_rim_landing_2d import (  # noqa: E402
    LEFT_RIM_ALPHA_LIMITS_DEG,
    LEFT_RIM_READY_THETA_RAD,
    beta_window_rows,
    choose_landing_beta_2d,
    run_beta_windows_2d_at_thetas,
    left_rim_rows,
    predicted_pivot_deg_2d,
    run_beta_windows_2d,
    run_left_rim_cells_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day10_11_swing_sweep_2d import (  # noqa: E402
    SwingGridSettings2D,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"

#: Past the measured saturation of the approach axis (spec Step 2 close-out
#: A: the threshold is 60 mm up to h = 200 mm and 100 mm above it), so the
#: approach is not what any of these sections is testing.
APPROACH_CLEARANCE_M = 0.10

#: Heights that span the Day 6--7 rolling map and reach past its ceiling.
WINDOW_HEIGHTS_M: tuple[float, ...] = (0.06, 0.10, 0.14, 0.20)

LANDING_HEIGHTS_MM: tuple[int, ...] = (60, 100, 140, 160, 200)
D_CORNER_M: tuple[float, ...] = (0.02, 0.04, 0.06, 0.10, 0.16)

BUDGET_HEIGHTS_MM: tuple[int, ...] = (60, 100, 140, 160, 200)
TOP_LENGTHS_M: tuple[float, ...] = (0.08, 0.12, 0.16, 0.20, 0.24, 0.28, 0.35)

#: How close to the corner the budget sweep lands.  Small on purpose: the
#: number section C reports is a *minimum* top length, so anything the landing
#: leaves unused past the corner would inflate it.
BUDGET_D_CORNER_M = 0.02

#: The diagnostic axes.  If the budget sweep fails everywhere, the question is
#: whether the swing is refused by geometry or merely by the approach posture
#: Step 2 happened to freeze, and these are the two knobs that decide it.
DIAGNOSTIC_HEIGHT_MM = 100
DIAGNOSTIC_TOP_LENGTH_M = 0.12
DIAGNOSTIC_THETAS_DEG: tuple[float, ...] = (40.0, 50.0, 60.0, 70.0, 85.0)
DIAGNOSTIC_CLEARANCES_M: tuple[float, ...] = (0.06, 0.10, 0.16)

#: The degenerate version spec §2.8 asks for: land short of 17 degrees on the
#: left rim, then retract on the top.  17 is in the list as the control -- it
#: is the non-degenerate case, and it has to reappear as infeasible here or the
#: two sweeps disagree.
DEGENERATE_LANDING_THETAS_DEG: tuple[float, ...] = (17.0, 25.0, 35.0, 45.0, 60.0)
DEGENERATE_HEIGHTS_MM: tuple[int, ...] = (60, 100, 140)
DEGENERATE_TOP_LENGTH_M = 0.20

#: The rolling side's own arrival, read off Step 0's handoff CSV so the swing's
#: freedom is reported against what the roll actually does rather than against
#: nothing.  Filled in by :func:`_rolling_arrival_alphas`.
_HANDOFF_CSV = "day10_11_step0_roll_exit_handoff.csv"


def _typed(row: dict) -> dict:
    """CSV strings back into the types the plotting code expects."""

    out: dict = {}
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


def _section(title: str) -> None:
    print(f"\n{'=' * 78}\n{title}\n{'=' * 78}", flush=True)


def _rolling_arrival_betas(output_dir: Path) -> list[float]:
    """Beta values the rolling ascent actually arrives at the corner with."""

    path = output_dir / _HANDOFF_CSV
    if not path.exists():
        return []
    with path.open(encoding="utf-8") as handle:
        return [
            float(row["beta_deg"])
            for row in csv.DictReader(handle)
            if row["stage"] == "WHEEL_TRANSITION" and row["rebuilt_rim"] == "left_rim"
            and row["beta_deg"]
        ]


def run_window(output_dir: Path) -> tuple[list, list[float]]:
    _section("A. (b) is the left rim reachable, and (c) how close is the seam?")
    started = time.time()
    windows = run_beta_windows_2d(WINDOW_HEIGHTS_M, sample_step_deg=1.0)
    for height, window in zip(WINDOW_HEIGHTS_M, windows):
        print(
            f"  h={height * 1e3:>4.0f} mm  beta in "
            f"[{window.beta_min_deg:>7.1f}, {window.beta_max_deg:>7.1f}] deg "
            f"width={window.width_deg:>6.1f}  contiguous={window.contiguous}  "
            f"alpha in [{window.alpha_min_deg:>7.1f}, {window.alpha_max_deg:>7.1f}]  "
            f"best beta={window.best_beta_deg:>7.1f} -> alpha={window.best_alpha_deg:>7.1f} "
            f"(seam margin {window.best_seam_margin_deg:.1f} deg)  "
            f"hip above top={window.hip_above_top_m * 1e3:.2f} mm",
            flush=True,
        )

    widths = {round(w.width_deg, 3) for w in windows}
    print(
        f"\n  window identical across heights: {len(widths) == 1} "
        f"-- it is a property of the leg at theta = 17 deg, not of the step. "
        f"({time.time() - started:.0f}s)",
        flush=True,
    )

    betas = _rolling_arrival_betas(output_dir)
    if betas:
        wrapped = [b + 360.0 * np.ceil(-b / 360.0) - 360.0 for b in betas]
        print(
            f"  rolling ascent arrives at beta in "
            f"[{min(wrapped):.1f}, {max(wrapped):.1f}] deg "
            f"({len(betas)} Step 0 handoff rows) -- inside the window: "
            f"{all(windows[0].beta_min_deg <= b <= windows[0].beta_max_deg for b in wrapped)}",
            flush=True,
        )
    return windows, betas


def choose_betas(window, heights_mm) -> dict:
    """One landing beta per height: enough rim for the pivot, then seam margin.

    The choice depends on the height because the corner pivot does: a taller
    corner turns the leg further, so it needs more of the left rim ahead of the
    contact, which forces the landing alpha closer to the -180 deg seam.  That
    exchange is the quantitative answer to spec §2.8(c).
    """

    print("\n  landing choice per height "
          "(pivot first, then seam margin -- no free parameter):", flush=True)
    print("    height | pivot needs | chosen beta | landing alpha | rim budget | seam margin",
          flush=True)
    choices = {}
    for height_mm in heights_mm:
        spec = SharedTerrainSpec2D(height_m=height_mm / 1000.0)
        pivot = predicted_pivot_deg_2d(spec)
        choice = choose_landing_beta_2d(window, pivot)
        choices[height_mm] = choice
        sample = choice.sample
        flag = "" if choice.sufficient else "   <- RIM TOO SHORT"
        print(
            f"    {height_mm:>6} | "
            f"{'--' if pivot is None else f'{pivot:>8.1f} deg'} | "
            f"{sample.beta_deg:>8.1f} deg | {sample.alpha_deg:>10.1f} deg | "
            f"{sample.rim_budget_deg:>7.1f} deg | {sample.seam_margin_deg:>8.1f} deg{flag}",
            flush=True,
        )
    return choices


def run_landing(choices: dict, workers: int) -> list:
    _section("B. (a) can the swing land there, and can Step 9R leave from it?")
    settings = SwingGridSettings2D()
    tasks = [
        (height_mm / 1000.0, 0.35, d_corner, APPROACH_CLEARANCE_M, settings,
         choices[height_mm].beta_deg, True)
        for height_mm in LANDING_HEIGHTS_MM
        for d_corner in D_CORNER_M
    ]
    cells = run_left_rim_cells_2d(tasks, workers=workers, label="landing")

    print("\n  usable (swing valid AND LEFT_RIM_READY AND Step 9R succeeds):", flush=True)
    print("    height |" + "".join(f"{d * 1e3:>7.0f}" for d in D_CORNER_M) + "   mm from corner",
          flush=True)
    for height_mm in LANDING_HEIGHTS_MM:
        row = "".join(
            "      +" if cell.usable else ("      ~" if cell.concession.feasible else "      -")
            for cell in cells if cell.concession.obstacle_height_m * 1e3 == height_mm
        )
        print(f"    {height_mm:>6} |{row}", flush=True)
    print("    + usable   ~ swing ok but descent not   - swing infeasible", flush=True)
    return cells


def run_budget(choices: dict, d_corner_m: float, workers: int) -> list:
    _section(f"C. the top-length budget (landing {d_corner_m * 1e3:.0f} mm from the corner)")
    settings = SwingGridSettings2D()
    tasks = [
        (height_mm / 1000.0, top_length, d_corner_m, APPROACH_CLEARANCE_M,
         settings, choices[height_mm].beta_deg, True)
        for height_mm in BUDGET_HEIGHTS_MM
        for top_length in TOP_LENGTHS_M
    ]
    cells = run_left_rim_cells_2d(tasks, workers=workers, label="budget ")

    print("\n  usable by (height, top length):", flush=True)
    print("    height |" + "".join(f"{t:>7.2f}" for t in TOP_LENGTHS_M) + "   m", flush=True)
    for height_mm in BUDGET_HEIGHTS_MM:
        subset = [c for c in cells if c.concession.obstacle_height_m * 1e3 == height_mm]
        row = "".join(
            "      +" if cell.usable else ("      ~" if cell.concession.feasible else "      -")
            for cell in subset
        )
        usable = [c.top_length_m for c in subset if c.usable]
        budget = f"{min(usable):.2f} m" if usable else "none in range"
        print(f"    {height_mm:>6} |{row}   -> min L_top = {budget}", flush=True)
    print("    + usable   ~ swing ok but descent not   - swing infeasible", flush=True)

    print("\n  hip travel the swing is asked for [m] (Step 2's ascent asks ~0.38 m "
          "to a hip 219 mm above the top; here the hip lands 144 mm above it):", flush=True)
    print("    height |" + "".join(f"{t:>7.2f}" for t in TOP_LENGTHS_M) + "   m", flush=True)
    for height_mm in BUDGET_HEIGHTS_MM:
        subset = [c for c in cells if c.concession.obstacle_height_m * 1e3 == height_mm]
        row = "".join(
            "     --" if c.hip_travel_m is None else f"{c.hip_travel_m:>7.2f}"
            for c in subset
        )
        print(f"    {height_mm:>6} |{row}", flush=True)

    failures = {
        (c.concession.binding_ceiling.value,
         None if c.concession.failure is None else c.concession.failure.value)
        for c in cells if not c.concession.feasible
    }
    if failures:
        print(f"\n  how the infeasible cells fail: {sorted(failures)}", flush=True)
    not_ready = [c for c in cells if c.readiness is not None and not c.readiness.ready]
    print(f"  landing poses that fail LEFT_RIM_READY: {len(not_ready)} / {len(cells)}",
          flush=True)
    no_descent = [
        c for c in cells
        if c.readiness is not None and c.readiness.ready and not c.readiness.descent_success
    ]
    print(f"  ready poses Step 9R could not descend from: {len(no_descent)} / "
          f"{len(cells) - len(not_ready)}", flush=True)
    return cells


def run_diagnostic(choices: dict, workers: int) -> list:
    """If the budget sweep fails everywhere, is it geometry or the approach posture?

    Step 2 froze ``theta_liftoff = 60 deg`` and the Step 2 close-out then showed
    theta is the swing's strongest internal freedom.  A negative result reported
    at one theta would therefore be a negative result about that theta, not
    about the strategy -- so before ``SWING_UP + ROLL_DOWN`` can be called
    refuted, the approach has to be optimised the way §3.1 says each primitive
    optimises over its own internal freedom.
    """

    _section(f"D. diagnostic: is it geometry, or is it the approach posture?  "
             f"(h = {DIAGNOSTIC_HEIGHT_MM} mm, L_top = {DIAGNOSTIC_TOP_LENGTH_M} m)")
    beta_deg = choices[DIAGNOSTIC_HEIGHT_MM].beta_deg
    tasks = [
        (DIAGNOSTIC_HEIGHT_MM / 1000.0, DIAGNOSTIC_TOP_LENGTH_M, BUDGET_D_CORNER_M,
         clearance, replace(SwingGridSettings2D(), theta_rad=float(np.deg2rad(theta_deg))),
         beta_deg, True)
        for theta_deg in DIAGNOSTIC_THETAS_DEG
        for clearance in DIAGNOSTIC_CLEARANCES_M
    ]
    cells = run_left_rim_cells_2d(tasks, workers=workers, label="diag   ")

    print("\n  usable by (approach theta, approach clearance):", flush=True)
    print("    theta |" + "".join(f"{c * 1e3:>7.0f}" for c in DIAGNOSTIC_CLEARANCES_M)
          + "   mm clearance", flush=True)
    for index, theta_deg in enumerate(DIAGNOSTIC_THETAS_DEG):
        subset = cells[index * len(DIAGNOSTIC_CLEARANCES_M):
                       (index + 1) * len(DIAGNOSTIC_CLEARANCES_M)]
        row = "".join(
            "      +" if cell.usable else ("      ~" if cell.concession.feasible else "      -")
            for cell in subset
        )
        travel = ", ".join(
            "--" if c.hip_travel_m is None else f"{c.hip_travel_m:.2f}" for c in subset
        )
        print(f"    {theta_deg:>5.0f} |{row}   hip travel [m]: {travel}", flush=True)
    return cells


def run_degenerate(workers: int) -> list:
    """How close to ``LEFT_RIM_READY`` can a swing actually land?

    Spec §2.8 says that if the direct landing fails, strategy #3 degrades to
    "land in a more extended pose, retract to 17 degrees, then roll down", and
    asks what that retract costs in top length.  The first half of that
    question is measurable here: sweep the landing theta and find the smallest
    one a swing can reach.  The gap between it and 17 degrees is what the
    retract has to close, and it is also the honest statement of *how far*
    strategy #3 misses -- a negative result with a distance attached rather
    than just a verdict.

    Each landing theta needs its own beta window, because which betas put the
    left rim on the ground is a function of theta.
    """

    _section("E. the degenerate version: how close to 17 deg can a swing land?")
    started = time.time()
    specs = [SharedTerrainSpec2D(height_m=h / 1000.0, top_length_m=DEGENERATE_TOP_LENGTH_M)
             for h in DEGENERATE_HEIGHTS_MM]
    windows = run_beta_windows_2d_at_thetas(
        specs[0], DEGENERATE_LANDING_THETAS_DEG, sample_step_deg=2.0)
    print(f"  beta windows per landing theta ({time.time() - started:.0f}s):", flush=True)
    betas: dict[float, float | None] = {}
    for theta_deg, window in zip(DEGENERATE_LANDING_THETAS_DEG, windows):
        if not window.samples:
            betas[theta_deg] = None
            print(f"    theta = {theta_deg:>4.0f} deg -> no left-rim window", flush=True)
            continue
        spec = specs[0]
        pivot = predicted_pivot_deg_2d(spec, theta_rad=float(np.deg2rad(theta_deg)))
        choice = choose_landing_beta_2d(window, pivot)
        betas[theta_deg] = choice.beta_deg
        print(f"    theta = {theta_deg:>4.0f} deg -> window "
              f"[{window.beta_min_deg:>7.1f}, {window.beta_max_deg:>7.1f}] "
              f"({window.width_deg:>5.1f} deg wide), hip pinned "
              f"{window.hip_above_top_m * 1e3:>6.1f} mm above the top, "
              f"chosen beta {choice.beta_deg:>7.1f} deg "
              f"(sufficient={choice.sufficient})", flush=True)

    settings = SwingGridSettings2D()
    tasks = [
        (height_mm / 1000.0, DEGENERATE_TOP_LENGTH_M, BUDGET_D_CORNER_M,
         APPROACH_CLEARANCE_M, settings, betas[theta_deg], True, theta_deg)
        for height_mm in DEGENERATE_HEIGHTS_MM
        for theta_deg in DEGENERATE_LANDING_THETAS_DEG
        if betas[theta_deg] is not None
    ]
    cells = run_left_rim_cells_2d(tasks, workers=workers, label="degen  ")

    print("\n  swing feasible by (height, landing theta):", flush=True)
    print("    height |" + "".join(f"{t:>7.0f}" for t in DEGENERATE_LANDING_THETAS_DEG)
          + "   deg", flush=True)
    for height_mm in DEGENERATE_HEIGHTS_MM:
        subset = [c for c in cells if c.concession.obstacle_height_m * 1e3 == height_mm]
        row = "".join(
            "      +" if cell.usable else ("      ~" if cell.concession.feasible else "      -")
            for cell in subset
        )
        reached = [c.landing_theta_deg for c in subset if c.concession.feasible]
        closest = f"{min(reached):.0f} deg" if reached else "none"
        print(f"    {height_mm:>6} |{row}   closest to 17 deg: {closest}", flush=True)
    print("    + usable (also LEFT_RIM_READY + descends)   ~ swing ok only   - swing infeasible",
          flush=True)
    return cells


def _plot_window(window, betas, choices, path: Path) -> Path:
    """The (b) window and the (c) budget/seam exchange, from the same samples."""

    samples = window.samples
    beta = [item.beta_deg for item in samples]

    fig, axes = plt.subplots(1, 2, figsize=(12.8, 4.6))

    ax = axes[0]
    ax.plot(beta, [item.alpha_deg for item in samples], "-", lw=2, color="#2563eb",
            label="left rim carries the contact")
    for limit in LEFT_RIM_ALPHA_LIMITS_DEG:
        ax.axhline(limit, color="#dc2626", ls="--", lw=1.2)
    ax.text(beta[0], LEFT_RIM_ALPHA_LIMITS_DEG[0] + 4, "  -180 deg seam (contact jumps 162 mm)",
            fontsize=7.5, color="#dc2626", va="bottom")
    ax.text(beta[0], LEFT_RIM_ALPHA_LIMITS_DEG[1] - 4, "  -40 deg: foot rim takes over",
            fontsize=7.5, color="#dc2626", va="top")
    if betas:
        wrapped = [b + 360.0 * np.ceil(-b / 360.0) - 360.0 for b in betas]
        ax.axvspan(min(wrapped), max(wrapped), color="#f59e0b", alpha=0.22,
                   label="where the rolling ascent arrives")
    ax.set_xlabel("beta [deg]")
    ax.set_ylabel("landing contact alpha [deg]")
    ax.set_title(f"(b) the left-rim window is {window.width_deg:.0f} deg wide "
                 f"and contiguous\nat theta = 17 deg, hip pinned "
                 f"{window.hip_above_top_m * 1e3:.1f} mm above the top")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, loc="best")

    ax = axes[1]
    ax.plot(beta, [item.rim_budget_deg for item in samples], "-", lw=2,
            color="#2563eb", label="rim arc ahead of the contact")
    ax.plot(beta, [item.seam_margin_deg for item in samples], "-", lw=2,
            color="#16a34a", label="distance to the nearer seam")
    colours = plt.cm.autumn(np.linspace(0.0, 0.75, len(choices)))
    for (height_mm, choice), colour in zip(sorted(choices.items()), colours):
        if choice.required_budget_deg is None:
            continue
        ax.axhline(choice.required_budget_deg, color=colour, ls=":", lw=1.3)
        ax.plot([choice.beta_deg], [choice.sample.seam_margin_deg], "o", ms=8,
                color=colour,
                label=f"h = {height_mm} mm: pivot {choice.required_budget_deg:.0f} deg "
                      f"-> margin {choice.sample.seam_margin_deg:.0f} deg")
    ax.set_xlabel("beta [deg]")
    ax.set_ylabel("[deg]")
    ax.set_title("(c) the exchange: buying pivot budget spends seam margin\n"
                 "dotted = what each height's corner pivot demands")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7.5, loc="best")

    fig.suptitle("Step 2b (b)+(c): the left rim is reachable, and the seam sets a height limit",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def _plot_cells(rows, path: Path) -> Path:
    """The evidence behind a negative result, not a picture of zeros.

    A usable/not map would be blank here -- nothing is usable -- so the panels
    show the three things that actually carry the finding: that the landing
    poses are fine and it is the *descent* that has a ceiling, that the ceiling
    is not the rim budget, and that every section fails the same way regardless
    of which axis was being swept.
    """

    budget = [r for r in rows if r["section"] == "budget"]
    heights = sorted({r["height_mm"] for r in budget})
    tops = sorted({r["top_length_m"] for r in budget})

    fig, axes = plt.subplots(1, 3, figsize=(15.0, 4.5))

    ax = axes[0]
    grid = np.full((len(heights), len(tops)), np.nan)
    for r in budget:
        grid[heights.index(r["height_mm"]), tops.index(r["top_length_m"])] = (
            1.0 if r["descent_success"] else 0.0
        )
    ax.imshow(grid, aspect="auto", origin="lower", cmap="RdYlGn", vmin=0, vmax=1)
    ax.set_xticks(range(len(tops)), [f"{t:.2f}" for t in tops])
    ax.set_yticks(range(len(heights)), [f"{h:.0f}" for h in heights])
    for i in range(len(heights)):
        for j in range(len(tops)):
            ax.text(j, i, "descends" if grid[i, j] else "stuck",
                    ha="center", va="center", fontsize=6.5)
    ax.set_xlabel("obstacle top length [m]")
    ax.set_ylabel("step height [mm]")
    ax.set_title("A. every landing pose passes LEFT_RIM_READY (35/35),\n"
                 "but Step 9R will not leave 14 of them", fontsize=10)

    ax = axes[1]
    first = {}
    for r in budget:
        first.setdefault(r["height_mm"], r)
    hs = sorted(first)
    ax.plot(hs, [first[h]["required_budget_deg"] for h in hs], "o--",
            color="#64748b", label="first-order pivot prediction")
    ax.plot(hs, [first[h]["chosen_rim_budget_deg"] for h in hs], "s-",
            color="#2563eb", label="rim arc the landing bought")
    for h in hs:
        ok = bool(first[h]["descent_success"])
        ax.plot(h, first[h]["chosen_rim_budget_deg"],
                "o" if ok else "X", ms=13 if not ok else 9,
                color="#16a34a" if ok else "#dc2626", zorder=5)
        ax.annotate(f"+{first[h]['chosen_rim_budget_deg'] - first[h]['required_budget_deg']:.1f}",
                    (h, first[h]["chosen_rim_budget_deg"]),
                    textcoords="offset points", xytext=(6, -12), fontsize=7)
    ax.set_xlabel("step height [mm]")
    ax.set_ylabel("left-rim arc [deg]")
    ax.set_title("B. and it is not the rim budget:\n"
                 "140 mm descends on +0.6 deg, 160 mm fails on +0.9", fontsize=10)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, loc="upper left")

    ax = axes[2]
    labels = {
        "landing": "B: L_top = 0.35 m\nx corner distance",
        "budget": "C: top length\n0.08 - 0.35 m",
        "diagnostic": "D: approach theta\nx clearance",
        "degenerate": "E: landing theta\n17 - 60 deg",
    }
    order = [k for k in labels if any(r["section"] == k for r in rows)]
    modes = sorted({str(r["failure"]) for r in rows if not r["feasible"]})
    bottom = np.zeros(len(order))
    for mode in modes:
        counts = np.array([
            sum(1 for r in rows if r["section"] == k and str(r["failure"]) == mode)
            for k in order
        ], dtype=float)
        ax.bar(range(len(order)), counts, bottom=bottom, label=mode)
        bottom += counts
    ax.set_xticks(range(len(order)), [labels[k] for k in order], fontsize=7.5)
    ax.set_ylabel("cells")
    ax.set_title("C. four different axes were swept.\n"
                 "none produced a feasible swing (0 / 90)", fontsize=10)
    ax.legend(fontsize=7.5)
    ax.grid(True, alpha=0.25, axis="y")

    fig.suptitle("Step 2b (a): SWING_UP cannot land in LEFT_RIM_READY -- "
                 "and the reason is not any of the four obvious ones", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description="Day 10--11 Step 2b")
    parser.add_argument("--workers", type=int,
                        default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--skip-budget", action="store_true")
    parser.add_argument(
        "--only", choices=("degenerate",), default=None,
        help="run one section and merge it into the existing CSV, instead of "
             "re-running the sweeps that already have answers",
    )
    parser.add_argument("--plots-only", action="store_true",
                        help="redraw from the CSV instead of re-running the sweeps")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    csv_path = args.output_dir / "day10_11_step2b_swing_to_left_rim_ready.csv"

    if args.plots_only:
        with csv_path.open(encoding="utf-8") as handle:
            rows = [_typed(row) for row in csv.DictReader(handle)]
        print(f"  wrote {_plot_cells(rows, args.output_dir / 'day10_11_step2b_landing_map.png')}",
              flush=True)
        return

    if args.only == "degenerate":
        cells = run_degenerate(args.workers)
        fresh = left_rim_rows(cells)
        for row in fresh:
            row["section"] = "degenerate"
        kept: list[dict] = []
        if csv_path.exists():
            with csv_path.open(encoding="utf-8") as handle:
                kept = [row for row in csv.DictReader(handle)
                        if row.get("section") != "degenerate"]
        # The kept rows come back as strings; the fresh ones are typed.  Writing
        # them together is fine -- csv stringifies either way -- but the columns
        # have to match, so any column only one side has is filled with "".
        columns = list(dict.fromkeys(
            [key for row in kept for key in row] + [key for row in fresh for key in row]
        ))
        merged = [{key: row.get(key, "") for key in columns} for row in kept + fresh]
        print(f"\nwrote {write_rows_csv(csv_path, merged)} "
              f"({len(kept)} kept + {len(fresh)} new)", flush=True)
        return

    started = time.time()
    windows, betas = run_window(args.output_dir)
    write_rows_csv(args.output_dir / "day10_11_step2b_beta_window.csv",
                   beta_window_rows(windows))

    if windows[0].best_beta_deg is None:
        print("\nNo left-rim window exists: strategy #3 is refuted at question (b).",
              flush=True)
        return

    heights = sorted(set(LANDING_HEIGHTS_MM) | set(BUDGET_HEIGHTS_MM))
    choices = choose_betas(windows[0], heights)
    print(f"  wrote {_plot_window(windows[0], betas, choices, args.output_dir / 'day10_11_step2b_alpha_seam_distance.png')}",
          flush=True)

    landing = run_landing(choices, args.workers)
    rows = left_rim_rows(landing)
    for row in rows:
        row["section"] = "landing"
        row.update(choices[round(row["height_mm"])].as_dict())

    budget: list = []
    if not args.skip_budget:
        budget = run_budget(choices, BUDGET_D_CORNER_M, args.workers)
        budget_rows = left_rim_rows(budget)
        for row in budget_rows:
            row["section"] = "budget"
            row.update(choices[round(row["height_mm"])].as_dict())
        rows += budget_rows

        if not any(cell.usable for cell in budget):
            diagnostic = run_diagnostic(choices, args.workers)
            diagnostic_rows = left_rim_rows(diagnostic)
            for row in diagnostic_rows:
                row["section"] = "diagnostic"
                row.update(choices[round(row["height_mm"])].as_dict())
            rows += diagnostic_rows

            degenerate = run_degenerate(args.workers)
            degenerate_rows = left_rim_rows(degenerate)
            for row in degenerate_rows:
                row["section"] = "degenerate"
            rows += degenerate_rows

    path = write_rows_csv(csv_path, rows)
    print(f"\nStep 2b finished in {time.time() - started:.1f}s; wrote {path}", flush=True)
    if budget:
        print(f"  wrote {_plot_cells(rows, args.output_dir / 'day10_11_step2b_landing_map.png')}",
              flush=True)
        usable = [cell for cell in budget if cell.usable]
        if usable:
            print(f"  minimum usable top length: {min(c.top_length_m for c in usable):.2f} m "
                  f"(L_transition for ROLL_UP + ROLL_DOWN is 0.20--0.27 m)", flush=True)
        else:
            print("  no usable (height, top length) cell: see section D before "
                  "calling strategy #3 refuted.", flush=True)

    flagged = sum(1 for row in rows if row["greedy_backtrack_suspected"])
    print(f"  greedy_backtrack_suspected: {flagged} (must be 0)", flush=True)
    kinds = {row["requirement_kind"] for row in rows if row["feasible"]}
    print(f"  requirement kinds among feasible cells: {kinds or '(none feasible)'}",
          flush=True)


if __name__ == "__main__":
    main()
