"""Day 10--11 Step 3: the descent map, and what the ascent leaves behind.

Four sections, in the order that costs least to learn the most:

``A. the map``
    ``height x takeoff distance``, ``arrival = SWING_UP``.  The primary
    deliverable: the descent's half of the Step 5 overlay, on the same heights
    Step 2 used so a sequence can be priced end to end.

``B. the 160 mm claim``
    ``swing_off_step_2d``'s docstring records one measurement -- at 160 mm a
    hip that drops with the foot collides, one that holds half the drop clears
    by about 1.8 mm.  Step 0 deliberately re-based both worlds onto the Day 6--7
    obstacle, so the numbers cannot be expected to match exactly; what has to
    survive is the *claim*.  Reproducing or refuting it is a completion
    criterion.

``C. theta``
    Step 2's close-out found theta is the swing's strongest internal freedom
    and that fixing it at 60 deg makes the ascent map an upper bound.  Doing
    the same check here *now* rather than afterwards is the whole lesson from
    that close-out.

``D. arrival = ROLL_UP``
    The coupling in spec §2.7.  A rolling ascent hands over a **right-rim**
    pose barely past the leading edge, so the takeoff distance is not free --
    it is whatever the top length leaves.  Sweeping the top length there gives
    the ``ROLL_UP + SWING_DOWN`` budget that §2.7's second row is missing, and
    it asks the mirror of the Step 2b question: a left-rim *landing* is blocked
    by the alpha = -40 deg seam, so is a right-rim *takeoff* blocked by +40?

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step3_driver.py
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

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day10_11_swing_sweep_2d import (  # noqa: E402
    standing_scene_2d,
)
from hybrid_note.scripts.experiments.day10_11_swing_off_sweep_2d import (  # noqa: E402
    DEFAULT_HEIGHTS_MM,
    DEFAULT_TAKEOFF_DISTANCES_M,
    SwingOffGridSettings2D,
    run_roll_up_descent_cells_2d,
    run_roll_up_exit_poses_2d,
    run_swing_off_cells_2d,
    swing_off_rows,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"

CLAIM_HEIGHT_M = 0.160
CLAIM_TAKEOFF_M = 0.16

THETA_HEIGHTS_MM: tuple[int, ...] = (100, 150, 200)
THETA_DEGS: tuple[float, ...] = (40.0, 50.0, 60.0, 70.0, 85.0)

ROLL_UP_HEIGHTS_MM: tuple[int, ...] = (60, 100, 140)
ROLL_UP_THETA_CLIMB_DEG: tuple[float, ...] = (40.0, 60.0, 85.0)
ROLL_UP_TOP_LENGTHS_M: tuple[float, ...] = (0.20, 0.26, 0.32, 0.38, 0.45)

#: Section E.  If the direct hand-over fails, the fall-back is not "swing from
#: further along the top" -- it is Day 6--7's own Step 6.5: retract and reset
#: until the *foot* rim carries the contact again, then take off from there.
#: ``day6_7_step12r_transition_measurements_all_cells.csv`` already measured
#: what that costs in top run: 20.5 mm at theta_climb = 40 deg rising to
#: 64.4 mm at 75 deg, against an ``L_transition`` of 196--249 mm.  So the
#: fall-back is 3 to 10 times cheaper in top length -- but it leaves the leg at
#: the wheel-mode theta, and section C says a low theta is expensive on the
#: descent.  That trade is what section E measures.
RETRACT_THETAS_DEG: tuple[float, ...] = (17.0, 25.0, 30.0)
RETRACT_HEIGHTS_MM: tuple[int, ...] = (60, 100, 140, 200)
RETRACT_SWEEP_HEIGHTS_MM: tuple[int, ...] = (100, 140)

#: Section F.  Section C found theta = 40 deg feasible and section E found
#: 30 deg not, at every takeoff distance it tried.  So the descent has a theta
#: floor between them.  Where it sits decides whether ``ROLL_UP + SWING_DOWN``
#: can exist at all: the fall-back has to retract *past* the floor to put the
#: foot rim back down, and if the floor is high there may be no pose that is
#: both foot-rim-ready and extended enough to take off from.
FLOOR_THETAS_DEG: tuple[float, ...] = (32.0, 35.0, 38.0)
FLOOR_TAKEOFFS_M: tuple[float, ...] = (0.08, 0.12, 0.16)
FLOOR_HEIGHT_MM = 100


def _section(title: str) -> None:
    print(f"\n{'=' * 78}\n{title}\n{'=' * 78}", flush=True)


def _hold_table(rows, index_key, column_key, columns, index_values, label) -> None:
    print(f"\n  {label}", flush=True)
    print("    height |" + "".join(f"{c:>8}" for c in columns), flush=True)
    for value in index_values:
        cells = []
        for column in columns:
            match = next(
                (r for r in rows
                 if r[index_key] == value and abs(r[column_key] - float(column)) < 1e-6),
                None,
            )
            if match is None:
                cells.append("       .")
            elif not match["feasible"]:
                cells.append("       x")
            else:
                cells.append(f"{match['min_hip_hold_fraction']:>8.3f}")
        print(f"    {value:>6} |" + "".join(cells), flush=True)


def run_map(settings: SwingOffGridSettings2D, workers: int) -> list:
    _section("A. the descent map: height x takeoff distance (arrival = SWING_UP)")
    tasks = [
        (h / 1000.0, takeoff, settings)
        for h in DEFAULT_HEIGHTS_MM
        for takeoff in DEFAULT_TAKEOFF_DISTANCES_M
    ]
    cells = run_swing_off_cells_2d(tasks, workers=workers, label="map    ")
    rows = swing_off_rows(cells)
    for row in rows:
        row["section"] = "map"

    _hold_table(rows, "height_mm", "takeoff_distance_m",
                [f"{t:.2f}" for t in DEFAULT_TAKEOFF_DISTANCES_M],
                [float(h) for h in DEFAULT_HEIGHTS_MM],
                "min hip hold [fraction of step height]  (x = infeasible)")

    feasible = [r for r in rows if r["feasible"]]
    print(f"\n  feasible: {len(feasible)} / {len(rows)}", flush=True)
    if feasible:
        heights = sorted({r["height_mm"] for r in feasible})
        print(f"  height ceiling for a descent: {max(heights):.0f} mm "
              f"(sweep went to {max(DEFAULT_HEIGHTS_MM)} mm)", flush=True)
    ceilings = {(r["binding_ceiling"], r["failure"]) for r in rows if not r["feasible"]}
    print(f"  how the infeasible cells fail: {sorted(ceilings) or '(none failed)'}",
          flush=True)
    flagged = sum(1 for r in rows if r["greedy_backtrack_suspected"])
    print(f"  greedy_backtrack_suspected: {flagged} (must be 0)", flush=True)
    refused = sum(r["planner_refusals"] for r in rows)
    if refused:
        print(f"  evaluations the planner refused to judge (leg finished in the "
              f"air): {refused:.0f} across "
              f"{sum(1 for r in rows if r['planner_refusals'])} cells", flush=True)
    return rows


def run_claim(settings: SwingOffGridSettings2D) -> list:
    _section("B. the 160 mm claim from swing_off_step_2d's docstring")
    print("  claim: at 160 mm a hip that drops with the foot collides; one that\n"
          "         holds half the drop clears by about 1.8 mm.\n"
          "  Two things differ from where that was measured, and they have to be\n"
          "  separated before the claim can be called reproduced or refuted:\n"
          "    geometry  Step 0 re-based both worlds onto the Day 6--7 obstacle\n"
          "              (0.10 / 0.35 rather than 0.20 / 0.45).  A real difference.\n"
          "    sampling  this sweep uses 121 leg samples and 31 trajectory samples;\n"
          "              the showcase used 241 and 51.  Spec 6.2 warns the\n"
          "              clearances in play are millimetres and move with sampling,\n"
          "              so a 1.6 mm verdict is exactly the size that could flip.",
          flush=True)

    from hybrid_note.scripts.experiments.day10_11_swing_off_sweep_2d import (
        minimum_swing_off_concession_2d,
    )

    variants = {
        "step3 sampling (121 / 31)": replace(settings),
        "showcase sampling (241 / 51)": replace(settings, arc_samples=241,
                                                sample_count=51),
    }

    rows = []
    verdicts = {}
    for label, base in variants.items():
        print(f"\n  --- {label} ---", flush=True)
        by_hold = {}
        for hold in (0.0, 0.25, 0.5):
            pinned = replace(base, hip_hold_ladder=(hold,),
                             touchdown_drop_ladder_m=(0.0,))
            cell = minimum_swing_off_concession_2d(
                CLAIM_HEIGHT_M, CLAIM_TAKEOFF_M, pinned)
            row = cell.as_dict()
            row["section"] = "claim"
            row["claim_variant"] = label
            row["claim_hold_fraction"] = hold
            rows.append(row)
            by_hold[hold] = row
            margin = row["min_clearance_mm"]
            print(f"    hold = {hold:>4.2f} ({hold * CLAIM_HEIGHT_M * 1e3:>5.1f} mm) -> "
                  f"{'clears' if row['feasible'] else 'FAILS '}  "
                  f"min clearance {'--' if margin is None else f'{margin:>6.2f} mm'}  "
                  f"failure={row['failure']}", flush=True)
        verdicts[label] = (not by_hold[0.0]["feasible"]) and bool(by_hold[0.5]["feasible"])
        print(f"    -> claim holds under this sampling: {verdicts[label]}", flush=True)

    print("\n  verdict:", flush=True)
    if all(verdicts.values()):
        print("    reproduced under both samplings.", flush=True)
    elif any(verdicts.values()):
        print("    **sampling-dependent** -- the claim holds under one sampling and\n"
              "    not the other, which makes it a statement about the discretisation\n"
              "    rather than about the leg.  Spec 6.2's warning is the finding.",
              flush=True)
    else:
        print("    not reproduced under either sampling, so it is not a sampling\n"
              "    artefact: on the Day 6--7 obstacle a 160 mm descent from 0.16 m\n"
              "    back is refused at every hold.  Section A says the same cell is\n"
              "    infeasible and that 160 mm *is* feasible from 0.14 m or less --\n"
              "    so the mechanism (hold rescues the descent) survives; the\n"
              "    operating point does not.", flush=True)
    return rows


def run_theta(settings: SwingOffGridSettings2D, workers: int) -> list:
    _section("C. does theta matter on the descent too? (Step 2 close-out said it "
             "does on the ascent)")
    tasks = [
        (h / 1000.0, CLAIM_TAKEOFF_M,
         replace(settings, theta_rad=float(np.deg2rad(theta))))
        for h in THETA_HEIGHTS_MM
        for theta in THETA_DEGS
    ]
    cells = run_swing_off_cells_2d(tasks, workers=workers, label="theta  ")
    rows = swing_off_rows(cells)
    for row in rows:
        row["section"] = "theta"

    _hold_table(rows, "height_mm", "theta_deg",
                [f"{t:.0f}" for t in THETA_DEGS],
                [float(h) for h in THETA_HEIGHTS_MM],
                "min hip hold [fraction] by (height, theta)  (x = infeasible)")
    return rows


def run_roll_up(settings: SwingOffGridSettings2D, workers: int) -> list:
    _section("D. arrival = ROLL_UP: the descent that starts on the right rim")
    started = time.time()
    exit_tasks = [
        (h / 1000.0, theta, max(ROLL_UP_TOP_LENGTHS_M), settings.x_start_m,
         settings.arc_samples, 0.04)
        for h in ROLL_UP_HEIGHTS_MM
        for theta in ROLL_UP_THETA_CLIMB_DEG
    ]
    print(f"  running {len(exit_tasks)} traversals to get the roll-up exit poses "
          f"(a few minutes each, in parallel)...", flush=True)
    poses = run_roll_up_exit_poses_2d(exit_tasks, workers=workers)
    print(f"  exits obtained in {time.time() - started:.0f}s", flush=True)

    usable = [p for p in poses if p.reached]
    rims = {p.rim for p in usable}
    print(f"\n  roll-up exits reached: {len(usable)} / {len(poses)};  rims: {rims}",
          flush=True)

    tasks = [
        (pose, top_length, settings)
        for pose in usable
        for top_length in ROLL_UP_TOP_LENGTHS_M
    ]
    cells = run_roll_up_descent_cells_2d(tasks, workers=workers)
    rows = swing_off_rows(cells)
    for row in rows:
        row["section"] = "roll_up_arrival"

    print("\n  feasible by (height, theta_climb) x top length:", flush=True)
    print("    h  theta |" + "".join(f"{t:>7.2f}" for t in ROLL_UP_TOP_LENGTHS_M)
          + "   m", flush=True)
    for pose in usable:
        subset = sorted(
            (r for r in rows
             if r["height_mm"] == pose.height_m * 1e3
             and abs(r["theta_deg"] - pose.theta_climb_deg) < 1e-6),
            key=lambda r: r["top_length_m"])
        line = "".join("      +" if r["feasible"] else "      -" for r in subset)
        ok = [r["top_length_m"] for r in subset if r["feasible"]]
        budget = f"min L_top = {min(ok):.2f} m" if ok else "none in range"
        print(f"    {pose.height_m * 1e3:>3.0f} {pose.theta_climb_deg:>5.0f} |{line}"
              f"   {budget}", flush=True)

    failures = {(r["binding_ceiling"], r["failure"]) for r in rows if not r["feasible"]}
    print(f"\n  how the infeasible cells fail: {sorted(failures) or '(none failed)'}",
          flush=True)
    return rows


def run_retract(settings: SwingOffGridSettings2D, workers: int) -> list:
    """Can the leg take off after Day 6--7's retract-and-reset?

    Section D refused the *direct* hand-over: a swing straight off the roll-up
    exit, which is a right-rim pose.  That is not the only way down.  Day 6--7's
    Step 6.5 already rolls the contact back onto the foot rim, and its measured
    cost is 20--64 mm of top run rather than the 196--249 mm a full
    ``L_transition`` spends.  The catch is the pose it leaves: wheel-mode theta.

    So the question section E asks is not "is there a fall-back" -- there is,
    and it is cheap in top length -- but "can a descent start from the pose the
    fall-back produces".  Section C already showed the descent gets expensive
    as theta drops (h = 150 mm needs the hip to hold *all* of the drop at
    40 deg), and 17 deg is far below that.
    """

    _section("E. the fall-back: take off after retract-and-reset (low theta, foot rim)")
    print("  Day 6--7 measured the retract's cost in top run:\n"
          "    theta_climb = 40 deg -> 20.5 mm      theta_climb = 60 deg -> 41.0 mm\n"
          "    theta_climb = 75 deg -> 64.4 mm      (L_transition itself: 196-249 mm)\n"
          "  so the fall-back is 3-10x cheaper in top length than rolling down.\n"
          "  What it costs instead is theta: the leg ends up in wheel mode.",
          flush=True)

    tasks = [
        (h / 1000.0, CLAIM_TAKEOFF_M,
         replace(settings, theta_rad=float(np.deg2rad(theta))))
        for h in RETRACT_HEIGHTS_MM
        for theta in RETRACT_THETAS_DEG
    ]
    tasks += [
        (h / 1000.0, takeoff,
         replace(settings, theta_rad=float(np.deg2rad(RETRACT_THETAS_DEG[0]))))
        for h in RETRACT_SWEEP_HEIGHTS_MM
        for takeoff in DEFAULT_TAKEOFF_DISTANCES_M
    ]
    cells = run_swing_off_cells_2d(tasks, workers=workers, label="retract")
    rows = swing_off_rows(cells)
    for row in rows:
        row["section"] = "retract"

    fixed = [r for r in rows if abs(r["takeoff_distance_m"] - CLAIM_TAKEOFF_M) < 1e-9]
    _hold_table(fixed, "height_mm", "theta_deg",
                [f"{t:.0f}" for t in RETRACT_THETAS_DEG],
                [float(h) for h in RETRACT_HEIGHTS_MM],
                f"min hip hold at takeoff = {CLAIM_TAKEOFF_M:.2f} m, low theta "
                f"(x = infeasible)")

    swept = [r for r in rows
             if abs(r["theta_deg"] - RETRACT_THETAS_DEG[0]) < 1e-6
             and r["height_mm"] in {float(h) for h in RETRACT_SWEEP_HEIGHTS_MM}]
    if swept:
        _hold_table(swept, "height_mm", "takeoff_distance_m",
                    [f"{t:.2f}" for t in DEFAULT_TAKEOFF_DISTANCES_M],
                    [float(h) for h in RETRACT_SWEEP_HEIGHTS_MM],
                    f"min hip hold at theta = {RETRACT_THETAS_DEG[0]:.0f} deg "
                    f"(wheel mode), by takeoff distance")

    ok = [r for r in rows if r["feasible"]]
    print(f"\n  feasible: {len(ok)} / {len(rows)}", flush=True)
    wheel = [r for r in rows if abs(r["theta_deg"] - 17.0) < 1e-6]
    wheel_ok = [r for r in wheel if r["feasible"]]
    print(f"  at wheel-mode theta (17 deg): {len(wheel_ok)} / {len(wheel)} feasible",
          flush=True)
    if wheel_ok:
        print(f"    -> the fall-back exists; cheapest hold seen is "
              f"{min(r['min_hip_hold_fraction'] for r in wheel_ok):.3f}", flush=True)
    else:
        print("    -> the fall-back does NOT produce a usable takeoff pose either:\n"
              "       ROLL_UP + SWING_DOWN would need a retract that stops short of\n"
              "       wheel mode, which Day 6--7's Step 6.5 does not currently do.",
              flush=True)
    return rows


def run_theta_floor(settings: SwingOffGridSettings2D, workers: int) -> list:
    """Where exactly is the descent's theta floor, and does takeoff distance move it?

    Two different shapes of answer, and they lead to different Step 5 rules.
    If the floor is independent of the takeoff distance it is a property of the
    leg and can be written as one number.  If a shorter takeoff buys a lower
    theta, the two are coupled and the rule needs both.
    """

    _section("F. where is the descent's theta floor? (C said 40 works, E said 30 does not)")
    tasks = [
        (FLOOR_HEIGHT_MM / 1000.0, takeoff,
         replace(settings, theta_rad=float(np.deg2rad(theta))))
        for theta in FLOOR_THETAS_DEG
        for takeoff in FLOOR_TAKEOFFS_M
    ]
    cells = run_swing_off_cells_2d(tasks, workers=workers, label="floor  ")
    rows = swing_off_rows(cells)
    for row in rows:
        row["section"] = "theta_floor"

    print(f"\n  min hip hold at h = {FLOOR_HEIGHT_MM} mm (x = infeasible)", flush=True)
    print("     theta |" + "".join(f"{t:>8.2f}" for t in FLOOR_TAKEOFFS_M)
          + "   m takeoff", flush=True)
    for theta in FLOOR_THETAS_DEG:
        cells_row = [
            next(r for r in rows
                 if abs(r["theta_deg"] - theta) < 1e-6
                 and abs(r["takeoff_distance_m"] - takeoff) < 1e-9)
            for takeoff in FLOOR_TAKEOFFS_M
        ]
        line = "".join(
            "       x" if not r["feasible"] else f"{r['min_hip_hold_fraction']:>8.3f}"
            for r in cells_row
        )
        print(f"    {theta:>6.0f} |{line}", flush=True)

    ok = [r for r in rows if r["feasible"]]
    if ok:
        floor = min(r["theta_deg"] for r in ok)
        independent = len({r["theta_deg"] for r in ok}) == len(
            {r["theta_deg"] for r in rows if r["feasible"]}
        ) and all(
            len([r for r in ok if abs(r["theta_deg"] - t) < 1e-6])
            == len(FLOOR_TAKEOFFS_M)
            for t in {r["theta_deg"] for r in ok}
        )
        print(f"\n  lowest feasible theta at this height: {floor:.0f} deg", flush=True)
        print(f"  floor independent of takeoff distance: {independent}", flush=True)
    else:
        print("\n  none of 32 / 35 / 38 deg works: the floor is at or above 40 deg.",
              flush=True)
    return rows


def _plot(rows, path: Path) -> Path:
    """Three panels, one per finding, rather than one per section.

    The sections are how the sweep was organised; they are not what it found.
    What it found is a ceiling on takeoff distance, a floor on theta, and two
    different reasons ``ROLL_UP + SWING_DOWN`` does not currently work.
    """

    mapping = [r for r in rows if r["section"] == "map"]
    theta = [r for r in rows if r["section"] in ("theta", "retract", "theta_floor")]

    fig, axes = plt.subplots(1, 3, figsize=(16.0, 4.8))

    # --- A: the map, and the ceiling that runs through it ------------------
    ax = axes[0]
    heights = sorted({r["height_mm"] for r in mapping})
    takeoffs = sorted({r["takeoff_distance_m"] for r in mapping})
    grid = np.full((len(heights), len(takeoffs)), np.nan)
    for r in mapping:
        if r["feasible"]:
            grid[heights.index(r["height_mm"]),
                 takeoffs.index(r["takeoff_distance_m"])] = r["min_hip_hold_fraction"]
    image = ax.imshow(grid, aspect="auto", origin="lower", cmap="viridis",
                      vmin=0.0, vmax=1.0)
    ceiling_x, ceiling_y = [], []
    for i, height in enumerate(heights):
        ok = [j for j in range(len(takeoffs)) if not np.isnan(grid[i, j])]
        if ok:
            # ``+ 0.5`` puts the line on the cell boundary; clamp it so a row
            # that is feasible everywhere does not draw outside the axes.
            ceiling_x.append(min(max(ok) + 0.5, len(takeoffs) - 0.5))
            ceiling_y.append(i)
        for j in range(len(takeoffs)):
            if np.isnan(grid[i, j]):
                ax.text(j, i, "x", ha="center", va="center",
                        color="#dc2626", fontsize=8, fontweight="bold")
    ax.step(ceiling_x, ceiling_y, where="mid", color="#dc2626", lw=2.2,
            label="max usable takeoff")
    ax.set_xticks(range(len(takeoffs)), [f"{t:.2f}" for t in takeoffs], fontsize=7)
    ax.set_yticks(range(len(heights)), [f"{h:.0f}" for h in heights], fontsize=7)
    ax.set_xlabel("takeoff distance from the trailing edge [m]")
    ax.set_ylabel("step height [mm]")
    ax.set_title("A. min hip hold, and the ceiling on takeoff distance\n"
                 "rolling needs L_top ABOVE a bound; this needs it BELOW one",
                 fontsize=9.5)
    ax.legend(fontsize=7.5, loc="lower left")
    fig.colorbar(image, ax=ax, fraction=0.046)

    # --- B: theta, in the currency Step 5 needs ---------------------------
    ax = axes[1]

    def standing_hip_mm(theta_deg: float) -> float:
        """Hip above the ground for a standing pose -- measured, not tabulated."""

        scene = standing_scene_2d(
            SwingOffGridSettings2D().spec_for(0.10),
            float(np.deg2rad(theta_deg)),
            hip_x_m=0.30,
            support_height_m=0.0,
        )
        return float(scene.hip_pose.position_world_xz_m[1]) * 1000.0

    fixed = [r for r in theta
             if abs(r["takeoff_distance_m"] - CLAIM_TAKEOFF_M) < 1e-9]
    # One colour per height, assigned up front.  Sections C, E and F each
    # cover a different theta band at the same height, so a height's curve is
    # assembled across sections rather than drawn per section.
    palette = {60.0: "#94a3b8", 100.0: "#2563eb", 140.0: "#7c3aed",
               150.0: "#16a34a", 200.0: "#ea580c"}
    heights_b = sorted({r["height_mm"] for r in fixed})
    for height_mm in heights_b:
        subset = sorted((r for r in fixed if r["height_mm"] == height_mm),
                        key=lambda r: r["theta_deg"])
        colour = palette.get(height_mm, "#64748b")
        good = [(r["theta_deg"],
                 standing_hip_mm(r["theta_deg"])
                 + r["min_hip_hold_fraction"] * height_mm)
                for r in subset if r["feasible"]]
        bad = [r["theta_deg"] for r in subset if not r["feasible"]]
        if good:
            ax.plot([t for t, _ in good], [y for _, y in good], "o-",
                    color=colour, lw=1.8, ms=5, label=f"h = {height_mm:.0f} mm")
            # The cheapest theta is the point Step 5 would actually pick.
            best = min(good, key=lambda pair: pair[1])
            ax.plot(best[0], best[1], "*", ms=15, color=colour,
                    markeredgecolor="black", markeredgewidth=0.6, zorder=5)
        # Infeasible thetas sit on their own row so they never overlap, and
        # a height with no feasible point at all is named in the row label.
        row_y = 132.0 - 7.0 * heights_b.index(height_mm)
        if bad:
            ax.plot(bad, [row_y] * len(bad), "x", ms=7, color=colour)
            ax.text(88.0, row_y, f"{height_mm:.0f}", fontsize=6.5, va="center",
                    ha="left", color=colour)
    ax.axvspan(12.0, 35.0, color="#dc2626", alpha=0.09)
    ax.axvline(35.0, color="#dc2626", ls="--", lw=1.5,
               label="theta floor = 35 deg (section F)")
    ax.axvline(17.0, color="#0ea5e9", ls=":", lw=1.5,
               label="17 deg = where the retract stops (section E)")
    ax.set_xlim(12, 92)
    ax.set_ylim(95, 300)
    ax.set_xlabel("theta [deg]")
    ax.set_ylabel("hip above the lower ground at touchdown [mm]")
    ax.set_title("B. a floor at 35 deg, and an optimum that RISES with height\n"
                 "star = cheapest theta; x rows = infeasible at every hold",
                 fontsize=9.5)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=6.5, loc="upper left")

    # --- C: why ROLL_UP + SWING_DOWN does not work, in two parts -----------
    ax = axes[2]
    groups = [
        ("D: direct\nhand-over", "roll_up_arrival"),
        ("E: after\nretract", "retract"),
        ("F: theta\n32-38", "theta_floor"),
        ("C: theta\n40-85", "theta"),
    ]
    labels, ok_counts, bad_counts = [], [], []
    for label, section in groups:
        subset = [r for r in rows if r["section"] == section]
        labels.append(label)
        ok_counts.append(sum(1 for r in subset if r["feasible"]))
        bad_counts.append(sum(1 for r in subset if not r["feasible"]))
    x = np.arange(len(labels))
    ax.bar(x, bad_counts, color="#dc2626", label="infeasible")
    ax.bar(x, ok_counts, bottom=bad_counts, color="#16a34a", label="feasible")
    for i, (bad, good) in enumerate(zip(bad_counts, ok_counts)):
        ax.text(i, bad + good + 0.8, f"{good}/{bad + good}", ha="center", fontsize=8)
    ax.set_xticks(x, labels, fontsize=7)
    ax.set_ylabel("cells")
    ax.set_title("C. two different walls, not one\n"
                 "D is the rim seam; E is the theta floor", fontsize=9.5)
    ax.legend(fontsize=7.5)
    ax.grid(True, alpha=0.25, axis="y")

    fig.suptitle("Step 3: the descent side -- a ceiling on takeoff distance, "
                 "a floor on theta", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def _typed(row: dict) -> dict:
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Day 10--11 Step 3")
    parser.add_argument("--workers", type=int,
                        default=max(1, (os.cpu_count() or 2) - 1))
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--skip-roll-up", action="store_true")
    parser.add_argument("--plots-only", action="store_true")
    parser.add_argument(
        "--sections", default="map,claim,theta,roll_up_arrival",
        help="comma-separated sections to run; anything already in the CSV and "
             "not re-run is kept.  Section A costs ~25 minutes, so a rerun that "
             "only needs C and D should not pay for it again.",
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "day10_11_step3_swing_off_sweep.csv"
    png_path = args.output_dir / "day10_11_step3_min_hip_hold_map.png"

    if args.plots_only:
        with csv_path.open(encoding="utf-8") as handle:
            rows = [_typed(row) for row in csv.DictReader(handle)]
        print(f"  wrote {_plot(rows, png_path)}", flush=True)
        return

    settings = SwingOffGridSettings2D()
    started = time.time()
    wanted = {name.strip() for name in args.sections.split(",") if name.strip()}

    # Sections not being re-run are kept from the existing CSV, so a rerun that
    # only needs C and D does not pay 25 minutes for A again.
    rows: list[dict] = []
    if csv_path.exists():
        with csv_path.open(encoding="utf-8") as handle:
            rows = [_typed(row) for row in csv.DictReader(handle)
                    if row.get("section") not in wanted]
        if rows:
            print(f"  kept {len(rows)} rows from sections "
                  f"{sorted({r['section'] for r in rows})} already in the CSV",
                  flush=True)

    def checkpoint(new_rows: list[dict], what: str) -> None:
        """Write after every section.

        The first attempt at this sweep died 85 cells into section A and lost
        all of them; the second died writing section B and would have lost C
        and D.  A section that has finished is a result and should not depend
        on the ones after it surviving.

        Rows from different sections do not all carry the same columns, so the
        header is the union and missing values are written empty -- otherwise
        ``csv.DictWriter`` refuses the whole file over one extra field.
        """

        rows.extend(new_rows)
        columns = list(dict.fromkeys(key for row in rows for key in row))
        write_rows_csv(csv_path, [{k: row.get(k, "") for k in columns} for row in rows])
        print(f"  [checkpoint] {what}: {len(new_rows)} rows, {len(rows)} total -> "
              f"{csv_path.name}", flush=True)

    if "map" in wanted:
        checkpoint(run_map(settings, args.workers), "section A")
    if "claim" in wanted:
        checkpoint(run_claim(settings), "section B")
    if "theta" in wanted:
        checkpoint(run_theta(settings, args.workers), "section C")
    if "roll_up_arrival" in wanted and not args.skip_roll_up:
        checkpoint(run_roll_up(settings, args.workers), "section D")
    if "retract" in wanted:
        checkpoint(run_retract(settings, args.workers), "section E")
    if "theta_floor" in wanted:
        checkpoint(run_theta_floor(settings, args.workers), "section F")

    columns = list(dict.fromkeys(key for row in rows for key in row))
    final = [{k: row.get(k, "") for k in columns} for row in rows]
    print(f"\nStep 3 finished in {time.time() - started:.1f}s; "
          f"wrote {write_rows_csv(csv_path, final)}", flush=True)
    print(f"  wrote {_plot(rows, png_path)}", flush=True)
    print(f"  total generate_swing_2d calls: {sum(r['evaluations'] for r in rows)}",
          flush=True)


if __name__ == "__main__":
    main()
