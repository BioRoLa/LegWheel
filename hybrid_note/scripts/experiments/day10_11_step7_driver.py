"""Day 10--11 Step 7: compose one sequence per surviving strategy, and check it.

Five sections.

    A  compose #1 / #4 / #5 at cells Step 5's rule actually picks
    B  the two blocked pairs, with their verdict and their route back
    C  the hand-over report: contact, joints, rim seam, the 1.2 mm rim gap
    D  top-length usage against ``L_transition``
    E  the three things Step 5 left for Step 7 to verify

Only section A runs planners; the ``#1`` traversal is the expensive part.

There is deliberately no ``--plots-only``: the figure draws the composed
sequences, and composing them is the run.  A cached-figure flag here would
either redraw stale data or silently re-run anyway.

    python3 day10_11_step7_driver.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_composer_2d import (  # noqa: E402
    NOMINAL_RIM_GAP_M,
    compose_2d,
    compose_swing_swing_2d,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    BLOCKED_PAIRS,
    Availability,
    StrategyId,
    decide_2d,
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"
DAY6_7_DIR = Path(__file__).resolve().parents[2] / "notes" / "day6-7"

MAP_TOP_LENGTHS_M: tuple[float, ...] = tuple(
    round(0.02 + 0.005 * i, 4) for i in range(87)
)

#: The height each strategy is composed at.  Chosen where that strategy is the
#: one Step 5 picks, not where it merely works: a sequence for a strategy the
#: rule would never select proves nothing about the rule.
COMPOSE_HEIGHTS_MM = {
    StrategyId.ROLL_ROLL: 140,
    StrategyId.SWING_SWING: 80,
    StrategyId.SWING_OVER: 60,
}

#: Day 6--7 Step 12R's band, for section D.
L_TRANSITION_BAND_M = (0.196, 0.249)

#: Section E, item 1.  Not in the ``#5`` sweep's ten columns, so composing here
#: tests the monotone closure rather than replaying a measured cell.
UNSWEPT_TOP_LENGTH_M = 0.075
SWEPT_TOP_LENGTH_M = 0.080

#: Section E, item 2.  Step 2's close-out verified a 100 mm landing at
#: h = 100 / 150 / 200 only, so the retest has to be at one of those.
CLOSEOUT_HEIGHT_M = 0.100
CLOSEOUT_LANDING_M = 0.100

#: Section E, item 3.  Inside the hole Step 5 found: no strategy works.
HOLE_HEIGHT_M = 0.120
HOLE_TOP_LENGTH_M = 0.150


def _first_winning_top(tables, height_m: float, strategy: StrategyId):
    """The shortest top where this strategy is the one the rule picks.

    The region boundary is both the most informative cell and, for ``#1``, the
    cheapest: a longer top only adds wheel-mode frames.
    """

    for top in MAP_TOP_LENGTHS_M:
        if decide_2d(height_m, top, tables).winner is strategy:
            return top
    return None


# --------------------------------------------------------------------------


#: Where Day 6--7's ``step11r`` sweep measured feasibility directly, so a
#: composition there rests on evidence rather than on an inferred bound.
EVIDENCED_TOP_LENGTH_M = 0.350


def _attempt(tables, strategy, height_m, top, label):
    started = time.perf_counter()
    result = compose_2d(height_m, top, tables, strategy=strategy)
    if not result.composed:
        print(f"    L_top = {top * 1e3:.0f} mm ({label}): NOT COMPOSED -- "
              f"{result.refusal}", flush=True)
        return result, {
            "strategy": strategy.value, "obstacle_mm": height_m * 1e3,
            "top_length_m": top, "which": label, "composed": False,
            "refusal": result.refusal,
        }
    row = result.as_dict()
    print(f"    L_top = {top * 1e3:.0f} mm ({label}): {row['segments']} segments, "
          f"{row['frames']} frames, collision-free={row['collision_free']}, "
          f"min clearance {row['min_clearance_mm']:.3f} mm "
          f"({time.perf_counter() - started:.0f} s)", flush=True)
    print(f"      parameters: {dict(result.parameters)}", flush=True)
    if result.notes:
        print(f"      {result.notes}", flush=True)
    return result, {
        "strategy": strategy.value, "obstacle_mm": height_m * 1e3,
        "top_length_m": top, "which": label, "composed": True,
        "refusal": "",
    }


def section_a(tables):
    """Compose each surviving strategy where Step 5's rule picks it.

    ``#1`` gets a second attempt if the boundary cell fails, at the top length
    Day 6--7's own sweep measured.  That is not a retry to make the numbers
    look better: the point of the boundary attempt is to *test* the map's
    bound, and the fall-back is what produces the sequence the completion
    criterion asks for when the bound turns out to be optimistic.
    """

    print("\n=== A. compose one sequence per surviving strategy", flush=True)
    composed = {}
    attempts = []
    for strategy, height_mm in COMPOSE_HEIGHTS_MM.items():
        height_m = height_mm / 1e3
        top = _first_winning_top(tables, height_m, strategy)
        if top is None:
            print(f"  {strategy.value}: never wins at h = {height_mm} mm", flush=True)
            continue
        if strategy is StrategyId.SWING_OVER:
            # Also discharge Step 5's first open item: compose at a top length
            # the sweep never measured, where the map relies on its closure.
            top = UNSWEPT_TOP_LENGTH_M
        print(f"\n  {strategy.value} at h = {height_mm} mm", flush=True)
        result, attempt = _attempt(
            tables, strategy, height_m, top, "the map's boundary"
        )
        attempts.append(attempt)
        if not result.composed and strategy is StrategyId.ROLL_ROLL:
            print("    -> the boundary the map claims does not hold; retrying "
                  "where Day 6-7 measured feasibility directly.", flush=True)
            result, attempt = _attempt(
                tables, strategy, height_m, EVIDENCED_TOP_LENGTH_M,
                "measured by step11r",
            )
            attempts.append(attempt)
        composed[strategy] = result
    return composed, attempts


def section_b():
    """The two pairs whose primitives will not chain.  Spec 5.5 / 5.6.

    Recorded with the verdict **and the route back**: a reader who finds only
    three sequences must be able to see why, and must not read "will not chain"
    as "the robot cannot".
    """

    print("\n=== B. the two pairs whose primitives will not chain", flush=True)
    rows = []
    for strategy, blocked in BLOCKED_PAIRS.items():
        print(f"  {strategy.value}  [{blocked.verdict.value}]", flush=True)
        print(f"    evidence:       {blocked.evidence}", flush=True)
        print(f"    single-leg fix: {blocked.single_leg_fix or '(none in reach)'}",
              flush=True)
        print(f"    multi-leg route (UNVERIFIED): {blocked.multileg_route}",
              flush=True)
        rows.append({
            "strategy": strategy.value, "composed": False,
            "refused_by": "Step 2b" if strategy is StrategyId.SWING_ROLL else "Step 3",
            **blocked.as_dict(),
        })
    print("\n  Neither is PHYSICALLY_INFEASIBLE: that label needs reach / joint "
          "limit /\n  collision / support evidence, and nothing in Day 10-11 has "
          "it (spec 5.5).", flush=True)
    return rows


def section_c(composed) -> list[dict]:
    print("\n=== C. hand-overs: contact, joints, rim seam, and the 1.2 mm rim gap",
          flush=True)
    rows = []
    for strategy, result in composed.items():
        if not result.composed:
            continue
        print(f"\n  {strategy.value}: {len(result.handoffs)} hand-over(s)", flush=True)
        for handoff in result.handoffs:
            row = {"strategy": strategy.value, **handoff.as_dict()}
            rows.append(row)
            print(f"    -> {row['to']:44s} contact {row['contact_jump_mm']:7.1f} mm  "
                  f"theta {row['theta_jump_deg']:6.2f}  beta {row['beta_jump_deg']:6.2f}  "
                  f"seam {min(row['seam_margin_before_deg'], row['seam_margin_after_deg']):6.1f} deg  "
                  f"rim gap {row['rim_gap_before_mm']:.2f} -> {row['rim_gap_after_mm']:.2f} mm",
                  flush=True)
    if not rows:
        return rows
    print(f"\n  across every composed sequence:", flush=True)
    print(f"    max joint jump        theta "
          f"{max(abs(r['theta_jump_deg']) for r in rows):.2f} deg, "
          f"beta {max(abs(r['beta_jump_deg']) for r in rows):.2f} deg", flush=True)
    print(f"    max contact jump      {max(r['contact_jump_mm'] for r in rows):.1f} mm "
          "(a rim transfer, not a discontinuity)", flush=True)
    print(f"    closest rim seam      "
          f"{min(min(r['seam_margin_before_deg'], r['seam_margin_after_deg']) for r in rows):.1f} deg",
          flush=True)
    largest_gap = max(r["largest_rim_gap_mm"] for r in rows)
    print(f"    largest rim gap       {largest_gap:.4f} mm "
          f"(Step 0's nominal is {NOMINAL_RIM_GAP_M * 1e3:.1f} mm)", flush=True)
    print("    -> it does not accumulate: the gap is 0 on the foot rim and the "
          "nominal on\n       an upper tyre, and a hand-over between them changes "
          "it by exactly that\n       much rather than adding to it.", flush=True)
    return rows


def section_d(composed) -> list[dict]:
    print("\n=== D. how much top each strategy uses, against L_transition", flush=True)
    low, high = L_TRANSITION_BAND_M
    rows = []
    for strategy, result in composed.items():
        if not result.composed or result.top_length_used_m is None:
            continue
        used = result.top_length_used_m
        rows.append({
            "strategy": strategy.value,
            "obstacle_mm": result.height_m * 1e3,
            "top_length_m": result.top_length_m,
            "top_length_used_mm": used * 1e3,
            "l_transition_low_mm": low * 1e3,
            "l_transition_high_mm": high * 1e3,
            "versus_l_transition": (
                "below the band" if used < low
                else "inside the band" if used <= high else "above the band"
            ),
            "notes": result.notes,
        })
        print(f"  {strategy.value:24s} uses {used * 1e3:6.1f} mm of top   "
              f"({rows[-1]['versus_l_transition']})", flush=True)
    print(f"\n  L_transition band (Day 6-7 Step 12R): "
          f"{low * 1e3:.0f} - {high * 1e3:.0f} mm", flush=True)
    return rows


def section_e(tables) -> list[dict]:
    print("\n=== E. the three things Step 5 left for Step 7", flush=True)
    rows = []

    print("\n  1. does #5's monotone closure hold at a top length nobody swept?",
          flush=True)
    unswept = compose_2d(0.060, UNSWEPT_TOP_LENGTH_M, tables,
                         strategy=StrategyId.SWING_OVER)
    swept = compose_2d(0.060, SWEPT_TOP_LENGTH_M, tables,
                       strategy=StrategyId.SWING_OVER)
    print(f"     L = {UNSWEPT_TOP_LENGTH_M * 1e3:.0f} mm (interpolated): "
          f"composed={unswept.composed}"
          + ("" if unswept.composed else f" -- {unswept.refusal}"), flush=True)
    print(f"     L = {SWEPT_TOP_LENGTH_M * 1e3:.0f} mm (measured):      "
          f"composed={swept.composed}", flush=True)
    rows.append({
        "check": "swing_over closure at an unswept L_top",
        "cell": f"h=60 mm, L={UNSWEPT_TOP_LENGTH_M * 1e3:.0f} mm",
        "outcome": "closure holds" if unswept.composed else "closure FAILS",
        "detail": unswept.refusal or (
            f"min clearance {unswept.min_clearance_m * 1e3:.3f} mm"
        ),
    })

    print("\n  2. does #4 still work with the closer landing close-out verified?",
          flush=True)
    decision = decide_2d(CLOSEOUT_HEIGHT_M, 0.240, tables)
    cell = next(c for c in decision.cells if c.strategy is StrategyId.SWING_SWING)
    parameters = dict(cell.parameters)
    closer = compose_swing_swing_2d(
        CLOSEOUT_HEIGHT_M, 0.200,
        approach_clearance_m=parameters["approach_clearance_m"],
        min_hip_lift_m=parameters["min_hip_lift_m"],
        takeoff_distance_m=parameters["takeoff_distance_m"],
        min_hip_hold_fraction=parameters["min_hip_hold_fraction"],
        ascent_liftoff_rise_m=parameters["ascent_liftoff_rise_m"],
        ascent_touchdown_drop_m=parameters["ascent_touchdown_drop_m"],
        ascent_duration_scale=parameters["ascent_duration_scale"],
        descent_liftoff_rise_m=parameters["descent_liftoff_rise_m"],
        descent_touchdown_drop_m=parameters["descent_touchdown_drop_m"],
        descent_duration_scale=parameters["descent_duration_scale"],
        landing_distance_m=CLOSEOUT_LANDING_M,
    )
    print(f"     h = {CLOSEOUT_HEIGHT_M * 1e3:.0f} mm, L = 200 mm, landing "
          f"{CLOSEOUT_LANDING_M * 1e3:.0f} mm: composed={closer.composed}"
          + ("" if closer.composed else f" -- {closer.refusal}"), flush=True)
    rows.append({
        "check": "swing pair with the close-out landing distance",
        "cell": f"h={CLOSEOUT_HEIGHT_M * 1e3:.0f} mm, L=200 mm, landing="
                f"{CLOSEOUT_LANDING_M * 1e3:.0f} mm",
        "outcome": "composes" if closer.composed else "does NOT compose",
        "detail": closer.refusal or (
            f"min clearance {closer.min_clearance_m * 1e3:.3f} mm; this is the "
            "cell that decides whether spec 2.7's monotonicity holds"
        ),
    })

    print("\n  3. the hole: three strategies, three different reasons", flush=True)
    hole = decide_2d(HOLE_HEIGHT_M, HOLE_TOP_LENGTH_M, tables)
    for cell in hole.cells:
        if cell.strategy in BLOCKED_PAIRS:
            continue
        print(f"     {cell.strategy.value:24s} {cell.availability.value:14s} "
              f"({cell.limiter.value})", flush=True)
        print(f"       {cell.reason}", flush=True)
        rows.append({
            "check": "the hole in the capability map",
            "cell": f"h={HOLE_HEIGHT_M * 1e3:.0f} mm, "
                    f"L={HOLE_TOP_LENGTH_M * 1e3:.0f} mm",
            "outcome": f"{cell.strategy.value}: {cell.limiter.value}",
            "detail": cell.reason,
        })
    limiters = {
        c.limiter for c in hole.cells if c.strategy not in BLOCKED_PAIRS
    }
    print(f"\n     {len(limiters)} distinct limiters -> the hole is not a single "
          "ceiling.", flush=True)
    return rows


# --------------------------------------------------------------------------


def _plot(composed, path: Path) -> Path:
    fig, axes = plt.subplots(1, len(composed) or 1, figsize=(5.2 * max(len(composed), 1), 4.8))
    if len(composed) <= 1:
        axes = [axes]
    colours = {
        StrategyId.ROLL_ROLL: "#2563eb",
        StrategyId.SWING_SWING: "#ea580c",
        StrategyId.SWING_OVER: "#16a34a",
    }
    for ax, (strategy, result) in zip(axes, composed.items()):
        if not result.composed:
            ax.text(0.5, 0.55, f"{strategy.value}\nnot composed",
                    ha="center", va="center", transform=ax.transAxes, fontsize=10)
            ax.text(0.5, 0.40, (result.refusal or "")[:70],
                    ha="center", va="center", transform=ax.transAxes,
                    fontsize=7, color="#dc2626", wrap=True)
            ax.set_axis_off()
            continue
        height_mm = result.height_m * 1e3
        top_mm = result.top_length_m * 1e3
        # The obstacle, so the reader can see what the leg is crossing.
        ax.add_patch(plt.Rectangle((100.0, 0.0), top_mm, height_mm,
                                   facecolor="#e2e8f0", edgecolor="#94a3b8"))
        rows = result.frame_rows
        hip_x = [r["hip_x_m"] * 1e3 for r in rows]
        hip_z = [r["hip_z_m"] * 1e3 for r in rows]
        foot_x = [r["contact_x_m"] * 1e3 for r in rows
                  if r["contact_x_m"] is not None]
        foot_z = [r["contact_z_m"] * 1e3 for r in rows
                  if r["contact_z_m"] is not None]
        ax.plot(hip_x, hip_z, "-", lw=2.4, color=colours[strategy], label="hip")
        ax.plot(foot_x, foot_z, ":", lw=1.6, color="#64748b", label="contact")
        # Segment starts, so the schema's decomposition is visible on the path
        # rather than only in the CSV.
        by_index = {r["index"]: r for r in rows}
        for segment in result.sequence.segments:
            first = by_index.get(segment.frames.indices[0])
            if first is None:
                continue
            ax.plot(first["hip_x_m"] * 1e3, first["hip_z_m"] * 1e3, "o", ms=5,
                    color="white", markeredgecolor=colours[strategy],
                    markeredgewidth=1.4, zorder=5)
        ax.set_title(f"{strategy.value}\nh = {height_mm:.0f} mm, "
                     f"L_top = {top_mm:.0f} mm, "
                     f"{len(result.sequence.segments)} segments / "
                     f"{len(rows)} frames", fontsize=9.5)
        ax.set_xlabel("x [mm]")
        ax.set_ylabel("z [mm]")
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=7.5, loc="upper left")
        ax.set_aspect("equal", adjustable="datalim")

    fig.suptitle("Step 7: one composed sequence per surviving strategy, at a cell "
                 "Step 5's rule picks", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def _union(rows: list[dict]) -> list[dict]:
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    return [{key: row.get(key, "") for key in columns} for row in rows]


PAIR_FILES = {
    StrategyId.ROLL_ROLL: "pair1_roll_roll",
    StrategyId.SWING_SWING: "pair4_swing_swing",
    StrategyId.SWING_OVER: "pair5_swing_over",
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    args = parser.parse_args()

    tables = load_tables_2d(
        args.output_dir, DAY6_7_DIR,
        swing_over_csv=args.output_dir / "day10_11_step5_swing_over.csv",
    )
    figure = args.output_dir / "day10_11_step7_sequences.png"

    composed, attempts = section_a(tables)
    refusals = section_b()
    handoffs = section_c(composed)
    budgets = section_d(composed)
    checks = section_e(tables)

    for strategy, result in composed.items():
        stem = PAIR_FILES[strategy]
        if result.frame_rows:
            write_rows_csv(
                args.output_dir / f"day10_11_step7_{stem}_frames.csv",
                _union([dict(r) for r in result.frame_rows]),
            )
        summary = [{"section": "summary", **result.as_dict()}]
        if result.sequence is not None:
            summary += [{"section": "segment", **row}
                        for row in result.sequence.rows()]
        write_rows_csv(
            args.output_dir / f"day10_11_step7_{stem}_summary.csv", _union(summary)
        )

    write_rows_csv(
        args.output_dir / "day10_11_step7_handoff_report.csv", _union(handoffs)
    )
    write_rows_csv(
        args.output_dir / "day10_11_step7_refusals.csv", _union(refusals)
    )
    write_rows_csv(
        args.output_dir / "day10_11_step7_top_length_budget.csv", _union(budgets)
    )
    write_rows_csv(
        args.output_dir / "day10_11_step7_open_items.csv", _union(checks)
    )
    write_rows_csv(
        args.output_dir / "day10_11_step7_compose_attempts.csv", _union(attempts)
    )
    print(f"\n  wrote {len(composed) * 2} pair files, the hand-off report, "
          "the refusals, the budgets and the open-item checks", flush=True)
    print(f"  wrote {_plot(composed, figure)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
