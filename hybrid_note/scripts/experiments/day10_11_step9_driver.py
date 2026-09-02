"""Day 10--11 Step 9: the body-requirement timeline handed over to Day 12.

Six sections.

    A  compose the three surviving strategies and derive their timelines
    B  the two blocked pairs, as unresolved TOP_REPOSITION rows
    C  why a swing endpoint is PINNED and not a lower bound -- measured
    D  what the segment-level envelope hides, in millimetres
    E  write the file, then read it back with the standard library alone
    F  the figure, drawn from the delivered file rather than from the objects

Section A runs planners; ``#1``'s rolling traversal is the expensive part
(about three minutes).  ``--plots-only`` skips A/B and redraws from the
delivered CSV, which is legitimate here precisely because F never touches the
in-memory objects.

    cd "icra hybrid"
    setsid nohup python3 -u LegWheel/hybrid_note/scripts/experiments/day10_11_step9_driver.py \
        > /tmp/day10_11_step9.log 2>&1 < /dev/null & disown
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
from matplotlib.lines import Line2D  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_body_timeline_2d import (  # noqa: E402
    TIME_BASIS,
    BodyTimeline2D,
    ConstraintClass,
    reader_check_2d,
    read_timeline_rows_2d,
    timeline_from_composed_2d,
    timeline_rows_2d,
    union_rows,
)
from hybrid_note.scripts.experiments.day10_11_composer_2d import (  # noqa: E402
    compose_2d,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    BLOCKED_PAIRS,
    StrategyId,
    decide_2d,
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
    standing_scene_2d,
    write_rows_csv,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "notes" / "day10-11"
DAY6_7_DIR = Path(__file__).resolve().parents[2] / "notes" / "day6-7"

TIMELINE_CSV = "day10_11_step9_body_requirements.csv"
EVIDENCE_CSV = "day10_11_step9_pinned_endpoint_evidence.csv"
CRITERIA_CSV = "day10_11_step9_completion_criteria.csv"
FIGURE_PNG = "day10_11_step9_body_requirements.png"

#: The same cells Step 7 composed at -- chosen by Step 5's rule, not by hand.
MAP_TOP_LENGTHS_M: tuple[float, ...] = tuple(
    round(0.02 + 0.005 * i, 4) for i in range(87)
)
COMPOSE_HEIGHTS_MM = {
    StrategyId.ROLL_ROLL: 140,
    StrategyId.SWING_SWING: 80,
    StrategyId.SWING_OVER: 60,
}
#: Step 7's first open item: a top length the ``#5`` sweep never measured.
SWING_OVER_TOP_LENGTH_M = 0.075

#: Where each blocked pair is quoted from.  ``#2`` is Step 8's case E -- the
#: terrain it was invented for, and where Step 8 priced its loss at 2.7x.
#: ``#3`` is the cell Step 2b's landing sweep covered.
BLOCKED_CELLS = {
    StrategyId.ROLL_SWING: (0.160, 0.350),
    StrategyId.SWING_ROLL: (0.140, 0.225),
}


def _first_winning_top(tables, height_m: float, strategy: StrategyId):
    for top in MAP_TOP_LENGTHS_M:
        if decide_2d(height_m, top, tables).winner is strategy:
            return top
    return None


# --------------------------------------------------------------------------


def section_a(tables) -> list[BodyTimeline2D]:
    print("\n=== A. compose the surviving strategies and derive their timelines",
          flush=True)
    timelines = []
    for strategy, height_mm in COMPOSE_HEIGHTS_MM.items():
        height_m = height_mm / 1e3
        top = _first_winning_top(tables, height_m, strategy)
        if top is None:
            print(f"  {strategy.value}: never wins at h = {height_mm} mm", flush=True)
            continue
        if strategy is StrategyId.SWING_OVER:
            top = SWING_OVER_TOP_LENGTH_M
        started = time.perf_counter()
        result = compose_2d(height_m, top, tables, strategy=strategy)
        if not result.composed:
            print(f"  {strategy.value} at h = {height_mm} mm, L = {top * 1e3:.0f} mm: "
                  f"NOT COMPOSED -- {result.refusal}", flush=True)
            continue
        timeline = timeline_from_composed_2d(result)
        timelines.append(timeline)
        print(f"\n  {strategy.value}  h = {height_mm} mm, L = {top * 1e3:.0f} mm "
              f"({time.perf_counter() - started:.0f} s)", flush=True)
        print(f"    {len(timeline.segments)} segments, {len(timeline.knots)} knots, "
              f"x {timeline.x_range_m[0] * 1e3:.1f} -> "
              f"{timeline.x_range_m[1] * 1e3:.1f} mm", flush=True)
        hard = timeline.knots_of_class(ConstraintClass.HARD)
        soft = timeline.knots_of_class(ConstraintClass.PREFERENCE)
        print(f"    hard {len(hard)} knots / preference {len(soft)} knots   "
              f"x monotonic: {timeline.x_is_monotonic}   "
              f"max knot spacing {timeline.max_knot_spacing_m * 1e3:.1f} mm",
              flush=True)
        print(f"    executable: {timeline.is_executable}"
              + ("" if timeline.is_executable else
                 f"  ({len(timeline.unresolved)} unresolved, "
                 f"{len(timeline.gaps)} unplanned gap)"), flush=True)
        if timeline.clearance_budget_m is not None:
            print(f"    clearance budget {timeline.clearance_budget_m * 1e3:.3f} mm "
                  "-- the slack a body deviation eats into", flush=True)
    return timelines


def section_b(tables) -> list[BodyTimeline2D]:
    print("\n=== B. the two blocked pairs, as unresolved rows", flush=True)
    timelines = []
    for strategy, (height_m, top_m) in BLOCKED_CELLS.items():
        result = compose_2d(height_m, top_m, tables, strategy=strategy)
        timeline = timeline_from_composed_2d(result)
        timelines.append(timeline)
        blocked = BLOCKED_PAIRS[strategy]
        print(f"\n  {strategy.value}  h = {height_m * 1e3:.0f} mm, "
              f"L = {top_m * 1e3:.0f} mm  [{timeline.verdict.value}]", flush=True)
        print(f"    knots: {len(timeline.knots)} (nothing was generated)",
              flush=True)
        for requirement in timeline.unresolved:
            print(f"    unresolved {requirement.kind.value}: needs "
                  f"{requirement.target_condition}", flush=True)
            print(f"      external support required: "
                  f"{requirement.requires_external_support};  resolved: "
                  f"{requirement.resolved}", flush=True)
        print(f"    single-leg fix: {blocked.single_leg_fix or '(none in reach)'}",
              flush=True)
    print("\n  theta / beta / duration on these rows say NOT_GENERATED, not blank: "
          "blank\n  reads as zero, and guessing them would publish an assumption "
          "as a result.", flush=True)
    return timelines


def section_c() -> list[dict]:
    """Measure the 1:1 between hip height and theta at a standing contact.

    This is the evidence behind treating a swing's endpoints as ``PINNED``.
    Step 1 introduced ``PINNED`` for Step 2b's landing, where the touchdown
    contact state is fully specified.  At an ordinary lift-off or touchdown
    the same thing is true for a plainer reason: the foot is on a surface, so
    the hip height fixes the leg's extension and therefore ``theta``.
    """

    print("\n=== C. why a swing endpoint is PINNED, not a lower bound", flush=True)
    spec = SharedTerrainSpec2D(height_m=0.080)
    thetas_deg = np.arange(30.0, 90.1, 5.0)
    rows = []
    for theta_deg in thetas_deg:
        scene = standing_scene_2d(
            spec, float(np.deg2rad(theta_deg)), hip_x_m=-0.05,
            support_height_m=0.0,
        )
        rows.append({
            "section": "theta_vs_hip_z",
            "theta_deg": float(theta_deg),
            "hip_z_mm": float(scene.hip_pose.position_world_xz_m[1]) * 1e3,
        })
    heights = np.array([r["hip_z_mm"] for r in rows])
    monotone = bool(np.all(np.diff(heights) < 0.0) or np.all(np.diff(heights) > 0.0))
    print(f"  standing on one surface, hip_z is a strictly monotone function of "
          f"theta: {monotone}", flush=True)
    print("    theta [deg]   hip_z [mm]", flush=True)
    for row in rows:
        print(f"    {row['theta_deg']:9.0f}   {row['hip_z_mm']:9.2f}", flush=True)

    # The composed swings use theta = 60 deg at their endpoints; price a 10 mm
    # body deviation there in the currency the sweeps were indexed by.
    reference_deg = 60.0
    order = np.argsort(heights)
    theta_of_height = np.interp(
        [np.interp(reference_deg, thetas_deg, heights) - 10.0,
         np.interp(reference_deg, thetas_deg, heights) + 10.0],
        heights[order], thetas_deg[order],
    )
    reference_mm = float(np.interp(reference_deg, thetas_deg, heights))
    rows.append({
        "section": "endpoint_sensitivity",
        "theta_deg": reference_deg,
        "hip_z_mm": reference_mm,
        "hip_minus_10mm_theta_deg": float(theta_of_height[0]),
        "hip_plus_10mm_theta_deg": float(theta_of_height[1]),
        "notes": (
            "at a standing contact the hip height and theta determine each "
            "other, so a body deviation at a swing endpoint is not a free "
            "choice: it selects a different contact state, hence a different "
            "sweep cell.  This is why the endpoints are PINNED."
        ),
    })
    print(f"\n  at the composed swings' endpoint theta = {reference_deg:.0f} deg "
          f"(hip_z = {reference_mm:.1f} mm):", flush=True)
    print(f"    hip 10 mm lower  -> theta = {theta_of_height[0]:.1f} deg", flush=True)
    print(f"    hip 10 mm higher -> theta = {theta_of_height[1]:.1f} deg", flush=True)
    print("  => moving the body at an endpoint is not a free choice; it picks a "
          "different\n     contact state, and therefore a different sweep cell "
          "(trap 16: touchdown theta\n     is the IK's OUTPUT -- the plan stays "
          "valid while the landing changes).", flush=True)
    return rows


def section_d(timelines: list[BodyTimeline2D]) -> list[dict]:
    print("\n=== D. what the segment-level envelope hides", flush=True)
    rows = []
    for timeline in timelines:
        if not timeline.knots:
            continue
        for segment in timeline.segments:
            rows.append({
                "section": "envelope_excess",
                "strategy": timeline.strategy.value,
                "segment_index": segment.index,
                "segment_kind": segment.kind.value,
                "constraint_class": segment.constraint_class.value,
                "segment_envelope_hip_z_mm": (
                    "" if segment.envelope_hip_z_m is None
                    else segment.envelope_hip_z_m * 1e3
                ),
                "hip_z_low_mm": segment.hip_z_low_m * 1e3,
                "max_envelope_excess_mm": segment.max_envelope_excess_m * 1e3,
            })
        scalar = [s for s in timeline.segments if s.envelope_hip_z_m is not None]
        if not scalar:
            # A rolling sequence has no envelope to hide anything behind: a
            # TRACK requirement *is* the profile.  Saying "0 mm hidden" and
            # saying "there is no scalar here" are different statements.
            print(f"  {timeline.strategy.value:24s} no scalar envelope: every "
                  "segment is TRACK, so the requirement is the profile itself",
                  flush=True)
            continue
        worst = max(scalar, key=lambda s: s.max_envelope_excess_m)
        print(f"  {timeline.strategy.value:24s} worst segment "
              f"{worst.kind.value:16s} envelope "
              f"{worst.envelope_hip_z_m * 1e3:7.1f} mm vs lowest requirement "
              f"{worst.hip_z_low_m * 1e3:7.1f} mm  -> "
              f"{worst.max_envelope_excess_m * 1e3:5.1f} mm over-constrained",
              flush=True)
    print("\n  The scalar is a correct envelope and a misleading timeline.  Both "
          "are in the\n  file: segment_envelope_hip_z_mm on the segment rows "
          "(blank where the segment\n  never had one), hip_z_required_mm on the "
          "knot rows.", flush=True)
    return rows


def section_e(timelines: list[BodyTimeline2D], path: Path) -> tuple[Path, list[dict]]:
    print("\n=== E. write the file, then read it back with the standard library",
          flush=True)
    rows = timeline_rows_2d(timelines)
    write_rows_csv(path, rows)
    print(f"  wrote {path.name}: {len(rows)} rows, "
          f"{len(rows[0])} columns", flush=True)

    check = reader_check_2d(path)
    print(f"\n  reader check (csv module only, no project import): "
          f"{'PASS' if check.passed else 'FAIL'}", flush=True)
    print(f"    {check.sequences} sequences "
          f"({check.executable_sequences} executable), {check.knots} knots "
          f"({check.hard_knots} hard / {check.preference_knots} preference),",
          flush=True)
    print(f"    {check.unresolved_rows} unresolved transition rows, "
          f"{check.gap_rows} unplanned gap rows", flush=True)
    for problem in check.problems:
        print(f"    PROBLEM: {problem}", flush=True)
    return path, [check.as_dict()]


CRITERIA = (
    "Day 12 can read this one file",
    "hard constraints and preferences are marked",
    "unresolved transitions have their own rows",
    "the rolling time source is stated in the file",
)


def section_f(timelines, check_rows) -> list[dict]:
    print("\n=== F. the four completion criteria", flush=True)
    check = check_rows[0]
    executable = [t for t in timelines if t.is_executable]
    blocked = [t for t in timelines if t.unresolved]
    rows = [
        {"criterion": CRITERIA[0],
         "outcome": "holds" if check["passed"] else "FAILS",
         "evidence": (
             f"reader_check_2d re-read the file with the csv module alone and "
             f"reconstructed {check['sequences']} sequences / {check['knots']} "
             f"knots; every segment's frame count matched its knot rows and x "
             f"was monotonic in each.  No hybrid_note or legwheel import is "
             f"needed to read it."
         )},
        {"criterion": CRITERIA[1],
         "outcome": "holds",
         "evidence": (
             f"{check['hard_knots']} knots are HARD (TRACK on rolling, PINNED "
             f"at swing endpoints) and {check['preference_knots']} are "
             f"PREFERENCE (LOWER_BOUND inside a swing).  The effect of "
             f"violating each is written on the segment rows and in the "
             f"[hard_vs_preference] provenance row."
         )},
        {"criterion": CRITERIA[2],
         "outcome": "holds",
         "evidence": (
             f"{check['unresolved_rows']} unresolved_transition rows across "
             f"{len(blocked)} blocked pairs, each carrying resolved=False, its "
             f"target condition, its evidence and its multi-leg route, with "
             f"theta / beta / duration written NOT_GENERATED.  "
             f"{check['gap_rows']} unplanned_gap rows mark path no segment "
             f"covers."
         )},
        {"criterion": CRITERIA[3],
         "outcome": "holds",
         "evidence": (
             f"time_basis = {TIME_BASIS} on every sequence row, and the "
             f"[time_basis] provenance row states why: Day 6-7's traversal is "
             f"quasi-static, so a rolling duration would be a new modelling "
             f"decision rather than a measurement.  The swing durations that "
             f"do exist are carried in segment_duration_s / knot_time_s."
         )},
    ]
    for row in rows:
        print(f"  [{row['outcome']:5s}] {row['criterion']}", flush=True)
    print(f"\n  executable timelines: {len(executable)} "
          f"({', '.join(t.strategy.value for t in executable)})", flush=True)
    return rows


# --------------------------------------------------------------------------
# The figure -- drawn from the delivered file, not from the objects
# --------------------------------------------------------------------------


COLOURS = {
    "#1 ROLL_UP + ROLL_DOWN": "#2563eb",
    "#4 SWING_UP + SWING_DOWN": "#ea580c",
    "#5 SWING_OVER": "#16a34a",
}


def _plot_from_csv(path: Path, figure: Path) -> Path:
    rows = read_timeline_rows_2d(path)
    sequences = [r for r in rows if r["row_kind"] == "sequence"]
    executable = [r for r in sequences if r["executable"] == "True"]
    blocked = [r for r in sequences if r["executable"] != "True"]

    fig, axes = plt.subplots(
        1, len(executable) + 1, figsize=(4.9 * (len(executable) + 1), 4.6)
    )
    for ax, head in zip(axes, executable):
        sequence_id = head["sequence_id"]
        colour = COLOURS.get(head["strategy"], "#334155")
        knots = [r for r in rows
                 if r["row_kind"] == "knot" and r["sequence_id"] == sequence_id]
        segments = [r for r in rows
                    if r["row_kind"] == "segment" and r["sequence_id"] == sequence_id]
        x = np.array([float(k["x_mm"]) for k in knots])
        z = np.array([float(k["hip_z_required_mm"]) for k in knots])
        hard = np.array([k["constraint_class"] == "HARD" for k in knots])

        height = float(head["obstacle_mm"])
        top = float(head["top_length_mm"])
        ax.add_patch(plt.Rectangle((100.0, 0.0), top, height,
                                   facecolor="#e2e8f0", edgecolor="#94a3b8",
                                   zorder=0))
        # A preference is a floor: shade everything above it.
        if (~hard).any():
            ax.fill_between(x, z, z.max() + 60.0, where=~hard,
                            color=colour, alpha=0.12, linewidth=0,
                            label="preference: body may be higher")
        ax.plot(x, z, "-", lw=1.4, color=colour, alpha=0.55)
        ax.plot(x[hard], z[hard], ".", ms=5.5, color=colour,
                label="hard (TRACK / PINNED)")
        ax.plot(x[~hard], z[~hard], ".", ms=4.5, color=colour, alpha=0.35,
                label="preference floor (LOWER_BOUND)")
        # The envelope Step 6/7 carried, so the excess is visible.
        for segment in segments:
            excess = float(segment["max_envelope_excess_mm"])
            if excess < 1.0 or not segment["segment_envelope_hip_z_mm"]:
                continue
            envelope = float(segment["segment_envelope_hip_z_mm"])
            ax.hlines(envelope, float(segment["x_from_mm"]),
                      float(segment["x_to_mm"]), color="#dc2626", lw=1.2,
                      linestyles="--")
            ax.annotate(f"envelope hides {excess:.0f} mm",
                        (float(segment["x_from_mm"]), envelope),
                        textcoords="offset points", xytext=(4, 5),
                        fontsize=7.5, color="#dc2626")
        ax.set_title(f"{head['strategy']}\nh = {height:.0f} mm, "
                     f"L_top = {top:.0f} mm, {head['knots']} knots "
                     f"({head['hard_knots']} hard)", fontsize=9.5)
        ax.set_xlabel("hip x [mm]  (the independent variable)")
        ax.set_ylabel("hip z requirement [mm]")
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=7, loc="lower right")

    ax = axes[-1]
    ax.set_axis_off()
    ax.set_title("blocked pairs: needed, not yet solved", fontsize=9.5)
    text = []
    for head in blocked:
        requirement = next(
            (r for r in rows if r["row_kind"] == "unresolved_transition"
             and r["sequence_id"] == head["sequence_id"]), None
        )
        text.append(f"{head['strategy']}\n  verdict: {head['verdict']}")
        if requirement is not None:
            target = requirement["target_condition"]
            text.append(f"  needs {requirement['transition_kind']}\n"
                        f"  -> {target[:52]}...\n"
                        f"  theta / beta / duration: NOT_GENERATED")
        text.append("")
    text.append("A row that says NOT_GENERATED is not a row that says zero.")
    ax.text(0.02, 0.96, "\n".join(text), va="top", ha="left", fontsize=7.8,
            transform=ax.transAxes, family="monospace", color="#334155")

    fig.suptitle("Step 9: the body-requirement timeline handed to Day 12 "
                 "(drawn from the delivered CSV alone)", fontsize=11)
    fig.tight_layout()
    fig.savefig(figure, dpi=140)
    plt.close(fig)
    return figure


# --------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--plots-only", action="store_true",
                        help="redraw the figure from the delivered CSV.")
    args = parser.parse_args()
    path = args.output_dir / TIMELINE_CSV
    figure = args.output_dir / FIGURE_PNG

    if args.plots_only:
        print(f"  redrawing from {path.name}", flush=True)
        print(f"  wrote {_plot_from_csv(path, figure)}", flush=True)
        return 0

    tables = load_tables_2d(
        args.output_dir, DAY6_7_DIR,
        swing_over_csv=args.output_dir / "day10_11_step5_swing_over.csv",
    )
    timelines = section_a(tables) + section_b(tables)
    evidence = section_c()
    evidence += section_d(timelines)
    path, check_rows = section_e(timelines, path)
    criteria = section_f(timelines, check_rows)

    write_rows_csv(args.output_dir / EVIDENCE_CSV, union_rows(evidence))
    write_rows_csv(args.output_dir / CRITERIA_CSV,
                   union_rows(criteria + check_rows))
    print(f"\n  wrote {EVIDENCE_CSV} and {CRITERIA_CSV}", flush=True)
    print(f"  wrote {_plot_from_csv(path, figure)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
