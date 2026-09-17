"""Day 12 appendix B driver: register the crossing, then draw what that shows.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_obstacle_registration.csv``   every crossing segment's implied obstacle
``day12_obstacle_registration.png``   the same, on one world-x axis
``day12_obstacle_viewing_height.csv`` the drawn height per sample
``day12_obstacle_frames.png``         one frame per crossing segment
``day12_obstacle_animation.gif``      the whole 4 cm run

The terrain is a **parameter**, exactly as Step 10 requires: nothing here is
written for 4 cm.  Pass ``--height-mm`` / ``--top-length-mm`` / ``--x-start-mm``
for any other rectangle.

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_obstacle_animation_driver.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (  # noqa: E402
    LEG_ORDER,
)
from hybrid_note.scripts.experiments.day12_obstacle_registration_2d import (  # noqa: E402
    plot_registration_2d,
    registration_report_2d,
    registration_rows,
)
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (  # noqa: E402
    plan_terrain_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (  # noqa: E402
    TransitionPhase,
)
from hybrid_note.scripts.experiments.day12_whole_body_animation_2d import (  # noqa: E402
    animate_whole_body_2d,
    planned_surfaces_2d,
    plot_frame_strip_2d,
    viewing_height_rows,
    viewing_heights_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (  # noqa: E402
    assemble_whole_body_2d,
)

NOTES = Path(__file__).resolve().parents[2] / "notes"
OUT = NOTES / "day12"

#: Step 7 left both blocked pairs unresolved (the Step 8 driver quotes it too).
REPOSITION_UNRESOLVED = 2


def crossing_frame_indices(whole, report) -> list[int]:
    """Four frames around the first leg's crossing.

    Evenly spaced frames mostly miss it: the whole crossing is 0.2 s out of
    3.0 s.  These are the two swings, the instant between them -- where the
    ascent's touchdown and the descent's lift-off meet -- and one frame after
    the leg is back down.
    """

    ascent = next((r for r in sorted(report.registrations, key=lambda r: r.start_s)
                   if r.phase is TransitionPhase.ASCENT), None)
    if ascent is None:
        return [0, len(whole.samples) // 2]
    descent = next((r for r in report.registrations
                    if r.leg is ascent.leg and r.start_s >= ascent.end_s - 1e-9),
                   None)
    wanted = [0.5 * (ascent.start_s + ascent.end_s), ascent.end_s]
    if descent is not None:
        wanted += [0.5 * (descent.start_s + descent.end_s),
                   descent.end_s + 0.25]
    times = [s.time_s for s in whole.samples]
    picked: list[int] = []
    for target in wanted:
        index = min(range(len(times)), key=lambda i: abs(times[i] - target))
        if index not in picked:
            picked.append(index)
    return picked


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--height-mm", type=float, default=40.0)
    parser.add_argument("--top-length-mm", type=float, default=400.0)
    parser.add_argument("--x-start-mm", type=float, default=1000.0)
    parser.add_argument("--samples", type=int, default=241)
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--no-animation", action="store_true")
    args = parser.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    started = time.time()

    terrain = SharedTerrainSpec2D(
        height_m=args.height_mm * 1e-3, top_length_m=args.top_length_mm * 1e-3,
        x_start_m=args.x_start_mm * 1e-3, obstacle_id="day12_platform")
    tables = load_tables_2d(NOTES / "day10-11", NOTES / "day6-7")
    run = plan_terrain_2d(terrain, tables, samples=args.samples,
                          reposition_unresolved=REPOSITION_UNRESOLVED)
    if not run.planned:
        print("this terrain produced no trajectory:")
        for failure in run.failures:
            print(f"  [{failure.stage.value}] {failure.detail}")
        return
    print(f"planned {run.planned}  feasible {run.feasible}   "
          f"({time.time() - started:.1f} s)")

    report = registration_report_2d(run.plan, run.body, terrain)
    write_rows_csv(OUT / "day12_obstacle_registration.csv",
                   registration_rows(report))
    plot_registration_2d(report, OUT / "day12_obstacle_registration.png")

    summary = report.as_dict()
    print(f"\nregistering the crossing against the platform it was planned for")
    print(f"  crossing segments             {summary['crossing_segments']}")
    print(f"  one obstacle fits them all    {summary['is_registrable']}"
          f"   (tolerance {summary['tolerance_mm']:.1f} mm)")
    print(f"  implied positions spread over {summary['implied_spread_mm']:.1f} mm")
    print(f"  worst leg disagreeing with itself "
          f"{summary['worst_within_leg_gap_mm']:.1f} mm  (its ascent vs its descent)")
    print(f"  worst drift within one segment {summary['worst_drift_mm']:.1f} mm")
    print(f"  worst body-advance ratio       "
          f"{summary['worst_advance_ratio']:.1f}x"
          f"   <- the segment's own frame moves the hip that much further "
          f"than the schedule does")
    print(f"  planned platform at            "
          f"{summary['planned_terrain_x_start_mm']:.0f} mm; nearest implied "
          f"obstacle is {summary['distance_to_planned_terrain_mm']:.0f} mm away")
    for registration in report.registrations:
        row = registration.as_dict()
        print(f"    {row['leg']:>2} {row['kind']:11s} implied "
              f"{row['implied_x_start_entry_mm']:8.1f} -> "
              f"{row['implied_x_start_exit_mm']:8.1f} mm   "
              f"needs {row['demanded_advance_mm']:6.1f}, gets "
              f"{row['delivered_advance_mm']:5.1f} mm")

    whole = assemble_whole_body_2d(
        run.plan, run.body, run.stability, samples=args.samples,
        reposition_unresolved=REPOSITION_UNRESOLVED, use_generator_frames=True)
    heights = viewing_heights_2d(run.plan, whole,
                                 nominal_body_z_m=run.body.nominal_body_z_m)
    surfaces = planned_surfaces_2d(run.plan, whole)
    write_rows_csv(OUT / "day12_obstacle_viewing_height.csv",
                   viewing_height_rows(heights))

    from_frames = sum(1 for s in whole.samples for leg in LEG_ORDER
                      if leg in s.legs and s.legs[leg].from_generator_frame)
    total = sum(len(s.legs) for s in whole.samples)
    raised = sum(1 for row in surfaces
                 for value in row.values() if value.raised_z_m is not None)
    print(f"\nthe drawing")
    print(f"  leg samples from generator frames {from_frames} of {total}"
          f"   <- the crossing segments have no frame registry, so those "
          f"interpolate")
    print(f"  leg samples whose segment touches the top  {raised} of {total}")
    print(f"  time any leg stands ON the obstacle        "
          f"{summary['stance_on_top_s']:.3f} s"
          f"   <- the ascent's touchdown and the descent's lift-off are the "
          f"same instant; the segment between them is Step 7's unresolved "
          f"TOP_REPOSITION")
    print(f"  worst foot left off its surface   "
          f"{max(h.worst_gap_m for h in heights) * 1e3:.3f} mm")

    footnote = (
        "no platform is drawn: the crossing does not register.\n"
        f"its {summary['crossing_segments']} segments imply obstacle positions "
        f"{summary['implied_spread_mm']:.0f} mm apart (appendix B chart).\n"
        "dashed purple = the surface a swing lands on or leaves."
    )
    indices = crossing_frame_indices(whole, report)
    plot_frame_strip_2d(
        whole, heights, OUT / "day12_obstacle_frames.png", indices=indices,
        surfaces=surfaces,
        subtitle=(
            f"Day 12 appendix B -- the {args.height_mm:.0f} mm crossing, drawn\n"
            "no platform: the crossing does not register (its segments imply "
            f"obstacles {summary['implied_spread_mm']:.0f} mm apart)\n"
            f"and no foot ever stands on it ({summary['stance_on_top_s']:.3f} s "
            "of stance on the top)"),
    )
    print(f"\nstrip frames at t = "
          f"{', '.join(f'{whole.samples[i].time_s:.3f}' for i in indices)} s"
          f"   -> day12_obstacle_frames.png")

    if not args.no_animation:
        frames = animate_whole_body_2d(
            whole, heights, OUT / "day12_obstacle_animation.gif",
            stride=args.stride, surfaces=surfaces, footnote=footnote)
        print(f"animation {frames} frames -> day12_obstacle_animation.gif")

    print(f"\nwrote -> {OUT}   ({time.time() - started:.1f} s total)")


if __name__ == "__main__":
    main()
