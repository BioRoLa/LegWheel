"""Day 12 problem A4 driver: scan the support margin over duty and sequence.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_a4_margin_scan.csv``   the stride measurement, the probe, every point
``day12_a4_margin_scan.png``   margin and motor cost against stance duty

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_a4_margin_scan_driver.py
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_composer_2d import (  # noqa: E402
    ComposedSequence2D,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    StrategyId,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (  # noqa: E402
    HIP_TO_BODY_Z_M,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (  # noqa: E402
    LEG_ORDER,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (  # noqa: E402
    NominalPosture2D,
    run_foot_rim_roll_2d,
)
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (  # noqa: E402
    CRITICAL_STANCE_DUTY,
    LIFTOFF_SEQUENCES,
    best_body_offset_2d,
    best_within_motor_budget_2d,
    frame_motor_rate_2d,
    liftoff_order_2d,
    phase_offsets_for_2d,
    rolling_stride_2d,
    scan_point_2d,
    scan_rows,
    scan_support_margin_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (  # noqa: E402
    GaitTiming2D,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (  # noqa: E402
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (  # noqa: E402
    DEFAULT_MARGIN_FLOOR_M,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (  # noqa: E402
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (  # noqa: E402
    build_leg_plan_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"

DUTIES = (0.750, 0.775, 0.800, 0.825, 0.850, 0.875, 0.900, 0.925, 0.950, 0.970)

#: Two grids, to show that the resampled motor rate is a property of the grid.
RESAMPLE_GRIDS = (241, 481, 961, 1921)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    # ---- 1. what the support polygon is given -------------------------------
    stride = rolling_stride_2d()
    row = stride.as_dict()
    print("1. what a rolling stance leaves for the support polygon")
    print(f"   hip advance        {row['hip_advance_mm']:9.3f} mm")
    print(f"   contact advance    {row['contact_advance_mm']:9.3f} mm")
    print(f"   relative stride    {row['relative_stride_mm']:9.3f} mm"
          f"   <- all the polygon responds to")
    print(f"   a planted foot would give {row['stride_loss_ratio']:.3f}x more")
    print(f"   rolling a shorter stroke scales it down proportionally: "
          f"{row['relative_stride_is_proportional_to_roll']}")
    for fraction, partial in zip(stride.partial_roll_fraction,
                                 stride.partial_relative_stride_m):
        print(f"     roll {fraction:4.2f} -> relative stride "
              f"{partial * 1e3:7.3f} mm  "
              f"({partial / stride.relative_stride_m:5.3f} of full)")

    # ---- 2. the gait the project already has --------------------------------
    library = walk_timing_2d()
    order = liftoff_order_2d(library)
    named = [name for name, seq in LIFTOFF_SEQUENCES.items() if seq == order]
    print(f"\n2. the gait GAIT_LIBRARY['Walk'] encodes")
    print(f"   stance duty        {library.stance_duty:.3f}"
          f"   (critical duty = {CRITICAL_STANCE_DUTY:.3f})")
    print(f"   liftoff order      {' '.join(l.value for l in order)}"
          f"   = {named[0] if named else 'not in LIFTOFF_SEQUENCES'}")
    print(f"   phase offsets      {phase_offsets_for_2d(order, library.stance_duty)}")

    # ---- 3. the plans, in the Day 13 configuration --------------------------
    fixed = NominalPosture2D()
    held = float(max(f.hip_xz_m[1] for f in run_foot_rim_roll_2d(fixed).frames))
    levelled = replace(fixed, hold_hip_z_m=held)
    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="flat run, no crossing")
    print(f"\n3. building the leg plans "
          f"(theta-compensated rolling, continuously chained) ...")
    plans = {leg: build_leg_plan_2d(leg, composed, posture=levelled,
                                    continuous_nominal=True)
             for leg in LEG_ORDER}
    body_z = held - HIP_TO_BODY_Z_M

    # ---- 4. the sequence axis ----------------------------------------------
    print(f"\n4. liftoff sequence, at the critical duty and just above it")
    sequence_points = scan_support_margin_2d(
        plans, nominal_body_z_m=body_z, duties=(0.75, 0.80),
        sequences=tuple(LIFTOFF_SEQUENCES))
    print(f"   {'sequence':>26} {'order':>14} {'d=0.750':>10} {'d=0.800':>10}")
    for name, seq in LIFTOFF_SEQUENCES.items():
        cells = []
        for duty in (0.75, 0.80):
            point = next(p for p in sequence_points
                         if p.sequence_name == name
                         and abs(p.stance_duty - duty) < 1e-9)
            cells.append("       n/a" if point.min_margin_m is None
                         else f"{point.min_margin_m * 1e3:10.4f}")
        mark = "  <- the project's own" if name == "project_walk" else ""
        print(f"   {name:>26} {' '.join(l.value for l in seq):>14} "
              + " ".join(cells) + mark)

    # ---- 5. the number the old measurement was giving ----------------------
    print(f"\n5. the motor cost, and the measurement that used to move")
    print(f"   The frame path used to snap to the nearest generator frame, so "
          f"the sampled")
    print(f"   signal was a staircase and its finite difference read 48.1% at "
          f"241 samples")
    print(f"   and 127.7% at 1921 -- the discretisation, not the gait.  The "
          f"frames are now")
    print(f"   interpolated, so the two columns below must agree at every "
          f"density:")
    print(f"   {'samples':>9} {'resampled deg/s':>17} {'util':>7}   "
          f"{'frame-to-frame deg/s':>21} {'util':>7}")
    for grid in RESAMPLE_GRIDS:
        point = scan_point_2d(plans, sequence_name="project_walk",
                              stance_duty=0.75, nominal_body_z_m=body_z,
                              resample_at=grid)
        data = point.as_dict()
        print(f"   {grid:9d} {data['resampled_motor_rate_deg_s']:17.2f} "
              f"{data['resampled_motor_rate_deg_s'] / data['motor_limit_deg_s']:7.3f}"
              f"   {data['peak_motor_rate_deg_s']:21.2f} "
              f"{data['motor_utilisation']:7.3f}")
    print(f"   -- if the two columns ever separate again, the staircase is "
          f"back (trap 61).")

    # ---- 6. the duty axis, priced ------------------------------------------
    print(f"\n6. stance duty, with what it costs the motors")
    points = scan_support_margin_2d(
        plans, nominal_body_z_m=body_z, duties=DUTIES,
        sequences=("project_walk",))
    print(f"   {'duty':>6} {'swing_s':>8} {'margin mm':>11} "
          f"{'motor deg/s':>12} {'util':>7} {'min period s':>13} "
          f"{'speed':>7}  budget")
    for point in points:
        data = point.as_dict()
        margin = data["min_margin_mm"]
        speed = point.cycle_period_s / data["min_cycle_period_s"]
        print(f"   {data['stance_duty']:6.3f} {data['swing_window_s']:8.3f} "
              f"{'        n/a' if margin is None else f'{margin:11.4f}'} "
              f"{data['peak_motor_rate_deg_s']:12.2f} "
              f"{data['motor_utilisation']:7.3f} "
              f"{data['min_cycle_period_s']:13.3f} {speed:7.3f}  "
              f"{'ok' if data['motor_is_within_budget'] else 'slower'}")
    print(f"   -- 'min period' is the fastest cycle the motors allow at that "
          f"duty; 'speed' is that")
    print(f"      against the {points[0].cycle_period_s:.1f} s nominal.  A duty "
          f"over budget is not refused, it is slower.")

    best = best_within_motor_budget_2d(points)
    floor_mm = DEFAULT_MARGIN_FLOOR_M * 1e3
    print(f"\n   the most margin the motors can pay for at full speed:")
    if best is None:
        print("     none -- every duty is either unknown or over budget")
    else:
        data = best.as_dict()
        print(f"     duty {data['stance_duty']:.3f}"
              f"   margin {data['min_margin_mm']:.4f} mm"
              f"   motors {data['motor_utilisation'] * 100:.1f}%")
    clears = [p for p in points if p.clears(DEFAULT_MARGIN_FLOOR_M)]
    reachable = [p for p in points
                 if p.min_margin_m is not None
                 and p.min_margin_m > DEFAULT_MARGIN_FLOOR_M]
    print(f"   the planning floor asks for {floor_mm:.1f} mm.")
    print(f"     duties clearing it at full speed:  "
          f"{[f'{p.stance_duty:.3f}' for p in clears] or 'NONE'}")
    print(f"     duties clearing it at ANY speed:   "
          f"{[f'{p.stance_duty:.3f}' for p in reachable] or 'NONE'}")
    if not reachable:
        top = max((p for p in points if p.min_margin_m is not None),
                  key=lambda p: p.min_margin_m)
        print(f"     -- the margin is still climbing at duty "
              f"{top.stance_duty:.3f} ({top.min_margin_m * 1e3:.4f} mm) and "
              f"the swing has")
        print(f"        already shrunk to {top.swing_window_s * 1e3:.0f} ms.  "
              f"The floor is above what this geometry")
        print(f"        produces, so it is the floor that has to be answered "
              f"for, not the gait.")

    # ---- 7. could a body shift do it instead? -------------------------------
    print(f"\n7. and the offset nobody can install: fore-aft body shift")
    probe = best_body_offset_2d(plans, walk_timing_2d(), nominal_body_z_m=body_z)
    data = probe.as_dict()
    print(f"   nominal margin              {data['nominal_margin_mm']:9.4f} mm")
    print(f"   best margin any shift gives {data['achievable_margin_mm']:9.4f} mm")
    print(f"   the shift it needs          "
          f"{data['required_offset_min_mm']:+.1f} .. "
          f"{data['required_offset_max_mm']:+.1f} mm")
    print(f"   the sign reverses within the cycle: {data['offset_reverses_sign']}")
    print(f"   -- so no constant CoM correction captures any of it, and in a")
    print(f"      rolling stance the hip's x is a consequence of beta.")

    # ---- 8. write ------------------------------------------------------------
    nominal_timing = GaitTiming2D(
        cycle_period_s=2.4, stance_duty=0.85,
        phase_offsets=phase_offsets_for_2d(LIFTOFF_SEQUENCES["project_walk"],
                                           0.85),
        gait_name="project_walk@duty0.85")
    demands = frame_motor_rate_2d(plan_four_legs_2d(plans, nominal_timing))
    rows = scan_rows(tuple(sequence_points) + tuple(points), stride, probe,
                     demands)
    write_rows_csv(OUT / "day12_a4_margin_scan.csv", rows)

    fig, (left, right) = plt.subplots(1, 2, figsize=(12.0, 4.6))
    duties = [p.stance_duty for p in points]
    margins = [np.nan if p.min_margin_m is None else p.min_margin_m * 1e3
               for p in points]
    utilisation = [np.nan if p.motor_utilisation is None else p.motor_utilisation
                   for p in points]
    left.plot(duties, margins, marker="o", color="#2a6f4e", lw=1.8)
    left.axhline(floor_mm, color="#c5221f", ls="--", lw=1.0,
                 label=f"planning floor {floor_mm:.0f} mm")
    left.axhline(0.0, color="#333333", lw=1.0)
    left.axvline(CRITICAL_STANCE_DUTY, color="#b06000", ls=":", lw=1.2,
                 label=f"critical duty {CRITICAL_STANCE_DUTY:g}")
    left.set_xlabel("stance duty"); left.set_ylabel("min support margin [mm]")
    left.set_title("the margin is zero at the critical duty, by construction",
                   fontsize=9, loc="left")
    left.legend(fontsize=8, frameon=False); left.grid(alpha=0.25)

    right.plot(duties, [u * 100.0 for u in utilisation], marker="o",
               color="#1f4e9c", lw=1.8)
    right.axhline(100.0, color="#c5221f", ls="--", lw=1.0, label="330 rpm")
    right.set_xlabel("stance duty"); right.set_ylabel("peak motor use [%]")
    right.set_title("and what buying margin costs the motors",
                    fontsize=9, loc="left")
    right.legend(fontsize=8, frameon=False); right.grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(OUT / "day12_a4_margin_scan.png", dpi=150)
    plt.close(fig)

    print(f"\nwrote -> {OUT / 'day12_a4_margin_scan.csv'}")
    print(f"wrote -> {OUT / 'day12_a4_margin_scan.png'}")


if __name__ == "__main__":
    main()
