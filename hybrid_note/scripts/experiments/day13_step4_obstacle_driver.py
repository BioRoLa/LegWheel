"""Day 13: export an obstacle crossing as a controller CSV, for simulation.

**This trajectory does not pass Step 9.**  It is exported anyway, deliberately
and with the failures written into its own summary, because several of those
failures are properties of the *model* rather than of the robot and a simulator
is the cheaper way to find out which:

``support_margin``
    a quasi-static geometric criterion whose 3 mm floor is derived from a
    **guessed** +-2 mm centre-of-mass uncertainty (project owner, 2026-09-06).
    The model carries no mass, inertia, contact force, friction or dynamics, so
    it cannot say whether the robot tips -- only that it is outside a margin
    built on a guess.  ``--margin-floor-mm`` relaxes it.

``body_requirement_satisfied``
    the model pins ``body_rpy = (0, 0, 0)``: the body may translate but never
    tilt.  Two legs asking for different body heights is therefore recorded as
    a conflict, when on a rigid frame that is simply a slope -- four hips
    define a plane, not one height.  See ``day13_b3b_body_tilt.csv``.

``at_most_one_airborne`` / ``three_support_legs``
    real, and Day 13 measured that they cannot be fixed in the scheduling layer
    (log 25): under position scheduling a leg's time *is* its claim about the
    body's position, so retiming two legs makes them disagree about the body.

``motor_rate_limit``
    the one hardware fact in the list.  ``--period`` buys it directly: the rate
    scales as 1/period, and the owner has no speed requirement.

Run ``--list-only`` to see the failures for a size without writing anything.
"""

from __future__ import annotations

import argparse
import collections
import sys
from dataclasses import replace
from pathlib import Path

# Same bootstrap as the other drivers: they are run as scripts, so the package
# root has to be on the path before any hybrid_note import.
_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np  # noqa: E402

from hybrid_note.scripts.experiments.day10_11_decision_map_2d import load_tables_2d
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import RecoveryConfig2D
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (
    hybrid_posture_2d,
    hybrid_timing_2d,
)
from hybrid_note.scripts.experiments.day12_world_registration_2d import (
    SpeedZone2D,
    body_speed_m_s,
    swing_hip_advance_m,
)
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (
    plan_terrain_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (
    MOTOR_MAX_RATE_RAD_S,
    motor_rates_rad_s,
)
from hybrid_note.scripts.experiments.day13_hardware_export_2d import (
    NotExportable,
    hardware_command_2d,
    write_hardware_csv_2d,
)

HERE = Path(__file__).resolve()
NOTES = HERE.parents[2] / "notes"
OUT = NOTES / "day13"


def peak_motor_rate_deg_s(rows: np.ndarray, dt_s: float) -> float:
    """The **per-motor** peak, which is what the 330 rpm rating limits.

    Not ``|dtheta| + |dbeta|``: this leg has two motors and the rating is each
    motor's, so the quantity to bound is ``max(|phi_r_dot|, |phi_l_dot|)`` with
    ``phi_r = theta + beta`` and ``phi_l = beta - theta`` (project owner,
    2026-09-06).  The sum overstates it whenever the two joints move oppositely
    and understates nothing, so reporting the sum was reporting the wrong
    number.
    """

    theta = rows[:, [0, 2, 4, 6]]
    beta = rows[:, [1, 3, 5, 7]]
    d_theta = np.diff(theta, axis=0) / dt_s
    d_beta = np.diff(beta, axis=0) / dt_s
    phi_r = np.abs(d_theta + d_beta)
    phi_l = np.abs(d_beta - d_theta)
    return float(np.rad2deg(max(phi_r.max(), phi_l.max())))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--height-mm", type=float, default=40.0)
    ap.add_argument("--top-length-mm", type=float, default=400.0,
                    help="along travel; the 2D sagittal model has no width")
    ap.add_argument("--x-start-m", type=float, default=1.0,
                    help="where the obstacle's leading face is, in world x")
    ap.add_argument("--period", type=float, default=2.4,
                    help="cycle period, which sets the BASE body speed.  The "
                         "motor rate scales as 1/period, so this is the direct "
                         "knob on it.  2.4 s is the flat-ground default.")
    ap.add_argument("--crossing-slowdown", type=float, default=1.0,
                    help="divide the body speed by this while crossing.  1.0 "
                         "keeps one speed throughout (the Day 12 behaviour); "
                         "4.0 crosses at a quarter speed while the approach "
                         "still runs at the flat-ground pace.  Creeping the "
                         "whole way is both slow and, on a real machine, no "
                         "safer than walking the approach normally.")
    ap.add_argument("--slow-margin-m", type=float, default=0.6,
                    help="how far before the obstacle the slow zone starts, "
                         "and how far past its trailing edge it ends")
    ap.add_argument("--margin-floor-mm", type=float, default=3.0)
    ap.add_argument("--speed-mm-s", type=float, default=None,
                    help="body speed on the approach.  The period is SOLVED "
                         "from it, because speed = cycle hip advance / period "
                         "and the advance does not depend on the period.  "
                         "Overrides --period.")
    ap.add_argument("--arc-reserve", type=float, default=0.0,
                    help="fraction of the foot-rim arc left unspent per "
                         "stroke, 0.0-1.0.  0 rolls to RIM_ARC_EXHAUSTED, "
                         "which is Day 12's behaviour and leaves no room to "
                         "adjust a landing point (log 26.8).")
    ap.add_argument("--samples", type=int, default=121,
                    help="samples over the WHOLE trajectory -- a count, not a "
                         "rate.  Day 13 log 11.4: the measured motor rate is "
                         "grid-dependent, so this changes the reported peak "
                         "and must be stated with it.  121 is the default the "
                         "frozen numbers were measured at.")
    ap.add_argument("--forward", dest="reverse", action="store_false",
                    default=True,
                    help="emit the planner's own beta sign.  The default is "
                         "REVERSED, because the 2D planner has one rolling "
                         "direction and which sign drives the real robot "
                         "forward is a hardware fact the model does not hold: "
                         "the un-reversed export was driven and went backwards "
                         "(project owner, 2026-09-06).")
    ap.add_argument("--list-only", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    terrain = SharedTerrainSpec2D(height_m=args.height_mm / 1e3,
                                  top_length_m=args.top_length_mm / 1e3,
                                  x_start_m=args.x_start_m)
    tables = load_tables_2d(NOTES / "day10-11", NOTES / "day6-7")

    posture = hybrid_posture_2d()
    if args.arc_reserve > 0.0:
        posture = replace(posture, arc_reserve=args.arc_reserve)

    period = args.period
    if args.speed_mm_s is not None:
        # Solve the period from the requested speed rather than writing one in:
        # the cycle's hip advance does not depend on the period, so this is a
        # single division, and pinning the speed keeps it pinned when theta,
        # duty or the arc reserve change the advance.
        probe = hybrid_timing_2d(cycle_period_s=args.period)
        advance = (body_speed_m_s(probe, posture,
                                  RecoveryConfig2D(hip_advance_m=
                                      swing_hip_advance_m(probe, posture)))
                   * probe.cycle_period_s)
        period = advance / (args.speed_mm_s / 1e3)
        print(f"  speed {args.speed_mm_s} mm/s -> period {period:.4f} s")
    timing = hybrid_timing_2d(cycle_period_s=period)

    print(f"planning {args.height_mm:.0f} mm x {args.top_length_mm:.0f} mm "
          f"at x = {args.x_start_m} m, period {args.period} s ...")
    zones: tuple[SpeedZone2D, ...] = ()
    if args.crossing_slowdown != 1.0:
        base = body_speed_m_s(timing, posture,
                              RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(
                                  timing, posture)))
        zones = (SpeedZone2D(
            x_start_m=terrain.x_start_m - args.slow_margin_m,
            x_end_m=terrain.x_start_m + terrain.top_length_m + args.slow_margin_m,
            speed_m_s=base / args.crossing_slowdown),)
        print(f"  base speed    {base * 1e3:.3f} mm/s")
        print(f"  slow zone     x in [{zones[0].x_start_m:.3f}, "
              f"{zones[0].x_end_m:.3f}] at {zones[0].speed_m_s * 1e3:.3f} mm/s")

    run = plan_terrain_2d(terrain, tables, timing=timing, posture=posture,
                          margin_floor_m=args.margin_floor_mm / 1e3,
                          samples=args.samples, speed_zones=zones)

    strategy = getattr(run.composed.strategy, "value", None) if run.composed else None
    print(f"  strategy      {strategy}")
    print(f"  planned       {run.planned}")
    if not run.planned:
        # A size with no strategy has no trajectory to export.  Say so with the
        # reason rather than writing an empty file.
        print("  NOT PLANNED -- nothing to export:")
        for f in run.failures:
            print(f"     {f.stage.value}: {f.detail[:100]}")
        return

    failed = collections.Counter(f.check.value for f in run.report.failures)
    print(f"  Step 9        {len(run.report.failures)} failures  {dict(failed)}")
    margins = [f.value for f in run.report.failures
               if f.check.value == "support_margin" and f.value is not None]
    if margins:
        print(f"  worst margin  {min(margins) * 1e3:.3f} mm")
    if args.list_only:
        return

    try:
        command = hardware_command_2d(run.trajectory, reverse=args.reverse)
    except NotExportable as exc:
        print(f"  NOT EXPORTABLE: {exc}")
        return

    name = args.out or f"hybrid_obstacle_{args.height_mm:.0f}mm_SIM"
    csv_path, phase_path = write_hardware_csv_2d(command, OUT / f"{name}.csv")

    peak = peak_motor_rate_deg_s(command.rows, command.controller_dt_s)
    limit = float(np.rad2deg(MOTOR_MAX_RATE_RAD_S))
    print(f"  peak per-motor {peak:.2f} deg/s  ({peak / limit * 100:.1f}% of limit)")

    rows = [{
        "row_kind": "command",
        **command.as_dict(),
        "SIMULATION_ONLY": "yes -- this trajectory does NOT pass Step 9",
        "height_mm": args.height_mm,
        "top_length_mm": args.top_length_mm,
        "x_start_m": args.x_start_m,
        "cycle_period_s": period,
        "requested_speed_mm_s": args.speed_mm_s or "",
        "arc_reserve": args.arc_reserve,
        "crossing_slowdown": args.crossing_slowdown,
        "slow_zone_margin_m": args.slow_margin_m,
        "reverse": bool(args.reverse),
        "margin_floor_mm": args.margin_floor_mm,
        "trajectory_samples": args.samples,
        "strategy": strategy,
        "step9_failures": len(run.report.failures),
        "step9_failed_checks": ";".join(f"{k}x{v}" for k, v in sorted(failed.items())),
        "worst_support_margin_mm": (min(margins) * 1e3) if margins else "",
        "peak_per_motor_deg_s": peak,
        "peak_percent_of_limit": peak / limit * 100.0,
        "note_support_margin": ("floor derived from a GUESSED +-2 mm CoM "
                                "uncertainty; model has no mass or dynamics"),
        "note_body_requirement": ("model pins body_rpy=(0,0,0); a rigid body "
                                  "may tilt, so a height disagreement is a "
                                  "slope, not necessarily a conflict"),
        "note_airborne": ("real; Day 13 log 25 shows it cannot be fixed in the "
                          "scheduling layer"),
    }]
    write_rows_csv(OUT / f"{name}_summary.csv", rows)

    print(f"\nwrote -> {csv_path}")
    print(f"wrote -> {phase_path}")
    print(f"wrote -> {OUT / f'{name}_summary.csv'}")


if __name__ == "__main__":
    main()
