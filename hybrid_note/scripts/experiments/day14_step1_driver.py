"""Day 14 Step 1 driver: the gait-first planner on flat ground.

Two things, both measured rather than asserted:

1. against the frozen Day 12 flat run (``plan_terrain_2d(None)``): the same
   trajectory, sample for sample;
2. the long flat run that ``hybrid_flat_v1`` was built from (9 cycles, 200 Hz
   planner grid), exported through the same ``hardware_command_2d`` so the
   peak motor rate can be read against the frozen 95.0%.

Writes into ``hybrid_note/notes/day14/``::

    day14_step1_flat_comparison.csv    per-sample worst differences
    day14_step1_flat_events.csv        every swing on the clock
    day14_step1_flat_hardware.csv      (with --hardware) the 1 kHz CSV + phase

Run from ``LegWheel/``::

    python3 -u hybrid_note/scripts/experiments/day14_step1_driver.py [--hardware]
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

import numpy as np  # noqa: E402

from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER  # noqa: E402
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (  # noqa: E402
    plan_terrain_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (  # noqa: E402
    MOTOR_MAX_RATE_RAD_S,
)
from hybrid_note.scripts.experiments.day13_hardware_export_2d import (  # noqa: E402
    hardware_command_2d,
    write_hardware_csv_2d,
)
from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import (  # noqa: E402
    plan_flat_gait_first_2d,
)

NOTES = Path(__file__).resolve().parents[2] / "notes"
OUT = NOTES / "day14"


def peak_motor_rate_deg_s(rows: np.ndarray, dt_s: float) -> float:
    """Worst of the two motors over the whole file, per leg, in deg/s."""

    theta = rows[:, [0, 2, 4, 6]]
    beta = rows[:, [1, 3, 5, 7]]
    dtheta = np.diff(theta, axis=0) / dt_s
    dbeta = np.diff(beta, axis=0) / dt_s
    right = np.abs(dtheta + dbeta)
    left = np.abs(dbeta - dtheta)
    return float(np.rad2deg(max(right.max(), left.max())))


def padded(rows: list[dict]) -> list[dict]:
    """One key set for every row, so mixed row kinds share a CSV."""

    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]


def compare(run, frozen) -> dict:
    ours, theirs = run.trajectory.samples, frozen.trajectory.samples
    worst = {"dt_s": 0.0, "dq_rad": 0.0, "dz_m": 0.0, "dx_const_m": 0.0,
             "mode_mismatches": 0}
    offset = ours[0].body_position_world_m[0] - theirs[0].body_position_world_m[0]
    for a, b in zip(ours, theirs):
        worst["dt_s"] = max(worst["dt_s"], abs(a.time_s - b.time_s))
        worst["dz_m"] = max(worst["dz_m"], abs(a.body_position_world_m[2]
                                               - b.body_position_world_m[2]))
        worst["dx_const_m"] = max(worst["dx_const_m"], abs(
            a.body_position_world_m[0] - b.body_position_world_m[0] - offset))
        for leg in LEG_ORDER:
            la, lb = a.legs[leg], b.legs[leg]
            worst["dq_rad"] = max(worst["dq_rad"], abs(la.theta_rad - lb.theta_rad),
                                  abs(la.beta_rad - lb.beta_rad))
            worst["mode_mismatches"] += int(la.mode != lb.mode)
    return {
        "samples": len(ours), "frozen_samples": len(theirs),
        "body_x_offset_mm": offset * 1e3,
        "worst_time_diff_s": worst["dt_s"],
        "worst_joint_diff_deg": float(np.rad2deg(worst["dq_rad"])),
        "worst_body_z_diff_mm": worst["dz_m"] * 1e3,
        "worst_body_x_drift_mm": worst["dx_const_m"] * 1e3,
        "mode_mismatches": worst["mode_mismatches"],
        "ours_margin_mm": run.stability.minimum_margin_m * 1e3,
        "frozen_margin_mm": frozen.stability.minimum_margin_m * 1e3,
        "ours_failed": ";".join(c.value for c in run.report.failed_checks()),
        "frozen_failed": ";".join(c.value for c in frozen.report.failed_checks()),
        "ours_world_x_spread_mm": (None if run.body.world_x_spread_m is None
                                   else run.body.world_x_spread_m * 1e3),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--samples", type=int, default=121)
    ap.add_argument("--hardware", action="store_true",
                    help="also build the 9-cycle run at 200 Hz and export it")
    ap.add_argument("--metres", type=float, default=3.0)
    ap.add_argument("--planner-hz", type=float, default=200.0)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    t0 = time.perf_counter()
    run, plan = plan_flat_gait_first_2d(cycles=2, samples=args.samples)
    print(f"[{time.perf_counter() - t0:6.1f}s] gait-first flat: "
          f"feasible={run.feasible}  failed={[c.value for c in run.report.failed_checks()]}")
    print(f"           {plan.as_dict()}")
    for event in plan.swings:
        print(f"           swing {event.leg.value:2s} {event.kind:16s} "
              f"{event.start_s:8.4f} .. {event.end_s:8.4f} s  "
              f"body x {event.body_x_start_m * 1e3:8.2f} .. {event.body_x_end_m * 1e3:8.2f} mm"
              f"  min {event.minimum_duration_s:.4f} s")
    write_rows_csv(OUT / "day14_step1_flat_events.csv", padded(
        [{"row_kind": "swing", **e.as_dict()} for e in plan.swings]
        + [{"row_kind": "plan", **plan.as_dict()}]))

    tables = load_tables_2d(NOTES / "day10-11", NOTES / "day6-7")
    frozen = plan_terrain_2d(None, tables, samples=args.samples)
    row = compare(run, frozen)
    print(f"[{time.perf_counter() - t0:6.1f}s] against the frozen flat run:")
    for key, value in row.items():
        print(f"           {key:26s} {value}")
    write_rows_csv(OUT / "day14_step1_flat_comparison.csv",
                   [{"row_kind": "comparison", **row}])

    if args.hardware:
        # The same build ``day13_step3_hardware_driver`` does: cycles from the
        # distance asked for, 200 Hz planner grid, then the Walk-contract CSV.
        speed = plan.clock.speed_m_s
        period = plan.clock.timing.cycle_period_s
        wanted_s = args.metres / speed
        cycles = max(2, int(np.ceil((wanted_s - 0.6) / period)) + 1)
        span_guess = (cycles - 1) * period + 0.6
        samples = int(round(span_guess * args.planner_hz)) + 1
        print(f"[{time.perf_counter() - t0:6.1f}s] building {cycles} cycles at "
              f"{args.planner_hz:g} Hz ({samples} samples) ...")
        long_run, long_plan = plan_flat_gait_first_2d(cycles=cycles, samples=samples)
        lo, hi = long_plan.schedule.covered_interval_s
        print(f"           covered {lo:.4f} .. {hi:.4f} s  "
              f"distance {(hi - lo) * speed:.4f} m  "
              f"failed={[c.value for c in long_run.report.failed_checks()]}")
        command = hardware_command_2d(long_run.trajectory, reverse=True)
        csv_path, phase_path = write_hardware_csv_2d(
            command, OUT / "day14_step1_flat_hardware.csv")
        peak = peak_motor_rate_deg_s(command.rows, command.controller_dt_s)
        limit = float(np.rad2deg(MOTOR_MAX_RATE_RAD_S))
        print(f"           rows {len(command.rows)}  peak per-motor "
              f"{peak:.2f} deg/s ({peak / limit * 100:.1f}% of limit)  "
              f"frozen hybrid_flat_v1: 1881.61 deg/s (95.0%)")
        write_rows_csv(OUT / "day14_step1_flat_hardware_summary.csv", [{
            "row_kind": "command", **command.as_dict(), "cycles": cycles,
            "planner_hz": args.planner_hz, "body_speed_mm_s": speed * 1e3,
            "peak_per_motor_deg_s": peak, "peak_percent_of_limit": peak / limit * 100,
            "reverse": True,
            "step9_failed_checks": ";".join(
                c.value for c in long_run.report.failed_checks()),
        }])
        print(f"wrote -> {csv_path}\nwrote -> {phase_path}")
    print(f"[{time.perf_counter() - t0:6.1f}s] done")


if __name__ == "__main__":
    main()
