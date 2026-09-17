"""Day 13 Step 3 driver: the Hybrid flat gait as a Walk-compatible CSV.

Writes, into ``hybrid_note/notes/day13/``::

    day13_hybrid_flat_hardware.csv         12 columns, no header, 1 kHz
    day13_hybrid_flat_hardware_phase.csv   row-aligned stance(0)/swing(1)
    day13_hybrid_flat_hardware_summary.csv what it is and what it rests on

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day13_step3_hardware_driver.py

``--metres`` says how far the robot should walk; the cycle count follows from
the gait's own body speed rather than being typed in.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np  # noqa: E402

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
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (  # noqa: E402
    LEG_ORDER,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (  # noqa: E402
    NominalPosture2D,
    RecoveryConfig2D,
    run_foot_rim_roll_2d,
)
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (  # noqa: E402
    HYBRID_MARGIN_FLOOR_M,
    LIFTOFF_SEQUENCES,
    hybrid_timing_2d,
    phase_offsets_for_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (  # noqa: E402
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (  # noqa: E402
    build_leg_plan_2d,
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (  # noqa: E402
    assemble_whole_body_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (  # noqa: E402
    validate_whole_body_2d,
)
from hybrid_note.scripts.experiments.day12_world_registration_2d import (  # noqa: E402
    body_speed_m_s,
    swing_hip_advance_m,
)
from hybrid_note.scripts.experiments.day13_hardware_export_2d import (  # noqa: E402
    hardware_command_2d,
    write_hardware_csv_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day13"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--metres", type=float, default=3.0,
                        help="how far the robot should travel (default 3.0)")
    parser.add_argument("--planner-hz", type=float, default=200.0,
                        help="planner grid before the 1 kHz resample.  The peak "
                             "motor rate only converges once this is fine "
                             "enough -- 50 Hz reports 95.4%%, 200 Hz 95.0%% "
                             "(log 11.4).  Coarser is faster to build and "
                             "under-reports the rate.")
    parser.add_argument("--period", type=float, default=2.4,
                        help="gait cycle period in seconds.  Body speed is "
                             "cycle hip advance / period, so this is the only "
                             "direct speed knob -- and it divides into the "
                             "motor rate, which is already at 95%% (default 2.4)")
    parser.add_argument("--speed-mm-s", type=float, default=None,
                        help="ask for a body speed instead of a period.  The "
                             "period is then *solved* for, not typed in: speed "
                             "is cycle hip advance / period and the advance "
                             "does not depend on the period, so the solve is "
                             "one division.  Use this to hold a comparison's "
                             "speed baseline -- typing the period in as a "
                             "literal silently stops holding it the moment "
                             "--theta-deg, --duty or --roll-step-mm changes "
                             "the advance.  Overrides --period.")
    parser.add_argument("--duty", type=float, default=None,
                        help="stance duty.  Default is the Hybrid's 0.85.  "
                             "0.75 is the CRITICAL duty where the longitudinal "
                             "support margin is identically zero -- not a "
                             "setting, a singularity (log 1.6)")
    parser.add_argument("--theta-deg", type=float, default=None,
                        help="nominal posture theta in degrees (default 60).  "
                             "Changes the rolling stroke length, so it changes "
                             "the body speed too")
    parser.add_argument("--hold-hip-z-mm", type=float, default=None,
                        help="held hip height in mm.  Default is the top of "
                             "the rolling arc, which is what makes the four "
                             "stance legs agree about the body height")
    parser.add_argument("--roll-step-mm", type=float, default=None,
                        help="contact advance per generated roll step "
                             "(default 4.0).  Build time scales with this")
    parser.add_argument("--reverse", action="store_true",
                        help="negate beta on every leg, so the robot drives "
                             "the other way.  The 2D planner has exactly one "
                             "rolling direction (beta decreases while rolling "
                             "'forward'), so which sign goes forward on the "
                             "real robot is not something the model knows -- "
                             "it was settled by driving it.")
    parser.add_argument("--out", type=str, default="day13_hybrid_flat_hardware",
                        help="output basename inside notes/day13/")
    args = parser.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)

    # The posture.  Day 13 fixed it by holding the hip at the top of the
    # rolling arc, so the four stance legs agree about the body height; every
    # override below keeps that shape and only moves a number.
    fixed = NominalPosture2D()
    if args.theta_deg is not None:
        fixed = replace(fixed, theta_rad=float(np.deg2rad(args.theta_deg)))
    if args.roll_step_mm is not None:
        fixed = replace(fixed, roll_step_m=float(args.roll_step_mm) / 1e3)
    if args.hold_hip_z_mm is None:
        held = float(max(f.hip_xz_m[1]
                         for f in run_foot_rim_roll_2d(fixed).frames))
    else:
        held = float(args.hold_hip_z_mm) / 1e3
    posture = replace(fixed, hold_hip_z_m=held)

    timing = hybrid_timing_2d(cycle_period_s=float(args.period))
    if args.duty is not None:
        timing = replace(timing, stance_duty=float(args.duty),
                         phase_offsets=phase_offsets_for_2d(
                             LIFTOFF_SEQUENCES["project_walk"],
                             float(args.duty)),
                         gait_name=f"Walk@duty{float(args.duty):g}")
    config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))

    # --speed-mm-s: solve the period rather than trusting a typed-in literal.
    # ``body_speed_m_s`` is (cycle hip advance) / period and the advance is a
    # property of the posture and duty alone, so one probe at the current
    # period gives the advance and the solve is exact, not iterative.  Note
    # the swing advance rides on ``timing.stance_duty``, which the solve does
    # not touch, so ``config`` stays valid across it.
    if args.speed_mm_s is not None:
        wanted = float(args.speed_mm_s) / 1e3
        if wanted <= 0.0:
            parser.error("--speed-mm-s must be positive")
        advance_m = body_speed_m_s(timing, posture, config) * timing.cycle_period_s
        timing = replace(timing, cycle_period_s=advance_m / wanted)
        print(f"speed solve       asked {args.speed_mm_s:.3f} mm/s"
              f"   cycle advance {advance_m * 1e3:.4f} mm"
              f"   -> period {timing.cycle_period_s:.6f} s")

    print(f"posture           theta {np.rad2deg(posture.theta_rad):.3f} deg"
          f"   hold hip_z {held * 1e3:.4f} mm"
          f"   roll step {posture.roll_step_m * 1e3:.3f} mm")
    print(f"gait              duty {timing.stance_duty:.3f}"
          f"   period {timing.cycle_period_s:.3f} s")

    speed = body_speed_m_s(timing, posture, config)
    period = float(timing.cycle_period_s)

    # How many cycles for the distance asked for.  A run of N cycles covers
    # (N-1) periods plus one swing window -- measured below, not trusted here.
    wanted_s = float(args.metres) / speed
    cycles = max(2, int(np.ceil((wanted_s - 0.6) / period)) + 1)
    print(f"body speed        {speed * 1e3:9.3f} mm/s")
    print(f"asked for         {args.metres:9.3f} m  -> {wanted_s:.3f} s")
    print(f"cycles to build   {cycles}")

    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="flat run, no crossing")
    before = cycles // 2
    plans = {leg: build_leg_plan_2d(leg, composed, posture=posture,
                                    config=config, continuous_nominal=True,
                                    cycles_before=before,
                                    cycles_after=cycles - before)
             for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, timing)
    lo, hi = four.schedule.covered_interval_s
    span = hi - lo
    print(f"covered interval  {lo:9.4f} .. {hi:.4f} s   span {span:.4f} s")
    print(f"distance covered  {span * speed:9.4f} m")
    if span * speed < args.metres:
        print(f"  NOT ENOUGH -- asked for {args.metres} m; rebuild with more cycles")

    samples = int(round(span * float(args.planner_hz))) + 1
    body = body_trajectory_2d(four, nominal_body_z_m=held - HIP_TO_BODY_Z_M,
                              samples=samples)
    stability = swing_stability_2d(four, body,
                                   margin_floor_m=HYBRID_MARGIN_FLOOR_M)
    trajectory = assemble_whole_body_2d(four, body, stability, samples=samples,
                                        use_generator_frames=True)
    report = validate_whole_body_2d(trajectory, body, stability)
    failed = [c.value for c in report.failed_checks()]
    print(f"planner samples   {samples}  ({args.planner_hz:g} Hz)")
    print(f"Step 9            {len(failed)} failed check(s)  {failed}")

    command = hardware_command_2d(trajectory, reverse=args.reverse)
    csv_path, phase_path = write_hardware_csv_2d(
        command, OUT / f"{args.out}.csv")

    print()
    print("the CSV, in the Walk pipeline's contract")
    for key, value in command.as_dict().items():
        print(f"  {key:24s} {value}")
    print(f"  columns 0-7              (theta, beta) per leg, project index"
          f" 0 FL / 1 FR / 2 RR / 3 RL")
    print(f"  columns 8-11             gamma, fixed at 0 (Day 12 fixes it there)")
    print(f"  travel over trajectory   "
          f"{command.trajectory_duration_s * speed:.4f} m")

    theta = command.rows[:, [0, 2, 4, 6]]
    beta = command.rows[:, [1, 3, 5, 7]]
    rate = np.abs(np.diff(theta, axis=0)) + np.abs(np.diff(beta, axis=0))
    peak = float(np.rad2deg(rate.max() / command.controller_dt_s))
    print(f"  peak |dtheta|+|dbeta|    {peak:.2f} deg/s"
          f"   ({peak / 1980.0 * 100.0:.1f}% of 330 rpm)")

    rows = [{"row_kind": "command", **command.as_dict(),
             "metres_requested": args.metres,
             "metres_delivered": command.trajectory_duration_s * speed,
             "cycles": cycles, "planner_hz": args.planner_hz,
             "body_speed_mm_s": speed * 1e3,
             "stance_duty": timing.stance_duty,
             "theta_deg": float(np.rad2deg(posture.theta_rad)),
             "hold_hip_z_mm": held * 1e3,
             "roll_step_mm": posture.roll_step_m * 1e3,
             "cycle_period_s": period,
             "reverse": bool(args.reverse),
             "peak_motor_rate_deg_s": peak,
             "step9_failed_checks": ";".join(failed)}]
    write_rows_csv(OUT / f"{args.out}_summary.csv", rows)

    print()
    print(f"wrote -> {csv_path}")
    print(f"wrote -> {phase_path}")
    print(f"wrote -> {OUT / f'{args.out}_summary.csv'}")


if __name__ == "__main__":
    main()
