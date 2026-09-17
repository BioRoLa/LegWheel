"""Day 13: the flat-ground Hybrid gait, as motor commands.

Outputs (into ``hybrid_note/notes/day13/``):

``day13_motor_command.csv``      the motor command, through the project's writer
``day13_motor_summary.csv``      what it asks of the motors, per leg

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day13_step2_motor_driver.py
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
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
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (  # noqa: E402
    LEG_ORDER,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (  # noqa: E402
    NominalPosture2D,
    run_foot_rim_roll_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (  # noqa: E402
    DEFAULT_MARGIN_FLOOR_M,
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (  # noqa: E402
    COM_UNCERTAINTY_PER_AXIS_M,
    HYBRID_MARGIN_FLOOR_M,
    MARGIN_LOST_PER_COM_OFFSET,
    derived_margin_floor_m,
    frame_motor_rate_2d,
    hybrid_timing_2d,
    liftoff_order_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (  # noqa: E402
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (  # noqa: E402
    build_leg_plan_2d,
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (  # noqa: E402
    assemble_whole_body_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (  # noqa: E402
    MOTOR_MAX_RATE_RAD_S,
    CheckId,
    validate_whole_body_2d,
)
from hybrid_note.scripts.experiments.day13_motor_export_2d import (  # noqa: E402
    NotExportable,
    command_rows,
    motor_command_2d,
    write_motor_csv_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day13"

#: Step 9 failures this export is willing to carry.  **Empty**, and that is
#: the whole point: it used to hold ``SUPPORT_MARGIN``, first at a margin of
#: literally zero and then at 4.84 mm under an unreachable 10 mm floor.
#:
#: With ``HYBRID_MARGIN_FLOOR_M`` -- derived from the project owner's own
#: measurement of the centre of mass rather than picked -- the check passes on
#: its merits.  If anything ever needs to go back in here, that is a decision
#: and it belongs in the caller's source where it is visible.
ACCEPTED: tuple[CheckId, ...] = ()


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    fixed = NominalPosture2D()
    hip_z = [f.hip_xz_m[1] for f in run_foot_rim_roll_2d(fixed).frames]
    held = float(max(hip_z))
    levelled = replace(fixed, hold_hip_z_m=held)

    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="flat run, no crossing")
    plans = {leg: build_leg_plan_2d(leg, composed, posture=levelled,
                                    continuous_nominal=True)
             for leg in LEG_ORDER}
    timing = hybrid_timing_2d()
    four = plan_four_legs_2d(plans, timing)
    body = body_trajectory_2d(four, nominal_body_z_m=held - HIP_TO_BODY_Z_M,
                              samples=241)
    stability = swing_stability_2d(four, body,
                                   margin_floor_m=HYBRID_MARGIN_FLOOR_M)

    print("the four things that had to be true first")
    print(f"  theta-compensated rolling   hold hip at {held*1e3:.4f} mm")
    print(f"  continuously chained runs   one source per leg")
    print(f"  built from generator frames not from segment endpoints")
    print(f"  frames interpolated, not snapped to the nearest one")

    library = walk_timing_2d()
    print(f"\nthe gait")
    print(f"  liftoff order        "
          f"{' '.join(l.value for l in liftoff_order_2d(timing))}"
          f"   (LEG_ORDER is the reading order, not this -- trap 19)")
    print(f"  stance duty          {timing.stance_duty:.3f}"
          f"   (GAIT_LIBRARY['Walk'] has {library.stance_duty:.3f},"
          f" which is the critical duty)")
    print(f"  swing window         {timing.swing_duration_s*1e3:.0f} ms"
          f"   of a {timing.cycle_period_s:.1f} s cycle")
    margin = stability.minimum_margin_m
    print(f"  support margin       "
          f"{'n/a' if margin is None else f'{margin*1e3:.4f} mm'}"
          f"   (was 0.000 mm at duty {library.stance_duty:g})")

    print(f"\nthe floor, and where it comes from")
    print(f"  CoM measured at the geometric centre  "
          f"(project owner, 2026-09-02)")
    print(f"  assumed residual uncertainty          "
          f"+/-{COM_UNCERTAINTY_PER_AXIS_M*1e3:.1f} mm per axis")
    print(f"  margin lost per mm of CoM offset      "
          f"{MARGIN_LOST_PER_COM_OFFSET:.3f} mm/mm  (both axes, measured)")
    print(f"  -> floor                              "
          f"{derived_margin_floor_m()*1e3:.3f} mm"
          f"  -> {HYBRID_MARGIN_FLOOR_M*1e3:.1f} mm used")
    print(f"  what is left for everything unmodelled "
          f"{margin*1e3 - HYBRID_MARGIN_FLOOR_M*1e3:.3f} mm")
    print(f"     (no dynamics, no contact forces, no friction, no terrain "
          f"error -- Day 12 models none)")

    interpolated = assemble_whole_body_2d(four, body, stability, samples=241)
    trajectory = assemble_whole_body_2d(four, body, stability, samples=241,
                                        use_generator_frames=True)
    report = validate_whole_body_2d(trajectory, body, stability)

    def theta_span(whole):
        values = np.array([leg.theta_rad for s in whole.samples
                           for leg in s.legs.values()])
        return np.rad2deg(values.min()), np.rad2deg(values.max())

    lo_i, hi_i = theta_span(interpolated)
    lo_f, hi_f = theta_span(trajectory)
    print(f"\nwhat the frames put back")
    print(f"  endpoint interpolation  theta {lo_i:7.3f} .. {hi_i:7.3f} deg"
          f"   sweep {hi_i-lo_i:7.3f}")
    print(f"  generator frames        theta {lo_f:7.3f} .. {hi_f:7.3f} deg"
          f"   sweep {hi_f-lo_f:7.3f}")
    print(f"  -- the retraction to wheel mode is the whole difference, and it "
          f"is what the leg does to clear the ground.")

    print(f"\nStep 9 on the exported trajectory")
    print(f"  checks failed  {len(report.failed_checks())}"
          f"   {[c.value for c in report.failed_checks()]}")
    print(f"  failures       {len(report.failures)}")

    print(f"\nthe export refuses the interpolated one")
    try:
        motor_command_2d(interpolated, report, accept_failures=ACCEPTED)
        print("  IT DID NOT -- that is a bug")
    except NotExportable as refusal:
        print(f"  {refusal}")

    command = motor_command_2d(trajectory, report, accept_failures=ACCEPTED)
    summary = command.as_dict()
    print(f"\nthe command")
    for key in ("samples", "dt_s", "duration_s", "playback_hz",
                "peak_motor_rate_deg_s", "motor_limit_deg_s",
                "motor_utilisation", "accepted_failures"):
        print(f"  {key:24s} {summary[key]}")

    # The command's own rate is differenced across its playback dt; the plan's
    # is differenced between generator frames.  They are two measurements of
    # the same thing and they only agree once the frames are interpolated --
    # which is the point of checking them against each other here rather than
    # trusting either alone.
    demands = frame_motor_rate_2d(four)
    planned = max(d.peak_motor_rate_rad_s for d in demands)
    commanded = command.peak_motor_rate_rad_s()
    print(f"\nthe two rate measurements, which must now agree")
    print(f"  between generator frames  {np.rad2deg(planned):9.2f} deg/s"
          f"   ({planned / MOTOR_MAX_RATE_RAD_S * 100:5.1f}%)")
    print(f"  across the command's dt   {np.rad2deg(commanded):9.2f} deg/s"
          f"   ({commanded / MOTOR_MAX_RATE_RAD_S * 100:5.1f}%)")
    print(f"  the command asks for      "
          f"{commanded / planned:.3f}x the planned rate")
    if commanded > planned * 1.05:
        print(f"  -- WARNING: the command is still coarser than the plan.")

    print(f"\n{'leg':>4} {'idx':>4} {'theta (deg)':>18} {'beta (deg)':>22} "
          f"{'peak motor':>12}")
    for row in command_rows(command):
        if row["row_kind"] != "leg":
            continue
        print(f"{row['leg']:>4} {row['leg_index']:>4} "
              f"{row['theta_min_deg']:8.2f}..{row['theta_max_deg']:<8.2f} "
              f"{row['beta_min_deg']:10.2f}..{row['beta_max_deg']:<10.2f} "
              f"{row['peak_motor_rate_deg_s']:12.2f}")

    written = write_motor_csv_2d(command, OUT / "day13_motor_command")
    write_rows_csv(OUT / "day13_motor_summary.csv", command_rows(command))
    print(f"\nwrote -> {written}")
    print(f"wrote -> {OUT / 'day13_motor_summary.csv'}")

    print(f"\nBEFORE RUNNING THIS ON HARDWARE")
    print(f"  - the support margin is {margin*1e3:.3f} mm at its worst against "
          f"a {HYBRID_MARGIN_FLOOR_M*1e3:.0f} mm floor.")
    print(f"    That floor covers the CoM being somewhere other than measured, "
          f"and NOTHING else.")
    print(f"  - {COM_UNCERTAINTY_PER_AXIS_M*1e3:.0f} mm per axis is an ASSUMED "
          f"precision for that measurement.  If the real")
    print(f"    measurement was cruder, multiply it by "
          f"{MARGIN_LOST_PER_COM_OFFSET:.3f} and compare against "
          f"{margin*1e3:.3f} mm.")
    print(f"  - a CoM {margin*1e3/MARGIN_LOST_PER_COM_OFFSET:.2f} mm off centre "
          f"on both axes takes the margin to zero.")
    print(f"  - Day 12 modelled no dynamics, no contact forces and no friction,")
    print(f"    so no-slip here is a geometric property of the rim, not a")
    print(f"    verified physical one.")
    print(f"  - the writer prepends a ramp from zero to the first pose; check")
    print(f"    that ramp against the robot's actual start posture.")


if __name__ == "__main__":
    main()
