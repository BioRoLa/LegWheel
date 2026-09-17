"""Day 12 Step 9 driver: validate the assembled whole-body trajectory.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step9_validation.csv``   summary, per-check verdicts, failures, delegations

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step9_driver.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

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
    SharedTerrainSpec2D,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (  # noqa: E402
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (  # noqa: E402
    LEG_ORDER,
    initialize_four_leg_state_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (  # noqa: E402
    swing_stability_2d,
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
    validate_whole_body_2d,
    validation_rows,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"

REPOSITION_UNRESOLVED = 2


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="flat run, no crossing")
    plans = {leg: build_leg_plan_2d(leg, composed) for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, walk_timing_2d())
    state = initialize_four_leg_state_2d(SharedTerrainSpec2D(
        height_m=0.04, top_length_m=0.40, x_start_m=1.00,
        obstacle_id="day12_platform"))
    body = body_trajectory_2d(
        four, nominal_body_z_m=float(state.body_position_world_m[2]))
    stability = swing_stability_2d(four, body)
    whole = assemble_whole_body_2d(
        four, body, stability, reposition_unresolved=REPOSITION_UNRESOLVED)

    report = validate_whole_body_2d(whole, body, stability)
    write_rows_csv(OUT / "day12_step9_validation.csv", validation_rows(report))

    summary = report.as_dict()
    print(f"checks run       {summary['checks_run']}")
    print(f"  passed         {summary['checks_passed']}")
    print(f"  failed         {summary['checks_failed']}")
    print(f"failures         {summary['failures']}")
    print(f"is_valid         {summary['is_valid']}")

    print(f"\nper check")
    for check in report.checks_run:
        found = report.failures_of(check)
        mark = "PASS" if not found else f"FAIL ({len(found)})"
        print(f"  {check.value:32s} {mark}")

    print(f"\nthe first failure of each failing check, in full")
    for check in report.failed_checks():
        first = report.failures_of(check)[0].as_dict()
        print(f"\n  {check.value}")
        for key, value in first.items():
            if key != "check" and value not in (None, ""):
                print(f"    {key:16s} {value}")

    print(f"\nchecked elsewhere, NOT re-derived here"
          f"   ({summary['delegated_checks']})")
    for name, why in report.delegated.items():
        print(f"  {name}")
        print(f"    {why}")

    print(f"\ncannot be decided here at all"
          f"   ({summary['unevaluable_checks']})")
    for name, why in report.unevaluable.items():
        print(f"  {name}")
        print(f"    {why}")

    print(f"\nthe rates Step 3 deferred to here -- now judged")
    print(f"{'leg':>4} {'peak theta':>12} {'peak beta':>12} {'stance beta':>13} "
          f"{'airborne beta':>15} {'ratio':>8}")
    for rate in report.joint_rates:
        row = rate.as_dict()
        ratio = row["airborne_to_stance_ratio"]
        print(f"{row['leg']:>4} {row['peak_theta_rate_deg_s']:12.2f} "
              f"{row['peak_beta_rate_deg_s']:12.2f} "
              f"{row['peak_stance_beta_rate_deg_s']:13.2f} "
              f"{row['peak_airborne_beta_rate_deg_s']:15.2f} "
              f"{'n/a' if ratio is None else f'{ratio:8.3f}'}")
    for rate in report.joint_rates[:1]:
        row = rate.as_dict()
        print(f"\n  motor budget   |theta'| + |beta'| <= "
              f"{row['motor_limit_deg_s']:.0f} deg/s   (330 rpm, shared)")
        print(f"  peak motor     {row['peak_motor_rate_deg_s']:.2f} deg/s"
              f"   = {row['motor_utilisation']*100:.1f}% of it")
        print(f"  theta headroom {row['theta_rate_headroom_deg_s']:.2f} deg/s"
              f"   left over while beta is at its peak")
    print("\n  units are deg/s.  Step 3 predicted the airborne/stance beta ratio")
    print("  would be about 10.553x; this is that number, measured on the")
    print("  assembled trajectory rather than derived from the duty.")
    print("  peak theta reads 0 because Step 8 interpolates between segment")
    print("  endpoints and a recovery starts and ends at the same theta -- the")
    print("  retraction in between is not in the assembled samples.  It is a")
    print("  LOWER BOUND, and every row says so.")

    print(f"\nthis verdict is about a trajectory built on top of"
          f"   ({summary['unresolved_assumptions']} unresolved)")
    for note in report.assumptions:
        print(f"  - {note}")

    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
