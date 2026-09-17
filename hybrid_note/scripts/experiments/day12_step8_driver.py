"""Day 12 Step 8 driver: the complete synchronized four-leg trajectory.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step8_whole_body.csv``   summary, assumptions, handoffs, samples

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step8_driver.py
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
    whole_body_rows,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"

#: Step 7 left both blocked pairs unresolved at the planning floor.  Quoted so
#: the assumption list is the real one and not a guess.
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

    result = assemble_whole_body_2d(
        four, body, stability, reposition_unresolved=REPOSITION_UNRESOLVED)
    write_rows_csv(OUT / "day12_step8_whole_body.csv", whole_body_rows(result))

    summary = result.as_dict()
    print("the summary plan §15 asks for")
    print(f"  max joint jump           {summary['max_joint_jump_deg']:.3f} deg"
          f"   (raw: whole revolutions included)")
    print(f"  max joint discontinuity  "
          f"{summary['max_joint_discontinuity_deg']:.3f} deg"
          f"   <- wrapped; {summary['whole_turn_handoffs']} handoff(s) are a "
          f"full turn, not a break")
    print(f"  max contact gap          {summary['max_contact_gap_mm']:.3f} mm")
    print(f"  min stability margin     "
          f"{summary['minimum_stability_margin_mm']:.3f} mm")
    print(f"  max rim geometry gap     "
          f"{summary['max_rim_geometry_gap_mm']:.4f} mm"
          f"   <- measured, not fixed (requirement 6)")
    print(f"    {summary['rim_gap_note']}")
    print(f"  finite body_z samples    {summary['finite_body_z_samples']} of "
          f"{summary['samples']}   (Step 5 left the rest INFEASIBLE)")

    print(f"\nshape")
    print(f"  samples                  {summary['samples']}")
    print(f"  handoffs checked         {summary['handoffs']}")
    print(f"  time monotonic           {summary['time_is_monotonic']}")
    print(f"  every sample has 4 legs  {summary['every_sample_has_four_legs']}")
    print(f"  body basis               {summary['body_basis']}")
    print(f"  CoM basis                {summary['com_basis']}")

    print(f"\nwhat this trajectory is assembled ON TOP OF  "
          f"({summary['unresolved_assumptions']} unresolved)")
    for note in result.assumptions:
        print(f"  - {note}")

    print(f"\nhandoffs, per leg")
    print(f"{'leg':>4} {'from':>4} {'to':>4} {'from kind':16s} {'to kind':16s} "
          f"{'joint':>8} {'wrapped':>8} {'body':>8} {'contact':>9} "
          f"{'rim gap':>8}  turn?")
    for handoff in result.handoffs:
        row = handoff.as_dict()
        print(f"{row['leg']:>4} {row['from_segment']:>4} {row['to_segment']:>4} "
              f"{row['from_kind']:16s} {row['to_kind']:16s} "
              f"{row['joint_jump_deg']:8.3f} "
              f"{row['joint_jump_wrapped_deg']:8.3f} {row['body_jump_mm']:8.3f} "
              f"{row['contact_jump_mm']:9.3f} {row['rim_geometry_gap_mm']:8.4f}  "
              f"{row['is_whole_turn']}")

    first = result.samples[0]
    print(f"\none sample in full, t = {first.time_s:.3f} s")
    row = first.as_dict()
    for key in ("body_x_mm", "body_z_mm", "body_roll_deg", "swing_leg",
                "support_legs", "stability_margin_mm",
                "max_rim_geometry_gap_mm"):
        print(f"  {key:26s} {row[key]}")
    for leg in LEG_ORDER:
        sample = first.legs[leg].as_dict()
        print(f"  {leg.value}  theta {sample['theta_deg']:7.2f}  "
              f"beta {sample['beta_deg']:8.2f}  gamma {sample['gamma_deg']:5.2f}  "
              f"{sample['mode']:8s} {sample['rim']:10s} "
              f"alpha {sample['alpha_deg']:7.2f}  "
              f"contact ({sample['contact_x_mm']:8.1f}, "
              f"{sample['contact_y_mm']:8.1f}) mm  "
              f"{sample['segment_kind']}")

    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
