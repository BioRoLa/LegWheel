"""Day 12 Step 7 driver: resolve ``TOP_REPOSITION`` in the four-leg context.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step7_reposition.csv``   one row per attempt, one per support gate

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step7_driver.py
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
    BLOCKED_PAIRS,
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
    LegId,
    initialize_four_leg_state_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (  # noqa: E402
    DEFAULT_MARGIN_FLOOR_M,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (  # noqa: E402
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_top_reposition_2d import (  # noqa: E402
    REPOSITION_TARGETS,
    RepositionOutcome,
    reposition_rows,
    resolve_top_reposition_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (  # noqa: E402
    build_leg_plan_2d,
    plan_four_legs_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"

#: Day 10--11 Step 9's own cells for the two blocked pairs.  Quoted, not chosen.
BLOCKED_CELLS = {
    StrategyId.ROLL_SWING: (0.160, 0.350),
    StrategyId.SWING_ROLL: (0.140, 0.225),
}


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
    trajectory = body_trajectory_2d(
        four, nominal_body_z_m=float(state.body_position_world_m[2]))

    print("the two cases Day 10-11 left unresolved  (plan §14 requirement 9)")
    for strategy, (height_m, top_length_m) in BLOCKED_CELLS.items():
        target = REPOSITION_TARGETS[strategy]
        print(f"\n  {strategy.value}   h = {height_m*1e3:.0f} mm, "
              f"L_top = {top_length_m*1e3:.0f} mm")
        print(f"    verdict from Day 10-11   "
              f"{BLOCKED_PAIRS[strategy].verdict.value}")
        print(f"    target condition         {target.rim.value}"
              + (f", theta >= {target.theta_min_rad*57.29578:.0f} deg"
                 if target.theta_min_rad else "")
              + (f", theta <= {target.theta_max_rad*57.29578:.0f} deg"
                 if target.theta_max_rad else ""))

    attempts = []
    for strategy, (height_m, top_length_m) in BLOCKED_CELLS.items():
        attempts.append(resolve_top_reposition_2d(
            four, trajectory, strategy, height_m, top_length_m, leg=LegId.LF))

    # A second pass with a floor low enough that the gate opens.  It is NOT the
    # answer -- it exists so the gate is shown to work in both directions, and
    # every row it produces is stamped ``relaxed_floor``.
    for strategy, (height_m, top_length_m) in BLOCKED_CELLS.items():
        attempts.append(resolve_top_reposition_2d(
            four, trajectory, strategy, height_m, top_length_m, leg=LegId.LF,
            margin_floor_m=-1.0))

    write_rows_csv(OUT / "day12_step7_reposition.csv", reposition_rows(attempts))

    print(f"\n{'strategy':>26} {'floor':>8}  {'outcome':>22}  resolved")
    for attempt in attempts:
        row = attempt.as_dict()
        floor = "RELAXED" if attempt.relaxed_floor else "planning"
        print(f"{row['strategy']:>26} {floor:>8}  {row['outcome']:>22}  "
              f"{row['resolved']}")

    print("\nthe planning-floor answers, in full")
    for attempt in attempts:
        if attempt.relaxed_floor:
            continue
        print(f"\n  {attempt.strategy.value}")
        print(f"    outcome            {attempt.outcome.value}")
        print(f"    reason             {attempt.reason}")
        print(f"    support margin     "
              f"{attempt.gate.as_dict()['minimum_margin_mm']} mm"
              f"  (floor {DEFAULT_MARGIN_FLOOR_M*1e3:.1f} mm)")
        print(f"    original evidence  {attempt.original_evidence[:150]}...")

    print("\nwhat the relaxed-floor pass shows  (NOT the answer)")
    for attempt in attempts:
        if not attempt.relaxed_floor:
            continue
        row = attempt.as_dict()
        print(f"\n  {attempt.strategy.value}")
        print(f"    outcome            {row['outcome']}")
        print(f"    reason             {attempt.reason}")
        print(f"    swing valid        {row['swing_valid']}"
              f"   failure = {row['swing_failure']}")
        print(f"    segment kind       {row['segment_kind']}")
        print(f"    touchdown          rim {row['touchdown_rim']}, theta "
              f"{row['touchdown_theta_deg']}")

    resolved = [a for a in attempts if a.resolved and not a.relaxed_floor]
    print(f"\nresolved at the planning floor: {len(resolved)} of "
          f"{len([a for a in attempts if not a.relaxed_floor])}")
    print("plan §14 acceptance is satisfied either way: a case must reach a")
    print("real verdict -- resolved, or a specific support-related failure.")
    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
