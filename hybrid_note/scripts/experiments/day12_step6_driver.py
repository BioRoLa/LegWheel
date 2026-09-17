"""Day 12 Step 6 driver: three-leg support stability over every swing.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step6_stability.csv``   traversal summary, one row per swing, per sample
``day12_step6_stability.png``   support triangle at the worst instant + margins

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step6_driver.py
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
    leg_mounts_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (  # noqa: E402
    COM_BASIS,
    DEFAULT_MARGIN_FLOOR_M,
    GAMMA_RAD,
    plot_stability_2d,
    stability_rows,
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (  # noqa: E402
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (  # noqa: E402
    build_leg_plan_2d,
    plan_four_legs_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    print(f"gamma            {GAMMA_RAD:.1f} rad   (Day 12 keeps ABAD at zero)")
    print(f"CoM basis        {COM_BASIS}")
    print("\nmounting offsets that fix the lateral geometry (Step 2)")
    for mount in leg_mounts_2d(GAMMA_RAD):
        row = mount.as_dict()
        print(f"  {row['leg']}  x {row['offset_x_mm']:+8.3f}  "
              f"y {row['offset_y_mm']:+9.3f} mm")

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

    stability = swing_stability_2d(four, trajectory,
                                   margin_floor_m=DEFAULT_MARGIN_FLOOR_M)
    write_rows_csv(OUT / "day12_step6_stability.csv", stability_rows(stability))

    summary = stability.as_dict()
    print(f"\nbody assumption  {summary['body_basis']}")
    print(f"\nswings           {summary['swings']}"
          f"   unstable = {summary['unstable_swings']}")
    print(f"margin floor     {summary['margin_floor_mm']:.1f} mm"
          f"   (a planning floor, not a measured limit)")
    minimum = summary["minimum_stability_margin_mm"]
    print(f"minimum margin   "
          f"{'n/a' if minimum is None else f'{minimum:.3f} mm'}"
          f"   worst = {summary['worst_swing_leg']} at "
          f"t = {summary['worst_time_s']} s")
    print(f"stable           {summary['stable']}")

    print(f"\n{'swing':>6} {'support':>14} {'from':>7} {'to':>7} "
          f"{'min margin':>11} {'at':>7}  stable")
    for swing in stability.swings:
        row = swing.as_dict()
        minimum = row["minimum_stability_margin_mm"]
        worst_at = row["worst_time_s"]
        print(f"{row['swing_leg']:>6} {row['support_legs']:>14} "
              f"{row['start_s']:7.3f} {row['end_s']:7.3f} "
              f"{'        n/a' if minimum is None else f'{minimum:11.3f}'} "
              f"{'    n/a' if worst_at is None else f'{worst_at:7.3f}'}  "
              f"{row['stable']}")

    worst = stability.worst_swing
    if worst is not None:
        sample = min((s for s in worst.samples if s.margin_m is not None),
                     key=lambda s: s.margin_m)
        print(f"\nthe worst instant, in full")
        for key, value in sample.as_dict().items():
            print(f"  {key:18s} {value}")

    plot_stability_2d(four, trajectory, stability,
                      path=OUT / "day12_step6_stability.png")
    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
