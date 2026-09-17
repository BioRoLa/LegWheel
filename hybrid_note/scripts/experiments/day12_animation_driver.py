"""Day 12 appendix driver: draw the assembled whole-body trajectory.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_whole_body_viewing_height.csv``  the drawn height per sample, and the
                                         foot gap it leaves on every other leg
``day12_whole_body_frames.png``          four frames, one per swing
``day12_whole_body_animation.gif``       the whole run

The inputs are rebuilt exactly as Step 8's driver builds them, with one stated
difference: ``use_generator_frames=True``, so the recovery swing shows the
retract it is made of.  Every frozen Day 12 metric was measured on the
interpolated variant; this is a viewer, not a re-measurement.

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_animation_driver.py
    python3 -u .../day12_animation_driver.py --no-animation   # strip + CSV only
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
from hybrid_note.scripts.experiments.day12_whole_body_animation_2d import (  # noqa: E402
    VIEWING_BASIS,
    animate_whole_body_2d,
    plot_frame_strip_2d,
    viewing_height_rows,
    viewing_heights_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (  # noqa: E402
    assemble_whole_body_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"

#: Step 7 left both blocked pairs unresolved (Step 8 driver quotes the same).
REPOSITION_UNRESOLVED = 2


def swing_frame_indices(whole, limit: int = 4) -> list[int]:
    """The middle of each of the first ``limit`` swings.

    A frame chosen this way is one where something is actually happening; an
    evenly spaced pick lands on whatever the clock happened to hit.
    """

    runs: list[tuple[object, int, int]] = []
    for index, sample in enumerate(whole.samples):
        leg = sample.swing_leg
        if leg is None:
            continue
        if runs and runs[-1][0] is leg and runs[-1][2] == index - 1:
            runs[-1] = (leg, runs[-1][1], index)
        else:
            runs.append((leg, index, index))
    return [(start + end) // 2 for _, start, end in runs[:limit]]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-animation", action="store_true")
    parser.add_argument("--stride", type=int, default=4)
    args = parser.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    started = time.time()

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
        four, body, stability, reposition_unresolved=REPOSITION_UNRESOLVED,
        use_generator_frames=True)
    print(f"assembled {len(whole.samples)} samples in {time.time() - started:.1f} s"
          f"   (use_generator_frames=True)")

    heights = viewing_heights_2d(four, whole,
                                 nominal_body_z_m=body.nominal_body_z_m)
    write_rows_csv(OUT / "day12_whole_body_viewing_height.csv",
                   viewing_height_rows(heights))

    finite = sum(1 for s in whole.samples
                 if s.body_position_world_m[2] == s.body_position_world_m[2])
    worst = max(h.worst_gap_m for h in heights)
    worst_at = max(heights, key=lambda h: h.worst_gap_m)
    spread = max(h.spread_m for h in heights)
    slacks = [h.lower_bound_slack_m for h in heights
              if h.lower_bound_slack_m is not None]
    off_ground = sum(1 for h in heights if h.legs_off_the_ground)

    print(f"\nthe height this drawing had to invent")
    print(f"  {VIEWING_BASIS}")
    print(f"  planned body_z, finite samples   {finite} of {len(whole.samples)}"
          f"   <- Step 5 left every one of them nan")
    print(f"  drawn body_z range               "
          f"{min(h.body_z_m for h in heights) * 1e3:.3f} .. "
          f"{max(h.body_z_m for h in heights) * 1e3:.3f} mm")
    print(f"  worst hard-demand spread         {spread * 1e3:.3f} mm")
    print(f"  worst foot left off the ground   {worst * 1e3:.3f} mm "
          f"at t = {worst_at.time_s:.3f} s "
          f"({', '.join(l.value for l in worst_at.legs_off_the_ground)})")
    print(f"  samples with a foot off ground   {off_ground} of {len(heights)}")
    print(f"  airborne lower bounds            "
          f"{'none active' if not slacks else f'min slack {min(slacks) * 1e3:.3f} mm'}"
          f"   (negative would mean the drawn body is too low)")

    indices = swing_frame_indices(whole)
    plot_frame_strip_2d(whole, heights, OUT / "day12_whole_body_frames.png",
                        indices=indices)
    print(f"\nstrip frames at t = "
          f"{', '.join(f'{whole.samples[i].time_s:.3f}' for i in indices)} s"
          f"   -> day12_whole_body_frames.png")

    if not args.no_animation:
        frames = animate_whole_body_2d(
            whole, heights, OUT / "day12_whole_body_animation.gif",
            stride=args.stride)
        print(f"animation {frames} frames -> day12_whole_body_animation.gif "
              f"({time.time() - started:.1f} s total)")

    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
