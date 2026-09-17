"""Day 12 Step 11 driver: paper-oriented trajectory metrics.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step11_metrics.csv``   one row per terrain, plus the provenance notes

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step11_driver.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (  # noqa: E402
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_paper_metrics_2d import (  # noqa: E402
    COM_METRICS_ABSENT,
    energy_vocabulary,
    metrics_rows,
    trajectory_metrics_2d,
)
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (  # noqa: E402
    plan_terrain_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"
DAY10_11 = OUT.parent / "day10-11"
DAY6_7 = OUT.parent / "day6-7"

#: Evaluation queries, in the driver.  Same set Step 10 uses.
TERRAIN_QUERIES: tuple[tuple[str, float | None, float], ...] = (
    ("flat", None, 0.0),
    ("40mm x 400mm", 0.04, 0.40),
    ("100mm x 400mm", 0.10, 0.40),
    ("190mm x 400mm", 0.19, 0.40),
)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    tables = load_tables_2d(DAY10_11, DAY6_7)

    leftovers = energy_vocabulary()
    print(f"energy vocabulary in the metrics module: "
          f"{leftovers if leftovers else 'none'}")
    print("plan §18: these are mechanism metrics.  Nothing here computes or")
    print("implies a cost of transport -- that needs an experiment.\n")

    metrics = []
    for label, height_m, top_length_m in TERRAIN_QUERIES:
        terrain = None if height_m is None else SharedTerrainSpec2D(
            height_m=height_m, top_length_m=top_length_m, x_start_m=1.00,
            obstacle_id="day12_platform")
        run = plan_terrain_2d(terrain, tables)
        if not run.planned:
            first = run.first_limiting_constraint
            print(f"{label:16s} no trajectory to measure -- "
                  f"[{first.stage.value}] {first.detail[:80]}")
            continue
        metrics.append(trajectory_metrics_2d(
            label, run.trajectory, run.plan, run.body, run.stability))

    write_rows_csv(OUT / "day12_step11_metrics.csv", metrics_rows(metrics))

    print(f"\n{'terrain':>16} {'dist mm':>9} {'dur s':>7} {'swings':>7} "
          f"{'nom':>4} {'tt':>4} {'roll ls':>8} {'swing ls':>9}")
    print("  ls = leg-seconds (summed over four legs), not wall clock:")
    print("  an 'any leg' wall-clock total would equal the whole run every time.")
    for m in metrics:
        row = m.as_dict()
        print(f"{row['terrain']:>16} {row['traversal_distance_mm']:9.2f} "
              f"{row['traversal_duration_s']:7.3f} "
              f"{row['total_swing_segments']:7d} "
              f"{row['nominal_recovery_swings']:4d} "
              f"{row['terrain_transition_swings']:4d} "
              f"{row['foot_rim_roll_leg_seconds']:8.3f} "
              f"{row['swing_leg_seconds']:9.3f}")

    print(f"\n{'terrain':>16} {'bodyz p2p':>10} {'bodyz std':>10} "
          f"{'usable':>8} {'min margin':>11} {'mean margin':>12} "
          f"{'hip lift':>9}")
    for m in metrics:
        row = m.as_dict()
        p2p = row["body_z_peak_to_peak_mm"]
        std = row["body_z_std_mm"]
        print(f"{row['terrain']:>16} "
              f"{'n/a' if p2p is None else format(p2p, '10.3f')} "
              f"{'n/a' if std is None else format(std, '10.3f')} "
              f"{row['usable_body_samples']:>3}/{row['total_samples']:<4} "
              f"{row['minimum_stability_margin_mm']:11.4f} "
              f"{row['mean_swing_stability_margin_mm']:12.4f} "
              f"{row['max_hip_lift_mm']:9.3f}")

    print(f"\n{'terrain':>16} {'max joint disc':>15} {'max contact gap':>16}")
    for m in metrics:
        row = m.as_dict()
        print(f"{row['terrain']:>16} "
              f"{row['max_joint_discontinuity_deg']:15.4f} "
              f"{row['max_contact_handoff_gap_mm']:16.3f}")

    print(f"\n{'terrain':>16} {'roll dist mm':>13} {'transition dist mm':>19}")
    print("  these overlap (different legs, different kinds) -- do not sum them")
    for m in metrics:
        row = m.as_dict()
        print(f"{row['terrain']:>16} {row['foot_rim_roll_distance_mm']:13.2f} "
              f"{row['transition_roll_distance_mm']:19.2f}")

    print(f"\nwhole-robot CoM metrics")
    print(f"  {COM_METRICS_ABSENT}")
    for m in metrics:
        assert m.com_z_peak_to_peak_m is None and m.com_z_std_m is None

    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
