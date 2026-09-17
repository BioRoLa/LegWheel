"""Day 12 Step 2 driver: four-leg registration, as tables and one figure.

Outputs (into ``hybrid_note/notes/day12/``):

``day12_step2_leg_mounts.csv``       the four mounting offsets and their source
``day12_step2_four_leg_state.csv``   body + platform + four legs + symmetry
``day12_step2_terrain_sweep.csv``    same code, several platform parameters
``day12_step2_four_leg_state.png``   sagittal and top views

Run from the repository root::

    python3 -u LegWheel/hybrid_note/scripts/experiments/day12_step2_driver.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import numpy as np  # noqa: E402

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (  # noqa: E402
    SharedTerrainSpec2D,
    write_rows_csv,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (  # noqa: E402
    LEG_ORDER,
    FlatRunExtent2D,
    LegId,
    four_leg_rows,
    initialize_four_leg_state_2d,
    leg_mounts_2d,
    plot_four_leg_state_2d,
    sagittal_reach_agreement_2d,
)

OUT = Path(__file__).resolve().parents[2] / "notes" / "day12"

#: Platform parameters to demonstrate that only the parameters change.  These
#: are **driver inputs**, deliberately not importable planner constants: plan
#: §0.1.  The first is a near-flat degenerate case, the last is off the
#: evaluation set entirely, so the sweep cannot be read as a list of supported
#: sizes.
TERRAIN_QUERIES: tuple[tuple[float, float], ...] = (
    (0.001, 0.40),
    (0.04, 0.40),
    (0.10, 0.40),
    (0.19, 0.40),
    (0.07, 0.22),
)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    mounts = leg_mounts_2d()
    write_rows_csv(OUT / "day12_step2_leg_mounts.csv", [m.as_dict() for m in mounts])
    print("leg mounts (sagittal plane origin in body frame)")
    for mount in mounts:
        row = mount.as_dict()
        print(f"  {row['leg']}  index {row['leg_index']}  "
              f"x {row['offset_x_mm']:+8.3f}  y {row['offset_y_mm']:+9.3f}  "
              f"z {row['offset_z_mm']:+8.3f}  mm")

    agreement = sagittal_reach_agreement_2d()
    print("\n2D / 3D reach agreement (what licenses mounting Step 1 here)")
    print(f"  3D hip-to-foot drop        "
          f"{agreement['hip_to_foot_drop_3d_mm']:.6f} mm")
    print(f"  2D flat stance height      "
          f"{agreement['hip_stance_height_2d_mm']:.6f} mm")
    print(f"  difference                 {agreement['difference_mm']:+.3e} mm"
          f"   (the 2D scene's deliberate 1e-9 m surface offset)")

    terrain = SharedTerrainSpec2D(
        height_m=0.04, top_length_m=0.40, x_start_m=1.00,
        obstacle_id="day12_platform",
    )
    extent = FlatRunExtent2D()
    state = initialize_four_leg_state_2d(terrain, extent=extent)
    write_rows_csv(OUT / "day12_step2_four_leg_state.csv", four_leg_rows(state))

    bx, by, bz = (float(v) for v in state.body_position_world_m)
    print(f"\nbody           ({bx*1e3:.1f}, {by*1e3:.1f}, {bz*1e3:.3f}) mm   "
          f"rpy = (0, 0, 0)")
    print(f"platform       {terrain.height_m*1e3:.0f} mm high x "
          f"{terrain.top_length_m*1e3:.0f} mm long, "
          f"x {terrain.x_start_m*1e3:.0f} -> {terrain.x_max_m*1e3:.0f} mm")
    print(f"flat run       {extent.flat_before_m*1e3:.0f} mm before / "
          f"{extent.flat_after_m*1e3:.0f} mm after")
    print(f"{'leg':>4} {'hip x':>9} {'hip y':>9} {'hip z':>8} "
          f"{'contact':>8} {'surface':>9} {'alpha':>7} {'gap':>9}")
    for leg in LEG_ORDER:
        row = state.leg(leg).as_dict()
        print(f"{row['leg']:>4} {row['hip_x_mm']:9.1f} {row['hip_y_mm']:9.1f} "
              f"{row['hip_z_mm']:8.1f} {str(row['in_contact']):>8} "
              f"{str(row['surface_id']):>9} "
              f"{0.0 if row['alpha_deg'] is None else row['alpha_deg']:7.2f} "
              f"{row['surface_gap_mm']:+9.4f}")

    failed = [c for c in state.symmetry_checks() if not c.symmetric]
    print(f"\nsymmetry       {len(state.symmetry_checks())} checks, "
          f"{len(failed)} failed   ->  is_symmetric = {state.is_symmetric}")
    for check in failed:
        print(f"  FAILED {check.as_dict()}")
    print(f"all_in_contact {state.all_in_contact}")

    sweep = []
    for height_m, top_length_m in TERRAIN_QUERIES:
        query = SharedTerrainSpec2D(
            height_m=height_m, top_length_m=top_length_m, x_start_m=1.00,
            obstacle_id="day12_platform",
        )
        result = initialize_four_leg_state_2d(query, extent=extent)
        sweep.append({
            "platform_height_mm": height_m * 1e3,
            "platform_top_length_mm": top_length_m * 1e3,
            "platform_x_start_mm": query.x_start_m * 1e3,
            "platform_x_end_mm": query.x_max_m * 1e3,
            "body_x_mm": float(result.body_position_world_m[0]) * 1e3,
            "body_z_mm": float(result.body_position_world_m[2]) * 1e3,
            "all_in_contact": result.all_in_contact,
            "is_symmetric": result.is_symmetric,
        })
    write_rows_csv(OUT / "day12_step2_terrain_sweep.csv", sweep)
    print("\nsame code, only terrain parameters change")
    print(f"{'height':>8} {'top len':>8} {'body_z':>10} {'contact':>8} {'symmetric':>10}")
    for row in sweep:
        print(f"{row['platform_height_mm']:8.1f} {row['platform_top_length_mm']:8.1f} "
              f"{row['body_z_mm']:10.3f} {str(row['all_in_contact']):>8} "
              f"{str(row['is_symmetric']):>10}")
    heights = {round(row["body_z_mm"], 6) for row in sweep}
    print(f"  -> body_z takes {len(heights)} distinct value(s) across "
          f"{len(sweep)} platforms; the initial stance is on the lower ground, "
          f"so it must not depend on the platform.")

    plot_four_leg_state_2d(state, path=OUT / "day12_step2_four_leg_state.png")
    print(f"\nwrote -> {OUT}")


if __name__ == "__main__":
    main()
