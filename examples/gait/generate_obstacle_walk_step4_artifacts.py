"""Generate reproducible Step 4 step-up/down swing prototype artifacts."""

from __future__ import annotations

import argparse
import contextlib
import io
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk import (
    RectangleObstacle1D,
    WalkTerrain1D,
    generate_flat_walk_segment,
    generate_swing_segment,
    plot_swing_plan,
    slice_segment,
)


HEIGHT_M = 0.04
CLEARANCE_M = 0.03


def _summary(plan) -> dict[str, object]:
    return {
        "swing_leg": plan.swing_leg.value,
        "sample_count": plan.segment.sample_count,
        "touchdown_world_m": plan.touchdown_world_m.tolist(),
        "target_surface_id": plan.target_surface_id,
        "rim_alpha_td_deg": plan.rim_alpha_td_deg,
        "terrain_max_height_world_m": plan.terrain_max_height_world_m,
        "requested_apex_height_world_m": plan.requested_apex_height_world_m,
        "achieved_apex_height_world_m": plan.achieved_apex_height_world_m,
        "maximum_tracking_error_m": plan.maximum_tracking_error_m,
        "full_geometry_collision_checked": plan.full_geometry_collision_checked,
    }


def generate_artifacts(output_dir: Path) -> tuple[Path, Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    with contextlib.redirect_stdout(io.StringIO()):
        generator = GaitGenerator3D(
            stand_height=0.25,
            twist=[0.0, 0.05, 0.0],
            step_height=0.04,
            period=1.0,
            gait_type="Walk",
            dt=0.02,
            stability_margin=0.0,
        )
        flat = generate_flat_walk_segment(generator, n_cycles=2)
    ground_state = slice_segment(flat, 0, 51).final_state
    ground_foot = ground_state.foot_contact_points_world_m[0]

    up_terrain = WalkTerrain1D(
        RectangleObstacle1D(ground_foot[0] + 0.04, 0.25, HEIGHT_M, 0.02)
    )
    up_target = np.array([ground_foot[0] + 0.10, ground_foot[1], HEIGHT_M])
    up_plan = generate_swing_segment(
        generator,
        ground_state,
        "FL",
        up_target,
        up_terrain,
        CLEARANCE_M,
    )

    top_body = ground_state.body_pose_world.copy()
    top_body[2] += HEIGHT_M
    top_feet = ground_state.foot_contact_points_world_m.copy()
    top_feet[:, 2] += HEIGHT_M
    top_state = replace(
        ground_state,
        body_pose_world=top_body,
        foot_contact_points_world_m=top_feet,
        surface_ids=("obstacle_top",) * 4,
    )
    top_foot = top_state.foot_contact_points_world_m[0]
    obstacle_end = top_foot[0] + 0.03
    down_terrain = WalkTerrain1D(
        RectangleObstacle1D(-0.30, obstacle_end + 0.30, HEIGHT_M, 0.01)
    )
    down_target = np.array([top_foot[0] + 0.08, top_foot[1], 0.0])
    down_plan = generate_swing_segment(
        generator,
        top_state,
        "FL",
        down_target,
        down_terrain,
        CLEARANCE_M,
    )

    up_path = plot_swing_plan(up_plan, up_terrain, output_dir / "step4_step_up.png")
    down_path = plot_swing_plan(down_plan, down_terrain, output_dir / "step4_step_down.png")
    summary_path = output_dir / "step4_summary.json"
    summary_path.write_text(
        json.dumps(
            {
                "status": "static-body single-leg trajectory prototype",
                "height_m": HEIGHT_M,
                "clearance_m": CLEARANCE_M,
                "step_up": _summary(up_plan),
                "step_down": _summary(down_plan),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return up_path, down_path, summary_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/obstacle_walk_step4"),
    )
    args = parser.parse_args()
    for path in generate_artifacts(args.output_dir):
        print(path)
