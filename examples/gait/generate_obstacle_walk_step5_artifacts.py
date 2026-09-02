"""Generate a reproducible Step 5 two-top/two-ground stance artifact."""

from __future__ import annotations

import argparse
import contextlib
import io
import json
from pathlib import Path

import numpy as np

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk import (
    LEG_ORDER,
    RectangleObstacle1D,
    WalkTerrain1D,
    generate_flat_walk_segment,
    generate_stance_segment,
    generate_swing_segment,
    plot_stance_plan,
    slice_segment,
)


HEIGHT_M = 0.04


def generate_artifacts(output_dir: Path) -> tuple[Path, Path]:
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
    ground = slice_segment(flat, 0, 51).final_state
    front_left = ground.foot_contact_points_world_m[0]
    terrain = WalkTerrain1D(
        RectangleObstacle1D(front_left[0] + 0.04, 0.25, HEIGHT_M, 0.02)
    )
    fl_target = np.array([front_left[0] + 0.10, front_left[1], HEIGHT_M])
    one_top = generate_swing_segment(
        generator,
        ground,
        "FL",
        fl_target,
        terrain,
        clearance_m=0.03,
    ).segment.final_state
    front_right = one_top.foot_contact_points_world_m[1]
    fr_target = np.array(
        [terrain.obstacle.legal_top_x_min_m + 0.01, front_right[1], HEIGHT_M]
    )
    two_top = generate_swing_segment(
        generator,
        one_top,
        "FR",
        fr_target,
        terrain,
        clearance_m=0.03,
    ).segment.final_state

    target_body = two_top.body_pose_world.copy()
    target_body[0] += 0.01
    target_body[2] += 0.003
    target_body[4] += np.deg2rad(0.5)
    plan = generate_stance_segment(
        generator,
        two_top,
        target_body,
        terrain,
        motion_duration_s=0.30,
    )

    plot_path = plot_stance_plan(plan, terrain, output_dir / "step5_two_top_stance.png")
    summary_path = output_dir / "step5_summary.json"
    summary_path.write_text(
        json.dumps(
            {
                "status": "quasi-static world-fixed contact stance prototype",
                "height_m": HEIGHT_M,
                "surface_ids": list(plan.segment.start_state.surface_ids),
                "sample_count": plan.segment.sample_count,
                "dt_s": plan.segment.dt_s,
                "requested_motion_duration_s": plan.requested_motion_duration_s,
                "body_trajectory_assumption": plan.body_trajectory_assumption,
                "start_body_pose_world": plan.segment.body_pose_world[0].tolist(),
                "target_body_pose_world": plan.requested_body_pose_world.tolist(),
                "maximum_contact_drift_m": plan.maximum_contact_drift_m,
                "maximum_contact_drift_by_leg_m": {
                    leg.value: float(np.max(plan.contact_drift_m[:, index]))
                    for index, leg in enumerate(LEG_ORDER)
                },
                "full_geometry_collision_checked": (
                    plan.full_geometry_collision_checked
                ),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return plot_path, summary_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/obstacle_walk_step5"),
    )
    args = parser.parse_args()
    for path in generate_artifacts(args.output_dir):
        print(path)
