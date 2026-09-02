"""Generate a reproducible Step 6 obstacle request schedule JSON."""

from __future__ import annotations

import argparse
import contextlib
import io
import json
from pathlib import Path

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk import (
    RectangleObstacle1D,
    WalkTerrain1D,
    generate_flat_walk_segment,
    schedule_obstacle_walk,
    schedule_to_dict,
    slice_segment,
)


def generate_artifact(output_dir: Path) -> Path:
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
    initial_state = slice_segment(flat, 0, 51).final_state
    terrain = WalkTerrain1D(
        RectangleObstacle1D(
            x_start_m=0.65,
            length_m=0.35,
            height_m=0.04,
            edge_margin_m=0.02,
        )
    )
    schedule = schedule_obstacle_walk(
        generator,
        initial_state,
        terrain,
        post_distance_m=0.30,
        maximum_touchdown_bias_m=0.04,
    )
    summary = schedule_to_dict(schedule)
    summary["inputs"] = {
        "initial_body_pose_world": initial_state.body_pose_world.tolist(),
        "initial_gait_cycle_phase": initial_state.gait_cycle_phase,
        "phase_offsets": list(generator.phase_offsets),
        "stance_duty": generator.stance_duty,
        "period_s": generator.T,
        "dt_s": generator.dt,
        "velocity_x_mps": float(generator.v_com[0]),
        "obstacle": {
            "x_start_m": terrain.obstacle.x_start_m,
            "length_m": terrain.obstacle.length_m,
            "height_m": terrain.obstacle.height_m,
            "edge_margin_m": terrain.obstacle.edge_margin_m,
        },
    }
    output = output_dir / "step6_schedule.json"
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/obstacle_walk_step6"),
    )
    args = parser.parse_args()
    print(generate_artifact(args.output_dir))
