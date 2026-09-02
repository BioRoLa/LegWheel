"""Step 8 artifacts: full-geometry traversal validation plus visual evidence.

Generates one obstacle walk, re-checks the assembled trajectory end to end
(including full leg geometry against the obstacle and support-polygon
stability), and writes inspectable pictures:

* ``step8_stage_frames.png``  one frame per traversal stage, full leg geometry
* ``step8_body_and_contacts.png``  body path, contact footfalls and stage bands
* ``step8_traversal.gif``  animation of the whole traversal
* ``step8_validation.json``  the machine-readable validation report

Everything here is offline geometry and kinematics.  No simulation, contact
force, friction or hardware result is claimed.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from legwheel.planners.obstacle_walk.collision import (
    DEFAULT_ARC_SAMPLES,
    LegGeometrySampler,
)
from legwheel.planners.obstacle_walk.traversal import (
    ObstacleWalkRequest,
    ObstacleWalkResult,
    generate_obstacle_walk,
)
from legwheel.planners.obstacle_walk.types import LEG_ORDER
from legwheel.planners.obstacle_walk.validation import validate_traversal

LEG_COLOURS = {"FL": "tab:blue", "FR": "tab:orange", "RR": "tab:green", "RL": "tab:red"}


def _sampler(result: ObstacleWalkResult, arc_samples: int) -> LegGeometrySampler:
    generator = result.walk_generator
    if generator is None:
        raise ValueError("result.walk_generator is required to draw leg geometry.")
    return LegGeometrySampler(
        kinematics=list(generator.legs),
        hip_positions_body_m=list(generator.hip_positions),
        arc_samples=arc_samples,
    )


def _draw_terrain(axis, result: ObstacleWalkResult) -> None:
    from matplotlib.patches import Rectangle

    terrain = result.terrain
    obstacle = terrain.obstacle
    axis.axhline(terrain.ground_height_m, color="black", linewidth=1.1, zorder=1)
    axis.add_patch(
        Rectangle(
            (obstacle.x_start_m, terrain.ground_height_m),
            obstacle.length_m,
            obstacle.height_m,
            facecolor="0.82",
            edgecolor="0.25",
            zorder=1,
        )
    )


def _draw_frame(axis, result: ObstacleWalkResult, sampler: LegGeometrySampler, row: int) -> None:
    """Draw the obstacle, the body and all four legs' geometry at one row."""

    _draw_terrain(axis, result)
    pose = result.segment.body_pose_world[row]
    commands = result.segment.commands_rad[row]
    contacts = result.segment.foot_contact_points_world_m[row]
    active = result.segment.contact_active[row]

    hips = []
    for leg_index, leg in enumerate(LEG_ORDER):
        geometry = sampler.sample(leg_index, commands[leg_index], pose)
        colour = LEG_COLOURS[leg.value]
        points = geometry.points_world_xz_m
        axis.plot(points[:, 0], points[:, 1], ".", markersize=1.4, color=colour, zorder=3)
        for segment in geometry.link_segments_world_xz_m:
            axis.plot(
                segment[:, 0], segment[:, 1], "-", linewidth=1.0, color=colour, zorder=3
            )
        hip_body = sampler.hip_positions_body_m[leg_index]
        hips.append((pose[0] + hip_body[0], pose[2] + hip_body[2]))
        marker = "o" if active[leg_index] else "x"
        axis.plot(
            contacts[leg_index, 0],
            contacts[leg_index, 2],
            marker,
            markersize=5,
            markerfacecolor="none",
            color=colour,
            zorder=4,
        )
    hips = np.asarray(hips)
    order = np.argsort(hips[:, 0])
    axis.plot(
        hips[order, 0], hips[order, 1], "-", color="0.35", linewidth=2.4, zorder=2
    )
    axis.plot([pose[0]], [pose[2]], "s", color="black", markersize=4, zorder=5)
    axis.set_aspect("equal")
    axis.grid(True, alpha=0.2)


def _stage_rows(result: ObstacleWalkResult) -> list[tuple[str, int]]:
    """Pick one representative row per traversal stage, in trajectory order."""

    chosen: dict[str, int] = {}
    for record in result.records:
        if record.kind != "swing":
            continue
        stage = record.stage.value
        if stage not in chosen:
            chosen[stage] = (record.start_row + record.end_row) // 2
    return sorted(chosen.items(), key=lambda item: item[1])


def plot_stage_frames(result: ObstacleWalkResult, output: Path, arc_samples: int) -> Path:
    import matplotlib.pyplot as plt

    sampler = _sampler(result, arc_samples)
    stages = _stage_rows(result)
    figure, axes = plt.subplots(len(stages), 1, figsize=(11, 3.0 * len(stages)))
    axes = np.atleast_1d(axes)
    for axis, (stage, row) in zip(axes, stages):
        _draw_frame(axis, result, sampler, row)
        record_index = next(
            (item.index for item in result.records if item.start_row <= row <= item.end_row),
            None,
        )
        axis.set_title(
            f"stage {stage}  |  row {row}  |  segment {record_index}", fontsize=10
        )
        axis.set_ylabel("world z (m)")
    axes[-1].set_xlabel("world x (m)")
    figure.suptitle(
        "Step 8 full leg geometry at each traversal stage "
        f"(obstacle {result.request.obstacle_height_m * 1e3:.0f} mm)",
        fontsize=11,
    )
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    plt.close(figure)
    return output


def plot_body_and_contacts(result: ObstacleWalkResult, output: Path) -> Path:
    import matplotlib.pyplot as plt

    figure, (top, bottom) = plt.subplots(
        2, 1, figsize=(11, 7), gridspec_kw={"height_ratios": [2, 1]}
    )
    _draw_terrain(top, result)
    poses = result.segment.body_pose_world
    top.plot(poses[:, 0], poses[:, 2], "-", color="black", linewidth=1.4, label="body origin")
    for record in result.records:
        if record.touchdown_world_m is None:
            continue
        x, _y, z = record.touchdown_world_m
        colour = LEG_COLOURS[record.leg.value]
        top.plot([x], [z], "o", color=colour, markersize=5)
    for leg in LEG_ORDER:
        top.plot([], [], "o", color=LEG_COLOURS[leg.value], label=f"{leg.value} touchdown")
    top.set_ylabel("world z (m)")
    top.set_aspect("equal")
    top.grid(True, alpha=0.25)
    top.legend(loc="upper left", ncol=3, fontsize=8)
    top.set_title("body path and every touchdown, with the obstacle to scale")

    phase = result.segment.phase
    rows = np.arange(len(phase))
    for leg_index, leg in enumerate(LEG_ORDER):
        bottom.fill_between(
            rows,
            leg_index,
            leg_index + phase[:, leg_index] * 0.8,
            color=LEG_COLOURS[leg.value],
            step="post",
        )
    bottom.set_yticks(np.arange(4) + 0.4)
    bottom.set_yticklabels([leg.value for leg in LEG_ORDER])
    bottom.set_xlabel("trajectory row")
    bottom.set_title("swing phase (filled = swing); exactly one leg at a time")
    bottom.grid(True, alpha=0.25)
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    plt.close(figure)
    return output


def animate_traversal(
    result: ObstacleWalkResult, output: Path, arc_samples: int, frame_stride: int
) -> Path:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    sampler = _sampler(result, arc_samples)
    rows = list(range(0, result.segment.sample_count, max(1, frame_stride)))
    poses = result.segment.body_pose_world
    obstacle = result.terrain.obstacle
    x_min = float(min(poses[:, 0].min(), obstacle.x_start_m) - 0.6)
    x_max = float(max(poses[:, 0].max(), obstacle.x_end_m) + 0.6)
    figure, axis = plt.subplots(figsize=(11, 4.2))

    def draw(row: int) -> None:
        axis.clear()
        _draw_frame(axis, result, sampler, row)
        record = next(
            (item for item in result.records if item.start_row <= row <= item.end_row),
            None,
        )
        stage = "-" if record is None else record.stage.value
        event = "-" if record is None or record.event_index is None else record.event_index
        axis.set_xlim(x_min, x_max)
        axis.set_ylim(-0.05, float(poses[:, 2].max()) + 0.25)
        axis.set_xlabel("world x (m)")
        axis.set_ylabel("world z (m)")
        axis.set_title(f"row {row}  |  event {event}  |  stage {stage}", fontsize=10)

    animation = FuncAnimation(figure, draw, frames=rows, interval=60)
    output.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output, writer=PillowWriter(fps=16))
    plt.close(figure)
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--obstacle-x", type=float, default=0.65)
    parser.add_argument("--obstacle-length", type=float, default=0.35)
    parser.add_argument("--obstacle-height", type=float, default=0.06)
    parser.add_argument("--step-length", type=float, default=0.15)
    parser.add_argument("--period", type=float, default=2.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--step-clearance", type=float, default=0.03)
    parser.add_argument("--approach-distance", type=float, default=0.45)
    parser.add_argument("--post-distance", type=float, default=0.30)
    parser.add_argument("--arc-samples", type=int, default=DEFAULT_ARC_SAMPLES)
    parser.add_argument("--collision-stride", type=int, default=5,
                        help="rows between full-geometry collision evaluations")
    parser.add_argument("--animation-stride", type=int, default=12)
    parser.add_argument("--skip-animation", action="store_true")
    parser.add_argument("--output-dir", type=Path,
                        default=Path("outputs/obstacle_walk_step8"))
    args = parser.parse_args(argv)

    request = ObstacleWalkRequest(
        obstacle_x_start_m=args.obstacle_x,
        obstacle_length_m=args.obstacle_length,
        obstacle_height_m=args.obstacle_height,
        step_length_m=args.step_length,
        period_s=args.period,
        dt_s=args.dt,
        step_clearance_m=args.step_clearance,
        approach_distance_m=args.approach_distance,
        post_distance_m=args.post_distance,
    )
    print("generating trajectory ...")
    result = generate_obstacle_walk(request)
    print(f"  {result.segment.sample_count} rows, {len(result.records)} segments")

    print(f"validating (collision every {args.collision_stride} rows) ...")
    report = validate_traversal(
        result,
        frame_stride=args.collision_stride,
        arc_samples=args.arc_samples,
    )

    output_dir = args.output_dir
    outputs = [
        plot_stage_frames(result, output_dir / "step8_stage_frames.png", args.arc_samples),
        plot_body_and_contacts(result, output_dir / "step8_body_and_contacts.png"),
    ]
    if not args.skip_animation:
        print("rendering animation ...")
        outputs.append(
            animate_traversal(
                result,
                output_dir / "step8_traversal.gif",
                args.arc_samples,
                args.animation_stride,
            )
        )
    report_path = output_dir / "step8_validation.json"
    report_path.write_text(json.dumps(report.to_dict(), indent=2) + "\n", encoding="utf-8")
    outputs.append(report_path)

    print("\n[STAGES]")
    for stage, reached in report.stage_reached.items():
        print(f"  {stage:<32}: {reached}")
    print("\n[CHECKS]")
    for check in report.checks:
        print(f"  {check.name:<32}: {check.status.value:<8} {check.detail}")
    print(f"\n  offline_complete_traversal      : {report.offline_complete_traversal}")
    clearance = report.minimum_obstacle_clearance_m
    clearance_text = "n/a" if clearance is None else f"{clearance * 1e3:.3f} mm"
    print(f"  minimum obstacle clearance      : {clearance_text}")
    print(f"  minimum stability margin        : "
          f"{report.minimum_stability_margin_m * 1e3:.2f} mm")
    print("\n[FILES]")
    for path in outputs:
        print(f"  {path}")
    print("\nOffline geometry and kinematics only: no simulation, contact force,")
    print("friction or hardware validation is claimed.")
    return 0 if report.offline_complete_traversal else 1


if __name__ == "__main__":
    raise SystemExit(main())
