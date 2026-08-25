"""Deterministic serialization for offline hybrid trajectories."""

from __future__ import annotations

import csv
from pathlib import Path

from .types import HybridTrajectory


def write_trajectory_csv(trajectory: HybridTrajectory, path: str | Path) -> Path:
    """Write the frozen trajectory schema used by simulation and hardware."""

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    body_names = ("body_x", "body_y", "body_z", "body_roll", "body_pitch", "body_yaw")
    header = ["time_s", *body_names]
    for leg in range(4):
        header.extend(
            [
                f"leg{leg}_theta",
                f"leg{leg}_beta",
                f"leg{leg}_gamma",
                f"leg{leg}_mode",
                f"leg{leg}_rim",
                f"leg{leg}_alpha",
                f"leg{leg}_contact_active",
                f"leg{leg}_swing_phase",
                f"leg{leg}_foothold_x",
                f"leg{leg}_foothold_y",
                f"leg{leg}_foothold_z",
            ]
        )
    header.append("stability_margin")

    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for sample in range(trajectory.sample_count):
            row: list[object] = [trajectory.time_s[sample], *trajectory.body_pose_world[sample]]
            for leg in range(4):
                row.extend(
                    [
                        *trajectory.joint_position_rad[sample, leg],
                        trajectory.modes[sample][leg].value,
                        trajectory.rims[sample][leg].value,
                        trajectory.alpha_rad[sample, leg],
                        int(trajectory.contact_active[sample, leg]),
                        trajectory.swing_phase[sample, leg],
                        *trajectory.foothold_world_m[sample, leg],
                    ]
                )
            row.append(trajectory.stability_margin_m[sample])
            writer.writerow(row)
    return output
