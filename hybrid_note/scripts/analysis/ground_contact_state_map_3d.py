"""Theta-beta ground-contact state map from 3D LegWheel geometry at fixed gamma.

This script keeps the output as a 2D theta-beta categorical map, but each cell
is evaluated with ``compute_ground_contact_pose_3d(theta, beta, gamma)``. Use it
when gamma changes the 3D lowest point but you still want to inspect the result
as a theta-beta state map.

Run from the project root:

    .venv/bin/python hybrid_note/scripts/analysis/ground_contact_state_map_3d.py
    .venv/bin/python hybrid_note/scripts/analysis/ground_contact_state_map_3d.py --gamma-deg 10
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import tempfile
from collections import Counter
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "legwheel_matplotlib"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Patch

NOTE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = NOTE_ROOT.parent
KINEMATICS_DIR = NOTE_ROOT / "scripts" / "kinematics"
ANALYSIS_DIR = NOTE_ROOT / "scripts" / "analysis"
for path in [PROJECT_ROOT, KINEMATICS_DIR, ANALYSIS_DIR]:
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from legwheel.config import RobotParams  # noqa: E402
from ground_contact_pose_3d import (  # noqa: E402
    CONTACT_HEIGHT_TOL,
    LATERAL_SAMPLES,
    RIM_ARC_SAMPLES,
    compute_ground_contact_pose_3d,
)
from ground_contact_state_map import (  # noqa: E402
    CONTACT_STATE_INFO,
    CONTACT_STATE_ORDER,
    STATE_COLORS,
    centers_to_edges,
    detect_state_boundaries,
    state_label,
    state_to_id_map,
    value_grid,
)

DEFAULT_TABLE_DIR = NOTE_ROOT / "outputs" / "tables" / "analysis"
DEFAULT_FIGURE_DIR = NOTE_ROOT / "outputs" / "figures" / "analysis"


def generate_contact_state_map_3d(
    theta_min_deg: float,
    theta_max_deg: float,
    theta_step_deg: float,
    beta_min_deg: float,
    beta_max_deg: float,
    beta_step_deg: float,
    gamma_deg: float,
    contact_height_tol: float = CONTACT_HEIGHT_TOL,
    arc_samples: int = RIM_ARC_SAMPLES,
    lateral_samples: int = LATERAL_SAMPLES,
    include_reference_points: bool = False,
) -> dict:
    """Scan theta-beta grid and classify each pose by 3D lowest contact state."""
    theta_values = value_grid(theta_min_deg, theta_max_deg, theta_step_deg)
    beta_values = value_grid(beta_min_deg, beta_max_deg, beta_step_deg)
    gamma = np.deg2rad(gamma_deg)
    state_to_id, id_to_state = state_to_id_map()
    state_grid = np.empty((theta_values.size, beta_values.size), dtype=object)
    class_grid = np.full((theta_values.size, beta_values.size), -1, dtype=int)
    rows = []

    for theta_index, theta_deg in enumerate(theta_values):
        theta = np.deg2rad(theta_deg)
        for beta_index, beta_deg in enumerate(beta_values):
            beta = np.deg2rad(beta_deg)
            result = compute_ground_contact_pose_3d(
                theta,
                beta,
                gamma,
                contact_height_tol=contact_height_tol,
                arc_samples=arc_samples,
                lateral_samples=lateral_samples,
                include_reference_points=include_reference_points,
            )
            state = result["lowest_contact_state"]
            state_grid[theta_index, beta_index] = state
            class_grid[theta_index, beta_index] = state_to_id[state]
            rows.append(
                {
                    "theta_deg": float(theta_deg),
                    "beta_deg": float(beta_deg),
                    "gamma_deg": float(gamma_deg),
                    "lowest_contact_state": state,
                    "lowest_contact_state_id": result["lowest_contact_state_id"],
                    "lowest_surface_name": result["lowest_surface_name"],
                    "lowest_geometry_type": result["lowest_geometry_type"],
                    "lowest_point_x": result["lowest_point_x"],
                    "lowest_point_y": result["lowest_point_y"],
                    "lowest_point_z": result["lowest_point_z"],
                    "height_min": result["height_min"],
                    "number_of_contact_points": result["number_of_contact_points"],
                    "contact_states": result["contact_states"],
                    "x_span": result["x_span"],
                    "y_span": result["y_span"],
                    "z_span": result["z_span"],
                }
            )

    return {
        "theta_values": theta_values,
        "beta_values": beta_values,
        "gamma_deg": float(gamma_deg),
        "state_grid": state_grid,
        "class_grid": class_grid,
        "rows": rows,
        "state_to_id": state_to_id,
        "id_to_state": id_to_state,
    }


def save_contact_map_csv_3d(path: Path, rows: list[dict]) -> None:
    fields = [
        "theta_deg",
        "beta_deg",
        "gamma_deg",
        "lowest_contact_state",
        "lowest_contact_state_id",
        "lowest_surface_name",
        "lowest_geometry_type",
        "lowest_point_x",
        "lowest_point_y",
        "lowest_point_z",
        "height_min",
        "number_of_contact_points",
        "contact_states",
        "x_span",
        "y_span",
        "z_span",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def plot_contact_state_map_3d(
    map_data: dict,
    boundary_segments: list[tuple[tuple[float, float], tuple[float, float]]],
    output_png: Path,
    dpi: int = 240,
    return_fig: bool = False,
) -> plt.Figure | None:
    theta_values = map_data["theta_values"]
    beta_values = map_data["beta_values"]
    class_grid = map_data["class_grid"]
    beta_edges = centers_to_edges(beta_values)
    theta_edges = centers_to_edges(theta_values)

    colors = [STATE_COLORS[state] for state in CONTACT_STATE_ORDER]
    cmap = mcolors.ListedColormap(colors)
    norm = mcolors.BoundaryNorm(np.arange(len(colors) + 1) - 0.5, cmap.N)

    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(11.0, 7.0))
    ax.pcolormesh(beta_edges, theta_edges, class_grid, cmap=cmap, norm=norm, shading="flat")

    if boundary_segments:
        collection = LineCollection(
            boundary_segments,
            colors="#111111",
            linewidths=0.45,
            alpha=0.72,
            zorder=5,
        )
        ax.add_collection(collection)

    handles = [
        Patch(
            facecolor=STATE_COLORS[state],
            edgecolor="black",
            linewidth=0.4,
            label=state_label(state),
        )
        for state in CONTACT_STATE_ORDER
    ]
    ax.legend(handles=handles, loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax.set_xlabel("beta [deg]")
    ax.set_ylabel("theta [deg]")
    ax.set_xlim(beta_edges[0], beta_edges[-1])
    ax.set_ylim(theta_edges[0], theta_edges[-1])
    ax.grid(True, linestyle=":", linewidth=0.35, alpha=0.25)
    ax.set_title(f"3D Ground Contact State Map: theta-beta at gamma={map_data['gamma_deg']:.2f} deg")
    fig.tight_layout()
    fig.savefig(output_png, dpi=dpi, bbox_inches="tight")
    if return_fig:
        return fig
    plt.close(fig)
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate theta-beta contact state map from 3D geometry at fixed gamma."
    )
    parser.add_argument("--theta-min-deg", type=float, default=17.0)
    parser.add_argument("--theta-max-deg", type=float, default=160.0)
    parser.add_argument("--theta-step-deg", type=float, default=1.0)
    parser.add_argument("--beta-min-deg", type=float, default=-180.0)
    parser.add_argument("--beta-max-deg", type=float, default=180.0)
    parser.add_argument("--beta-step-deg", type=float, default=1.0)
    parser.add_argument("--gamma-deg", type=float, default=10.0)
    parser.add_argument("--contact-height-tol", type=float, default=CONTACT_HEIGHT_TOL)
    parser.add_argument("--arc-samples", type=int, default=181)
    parser.add_argument("--lateral-samples", type=int, default=11)
    parser.add_argument(
        "--include-reference-points",
        action="store_true",
        help="Also include explicit non-contact HL/HR/O reference points.",
    )
    parser.add_argument("--dpi", type=int, default=240)
    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "ground_contact_state_map_3d_theta_beta_gamma.csv",
    )
    parser.add_argument(
        "--png",
        type=Path,
        default=DEFAULT_FIGURE_DIR / "ground_contact_state_map_3d_theta_beta_gamma.png",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.theta_max_deg > RobotParams.MAX_THETA_DEG:
        print(
            "Warning: requested theta range extends beyond RobotParams.MAX_THETA_DEG="
            f"{RobotParams.MAX_THETA_DEG:.1f} deg. The underlying LegModel clips theta internally."
        )

    map_data = generate_contact_state_map_3d(
        theta_min_deg=args.theta_min_deg,
        theta_max_deg=args.theta_max_deg,
        theta_step_deg=args.theta_step_deg,
        beta_min_deg=args.beta_min_deg,
        beta_max_deg=args.beta_max_deg,
        beta_step_deg=args.beta_step_deg,
        gamma_deg=args.gamma_deg,
        contact_height_tol=args.contact_height_tol,
        arc_samples=args.arc_samples,
        lateral_samples=args.lateral_samples,
        include_reference_points=args.include_reference_points,
    )
    boundary_segments, transition_counts = detect_state_boundaries(
        map_data["state_grid"],
        map_data["theta_values"],
        map_data["beta_values"],
    )

    save_contact_map_csv_3d(args.csv, map_data["rows"])
    plot_contact_state_map_3d(map_data, boundary_segments, args.png, dpi=args.dpi)

    state_counts = Counter(row["lowest_contact_state"] for row in map_data["rows"])
    print(f"Gamma: {args.gamma_deg:.3f} deg")
    print(f"Sampled poses: {len(map_data['rows'])}")
    print(f"Theta samples: {len(map_data['theta_values'])}")
    print(f"Beta samples: {len(map_data['beta_values'])}")
    print("State counts:")
    for state in CONTACT_STATE_ORDER:
        print(f"  {CONTACT_STATE_INFO[state]['state_id']} {state}: {state_counts[state]}")
    print(f"Boundary segments: {len(boundary_segments)}")
    print("Boundary transition counts:")
    for transition, count in sorted(transition_counts.items()):
        print(f"  {' <-> '.join(transition)}: {count}")
    print(f"Saved CSV: {args.csv}")
    print(f"Saved PNG: {args.png}")


if __name__ == "__main__":
    main()
