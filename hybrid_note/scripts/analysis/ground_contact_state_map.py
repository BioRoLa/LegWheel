"""Theta-beta ground-contact state map for the 2D LegWheel geometry.

This script builds a categorical configuration-space map by repeatedly calling
``compute_ground_contact(theta, beta)`` from ``ground_contact_single_pose.py``.
It deliberately reuses the single-pose contact classifier instead of
re-implementing contact geometry here.

Run from the project root:

    .venv/bin/python hybrid_note/scripts/analysis/ground_contact_state_map.py
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
for path in [PROJECT_ROOT, KINEMATICS_DIR]:
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from legwheel.config import RobotParams  # noqa: E402
from ground_contact_single_pose import (  # noqa: E402
    CONTACT_HEIGHT_TOL,
    CONTACT_STATE_INFO,
    CONTACT_STATE_ORDER,
    RIM_ARC_SAMPLES,
    compute_ground_contact,
)

DEFAULT_TABLE_DIR = NOTE_ROOT / "outputs" / "tables" / "analysis"
DEFAULT_FIGURE_DIR = NOTE_ROOT / "outputs" / "figures" / "analysis"

STATE_COLORS = {
    "foot_rim": "#adc4a9",
    "left_rim": "#e29196",
    "right_rim": "#ECD09C",
    "non_contact_region": "#aad3f0",
}



def value_grid(min_deg: float, max_deg: float, step_deg: float) -> np.ndarray:
    if step_deg <= 0:
        raise ValueError("step_deg must be positive")

    values = list(np.arange(min_deg, max_deg + step_deg * 0.5, step_deg, dtype=float))
    if not values or not np.isclose(values[-1], max_deg):
        values.append(float(max_deg))
    values[0] = float(min_deg)
    values[-1] = float(max_deg)
    return np.array(values, dtype=float)


def centers_to_edges(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if values.size == 1:
        return np.array([values[0] - 0.5, values[0] + 0.5], dtype=float)

    edges = np.empty(values.size + 1, dtype=float)
    edges[1:-1] = (values[:-1] + values[1:]) / 2.0
    edges[0] = values[0] - (values[1] - values[0]) / 2.0
    edges[-1] = values[-1] + (values[-1] - values[-2]) / 2.0
    return edges


def state_to_id_map() -> tuple[dict[str, int], dict[int, str]]:
    state_to_id = {state: index for index, state in enumerate(CONTACT_STATE_ORDER)}
    id_to_state = {index: state for state, index in state_to_id.items()}
    return state_to_id, id_to_state


def generate_contact_state_map(
    theta_min_deg: float,
    theta_max_deg: float,
    theta_step_deg: float,
    beta_min_deg: float,
    beta_max_deg: float,
    beta_step_deg: float,
    contact_height_tol: float = CONTACT_HEIGHT_TOL,
    arc_samples: int = RIM_ARC_SAMPLES,
    include_reference_points: bool = True,
) -> dict:
    """Scan theta-beta grid and classify each pose by lowest contact state."""
    theta_values = value_grid(theta_min_deg, theta_max_deg, theta_step_deg)
    beta_values = value_grid(beta_min_deg, beta_max_deg, beta_step_deg)
    state_to_id, id_to_state = state_to_id_map()
    state_grid = np.empty((theta_values.size, beta_values.size), dtype=object)
    class_grid = np.full((theta_values.size, beta_values.size), -1, dtype=int)
    rows = []

    for theta_index, theta_deg in enumerate(theta_values):
        theta = np.deg2rad(theta_deg)
        for beta_index, beta_deg in enumerate(beta_values):
            beta = np.deg2rad(beta_deg)
            result = compute_ground_contact(
                theta,
                beta,
                contact_height_tol=contact_height_tol,
                arc_samples=arc_samples,
                include_reference_points=include_reference_points,
            )
            state = result["lowest_contact_state"]
            state_grid[theta_index, beta_index] = state
            class_grid[theta_index, beta_index] = state_to_id[state]
            rows.append(
                {
                    "theta_deg": float(theta_deg),
                    "beta_deg": float(beta_deg),
                    "lowest_contact_state": state,
                    "lowest_contact_state_id": result["lowest_contact_state_id"],
                    "lowest_surface_name": result["lowest_surface_name"],
                    "lowest_geometry_type": result["lowest_geometry_type"],
                    "lowest_point_x": result["lowest_point_x"],
                    "lowest_point_y": result["lowest_point_y"],
                    "height_min": result["height_min"],
                }
            )

    return {
        "theta_values": theta_values,
        "beta_values": beta_values,
        "state_grid": state_grid,
        "class_grid": class_grid,
        "rows": rows,
        "state_to_id": state_to_id,
        "id_to_state": id_to_state,
    }


def detect_state_boundaries(
    state_grid: np.ndarray,
    theta_values: np.ndarray,
    beta_values: np.ndarray,
) -> tuple[list[tuple[tuple[float, float], tuple[float, float]]], Counter]:
    """Return line segments where neighboring grid cells change state."""
    theta_edges = centers_to_edges(theta_values)
    beta_edges = centers_to_edges(beta_values)
    segments = []
    transition_counts = Counter()
    n_theta, n_beta = state_grid.shape

    for theta_index in range(n_theta):
        for beta_index in range(n_beta - 1):
            left_state = state_grid[theta_index, beta_index]
            right_state = state_grid[theta_index, beta_index + 1]
            if left_state == right_state:
                continue
            x = (beta_values[beta_index] + beta_values[beta_index + 1]) / 2.0
            y0 = theta_edges[theta_index]
            y1 = theta_edges[theta_index + 1]
            segments.append(((x, y0), (x, y1)))
            transition_counts[tuple(sorted((left_state, right_state)))] += 1

    for theta_index in range(n_theta - 1):
        for beta_index in range(n_beta):
            lower_state = state_grid[theta_index, beta_index]
            upper_state = state_grid[theta_index + 1, beta_index]
            if lower_state == upper_state:
                continue
            x0 = beta_edges[beta_index]
            x1 = beta_edges[beta_index + 1]
            y = (theta_values[theta_index] + theta_values[theta_index + 1]) / 2.0
            segments.append(((x0, y), (x1, y)))
            transition_counts[tuple(sorted((lower_state, upper_state)))] += 1

    return segments, transition_counts


def save_contact_map_csv(path: Path, rows: list[dict]) -> None:
    fields = [
        "theta_deg",
        "beta_deg",
        "lowest_contact_state",
        "lowest_contact_state_id",
        "lowest_surface_name",
        "lowest_geometry_type",
        "lowest_point_x",
        "lowest_point_y",
        "height_min",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def state_label(state: str) -> str:
    info = CONTACT_STATE_INFO[state]
    return f"{info['state_id']}: {info['label']}"


def plot_contact_state_map(
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
        Patch(facecolor=STATE_COLORS[state], edgecolor="black", linewidth=0.4, label=state_label(state))
        for state in CONTACT_STATE_ORDER
    ]
    ax.legend(handles=handles, loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)
    ax.set_xlabel("beta [deg]")
    ax.set_ylabel("theta [deg]")
    ax.set_xlim(beta_edges[0], beta_edges[-1])
    ax.set_ylim(theta_edges[0], theta_edges[-1])
    ax.grid(True, linestyle=":", linewidth=0.35, alpha=0.25)
    ax.set_title("2D Ground Contact State Map: theta-beta")
    fig.tight_layout()
    fig.savefig(output_png, dpi=dpi, bbox_inches="tight")
    if return_fig:
        return fig
    plt.close(fig)
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate theta-beta ground contact state map.")
    parser.add_argument("--theta-min-deg", type=float, default=17.0)
    parser.add_argument("--theta-max-deg", type=float, default=160.0)
    parser.add_argument("--theta-step-deg", type=float, default=0.1)
    parser.add_argument("--beta-min-deg", type=float, default=-180.0)
    parser.add_argument("--beta-max-deg", type=float, default=180.0)
    parser.add_argument("--beta-step-deg", type=float, default=0.1)
    parser.add_argument("--contact-height-tol", type=float, default=CONTACT_HEIGHT_TOL)
    parser.add_argument("--arc-samples", type=int, default=RIM_ARC_SAMPLES)
    parser.add_argument(
        "--exclude-reference-points",
        action="store_true",
        help="Pass through to compute_ground_contact; excludes explicit non-contact reference points.",
    )
    parser.add_argument("--dpi", type=int, default=240)
    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "ground_contact_state_map_theta_beta.csv",
    )
    parser.add_argument(
        "--png",
        type=Path,
        default=DEFAULT_FIGURE_DIR / "ground_contact_state_map_theta_beta.png",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.theta_max_deg > RobotParams.MAX_THETA_DEG:
        print(
            "Warning: requested theta range extends beyond RobotParams.MAX_THETA_DEG="
            f"{RobotParams.MAX_THETA_DEG:.1f} deg. The underlying LegModel clips theta internally."
        )

    map_data = generate_contact_state_map(
        theta_min_deg=args.theta_min_deg,
        theta_max_deg=args.theta_max_deg,
        theta_step_deg=args.theta_step_deg,
        beta_min_deg=args.beta_min_deg,
        beta_max_deg=args.beta_max_deg,
        beta_step_deg=args.beta_step_deg,
        contact_height_tol=args.contact_height_tol,
        arc_samples=args.arc_samples,
        include_reference_points=not args.exclude_reference_points,
    )
    boundary_segments, transition_counts = detect_state_boundaries(
        map_data["state_grid"],
        map_data["theta_values"],
        map_data["beta_values"],
    )

    save_contact_map_csv(args.csv, map_data["rows"])
    plot_contact_state_map(map_data, boundary_segments, args.png, dpi=args.dpi)

    state_counts = Counter(row["lowest_contact_state"] for row in map_data["rows"])
    print(f"Sampled poses: {len(map_data['rows'])}")
    print(f"Theta samples: {len(map_data['theta_values'])}")
    print(f"Beta samples: {len(map_data['beta_values'])}")
    print("State counts:")
    for state in CONTACT_STATE_ORDER:
        print(f"  {state}: {state_counts[state]}")
    print(f"Boundary segments: {len(boundary_segments)}")
    print("Boundary transition counts:")
    for transition, count in sorted(transition_counts.items()):
        print(f"  {' <-> '.join(transition)}: {count}")
    print(f"Saved CSV: {args.csv}")
    print(f"Saved PNG: {args.png}")


if __name__ == "__main__":
    main()
