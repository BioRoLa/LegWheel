"""Fast theta-beta ground-contact state map for the 2D LegWheel geometry.

The original state-map script evaluates every pose independently.  In the
LegModel, beta is only a rigid rotation about O, so this version builds the rim
geometry once per theta and evaluates all beta values with NumPy arrays.

Each Matplotlib Arc is minimized over the same discrete ``arc_samples`` used by
``ground_contact_single_pose.py``.  The only possible minimum samples on a
short circular arc are its two endpoints and the samples adjacent to the
downward direction, so scanning every point at every pose is unnecessary.

The command-line interface writes PNG and compact NPZ outputs by default.  CSV
is optional because a 0.1-degree full-range scan contains more than five
million rows and text serialization alone can take substantial time.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import tempfile
import time
from collections import Counter
from dataclasses import dataclass
from functools import lru_cache
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
from legwheel.visualization.plot_leg import PlotLeg  # noqa: E402
from ground_contact_single_pose import (  # noqa: E402
    CONTACT_HEIGHT_TOL,
    CONTACT_STATE_INFO,
    CONTACT_STATE_ORDER,
    REFERENCE_POINTS,
    RIM_ARC_SAMPLES,
    RIM_SURFACES,
    as_xy,
    surface_contact_state,
)
from ground_contact_state_map import (  # noqa: E402
    STATE_COLORS,
    centers_to_edges,
    state_label,
    state_to_id_map,
    value_grid,
)

DEFAULT_DATA_DIR = NOTE_ROOT / "outputs" / "data" / "analysis"
DEFAULT_FIGURE_DIR = NOTE_ROOT / "outputs" / "figures" / "analysis"

SURFACE_NAMES = tuple(RIM_SURFACES) + tuple(REFERENCE_POINTS)
SURFACE_TO_ID = {name: index for index, name in enumerate(SURFACE_NAMES)}
GEOMETRY_TYPES_2D = ("rim_arc", "reference_point")


@dataclass(frozen=True)
class ArcPrimitive:
    surface_id: int
    center_x: float
    center_y: float
    radius: float
    start_angle: float
    angle_step: float
    state_ids: np.ndarray


@dataclass(frozen=True)
class PointPrimitive:
    surface_id: int
    x: float
    y: float
    state_id: int


def _arc_angle_parameters(arc, arc_samples: int) -> tuple[float, float]:
    start = float(np.deg2rad(arc.theta1))
    end = float(np.deg2rad(arc.theta2))
    diff = end - start
    while diff > np.pi:
        diff -= 2.0 * np.pi
    while diff < -np.pi:
        diff += 2.0 * np.pi
    step = 0.0 if arc_samples <= 1 else diff / float(arc_samples - 1)
    return start, step


@lru_cache(maxsize=None)
def _surface_state_ids(surface_name: str, arc_samples: int) -> np.ndarray:
    state_to_id = {state: index for index, state in enumerate(CONTACT_STATE_ORDER)}
    state_ids = np.fromiter(
        (
            state_to_id[surface_contact_state(surface_name, index, arc_samples)]
            for index in range(arc_samples)
        ),
        dtype=np.int8,
        count=arc_samples,
    )
    state_ids.flags.writeable = False
    return state_ids


def build_theta_primitives_2d(
    leg: PlotLeg,
    theta: float,
    arc_samples: int,
    include_reference_points: bool,
    state_to_id: dict[str, int],
) -> list[ArcPrimitive | PointPrimitive]:
    """Build beta=0 primitives once; beta is applied analytically later."""
    if arc_samples < 2:
        raise ValueError("arc_samples must be at least 2")

    leg.forward(theta, 0.0, vector=False)
    leg.leg_shape.get_shape(np.array([0.0, 0.0]))
    primitives: list[ArcPrimitive | PointPrimitive] = []

    for surface_name in RIM_SURFACES:
        rim_obj = getattr(leg.leg_shape, surface_name, None)
        if rim_obj is None or not hasattr(rim_obj, "arc"):
            continue
        outer_arc = rim_obj.arc[1]
        radius_x = float(outer_arc.width) / 2.0
        radius_y = float(outer_arc.height) / 2.0
        if not np.isclose(radius_x, radius_y):
            raise ValueError(f"Fast arc evaluator requires a circular arc: {surface_name}")
        start, angle_step = _arc_angle_parameters(outer_arc, arc_samples)
        primitives.append(
            ArcPrimitive(
                surface_id=SURFACE_TO_ID[surface_name],
                center_x=float(outer_arc.center[0]),
                center_y=float(outer_arc.center[1]),
                radius=radius_x,
                start_angle=start,
                angle_step=angle_step,
                state_ids=_surface_state_ids(surface_name, arc_samples),
            )
        )

    if include_reference_points:
        for point_name, point_info in REFERENCE_POINTS.items():
            if point_info["leg_attr"] is None:
                xy = np.array([0.0, 0.0])
            else:
                xy = as_xy(getattr(leg, point_info["leg_attr"]))
            primitives.append(
                PointPrimitive(
                    surface_id=SURFACE_TO_ID[point_name],
                    x=float(xy[0]),
                    y=float(xy[1]),
                    state_id=state_to_id[point_info["contact_state"]],
                )
            )
    return primitives


def _minimum_arc_samples(
    primitive: ArcPrimitive,
    beta: np.ndarray,
    sin_beta: np.ndarray,
    cos_beta: np.ndarray,
    target_world_angle: float = -np.pi / 2.0,
    objective_sign: float = 1.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return the exact lowest point among the discretely sampled arc points."""
    sample_count = primitive.state_ids.size
    arc_end = primitive.start_angle + primitive.angle_step * (sample_count - 1)
    arc_mid = 0.5 * (primitive.start_angle + arc_end)

    target = target_world_angle - beta
    target += 2.0 * np.pi * np.rint((arc_mid - target) / (2.0 * np.pi))

    if np.isclose(primitive.angle_step, 0.0):
        lower_index = np.zeros(beta.size, dtype=np.int32)
        upper_index = lower_index
    else:
        sample_position = (target - primitive.start_angle) / primitive.angle_step
        sample_position = np.clip(sample_position, 0.0, sample_count - 1.0)
        lower_index = np.floor(sample_position).astype(np.int32)
        upper_index = np.ceil(sample_position).astype(np.int32)

    candidate_indices = np.vstack(
        (
            np.zeros(beta.size, dtype=np.int32),
            lower_index,
            upper_index,
            np.full(beta.size, sample_count - 1, dtype=np.int32),
        )
    )
    candidate_angles = primitive.start_angle + primitive.angle_step * candidate_indices
    center_y = sin_beta * primitive.center_x + cos_beta * primitive.center_y
    candidate_y = center_y[None, :] + primitive.radius * np.sin(candidate_angles + beta[None, :])
    local_choice = np.argmin(objective_sign * candidate_y, axis=0)
    columns = np.arange(beta.size)
    sample_indices = candidate_indices[local_choice, columns]
    point_y = candidate_y[local_choice, columns]

    selected_world_angle = primitive.start_angle + primitive.angle_step * sample_indices + beta
    center_x = cos_beta * primitive.center_x - sin_beta * primitive.center_y
    point_x = center_x + primitive.radius * np.cos(selected_world_angle)
    state_ids = primitive.state_ids[sample_indices]
    return point_x, point_y, sample_indices, state_ids


def evaluate_theta_primitives_2d(
    primitives: list[ArcPrimitive | PointPrimitive],
    beta: np.ndarray,
) -> dict[str, np.ndarray]:
    sin_beta = np.sin(beta)
    cos_beta = np.cos(beta)
    count = beta.size
    height = np.full(count, np.inf)
    point_x = np.full(count, np.nan)
    state_ids = np.full(count, -1, dtype=np.int8)
    surface_ids = np.full(count, -1, dtype=np.int8)
    geometry_type_ids = np.full(count, -1, dtype=np.int8)
    sample_indices = np.full(count, -1, dtype=np.int32)

    for primitive in primitives:
        if isinstance(primitive, ArcPrimitive):
            x, y, indices, primitive_state_ids = _minimum_arc_samples(
                primitive, beta, sin_beta, cos_beta
            )
            geometry_type_id = 0
        else:
            x = cos_beta * primitive.x - sin_beta * primitive.y
            y = sin_beta * primitive.x + cos_beta * primitive.y
            indices = np.full(count, -1, dtype=np.int32)
            primitive_state_ids = np.full(count, primitive.state_id, dtype=np.int8)
            geometry_type_id = 1

        # Strict comparison preserves the original record order for exact ties.
        replace = y < height
        height[replace] = y[replace]
        point_x[replace] = x[replace]
        state_ids[replace] = primitive_state_ids[replace]
        surface_ids[replace] = primitive.surface_id
        geometry_type_ids[replace] = geometry_type_id
        sample_indices[replace] = indices[replace]

    return {
        "class_ids": state_ids,
        "surface_ids": surface_ids,
        "geometry_type_ids": geometry_type_ids,
        "arc_sample_indices": sample_indices,
        "lowest_point_x": point_x,
        "lowest_point_y": height,
        "height_min": height,
    }


def generate_contact_state_map_fast(
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
    """Generate the same lowest-state map while reusing geometry across beta."""
    theta_values = value_grid(theta_min_deg, theta_max_deg, theta_step_deg)
    beta_values = value_grid(beta_min_deg, beta_max_deg, beta_step_deg)
    beta = np.deg2rad(beta_values)
    state_to_id, id_to_state = state_to_id_map()
    shape = (theta_values.size, beta_values.size)

    class_grid = np.full(shape, -1, dtype=np.int8)
    surface_id_grid = np.full(shape, -1, dtype=np.int8)
    geometry_type_id_grid = np.full(shape, -1, dtype=np.int8)
    arc_sample_index_grid = np.full(shape, -1, dtype=np.int32)
    lowest_point_x_grid = np.full(shape, np.nan)
    lowest_point_y_grid = np.full(shape, np.nan)

    leg = PlotLeg()
    for theta_index, theta_deg in enumerate(theta_values):
        primitives = build_theta_primitives_2d(
            leg=leg,
            theta=np.deg2rad(theta_deg),
            arc_samples=arc_samples,
            include_reference_points=include_reference_points,
            state_to_id=state_to_id,
        )
        row = evaluate_theta_primitives_2d(primitives, beta)
        class_grid[theta_index] = row["class_ids"]
        surface_id_grid[theta_index] = row["surface_ids"]
        geometry_type_id_grid[theta_index] = row["geometry_type_ids"]
        arc_sample_index_grid[theta_index] = row["arc_sample_indices"]
        lowest_point_x_grid[theta_index] = row["lowest_point_x"]
        lowest_point_y_grid[theta_index] = row["lowest_point_y"]

    state_names = np.asarray(CONTACT_STATE_ORDER, dtype=object)
    return {
        "theta_values": theta_values,
        "beta_values": beta_values,
        "class_grid": class_grid,
        "state_grid": state_names[class_grid],
        "surface_id_grid": surface_id_grid,
        "surface_names": SURFACE_NAMES,
        "geometry_type_id_grid": geometry_type_id_grid,
        "geometry_types": GEOMETRY_TYPES_2D,
        "arc_sample_index_grid": arc_sample_index_grid,
        "lowest_point_x_grid": lowest_point_x_grid,
        "lowest_point_y_grid": lowest_point_y_grid,
        "height_min_grid": lowest_point_y_grid,
        "state_to_id": state_to_id,
        "id_to_state": id_to_state,
        "arc_samples": int(arc_samples),
        "contact_height_tol": float(contact_height_tol),
        "include_reference_points": bool(include_reference_points),
        "pose_count": int(theta_values.size * beta_values.size),
    }


def detect_class_boundaries_fast(map_data: dict) -> tuple[np.ndarray, Counter]:
    """Vectorized boundary extraction for a categorical class grid."""
    class_grid = map_data["class_grid"]
    theta_values = map_data["theta_values"]
    beta_values = map_data["beta_values"]
    id_to_state = map_data["id_to_state"]
    theta_edges = centers_to_edges(theta_values)
    beta_edges = centers_to_edges(beta_values)
    segment_blocks = []
    transition_counts = Counter()

    theta_indices, beta_indices = np.nonzero(class_grid[:, :-1] != class_grid[:, 1:])
    if theta_indices.size:
        x = (beta_values[beta_indices] + beta_values[beta_indices + 1]) / 2.0
        segments = np.empty((theta_indices.size, 2, 2), dtype=float)
        segments[:, 0, 0] = x
        segments[:, 1, 0] = x
        segments[:, 0, 1] = theta_edges[theta_indices]
        segments[:, 1, 1] = theta_edges[theta_indices + 1]
        segment_blocks.append(segments)
        pairs = np.sort(
            np.column_stack(
                (class_grid[theta_indices, beta_indices], class_grid[theta_indices, beta_indices + 1])
            ),
            axis=1,
        )
        unique_pairs, counts = np.unique(pairs, axis=0, return_counts=True)
        for pair, count in zip(unique_pairs, counts):
            transition_counts[tuple(sorted((id_to_state[int(pair[0])], id_to_state[int(pair[1])])))] += int(count)

    theta_indices, beta_indices = np.nonzero(class_grid[:-1, :] != class_grid[1:, :])
    if theta_indices.size:
        y = (theta_values[theta_indices] + theta_values[theta_indices + 1]) / 2.0
        segments = np.empty((theta_indices.size, 2, 2), dtype=float)
        segments[:, 0, 0] = beta_edges[beta_indices]
        segments[:, 1, 0] = beta_edges[beta_indices + 1]
        segments[:, 0, 1] = y
        segments[:, 1, 1] = y
        segment_blocks.append(segments)
        pairs = np.sort(
            np.column_stack(
                (class_grid[theta_indices, beta_indices], class_grid[theta_indices + 1, beta_indices])
            ),
            axis=1,
        )
        unique_pairs, counts = np.unique(pairs, axis=0, return_counts=True)
        for pair, count in zip(unique_pairs, counts):
            transition_counts[tuple(sorted((id_to_state[int(pair[0])], id_to_state[int(pair[1])])))] += int(count)

    if not segment_blocks:
        return np.empty((0, 2, 2), dtype=float), transition_counts
    return np.concatenate(segment_blocks), transition_counts


def plot_contact_state_map_fast(
    map_data: dict,
    boundary_segments: np.ndarray,
    output_png: Path,
    dpi: int = 240,
    return_fig: bool = False,
    title: str | None = None,
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
    ax.imshow(
        class_grid,
        origin="lower",
        interpolation="nearest",
        aspect="auto",
        extent=(beta_edges[0], beta_edges[-1], theta_edges[0], theta_edges[-1]),
        cmap=cmap,
        norm=norm,
    )
    if boundary_segments.size:
        ax.add_collection(
            LineCollection(
                boundary_segments,
                colors="#111111",
                linewidths=0.45,
                alpha=0.72,
                zorder=5,
            )
        )

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
    ax.set_title(title or "2D Ground Contact State Map: theta-beta (fast)")
    fig.tight_layout()
    fig.savefig(output_png, dpi=dpi, bbox_inches="tight")
    if return_fig:
        return fig
    plt.close(fig)
    return None


def save_contact_map_npz_fast(path: Path, map_data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        theta_values=map_data["theta_values"],
        beta_values=map_data["beta_values"],
        class_grid=map_data["class_grid"],
        contact_state_order=np.asarray(CONTACT_STATE_ORDER, dtype="U"),
        surface_id_grid=map_data["surface_id_grid"],
        surface_names=np.asarray(map_data["surface_names"], dtype="U"),
        geometry_type_id_grid=map_data["geometry_type_id_grid"],
        geometry_types=np.asarray(map_data["geometry_types"], dtype="U"),
        arc_sample_index_grid=map_data["arc_sample_index_grid"],
        lowest_point_x_grid=map_data["lowest_point_x_grid"],
        lowest_point_y_grid=map_data["lowest_point_y_grid"],
        arc_samples=np.array(map_data["arc_samples"]),
        include_reference_points=np.array(map_data["include_reference_points"]),
    )


def save_contact_map_csv_fast(path: Path, map_data: dict) -> None:
    """Stream the optional CSV without constructing millions of row dictionaries."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "theta_deg",
        "beta_deg",
        "lowest_contact_state",
        "lowest_contact_state_id",
        "lowest_surface_name",
        "lowest_geometry_type",
        "arc_sample_index",
        "lowest_point_x",
        "lowest_point_y",
        "height_min",
    ]
    with path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(fields)
        for theta_index, theta_deg in enumerate(map_data["theta_values"]):
            for beta_index, beta_deg in enumerate(map_data["beta_values"]):
                class_id = int(map_data["class_grid"][theta_index, beta_index])
                state = map_data["id_to_state"][class_id]
                surface_id = int(map_data["surface_id_grid"][theta_index, beta_index])
                geometry_type_id = int(map_data["geometry_type_id_grid"][theta_index, beta_index])
                y = float(map_data["lowest_point_y_grid"][theta_index, beta_index])
                writer.writerow(
                    (
                        float(theta_deg),
                        float(beta_deg),
                        state,
                        CONTACT_STATE_INFO[state]["state_id"],
                        map_data["surface_names"][surface_id],
                        map_data["geometry_types"][geometry_type_id],
                        int(map_data["arc_sample_index_grid"][theta_index, beta_index]),
                        float(map_data["lowest_point_x_grid"][theta_index, beta_index]),
                        y,
                        y,
                    )
                )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a vectorized 2D ground-contact state map.")
    parser.add_argument("--theta-min-deg", type=float, default=17.0)
    parser.add_argument("--theta-max-deg", type=float, default=160.0)
    parser.add_argument("--theta-step-deg", type=float, default=0.1)
    parser.add_argument("--beta-min-deg", type=float, default=-180.0)
    parser.add_argument("--beta-max-deg", type=float, default=180.0)
    parser.add_argument("--beta-step-deg", type=float, default=0.1)
    parser.add_argument("--contact-height-tol", type=float, default=CONTACT_HEIGHT_TOL)
    parser.add_argument("--arc-samples", type=int, default=RIM_ARC_SAMPLES)
    parser.add_argument("--exclude-reference-points", action="store_true")
    parser.add_argument("--dpi", type=int, default=240)
    parser.add_argument(
        "--npz",
        type=Path,
        default=DEFAULT_DATA_DIR / "ground_contact_state_map_theta_beta_fast.npz",
    )
    parser.add_argument(
        "--png",
        type=Path,
        default=DEFAULT_FIGURE_DIR / "ground_contact_state_map_theta_beta_fast.png",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Optional CSV path. Full 0.1-degree maps contain over five million text rows.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.theta_max_deg > RobotParams.MAX_THETA_DEG:
        print(
            "Warning: requested theta range extends beyond RobotParams.MAX_THETA_DEG="
            f"{RobotParams.MAX_THETA_DEG:.1f} deg. The underlying LegModel clips theta internally."
        )

    started = time.perf_counter()
    map_data = generate_contact_state_map_fast(
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
    generated_at = time.perf_counter()
    boundaries, transition_counts = detect_class_boundaries_fast(map_data)
    save_contact_map_npz_fast(args.npz, map_data)
    if args.csv is not None:
        save_contact_map_csv_fast(args.csv, map_data)
    plot_contact_state_map_fast(map_data, boundaries, args.png, dpi=args.dpi)
    finished_at = time.perf_counter()

    state_counts = np.bincount(map_data["class_grid"].ravel(), minlength=len(CONTACT_STATE_ORDER))
    print(f"Sampled poses: {map_data['pose_count']}")
    print(f"Theta samples: {len(map_data['theta_values'])}")
    print(f"Beta samples: {len(map_data['beta_values'])}")
    print("State counts:")
    for state, count in zip(CONTACT_STATE_ORDER, state_counts):
        print(f"  {state}: {int(count)}")
    print(f"Boundary segments: {len(boundaries)}")
    for transition, count in sorted(transition_counts.items()):
        print(f"  {' <-> '.join(transition)}: {count}")
    print(f"Geometry and classification: {generated_at - started:.3f} s")
    print(f"Total including output: {finished_at - started:.3f} s")
    print(f"Saved NPZ: {args.npz}")
    if args.csv is not None:
        print(f"Saved CSV: {args.csv}")
    print(f"Saved PNG: {args.png}")


if __name__ == "__main__":
    main()
