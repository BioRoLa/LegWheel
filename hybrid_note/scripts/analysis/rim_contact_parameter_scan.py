"""Grid scan for which named LegWheel rim surfaces become lowest.

This is a first-pass parameter-space analyzer. It samples a theta/beta/gamma
grid, finds the geometric ground-contact candidate set for each pose, and
summarizes the observed parameter ranges for each named rim surface.

The scan reuses the current ``PlotLeg`` geometry and samples only the outer arc
of named rim primitives. Joint centers, links, labels, and markers are excluded.

Run from the project root:

    .venv/bin/python hybrid_note/scripts/analysis/rim_contact_parameter_scan.py
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
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

NOTE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = NOTE_ROOT.parent
KINEMATICS_DIR = NOTE_ROOT / "scripts" / "kinematics"
for path in [PROJECT_ROOT, KINEMATICS_DIR]:
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from legwheel.config import RobotParams  # noqa: E402
from legwheel.visualization.plot_leg import PlotLeg  # noqa: E402
from plot_leg_3d import to_display_xyz  # noqa: E402

DEFAULT_TABLE_DIR = NOTE_ROOT / "outputs" / "tables" / "analysis"
DEFAULT_FIGURE_DIR = NOTE_ROOT / "outputs" / "figures" / "analysis"
CONTACT_Z_TOL = 1e-3
TOP_NON_CONTACT_FRACTION = 0.35

CONTACT_STATE_INFO = {
    "foot_rim": {
        "state_id": "F",
        "label": "foot_rim",
        "short_label": "foot\nrim",
        "color": "#adc4a9",
    },
    "left_rim": {
        "state_id": "L",
        "label": "left_rim",
        "short_label": "left\nrim",
        "color": "#e29196",
    },
    "right_rim": {
        "state_id": "R",
        "label": "right_rim",
        "short_label": "right\nrim",
        "color": "#ECD09C",
    },
    "non_contact_region": {
        "state_id": "N",
        "label": "non_contact_region",
        "short_label": "non\ncontact",
        "color": "#aad3f0",
    },
}
CONTACT_STATE_ORDER = list(CONTACT_STATE_INFO)
CONTACT_STATE_COLORS = {
    state_name: state_info["color"] for state_name, state_info in CONTACT_STATE_INFO.items()
}

GROUP_COLORS = {
    "foot_rim": "#adc4a9",
    "left_rim": "#e29196",
    "right_rim": "#ECD09C",
    "non_contact_region": "#aad3f0",
}

SURFACE_COLORS = {
    "foot_rim": "#adc4a9",
    "upper_rim_l": "#e29196",
    "upper_rim_r": "#ECD09C",
    "lower_rim_l": "#e29196",
    "lower_rim_r": "#ECD09C",
    "upper_rim_l_f": "#aad3f0",
    "upper_rim_r_f": "#aad3f0",
}

RIM_SURFACES = {
    "foot_rim": {
        "group": "foot_rim",
        "description": "bottom tire/foot rim outer arc",
    },
    "upper_rim_l": {
        "group": "left_rim",
        "description": "left upper structural rim outer arc",
    },
    "upper_rim_r": {
        "group": "right_rim",
        "description": "right upper structural rim outer arc",
    },
    "lower_rim_l": {
        "group": "left_rim",
        "description": "left lower structural rim outer arc",
    },
    "lower_rim_r": {
        "group": "right_rim",
        "description": "right lower structural rim outer arc",
    },
    "upper_rim_l_f": {
        "group": "non_contact_region",
        "description": "left upper tire outer arc",
    },
    "upper_rim_r_f": {
        "group": "non_contact_region",
        "description": "right upper tire outer arc",
    },
}

SURFACE_TO_CONTACT_STATE = {
    "foot_rim": "foot_rim",
    "lower_rim_l": "left_rim",
    "lower_rim_r": "right_rim",
}


def contact_state_for_surface_point(
    surface_name: str,
    local_x_m: float,
    surface_center_x_m: float,
) -> str:
    """Map raw sampled rim points to the paper-style contact state used for maps."""
    if surface_name == "foot_rim":
        return "foot_rim"
    return SURFACE_TO_CONTACT_STATE.get(surface_name, "unknown")


def contact_states_for_surface_samples(surface_name: str, sample_count: int) -> np.ndarray:
    """Vectorized contact-state mapping for one sampled rim arc."""
    if surface_name == "foot_rim":
        return np.full(sample_count, "foot_rim", dtype=object)

    if surface_name in {"upper_rim_l", "upper_rim_l_f"}:
        normalized = np.linspace(0.0, 1.0, sample_count)
        return np.where(
            normalized <= TOP_NON_CONTACT_FRACTION,
            "non_contact_region",
            "left_rim",
        ).astype(object)

    if surface_name in {"upper_rim_r", "upper_rim_r_f"}:
        normalized = np.linspace(0.0, 1.0, sample_count)
        return np.where(
            normalized >= 1.0 - TOP_NON_CONTACT_FRACTION,
            "non_contact_region",
            "right_rim",
        ).astype(object)

    return np.full(
        sample_count,
        SURFACE_TO_CONTACT_STATE.get(surface_name, "unknown"),
        dtype=object,
    )


def state_metadata(state_name: str) -> dict:
    return CONTACT_STATE_INFO.get(
        state_name,
        {
            "state_id": "",
            "label": state_name,
            "short_label": state_name,
            "color": "#7f7f7f",
        },
    )


def ordered_names(names: set[str], preferred_order: list[str]) -> list[str]:
    order_index = {name: index for index, name in enumerate(preferred_order)}
    return sorted(names, key=lambda name: (order_index.get(name, len(preferred_order)), name))


def most_common_ordered(counter: Counter, preferred_order: list[str]) -> str:
    order_index = {name: index for index, name in enumerate(preferred_order)}
    return min(
        counter,
        key=lambda name: (-counter[name], order_index.get(name, len(preferred_order)), name),
    )


def arc_points_with_angles(arc, n: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t1 = np.deg2rad(arc.theta1)
    t2 = np.deg2rad(arc.theta2)
    diff = t2 - t1
    while diff > np.pi:
        diff -= 2.0 * np.pi
    while diff < -np.pi:
        diff += 2.0 * np.pi
    ang = np.linspace(t1, t1 + diff, n)
    cx, cy = arc.center
    x = cx + (arc.width / 2.0) * np.cos(ang)
    y = cy + (arc.height / 2.0) * np.sin(ang)
    return x, y, np.rad2deg(ang)


def sample_named_rim_arrays(
    theta: float,
    beta: float,
    gamma: float,
    lateral_samples: int,
    arc_samples: int,
    leg: PlotLeg | None = None,
) -> dict[str, np.ndarray]:
    """Sample named rim outer arcs as arrays for efficient grid scanning."""
    if leg is None:
        leg = PlotLeg()
    half_w = RobotParams.WHEEL_THICKNESS / 2.0
    laterals = np.linspace(-half_w, half_w, lateral_samples)
    chunks = {
        "point_index": [],
        "surface_name": [],
        "surface_group": [],
        "surface_description": [],
        "contact_state": [],
        "lateral_index": [],
        "arc_sample_index": [],
        "arc_angle_deg": [],
        "lateral_m": [],
        "surface_center_x_m": [],
        "surface_center_y_2d_m": [],
        "local_xyz": [],
        "display_xyz": [],
    }
    point_index = 0

    leg.forward(theta, beta, vector=False)
    for lateral_index, lateral_m in enumerate(laterals):
        leg.leg_shape.get_shape(np.array([0.0, 0.0]), tyre_offset=leg.tyre_offset_at_w(lateral_m))

        for surface_name, surface_info in RIM_SURFACES.items():
            rim_obj = getattr(leg.leg_shape, surface_name, None)
            if rim_obj is None or not hasattr(rim_obj, "arc"):
                continue

            outer_arc = rim_obj.arc[1]
            xs, ys, arc_angles = arc_points_with_angles(outer_arc, arc_samples)
            surface_center_x_m = float(outer_arc.center[0])
            surface_center_y_2d_m = float(outer_arc.center[1])
            sample_count = len(xs)
            local_xyz = np.column_stack(
                [xs, ys, np.full(sample_count, float(lateral_m), dtype=float)]
            )
            display_xyz = to_display_xyz(local_xyz, gamma)
            contact_states = contact_states_for_surface_samples(surface_name, sample_count)

            chunks["point_index"].append(np.arange(point_index, point_index + sample_count))
            chunks["surface_name"].append(np.full(sample_count, surface_name, dtype=object))
            chunks["surface_group"].append(contact_states.copy())
            chunks["surface_description"].append(
                np.full(sample_count, surface_info["description"], dtype=object)
            )
            chunks["contact_state"].append(contact_states.astype(object))
            chunks["lateral_index"].append(np.full(sample_count, lateral_index, dtype=int))
            chunks["arc_sample_index"].append(np.arange(sample_count, dtype=int))
            chunks["arc_angle_deg"].append(arc_angles.astype(float))
            chunks["lateral_m"].append(np.full(sample_count, float(lateral_m), dtype=float))
            chunks["surface_center_x_m"].append(
                np.full(sample_count, surface_center_x_m, dtype=float)
            )
            chunks["surface_center_y_2d_m"].append(
                np.full(sample_count, surface_center_y_2d_m, dtype=float)
            )
            chunks["local_xyz"].append(local_xyz)
            chunks["display_xyz"].append(display_xyz)
            point_index += sample_count

    return {
        key: np.concatenate(value) if key not in {"local_xyz", "display_xyz"} else np.vstack(value)
        for key, value in chunks.items()
    }


def rim_sample_record(samples: dict[str, np.ndarray], index: int) -> dict:
    contact_state = str(samples["contact_state"][index])
    state_info = state_metadata(contact_state)
    local_xyz = samples["local_xyz"][index]
    display_xyz = samples["display_xyz"][index]
    return {
        "point_index": int(samples["point_index"][index]),
        "surface_name": str(samples["surface_name"][index]),
        "surface_group": str(samples["surface_group"][index]),
        "surface_description": str(samples["surface_description"][index]),
        "contact_state": contact_state,
        "contact_state_id": state_info["state_id"],
        "contact_state_label": state_info["label"],
        "lateral_index": int(samples["lateral_index"][index]),
        "arc_sample_index": int(samples["arc_sample_index"][index]),
        "alpha_deg": np.nan,
        "arc_angle_deg": float(samples["arc_angle_deg"][index]),
        "lateral_m": float(samples["lateral_m"][index]),
        "surface_center_x_m": float(samples["surface_center_x_m"][index]),
        "surface_center_y_2d_m": float(samples["surface_center_y_2d_m"][index]),
        "local_x_m": float(local_xyz[0]),
        "local_y_2d_m": float(local_xyz[1]),
        "local_lateral_m": float(local_xyz[2]),
        "x_m": float(display_xyz[0]),
        "y_m": float(display_xyz[1]),
        "z_m": float(display_xyz[2]),
    }


def sample_named_rim_points(
    theta: float,
    beta: float,
    gamma: float,
    lateral_samples: int,
    arc_samples: int,
    leg: PlotLeg | None = None,
) -> list[dict]:
    """Sample only named rim outer arcs from the current PlotLeg geometry."""
    samples = sample_named_rim_arrays(theta, beta, gamma, lateral_samples, arc_samples, leg=leg)
    return [rim_sample_record(samples, index) for index in range(len(samples["point_index"]))]


def compute_pose_rim_contacts(
    theta: float,
    beta: float,
    gamma: float,
    contact_z_tol: float,
    lateral_samples: int,
    arc_samples: int,
    leg: PlotLeg | None = None,
) -> dict:
    samples = sample_named_rim_arrays(theta, beta, gamma, lateral_samples, arc_samples, leg=leg)
    points = samples["display_xyz"]
    lowest_idx = int(np.argmin(points[:, 2]))
    z_min = float(points[lowest_idx, 2])
    candidate_indices = np.flatnonzero(points[:, 2] - z_min <= contact_z_tol)
    candidates = [rim_sample_record(samples, int(idx)) for idx in candidate_indices]
    candidate_points = points[candidate_indices]
    representative = candidate_points.mean(axis=0)

    candidate_surface_names = [str(samples["surface_name"][idx]) for idx in candidate_indices]
    candidate_surface_groups = [str(samples["surface_group"][idx]) for idx in candidate_indices]
    candidate_states = [str(samples["contact_state"][idx]) for idx in candidate_indices]
    surface_counts = Counter(candidate_surface_names)
    group_counts = Counter(candidate_surface_groups)
    state_counts = Counter(candidate_states)
    dominant_surface = surface_counts.most_common(1)[0][0]
    dominant_group = group_counts.most_common(1)[0][0]
    dominant_state = most_common_ordered(state_counts, CONTACT_STATE_ORDER)
    primary_state = str(samples["contact_state"][lowest_idx])
    primary_state_info = state_metadata(primary_state)
    dominant_state_info = state_metadata(dominant_state)
    contact_states = ordered_names(set(state_counts), CONTACT_STATE_ORDER)
    mins = candidate_points.min(axis=0)
    maxs = candidate_points.max(axis=0)
    spans = maxs - mins

    return {
        "theta": float(theta),
        "beta": float(beta),
        "gamma": float(gamma),
        "theta_deg": float(np.rad2deg(theta)),
        "beta_deg": float(np.rad2deg(beta)),
        "gamma_deg": float(np.rad2deg(gamma)),
        "z_min": z_min,
        "lowest_point_x": float(points[lowest_idx, 0]),
        "lowest_point_y": float(points[lowest_idx, 1]),
        "lowest_point_z": float(points[lowest_idx, 2]),
        "lowest_surface_name": str(samples["surface_name"][lowest_idx]),
        "lowest_surface_group": str(samples["surface_group"][lowest_idx]),
        "lowest_contact_state": primary_state,
        "lowest_contact_state_id": primary_state_info["state_id"],
        "lowest_contact_state_label": primary_state_info["label"],
        "contact_candidate_count": int(len(candidates)),
        "contact_surface_names": ";".join(sorted(surface_counts)),
        "contact_surface_groups": ";".join(sorted(group_counts)),
        "contact_states": ";".join(contact_states),
        "dominant_surface_name": dominant_surface,
        "dominant_surface_group": dominant_group,
        "dominant_contact_state": dominant_state,
        "dominant_contact_state_id": dominant_state_info["state_id"],
        "dominant_contact_state_label": dominant_state_info["label"],
        "primary_contact_state": primary_state,
        "primary_contact_state_id": primary_state_info["state_id"],
        "primary_contact_state_label": primary_state_info["label"],
        "representative_contact_x": float(representative[0]),
        "representative_contact_y": float(representative[1]),
        "representative_contact_z": float(representative[2]),
        "x_min": float(mins[0]),
        "x_max": float(maxs[0]),
        "x_span": float(spans[0]),
        "y_min": float(mins[1]),
        "y_max": float(maxs[1]),
        "y_span": float(spans[1]),
        "z_contact_min": float(mins[2]),
        "z_contact_max": float(maxs[2]),
        "z_span": float(spans[2]),
        "surface_counts": dict(surface_counts),
        "group_counts": dict(group_counts),
        "state_counts": dict(state_counts),
        "contact_records": candidates,
    }


def value_grid(bounds: tuple[float, float], count: int) -> np.ndarray:
    if count <= 1:
        return np.array([(bounds[0] + bounds[1]) / 2.0])
    return np.linspace(bounds[0], bounds[1], count)


def scan_parameter_grid(
    theta_bounds_deg: tuple[float, float],
    beta_bounds_deg: tuple[float, float],
    gamma_bounds_deg: tuple[float, float],
    theta_samples: int,
    beta_samples: int,
    gamma_samples: int,
    contact_z_tol: float,
    lateral_samples: int,
    arc_samples: int,
) -> tuple[list[dict], list[dict]]:
    pose_rows = []
    candidate_rows = []

    theta_values = np.deg2rad(value_grid(theta_bounds_deg, theta_samples))
    beta_values = np.deg2rad(value_grid(beta_bounds_deg, beta_samples))
    gamma_values = np.deg2rad(value_grid(gamma_bounds_deg, gamma_samples))

    pose_index = 0
    leg = PlotLeg()
    for theta in theta_values:
        for beta in beta_values:
            for gamma in gamma_values:
                result = compute_pose_rim_contacts(
                    theta,
                    beta,
                    gamma,
                    contact_z_tol=contact_z_tol,
                    lateral_samples=lateral_samples,
                    arc_samples=arc_samples,
                    leg=leg,
                )
                result["pose_index"] = pose_index
                pose_rows.append(result)
                for record in result["contact_records"]:
                    row = {
                        "pose_index": pose_index,
                        "theta_deg": result["theta_deg"],
                        "beta_deg": result["beta_deg"],
                        "gamma_deg": result["gamma_deg"],
                    }
                    row.update(record)
                    candidate_rows.append(row)
                pose_index += 1
    return pose_rows, candidate_rows


def aggregate_ranges(pose_rows: list[dict], key_name: str, all_keys: list[str]) -> list[dict]:
    rows = []
    for key in all_keys:
        matching = [row for row in pose_rows if key in row[key_name].split(";")]
        if not matching:
            rows.append(
                {
                    "name": key,
                    "observed": False,
                    "pose_count": 0,
                    "theta_min_deg": "",
                    "theta_max_deg": "",
                    "beta_min_deg": "",
                    "beta_max_deg": "",
                    "gamma_min_deg": "",
                    "gamma_max_deg": "",
                    "y_span_min_m": "",
                    "y_span_max_m": "",
                    "z_min_min_m": "",
                    "z_min_max_m": "",
                }
            )
            continue

        rows.append(
            {
                "name": key,
                "observed": True,
                "pose_count": len(matching),
                "theta_min_deg": min(row["theta_deg"] for row in matching),
                "theta_max_deg": max(row["theta_deg"] for row in matching),
                "beta_min_deg": min(row["beta_deg"] for row in matching),
                "beta_max_deg": max(row["beta_deg"] for row in matching),
                "gamma_min_deg": min(row["gamma_deg"] for row in matching),
                "gamma_max_deg": max(row["gamma_deg"] for row in matching),
                "y_span_min_m": min(row["y_span"] for row in matching),
                "y_span_max_m": max(row["y_span"] for row in matching),
                "z_min_min_m": min(row["z_min"] for row in matching),
                "z_min_max_m": max(row["z_min"] for row in matching),
            }
        )
    return rows


def write_csv(path: Path, rows: list[dict], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def contact_color_key(row: dict, color_by: str) -> str:
    if color_by == "surface":
        return row["dominant_surface_name"]
    if color_by == "group":
        return row["dominant_surface_group"]
    return row["primary_contact_state"]


def contact_color(key: str, color_by: str) -> str:
    if color_by == "surface":
        palette = SURFACE_COLORS
    elif color_by == "group":
        palette = GROUP_COLORS
    else:
        palette = CONTACT_STATE_COLORS
    return palette.get(key, "#7f7f7f")


def observed_color_keys(rows: list[dict], color_by: str) -> list[str]:
    if color_by == "surface":
        ordered_palette = SURFACE_COLORS
    elif color_by == "group":
        ordered_palette = GROUP_COLORS
    else:
        ordered_palette = CONTACT_STATE_COLORS

    observed = {contact_color_key(row, color_by) for row in rows}
    if color_by == "state":
        ordered = [key for key in ordered_palette]
    else:
        ordered = [key for key in ordered_palette if key in observed]
    ordered.extend(sorted(observed - set(ordered)))
    return ordered


def contact_display_label(key: str, color_by: str, short: bool = False) -> str:
    if color_by == "state":
        info = state_metadata(key)
        return info["short_label"] if short else info["label"]
    return key


def centers_to_edges(values: np.ndarray, bounds: tuple[float, float] | None = None) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if values.size == 1:
        center = float(values[0])
        if bounds is not None and bounds[0] != bounds[1]:
            return np.array([bounds[0], bounds[1]], dtype=float)
        return np.array([center - 0.5, center + 0.5], dtype=float)

    midpoints = (values[:-1] + values[1:]) / 2.0
    edges = np.empty(values.size + 1, dtype=float)
    edges[1:-1] = midpoints
    edges[0] = bounds[0] if bounds is not None else values[0] - (values[1] - values[0]) / 2.0
    edges[-1] = bounds[1] if bounds is not None else values[-1] + (values[-1] - values[-2]) / 2.0
    return edges


def rounded_lookup(values: np.ndarray) -> dict[float, int]:
    return {round(float(value), 9): index for index, value in enumerate(values)}


def sorted_unique_degrees(rows: list[dict], key: str) -> np.ndarray:
    return np.array(sorted({round(float(row[key]), 9) for row in rows}), dtype=float)


def contact_class_id_maps(rows: list[dict], color_by: str) -> tuple[dict[str, int], dict[int, str]]:
    keys = observed_color_keys(rows, color_by)
    key_to_id = {key: index + 1 for index, key in enumerate(keys)}
    id_to_key = {index: key for key, index in key_to_id.items()}
    return key_to_id, id_to_key


def contact_cmap_norm(keys: list[str], color_by: str) -> tuple[mcolors.ListedColormap, mcolors.BoundaryNorm]:
    colors = ["#f4f4f4"] + [contact_color(key, color_by) for key in keys]
    cmap = mcolors.ListedColormap(colors)
    norm = mcolors.BoundaryNorm(np.arange(len(colors) + 1) - 0.5, cmap.N)
    return cmap, norm


def luminance(hex_color: str) -> float:
    r, g, b = mcolors.to_rgb(hex_color)
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def connected_components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    visited = np.zeros(mask.shape, dtype=bool)
    components = []
    rows, cols = mask.shape
    for start_i in range(rows):
        for start_j in range(cols):
            if visited[start_i, start_j] or not mask[start_i, start_j]:
                continue
            stack = [(start_i, start_j)]
            visited[start_i, start_j] = True
            component = []
            while stack:
                i, j = stack.pop()
                component.append((i, j))
                for ni, nj in ((i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)):
                    if ni < 0 or ni >= rows or nj < 0 or nj >= cols:
                        continue
                    if visited[ni, nj] or not mask[ni, nj]:
                        continue
                    visited[ni, nj] = True
                    stack.append((ni, nj))
            components.append(component)
    return components


def label_2d_regions(
    ax,
    class_grid: np.ndarray,
    theta_values: np.ndarray,
    beta_values: np.ndarray,
    id_to_key: dict[int, str],
    color_by: str,
) -> None:
    min_cells = max(4, int(0.018 * class_grid.size))
    for class_id, key in id_to_key.items():
        if key == "non_contact_region":
            continue
        for component in connected_components(class_grid == class_id):
            if len(component) < min_cells:
                continue
            theta_center = float(np.mean([theta_values[i] for i, _ in component]))
            beta_center = float(np.mean([beta_values[j] for _, j in component]))
            text_color = "white" if luminance(contact_color(key, color_by)) < 0.45 else "black"
            stroke_color = "black" if text_color == "white" else "white"
            ax.text(
                beta_center,
                theta_center,
                contact_display_label(key, color_by, short=True),
                ha="center",
                va="center",
                fontsize=8,
                fontweight="bold",
                color=text_color,
                path_effects=[pe.withStroke(linewidth=1.8, foreground=stroke_color)],
            )


def build_2d_class_grid(
    rows: list[dict],
    color_by: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[int, str], list[str]]:
    theta_values = sorted_unique_degrees(rows, "theta_deg")
    beta_values = sorted_unique_degrees(rows, "beta_deg")
    theta_lookup = rounded_lookup(theta_values)
    beta_lookup = rounded_lookup(beta_values)
    key_to_id, id_to_key = contact_class_id_maps(rows, color_by)
    class_grid = np.zeros((theta_values.size, beta_values.size), dtype=int)

    for row in rows:
        theta_index = theta_lookup[round(float(row["theta_deg"]), 9)]
        beta_index = beta_lookup[round(float(row["beta_deg"]), 9)]
        class_grid[theta_index, beta_index] = key_to_id[contact_color_key(row, color_by)]
    return class_grid, theta_values, beta_values, id_to_key, list(key_to_id)


def build_3d_class_grid(
    rows: list[dict],
    color_by: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[int, str], list[str]]:
    theta_values = sorted_unique_degrees(rows, "theta_deg")
    beta_values = sorted_unique_degrees(rows, "beta_deg")
    gamma_values = sorted_unique_degrees(rows, "gamma_deg")
    theta_lookup = rounded_lookup(theta_values)
    beta_lookup = rounded_lookup(beta_values)
    gamma_lookup = rounded_lookup(gamma_values)
    key_to_id, id_to_key = contact_class_id_maps(rows, color_by)
    class_grid = np.zeros((beta_values.size, theta_values.size, gamma_values.size), dtype=int)

    for row in rows:
        beta_index = beta_lookup[round(float(row["beta_deg"]), 9)]
        theta_index = theta_lookup[round(float(row["theta_deg"]), 9)]
        gamma_index = gamma_lookup[round(float(row["gamma_deg"]), 9)]
        class_grid[beta_index, theta_index, gamma_index] = key_to_id[
            contact_color_key(row, color_by)
        ]
    return class_grid, theta_values, beta_values, gamma_values, id_to_key, list(key_to_id)


def legend_handles(keys: list[str], color_by: str) -> list[Patch]:
    return [
        Patch(
            facecolor=contact_color(key, color_by),
            edgecolor="black",
            linewidth=0.4,
            label=contact_display_label(key, color_by),
        )
        for key in keys
    ]


def plot_contact_map_2d(
    rows: list[dict],
    output_png: Path,
    color_by: str,
    gamma_deg: float,
    contact_z_tol: float,
    return_fig: bool = False,
) -> plt.Figure | None:
    """Plot filled theta-beta contact regions with gamma fixed/ignored."""
    output_png.parent.mkdir(parents=True, exist_ok=True)
    class_grid, theta_values, beta_values, id_to_key, keys = build_2d_class_grid(rows, color_by)
    cmap, norm = contact_cmap_norm(keys, color_by)
    theta_edges = centers_to_edges(theta_values, (float(theta_values.min()), float(theta_values.max())))
    beta_edges = centers_to_edges(beta_values, (float(beta_values.min()), float(beta_values.max())))

    fig, ax = plt.subplots(figsize=(10.8, 6.8))
    ax.pcolormesh(beta_edges, theta_edges, class_grid, cmap=cmap, norm=norm, shading="flat")
    label_2d_regions(ax, class_grid, theta_values, beta_values, id_to_key, color_by)

    ax.set_xlabel("beta [deg]")
    ax.set_ylabel("theta [deg]")
    ax.set_title(
        "2D Rim Contact Regions: beta-theta\n"
        f"gamma fixed at {gamma_deg:.2f} deg, tol={contact_z_tol:.4f} m"
    )
    ax.set_xlim(beta_edges[0], beta_edges[-1])
    ax.set_ylim(theta_edges[0], theta_edges[-1])
    ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.25)
    ax.legend(
        handles=legend_handles(keys, color_by),
        loc="center left",
        bbox_to_anchor=(1.01, 0.5),
        fontsize=8,
        title=color_by,
    )
    fig.tight_layout()
    fig.savefig(output_png, dpi=180, bbox_inches="tight")
    if return_fig:
        return fig
    plt.close(fig)
    return None


def plot_contact_map_3d(
    rows: list[dict],
    output_png: Path,
    color_by: str,
    contact_z_tol: float,
    return_fig: bool = False,
) -> plt.Figure | None:
    """Plot filled theta-beta-gamma contact regions with rim identity encoded by color."""
    output_png.parent.mkdir(parents=True, exist_ok=True)
    class_grid, theta_values, beta_values, gamma_values, _, keys = build_3d_class_grid(rows, color_by)
    beta_edges = centers_to_edges(beta_values, (float(beta_values.min()), float(beta_values.max())))
    theta_edges = centers_to_edges(theta_values, (float(theta_values.min()), float(theta_values.max())))
    gamma_edges = centers_to_edges(gamma_values, (float(gamma_values.min()), float(gamma_values.max())))
    x_edges, y_edges, z_edges = np.meshgrid(beta_edges, theta_edges, gamma_edges, indexing="ij")
    filled = class_grid > 0
    facecolors = np.zeros(class_grid.shape + (4,), dtype=float)
    for class_id, key in enumerate(keys, start=1):
        facecolors[class_grid == class_id] = mcolors.to_rgba(contact_color(key, color_by), alpha=0.82)

    fig = plt.figure(figsize=(12.0, 8.2))
    ax = fig.add_subplot(111, projection="3d")
    ax.voxels(
        x_edges,
        y_edges,
        z_edges,
        filled,
        facecolors=facecolors,
        edgecolors=(1.0, 1.0, 1.0, 0.18),
        linewidth=0.12,
        shade=False,
    )

    ax.set_xlabel("beta [deg]")
    ax.set_ylabel("theta [deg]")
    ax.set_zlabel("gamma [deg]")
    ax.set_title(f"3D Rim Contact Regions: beta-theta-gamma, tol={contact_z_tol:.4f} m")
    ax.view_init(elev=22.0, azim=-58.0)
    ax.legend(
        handles=legend_handles(keys, color_by),
        loc="upper left",
        bbox_to_anchor=(1.02, 1.0),
        fontsize=8,
        title=color_by,
    )
    fig.tight_layout()
    fig.savefig(output_png, dpi=180, bbox_inches="tight")
    if return_fig:
        return fig
    plt.close(fig)
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan theta/beta/gamma ranges for rim contact.")
    parser.add_argument("--theta-min-deg", type=float, default=RobotParams.MIN_THETA_DEG)
    parser.add_argument("--theta-max-deg", type=float, default=RobotParams.MAX_THETA_DEG)
    parser.add_argument("--beta-min-deg", type=float, default=0.0)
    parser.add_argument("--beta-max-deg", type=float, default=360.0)
    parser.add_argument("--gamma-min-deg", type=float, default=-RobotParams.GAMMA_MAX_DEG)
    parser.add_argument("--gamma-max-deg", type=float, default=RobotParams.GAMMA_MAX_DEG)
    parser.add_argument("--theta-samples", type=int, default=11)
    parser.add_argument("--beta-samples", type=int, default=37)
    parser.add_argument("--gamma-samples", type=int, default=7)
    parser.add_argument("--contact-z-tol", type=float, default=CONTACT_Z_TOL)
    parser.add_argument("--lateral-samples", type=int, default=5)
    parser.add_argument("--arc-samples", type=int, default=25)
    parser.add_argument(
        "--color-by",
        choices=["state", "group", "surface"],
        default="state",
        help="Color contact maps by paper-style state, semantic rim group, or individual surface.",
    )
    parser.add_argument(
        "--map-2d-gamma-deg",
        type=float,
        default=0.0,
        help="Gamma value used for the theta-beta 2D contact map.",
    )
    parser.add_argument(
        "--map-2d-theta-samples",
        type=int,
        default=31,
        help="Theta samples for the filled 2D map.",
    )
    parser.add_argument(
        "--map-2d-beta-samples",
        type=int,
        default=73,
        help="Beta samples for the filled 2D map.",
    )
    parser.add_argument(
        "--pose-csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "rim_contact_scan_pose_contacts.csv",
    )
    parser.add_argument(
        "--surface-ranges-csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "rim_contact_scan_surface_ranges.csv",
    )
    parser.add_argument(
        "--group-ranges-csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "rim_contact_scan_group_ranges.csv",
    )
    parser.add_argument(
        "--state-ranges-csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "rim_contact_scan_state_ranges.csv",
    )
    parser.add_argument(
        "--candidate-csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "rim_contact_scan_candidates.csv",
    )
    parser.add_argument(
        "--map-2d-png",
        type=Path,
        default=DEFAULT_FIGURE_DIR / "rim_contact_map_2d_theta_beta.png",
    )
    parser.add_argument(
        "--map-3d-png",
        type=Path,
        default=DEFAULT_FIGURE_DIR / "rim_contact_map_3d_theta_beta_gamma.png",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    pose_rows, candidate_rows = scan_parameter_grid(
        theta_bounds_deg=(args.theta_min_deg, args.theta_max_deg),
        beta_bounds_deg=(args.beta_min_deg, args.beta_max_deg),
        gamma_bounds_deg=(args.gamma_min_deg, args.gamma_max_deg),
        theta_samples=args.theta_samples,
        beta_samples=args.beta_samples,
        gamma_samples=args.gamma_samples,
        contact_z_tol=args.contact_z_tol,
        lateral_samples=args.lateral_samples,
        arc_samples=args.arc_samples,
    )
    map_2d_theta_samples = args.map_2d_theta_samples or args.theta_samples
    map_2d_beta_samples = args.map_2d_beta_samples or args.beta_samples
    map_2d_rows, _ = scan_parameter_grid(
        theta_bounds_deg=(args.theta_min_deg, args.theta_max_deg),
        beta_bounds_deg=(args.beta_min_deg, args.beta_max_deg),
        gamma_bounds_deg=(args.map_2d_gamma_deg, args.map_2d_gamma_deg),
        theta_samples=map_2d_theta_samples,
        beta_samples=map_2d_beta_samples,
        gamma_samples=1,
        contact_z_tol=args.contact_z_tol,
        lateral_samples=args.lateral_samples,
        arc_samples=args.arc_samples,
    )

    pose_fields = [
        "pose_index",
        "theta_deg",
        "beta_deg",
        "gamma_deg",
        "z_min",
        "lowest_point_x",
        "lowest_point_y",
        "lowest_point_z",
        "lowest_surface_name",
        "lowest_surface_group",
        "lowest_contact_state",
        "lowest_contact_state_id",
        "lowest_contact_state_label",
        "contact_candidate_count",
        "contact_surface_names",
        "contact_surface_groups",
        "contact_states",
        "dominant_surface_name",
        "dominant_surface_group",
        "dominant_contact_state",
        "dominant_contact_state_id",
        "dominant_contact_state_label",
        "primary_contact_state",
        "primary_contact_state_id",
        "primary_contact_state_label",
        "representative_contact_x",
        "representative_contact_y",
        "representative_contact_z",
        "x_span",
        "y_span",
        "z_span",
    ]
    candidate_fields = [
        "pose_index",
        "theta_deg",
        "beta_deg",
        "gamma_deg",
        "point_index",
        "surface_name",
        "surface_group",
        "contact_state",
        "contact_state_id",
        "contact_state_label",
        "lateral_index",
        "arc_sample_index",
        "alpha_deg",
        "arc_angle_deg",
        "lateral_m",
        "surface_center_x_m",
        "surface_center_y_2d_m",
        "local_x_m",
        "local_y_2d_m",
        "local_lateral_m",
        "x_m",
        "y_m",
        "z_m",
    ]
    range_fields = [
        "name",
        "observed",
        "pose_count",
        "theta_min_deg",
        "theta_max_deg",
        "beta_min_deg",
        "beta_max_deg",
        "gamma_min_deg",
        "gamma_max_deg",
        "y_span_min_m",
        "y_span_max_m",
        "z_min_min_m",
        "z_min_max_m",
    ]

    surface_names = list(RIM_SURFACES.keys())
    group_names = sorted({surface["group"] for surface in RIM_SURFACES.values()})
    state_names = CONTACT_STATE_ORDER
    surface_range_rows = aggregate_ranges(pose_rows, "contact_surface_names", surface_names)
    group_range_rows = aggregate_ranges(pose_rows, "contact_surface_groups", group_names)
    state_range_rows = aggregate_ranges(pose_rows, "contact_states", state_names)

    write_csv(args.pose_csv, pose_rows, pose_fields)
    write_csv(args.candidate_csv, candidate_rows, candidate_fields)
    write_csv(args.surface_ranges_csv, surface_range_rows, range_fields)
    write_csv(args.group_ranges_csv, group_range_rows, range_fields)
    write_csv(args.state_ranges_csv, state_range_rows, range_fields)
    plot_contact_map_2d(
        map_2d_rows,
        output_png=args.map_2d_png,
        color_by=args.color_by,
        gamma_deg=args.map_2d_gamma_deg,
        contact_z_tol=args.contact_z_tol,
    )
    plot_contact_map_3d(
        pose_rows,
        output_png=args.map_3d_png,
        color_by=args.color_by,
        contact_z_tol=args.contact_z_tol,
    )

    observed_groups = [row["name"] for row in group_range_rows if row["observed"]]
    unobserved_groups = [row["name"] for row in group_range_rows if not row["observed"]]
    observed_states = [state_metadata(row["name"])["label"] for row in state_range_rows if row["observed"]]
    unobserved_states = [
        state_metadata(row["name"])["label"] for row in state_range_rows if not row["observed"]
    ]
    print(f"Scanned poses: {len(pose_rows)}")
    print(f"2D map poses: {len(map_2d_rows)}")
    print(f"Contact candidate records: {len(candidate_rows)}")
    print(f"Observed groups: {', '.join(observed_groups) if observed_groups else 'none'}")
    print(f"Unobserved groups: {', '.join(unobserved_groups) if unobserved_groups else 'none'}")
    print(f"Observed states: {', '.join(observed_states) if observed_states else 'none'}")
    print(f"Unobserved states: {', '.join(unobserved_states) if unobserved_states else 'none'}")
    print(f"Saved pose contacts: {args.pose_csv}")
    print(f"Saved candidate contacts: {args.candidate_csv}")
    print(f"Saved surface ranges: {args.surface_ranges_csv}")
    print(f"Saved group ranges: {args.group_ranges_csv}")
    print(f"Saved state ranges: {args.state_ranges_csv}")
    print(f"Saved 2D contact map: {args.map_2d_png}")
    print(f"Saved 3D contact map: {args.map_3d_png}")


if __name__ == "__main__":
    main()
