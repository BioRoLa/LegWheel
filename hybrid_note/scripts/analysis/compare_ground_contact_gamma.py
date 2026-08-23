"""Compare theta-beta ground-contact maps across multiple gamma values.

All gamma values reuse the same sampled theta/lateral geometry.  For the usual
range where cos(gamma) has the same sign, every rim arc is minimized in the 2D
leg plane only once and then projected to every requested gamma value.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "legwheel_matplotlib"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import PercentFormatter

NOTE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = NOTE_ROOT.parent
KINEMATICS_DIR = NOTE_ROOT / "scripts" / "kinematics"
ANALYSIS_DIR = NOTE_ROOT / "scripts" / "analysis"
for path in [PROJECT_ROOT, KINEMATICS_DIR, ANALYSIS_DIR]:
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from legwheel.visualization.plot_leg import PlotLeg  # noqa: E402
from ground_contact_single_pose import (  # noqa: E402
    CONTACT_STATE_INFO,
    CONTACT_STATE_ORDER,
)
from ground_contact_state_map import centers_to_edges, state_to_id_map, value_grid  # noqa: E402
from ground_contact_state_map_fast import (  # noqa: E402
    ArcPrimitive,
    PointPrimitive,
    _minimum_arc_samples,
)
from ground_contact_state_map_3d_fast import (  # noqa: E402
    Primitive3D,
    build_theta_primitives_3d,
)
from rim_contact_parameter_scan import CONTACT_STATE_COLORS  # noqa: E402

DEFAULT_DATA_DIR = NOTE_ROOT / "outputs" / "data" / "analysis"
DEFAULT_TABLE_DIR = NOTE_ROOT / "outputs" / "tables" / "analysis"
DEFAULT_FIGURE_DIR = NOTE_ROOT / "outputs" / "figures" / "analysis"


def evaluate_theta_primitives_multi_gamma(
    primitives: list[Primitive3D],
    beta: np.ndarray,
    gamma: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return lowest state and height arrays shaped ``[gamma, beta]``."""
    sin_beta = np.sin(beta)
    cos_beta = np.cos(beta)
    sin_gamma = np.sin(gamma)
    cos_gamma = np.cos(gamma)
    objective_signs = np.where(cos_gamma >= 0.0, 1.0, -1.0)

    shape = (gamma.size, beta.size)
    height = np.full(shape, np.inf)
    class_ids = np.full(shape, -1, dtype=np.int8)

    for primitive_3d in primitives:
        primitive = primitive_3d.geometry
        lateral = primitive_3d.lateral

        if isinstance(primitive, ArcPrimitive):
            sign_results = {}
            for objective_sign in np.unique(objective_signs):
                _, y_2d, _, primitive_state_ids = _minimum_arc_samples(
                    primitive,
                    beta,
                    sin_beta,
                    cos_beta,
                    target_world_angle=-objective_sign * np.pi / 2.0,
                    objective_sign=float(objective_sign),
                )
                sign_results[float(objective_sign)] = (y_2d, primitive_state_ids)
        elif isinstance(primitive, PointPrimitive):
            y_2d = sin_beta * primitive.x + cos_beta * primitive.y
            primitive_state_ids = np.full(beta.size, primitive.state_id, dtype=np.int8)
            sign_results = {
                1.0: (y_2d, primitive_state_ids),
                -1.0: (y_2d, primitive_state_ids),
            }
        else:
            raise TypeError(f"Unsupported primitive type: {type(primitive)!r}")

        for objective_sign in np.unique(objective_signs):
            gamma_indices = np.flatnonzero(objective_signs == objective_sign)
            y_2d, primitive_state_ids = sign_results[float(objective_sign)]
            candidate_z = (
                sin_gamma[gamma_indices, None] * lateral
                + cos_gamma[gamma_indices, None] * y_2d[None, :]
            )
            current_height = height[gamma_indices]
            replace = candidate_z < current_height
            height[gamma_indices] = np.where(replace, candidate_z, current_height)
            class_ids[gamma_indices] = np.where(
                replace,
                primitive_state_ids[None, :],
                class_ids[gamma_indices],
            )

    return class_ids, height


def generate_gamma_comparison(
    gamma_values_deg: list[float] | np.ndarray,
    baseline_gamma_deg: float,
    theta_min_deg: float,
    theta_max_deg: float,
    theta_step_deg: float,
    beta_min_deg: float,
    beta_max_deg: float,
    beta_step_deg: float,
    arc_samples: int = 181,
    lateral_samples: int = 11,
    include_reference_points: bool = False,
) -> dict:
    """Generate contact maps and comparison metrics for several gamma values."""
    gamma_values = np.asarray(gamma_values_deg, dtype=float).reshape(-1)
    if gamma_values.size == 0:
        raise ValueError("gamma_values_deg must contain at least one value")
    if np.unique(gamma_values).size != gamma_values.size:
        raise ValueError("gamma_values_deg must not contain duplicates")

    baseline_matches = np.flatnonzero(np.isclose(gamma_values, baseline_gamma_deg))
    if baseline_matches.size != 1:
        raise ValueError("baseline_gamma_deg must appear exactly once in gamma_values_deg")
    baseline_index = int(baseline_matches[0])

    theta_values = value_grid(theta_min_deg, theta_max_deg, theta_step_deg)
    beta_values = value_grid(beta_min_deg, beta_max_deg, beta_step_deg)
    beta = np.deg2rad(beta_values)
    gamma = np.deg2rad(gamma_values)
    state_to_id, id_to_state = state_to_id_map()

    shape = (gamma_values.size, theta_values.size, beta_values.size)
    class_cube = np.full(shape, -1, dtype=np.int8)
    height_min_cube = np.full(shape, np.nan)
    leg = PlotLeg()

    for theta_index, theta_deg in enumerate(theta_values):
        primitives = build_theta_primitives_3d(
            leg=leg,
            theta=np.deg2rad(theta_deg),
            gamma=0.0,
            arc_samples=arc_samples,
            lateral_samples=lateral_samples,
            include_reference_points=include_reference_points,
            state_to_id=state_to_id,
            prune_lateral_invariant=False,
        )
        class_ids, heights = evaluate_theta_primitives_multi_gamma(
            primitives=primitives,
            beta=beta,
            gamma=gamma,
        )
        class_cube[:, theta_index, :] = class_ids
        height_min_cube[:, theta_index, :] = heights

    baseline_grid = class_cube[baseline_index]
    change_mask_cube = class_cube != baseline_grid[None, :, :]
    changed_cell_ratio = change_mask_cube.mean(axis=(1, 2))
    state_area_ratio = np.stack(
        [(class_cube == state_to_id[state]).mean(axis=(1, 2)) for state in CONTACT_STATE_ORDER],
        axis=1,
    )

    summary_rows = []
    for gamma_index, gamma_deg in enumerate(gamma_values):
        row = {
            "gamma_deg": float(gamma_deg),
            "baseline_gamma_deg": float(baseline_gamma_deg),
            "changed_cell_ratio": float(changed_cell_ratio[gamma_index]),
            "changed_cell_count": int(change_mask_cube[gamma_index].sum()),
            "total_cell_count": int(theta_values.size * beta_values.size),
        }
        for state_index, state in enumerate(CONTACT_STATE_ORDER):
            row[f"{state}_ratio"] = float(state_area_ratio[gamma_index, state_index])
        summary_rows.append(row)

    return {
        "gamma_values_deg": gamma_values,
        "baseline_gamma_deg": float(baseline_gamma_deg),
        "baseline_index": baseline_index,
        "theta_values": theta_values,
        "beta_values": beta_values,
        "class_cube": class_cube,
        "height_min_cube": height_min_cube,
        "change_mask_cube": change_mask_cube,
        "changed_cell_ratio": changed_cell_ratio,
        "state_area_ratio": state_area_ratio,
        "summary_rows": summary_rows,
        "state_to_id": state_to_id,
        "id_to_state": id_to_state,
        "arc_samples": int(arc_samples),
        "lateral_samples": int(lateral_samples),
        "include_reference_points": bool(include_reference_points),
        "pose_count": int(np.prod(shape)),
    }


def plot_gamma_comparison(
    comparison: dict,
    output_png: Path,
    state_colors: dict[str, str] | None = None,
    dpi: int = 180,
    return_fig: bool = False,
) -> plt.Figure | None:
    """Plot maps, baseline differences, state ratios, and changed-cell ratio."""
    colors = dict(CONTACT_STATE_COLORS if state_colors is None else state_colors)
    gamma_values = comparison["gamma_values_deg"]
    theta_values = comparison["theta_values"]
    beta_values = comparison["beta_values"]
    class_cube = comparison["class_cube"]
    change_mask_cube = comparison["change_mask_cube"]
    state_area_ratio = comparison["state_area_ratio"]
    changed_cell_ratio = comparison["changed_cell_ratio"]
    baseline_gamma = comparison["baseline_gamma_deg"]

    state_color_list = [colors[state] for state in CONTACT_STATE_ORDER]
    state_cmap = mcolors.ListedColormap(state_color_list)
    state_norm = mcolors.BoundaryNorm(
        np.arange(len(CONTACT_STATE_ORDER) + 1) - 0.5,
        state_cmap.N,
    )
    difference_cmap = mcolors.ListedColormap(["#e5e7eb", *state_color_list])
    difference_norm = mcolors.BoundaryNorm(
        np.arange(len(CONTACT_STATE_ORDER) + 2) - 0.5,
        difference_cmap.N,
    )

    beta_edges = centers_to_edges(beta_values)
    theta_edges = centers_to_edges(theta_values)
    extent = (beta_edges[0], beta_edges[-1], theta_edges[0], theta_edges[-1])
    column_count = gamma_values.size
    figure_width = max(12.0, 2.75 * column_count)
    fig = plt.figure(figsize=(figure_width, 13.0), layout="constrained")
    grid = fig.add_gridspec(4, column_count, height_ratios=(1.0, 1.0, 0.72, 0.52))

    for gamma_index, gamma_deg in enumerate(gamma_values):
        map_ax = fig.add_subplot(grid[0, gamma_index])
        map_ax.imshow(
            class_cube[gamma_index],
            origin="lower",
            interpolation="nearest",
            aspect="auto",
            extent=extent,
            cmap=state_cmap,
            norm=state_norm,
        )
        map_ax.set_title(f"gamma = {gamma_deg:g} deg", fontsize=10)
        map_ax.set_xlabel("beta [deg]")
        if gamma_index == 0:
            map_ax.set_ylabel("theta [deg]\ncontact map")
        else:
            map_ax.set_yticklabels([])

        difference_ax = fig.add_subplot(grid[1, gamma_index])
        difference_grid = np.where(
            change_mask_cube[gamma_index],
            class_cube[gamma_index] + 1,
            0,
        )
        difference_ax.imshow(
            difference_grid,
            origin="lower",
            interpolation="nearest",
            aspect="auto",
            extent=extent,
            cmap=difference_cmap,
            norm=difference_norm,
        )
        difference_ax.set_title(
            f"changed: {changed_cell_ratio[gamma_index]:.3%}",
            fontsize=10,
        )
        difference_ax.set_xlabel("beta [deg]")
        if gamma_index == 0:
            difference_ax.set_ylabel(f"theta [deg]\nvs gamma={baseline_gamma:g} deg")
        else:
            difference_ax.set_yticklabels([])

    ratio_ax = fig.add_subplot(grid[2, :])
    for state_index, state in enumerate(CONTACT_STATE_ORDER):
        ratio_ax.plot(
            gamma_values,
            state_area_ratio[:, state_index],
            color=colors[state],
            marker="o",
            linewidth=1.8,
            markersize=4.5,
            label=CONTACT_STATE_INFO[state]["label"],
        )
    ratio_ax.axvline(baseline_gamma, color="#6b7280", linestyle="--", linewidth=1.0)
    ratio_ax.set_ylabel("state area ratio")
    ratio_ax.set_xlabel("gamma [deg]")
    ratio_ax.yaxis.set_major_formatter(PercentFormatter(1.0))
    ratio_ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.45)
    ratio_ax.legend(loc="center left", bbox_to_anchor=(1.005, 0.5), fontsize=8)

    change_ax = fig.add_subplot(grid[3, :])
    change_ax.plot(
        gamma_values,
        changed_cell_ratio,
        color="#374151",
        marker="o",
        linewidth=2.0,
        markersize=5.0,
    )
    change_ax.axvline(baseline_gamma, color="#6b7280", linestyle="--", linewidth=1.0)
    change_ax.set_ylabel("changed-cell ratio")
    change_ax.set_xlabel("gamma [deg]")
    change_ax.yaxis.set_major_formatter(PercentFormatter(1.0))
    change_ax.set_ylim(0.0, max(0.001, float(changed_cell_ratio.max()) * 1.15))
    change_ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.45)

    fig.suptitle("Ground Contact Map Comparison Across Gamma", fontsize=14)

    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png, dpi=dpi, bbox_inches="tight")
    if return_fig:
        return fig
    plt.close(fig)
    return None


def save_gamma_comparison_npz(path: Path, comparison: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        gamma_values_deg=comparison["gamma_values_deg"],
        baseline_gamma_deg=np.array(comparison["baseline_gamma_deg"]),
        theta_values=comparison["theta_values"],
        beta_values=comparison["beta_values"],
        class_cube=comparison["class_cube"],
        height_min_cube=comparison["height_min_cube"],
        change_mask_cube=comparison["change_mask_cube"],
        changed_cell_ratio=comparison["changed_cell_ratio"],
        state_area_ratio=comparison["state_area_ratio"],
        contact_state_order=np.asarray(CONTACT_STATE_ORDER, dtype="U"),
        arc_samples=np.array(comparison["arc_samples"]),
        lateral_samples=np.array(comparison["lateral_samples"]),
        include_reference_points=np.array(comparison["include_reference_points"]),
    )


def save_gamma_comparison_summary_csv(path: Path, comparison: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(comparison["summary_rows"][0])
    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(comparison["summary_rows"])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare contact maps across gamma values.")
    parser.add_argument("--gamma-deg", type=float, nargs="+", default=[-20, -10, 0, 10, 20])
    parser.add_argument("--baseline-gamma-deg", type=float, default=0.0)
    parser.add_argument("--theta-min-deg", type=float, default=17.0)
    parser.add_argument("--theta-max-deg", type=float, default=160.0)
    parser.add_argument("--theta-step-deg", type=float, default=0.5)
    parser.add_argument("--beta-min-deg", type=float, default=-180.0)
    parser.add_argument("--beta-max-deg", type=float, default=180.0)
    parser.add_argument("--beta-step-deg", type=float, default=0.5)
    parser.add_argument("--arc-samples", type=int, default=181)
    parser.add_argument("--lateral-samples", type=int, default=11)
    parser.add_argument("--include-reference-points", action="store_true")
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument(
        "--png",
        type=Path,
        default=DEFAULT_FIGURE_DIR / "ground_contact_gamma_comparison.png",
    )
    parser.add_argument(
        "--npz",
        type=Path,
        default=DEFAULT_DATA_DIR / "ground_contact_gamma_comparison.npz",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "ground_contact_gamma_comparison_summary.csv",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    started = time.perf_counter()
    comparison = generate_gamma_comparison(
        gamma_values_deg=args.gamma_deg,
        baseline_gamma_deg=args.baseline_gamma_deg,
        theta_min_deg=args.theta_min_deg,
        theta_max_deg=args.theta_max_deg,
        theta_step_deg=args.theta_step_deg,
        beta_min_deg=args.beta_min_deg,
        beta_max_deg=args.beta_max_deg,
        beta_step_deg=args.beta_step_deg,
        arc_samples=args.arc_samples,
        lateral_samples=args.lateral_samples,
        include_reference_points=args.include_reference_points,
    )
    generated_at = time.perf_counter()
    save_gamma_comparison_npz(args.npz, comparison)
    save_gamma_comparison_summary_csv(args.csv, comparison)
    plot_gamma_comparison(comparison, args.png, dpi=args.dpi)
    finished_at = time.perf_counter()

    print(f"Gamma values: {comparison['gamma_values_deg'].tolist()}")
    print(f"Pose evaluations: {comparison['pose_count']}")
    for row in comparison["summary_rows"]:
        print(
            f"  gamma={row['gamma_deg']:g} deg: "
            f"changed vs baseline={row['changed_cell_ratio']:.3%}"
        )
    print(f"Geometry and classification: {generated_at - started:.3f} s")
    print(f"Total including output: {finished_at - started:.3f} s")
    print(f"Saved PNG: {args.png}")
    print(f"Saved NPZ: {args.npz}")
    print(f"Saved CSV: {args.csv}")


if __name__ == "__main__":
    main()
