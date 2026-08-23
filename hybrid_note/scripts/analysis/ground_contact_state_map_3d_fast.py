"""Fast theta-beta state map from 3D geometry at a fixed gamma.

This is the accelerated counterpart of ``ground_contact_state_map_3d.py``.
The 3D rim surface is built once per theta and lateral sample.  Because beta is
a rigid rotation in the 2D leg plane, all requested beta values are evaluated
as NumPy arrays instead of rebuilding the model for every map cell.

The result remains a 2D theta-beta map for one selected gamma.  PNG and compact
NPZ outputs are written by default; the multi-million-row CSV is optional.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "legwheel_matplotlib"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

NOTE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = NOTE_ROOT.parent
KINEMATICS_DIR = NOTE_ROOT / "scripts" / "kinematics"
ANALYSIS_DIR = NOTE_ROOT / "scripts" / "analysis"
for path in [PROJECT_ROOT, KINEMATICS_DIR, ANALYSIS_DIR]:
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from legwheel.config import RobotParams  # noqa: E402
from legwheel.visualization.plot_leg import PlotLeg  # noqa: E402
from ground_contact_pose_3d import LATERAL_SAMPLES  # noqa: E402
from ground_contact_single_pose import (  # noqa: E402
    CONTACT_HEIGHT_TOL,
    CONTACT_STATE_INFO,
    CONTACT_STATE_ORDER,
    REFERENCE_POINTS,
    RIM_ARC_SAMPLES,
    RIM_SURFACES,
    as_xy,
)
from ground_contact_state_map import state_to_id_map, value_grid  # noqa: E402
from ground_contact_state_map_fast import (  # noqa: E402
    ArcPrimitive,
    PointPrimitive,
    SURFACE_NAMES,
    SURFACE_TO_ID,
    _arc_angle_parameters,
    _minimum_arc_samples,
    _surface_state_ids,
    detect_class_boundaries_fast,
    plot_contact_state_map_fast,
)

DEFAULT_DATA_DIR = NOTE_ROOT / "outputs" / "data" / "analysis"
DEFAULT_FIGURE_DIR = NOTE_ROOT / "outputs" / "figures" / "analysis"
GEOMETRY_TYPES_3D = ("rim_surface", "reference_point")
LATERAL_INVARIANT_SURFACES = {
    "upper_rim_l",
    "upper_rim_r",
    "lower_rim_l",
    "lower_rim_r",
}


@dataclass(frozen=True)
class Primitive3D:
    geometry: ArcPrimitive | PointPrimitive
    lateral: float


def build_theta_primitives_3d(
    leg: PlotLeg,
    theta: float,
    gamma: float,
    arc_samples: int,
    lateral_samples: int,
    include_reference_points: bool,
    state_to_id: dict[str, int],
    prune_lateral_invariant: bool = True,
) -> list[Primitive3D]:
    if arc_samples < 2:
        raise ValueError("arc_samples must be at least 2")
    if lateral_samples < 1:
        raise ValueError("lateral_samples must be at least 1")

    leg.forward(theta, 0.0, vector=False)
    half_width = RobotParams.WHEEL_THICKNESS / 2.0
    laterals = np.linspace(-half_width, half_width, lateral_samples)
    # Structural rim cross-sections do not depend on tyre_offset.  For them,
    # z = sin(gamma) * lateral + constant, so only one lateral can be lowest.
    structural_lateral_index = 0 if np.sin(gamma) >= 0.0 else lateral_samples - 1
    primitives: list[Primitive3D] = []

    # Preserve the original record order: lateral first, then named surface.
    for lateral_index, lateral in enumerate(laterals):
        tyre_offset = leg.tyre_offset_at_w(float(lateral))
        leg.leg_shape.get_shape(np.array([0.0, 0.0]), tyre_offset=tyre_offset)
        for surface_name in RIM_SURFACES:
            if (
                prune_lateral_invariant
                and surface_name in LATERAL_INVARIANT_SURFACES
                and lateral_index != structural_lateral_index
            ):
                continue
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
                Primitive3D(
                    geometry=ArcPrimitive(
                        surface_id=SURFACE_TO_ID[surface_name],
                        center_x=float(outer_arc.center[0]),
                        center_y=float(outer_arc.center[1]),
                        radius=radius_x,
                        start_angle=start,
                        angle_step=angle_step,
                        state_ids=_surface_state_ids(surface_name, arc_samples),
                    ),
                    lateral=float(lateral),
                )
            )

    if include_reference_points:
        for point_name, point_info in REFERENCE_POINTS.items():
            if point_info["leg_attr"] is None:
                xy = np.array([0.0, 0.0])
            else:
                xy = as_xy(getattr(leg, point_info["leg_attr"]))
            primitives.append(
                Primitive3D(
                    geometry=PointPrimitive(
                        surface_id=SURFACE_TO_ID[point_name],
                        x=float(xy[0]),
                        y=float(xy[1]),
                        state_id=state_to_id[point_info["contact_state"]],
                    ),
                    lateral=0.0,
                )
            )
    return primitives


def evaluate_theta_primitives_3d(
    primitives: list[Primitive3D],
    beta: np.ndarray,
    gamma: float,
) -> dict[str, np.ndarray]:
    sin_beta = np.sin(beta)
    cos_beta = np.cos(beta)
    sin_gamma = float(np.sin(gamma))
    cos_gamma = float(np.cos(gamma))
    count = beta.size
    height = np.full(count, np.inf)
    point_x = np.full(count, np.nan)
    point_y = np.full(count, np.nan)
    local_y = np.full(count, np.nan)
    lateral_values = np.full(count, np.nan)
    state_ids = np.full(count, -1, dtype=np.int8)
    surface_ids = np.full(count, -1, dtype=np.int8)
    geometry_type_ids = np.full(count, -1, dtype=np.int8)
    sample_indices = np.full(count, -1, dtype=np.int32)

    for primitive_3d in primitives:
        primitive = primitive_3d.geometry
        lateral = primitive_3d.lateral
        if isinstance(primitive, ArcPrimitive):
            if cos_gamma == 0.0:
                indices = np.zeros(count, dtype=np.int32)
                world_angle = primitive.start_angle + beta
                center_x = cos_beta * primitive.center_x - sin_beta * primitive.center_y
                center_y = sin_beta * primitive.center_x + cos_beta * primitive.center_y
                x = center_x + primitive.radius * np.cos(world_angle)
                y_2d = center_y + primitive.radius * np.sin(world_angle)
                primitive_state_ids = np.full(count, primitive.state_ids[0], dtype=np.int8)
            else:
                objective_sign = 1.0 if cos_gamma > 0.0 else -1.0
                target_angle = -objective_sign * np.pi / 2.0
                x, y_2d, indices, primitive_state_ids = _minimum_arc_samples(
                    primitive,
                    beta,
                    sin_beta,
                    cos_beta,
                    target_world_angle=target_angle,
                    objective_sign=objective_sign,
                )
            geometry_type_id = 0
        else:
            x = cos_beta * primitive.x - sin_beta * primitive.y
            y_2d = sin_beta * primitive.x + cos_beta * primitive.y
            indices = np.full(count, -1, dtype=np.int32)
            primitive_state_ids = np.full(count, primitive.state_id, dtype=np.int8)
            geometry_type_id = 1

        y_display = cos_gamma * lateral - sin_gamma * y_2d
        z = sin_gamma * lateral + cos_gamma * y_2d
        replace = z < height
        height[replace] = z[replace]
        point_x[replace] = x[replace]
        point_y[replace] = y_display[replace]
        local_y[replace] = y_2d[replace]
        lateral_values[replace] = lateral
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
        "lowest_point_y": point_y,
        "lowest_point_z": height,
        "local_y_2d": local_y,
        "lateral": lateral_values,
        "height_min": height,
    }


def generate_contact_state_map_3d_fast(
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
    theta_values = value_grid(theta_min_deg, theta_max_deg, theta_step_deg)
    beta_values = value_grid(beta_min_deg, beta_max_deg, beta_step_deg)
    beta = np.deg2rad(beta_values)
    gamma = float(np.deg2rad(gamma_deg))
    state_to_id, id_to_state = state_to_id_map()
    shape = (theta_values.size, beta_values.size)

    class_grid = np.full(shape, -1, dtype=np.int8)
    surface_id_grid = np.full(shape, -1, dtype=np.int8)
    geometry_type_id_grid = np.full(shape, -1, dtype=np.int8)
    arc_sample_index_grid = np.full(shape, -1, dtype=np.int32)
    lowest_point_x_grid = np.full(shape, np.nan)
    lowest_point_y_grid = np.full(shape, np.nan)
    lowest_point_z_grid = np.full(shape, np.nan)
    local_y_2d_grid = np.full(shape, np.nan)
    lateral_grid = np.full(shape, np.nan)

    leg = PlotLeg()
    for theta_index, theta_deg in enumerate(theta_values):
        primitives = build_theta_primitives_3d(
            leg=leg,
            theta=np.deg2rad(theta_deg),
            gamma=gamma,
            arc_samples=arc_samples,
            lateral_samples=lateral_samples,
            include_reference_points=include_reference_points,
            state_to_id=state_to_id,
        )
        row = evaluate_theta_primitives_3d(primitives, beta, gamma)
        class_grid[theta_index] = row["class_ids"]
        surface_id_grid[theta_index] = row["surface_ids"]
        geometry_type_id_grid[theta_index] = row["geometry_type_ids"]
        arc_sample_index_grid[theta_index] = row["arc_sample_indices"]
        lowest_point_x_grid[theta_index] = row["lowest_point_x"]
        lowest_point_y_grid[theta_index] = row["lowest_point_y"]
        lowest_point_z_grid[theta_index] = row["lowest_point_z"]
        local_y_2d_grid[theta_index] = row["local_y_2d"]
        lateral_grid[theta_index] = row["lateral"]

    state_names = np.asarray(CONTACT_STATE_ORDER, dtype=object)
    return {
        "theta_values": theta_values,
        "beta_values": beta_values,
        "gamma_deg": float(gamma_deg),
        "class_grid": class_grid,
        "state_grid": state_names[class_grid],
        "surface_id_grid": surface_id_grid,
        "surface_names": SURFACE_NAMES,
        "geometry_type_id_grid": geometry_type_id_grid,
        "geometry_types": GEOMETRY_TYPES_3D,
        "arc_sample_index_grid": arc_sample_index_grid,
        "lowest_point_x_grid": lowest_point_x_grid,
        "lowest_point_y_grid": lowest_point_y_grid,
        "lowest_point_z_grid": lowest_point_z_grid,
        "height_min_grid": lowest_point_z_grid,
        "local_y_2d_grid": local_y_2d_grid,
        "lateral_grid": lateral_grid,
        "state_to_id": state_to_id,
        "id_to_state": id_to_state,
        "contact_height_tol": float(contact_height_tol),
        "arc_samples": int(arc_samples),
        "lateral_samples": int(lateral_samples),
        "include_reference_points": bool(include_reference_points),
        "pose_count": int(theta_values.size * beta_values.size),
    }


def plot_contact_state_map_3d_fast(
    map_data: dict,
    boundary_segments: np.ndarray,
    output_png: Path,
    dpi: int = 240,
    return_fig: bool = False,
) -> plt.Figure | None:
    return plot_contact_state_map_fast(
        map_data,
        boundary_segments,
        output_png,
        dpi=dpi,
        return_fig=return_fig,
        title=(
            "3D Ground Contact State Map: theta-beta "
            f"at gamma={map_data['gamma_deg']:.2f} deg (fast)"
        ),
    )


def save_contact_map_npz_3d_fast(path: Path, map_data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        theta_values=map_data["theta_values"],
        beta_values=map_data["beta_values"],
        gamma_deg=np.array(map_data["gamma_deg"]),
        class_grid=map_data["class_grid"],
        contact_state_order=np.asarray(CONTACT_STATE_ORDER, dtype="U"),
        surface_id_grid=map_data["surface_id_grid"],
        surface_names=np.asarray(map_data["surface_names"], dtype="U"),
        geometry_type_id_grid=map_data["geometry_type_id_grid"],
        geometry_types=np.asarray(map_data["geometry_types"], dtype="U"),
        arc_sample_index_grid=map_data["arc_sample_index_grid"],
        lowest_point_x_grid=map_data["lowest_point_x_grid"],
        lowest_point_y_grid=map_data["lowest_point_y_grid"],
        lowest_point_z_grid=map_data["lowest_point_z_grid"],
        local_y_2d_grid=map_data["local_y_2d_grid"],
        lateral_grid=map_data["lateral_grid"],
        contact_height_tol=np.array(map_data["contact_height_tol"]),
        arc_samples=np.array(map_data["arc_samples"]),
        lateral_samples=np.array(map_data["lateral_samples"]),
        include_reference_points=np.array(map_data["include_reference_points"]),
    )


def save_contact_map_csv_3d_fast(path: Path, map_data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "theta_deg",
        "beta_deg",
        "gamma_deg",
        "lowest_contact_state",
        "lowest_contact_state_id",
        "lowest_surface_name",
        "lowest_geometry_type",
        "arc_sample_index",
        "lowest_lateral_m",
        "lowest_local_y_2d_m",
        "lowest_point_x",
        "lowest_point_y",
        "lowest_point_z",
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
                z = float(map_data["lowest_point_z_grid"][theta_index, beta_index])
                writer.writerow(
                    (
                        float(theta_deg),
                        float(beta_deg),
                        float(map_data["gamma_deg"]),
                        state,
                        CONTACT_STATE_INFO[state]["state_id"],
                        map_data["surface_names"][surface_id],
                        map_data["geometry_types"][geometry_type_id],
                        int(map_data["arc_sample_index_grid"][theta_index, beta_index]),
                        float(map_data["lateral_grid"][theta_index, beta_index]),
                        float(map_data["local_y_2d_grid"][theta_index, beta_index]),
                        float(map_data["lowest_point_x_grid"][theta_index, beta_index]),
                        float(map_data["lowest_point_y_grid"][theta_index, beta_index]),
                        z,
                        z,
                    )
                )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a vectorized 3D ground-contact map at fixed gamma."
    )
    parser.add_argument("--theta-min-deg", type=float, default=17.0)
    parser.add_argument("--theta-max-deg", type=float, default=160.0)
    parser.add_argument("--theta-step-deg", type=float, default=0.1)
    parser.add_argument("--beta-min-deg", type=float, default=-180.0)
    parser.add_argument("--beta-max-deg", type=float, default=180.0)
    parser.add_argument("--beta-step-deg", type=float, default=0.1)
    parser.add_argument("--gamma-deg", type=float, default=10.0)
    parser.add_argument("--contact-height-tol", type=float, default=CONTACT_HEIGHT_TOL)
    parser.add_argument("--arc-samples", type=int, default=181)
    parser.add_argument("--lateral-samples", type=int, default=11)
    parser.add_argument("--include-reference-points", action="store_true")
    parser.add_argument("--dpi", type=int, default=240)
    parser.add_argument(
        "--npz",
        type=Path,
        default=DEFAULT_DATA_DIR / "ground_contact_state_map_3d_theta_beta_gamma_fast.npz",
    )
    parser.add_argument(
        "--png",
        type=Path,
        default=DEFAULT_FIGURE_DIR / "ground_contact_state_map_3d_theta_beta_gamma_fast.png",
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
    map_data = generate_contact_state_map_3d_fast(
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
    generated_at = time.perf_counter()
    boundaries, transition_counts = detect_class_boundaries_fast(map_data)
    save_contact_map_npz_3d_fast(args.npz, map_data)
    if args.csv is not None:
        save_contact_map_csv_3d_fast(args.csv, map_data)
    plot_contact_state_map_3d_fast(map_data, boundaries, args.png, dpi=args.dpi)
    finished_at = time.perf_counter()

    state_counts = np.bincount(map_data["class_grid"].ravel(), minlength=len(CONTACT_STATE_ORDER))
    print(f"Gamma: {args.gamma_deg:.3f} deg")
    print(f"Sampled poses: {map_data['pose_count']}")
    print(f"Theta samples: {len(map_data['theta_values'])}")
    print(f"Beta samples: {len(map_data['beta_values'])}")
    print("State counts:")
    for state, count in zip(CONTACT_STATE_ORDER, state_counts):
        print(f"  {CONTACT_STATE_INFO[state]['state_id']} {state}: {int(count)}")
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
