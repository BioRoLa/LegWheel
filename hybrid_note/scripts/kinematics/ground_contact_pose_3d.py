"""Single-pose 3D geometric ground-contact analysis for the LegWheel.

The core function is ``compute_ground_contact_pose_3d(theta, beta, gamma)``.
Angles are in radians, matching the model code. The command-line interface
accepts degrees.

This extends the 2D contact classifier by sampling every named rim outer arc at
multiple lateral positions across the wheel width. Local points are interpreted
as ``[x, y_2d, lateral]`` and converted to display/world-like coordinates
``[x, y_lateral, z_height]`` with the same ``to_display_xyz`` helper used by the
3D plotting script.
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

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
NOTE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = NOTE_ROOT.parent
for path in [SCRIPT_DIR, PROJECT_ROOT]:
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from legwheel.config import RobotParams  # noqa: E402
from legwheel.visualization.plot_leg import PlotLeg  # noqa: E402
from ground_contact_single_pose import (  # noqa: E402
    CONTACT_HEIGHT_TOL,
    CONTACT_STATE_ORDER,
    REFERENCE_POINTS,
    RIM_ARC_SAMPLES,
    RIM_SURFACES,
    arc_points_with_angles,
    as_xy,
    dominant_contact_state,
    ordered_contact_states,
    state_metadata,
    surface_contact_state,
)

DEFAULT_TABLE_DIR = NOTE_ROOT / "outputs" / "tables" / "kinematics"
LATERAL_SAMPLES = 41


def to_display_xyz(points, gamma_rad: float = 0.0) -> np.ndarray:
    """Map local [x, y_2d, lateral] to [x, y_lateral, z_height] and roll by gamma."""
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return pts.reshape((-1, 3))

    display = pts[..., [0, 2, 1]].copy()
    lateral = display[..., 1].copy()
    vertical = display[..., 2].copy()
    c = np.cos(gamma_rad)
    s = np.sin(gamma_rad)
    display[..., 1] = c * lateral - s * vertical
    display[..., 2] = s * lateral + c * vertical
    return display


def sample_contact_geometry_points_3d(
    theta: float,
    beta: float,
    gamma: float,
    arc_samples: int = RIM_ARC_SAMPLES,
    lateral_samples: int = LATERAL_SAMPLES,
    include_reference_points: bool = False,
) -> list[dict]:
    """Sample named 3D rim outer surfaces and optional non-contact references."""
    leg = PlotLeg()
    leg.forward(theta, beta, vector=False)

    half_w = RobotParams.WHEEL_THICKNESS / 2.0
    laterals = np.linspace(-half_w, half_w, lateral_samples)
    records = []
    point_index = 0

    for lateral_index, lateral_m in enumerate(laterals):
        tyre_offset = leg.tyre_offset_at_w(float(lateral_m))
        leg.leg_shape.get_shape(np.array([0.0, 0.0]), tyre_offset=tyre_offset)

        for surface_name, surface_info in RIM_SURFACES.items():
            rim_obj = getattr(leg.leg_shape, surface_name, None)
            if rim_obj is None or not hasattr(rim_obj, "arc"):
                continue

            outer_arc = rim_obj.arc[1]
            xs, ys, arc_angles = arc_points_with_angles(outer_arc, arc_samples)
            center_x_m = float(outer_arc.center[0])
            center_y_m = float(outer_arc.center[1])

            for arc_sample_index, (x_m, y_2d_m, arc_angle_deg) in enumerate(
                zip(xs, ys, arc_angles)
            ):
                contact_state = surface_contact_state(surface_name, arc_sample_index, len(xs))
                state_info = state_metadata(contact_state)
                local_xyz = np.array([float(x_m), float(y_2d_m), float(lateral_m)])
                xyz = to_display_xyz(local_xyz, gamma)
                records.append(
                    {
                        "point_index": point_index,
                        "geometry_type": "rim_surface",
                        "surface_name": surface_name,
                        "surface_description": surface_info["description"],
                        "contact_state": contact_state,
                        "contact_state_id": state_info["state_id"],
                        "contact_state_label": state_info["label"],
                        "lateral_index": lateral_index,
                        "lateral_m": float(lateral_m),
                        "arc_sample_index": arc_sample_index,
                        "arc_angle_deg": float(arc_angle_deg),
                        "surface_center_x_m": center_x_m,
                        "surface_center_y_2d_m": center_y_m,
                        "local_x_m": float(local_xyz[0]),
                        "local_y_2d_m": float(local_xyz[1]),
                        "local_lateral_m": float(local_xyz[2]),
                        "x_m": float(xyz[0]),
                        "y_m": float(xyz[1]),
                        "z_m": float(xyz[2]),
                    }
                )
                point_index += 1

    if include_reference_points:
        leg.forward(theta, beta, vector=False)
        for point_name, point_info in REFERENCE_POINTS.items():
            if point_info["leg_attr"] is None:
                xy = np.array([0.0, 0.0])
            else:
                xy = as_xy(getattr(leg, point_info["leg_attr"]))

            contact_state = point_info["contact_state"]
            state_info = state_metadata(contact_state)
            local_xyz = np.array([float(xy[0]), float(xy[1]), 0.0])
            xyz = to_display_xyz(local_xyz, gamma)
            records.append(
                {
                    "point_index": point_index,
                    "geometry_type": "reference_point",
                    "surface_name": point_name,
                    "surface_description": point_info["description"],
                    "contact_state": contact_state,
                    "contact_state_id": state_info["state_id"],
                    "contact_state_label": state_info["label"],
                    "lateral_index": "",
                    "lateral_m": 0.0,
                    "arc_sample_index": "",
                    "arc_angle_deg": "",
                    "surface_center_x_m": "",
                    "surface_center_y_2d_m": "",
                    "local_x_m": float(local_xyz[0]),
                    "local_y_2d_m": float(local_xyz[1]),
                    "local_lateral_m": float(local_xyz[2]),
                    "x_m": float(xyz[0]),
                    "y_m": float(xyz[1]),
                    "z_m": float(xyz[2]),
                }
            )
            point_index += 1

    return records


def compute_ground_contact_pose_3d(
    theta: float,
    beta: float,
    gamma: float,
    contact_height_tol: float = CONTACT_HEIGHT_TOL,
    arc_samples: int = RIM_ARC_SAMPLES,
    lateral_samples: int = LATERAL_SAMPLES,
    include_reference_points: bool = False,
) -> dict:
    """Compute 3D geometric ground-contact candidates for one theta-beta-gamma pose."""
    surface_records = sample_contact_geometry_points_3d(
        theta=theta,
        beta=beta,
        gamma=gamma,
        arc_samples=arc_samples,
        lateral_samples=lateral_samples,
        include_reference_points=include_reference_points,
    )
    surface_points = np.array([[r["x_m"], r["y_m"], r["z_m"]] for r in surface_records])

    lowest_index = int(np.argmin(surface_points[:, 2]))
    z_min = float(surface_points[lowest_index, 2])
    candidate_indices = np.flatnonzero(surface_points[:, 2] - z_min <= contact_height_tol)
    contact_points = surface_points[candidate_indices]
    contact_records = [surface_records[int(index)] for index in candidate_indices]
    representative_contact = contact_points.mean(axis=0)

    mins = contact_points.min(axis=0)
    maxs = contact_points.max(axis=0)
    spans = maxs - mins
    lowest_point = surface_points[lowest_index]
    lowest_record = surface_records[lowest_index]

    state_counts = Counter(record["contact_state"] for record in contact_records)
    surface_counts = Counter(record["surface_name"] for record in contact_records)
    geometry_type_counts = Counter(record["geometry_type"] for record in contact_records)
    states = ordered_contact_states(set(state_counts))
    dominant_state = dominant_contact_state(state_counts)
    dominant_state_info = state_metadata(dominant_state)

    return {
        "theta": float(theta),
        "beta": float(beta),
        "gamma": float(gamma),
        "theta_deg": float(np.rad2deg(theta)),
        "beta_deg": float(np.rad2deg(beta)),
        "gamma_deg": float(np.rad2deg(gamma)),
        "contact_height_tol": float(contact_height_tol),
        "arc_samples": int(arc_samples),
        "lateral_samples": int(lateral_samples),
        "include_reference_points": bool(include_reference_points),
        "surface_points": surface_points,
        "surface_records": surface_records,
        "height_min": z_min,
        "z_min": z_min,
        "lowest_point": lowest_point,
        "lowest_point_x": float(lowest_point[0]),
        "lowest_point_y": float(lowest_point[1]),
        "lowest_point_z": float(lowest_point[2]),
        "lowest_record": lowest_record,
        "lowest_surface_name": lowest_record["surface_name"],
        "lowest_geometry_type": lowest_record["geometry_type"],
        "lowest_contact_state": lowest_record["contact_state"],
        "lowest_contact_state_id": lowest_record["contact_state_id"],
        "lowest_contact_state_label": lowest_record["contact_state_label"],
        "contact_points": contact_points,
        "contact_records": contact_records,
        "number_of_contact_points": int(len(contact_points)),
        "contact_states": ";".join(states),
        "contact_surface_names": ";".join(sorted(surface_counts)),
        "contact_geometry_types": ";".join(sorted(geometry_type_counts)),
        "dominant_contact_state": dominant_state,
        "dominant_contact_state_id": dominant_state_info["state_id"],
        "dominant_contact_state_label": dominant_state_info["label"],
        "representative_contact": representative_contact,
        "representative_contact_x": float(representative_contact[0]),
        "representative_contact_y": float(representative_contact[1]),
        "representative_contact_z": float(representative_contact[2]),
        "x_min": float(mins[0]),
        "x_max": float(maxs[0]),
        "x_span": float(spans[0]),
        "y_min": float(mins[1]),
        "y_max": float(maxs[1]),
        "y_span": float(spans[1]),
        "z_contact_min": float(mins[2]),
        "z_contact_max": float(maxs[2]),
        "z_span": float(spans[2]),
        "state_counts": dict(state_counts),
        "surface_counts": dict(surface_counts),
        "geometry_type_counts": dict(geometry_type_counts),
    }


def write_contact_summary_csv(path: Path, result: dict) -> None:
    fields = [
        "theta",
        "beta",
        "gamma",
        "theta_deg",
        "beta_deg",
        "gamma_deg",
        "contact_height_tol",
        "arc_samples",
        "lateral_samples",
        "include_reference_points",
        "height_min",
        "z_min",
        "lowest_point_x",
        "lowest_point_y",
        "lowest_point_z",
        "lowest_surface_name",
        "lowest_geometry_type",
        "lowest_contact_state",
        "lowest_contact_state_id",
        "lowest_contact_state_label",
        "number_of_contact_points",
        "contact_states",
        "contact_surface_names",
        "contact_geometry_types",
        "dominant_contact_state",
        "dominant_contact_state_id",
        "dominant_contact_state_label",
        "representative_contact_x",
        "representative_contact_y",
        "representative_contact_z",
        "x_min",
        "x_max",
        "x_span",
        "y_min",
        "y_max",
        "y_span",
        "z_contact_min",
        "z_contact_max",
        "z_span",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerow({field: result[field] for field in fields})


def write_contact_points_csv(path: Path, result: dict) -> None:
    fields = [
        "point_index",
        "geometry_type",
        "surface_name",
        "surface_description",
        "contact_state",
        "contact_state_id",
        "contact_state_label",
        "lateral_index",
        "lateral_m",
        "arc_sample_index",
        "arc_angle_deg",
        "surface_center_x_m",
        "surface_center_y_2d_m",
        "local_x_m",
        "local_y_2d_m",
        "local_lateral_m",
        "x_m",
        "y_m",
        "z_m",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        for record in result["contact_records"]:
            writer.writerow({field: record[field] for field in fields})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze 3D geometric ground contact for one LegWheel theta-beta-gamma pose."
    )
    parser.add_argument("--theta-deg", type=float, default=17.0, help="Theta in degrees.")
    parser.add_argument("--beta-deg", type=float, default=45.0, help="Beta in degrees.")
    parser.add_argument("--gamma-deg", type=float, default=10.0, help="Gamma in degrees.")
    parser.add_argument(
        "--contact-height-tol",
        type=float,
        default=CONTACT_HEIGHT_TOL,
        help="Contact candidate height tolerance in meters.",
    )
    parser.add_argument(
        "--arc-samples",
        type=int,
        default=RIM_ARC_SAMPLES,
        help="Number of samples on each named rim outer arc.",
    )
    parser.add_argument(
        "--lateral-samples",
        type=int,
        default=LATERAL_SAMPLES,
        help="Number of samples across wheel width.",
    )
    parser.add_argument(
        "--include-reference-points",
        action="store_true",
        help="Also include non-surface HL/HR/O reference points.",
    )
    parser.add_argument(
        "--summary-csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "ground_contact_pose_3d_summary.csv",
        help="Output summary CSV path.",
    )
    parser.add_argument(
        "--contact-csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "ground_contact_pose_3d_candidates.csv",
        help="Output contact candidate CSV path.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = compute_ground_contact_pose_3d(
        theta=np.deg2rad(args.theta_deg),
        beta=np.deg2rad(args.beta_deg),
        gamma=np.deg2rad(args.gamma_deg),
        contact_height_tol=args.contact_height_tol,
        arc_samples=args.arc_samples,
        lateral_samples=args.lateral_samples,
        include_reference_points=args.include_reference_points,
    )

    write_contact_summary_csv(args.summary_csv, result)
    write_contact_points_csv(args.contact_csv, result)

    lowest = result["lowest_point"]
    rep = result["representative_contact"]
    print(f"Saved summary CSV: {args.summary_csv}")
    print(f"Saved contact candidate CSV: {args.contact_csv}")
    print(f"height_min/z_min: {result['height_min']:.9f} m")
    print(
        "lowest point: "
        f"x={lowest[0]:.9f}, y={lowest[1]:.9f}, z={lowest[2]:.9f}"
    )
    print(
        "lowest contact state: "
        f"{result['lowest_contact_state_label']} ({result['lowest_contact_state_id']})"
    )
    print(f"contact states: {result['contact_states']}")
    print(f"number of contact candidates: {result['number_of_contact_points']}")
    print(
        "representative contact point: "
        f"x={rep[0]:.9f}, y={rep[1]:.9f}, z={rep[2]:.9f}"
    )
    print(f"x_span: {result['x_span']:.9f} m")
    print(f"y_span: {result['y_span']:.9f} m")
    print(f"z_span: {result['z_span']:.9f} m")


if __name__ == "__main__":
    main()
