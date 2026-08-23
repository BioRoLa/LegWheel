"""Single-pose 2D geometric ground-contact analysis for the LegWheel.

The core function is ``compute_ground_contact(theta, beta)``. Angles are in
radians, matching the model code. The command-line interface accepts degrees.

The analysis samples the current ``PlotLeg`` geometry directly. Valid contact
states are foot rim, left rim, and right rim. All other sampled geometry is
grouped as ``non_contact_region`` while retaining its original ``surface_name``
for tracing.

Run from the project root:

    .venv/bin/python hybrid_note/scripts/kinematics/ground_contact_single_pose.py
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

NOTE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = NOTE_ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from legwheel.visualization.plot_leg import PlotLeg  # noqa: E402

DEFAULT_TABLE_DIR = NOTE_ROOT / "outputs" / "tables" / "kinematics"

CONTACT_HEIGHT_TOL = 1e-3
RIM_ARC_SAMPLES = 721

TOP_NON_CONTACT_FRACTION = 0.35

CONTACT_STATE_INFO = {
    "foot_rim": {
        "state_id": "F",
        "label": "foot_rim",
    },
    "left_rim": {
        "state_id": "L",
        "label": "left_rim",
    },
    "right_rim": {
        "state_id": "R",
        "label": "right_rim",
    },
    "non_contact_region": {
        "state_id": "N",
        "label": "non_contact_region",
    },
}
CONTACT_STATE_ORDER = list(CONTACT_STATE_INFO)

RIM_SURFACES = {
    "foot_rim": {
        "description": "bottom foot rim outer arc",
    },
    "upper_rim_l": {
        "description": "left upper structural rim outer arc",
    },
    "upper_rim_r": {
        "description": "right upper structural rim outer arc",
    },
    "lower_rim_l": {
        "description": "left lower structural rim outer arc",
    },
    "lower_rim_r": {
        "description": "right lower structural rim outer arc",
    },
    "upper_rim_l_f": {
        "description": "left top-wheel outer arc near HL",
    },
    "upper_rim_r_f": {
        "description": "right top-wheel outer arc near HR",
    },
}

REFERENCE_POINTS = {
    "HL": {
        "leg_attr": "H_l",
        "contact_state": "non_contact_region",
        "description": "left top-wheel reference point",
    },
    "HR": {
        "leg_attr": "H_r",
        "contact_state": "non_contact_region",
        "description": "right top-wheel reference point",
    },
    "O": {
        "leg_attr": None,
        "contact_state": "non_contact_region",
        "description": "leg origin reference point",
    },
}


def as_xy(value) -> np.ndarray:
    """Return a model point as a flat ``[x, y]`` numpy array."""
    if isinstance(value, (complex, np.complexfloating)):
        return np.array([float(value.real), float(value.imag)])

    arr = np.asarray(value, dtype=float)
    if arr.ndim == 0:
        return np.array([float(arr), 0.0])
    return arr.reshape(-1)[:2].astype(float)


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


def state_metadata(contact_state: str) -> dict:
    return CONTACT_STATE_INFO.get(contact_state, {"state_id": "", "label": contact_state})


def surface_contact_state(
    surface_name: str,
    arc_sample_index: int,
    sample_count: int,
) -> str:
    if sample_count <= 1:
        normalized_arc_position = 0.5
    else:
        normalized_arc_position = arc_sample_index / float(sample_count - 1)

    if surface_name == "foot_rim":
        return "foot_rim"

    if surface_name == "lower_rim_l":
        return "left_rim"
    if surface_name == "lower_rim_r":
        return "right_rim"

    if surface_name in {"upper_rim_l", "upper_rim_l_f"}:
        if normalized_arc_position <= TOP_NON_CONTACT_FRACTION:
            return "non_contact_region"
        return "left_rim"

    if surface_name in {"upper_rim_r", "upper_rim_r_f"}:
        if normalized_arc_position >= 1.0 - TOP_NON_CONTACT_FRACTION:
            return "non_contact_region"
        return "right_rim"

    return "non_contact_region"


def ordered_contact_states(states: set[str]) -> list[str]:
    order_index = {state: index for index, state in enumerate(CONTACT_STATE_ORDER)}
    return sorted(states, key=lambda state: (order_index.get(state, len(CONTACT_STATE_ORDER)), state))


def dominant_contact_state(counter: Counter) -> str:
    order_index = {state: index for index, state in enumerate(CONTACT_STATE_ORDER)}
    return min(counter, key=lambda state: (-counter[state], order_index.get(state, 999), state))


def sample_contact_geometry_points(
    theta: float,
    beta: float,
    arc_samples: int = RIM_ARC_SAMPLES,
    include_reference_points: bool = True,
) -> list[dict]:
    """Sample 2D rim outer arcs and optional non-contact reference points."""
    leg = PlotLeg()
    leg.forward(theta, beta, vector=False)
    leg.leg_shape.get_shape(np.array([0.0, 0.0]))

    records = []
    point_index = 0

    for surface_name, surface_info in RIM_SURFACES.items():
        rim_obj = getattr(leg.leg_shape, surface_name, None)
        if rim_obj is None or not hasattr(rim_obj, "arc"):
            continue

        outer_arc = rim_obj.arc[1]
        xs, ys, arc_angles = arc_points_with_angles(outer_arc, arc_samples)
        center_x_m = float(outer_arc.center[0])
        center_y_m = float(outer_arc.center[1])

        for arc_sample_index, (x_m, y_m, arc_angle_deg) in enumerate(zip(xs, ys, arc_angles)):
            contact_state = surface_contact_state(surface_name, arc_sample_index, len(xs))
            state_info = state_metadata(contact_state)
            records.append(
                {
                    "point_index": point_index,
                    "geometry_type": "rim_arc",
                    "surface_name": surface_name,
                    "surface_description": surface_info["description"],
                    "contact_state": contact_state,
                    "contact_state_id": state_info["state_id"],
                    "contact_state_label": state_info["label"],
                    "arc_sample_index": arc_sample_index,
                    "arc_angle_deg": float(arc_angle_deg),
                    "surface_center_x_m": center_x_m,
                    "surface_center_y_m": center_y_m,
                    "x_m": float(x_m),
                    "y_m": float(y_m),
                }
            )
            point_index += 1

    if include_reference_points:
        for point_name, point_info in REFERENCE_POINTS.items():
            if point_info["leg_attr"] is None:
                xy = np.array([0.0, 0.0])
            else:
                xy = as_xy(getattr(leg, point_info["leg_attr"]))

            contact_state = point_info["contact_state"]
            state_info = state_metadata(contact_state)
            records.append(
                {
                    "point_index": point_index,
                    "geometry_type": "reference_point",
                    "surface_name": point_name,
                    "surface_description": point_info["description"],
                    "contact_state": contact_state,
                    "contact_state_id": state_info["state_id"],
                    "contact_state_label": state_info["label"],
                    "arc_sample_index": "",
                    "arc_angle_deg": "",
                    "surface_center_x_m": "",
                    "surface_center_y_m": "",
                    "x_m": float(xy[0]),
                    "y_m": float(xy[1]),
                }
            )
            point_index += 1

    return records


def compute_ground_contact(
    theta: float,
    beta: float,
    contact_height_tol: float = CONTACT_HEIGHT_TOL,
    arc_samples: int = RIM_ARC_SAMPLES,
    include_reference_points: bool = True,
) -> dict:
    """Compute geometric ground-contact candidates for one 2D theta-beta pose."""
    surface_records = sample_contact_geometry_points(
        theta=theta,
        beta=beta,
        arc_samples=arc_samples,
        include_reference_points=include_reference_points,
    )
    surface_points = np.array([[record["x_m"], record["y_m"]] for record in surface_records])

    lowest_index = int(np.argmin(surface_points[:, 1]))
    y_min = float(surface_points[lowest_index, 1])
    candidate_indices = np.flatnonzero(surface_points[:, 1] - y_min <= contact_height_tol)
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
        "theta_deg": float(np.rad2deg(theta)),
        "beta_deg": float(np.rad2deg(beta)),
        "contact_height_tol": float(contact_height_tol),
        "arc_samples": int(arc_samples),
        "include_reference_points": bool(include_reference_points),
        "surface_points": surface_points,
        "surface_records": surface_records,
        "height_min": y_min,
        "y_min": y_min,
        "lowest_point": lowest_point,
        "lowest_point_x": float(lowest_point[0]),
        "lowest_point_y": float(lowest_point[1]),
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
        "x_min": float(mins[0]),
        "x_max": float(maxs[0]),
        "x_span": float(spans[0]),
        "y_contact_min": float(mins[1]),
        "y_contact_max": float(maxs[1]),
        "y_span": float(spans[1]),
        "state_counts": dict(state_counts),
        "surface_counts": dict(surface_counts),
        "geometry_type_counts": dict(geometry_type_counts),
    }


def write_contact_summary_csv(path: Path, result: dict) -> None:
    fields = [
        "theta",
        "beta",
        "theta_deg",
        "beta_deg",
        "contact_height_tol",
        "arc_samples",
        "include_reference_points",
        "height_min",
        "y_min",
        "lowest_point_x",
        "lowest_point_y",
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
        "x_min",
        "x_max",
        "x_span",
        "y_contact_min",
        "y_contact_max",
        "y_span",
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
        "arc_sample_index",
        "arc_angle_deg",
        "surface_center_x_m",
        "surface_center_y_m",
        "x_m",
        "y_m",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        for record in result["contact_records"]:
            writer.writerow({field: record[field] for field in fields})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze 2D geometric ground contact for one LegWheel theta-beta pose."
    )
    parser.add_argument("--theta-deg", type=float, default=17.0, help="Theta in degrees.")
    parser.add_argument("--beta-deg", type=float, default=45.0, help="Beta in degrees.")
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
        "--exclude-reference-points",
        action="store_true",
        help="Only use rim arcs; exclude explicit non-contact reference points.",
    )
    parser.add_argument(
        "--summary-csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "ground_contact_single_pose_summary.csv",
        help="Output summary CSV path.",
    )
    parser.add_argument(
        "--contact-csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "ground_contact_single_pose_candidates.csv",
        help="Output contact candidate CSV path.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    theta = np.deg2rad(args.theta_deg)
    beta = np.deg2rad(args.beta_deg)

    result = compute_ground_contact(
        theta,
        beta,
        contact_height_tol=args.contact_height_tol,
        arc_samples=args.arc_samples,
        include_reference_points=not args.exclude_reference_points,
    )

    write_contact_summary_csv(args.summary_csv, result)
    write_contact_points_csv(args.contact_csv, result)

    lowest = result["lowest_point"]
    rep = result["representative_contact"]
    print(f"Saved summary CSV: {args.summary_csv}")
    print(f"Saved contact candidate CSV: {args.contact_csv}")
    print(f"height_min/y_min: {result['height_min']:.9f} m")
    print(f"lowest point: x={lowest[0]:.9f}, y={lowest[1]:.9f}")
    print(
        "lowest contact state: "
        f"{result['lowest_contact_state_label']} ({result['lowest_contact_state_id']})"
    )
    print(f"contact states: {result['contact_states']}")
    print(f"number of contact candidates: {result['number_of_contact_points']}")
    print(f"representative contact point: x={rep[0]:.9f}, y={rep[1]:.9f}")
    print(f"x_span: {result['x_span']:.9f} m")
    print(f"y_span: {result['y_span']:.9f} m")


if __name__ == "__main__":
    main()
