#!/usr/bin/env python3
"""Interactive Plotly renderer for the Corgi robot in Body Frame {B}.

This script is intentionally standalone and non-invasive: it reuses the existing
LegWheel kinematics geometry, writes an interactive HTML file, and leaves the
Matplotlib renderer untouched.
"""

import argparse
import os
import sys
from typing import Iterable, List, Optional, Sequence

import numpy as np

try:
    import plotly.graph_objects as go
except ImportError as exc:  # pragma: no cover - exercised only without optional extra
    raise SystemExit(
        "Plotly backend requires the optional extra. " "Install it with: uv sync --extra plotly"
    ) from exc

from legwheel.config import RobotParams
from legwheel.models.corgi_leg import CorgiLegKinematics

DEFAULT_HTML_PATH = "outputs/plotly/corgi_robot.html"
AXIS_COLORS = ["red", "green", "blue"]
LIMB_NAMES = ["FL", "FR", "RR", "RL"]
LEG_COLORS = ["royalblue", "tomato", "firebrick", "dodgerblue"]


def _rgba_to_plotly(color, fallback: str = "black") -> str:
    """Convert Matplotlib-style colors to Plotly-compatible color strings.

    Args:
        color: A Matplotlib color string or RGBA tuple.
        fallback: Color used if conversion fails.

    Returns:
        str: Plotly-compatible CSS color string.
    """
    if isinstance(color, str):
        return color
    try:
        r, g, b = color[:3]
        a = color[3] if len(color) > 3 else 1.0
        return f"rgba({int(r * 255)}, {int(g * 255)}, {int(b * 255)}, {a:.3f})"
    except Exception:
        return fallback


def _line_trace(
    points: np.ndarray,
    name: str,
    color: str = "black",
    width: float = 3.0,
    opacity: float = 1.0,
    showlegend: bool = False,
) -> go.Scatter3d:
    """Create a Plotly 3D line trace from points in Body Frame {B}."""
    pts = np.asarray(points, dtype=float)
    return go.Scatter3d(
        x=pts[:, 0],
        y=pts[:, 1],
        z=pts[:, 2],
        mode="lines",
        name=name,
        line={"color": color, "width": width},
        opacity=opacity,
        showlegend=showlegend,
    )


def _marker_trace(
    points: np.ndarray,
    name: str,
    color: str = "black",
    size: float = 4.0,
    symbol: str = "circle",
    showlegend: bool = True,
) -> go.Scatter3d:
    """Create a Plotly 3D marker trace from points in Body Frame {B}."""
    pts = np.asarray(points, dtype=float)
    return go.Scatter3d(
        x=pts[:, 0],
        y=pts[:, 1],
        z=pts[:, 2],
        mode="markers",
        name=name,
        marker={"color": color, "size": size, "symbol": symbol},
        showlegend=showlegend,
    )


def _text_trace(points: np.ndarray, labels: Sequence[str], name: str) -> go.Scatter3d:
    """Create a Plotly text trace for frame labels in Body Frame {B}."""
    pts = np.asarray(points, dtype=float)
    return go.Scatter3d(
        x=pts[:, 0],
        y=pts[:, 1],
        z=pts[:, 2],
        mode="text",
        text=list(labels),
        name=name,
        textposition="top center",
        showlegend=False,
    )


def chassis_traces() -> List[go.Scatter3d]:
    """Return octagonal chassis wireframe traces in Body Frame {B}."""
    length = RobotParams.CHASSIS_LENGTH
    width = RobotParams.CHASSIS_WIDTH
    height = RobotParams.CHASSIS_HEIGHT
    z0 = RobotParams.ABAD_AXIS_OFFSET
    chamfer = 0.04

    y_pts = np.array(
        [
            width / 2 - chamfer,
            width / 2,
            width / 2,
            width / 2 - chamfer,
            -width / 2 + chamfer,
            -width / 2,
            -width / 2,
            -width / 2 + chamfer,
        ]
    )
    z_pts = (
        np.array(
            [
                height / 2,
                height / 2 - chamfer,
                -height / 2 + chamfer,
                -height / 2,
                -height / 2,
                -height / 2 + chamfer,
                height / 2 - chamfer,
                height / 2,
            ]
        )
        + z0
    )

    traces: List[go.Scatter3d] = []
    for x_value, name in [(length / 2, "front chassis"), (-length / 2, "rear chassis")]:
        pts = np.column_stack(
            [
                np.append(np.full(8, x_value), x_value),
                np.append(y_pts, y_pts[0]),
                np.append(z_pts, z_pts[0]),
            ]
        )
        traces.append(_line_trace(pts, name=name, color="black", width=5, opacity=0.8))

    for idx in range(8):
        pts = np.array(
            [
                [length / 2, y_pts[idx], z_pts[idx]],
                [-length / 2, y_pts[idx], z_pts[idx]],
            ]
        )
        traces.append(_line_trace(pts, name="chassis edge", color="black", width=3, opacity=0.55))

    return traces


def _project_shape_trace(kin: CorgiLegKinematics, x_data, y_data, z_offset, gamma) -> np.ndarray:
    """Project 2D linkage points into Body Frame {B}."""
    z_data = np.full_like(np.asarray(x_data, dtype=float), z_offset, dtype=float)
    pts_l = np.vstack([x_data, y_data, z_data]).T
    return np.array([kin._transform_to_body(point, gamma) for point in pts_l])


def leg_mechanism_traces(
    leg_index: int,
    theta: float,
    beta: float,
    gamma: float,
    include_rim_thickness: bool = True,
) -> List[go.Scatter3d]:
    """Return detailed leg mechanism traces in Body Frame {B}.

    Args:
        leg_index: Limb index, 0=FL, 1=FR, 2=RR, 3=RL.
        theta: Leg extension angle in rad.
        beta: Leg swing angle in rad.
        gamma: ABAD angle in rad.
        include_rim_thickness: Whether to draw grey wheel-width connector lines.

    Returns:
        list[go.Scatter3d]: Plotly traces for one leg mechanism.
    """
    kin = CorgiLegKinematics(leg_index)
    traces: List[go.Scatter3d] = []

    kin.fk_sagittal(theta, beta)
    shape = kin.solver.leg_shape
    shape.get_shape(shape.O)

    component_specs = [
        (kin.wheel_thickness / 2, {"rims", "joints"}),
        (-kin.wheel_thickness / 2, {"rims", "joints"}),
        (0.0, {"bars"}),
    ]

    for z_offset, components in component_specs:
        for key, value in shape.__dict__.items():
            if "bar" in key and hasattr(value, "get_xdata") and "bars" in components:
                pts = _project_shape_trace(
                    kin, value.get_xdata(), value.get_ydata(), z_offset, gamma
                )
                traces.append(
                    _line_trace(
                        pts,
                        name=f"{LIMB_NAMES[leg_index]} bars",
                        color=_rgba_to_plotly(value.get_color(), LEG_COLORS[leg_index]),
                        width=max(float(value.get_linewidth()) * 4.0, 2.0),
                    )
                )
            elif "rim" in key and hasattr(value, "arc") and "rims" in components:
                for arc in value.arc:
                    theta1, theta2 = np.deg2rad(arc.theta1), np.deg2rad(arc.theta2)
                    diff = theta2 - theta1
                    while diff > np.pi:
                        diff -= 2 * np.pi
                    while diff < -np.pi:
                        diff += 2 * np.pi
                    arc_angles = np.linspace(theta1, theta1 + diff, 24)
                    center_x, center_y = arc.center
                    x_data = center_x + (arc.width / 2) * np.cos(arc_angles)
                    y_data = center_y + (arc.height / 2) * np.sin(arc_angles)
                    pts = _project_shape_trace(kin, x_data, y_data, z_offset, gamma)
                    traces.append(
                        _line_trace(
                            pts,
                            name=f"{LIMB_NAMES[leg_index]} rims",
                            color=_rgba_to_plotly(arc.get_edgecolor(), LEG_COLORS[leg_index]),
                            width=max(float(arc.get_linewidth()) * 4.0, 2.0),
                        )
                    )
            elif "joint" in key and hasattr(value, "center") and "joints" in components:
                center_x, center_y = value.center
                joint_angles = np.linspace(0, 2 * np.pi, 24)
                x_data = center_x + value.radius * np.cos(joint_angles)
                y_data = center_y + value.radius * np.sin(joint_angles)
                pts = _project_shape_trace(kin, x_data, y_data, z_offset, gamma)
                traces.append(
                    _line_trace(
                        pts,
                        name=f"{LIMB_NAMES[leg_index]} joints",
                        color=_rgba_to_plotly(value.get_edgecolor(), LEG_COLORS[leg_index]),
                        width=max(float(value.get_linewidth()) * 4.0, 2.0),
                    )
                )

    if include_rim_thickness:
        for alpha in np.linspace(-180, 180, 28):
            rim_pos = kin.forward_kinematics(
                theta, beta, gamma, alpha=alpha, w=kin.wheel_thickness / 2
            )
            rim_neg = kin.forward_kinematics(
                theta, beta, gamma, alpha=alpha, w=-kin.wheel_thickness / 2
            )
            traces.append(
                _line_trace(
                    np.vstack([rim_pos, rim_neg]),
                    name=f"{LIMB_NAMES[leg_index]} tyre width",
                    color="gray",
                    width=2,
                    opacity=0.45,
                )
            )

    return traces


def frame_traces(gamma_values: Sequence[float], axis_len: float = 0.05) -> List[go.Scatter3d]:
    """Return Body, Module, and Leg frame axes in Body Frame {B}."""
    traces: List[go.Scatter3d] = []
    labels = ["X", "Y", "Z"]

    for axis_index, color in enumerate(AXIS_COLORS):
        end = np.zeros(3)
        end[axis_index] = axis_len
        traces.append(
            _line_trace(np.vstack([np.zeros(3), end]), f"{{B}} {labels[axis_index]}", color, 5)
        )

    text_points = [np.array([0.0, 0.0, 0.02])]
    text_labels = ["{B}"]

    for leg_index, gamma in enumerate(gamma_values):
        kin = CorgiLegKinematics(leg_index)
        t_l_to_m, t_m_to_b = kin._get_transformation_matrices(gamma=gamma)
        module_origin = kin._M_to_B(np.array([0, 0, 0]), gamma)
        leg_origin = kin._transform_to_body(np.array([0, 0, 0]), gamma)
        r_m_to_b = t_m_to_b[:3, :3]
        r_l_to_b = r_m_to_b @ t_l_to_m[:3, :3]

        for axis_index, color in enumerate(AXIS_COLORS):
            module_axis = r_m_to_b[:, axis_index] * axis_len
            leg_axis = r_l_to_b[:, axis_index] * axis_len
            traces.append(
                _line_trace(
                    np.vstack([module_origin, module_origin + module_axis]),
                    f"{LIMB_NAMES[leg_index]} {{Mi}} {labels[axis_index]}",
                    color,
                    3,
                    0.75,
                )
            )
            traces.append(
                _line_trace(
                    np.vstack([leg_origin, leg_origin + leg_axis]),
                    f"{LIMB_NAMES[leg_index]} {{Li}} {labels[axis_index]}",
                    color,
                    3,
                    0.75,
                )
            )

        text_points.extend([module_origin + np.array([0.0, 0.0, 0.02]), leg_origin])
        text_labels.extend([f"{LIMB_NAMES[leg_index]} {{Mi}}", f"{{L{leg_index}}}"])

    traces.append(_text_trace(np.array(text_points), text_labels, "frame labels"))
    return traces


def collision_bound_traces(
    theta: float,
    beta: float,
    gamma_values: Sequence[float],
) -> List[go.Scatter3d]:
    """Return collision marker traces in Body Frame {B}."""
    from legwheel.models.corgi_robot import CorgiRobot
    from legwheel.models.collision_model import CorgiCollisionModel

    q_list = [[theta, beta, gamma] for gamma in gamma_values]
    robot = CorgiRobot()
    collision = CorgiCollisionModel(robot)
    points = collision.get_all_collision_points(q_list)

    return [
        _marker_trace(points["chassis"], "Chassis corners", "gray", 3, "square"),
        _marker_trace(points["m6_studs"], "M6 studs", "red", 5, "circle"),
        _marker_trace(points["wheels"], "Wheel contacts", "dodgerblue", 5, "diamond"),
    ]


def build_figure(
    theta: float,
    beta: float,
    gamma: float,
    gamma_values: Optional[Sequence[float]] = None,
    show_frames: bool = True,
    show_bounds: bool = False,
    include_rim_thickness: bool = True,
) -> go.Figure:
    """Build an interactive Plotly Corgi robot figure in Body Frame {B}."""
    if gamma_values is None:
        gamma_values = [gamma] * 4
    if len(gamma_values) != 4:
        raise ValueError("gamma_values must contain four ABAD angles: [FL, FR, RR, RL]")

    traces: List[go.Scatter3d] = []
    traces.extend(chassis_traces())

    for leg_index, leg_gamma in enumerate(gamma_values):
        traces.extend(
            leg_mechanism_traces(
                leg_index,
                theta,
                beta,
                leg_gamma,
                include_rim_thickness=include_rim_thickness,
            )
        )

    if show_frames:
        traces.extend(frame_traces(gamma_values))

    if show_bounds:
        traces.extend(collision_bound_traces(theta, beta, gamma_values))

    title = (
        "Corgi Robot 3D (Plotly, Body Frame {B})<br>"
        f"theta={np.rad2deg(theta):.1f}°, beta={np.rad2deg(beta):.1f}°, "
        f"gamma={np.rad2deg(gamma):.1f}°"
    )
    figure = go.Figure(data=traces)
    figure.update_layout(
        title=title,
        scene={
            "xaxis_title": "X Front (m)",
            "yaxis_title": "Y Left (m)",
            "zaxis_title": "Z Up (m)",
            "aspectmode": "data",
        },
        legend={"itemsizing": "constant"},
        margin={"l": 0, "r": 0, "t": 60, "b": 0},
    )
    return figure


def parse_gamma_list(values: Optional[Iterable[float]]) -> Optional[List[float]]:
    """Parse optional per-leg gamma values in degrees."""
    if values is None:
        return None
    gamma_degrees = list(values)
    if len(gamma_degrees) != 4:
        raise ValueError("gamma list must contain four values: FL FR RR RL")
    return [np.deg2rad(value) for value in gamma_degrees]


def main() -> int:
    """CLI entry point for the Plotly robot viewer."""
    parser = argparse.ArgumentParser(description="Render Corgi robot with Plotly.")
    parser.add_argument("--theta", type=float, default=75.0, help="Theta in degrees")
    parser.add_argument("--beta", type=float, default=0.0, help="Beta in degrees")
    parser.add_argument("--gamma", type=float, default=0.0, help="Uniform gamma in degrees")
    parser.add_argument("--gamma-list", nargs=4, type=float, help="Per-leg gamma values in degrees")
    parser.add_argument("--html", default=DEFAULT_HTML_PATH, help="Output HTML path")
    parser.add_argument(
        "--show", action="store_true", help="Open the figure in the default browser"
    )
    parser.add_argument("--bounds", action="store_true", help="Show collision bound markers")
    parser.add_argument("--no-frames", action="store_true", help="Hide coordinate frame overlays")
    parser.add_argument(
        "--no-rim-thickness", action="store_true", help="Hide wheel-width connector lines"
    )
    args = parser.parse_args()

    try:
        gamma_values = parse_gamma_list(args.gamma_list)
    except ValueError as exc:
        parser.error(str(exc))

    figure = build_figure(
        theta=np.deg2rad(args.theta),
        beta=np.deg2rad(args.beta),
        gamma=np.deg2rad(args.gamma),
        gamma_values=gamma_values,
        show_frames=not args.no_frames,
        show_bounds=args.bounds,
        include_rim_thickness=not args.no_rim_thickness,
    )

    os.makedirs(os.path.dirname(args.html) or ".", exist_ok=True)
    figure.write_html(args.html, include_plotlyjs="cdn", auto_open=False)
    print(f"Saved Plotly Corgi robot viewer -> {args.html}")

    if args.show:
        figure.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
