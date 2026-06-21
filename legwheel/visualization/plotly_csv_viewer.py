#!/usr/bin/env python3
"""Plotly HTML trajectory viewer for Corgi hardware CSV files.

CSV column order (hardware format):
    [FL_t, FL_b, FR_t, FR_b, RR_t, RR_b, RL_t, RL_b, FL_g, FR_g, RR_g, RL_g]

The output is a standalone interactive HTML file with a frame slider and play button.
"""

import argparse
import os
import sys
from typing import List, Sequence

import numpy as np

try:
    import plotly.graph_objects as go
except ImportError as exc:  # pragma: no cover - exercised only without optional extra
    raise SystemExit(
        "Plotly CSV viewer requires the optional extra. " "Install it with: uv sync --extra plotly"
    ) from exc

from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.visualization.plotly_robot import (
    LEG_COLORS,
    LEG_COLORS as TRACE_COLORS,
    LIMB_NAMES,
    _line_trace,
    _marker_trace,
    chassis_traces,
    leg_mechanism_traces,
)

DEFAULT_HTML_PATH = "outputs/plotly/gait_viewer.html"
DT_INPUT = 0.001


def hw_to_kin(hw_row: Sequence[float]) -> np.ndarray:
    """Convert one hardware CSV row to per-leg kinematics order.

    Args:
        hw_row: 12-column hardware command row.

    Returns:
        np.ndarray: Shape `(4, 3)` with rows `[theta, beta, gamma]` for FL/FR/RR/RL.
    """
    row = np.asarray(hw_row, dtype=float)
    if row.shape[0] != 12:
        raise ValueError("hardware CSV row must have 12 columns")
    return np.array(
        [
            [row[0], row[1], row[8]],
            [row[2], row[3], row[9]],
            [row[4], row[5], row[10]],
            [row[6], row[7], row[11]],
        ]
    )


def load_hardware_csv(csv_path: str) -> np.ndarray:
    """Load and validate a 12-column Corgi hardware CSV."""
    data = np.loadtxt(csv_path, delimiter=",")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] != 12:
        raise ValueError(f"expected 12 CSV columns, got {data.shape[1]}")
    return data


def select_frame_indices(n_rows: int, frame_step: int, max_frames: int) -> np.ndarray:
    """Select render frame indices from raw CSV rows."""
    if frame_step < 1:
        raise ValueError("frame_step must be >= 1")
    indices = np.arange(0, n_rows, frame_step, dtype=int)
    if max_frames > 0 and len(indices) > max_frames:
        sample = np.linspace(0, len(indices) - 1, max_frames).astype(int)
        indices = indices[sample]
    return indices


def precompute_foot_traces(data: np.ndarray, frame_indices: np.ndarray) -> List[np.ndarray]:
    """Precompute contact traces for all selected frames in Body Frame {B}."""
    legs = [CorgiLegKinematics(i) for i in range(4)]
    traces = [[] for _ in range(4)]
    for raw_index in frame_indices:
        q_all = hw_to_kin(data[raw_index])
        for leg_index, q in enumerate(q_all):
            alpha, w_contact = legs[leg_index].foot_rim_contact_fk(*q)
            point = legs[leg_index].forward_kinematics(*q, alpha=alpha, w=w_contact)
            traces[leg_index].append(point)
    return [np.array(points) for points in traces]


def ground_plane_trace(z_ground: float, extent: float = 0.55) -> go.Surface:
    """Return a translucent ground plane at `z_ground` in Body Frame {B}."""
    x_vals = np.array([[-extent, extent], [-extent, extent]])
    y_vals = np.array([[-extent, -extent], [extent, extent]])
    z_vals = np.full_like(x_vals, z_ground)
    return go.Surface(
        x=x_vals,
        y=y_vals,
        z=z_vals,
        opacity=0.16,
        colorscale=[[0, "sienna"], [1, "sienna"]],
        showscale=False,
        name="Ground",
        showlegend=False,
    )


def body_trace(legs: Sequence[CorgiLegKinematics]) -> go.Scatter3d:
    """Return a simple hip-rectangle body trace."""
    hips = np.array([leg.p_Mi_in_B for leg in legs])
    order = [0, 1, 2, 3, 0]
    return _line_trace(hips[order], "Hip rectangle", "black", width=5, opacity=0.65)


def static_trace_bundle(
    legs: Sequence[CorgiLegKinematics],
    foot_traces: Sequence[np.ndarray],
    z_ground: float,
) -> List[go.Scatter3d]:
    """Build static traces shared by every animation frame."""
    traces: List[go.Scatter3d] = [ground_plane_trace(z_ground), body_trace(legs)]
    traces.extend(chassis_traces())
    for leg_index, points in enumerate(foot_traces):
        traces.append(
            _line_trace(
                points,
                f"{LIMB_NAMES[leg_index]} contact trace",
                TRACE_COLORS[leg_index],
                width=3,
                opacity=0.35,
                showlegend=True,
            )
        )
    return traces


def frame_trace_bundle(
    data: np.ndarray,
    frame_indices: np.ndarray,
    selected_frame: int,
    foot_traces: Sequence[np.ndarray],
    static_traces: Sequence[go.Scatter3d],
) -> List[go.Scatter3d]:
    """Build all traces for one selected animation frame."""
    raw_index = int(frame_indices[selected_frame])
    q_all = hw_to_kin(data[raw_index])
    traces = list(static_traces)

    for leg_index, q in enumerate(q_all):
        traces.extend(
            leg_mechanism_traces(
                leg_index,
                q[0],
                q[1],
                q[2],
                include_rim_thickness=False,
            )
        )
        current_foot = foot_traces[leg_index][selected_frame]
        traces.append(
            _marker_trace(
                current_foot.reshape(1, 3),
                f"{LIMB_NAMES[leg_index]} current contact",
                LEG_COLORS[leg_index],
                size=6,
                showlegend=False,
            )
        )

    return traces


def build_figure(
    data: np.ndarray,
    frame_indices: np.ndarray,
    title: str,
) -> go.Figure:
    """Build a Plotly trajectory animation figure."""
    legs = [CorgiLegKinematics(i) for i in range(4)]
    foot_traces = precompute_foot_traces(data, frame_indices)
    z_ground = min(points[:, 2].min() for points in foot_traces)
    static_traces = static_trace_bundle(legs, foot_traces, z_ground)

    frames = []
    for selected_frame in range(len(frame_indices)):
        raw_index = int(frame_indices[selected_frame])
        time_sec = raw_index * DT_INPUT
        frames.append(
            go.Frame(
                data=frame_trace_bundle(
                    data,
                    frame_indices,
                    selected_frame,
                    foot_traces,
                    static_traces,
                ),
                name=str(selected_frame),
                layout={"title": f"{title}<br>frame={raw_index}, t={time_sec:.3f}s"},
            )
        )

    figure = go.Figure(data=frames[0].data, frames=frames)
    slider_steps = [
        {
            "args": [[frame.name], {"frame": {"duration": 0, "redraw": True}, "mode": "immediate"}],
            "label": frame.name,
            "method": "animate",
        }
        for frame in frames
    ]
    figure.update_layout(
        title=frames[0].layout.title.text,
        scene={
            "xaxis_title": "X Front (m)",
            "yaxis_title": "Y Left (m)",
            "zaxis_title": "Z Up (m)",
            "aspectmode": "data",
            "xaxis": {"range": [-0.5, 0.5]},
            "yaxis": {"range": [-0.5, 0.5]},
            "zaxis": {"range": [z_ground - 0.08, 0.25]},
        },
        margin={"l": 0, "r": 0, "t": 70, "b": 0},
        updatemenus=[
            {
                "type": "buttons",
                "showactive": False,
                "buttons": [
                    {
                        "label": "Play",
                        "method": "animate",
                        "args": [
                            None,
                            {
                                "frame": {"duration": 80, "redraw": True},
                                "fromcurrent": True,
                                "transition": {"duration": 0},
                            },
                        ],
                    },
                    {
                        "label": "Pause",
                        "method": "animate",
                        "args": [
                            [None],
                            {
                                "frame": {"duration": 0, "redraw": False},
                                "mode": "immediate",
                                "transition": {"duration": 0},
                            },
                        ],
                    },
                ],
            }
        ],
        sliders=[{"active": 0, "steps": slider_steps}],
    )
    return figure


def main() -> int:
    """CLI entry point for the Plotly CSV trajectory viewer."""
    parser = argparse.ArgumentParser(description="Render a Corgi hardware CSV as Plotly HTML.")
    parser.add_argument("csv_file", help="Path to a 12-column hardware CSV file")
    parser.add_argument("--html", default=DEFAULT_HTML_PATH, help="Output HTML path")
    parser.add_argument("--frame-step", type=int, default=20, help="Raw CSV row stride")
    parser.add_argument("--max-frames", type=int, default=200, help="Maximum rendered frames")
    parser.add_argument("--show", action="store_true", help="Open the figure in a browser")
    args = parser.parse_args()

    data = load_hardware_csv(args.csv_file)
    frame_indices = select_frame_indices(data.shape[0], args.frame_step, args.max_frames)
    csv_name = os.path.basename(args.csv_file).replace(".csv", "")
    title = f"Corgi CSV Trajectory Viewer: {csv_name}"

    print(f"Loaded {data.shape[0]} raw rows from {args.csv_file}")
    print(f"Rendering {len(frame_indices)} Plotly frames")
    figure = build_figure(data, frame_indices, title)

    os.makedirs(os.path.dirname(args.html) or ".", exist_ok=True)
    figure.write_html(args.html, include_plotlyjs="cdn", auto_open=False)
    print(f"Saved Plotly CSV viewer -> {args.html}")

    if args.show:
        figure.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
