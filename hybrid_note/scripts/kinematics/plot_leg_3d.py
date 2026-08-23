"""Draw the current LegWheel mechanism as local 3D and projection views.

The model already has a body-frame 3D wrapper in ``CorgiLegKinematics``. This
script keeps the mechanism origin ``O`` fixed at ``(0, 0, 0)`` and uses the
existing 2D ``PlotLeg`` geometry. The saved CSV uses local model coordinates
``[x, y_2d, lateral]``. The figure display maps them to ``[x, lateral, y_2d]``
so the leg stands upright.
The PNG contains three panels from left to right: 3D, leg plane view, and YZ
projection.

Run from the project root:

    .venv/bin/python hybrid_note/scripts/kinematics/plot_leg_3d.py
    .venv/bin/python hybrid_note/scripts/kinematics/plot_leg_3d.py --gamma-deg 20
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "legwheel_matplotlib"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

NOTE_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = NOTE_ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from legwheel.config import RobotParams  # noqa: E402
from legwheel.visualization.plot_leg import PlotLeg  # noqa: E402

DEFAULT_FIGURE_DIR = NOTE_ROOT / "outputs" / "figures" / "kinematics"
DEFAULT_TABLE_DIR = NOTE_ROOT / "outputs" / "tables" / "kinematics"

JOINT_KEYS = [
    "O",
    "A_l",
    "A_r",
    "B_l",
    "B_r",
    "C_l",
    "C_r",
    "D_l",
    "D_r",
    "E",
    "F_l",
    "F_r",
    "G",
    "H_l",
    "H_r",
    "U_l",
    "U_r",
    "L_l",
    "L_r",
    "O_r",
    "I_l",
    "I_r",
    "J_l",
    "J_r",
    "H_extend_l",
    "H_extend_r",
]


def as_xy(value) -> np.ndarray:
    if isinstance(value, (complex, np.complexfloating)):
        return np.array([float(value.real), float(value.imag)])

    arr = np.asarray(value, dtype=float)
    if arr.ndim == 0:
        return np.array([float(arr), 0.0])
    return arr.reshape(-1)[:2].astype(float)


def to_display_xyz(points, gamma_rad: float = 0.0) -> np.ndarray:
    """Map local [x, y_2d, lateral] to displayed [x, lateral, y_2d], then roll by gamma."""
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


def collect_joint_xyz(leg: PlotLeg, z_m: float = 0.0) -> dict[str, np.ndarray]:
    joints = {"O": np.array([0.0, 0.0, z_m])}
    for key in JOINT_KEYS:
        if key == "O" or not hasattr(leg, key):
            continue
        xy = as_xy(getattr(leg, key))
        joints[key] = np.array([xy[0], xy[1], z_m])
    return joints


def write_joint_csv(path: Path, joints: dict[str, np.ndarray]) -> None:
    with path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["point", "x_m", "y_2d_m", "lateral_m"])
        for name, xyz in joints.items():
            writer.writerow([name, f"{xyz[0]:.9f}", f"{xyz[1]:.9f}", f"{xyz[2]:.9f}"])


def arc_points(arc, n: int = 40) -> tuple[np.ndarray, np.ndarray]:
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
    return x, y


def draw_shape_plane(
    ax,
    leg: PlotLeg,
    z_m: float,
    w_m: float,
    include_bars: bool,
    include_joints: bool,
    alpha: float,
    gamma_rad: float,
) -> None:
    shape = leg.leg_shape
    shape.get_shape(np.array([0.0, 0.0]), tyre_offset=leg.tyre_offset_at_w(w_m))

    for key, val in shape.__dict__.items():
        if "bar" in key and hasattr(val, "get_xdata") and include_bars:
            x = np.asarray(val.get_xdata(), dtype=float)
            y = np.asarray(val.get_ydata(), dtype=float)
            z = np.full_like(x, z_m)
            pts = to_display_xyz(np.vstack([x, y, z]).T, gamma_rad)
            ax.plot(
                pts[:, 0],
                pts[:, 1],
                pts[:, 2],
                color=val.get_color(),
                linewidth=val.get_linewidth(),
                alpha=alpha,
            )
        elif "rim" in key and hasattr(val, "arc"):
            for arc in val.arc:
                x, y = arc_points(arc)
                z = np.full_like(x, z_m)
                pts = to_display_xyz(np.vstack([x, y, z]).T, gamma_rad)
                ax.plot(
                    pts[:, 0],
                    pts[:, 1],
                    pts[:, 2],
                    color=arc.get_edgecolor(),
                    linewidth=arc.get_linewidth(),
                    alpha=alpha,
                )
        elif "joint" in key and hasattr(val, "center") and include_joints and val.radius > 0:
            ang = np.linspace(0.0, 2.0 * np.pi, 40)
            cx, cy = val.center
            x = cx + val.radius * np.cos(ang)
            y = cy + val.radius * np.sin(ang)
            z = np.full_like(x, z_m)
            pts = to_display_xyz(np.vstack([x, y, z]).T, gamma_rad)
            ax.plot(
                pts[:, 0],
                pts[:, 1],
                pts[:, 2],
                color=val.get_edgecolor(),
                linewidth=val.get_linewidth(),
                alpha=alpha,
            )


def draw_tire_width_lines(
    ax, leg: PlotLeg, theta: float, beta: float, half_w: float, gamma_rad: float
) -> None:
    alphas = np.linspace(-180.0, 180.0, 72)
    w_samples = np.linspace(-half_w, half_w, 17)
    for alpha in alphas:
        points = []
        for w_m in w_samples:
            leg.forward(theta, beta, vector=True)
            xy = as_xy(leg.rim_point(alpha, w_m))
            points.append([xy[0], xy[1], w_m])
        points = to_display_xyz(np.asarray(points), gamma_rad)
        ax.plot(points[:, 0], points[:, 1], points[:, 2], color="#6b7280", linewidth=0.7, alpha=0.35)


def collect_shape_segments(
    leg: PlotLeg,
    z_m: float,
    w_m: float,
    include_bars: bool,
    include_joints: bool,
    alpha: float,
    gamma_rad: float,
) -> list[dict]:
    shape = leg.leg_shape
    shape.get_shape(np.array([0.0, 0.0]), tyre_offset=leg.tyre_offset_at_w(w_m))
    segments = []

    for key, val in shape.__dict__.items():
        if "bar" in key and hasattr(val, "get_xdata") and include_bars:
            x = np.asarray(val.get_xdata(), dtype=float)
            y = np.asarray(val.get_ydata(), dtype=float)
            z = np.full_like(x, z_m)
            segments.append(
                {
                    "points": to_display_xyz(np.vstack([x, y, z]).T, gamma_rad),
                    "color": val.get_color(),
                    "linewidth": val.get_linewidth(),
                    "alpha": alpha,
                }
            )
        elif "rim" in key and hasattr(val, "arc"):
            for arc in val.arc:
                x, y = arc_points(arc)
                z = np.full_like(x, z_m)
                segments.append(
                    {
                        "points": to_display_xyz(np.vstack([x, y, z]).T, gamma_rad),
                        "color": arc.get_edgecolor(),
                        "linewidth": arc.get_linewidth(),
                        "alpha": alpha,
                    }
                )
        elif "joint" in key and hasattr(val, "center") and include_joints and val.radius > 0:
            ang = np.linspace(0.0, 2.0 * np.pi, 40)
            cx, cy = val.center
            x = cx + val.radius * np.cos(ang)
            y = cy + val.radius * np.sin(ang)
            z = np.full_like(x, z_m)
            segments.append(
                {
                    "points": to_display_xyz(np.vstack([x, y, z]).T, gamma_rad),
                    "color": val.get_edgecolor(),
                    "linewidth": val.get_linewidth(),
                    "alpha": alpha,
                }
            )
    return segments


def collect_tire_width_segments(
    leg: PlotLeg, theta: float, beta: float, half_w: float, gamma_rad: float
) -> list[dict]:
    segments = []
    for alpha in np.linspace(-180.0, 180.0, 72):
        points = []
        for w_m in np.linspace(-half_w, half_w, 17):
            leg.forward(theta, beta, vector=True)
            xy = as_xy(leg.rim_point(alpha, w_m))
            points.append([xy[0], xy[1], w_m])
        segments.append(
            {
                "points": to_display_xyz(np.asarray(points), gamma_rad),
                "color": "#6b7280",
                "linewidth": 0.7,
                "alpha": 0.35,
            }
        )
    return segments


def draw_segments_3d(ax, segments: list[dict]) -> None:
    for segment in segments:
        pts = segment["points"]
        ax.plot(
            pts[:, 0],
            pts[:, 1],
            pts[:, 2],
            color=segment["color"],
            linewidth=segment["linewidth"],
            alpha=segment["alpha"],
        )


def draw_segments_2d(ax, segments: list[dict], dims: tuple[int, int]) -> None:
    for segment in segments:
        pts = segment["points"]
        ax.plot(
            pts[:, dims[0]],
            pts[:, dims[1]],
            color=segment["color"],
            linewidth=segment["linewidth"],
            alpha=segment["alpha"],
        )


def sampled_display_points(
    leg: PlotLeg, theta: float, beta: float, half_w: float, gamma_rad: float
) -> np.ndarray:
    points = []
    for alpha in np.linspace(-180.0, 180.0, 72):
        for w_m in np.linspace(-half_w, half_w, 9):
            leg.forward(theta, beta, vector=True)
            xy = as_xy(leg.rim_point(alpha, w_m))
            points.append([xy[0], xy[1], w_m])
    return to_display_xyz(np.asarray(points), gamma_rad)


def set_axes_equal(ax, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    centers = (mins + maxs) / 2.0
    radius = max(float((maxs - mins).max()) / 2.0, 0.08)
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def set_axes_equal_2d(ax, points: np.ndarray, dims: tuple[int, int]) -> None:
    pts = points[:, dims]
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    centers = (mins + maxs) / 2.0
    radius = max(float((maxs - mins).max()) / 2.0, 0.08)
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_aspect("equal", adjustable="box")


def plot_3d(
    theta_deg: float,
    beta_deg: float,
    gamma_deg: float,
    alpha_deg: float,
    output_png: Path,
    output_csv: Path,
    return_fig: bool = False,
) -> dict[str, np.ndarray] | tuple[dict[str, np.ndarray], plt.Figure]:
    theta = np.deg2rad(theta_deg)
    beta = np.deg2rad(beta_deg)
    gamma = np.deg2rad(gamma_deg)
    half_w = RobotParams.WHEEL_THICKNESS / 2.0

    leg = PlotLeg()
    leg.leg_shape.link_alpha = 0.15
    leg.leg_shape.line_width = 1.7
    leg.leg_shape.mark_size = 3.5
    leg.leg_shape.Construction = False
    leg.forward(theta, beta, vector=False)

    segments = []
    segments.extend(
        collect_shape_segments(
            leg,
            z_m=-half_w,
            w_m=-half_w,
            include_bars=False,
            include_joints=False,
            alpha=0.70,
            gamma_rad=gamma,
        )
    )
    segments.extend(
        collect_shape_segments(
            leg,
            z_m=half_w,
            w_m=half_w,
            include_bars=False,
            include_joints=False,
            alpha=0.70,
            gamma_rad=gamma,
        )
    )
    segments.extend(
        collect_shape_segments(
            leg,
            z_m=0.0,
            w_m=0.0,
            include_bars=True,
            include_joints=True,
            alpha=1.0,
            gamma_rad=gamma,
        )
    )
    segments.extend(collect_tire_width_segments(leg, theta, beta, half_w, gamma))
    leg_plane_segments = collect_shape_segments(
        leg,
        z_m=0.0,
        w_m=0.0,
        include_bars=True,
        include_joints=True,
        alpha=1.0,
        gamma_rad=0.0,
    )

    fig = plt.figure(figsize=(18.0, 6.2))
    ax_3d = fig.add_subplot(1, 3, 1, projection="3d")
    ax_leg = fig.add_subplot(1, 3, 2)
    ax_yz = fig.add_subplot(1, 3, 3)

    draw_segments_3d(ax_3d, segments)
    draw_segments_2d(ax_leg, leg_plane_segments, dims=(0, 2))
    draw_segments_2d(ax_yz, segments, dims=(1, 2))

    leg.forward(theta, beta, vector=True)
    joints = collect_joint_xyz(leg)
    joint_points = to_display_xyz(np.asarray(list(joints.values())), gamma)
    leg_plane_joint_points = to_display_xyz(np.asarray(list(joints.values())), 0.0)
    ax_3d.scatter(
        joint_points[:, 0],
        joint_points[:, 1],
        joint_points[:, 2],
        s=22,
        color="#111827",
        depthshade=False,
    )
    ax_leg.scatter(
        leg_plane_joint_points[:, 0], leg_plane_joint_points[:, 2], s=16, color="#111827"
    )
    ax_yz.scatter(joint_points[:, 1], joint_points[:, 2], s=16, color="#111827")

    contact_xy = as_xy(leg.rim_point(alpha_deg, 0.0))
    contact = np.array([contact_xy[0], contact_xy[1], 0.0])
    display_contact = to_display_xyz(contact, gamma)
    leg_plane_contact = to_display_xyz(contact, 0.0)
    ax_3d.scatter(
        [display_contact[0]],
        [display_contact[1]],
        [display_contact[2]],
        s=80,
        marker="x",
        linewidths=2.2,
        color="#d62728",
        depthshade=False,
        label=f"rim_point alpha={alpha_deg:g} deg, lateral=0",
    )
    ax_leg.scatter(
        [leg_plane_contact[0]],
        [leg_plane_contact[2]],
        s=62,
        marker="x",
        linewidths=2.0,
        color="#d62728",
    )
    ax_yz.scatter(
        [display_contact[1]],
        [display_contact[2]],
        s=62,
        marker="x",
        linewidths=2.0,
        color="#d62728",
    )

    for name in ["O", "B_l", "B_r", "C_l", "C_r", "F_l", "F_r", "G", "H_l", "H_r"]:
        if name in joints:
            xyz = to_display_xyz(joints[name], gamma)
            leg_xyz = to_display_xyz(joints[name], 0.0)
            ax_3d.text(xyz[0], xyz[1], xyz[2] + 0.006, name, fontsize=8)
            ax_leg.annotate(
                name,
                xy=(leg_xyz[0], leg_xyz[2]),
                xytext=(3, 3),
                textcoords="offset points",
                fontsize=7,
            )
            ax_yz.annotate(name, xy=(xyz[1], xyz[2]), xytext=(3, 3), textcoords="offset points", fontsize=7)

    all_points = np.vstack(
        [
            sampled_display_points(leg, theta, beta, half_w, gamma),
            joint_points,
            display_contact.reshape(1, 3),
        ]
    )
    set_axes_equal(ax_3d, all_points)
    leg_plane_points = np.vstack(
        [
            sampled_display_points(leg, theta, beta, half_w, 0.0),
            leg_plane_joint_points,
            leg_plane_contact.reshape(1, 3),
        ]
    )
    set_axes_equal_2d(ax_leg, leg_plane_points, dims=(0, 2))
    set_axes_equal_2d(ax_yz, all_points, dims=(1, 2))

    ax_3d.set_xlabel("X [m]")
    ax_3d.set_ylabel("Y lateral [m]")
    ax_3d.set_zlabel("Z from 2D Y [m]")
    ax_3d.set_title("3D")
    ax_3d.view_init(elev=18.0, azim=-64.0)
    ax_3d.legend(loc="upper right", fontsize=8)

    ax_leg.set_xlabel("X [m]")
    ax_leg.set_ylabel("Y 2D [m]")
    ax_leg.set_title("Leg Plane View")
    ax_leg.grid(True, linestyle=":", linewidth=0.8, alpha=0.55)

    ax_yz.set_xlabel("Y lateral [m]")
    ax_yz.set_ylabel("Z from 2D Y [m]")
    ax_yz.set_title("YZ Projection")
    ax_yz.grid(True, linestyle=":", linewidth=0.8, alpha=0.55)

    fig.suptitle(
        "Current LegWheel Upright Local Geometry, O at Origin\n"
        f"theta={theta_deg:.2f} deg, beta={beta_deg:.2f} deg, gamma={gamma_deg:.2f} deg, "
        f"wheel thickness={RobotParams.WHEEL_THICKNESS:.3f} m",
        fontsize=14,
    )

    write_joint_csv(output_csv, joints)
    fig.tight_layout(rect=(0, 0, 1, 0.91))
    fig.savefig(output_png, dpi=180)
    if return_fig:
        return joints, fig
    plt.close(fig)
    return joints


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot the active LegWheel 3D local geometry with O fixed at the origin."
    )
    parser.add_argument(
        "--theta-deg",
        type=float,
        default=RobotParams.THETA0_DEG,
        help="Theta joint angle in degrees.",
    )
    parser.add_argument("--beta-deg", type=float, default=0.0, help="Beta joint angle in degrees.")
    parser.add_argument(
        "--alpha-deg",
        type=float,
        default=0.0,
        help="Rim contact angle in degrees for the highlighted center-plane point.",
    )
    parser.add_argument(
        "--gamma-deg",
        type=float,
        default=0.0,
        help="Local ABAD/roll angle in degrees. Positive values tilt the upright leg around X.",
    )
    parser.add_argument(
        "--png",
        type=Path,
        default=DEFAULT_FIGURE_DIR / "leg_3d.png",
        help="Output PNG path.",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "leg_3d_points.csv",
        help="Output CSV path.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.png.parent.mkdir(parents=True, exist_ok=True)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    joints = plot_3d(
        theta_deg=args.theta_deg,
        beta_deg=args.beta_deg,
        gamma_deg=args.gamma_deg,
        alpha_deg=args.alpha_deg,
        output_png=args.png,
        output_csv=args.csv,
    )
    print(f"Saved local 3D plot: {args.png}")
    print(f"Saved joint coordinates: {args.csv}")
    print("Point coordinates [m], local model frame [x, y_2d, lateral], O fixed at origin:")
    for name, xyz in joints.items():
        print(
            f"  {name:12s} x={xyz[0]: .6f}, y_2d={xyz[1]: .6f}, lateral={xyz[2]: .6f}"
        )


if __name__ == "__main__":
    main()
