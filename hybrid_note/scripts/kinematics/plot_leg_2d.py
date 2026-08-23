"""Draw the current LegWheel 2D linkage geometry in XY coordinates.

This script reads the active kinematic model from ``legwheel.models`` via
``PlotLeg`` and saves a PNG plus a CSV coordinate dump under ``hybrid_note/outputs``.
Run from the project root:

    python3 hybrid_note/scripts/kinematics/plot_leg_2d.py
    python3 hybrid_note/scripts/kinematics/plot_leg_2d.py --theta-deg 100 --beta-deg 45
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import tempfile
import warnings
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "legwheel_matplotlib"))
warnings.filterwarnings("ignore", message="Unable to import Axes3D.*", category=UserWarning)

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

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

LABELS = {
    "H_extend_l": "Hext_l",
    "H_extend_r": "Hext_r",
}


def as_xy(value) -> np.ndarray:
    """Return a point as a flat [x, y] numpy array."""
    if isinstance(value, (complex, np.complexfloating)):
        return np.array([float(value.real), float(value.imag)])

    arr = np.asarray(value, dtype=float)
    if arr.ndim == 0:
        return np.array([float(arr), 0.0])
    return arr.reshape(-1)[:2].astype(float)


def collect_joint_xy(leg: PlotLeg) -> dict[str, np.ndarray]:
    """Collect named linkage points from the current PlotLeg state."""
    joints = {"O": np.array([0.0, 0.0])}
    for key in JOINT_KEYS:
        if key == "O":
            continue
        if hasattr(leg, key):
            joints[key] = as_xy(getattr(leg, key))
    return joints


def write_joint_csv(path: Path, joints: dict[str, np.ndarray]) -> None:
    with path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["point", "x_m", "y_m"])
        for name, xy in joints.items():
            writer.writerow([name, f"{xy[0]:.9f}", f"{xy[1]:.9f}"])


def plot_xy(
    theta_deg: float,
    beta_deg: float,
    alpha_deg: float,
    w_m: float,
    output_png: Path,
    output_csv: Path,
    return_fig: bool = False,
) -> dict[str, np.ndarray] | tuple[dict[str, np.ndarray], plt.Figure]:
    theta = np.deg2rad(theta_deg)
    beta = np.deg2rad(beta_deg)

    leg = PlotLeg()
    leg.leg_shape.link_alpha = 0.18
    leg.leg_shape.line_width = 2.0
    leg.leg_shape.mark_size = 4.0
    leg.leg_shape.Construction = True

    fig, ax = plt.subplots(figsize=(9.5, 8.0))
    leg.plot_by_angle(theta=theta, beta=beta, O=np.array([0.0, 0.0]), ax=ax)

    joints = collect_joint_xy(leg)
    rim_alphas = np.linspace(-180.0, 180.0, 241)
    rim_points = np.array([as_xy(leg.rim_point(alpha, w_m)) for alpha in rim_alphas])
    contact = as_xy(leg.rim_point(alpha_deg, w_m))

    ax.scatter(
        rim_points[:, 0],
        rim_points[:, 1],
        s=7,
        c=rim_alphas,
        cmap="viridis",
        alpha=0.45,
        linewidths=0,
        label="sampled rim points",
        zorder=7,
    )
    ax.scatter(
        [contact[0]],
        [contact[1]],
        s=80,
        marker="x",
        color="#d62728",
        linewidths=2.2,
        label=f"rim_point alpha={alpha_deg:g} deg, w={w_m:g} m",
        zorder=10,
    )

    left_xy = np.array([xy for name, xy in joints.items() if name.endswith("_l")])
    right_xy = np.array([xy for name, xy in joints.items() if name.endswith("_r")])
    center_xy = np.array(
        [xy for name, xy in joints.items() if not name.endswith("_l") and not name.endswith("_r")]
    )

    if len(left_xy):
        ax.scatter(left_xy[:, 0], left_xy[:, 1], s=28, color="#1f77b4", label="left-side joints")
    if len(right_xy):
        ax.scatter(right_xy[:, 0], right_xy[:, 1], s=28, color="#ff7f0e", label="right-side joints")
    if len(center_xy):
        ax.scatter(center_xy[:, 0], center_xy[:, 1], s=34, color="#2ca02c", label="center points")

    for name, xy in joints.items():
        label = LABELS.get(name, name)
        ax.annotate(
            label,
            xy=(xy[0], xy[1]),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=7.5,
            color="#1f2933",
            zorder=11,
        )

    all_points = np.vstack([np.array(list(joints.values())), rim_points, contact.reshape(1, 2)])
    span = np.ptp(all_points, axis=0)
    pad = max(float(span.max()) * 0.12, 0.025)
    ax.set_xlim(float(all_points[:, 0].min() - pad), float(all_points[:, 0].max() + pad))
    ax.set_ylim(float(all_points[:, 1].min() - pad), float(all_points[:, 1].max() + pad))

    ax.axhline(0.0, color="#6b7280", linewidth=0.8, alpha=0.6)
    ax.axvline(0.0, color="#6b7280", linewidth=0.8, alpha=0.6)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", linewidth=0.8, alpha=0.55)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(
        "Current LegWheel 2D XY geometry\n"
        f"theta={theta_deg:.2f} deg, beta={beta_deg:.2f} deg, "
        f"R={RobotParams.WHEEL_RADIUS_PITCH:.3f} m, "
        f"outer radius={RobotParams.WHEEL_RADIUS_OUTER:.3f} m"
    )
    ax.legend(loc="upper right", fontsize=8)

    write_joint_csv(output_csv, joints)
    fig.tight_layout()
    fig.savefig(output_png, dpi=180)
    if return_fig:
        return joints, fig
    plt.close(fig)
    return joints


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot the active LegWheel 2D linkage geometry using the current model."
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
        help="Rim contact angle in degrees for the highlighted point.",
    )
    parser.add_argument(
        "--w-m",
        type=float,
        default=0.0,
        help="Lateral rim contact offset in meters for rim sampling.",
    )
    parser.add_argument(
        "--png",
        type=Path,
        default=DEFAULT_FIGURE_DIR / "leg_2d.png",
        help="Output PNG path.",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_TABLE_DIR / "leg_2d_points.csv",
        help="Output CSV path.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.png.parent.mkdir(parents=True, exist_ok=True)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    joints = plot_xy(
        theta_deg=args.theta_deg,
        beta_deg=args.beta_deg,
        alpha_deg=args.alpha_deg,
        w_m=args.w_m,
        output_png=args.png,
        output_csv=args.csv,
    )
    print(f"Saved XY plot: {args.png}")
    print(f"Saved joint coordinates: {args.csv}")
    print("Point coordinates [m]:")
    for name, xy in joints.items():
        print(f"  {name:12s} x={xy[0]: .6f}, y={xy[1]: .6f}")


if __name__ == "__main__":
    main()
