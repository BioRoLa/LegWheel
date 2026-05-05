"""
demo_phase_transitions.py
=========================

Shows all 5 boundary states (S4→S3→S2→S1→S0) plus one intermediate
angle between each consecutive pair, arranged in a 3×3 grid.

Each panel shows:
  • Top-view (XY) support polygon with CoM projection and stability badge
  • 3-D robot pose (isometric, dark background)

Layout (3 rows × 3 cols, then extra row for last 3 cols)
---------------------------------------------------------
  Row 0: S4 (180°)  |  mid (153°)  |  S3 (126°)
  Row 1: S2 (115°)  |  mid (100°)  |  S1 ( 85°)
  Row 2: mid ( 42°) |  S0 (  0°)   |  [spacer]
         ← each cell = top-view (left) + 3-D (right) →

Actually rendered as:
  Grid 3 × 3, each cell has a composite panel (top-view above, 3-D below).
  Achieved with nested axes.

Usage
-----
    uv run python examples/self_righting/boundary_states/demo_phase_transitions.py --no-show
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
from scipy.spatial import ConvexHull

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.collision_model import CorgiCollisionModel
from legwheel.config import RobotParams
from render.plot_leg_envelope import draw_leg_envelope

import examples.self_righting.boundary_states.render_boundary_sequence as renderer

# ─────────────────────────────────────────────────────────────────────────────
# Phase definitions
# ─────────────────────────────────────────────────────────────────────────────

_q = lambda t, b, g: [np.deg2rad(t), np.deg2rad(b), np.deg2rad(g)]


def get_phases(beta_deg):
    q_nom = _q(60, beta_deg, 0)
    q_g30 = _q(60, beta_deg, 30)
    beta_label = int(beta_deg)
    return [
       dict(id="S4",       roll=180, pitch=0, q_list=[q_nom]*4,
           label=f"S4  Upside-Down\nRoll=180°, β={beta_label}°, γ=0°", color="#4CAF50", is_boundary=True),
       dict(id="mid1_4_3", roll=167, pitch=0, q_list=[q_nom]*4,
           label=f"→  Roll=167°\nβ={beta_label}°, γ=0°", color="#888888", is_boundary=False),
       dict(id="mid2_4_3", roll=153, pitch=0, q_list=[q_nom]*4,
           label=f"→  Roll=153°\nβ={beta_label}°, γ=0°", color="#888888", is_boundary=False),
       dict(id="S3",       roll=126, pitch=0, q_list=[q_nom, q_g30, q_g30, q_nom],
           label=f"S3  Reach Down\nRoll=126°, β={beta_label}°, γ_lower=30°", color="#E91E63", is_boundary=True),
       dict(id="mid_3_2",  roll=120, pitch=0, q_list=[q_nom, q_g30, q_g30, q_nom],
           label=f"→  Roll=120°\nβ={beta_label}°, γ_lower=30°", color="#888888", is_boundary=False),
       dict(id="S2",       roll=115, pitch=0, q_list=[q_nom, q_g30, q_g30, q_nom],
           label=f"S2  Wedge Contact\nRoll=115°, β={beta_label}°, γ_lower=30°", color="#9C27B0", is_boundary=True),
       dict(id="mid1_2_1", roll=107, pitch=0, q_list=[q_nom]*4,
           label=f"→  Roll=107°\nβ={beta_label}°, γ=0°", color="#888888", is_boundary=False),
       dict(id="mid2_2_1", roll=100, pitch=0, q_list=[q_nom]*4,
           label=f"→  Roll=100°\nβ={beta_label}°, γ=0°", color="#888888", is_boundary=False),
       dict(id="S1",       roll=85,  pitch=0, q_list=[q_nom]*4,
           label=f"S1  Side-Fall Lead\nRoll=85°, β={beta_label}°, γ=0°", color="#FF9800", is_boundary=True),
       dict(id="mid1_1_0", roll=63,  pitch=0, q_list=[q_nom]*4,
           label=f"→  Roll=63°\nβ={beta_label}°, γ=0°", color="#888888", is_boundary=False),
       dict(id="mid2_1_0", roll=42,  pitch=0, q_list=[q_nom]*4,
           label=f"→  Roll=42°\nβ={beta_label}°, γ=0°", color="#888888", is_boundary=False),
       dict(id="S0",       roll=0,   pitch=0, q_list=[q_nom]*4,
           label=f"S0  Upright\nRoll=0°, β={beta_label}°, γ=0°", color="#2196F3", is_boundary=True),
    ]

CONTACT_TOL = 3e-3

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def build_and_ground(phase):
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(phase["roll"]), np.deg2rad(phase["pitch"]), 0.0])
    robot.base_pos = np.array([0.0, 0.0, 0.5])
    col = CorgiCollisionModel(robot)
    pts = col.get_all_collision_points(phase["q_list"])
    all_p = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= all_p[:, 2].min()
    return robot, col


def compute_stability(robot, col, q_list):
    pts = col.get_all_collision_points(q_list)
    all_p = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    min_z = all_p[:, 2].min()
    mask = all_p[:, 2] <= min_z + CONTACT_TOL
    pivots_xy = all_p[mask, :2]
    com_w = robot.body_to_world(np.zeros(3))
    com_xy = com_w[:2]
    margin = -1.0
    if len(pivots_xy) >= 3:
        try:
            hull = ConvexHull(pivots_xy)
            s = -(hull.equations[:, :2] @ com_xy + hull.equations[:, 2])
            min_dist = float(s.min())
            hp = pivots_xy[hull.vertices]
            n = len(hp)
            perim = sum(np.linalg.norm(hp[(i + 1) % n] - hp[i]) for i in range(n))
            insc_r = 2 * hull.volume / perim if perim > 1e-6 else 1.0
            margin = min_dist / insc_r
        except Exception:
            pass
    return pivots_xy, com_xy, margin


# ─────────────────────────────────────────────────────────────────────────────
# Drawing
# ─────────────────────────────────────────────────────────────────────────────

def draw_top_panel(ax, phase):
    """Top-view support polygon panel (XY ground plane)."""
    robot, col = build_and_ground(phase)
    pivots_xy, com_xy, margin = compute_stability(robot, col, phase["q_list"])

    ax.set_aspect("equal")
    ax.set_facecolor("#f8f9fa")
    ax.grid(True, alpha=0.2, lw=0.5)

    # Support polygon
    if len(pivots_xy) >= 3:
        try:
            hull = ConvexHull(pivots_xy)
            verts = pivots_xy[hull.vertices]
            poly = plt.Polygon(verts, closed=True,
                               facecolor="limegreen", edgecolor="#1a7a1a",
                               alpha=0.30, lw=2.0, zorder=2)
            ax.add_patch(poly)
        except Exception:
            pass
    elif len(pivots_xy) == 2:
        ax.plot(pivots_xy[:, 0], pivots_xy[:, 1],
                color="limegreen", lw=2.5, zorder=2)

    # Contact pivots
    ax.scatter(pivots_xy[:, 0], pivots_xy[:, 1],
               c="gold", edgecolors="black", s=80, marker="*", zorder=5)

    # CoM projection
    stable = margin > 0
    com_color = "#1a7a1a" if stable else "#c0392b"
    ax.scatter(*com_xy, c=com_color, s=120, marker="D", zorder=6)

    # Margin badge
    badge_bg = "#d4edda" if stable else "#f8d7da"
    badge_txt = "#155724" if stable else "#721c24"
    status = "STABLE" if stable else "UNSTABLE"
    margin_s = f"η={margin:+.2f}" if abs(margin) < 10 else "η=N/A"
    ax.text(0.97, 0.97, f"{status}\n{margin_s}",
            transform=ax.transAxes, ha="right", va="top",
            fontsize=7.5, fontweight="bold", color=badge_txt,
            bbox=dict(boxstyle="round,pad=0.3", fc=badge_bg, ec=badge_txt, lw=1.0),
            zorder=9)

    # Phase label
    border_col = phase["color"]
    ax.set_title(phase["label"], fontsize=8, fontweight="bold",
                 color=border_col, pad=4)
    ax.set_xlabel("X (m)", fontsize=6)
    ax.set_ylabel("Y (m)", fontsize=6)
    ax.tick_params(labelsize=5)
    ax.set_xlim(-0.38, 0.38)
    ax.set_ylim(-0.38, 0.38)

    # Mark boundary phases with coloured frame
    for spine in ax.spines.values():
        spine.set_edgecolor(phase["color"])
        spine.set_linewidth(2.0 if phase["is_boundary"] else 0.8)

    return margin


def draw_3d_panel(ax, phase, view_elev=8, view_azim=0):
    """
    3-D robot render panel (light background).
    Front view: view_elev=8,  view_azim=0   -- camera from +X, sees YZ body profile + roll
    Top view:   view_elev=85, view_azim=0   -- near-zenith, sees XY support footprint
    """
    robot, col = build_and_ground(phase)
    q_list = phase["q_list"]

    ax.set_facecolor("#f5f5f5")
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.fill = True
        pane.set_facecolor("#eeeeee")
        pane.set_edgecolor("#cccccc")

    renderer._draw_ground(ax)
    renderer._draw_chassis_wireframe(ax, robot, color="#222222")
    for leg_idx, (theta, beta, _) in enumerate(q_list):
        draw_leg_envelope(
            ax,
            leg_idx,
            theta,
            beta,
            gamma_max_deg=RobotParams.GAMMA_MAX_DEG,
            n_gamma_slices=4,
            N_alpha=36,
            N_t=8,
            point_transform=robot.body_to_world,
        )
    renderer._draw_legs_world(ax, robot, q_list)
    renderer._draw_collision_markers(ax, robot, col, q_list)
    renderer._draw_body_frame(ax, robot, scale=0.12)

    ax.view_init(elev=view_elev, azim=view_azim)
    for a in [ax.xaxis, ax.yaxis, ax.zaxis]:
        a.label.set_color("black")
        a.label.set_size(5)
    ax.tick_params(colors="black", labelsize=4)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(-0.45, 0.45)
    ax.set_ylim(-0.45, 0.45)
    ax.set_zlim(0.0, 0.55)
    ax.set_box_aspect([1, 1, 0.55])


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--beta", type=float, default=90.0)
    parser.add_argument("--save-dir", "-o", default=None)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    phases = get_phases(args.beta)

    cols = 4
    PHASE_ROWS = [phases[i : i + cols] for i in range(0, len(phases), cols)]
    ROW_NAMES  = ["01_S4_to_S3", "02_S2_to_S1", "03_S1_to_S0"]
    ROW_TITLES = [
        f"S4 -> S3  (beta={int(args.beta)}°)",
        f"S2 -> S1  (beta={int(args.beta)}°)",
        f"S1 -> S0  (beta={int(args.beta)}°)",
    ]

    save_dir = args.save_dir or f"output/phase_transitions_beta{int(args.beta)}"
    os.makedirs(save_dir, exist_ok=True)
    saved = []

    for row_idx, (phases_row, row_name, row_title) in enumerate(
        zip(PHASE_ROWS, ROW_NAMES, ROW_TITLES)
    ):
        n_this = len(phases_row)
        print(f"\n-- Row {row_idx + 1}/3  [{row_name}] --")
        fig = plt.figure(figsize=(n_this * 5.5, 9.0))
        fig.patch.set_facecolor("white")
        fig.suptitle(
            f"Self-Righting Phase Sequence  -  {row_title}",
            fontsize=11, fontweight="bold", color="black", y=1.01,
        )

        outer_gs = gridspec.GridSpec(1, n_this, figure=fig,
                                     hspace=0.0, wspace=0.30)

        for col_idx, phase in enumerate(phases_row):
            inner_gs = gridspec.GridSpecFromSubplotSpec(
                2, 1, subplot_spec=outer_gs[0, col_idx],
                height_ratios=[1, 1.1], hspace=0.05,
            )
            ax_top = fig.add_subplot(inner_gs[0])

            # Split 3D row: front view (+X) | top view (+Z)
            gs_3d = gridspec.GridSpecFromSubplotSpec(
                1, 2, subplot_spec=inner_gs[1], wspace=0.02,
            )
            ax_front = fig.add_subplot(gs_3d[0], projection="3d")
            ax_topv  = fig.add_subplot(gs_3d[1], projection="3d")

            m = draw_top_panel(ax_top, phase)
            # Front: camera at +X looking toward -X => sees YZ body profile + lateral roll
            draw_3d_panel(ax_front, phase, view_elev=8,  view_azim=0)
            # Top: near-zenith => sees XY footprint (same as support polygon)
            draw_3d_panel(ax_topv,  phase, view_elev=85, view_azim=0)
            ax_front.set_title("Front (+X)", color="#333333", fontsize=6, pad=2)
            ax_topv.set_title( "Top  (+Z)",  color="#333333", fontsize=6, pad=2)
            print(f"  {phase['id']:15s}  roll={phase['roll']:>3d}  eta={m:+.3f}")

        save_path = os.path.join(save_dir, f"phase_{row_name}.png")
        plt.savefig(save_path, dpi=160, bbox_inches="tight", facecolor="white")
        print(f"  -> Saved: {save_path}")
        saved.append(save_path)
        plt.close(fig)

    print(f"\nAll {len(saved)} files saved to: {save_dir}/")


if __name__ == "__main__":
    main()
