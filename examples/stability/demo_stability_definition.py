"""
demo_stability_definition.py
============================

A standalone demo that visualises the definition of static stability margin
for the Corgi robot across four representative roll orientations.

Layout
------
  Row 1 (3 subplots): S0 (upright), S4 (upside-down), S1 (side-fall leading)
  Row 2 (full-width): Stability margin vs Roll — summary sweep

Each top-view panel shows:
  ● Contact pivots        (yellow ★)
  ● Support polygon       (limegreen fill + outline)
  ● CoM XY projection     (gold ★)
  ● Margin annotation     (signed distance from CoM to nearest polygon edge)
  ● Inscribed circle      (shows the normalisation radius)

Usage
-----
    uv run python examples/stability/demo_stability_definition.py
    uv run python examples/stability/demo_stability_definition.py --save-dir output/stability_demo
"""

import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Circle, FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.collision_model import CorgiCollisionModel

import examples.self_righting.boundary_states.render_boundary_sequence as renderer

# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────

CONTACT_TOL = 3e-3   # m  —  z tolerance to detect ground contacts

# q = [theta_rad, beta_rad, gamma_rad]
_q = lambda t, b, g: [np.deg2rad(t), np.deg2rad(b), np.deg2rad(g)]

DEMO_CASES = [
    dict(
        label="S0  Upright\n(Roll=0°, γ=0°)",
        roll=0,
        pitch=0,
        q_list=[_q(60, 90, 0)] * 4,
        color="#2196F3",
    ),
    dict(
        label="S4  Upside-Down\n(Roll=180°, γ=0°)",
        roll=180,
        pitch=0,
        q_list=[_q(60, 90, 0)] * 4,
        color="#4CAF50",
    ),
    dict(
        label="S1  Side-Fall Leading\n(Roll=85°, γ=0°)",
        roll=85,
        pitch=0,
        q_list=[_q(60, 90, 0)] * 4,
        color="#FF9800",
    ),
    dict(
        label="Unstable  (Roll=100°, γ=0°)",
        roll=100,
        pitch=0,
        q_list=[_q(60, 90, 0)] * 4,
        color="#e74c3c",
    ),
]


# ─────────────────────────────────────────────────────────────────────────────
# Core helpers
# ─────────────────────────────────────────────────────────────────────────────

def build_and_ground(roll_deg, pitch_deg, q_list):
    """Create CorgiRobot for the given orientation, auto-grounded to Z=0."""
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), np.deg2rad(pitch_deg), 0.0])
    robot.base_pos = np.array([0.0, 0.0, 0.5])
    col = CorgiCollisionModel(robot)
    pts = col.get_all_collision_points(q_list)
    all_p = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= all_p[:, 2].min()
    return robot, col


def get_contact_info(robot, col, q_list):
    """
    Returns (pivots_xy, com_xy, margin, inscribed_r, nearest_edge_pts).

    margin > 0  →  stable  (CoM inside polygon)
    margin < 0  →  unstable
    margin normalised by inscribed radius of the support polygon.
    """
    pts = col.get_all_collision_points(q_list)
    all_p = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    min_z = all_p[:, 2].min()
    mask = all_p[:, 2] <= min_z + CONTACT_TOL
    pivots_xy = all_p[mask, :2]

    com_w = robot.body_to_world(np.zeros(3))
    com_xy = com_w[:2]

    margin = -1.0
    inscribed_r = 0.0
    nearest_edge = None

    if len(pivots_xy) >= 3:
        try:
            hull = ConvexHull(pivots_xy)
            # Signed distance from com_xy to each hull facet (positive = inside)
            s = -(hull.equations[:, :2] @ com_xy + hull.equations[:, 2])
            k = int(np.argmin(s))
            min_dist = float(s[k])

            # Inscribed radius via area / perimeter
            hp = pivots_xy[hull.vertices]
            n = len(hp)
            perim = sum(np.linalg.norm(hp[(i + 1) % n] - hp[i]) for i in range(n))
            inscribed_r = 2 * hull.volume / perim if perim > 1e-6 else 1.0
            margin = min_dist / inscribed_r

            # Two endpoints of the nearest edge for annotation
            v0, v1 = hull.simplices[k]
            nearest_edge = (pivots_xy[v0], pivots_xy[v1])
        except Exception:
            pass

    return pivots_xy, com_xy, margin, inscribed_r, nearest_edge


# ─────────────────────────────────────────────────────────────────────────────
# 3-D robot panel
# ─────────────────────────────────────────────────────────────────────────────

def draw_robot_3d(ax, case):
    """Render 3-D robot pose with support polygon overlay for a demo case."""
    robot, col = build_and_ground(case["roll"], case["pitch"], case["q_list"])
    q_list = case["q_list"]

    ax.set_facecolor("#f5f5f5")
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.fill = True
        pane.set_facecolor("#eeeeee")
        pane.set_edgecolor("#cccccc")

    renderer._draw_ground(ax)
    renderer._draw_chassis_wireframe(ax, robot, color="#222222")
    renderer._draw_legs_world(ax, robot, q_list)
    renderer._draw_collision_markers(ax, robot, col, q_list)
    renderer._draw_body_frame(ax, robot, scale=0.12)

    ax.view_init(elev=22, azim=-50)
    ax.set_xlabel("X", color="black", fontsize=6)
    ax.set_ylabel("Y", color="black", fontsize=6)
    ax.set_zlabel("Z", color="black", fontsize=6)
    ax.tick_params(colors="black", labelsize=5)
    ax.set_xlim(-0.45, 0.45)
    ax.set_ylim(-0.45, 0.45)
    ax.set_zlim(0.0, 0.55)
    ax.set_box_aspect([1, 1, 0.55])
    ax.set_title("3-D Pose", color="#333333", fontsize=8, pad=4)


# ─────────────────────────────────────────────────────────────────────────────
# Top-view panel
# ─────────────────────────────────────────────────────────────────────────────

def draw_top_view(ax, case):
    """
    Draw a top-down (XY) view illustrating the stability definition:
      - Support polygon with fill
      - CoM projection with margin annotation
      - Inscribed circle (normalisation reference)
    """
    robot, col = build_and_ground(case["roll"], case["pitch"], case["q_list"])
    pivots_xy, com_xy, margin, inscribed_r, nearest_edge = get_contact_info(
        robot, col, case["q_list"]
    )

    ax.set_aspect("equal")
    ax.set_facecolor("#f8f9fa")
    ax.grid(True, alpha=0.25, lw=0.5)

    # ── Support polygon ───────────────────────────────────────────────────────
    if len(pivots_xy) >= 3:
        try:
            hull = ConvexHull(pivots_xy)
            verts = pivots_xy[hull.vertices]
            poly = plt.Polygon(verts, closed=True,
                               facecolor="limegreen", edgecolor="#1a7a1a",
                               alpha=0.30, lw=2.0, zorder=2, label="Support polygon")
            ax.add_patch(poly)
        except Exception:
            pass
    elif len(pivots_xy) == 2:
        ax.plot(pivots_xy[:, 0], pivots_xy[:, 1],
                color="limegreen", lw=2.5, label="Support line", zorder=2)

    # ── Inscribed circle (normalisation reference) ─────────────────────────
    if inscribed_r > 0:
        # Approximate inscribed circle centred at centroid of hull vertices
        if len(pivots_xy) >= 3:
            try:
                hull2 = ConvexHull(pivots_xy)
                centroid = pivots_xy[hull2.vertices].mean(axis=0)
                circ = Circle(centroid, inscribed_r,
                              fill=False, edgecolor="#888", lw=1.0,
                              ls="--", zorder=3, label=f"Inscribed r={inscribed_r*100:.1f}cm")
                ax.add_patch(circ)
            except Exception:
                pass

    # ── Contact pivots ────────────────────────────────────────────────────────
    ax.scatter(pivots_xy[:, 0], pivots_xy[:, 1],
               c="gold", edgecolors="black", s=120, marker="*",
               zorder=5, label="Contact pivots")

    # ── CoM projection ────────────────────────────────────────────────────────
    stable = margin > 0
    com_color = "#1a7a1a" if stable else "#c0392b"
    ax.scatter(*com_xy, c=com_color, s=160, marker="D", zorder=6, label="CoM projection")

    # ── Nearest edge + margin arrow ───────────────────────────────────────────
    if nearest_edge is not None:
        ep0, ep1 = nearest_edge
        ax.plot([ep0[0], ep1[0]], [ep0[1], ep1[1]],
                color="red", lw=2.5, zorder=4, label="Nearest edge")
        # Foot of perpendicular from com_xy to nearest edge
        seg = ep1 - ep0
        seg_len = np.linalg.norm(seg)
        if seg_len > 1e-6:
            t = np.dot(com_xy - ep0, seg) / seg_len ** 2
            foot = ep0 + t * seg
            ax.annotate(
                "",
                xy=foot, xytext=com_xy,
                arrowprops=dict(
                    arrowstyle="<->",
                    color="purple", lw=1.8,
                ),
                zorder=7,
            )
            mid = (com_xy + foot) / 2
            dist_m = np.linalg.norm(com_xy - foot)
            sign_str = "+" if stable else "−"
            ax.text(mid[0] + 0.01, mid[1] + 0.01,
                    f"d={sign_str}{dist_m*100:.1f}cm",
                    fontsize=8, color="purple", fontweight="bold", zorder=8)

    # ── Margin badge ──────────────────────────────────────────────────────────
    badge_color = "#d4edda" if stable else "#f8d7da"
    border_color = "#155724" if stable else "#721c24"
    text_color = "#155724" if stable else "#721c24"
    status_str = "STABLE" if stable else "UNSTABLE"
    margin_str = f"η = {margin:+.3f}" if abs(margin) < 10 else "η = N/A"
    ax.text(0.97, 0.97, f"{status_str}\n{margin_str}",
            transform=ax.transAxes,
            ha="right", va="top", fontsize=9, fontweight="bold",
            color=text_color,
            bbox=dict(boxstyle="round,pad=0.35", fc=badge_color,
                      ec=border_color, lw=1.2),
            zorder=9)

    ax.set_title(case["label"], fontsize=9, fontweight="bold", color=case["color"], pad=6)
    ax.set_xlabel("X  (m)", fontsize=8)
    ax.set_ylabel("Y  (m)", fontsize=8)
    ax.tick_params(labelsize=7)

    lim = 0.40
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)

    return margin


# ─────────────────────────────────────────────────────────────────────────────
# Summary sweep (Roll 0° → 180°)
# ─────────────────────────────────────────────────────────────────────────────

def draw_margin_sweep(ax):
    """Panel: normalised stability margin vs Roll angle for default γ=0° config."""
    q_nom = [_q(60, 90, 0)] * 4

    rolls = np.arange(0, 181, 5)
    margins = []
    for r in rolls:
        robot, col = build_and_ground(r, 0, q_nom)
        _, _, m, _, _ = get_contact_info(robot, col, q_nom)
        margins.append(m)
    margins = np.array(margins)

    ax.axhline(0, color="black", lw=1.2, ls="--", zorder=1)
    stable_mask = margins > 0
    ax.fill_between(rolls, margins, 0, where=stable_mask,
                    color="#2ecc71", alpha=0.35, label="Stable (η > 0)")
    ax.fill_between(rolls, margins, 0, where=~stable_mask,
                    color="#e74c3c", alpha=0.35, label="Unstable (η < 0)")
    ax.plot(rolls, margins, "ko-", ms=4, lw=1.5, zorder=3)

    # Annotate demo cases
    for case in DEMO_CASES:
        r = case["roll"]
        idx = np.searchsorted(rolls, r)
        if idx < len(margins):
            ax.scatter(r, margins[idx], color=case["color"], s=120,
                       zorder=5, edgecolors="black", lw=0.8)
            ax.annotate(
                case["label"].split("\n")[0],
                xy=(r, margins[idx]),
                xytext=(r + 4, margins[idx] + 0.12),
                fontsize=7.5, color=case["color"], fontweight="bold",
                arrowprops=dict(arrowstyle="->", color=case["color"], lw=1.0),
            )

    ax.set_xlim(-2, 182)
    ax.set_ylim(-1.4, 1.4)
    ax.set_xlabel("Roll (deg)", fontsize=10)
    ax.set_ylabel("Normalised CoM margin  η", fontsize=10)
    ax.set_title(
        "Stability Margin vs. Roll  (θ=60°, β=90°, γ=0°, all legs)\n"
        "η = signed_distance_to_nearest_edge / inscribed_radius",
        fontsize=10,
    )
    ax.legend(fontsize=9, loc="lower right")
    ax.grid(True, alpha=0.3)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--save-dir", "-o", default="output/stability_demo",
                        help="Output directory for saved PNGs (default: output/stability_demo)")
    parser.add_argument("--no-show", action="store_true",
                        help="Skip interactive display, only save.")
    args = parser.parse_args()

    print("Building demo panels …")

    n_cases = len(DEMO_CASES)

    # ── Figure layout ──────────────────────────────────────────────────────────
    # Row 0: top-view (2D support polygon), Row 1: 3D robot render, Row 2: sweep
    fig = plt.figure(figsize=(5 * n_cases, 16))
    fig.patch.set_facecolor("white")
    fig.suptitle(
        "Corgi Static Stability Definition  —  Support Polygon & Normalised CoM Margin (η)",
        fontsize=13, fontweight="bold", y=0.99,
    )

    import matplotlib.gridspec as gridspec
    gs = gridspec.GridSpec(3, n_cases, figure=fig,
                           height_ratios=[1, 1, 0.9],
                           hspace=0.65, wspace=0.35)

    top_axes = [fig.add_subplot(gs[0, i]) for i in range(n_cases)]
    bot3d_axes = [fig.add_subplot(gs[1, i], projection="3d") for i in range(n_cases)]
    ax_sweep = fig.add_subplot(gs[2, :])

    print("  Drawing top-view panels …")
    for ax, case in zip(top_axes, DEMO_CASES):
        m = draw_top_view(ax, case)
        print(f"    {case['label'].split(chr(10))[0]:35s}  η = {m:+.3f}")

    print("  Drawing 3-D robot panels …")
    for ax3d, case in zip(bot3d_axes, DEMO_CASES):
        draw_robot_3d(ax3d, case)
        # Match title colour to the case
        ax3d.set_title(f"3-D  Roll={case['roll']}°",
                       color=case["color"], fontsize=8, fontweight="bold", pad=4)

    # Shared legend for row 0
    legend_handles = [
        mpatches.Patch(facecolor="limegreen", edgecolor="#1a7a1a", alpha=0.5, label="Support polygon"),
        plt.Line2D([0], [0], color="red", lw=2, label="Nearest edge"),
        plt.Line2D([0], [0], color="purple", lw=1.8, label="Margin distance"),
        plt.scatter([], [], c="gold", edgecolors="black", s=100,
                    marker="*", label="Contact pivots"),
        plt.scatter([], [], c="#1a7a1a", s=100, marker="D", label="CoM (stable)"),
        plt.scatter([], [], c="#c0392b", s=100, marker="D", label="CoM (unstable)"),
        mpatches.Patch(fill=False, edgecolor="#888", ls="--", label="Inscribed circle (r)"),
    ]
    fig.legend(handles=legend_handles, loc="lower center",
               bbox_to_anchor=(0.5, 0.665), ncol=7, fontsize=8, framealpha=0.85)

    print("  Running margin sweep (Roll 0°→180°, step 5°) …")
    draw_margin_sweep(ax_sweep)

    plt.tight_layout(rect=[0, 0, 1, 0.98])
    plt.subplots_adjust(hspace=0.65, bottom=0.05)

    # ── Save ──────────────────────────────────────────────────────────────────
    os.makedirs(args.save_dir, exist_ok=True)
    save_path = os.path.join(args.save_dir, "stability_definition.png")
    plt.savefig(save_path, dpi=180, bbox_inches="tight")
    print(f"\nSaved → {save_path}")

    if not args.no_show:
        plt.show()
    plt.close("all")


if __name__ == "__main__":
    main()
