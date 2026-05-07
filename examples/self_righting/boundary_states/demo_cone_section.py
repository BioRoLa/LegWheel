"""
demo_cone_section.py
====================

Visualises the cone topology of beta=0 contact locus and explains
why adjusting gamma can bring the wheel rim to the ground.

Physical model
--------------
  For beta=0, theta fixed, the ABAD joint (gamma) rotates the contact point
  around the hip-roll axis.  In Body Frame {B} this traces a circle centred
  on the ABAD axis.  In World Frame {W} (after applying body roll), this
  circle becomes an ellipse that may intersect the ground plane z=0.
  The gamma values where z_world = 0 are the "cone-on-ground" solutions.

Layout (2 rows × 3 cols)
-------------------------
  [0,0] 3D body-frame locus  — Full circle for theta=60, all gamma
  [0,1] 3D world-frame at roll=80 — Same circle + z=0 ground plane,
                                     contact-on-ground gammas highlighted
  [0,2] gamma vs z_world curve  — Clearly shows the two zero-crossings
  [1,0] YZ-plane projection     — The "cone cross-section" in body frame
  [1,1] gamma sensitivity sweep — margin vs g_lower at three roll values
  [1,2] Pose snapshot at optimal gamma (roll=80)

Usage
-----
    uv run python examples/self_righting/boundary_states/demo_cone_section.py --no-show
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from scipy.spatial import ConvexHull

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.models.collision_model import CorgiCollisionModel
from render.plot_leg_envelope import draw_leg_envelope

import examples.self_righting.boundary_states.render_boundary_sequence as renderer

# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────

THETA_DEG = 60        # extension angle used throughout
LEG_IDX = 1           # FR leg (right side, lower at positive roll)
DEMO_ROLLS = [80, 65, 50]
DEMO_ROLL_COLORS = ["#e74c3c", "#e67e22", "#f1c40f"]
GAMMA_FINE = np.linspace(-90, 90, 361)

# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def contact_locus_body(theta_deg, leg_idx=1):
    """All contact positions in {B} for beta=0, theta fixed, gamma sweep."""
    leg = CorgiLegKinematics(leg_idx)
    pts = []
    for g in GAMMA_FINE:
        p_b = leg.forward_kinematics(np.deg2rad(theta_deg), 0.0, np.deg2rad(g))
        pts.append(p_b)
    return np.array(pts)


def contact_locus_world(theta_deg, roll_deg, leg_idx=1):
    """
    All contact positions in {W} for beta=0, theta fixed, gamma sweep,
    with robot body at given roll.  Robot is NOT grounded — raw world frame.
    """
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), 0.0, 0.0])
    robot.base_pos = np.zeros(3)    # body origin at world origin for clarity
    leg = CorgiLegKinematics(leg_idx)
    R = robot._rot_matrix(robot.base_ori)
    pts = []
    for g in GAMMA_FINE:
        p_b = leg.forward_kinematics(np.deg2rad(theta_deg), 0.0, np.deg2rad(g))
        p_w = R @ p_b   # no translation offset — only rotation
        pts.append(p_w)
    return np.array(pts)


def gamma_vs_z(theta_deg, roll_deg, leg_idx=1):
    """Return (gamma_array_deg, z_world_array) without body-frame offset."""
    pts = contact_locus_world(theta_deg, roll_deg, leg_idx)
    return GAMMA_FINE, pts[:, 2]


def find_zero_crossings(gammas, z_vals):
    """Linear interpolation of z=0 crossings."""
    crossings = []
    for i in range(len(z_vals) - 1):
        if z_vals[i] * z_vals[i + 1] < 0:
            t = -z_vals[i] / (z_vals[i + 1] - z_vals[i])
            g_cross = gammas[i] + t * (gammas[i + 1] - gammas[i])
            crossings.append(g_cross)
    return crossings


def stability_margin_beta0(roll_deg, theta_deg, g_lower_deg, tol=4e-3):
    """Quick stability margin for beta=0 with symmetric lower gamma."""
    g_upper = 0.0
    q_list = [
        [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_upper)],  # FL
        [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_lower_deg)],  # FR
        [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_lower_deg)],  # RR
        [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_upper)],  # RL
    ]
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), 0.0, 0.0])
    robot.base_pos = np.array([0.0, 0.0, 0.5])
    col = CorgiCollisionModel(robot)
    pts = col.get_all_collision_points(q_list)
    all_p = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= all_p[:, 2].min()
    pts2 = col.get_all_collision_points(q_list)
    all_p2 = np.vstack([pts2["chassis"], pts2["m6_studs"], pts2["wheels"]])
    z_min = all_p2[:, 2].min()
    contact_xy = all_p2[all_p2[:, 2] <= z_min + tol, :2]
    if len(contact_xy) < 3:
        return -1.0
    try:
        hull = ConvexHull(contact_xy)
        com_w = robot.body_to_world(np.zeros(3))
        vals = hull.equations @ np.append(com_w[:2], 1.0)
        dist = -vals.max()
        hp = contact_xy[hull.vertices]
        n = len(hp)
        perim = sum(np.linalg.norm(hp[(i + 1) % n] - hp[i]) for i in range(n))
        insc_r = 2 * hull.volume / perim if perim > 1e-6 else 1.0
        return float(dist / insc_r)
    except Exception:
        return -1.0


def build_cone_triangles(apex, locus):
    """Triangulate a single-leg cone surface from apex to swept contact locus."""
    triangles = []
    for idx in range(len(locus) - 1):
        triangles.append([apex, locus[idx], locus[idx + 1]])
    triangles.append([apex, locus[-1], locus[0]])
    return triangles


# ─────────────────────────────────────────────────────────────────────────────
# Panel helpers
# ─────────────────────────────────────────────────────────────────────────────

def draw_robot_with_cone_overlay(ax, roll_deg=0):
    """
    Panel [0,0]: Physical robot render with ABAD rotation cone and section plane.
    The red ring = contact locus (all gamma, beta=0) = intersection of the ABAD
    rotation cone with the green section plane (perpendicular to ABAD axis).
    """
    q_list = [[np.deg2rad(THETA_DEG), 0.0, 0.0]] * 4
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), 0.0, 0.0])
    robot.base_pos = np.array([0.0, 0.0, 0.5])
    col = CorgiCollisionModel(robot)
    pts = col.get_all_collision_points(q_list)
    all_p = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= all_p[:, 2].min()

    # Light background
    ax.set_facecolor("#f5f5f5")
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.fill = True
        pane.set_facecolor("#eeeeee")
        pane.set_edgecolor("#cccccc")

    # Robot body
    renderer._draw_ground(ax)
    renderer._draw_chassis_wireframe(ax, robot, color="#333333")
    renderer._draw_legs_world(ax, robot, q_list)
    renderer._draw_body_frame(ax, robot, scale=0.12)

    leg = CorgiLegKinematics(LEG_IDX)
    R = robot._rot_matrix(robot.base_ori)

    # Thick single-leg cone using the LegWheel 3D envelope logic
    draw_leg_envelope(
        ax,
        LEG_IDX,
        np.deg2rad(THETA_DEG),
        0.0,
        gamma_max_deg=90,
        n_gamma_slices=1,
        N_alpha=40,
        N_t=10,
        point_transform=robot.body_to_world,
        gamma_samples_deg=[0.0],
    )

    # Keep the gamma sweep locus for defining the section plane only
    locus_w = np.array([
        robot.base_pos + R @ leg.forward_kinematics(np.deg2rad(THETA_DEG), 0.0, np.deg2rad(g))
        for g in GAMMA_FINE
    ])

    # ABAD joint and gamma=0 arm
    abad_w = robot.base_pos + R @ leg.p_Mi_in_B
    ax.scatter(*abad_w, c="orange", s=200, marker="^", zorder=8,
               depthshade=False, label="ABAD joint (FR)")
    c0_w = robot.base_pos + R @ leg.forward_kinematics(np.deg2rad(THETA_DEG), 0.0, 0.0)
    ax.plot([abad_w[0], c0_w[0]], [abad_w[1], c0_w[1]], [abad_w[2], c0_w[2]],
            lw=2.5, color="#e74c3c", zorder=5)
    ax.scatter(*c0_w, c="lime", s=120, marker="D", zorder=7,
               depthshade=False, label="Contact at gamma=0")

    # ABAD axis arrow
    abad_axis_w = R[:, 0]
    span = 0.15
    ax.quiver(abad_w[0], abad_w[1], abad_w[2],
              span * abad_axis_w[0], span * abad_axis_w[1], span * abad_axis_w[2],
              color="#f39c12", lw=2.5, arrow_length_ratio=0.25,
              label="ABAD axis (Body X)")

    # Section plane: disc perpendicular to ABAD axis at locus centroid
    locus_center_w = locus_w.mean(axis=0)
    n_vec = abad_axis_w / np.linalg.norm(abad_axis_w)
    ref = np.array([0.0, 1.0, 0.0]) if abs(n_vec[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u_vec = np.cross(n_vec, ref);  u_vec /= np.linalg.norm(u_vec)
    v_vec = np.cross(n_vec, u_vec)
    r_disc = float(np.max(np.linalg.norm(locus_w - locus_center_w, axis=1))) * 1.3
    theta_d = np.linspace(0, 2 * np.pi, 64)
    disc = (locus_center_w[:, None]
            + r_disc * (u_vec[:, None] * np.cos(theta_d)
                        + v_vec[:, None] * np.sin(theta_d))).T
    poly = Poly3DCollection(
        [disc.tolist()], alpha=0.20, facecolor="#2ecc71", edgecolor="#27ae60", lw=1.5
    )
    ax.add_collection3d(poly)
    ax.plot(disc[:, 0], disc[:, 1], disc[:, 2],
            lw=1.8, color="#27ae60", ls="--", alpha=0.85,
            label="Section plane (perp. to ABAD axis)")

    # Annotation
    ax.text2D(0.02, 0.95,
              f"LegWheel thick cone + section plane\nFR leg, beta=0, theta={THETA_DEG}deg",
              transform=ax.transAxes, fontsize=8, color="#333333",
              bbox=dict(boxstyle="round", fc="white", ec="#777777", alpha=0.9, lw=1.0))

    ax.set_xlabel("X (m)", fontsize=7, color="black")
    ax.set_ylabel("Y (m)", fontsize=7, color="black")
    ax.set_zlabel("Z (m)", fontsize=7, color="black")
    ax.tick_params(colors="black", labelsize=5)
    ax.set_xlim(-0.45, 0.45)
    ax.set_ylim(-0.45, 0.45)
    ax.set_zlim(0.0, 0.60)
    ax.set_box_aspect([1, 1, 0.60])
    ax.set_title(
        f"Robot + Thick Leg Cone + Section Plane\n(Roll={roll_deg}deg, FR, beta=0, theta={THETA_DEG}deg)",
        fontsize=9, fontweight="bold", color="black",
    )
    ax.legend(fontsize=6.5, loc="upper right",
              facecolor="white", edgecolor="#cccccc", framealpha=0.9)
    ax.view_init(elev=20, azim=-30)


def draw_single_leg_cone_render(ax):
    """Standalone single-leg cone render in body frame for quick inspection."""
    ax.set_facecolor("#f5f5f5")
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.fill = True
        pane.set_facecolor("#eeeeee")
        pane.set_edgecolor("#cccccc")

    draw_leg_envelope(
        ax,
        LEG_IDX,
        np.deg2rad(THETA_DEG),
        0.0,
        gamma_max_deg=90,
        n_gamma_slices=1,
        N_alpha=40,
        N_t=10,
        gamma_samples_deg=[0.0],
    )

    ax.text2D(0.03, 0.94,
              f"Single-leg thick cone preview\nFR leg, beta=0, theta={THETA_DEG}deg",
              transform=ax.transAxes, fontsize=8, color="#333333",
              bbox=dict(boxstyle="round", fc="white", ec="#888888", alpha=0.9, lw=0.8))

    ax.set_xlabel("X_B (m)", fontsize=7, color="black")
    ax.set_ylabel("Y_B (m)", fontsize=7, color="black")
    ax.set_zlabel("Z_B (m)", fontsize=7, color="black")
    ax.tick_params(colors="black", labelsize=6)
    ax.set_xlim(-0.18, 0.32)
    ax.set_ylim(-0.30, 0.30)
    ax.set_zlim(-0.30, 0.20)
    ax.set_box_aspect([0.40, 0.60, 0.50])
    ax.set_title("Single-Leg Thick Cone Render", fontsize=10, fontweight="bold", color="black")
    ax.legend(fontsize=7, loc="upper right", facecolor="white", edgecolor="#cccccc", framealpha=0.9)
    ax.view_init(elev=22, azim=-55)


def save_single_leg_cone_preview(save_dir):
    """Export a standalone preview of the single-leg cone before the full demo."""
    fig = plt.figure(figsize=(7.2, 6.4))
    fig.patch.set_facecolor("white")
    ax = fig.add_subplot(111, projection="3d")
    draw_single_leg_cone_render(ax)
    save_path = os.path.join(save_dir, "single_leg_cone_preview.png")
    plt.savefig(save_path, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return save_path


def _draw_cone_3d(ax, locus, title, abad_pos, roll_label=None, ground_z=None):
    """Draw 3D contact locus with ABAD apex and optional ground plane."""
    # Locus ring
    ax.plot(locus[:, 0], locus[:, 1], locus[:, 2],
            lw=1.8, color="#3498db", alpha=0.9, label="Contact locus")
    ax.scatter(locus[::20, 0], locus[::20, 1], locus[::20, 2],
               c="#3498db", s=18, depthshade=False)

    # ABAD apex
    ax.scatter(*abad_pos, c="orange", s=120, marker="^",
               zorder=6, label="ABAD apex", depthshade=False)

    # Lines from ABAD to a few points on locus (cone generators)
    for idx in range(0, len(locus), 45):
        ax.plot([abad_pos[0], locus[idx, 0]],
                [abad_pos[1], locus[idx, 1]],
                [abad_pos[2], locus[idx, 2]],
                lw=0.7, color="#95a5a6", alpha=0.5)

    # Ground plane (z=0 or given)
    if ground_z is not None:
        # Highlight points near z=ground_z
        near = np.abs(locus[:, 2] - ground_z) < 0.015
        if near.any():
            ax.scatter(locus[near, 0], locus[near, 1], locus[near, 2],
                       c="#e74c3c", s=80, zorder=7, label="Ground contact γ", depthshade=False)
        # Draw z=0 plane
        span = 0.35
        cx, cy = locus[:, 0].mean(), locus[:, 1].mean()
        xs, ys = np.meshgrid([cx - span, cx + span], [cy - span, cy + span])
        zs = np.full_like(xs, ground_z)
        ax.plot_surface(xs, ys, zs, alpha=0.12, color="gray", zorder=0)

    ax.set_xlabel("X (m)", fontsize=7)
    ax.set_ylabel("Y (m)", fontsize=7)
    ax.set_zlabel("Z (m)", fontsize=7)
    ax.tick_params(labelsize=6)
    ax.set_title(title, fontsize=9, fontweight="bold")
    if roll_label:
        ax.text2D(0.02, 0.95, roll_label, transform=ax.transAxes, fontsize=8, color="#333333",
                  bbox=dict(boxstyle="round", fc="white", ec="#555555", alpha=0.85, lw=0.8))


def draw_body_frame_locus(ax):
    """Panel [0,0]: 3D locus in body frame."""
    locus_b = contact_locus_body(THETA_DEG, LEG_IDX)
    leg = CorgiLegKinematics(LEG_IDX)
    abad = leg.p_Mi_in_B
    _draw_cone_3d(ax, locus_b, f"Contact locus — Body Frame\n(θ={THETA_DEG}°, β=0, γ: −90°→+90°)", abad)
    # Gamma=0 point
    p0_b = leg.forward_kinematics(np.deg2rad(THETA_DEG), 0.0, 0.0)
    ax.scatter(*p0_b, c="lime", s=100, marker="D", zorder=8, label="γ=0", depthshade=False)
    ax.legend(fontsize=7, loc="upper right")
    ax.view_init(elev=18, azim=40)


def draw_world_frame_locus(ax, roll_deg=80):
    """Panel [0,1]: 3D locus in world frame (body at roll, origin at hip level)."""
    locus_w = contact_locus_world(THETA_DEG, roll_deg, LEG_IDX)
    # ABAD position in world (body at origin)
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), 0.0, 0.0])
    robot.base_pos = np.zeros(3)
    R = robot._rot_matrix(robot.base_ori)
    abad_w = R @ CorgiLegKinematics(LEG_IDX).p_Mi_in_B

    # Find z=0 crossings
    gammas, z_vals = gamma_vs_z(THETA_DEG, roll_deg, LEG_IDX)
    crossings = find_zero_crossings(gammas, z_vals)

    ground_z = 0.0
    _draw_cone_3d(ax, locus_w,
                  f"Contact locus — World Frame\n(Roll={roll_deg}°, θ={THETA_DEG}°, β=0)",
                  abad_w, ground_z=ground_z,
                  roll_label=f"Roll={roll_deg}°")

    # Mark crossing gammas
    for gc in crossings:
        robot2 = CorgiRobot()
        robot2.base_ori = np.array([np.deg2rad(roll_deg), 0.0, 0.0])
        robot2.base_pos = np.zeros(3)
        R2 = robot2._rot_matrix(robot2.base_ori)
        leg = CorgiLegKinematics(LEG_IDX)
        p_b = leg.forward_kinematics(np.deg2rad(THETA_DEG), 0.0, np.deg2rad(gc))
        p_w = R2 @ p_b
        ax.scatter(*p_w, c="#e74c3c", s=200, marker="*", zorder=9, depthshade=False)
        ax.text(p_w[0], p_w[1], p_w[2] + 0.03, f"γ={gc:.1f}°",
                fontsize=7, color="#e74c3c", fontweight="bold")

    ax.legend(fontsize=7, loc="upper right")
    ax.view_init(elev=18, azim=-45)


def draw_gamma_z_curve(ax):
    """Panel [0,2]: gamma vs z_world for multiple roll values."""
    ax.axhline(0, color="black", lw=1.5, ls="--", label="Ground (z=0)")
    for roll, color in zip(DEMO_ROLLS, DEMO_ROLL_COLORS):
        gammas, z_vals = gamma_vs_z(THETA_DEG, roll, LEG_IDX)
        ax.plot(gammas, z_vals, lw=2.0, color=color, label=f"Roll={roll}°")
        crossings = find_zero_crossings(gammas, z_vals)
        for gc in crossings:
            ax.axvline(gc, color=color, lw=0.8, ls=":", alpha=0.7)
            ax.scatter(gc, 0, color=color, s=80, marker="*", zorder=5)
            ax.annotate(f"γ={gc:.1f}°", xy=(gc, 0), xytext=(gc + 4, 0.03),
                        fontsize=7, color=color, arrowprops=dict(arrowstyle="->", color=color, lw=0.8))

    ax.set_xlabel("γ (deg)", fontsize=9)
    ax.set_ylabel("Contact z — world (m)", fontsize=9)
    ax.set_title(f"γ sweep → world z of contact\n(θ={THETA_DEG}°, β=0, FR leg)", fontsize=9, fontweight="bold")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_xlim(-95, 95)


def draw_yz_projection(ax):
    """Panel [1,0]: Body-frame YZ projection — the true cone cross-section."""
    locus_b = contact_locus_body(THETA_DEG, LEG_IDX)
    leg = CorgiLegKinematics(LEG_IDX)
    abad = leg.p_Mi_in_B

    # Project onto YZ plane (x = constant)
    ax.plot(locus_b[:, 1], locus_b[:, 2], lw=2.0, color="#3498db", label="Contact locus (YZ)")
    ax.scatter(abad[1], abad[2], c="orange", s=150, marker="^", zorder=5, label="ABAD apex")

    # Draw lines from ABAD to locus for cone visualization
    for idx in range(0, len(locus_b), 36):
        ax.plot([abad[1], locus_b[idx, 1]], [abad[2], locus_b[idx, 2]],
                lw=0.7, color="#95a5a6", alpha=0.6)

    # Mark gamma=0 point
    p0_b = leg.forward_kinematics(np.deg2rad(THETA_DEG), 0.0, 0.0)
    ax.scatter(p0_b[1], p0_b[2], c="lime", s=120, marker="D", zorder=6, label="γ=0")

    # Body frame: +Y is right, +Z is up. The body is at roll=0.
    ax.axhline(0, color="gray", lw=0.8, ls="--", alpha=0.5)
    ax.set_xlabel("Y_body (m)", fontsize=9)
    ax.set_ylabel("Z_body (m)", fontsize=9)
    ax.set_title(f"Body-frame YZ projection\n(θ={THETA_DEG}°, β=0) — Cone cross-section view", fontsize=9, fontweight="bold")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # Annotate the rotation direction
    mid_idx = len(locus_b) // 4
    ax.annotate("", xy=(locus_b[mid_idx + 5, 1], locus_b[mid_idx + 5, 2]),
                xytext=(locus_b[mid_idx, 1], locus_b[mid_idx, 2]),
                arrowprops=dict(arrowstyle="->", color="#3498db", lw=1.5))
    ax.text(abad[1] + 0.02, abad[2] - 0.06, "γ rotation\naround ABAD axis",
            fontsize=7.5, color="#3498db")


def draw_margin_vs_gamma(ax):
    """Panel [1,1]: Stability margin vs g_lower for different roll values."""
    g_arr = np.linspace(-60, 60, 61)
    ax.axhline(0, color="black", lw=1.2, ls="--")
    for roll, color in zip(DEMO_ROLLS, DEMO_ROLL_COLORS):
        margins = [stability_margin_beta0(roll, THETA_DEG, g) for g in g_arr]
        ax.plot(g_arr, margins, lw=2.0, color=color, label=f"Roll={roll}°")
        best_g = g_arr[np.argmax(margins)]
        best_m = max(margins)
        ax.scatter(best_g, best_m, c=color, s=100, marker="*", zorder=5)
        ax.annotate(f"γ_opt={best_g:.0f}°", xy=(best_g, best_m),
                    xytext=(best_g + 4, best_m + 0.05),
                    fontsize=7, color=color,
                    arrowprops=dict(arrowstyle="->", color=color, lw=0.8))

    ax.fill_between(g_arr, 0, 1.2, alpha=0.08, color="green")
    ax.set_xlabel("γ_lower (deg) — lower-side legs", fontsize=9)
    ax.set_ylabel("Normalised stability margin η", fontsize=9)
    ax.set_title(f"Margin vs γ_lower  (θ={THETA_DEG}°, β=0)\n"
                 "Upper-side legs: γ=0 (chassis contact)", fontsize=9, fontweight="bold")
    ax.set_ylim(-1.2, 1.2)
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)


def draw_optimal_pose_snapshot(ax, roll_deg=80):
    """Panel [1,2]: 3D robot pose at optimal gamma for given roll."""
    # Find best gamma for this roll
    g_arr = np.linspace(-60, 60, 61)
    margins = [stability_margin_beta0(roll_deg, THETA_DEG, g) for g in g_arr]
    best_g = float(g_arr[np.argmax(margins)])

    q_list = [
        [np.deg2rad(THETA_DEG), 0.0, 0.0],
        [np.deg2rad(THETA_DEG), 0.0, np.deg2rad(best_g)],
        [np.deg2rad(THETA_DEG), 0.0, np.deg2rad(best_g)],
        [np.deg2rad(THETA_DEG), 0.0, 0.0],
    ]

    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), 0.0, 0.0])
    robot.base_pos = np.array([0.0, 0.0, 0.5])
    col = CorgiCollisionModel(robot)
    pts = col.get_all_collision_points(q_list)
    all_p = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= all_p[:, 2].min()

    ax.set_facecolor("#f5f5f5")
    for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
        pane.fill = True
        pane.set_facecolor("#eeeeee")
        pane.set_edgecolor("#cccccc")
    renderer._draw_ground(ax)
    renderer._draw_chassis_wireframe(ax, robot, color="#222222")
    renderer._draw_legs_world(ax, robot, q_list)
    renderer._draw_collision_markers(ax, robot, col, q_list)
    renderer._draw_body_frame(ax, robot)

    ax.view_init(elev=20, azim=-40)
    ax.set_xlabel("X", color="black", fontsize=7)
    ax.set_ylabel("Y", color="black", fontsize=7)
    ax.set_zlabel("Z", color="black", fontsize=7)
    ax.tick_params(colors="black", labelsize=6)
    ax.set_xlim(-0.45, 0.45)
    ax.set_ylim(-0.45, 0.45)
    ax.set_zlim(0.0, 0.55)
    ax.set_box_aspect([1, 1, 0.55])
    ax.set_title(
        f"Optimal pose  Roll={roll_deg}°\nβ=0, θ={THETA_DEG}°, γ_lower={best_g:.0f}°",
        fontsize=9, fontweight="bold", color="black",
    )


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--save-dir", "-o", default="output/cone_section_demo")
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    print("Building cone section demo …")
    os.makedirs(args.save_dir, exist_ok=True)
    preview_path = save_single_leg_cone_preview(args.save_dir)
    print(f"  [preview] Single-leg cone render … -> {preview_path}")

    # Layout: 2 rows × 4 cols
    #   [0,0] Leg geometry + ABAD cone + section plane   ← NEW
    #   [0,1] 3D body-frame locus (was [0,0])
    #   [0,2] 3D world-frame at roll=80 + ground plane
    #   [0,3] γ vs z_world curve
    #   [1,0] YZ projection (cone cross-section)
    #   [1,1] Margin vs γ sweep
    #   [1,2] Optimal pose snapshot
    #   [1,3] empty / spare
    fig = plt.figure(figsize=(26, 12))
    fig.patch.set_facecolor("white")
    fig.suptitle(
        "beta=0 Contact Cone: Why Adjusting gamma Can Provide Ground Support\n"
        f"(Leg: FR, theta={THETA_DEG}deg)",
        fontsize=13, fontweight="bold", color="black", y=0.99,
    )

    gs = gridspec.GridSpec(2, 4, figure=fig, hspace=0.48, wspace=0.38)

    # Row 0 -- all 3D (+ one 2D)
    ax00 = fig.add_subplot(gs[0, 0], projection="3d")   # robot + cone + section plane
    ax01 = fig.add_subplot(gs[0, 1], projection="3d")   # body-frame locus
    ax02 = fig.add_subplot(gs[0, 2], projection="3d")   # world-frame + ground
    ax03 = fig.add_subplot(gs[0, 3])                    # gamma vs z curve

    # Row 1 -- 2D + 3D pose snapshot
    ax10 = fig.add_subplot(gs[1, 0])                    # YZ projection
    ax11 = fig.add_subplot(gs[1, 1])                    # margin vs gamma
    ax12 = fig.add_subplot(gs[1, 2], projection="3d")   # optimal pose
    # gs[1,3] left empty

    for ax3d in [ax00, ax01, ax02, ax12]:
        ax3d.set_facecolor("#f5f5f5")
        ax3d.tick_params(colors="black", labelsize=6)
        for pane in [ax3d.xaxis.pane, ax3d.yaxis.pane, ax3d.zaxis.pane]:
            pane.fill = True
            pane.set_facecolor("#eeeeee")
            pane.set_edgecolor("#cccccc")

    for ax2d in [ax03, ax10, ax11]:
        ax2d.set_facecolor("#f8f9fa")

    print("  [0,0] Robot + ABAD cone + section plane (physical) ...")
    draw_robot_with_cone_overlay(ax00)
    print("  [0,1] Body-frame locus …")
    draw_body_frame_locus(ax01)
    print("  [0,2] World-frame locus (roll=80°) …")
    draw_world_frame_locus(ax02, roll_deg=80)
    print("  [0,3] γ vs z_world curve …")
    draw_gamma_z_curve(ax03)
    print("  [1,0] YZ projection …")
    draw_yz_projection(ax10)
    print("  [1,1] Margin vs γ sweep …")
    draw_margin_vs_gamma(ax11)
    print("  [1,2] Optimal pose snapshot …")
    draw_optimal_pose_snapshot(ax12, roll_deg=80)

    save_path = os.path.join(args.save_dir, "cone_section_demo.png")
    plt.savefig(save_path, dpi=180, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"\nSaved → {save_path}")

    if not args.no_show:
        plt.show()
    plt.close("all")


if __name__ == "__main__":
    main()
