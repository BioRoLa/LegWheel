"""
Improved beta comparison demo for self-righting phase checks.

This script compares beta=0 and beta=90 over a roll sweep, then renders
representative boundary-state poses for both cases.

Outputs are indexed for stable filename sorting.
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from legwheel.models.collision_model import CorgiCollisionModel
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.models.corgi_robot import CorgiRobot

from examples.self_righting.boundary_states import render_boundary_sequence as renderer

M_TOTAL = 27.5
M_LEG = 1.0
M_BODY = 23.5
LEG_COM_FRAC = 0.40


def system_com_b(q_list):
    weighted_sum = M_BODY * np.zeros(3)
    for i, q in enumerate(q_list):
        leg = CorgiLegKinematics(i)
        weighted_sum += M_LEG * (
            leg.p_Mi_in_B + LEG_COM_FRAC * (leg.forward_kinematics(*q) - leg.p_Mi_in_B)
        )
    return weighted_sum / M_TOTAL


def stability_margin(roll_deg, q_list, tol=4e-3):
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), 0.0, 0.0])
    robot.base_pos = np.array([0.0, 0.0, 0.5])

    col = CorgiCollisionModel(robot)
    pts = col.get_all_collision_points(q_list)
    c = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= c[:, 2].min()

    com_w = robot.body_to_world(system_com_b(q_list))
    pts2 = col.get_all_collision_points(q_list)
    c2 = np.vstack([pts2["chassis"], pts2["m6_studs"], pts2["wheels"]])
    xy = c2[c2[:, 2] <= c2[:, 2].min() + tol, :2]

    if len(xy) < 3:
        return -1.0

    try:
        hull = ConvexHull(xy)
        vals = hull.equations @ np.append(com_w[:2], 1.0)
        dist = -vals.max()
        hp = xy[hull.vertices]
        n = len(hp)
        perim = sum(np.linalg.norm(hp[(i + 1) % n] - hp[i]) for i in range(n))
        return float(dist / (2 * hull.volume / perim))
    except Exception:
        return -1.0


def grid_search_best(roll_deg, beta_deg, theta_choices, gamma_l_choices, gamma_u_choices):
    best_margin = -99.0
    best_cfg = None
    best_q_list = None

    for theta in theta_choices:
        for gamma_l in gamma_l_choices:
            for gamma_u in gamma_u_choices:
                q_l = [np.deg2rad(theta), np.deg2rad(beta_deg), np.deg2rad(gamma_l)]
                q_u = [np.deg2rad(theta), np.deg2rad(beta_deg), np.deg2rad(gamma_u)]
                candidates = [[q_u, q_l, q_l, q_u], [q_l, q_u, q_u, q_l]]

                for q_list in candidates:
                    margin = stability_margin(roll_deg, q_list)
                    if margin > best_margin:
                        best_margin = margin
                        best_cfg = {
                            "theta": theta,
                            "beta": beta_deg,
                            "gamma_l": gamma_l,
                            "gamma_u": gamma_u,
                        }
                        best_q_list = q_list

    return best_margin, best_cfg, best_q_list


# ---------------------------------------------------------------------------
# Cone-contact gamma analysis for beta=0
# ---------------------------------------------------------------------------

def get_best_beta0(roll_deg, theta_choices=(30, 45, 60, 75, 90)):
    """
    For beta=0, find the optimal gamma for the *lower-side* legs using the
    cone-on-ground criterion: sweep g_lower so that the wheel rim circle
    (the cone cross-section at that theta) intersects z=0 simultaneously
    with the chassis corner contact.

    Physical reasoning:
    - At beta=0 the leg end-effector traces a cone surface as gamma varies.
    - For a tilted robot, the contact circle (cone cross-section) is an ellipse
      in world space that may intersect the ground plane.
    - The intersection gives gamma values where the rim touches the ground.
    - By projecting this cone onto the {B} YZ plane, you can visualise the
      two "cone lines" from the ABAD apex.  Adjusting gamma to bring one of
      those projected lines tangent to the world horizontal = the wheel on
      the ground, adding a real support polygon to the existing chassis contact.
    - Upper-side legs use gamma=0; the tilted chassis corner provides their
      ground contact automatically.

    Args:
        roll_deg: body roll angle (deg). Positive = right-side down.
        theta_choices: extension angles to try.

    Returns:
        (best_margin, best_cfg, best_q_list)
    """
    best_margin = -99.0
    best_cfg = None
    best_q_list = None

    # Identify lower-side legs from roll direction:
    # roll > 0 → right side down → lower = FR (1) and RR (2)
    # roll < 0 → left  side down → lower = FL (0) and RL (3)
    right_side_down = roll_deg >= 0

    g_upper = 0  # upper-side legs: chassis handles their contact

    for theta_deg in theta_choices:
        # Stage 1: coarse 5-degree sweep
        best_g_coarse = 0.0
        best_m_coarse = -99.0
        for g_lower in np.linspace(-60, 60, 25):
            if right_side_down:
                q_list = [
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_upper)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_lower)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_lower)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_upper)],
                ]
            else:
                q_list = [
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_lower)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_upper)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_upper)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_lower)],
                ]
            m = stability_margin(roll_deg, q_list)
            if m > best_m_coarse:
                best_m_coarse = m
                best_g_coarse = g_lower

        # Stage 2: fine 1-degree sweep around coarse best
        for g_lower in np.linspace(best_g_coarse - 5, best_g_coarse + 5, 21):
            if right_side_down:
                q_list = [
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_upper)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_lower)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_lower)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_upper)],
                ]
            else:
                q_list = [
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_lower)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_upper)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_upper)],
                    [np.deg2rad(theta_deg), 0.0, np.deg2rad(g_lower)],
                ]
            margin = stability_margin(roll_deg, q_list)
            if margin > best_margin:
                best_margin = margin
                best_cfg = {
                    "theta": theta_deg,
                    "beta": 0,
                    "g_lower": round(float(g_lower), 1),
                    "g_upper": g_upper,
                    "gammas": [round(np.rad2deg(q[2]), 1) for q in q_list],
                }
                best_q_list = q_list

    return best_margin, best_cfg, best_q_list


# ---------------------------------------------------------------------------


def render_pose_snapshot(roll_deg, q_list, title, save_path, show=False):
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), 0.0, 0.0])
    robot.base_pos = np.array([0.0, 0.0, 0.5])

    col = CorgiCollisionModel(robot)
    pts = col.get_all_collision_points(q_list)
    all_p = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= all_p[:, 2].min()

    fig = plt.figure(figsize=(8.0, 6.0))
    fig.patch.set_facecolor("#1a1a2e")
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    ax.set_facecolor("#16213e")

    renderer._draw_ground(ax)
    renderer._draw_chassis_wireframe(ax, robot, color="white")
    renderer._draw_legs_world(ax, robot, q_list)
    renderer._draw_collision_markers(ax, robot, col, q_list)
    renderer._draw_body_frame(ax, robot)

    ax.view_init(elev=20, azim=-55)
    ax.set_xlabel("X", color="white", fontsize=8)
    ax.set_ylabel("Y", color="white", fontsize=8)
    ax.set_zlabel("Z", color="white", fontsize=8)
    ax.tick_params(colors="white", labelsize=7)
    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.5, 0.5)
    ax.set_zlim(0.0, 0.55)
    ax.set_box_aspect([1, 1, 0.55])

    handles, labels = ax.get_legend_handles_labels()
    if handles:
        fig.legend(
            handles,
            labels,
            loc="lower center",
            ncol=min(len(handles), 4),
            fontsize=7,
            facecolor="#1a1a2e",
            labelcolor="white",
            framealpha=0.7,
            bbox_to_anchor=(0.5, 0.0),
        )

    fig.suptitle(title, color="white", fontsize=11, y=0.98)
    plt.tight_layout(rect=[0, 0.05, 1, 1])
    plt.savefig(save_path, dpi=220, bbox_inches="tight", facecolor=fig.get_facecolor())
    if show:
        plt.show()
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description="Compare beta=0 and beta=90 with improved visual outputs.")
    parser.add_argument("--save-dir", default="output/beta_0_90_comparison", help="Output directory")
    parser.add_argument("--no-show", action="store_true", help="Do not open windows")
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.save_dir, exist_ok=True)

    theta_choices = [45, 60, 75]
    gamma_l_choices = [-30, -10, 0, 10, 30]
    gamma_u_choices = [-90, 0, 90]
    rolls = list(range(85, 49, -5))

    results = {0: [], 90: []}

    print("Running beta=0 sweep (cone-parallel gamma criterion)...")
    for roll in rolls:
        margin, cfg, q_list = get_best_beta0(roll, theta_choices=(45, 60, 75, 90))
        results[0].append({"roll": roll, "margin": margin, "cfg": cfg, "q_list": q_list})
        gammas_str = ", ".join(str(g) for g in cfg["gammas"])
        print(
            f"  beta=  0, roll={roll:>3d}, margin={margin:+.4f}, "
            f"theta={cfg['theta']}, gammas=[{gammas_str}]"
        )

    print("Running beta=90 sweep (grid search)...")
    for roll in rolls:
        margin, cfg, q_list = grid_search_best(
            roll, 90, theta_choices, gamma_l_choices, gamma_u_choices
        )
        results[90].append({"roll": roll, "margin": margin, "cfg": cfg, "q_list": q_list})
        print(
            f"  beta= 90, roll={roll:>3d}, margin={margin:+.4f}, "
            f"theta={cfg['theta']}, gamma_l={cfg['gamma_l']}, gamma_u={cfg['gamma_u']}"
        )

    margin0 = np.array([r["margin"] for r in results[0]])
    margin90 = np.array([r["margin"] for r in results[90]])
    delta = margin90 - margin0

    fig, ax = plt.subplots(figsize=(9, 5.5))
    ax.plot(rolls, margin0, marker="o", lw=2.2, color="#1f77b4", label="beta=0")
    ax.plot(rolls, margin90, marker="s", lw=2.2, color="#d62728", label="beta=90")
    ax.plot(rolls, delta, marker="^", lw=1.8, color="#2ca02c", linestyle="--", label="delta (90-0)")
    ax.axhline(0.0, color="#666666", lw=1.0)
    ax.set_xlabel("Roll (deg)")
    ax.set_ylabel("Normalized stability margin")
    ax.set_title("Phase-3 region beta comparison (improved demo)")
    ax.grid(alpha=0.3)
    ax.legend()
    summary_path = os.path.join(args.save_dir, "01_margin_sweep_beta_0_90.png")
    plt.tight_layout()
    plt.savefig(summary_path, dpi=220)
    if not args.no_show:
        plt.show(block=False)
        plt.pause(2.0)
    plt.close(fig)

    idx = int(np.argmax(np.abs(delta)))
    target_roll = rolls[idx]
    pose0 = results[0][idx]
    pose90 = results[90][idx]

    gammas_str = ", ".join(str(g) for g in pose0["cfg"]["gammas"])
    title0 = (
        f"beta=0 (cone-on-ground) at roll={target_roll}\u00b0 | margin={pose0['margin']:+.4f} | "
        f"theta={pose0['cfg']['theta']}\u00b0, \u03b3_lower={pose0['cfg']['g_lower']}\u00b0"
    )
    title90 = (
        f"beta=90 (grid search) at roll={target_roll}\u00b0 | margin={pose90['margin']:+.4f} | "
        f"theta={pose90['cfg']['theta']}\u00b0, \u03b3_l={pose90['cfg']['gamma_l']}\u00b0, \u03b3_u={pose90['cfg']['gamma_u']}\u00b0"
    )

    pose0_path = os.path.join(args.save_dir, "02_pose_beta_0.png")
    pose90_path = os.path.join(args.save_dir, "03_pose_beta_90.png")

    render_pose_snapshot(target_roll, pose0["q_list"], title0, pose0_path, show=False)
    render_pose_snapshot(target_roll, pose90["q_list"], title90, pose90_path, show=False)

    print("Saved files:")
    print(f"  {summary_path}")
    print(f"  {pose0_path}")
    print(f"  {pose90_path}")
    print(f"Representative roll chosen by max |delta|: {target_roll} deg")


if __name__ == "__main__":
    main()
