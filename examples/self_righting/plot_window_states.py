"""
plot_window_states.py  —  CLI tool to visualize Corgi stability windows.

Usage examples:
    # Show 3-view (front/side/top) for window S4 and save PNG
    uv run python examples/plot_window_states.py --window S4

    # Interactive 3D + front view for S1, keep open 60s
    uv run python examples/plot_window_states.py --window S1 --views front 3d --timeout 60

    # Generate all windows, only front+top, save without showing
    uv run python examples/plot_window_states.py --window all --views front top --no-show

    # Custom save path
    uv run python examples/plot_window_states.py --window S0 S4 --save-dir output/states
"""
import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ── ensure legwheel is importable when run from repo root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.models.collision_model import CorgiCollisionModel
from legwheel.config import RobotParams

# ─────────────────────────────────────────────────────────────────────────────
# Window definitions
# ─────────────────────────────────────────────────────────────────────────────

def _q(theta_deg, beta_deg, gamma_deg):
    return [np.deg2rad(theta_deg), np.deg2rad(beta_deg), np.deg2rad(gamma_deg)]


_q_nom = _q(60, 90, 0)
_q_up  = _q(60, 90, 0)
_q_g30 = _q(60, 90, 30)

WINDOWS = {
    "S0": dict(
        roll=0,  pitch=0,
        q_list=[_q_nom] * 4,
        label="S0 · Upright (γ=0°, Roll=0°)",
        color="#2196F3",
    ),
    "S1": dict(
        roll=85, pitch=0,
        q_list=[_q_nom] * 4,
        label="S1 · SFL-α  Side-Fall Leading (γ=0°, Roll=85°)",
        color="#FF9800",
    ),
    "S2": dict(
        roll=115, pitch=0,
        q_list=[_q_up, _q_g30, _q_g30, _q_up],
        label="S2 · SFT-α  Side-Fall Trailing (γ_lower=30°, Roll=115°)",
        color="#9C27B0",
    ),
    "S3": dict(
        roll=126, pitch=0,
        q_list=[_q_up, _q_g30, _q_g30, _q_up],
        label="S3 · SFT-β  Side-Fall Trailing (γ_lower=30°, Roll=126°)",
        color="#E91E63",
    ),
    "S4": dict(
        roll=180, pitch=0,
        q_list=[_q_nom] * 4,
        label="S4 · Upside-Down (γ=0°, Roll=180°)",
        color="#4CAF50",
    ),
}

# ─────────────────────────────────────────────────────────────────────────────
# View presets  (name → (elev, azim, title))
# ─────────────────────────────────────────────────────────────────────────────

VIEW_PRESETS = {
    "front": (0,   -90, "Front View\n(looking from +X → −X)"),
    "side":  (0,   180, "Side View\n(looking from +Y → −Y)"),
    "top":   (90,    0, "Top View\n(looking from +Z → −Z)"),
    "3d":    (22,  -55, "Isometric 3D"),
}

# ─────────────────────────────────────────────────────────────────────────────
# Drawing helpers
# ─────────────────────────────────────────────────────────────────────────────

def _draw_ground(ax, size=0.55):
    xx, yy = np.meshgrid([-size, size], [-size, size])
    ax.plot_surface(xx, yy, np.zeros_like(xx), color="gray", alpha=0.15, zorder=0)
    # Ground grid lines
    for v in np.linspace(-size, size, 7):
        ax.plot([v, v], [-size, size], [0, 0], color="gray", lw=0.4, alpha=0.4)
        ax.plot([-size, size], [v, v], [0, 0], color="gray", lw=0.4, alpha=0.4)


def _draw_chassis_wireframe(ax, robot, color="black"):
    """Draw the octagonal prism chassis in World Frame."""
    l = RobotParams.CHASSIS_LENGTH
    w = RobotParams.CHASSIS_WIDTH
    h = RobotParams.CHASSIS_HEIGHT
    z0 = RobotParams.ABAD_AXIS_OFFSET
    c  = 0.04

    y_pts = np.array([w/2-c, w/2, w/2, w/2-c, -w/2+c, -w/2, -w/2, -w/2+c])
    z_pts = np.array([h/2, h/2-c, -h/2+c, -h/2, -h/2, -h/2+c, h/2-c, h/2]) + z0

    front_pts_B = np.column_stack([[l/2]*8, y_pts, z_pts])
    back_pts_B  = np.column_stack([[-l/2]*8, y_pts, z_pts])

    front_pts_W = np.array([robot.body_to_world(p) for p in front_pts_B])
    back_pts_W  = np.array([robot.body_to_world(p) for p in back_pts_B])

    for i in range(8):
        nxt = (i + 1) % 8
        # Front / back rings
        ax.plot(*zip(front_pts_W[i], front_pts_W[nxt]), color=color, lw=1.5, alpha=0.7)
        ax.plot(*zip(back_pts_W[i],  back_pts_W[nxt]),  color=color, lw=1.5, alpha=0.7)
        # Longitudinal struts
        ax.plot(*zip(front_pts_W[i], back_pts_W[i]), color=color, lw=1.0, alpha=0.5)


def _draw_legs_world(ax, robot, q_list):
    """Draw all 4 leg mechanisms in World Frame via _transform_to_body monkey-patch."""
    for i in range(4):
        leg = robot.legs[i]
        theta, beta, gamma = q_list[i]
        orig = leg._transform_to_body

        def _make_world(orig_fn):
            def wrapper(p_L, gamma_val=None, type="pos"):
                p_B = orig_fn(p_L, gamma_val, type)
                if type == "pos":
                    return robot.body_to_world(p_B)
                return robot._rot_matrix(robot.base_ori) @ p_B
            return wrapper

        leg._transform_to_body = _make_world(orig)
        leg.plot_leg_3d(theta, beta, gamma, ax)
        leg._transform_to_body = orig


def _draw_collision_markers(ax, robot, col, q_list):
    """Draw M6 studs (red), wheel bottoms (blue), contact pivots (yellow★)."""
    pts = col.get_all_collision_points(q_list)
    m6  = pts["m6_studs"]
    wh  = pts["wheels"]

    ax.scatter(m6[:, 0], m6[:, 1], m6[:, 2], c="red",  s=60, marker="o",
               label="M6 Studs", depthshade=False, zorder=5)
    ax.scatter(wh[:, 0], wh[:, 1], wh[:, 2], c="blue", s=60, marker="^",
               label="Wheel contact", depthshade=False, zorder=5)

    # Contact pivots (Z ≈ 0)
    all_pts = np.vstack([pts["chassis"], m6, wh])
    min_z   = all_pts[:, 2].min()
    tol     = 3e-3
    pivots  = all_pts[all_pts[:, 2] <= min_z + tol]
    if len(pivots):
        ax.scatter(pivots[:, 0], pivots[:, 1], pivots[:, 2],
                   c="yellow", edgecolors="black", s=200, marker="*",
                   label="Contact pivot", zorder=6)
    # CoM
    com = robot.body_to_world(np.zeros(3))
    ax.scatter(*com, c="lime", s=120, marker="D", label="CoM", zorder=7)
    ax.plot([com[0], com[0]], [com[1], com[1]], [0, com[2]],
            color="lime", lw=1, ls="--", alpha=0.6)


def build_robot_for_window(wdef):
    """Create and ground-shift robot for a given window definition."""
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(wdef["roll"]),
                                np.deg2rad(wdef["pitch"]), 0.0])
    robot.base_pos = np.array([0.0, 0.0, 0.5])
    col = CorgiCollisionModel(robot)
    pts = col.get_all_collision_points(wdef["q_list"])
    all_p = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= all_p[:, 2].min()
    return robot, col


# ─────────────────────────────────────────────────────────────────────────────
# Per-window render
# ─────────────────────────────────────────────────────────────────────────────

def render_window(wid, wdef, views, save_dir, show, timeout):
    print(f"  Rendering {wid}: {wdef['label']} ...")

    robot, col = build_robot_for_window(wdef)
    q_list = wdef["q_list"]

    n_views = len(views)
    fig = plt.figure(figsize=(5 * n_views, 5.5))
    fig.patch.set_facecolor("#1a1a2e")

    axes = []
    for idx, view in enumerate(views):
        ax = fig.add_subplot(1, n_views, idx + 1, projection="3d")
        ax.set_facecolor("#16213e")
        axes.append((ax, view))

    for ax, view in axes:
        elev, azim, vtitle = VIEW_PRESETS[view]

        _draw_ground(ax)
        _draw_chassis_wireframe(ax, robot, color="white")
        _draw_legs_world(ax, robot, q_list)
        _draw_collision_markers(ax, robot, col, q_list)

        ax.view_init(elev=elev, azim=azim)
        ax.set_xlabel("X", color="white", fontsize=7)
        ax.set_ylabel("Y", color="white", fontsize=7)
        ax.set_zlabel("Z", color="white", fontsize=7)
        ax.tick_params(colors="white", labelsize=6)
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor("#444466")
        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.5, 0.5)
        ax.set_zlim(0.0, 0.55)
        ax.set_box_aspect([1, 1, 0.55])
        ax.set_title(vtitle, color="white", fontsize=8, pad=6)

    # Shared legend from last axes
    handles, labels = axes[-1][0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="lower center", ncol=len(handles),
                   fontsize=7, facecolor="#1a1a2e", labelcolor="white",
                   framealpha=0.7, bbox_to_anchor=(0.5, 0.0))

    fig.suptitle(wdef["label"], color=wdef["color"],
                 fontsize=11, fontweight="bold", y=1.01)

    plt.tight_layout(rect=[0, 0.05, 1, 1])

    # Save
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, f"window_{wid}.png")
    plt.savefig(save_path, dpi=200, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"    Saved → {save_path}")

    if show:
        plt.show(block=False)
        plt.pause(timeout)
    plt.close("all")


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize Corgi stability windows (S0–S4) as multi-view screenshots.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--window", "-w", nargs="+", default=["S4"],
        choices=list(WINDOWS.keys()) + ["all"],
        metavar="WINDOW",
        help="Window ID(s) to render. Use 'all' for all windows. (default: S4)"
    )
    parser.add_argument(
        "--views", "-v", nargs="+",
        default=["front", "side", "top"],
        choices=list(VIEW_PRESETS.keys()),
        metavar="VIEW",
        help="Views to include: front side top 3d (default: front side top)"
    )
    parser.add_argument(
        "--save-dir", "-o", default="output/window_states",
        help="Output directory for saved PNGs (default: output/window_states)"
    )
    parser.add_argument(
        "--no-show", action="store_true",
        help="Skip interactive display, only save to file."
    )
    parser.add_argument(
        "--timeout", "-t", type=float, default=15.0,
        help="Seconds to keep plot open before auto-closing (default: 15)"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Resolve window list
    if "all" in args.window:
        win_ids = list(WINDOWS.keys())
    else:
        win_ids = args.window

    show = not args.no_show

    print(f"Windows : {win_ids}")
    print(f"Views   : {args.views}")
    print(f"Save dir: {args.save_dir}")
    print(f"Timeout : {args.timeout}s")
    print()

    for wid in win_ids:
        render_window(
            wid=wid,
            wdef=WINDOWS[wid],
            views=args.views,
            save_dir=args.save_dir,
            show=show,
            timeout=args.timeout,
        )

    print("\nDone.")


if __name__ == "__main__":
    main()
