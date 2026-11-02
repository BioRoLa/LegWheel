"""
Render all self-righting boundary states in one pass.

Default behavior:
- Render ordered sequence: S4 -> S3 -> S2 -> S1 -> S0
- Save indexed files so lexicographic sort follows state order

Examples:
    uv run python examples/self_righting/boundary_states/render_boundary_sequence.py
    uv run python examples/self_righting/boundary_states/render_boundary_sequence.py --no-show
    uv run python examples/self_righting/boundary_states/render_boundary_sequence.py --views front top 3d
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull

# Ensure local package import works when script is run directly.
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from legwheel.config import RobotParams
from legwheel.models.collision_model import CorgiCollisionModel
from legwheel.models.corgi_robot import CorgiRobot


def _q(theta_deg, beta_deg, gamma_deg):
    return [np.deg2rad(theta_deg), np.deg2rad(beta_deg), np.deg2rad(gamma_deg)]


_q_nom = _q(60, 90, 0)
_q_up = _q(60, 90, 0)
_q_g30 = _q(60, 90, 30)

WINDOWS = {
    "S0": {
        "roll": 0,
        "pitch": 0,
        "q_list": [_q_nom] * 4,
        "label": "S0 - Upright (gamma=0, Roll=0)",
        "color": "#2196F3",
    },
    "S1": {
        "roll": 85,
        "pitch": 0,
        "q_list": [_q_nom] * 4,
        "label": "S1 - Side-Fall Leading (gamma=0, Roll=85)",
        "color": "#FF9800",
    },
    "S2": {
        "roll": 115,
        "pitch": 0,
        "q_list": [_q_up, _q_g30, _q_g30, _q_up],
        "label": "S2 - Side-Fall Trailing alpha (gamma_lower=30, Roll=115)",
        "color": "#9C27B0",
    },
    "S3": {
        "roll": 126,
        "pitch": 0,
        "q_list": [_q_up, _q_g30, _q_g30, _q_up],
        "label": "S3 - Side-Fall Trailing beta (gamma_lower=30, Roll=126)",
        "color": "#E91E63",
    },
    "S4": {
        "roll": 180,
        "pitch": 0,
        "q_list": [_q_nom] * 4,
        "label": "S4 - Upside-Down (gamma=0, Roll=180)",
        "color": "#4CAF50",
    },
}

# Self-righting sequence: upside-down to upright.
DEFAULT_SEQUENCE = ["S4", "S3", "S2", "S1", "S0"]

VIEW_PRESETS = {
    "front": (0, -90, "Front View"),
    "side": (0, 180, "Side View"),
    "top": (90, 0, "Top View"),
    "3d": (22, -55, "Isometric 3D"),
}


def _draw_ground(ax, size=0.55):
    xx, yy = np.meshgrid([-size, size], [-size, size])
    ax.plot_surface(xx, yy, np.zeros_like(xx), color="gray", alpha=0.15, zorder=0)
    for v in np.linspace(-size, size, 7):
        ax.plot([v, v], [-size, size], [0, 0], color="gray", lw=0.4, alpha=0.4)
        ax.plot([-size, size], [v, v], [0, 0], color="gray", lw=0.4, alpha=0.4)


def _draw_body_frame(ax, robot, scale=0.15):
    """Draw {B} frame coordinate axes (X=red, Y=green, Z=blue) at the body origin."""
    origin = robot.base_pos
    R = robot._rot_matrix(robot.base_ori)
    colors = ["#ff4444", "#44ff44", "#4488ff"]
    labels = ["{B} X", "{B} Y", "{B} Z"]
    for i in range(3):
        d = R[:, i] * scale
        ax.quiver(
            origin[0], origin[1], origin[2],
            d[0], d[1], d[2],
            color=colors[i], linewidth=2.2, arrow_length_ratio=0.25,
            label=labels[i],
        )


def _draw_chassis_wireframe(ax, robot, color="black"):
    l = RobotParams.CHASSIS_LENGTH
    w = RobotParams.CHASSIS_WIDTH
    h = RobotParams.CHASSIS_HEIGHT
    z0 = RobotParams.ABAD_AXIS_OFFSET
    c = 0.04

    y_pts = np.array([w / 2 - c, w / 2, w / 2, w / 2 - c, -w / 2 + c, -w / 2, -w / 2, -w / 2 + c])
    z_pts = np.array([h / 2, h / 2 - c, -h / 2 + c, -h / 2, -h / 2, -h / 2 + c, h / 2 - c, h / 2]) + z0

    front_pts_b = np.column_stack([[l / 2] * 8, y_pts, z_pts])
    back_pts_b = np.column_stack([[-l / 2] * 8, y_pts, z_pts])

    front_pts_w = np.array([robot.body_to_world(p) for p in front_pts_b])
    back_pts_w = np.array([robot.body_to_world(p) for p in back_pts_b])

    for i in range(8):
        nxt = (i + 1) % 8
        ax.plot(*zip(front_pts_w[i], front_pts_w[nxt]), color=color, lw=1.5, alpha=0.7)
        ax.plot(*zip(back_pts_w[i], back_pts_w[nxt]), color=color, lw=1.5, alpha=0.7)
        ax.plot(*zip(front_pts_w[i], back_pts_w[i]), color=color, lw=1.0, alpha=0.5)


def _draw_legs_world(ax, robot, q_list):
    for i in range(4):
        leg = robot.legs[i]
        theta, beta, gamma = q_list[i]
        orig = leg._transform_to_body

        def _make_world(orig_fn):
            def wrapper(p_l, gamma_val=None, type="pos"):
                p_b = orig_fn(p_l, gamma_val, type)
                if type == "pos":
                    return robot.body_to_world(p_b)
                return robot._rot_matrix(robot.base_ori) @ p_b

            return wrapper

        leg._transform_to_body = _make_world(orig)
        leg.plot_leg_3d(theta, beta, gamma, ax)
        leg._transform_to_body = orig


def _draw_collision_markers(ax, robot, col, q_list):
    pts = col.get_all_collision_points(q_list)
    m6 = pts["m6_studs"]
    wh = pts["wheels"]

    ax.scatter(m6[:, 0], m6[:, 1], m6[:, 2], c="red", s=60, marker="o", label="M6 studs", depthshade=False, zorder=5)
    ax.scatter(wh[:, 0], wh[:, 1], wh[:, 2], c="blue", s=60, marker="^", label="Wheel contact", depthshade=False, zorder=5)

    all_pts = np.vstack([pts["chassis"], m6, wh])
    min_z = all_pts[:, 2].min()
    pivots = all_pts[all_pts[:, 2] <= min_z + 3e-3]

    if len(pivots):
        ax.scatter(
            pivots[:, 0],
            pivots[:, 1],
            pivots[:, 2],
            c="yellow",
            edgecolors="black",
            s=200,
            marker="*",
            label="Contact pivot",
            zorder=6,
        )

    z_poly = min_z + 2e-3
    if len(pivots) >= 3:
        try:
            hull = ConvexHull(pivots[:, :2])
            verts = pivots[hull.vertices, :2]
            verts_closed = np.vstack([verts, verts[0]])
            ax.plot(
                verts_closed[:, 0],
                verts_closed[:, 1],
                np.full(len(verts_closed), z_poly),
                color="limegreen",
                lw=2.5,
                label="Support polygon",
                zorder=7,
            )
            poly_verts = [np.column_stack([verts[:, 0], verts[:, 1], np.full(len(verts), z_poly)])]
            poly = Poly3DCollection(poly_verts, alpha=0.18, facecolor="limegreen", edgecolor="none")
            ax.add_collection3d(poly)
        except Exception:
            pass
    elif len(pivots) == 2:
        ax.plot(
            pivots[:, 0],
            pivots[:, 1],
            [z_poly, z_poly],
            color="limegreen",
            lw=2.5,
            label="Support line",
            zorder=7,
        )

    com = robot.body_to_world(np.zeros(3))
    ax.scatter(*com, c="lime", s=120, marker="D", label="CoM", zorder=8)
    ax.plot([com[0], com[0]], [com[1], com[1]], [min_z, com[2]], color="lime", lw=1, ls="--", alpha=0.6)
    ax.scatter(com[0], com[1], min_z + 4e-3, c="gold", edgecolors="black", s=130, marker="*", label="CoM projection", zorder=9)


def build_robot_for_window(wdef):
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(wdef["roll"]), np.deg2rad(wdef["pitch"]), 0.0])
    robot.base_pos = np.array([0.0, 0.0, 0.5])
    col = CorgiCollisionModel(robot)

    pts = col.get_all_collision_points(wdef["q_list"])
    all_p = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= all_p[:, 2].min()
    return robot, col


def render_window(wid, wdef, views, save_dir, show, timeout, index):
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
        _draw_body_frame(ax, robot)

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

    handles, labels = axes[-1][0].get_legend_handles_labels()
    if handles:
        fig.legend(
            handles,
            labels,
            loc="lower center",
            ncol=len(handles),
            fontsize=7,
            facecolor="#1a1a2e",
            labelcolor="white",
            framealpha=0.7,
            bbox_to_anchor=(0.5, 0.0),
        )

    fig.suptitle(wdef["label"], color=wdef["color"], fontsize=11, fontweight="bold", y=1.01)
    plt.tight_layout(rect=[0, 0.05, 1, 1])

    os.makedirs(save_dir, exist_ok=True)
    filename = f"{index:02d}_{wid}_boundary.png"
    save_path = os.path.join(save_dir, filename)
    plt.savefig(save_path, dpi=200, bbox_inches="tight", facecolor=fig.get_facecolor())

    if show:
        plt.show(block=False)
        plt.pause(timeout)
    plt.close("all")

    return save_path


def parse_args():
    parser = argparse.ArgumentParser(description="Render ordered self-righting boundary states with indexed outputs.")
    parser.add_argument(
        "--window",
        "-w",
        nargs="+",
        default=["all"],
        choices=list(WINDOWS.keys()) + ["all"],
        help="Window IDs to render. Use 'all' for the default ordered sequence.",
    )
    parser.add_argument(
        "--views",
        "-v",
        nargs="+",
        default=["front", "side", "top"],
        choices=list(VIEW_PRESETS.keys()),
        help="Views to render for each state.",
    )
    parser.add_argument(
        "--save-dir",
        "-o",
        default="output/window_states/ordered",
        help="Directory for PNG output.",
    )
    parser.add_argument("--no-show", action="store_true", help="Skip interactive display.")
    parser.add_argument("--timeout", "-t", type=float, default=10.0, help="Interactive display time in seconds.")
    return parser.parse_args()


def resolve_window_order(arg_windows):
    if "all" in arg_windows:
        return DEFAULT_SEQUENCE

    order_map = {wid: idx for idx, wid in enumerate(DEFAULT_SEQUENCE)}
    unique = []
    for wid in arg_windows:
        if wid not in unique:
            unique.append(wid)
    return sorted(unique, key=lambda wid: order_map.get(wid, 999))


def main():
    args = parse_args()
    window_ids = resolve_window_order(args.window)

    print("Boundary-state render run")
    print(f"  windows : {window_ids}")
    print(f"  views   : {args.views}")
    print(f"  out_dir : {args.save_dir}")

    rendered = []
    for idx, wid in enumerate(window_ids, start=1):
        out = render_window(
            wid=wid,
            wdef=WINDOWS[wid],
            views=args.views,
            save_dir=args.save_dir,
            show=not args.no_show,
            timeout=args.timeout,
            index=idx,
        )
        rendered.append(out)
        print(f"  saved   : {out}")

    print("Render manifest:")
    for i, path in enumerate(rendered, start=1):
        print(f"  {i:02d}. {path}")


if __name__ == "__main__":
    main()
