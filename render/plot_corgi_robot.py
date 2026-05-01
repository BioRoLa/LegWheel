import argparse
import importlib
import os
import sys
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.collision_model import CorgiCollisionModel
from legwheel.config import RobotParams

DISPLAY_SECONDS = 30


# ─────────────────────────────────────────────────────────────────────────────
# Chassis octagon wireframe
# ─────────────────────────────────────────────────────────────────────────────

def draw_chassis(ax, linewidth=2, color='k', alpha=0.8):
    """Draws the octagonal-prism chassis in Body Frame {B}."""
    l = RobotParams.CHASSIS_LENGTH
    w = RobotParams.CHASSIS_WIDTH
    h = RobotParams.CHASSIS_HEIGHT
    z0 = RobotParams.ABAD_AXIS_OFFSET
    c = 0.04  # chamfer

    y_pts = np.array([w/2-c, w/2, w/2, w/2-c, -w/2+c, -w/2, -w/2, -w/2+c])
    z_pts = np.array([h/2, h/2-c, -h/2+c, -h/2, -h/2, -h/2+c, h/2-c, h/2]) + z0

    ax.plot(np.append(np.full(8, l/2),  l/2),
            np.append(y_pts, y_pts[0]),
            np.append(z_pts, z_pts[0]),
            color=color, linewidth=linewidth, alpha=alpha)
    ax.plot(np.append(np.full(8, -l/2), -l/2),
            np.append(y_pts, y_pts[0]),
            np.append(z_pts, z_pts[0]),
            color=color, linewidth=linewidth, alpha=alpha)
    for i in range(8):
        ax.plot([l/2, -l/2], [y_pts[i], y_pts[i]], [z_pts[i], z_pts[i]],
                color=color, linewidth=linewidth * 0.6, alpha=alpha * 0.7)


# ─────────────────────────────────────────────────────────────────────────────
# Collision bounds overlay (M6 studs, chassis corners, wheel contacts)
# ─────────────────────────────────────────────────────────────────────────────

def draw_collision_bounds(ax, theta, beta, gamma_all=0.0, gamma_list=None,
                          show_chassis_pts=True, show_m6=True,
                          show_wheels=True, show_com=True,
                          show_contact_pivots=True,
                          show_support_polygon=False,
                          contact_tol=3e-3):
    """
    Draws the 24-point bounding-volume markers in Body Frame {B}.

    Args:
        ax: matplotlib 3D axes.
        theta, beta: joint angles (rad) — applied to all legs if gamma_list is None.
        gamma_all: ABAD angle (rad) applied to all legs (used when gamma_list is None).
        gamma_list: list of 4 individual gamma values [FL, FR, RR, RL]. Overrides gamma_all.
        show_chassis_pts: draw chassis octagon corner dots.
        show_m6: draw M6 stud tips (red).
        show_wheels: draw wheel contact-search lowest points (blue).
        show_com: draw CoM position (green diamond).
        show_contact_pivots: highlight ground-contact pivots (yellow star).
        show_support_polygon: draw the support polygon/line computed from contact pivots.
        contact_tol: z tolerance (m) for grouping points as ground contacts.
    """
    if gamma_list is None:
        gamma_list = [gamma_all] * 4

    q_list = [[theta, beta, g] for g in gamma_list]

    robot = CorgiRobot()   # default pose = upright, {B} = {W}
    col = CorgiCollisionModel(robot)
    pts = col.get_all_collision_points(q_list)

    if show_chassis_pts:
        cp = pts["chassis"]
        ax.scatter(cp[:, 0], cp[:, 1], cp[:, 2],
                   c='gray', s=18, marker='s', alpha=0.5, label='Chassis corners')

    if show_m6:
        mp = pts["m6_studs"]
        ax.scatter(mp[:, 0], mp[:, 1], mp[:, 2],
                   c='red', s=60, marker='o', zorder=5, label='M6 studs')

    if show_wheels:
        wp = pts["wheels"]
        ax.scatter(wp[:, 0], wp[:, 1], wp[:, 2],
                   c='dodgerblue', s=60, marker='^', zorder=5, label='Wheel contacts')

    all_pts = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    min_z = all_pts[:, 2].min()
    pivots = all_pts[all_pts[:, 2] <= min_z + contact_tol]

    if show_com:
        com = np.zeros(3)  # {B} origin = CoM in default pose
        ax.scatter(*com, c='limegreen', s=120, marker='D', zorder=6, label='CoM {B}')

    if show_contact_pivots and len(pivots):
        ax.scatter(pivots[:, 0], pivots[:, 1], pivots[:, 2],
                   c='yellow', edgecolors='black', s=200, marker='*',
                   zorder=7, label='Contact pivots')

    if show_support_polygon and len(pivots) >= 2:
        xy = pivots[:, :2]
        z = np.full(len(xy), min_z + 1e-3)  # keep overlay slightly above ground

        if len(xy) >= 3:
            try:
                hull = ConvexHull(xy)
                verts = xy[hull.vertices]
                verts_closed = np.vstack([verts, verts[0]])
                ax.plot(verts_closed[:, 0], verts_closed[:, 1],
                        np.full(len(verts_closed), min_z + 1e-3),
                        color='limegreen', linewidth=2.0,
                        label='Support polygon')
                ax.plot_trisurf(verts[:, 0], verts[:, 1],
                                np.full(len(verts), min_z + 8e-4),
                                color='limegreen', alpha=0.16, linewidth=0)
            except Exception:
                ax.plot(xy[:, 0], xy[:, 1], z,
                        color='limegreen', linewidth=2.0, label='Support hull (fallback)')
        else:  # len == 2
            ax.plot(xy[:, 0], xy[:, 1], z,
                    color='limegreen', linewidth=2.5, label='Support line')

        # COM projection to support plane for quick visual check
        com_proj = np.array([0.0, 0.0, min_z + 2e-3])
        ax.scatter(*com_proj, c='gold', edgecolors='black',
                   s=120, marker='*', zorder=8, label='CoM projection')
        ax.plot([0.0, 0.0], [0.0, 0.0], [0.0, min_z + 2e-3],
                color='gold', linestyle='--', linewidth=1.0)


# ─────────────────────────────────────────────────────────────────────────────
# Main draw function
# ─────────────────────────────────────────────────────────────────────────────

def draw_corgi_robot(ax, theta=np.deg2rad(90), beta=0.0, gamma=0.0,
                     show_axes=True, show_bounds=False, show_cones=True,
                     gamma_list=None, show_support_polygon=False):
    """
    Draws the Corgi robot (chassis + legs + optional collision bounds) in Body Frame {B}.

    Args:
        ax: matplotlib 3D axes.
        theta, beta, gamma: joint angles (rad), applied uniformly to all 4 legs.
        show_axes: show axis labels and grid.
        show_bounds: overlay collision markers.
        show_cones: overlay thick cone geometry when bounds are enabled.
        gamma_list: per-leg ABAD override [FL, FR, RR, RL].
        show_support_polygon: draw contact support polygon/line when bounds are on.
    """
    # 1. Chassis
    draw_chassis(ax)

    if show_bounds and show_cones:
        # Local import avoids circular import with plot_leg_envelope -> draw_chassis.
        try:
            from render.plot_leg_envelope import draw_leg_envelope
        except ModuleNotFoundError:
            sys.path.insert(0, os.path.dirname(__file__))
            draw_leg_envelope = importlib.import_module("plot_leg_envelope").draw_leg_envelope

        for i in range(4):
            draw_leg_envelope(
                ax,
                i,
                theta,
                beta,
                gamma_max_deg=RobotParams.GAMMA_MAX_DEG,
                n_gamma_slices=4,
                N_alpha=40,
                N_t=8,
            )

    # 2. Leg mechanisms
    g_list = gamma_list if gamma_list is not None else [gamma] * 4
    for i in range(4):
        kin = CorgiLegKinematics(i)
        kin.plot_leg_3d(theta, beta, g_list[i], ax)
        kin.plot_frames(ax, g_list[i])

    # 3. Optional collision bounds
    if show_bounds:
        draw_collision_bounds(
            ax,
            theta,
            beta,
            gamma_all=gamma,
            gamma_list=gamma_list,
            show_support_polygon=show_support_polygon,
        )

    # 4. Style
    if show_axes:
        ax.set_xlabel('X (Front)')
        ax.set_ylabel('Y (Left)')
        ax.set_zlabel('Z (Up)')
        ax.grid(True)
    else:
        ax.set_axis_off()
        ax.grid(False)

    max_range = 0.4
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-0.4, 0.2)
    ax.set_box_aspect([1, 1, 0.75])


def plot_corgi_robot(theta=np.deg2rad(90), beta=0.0, gamma=0.0,
                     show_bounds=False, show_cones=True, gamma_list=None,
                     show_support_polygon=False,
                     save_dir=None, show=True):
    """Static 3D plot of the Corgi robot in Body Frame {B}."""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    draw_corgi_robot(
        ax,
        theta,
        beta,
        gamma,
        show_bounds=show_bounds,
        show_cones=show_cones,
        gamma_list=gamma_list,
        show_support_polygon=show_support_polygon,
    )

    title = (f'Corgi Robot 3D  (Body Frame {{B}})\n'
             f'θ={np.rad2deg(theta):.1f}°  β={np.rad2deg(beta):.1f}°  '
             f'γ={np.rad2deg(gamma):.1f}°')
    if show_bounds and show_cones:
        title += '  [bounds + thick cones ON]'
    elif show_bounds:
        title += '  [bounds markers only]'
    if show_support_polygon:
        title += '  [support polygon ON]'
    ax.set_title(title)

    if show_bounds:
        ax.legend(fontsize=8, loc='upper right')

    plt.tight_layout()

    if save_dir is not None:
        os.makedirs(save_dir, exist_ok=True)
        tag = f"theta{int(np.rad2deg(theta)):03d}"
        if show_bounds and not show_cones:
            tag += "_bounds_markers"
        elif show_bounds:
            tag += "_bounds_cones"
        fname = os.path.join(save_dir, f"corgi_robot_{tag}.png")
        plt.savefig(fname, dpi=180, bbox_inches='tight', facecolor='white')
        print(f"Saved → {fname}")

    if show:
        plt.show(block=False)
        plt.pause(DISPLAY_SECONDS)
    plt.close('all')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Render Corgi robot and collision bounds markers.")
    parser.add_argument("--theta", type=float, default=75.0, help="Theta in degrees")
    parser.add_argument("--beta", type=float, default=0.0, help="Beta in degrees")
    parser.add_argument("--gamma", type=float, default=0.0, help="Gamma in degrees")
    parser.add_argument("--bounds", action="store_true", help="Show collision bounds markers")
    parser.add_argument("--no-cones", action="store_true", help="Disable thick cone geometry overlay")
    parser.add_argument("--support-polygon", action="store_true", help="Show support polygon")
    parser.add_argument("--save-dir", default="output", help="Directory to save output PNG")
    parser.add_argument("--no-show", action="store_true", help="Do not open display window")
    args = parser.parse_args()

    plot_corgi_robot(
        theta=np.deg2rad(args.theta),
        beta=np.deg2rad(args.beta),
        gamma=np.deg2rad(args.gamma),
        show_bounds=args.bounds,
        show_cones=not args.no_cones,
        show_support_polygon=args.support_polygon,
        save_dir=args.save_dir,
        show=not args.no_show,
    )
