"""
render/plot_cone_assembly.py

Visualizes the M6-stud + wheel disc-cone assembly geometry for the Corgi robot.

Three panels:
  1. Cone geometry in {M} frame — apex (M6 tip) and base rim with dimensions
  2. Theta sweep — how the effective cone / reach changes with θ ∈ [17°, 160°]
  3. Side-fall contact — stability margin vs θ at the S1 window (Roll ≈ 85°)

Usage:
    uv run python render/plot_cone_assembly.py [--timeout SECONDS]
"""
import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.models.collision_model import CorgiCollisionModel
from legwheel.visualization.plot_leg import PlotLeg
from legwheel.config import RobotParams
from scipy.spatial import ConvexHull

# ── Params ────────────────────────────────────────────────────────────────────
D_WHEEL  = RobotParams.WHEEL_AXIAL_OFFSET     # 91.675 mm
M6_EXTRA = 0.0345                              # 34.5 mm
R_OUTER  = RobotParams.WHEEL_RADIUS_OUTER      # TIRE_TREAD_RADIUS + TIRE_CORNER_RADIUS
THETA_MIN = RobotParams.MIN_THETA_DEG          # 17°
THETA_MAX = RobotParams.MAX_THETA_DEG          # 160°

THETAS_DEG = np.array([17, 30, 45, 60, 75, 90, 110, 130, 160])


# ── Helper: rim reach in 2D sagittal plane ────────────────────────────────────

def rim_reach_2d(theta_deg):
    """Returns (x_L, y_L) of the wheel rim bottom (alpha=0) at given theta."""
    pl = PlotLeg()
    pl.forward(np.deg2rad(theta_deg), np.deg2rad(0))
    pt = pl.rim_point(0.0)
    return float(pt[0]), float(pt[1])   # x_L, y_L


# ── Panel 1: Cone geometry in {M} ─────────────────────────────────────────────

def panel_cone_geometry(ax):
    """3D cone assembly in Module Frame {M}."""
    N = 72
    alphas = np.linspace(-np.pi, np.pi, N, endpoint=False)

    apex  = np.array([D_WHEEL + M6_EXTRA, 0, 0])
    rim   = np.column_stack([
        np.full(N, D_WHEEL),
        R_OUTER * np.cos(alphas),
        R_OUTER * np.sin(alphas),
    ])

    # Rim circle
    ax.plot(rim[:, 0] * 1000, rim[:, 1] * 1000, rim[:, 2] * 1000,
            'b-', lw=2, label='Wheel rim (R=134.5mm)')
    ax.scatter(*rim[0] * 1000, color='blue', s=0)   # dummy for close

    # Cone surface lines (every 15°)
    for i in range(0, N, N // 24):
        ax.plot([apex[0] * 1000, rim[i, 0] * 1000],
                [apex[1] * 1000, rim[i, 1] * 1000],
                [apex[2] * 1000, rim[i, 2] * 1000],
                color='orange', lw=0.8, alpha=0.6)

    # Apex
    ax.scatter(*apex * 1000, c='red', s=120, zorder=6, label='M6 stud tip (apex)')

    # Wheel plane
    theta_circ = np.linspace(0, 2 * np.pi, 60)
    xc = np.full(60, D_WHEEL * 1000)
    yc = R_OUTER * 1000 * np.cos(theta_circ)
    zc = R_OUTER * 1000 * np.sin(theta_circ)
    ax.plot(xc, yc, zc, 'b--', lw=1, alpha=0.3)

    # Annotations
    ax.text(apex[0] * 1000 + 5, 5, 5,
            f'Apex\nX_M={apex[0]*1000:.1f}mm', fontsize=7, color='red')
    ax.text(D_WHEEL * 1000 - 15, R_OUTER * 1000 + 5, 0,
            f'Rim\nX_M={D_WHEEL*1000:.1f}mm\nR={R_OUTER*1000:.1f}mm',
            fontsize=7, color='blue')

    # Cone half-angle annotation
    half_angle = np.degrees(np.arctan2(R_OUTER, M6_EXTRA))
    ax.text(apex[0] * 1000 - 10, 0, -R_OUTER * 1000 * 0.5,
            f'θ_c={half_angle:.1f}°', fontsize=9, color='orange', fontweight='bold')

    ax.set_xlabel('X_M (mm)')
    ax.set_ylabel('Y_M (mm)')
    ax.set_zlabel('Z_M (mm)')
    ax.set_title(f'Panel 1: Disc-Cone Geometry in {{M}}\n'
                 f'Δx={M6_EXTRA*1000:.1f}mm  R={R_OUTER*1000:.1f}mm  θ_c={half_angle:.1f}°')
    ax.legend(fontsize=8)
    ax.set_box_aspect([1, 2, 2])


# ── Panel 2: Theta sweep — sagittal reach ─────────────────────────────────────

def panel_theta_sweep(ax):
    """
    Shows the wheel-rim bottom locus (x_L, radius from origin) as θ sweeps,
    plotted in the sagittal plane (Body Frame XZ projection for FL leg).
    """
    thetas = np.linspace(THETA_MIN, THETA_MAX, 120)
    x_vals, r_vals = [], []
    for t in thetas:
        x_L, y_L = rim_reach_2d(t)
        x_vals.append(x_L)
        r_vals.append(np.hypot(x_L, y_L))

    # Radial reach curve
    ax.plot(np.degrees(np.deg2rad(thetas)), np.array(r_vals) * 1000,
            'b-', lw=2, label='Rim reach from linkage origin')

    # Annotate key theta values
    for t_deg in THETAS_DEG:
        x_L, y_L = rim_reach_2d(t_deg)
        r = np.hypot(x_L, y_L) * 1000
        ax.scatter(t_deg, r, s=60, zorder=5,
                   color=plt.cm.plasma((t_deg - 17) / 143))
        ax.annotate(f'{r:.0f}mm', (t_deg, r),
                    textcoords='offset points', xytext=(4, 4), fontsize=7)

    # Highlight min/max
    ax.axvline(THETA_MIN, color='gray', lw=1, ls=':', label=f'θ_min={THETA_MIN}° (wheel)')
    ax.axvline(THETA_MAX, color='gray', lw=1, ls='--', label=f'θ_max={THETA_MAX}°')
    ax.axvline(75, color='orange', lw=1.5, ls='-.', label='θ=75° (peak S1 margin)')

    # M6 stud distance from ABAD axis (fixed)
    m6_dist = (D_WHEEL + M6_EXTRA) * 1000
    ax.axhline(m6_dist, color='red', lw=1, ls='--',
               label=f'M6 lateral reach = {m6_dist:.0f}mm')

    ax.set_xlabel('θ (deg)')
    ax.set_ylabel('Rim reach from linkage origin (mm)')
    ax.set_title('Panel 2: Rim Reach vs θ  (β=0°, sagittal)\n'
                 'Reach grows monotonically; M6 stud is fixed')
    ax.legend(fontsize=7, loc='lower right')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(15, 165)


# ── Panel 3: Stability margin vs theta at S1 window ───────────────────────────

def stability_at_roll_theta(roll_deg, theta_deg, beta_deg=0, gamma_deg=0, tol=3e-3):
    """Compute normalised stability margin for given roll + joint angles."""
    q = [np.deg2rad(theta_deg), np.deg2rad(beta_deg), np.deg2rad(gamma_deg)]
    robot = CorgiRobot()
    robot.base_ori = np.array([np.deg2rad(roll_deg), 0, 0])
    robot.base_pos = np.array([0, 0, 0.5])
    col = CorgiCollisionModel(robot)
    pts = col.get_all_collision_points([q] * 4)
    c   = np.vstack([pts["chassis"], pts["m6_studs"], pts["wheels"]])
    robot.base_pos[2] -= c[:, 2].min()

    pts2 = col.get_all_collision_points([q] * 4)
    c2   = np.vstack([pts2["chassis"], pts2["m6_studs"], pts2["wheels"]])
    mask = c2[:, 2] <= c2[:, 2].min() + tol
    xy   = c2[mask, :2]

    if len(xy) < 3:
        return -1.0
    try:
        hull = ConvexHull(xy)
        com  = robot.body_to_world(np.zeros(3))[:2]
        vals = hull.equations @ np.append(com, 1.0)
        d    = -vals.max()
        hp   = xy[hull.vertices]; n = len(hp)
        perim = sum(np.linalg.norm(hp[(i+1)%n] - hp[i]) for i in range(n))
        return d / (2 * hull.volume / perim)
    except Exception:
        return -1.0


def panel_theta_stability(ax):
    """S1 stability margin as function of θ, for roll = 84, 85, 86°."""
    thetas = np.linspace(THETA_MIN, THETA_MAX, 60)

    for roll_deg, color, ls in [(84, 'green', '-'), (85, 'orange', '--'), (86, 'red', ':')]:
        margins = [stability_at_roll_theta(roll_deg, t) for t in thetas]
        ax.plot(thetas, margins, color=color, lw=2, ls=ls,
                label=f'Roll={roll_deg}°')

    ax.axhline(0, color='black', lw=1, ls='--')
    ax.axvline(75, color='orange', lw=1.5, ls='-.', alpha=0.7, label='θ=75° (peak)')
    ax.fill_between(thetas,
                    [stability_at_roll_theta(85, t) for t in thetas],
                    0,
                    where=[stability_at_roll_theta(85, t) > 0 for t in thetas],
                    alpha=0.15, color='orange')

    ax.set_xlabel('θ (deg)')
    ax.set_ylabel('Normalised CoM margin')
    ax.set_title('Panel 3: S1 Stability Margin vs θ\n(β=0°, γ=0°, Roll = 84/85/86°)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(15, 165)
    ax.set_ylim(-1.0, 1.0)


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--timeout', '-t', type=float, default=30,
                   help='Auto-close timeout in seconds (default 30)')
    p.add_argument('--no-show', action='store_true',
                   help='Save only, do not open window')
    p.add_argument('--save-dir', default='output',
                   help='Directory to save output PNG')
    return p.parse_args()


def main():
    args = parse_args()

    print("Building cone geometry panel 1 ...")
    fig = plt.figure(figsize=(18, 6))
    ax1 = fig.add_subplot(131, projection='3d')
    ax2 = fig.add_subplot(132)
    ax3 = fig.add_subplot(133)

    panel_cone_geometry(ax1)

    print("Building theta sweep panel 2 ...")
    panel_theta_sweep(ax2)

    print("Building stability panel 3 ...")
    panel_theta_stability(ax3)

    fig.suptitle('Corgi — M6 Stud + Wheel Disc-Cone Assembly Analysis\n'
                 '(θ_c = 75.6°,  S1 peak stability at θ ≈ 75°)',
                 fontsize=12, y=1.01)
    plt.tight_layout()

    os.makedirs(args.save_dir, exist_ok=True)
    save_path = os.path.join(args.save_dir, 'cone_assembly.png')
    plt.savefig(save_path, dpi=200, bbox_inches='tight')
    print(f"Saved → {save_path}")

    if not args.no_show:
        plt.show(block=False)
        plt.pause(args.timeout)
    plt.close('all')


if __name__ == '__main__':
    main()
