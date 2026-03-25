"""
Stance Phase Trajectory Visual Test (Single Leg).

Visualizes the FL leg executing the stance phase in 3 views (Isometric, Side, Front).
Uses CorgiLegKinematics.plot_leg_3d() for detailed single-leg rendering.

Two contact points are plotted per frame for comparison:
  - **Estimated (Red)**: from foot_rim_contact_fk() using simplified α = δ − β
  - **Actual (Blue)**: brute-force search for the lowest Z point on the rim

Usage:
    /home/starlee/envs/legwheel/bin/python examples/test_stance_trajectory.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import os

# Add the project root to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D
from legwheel.models.corgi_leg import CorgiLegKinematics


def draw_ground_plane(ax, z_ground, center_xy=(0, 0), extent=0.3):
    """Draws a translucent ground plane at the given z height."""
    cx, cy = center_xy
    xx, yy = np.meshgrid(
        np.linspace(cx - extent, cx + extent, 2),
        np.linspace(cy - extent, cy + extent, 2)
    )
    zz = np.full_like(xx, z_ground)
    ax.plot_surface(xx, yy, zz, alpha=0.15, color='sienna', zorder=0)


def find_actual_contact(kin, theta, beta, gamma, n_samples=360):
    """
    Brute-force search for the actual ground contact point on the rim.
    Samples alpha around the full rim and returns the point with the lowest Z in {B}.

    Returns:
        p_actual (np.ndarray): [x, y, z] of the lowest rim point in Body Frame.
        alpha_actual (float): The alpha angle (deg) at the lowest point.
    """
    alphas = np.linspace(-180, 180, n_samples)
    min_z = np.inf
    p_actual = None
    alpha_actual = 0.0

    for a in alphas:
        p = kin.forward_kinematics(theta, beta, gamma, alpha=a, w=0.0)
        if p[2] < min_z:
            min_z = p[2]
            p_actual = p
            alpha_actual = a

    return p_actual, alpha_actual


def test_stance_trajectory_animation():
    print("=========================================")
    print(" Stance Phase Trajectory — Single Leg   ")
    print(" Estimated vs Actual Contact Point      ")
    print("=========================================")

    # --- Parameters ---
    LEG_INDEX = 0  # FL
    DT_ANIM = 0.02  # Animation dt (larger than planner dt for speed)

    planner = TrajectoryPlanner3D(
        leg_index=LEG_INDEX, stand_height=0.3,
        step_length=0.4, period=1.0, dt=0.005
    )
    kin = CorgiLegKinematics(LEG_INDEX)

    # --- Generate full gait trajectory (stance + swing) ---
    print("Generating trajectory via generate_trajectory() ...")
    cmd = planner.generate_trajectory(lateral_offset=0.0)
    cmd = np.array(cmd)

    n_stance = int(planner.T * (1 - planner.duty) / planner.dt)
    n_stance = min(n_stance, len(cmd))

    print(f"  Total frames: {len(cmd)}  |  Stance frames: {n_stance}")

    # Subsample for animation
    step = max(1, int(DT_ANIM / planner.dt))
    stance_indices = list(range(0, n_stance, step))
    stance_cmds = cmd[stance_indices]

    # Pre-compute BOTH contact points for every frame
    est_contacts = []   # Estimated: foot_rim_contact_fk()
    act_contacts = []   # Actual: brute-force lowest Z on rim

    print("Pre-computing contact points (estimated & actual) ...")
    for idx, q in enumerate(stance_cmds):
        # Estimated
        alpha_est, w_est = kin.foot_rim_contact_fk(*q)
        p_est = kin.forward_kinematics(*q, alpha=alpha_est, w=w_est)
        est_contacts.append(p_est)

        # Actual (brute-force)
        p_act, alpha_act = find_actual_contact(kin, *q)
        act_contacts.append(p_act)

    est_contacts = np.array(est_contacts)
    act_contacts = np.array(act_contacts)

    # Print error statistics
    errors = np.linalg.norm(est_contacts - act_contacts, axis=1)
    print(f"  Contact estimation error — mean: {errors.mean():.4f} m, max: {errors.max():.4f} m")

    z_ground = min(est_contacts[:, 2].min(), act_contacts[:, 2].min())

    # --- Figure setup: 3 views ---
    fig = plt.figure(figsize=(18, 6))
    fig.suptitle("Stance Trajectory — Estimated (Red) vs Actual (Blue) Contact", fontsize=14, fontweight='bold')

    axes = [
        fig.add_subplot(131, projection='3d'),
        fig.add_subplot(132, projection='3d'),
        fig.add_subplot(133, projection='3d'),
    ]
    titles = ['Isometric View', 'Side View (Sagittal)', 'Front View (Coronal)']
    views = [(25, -50), (0, -90), (0, 0)]

    mo = kin.p_Mi_in_B
    lim = 0.25

    def update(frame_idx):
        q_f = stance_cmds[frame_idx]

        for i, ax in enumerate(axes):
            ax.clear()

            # Ground plane
            draw_ground_plane(ax, z_ground, center_xy=(mo[0], mo[1]), extent=lim)

            # Single-leg mechanism
            kin.plot_leg_3d(q_f[0], q_f[1], q_f[2], ax)
            kin.plot_frames(ax, q_f[2])

            # --- Estimated contact (Red) ---
            pe = est_contacts[frame_idx]
            ax.scatter(pe[0], pe[1], pe[2], color='red', s=80, zorder=10,
                       marker='o', label='Estimated Contact')
            trace_e = est_contacts[:frame_idx + 1]
            ax.plot(trace_e[:, 0], trace_e[:, 1], trace_e[:, 2],
                    'r--', linewidth=1.5, alpha=0.6)

            # --- Actual contact (Blue) ---
            pa = act_contacts[frame_idx]
            ax.scatter(pa[0], pa[1], pa[2], color='dodgerblue', s=80, zorder=10,
                       marker='^', label='Actual Contact')
            trace_a = act_contacts[:frame_idx + 1]
            ax.plot(trace_a[:, 0], trace_a[:, 1], trace_a[:, 2],
                    'b--', linewidth=1.5, alpha=0.6)

            # View & limits
            ax.view_init(elev=views[i][0], azim=views[i][1])
            ax.set_xlim(mo[0] - lim, mo[0] + lim)
            ax.set_ylim(mo[1] - lim, mo[1] + lim)
            ax.set_zlim(z_ground - 0.05, mo[2] + lim)
            ax.set_title(f"{titles[i]}  |  err={errors[frame_idx]:.4f} m")
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')

            if i == 0:
                ax.legend(loc='upper right', fontsize=8)

    print(f"Rendering {len(stance_cmds)} animation frames ... (close the window to exit)")
    ani = FuncAnimation(fig, update, frames=len(stance_cmds),
                        interval=DT_ANIM * 1000, repeat=True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    test_stance_trajectory_animation()
