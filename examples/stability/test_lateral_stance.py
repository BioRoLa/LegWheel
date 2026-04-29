"""
Lateral Stance Phase Trajectory Visual Test (Single Leg).

Visualizes the FL leg executing the stance phase with lateral (vy) velocity
in 3 views (Isometric, Side, Front).

Key difference from test_stance_trajectory.py:
  - The robot moves laterally (vy != 0) in addition to or instead of forward (vx).
  - Contact point is the lowest Z point on the w=0 plane only (soft tire, no lateral rolling).

Two contact points are plotted per frame for comparison:
  - **Estimated (Red)**: from foot_rim_contact_fk() using simplified α = δ − β
  - **Actual (Blue)**: brute-force search for the lowest Z point on the rim at w=0

Usage:
    /home/starlee/envs/legwheel/bin/python examples/test_lateral_stance.py
"""

from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import os

# Add the project root to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


def draw_ground_plane(ax, z_ground, center_xy=(0, 0), extent=0.3):
    """Draws a translucent ground plane at the given z height."""
    cx, cy = center_xy
    xx, yy = np.meshgrid(
        np.linspace(cx - extent, cx + extent, 2),
        np.linspace(cy - extent, cy + extent, 2)
    )
    zz = np.full_like(xx, z_ground)
    ax.plot_surface(xx, yy, zz, alpha=0.15, color='sienna', zorder=0)


def find_actual_contact_w0(kin, theta, beta, gamma, n_samples=360):
    """
    Brute-force search for the actual ground contact point on the rim at w=0 plane.
    Samples alpha around the full rim and returns the point with the lowest Z in {B},
    with w fixed at 0 (soft tire assumption: no lateral rolling).

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


def test_lateral_stance_animation():
    print("=========================================")
    print(" Lateral Stance Trajectory — Single Leg  ")
    print(" Estimated vs Actual Contact (w=0 plane) ")
    print("=========================================")

    # --- Parameters ---
    LEG_INDEX = 0  # FL
    DT_ANIM = 0.02  # Animation dt (larger than planner dt for speed)

    # Lateral velocity: vy = 0.05 m/s, with optional forward vx
    VX = 0.05
    VY = 0.05
    STAND_HEIGHT = 0.31

    planner = TrajectoryPlanner3D(
        leg_index=LEG_INDEX, stand_height=STAND_HEIGHT,
        velocity=[VX, VY, 0.0], period=1.0, dt=0.005,
        stance_duty=0.75
    )
    kin = CorgiLegKinematics(LEG_INDEX)

    print(f"  v_hip = [{VX}, {VY}, 0.0] m/s")
    print(f"  beta0 = {np.rad2deg(planner.beta0):.2f} deg")
    print(f"  gamma0 = {np.rad2deg(planner.gamma0):.2f} deg")
    print(f"  theta0 = {np.rad2deg(planner.theta0):.2f} deg")

    # --- Generate full gait trajectory (stance + swing) ---
    print("Generating trajectory via generate_trajectory() ...")
    cmd = planner.generate_trajectory(lateral_offset=0.0)
    cmd = np.array(cmd)

    n_stance = int(planner.T * planner.stance_duty / planner.dt)
    n_stance = min(n_stance, len(cmd))

    print(f"  Total frames: {len(cmd)}  |  Stance frames: {n_stance}")

    # Subsample for animation
    step = max(1, int(DT_ANIM / planner.dt))
    stance_indices = list(range(0, n_stance, step))
    stance_cmds = cmd[stance_indices]

    # Pre-compute BOTH contact points for every frame
    est_contacts = []   # Estimated: foot_rim_contact_fk()
    act_contacts = []   # Actual: brute-force lowest Z on rim at w=0

    print("Pre-computing contact points (estimated & actual at w=0) ...")
    for idx, q in enumerate(stance_cmds):
        # Estimated (w=0 for soft tire)
        alpha_est, _ = kin.foot_rim_contact_fk(*q)
        p_est = kin.forward_kinematics(*q, alpha=alpha_est, w=0.0)
        est_contacts.append(p_est)

        # Actual (brute-force at w=0)
        p_act, alpha_act = find_actual_contact_w0(kin, *q)
        act_contacts.append(p_act)

    est_contacts = np.array(est_contacts)
    act_contacts = np.array(act_contacts)

    # Print error statistics
    errors = np.linalg.norm(est_contacts - act_contacts, axis=1)
    print(
        f"  Contact estimation error — mean: {errors.mean():.4f} m, max: {errors.max():.4f} m")

    # Print gamma symmetry check
    gamma_init = np.rad2deg(stance_cmds[0, 2])
    gamma_final = np.rad2deg(stance_cmds[-1, 2])
    beta_init = np.rad2deg(stance_cmds[0, 1])
    beta_final = np.rad2deg(stance_cmds[-1, 1])
    print(f"  β sweep: {beta_init:.2f}° → {beta_final:.2f}°")
    print(f"  γ sweep: {gamma_init:.2f}° → {gamma_final:.2f}°")

    z_ground = min(est_contacts[:, 2].min(), act_contacts[:, 2].min())

    # --- Figure setup: 3 views ---
    fig = plt.figure(figsize=(18, 6))
    fig.suptitle("Lateral Stance Trajectory — Estimated (Red) vs Actual (Blue) Contact (w=0)",
                 fontsize=14, fontweight='bold')

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
            draw_ground_plane(ax, z_ground, center_xy=(
                mo[0], mo[1]), extent=lim)

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
            ax.set_box_aspect([1, 1, 1])  # Equal scaling
            ax.view_init(elev=views[i][0], azim=views[i][1])
            ax.set_xlim(mo[0] - lim, mo[0] + lim)
            ax.set_ylim(mo[1] - lim, mo[1] + lim)
            ax.set_zlim(z_ground - 0.05, mo[2] + lim)
            ax.set_title(f"{titles[i]}  |  err={errors[frame_idx]:.4f} m")
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            # --- Target Velocity Vector at Hip (Black arrow) ---
            v_vec = planner.velocity
            ax.quiver(mo[0], mo[1], mo[2], v_vec[0], v_vec[1], v_vec[2],
                      color='black', length=0.5, normalize=False,
                      label='Target Velocity' if frame_idx == 0 else "")

            # --- Real-time q display (bottom left) ---
            q_deg = np.rad2deg(q_f)
            q_text = (f"q (deg): θ={q_deg[0]:.1f}, β={q_deg[1]:.1f}, γ={q_deg[2]:.1f}\n"
                      f"q (rad): [{q_f[0]:.3f}, {q_f[1]:.3f}, {q_f[2]:.3f}]")
            ax.text2D(0.05, 0.05, q_text, transform=ax.transAxes,
                      fontsize=9, bbox=dict(facecolor='white', alpha=0.7))

            if i == 0:
                ax.legend(loc='upper right', fontsize=8)

    print(
        f"Rendering {len(stance_cmds)} animation frames ... (close the window to exit)")
    ani = FuncAnimation(fig, update, frames=len(stance_cmds),
                        interval=DT_ANIM * 1000, repeat=True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    test_lateral_stance_animation()
