"""
Full Gait Cycle Test (Single Leg): Stance + Swing Combined.

Visualizes the FL leg executing one complete gait cycle:
  1. Stance Phase: Rolling contact (beta sweep)
  2. Swing Phase: Bezier curve recovery (twist-mapped material point)

The animation shows the stance contact trace (green) and swing trajectory (magenta),
with the full mechanism rendered at each frame.

Usage:
    /home/starlee/envs/legwheel/bin/python examples/test_full_gait_cycle.py
"""

from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import argparse
from matplotlib.animation import FuncAnimation, FFMpegWriter

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


def draw_ground_plane(ax, z_ground, center_xy=(0, 0), extent=0.3):
    cx, cy = center_xy
    xx, yy = np.meshgrid(
        np.linspace(cx - extent, cx + extent, 2),
        np.linspace(cy - extent, cy + extent, 2)
    )
    zz = np.full_like(xx, z_ground)
    ax.plot_surface(xx, yy, zz, alpha=0.15, color='sienna', zorder=0)


def test_full_gait_cycle():
    print("=========================================")
    print(" Full Gait Cycle — Single Leg (FL)       ")
    print(" Stance (Green) + Swing (Magenta)        ")
    print("=========================================")

    LEG_INDEX = 0
    DT_ANIM = 0.02
    VX = 0.15
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

    print("Generating full gait trajectory ...")
    cmd = planner.generate_trajectory()
    cmd = np.array(cmd)

    n_stance = int(planner.T * planner.stance_duty / planner.dt)
    n_stance = min(n_stance, len(cmd))
    n_total = len(cmd)
    n_swing = n_total - n_stance

    print(f"  Total frames: {n_total}  |  Stance: {n_stance}  |  Swing: {n_swing}")

    # --- Pre-compute all foot positions ---
    # Stance: contact point (alpha from foot_rim_contact_fk)
    # Swing: material point at alpha_td
    stance_foot = []
    for q in cmd[:n_stance]:
        contact = kin.foot_rim_contact_fk(*q)
        stance_foot.append(kin.forward_kinematics(*q, alpha=contact[0], w=0.0))
    stance_foot = np.array(stance_foot)

    # Swing: use alpha_td from the stored target path
    swing_data = planner._last_swing_target_path
    alpha_td = swing_data[0][1]  # All same alpha_td

    swing_foot = []
    for q in cmd[n_stance:]:
        swing_foot.append(kin.forward_kinematics(*q, alpha=alpha_td, w=0.0))
    swing_foot = np.array(swing_foot)

    # Bezier target path
    bez_target = np.array([d[0] for d in swing_data])

    # IK tracking error for swing
    if len(swing_foot) == len(bez_target):
        rms = np.sqrt(np.mean((swing_foot - bez_target) ** 2))
        max_err = np.max(np.linalg.norm(swing_foot - bez_target, axis=1))
        print(f"  Swing IK Tracking — RMS: {rms:.6f} m, MAX: {max_err:.6f} m")

    # Joint angle analysis
    beta_start = np.rad2deg(cmd[0, 1])
    beta_end_stance = np.rad2deg(cmd[n_stance - 1, 1])
    gamma_start = np.rad2deg(cmd[0, 2])
    gamma_end_stance = np.rad2deg(cmd[n_stance - 1, 2])
    print(f"  Stance β: {beta_start:.2f}° → {beta_end_stance:.2f}°")
    print(f"  Stance γ: {gamma_start:.2f}° → {gamma_end_stance:.2f}°")

    # Ground level
    z_ground = stance_foot[:, 2].min()
    print(f"  Ground Z: {z_ground:.4f} m")
    print(f"  Swing max clearance: {swing_foot[:, 2].max() - z_ground:.4f} m")

    # --- Subsample for animation ---
    step = max(1, int(DT_ANIM / planner.dt))
    anim_indices = list(range(0, n_total, step))

    # --- Figure setup ---
    fig = plt.figure(figsize=(18, 6))
    fig.suptitle("Full Gait Cycle — Stance (Green) + Swing (Magenta)", fontsize=14, fontweight='bold')

    axes = [
        fig.add_subplot(131, projection='3d'),
        fig.add_subplot(132, projection='3d'),
        fig.add_subplot(133, projection='3d'),
    ]
    titles = ['Isometric View', 'Side View', 'Front View']
    views = [(25, -50), (0, -90), (0, 0)]
    mo = kin.p_Mi_in_B
    lim = 0.25

    def update(frame_count):
        idx = anim_indices[frame_count]
        q_f = cmd[idx]
        is_stance = idx < n_stance
        phase_name = "STANCE" if is_stance else "SWING"

        for i, ax in enumerate(axes):
            ax.clear()
            draw_ground_plane(ax, z_ground, center_xy=(mo[0], mo[1]), extent=lim)

            # Draw mechanism
            kin.plot_leg_3d(q_f[0], q_f[1], q_f[2], ax)

            # Full stance trace (green)
            ax.plot(stance_foot[:, 0], stance_foot[:, 1], stance_foot[:, 2],
                    'g-', linewidth=2.5, alpha=0.7, label='Stance Contact')

            # Full swing trace (magenta)
            ax.plot(swing_foot[:, 0], swing_foot[:, 1], swing_foot[:, 2],
                    'm--', linewidth=2, alpha=0.7, label='Swing IK Tracking')

            # Bezier target path (red dotted)
            ax.plot(bez_target[:, 0], bez_target[:, 1], bez_target[:, 2],
                    'r:', linewidth=1.5, alpha=0.5, label='Bezier Target')

            # Current foot point
            if is_stance:
                foot_idx = min(idx, len(stance_foot) - 1)
                p_now = stance_foot[foot_idx]
                ax.scatter(p_now[0], p_now[1], p_now[2], color='green', s=100, zorder=10)
            else:
                foot_idx = min(idx - n_stance, len(swing_foot) - 1)
                p_now = swing_foot[foot_idx]
                ax.scatter(p_now[0], p_now[1], p_now[2], color='magenta', s=100, zorder=10)

            ax.set_box_aspect([1, 1, 1])
            ax.view_init(elev=views[i][0], azim=views[i][1])
            
            # Equalize axes (1:1:1 scaling) - FIXED SCALE for consistent XYZ
            # Center around hip module origin (mo) in X/Y, but offset Z to show ground
            z_mid = (z_ground - 0.05 + mo[2] + 0.1) / 2
            fixed_span = 0.5 # Total span for each axis (0.5m)
            ax.set_xlim(mo[0] - fixed_span/2, mo[0] + fixed_span/2)
            ax.set_ylim(mo[1] - fixed_span/2, mo[1] + fixed_span/2)
            ax.set_zlim(z_mid - fixed_span/2, z_mid + fixed_span/2)

            q_deg = np.rad2deg(q_f)
            ax.set_title(f"{titles[i]}  |  {phase_name}  [{idx}/{n_total}]")
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')

            info = (f"q: θ={q_deg[0]:.1f}°, β={q_deg[1]:.1f}°, γ={q_deg[2]:.1f}°")
            ax.text2D(0.05, 0.05, info, transform=ax.transAxes,
                      fontsize=9, bbox=dict(facecolor='white', alpha=0.7))

            if i == 0:
                ax.legend(loc='upper right', fontsize=7)

    parser = argparse.ArgumentParser(description="Gait Cycle Animation")
    parser.add_argument("--save", action="store_true", help="Save animation as video")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second for saved video")
    args = parser.parse_args()

    print(f"Rendering {len(anim_indices)} frames ...")
    ani = FuncAnimation(fig, update, frames=len(anim_indices),
                        interval=DT_ANIM * 1000, repeat=not args.save)
    
    plt.tight_layout()

    if args.save:
        output_file = "gait_cycle.mp4"
        print(f"Saving animation to {output_file} (FPS: {args.fps})...")
        writer = FFMpegWriter(fps=args.fps, metadata=dict(artist='LegWheel'), bitrate=5000)
        ani.save(output_file, writer=writer)
        print("Done.")
    else:
        plt.show()


if __name__ == "__main__":
    test_full_gait_cycle()
