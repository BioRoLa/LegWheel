"""
Swing Phase Trajectory Visual Test (Single Leg).

Visualizes the FL leg executing the swing phase, validating the 3D Bezier curve
and the Inverse Kinematics tracking performance.

Usage:
    /home/starlee/envs/legwheel/bin/python examples/test_swing_trajectory.py
"""

from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

def draw_ground_plane(ax, z_ground, center_xy=(0, 0), extent=0.3):
    cx, cy = center_xy
    xx, yy = np.meshgrid(
        np.linspace(cx - extent, cx + extent, 2),
        np.linspace(cy - extent, cy + extent, 2)
    )
    zz = np.full_like(xx, z_ground)
    ax.plot_surface(xx, yy, zz, alpha=0.15, color='sienna', zorder=0)

def test_swing_trajectory_animation():
    print("=========================================")
    print(" Swing Phase Trajectory — Single Leg     ")
    print(" Bezier Target (Red) vs IK Tracking (Blue)")
    print("=========================================")

    LEG_INDEX = 0  # FL
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

    print("Generating pure stance+swing trajectory ...")
    cmd = planner.generate_trajectory()
    cmd = np.array(cmd)

    n_stance = int(planner.T * planner.stance_duty / planner.dt)
    
    # Extract only swing phase (we start right at life-off)
    swing_cmds = cmd[n_stance:]
    
    # Calculate geometric path of the foot purely from IK commands
    actual_foot_path = []
    bez_target_data = getattr(planner, '_last_swing_target_path', None)
    bez_target_path = None
    if bez_target_data is not None:
        bez_target_path = np.array([d[0] for d in bez_target_data])
        # To accurately compare with what IK was asked to track, we evaluate FK at that alpha
        alphas = [d[1] for d in bez_target_data]
    
    for i, q in enumerate(swing_cmds):
        if bez_target_data is not None:
            a = alphas[i]
        else:
            a = kin.foot_rim_contact_fk(*q)[0]
        actual_foot_path.append(kin.forward_kinematics(*q, alpha=a, w=0.0))
        
    actual_foot_path = np.array(actual_foot_path)

    print(f"  Swing frames: {len(swing_cmds)}")
    
    if bez_target_path is not None:
        rms = np.sqrt(np.mean((actual_foot_path - bez_target_path)**2))
        print(f"  IK Tracking RMS Error: {rms:.6f} m")
        max_err = np.max(np.linalg.norm(actual_foot_path - bez_target_path, axis=1))
        print(f"  IK Tracking MAX Error: {max_err:.6f} m")

    # Check max Z clearance
    max_z = actual_foot_path[:, 2].max()
    z_ground = actual_foot_path[0, 2]
    print(f"  Start Z: {z_ground:.4f} m")
    print(f"  Max clearance achieved: {max_z - z_ground:.4f} m (Target: {planner.step_height} m)")
    
    z_ground_viz = -0.31

    fig = plt.figure(figsize=(18, 6))
    fig.suptitle("Swing Phase Tracking (Isometric, Side, Front)", fontsize=14, fontweight='bold')

    axes = [
        fig.add_subplot(131, projection='3d'),
        fig.add_subplot(132, projection='3d'),
        fig.add_subplot(133, projection='3d'),
    ]
    titles = ['Isometric View', 'Side View', 'Front View']
    views = [(25, -50), (0, -90), (0, 0)]
    mo = kin.p_Mi_in_B
    lim = 0.25

    def update(frame_idx):
        q_f = swing_cmds[frame_idx]
        p_act = actual_foot_path[frame_idx]

        for i, ax in enumerate(axes):
            ax.clear()
            draw_ground_plane(ax, z_ground_viz, center_xy=(mo[0], mo[1]), extent=lim)
            
            # Draw robot leg
            kin.plot_leg_3d(q_f[0], q_f[1], q_f[2], ax)
            
            # Trace actual path
            ax.plot(actual_foot_path[:, 0], actual_foot_path[:, 1], actual_foot_path[:, 2],
                    'b--', linewidth=2, alpha=0.5, label='Actual IK Tracking')
            
            # Trace Bezier target path
            if bez_target_path is not None:
                ax.plot(bez_target_path[:, 0], bez_target_path[:, 1], bez_target_path[:, 2],
                        'r-', linewidth=1.5, alpha=0.8, label='Bezier Target Path')
            
            # Current Point
            ax.scatter(p_act[0], p_act[1], p_act[2], color='blue', s=80, zorder=10)

            ax.set_box_aspect([1, 1, 1])
            ax.view_init(elev=views[i][0], azim=views[i][1])
            ax.set_xlim(mo[0] - lim, mo[0] + lim)
            ax.set_ylim(mo[1] - lim, mo[1] + lim)
            ax.set_zlim(z_ground_viz - 0.05, mo[2] + lim)
            ax.set_title(titles[i])
            ax.set_xlabel('X') ; ax.set_ylabel('Y') ; ax.set_zlabel('Z')
            
            if i == 0:
                ax.legend()
    
    ani = FuncAnimation(fig, update, frames=len(swing_cmds), interval=DT_ANIM * 1000, repeat=True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    test_swing_trajectory_animation()
