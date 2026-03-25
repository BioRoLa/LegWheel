import os
import sys

# Add the project root to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.models.corgi_robot import CorgiRobot

def draw_ground_plane(ax, z_ground, center_xy=(0, 0), extent=0.5):
    """Draws a translucent ground plane at the given z height."""
    xg, yg = np.meshgrid(
        np.linspace(center_xy[0] - extent, center_xy[0] + extent, 2),
        np.linspace(center_xy[1] - extent, center_xy[1] + extent, 2)
    )
    zg = np.full_like(xg, z_ground)
    ax.plot_surface(xg, yg, zg, color='gray', alpha=0.3, edgecolor='none')

def main():
    DT_ANIM = 0.005
    VELOCITY = 0.15
    PERIOD = 1.0
    DUTY = 0.75
    STAND_HEIGHT = 0.3
    
    # 1. Generate full gait for all 4 legs
    print("Generating full robot gait ...")
    generator = GaitGenerator3D(
        gait_type="Trot", 
        velocity=VELOCITY, 
        period=PERIOD, 
        duty_factor=DUTY, 
        stand_height=STAND_HEIGHT, 
        step_height=0.04
    )
    # Generate 1 full cycle
    cmds = generator.generate_full_gait(n_cycles=1)
    
    # 2. Setup Robot for Kinematics Rendering
    robot = CorgiRobot()
    
    fig = plt.figure(figsize=(10, 8))
    fig.suptitle(f"Full Robot 3D Trot Gait  (vx={VELOCITY} m/s)", fontsize=14, fontweight='bold')
    ax = fig.add_subplot(111, projection='3d')
    
    lim = 0.4
    
    def update(frame_idx):
        ax.clear()
        
        # Ground plane
        draw_ground_plane(ax, -STAND_HEIGHT, extent=0.5)
        
        # Draw Robot Chassis (Approximate rectangular base)
        wb = robot.wheelbase / 2
        tw = robot.trackwidth / 2
        bx = [wb, wb, -wb, -wb, wb]
        by = [tw, -tw, -tw, tw, tw]
        bz = [0, 0, 0, 0, 0]
        ax.plot(bx, by, bz, color='k', linewidth=3, alpha=0.8, label="Chassis")
        
        q_frame = cmds[frame_idx]
        
        # Draw each leg
        for i in range(4):
            q_leg = q_frame[i*3 : i*3+3]
            leg = robot.legs[i]
            
            # The leg origin offset relative to Body center is purely determined by the CorgiLeg structure
            leg.plot_leg_3d(q_leg[0], q_leg[1], q_leg[2], ax=ax)
            leg.plot_frames(ax, gamma=q_leg[2], axis_len=0.05)
            
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-STAND_HEIGHT - 0.05, 0.1)
        ax.set_xlabel('X (Front)')
        ax.set_ylabel('Y (Left)')
        ax.set_zlabel('Z (Up)')
        
        # Add frame info text
        t_sec = frame_idx * DT_ANIM
        phase = t_sec / PERIOD
        ax.text2D(0.05, 0.95, f"Time: {t_sec:.3f} s  |  Phase: {phase:.2f}", 
                  transform=ax.transAxes, fontsize=10, bbox=dict(facecolor='white', alpha=0.8))
        
        # Legend (only needs to be drawn once)
        if frame_idx == 0:
            ax.legend()
            
    print(f"Rendering {len(cmds)} frames ... (Close window to exit)")
    ani = FuncAnimation(fig, update, frames=len(cmds), interval=DT_ANIM * 1000, repeat=True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
