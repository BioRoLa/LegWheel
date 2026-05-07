import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

project_root = Path(__file__).resolve().parent.parent.parent
sys.path.append(str(project_root))

from legwheel.models.active_banking_dynamics import ActiveBankingDynamics

def plot_active_banking_performance():
    """
    Plot the induced body roll, CoM height, and maximum lateral acceleration 
    across different active banking angles (gamma).
    """
    banking = ActiveBankingDynamics()
    
    # Sweep gamma from 0 (flat) to 30 degrees
    gamma_sweep = np.linspace(0, 30, 15)
    
    rolls = []
    margins = []
    z_coms = []
    a_lats = []
    
    theta_nom = 75.0
    beta_nom = 0.0
    turn_radius = 2.0 # Fixed radius for max velocity evaluation
    
    for g_bank in gamma_sweep:
        res = banking.calculate_cornering_stability(
            theta_deg=theta_nom, 
            beta_deg=beta_nom, 
            gamma_bank_deg=g_bank, 
            turn_radius=turn_radius
        )
        rolls.append(res["induced_roll_deg"])
        margins.append(res["y_margin"])
        z_coms.append(res["com_z"])
        a_lats.append(res["a_lat_max_g"])
        
    # Create a 3-panel plot for the performance metrics
    fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
    
    # Panel 1: Induced Body Roll
    axs[0].plot(gamma_sweep, rolls, 'bo-', linewidth=2)
    axs[0].set_ylabel('Induced Body Roll $\phi_{body}$ (°)')
    axs[0].set_title(f'Active Banking Kinematics & Stability\n($\\theta={theta_nom}^\circ, \\beta={beta_nom}^\circ$, Turn Radius={turn_radius}m)')
    axs[0].grid(True, linestyle='--')
    
    # Panel 2: Center of Mass Height
    axs[1].plot(gamma_sweep, z_coms, 'go-', linewidth=2)
    axs[1].set_ylabel('CoM Height $Z_{com}$ (m)')
    axs[1].grid(True, linestyle='--')
    
    # Panel 3: Maximum Lateral Acceleration (Cornering limit)
    axs[2].plot(gamma_sweep, a_lats, 'ro-', linewidth=2)
    axs[2].set_xlabel('Active Banking Command $\gamma_{bank}$ (°)')
    axs[2].set_ylabel('Max Lateral Accel (g)')
    axs[2].grid(True, linestyle='--')
    
    plt.tight_layout()
    
    output_dir = project_root / 'output' / 'wheeled_dynamics'
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path_perf = output_dir / 'active_banking_performance.png'
    plt.savefig(out_path_perf, dpi=300)
    print(f"Performance plot saved to {out_path_perf}")
    plt.close(fig)
    
    # Pick a few key poses for rendering the 3D robot separately
    key_gammas = [0, 15, 30]
    from legwheel.models.corgi_robot import CorgiRobot
    from render.plot_corgi_robot import draw_corgi_robot
    
    robot = banking.robot
    # Create a separate plot for the 3D robot renders
    fig_rob = plt.figure(figsize=(15, 6))
    
    # Render Robot Poses
    for i, g_bank in enumerate(key_gammas):
        res = banking.calculate_cornering_stability(
            theta_deg=theta_nom, 
            beta_deg=beta_nom, 
            gamma_bank_deg=g_bank, 
            turn_radius=turn_radius
        )
        
        ax_r = fig_rob.add_subplot(1, 3, i+1, projection='3d')
        roll_rad = np.deg2rad(res["induced_roll_deg"])
        
        # Override the robot's base orientation and position
        robot.base_ori = [roll_rad, 0, 0]
        # In calculate_induced_roll we know it shifted by min_z to touch ground Z=0
        _, z_off, _, q_list = banking.calculate_induced_roll(theta_nom, beta_nom, g_bank)
        robot.base_pos = np.array([0, 0, -z_off])
        
        # Set up the 3D plot view angle
        ax_r.view_init(elev=15, azim=180) # View from the front
        
        # Extract individual gammas for drawing
        gamma_list = [q[2] for q in q_list]
        
        draw_corgi_robot(ax_r, theta=np.deg2rad(theta_nom), beta=np.deg2rad(beta_nom), 
                         gamma=0.0, show_bounds=True, gamma_list=gamma_list, show_support_polygon=True, robot_state=robot)
        
        # Override limits after draw_corgi_robot sets them
        ax_r.set_xlim(-0.25, 0.25)
        ax_r.set_ylim(-0.35, 0.35)
        ax_r.set_zlim(0, 0.5)
        
        # Draw a line representing the Centrifugal Force and Gravity vector from CoM
        com_y = res["com_y"]
        com_z = res["com_z"]
        # Gravity is down (-z), centrifugal is right (-y) for a left turn
        scale = 0.1
        ax_r.quiver(0, com_y, com_z, 0, 0, -scale, color='g', linewidth=3, label='Gravity')
        ax_r.quiver(0, com_y, com_z, 0, -scale * res['a_lat_max_g'], 0, color='r', linewidth=3, label='Max Centrifugal')
        
        ax_r.set_title(f"$\gamma_{{bank}} = {g_bank}^\circ \Rightarrow \phi_{{body}} = {res['induced_roll_deg']:.1f}^\circ$\n$a_{{lat,max}} = {res['a_lat_max_g']:.2f}g$")
        if i == 0:
            ax_r.legend(loc='upper right')
            
    plt.tight_layout()
    out_path_poses = output_dir / 'active_banking_poses.png'
    plt.savefig(out_path_poses, dpi=300)
    print(f"Poses plot saved to {out_path_poses}")
    
    # Show without blocking
    plt.show(block=False)
    plt.pause(3)
    plt.close('all')

if __name__ == "__main__":
    plot_active_banking_performance()
