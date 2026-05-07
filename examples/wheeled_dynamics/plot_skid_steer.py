import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# Add the root directory of the LegWheel project to the Python path
project_root = Path(__file__).resolve().parent.parent.parent
sys.path.append(str(project_root))

from legwheel.models.wheeled_dynamics import WheeledDynamics

def plot_skid_steer_kinematics():
    """
    Plot the turning radius vs wheel speed difference under different skid factors.
    """
    dynamics = WheeledDynamics(mu_lat=0.6)
    W_track = dynamics.get_track_width()
    R_wheel = dynamics.R
    
    # Target forward velocity
    v_x = 2.0  # m/s
    
    # Turning radii to evaluate
    turn_radii = np.linspace(0.5, 5.0, 100) # m
    omega_z_vals = v_x / turn_radii
    
    # Evaluate at different skid factors
    skid_factors = [1.0, 1.2, 1.5, 2.0]
    
    plt.figure(figsize=(10, 6))
    
    for chi in skid_factors:
        delta_omegas = []
        for w_z in omega_z_vals:
            w_L, w_R = dynamics.differential_drive_ik(v_x, w_z, skid_factor=chi)
            delta_omegas.append(w_R - w_L) # rad/s
            
        plt.plot(turn_radii, delta_omegas, label=f'Skid Factor $\chi={chi}$')
        
    plt.title(f'Turning Radius vs Wheel Speed Difference\n($v_x = {v_x}$ m/s)')
    plt.xlabel('Target Turning Radius $R_{turn}$ (m)')
    plt.ylabel('Wheel Speed Difference $\Delta \omega_{whl}$ (rad/s)')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()
    
    output_dir = project_root / 'output' / 'wheeled_dynamics'
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / 'skid_steer_kinematics.png'
    plt.savefig(out_path, dpi=300)
    print(f"Plot saved to {out_path}")
    
    # Add a timeout so the script won't block the agent
    plt.show(block=False)
    plt.pause(3)
    plt.close()

if __name__ == '__main__':
    plot_skid_steer_kinematics()
