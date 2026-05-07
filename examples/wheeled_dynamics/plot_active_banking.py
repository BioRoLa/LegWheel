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
        
    # Create a 3-panel plot
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
    out_path = output_dir / 'active_banking_performance.png'
    plt.savefig(out_path, dpi=300)
    print(f"Plot saved to {out_path}")
    
    # Show without blocking
    plt.show(block=False)
    plt.pause(3)
    plt.close()

if __name__ == "__main__":
    plot_active_banking_performance()
