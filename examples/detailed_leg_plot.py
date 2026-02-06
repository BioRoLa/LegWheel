"""
Detailed Leg Visualization Example.
Derived from docs/notes/Note_While_ICRA.ipynb.

This script demonstrates how to plot the leg mechanism with detailed joint labels
and visualize the rim point (alpha) mapping across the linkage and wheel.
"""

import numpy as np
import matplotlib.pyplot as plt
from legwheel.visualization.plot_leg import PlotLeg
from legwheel.config import RobotParams

def main():
    # 1. Initialize Detailed Plotter
    plot_leg = PlotLeg()
    plot_leg.leg_shape.link_alpha = 0.6
    plot_leg.leg_shape.line_width = 2.5
    
    # 2. Define Pose
    theta = np.deg2rad(100)
    beta = np.deg2rad(45)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    plot_leg.plot_by_angle(theta, beta, ax=ax)
    
    # 3. Add Joint Labels (A, B, C, D, E, F, G, H, U, J)
    # We use the internal solver attributes to find label positions
    plot_leg.to_vector()
    joints_to_label = {
        'A_l': '$A$', 'B_l': '$B$', 'C_l': '$C$', 
        'D_l': '$D$', 'E': '$E$', 'F_l': '$F$', 
        'G': '$G$', 'H_l': '$H$', 'U_l': '$U$', 'J_l': '$J$'
    }
    
    offsets = {
        'H_l': (-0.01, 0.015), 'A_l': (0, -0.015), 'B_l': (0, -0.02),
        'U_l': (0, 0.003), 'D_l': (-0.015, -0.02), 'F_l': (-0.01, 0.01),
        'J_l': (0.005, -0.02), 'G': (0.01, -0.01)
    }

    for attr, label in joints_to_label.items():
        pos = getattr(plot_leg, attr)
        off_x, off_y = offsets.get(attr, (-0.01, -0.02))
        ax.text(pos[0] + off_x, pos[1] + off_y, label, 
                fontsize=14, fontfamily='serif', fontstyle='italic')

    # 4. Highlight alpha mapping (Rim Points)
    # Sampling alpha angles to show the gradient
    alphas = np.linspace(-180, 180, 200)
    cmap = plt.cm.plasma
    
    for a in alphas:
        p = plot_leg.rim_point(a)
        # Check if batch or single
        if isinstance(p, np.ndarray) and p.ndim == 1:
            ax.scatter(p[0], p[1], color=cmap((a + 180) / 360), s=2, alpha=0.3)

    ax.set_title(f"Leg Configuration and Alpha Mapping\ntheta={np.degrees(theta):.1f}, beta={np.degrees(beta):.1f}")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True, linestyle=':', alpha=0.6)
    
    print("Plotting detailed leg visualization...")
    plt.show()

if __name__ == "__main__":
    main()
