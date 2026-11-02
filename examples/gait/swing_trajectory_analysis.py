"""
Swing Trajectory Analysis Example.
Derived from docs/notes/Note_While_ICRA.ipynb.

This script demonstrates how to plan a swing phase trajectory, 
calculate the start joint angles (theta, beta), and visualize the 
resulting path of the contact point (G).
"""

import numpy as np
import matplotlib.pyplot as plt
from legwheel.planners.trajectory_planning import TrajectoryPlanner
from legwheel.models.leg_model import LegModel
from legwheel.visualization.plot_leg import PlotLeg

def main():
    # 1. Setup Planner
    # Based on assumptions: OO_r = G' - G0, pure rolling
    planner = TrajectoryPlanner(
        stand_height=0.35,
        step_length=0.4,
        leg=LegModel(),
        step_height=0.08,
        period=2.0,
        dt=0.01,
        duty=0.25
    )
    
    # 2. Derive Initial TB (Start of Swing)
    theta0, beta0, D = planner.get_init_tb()
    print(f"Calculated Initial Pose: Theta={np.degrees(theta0):.2f}°, Beta={np.degrees(beta0):.2f}°")
    print(f"Calculated Hip Movement (D): {D:.4f} m")

    # 3. Generate Trajectory
    planner.move()
    curve_cmd = np.array(planner.cmd)
    
    # 4. Extract Foot Positions (G point)
    leg = LegModel()
    g_positions = []
    for t, b in curve_cmd:
        leg.forward(t, b, vector=True)
        g_positions.append(leg.G.copy())
    g_positions = np.array(g_positions)

    # 5. Plot Results
    fig, ax = plt.subplots(figsize=(10, 5))
    
    # Plot G point path
    ax.plot(g_positions[:, 0], g_positions[:, 1], 'r--', label='G point path')
    
    # Overlay leg at start of swing
    plot_leg = PlotLeg()
    plot_leg.leg_shape.link_alpha = 0.3
    plot_leg.plot_by_angle(theta0, -beta0, ax=ax)
    
    # Overlay leg at end of swing (relative to D)
    plot_leg.plot_by_angle(theta0, beta0, O=(D, 0), ax=ax)

    ax.set_title("Swing Phase Trajectory Analysis")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal')
    
    print(f"Generated {len(curve_cmd)} command points for the swing phase.")
    plt.show()

if __name__ == "__main__":
    main()
