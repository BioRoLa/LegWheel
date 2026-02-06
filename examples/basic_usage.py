"""Basic usage example for LegWheel package."""

import numpy as np
import matplotlib.pyplot as plt

# Simple import after installation
from legwheel.models.leg_model import LegModel
from legwheel.planners.trajectory_planning import TrajectoryPlanner
from legwheel.visualization.plot_leg import PlotLeg
from legwheel.config import RobotParams, TrajectoryParams

def main():
    """Demonstrate basic leg model usage."""
    # Create leg model
    leg = LegModel(sim=True)
    
    # Forward kinematics
    theta = np.deg2rad(90)
    beta = np.deg2rad(30)
    leg.forward(theta, beta)
    
    print(f"Foot position (G): {leg.G}")
    
    # Visualization
    plot_leg = PlotLeg(sim=True)
    ax = plot_leg.plot_by_angle(theta, beta)
    plt.show()
    
    # Trajectory planning
    traj_planner = TrajectoryPlanner(
        stand_height=TrajectoryParams.STAND_HEIGHT,
        step_length=TrajectoryParams.STEP_LENGTH,
        leg=leg,
        step_height=TrajectoryParams.STEP_HEIGHT,
        period=TrajectoryParams.PERIOD,
        dt=TrajectoryParams.DT,
        duty=TrajectoryParams.DUTY,
    )
    
    traj_planner.move()
    print(f"Generated {len(traj_planner.cmd)} trajectory points")

if __name__ == "__main__":
    main()
