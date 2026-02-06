"""
LegWheel - A library for leg-wheel robot control and trajectory planning.

This package provides tools for:
- Leg kinematics and dynamics modeling
- Trajectory planning for leg-wheel robots
- Gait generation and control
- Visualization of leg movements
"""

__version__ = "0.1.0"
__author__ = "Your Name"

# Import main classes for convenient access
from legwheel.models.leg_model import LegModel
from legwheel.models.leg_kinematics import LegKinematics
from legwheel.planners.trajectory_planning import TrajectoryPlanner
from legwheel.planners.gait_generator import Gait_Generator, leg_object
from legwheel.visualization.plot_leg import PlotLeg
from legwheel.config import (
    RobotParams,
    TrajectoryParams,
    GaitParams,
    DATA_DIR,
    OUTPUT_DIR,
)

# Define public API
__all__ = [
    # Core classes
    "LegModel",
    "LegKinematics",
    "TrajectoryPlanner",
    "Gait_Generator",
    "leg_object",
    "PlotLeg",
    # Configuration
    "RobotParams",
    "TrajectoryParams",
    "GaitParams",
    "DATA_DIR",
    "OUTPUT_DIR",
]
