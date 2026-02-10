import numpy as np
import pandas as pd
from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.leg_model import LegModel
from legwheel.planners.trajectory_planning import TrajectoryPlanner
from legwheel.utils.utils import create_command_csv, create_command_csv_phi

class GaitGenerator3D:
    """
    Advanced Gait Generator for the Corgi robot supporting 3D movements.
    Coordinates individual limb phases to produce walking, trotting, or pacing gaits.
    """
    def __init__(self, gait_type="Trot", velocity=0.2, period=1.0, duty_factor=0.6, step_height=0.08):
        """
        Initializes the 3D gait generator.
        
        Args:
            gait_type (str): Type of gait (e.g., "Trot", "Walk").
            velocity (float): Forward velocity (m/s).
            period (float): Gait cycle duration (s).
            duty_factor (float): Fraction of period in stance phase.
            step_height (float): Swing height (m).
        """
        self.gait_type = gait_type
        self.velocity = velocity
        self.T = period
        self.duty = 1.0 - duty_factor
        self.step_height = step_height
        
        # Internal template planner for a single limb
        self.planner = TrajectoryPlanner(
            stand_height=0.3,
            step_length=0.4,
            leg=LegModel(),
            step_height=self.step_height,
            period=self.T,
            dt=0.001,
            duty=self.duty,
            overlap=0.0
        )

    def generate_base_trajectory(self):
        """Computes the primary trajectory command sequence."""
        self.planner.move()
        return self.planner.cmd

    def generate_full_gait(self, n_cycles=2):
        """
        Generates coordinated commands for all 4 legs.
        
        Args:
            n_cycles (int): Number of gait cycles to generate.
        Returns:
            np.ndarray: (N, 8) array of [theta, beta] for 4 legs.
        """
        base_traj = np.array(self.generate_base_trajectory())
        n_points = len(base_traj)
        
        # Phase offsets for FL, FR, RR, RL
        if self.gait_type == "Trot":
            offsets = [0.0, 0.5, 0.5, 0.0]
        elif self.gait_type == "Pace":
            offsets = [0.0, 0.5, 0.0, 0.5]
        else:
            offsets = [0.0, 0.0, 0.0, 0.0]
            
        total_len = n_points * n_cycles
        cmds = np.zeros((total_len, 8))
        
        for i in range(4):
            shift = int(offsets[i] * n_points)
            indices = (np.arange(total_len) + shift) % n_points
            cmds[:, i*2 : i*2+2] = base_arr_slice = base_traj[indices]
            
        self.CMDS = cmds
        return cmds

    def export_to_csv(self, base_name="gait_3d"):
        """Exports gait commands to robot-readable CSV formats."""
        if not hasattr(self, 'CMDS'):
            self.generate_full_gait()
            
        theta_cmds = self.CMDS[:, 0::2].T
        beta_cmds = self.CMDS[:, 1::2].T
        
        create_command_csv(theta_cmds, beta_cmds, base_name + "_std", transform=False)
        create_command_csv_phi(theta_cmds, beta_cmds, base_name + "_phi", transform=False)
        print(f"Gait exported to {base_name}_std.csv and {base_name}_phi.csv")

if __name__ == "__main__":
    gait = GaitGenerator3D(gait_type="Trot")
    data = gait.generate_full_gait()
    print(f"Generated 3D Gait data: {data.shape}")
