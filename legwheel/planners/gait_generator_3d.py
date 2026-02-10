import numpy as np
import pandas as pd
from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D
from legwheel.utils.utils import create_command_csv, create_command_csv_phi

class GaitGenerator3D:
    """
    Advanced 3D Gait Generator for the Corgi robot.
    Coordinates 4 TrajectoryPlanner3D instances to produce coordinated gaits.
    """
    def __init__(self, gait_type="Trot", velocity=0.2, period=1.0, duty_factor=0.6, 
                 stand_height=0.3, step_height=0.08):
        """
        Initializes the 3D gait generator.
        
        Args:
            gait_type (str): Type of gait (e.g., "Trot", "Walk").
            velocity (float): Forward velocity (m/s).
            period (float): Gait cycle duration (s).
            duty_factor (float): Fraction of period in stance phase.
            step_height (float): Swing height (m).
            stand_height (float): Body height (m).
        """
        self.gait_type = gait_type
        self.velocity = velocity
        self.T = period
        self.duty = 1.0 - duty_factor
        
        # Initialize 4 individual planners
        self.planners = [
            TrajectoryPlanner3D(stand_height=stand_height, step_length=0.4, 
                               step_height=step_height, period=self.T, duty=self.duty, 
                               leg_index=i) 
            for i in range(4)
        ]

    def generate_full_gait(self, n_cycles=2, lateral_offset=0.0):
        """
        Generates coordinated 3D commands for all 4 legs.
        
        Args:
            n_cycles (int): Number of gait cycles.
            lateral_offset (float): Target lateral displacement (m).
        Returns:
            np.ndarray: (N, 12) array of [theta, beta, gamma] * 4 legs.
        """
        # 1. Generate base trajectories for each leg
        all_leg_trajs = [np.array(p.generate_trajectory(lateral_offset)) for p in self.planners]
        n_points = len(all_leg_trajs[0])
        
        # 2. Define Phase Offsets
        if self.gait_type == "Trot":
            offsets = [0.0, 0.5, 0.5, 0.0]
        elif self.gait_type == "Pace":
            offsets = [0.0, 0.5, 0.0, 0.5]
        else:
            offsets = [0.0, 0.0, 0.0, 0.0]
            
        total_len = n_points * n_cycles
        cmds = np.zeros((total_len, 12)) # 4 legs * 3 joints
        
        for i in range(4):
            shift = int(offsets[i] * n_points)
            indices = (np.arange(total_len) + shift) % n_points
            cmds[:, i*3 : i*3+3] = all_leg_trajs[i][indices]
            
        self.CMDS = cmds
        return cmds

    def export_to_csv(self, base_name="gait_3d"):
        """Exports the 12-DOF gait commands to CSV."""
        if not hasattr(self, 'CMDS'):
            self.generate_full_gait()
            
        # Helper exports currently handle 8 joints. 
        # Need expansion or manual export for 12 joints.
        # For now, let's create a full data frame export.
        cols = []
        for l in ['FL', 'FR', 'RR', 'RL']:
            cols += [f'{l}_Theta', f'{l}_Beta', f'{l}_Gamma']
            
        df = pd.DataFrame(self.CMDS, columns=cols)
        df.to_csv(base_name + "_12dof.csv", index=False)
        print(f"3D Gait exported to {base_name}_12dof.csv")

if __name__ == "__main__":
    gait = GaitGenerator3D(gait_type="Trot")
    data = gait.generate_full_gait()
    print(f"Generated 12-DOF Gait data: {data.shape}")
    gait.export_to_csv()