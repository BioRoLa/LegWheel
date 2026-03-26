import numpy as np
from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.config import RobotParams


# --- Gait Type Definitions ---
# Each gait defines: phase_offsets [FL, FR, RR, RL] and stance_duty
# traj curve representation: [ stance duty | swing duty ]
# phase_offset = start partial in traj csv ex: 0.5 -> start from mid of traj curve
GAIT_LIBRARY = {
    "Walk": {
        "phase_offsets": [0.0, 0.5, 0.75, 0.25],
        "stance_duty": 0.75,
    },
    "Trot": {
        # Diagonal pairs: FL+RR (phase 0), FR+RL (phase 0.5)
        "phase_offsets": [0.0, 0.5, 0.0, 0.5],
        "stance_duty": 0.6,
    },
    "Pace": {
        # Same-side pairs: FL+RL (phase 0), FR+RR (phase 0.5)
        "phase_offsets": [0.0, 0.5, 0.5, 0.0],
        "stance_duty": 0.6,
    },
    "Bound": {
        "phase_offsets": [0.0, 0.0, 0.5, 0.5],
        "stance_duty": 0.6,
    },
    "Pronk": {
        "phase_offsets": [0.0, 0.0, 0.0, 0.0],
        "stance_duty": 0.5,
    },
}


class GaitGenerator3D:
    """
    3D Gait Generator for the Corgi robot.

    Coordinates 4 TrajectoryPlanner3D instances to produce coordinated gaits.
    Each leg's trajectory velocity is derived from the robot's body twist
    mapped to the corresponding hip mounting point via rigid-body velocity relation:
        v_hip_i = v_COM + omega x r_{COM -> hip_i}

    Args:
        stand_height (float): Target standing height (m).
        twist (np.ndarray): Body twist [omega_z, v_x, v_y] (yaw rate rad/s, forward m/s, lateral m/s).
                            Simplified planar twist for ground locomotion.
        step_height (float): Swing clearance height (m).
        period (float): Gait cycle duration (s).
        gait_type (str): One of "Walk", "Trot", "Pace", "Bound", "Pronk".
        dt (float): Planner time step (s).
    """

    def __init__(self, stand_height=0.31, twist=np.array([0.0, 0.15, 0.0]),
                 step_height=0.04, period=1.0, gait_type="Trot", dt=0.005):

        self.stand_height = stand_height
        self.step_height = step_height
        self.T = period
        self.dt = dt

        # Validate gait type
        if gait_type not in GAIT_LIBRARY:
            raise ValueError(
                f"Unknown gait type '{gait_type}'. Choose from: {list(GAIT_LIBRARY.keys())}")
        self.gait_type = gait_type
        gait_def = GAIT_LIBRARY[gait_type]
        self.phase_offsets = gait_def["phase_offsets"]
        self.stance_duty = gait_def["stance_duty"]

        # Parse the planar twist: [omega_z, v_x, v_y]
        self.twist = np.array(twist, dtype=float)
        self.omega_z = self.twist[0]  # yaw rate (rad/s)
        self.v_com = self.twist[1:3]  # [vx, vy] linear velocity of COM

        # --- Compute per-leg hip velocities from twist ---
        # Hip mounting points in Body Frame {B} (from CorgiLegKinematics)
        self.legs = [CorgiLegKinematics(i) for i in range(4)]
        self.hip_positions = [leg.p_Mi_in_B for leg in self.legs]

        # v_hip_i = v_COM + omega x r_{COM -> hip_i}
        # For planar motion: omega = [0, 0, omega_z], r = [rx, ry, 0]
        # omega x r = [-omega_z * ry, omega_z * rx, 0]
        self.hip_velocities = []
        for r_hip in self.hip_positions:
            v_hip_x = self.v_com[0] - self.omega_z * r_hip[1]
            v_hip_y = self.v_com[1] + self.omega_z * r_hip[0]
            self.hip_velocities.append(np.array([v_hip_x, v_hip_y, 0.0]))

        # --- Initialize 4 TrajectoryPlanner3D instances with per-leg velocities ---
        self.planners = [
            TrajectoryPlanner3D(
                stand_height=stand_height,
                velocity=self.hip_velocities[i].tolist(),
                step_height=step_height,
                period=self.T,
                stance_duty=self.stance_duty,
                dt=dt,
                leg_index=i
            )
            for i in range(4)
        ]

    def generate_full_gait(self, n_cycles=2):
        """
        Generates coordinated 3D commands for all 4 legs with phase offsets.

        Args:
            n_cycles (int): Number of gait cycles.
        Returns:
            np.ndarray: (N, 12) array of [theta, beta, gamma] * 4 legs.
        """
        # 1. Generate base trajectories for each leg (one cycle)
        all_leg_trajs = [np.array(p.generate_trajectory())
                         for p in self.planners]
        n_points = len(all_leg_trajs[0])

        # 2. Apply phase offsets
        total_len = n_points * n_cycles
        cmds = np.zeros((total_len, 12))  # 4 legs * 3 joints

        for i in range(4):
            shift = int(self.phase_offsets[i] * n_points)
            indices = (np.arange(total_len) + shift) % n_points
            cmds[:, i * 3: i * 3 + 3] = all_leg_trajs[i][indices]

        self.CMDS = cmds
        return cmds

    def print_summary(self):
        """Prints a summary of the gait configuration."""
        print(f"=== GaitGenerator3D Summary ===")
        print(f"  Gait Type:    {self.gait_type}")
        print(f"  Period:       {self.T:.2f} s")
        print(f"  Stance Duty:  {self.stance_duty:.2f}")
        print(f"  Body Twist:   ω_z={self.omega_z:.3f} rad/s, "
              f"v_x={self.v_com[0]:.3f} m/s, v_y={self.v_com[1]:.3f} m/s")
        print(f"  Stand Height: {self.stand_height:.3f} m")
        print(f"  Step Height:  {self.step_height:.3f} m")
        print()
        labels = ['FL', 'FR', 'RR', 'RL']
        for i in range(4):
            v = self.hip_velocities[i]
            r = self.hip_positions[i]
            phi = self.phase_offsets[i]
            print(f"  {labels[i]}: hip=[{r[0]:+.3f}, {r[1]:+.3f}, {r[2]:+.3f}] m, "
                  f"v_hip=[{v[0]:+.4f}, {v[1]:+.4f}, {v[2]:+.4f}] m/s, "
                  f"phase={phi:.2f}")

    def export_to_csv(self, base_name="gait_3d"):
        """Exports the 12-DOF gait commands to CSV."""
        if not hasattr(self, 'CMDS'):
            self.generate_full_gait()

        import pandas as pd
        cols = []
        for l in ['FL', 'FR', 'RR', 'RL']:
            cols += [f'{l}_Theta', f'{l}_Beta', f'{l}_Gamma']

        df = pd.DataFrame(self.CMDS, columns=cols)
        df.to_csv(base_name + "_12dof.csv", index=False)
        print(f"3D Gait exported to {base_name}_12dof.csv")


if __name__ == "__main__":
    # Example: Trot forward at 0.15 m/s
    gait = GaitGenerator3D(
        stand_height=0.31,
        twist=[0.0, 0.15, 0.0],  # [omega_z, vx, vy]
        step_height=0.04,
        period=1.0,
        gait_type="Trot"
    )
    gait.print_summary()
    data = gait.generate_full_gait(n_cycles=2)
    print(f"\nGenerated 12-DOF Gait data: {data.shape}")
