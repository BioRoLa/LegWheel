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
                 step_height=0.04, period=1.0, gait_type="Trot", dt=0.001):

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
        def calc_hip_vels(vx, vy, wz):
            vels = []
            for r_hip in self.hip_positions:
                hx = vx - wz * r_hip[1]
                hy = vy + wz * r_hip[0]
                vels.append(np.array([hx, hy, 0.0]))
            return vels

        raw_hip_velocities = calc_hip_vels(self.v_com[0], self.v_com[1], self.omega_z)

        # --- Workspace Guard (Global Twist Scaling) ---
        # Instead of clamping individual legs (which tears the rigid body geometry apart),
        # we find the worst-case violation across all 4 legs and scale the entire body twist.
        from scipy.optimize import root_scalar
        BETA_MAX = np.deg2rad(40)
        GAMMA_MAX = np.deg2rad(8)
        
        # Calculate theoretical max D for sagittal (x) and lateral (y)
        # using rough geometric estimates to find the scaling factor
        H_O = stand_height - RobotParams.WHEEL_RADIUS_PITCH
        R_link = RobotParams.WHEEL_RADIUS_PITCH * 0.2225 # approx offset
        L1 = RobotParams.WHEEL_RADIUS_PITCH * RobotParams.L1_RATIO
        R_arc = np.sqrt(L1**2 - R_link**2)
        H_hip = stand_height

        D_x_max = 2 * H_O * np.tan(BETA_MAX) + 2 * R_arc * BETA_MAX
        v_x_limit = D_x_max / (self.T * self.stance_duty)
        
        D_y_max = 2 * H_hip * np.sin(GAMMA_MAX)
        v_y_limit = D_y_max / (self.T * self.stance_duty)

        # Find maximum required scale down across all legs
        scale_x, scale_y = 1.0, 1.0
        for vel in raw_hip_velocities:
            if abs(vel[0]) > v_x_limit:
                scale_x = min(scale_x, v_x_limit / abs(vel[0]))
            if abs(vel[1]) > v_y_limit:
                scale_y = min(scale_y, v_y_limit / abs(vel[1]))
                
        global_scale = min(scale_x, scale_y)
        
        if global_scale < 1.0:
            print(f"  ⚠ Workspace guard: Scaling FULL body twist down to {global_scale*100:.1f}% "
                  f"to prevent leg kinematic singularity.")
            self.v_com *= global_scale
            self.omega_z *= global_scale
            self.twist[0] = self.omega_z
            self.twist[1:3] = self.v_com
            
        # Recompute final, safe hip velocities
        self.hip_velocities = calc_hip_vels(self.v_com[0], self.v_com[1], self.omega_z)

        # --- Dynamic Global Step Height Scaling ---
        # Find maximum beta and gamma usage among the planned safe velocities.
        # Approximation: D_stance = v_stance * T * duty_factor
        # beta ≈ D_stance / (2 * H_O)
        # gamma ≈ D_lat / (2 * H_hip)
        max_beta_ratio = 0.0
        max_gamma_ratio = 0.0
        for vel in self.hip_velocities:
            d_x = abs(vel[0]) * self.T * self.stance_duty
            beta_approx = d_x / (2 * H_O)
            max_beta_ratio = max(max_beta_ratio, beta_approx / BETA_MAX)

            d_y = abs(vel[1]) * self.T * self.stance_duty
            gamma_approx = d_y / (2 * H_hip)
            max_gamma_ratio = max(max_gamma_ratio, gamma_approx / GAMMA_MAX)

        global_usage = max(max_beta_ratio, max_gamma_ratio)
        global_step_scale = max(1.0 - 0.8 * global_usage, 0.2)

        if global_step_scale < 1.0:
            print(f"  ⚠ Step guard: Scaling FULL body step_height down to {global_step_scale*100:.1f}% "
                  f"(eff_step={step_height * global_step_scale:.4f}m) to preserve swing reach radius.")

        # --- Initialize 4 TrajectoryPlanner3D instances with safe per-leg velocities ---
        self.planners = [
            TrajectoryPlanner3D(
                stand_height=stand_height,
                velocity=self.hip_velocities[i].tolist(),
                step_height=step_height,
                period=self.T,
                stance_duty=self.stance_duty,
                dt=dt,
                leg_index=i,
                step_scale=global_step_scale
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

    def get_parameter_string(self) -> str:
        """
        Returns a string formatted with the actual (scaled) gait parameters.
        Useful for generating unique filenames based on the executed trajectory.
        """
        v_x_actual = self.v_com[0]
        v_y_actual = self.v_com[1]
        w_z_actual = self.omega_z
        step_actual = self.planners[0].step_height

        return f"{self.gait_type}_Vx{v_x_actual:.2f}_Vy{v_y_actual:.2f}_Wz{w_z_actual:.2f}_H{self.stand_height:.2f}_S{step_actual:.3f}_P{self.T:.1f}"

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
