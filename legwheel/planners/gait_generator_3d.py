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
        "phase_offsets": [0.75, 0.25, 0.5, 0.0],  # swing order: FL→RR→FR→RL (1→3→2→4)
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
        stance_duty (float): Optional stance duty override ``D_f`` in (0, 1).
    """

    def __init__(
        self,
        stand_height=0.31,
        twist=np.array([0.0, 0.15, 0.0]),
        step_height=0.04,
        period=1.0,
        gait_type="Trot",
        dt=0.001,
        stability_margin=0.02,
        stance_duty=None,
    ):

        self.stand_height = stand_height
        self.step_height = step_height
        self.T = period
        self.dt = dt
        self.n_cycles: int | None = None  # set by generate_full_gait()

        # Validate gait type
        if gait_type not in GAIT_LIBRARY:
            raise ValueError(
                f"Unknown gait type '{gait_type}'. Choose from: {list(GAIT_LIBRARY.keys())}"
            )
        self.gait_type = gait_type
        gait_def = GAIT_LIBRARY[gait_type]
        self.phase_offsets = gait_def["phase_offsets"]
        if stance_duty is None:
            self.stance_duty = gait_def["stance_duty"]
            self.custom_stance_duty = None
        else:
            if not 0.0 < stance_duty < 1.0:
                raise ValueError("stance_duty must be in the open interval (0, 1).")
            self.stance_duty = float(stance_duty)
            self.custom_stance_duty = self.stance_duty

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
        from legwheel.utils.solver import Solver as _Solver

        BETA_MAX = np.deg2rad(RobotParams.BETA_MAX_DEG)
        GAMMA_GUARD = np.deg2rad(RobotParams.GAMMA_GUARD_DEG)  # conservative for velocity guard

        # Use actual geometric values from the kinematics model
        R_arc = self.legs[0].solver.foot_radius  # 0.140 m (WHEEL_RADIUS_OUTER)
        R_link = self.legs[0].solver.R  # 0.100 m
        H_hip = stand_height + RobotParams.ABAD_AXIS_OFFSET
        H_O = H_hip - R_arc

        D_x_max = 2 * H_O * np.tan(BETA_MAX) + 2 * R_arc * BETA_MAX
        v_x_limit = D_x_max / (self.T * self.stance_duty)

        # One-sided lateral sweep: the contact rolls from the touchdown extreme γ_td
        # down to a small floor (~0), so peak tilt γ_td hits GAMMA_GUARD when
        # sin(γ_td) = D_y/H_hip (NOT D_y/(2·H_hip) as for a symmetric ±γ sweep). The
        # usable lateral travel per stance is therefore H_hip·sin(GAMMA_GUARD).
        D_y_max = H_hip * np.sin(GAMMA_GUARD)
        v_y_limit = D_y_max / (self.T * self.stance_duty)

        # Find maximum required scale down across all legs
        # Use relative tolerance so values exactly on the limit don't trigger scaling
        VEL_TOL = 1e-4  # 0.01% relative tolerance
        scale_x, scale_y = 1.0, 1.0
        for vel in raw_hip_velocities:
            if abs(vel[0]) > v_x_limit * (1.0 + VEL_TOL):
                scale_x = min(scale_x, v_x_limit / abs(vel[0]))
            if abs(vel[1]) > v_y_limit * (1.0 + VEL_TOL):
                scale_y = min(scale_y, v_y_limit / abs(vel[1]))

        global_scale = min(scale_x, scale_y)

        if global_scale < 1.0:
            print(
                f"  ⚠ Workspace guard: Scaling FULL body twist down to {global_scale*100:.1f}% "
                f"to prevent leg kinematic singularity."
            )
            self.v_com *= global_scale
            self.omega_z *= global_scale
            self.twist[0] = self.omega_z
            self.twist[1:3] = self.v_com

        # Recompute final, safe hip velocities
        self.hip_velocities = calc_hip_vels(self.v_com[0], self.v_com[1], self.omega_z)

        # --- Dynamic Global Step Height Scaling ---
        # Find maximum beta and gamma usage among the planned safe velocities.
        # Uses exact Secant solve for beta (not small-angle approximation).
        GAMMA_MAX_STEP = np.deg2rad(
            RobotParams.GAMMA_MAX_DEG
        )  # 15° geometric limit for step scaling
        max_beta_ratio = 0.0
        max_gamma_ratio = 0.0
        for vel in self.hip_velocities:
            d_x = abs(vel[0]) * self.T * self.stance_duty
            if d_x > 1e-6:
                # Exact beta solve via Secant method (replaces small-angle approx)
                def _eq(b):
                    return 2 * H_O * np.tan(b) + 2 * R_arc * b - d_x

                def _deq(b):
                    return 2 * H_O / (np.cos(b) ** 2) + 2 * R_arc

                _s = _Solver(method="Secant", tol=1e-6, max_iter=50, function=_eq, derivative=_deq)
                beta_exact = _s.solve(0.001, np.clip(d_x / (2 * H_O), 0.001, 0.69))
                max_beta_ratio = max(max_beta_ratio, beta_exact / BETA_MAX)

            d_y = abs(vel[1]) * self.T * self.stance_duty
            # One-sided sweep peaks at γ_td with sin(γ_td) ≈ d_y / H_hip.
            gamma_approx = np.arcsin(np.clip(d_y / H_hip, -1.0, 1.0))
            max_gamma_ratio = max(max_gamma_ratio, gamma_approx / GAMMA_MAX_STEP)

        global_usage = max(max_beta_ratio, max_gamma_ratio)

        # Deadband: no step scaling when workspace usage is below threshold
        threshold = RobotParams.STEP_USAGE_THRESHOLD
        if global_usage <= threshold:
            global_step_scale = 1.0
        else:
            effective_usage = (global_usage - threshold) / (1.0 - threshold)
            global_step_scale = max(
                1.0 - RobotParams.STEP_DECAY_COEFF * effective_usage, RobotParams.STEP_FLOOR
            )

        if global_step_scale < 1.0:
            print(
                f"  ⚠ Swing guard: Scaling liftoff/touchdown velocities to {global_step_scale*100:.1f}% "
                f"to reduce joint speed demands. step_height={step_height:.4f}m preserved."
            )

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
                step_scale=global_step_scale,
            )
            for i in range(4)
        ]

        # --- Walk gait CoM stability: pre-planned per-leg touchdown offset ---
        # Computes (x_bias, y_bias) for each leg so the CoM lies inside every
        # support triangle with the requested safety margin.  Only Walk has
        # triangular (3-leg) support where a static offset is sufficient;
        # other gaits rely on COMStabilityPlanner for dynamic correction.
        self.x_biases = np.zeros(4)
        self.y_biases = np.zeros(4)
        if gait_type == "Walk" and stability_margin > 0.0:
            print("  [WalkBias] Computing per-leg CoM stability offsets …")
            biases = self._compute_walk_bias(safety_margin=stability_margin)
            for i in range(4):
                self.planners[i].x_bias = float(biases[i, 0])
                self.planners[i].y_bias = float(biases[i, 1])
            self.x_biases = biases[:, 0]
            self.y_biases = biases[:, 1]
            kx_peak = float(np.max(np.abs(self.x_biases)))
            ky_peak = float(np.max(np.abs(self.y_biases)))
            print(
                f"  [WalkBias] Done ✓  peak |Kx|={kx_peak*100:.1f}cm, "
                f"peak |Ky|={ky_peak*100:.1f}cm"
            )

    def _compute_walk_bias(self, safety_margin: float = 0.02) -> np.ndarray:
        """
        Compute per-leg (x_bias, y_bias) touchdown offsets for Walk gait CoM stability.

        For each of the four swing phases, the leg that just touched down is the
        primary vertex controlling the support triangle shape.  This method finds
        the minimum-norm 2-D offset for that leg's touchdown position such that the
        projected CoM lies inside the triangle with the requested safety_margin.

        Strategy
        --------
        1. Generate a preliminary zero-bias trajectory to obtain actual foot
           positions via FK at each swing-start frame.
        2. At each swing transition compute the signed stability margin with
           ``_hull_signed_margin``.
        3. If the margin is below target, evaluate the numerical gradient of the
           margin w.r.t. the touchdown leg's (x, y) position and solve for the
           minimum-norm correction.

        Returns
        -------
        np.ndarray
            Shape (4, 2): ``[[x_bias_FL, y_bias_FL], ..., [x_bias_RL, y_bias_RL]]``
            in metres, body-frame.
        """
        from legwheel.planners.com_stability import _hull_signed_margin

        com_xy = np.array([RobotParams.COM_BIAS_X, RobotParams.COM_BIAS_Y])
        biases = np.zeros((4, 2))
        labels = ["FL", "FR", "RR", "RL"]

        # Step 1: preliminary trajectory (all biases zero)
        try:
            all_leg_trajs = [np.array(p.generate_trajectory()) for p in self.planners]
        except RuntimeError as e:
            print(
                f"  [WalkBias] Preliminary trajectory failed ({e}); skipping bias (reduce speed or period)"
            )
            return biases
        n_points = len(all_leg_trajs[0])
        n_stance = int(round(self.stance_duty * n_points))

        for swing_i in range(4):
            # Frame at which leg swing_i lifts off (stance → swing transition)
            shift_i = int(round(self.phase_offsets[swing_i] * n_points))
            f_lo = (n_stance - shift_i) % n_points

            stance_legs = [j for j in range(4) if j != swing_i]

            # Step 2: foot XY of the 3 stance legs at this frame (from zero-bias traj)
            foot_xy = np.zeros((3, 2))
            for k, j in enumerate(stance_legs):
                shift_j = int(round(self.phase_offsets[j] * n_points))
                idx_j = (f_lo + shift_j) % n_points
                q_j = all_leg_trajs[j][idx_j]
                foot_xy[k] = self.planners[j].kin.forward_kinematics(*q_j)[:2]

            margin, _ = _hull_signed_margin(com_xy, foot_xy)

            if margin >= safety_margin:
                continue  # already stable for this swing phase

            # Identify the just-touched-down leg (smallest τ among stance legs)
            min_tau, td_leg, td_k = 1.0, -1, -1
            for k, j in enumerate(stance_legs):
                shift_j = int(round(self.phase_offsets[j] * n_points))
                own_phase = (f_lo + shift_j) % n_points
                tau = own_phase / n_stance
                if tau < min_tau:
                    min_tau, td_leg, td_k = tau, j, k

            if td_leg == -1:
                continue

            # Step 3: numerical gradient of margin w.r.t. touchdown foot XY
            EPS = 1e-4
            fxy_px = foot_xy.copy()
            fxy_px[td_k, 0] += EPS
            m_px, _ = _hull_signed_margin(com_xy, fxy_px)

            fxy_py = foot_xy.copy()
            fxy_py[td_k, 1] += EPS
            m_py, _ = _hull_signed_margin(com_xy, fxy_py)

            grad = np.array([(m_px - margin) / EPS, (m_py - margin) / EPS])
            grad_norm_sq = float(np.dot(grad, grad))
            if grad_norm_sq < 1e-10:
                continue

            deficit = safety_margin - margin  # > 0: shortfall to overcome
            correction = deficit * grad / grad_norm_sq
            biases[td_leg] += correction

            print(
                f"  [WalkBias] {labels[swing_i]} swing: margin={margin*100:+.1f}cm  "
                f"→ {labels[td_leg]} TD bias "
                f"({correction[0]*100:+.1f}, {correction[1]*100:+.1f}) cm"
            )

        return biases

    def generate_full_gait(self, n_cycles=2):
        """
        Generates coordinated 3D commands for all 4 legs with phase offsets.

        Args:
            n_cycles (int): Number of gait cycles.
        Returns:
            np.ndarray: (N, 12) array of [theta, beta, gamma] * 4 legs.
        """
        self.n_cycles = n_cycles
        # 1. Generate base trajectories for each leg (one cycle)
        all_leg_trajs = [np.array(p.generate_trajectory()) for p in self.planners]
        n_points = len(all_leg_trajs[0])

        # 2. Apply phase offsets
        total_len = n_points * n_cycles
        cmds = np.zeros((total_len, 12))  # 4 legs * 3 joints

        for i in range(4):
            shift = int(self.phase_offsets[i] * n_points)
            indices = (np.arange(total_len) + shift) % n_points
            cmds[:, i * 3 : i * 3 + 3] = all_leg_trajs[i][indices]

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

        cycles_str = f"_C{self.n_cycles}" if self.n_cycles is not None else ""
        kx_peak = float(np.max(np.abs(self.x_biases)))
        ky_peak = float(np.max(np.abs(self.y_biases)))
        kbias_str = f"_Kx{kx_peak:.3f}_Ky{ky_peak:.3f}" if kx_peak > 1e-4 or ky_peak > 1e-4 else ""
        duty_str = f"_D{self.custom_stance_duty:.2f}" if self.custom_stance_duty is not None else ""
        return (
            f"{self.gait_type}{kbias_str}"
            f"_Vx{v_x_actual:.2f}_Vy{v_y_actual:.2f}_Wz{w_z_actual:.2f}"
            f"_H{self.stand_height:.2f}_S{step_actual:.3f}"
            f"_P{self.T:.1f}{duty_str}{cycles_str}_dt{self.dt:g}"
        )

    def print_summary(self):
        """Prints a summary of the gait configuration."""
        print(f"=== GaitGenerator3D Summary ===")
        print(f"  Gait Type:    {self.gait_type}")
        print(f"  Period:       {self.T:.2f} s")
        print(f"  Stance Duty:  {self.stance_duty:.2f}")
        print(
            f"  Body Twist:   ω_z={self.omega_z:.3f} rad/s, "
            f"v_x={self.v_com[0]:.3f} m/s, v_y={self.v_com[1]:.3f} m/s"
        )
        print(f"  Stand Height: {self.stand_height:.3f} m")
        print(f"  Step Height:  {self.step_height:.3f} m")
        print()
        labels = ["FL", "FR", "RR", "RL"]
        for i in range(4):
            v = self.hip_velocities[i]
            r = self.hip_positions[i]
            phi = self.phase_offsets[i]
            print(
                f"  {labels[i]}: hip=[{r[0]:+.3f}, {r[1]:+.3f}, {r[2]:+.3f}] m, "
                f"v_hip=[{v[0]:+.4f}, {v[1]:+.4f}, {v[2]:+.4f}] m/s, "
                f"phase={phi:.2f}"
            )

    def export_to_csv(self, base_name="gait_3d"):
        """Exports the 12-DOF gait commands to CSV."""
        if not hasattr(self, "CMDS"):
            self.generate_full_gait()

        import pandas as pd

        cols = []
        for l in ["FL", "FR", "RR", "RL"]:
            cols += [f"{l}_Theta", f"{l}_Beta", f"{l}_Gamma"]

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
        gait_type="Trot",
    )
    gait.print_summary()
    data = gait.generate_full_gait(n_cycles=2)
    print(f"\nGenerated 12-DOF Gait data: {data.shape}")
