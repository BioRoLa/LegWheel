import numpy as np
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.models.leg_model import LegModel
from legwheel.utils.solver import Solver
from legwheel.utils.fitted_coefficient import inv_G_dist_poly
from legwheel.utils import numerical_jacobian, pseudo_inverse_dls, rolling_arc_length
from legwheel.utils.screw import Screw
from legwheel.bezier import swing


class TrajectoryPlanner3D:
    """
    3D Trajectory Planner for a single leg of the Corgi robot.
    Extends the 2D logic to support the Abduction/Adduction (gamma) DOF.
    """

    def __init__(self, stand_height=0.3, velocity=None, step_height=0.04,
                 period=1.0, dt=0.001, stance_duty=0.75, leg_index=0):
        """
        Initializes the 3D trajectory planner.

        Args:
            stand_height (float):   Target body height (m).         default: 0.3   m
            velocity (array-like):  Target body velocity [vx,vy,vz] (m/s). default: [0.15, 0, 0]
            step_height (float) :   Swing clearance (m).            default: 0.04  m
            period (float)      :   Cycle time (s).                 default: 1.0   s
            dt (float)          :   Time step (s).                  default: 0.001 s
            stance_duty (float) :   Stance phase duty cycle (D_f).  default: 0.75
            leg_index (int)     :   Index of the leg (0-3).         default: 0
        """
        self.stand_height = stand_height
        self.velocity = np.array(
            velocity if velocity is not None else [0.15, 0.0, 0.0])
        self.step_height = step_height
        self.T = period
        self.dt = dt
        self.stance_duty = stance_duty
        self.leg_index = leg_index

        # 3D Kinematics model
        self.kin = CorgiLegKinematics(leg_index)

        # Geometric constants for the rolling foot arc
        self.R_arc = self.kin.solver.foot_radius  # Rolling arc radius (0.1345m)
        self.R_link = self.kin.solver.R           # Distance from Arc Center to G (0.1m)

        # H_hip: Vertical distance from Hip to Ground
        self.H_hip = stand_height + self.kin.d_abad
        # H_O: Vertical distance from Hip to Arc Center O_r
        self.H_O = self.H_hip - self.R_arc

        self.theta0 = np.deg2rad(17)  # Initial guess, will be refined
        self.beta0 = 0.0
        self.gamma0 = 0.0  # Initial ABAD angle for lateral offset

        # Derived gait distances (from unified gait equation)
        self.D_stance = 0.0  # Hip travel during stance = v_x * T * D_f
        self.D_swing = 0.0   # Hip travel during swing  = v_x * T * (1 - D_f)

        self._calculate_initial_pose()

        # Swing Planner (3D)
        self.swing_planner = swing.SwingLegPlanner(
            dt=dt,
            T_sw=self.T * (1 - self.stance_duty),
            T_st=self.T * self.stance_duty)

    @property
    def step_length(self):
        """Total stride length (m), derived from velocity and period."""
        return np.abs(self.velocity[0]) * self.T

    def _calculate_initial_pose(self):
        """
        Calculates theta0 and beta0 using the unified gait equation:
            v_x · T · D_f = 2(H − R)·tan(β) + 2R·β

        Uses small-angle approximation for initial guess, then refines
        with the exact nonlinear equation via Secant method.
        """
        v_x = np.abs(self.velocity[0])
        target = v_x * self.T * self.stance_duty  # D_stance

        # Small-angle initial guess: β ≈ D_stance / (2H_O)
        beta_guess = target / \
            (2 * self.H_O) if self.H_O > 0 else 0.01
        beta_guess = np.clip(beta_guess, 0.001, np.deg2rad(40))

        # Exact solve: 2H_O·tan(β) + 2R_arc·β - D_stance = 0
        def func(b):
            return 2 * self.H_O * np.tan(b) + 2 * self.R_arc * b - target

        def dfunc(b):
            return 2 * self.H_O / (np.cos(b) ** 2) + 2 * self.R_arc

        solver = Solver(
            method="Secant",
            tol=1e-6,
            max_iter=100,
            function=func,
            derivative=dfunc
        )
        self.beta0 = solver.solve(0.001, beta_guess)

        G_dist = self.H_O / np.cos(self.beta0) + self.R_link
        self.theta0 = inv_G_dist_poly(G_dist)

        # Store derived gait distances
        self.D_stance = target
        self.D_swing = v_x * self.T * (1 - self.stance_duty)

        # --- Lateral (Y-axis) initial ABAD angle ---
        # Symmetric lateral motion: Δy = 2 * H_true * sin(γ₀)
        # where H_true is the full distance from hip to contact point
        v_y = np.abs(self.velocity[1])
        D_lateral = v_y * self.T * self.stance_duty  # Total lateral travel during stance
        H_true = self.H_hip  # Hip to ground (full length)
        if D_lateral > 0 and H_true > 0:
            sin_arg = np.clip(D_lateral / (2 * H_true), -1.0, 1.0)
            self.gamma0 = np.arcsin(sin_arg)
        else:
            self.gamma0 = 0.0

    def solve_theta(self, beta):
        """Helper to find theta for a given beta to maintain height."""
        G_dist = self.H_O / np.cos(beta) + self.R_link
        return inv_G_dist_poly(G_dist)

    def generate_trajectory(self, lateral_offset=0.0):
        """
        Generates the full gait cycle commands for the leg.

        Uses stance_rt_solver (Rolling Jacobian + DLS) for the stance phase
        and Bézier swing planner for the swing phase.

        Args:
            lateral_offset (float): Target lateral (gamma) displacement (m).
        Returns:
            list: List of [theta, beta, gamma] commands.
        """
        self.cmd = []  # [theta, beta, gamma]

        # 1. Stance Phase (Rolling via stance_rt_solver)
        stance_duration = self.T * self.stance_duty

        # Touchdown: foot starts "ahead" of the body in both X and Y.
        # For X: beta starts at -beta0 (foot forward), sweeps to +beta0 (foot backward)
        # For Y: gamma starts at +gamma0 (foot outward), sweeps to -gamma0 (foot inward)
        #         when vy > 0 (body moving in +Y), foot in body frame retracts in -Y.
        gamma_sign = 1.0 if self.velocity[1] >= 0 else -1.0
        q = np.array([self.theta0, -self.beta0, gamma_sign * self.gamma0])
        self.cmd.append(q.tolist())

        for t in np.arange(self.dt, stance_duration, self.dt):
            q = self.stance_rt_solver(v_hip=self.velocity, q=q)
            if abs(q[1]) > np.deg2rad(45):
                break
            self.cmd.append(q.tolist())

        # 2. Swing Phase (Bezier)
        # Get lift-off and touchdown points in Body Frame
        # Lift-off: end of stance
        last_q = self.cmd[-1]
        p_lo = self.kin.forward_kinematics(last_q[0], last_q[1], last_q[2])

        # Touchdown: start of next stance (symmetric pose)
        p_td = self.kin.forward_kinematics(self.theta0, -self.beta0, 0.0)

        # Add lateral displacement if requested (mapping Y offset to p_td)
        p_td[1] += lateral_offset

        # Define velocities (rough estimate for smooth blending)
        v_mag = np.linalg.norm(self.velocity)
        v_lo = np.array([0, 0, v_mag])      # Vertical lift
        v_td = np.array([0, 0, -v_mag / 10])  # Soft landing

        # Solve 3D Bezier Swing
        swing_duration = self.T * (1 - self.stance_duty)
        swing_profile = self.swing_planner.solveSwingTrajectory(
            p_lo, p_td, self.step_height, v_lo, v_td)

        swing_points_3d = [swing_profile.getFootendPoint(ti)
                           for ti in np.linspace(0, 1, int(swing_duration / self.dt))]

        # Inverse Kinematics to recover joint angles for swing points
        for p in swing_points_3d:
            q = self.kin.inverse_kinematics(p, guess_q=np.array(self.cmd[-1]))
            self.cmd.append(q.tolist())

        return self.cmd

    def stance_rt_solver(self, v_hip=np.zeros(3), q=None, ground_slope=0.0, damping=1e-2):
        """
        Real-time stance phase solver using Rolling Jacobian velocity planning (§5.3.2).

        Instead of iteratively solving IK, this method computes the numerical
        Rolling Jacobian at the current state-dependent contact point α(q),
        resolves desired joint velocities via DLS pseudo-inverse, and integrates
        forward by one time step.

        Rolling Jacobian (Trajectory.md §5.3.2):
            J(q) ≈ [FK(q + δq, α(q + δq)) - FK(q, α(q))] / δq
            q̇_d  = J*(q) · v_target
            q_next = q + q̇_d · dt

        Args:
            v_hip (np.ndarray): Desired hip velocity vector [vx, vy, vz] (m/s).
            q (np.ndarray): Current joint angles [theta, beta, gamma] (rad).
            ground_slope (float): Local ground slope angle δ (radians).
            damping (float): DLS damping factor λ for singularity robustness.

        Returns:
            np.ndarray: Updated joint angles [theta, beta, gamma].
        """
        q = np.array(q, dtype=float) if q is not None else np.array(
            [self.theta0, self.beta0, 0.0])

        # --- Rolling Jacobian ---
        # FK wrapper with state-dependent contact angle α(q) = δ - β
        def rolling_fk(q_eval):
            contact = self.kin.foot_rim_contact_fk(
                *q_eval, ground_slope=ground_slope)
            return self.kin.forward_kinematics(
                *q_eval, alpha=contact[0], w=contact[1])

        # Numerical Jacobian evaluated at the continuously shifting contact
        J = numerical_jacobian(rolling_fk, q, diff=1e-5)

        # --- DLS Velocity Resolution ---
        # Theoretical Insight:
        # The rolling_fk tracks the GEOMETRIC contact point (always directly under the wheel center).
        # In Body Frame, the geometric X coordinate is essentially H_O * tan(beta).
        # However, the physical travel of the Hip on the ground is the sum of geometric sliding 
        # AND the arc length rolled: dx_ground = d(H_O*tan(beta)) + R_arc*d(beta).
        # So true speed: v_hip = beta_dot * (H_O*sec²(beta) + R_arc).
        # The Jacobian J evaluated on rolling_fk only gives d(geom_X)/dbeta = H_O*sec²(beta).
        # We must scale the Cartesian v_target so the solver produces the correct beta_dot.
        #
        # IMPORTANT: This rolling correction applies ONLY to the X-axis (sagittal rolling).
        # The Y-axis (lateral) assumes NO rolling (soft tire contact), so v_target_y = -v_hip_y directly.
        
        # Scaling factor for X only: v_geom = v_hip * (H_O*sec²(beta)) / (H_O*sec²(beta) + R_arc)
        sec2_beta = 1.0 / (np.cos(q[1]) ** 2)
        geom_grad = self.H_O * sec2_beta
        velocity_scale_x = geom_grad / (geom_grad + self.R_arc)
        
        # v_target: rolling-corrected for X, direct for Y and Z
        v_target = np.array([
            -v_hip[0] * velocity_scale_x,
            -v_hip[1],
            -v_hip[2]
        ])
        J_star = pseudo_inverse_dls(J, damping_factor=damping)
        q_dot = J_star @ v_target

        # --- Forward Integration ---
        q_next = q + q_dot * self.dt

        return q_next

    def compute_leg_twist(self, q, q_dot):
        """
        Synthesizes the leg's spatial twist from joint velocities (§5.3.2 Twist Synthesis).

        Computes:  [V]_leg = [S_γ]·γ̇ + [S_β]·β̇ + [S_θ]·θ̇

        Each joint screw S_i is defined by its rotation axis passing through
        the hip origin. The resulting 4×4 se(3) twist matrix encapsulates the
        leg's overall spatial velocity, used as the boundary condition for
        swing phase Bézier planning (§5.4).

        Args:
            q (np.ndarray): Current joint angles [theta, beta, gamma] (rad).
            q_dot (np.ndarray): Current joint velocities [θ̇, β̇, γ̇] (rad/s).

        Returns:
            np.ndarray: 4×4 se(3) twist matrix [V]_leg.
            Screw: The resultant twist as a Screw object.
        """
        q = np.array(q, dtype=float)
        q_dot = np.array(q_dot, dtype=float)

        # Joint screw axes defined at the hip origin in Body Frame {B}
        # θ (leg extension): rotation about the Module Z-axis (longitudinal roll axis)
        # β (leg swing):     rotation about the Module Y-axis (sagittal swing)
        # γ (ABAD):          rotation about the Module X-axis (hip roll)
        p_hip = self.kin.p_Mi_in_B  # Hip origin in {B}

        # Get the rotation from Module to Body to express screw axes in {B}
        _, R_M_to_B = self.kin._get_transformation_matrices(
            gamma=q[2], type="vec")

        # Module frame axes mapped to Body frame
        axis_theta = R_M_to_B @ np.array([0, 0, 1])  # Z_M → extension axis
        axis_beta = R_M_to_B @ np.array([0, 1, 0])  # Y_M → swing axis
        axis_gamma = R_M_to_B @ np.array([1, 0, 0])  # X_M → ABAD roll axis

        # Construct unit screws (pure rotation, pitch h=0)
        S_theta = Screw.from_axis(p_hip, axis_theta, h=0.0)
        S_beta = Screw.from_axis(p_hip, axis_beta,  h=0.0)
        S_gamma = Screw.from_axis(p_hip, axis_gamma, h=0.0)

        # Spatial twist superposition: [V]_leg = Σ [S_i] · q̇_i
        V_leg = S_theta * q_dot[0] + S_beta * q_dot[1] + S_gamma * q_dot[2]
        V_matrix = V_leg.to_matrix()  # 4×4 se(3)

        return V_matrix, V_leg
