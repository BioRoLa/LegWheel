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

    def __init__(self, stand_height=0.3, step_length=0.4, step_height=0.04,
                 period=1.0, dt=0.001, duty=0.25, leg_index=0):
        """
        Initializes the 3D trajectory planner.

        Args:
            stand_height (float):   Target body height (m).         default: 0.3   m
            step_length (float) :   Total horizontal stride (m).    default: 0.4   m
            step_height (float) :   Swing clearance (m).            default: 0.04  m
            period (float)      :   Cycle time (s).                 default: 1.0   s
            dt (float)          :   Time step (s).                  default: 0.001 s
            duty (float)        :   Swing phase duty cycle.         default: 0.25
            leg_index (int)     :   Index of the leg (0-3).         default: 0
        """
        self.stand_height = stand_height
        self.step_length = step_length
        self.step_height = step_height
        self.T = period
        self.dt = dt
        self.duty = duty
        self.leg_index = leg_index

        # 3D Kinematics model
        self.kin = CorgiLegKinematics(leg_index)

        # Internal params derived from 2D logic for stance phase
        self.H = stand_height - self.kin.solver["foot_radius"]
        self.theta0 = np.deg2rad(17)  # Initial guess, will be refined
        self.beta0 = 0.0
        self.D = 0.0  # Forward hip movement per stride

        self._calculate_initial_pose()

        # Swing Planner (3D)
        self.swing_planner = swing.SwingLegPlanner(
            dt=dt, T_sw=self.T * self.duty, T_st=self.T * (1-self.duty))

    def _calculate_initial_pose(self):
        """Calculates theta0 and beta0 based on target stand height and step length."""
        # Use existing 2D solver logic via Solver utility
        def func(x): return self.H * np.tan(x) + \
            self.kin.solver["foot_radius"] * x - 3 * self.step_length / 8
        solver = Solver(
            method="Secant",
            tol=1e-6,
            max_iter=100,
            function=func,
            derivative=lambda x: self.H *
            (1 / np.cos(x))**2 +
            self.kin.solver["foot_radius"] - 3 * self.step_length / 8
        )
        self.beta0 = solver.solve(0, np.deg2rad(40))

        G_dist = self.H / np.cos(self.beta0) + self.kin.solver["R"]
        self.theta0 = inv_G_dist_poly(G_dist)

        OO_r_Dist = G_dist - self.kin.solver["R"]
        L = 2 * OO_r_Dist * np.sin(self.beta0)
        self.D = (L + self.kin.solver.foot_radius * 2 * self.beta0) / 3
        self.V = self.D / (self.T * (1 - self.duty))  # Body forward velocity

    def solve_theta(self, beta):
        """Helper to find theta for a given beta to maintain height."""
        G_dist = self.H / np.cos(beta) + self.kin.solver["R"]
        return inv_G_dist_poly(G_dist)

    def generate_trajectory(self, lateral_offset=0.0):
        """
        Generates the full gait cycle commands for the leg.

        Args:
            lateral_offset (float): Target lateral (gamma) displacement (m).
        Returns:
            list: List of [theta, beta, gamma] commands.
        """
        self.cmd = []  # [theta, beta, gamma]

        # 1. Stance Phase (Rolling)
        # We assume gamma=0 during pure forward stance for now
        stance_duration = self.T * (1 - self.duty)
        for t in np.arange(0, stance_duration, self.dt):
            # Solve for beta to match forward velocity V
            solver = Solver(
                method="Newton",
                tol=1e-9,
                max_iter=100,
                function=lambda b: self.H * (np.sin(self.beta0) - np.sin(b)) +
                self.kin.solver["foot_radius"] * (self.beta0 - b) - self.V * t,
                derivative=lambda b: -self.H *
                np.cos(b) - self.kin.solver["foot_radius"]
            )
            beta = solver.solve(self.cmd[-1][1] if self.cmd else self.beta0)
            if abs(beta) > np.deg2rad(45):
                break

            theta = self.solve_theta(beta)
            self.cmd.append([theta, beta, 0.0])  # gamma=0 in stance

        # 2. Swing Phase (Bezier)
        # Get lift-off and touchdown points in Body Frame
        # Lift-off: end of stance
        last_q = self.cmd[-1]
        p_lo = self.kin.forward_kinematics(last_q[0], last_q[1], last_q[2])

        # Touchdown: start of next stance (symmetric pose)
        p_td = self.kin.forward_kinematics(self.theta0, self.beta0, 0.0)

        # Add lateral displacement if requested (mapping Y offset to p_td)
        p_td[1] += lateral_offset

        # Define velocities (rough estimate for smooth blending)
        v_lo = np.array([0, 0, self.V])  # Vertical lift? Need calibration.
        v_td = np.array([0, 0, -self.V/10])

        # Solve 3D Bezier Swing
        swing_profile = self.swing_planner.solveSwingTrajectory(
            p_lo, p_td, self.step_height, v_lo, v_td)

        swing_points_3d = [swing_profile.getFootendPoint(ti)
                           for ti in np.linspace(0, 1, int(self.T * self.duty / self.dt))]

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
        # q̇_d = J^T (J J^T + λ² I)^-1 · v_target
        # v_target is the negative of the hip velocity (foot moves opposite to body)
        v_target = -v_hip
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
