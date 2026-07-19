import warnings
from typing import Callable

import numpy as np
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.models.leg_model import LegModel
from legwheel.utils.solver import Solver
from legwheel.utils.fitted_coefficient import inv_G_dist_poly
from legwheel.utils import numerical_jacobian, pseudo_inverse_dls, rolling_arc_length
from legwheel.utils.screw import Screw
from legwheel.bezier import swing
from legwheel.config import RobotParams


class TrajectoryPlanner3D:
    """
    3D Trajectory Planner for a single leg of the Corgi robot.
    Extends the 2D logic to support the Abduction/Adduction (gamma) DOF.
    """

    def __init__(
        self,
        stand_height=0.3,
        velocity=None,
        step_height=0.04,
        period=1.0,
        dt=0.001,
        stance_duty=0.75,
        leg_index=0,
        step_scale=None,
        x_bias=0.0,
        y_bias=0.0,
        hip_velocity_fn: Callable[[float], np.ndarray] | None = None,
    ):
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
            step_scale (float)  :   Swing velocity scaling factor (1.0 = full speed).
            hip_velocity_fn (Callable[[float], np.ndarray] | None):
                Optional additive stance hip-velocity term as a function of
                local stance time (s since touchdown, i.e. the loop time ``t``
                below). Added on top of ``velocity`` before each
                ``stance_rt_solver`` call, so it passes through the same
                rolling-arc scaling (sagittal-only) as the base velocity. Used
                by the Bound/Pace attitude-oscillation compensation (see
                ``legwheel/planners/attitude_oscillation.py`` and CH4 Theory
                doc "Attitude Oscillation Compensation for Line-Support
                Gaits"); ``None`` (default) reproduces prior behavior exactly.
        """
        self.input_step_scale = step_scale
        self.x_bias = float(x_bias)
        self.y_bias = float(y_bias)
        self.hip_velocity_fn = hip_velocity_fn
        self.stand_height = stand_height
        self.velocity = np.array(velocity if velocity is not None else [0.15, 0.0, 0.0])
        self.step_height = step_height
        self.T = period
        self.dt = dt
        self.stance_duty = stance_duty
        self.leg_index = leg_index

        # 3D Kinematics model
        self.kin = CorgiLegKinematics(leg_index)

        # Geometric constants for the rolling foot arc
        # Rolling arc radius (0.140m = WHEEL_RADIUS_OUTER)
        self.R_arc = self.kin.solver.foot_radius
        # Distance from Arc Center to G (0.1m)
        self.R_link = self.kin.solver.R

        # H_hip: Vertical distance from Hip to Ground
        self.H_hip = stand_height + self.kin.d_abad
        # H_O: Vertical distance from Hip to Arc Center O_r
        self.H_O = self.H_hip - self.R_arc

        self.theta0 = np.deg2rad(17)  # Initial guess, will be refined
        self.beta0 = 0.0
        self.gamma0 = 0.0  # Initial ABAD angle for lateral offset

        # Derived gait distances (from unified gait equation)
        self.D_stance = 0.0  # Hip travel during stance = v_x * T * D_f
        self.D_swing = 0.0  # Hip travel during swing  = v_x * T * (1 - D_f)

        self._calculate_initial_pose()

        # Swing Planner (3D)
        self.swing_planner = swing.SwingLegPlanner(
            dt=dt, T_sw=self.T * (1 - self.stance_duty), T_st=self.T * self.stance_duty
        )

    @property
    def step_length(self):
        """Total stride length (m), derived from velocity and period."""
        return np.abs(self.velocity[0]) * self.T

    def _calculate_initial_pose(self):
        """
        Calculates theta0, beta0, and gamma0 for the touchdown configuration.

        Order of operations matters: gamma0/gamma_td are solved first because
        lateral tilt changes the effective sagittal drop height (H_O_eff), which
        in turn affects beta0 and theta0.

        Lateral kinematics (one-sided sweep):
            Δy = H·(sin γ_td − sin γ_floor)  →  γ_td, γ0 = arcsin(Δy / 2H)

        Corrected sagittal height (wheel edge contact when gamma ≠ 0):
            d_eff    = gamma_sign·d_wheel − half_w
            H_O_eff  = (H_hip + sin(γ₀)·d_eff) / cos(γ₀) − R_arc

        Sagittal kinematics with corrected height:
            v_x·T·D_f = 2·H_O_eff·tan(β₀) + 2·R_arc·β₀
        """
        v_x = np.abs(self.velocity[0])
        target = v_x * self.T * self.stance_duty  # D_stance

        # --- Step 1: Lateral ABAD angles (must come first) ---
        # ONE-SIDED sweep: touchdown at γ_td, liftoff near γ_floor.
        # γ0 (symmetric half-amplitude) is kept for workspace ratio checks.
        v_y = np.abs(self.velocity[1])
        D_lateral = v_y * self.T * self.stance_duty
        H_true = self.H_hip
        self.gamma_floor = np.deg2rad(RobotParams.GAMMA_FLOOR_DEG)
        if D_lateral > 0 and H_true > 0:
            self.gamma0 = np.arcsin(np.clip(D_lateral / (2 * H_true), -1.0, 1.0))
            sin_td = np.clip(np.sin(self.gamma_floor) + D_lateral / H_true, -1.0, 1.0)
            self.gamma_td = np.arcsin(sin_td)
        else:
            self.gamma0 = 0.0
            self.gamma_td = 0.0

        # --- Step 2: Corrected sagittal height for lateral tilt ---
        # When gamma0 != 0 the wheel contacts at its edge, not the center.
        # Effective lateral offset at touchdown:
        #   d_eff = gamma_sign * d_wheel − half_w
        gamma_sign = 1.0 if self.velocity[1] >= 0 else -1.0
        d_w = self.kin.d_wheel
        half_w = self.kin.wheel_thickness / 2.0
        d_eff = gamma_sign * d_w - half_w
        if abs(self.gamma0) > 1e-6:
            H_O_eff = (self.H_hip + np.sin(self.gamma0) * d_eff) / np.cos(self.gamma0) - self.R_arc
        else:
            H_O_eff = self.H_O

        # --- Step 3: Sagittal beta0/theta0 using corrected height ---
        beta_guess = target / (2 * H_O_eff) if H_O_eff > 0 else 0.01
        beta_guess = np.clip(beta_guess, 0.001, np.deg2rad(40))

        def func(b):
            return 2 * H_O_eff * np.tan(b) + 2 * self.R_arc * b - target

        def dfunc(b):
            return 2 * H_O_eff / (np.cos(b) ** 2) + 2 * self.R_arc

        solver = Solver(method="Secant", tol=1e-6, max_iter=100, function=func, derivative=dfunc)
        self.beta0 = solver.solve(0.001, beta_guess)

        # Geometric constraint constants (for step-height scaling)
        BETA_MAX = np.deg2rad(40)  # From α geometric limit ±40°
        GAMMA_MAX = np.deg2rad(15)  # Safe lateral sweep limit

        G_dist = H_O_eff / np.cos(self.beta0) + self.R_link
        self.theta0 = inv_G_dist_poly(G_dist)

        # Store derived gait distances
        self.D_stance = target
        self.D_swing = np.abs(self.velocity[0]) * self.T * (1 - self.stance_duty)

        # --- Swing velocity scaling ---
        # Instead of reducing step_height (which clips clearance), we scale
        # liftoff/touchdown velocities to produce gentler swing transitions.
        # step_height is preserved at the commanded value.
        if self.input_step_scale is not None:
            self.swing_velocity_scale = self.input_step_scale
        else:
            beta_ratio = abs(self.beta0) / BETA_MAX
            # One-sided sweep peaks at γ_td (≈2·γ0), so workspace usage is measured
            # against the touchdown extreme, not the symmetric half-amplitude.
            gamma_ratio = abs(self.gamma_td) / GAMMA_MAX if GAMMA_MAX > 0 else 0.0
            usage = max(beta_ratio, gamma_ratio)
            threshold = RobotParams.STEP_USAGE_THRESHOLD
            if usage <= threshold:
                self.swing_velocity_scale = 1.0
            else:
                eff_usage = (usage - threshold) / (1.0 - threshold)
                self.swing_velocity_scale = max(
                    1.0 - RobotParams.STEP_DECAY_COEFF * eff_usage, RobotParams.STEP_FLOOR
                )
        # NOTE: self.step_height is NOT modified

    def solve_theta(self, beta):
        """Helper to find theta for a given beta to maintain height."""
        G_dist = self.H_O / np.cos(beta) + self.R_link
        return inv_G_dist_poly(G_dist)

    def _level_touchdown_q(self):
        """Touchdown joint angles via IK to a LEVEL, motion-leading foot target.

        The foot is placed at the nominal stance height (constant Z, so no body
        roll) with a lateral lead of one full stride D_y in the body's direction of
        travel; the stance solver then sweeps the contact back -Y while holding Z,
        landing near upright (gamma≈0) at liftoff (one-sided, no mid-stance edge flip).

        Why IK instead of open-loop [theta0, -beta0, ±gamma]: with a prescribed gamma
        the foot's Z depends on the per-leg ABAD tilt, so mirrored left/right gammas
        touched down at DIFFERENT heights (≈21 mm at vy=0.05). stance_rt_solver holds
        vz=0, so that height offset was locked in for the whole stance and the body
        had to roll (~5°) to keep all feet on the ground — which then prevented the
        feet from reaching their targets and induced phase lag. Specifying the level
        foot POSITION and solving IK keeps every foot at -stand_height regardless of
        the tilt it ends up using.
        """
        alpha0, _ = self.kin.foot_rim_contact_fk(self.theta0, -self.beta0, 0.0)
        nom = self.kin.forward_kinematics(self.theta0, -self.beta0, 0.0, alpha=alpha0, w=0.0)
        D_lat = np.abs(self.velocity[1]) * self.T * self.stance_duty
        lead_y = np.sign(self.velocity[1]) * D_lat
        target = np.array([nom[0] + self.x_bias, nom[1] + lead_y + self.y_bias, nom[2]])
        return self.kin.inverse_kinematics(
            target,
            guess_q=np.array([self.theta0, -self.beta0, 0.0]),
            rim_point=(alpha0, 0.0),
        )

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

        # Touchdown: foot leads the body in its direction of travel and is placed at
        # a LEVEL height (see _level_touchdown_q). The stance solver then sweeps it
        # back holding Z, so the body stays level the whole stance.
        q = self._level_touchdown_q()
        self.cmd.append(q.tolist())

        for t in np.arange(self.dt, stance_duration, self.dt):
            v_hip_t = (
                self.velocity
                if self.hip_velocity_fn is None
                else self.velocity + self.hip_velocity_fn(t)
            )
            q = self.stance_rt_solver(v_hip=v_hip_t, q=q)
            if abs(q[1]) > np.deg2rad(45):
                break
            self.cmd.append(q.tolist())

        # 2. Swing Phase (Bezier) — Twist-Mapped Material Point Tracking
        #
        # Instead of interpolating alpha across the swing, we track a SINGLE
        # material point on the rim (alpha_td) throughout the entire swing phase.
        # This ensures physical consistency: we are planning the trajectory of
        # the exact rubber point that will touch the ground at touchdown.
        #
        # Steps:
        #   1. Find alpha_td (the contact angle at the next touchdown pose)
        #   2. Compute p_lo_virtual: where alpha_td is in space at liftoff config
        #   3. Compute v_lo_virtual: velocity of alpha_td at liftoff via Jacobian twist
        #   4. Plan Bezier from p_lo_virtual → p_td using v_lo_virtual and v_td
        #   5. IK tracks all swing points at fixed alpha_td (no interpolation)

        # --- Step 1: Touchdown target ---
        q_td = self._level_touchdown_q()
        alpha_td, _ = self.kin.foot_rim_contact_fk(*q_td)
        p_td = self.kin.forward_kinematics(*q_td, alpha=alpha_td, w=0.0)

        # --- Step 2: Virtual liftoff point (alpha_td evaluated at liftoff config) ---
        last_q = np.array(self.cmd[-1])
        p_lo_virtual = self.kin.forward_kinematics(*last_q, alpha=alpha_td, w=0.0)

        # --- Step 3: Liftoff velocity via Jacobian twist mapping ---
        # The leg's joint velocity at end of stance:
        #   q_dot_lo ≈ (last_q - second_last_q) / dt
        if len(self.cmd) >= 2:
            second_last_q = np.array(self.cmd[-2])
            q_dot_lo = (last_q - second_last_q) / self.dt
        else:
            q_dot_lo = np.zeros(3)

        # Jacobian of FK at (last_q, alpha_td, w=0) w.r.t. joint angles
        def fk_at_alpha_td(q_eval):
            return self.kin.forward_kinematics(*q_eval, alpha=alpha_td, w=0.0)

        J_lo = numerical_jacobian(fk_at_alpha_td, last_q, diff=1e-5)
        v_lo_virtual = J_lo @ q_dot_lo  # 3D velocity of the material point at liftoff

        # --- Acceleration-budget model for liftoff/touchdown velocities ---
        # SWING_ACCEL_MAX (m/s²) is a unified Cartesian acceleration budget that replaces
        # the previous kinematic (2h/T_sw) liftoff estimate and the swing_delta/T_sw
        # touchdown estimate. Physical rationale:
        #
        # Liftoff vertical: setting v_lo_z = 0 means the Bézier starts with zero
        # vertical tangent (dH1 initial guess = 0). Height clearance is guaranteed
        # by the Bézier control point structure (c2.y = step_height regardless of dH1).
        # This eliminates the unachievable v_z_kinematic target (0.32 m/s for Walk)
        # that previously consumed optimizer budget without being matchable by dL1/dL2.
        #
        # Touchdown vertical: v_td_z = 0 → dH2 initial guess = 0 → foot arrives with
        # zero vertical velocity → zero vertical impulse at touchdown → no body bounce.
        #
        # Feasibility: SWING_ACCEL_MAX ≥ 8*h/T_sw² (from parabolic height clearance).
        # At SWING_ACCEL_MAX = 10 m/s²: Walk a_min = 5.12 (2× margin), Trot = 8.0 (1.25×).
        # Peak joint acc ≈ SWING_ACCEL_MAX / J_x ≈ 526 rad/s² (vs 5000+ previously).
        T_sw = self.T * (1.0 - self.stance_duty)
        a_max = RobotParams.SWING_ACCEL_MAX
        a_min_feasible = 8.0 * self.step_height / T_sw**2
        if a_max < a_min_feasible * 0.95:
            warnings.warn(
                f"SWING_ACCEL_MAX={a_max:.1f} < a_min={a_min_feasible:.2f} m/s² "
                f"(h={self.step_height:.3f}, T_sw={T_sw:.3f}). Step height clearance may be insufficient.",
                RuntimeWarning,
                stacklevel=2,
            )

        # Liftoff: zero vertical target (Bézier handles height via shape; see above).
        # Horizontal: keep Jacobian-derived velocity to maintain stance→swing continuity.
        v_lo_virtual[2] = 0.0

        # --- Step 4: Touchdown velocity ---
        # Horizontal: match body stance velocity to minimize foot-ground slip at landing.
        # Vertical: zero → no vertical impact impulse → eliminates body bounce.
        v_td_xy_body = self.velocity[:2].copy()
        v_td_h_max = RobotParams.TOUCHDOWN_VEL_H_MAX
        v_td_h_norm = np.linalg.norm(v_td_xy_body)
        if v_td_h_norm > v_td_h_max:
            v_td_xy_body *= v_td_h_max / v_td_h_norm
        v_td = np.array([v_td_xy_body[0], v_td_xy_body[1], 0.0])
        self._last_swing_boundary_velocities_B = (v_lo_virtual.copy(), v_td.copy())

        # --- Apply swing velocity scaling ---
        # Scale liftoff & touchdown velocities to reduce joint speed demands
        # while preserving full step_height clearance.
        svs = self.swing_velocity_scale
        if svs < 1.0:
            v_lo_virtual *= svs
            v_td *= svs

        # Convert from Body Frame [X, Y, Z] to Swing Frame [Forward, Up, Lateral] → [x, z, y]
        p_lo_swing = np.array([p_lo_virtual[0], p_lo_virtual[2], p_lo_virtual[1]])
        p_td_swing = np.array([p_td[0], p_td[2], p_td[1]])
        v_lo_swing = np.array([v_lo_virtual[0], v_lo_virtual[2], v_lo_virtual[1]])
        v_td_swing = np.array([v_td[0], v_td[2], v_td[1]])

        # --- Solve 3D Bezier Swing ---
        swing_duration = self.T * (1 - self.stance_duty)
        N_steps = int(swing_duration / self.dt)
        swing_profile = self.swing_planner.solveSwingTrajectory(
            p_lo_swing, p_td_swing, self.step_height, v_lo_swing, v_td_swing
        )

        # --- Step 5: Generate target points and IK (all at fixed alpha_td) ---
        swing_points_3d = []
        for ti in np.linspace(0, 1, N_steps):
            pt_swing = swing_profile.getFootendPoint(ti)
            pt_leg = np.array([pt_swing[0], pt_swing[2], pt_swing[1]])
            swing_points_3d.append((pt_leg, alpha_td))

        self._last_swing_target_path = swing_points_3d

        # Inverse Kinematics: track the material point alpha_td throughout
        for p, alpha_t in swing_points_3d:
            q = self.kin.inverse_kinematics(
                p, guess_q=np.array(self.cmd[-1]), rim_point=(alpha_t, 0.0)
            )
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
        q = np.array(q, dtype=float) if q is not None else np.array([self.theta0, self.beta0, 0.0])

        # --- Rolling Jacobian ---
        # FK wrapper with state-dependent contact angle α(q) = δ - β.
        #
        # The Jacobian must track the rolling contact on the wheel CENTER plane
        # (w = 0). The lateral edge term w from foot_rim_contact_fk is the center
        # of pressure across the flat tread; it shifts with the wheel tilt (γ) but
        # that shift is NOT the foot sliding sideways. Feeding w(γ) into the
        # velocity Jacobian injects a large spurious d(w)/dγ term that corrupts the
        # lateral velocity mapping (observed: lateral tracking ~4x too fast) and,
        # because of the ±half_w edge flip at γ = 0, a yaw-inducing impulse. Use
        # w = 0 so the lateral velocity comes purely from the pendulum (γ) motion.
        def rolling_fk(q_eval):
            contact = self.kin.foot_rim_contact_fk(*q_eval, ground_slope=ground_slope)
            return self.kin.forward_kinematics(*q_eval, alpha=contact[0], w=0.0)

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
        v_target = np.array([-v_hip[0] * velocity_scale_x, -v_hip[1], -v_hip[2]])
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
        _, R_M_to_B = self.kin._get_transformation_matrices(gamma=q[2], type="vec")

        # Module frame axes mapped to Body frame
        axis_theta = R_M_to_B @ np.array([0, 0, 1])  # Z_M → extension axis
        axis_beta = R_M_to_B @ np.array([0, 1, 0])  # Y_M → swing axis
        axis_gamma = R_M_to_B @ np.array([1, 0, 0])  # X_M → ABAD roll axis

        # Construct unit screws (pure rotation, pitch h=0)
        S_theta = Screw.from_axis(p_hip, axis_theta, h=0.0)
        S_beta = Screw.from_axis(p_hip, axis_beta, h=0.0)
        S_gamma = Screw.from_axis(p_hip, axis_gamma, h=0.0)

        # Spatial twist superposition: [V]_leg = Σ [S_i] · q̇_i
        V_leg = S_theta * q_dot[0] + S_beta * q_dot[1] + S_gamma * q_dot[2]
        V_matrix = V_leg.to_matrix()  # 4×4 se(3)

        return V_matrix, V_leg
