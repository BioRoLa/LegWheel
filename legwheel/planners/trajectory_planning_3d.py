import numpy as np
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.models.leg_model import LegModel
from legwheel.utils.solver import Solver
from legwheel.utils.fitted_coefficient import inv_G_dist_poly
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
            stand_height (float): Target body height (m).
            step_length (float): Total horizontal stride (m).
            step_height (float): Swing clearance (m).
            period (float): Cycle time (s).
            dt (float): Time step (s).
            duty (float): Swing phase duty cycle.
            leg_index (int): Index of the leg (0-3).
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
        self.theta0 = 0.0
        self.beta0 = 0.0
        self.D = 0.0 # Forward hip movement per stride
        
        self._calculate_initial_pose()
        
        # Swing Planner (3D)
        self.swing_planner = swing.SwingLegPlanner(dt=dt, T_sw=self.T * self.duty, T_st=self.T * (1-self.duty))

    def _calculate_initial_pose(self):
        """Calculates theta0 and beta0 based on target stand height and step length."""
        # Use existing 2D solver logic via Solver utility
        func = lambda x: self.H * np.tan(x) + self.kin.solver["foot_radius"] * x - 3 * self.step_length / 8
        solver = Solver(
            method="Secant",
            tol=1e-6,
            max_iter=100,
            function=func,
            derivative=lambda x: self.H * (1 / np.cos(x))**2 + self.kin.solver["foot_radius"] - 3 * self.step_length / 8
        )
        self.beta0 = solver.solve(0, np.deg2rad(40))
        
        G_dist = self.H / np.cos(self.beta0) + self.kin.solver["R"]
        self.theta0 = inv_G_dist_poly(G_dist)
        
        OO_r_Dist = G_dist - self.kin.solver["R"]
        L = 2 * OO_r_Dist * np.sin(self.beta0)
        self.D = (L + self.kin.solver.foot_radius * 2 * self.beta0) / 3
        self.V = self.D / (self.T * (1 - self.duty)) # Body forward velocity

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
        self.cmd = [] # [theta, beta, gamma]
        
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
                derivative=lambda b: -self.H * np.cos(b) - self.kin.solver["foot_radius"]
            )
            beta = solver.solve(self.cmd[-1][1] if self.cmd else self.beta0)
            if abs(beta) > np.deg2rad(45): break
            
            theta = self.solve_theta(beta)
            self.cmd.append([theta, beta, 0.0]) # gamma=0 in stance
            
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
        v_lo = np.array([0, 0, self.V]) # Vertical lift? Need calibration.
        v_td = np.array([0, 0, -self.V/10])
        
        # Solve 3D Bezier Swing
        swing_profile = self.swing_planner.solveSwingTrajectory(p_lo, p_td, self.step_height, v_lo, v_td)
        
        swing_points_3d = [swing_profile.getFootendPoint(ti) 
                           for ti in np.linspace(0, 1, int(self.T * self.duty / self.dt))]
        
        # Inverse Kinematics to recover joint angles for swing points
        for p in swing_points_3d:
            q = self.kin.inverse_kinematics(p, guess_q=np.array(self.cmd[-1]))
            self.cmd.append(q.tolist())
            
        return self.cmd
