import numpy as np
from legwheel.config import RobotParams
from legwheel.models.corgi_robot import CorgiRobot
from legwheel.models.collision_model import CorgiCollisionModel
from scipy.optimize import minimize_scalar

class ActiveBankingDynamics:
    """
    3D Dynamics model for Active Banking cornering.
    Evaluates the induced body roll caused by wheel edge contacts when ABAD is engaged,
    and calculates the resulting cornering stability margins.
    """
    def __init__(self, robot=None):
        self.robot = robot if robot is not None else CorgiRobot()
        self.collision = CorgiCollisionModel(self.robot)
        
        self.mass = RobotParams.MASS if hasattr(RobotParams, 'MASS') else 15.0
        self.g = 9.81
        
        # COM bias in Body Frame
        self.com_bias_x = getattr(RobotParams, 'COM_BIAS_X', 0.0)
        self.com_bias_y = getattr(RobotParams, 'COM_BIAS_Y', 0.0)
        self.com_bias_B = np.array([self.com_bias_x, self.com_bias_y, 0.0])

    def calculate_induced_roll(self, theta_deg, beta_deg, gamma_bank_deg):
        """
        Calculates the physical body roll induced by banking the ABAD joints.
        Since the wheel has thickness, banking shifts the contact point to the edge,
        changing the effective leg height differently for the left and right sides.
        
        Args:
            theta_deg: Leg extension angle in degrees.
            beta_deg: Leg swing angle in degrees.
            gamma_bank_deg: Banking angle amplitude. 
                            If > 0 (Left Turn), Left legs abduct (+gamma), Right legs adduct (-gamma).
                            
        Returns:
            tuple: (roll_deg, min_z_clearance, wheel_contact_pts_W)
        """
        theta = np.deg2rad(theta_deg)
        beta = np.deg2rad(beta_deg)
        gamma_bank = np.deg2rad(gamma_bank_deg)
        
        # Left Turn Banking convention:
        # Left legs (FL, RL): +gamma (Abduction, retracts leg up)
        # Right legs (FR, RR): -gamma (Adduction, extends leg down)
        q_FL = [theta, beta, gamma_bank]
        q_FR = [theta, beta, -gamma_bank]
        q_RR = [theta, beta, -gamma_bank]
        q_RL = [theta, beta, gamma_bank]
        q_list = [q_FL, q_FR, q_RR, q_RL]
        
        # Objective: Find the Body Roll angle where left and right wheels are at the same Z level
        def objective(roll_deg):
            self.robot.base_ori = [np.deg2rad(roll_deg), 0, 0]
            wheel_pts_W = self.collision.get_wheel_contact_points(q_list)
            # Find the lowest contact point for left and right sides
            left_z = min(wheel_pts_W[0][2], wheel_pts_W[3][2])
            right_z = min(wheel_pts_W[1][2], wheel_pts_W[2][2])
            return (left_z - right_z)**2

        # Search within a physically reasonable boundary (-45 to 45 deg)
        res = minimize_scalar(objective, bounds=(-45, 45), method='bounded')
        induced_roll_deg = res.x
        
        # Calculate final state with the induced roll
        self.robot.base_ori = [np.deg2rad(induced_roll_deg), 0, 0]
        wheel_pts_W = self.collision.get_wheel_contact_points(q_list)
        # Shift the robot so the lowest contact point is exactly at Z=0 (ground)
        min_z = min(pt[2] for pt in wheel_pts_W)
        
        # Correct wheel points relative to ground
        for pt in wheel_pts_W:
            pt[2] -= min_z
            
        return induced_roll_deg, min_z, wheel_pts_W

    def calculate_cornering_stability(self, theta_deg=75.0, beta_deg=0.0, gamma_bank_deg=0.0, turn_radius=2.0):
        """
        Calculates the maximum cornering velocity and lateral acceleration limit 
        for a given banking posture, considering the true CoM and wheel contacts.
        
        Args:
            theta_deg, beta_deg, gamma_bank_deg: Posture configuration.
            turn_radius: Intended turning radius (meters). Left turn = positive radius.
            
        Returns:
            dict: Containing roll, CoM, margin, max velocity, and max lat. accel.
        """
        roll_deg, z_offset, wheel_pts = self.calculate_induced_roll(theta_deg, beta_deg, gamma_bank_deg)
        
        # 1. Calculate true CoM position in World Frame
        # Base was internally shifted by z_offset to touch Z=0
        self.robot.base_pos = np.array([0, 0, -z_offset])
        com_W = self.robot.body_to_world(self.com_bias_B)
        
        z_com = com_W[2]
        y_com = com_W[1]
        
        # 2. Determine the outer tipping boundary (Right side wheels for a left turn)
        # Assuming Left Turn (gamma_bank > 0), outer wheels are FR (index 1) and RR (index 2)
        if gamma_bank_deg >= 0:
            outer_y = max(wheel_pts[1][1], wheel_pts[2][1]) # Max Y (Right side in FLU is negative, but depends on coordinate definition, let's take absolute outer limit)
            # In FLU: Y is left(+), Right is (-). For Left turn, centrifugal force pushes Right(-).
            # The margin is the distance from CoM to the Right wheels.
            y_margin = y_com - min(wheel_pts[1][1], wheel_pts[2][1])
        else:
            # Right Turn (gamma_bank < 0), outer wheels are FL and RL
            y_margin = max(wheel_pts[0][1], wheel_pts[3][1]) - y_com
            
        # 3. Calculate Limits based on Moment Balance: F_c * Z_com = m * g * Y_margin
        # a_lat = F_c / m = (v^2 / R)
        # a_lat * Z_com = g * Y_margin  =>  a_lat_max = g * (Y_margin / Z_com)
        
        a_lat_max = self.g * (y_margin / z_com)
        v_max = np.sqrt(a_lat_max * abs(turn_radius))
        
        return {
            "induced_roll_deg": roll_deg,
            "com_z": z_com,
            "y_margin": y_margin,
            "a_lat_max_g": a_lat_max / self.g, # in units of g
            "v_max": v_max
        }
