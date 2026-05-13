import numpy as np
from legwheel.config import RobotParams

class WheeledDynamics:
    """
    2D / 3D Wheeled Dynamics model for the Corgi Leg-Wheel robot.
    Handles differential drive mapping, skid-steering, and lateral forces.
    """
    def __init__(self, mu_lat=0.6, mass=30.0):
        """
        Initialize the WheeledDynamics model.
        
        Args:
            mu_lat (float): Lateral friction coefficient for skid-steering.
            mass (float): Total mass of the robot in kg.
        """
        self.L_body = RobotParams.WHEEL_BASE
        self.W_body = RobotParams.BODY_WIDTH
        self.d_abad = RobotParams.ABAD_AXIS_OFFSET
        self.d_wheel = RobotParams.WHEEL_AXIAL_OFFSET
        self.R = RobotParams.WHEEL_RADIUS_PITCH
        self.mass = mass
        self.g = 9.81
        self.mu_lat = mu_lat
        
    def get_track_width(self, gamma=0.0):
        """
        Calculate the equivalent track width (distance between left and right wheel contact points).
        
        Args:
            gamma (float): ABAD angle in radians. Currently assumes symmetric posture.
            
        Returns:
            float: Track width in meters.
        """
        # For simple 2D unbanked turns (gamma = 0), track width is constant.
        # W_track = W_body + 2 * (d_abad + d_wheel)
        # Note: If gamma != 0, track width changes due to mechanism projection.
        # Here we implement the base gamma=0 case first.
        track_width = self.W_body + 2 * (self.d_abad + self.d_wheel)
        
        if gamma != 0.0:
            # Approximate projection for banked turns (will be expanded in Active Banking)
            # track_width += 2 * L_leg * sin(gamma) ... (simplified)
            pass
            
        return track_width
        
    def differential_drive_ik(self, v_x, omega_z, skid_factor=1.0):
        """
        Calculate target wheel speeds for a given body velocity twist.
        
        Args:
            v_x (float): Target forward velocity (m/s)
            omega_z (float): Target yaw rate (rad/s)
            skid_factor (float): Skid factor (chi) to compensate for lateral slipping.
                                 W_eq = W_track * skid_factor
                                 
        Returns:
            tuple: (omega_L, omega_R) target wheel angular velocities in rad/s
        """
        W_track = self.get_track_width()
        W_eq = W_track * skid_factor
        
        v_L = v_x - (W_eq / 2.0) * omega_z
        v_R = v_x + (W_eq / 2.0) * omega_z
        
        omega_L = v_L / self.R
        omega_R = v_R / self.R
        
        return omega_L, omega_R
        
    def estimate_resistive_moment(self):
        """
        Estimate the resistive moment M_r caused by lateral sliding friction during a turn.
        Assumes uniform weight distribution across all 4 wheels.
        
        Returns:
            float: Resistive moment M_r in N.m
        """
        # F_z per wheel approx mg / 4
        F_z_i = (self.mass * self.g) / 4.0
        
        # Lateral friction per wheel
        F_y_i = self.mu_lat * F_z_i
        
        # M_r = 4 * F_y_i * (L_body / 2)
        M_r = 4 * F_y_i * (self.L_body / 2.0)
        
        return M_r
        
    def calculate_required_traction(self):
        """
        Calculate the required longitudinal force difference (Delta F_x) 
        between right and left wheels to overcome the resistive moment.
        
        Returns:
            float: Required delta traction force (N)
        """
        M_r = self.estimate_resistive_moment()
        W_track = self.get_track_width()
        
        # M_r = Delta_F_x * (W_track / 2)
        delta_F_x = M_r / (W_track / 2.0)
        return delta_F_x
