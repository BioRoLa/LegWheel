import numpy as np
from legwheel.config import RobotParams


class WheeledDynamics:
    """
    2D / 3D Wheeled Dynamics model for the Corgi Leg-Wheel robot.
    Handles differential drive mapping, skid-steering, and lateral forces.
    """

    def __init__(self, mu_lat=0.6, mass=None):
        """
        Initialize the WheeledDynamics model.

        Args:
            mu_lat (float): Lateral friction coefficient for skid-steering.
            mass (float): Total mass of the robot in kg. Defaults to RobotParams.MASS.
        """
        self.L_body = RobotParams.WHEEL_BASE
        self.W_body = RobotParams.BODY_WIDTH
        self.d_wheel = RobotParams.WHEEL_AXIAL_OFFSET
        self.R = RobotParams.WHEEL_RADIUS_PITCH
        self.mass = mass if mass is not None else RobotParams.MASS
        self.g = 9.81
        self.mu_lat = mu_lat

    def get_track_width(self, gamma=0.0):
        """
        Calculate the equivalent track width (distance between left and right wheel contact points).

        At gamma=0 the wheel center lies at W_body/2 + d_wheel from the body midplane on each side.
        ABAD rotation (gamma) projects d_wheel by cos(gamma), reducing the lateral reach.

        Args:
            gamma (float): ABAD angle in radians (symmetric left/right posture assumed).

        Returns:
            float: Track width in meters.
        """
        # W_track = W_body + 2 * d_wheel * cos(gamma)
        # Derivation: wheel center in body frame Y = W_body/2 + d_wheel*cos(gamma) per side.
        # ABAD_AXIS_OFFSET is a Z-direction offset (hip roll height), not lateral.
        return self.W_body + 2 * self.d_wheel * np.cos(gamma)

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
