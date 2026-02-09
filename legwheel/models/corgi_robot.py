import numpy as np
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.config import RobotParams
from legwheel.utils.screw import Screw

class CorgiRobot:
    """
    Model representing the full Corgi Robot with 4 limbs.
    Coordinates the kinematics of all legs and manages the robot's base pose.
    
    Limbs:
        0: FL (Front-Left)
        1: FR (Front-Right)
        2: RR (Rear-Right)
        3: RL (Rear-Left)
    """
    def __init__(self):
        """
        Initializes the Corgi robot with 4 legs and default base pose.
        """
        # Limbs initialization
        self.legs = [CorgiLegKinematics(i) for i in range(4)]
        
        # Robot Base Pose in World Frame {W}
        # [x, y, z, roll, pitch, yaw]
        self.base_pos = np.zeros(3)
        self.base_ori = np.zeros(3) # Roll, Pitch, Yaw (Euler angles)
        
        # Dimensions for reference
        self.wheelbase = RobotParams.WHEEL_BASE
        self.trackwidth = RobotParams.BODY_WIDTH

    def get_leg_positions(self, q_list):
        """
        Calculates the 3D positions of all 4 feet in the Body Frame {B}.
        Args:
            q_list (list of lists): List of 4 [theta, beta, gamma] joint angle sets.
        Returns:
            np.ndarray: 4x3 array of foot contact positions.
        """
        positions = []
        for i, q in enumerate(q_list):
            pos = self.legs[i].forward_kinematics(q[0], q[1], q[2])
            positions.append(pos)
        return np.array(positions)

    def inverse_kinematics(self, target_positions, guess_q=None):
        """
        Calculates joint angles for all 4 legs to reach target positions in {B}.
        Args:
            target_positions (np.ndarray): 4x3 target positions.
            guess_q (list of lists): Initial joint angle guesses.
        Returns:
            list: List of 4 [theta, beta, gamma] angle sets.
        """
        results = []
        for i in range(4):
            guess = guess_q[i] if guess_q is not None else None
            q = self.legs[i].inverse_kinematics(target_positions[i], guess_q=guess)
            results.append(q)
        return results

    def _rot_matrix(self, rpy):
        """Calculates 3x3 rotation matrix from RPY Euler angles."""
        r, p, y = rpy
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        
        R = np.array([
            [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
            [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
            [-sp, sr*cp, cr*cp]
        ])
        return R

    def body_to_world(self, p_B):
        """
        Transforms a point from Body Frame {B} to World Frame {W}.
        Args:
            p_B (np.ndarray): Point in {B}.
        Returns:
            np.ndarray: Point in {W}.
        """
        R_W = self._rot_matrix(self.base_ori)
        return R_W @ p_B + self.base_pos

    def world_to_body(self, p_W):
        """
        Transforms a point from World Frame {W} to Body Frame {B}.
        Args:
            p_W (np.ndarray): Point in {W}.
        Returns:
            np.ndarray: Point in {B}.
        """
        R_W = self._rot_matrix(self.base_ori)
        return R_W.T @ (p_W - self.base_pos)

    def __repr__(self):
        return f"CorgiRobot(BasePos={self.base_pos}, BaseOri={self.base_ori})"