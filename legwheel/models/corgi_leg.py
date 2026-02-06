import numpy as np
from legwheel.config import RobotParams
from legwheel.utils.fitted_coefficient import inv_G_dist_poly

class CorgiLegKinematics:
    """
    3D Kinematics for the Corgi Leg-Wheel module with ABAD (Abduction/Adduction).
    
    Frames:
        {W}: World Frame
        {R}: Robot Frame (Origin at IMU center)
        {Mi}: Module Frame (Base for each limb)
        {Li}: Leg Frame (Attached to ABAD output, rotates with gamma)
    
    Leg Indexing (i):
        0: FL (Front-Left)
        1: FR (Front-Right)
        2: RR (Rear-Right)
        3: RL (Rear-Left)
    """
    
    def __init__(self, leg_index):
        self.leg_index = leg_index
        self.is_left = leg_index in [0, 3]
        self.is_front = leg_index in [0, 1]
        
        # Dimensions from RobotParams
        self.w_body = RobotParams.BODY_WIDTH
        self.l_body = RobotParams.WHEEL_BASE
        self.d_abad = RobotParams.ABAD_AXIS_OFFSET
        self.d_wheel = RobotParams.WHEEL_AXIAL_OFFSET
        self.r_wheel = RobotParams.WHEEL_RADIUS_PITCH
        
        # Determine signs for frame origins
        self.sx = 1.0 if self.is_front else -1.0
        self.sy = 1.0 if self.is_left else -1.0
        
        # Origin of Module Frame {Mi} in Robot Frame {R}
        # FL: (+L/2, +W/2, +d_abad)
        # FR: (+L/2, -W/2, +d_abad) ... etc
        self.p_Mi_in_R = np.array([
            self.sx * 0.5 * self.l_body,
            self.sy * 0.5 * self.w_body,
            self.d_abad
        ])

    def _rot_x(self, angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])

    def _rot_y(self, angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, 0, s],
            [0, 1, 0],
            [-s, 0, c]
        ])
        
    def _rot_z(self, angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])

    def get_module_to_robot_transform(self, gamma):
        """
        Calculates T_Mi_to_R.
        Accounts for hip offset and active ABAD rotation gamma.
        """
        # This method is effectively replaced by the logic in forward_kinematics
        # but kept for potential separate use if needed.
        pass

    def fk_sagittal(self, theta, beta, alpha=0.0):
        """
        2D Forward Kinematics in the leg plane.
        Calculates the contact point [x, y, 0] in the Leg Frame {Li} (Sagittal Plane).
        
        Using polynomial approximations for the mechanism kinematics.
        """
        from legwheel.utils.fitted_coefficient import G_poly, inv_O_r_dist_poly
        from legwheel.config import RobotParams
        
        # We need a helper LegModel instance to access the polynomial logic consistently
        if not hasattr(self, '_leg_model_solver'):
            from legwheel.models.leg_model import LegModel
            self._leg_model_solver = LegModel()
            
        # Use the helper to calculate forward kinematics
        self._leg_model_solver.forward(theta, beta)
        
        # Get rim point based on alpha
        # rim_point returns complex number or numpy array [x, y]
        p_contact = self._leg_model_solver.rim_point(np.rad2deg(alpha))
        
        # If it's a vector [x, y]
        if isinstance(p_contact, np.ndarray):
            return np.array([p_contact[0], p_contact[1], 0.0])
        
        # Map complex result to 3D vector [x, y, 0]
        return np.array([p_contact.real, p_contact.imag, 0.0])

    def forward_kinematics(self, theta, beta, gamma, alpha=0.0):
        """
        Full 3D Forward Kinematics.
        Returns p_contact in Robot Frame {R}.
        """
        # 1. Sagittal FK in {Li} -> [x, y, 0] (complex real/imag mapped to x/y)
        p_L = self.fk_sagittal(theta, beta, alpha) 
        
        # 2. Transform {Li} -> {Mi}
        if self.is_left:
            # Basis vectors of {Li} expressed in {Mi} for Left Legs
            R_L_to_M = np.array([
                [0, 0, 1],
                [0, 1, 0],
                [-1, 0, 0]
            ])
        else: # Right Legs
            R_L_to_M = np.array([
                [0, 0, -1],
                [0, 1, 0],
                [1, 0, 0]
            ])

        # p_M = R_L_to_M @ p_L + [d_wheel, 0, 0]
        p_M = R_L_to_M @ p_L + np.array([self.d_wheel, 0, 0])
        
        # 3. Transform {Mi} -> {R}
        # Rotation about Z-axis of {Mi} (which aligns with +/- X_R)
        R_gamma = self._rot_z(gamma)
        
        # Final Position in {Mi} (rotated)
        p_M_rot = R_gamma @ p_M
        
        # Transform to {R}
        if self.is_left:
            # X_M = Y_R = [0, 1, 0]
            # Y_M = Z_R = [0, 0, 1]
            # Z_M = X_R = [1, 0, 0]
            R_M_to_R = np.array([
                [0, 0, 1],
                [1, 0, 0],
                [0, 1, 0]
            ]).T
        else: # Right
            # X_M = -Y_R = [0, -1, 0]
            # Y_M = Z_R = [0, 0, 1]
            # Z_M = -X_R = [-1, 0, 0]
            R_M_to_R = np.array([
                [0, -1, 0], 
                [0, 0, 1],  
                [-1, 0, 0]  
            ]).T

        p_R_rel = R_M_to_R @ p_M_rot
        
        return p_R_rel + self.p_Mi_in_R

    def inverse_kinematics(self, target_pos, guess_q=None):
        """
        3D Inverse Kinematics using Gauss-Newton.
        Target: [x, y, z] in {R}
        Output: [theta, beta, gamma]
        """
        if guess_q is None:
            guess_q = np.array([self.theta0, self.beta0, 0.0])
            
        q = guess_q
        for i in range(10): # Iterations
            # 1. Forward Kinematics
            curr_pos = self.forward_kinematics(*q)
            
            # 2. Error
            error = target_pos - curr_pos
            if np.linalg.norm(error) < 1e-4:
                break
                
            # 3. Jacobian (Numerical)
            delta = 1e-5
            J = np.zeros((3, 3))
            
            # d/d_theta
            fk_dth = self.forward_kinematics(q[0]+delta, q[1], q[2])
            J[:, 0] = (fk_dth - curr_pos) / delta
            
            # d/d_beta
            fk_dbe = self.forward_kinematics(q[0], q[1]+delta, q[2])
            J[:, 1] = (fk_dbe - curr_pos) / delta
            
            # d/d_gamma
            fk_dga = self.forward_kinematics(q[0], q[1], q[2]+delta)
            J[:, 2] = (fk_dga - curr_pos) / delta
            
            # 4. Update (Damped Least Squares / Pseudo-inverse)
            # dq = pinv(J) * error
            dq = np.linalg.pinv(J) @ error
            q = q + dq
            
        return q