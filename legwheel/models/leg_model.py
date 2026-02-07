import numpy as np
from scipy.optimize import fsolve
import time
from legwheel.utils.fitted_coefficient import *
from legwheel.config import RobotParams

#### LegModel ####
# Using approximate coefficients instead of kinematics.
# Init param - sim: True means using wheel radius without tire. False means using actual wheel radius (including tire thick).
class LegModel:
    """
    2D Kinematics model for the Leg-Wheel mechanism.
    Handles joint positions calculation and rim point mapping.
    """
    def __init__(self):
        #### Constant values from Config ####
        self.max_theta = np.deg2rad(RobotParams.MAX_THETA_DEG)
        self.min_theta = np.deg2rad(RobotParams.MIN_THETA_DEG)
        self.theta0 = np.deg2rad(RobotParams.THETA0_DEG)
        self.beta0 = np.deg2rad(RobotParams.BETA0_DEG)
        
        # Wheel and Foot dimensions
        # R: Effective wheel radius (100mm)
        # r: Linkage rim radius (19mm)
        self.R = RobotParams.WHEEL_RADIUS_PITCH
        self.r = 0.019 # Default linkage radius
        self.radius = self.R + self.r
        
        # Foot design parameters
        self.foot_offset = 0.02225  # 22.25 mm
        self.tyre_thickness = 0.01225 # 12.25 mm
        self.foot_radius = self.R + self.foot_offset + self.tyre_thickness

        # Linkage parameters (proportional to R)
        self.arc_HF = np.deg2rad(RobotParams.ARC_HF_DEG)
        self.arc_BC = np.deg2rad(RobotParams.ARC_BC_DEG)
        self.l1 = RobotParams.L1_RATIO * self.R
        self.l2 = self.R - self.l1
        self.l3 = 2.0 * self.R * np.sin(self.arc_BC / 2)
        self.l4 = 0.882966335 * self.R
        self.l5 = RobotParams.L5_RATIO * self.R
        self.l6 = RobotParams.L6_RATIO * self.R
        self.l7 = 2.0 * self.R * np.sin((self.arc_HF - self.arc_BC - self.theta0) / 2)
        self.l8 = 2.0 * self.R * np.sin((np.pi - self.arc_HF) / 2)
        
        self.l_AE = self.l5 + self.l6
        self.l_BF = 2.0 * self.R * np.sin((self.arc_HF - self.theta0) / 2)
        self.l_BH = 2.0 * self.R * np.sin( self.theta0 / 2)
        self.ang_UBC = (np.pi - self.arc_BC) / 2
        self.ang_LFG = (np.pi - (np.pi - self.arc_HF)) / 2
        self.ang_BCF = np.arccos((self.l3**2 + self.l7**2 - self.l_BF**2) / (2 * self.l3 * self.l7))
        
        #### Variable values ####
        self.l_BD = 0
        self.ang_OEA = 0
        self.ang_DBC = 0
        self.ang_OGF = 0
        
        # Initial pose
        self.forward(self.theta0, 0.0, vector=True)

    def forward(self, theta, beta, vector=True):
        """
        Calculates joint positions given theta and beta.
        Args:
            theta, beta: Joint angles in radians.
            vector (bool): If True, converts complex positions to [x, y] arrays.
        """
        self.theta = np.array(theta)
        self.beta = np.array(beta)
        self.n_elements = 0 if self.theta.ndim == 0 else self.theta.shape[0]
        
        # Clip theta to limits
        self.theta = np.clip(self.theta, self.min_theta, self.max_theta)
        
        self.calculate()
        self.rotate()
        
        if vector:
            self.to_vector()

    def calculate(self, coefficient=False):
        """Internal linkage geometry solver."""
        if not coefficient:
            self.A_l = self.l1 * np.exp( 1j*(self.theta) )
            self.B_l = self.R * np.exp( 1j*(self.theta) )
            self.ang_OEA = np.arcsin(np.abs(self.A_l.imag) / self.l_AE)
            self.E = self.A_l.real - self.l_AE * np.cos(self.ang_OEA)
            self.D_l = self.E + self.l6 * np.exp( 1j*(self.ang_OEA) )
            self.l_BD = np.abs(self.D_l - self.B_l)
            self.ang_DBC = np.arccos((self.l_BD**2 + self.l3**2 - self.l4**2) / (2 * self.l_BD * self.l3))
            self.C_l = self.B_l + (self.D_l - self.B_l) * np.exp( -1j*(self.ang_DBC) ) * (self.l3 / self.l_BD)
            self.F_l = self.C_l + (self.B_l - self.C_l) * np.exp( -1j*(self.ang_BCF) ) * (self.l7 / self.l3)
            self.ang_OGF = np.arcsin(np.abs(self.F_l.imag) / self.l8)
            self.G = self.F_l.real - self.l8 * np.cos(self.ang_OGF)
            self.U_l = self.B_l + (self.C_l - self.B_l) * np.exp( 1j*(self.ang_UBC) ) * (self.R / self.l3)
            self.L_l = self.F_l + (self.G - self.F_l) * np.exp( 1j*(self.ang_LFG) ) * (self.R / self.l8)
            self.H_l = self.U_l + (self.B_l - self.U_l) * np.exp( -1j*(self.theta0) )
            
            # Foot characteristics
            self.O_r = self.G.real + self.R
            self.I_l = self.O_r + (self.R + self.foot_offset) * np.exp( 1j*(np.deg2rad(180-40)) )
            self.ang_OC = np.angle(self.C_l)
            self.J_l = self.U_l + (self.R + self.foot_offset) * np.exp( 1j*(np.deg2rad(140)+np.angle(self.H_l - self.U_l)))
            self.H_extend_l = self.U_l + (self.R + self.foot_offset) * np.exp( 1j*(np.angle(self.H_l - self.U_l)))
        else:
            self.A_l = A_l_poly[1](self.theta) - 1j * A_l_poly[0](self.theta)
            self.B_l = B_l_poly[1](self.theta) - 1j * B_l_poly[0](self.theta)
            self.C_l = C_l_poly[1](self.theta) - 1j * C_l_poly[0](self.theta)
            self.D_l = D_l_poly[1](self.theta) - 1j * D_l_poly[0](self.theta)
            self.E   = E_poly[1](self.theta)   - 1j * E_poly[0](self.theta)
            self.F_l = F_l_poly[1](self.theta) - 1j * F_l_poly[0](self.theta)
            self.G   = G_poly[1](self.theta)   - 1j * G_poly[0](self.theta)
            self.H_l = H_l_poly[1](self.theta) - 1j * H_l_poly[0](self.theta)
            self.U_l = U_l_poly[1](self.theta) - 1j * U_l_poly[0](self.theta)
            self.L_l = L_l_poly[1](self.theta) - 1j * L_l_poly[0](self.theta)
        
        self.symmetry()

    def rotate(self):
        """Rotates joint positions by beta."""
        rot_ang = np.exp( 1j*(np.array(self.beta) + self.beta0) )
        attrs = ['A_l', 'A_r', 'B_l', 'B_r', 'C_l', 'C_r', 'D_l', 'D_r', 'E', 'F_l', 'F_r', 
                 'G', 'H_l', 'H_r', 'U_l', 'U_r', 'L_l', 'L_r', 'O_r', 'I_l', 'I_r', 
                 'J_l', 'J_r', 'H_extend_l', 'H_extend_r']
        for attr in attrs:
            if hasattr(self, attr):
                setattr(self, attr, rot_ang * getattr(self, attr))

    def symmetry(self):
        """Generates right-side symmetric joint positions."""
        self.A_r = np.conjugate(self.A_l)
        self.B_r = np.conjugate(self.B_l)
        self.C_r = np.conjugate(self.C_l)
        self.D_r = np.conjugate(self.D_l)
        self.F_r = np.conjugate(self.F_l)
        self.H_r = np.conjugate(self.H_l)
        self.U_r = np.conjugate(self.U_l)
        self.L_r = np.conjugate(self.L_l)
        self.I_r = np.conjugate(self.I_l)
        self.J_r = np.conjugate(self.J_l)
        self.H_extend_r = np.conjugate(self.H_extend_l)

    def to_vector(self):
        """Converts complex positions to numpy [x, y] arrays."""
        attrs = ['A_l', 'A_r', 'B_l', 'B_r', 'C_l', 'C_r', 'D_l', 'D_r', 'E', 'F_l', 'F_r', 
                 'G', 'H_l', 'H_r', 'U_l', 'U_r', 'L_l', 'L_r', 'O_r', 'I_l', 'I_r', 
                 'J_l', 'J_r', 'H_extend_l', 'H_extend_r']
        for attr in attrs:
            if not hasattr(self, attr): continue
            val = getattr(self, attr)
            if self.n_elements == 0:
                setattr(self, attr, np.array([val.real, val.imag]))
            else:
                setattr(self, attr, np.array([val.real, val.imag]).transpose(1, 0))

    def rot(self, ang):
        """Returns 2D rotation matrix."""
        return np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang),  np.cos(ang)]])

    def rim_point(self, alpha=0.0):
        """
        Calculates point on the wheel rim for given alpha angle (degrees).
        Args:
            alpha: Angle in degrees, where 0° is directly in front of the wheel (in the direction of motion), and positive angles rotate counterclockwise.
        """
        alpha_rad = np.deg2rad(alpha)
        # We ensure it's calculated in vector form internally
        self.forward(self.theta, self.beta, vector=True)
        
        # Logic matches previous version but handles vector outputs consistently
        if self.n_elements == 0:
            a_mod = ((alpha + 180) % 360) - 180
            if -40 <= a_mod <= 40:
                # Foot rim
                vec = (self.G - self.O_r) / np.linalg.norm(self.G - self.O_r)
                return self.O_r + (self.R + self.foot_offset + self.tyre_thickness) * (self.rot(alpha_rad) @ vec)
            elif 40 < a_mod <= 180:
                # Upper rim RHS
                vec = (self.J_r - self.U_r) / np.linalg.norm(self.J_r - self.U_r)
                return self.U_r + (self.R + self.foot_offset + self.tyre_thickness) * (self.rot(np.deg2rad(a_mod-40)) @ vec)
            else:
                # Upper rim LHS
                vec = (self.J_l - self.U_l) / np.linalg.norm(self.J_l - self.U_l)
                return self.U_l + (self.R + self.foot_offset + self.tyre_thickness) * (self.rot(np.deg2rad(a_mod+40)) @ vec)
        else:
            # Batch calculation for arrays (simplified for this refactor)
            return np.array([self.rim_point(a) for a in np.atleast_1d(alpha)])

    def __getitem__(self, key):
        if key not in self.__dict__: raise KeyError(f"Joint '{key}' not found.")
        return self.__dict__[key]

if __name__ == '__main__':
    lm = LegModel()
    print("Default G:", lm.G)