import numpy as np
from scipy.optimize import fsolve
import time
from legwheel.utils.fitted_coefficient import *

#### To Do ####


#### LegModel ####
# Using approximate coefficients instead of kinematics.
# Init param - sim: True means using wheel radius without tire. False means using actual wheel radius (including tire thick).
class LegModel:
    def __init__(self):
        #### Constant values ####
        # max/min theta
        self.max_theta = np.deg2rad(160.0)  # maximum theta = 160 deg
        self.min_theta = np.deg2rad(17.0)   # minimum theta = 17 deg
        self.theta0 = np.deg2rad(17.0)      # theta0 = 17 deg
        self.beta0 = np.deg2rad(90.0)       # beta0  = 90 deg
        # wheel radius 
        self.R = 0.1    # 100 mm
        self.r = 0.019  # with tire: 19 mm
        self.radius = self.R + self.r
        # new foot design parameters
        self.foot_offset = 0.02225  # rim offset for new foot 22.25 mm
        self.tyre_thickness = 0.01225  # tire thickness 12.25 mm
        self.foot_radius = self.R + self.foot_offset + self.tyre_thickness  # foot radius 134.5 mm

        # linkage parameters
        self.arc_HF = np.deg2rad(130)   # arc HF
        self.arc_BC = np.deg2rad(101)   # arc BC
        self.l1 = 0.8 * self.R                                                          # l1: OA
        self.l2 = self.R - self.l1                                                      # l2: AB
        self.l3 = 2.0 * self.R * np.sin(self.arc_BC / 2)                                # l3: BC
        self.l4 = 0.882966335 * self.R                                                  # l4: CD
        self.l5 = 0.9 * self.R                                                          # l5: AD
        self.l6 = 0.4 * self.R                                                          # l6: DE
        self.l7 = 2.0 * self.R * np.sin((self.arc_HF - self.arc_BC - self.theta0) / 2)  # l7: CF
        self.l8 = 2.0 * self.R * np.sin((np.pi - self.arc_HF) / 2)                      # l8: FG
        # some useful paramters in the calculation
        self.l_AE = self.l5 + self.l6                                       # length of AE
        self.l_BF = 2.0 * self.R * np.sin((self.arc_HF - self.theta0) / 2)  # length of BF
        self.l_BH = 2.0 * self.R * np.sin( self.theta0 / 2)                 # length of BH
        self.ang_UBC = (np.pi - self.arc_BC) / 2                            # angle upperBC
        self.ang_LFG = (np.pi - (np.pi - self.arc_HF)) / 2                  # angle lowerFG
        self.ang_BCF = np.arccos((self.l3**2 + self.l7**2 - self.l_BF**2) / (2 * self.l3 * self.l7))    # angle BCF
        
        #### Variable values ####
        # intermediate values during the calculation
        self.l_BD = 0       # length of BD
        self.ang_OEA = 0    # angle OEA
        self.ang_DBC = 0    # angle DBC
        self.ang_OGF = 0    # angle OGF
        # get initial positions of all joints ([x, y])
        self.forward(np.deg2rad(17), np.deg2rad(0))
        
        #### Contact map variable ####
        self.rim = 3    # 1 -> 2 -> 3 -> 4 -> 5 -> 0: 
                        # U_l -> L_l -> G -> L_r -> U_r -> None
                        
        
    #### Forward kinematics ####
    # Input theta, beta: radian unit. Both can be either a single value or an 1-dimensional array with any size, but they have to be the same size.
    #           Values outside max/min limit will be set to max/min limit. 
    # Dimensions of joint position: size n array input: n*2 
    #                               single value input: 2
    def forward(self, theta, beta, vector=True):
        self.theta = np.array(theta)
        self.beta = np.array(beta)
        self.n_elements = 0 if self.theta.ndim == 0 else self.theta.shape[0]  # amount of theta given in an array, 0: single value.
        # Check theta range
        limit_u = self.theta > self.max_theta   # theta exceeding upper bound set to upper bound
        self.theta[limit_u] = self.max_theta
        limit_l = self.theta < self.min_theta   # theta below lower bound set to lower bound
        self.theta[limit_l] = self.min_theta
        if np.sum(limit_u) != 0:
            print("Limit upper bound:", np.sum(limit_u))
        if np.sum(limit_l) != 0:
            print("Limit lower bound:", np.sum(limit_l))
        # Forward kinematics
        self.calculate()
        self.rotate()
        if vector:
            self.to_vector()
    
    # Calculate by theta using coefficient
    def calculate(self, coefficient=False):
        if not coefficient:
            # Forward kinematics
            self.A_l = self.l1 * np.exp( 1j*(self.theta) )
            self.B_l = self.R * np.exp( 1j*(self.theta) )
            self.ang_OEA = np.arcsin(abs(self.A_l.imag) / self.l_AE)
            self.E = self.A_l.real - self.l_AE * np.cos(self.ang_OEA)   # OE = OA - EA
            self.D_l = self.E + self.l6 * np.exp( 1j*(self.ang_OEA) )
            self.l_BD = abs(self.D_l - self.B_l)
            self.ang_DBC = np.arccos((self.l_BD**2 + self.l3**2 - self.l4**2) / (2 * self.l_BD * self.l3))
            self.C_l = self.B_l + (self.D_l - self.B_l) * np.exp( -1j*(self.ang_DBC) ) * (self.l3 / self.l_BD) # OC = OB + BC
            self.F_l = self.C_l + (self.B_l - self.C_l) * np.exp( -1j*(self.ang_BCF) ) * (self.l7 / self.l3) # OF = OC + CF
            self.ang_OGF = np.arcsin(abs(self.F_l.imag) / self.l8)
            self.G = self.F_l.real - self.l8 * np.cos(self.ang_OGF) # OG = OF - GF
            self.U_l = self.B_l + (self.C_l - self.B_l) * np.exp( 1j*(self.ang_UBC) ) * (self.R / self.l3)   # OOU = OB + BOU
            self.L_l = self.F_l + (self.G - self.F_l) * np.exp( 1j*(self.ang_LFG) ) * (self.R / self.l8)   # OOL = OF + FOL
            self.H_l = self.U_l + (self.B_l - self.U_l) * np.exp( -1j*(self.theta0) )  # OH = OOU + OUH
            # foot characteristics points
            self.O_r = self.G.real + self.R  # rim center
            self.I_l = self.O_r + (self.R + self.foot_offset) * np.exp( 1j*(np.deg2rad(180-40)) )
            self.ang_OC = np.angle(self.C_l)
            self.J_l = self.U_l + (self.R + self.foot_offset) * np.exp( 1j*(np.deg2rad(140)+np.angle(self.H_l - self.U_l)))
            self.H_extend_l = self.U_l + (self.R + self.foot_offset) * np.exp( 1j*(np.angle(self.H_l - self.U_l)))

        else:
            # Using coefficient
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
        
    # Rotate by beta
    def rotate(self):
        rot_ang  = np.exp( 1j*(np.array(self.beta) + self.beta0) )
        # Rotate
        self.A_l = rot_ang * self.A_l
        self.A_r = rot_ang * self.A_r
        self.B_l = rot_ang * self.B_l
        self.B_r = rot_ang * self.B_r
        self.C_l = rot_ang * self.C_l
        self.C_r = rot_ang * self.C_r
        self.D_l = rot_ang * self.D_l
        self.D_r = rot_ang * self.D_r
        self.E   = rot_ang * self.E
        self.F_l = rot_ang * self.F_l
        self.F_r = rot_ang * self.F_r
        self.G   = rot_ang * self.G
        self.H_l = rot_ang * self.H_l
        self.H_r = rot_ang * self.H_r
        self.U_l = rot_ang * self.U_l
        self.U_r = rot_ang * self.U_r
        self.L_l = rot_ang * self.L_l
        self.L_r = rot_ang * self.L_r
        self.O_r = rot_ang * self.O_r
        self.I_l = rot_ang * self.I_l
        self.I_r = rot_ang * self.I_r
        self.J_l = rot_ang * self.J_l
        self.J_r = rot_ang * self.J_r
        self.H_extend_l = rot_ang * self.H_extend_l
        self.H_extend_r = rot_ang * self.H_extend_r

    # Get right side joints before rotate beta
    def symmetry(self):
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

    # Convert position expressions from complex numbers to vectors
    def to_vector(self):
        if self.n_elements == 0:
            self.A_l = np.array([self.A_l.real, self.A_l.imag])
            self.A_r = np.array([self.A_r.real, self.A_r.imag])
            self.B_l = np.array([self.B_l.real, self.B_l.imag])
            self.B_r = np.array([self.B_r.real, self.B_r.imag])
            self.C_l = np.array([self.C_l.real, self.C_l.imag])
            self.C_r = np.array([self.C_r.real, self.C_r.imag])
            self.D_l = np.array([self.D_l.real, self.D_l.imag])
            self.D_r = np.array([self.D_r.real, self.D_r.imag])
            self.E   = np.array([self.E.real, self.E.imag])
            self.F_l = np.array([self.F_l.real, self.F_l.imag])
            self.F_r = np.array([self.F_r.real, self.F_r.imag])
            self.G   = np.array([self.G.real, self.G.imag])
            self.H_l = np.array([self.H_l.real, self.H_l.imag])
            self.H_r = np.array([self.H_r.real, self.H_r.imag])
            self.U_l = np.array([self.U_l.real, self.U_l.imag])
            self.U_r = np.array([self.U_r.real, self.U_r.imag])
            self.L_l = np.array([self.L_l.real, self.L_l.imag])
            self.L_r = np.array([self.L_r.real, self.L_r.imag])
            self.O_r = np.array([self.O_r.real, self.O_r.imag])
            self.I_l = np.array([self.I_l.real, self.I_l.imag])
            self.I_r = np.array([self.I_r.real, self.I_r.imag])
            self.J_l = np.array([self.J_l.real, self.J_l.imag])
            self.J_r = np.array([self.J_r.real, self.J_r.imag])
            self.H_extend_l = np.array([self.H_extend_l.real, self.H_extend_l.imag])
            self.H_extend_r = np.array([self.H_extend_r.real, self.H_extend_r.imag])
        else:
            self.A_l = np.array([self.A_l.real, self.A_l.imag]).transpose(1, 0)
            self.A_r = np.array([self.A_r.real, self.A_r.imag]).transpose(1, 0)
            self.B_l = np.array([self.B_l.real, self.B_l.imag]).transpose(1, 0)
            self.B_r = np.array([self.B_r.real, self.B_r.imag]).transpose(1, 0)
            self.C_l = np.array([self.C_l.real, self.C_l.imag]).transpose(1, 0)
            self.C_r = np.array([self.C_r.real, self.C_r.imag]).transpose(1, 0)
            self.D_l = np.array([self.D_l.real, self.D_l.imag]).transpose(1, 0)
            self.D_r = np.array([self.D_r.real, self.D_r.imag]).transpose(1, 0)
            self.E   = np.array([self.E.real, self.E.imag]).transpose(1, 0)
            self.F_l = np.array([self.F_l.real, self.F_l.imag]).transpose(1, 0)
            self.F_r = np.array([self.F_r.real, self.F_r.imag]).transpose(1, 0)
            self.G   = np.array([self.G.real, self.G.imag]).transpose(1, 0)
            self.H_l = np.array([self.H_l.real, self.H_l.imag]).transpose(1, 0)
            self.H_r = np.array([self.H_r.real, self.H_r.imag]).transpose(1, 0)
            self.U_l = np.array([self.U_l.real, self.U_l.imag]).transpose(1, 0)
            self.U_r = np.array([self.U_r.real, self.U_r.imag]).transpose(1, 0)
            self.L_l = np.array([self.L_l.real, self.L_l.imag]).transpose(1, 0)
            self.L_r = np.array([self.L_r.real, self.L_r.imag]).transpose(1, 0)
            self.I_l = np.array([self.I_l.real, self.I_l.imag]).transpose(1, 0)
            self.I_r = np.array([self.I_r.real, self.I_r.imag]).transpose(1, 0)
            self.O_r = np.array([self.O_r.real, self.O_r.imag]).transpose(1, 0)
            self.J_l = np.array([self.J_l.real, self.J_l.imag]).transpose(1, 0)
            self.J_r = np.array([self.J_r.real, self.J_r.imag]).transpose(1, 0)
            self.H_extend_l = np.array([self.H_extend_l.real, self.H_extend_l.imag]).transpose(1, 0)
            self.H_extend_r = np.array([self.H_extend_r.real, self.H_extend_r.imag]).transpose(1, 0)

    
    def rot(self, ang):
        """returns the rotation matrix for a given angle.

        Args:
            ang (float): The angle (in radians) to rotate.

        Returns:
            np.ndarray: The 2D rotation matrix.
        """
        ang = np.float64(ang)
        rot_matrix = np.array([[np.cos(ang), -np.sin(ang)],
                                [np.sin(ang),  np.cos(ang)]])
        return rot_matrix

    # Calculate rim point position
    def rim_point(self, alpha = 0.0):
        """
        Calculate the rim point position for a given alpha angle.\n
        note: its for the leg designed by starlee, not for the origin one
        ## Rim point position definition
        foot: [-40,+40] deg\n
        Upper rim RHS: (40, 180] deg \n
        Upper rim LHS: [-180, -40) deg
        Args:
            alpha (float): The alpha angle (degree) for which to calculate the rim point.
        Returns:
            return: The position of the rim point as a complex number.
        """
        # calculate in vector forms
        self.forward(self.theta, self.beta, vector=True)
        alpha = np.array(alpha)
        n_elements = 0 if alpha.ndim == 0 else alpha.shape[0]  # amount of theta given in an array, 0: single value.
        if n_elements == 0 and self.n_elements == 0:
            # check if alpha in range and turn it into the range [-180, 180]
            if alpha < -180 or alpha > 180:
                alpha = ((alpha + 180) % 360) - 180
            if -40 <= alpha <= 40:
                # Foot rim
                # P(alpha) = O_r + (R+foot_offset)*rot(alpha)*norm(O_r -> G)
                rim_point = self.O_r + (self.R+self.foot_offset+self.tyre_thickness)*(self.rot(np.deg2rad(alpha))@(self.G-self.O_r))/np.linalg.norm(self.G-self.O_r)
            elif 40 < alpha <= 180:
                # Upper rim RHS
                # P(alpha) = U_r + (R+foot_offset)*rot(alpha-40)*norm(U_r -> J_r)
                rim_point = self.U_r + (self.R+self.foot_offset+self.tyre_thickness)*(self.rot(np.deg2rad(alpha-40))@(self.J_r-self.U_r))/np.linalg.norm(self.J_r-self.U_r)
            elif -180 <= alpha < -40:
                # Upper rim LHS
                # P(alpha) = U_l + (R+foot_offset)*rot(alpha+40)*norm(U_l -> J_l)
                rim_point = self.U_l + (self.R+self.foot_offset+self.tyre_thickness)*(self.rot(np.deg2rad(alpha+40))@(self.J_l-self.U_l))/np.linalg.norm(self.J_l-self.U_l)
            else:
                raise ValueError("Invalid alpha angle.")
            return rim_point
        elif n_elements == 0:
            if -40 <= alpha <= 40:
                # Foot rim
                # P(alpha) = O_r + (R+foot_offset)*rot(alpha)*norm(O_r -> G)
                rim_point = self.O_r + (self.R+self.foot_offset+self.tyre_thickness)*   (self.rot(np.deg2rad(alpha))   @(self.G-self.O_r).T    ).T/ np.linalg.norm(self.G-self.O_r)
            elif 40 < alpha <= 180:
                # Upper rim RHS
                # P(alpha) = U_r + (R+foot_offset)*rot(alpha-40)*norm(U_r -> J_r)
                rim_point = self.U_r + (self.R+self.foot_offset+self.tyre_thickness)*   (self.rot(np.deg2rad(alpha-40))@(self.J_r-self.U_r).T  ).T/ np.linalg.norm(self.J_r-self.U_r)
            elif -180 <= alpha < -40:
                # Upper rim LHS
                # P(alpha) = U_l + (R+foot_offset)*rot(alpha+40)*norm(U_l -> J_l)
                rim_point = self.U_l + (self.R+self.foot_offset+self.tyre_thickness)*   (self.rot(np.deg2rad(alpha+40))@(self.J_l-self.U_l).T  ).T/ np.linalg.norm(self.J_l-self.U_l)
            else:
                raise ValueError("Invalid alpha angle.")
            return rim_point
        else:
            # check if alpha in range and turn it into the range [-180, 180]
            alpha = np.asarray(alpha)
            alpha_mod = ((alpha + 180) % 360) - 180
            
            # Create masks for each region
            mask_foot = (-40 <= alpha_mod) & (alpha_mod <= 40)
            mask_upper_rhs = (40 < alpha_mod) & (alpha_mod <= 180)
            mask_upper_lhs = (-180 <= alpha_mod) & (alpha_mod < -40)
            
            # Prepare output array
            rim_points = np.zeros((alpha_mod.size, 2))  # or appropriate shape

            # Assign for foot rim
            indices_foot = np.where(mask_foot)[0]
            if indices_foot.size > 0:
                rim_points[indices_foot] = np.array([self.rim_point(a) for a in alpha_mod[mask_foot]])

            # Assign for upper rim RHS
            indices_rhs = np.where(mask_upper_rhs)[0]
            if indices_rhs.size > 0:
                rim_points[indices_rhs] = np.array([self.rim_point(a) for a in alpha_mod[mask_upper_rhs]])

            # Assign for upper rim LHS
            indices_lhs = np.where(mask_upper_lhs)[0]
            if indices_lhs.size > 0:
                rim_points[indices_lhs] = np.array([self.rim_point(a) for a in alpha_mod[mask_upper_lhs]])

            return rim_points

    def __getitem__(self, key):
        """
        Allows access to joint positions using [] operator.
        Example: legmodel['G'] returns the position of joint G.
        """
        if key not in self.__dict__:
            raise KeyError(f"Joint '{key}' not found.")
        return self.__dict__[key]

if __name__ == '__main__':
    legmodel = LegModel()
    
    #### Forward kinematics ####
    print("****************************************")
    print("****** Forward kinematics example ******")
    print("****************************************")
    # input single value
    print("==========Single Input==========")
    theta = np.deg2rad(130)
    beta  = np.deg2rad(10)
    legmodel.forward(theta, beta)
    print("Output G with single value input:", legmodel.G)
    legmodel.contact_map(theta, beta)
    print("Output rim with single value input:", legmodel.rim)
    print("Output alpha with single value input:", legmodel.alpha)
    print("Output contact_p with single value input:", legmodel.contact_p)
    # input array with 1 element
    print("==========1 Element Array Input==========")
    theta = np.array([theta])
    beta  = np.array([beta])
    legmodel.forward(theta, beta)
    print("Output G with 1 element array input:", legmodel.G)
    legmodel.contact_map(theta, beta)
    print("Output rim with 1 element array input:", legmodel.rim)
    print("Output alpha with 1 element array input:", legmodel.alpha)
    print("Output contact_p with 1 element array input:", legmodel.contact_p)
    # input array with n elements
    print("==========n Elements Array Input==========")
    theta = np.linspace(17, 160, 3)
    theta = np.deg2rad(theta)
    beta = np.linspace(0, 360, 3)
    beta = np.deg2rad(beta)
    legmodel.forward(theta, beta)
    print("Output G with n elements array input:", legmodel.G)
    legmodel.contact_map(theta, beta)
    print("Output rim with n elements array input:", legmodel.rim)
    print("Output alpha with n elements array input:", legmodel.alpha)
    print("Output contact_p with n elements array input:", legmodel.contact_p)
