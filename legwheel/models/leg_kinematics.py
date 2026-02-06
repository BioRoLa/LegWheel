import numpy as np
import time

#### LegKinematics ####
# Forward kinematics getting approximate coefficient used in class LegWheel.
# 4-th linkage length needs to be determined for the first time.
class LegKinematics:
    def __init__(self) -> None:
        #### Constant values ####
        # max/min theta
        self.max_theta = np.deg2rad(160.0)  # maximum theta = 160 deg
        self.min_theta = np.deg2rad(17.0)   # minimum theta = 17 deg
        self.theta0 = np.deg2rad(17.0)      # theta0 = 17 deg
        self.beta0 = np.deg2rad(90)         # beta0  = 90 deg
        # wheel radius 
        self.R = 0.1    # 10 cm
        
        # new foot design parameters
        self.foot_offset = 0.02225  # rim offset for new foot 22.25 mm
        self.tyre_thickness = 0.01225  # tire thickness 12.25 mm
        self.foot_radius = self.R + self.foot_offset + self.tyre_thickness  # foot radius 134.5 mm
        
        # linkage parameters
        self.arc_HF = np.deg2rad(130)   # arc HF
        self.arc_BC = np.deg2rad(101)   # arc BC
        self.l1 = 0.8 * self.R                                                          # l1
        self.l2 = self.R - self.l1                                                      # l2
        self.l3 = 2.0 * self.R * np.sin(self.arc_BC / 2)                                # l3
        self.l4 = 0.882966335 * self.R                                                  # l4
        self.l5 = 0.9 * self.R                                                          # l5
        self.l6 = 0.4 * self.R                                                          # l6
        self.l7 = 2.0 * self.R * np.sin((self.arc_HF - self.arc_BC - self.theta0) / 2)  # l7
        self.l8 = 2.0 * self.R * np.sin((np.pi - self.arc_HF) / 2)                      # l8
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
        
    #### Forward kinematics ####
    def forward(self, theta, beta, vector=True):
        self.theta = np.array(theta)
        self.beta = np.array(beta)
        self.n_elements = 0 if self.theta.ndim == 0 else self.theta.shape[0]  # amount of theta given in an array, 0: single value.
        # Check theta range
        limit_u = self.theta > self.max_theta   # theta exceeding upper bound set to upper bound
        limit_l = self.theta < self.min_theta   # theta below lower bound set to lower bound
        self.theta[limit_u] = self.max_theta
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
    
    # Calculate by theta using complex (real:x, imaginary: y)
    def calculate(self):
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
        self.J_l = self.U_l + (self.R + self.foot_offset) * np.exp( 1j*(np.deg2rad(135)+np.angle(self.H_l - self.U_l)))
        self.H_extend_l = self.U_l + (self.R + self.foot_offset) * np.exp( 1j*(np.angle(self.H_l - self.U_l)))
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
    
    #### Calculate 4-th linkage length from wheel mode ####
    def calculate_l4(self):
        self.theta = self.theta0
        # part of the forward kinematics
        self.A_l = self.l1 * np.exp( 1j*(self.theta) )
        self.ang_OEA = np.arcsin(abs(self.A_l.imag) / self.l_AE)
        self.E = self.A_l.real - self.l_AE * np.cos(self.ang_OEA)   # OE = OA - EA
        self.D_l = self.E + self.l6 * np.exp( 1j*(self.ang_OEA) )
        # C is calculated for the case of theta = 17 deg.
        self.C_l = self.R * np.exp( 1j*(self.arc_BC + self.theta0) ) # OC = OH rotate arc_HC
        print(f"l4 = {abs(self.D_l - self.C_l):.20f}" )  # l4 = length of CD
    
    
if __name__ == '__main__':
    legwheel = LegKinematics()
    
    #### Calculate l4 ####
    legwheel.calculate_l4()
    
    #### Forward kinematics with beta=0 ####
    start_time = time.time()  # start time
    theta = np.linspace(17, 160, 1000000) # lower this number if use too much memory
    theta = np.deg2rad(theta)
    beta = np.deg2rad(0)
    legwheel.forward(theta, beta)
    end_time = time.time()  # end time
    print("Forward Kinematics Calculation Time:", end_time - start_time, "seconds")
    
    #### Coefficient fitting (Only fit left side) ####
    start_time = time.time()  # start time
    A_x_coef = np.polyfit(theta, legwheel.A_l[:, 0], 7)
    A_y_coef = np.polyfit(theta, legwheel.A_l[:, 1], 7)
    B_x_coef = np.polyfit(theta, legwheel.B_l[:, 0], 7)
    B_y_coef = np.polyfit(theta, legwheel.B_l[:, 1], 7)
    C_x_coef = np.polyfit(theta, legwheel.C_l[:, 0], 7)
    C_y_coef = np.polyfit(theta, legwheel.C_l[:, 1], 7)
    D_x_coef = np.polyfit(theta, legwheel.D_l[:, 0], 7)
    D_y_coef = np.polyfit(theta, legwheel.D_l[:, 1], 7)
    E_y_coef = np.polyfit(theta, legwheel.E[:, 1], 7)
    F_x_coef = np.polyfit(theta, legwheel.F_l[:, 0], 7)
    F_y_coef = np.polyfit(theta, legwheel.F_l[:, 1], 7)
    G_y_coef = np.polyfit(theta, legwheel.G[:, 1], 7)
    H_x_coef = np.polyfit(theta, legwheel.H_l[:, 0], 7)
    H_y_coef = np.polyfit(theta, legwheel.H_l[:, 1], 7)
    U_x_coef = np.polyfit(theta, legwheel.U_l[:, 0], 7)
    U_y_coef = np.polyfit(theta, legwheel.U_l[:, 1], 7)
    L_x_coef = np.polyfit(theta, legwheel.L_l[:, 0], 7)
    L_y_coef = np.polyfit(theta, legwheel.L_l[:, 1], 7)
    I_x_coef = np.polyfit(theta, legwheel.I_l[:, 0], 7)
    I_y_coef = np.polyfit(theta, legwheel.I_l[:, 1], 7)
    J_x_coef = np.polyfit(theta, legwheel.J_l[:, 0], 7)
    J_y_coef = np.polyfit(theta, legwheel.J_l[:, 1], 7)
    O_r_dist = np.linalg.norm(legwheel.O_r, axis=1)
    G_dist = np.linalg.norm(legwheel.G  , axis=1)
    U_dist = np.linalg.norm(legwheel.U_l, axis=1)
    L_dist = np.linalg.norm(legwheel.L_l, axis=1)
    inv_G_dist_coef = np.polyfit(G_dist, theta, 7)
    inv_U_dist_coef = np.polyfit(U_dist, theta, 7)
    inv_L_dist_coef = np.polyfit(L_dist, theta, 7)
    inv_O_r_dist_coef = np.polyfit(O_r_dist, theta, 7)
    end_time = time.time()  # end time
    print("Coefficient Fitting Time:", end_time - start_time, "seconds")
        
    #### Print fitted coefficients ####
    print(f'A_x_coef = {A_x_coef[::-1].tolist()}')
    print(f'A_y_coef = {A_y_coef[::-1].tolist()}')
    print(f'B_x_coef = {B_x_coef[::-1].tolist()}')
    print(f'B_y_coef = {B_y_coef[::-1].tolist()}')
    print(f'C_x_coef = {C_x_coef[::-1].tolist()}')
    print(f'C_y_coef = {C_y_coef[::-1].tolist()}')
    print(f'D_x_coef = {D_x_coef[::-1].tolist()}')
    print(f'D_y_coef = {D_y_coef[::-1].tolist()}')
    print(f'E_y_coef = {E_y_coef[::-1].tolist()}')
    print(f'F_x_coef = {F_x_coef[::-1].tolist()}')
    print(f'F_y_coef = {F_y_coef[::-1].tolist()}')
    print(f'G_y_coef = {G_y_coef[::-1].tolist()}')
    print(f'H_x_coef = {H_x_coef[::-1].tolist()}')
    print(f'H_y_coef = {H_y_coef[::-1].tolist()}')
    print(f'U_x_coef = {U_x_coef[::-1].tolist()}')
    print(f'U_y_coef = {U_y_coef[::-1].tolist()}')
    print(f'L_x_coef = {L_x_coef[::-1].tolist()}')
    print(f'L_y_coef = {L_y_coef[::-1].tolist()}')
    print(f'I_x_coef = {I_x_coef[::-1].tolist()}')
    print(f'I_y_coef = {I_y_coef[::-1].tolist()}')
    print(f'J_x_coef = {J_x_coef[::-1].tolist()}')
    print(f'J_y_coef = {J_y_coef[::-1].tolist()}')
    print(f'inv_G_dist_coef = {inv_G_dist_coef[::-1].tolist()}')
    print(f'inv_U_dist_coef = {inv_U_dist_coef[::-1].tolist()}')
    print(f'inv_L_dist_coef = {inv_L_dist_coef[::-1].tolist()}')
    print(f'inv_O_r_dist_coef = {inv_O_r_dist_coef[::-1].tolist()}')
