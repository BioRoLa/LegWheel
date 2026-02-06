import numpy as np
from legwheel.config import RobotParams

class CorgiLegKinematics:
    """
    3D Kinematics for the Corgi Leg-Wheel module with ABAD (Abduction/Adduction).
    
    Frames:
        {R}: Robot Frame (Origin at IMU/Chassis center, Forward: +X, Left: +Y, Up: +Z)
        {Mi}: Module Frame (Base for each limb, centered at the Roll Axis of the ABAD joint)
        {Li}: Leg Frame (Sagittal plane attached to ABAD output, rotates with gamma about Z_M)
    """
    
    def __init__(self, leg_index):
        """
        Initializes the kinematics model for a specific leg.
        """
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
        
        self.p_Mi_in_R = np.array([
            self.sx * 0.5 * self.l_body,
            self.sy * 0.5 * self.w_body,
            0.0 
        ])

        from legwheel.visualization.plot_leg import PlotLeg
        self.solver = PlotLeg()
        self.theta0 = self.solver.theta0
        self.beta0 = self.solver.beta0

    def _rot_z(self, angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def fk_sagittal(self, theta, beta, alpha=0.0):
        self.solver.forward(theta, beta, vector=True)
        p_contact = self.solver.rim_point(np.rad2deg(alpha))
        if isinstance(p_contact, np.ndarray):
            return np.array([p_contact[0], p_contact[1], 0.0])
        return np.array([p_contact.real, p_contact.imag, 0.0])

    def _transform_to_robot(self, p_L, gamma, is_wheel=True):
        if self.is_left:
            R_L_to_M = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
            R_M_to_R = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        else:
            R_L_to_M = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
            R_M_to_R = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])

        d_lat = self.d_abad + (self.d_wheel if is_wheel else 0.0)
        p_M = R_L_to_M @ p_L + np.array([d_lat, 0, 0])
        return R_M_to_R @ (self._rot_z(gamma) @ p_M) + self.p_Mi_in_R

    def forward_kinematics(self, theta, beta, gamma, alpha=0.0):
        p_L = self.fk_sagittal(theta, beta, alpha) 
        return self._transform_to_robot(p_L, gamma, is_wheel=True)

    def get_joint_positions(self, theta, beta, gamma):
        self.solver.forward(theta, beta, vector=True)
        lm = self.solver
        joints_2d = {'O':[0,0], 'A':lm.A_l, 'B':lm.B_l, 'C':lm.C_l, 'D':lm.D_l, 'E':lm.E, 'F':lm.F_l, 'G':lm.G}
        return {k: self._transform_to_robot(np.array([v[0], v[1], 0]), gamma, is_wheel=(k=='G')) for k, v in joints_2d.items()}

    def plot_leg_3d(self, theta, beta, gamma, ax):
        """
        Projects 2D PlotLeg geometry into 3D robot space.
        """
        self.solver.forward(theta, beta, vector=False)
        self.solver.leg_shape.get_shape([0, 0])
        shape = self.solver.leg_shape

        def project_and_plot(xdata, ydata, color, lw, is_w=False):
            pts_L = np.vstack([xdata, ydata, np.zeros_like(xdata)]).T
            pts_R = np.array([self._transform_to_robot(p, gamma, is_wheel=is_w) for p in pts_L])
            ax.plot(pts_R[:,0], pts_R[:,1], pts_R[:,2], color=color, linewidth=lw)

        for key, val in shape.__dict__.items():
            # 1. Handle Bars (Line2D)
            if "bar" in key and hasattr(val, 'get_xdata'):
                is_wheel = ('foot' in key or 'center' in key)
                project_and_plot(val.get_xdata(), val.get_ydata(), val.get_color(), val.get_linewidth(), is_w=is_wheel)
            
            # 2. Handle Rims (Custom rim objects with Arcs)
            elif "rim" in key and hasattr(val, 'arc'):
                is_wheel = ('lower' in key or 'foot' in key)
                for arc in val.arc:
                    # Sample points from the Arc
                    theta1, theta2 = np.deg2rad(arc.theta1), np.deg2rad(arc.theta2)
                    angles = np.linspace(theta1, theta2, 20)
                    # Center of arc is in [x, y] format from PlotLeg
                    cx, cy = arc.center 
                    r = arc.width / 2
                    x = cx + r * np.cos(angles)
                    y = cy + r * np.sin(angles)
                    project_and_plot(x, y, arc.get_edgecolor(), arc.get_linewidth(), is_w=is_wheel)

    def inverse_kinematics(self, target_pos, guess_q=None):
        if guess_q is None: guess_q = np.array([self.theta0, self.beta0, 0.0])
        q = guess_q
        for _ in range(10):
            err = target_pos - self.forward_kinematics(*q)
            if np.linalg.norm(err) < 1e-4: break
            d, J = 1e-5, np.zeros((3, 3))
            for j in range(3):
                q_d = q.copy(); q_d[j] += d
                J[:, j] = (self.forward_kinematics(*q_d) - self.forward_kinematics(*q)) / d
            q += np.linalg.pinv(J) @ err
        return q