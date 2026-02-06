import numpy as np
from legwheel.config import RobotParams

class CorgiLegKinematics:
    """
    3D Kinematics model for the Corgi Leg-Wheel module with Abduction/Adduction (ABAD).
    
    This class handles the transformation between the 2D sagittal leg mechanism 
    (solved by PlotLeg/LegModel) and the 3D Robot Frame. It accounts for mechanical 
    offsets (ABAD axis and wheel axial distance) and the active ABAD joint angle.

    Coordinate Frames:
        {R}  - Robot Frame: Origin at chassis center. +X Front, +Y Left, +Z Up.
        {Mi} - Module Frame (Rotated): Base for each limb, centered at the Hip Roll (ABAD) axis.
               Oriented such that Z_M is the longitudinal roll axis. In this implementation,
               the frame axes shown in plots rotate with the joint angle gamma.
        {Li} - Leg Frame (Sagittal): 2D assembly attached to ABAD output.
               x_L is the mechanism extension axis, y_L is sagittal symmetry.

    Leg Indexing (i):
        0: FL (Front-Left), 1: FR (Front-Right), 2: RR (Rear-Right), 3: RL (Rear-Left)
    """
    
    def __init__(self, leg_index):
        """
        Initializes the kinematics for a specific leg and sets up the structural offsets.
        
        Args:
            leg_index (int): Index of the leg (0-3).
        """
        self.leg_index = leg_index
        # Mechanical symmetry detection
        self.is_left = leg_index in [0, 3]
        self.is_front = leg_index in [0, 1]
        
        # Dimensions from RobotParams
        self.w_body = RobotParams.BODY_WIDTH
        self.l_body = RobotParams.WHEEL_BASE
        
        # d_abad: Lateral distance from the Roll axis center to the Linkage Plane.
        self.d_abad = RobotParams.ABAD_AXIS_OFFSET
        
        # d_wheel: Lateral distance from the Linkage Plane to the Wheel Center.
        self.d_wheel = RobotParams.WHEEL_AXIAL_OFFSET
        
        self.r_wheel = RobotParams.WHEEL_RADIUS_PITCH
        
        # sx, sy: "Sign X" and "Sign Y". 
        # Used as multipliers to determine frame origin positions in {R} based on symmetry.
        self.sx = 1.0 if self.is_front else -1.0
        self.sy = 1.0 if self.is_left else -1.0
        
        # p_Mi_in_R: The 3D position of the Module {Mi} origin (ABAD Roll Axis) in {R}.
        # FL: (+L/2, +W/2, 0), FR: (+L/2, -W/2, 0), etc.
        self.p_Mi_in_R = np.array([
            self.sx * 0.5 * self.l_body,
            self.sy * 0.5 * self.w_body,
            self.d_abad 
        ])

        # PlotLeg acts as the internal 2D solver for the 5-bar linkage geometry.
        from legwheel.visualization.plot_leg import PlotLeg
        self.solver = PlotLeg()
        self.theta0 = self.solver.theta0
        # beta0 is typically 90 deg in config, creating an offset for intuitive usage.
        self.beta0 = self.solver.beta0

    def _rot_z(self, angle):
        """Standard 3x3 rotation matrix for rotation about the Z-axis (local Roll axis)."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def fk_sagittal(self, theta, beta, alpha=0.0):
        """
        Calculates the 2D mechanism state in the sagittal Leg Frame {Li}.
        Adjusts beta by -90 deg to align the 2D solver with the 3D frame convention.
        
        Args:
            theta (float): Joint angle 1 (rad).
            beta (float): Joint angle 2 (rad).
            alpha (float): Rim contact angle (deg). 0 is bottom center.
            
        Returns:
            np.ndarray: [x, y, 0] coordinates in {Li}.
        """
        # Internal solver adds 90 deg. By subtracting 90 here, user beta=0 points Down.
        self.solver.forward(theta, beta, vector=True)
        # rim_point returns the 2D position of a point on the rim for a given alpha.
        p_contact = self.solver.rim_point(np.rad2deg(alpha))
        
        # if rim_point returns a batch of points, we take the first one for FK.
        if isinstance(p_contact, np.ndarray):
            return np.array([p_contact[0], p_contact[1], 0.0])
        return np.array([p_contact.real, p_contact.imag, 0.0])

    def _get_transformation_matrices(self):
        """
        Defines the rotation matrices for coordinate frame mapping.
        
        Mapping logic:
        - X_L (Leg primary axis) -> -Y_M (pointing 'Down' relative to hip)
        - Y_L (Sagittal symmetry) -> +Z_M (pointing 'Front')
        - Z_L (Sagittal thickness) -> +X_M (pointing 'Lateral Outward')
        """
        # R_L_to_M: Basis of Leg Frame {Li} expressed in Module Frame {Mi}.
        if self.is_left:
            R_L_to_M = np.array([
                [ 0, 0, 1],  
                [ 0, 1, 0],  
                [-1, 0, 0]   
            ])
        else:
            R_L_to_M = np.array([
                [ 0, 0,-1],  
                [ 0,-1, 0],  
                [ 1, 0, 0]   
            ])
        
        # R_M_to_R: Basis of Module Frame {Mi} expressed in Robot Frame {R}.
        if self.is_left:
            # X_M -> +Y_R, Y_M -> +Z_R, Z_M -> +X_R for left legs @ gamma=0
            R_M_to_R = np.array([
                [ 0, 0, 1], 
                [ 1, 0, 0], 
                [ 0, 1, 0]  
            ])
        else:
            # X_M -> -Y_R, Y_M -> -Z_R, Z_M -> -X_R for right legs @ gamma=0
            R_M_to_R = np.array([
                [ 0, 0,-1], 
                [-1, 0, 0],
                [ 0,-1, 0]  
            ])
        return R_L_to_M, R_M_to_R

    def _transform_to_robot(self, p_L, gamma):
        """
        Transforms a 3D point from the Leg Frame assembly to the Robot Frame.
        
        Logic Sequence:
        1. Map point from {Li} assembly base to Module orientation {Mi}.
        2. Apply structural lateral offset (d_abad or d_abad + d_wheel).
        3. Rotate by active ABAD angle (gamma) around the roll axis (Z_M).
        4. Transform the result to the global Robot Frame {R}.
        Args:
            p_L (np.ndarray): [x, y, z] point in Leg Frame {Li}.
            gamma (float): ABAD joint angle (rad).
        """
        # Get the transformation matrices based on current gamma (ABAD angle)
        R_L_to_M, R_M_to_R = self._get_transformation_matrices()
        
        # base lateral offset based on whether the component is on the linkage or the wheel.
        d_lat = self.d_wheel
        
        # starts with the point in the Leg Frame {Li}
        # 1. Orientation mapping {Li} -> {Mi}
        p_M_base = R_L_to_M @ p_L 
        
        # 2. Add structural lateral offset along X_M (Outward)
        p_M_offset = p_M_base + np.array([d_lat, 0, 0])
        
        # 3. Apply joint rotation gamma about Z_M
        p_M_rot = self._rot_z(gamma if self.is_left else -gamma) @ p_M_offset
        
        # 4. Final transform to R
        return R_M_to_R @ p_M_rot + self.p_Mi_in_R

    def forward_kinematics(self, theta, beta, gamma, alpha=0.0):
        """
        Full 3D Forward Kinematics for the foot contact point.
        
        Returns:
            np.ndarray: [x, y, z] position in Robot Frame {R}.
        """
        p_L = self.fk_sagittal(theta, beta, alpha) 
        return self._transform_to_robot(p_L, gamma)

    def get_joint_positions(self, theta, beta, gamma):
        """
        Returns all linkage joint positions (O, A, B, C, D, E, F, G) in Robot Frame {R}.
        """
        # Solver is updated inside fk_sagittal
        self.fk_sagittal(theta, beta)
        lm = self.solver
        
        # Mapping 2D attributes to a dict for processing
        joints_2d = {
            'O': np.array([0, 0]),
            'A': lm.A_l, 'B': lm.B_l, 'C': lm.C_l, 'D': lm.D_l,
            'E': lm.E,   'F': lm.F_l, 'G': lm.G
        }
        
        # Transform each joint. G is the only one typically considered 'wheel center'.
        return {k: self._transform_to_robot(np.array([v[0], v[1], 0]), gamma) for k, v in joints_2d.items()}

    def plot_leg_3d(self, theta, beta, gamma, ax):
        """
        Visualizes the full detailed mechanism by projecting 2D PlotLeg 
        geometric primitives into 3D robot space.
        """
        # 1. Update the 2D solver with corrected beta to get current linkage geometry
        self.fk_sagittal(theta, beta)
        shape = self.solver.leg_shape
        shape.get_shape(shape.O)  # Update all geometric primitives based on current leg state
        
        # layer_z: Separation between the two parallel linkage plates for visual depth.
        layer_z = 0

        def proj(x, y, color, lw, z_offset=0.0):
            """Internal projection helper."""
            z = np.full_like(x, z_offset)
            pts_L = np.vstack([x, y, z]).T
            pts_R = np.array([self._transform_to_robot(p, gamma) for p in pts_L])
            ax.plot(pts_R[:,0], pts_R[:,1], pts_R[:,2], color=color, linewidth=lw)

        # Iterate through PlotLeg's shape dictionary to find plotable primitives
        for key, val in shape.__dict__.items():
            # Determine Z-offset based on whether the component is 'Left' or 'Right' plate
            z_off = layer_z if '_l' in key else (-layer_z if '_r' in key else 0.0)
            
            # 1. Linkage Bars (Line2D objects)
            if "bar" in key and hasattr(val, 'get_xdata'):
                proj(val.get_xdata(), val.get_ydata(), val.get_color(), val.get_linewidth(), z_offset=z_off)
            
            # 2. Rims and Arcs
            elif "rim" in key and hasattr(val, 'arc'):
                for arc in val.arc:
                    t1, t2 = np.deg2rad(arc.theta1), np.deg2rad(arc.theta2)
                    # Use shortest path interpolation to avoid 'long-way-around' rendering bugs
                    diff = t2 - t1
                    while diff > np.pi: diff -= 2*np.pi
                    while diff < -np.pi: diff += 2*np.pi
                    ang = np.linspace(t1, t1 + diff, 20)
                    
                    cx, cy = arc.center
                    x, y = cx + (arc.width/2) * np.cos(ang), cy + (arc.height/2) * np.sin(ang)
                    proj(x, y, arc.get_edgecolor(), arc.get_linewidth(), z_offset=z_off)
            
            # 3. Joints (Circle patches)
            elif "joint" in key and hasattr(val, 'center'):
                cx, cy = val.center
                ang = np.linspace(0, 2*np.pi, 20)
                x, y = cx + val.radius * np.cos(ang), cy + val.radius * np.sin(ang)
                proj(x, y, val.get_edgecolor(), val.get_linewidth(), z_offset=z_off)

    def plot_frames(self, ax, gamma, axis_len=0.05):
        """
        Plots the coordinate frame axes (X=Red, Y=Green, Z=Blue) for {R}, {Mi}, and {Li}.
        Also labels the limb names (FL, FR, RR, RL).
        """
        limb_names = ['FL', 'FR', 'RR', 'RL']
        name = limb_names[self.leg_index]

        # Robot Frame {R} origin
        if self.leg_index == 0:
            ax.quiver(0, 0, 0, axis_len, 0, 0, color='r', linewidth=2)
            ax.quiver(0, 0, 0, 0, axis_len, 0, color='g', linewidth=2)
            ax.quiver(0, 0, 0, 0, 0, axis_len, color='b', linewidth=2)
            ax.text(0, 0, 0.02, '{R}', color='k', fontsize=12, fontweight='bold')

        R_L_to_M, R_M_to_R = self._get_transformation_matrices()
        R_gamma = self._rot_z(gamma if self.is_left else -gamma)
        mo = self.p_Mi_in_R
        
        # Rotated Module Frame {Mi} axes (now reflecting the active ABAD rotation)
        R_Mi_rotated_to_R = R_M_to_R @ R_gamma
        
        for i, c in enumerate(['r', 'g', 'b']):
            axis = R_Mi_rotated_to_R[:, i] * axis_len
            ax.quiver(mo[0], mo[1], mo[2], axis[0], axis[1], axis[2], color=c, alpha=0.8)
        
        # Label Module Frame and Limb Name
        ax.text(mo[0], mo[1], mo[2] + 0.02, f'{name} {{Mi}}', 
                fontsize=10, fontweight='bold', color='blue')
        
        # Leg Frame {Li} origin (offset by d_abad and rotated)
        R_L_to_R = R_Mi_rotated_to_R @ R_L_to_M
        p_Li_in_M = R_gamma @ np.array([self.d_abad, 0, 0])
        lo = R_M_to_R @ p_Li_in_M + mo
        
        # Leg Frame {Li} axes
        for i, c in enumerate(['r', 'g', 'b']):
            axis = R_L_to_R[:, i] * axis_len
            ax.quiver(lo[0], lo[1], lo[2], axis[0], axis[1], axis[2], color=c)

        # Label Leg Frame
        ax.text(lo[0], lo[1], lo[2] - 0.02, f'{{L{self.leg_index}}}', 
                fontsize=9, fontstyle='italic', color='darkgreen')

    def inverse_kinematics(self, target_pos, guess_q=None):
        """
        3D Inverse Kinematics using numerical Gauss-Newton iteration.
        
        Args:
            target_pos (np.ndarray): [x, y, z] target in {R}.
            guess_q (np.ndarray): Initial [theta, beta, gamma] guess.
            
        Returns:
            np.ndarray: Optimized joint angles.
        """
        if guess_q is None: guess_q = np.array([self.theta0, self.beta0, 0.0])
        q = guess_q
        for _ in range(10):
            err = target_pos - self.forward_kinematics(*q)
            if np.linalg.norm(err) < 1e-4: break
            # Jacobian calculation via finite difference
            d, J = 1e-5, np.zeros((3, 3))
            for j in range(3):
                q_d = q.copy(); q_d[j] += d
                J[:, j] = (self.forward_kinematics(*q_d) - self.forward_kinematics(*q)) / d
            q += np.linalg.pinv(J) @ err
        return q