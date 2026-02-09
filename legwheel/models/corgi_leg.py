import numpy as np
from legwheel.config import RobotParams

class CorgiLegKinematics:
    """
    3D Kinematics model for the Corgi Leg-Wheel module with Abduction/Adduction (ABAD).
    
    This class handles the transformation between the 2D sagittal leg mechanism 
    (solved by PlotLeg/LegModel) and the 3D Body Frame. It accounts for mechanical 
    offsets (ABAD axis and wheel axial distance) and the active ABAD joint angle.

    Coordinate Frames:
        {B}  - Body Frame: Origin at chassis center. +X Front, +Y Left, +Z Up.
        {Mi} - Module Frame (Rotated): Base for each limb, centered at the Hip Roll (ABAD) axis.
               Oriented such that Z_M is the longitudinal roll axis. In this implementation,
               the frame axes shown in plots rotate with the joint angle gamma.
        {Li} - Leg Frame (Sagittal): 2D assembly attached to ABAD output.
               x_L is the mechanism extension axis, y_L is sagittal symmetry.

    Leg Indexing (i):
        0: FL (Front-Left), 1: FR (Front-Right), 2: RR (Rear-Right), 3: RL (Rear-Left)
    """
    
    def __init__(self, leg_index, gamma=0.0):
        """
        Initializes the kinematics for a specific leg and sets up the structural offsets.
        
        Args:
            leg_index (int): Index of the leg (0-3).
            gamma (float): ABAD joint angle (rad).
        """
        self.leg_index = leg_index
        self.gamma = gamma
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
        
        # r_wheel: Effective radius for kinematics (distance from ABAD axis to ground contact point).
        self.r_wheel = RobotParams.WHEEL_RADIUS_PITCH
        
        # wheel_thickness: Thickness of the wheel for visualization and collision purposes.
        self.wheel_thickness = RobotParams.WHEEL_THICKNESS
        
        # sx, sy: "Sign X" and "Sign Y". 
        # Used as multipliers to determine frame origin positions in {B} based on symmetry.
        self.sx = 1.0 if self.is_front else -1.0
        self.sy = 1.0 if self.is_left else -1.0
        
        # p_Mi_in_B: The 3D position of the Module {Mi} origin (ABAD Roll Axis) in {B}.
        # FL: (+L/2, +W/2, 0), FR: (+L/2, -W/2, 0), etc.
        self.p_Mi_in_B = np.array([
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
        p_contact = self.solver.rim_point(alpha)
        
        # if rim_point returns a batch of points, we take the first one for FK.
        if isinstance(p_contact, np.ndarray):
            return np.array([p_contact[0], p_contact[1], 0.0])
        return np.array([p_contact.real, p_contact.imag, 0.0])

    def _get_transformation_matrices(self, gamma=None):
        """
        Defines the rotation matrices for coordinate frame mapping.
        
        Mapping logic:
        - X_L (Leg primary axis) -> -Y_M (pointing 'Down' relative to hip)
        - Y_L (Sagittal symmetry) -> +Z_M (pointing 'Front')
        - Z_L (Sagittal thickness) -> +X_M (pointing 'Lateral Outward')
        """
        if gamma is None: gamma = self.gamma
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
        
        # R_M_to_B: Basis of Module Frame {Mi} expressed in Body Frame {B}.
        if self.is_left:
            # X_M -> +Y_R*cos(\gamma) + +Z_R *sin(\gamma),
            # Y_M -> +Z_R*cos(\gamma) - +Y_R*sin(\gamma),
            # Z_M -> +X_R for left legs
            R_M_to_B = np.array([
                [ 0             , 0             , 1], 
                [ np.cos(gamma) ,-np.sin(gamma) , 0], 
                [ np.sin(gamma) , np.cos(gamma) , 0]  
            ])
        else:
            # X_M -> -Y_R*cos(\gamma) + +Z_R *sin(\gamma),
            # Y_M -> -Z_R*cos(\gamma) - +Y_R *sin(\gamma),
            # Z_M -> -X_R for right legs
            R_M_to_B = np.array([
                [ 0             , 0             ,-1], 
                [-np.cos(gamma) ,-np.sin(gamma) , 0],
                [ np.sin(gamma) ,-np.cos(gamma) , 0]  
            ])
        
        # Transform matrices T:
        # | R | offset |
        # |---|--------|
        # | 0 |   1    |
        
        # T_L_to_M includes the rotation and the translation from Leg Frame to Module Frame
        T_L_to_M = np.eye(4)
        T_L_to_M[:3, :3] = R_L_to_M
        offset_L_to_M = np.array([self.d_wheel, 0, 0])  # Lateral offset from Leg Frame to Module Frame
        T_L_to_M[:3, 3] = offset_L_to_M
        T_L_to_M[3, 3] = 1
        
        # T_M_to_R includes the rotation and the translation from Module Frame to Robot Frame
        T_M_to_B = np.eye(4)
        T_M_to_B[:3, :3] = R_M_to_B
        T_M_to_B[:3, 3] = self.p_Mi_in_B
        T_M_to_B[3, 3] = 1
        return T_L_to_M, T_M_to_B
    
    def _L_to_M(self, p_L, gamma=None):
        """
        Transforms a point from Leg Frame {Li} to Module Frame {Mi}.
        Args:
            p_L (np.ndarray): [x, y, z] point in Leg Frame {Li}.
        Returns:
            np.ndarray: [x, y, z] point in Module Frame {Mi}.
        """
        T_L_to_M, _ = self._get_transformation_matrices(gamma)
        # append scale factor to set p_l in to 1x4 homogeneous coordinates for transformation
        p_L_homogeneous = np.append(p_L, 1)  # Convert to homogeneous coordinates
        p_M = T_L_to_M @ p_L_homogeneous
        return p_M[:3]  # Return only the x, y, z components
    
    def _M_to_B(self, p_M, gamma=None):
        """
        Transforms a point from Module Frame {Mi} to Body Frame {B}.
        Args:
            p_M (np.ndarray): [x, y, z] point in Module Frame {Mi}.
        Returns:
            np.ndarray: [x, y, z] point in Body Frame {B}.
        """
        _, T_M_to_B = self._get_transformation_matrices(gamma)
        p_M_homogeneous = np.append(p_M, 1)  # Convert to homogeneous coordinates
        p_B = T_M_to_B @ p_M_homogeneous
        return p_B[:3]  # Return only the x, y, z components
        
    def _transform_to_body(self, p_L, gamma=None):
        """
        Transforms a 3D point from the Leg Frame assembly to the Body Frame.
        
        Logic Sequence:
        1. Map point from {Li} assembly base to Module orientation {Mi}.
        3. Transform the result to the global Body Frame {B}.
        Args:
            p_L (np.ndarray): [x, y, z] point in Leg Frame {Li}.
            gamma (float): ABAD joint angle (rad).
        """
        
        # starts with the point in the Leg Frame {Li}
        # 1. Orientation mapping {Li} -> {Mi}
        p_M = self._L_to_M(p_L, gamma)
        p_B = self._M_to_B(p_M, gamma)
        return p_B

    def forward_kinematics(self, theta, beta, gamma=None, alpha=0.0, w=0.0):
        """
        Full 3D Forward Kinematics for the foot contact point.
        Args:
            theta (float): Joint angle 1 (rad).
            beta (float) : Joint angle 2 (rad).
            gamma (float): ABAD joint angle (rad).
            alpha (float): Rim contact angle (deg). 0 is bottom center.
            w (float)    : Contact point depth(mm). ±20 mm from the mid surface.
        Returns:
            np.ndarray: [x, y, z] position in Body Frame {B}.
        """
        p_L = self.fk_sagittal(theta, beta, alpha) + np.array([0, 0, w])
        return self._transform_to_body(p_L, gamma)

    def get_joint_positions(self, theta, beta, gamma=None):
        """
        Returns all linkage joint positions (O, A, B, C, D, E, F, G) in Body Frame {B}.
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
        return {k: self._transform_to_body(np.array([v[0], v[1], 0]), gamma) for k, v in joints_2d.items()}

    def plot_leg_in_3d_plane(self, theta, beta, gamma=None,
                             z_offset = 0.0, ax=None,
                             select_components=["bars", "rims", "joints"]):
        """
        Visualizes the full detailed mechanism by projecting 2D PlotLeg into specific planes in 3D space.
        This method uses the internal geometry of the PlotLeg solver to render the geometric primitives into 3D Body space.
        """
        # 1. Update the 2D solver with corrected beta to get current linkage geometry
        self.fk_sagittal(theta, beta)
        shape = self.solver.leg_shape
        shape.get_shape(shape.O)  # Update all geometric primitives based on current leg state

        def proj(x, y, color, lw, z_offset=z_offset):
            """Internal projection helper."""
            z = np.full_like(x, z_offset)
            pts_L = np.vstack([x, y, z]).T
            pts_B = np.array([self._transform_to_body(p, gamma) for p in pts_L])
            ax.plot(pts_B[:,0], pts_B[:,1], pts_B[:,2], color=color, linewidth=lw)

        # Iterate through PlotLeg's shape dictionary to find plotable primitives
        for key, val in shape.__dict__.items():
            # Determine Z-offset based on whether the component is 'Left' or 'Right' plate
            z_off = z_offset
            
            # 1. Linkage Bars (Line2D objects)
            if "bar" in key and hasattr(val, 'get_xdata') and "bars" in select_components:
                proj(val.get_xdata(), val.get_ydata(), val.get_color(), val.get_linewidth(), z_offset=z_off)
            
            # 2. Rims and Arcs
            elif "rim" in key and hasattr(val, 'arc') and "rims" in select_components:
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
            elif "joint" in key and hasattr(val, 'center') and "joints" in select_components:
                cx, cy = val.center
                ang = np.linspace(0, 2*np.pi, 20)
                x, y = cx + val.radius * np.cos(ang), cy + val.radius * np.sin(ang)
                proj(x, y, val.get_edgecolor(), val.get_linewidth(), z_offset=z_off)
                
    def plot_leg_3d(self, theta, beta, gamma=None, ax=None):
        """
        Wrapper to plot the leg in 3D with appropriate z-offsets for different components.
        This method calls plot_leg_in_3d_plane multiple times to layer the main linkage and the wheel rim with a slight offset to prevent visual overlap.
        """
        # Plot the mechanism two times with tyre thickness spacing for rim and joints
        self.plot_leg_in_3d_plane(theta, beta, z_offset= self.wheel_thickness/2, gamma=gamma, ax=ax, select_components=["rims", "joints"])
        self.plot_leg_in_3d_plane(theta, beta, z_offset=-self.wheel_thickness/2, gamma=gamma, ax=ax, select_components=["rims", "joints"])
        self.plot_leg_in_3d_plane(theta, beta, z_offset=0.0, gamma=gamma, ax=ax, select_components=["bars"])
        
        # Add lines connecting the two rim layers to visualize the wheel thickness
        if ax is not None:
            # set rim points in range ±π, to ensure correct rendering
            alpha_spacing = np.linspace(-180, 180, 100) # 100 points around the rim
            for alpha in alpha_spacing:
                # Calculate the rim point in the Leg Frame and transform it to Robot Frame for both top and bottom layers
                rim_point_pos_offset = self.forward_kinematics(theta, beta, gamma, alpha=alpha, w=self.wheel_thickness/2)
                rim_point_neg_offset = self.forward_kinematics(theta, beta, gamma, alpha=alpha, w=-self.wheel_thickness/2)
                ax.plot([rim_point_pos_offset[0], rim_point_neg_offset[0]],
                        [rim_point_pos_offset[1], rim_point_neg_offset[1]],
                        [rim_point_pos_offset[2], rim_point_neg_offset[2]], color='gray', linewidth=2, alpha=0.5)
        
    def plot_frames(self, ax, gamma=None, axis_len=0.05):
        """
        Plots the coordinate frame axes (X=Red, Y=Green, Z=Blue) for {R}, {Mi}, and {Li}.
        Also labels the limb names (FL, FR, RR, RL).
        """
        limb_names = ['FL', 'FR', 'RR', 'RL']
        name = limb_names[self.leg_index]

        # Robot Frame {B} origin
        if self.leg_index == 0:
            ax.quiver(0, 0, 0, axis_len, 0, 0, color='r', linewidth=2) # X_B
            ax.quiver(0, 0, 0, 0, axis_len, 0, color='g', linewidth=2) # Y_B
            ax.quiver(0, 0, 0, 0, 0, axis_len, color='b', linewidth=2) # Z_B
            ax.text(0, 0, 0.02, '{B}', color='k', fontsize=12, fontweight='bold')

        T_L_to_M, T_M_to_B = self._get_transformation_matrices(gamma=gamma)
        mo = self._M_to_B(np.array([0,0,0]), gamma)                     # Origin of Module Frame {Mi} in Body Frame {B}
        lo = self._transform_to_body(np.array([0,0,0]), gamma)          # Origin of Leg Frame {Li} in Body Frame {B}
        
        R_M_to_B = T_M_to_B[:3, :3]
        for i, c in enumerate(['r', 'g', 'b']):
            # Module Frame {Mi} axes
            axis = R_M_to_B[:, i] * axis_len
            # Plot the Module Frame axes from the module origin in Robot Frame
            ax.quiver(mo[0], mo[1], mo[2], axis[0], axis[1], axis[2], color=c, alpha=0.8)
        
        # Label Module Frame and Limb Name
        ax.text(mo[0], mo[1], mo[2] + 0.02, f'{name} {{Mi}}', fontsize=10, fontweight='bold', color='blue')
        
        # Leg Frame {Li} origin (offset by d_abad and rotated)
        R_L_to_B = R_M_to_B @ T_L_to_M[:3, :3]
        # Leg Frame {Li} axes
        for i, c in enumerate(['r', 'g', 'b']):
            # Plot the Leg Frame axes from the leg origin in Robot Frame
            axis = R_L_to_B[:, i] * axis_len
            # Plot the Leg Frame axes from the leg origin in Robot Frame
            ax.quiver(lo[0], lo[1], lo[2], axis[0], axis[1], axis[2], color=c)

        # Label Leg Frame
        ax.text(lo[0], lo[1], lo[2] - 0.02, f'{{L{self.leg_index}}}', 
                fontsize=9, fontweight='bold', color='darkgreen')

    def inverse_kinematics(self, target_pos, guess_q=None, rim_point=(0.0, 0.0)):
        """
        3D Inverse Kinematics using numerical Gauss-Newton iteration.
        assign specific rim point to ensure the IK solution corresponds to a desired contact point on the wheel.
        Args:
            target_pos (np.ndarray): [x, y, z] target in {R}.
            guess_q (np.ndarray): Initial [theta, beta, gamma] guess.
            rim_point (tuple): (alpha, w) rim contact parameters.
            
        Returns:
            np.ndarray: Optimized joint angles.
        """
        if guess_q is None: guess_q = np.array([self.theta0, self.beta0, 0.0])
        q = guess_q
        alpha, w = rim_point
        for _ in range(10):
            err = target_pos - self.forward_kinematics(*q, alpha=alpha, w=w)
            if np.linalg.norm(err) < 1e-4: break
            # Jacobian calculation via finite difference
            d, J = 1e-5, np.zeros((3, 3))
            for j in range(3):
                q_d = q.copy(); q_d[j] += d
                J[:, j] = (self.forward_kinematics(*q_d, alpha=alpha, w=w) - self.forward_kinematics(*q, alpha=alpha, w=w)) / d
            q += np.linalg.pinv(J) @ err
        return q
    
    def set_gamma(self, gamma):
        """Updates the ABAD angle and recalculates the module position."""
        self.gamma = gamma
        # Recalculate the module position in Robot Frame based on new gamma
        self.p_Mi_in_R = self._M_to_R(np.array([0, 0, self.d_abad]), gamma)