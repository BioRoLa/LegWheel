import numpy as np

class Screw:
    """
    Representation of a Screw (Twist or Wrench) in 3D space.
    Used for applying Screw Theory to robotic kinematics and dynamics.
    
    A Twist V is represented as [omega, v] where omega is angular velocity
    and v is linear velocity.
    """
    def __init__(self, vec):
        """
        Initializes a Screw with a 6D vector.
        Args:
            vec (np.ndarray): 6D vector [omega_x, omega_y, omega_z, v_x, v_y, v_z]
        """
        self.vec = np.array(vec, dtype=float)
        self.omega = self.vec[:3]
        self.v = self.vec[3:]

    @classmethod
    def from_axis(cls, q, s, h):
        """
        Constructs a unit Screw from an axis point, direction, and pitch.
        Args:
            q (np.ndarray): A point on the screw axis.
            s (np.ndarray): Unit direction vector of the axis.
            h (float): Pitch of the screw (linear motion / angular motion).
        """
        s = s / np.linalg.norm(s)
        omega = s
        v = -np.cross(s, q) + h * s
        return cls(np.hstack((omega, v)))

    def skew_symmetric(self, w):
        """Returns the 3x3 skew-symmetric matrix of a 3D vector."""
        return np.array([[0, -w[2], w[1]],
                         [w[2], 0, -w[0]],
                         [-w[1], w[0], 0]])

    def to_matrix(self):
        """Returns the 4x4 se(3) matrix representation of the twist."""
        omega_mat = self.skew_symmetric(self.omega)
        se3 = np.zeros((4, 4))
        se3[:3, :3] = omega_mat
        se3[:3, 3] = self.v
        return se3

    def exp6(self, theta):
        """
        Exponential mapping from se(3) twist to SE(3) transformation matrix.
        Args:
            theta (float): The magnitude of the motion (rotation in radians or translation).
        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix.
        """
        omega = self.omega
        v = self.v
        
        if np.linalg.norm(omega) < 1e-9:
            # Pure translation
            T = np.eye(4)
            T[:3, 3] = v * theta
            return T
        
        # Rotation and translation (Rodrigues' Formula extension)
        w_mat = self.skew_symmetric(omega)
        R = np.eye(3) + np.sin(theta) * w_mat + (1 - np.cos(theta)) * (w_mat @ w_mat)
        
        V = np.eye(3) * theta + (1 - np.cos(theta)) * w_mat + (theta - np.sin(theta)) * (w_mat @ w_mat)
        p = V @ v
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p
        return T

    @staticmethod
    def adjoint(T):
        """Returns the 6x6 Adjoint transformation matrix of a 4x4 SE(3) matrix."""
        R = T[:3, :3]
        p = T[:3, 3]
        p_mat = np.array([[0, -p[2], p[1]],
                          [p[2], 0, -p[0]],
                          [-p[1], p[0], 0]])
        
        Adj = np.zeros((6, 6))
        Adj[:3, :3] = R
        Adj[3:, 3:] = R
        Adj[3:, :3] = p_mat @ R
        return Adj

    def transform(self, T):
        """
        Transforms the screw using an SE(3) transformation matrix (Adjoint map).
        Args:
            T (np.ndarray): 4x4 homogeneous transformation matrix.
        Returns:
            Screw: The transformed screw.
        """
        Adj = self.adjoint(T)
        return Screw(Adj @ self.vec)

    def __repr__(self):
        return f"Screw({self.vec})"

    def __add__(self, other):
        return Screw(self.vec + other.vec)

    def __mul__(self, scalar):
        return Screw(self.vec * scalar)
