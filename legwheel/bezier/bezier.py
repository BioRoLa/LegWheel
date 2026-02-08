import numpy as np
from math import pow
import matplotlib.pyplot as plt


class Bezier:
    """
    Implements a general n-th order Bezier curve in N dimensions.
    
    A Bezier curve is defined by a set of control points. The curve passes 
    through the first and last control points and is contained within the 
    convex hull of all control points.
    """
    def __init__(self, control_pts):
        """
        Initializes the Bezier curve with a list of control points.

        Args:
            control_pts (list of np.ndarray): List of control points [x, y, ...].
        """
        self.control_pts = control_pts
        # Pre-calculate the binomial coefficients (n choose k) for the given order
        self.bz_cff = self.bz_coeff(control_pts)

    def fact(self, n):
        """Calculates the factorial of n."""
        return 1 if (n == 0) or (n == 1) else n * self.fact(n - 1)

    def comb(self, n, k):
        """
        Calculates the binomial coefficient 'n choose k'.
        Used for Bernstein basis polynomials.
        """
        return self.fact(n) // (self.fact(k) * self.fact(n - k))

    def bz_coeff(self, cp_list):
        """
        Calculates the static part of Bernstein polynomials (the binomial coefficients).
        
        Args:
            cp_list (list): List of control points to determine the order (n = len - 1).
        """
        sz = len(cp_list)
        bzc = []
        for i in range(sz):
            bzc.append(self.comb(sz - 1, i))
        return bzc

    def bzt_coeff(self, cp_list, t):
        """
        Calculates the time-dependent part of Bernstein polynomials: (1-t)^(n-i) * t^i.
        
        Args:
            cp_list (list): List of control points.
            t (float): Normalized time parameter [0, 1].
        """
        sz = len(cp_list)
        bzc = []
        for i in range(sz):
            ord_t_1 = (sz - 1) - i
            ord_t = i
            bzc.append(pow((1 - t), ord_t_1) * pow(t, ord_t))
        return bzc

    def getBzPoint(self, t, offset_x=0, offset_y=0, offset_z=0):
        """
        Calculates the coordinates of a point on the Bezier curve at time t.
        Supports 2D and 3D curves.
        
        The point is calculated using the formula:
        P(t) = sum_{i=0}^{n} B_{i,n}(t) * P_i + Offset

        Args:
            t (float): Normalized time parameter [0, 1].
            offset_x (float): Translation along X.
            offset_y (float): Translation along Y.
            offset_z (float): Translation along Z (defaults to 0 for 2D).

        Returns:
            np.ndarray: The [x, y, (z)] position on the curve.
        """
        bzt_cff = self.bzt_coeff(self.control_pts, t)
        
        # Determine dimensionality from the first control point
        dim = len(self.control_pts[0])
        point = np.zeros(dim)
        
        for i in range(len(self.control_pts)):
            weight = bzt_cff[i] * self.bz_cff[i]
            point += weight * self.control_pts[i]
        
        # Apply offsets
        point[0] += offset_x
        if dim > 1: point[1] += offset_y
        if dim > 2: point[2] += offset_z
            
        return point


if __name__ == "__main__":
    # Test 2D
    cp_2d = [np.array([0, 0]), np.array([1, 2]), np.array([3, 0])]
    bz_2d = Bezier(cp_2d)
    print(f"2D Point t=0.5: {bz_2d.getBzPoint(0.5)}")

    # Test 3D
    cp_3d = [np.array([0, 0, 0]), np.array([1, 2, 1]), np.array([3, 0, 0])]
    bz_3d = Bezier(cp_3d)
    print(f"3D Point t=0.5: {bz_3d.getBzPoint(0.5, offset_z=1.0)}")
