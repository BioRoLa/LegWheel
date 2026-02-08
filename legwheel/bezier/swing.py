import numpy as np
import nlopt
import time

if __name__ == "__main__":
    from bezier import *
else:
    from .bezier import *
    
    
class SwingProfile:
    """
    Represents a specific swing trajectory curve based on a 12-point Bezier curve.
    Supports both 2D (x, y) and 3D (x, y, z) trajectories.
    """
    def __init__(self, L, h, dh, dL1, dL2, dL3, dL4, offset_x=0, offset_y=0, offset_z=0, diff_h=0, diff_lat=0):
        """
        Initializes the SwingProfile.

        Args:
            L (float): Step length (forward distance).
            h (float): Step height (clearance).
            dh (float): Secondary height adjustment.
            dL1-dL4 (float): Horizontal control parameters.
            offset_x, offset_y, offset_z (float): Start position offsets.
            diff_h (float): Height difference (End Height - Start Height).
            diff_lat (float): Lateral difference (End Lateral - Start Lateral).
        """
        self.L = L
        self.h = h
        self.dh = dh
        self.dL1 = dL1
        self.dL2 = dL2
        self.dL3 = dL3
        self.dL4 = dL4
        
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z
        self.diff_h = diff_h
        self.diff_lat = diff_lat
        
        self.getControlPoint()
        self.bezier = Bezier(self.control_points)

    def getControlPoint(self):
        """
        Generates 3D control points: [Forward, Height, Lateral].
        """
        # Interpolate lateral movement across control points
        # 0..11 points. Simple linear interpolation for z-component.
        z_vals = np.linspace(0, self.diff_lat, 12)

        # Base 2D points (Forward, Height)
        # Note: Original code used [x, y] where y is height.
        c0 = np.array([0, 0])
        c1 = c0 - np.array([self.dL1, 0])
        c2 = c1 - np.array([self.dL2, 0]) + np.array([0, self.h])
        c3 = c2
        c4 = c2
        c5 = c4 + np.array([0.5 * self.L + self.dL1 + self.dL2, 0])
        c6 = c5
        c7 = c5 + np.array([0, self.dh]) + np.array([0.5 * self.L + self.dL3 + self.dL4, 0])
        c8 = c7 
        c9 = c8
        c10 = c8 - np.array([self.dL4, self.h + self.dh]) + np.array([0, self.diff_h])
        c11 = c10 - np.array([self.dL3, 0])
        
        raw_2d = [c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11]
        
        # Convert to 3D: [x, y, z] -> [Forward, Height, Lateral]
        self.control_points = [np.array([p[0], p[1], z_vals[i]]) for i, p in enumerate(raw_2d)]

    def getFootendPoint(self, t_duty):
        """
        Returns [x, y, z] position.
        x: Forward, y: Height, z: Lateral
        """
        return self.bezier.getBzPoint(t_duty, self.offset_x, self.offset_y, self.offset_z)


class SwingLegPlanner:
    """
    Planner for calculating optimal foot swing trajectories in 3D.
    """
    def __init__(self, dt, T_sw, T_st):
        self.dt = dt
        self.T_sw = T_sw
        self.T_st = T_st

        self.step_L = 0.1
        self.step_h = 0.05
        self.step_dh = 0.01
        self.v_liftoff = np.array([0, 0, 0]) # 3D velocity
        self.v_touchdown = np.array([0, 0, 0])

        self.opt = None
        self.opt_lb = np.array([-0.0, -0.0])
        self.opt_ub = np.array([0.5, 0.5])
        self.optimizerSetup()
        self.dL1_preset = 0.05
        self.dL2_preset = -0.0
        self.dL3_preset = 0.05
        self.dL4_preset = -0.0

    def optimizerSetup(self):
        self.opt = nlopt.opt(nlopt.LN_COBYLA, 2)
        self.opt.set_xtol_abs(1e-5)
        self.opt.set_maxeval(40)
        self.opt.set_upper_bounds(self.opt_ub)
        self.opt.set_lower_bounds(self.opt_lb)

    def solveSwingTrajectory(self, p_lo, p_td, step_h, v_lo, v_td):
        """
        Computes optimal swing trajectory. Accepts 2D or 3D points.
        """
        # Ensure 3D vectors
        if len(p_lo) == 2: p_lo = np.append(p_lo, 0)
        if len(p_td) == 2: p_td = np.append(p_td, 0)
        if len(v_lo) == 2: v_lo = np.append(v_lo, 0)
        if len(v_td) == 2: v_td = np.append(v_td, 0)

        # step_L is now the horizontal plan distance (Forward-Lateral magnitude)
        # But SwingProfile L parameter is strictly "Forward" distance in the local frame defined by start/end.
        # Simplification: We treat L as the norm of horizontal displacement.
        # But we need to separate Height (y) from Plane (x, z).
        
        # In SwingProfile, index 1 is Height.
        # Let's map inputs:
        # Input p_lo: [x, y, z] (Global)
        # We assume y is height? Or z?
        # Standard: z is height? NO, standard 2D code used y as height.
        # Let's assume input is [Forward, Height, Lateral] to match SwingProfile.
        
        # Calculate deltas
        diff = p_td - p_lo
        L_forward = diff[0]
        diff_h = diff[1]
        diff_lat = diff[2]
        
        # Total "Step Length" for optimization constraints usually refers to forward progress
        self.step_L = L_forward 
        self.step_h = step_h
        self.step_dh = 0.01
        self.v_liftoff = v_lo
        self.v_touchdown = v_td

        # Run Optimization (only depends on Forward/Height dynamics usually)
        self.opt.set_min_objective(self.objectiveFunc_lo)
        self.opt.add_inequality_constraint(self.constraint_lo)
        x_lo_opt = self.opt.optimize(np.array([self.dL1_preset, self.dL2_preset]))

        self.optimizerSetup()
        self.opt.set_min_objective(self.objectiveFunc_td)
        self.opt.add_inequality_constraint(self.constraint_td)
        x_td_opt = self.opt.optimize(np.array([self.dL3_preset, self.dL4_preset]))

        return SwingProfile(
            self.step_L,
            self.step_h,
            self.step_dh,
            x_lo_opt[0],
            x_lo_opt[1],
            x_td_opt[0],
            x_td_opt[1],
            offset_x=p_lo[0],
            offset_y=p_lo[1],
            offset_z=p_lo[2],
            diff_h=diff_h,
            diff_lat=diff_lat
        )

    # Objectives calculate velocity error. Need to handle 3D velocity norm.
    def objectiveFunc_lo(self, x, grad):
        sp = SwingProfile(self.step_L, self.step_h, self.step_dh, x[0], x[1], self.dL3_preset, self.dL4_preset)
        # Finite difference velocity check
        p0 = sp.getFootendPoint(0.0)
        p1 = sp.getFootendPoint(0.001 / self.T_sw)
        v_calc = (p1 - p0) / self.dt
        
        # Optimization only cares about matching the provided velocity profile
        # We compare 3D velocity vectors
        return np.linalg.norm(self.v_liftoff - v_calc)

    def objectiveFunc_td(self, x, grad):
        sp = SwingProfile(self.step_L, self.step_h, self.step_dh, self.dL1_preset, self.dL2_preset, x[0], x[1])
        p2 = sp.getFootendPoint(1.0 - (0.001 / self.T_sw))
        p3 = sp.getFootendPoint(1.0)
        v_calc = (p3 - p2) / self.dt
        return np.linalg.norm(self.v_touchdown - v_calc)

    def constraint_lo(self, x, grad):
        return -x[0] - x[1] - (self.step_L / 2 - 0.01)

    def constraint_td(self, x, grad):
        return (self.step_L / 2 + 0.01) - (self.step_L + x[0] + x[1])


if __name__ == "__main__":
    # Test 3D Trajectory
    planner = SwingLegPlanner(0.01, 0.6, 1.8)
    p_start = np.array([0, 0, 0])
    p_end = np.array([0.3, 0.05, 0.1]) # Forward 0.3, Up 0.05, Lateral 0.1
    v_start = np.array([0.5, 0, 0])
    v_end = np.array([0, -0.1, 0])
    
    profile = planner.solveSwingTrajectory(p_start, p_end, 0.1, v_start, v_end)
    
    t = np.linspace(0, 1, 50)
    path = np.array([profile.getFootendPoint(ti) for ti in t])
    
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(path[:,0], path[:,2], path[:,1], label='Swing 3D') # Plot X-Z-Y (Front, Lat, Height)
    ax.set_xlabel('Forward (X)')
    ax.set_ylabel('Lateral (Z)')
    ax.set_zlabel('Height (Y)')
    plt.show()