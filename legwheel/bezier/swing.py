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
    
    The local frame for the profile is:
    - x: Forward progress (from 0 to L)
    - y: Height profile (max height h)
    - z: Lateral progress (from 0 to diff_lat)
    """
    def __init__(self, L, h, dh, dL1, dL2, dL3, dL4, dH1=0.0, dH2=0.0,
                 offset_x=0, offset_y=0, offset_z=0, diff_h=0, diff_lat=0):
        """
        Initializes the SwingProfile with geometric parameters.

        Args:
            L (float): Total step length (horizontal forward distance).
            h (float): Maximum step height (clearance).
            dh (float): Secondary height adjustment for mid-trajectory.
            dL1 (float): Horizontal control for lift-off acceleration.
            dL2 (float): Horizontal control for mid-lift shape.
            dL3 (float): Horizontal control for mid-touchdown shape.
            dL4 (float): Horizontal control for touchdown deceleration.
            dH1 (float): Vertical offset at c1 — controls Height velocity at liftoff.
                         c1.y = dH1 ≥ 0; c2.y is adjusted to keep peak at h.
            dH2 (float): Vertical offset at c10 — controls Height velocity at touchdown.
                         c10.y = diff_h + dH2; c11.y = diff_h (arrives from above).
            offset_x, offset_y, offset_z (float): Start position offsets in world frame.
            diff_h (float): Vertical height difference between start and end.
            diff_lat (float): Lateral difference between start and end.
        """
        self.L = L
        self.h = h
        self.dh = dh
        self.dL1 = dL1
        self.dL2 = dL2
        self.dL3 = dL3
        self.dL4 = dL4
        self.dH1 = dH1   # liftoff height offset (c1.y)
        self.dH2 = dH2   # touchdown height offset (c10/c11 y-gap)
        
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z
        self.diff_h = diff_h
        self.diff_lat = diff_lat
        
        self.control_points = []
        self.getControlPoint()
        self.bezier = Bezier(self.control_points)

    def getControlPoint(self):
        """
        Generates 12 3D control points: [Forward, Height, Lateral].
        Interpolates lateral movement linearly across the curve.

        Height boundary fix (2026-04-14):
          c1.y = dH1 ≥ 0  allows non-zero Height velocity at liftoff.
          c2.y is adjusted to (h - dH1) so the trajectory peak stays at h.
          c10.y = diff_h + dH2, c11.y = diff_h  gives downward touchdown velocity.
        """
        # Linear interpolation for lateral z-component across 12 points
        z_vals = np.linspace(0, self.diff_lat, 12)

        # Sequence mapping:
        # c0: Lift-off (height = 0, ground level)
        # c1: Lift-off velocity control — dH1 allows upward height velocity
        # c2-c4: Peak height maintenance at h (adjusted for dH1)
        # c5-c6: Mid-swing at height h
        # c10-c11: Descent and touchdown — dH2 gives downward touch velocity
        c0  = np.array([0, 0])
        c1  = c0  - np.array([self.dL1, 0])          + np.array([0, self.dH1])
        c2  = c1  - np.array([self.dL2, 0])          + np.array([0, self.h - self.dH1])
        c3  = c2
        c4  = c2
        c5  = c4  + np.array([0.5 * self.L + self.dL1 + self.dL2, 0])
        c6  = c5
        c7  = c5  + np.array([0, self.dh]) + np.array([0.5 * self.L + self.dL3 + self.dL4, 0])
        c8  = c7
        c9  = c8
        c10 = c8  - np.array([self.dL4, self.h + self.dh]) + np.array([0, self.diff_h + self.dH2])
        c11 = c10 - np.array([self.dL3, 0])          - np.array([0, self.dH2])

        raw_2d = [c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11]

        # Merge 2D [x, height] with interpolated lateral [z]
        self.control_points = [np.array([p[0], p[1], z_vals[i]]) for i, p in enumerate(raw_2d)]

    def getFootendPoint(self, t_duty):
        """
        Calculates position at normalized time t.
        Returns: [x, height, lateral]
        """
        pt = self.bezier.getBzPoint(t_duty, self.offset_x, self.offset_y, self.offset_z)
        # Apply backward-step mirroring if needed
        if hasattr(self, '_step_sign') and self._step_sign < 0:
            # x was solved in |L| space relative to offset_x=0.
            # Mirror: x_true = offset_x_true - x_local (where x_local = pt[0])
            pt[0] = self._offset_x_true - pt[0]
        elif hasattr(self, '_offset_x_true'):
            pt[0] = self._offset_x_true + pt[0]
        return pt


class SwingLegPlanner:
    """
    Planner for computing optimal swing trajectories in 3D.
    Uses NLopt optimization to match desired lift-off and touchdown velocities.
    """
    def __init__(self, dt, T_sw, T_st):
        """
        Args:
            dt (float): Time step (s).
            T_sw (float): Swing duration (s).
            T_st (float): Stance duration (s).
        """
        self.dt = dt
        self.T_sw = T_sw
        self.T_st = T_st

        # Defaults
        self.step_L = 0.1
        self.step_h = 0.05
        self.step_dh = 0.01
        self.v_liftoff = np.zeros(3)
        self.v_touchdown = np.zeros(3)

        # Optimizer boundaries — 3 params: [dL, dL_mid, dH]
        # dL: horizontal backward/forward shape [0, 0.5]
        # dH: height offset at boundary [0, step_h * 0.9] (set dynamically)
        self.opt_lb = np.array([0.0, 0.0, 0.0])
        self.opt_ub = np.array([0.5, 0.5, 0.05])  # dH_ub updated per call
        
        self.optimizerSetup()
        
        # Presets for dL / dH parameters
        self.dL1_preset = 0.05
        self.dL2_preset = 0.0
        self.dL3_preset = 0.05
        self.dL4_preset = 0.0
        self.dH1_preset = 0.0   # updated in solveSwingTrajectory
        self.dH2_preset = 0.0   # updated in solveSwingTrajectory

    def optimizerSetup(self):
        """Configures the COBYLA solver for non-linear optimization.
        3 parameters: [dL_main, dL_mid, dH]
        """
        self.opt = nlopt.opt(nlopt.LN_COBYLA, 3)
        self.opt.set_xtol_abs(1e-5)
        self.opt.set_maxeval(60)   # increased from 40 to handle 3-param space
        self.opt.set_upper_bounds(self.opt_ub)
        self.opt.set_lower_bounds(self.opt_lb)

    def solveSwingTrajectory(self, p_lo, p_td, step_h, v_lo, v_td):
        """
        Calculates the optimal 3D swing profile.
        
        Args:
            p_lo (np.ndarray): Lift-off point [x, y, z].
            p_td (np.ndarray): Touchdown point [x, y, z].
            step_h (float): Max clearance height.
            v_lo (np.ndarray): Desired lift-off velocity vector.
            v_td (np.ndarray): Desired touchdown velocity vector.
        """
        # Normalize to 3D
        p_lo = np.pad(p_lo, (0, 3 - len(p_lo)))
        p_td = np.pad(p_td, (0, 3 - len(p_td)))
        v_lo = np.pad(v_lo, (0, 3 - len(v_lo)))
        v_td = np.pad(v_td, (0, 3 - len(v_td)))

        diff = p_td - p_lo
        
        # Handle negative step_L (backward-moving legs in yaw turns):
        # The Bezier control point geometry assumes positive L.
        # We mirror the problem to always solve with |L|, then flip the result.
        self._step_sign = np.sign(diff[0]) if abs(diff[0]) > 1e-6 else 1.0
        self.step_L = abs(diff[0])
        self.diff_h = diff[1]
        self.diff_lat = diff[2]
        
        self.step_h = step_h
        # Mirror velocities if step is backward
        self.v_liftoff = v_lo.copy()
        self.v_touchdown = v_td.copy()
        if self._step_sign < 0:
            self.v_liftoff[0] = -self.v_liftoff[0]
            self.v_touchdown[0] = -self.v_touchdown[0]

        # --- Physics-based initial guesses for dH ---
        # The Bezier endpoint tangent in height: T(0).y = 11 * dH1 / T_sw
        # Matching v_lo[1] (height): 11 * dH1 / T_sw ≈ v_lo[1]
        # → dH1_guess = v_lo[1] * T_sw / 11
        v_lo_h = abs(self.v_liftoff[1])          # swing-frame Height component
        v_td_h = abs(self.v_touchdown[1])
        dH1_guess = min(v_lo_h * self.T_sw / 11.0, step_h * 0.8)
        dH2_guess = min(v_td_h * self.T_sw / 11.0, step_h * 0.8)
        self.dH1_preset = max(0.0, dH1_guess)
        self.dH2_preset = max(0.0, dH2_guess)

        # Update upper bounds for dH dynamically (≤ step_h to keep peak at h)
        dH_ub = min(step_h * 0.9, 0.5)
        self.opt_ub = np.array([0.5, 0.5, dH_ub])
        self.opt_lb = np.array([0.0, 0.0, 0.0])

        # Optimize Lift-off parameters (dL1, dL2, dH1)
        self.optimizerSetup()
        self.opt.set_min_objective(self.objectiveFunc_lo)
        self.opt.add_inequality_constraint(self.constraint_lo)
        x_lo_opt = self.opt.optimize(np.array([self.dL1_preset, self.dL2_preset, self.dH1_preset]))

        # Optimize Touchdown parameters (dL3, dL4, dH2)
        self.optimizerSetup() # Reset for second pass
        self.opt.set_min_objective(self.objectiveFunc_td)
        self.opt.add_inequality_constraint(self.constraint_td)
        x_td_opt = self.opt.optimize(np.array([self.dL3_preset, self.dL4_preset, self.dH2_preset]))

        # Build the profile with |L|, then the caller reverses X if needed
        profile = SwingProfile(
            self.step_L, self.step_h, 0.01,
            x_lo_opt[0], x_lo_opt[1], x_td_opt[0], x_td_opt[1],
            dH1=x_lo_opt[2], dH2=x_td_opt[2],
            offset_x=0, offset_y=p_lo[1], offset_z=p_lo[2],
            diff_h=self.diff_h, diff_lat=self.diff_lat
        )
        profile._step_sign = self._step_sign
        profile._offset_x_true = p_lo[0]
        return profile

    def objectiveFunc_lo(self, x, grad):
        """Calculates 3D velocity error at lift-off.
        x = [dL1, dL2, dH1]
        """
        sp = SwingProfile(self.step_L, self.step_h, 0.01,
                          x[0], x[1], self.dL3_preset, self.dL4_preset,
                          dH1=x[2])
        p0 = sp.getFootendPoint(0.0)
        p1 = sp.getFootendPoint(0.001 / self.T_sw)
        v_calc = (p1 - p0) / self.dt
        return np.linalg.norm(self.v_liftoff - v_calc)

    def objectiveFunc_td(self, x, grad):
        """Calculates 3D velocity error at touchdown.
        x = [dL3, dL4, dH2]
        """
        sp = SwingProfile(self.step_L, self.step_h, 0.01,
                          self.dL1_preset, self.dL2_preset, x[0], x[1],
                          dH2=x[2])
        p2 = sp.getFootendPoint(1.0 - (0.001 / self.T_sw))
        p3 = sp.getFootendPoint(1.0)
        v_calc = (p3 - p2) / self.dt
        return np.linalg.norm(self.v_touchdown - v_calc)

    def constraint_lo(self, x, grad):
        """Ensures x-control points don't exceed half step length.
        Only applies to dL1, dL2 (x[0], x[1]); dH1 (x[2]) bounded separately.
        """
        return -x[0] - x[1] - (self.step_L / 2 - 0.01)

    def constraint_td(self, x, grad):
        """Ensures x-control points don't exceed step boundaries.
        Only applies to dL3, dL4 (x[0], x[1]); dH2 (x[2]) bounded separately.
        """
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