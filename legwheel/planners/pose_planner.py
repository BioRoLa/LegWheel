"""
Whole-body pose planner for the Corgi robot with all feet on the ground.

The robot leans (rolls/pitches/yaws) or changes height while all four feet
remain at fixed contact points in the world frame. Given a sequence of body
poses, this planner computes joint angles via per-leg IK.

Coordinate convention:
    {B}  - Body Frame: +X Front, +Y Left, +Z Up. Origin at chassis center.
    World - {B} at neutral pose. Ground plane is z=0.

Body pose parameterization:
    height (float): Distance from ground to body-frame origin (m).
    roll   (float): Rotation about X (lateral tilt, rad). Positive = left-side up.
    pitch  (float): Rotation about Y (fore-aft tilt, rad). Positive = nose down.
    yaw    (float): Rotation about Z (heading, rad).
"""

import numpy as np
import pandas as pd
from legwheel.models.corgi_leg import CorgiLegKinematics
from legwheel.utils.fitted_coefficient import inv_G_dist_poly


LEG_LABELS = ["FL", "FR", "RR", "RL"]


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rot_zyx(roll, pitch, yaw):
    """Rotation matrix R_WB: body → world (ZYX Euler)."""
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


class PosePlanner:
    """
    Whole-body static pose planner: lean, pitch, roll, or height change
    with all four feet planted on flat ground.

    Args:
        stand_height (float): Nominal standing height (body origin above ground, m).
        dt (float): Time step for trajectory sampling (s).
    """

    def __init__(self, stand_height: float = 0.30, dt: float = 0.001):
        self.stand_height = stand_height
        self.dt = dt

        self._kins = [CorgiLegKinematics(i) for i in range(4)]

        # Compute neutral joint angles and fixed foot positions in world frame.
        self._q_neutral = np.zeros((4, 3))   # [theta, beta, gamma] per leg
        self._p_feet_W = np.zeros((4, 3))    # foot contacts in world frame

        self._init_neutral_pose()

        # Trajectory storage (populated by plan_sequence / plan_lean)
        self.CMDS: np.ndarray | None = None

    # ------------------------------------------------------------------
    # Initialisation helpers
    # ------------------------------------------------------------------

    def _neutral_theta(self, H_O: float, R_arc: float, R_link: float) -> float:
        """Solve theta0 from the height constraint: G_dist = H_O + R_link."""
        # At beta=0: foot directly below hip, G_dist = H_O / cos(0) + R_link
        G_dist = H_O + R_link
        return inv_G_dist_poly(G_dist)

    def _init_neutral_pose(self):
        """
        For each leg, compute the neutral joint angles at stand_height with
        beta=0 (no sagittal swing) and gamma=0, then FK to get world foot positions.
        """
        for i, kin in enumerate(self._kins):
            R_arc = kin.solver.foot_radius
            R_link = kin.solver.R
            d_abad = kin.d_abad
            H_hip = self.stand_height + d_abad  # hip height above ground
            H_O = H_hip - R_arc                 # arc-center height above ground

            theta0 = self._neutral_theta(H_O, R_arc, R_link)
            beta0 = 0.0
            gamma0 = 0.0

            self._q_neutral[i] = [theta0, beta0, gamma0]

            # FK: foot position in {B} at neutral pose
            p_B = kin.forward_kinematics(theta0, beta0, gamma0)
            # Convert to world frame (neutral body origin at [0, 0, stand_height])
            self._p_feet_W[i] = p_B + np.array([0.0, 0.0, self.stand_height])

    # ------------------------------------------------------------------
    # Core solver
    # ------------------------------------------------------------------

    def solve_pose(
        self,
        height: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        x_offset: float = 0.0,
        y_offset: float = 0.0,
        q_guess: np.ndarray | None = None,
        height_compensation: float = 0.0,
    ) -> np.ndarray:
        """
        Compute joint angles for all 4 legs for a given body pose.

        The body frame origin is placed at (x_offset, y_offset, height) in
        world, rotated by (roll, pitch, yaw). Each foot remains at its fixed
        world contact point.

        Args:
            height (float): Body height above ground (m).
            roll   (float): Lateral tilt, rad. Positive = left-side up.
            pitch  (float): Fore-aft tilt, rad. Positive = nose down.
            yaw    (float): Heading rotation, rad.
            x_offset (float): Body X translation from world origin (m, +X forward).
            y_offset (float): Body Y translation from world origin (m, +Y left).
            q_guess (np.ndarray | None): (4, 3) warm-start joint angles.
                                         Defaults to neutral pose.
            height_compensation (float): Auto-lower body height proportional to
                lean angle magnitude (m/rad). Prevents workspace overflow for
                larger lean angles. Typical value: 0.1–0.3.

        Returns:
            np.ndarray: (4, 3) joint angles [[theta, beta, gamma], ...].
        """
        lean_mag = np.sqrt(roll ** 2 + pitch ** 2)
        height = height - height_compensation * lean_mag
        R_WB = _rot_zyx(roll, pitch, yaw)   # body → world
        p_body_W = np.array([x_offset, y_offset, height])

        if q_guess is None:
            q_guess = self._q_neutral.copy()

        q_result = np.zeros((4, 3))
        for i, kin in enumerate(self._kins):
            # Foot position expressed in the (now rotated) body frame
            p_foot_B = R_WB.T @ (self._p_feet_W[i] - p_body_W)
            try:
                q_result[i] = kin.inverse_kinematics(
                    p_foot_B, guess_q=q_guess[i], rim_point=(0.0, 0.0)
                )
            except RuntimeError:
                try:
                    # Warm-start may have led to a poor basin; retry from neutral.
                    q_result[i] = kin.inverse_kinematics(
                        p_foot_B, guess_q=self._q_neutral[i], rim_point=(0.0, 0.0)
                    )
                except RuntimeError:
                    # Target is near or outside workspace boundary.  Clamp to the
                    # closest reachable point by interpolating toward the flat-body
                    # foot target (zero rotation, same height — guaranteed reachable).
                    p_flat_B = self._p_feet_W[i] - p_body_W   # no rotation
                    p_clamped = 0.5 * (p_foot_B + p_flat_B)
                    try:
                        q_result[i] = kin.inverse_kinematics(
                            p_clamped, guess_q=self._q_neutral[i], rim_point=(0.0, 0.0)
                        )
                        print(
                            f"  ⚠ Leg {i}: clamped to 50% of requested lean "
                            f"(target outside workspace). Use height_compensation "
                            f"to reach larger lean angles."
                        )
                    except RuntimeError:
                        q_result[i] = self._q_neutral[i].copy()
                        print(f"  ⚠ Leg {i}: IK failed — using neutral pose.")
        return q_result

    def _is_feasible(
        self,
        height: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        x_offset: float = 0.0,
        y_offset: float = 0.0,
    ) -> bool:
        """Return True only if all 4 legs reach the pose without any IK fallback."""
        R_WB = _rot_zyx(roll, pitch, yaw)
        p_body_W = np.array([x_offset, y_offset, height])
        for i, kin in enumerate(self._kins):
            p_foot_B = R_WB.T @ (self._p_feet_W[i] - p_body_W)
            try:
                kin.inverse_kinematics(p_foot_B, guess_q=self._q_neutral[i], rim_point=(0.0, 0.0))
            except RuntimeError:
                return False
        return True

    def compute_workspace(
        self,
        height: float | None = None,
        tol_deg: float = 0.1,
        tol_m: float = 0.0005,
    ) -> dict:
        """
        Binary-search the feasible movement range at a given stand height.

        Each DOF is swept independently (others held at zero/neutral).
        Returns a dict with keys: roll, pitch, yaw, x, y — each a (min, max) tuple.

        Args:
            height  : Body height to evaluate (defaults to stand_height).
            tol_deg : Angular resolution for roll/pitch/yaw search (degrees).
            tol_m   : Linear resolution for x/y search (metres).
        """
        h = height if height is not None else self.stand_height

        def bisect_positive(fn, hi_init, tol):
            lo, hi = 0.0, hi_init
            if not fn(hi):
                while hi > tol and not fn(hi):
                    hi /= 2.0
            for _ in range(60):
                mid = (lo + hi) / 2.0
                if fn(mid):
                    lo = mid
                else:
                    hi = mid
                if hi - lo < tol:
                    break
            return lo

        tr = np.deg2rad(tol_deg)

        roll_max  = bisect_positive(lambda v: self._is_feasible(h, roll=v),    np.deg2rad(45), tr)
        roll_min  = bisect_positive(lambda v: self._is_feasible(h, roll=-v),   np.deg2rad(45), tr)
        pitch_max = bisect_positive(lambda v: self._is_feasible(h, pitch=v),   np.deg2rad(45), tr)
        pitch_min = bisect_positive(lambda v: self._is_feasible(h, pitch=-v),  np.deg2rad(45), tr)
        yaw_max   = bisect_positive(lambda v: self._is_feasible(h, yaw=v),     np.deg2rad(60), tr)
        yaw_min   = bisect_positive(lambda v: self._is_feasible(h, yaw=-v),    np.deg2rad(60), tr)
        x_max     = bisect_positive(lambda v: self._is_feasible(h, x_offset=v),  0.15, tol_m)
        x_min     = bisect_positive(lambda v: self._is_feasible(h, x_offset=-v), 0.15, tol_m)
        y_max     = bisect_positive(lambda v: self._is_feasible(h, y_offset=v),  0.15, tol_m)
        y_min     = bisect_positive(lambda v: self._is_feasible(h, y_offset=-v), 0.15, tol_m)

        result = {
            "height": h,
            "roll":  (-roll_min,  roll_max),
            "pitch": (-pitch_min, pitch_max),
            "yaw":   (-yaw_min,   yaw_max),
            "x":     (-x_min,     x_max),
            "y":     (-y_min,     y_max),
        }
        return result

    def print_workspace(self, height: float | None = None, **kwargs) -> dict:
        """Compute and print the feasible workspace table at a given height."""
        h = height if height is not None else self.stand_height
        print(f"Computing workspace at height = {h:.3f} m …")
        ws = self.compute_workspace(h, **kwargs)
        print(f"\n{'─'*42}")
        print(f"  Workspace at stand_height = {h:.3f} m")
        print(f"{'─'*42}")
        r_lo, r_hi = np.rad2deg(ws['roll'])
        p_lo, p_hi = np.rad2deg(ws['pitch'])
        y_lo, y_hi = np.rad2deg(ws['yaw'])
        print(f"  Roll   : {r_lo:+6.1f}°  →  {r_hi:+6.1f}°")
        print(f"  Pitch  : {p_lo:+6.1f}°  →  {p_hi:+6.1f}°")
        print(f"  Yaw    : {y_lo:+6.1f}°  →  {y_hi:+6.1f}°")
        print(f"  X      : {ws['x'][0]*1000:+6.1f} mm  →  {ws['x'][1]*1000:+6.1f} mm")
        print(f"  Y      : {ws['y'][0]*1000:+6.1f} mm  →  {ws['y'][1]*1000:+6.1f} mm")
        print(f"{'─'*42}\n")
        return ws

    # ------------------------------------------------------------------
    # Sequence / trajectory planning
    # ------------------------------------------------------------------

    @staticmethod
    def _profile_map(ts: np.ndarray, profile: str, ramp_ratio: float) -> np.ndarray:
        """
        Map a linear parameter array ts ∈ [0, 1] through a velocity profile,
        returning a position array s ∈ [0, 1] with zero velocity at both ends.

        profile='cosine'   : smooth S-curve (sinusoidal velocity, no jerk steps)
        profile='trapezoid': constant-accel ramp + cruise + ramp-down
        profile='linear'   : no remapping (original behaviour)
        """
        if profile == "cosine":
            return 0.5 * (1.0 - np.cos(np.pi * ts))
        if profile == "trapezoid":
            r = float(np.clip(ramp_ratio, 1e-3, 0.499))
            # v_max chosen so that area under trapezoid = 1
            v_max = 1.0 / (1.0 - r)
            denom = 2.0 * r * (1.0 - r)
            s = np.where(
                ts < r,
                ts ** 2 / denom,
                np.where(
                    ts <= 1.0 - r,
                    r / (2.0 * (1.0 - r)) + (ts - r) / (1.0 - r),
                    1.0 - (1.0 - ts) ** 2 / denom,
                ),
            )
            return np.clip(s, 0.0, 1.0)
        # 'linear' – no remapping
        return ts

    def plan_sequence(
        self,
        waypoints: list[dict],
        n_steps: int | list[int] = 200,
        height_compensation: float = 0.0,
        profile: str = "cosine",
        ramp_ratio: float = 0.25,
    ) -> np.ndarray:
        """
        Generate a trajectory by interpolating between body-pose waypoints.

        Each waypoint is a dict with keys (all optional, defaulting to current):
            height, roll, pitch, yaw, x_offset, y_offset

        Args:
            waypoints (list[dict]): Ordered list of target poses.
                                    First waypoint is the starting pose.
            n_steps (int | list[int]): Number of IK samples between consecutive
                                       waypoints. A single int is broadcast to
                                       all segments.
            profile (str): Velocity profile for each segment.
                'cosine'    – smooth S-curve, zero velocity at endpoints (default).
                'trapezoid' – ramp-up / cruise / ramp-down, zero at endpoints.
                'linear'    – constant velocity (original behaviour).
            ramp_ratio (float): Fraction of segment time used for accel (and decel)
                                in 'trapezoid' mode. Must be in (0, 0.5).
                                Ignored for other profiles.
        Returns:
            np.ndarray: (N, 12) joint-angle trajectory.
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints (start + end).")

        def _parse(wp):
            return (
                float(wp.get("height",   self.stand_height)),
                float(wp.get("roll",     0.0)),
                float(wp.get("pitch",    0.0)),
                float(wp.get("yaw",      0.0)),
                float(wp.get("x_offset", 0.0)),
                float(wp.get("y_offset", 0.0)),
            )

        n_segs = len(waypoints) - 1
        if isinstance(n_steps, int):
            steps_per_seg = [n_steps] * n_segs
        else:
            if len(n_steps) != n_segs:
                raise ValueError("len(n_steps) must equal len(waypoints) - 1.")
            steps_per_seg = list(n_steps)

        all_cmds = []
        q_prev = self._q_neutral.copy()

        for seg, (n, wp_start, wp_end) in enumerate(
            zip(steps_per_seg, waypoints[:-1], waypoints[1:])
        ):
            h0, r0, p0, y0, x0, yo0 = _parse(wp_start)
            h1, r1, p1, y1, x1, yo1 = _parse(wp_end)

            ts_linear = np.linspace(0.0, 1.0, n, endpoint=(seg == n_segs - 1))
            ts = self._profile_map(ts_linear, profile, ramp_ratio)

            for t in ts:
                pose = (
                    h0 + t * (h1 - h0),
                    r0 + t * (r1 - r0),
                    p0 + t * (p1 - p0),
                    y0 + t * (y1 - y0),
                    x0 + t * (x1 - x0),
                    yo0 + t * (yo1 - yo0),
                )
                q = self.solve_pose(*pose, q_guess=q_prev,
                                    height_compensation=height_compensation)
                all_cmds.append(q.flatten())
                q_prev = q

        self.CMDS = np.array(all_cmds)
        return self.CMDS

    def plan_lean(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        height: float | None = None,
        x_offset: float = 0.0,
        y_offset: float = 0.0,
        n_steps: int = 1000,
        return_to_neutral: bool = True,
        height_compensation: float = 0.0,
        n_repeats: int = 1,
        rock: bool = False,
        profile: str = "cosine",
        ramp_ratio: float = 0.25,
    ) -> np.ndarray:
        """
        Convenience wrapper: ramp from neutral to a target lean pose,
        optionally hold, then return to neutral.  Repeat N times.

        Args:
            roll     (float): Target roll (rad).
            pitch    (float): Target pitch (rad).
            yaw      (float): Target yaw (rad).
            height   (float | None): Target height; defaults to stand_height.
            x_offset (float): Body X translation from neutral (m, +X forward).
            y_offset (float): Body Y translation from neutral (m, +Y left).
            n_steps  (int): Steps for each ramp segment (segment duration T = n_steps*dt).
                         Peak commanded angular acceleration for the cosine profile is
                         0.5*A*(pi/T)^2 (A = ramp amplitude), so a smaller n_steps/T
                         raises acceleration roughly with 1/T^2. Default 1000 (T=1.0s
                         at dt=0.001s) was chosen after the previous default of 200
                         (T=0.2s) was found to command enough angular acceleration
                         during yaw rocking to exceed foot-ground friction and cause
                         intermittent foot lift-off (100% ground contact restored at
                         T=1.0s vs. 62.8% at T=0.5s, same amplitude) -- see
                         Biorola Notes/03_Simulation/05_Experiment/21_Lean_Rock_FK_IK_Validation.md.
            return_to_neutral (bool): Append a return ramp after the last rep.
            height_compensation (float): Lower body height per rad of lean (m/rad).
            n_repeats (int): Number of lean cycles. Must be >= 1.
            rock (bool): If True, each cycle goes +target → neutral → -target → neutral,
                         oscillating symmetrically around neutral. RPY and XY are negated
                         for the minus phase; height stays the same.

        Returns:
            np.ndarray: (N, 12) trajectory array.
        """
        if n_repeats < 1:
            raise ValueError("n_repeats must be >= 1")
        h = height if height is not None else self.stand_height
        neutral = {
            "height": self.stand_height, "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "x_offset": 0.0, "y_offset": 0.0,
        }
        target_pos = {
            "height": h, "roll": roll, "pitch": pitch, "yaw": yaw,
            "x_offset": x_offset, "y_offset": y_offset,
        }
        target_neg = {
            "height": h, "roll": -roll, "pitch": -pitch, "yaw": -yaw,
            "x_offset": -x_offset, "y_offset": -y_offset,
        }

        wps = [neutral]
        for _ in range(n_repeats):
            wps.append(target_pos)
            wps.append(neutral)
            if rock:
                wps.append(target_neg)
                wps.append(neutral)
        if not return_to_neutral:
            wps = wps[:-1]

        steps = [n_steps] * (len(wps) - 1)
        return self.plan_sequence(
            wps,
            n_steps=steps,
            height_compensation=height_compensation,
            profile=profile,
            ramp_ratio=ramp_ratio,
        )

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def export_to_csv(self, base_name: str = "pose_lean") -> None:
        """Export the last planned trajectory to a 12-DOF CSV file."""
        if self.CMDS is None:
            raise RuntimeError("No trajectory planned yet. Call plan_sequence() first.")

        cols = []
        for label in LEG_LABELS:
            cols += [f"{label}_Theta", f"{label}_Beta", f"{label}_Gamma"]

        df = pd.DataFrame(self.CMDS, columns=cols)
        path = base_name + "_12dof.csv"
        df.to_csv(path, index=False)
        print(f"Pose trajectory exported to {path}  ({len(df)} steps)")

    def print_summary(self) -> None:
        """Print planner configuration and neutral foot positions."""
        print("=== PosePlanner Summary ===")
        print(f"  Stand height : {self.stand_height:.3f} m")
        print(f"  Time step    : {self.dt*1000:.1f} ms")
        print()
        print("  Neutral joint angles and foot contacts (world frame):")
        for i, label in enumerate(LEG_LABELS):
            q = self._q_neutral[i]
            p = self._p_feet_W[i]
            print(
                f"    {label}: θ={np.rad2deg(q[0]):.1f}°  β={np.rad2deg(q[1]):.1f}°  "
                f"γ={np.rad2deg(q[2]):.1f}°  "
                f"foot=[{p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}] m"
            )
        if self.CMDS is not None:
            print(f"\n  Trajectory: {len(self.CMDS)} steps")
