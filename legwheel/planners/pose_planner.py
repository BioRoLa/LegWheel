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
        q_guess: np.ndarray | None = None,
        height_compensation: float = 0.0,
    ) -> np.ndarray:
        """
        Compute joint angles for all 4 legs for a given body pose.

        The body frame origin is placed at (0, 0, height) in world, rotated by
        (roll, pitch, yaw). Each foot remains at its fixed world contact point.

        Args:
            height (float): Body height above ground (m).
            roll   (float): Lateral tilt, rad. Positive = left-side up.
            pitch  (float): Fore-aft tilt, rad. Positive = nose down.
            yaw    (float): Heading rotation, rad.
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
        p_body_W = np.array([0.0, 0.0, height])

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

    # ------------------------------------------------------------------
    # Sequence / trajectory planning
    # ------------------------------------------------------------------

    def plan_sequence(
        self,
        waypoints: list[dict],
        n_steps: int | list[int] = 200,
        height_compensation: float = 0.0,
    ) -> np.ndarray:
        """
        Generate a trajectory by interpolating between body-pose waypoints.

        Each waypoint is a dict with keys (all optional, defaulting to current):
            height, roll, pitch, yaw

        Args:
            waypoints (list[dict]): Ordered list of target poses.
                                    First waypoint is the starting pose.
            n_steps (int | list[int]): Number of IK samples between consecutive
                                       waypoints. A single int is broadcast to
                                       all segments.
        Returns:
            np.ndarray: (N, 12) joint-angle trajectory.
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints (start + end).")

        def _parse(wp):
            return (
                float(wp.get("height", self.stand_height)),
                float(wp.get("roll",   0.0)),
                float(wp.get("pitch",  0.0)),
                float(wp.get("yaw",    0.0)),
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
            h0, r0, p0, y0 = _parse(wp_start)
            h1, r1, p1, y1 = _parse(wp_end)

            ts = np.linspace(0.0, 1.0, n, endpoint=(seg == n_segs - 1))
            for t in ts:
                pose = (
                    h0 + t * (h1 - h0),
                    r0 + t * (r1 - r0),
                    p0 + t * (p1 - p0),
                    y0 + t * (y1 - y0),
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
        n_steps: int = 200,
        return_to_neutral: bool = True,
        height_compensation: float = 0.0,
    ) -> np.ndarray:
        """
        Convenience wrapper: ramp from neutral to a target lean pose,
        optionally hold, then return to neutral.

        Args:
            roll   (float): Target roll (rad).
            pitch  (float): Target pitch (rad).
            yaw    (float): Target yaw (rad).
            height (float | None): Target height; defaults to stand_height.
            n_steps (int): Steps for each ramp segment.
            return_to_neutral (bool): Append a return ramp if True.

        Returns:
            np.ndarray: (N, 12) trajectory array.
        """
        h = height if height is not None else self.stand_height
        neutral = {"height": self.stand_height, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        target  = {"height": h, "roll": roll, "pitch": pitch, "yaw": yaw}

        wps = [neutral, target]
        steps = [n_steps]
        if return_to_neutral:
            wps.append(neutral)
            steps.append(n_steps)

        return self.plan_sequence(wps, n_steps=steps,
                                  height_compensation=height_compensation)

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
