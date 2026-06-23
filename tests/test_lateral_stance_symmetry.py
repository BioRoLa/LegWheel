"""Regression tests for yaw-free lateral (sideways) gait planning.

Two bugs previously produced spurious body yaw during pure lateral motion:

1. Touchdown gamma sign was NOT mirrored per leg side. Left and right ABAD
   joints are mirror-imaged, so for a given body +Y velocity the left leg must
   sweep gamma one way and the right leg the opposite way. Using the same sign
   for all legs made one side sweep symmetrically through zero (full lateral
   travel) while the other swept away from zero (tiny travel); a diagonal
   stance pair then covered unequal lateral distances and the body yawed.

2. The Rolling Jacobian differentiated the lateral contact edge w(gamma) from
   foot_rim_contact_fk. That w is the center of pressure across the flat tread;
   its shift with tilt is not foot sliding, so feeding it into the velocity
   Jacobian corrupted the lateral velocity mapping. rolling_fk now uses w = 0.
"""
import numpy as np

from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D


def _stance_foot_travel_Y(leg_index, vy, n_steps=150, dt=0.002):
    """Lateral (body-Y) travel of the wheel-center contact over a stance sweep."""
    p = TrajectoryPlanner3D(stand_height=0.3, velocity=[0.0, vy, 0.0],
                            leg_index=leg_index, dt=dt)
    gamma_sign = (1.0 if vy >= 0 else -1.0) * p.kin.sy
    q = np.array([p.theta0, -p.beta0, gamma_sign * p.gamma0])

    def foot_y(qq):
        alpha, _ = p.kin.foot_rim_contact_fk(*qq)
        return p.kin.forward_kinematics(*qq, alpha=alpha, w=0.0)[1]

    y0 = foot_y(q)
    for _ in range(n_steps):
        q = p.stance_rt_solver(v_hip=p.velocity, q=q)
    return foot_y(q) - y0


def test_left_right_lateral_travel_is_symmetric():
    """Left and right legs must cover equal lateral distance during stance."""
    vy = 0.05
    travel = {i: _stance_foot_travel_Y(i, vy) for i in range(4)}
    # FL(0)/RL(3) are left, FR(1)/RR(2) are right.
    left = np.mean([travel[0], travel[3]])
    right = np.mean([travel[1], travel[2]])
    assert abs(left - right) < 0.05 * abs(left), (
        f"left/right lateral travel mismatch: {left*1000:.1f} vs {right*1000:.1f} mm")


def test_lateral_velocity_tracking_is_accurate():
    """rolling_fk (w=0) must track the commanded lateral velocity, not 3-4x it."""
    vy = 0.05
    p = TrajectoryPlanner3D(stand_height=0.3, velocity=[0.0, vy, 0.0],
                            leg_index=0, dt=0.002)
    gamma_sign = p.kin.sy  # vy > 0
    q = np.array([p.theta0, -p.beta0, gamma_sign * p.gamma0])

    def foot_y(qq):
        alpha, _ = p.kin.foot_rim_contact_fk(*qq)
        return p.kin.forward_kinematics(*qq, alpha=alpha, w=0.0)[1]

    vels = []
    for _ in range(100):
        y0 = foot_y(q)
        q = p.stance_rt_solver(v_hip=p.velocity, q=q)
        vels.append((foot_y(q) - y0) / p.dt)
    achieved = abs(np.mean(vels))
    assert abs(achieved - vy) < 0.1 * vy, (
        f"lateral tracking {achieved:.4f} m/s vs target {vy:.4f} m/s")


def test_touchdown_gamma_is_mirrored_by_side():
    """Touchdown gamma sign must flip between left and right legs."""
    vy = 0.05
    signs = {}
    for i in range(4):
        p = TrajectoryPlanner3D(stand_height=0.3, velocity=[0.0, vy, 0.0],
                                leg_index=i, dt=0.002)
        signs[i] = np.sign((1.0 if vy >= 0 else -1.0) * p.kin.sy * p.gamma0)
    assert signs[0] == signs[3]          # both left
    assert signs[1] == signs[2]          # both right
    assert signs[0] == -signs[1]         # left opposite to right
