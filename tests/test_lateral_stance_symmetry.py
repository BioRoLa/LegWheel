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


def _stance_gamma_trace(leg_index, vy, dt=0.001):
    """Run a full stance sweep; return (gamma_trace, wheel-center foot-Y trace)."""
    p = TrajectoryPlanner3D(stand_height=0.25, velocity=[0.0, vy, 0.0],
                            leg_index=leg_index, dt=dt)
    gamma_sign = (1.0 if vy >= 0 else -1.0) * p.kin.sy
    q = np.array([p.theta0, -p.beta0, gamma_sign * p.gamma_td])

    def foot_y(qq):
        alpha, _ = p.kin.foot_rim_contact_fk(*qq)
        return p.kin.forward_kinematics(*qq, alpha=alpha, w=0.0)[1]

    gammas, ys = [q[2]], [foot_y(q)]
    for _ in range(int(p.T * p.stance_duty / dt)):
        q = p.stance_rt_solver(v_hip=p.velocity, q=q)
        if abs(q[1]) > np.deg2rad(45):
            break
        gammas.append(q[2])
        ys.append(foot_y(q))
    return np.array(gammas), np.array(ys)


def _stance_foot_travel_Y(leg_index, vy):
    g, ys = _stance_gamma_trace(leg_index, vy)
    return ys[-1] - ys[0]


def test_left_right_lateral_travel_is_symmetric():
    """Left and right legs must cover equal lateral distance during stance."""
    vy = 0.05
    travel = {i: _stance_foot_travel_Y(i, vy) for i in range(4)}
    # FL(0)/RL(3) are left, FR(1)/RR(2) are right.
    left = np.mean([travel[0], travel[3]])
    right = np.mean([travel[1], travel[2]])
    assert abs(left - right) < 0.05 * abs(left), (
        f"left/right lateral travel mismatch: {left*1000:.1f} vs {right*1000:.1f} mm")


def test_lateral_sweep_stays_one_sided():
    """Loaded ABAD sweep must not cross gamma=0 (no contact-edge / COP flip)."""
    vy = 0.05
    for i in range(4):
        g, _ = _stance_gamma_trace(i, vy)
        # All samples share one sign (allow a hair of numerical slack near the floor).
        assert g.min() > -np.deg2rad(0.2) or g.max() < np.deg2rad(0.2), (
            f"leg {i} crosses upright: gamma range "
            f"[{np.rad2deg(g.min()):+.2f}, {np.rad2deg(g.max()):+.2f}] deg")
        assert np.all(np.sign(g[g != 0]) == np.sign(g[0])), (
            f"leg {i} ABAD sweep changes sign mid-stance")


def test_touchdown_gamma_is_mirrored_by_side():
    """Touchdown gamma sign must flip between left and right legs."""
    vy = 0.05
    signs = {}
    for i in range(4):
        p = TrajectoryPlanner3D(stand_height=0.25, velocity=[0.0, vy, 0.0],
                                leg_index=i, dt=0.002)
        signs[i] = np.sign((1.0 if vy >= 0 else -1.0) * p.kin.sy * p.gamma_td)
    assert signs[0] == signs[3]          # both left
    assert signs[1] == signs[2]          # both right
    assert signs[0] == -signs[1]         # left opposite to right
