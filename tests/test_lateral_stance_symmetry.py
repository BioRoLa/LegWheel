"""Regression tests for level, yaw-free lateral (sideways) gait planning.

Three coupled defects produced bad lateral motion and were fixed in turn:

1. Touchdown gamma sign was not mirrored per leg side, so a diagonal stance pair
   covered unequal lateral distance and the body yawed.
2. The Rolling Jacobian differentiated the lateral contact edge w(gamma) (center
   of pressure, not foot sliding), corrupting lateral velocity tracking.
3. Touchdown joints were set open-loop ([theta0, -beta0, ±gamma]); with a
   prescribed gamma the foot Z depends on the per-leg ABAD tilt, so mirrored
   left/right gammas touched down at different heights. stance_rt_solver holds
   vz=0, locking that offset in for the whole stance, so the body had to ROLL to
   keep every foot on the ground (which prevented feet reaching targets and
   induced phase lag). Touchdown is now an IK solve to a LEVEL, motion-leading
   foot position (_level_touchdown_q), so every foot starts at -stand_height.
"""
import numpy as np

from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D
from legwheel.models.corgi_leg import CorgiLegKinematics

STAND_H = 0.25


def _planner(leg_index, vy):
    return TrajectoryPlanner3D(stand_height=STAND_H, velocity=[0.0, vy, 0.0],
                               leg_index=leg_index, dt=0.001)


def _foot(kin, q):
    alpha, _ = kin.foot_rim_contact_fk(*q)
    return kin.forward_kinematics(*q, alpha=alpha, w=0.0)


def _stance_trace(leg_index, vy):
    """Full stance sweep from the IK level touchdown; return (gammas, foot xyz)."""
    p = _planner(leg_index, vy)
    q = p._level_touchdown_q()
    gammas, feet = [q[2]], [_foot(p.kin, q)]
    for _ in range(int(p.T * p.stance_duty / p.dt)):
        q = p.stance_rt_solver(v_hip=p.velocity, q=q)
        if abs(q[1]) > np.deg2rad(45):
            break
        gammas.append(q[2])
        feet.append(_foot(p.kin, q))
    return np.array(gammas), np.array(feet)


def test_touchdown_is_level_across_all_legs():
    """Every leg's IK touchdown foot must sit at the same height (no body roll)."""
    vy = 0.05
    zs = []
    for i in range(4):
        p = _planner(i, vy)
        zs.append(_foot(p.kin, p._level_touchdown_q())[2])
    spread = (max(zs) - min(zs)) * 1000.0
    assert spread < 1.0, f"touchdown foot Z spread {spread:.2f} mm -> body roll"
    # And it should be at the commanded stand height.
    assert abs(np.mean(zs) + STAND_H) < 2e-3


def test_left_right_lateral_travel_is_symmetric():
    """Left and right legs cover equal lateral distance during stance (no yaw)."""
    vy = 0.05
    travel = {i: (_stance_trace(i, vy)[1][-1, 1] - _stance_trace(i, vy)[1][0, 1])
              for i in range(4)}
    left = np.mean([travel[0], travel[3]])
    right = np.mean([travel[1], travel[2]])
    assert abs(left - right) < 0.05 * abs(left), (
        f"left/right lateral travel mismatch: {left*1000:.1f} vs {right*1000:.1f} mm")


def test_stance_height_held_through_sweep():
    """Foot Z must stay at touchdown height for the whole stance (level body)."""
    for i in range(4):
        _, feet = _stance_trace(i, 0.05)
        z_drift = (feet[:, 2].max() - feet[:, 2].min()) * 1000.0
        assert z_drift < 1.0, f"leg {i} foot Z drifts {z_drift:.2f} mm during stance"


def test_lateral_sweep_stays_one_sided():
    """Loaded ABAD sweep must not cross gamma=0 (no contact-edge / COP flip)."""
    for i in range(4):
        g, _ = _stance_trace(i, 0.05)
        crosses = g.min() < -np.deg2rad(0.2) and g.max() > np.deg2rad(0.2)
        assert not crosses, (
            f"leg {i} crosses upright under load: gamma range "
            f"[{np.rad2deg(g.min()):+.2f}, {np.rad2deg(g.max()):+.2f}] deg")
