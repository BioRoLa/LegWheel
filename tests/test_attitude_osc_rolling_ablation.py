"""V4 (Plan doc): attitude-tracking ablation, with vs. without the sagittal
rolling-arc scale s_x.

Central claim under test (CH4 Theory doc, "Attitude Oscillation Compensation
for Line-Support Gaits", Section 4.2): Bound's pitch-induced SAGITTAL hip
velocity must go through the same s_x correction as ordinary v_x, or the
realized rolling rate overshoots the commanded rate; Pace's roll-induced
LATERAL hip velocity needs no such correction, and applying one would make
tracking worse, not better.

This is verified purely kinematically: no dynamics, no forces. "Realized
velocity" means the rolling-consistent hip speed implied by the solved joint
motion (the same v_hip = beta_dot*(H_O*sec^2(beta)+R_arc) relation already
documented in TrajectoryPlanner3D.stance_rt_solver), not a measured physical
quantity.

The production `stance_rt_solver` is never modified -- these tests build a
small local reimplementation with the scale factor exposed as a knob, and a
cross-check test confirms that reimplementation reproduces the production
solver bit-for-bit at its default (correct) settings.
"""
import numpy as np
import pytest

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.trajectory_planning_3d import TrajectoryPlanner3D
from legwheel.utils import numerical_jacobian, pseudo_inverse_dls

COMMON_KW = dict(stand_height=0.25, twist=[0.0, 0.10, 0.0], period=1.0, dt=0.001)
AMPLITUDE = np.deg2rad(6.0)


def _rolling_fk(planner, q_eval):
    contact = planner.kin.foot_rim_contact_fk(*q_eval, ground_slope=0.0)
    return planner.kin.forward_kinematics(*q_eval, alpha=contact[0], w=0.0)


def _correct_scale_x(planner, beta):
    sec2_beta = 1.0 / (np.cos(beta) ** 2)
    geom_grad = planner.H_O * sec2_beta
    return geom_grad / (geom_grad + planner.R_arc)


def _stance_step(planner, q, v_hip, scale_x=None, scale_y=1.0, damping=1e-2):
    """Local reimplementation of TrajectoryPlanner3D.stance_rt_solver with the
    per-axis scale exposed. scale_x=None reproduces production behavior
    (correct s_x); any other value is a deliberate ablation."""
    q = np.array(q, dtype=float)
    sx = _correct_scale_x(planner, q[1]) if scale_x is None else scale_x

    def rolling_fk(q_eval):
        return _rolling_fk(planner, q_eval)

    J = numerical_jacobian(rolling_fk, q, diff=1e-5)
    v_target = np.array([-v_hip[0] * sx, -v_hip[1] * scale_y, -v_hip[2]])
    J_star = pseudo_inverse_dls(J, damping_factor=damping)
    q_dot = J_star @ v_target
    return q + q_dot * planner.dt, q_dot


def test_local_reimplementation_matches_production_solver():
    """Sanity check: at default (correct) settings, the local helper above
    must reproduce TrajectoryPlanner3D.stance_rt_solver bit-for-bit, so the
    ablation below is exercising the same math as production, not a drifted
    copy of it."""
    planner = TrajectoryPlanner3D(stand_height=0.25, velocity=[0.1, 0.02, 0.0], leg_index=0, dt=0.001)
    q = np.array([planner.theta0, planner.beta0, 0.0])
    v_hip = np.array([0.1, 0.02, 0.0])

    q_next_production = planner.stance_rt_solver(v_hip=v_hip, q=q)
    q_next_local, _ = _stance_step(planner, q, v_hip)

    assert np.allclose(q_next_production, q_next_local, atol=1e-12)


def _bound_probe_v_hip():
    """Pull a real commanded v_hip(t) sample from an actual Bound
    GaitGenerator3D with the oscillation enabled -- ties this ablation to the
    Step 3 feature rather than a synthetic value."""
    gen = GaitGenerator3D(gait_type="Bound", attitude_osc_amplitude=AMPLITUDE, **COMMON_KW)
    planner = gen.planners[0]  # FL, phase_offset 0.0
    t_probe = 0.25 * gen.T
    v_hip = planner.velocity + planner.hip_velocity_fn(t_probe)
    return planner, v_hip


def test_bound_sagittal_scaling_required_for_accurate_tracking():
    planner, v_hip = _bound_probe_v_hip()
    q = np.array([planner.theta0, planner.beta0, 0.0])

    _, q_dot_scaled = _stance_step(planner, q, v_hip, scale_x=None)
    _, q_dot_unscaled = _stance_step(planner, q, v_hip, scale_x=1.0)

    sec2_beta = 1.0 / (np.cos(q[1]) ** 2)
    rolling_factor = planner.H_O * sec2_beta + planner.R_arc

    v_real_scaled = q_dot_scaled[1] * rolling_factor
    v_real_unscaled = q_dot_unscaled[1] * rolling_factor

    err_scaled = abs(v_real_scaled - v_hip[0])
    err_unscaled = abs(v_real_unscaled - v_hip[0])

    # Scaled tracking should be accurate (small residual from Jacobian
    # linearization); unscaled should overshoot substantially.
    assert err_scaled < 0.02 * abs(v_hip[0])
    assert err_unscaled > 5.0 * err_scaled
    assert v_real_unscaled > v_hip[0] > 0  # overshoots in the commanded direction


def _pace_probe_v_hip():
    gen = GaitGenerator3D(gait_type="Pace", attitude_osc_amplitude=AMPLITUDE, **COMMON_KW)
    planner = gen.planners[0]  # FL, phase_offset 0.0
    t_probe = 0.25 * gen.T
    v_hip = planner.velocity + planner.hip_velocity_fn(t_probe)
    return planner, v_hip


def test_pace_lateral_needs_no_scaling_and_misapplied_scale_degrades_tracking():
    planner, v_hip = _pace_probe_v_hip()
    q = np.array([planner.theta0, planner.beta0, 0.0])

    q_next_baseline, _ = _stance_step(planner, q, v_hip, scale_x=1.0, scale_y=1.0)
    wrong_scale = _correct_scale_x(planner, q[1])  # sagittal-style factor, misapplied to Y
    q_next_wrong, _ = _stance_step(planner, q, v_hip, scale_x=1.0, scale_y=wrong_scale)

    v_real_baseline = -(_rolling_fk(planner, q_next_baseline)[1] - _rolling_fk(planner, q)[1]) / planner.dt
    v_real_wrong = -(_rolling_fk(planner, q_next_wrong)[1] - _rolling_fk(planner, q)[1]) / planner.dt

    err_baseline = abs(v_real_baseline - v_hip[1])
    err_wrong = abs(v_real_wrong - v_hip[1])

    assert err_baseline < 0.05 * abs(v_hip[1])
    assert err_wrong > 5.0 * err_baseline
