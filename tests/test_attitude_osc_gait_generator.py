"""Tests for wiring the attitude-oscillation compensation into GaitGenerator3D
(Step 3 of the Attitude Oscillation Compensation plan).

Covers:
    V1 - zero regression at the default amplitude (0.0)
    Validation - amplitude only accepted for Bound/Pace
    Wiring - nonzero amplitude actually perturbs the stance trajectory
    Global-phase correctness - the two legs of a pair (phase_offset 0.0 vs 0.5
        for Bound/Pace) must see opposite-signed injected velocity at their
        own touchdown, since the SAME global attitude reference must be
        realized regardless of which pair is currently on the ground
        (CH4 Theory doc Section 4.2/4.3).
"""
import numpy as np
import pytest

from legwheel.planners.gait_generator_3d import GaitGenerator3D

COMMON_KW = dict(stand_height=0.25, twist=[0.0, 0.10, 0.0], period=1.0, dt=0.002)


def _stack_cmds(gen):
    return np.array([p.generate_trajectory() for p in gen.planners], dtype=object)


def test_default_amplitude_is_zero_and_disabled():
    gen = GaitGenerator3D(gait_type="Bound", **COMMON_KW)
    assert gen.attitude_osc_amplitude == 0.0
    for planner in gen.planners:
        assert planner.hip_velocity_fn is None


def test_zero_amplitude_matches_pre_feature_trajectory():
    """Explicitly passing amplitude=0.0 must be bit-identical to a
    GaitGenerator3D built without the new kwargs at all."""
    gen_baseline = GaitGenerator3D(gait_type="Bound", **COMMON_KW)
    gen_explicit_zero = GaitGenerator3D(
        gait_type="Bound", attitude_osc_amplitude=0.0, attitude_osc_phase_lead=0.0, **COMMON_KW
    )
    for p_base, p_zero in zip(gen_baseline.planners, gen_explicit_zero.planners):
        cmd_base = np.array(p_base.generate_trajectory())
        cmd_zero = np.array(p_zero.generate_trajectory())
        assert np.array_equal(cmd_base, cmd_zero)


@pytest.mark.parametrize("gait_type", ["Walk", "Trot", "Pronk"])
def test_nonzero_amplitude_rejected_for_non_line_support_gaits(gait_type):
    with pytest.raises(ValueError, match="attitude_osc_amplitude"):
        GaitGenerator3D(
            gait_type=gait_type, attitude_osc_amplitude=np.deg2rad(5.0), **COMMON_KW
        )


@pytest.mark.parametrize("gait_type", ["Bound", "Pace"])
def test_nonzero_amplitude_accepted_for_line_support_gaits(gait_type):
    gen = GaitGenerator3D(
        gait_type=gait_type, attitude_osc_amplitude=np.deg2rad(5.0), **COMMON_KW
    )
    for planner in gen.planners:
        assert planner.hip_velocity_fn is not None


@pytest.mark.parametrize("gait_type", ["Bound", "Pace"])
def test_nonzero_amplitude_perturbs_stance_trajectory(gait_type):
    gen_off = GaitGenerator3D(gait_type=gait_type, attitude_osc_amplitude=0.0, **COMMON_KW)
    gen_on = GaitGenerator3D(
        gait_type=gait_type, attitude_osc_amplitude=np.deg2rad(6.0), **COMMON_KW
    )
    any_changed = False
    for p_off, p_on in zip(gen_off.planners, gen_on.planners):
        cmd_off = np.array(p_off.generate_trajectory())
        cmd_on = np.array(p_on.generate_trajectory())
        assert cmd_off.shape == cmd_on.shape
        if not np.allclose(cmd_off, cmd_on, atol=1e-6):
            any_changed = True
    assert any_changed


def test_bound_front_and_rear_pair_get_opposite_signed_injection_mid_stance():
    """FL/FR (phase_offset=0.0) and RR/RL (phase_offset=0.5) must inject
    opposite-signed sagittal velocity at a matched GLOBAL phase, because the
    global phase phi_g differs by 0.5 between the two pairs and ds_dphi is
    half-period antisymmetric (Theory doc Section 3.1/4.2).

    Evaluated at t_local = 0.25*T rather than touchdown (t_local=0): s(phi_g)
    peaks (zero slope) exactly at touchdown, so the injected RATE is
    momentarily zero there by construction -- the sign flip only shows up
    away from that turning point.
    """
    gen = GaitGenerator3D(
        gait_type="Bound", attitude_osc_amplitude=np.deg2rad(6.0), **COMMON_KW
    )
    front_fn = gen.planners[0].hip_velocity_fn  # FL, phase_offset 0.0
    rear_fn = gen.planners[2].hip_velocity_fn  # RR, phase_offset 0.5
    t_probe = 0.25 * gen.T

    dv_front = front_fn(t_probe)
    dv_rear = rear_fn(t_probe)

    assert dv_front[0] != 0.0
    assert np.isclose(dv_front[0], -dv_rear[0])
    # Pitch channel only -- lateral/vertical untouched.
    assert dv_front[1] == 0.0 and dv_front[2] == 0.0


def test_pace_left_and_right_pair_get_opposite_signed_injection_mid_stance():
    """FL/RL (phase_offset=0.0) and FR/RR (phase_offset=0.5) must inject
    opposite-signed LATERAL velocity at a matched global phase (see rationale
    in the Bound test above)."""
    gen = GaitGenerator3D(
        gait_type="Pace", attitude_osc_amplitude=np.deg2rad(6.0), **COMMON_KW
    )
    left_fn = gen.planners[0].hip_velocity_fn  # FL, phase_offset 0.0
    right_fn = gen.planners[1].hip_velocity_fn  # FR, phase_offset 0.5
    t_probe = 0.25 * gen.T

    dv_left = left_fn(t_probe)
    dv_right = right_fn(t_probe)

    assert dv_left[1] != 0.0
    assert np.isclose(dv_left[1], -dv_right[1])
    # Roll channel only -- sagittal/vertical untouched.
    assert dv_left[0] == 0.0 and dv_left[2] == 0.0


def test_rolling_contact_point_has_no_discontinuity_during_oscillation():
    """V3 (lite): with the oscillation enabled, the world-frame contact point
    should not jump beyond what v_hip*dt allows -- i.e. no artificial slip is
    introduced by the injected velocity (Theory doc Section 4.1 concern)."""
    gen = GaitGenerator3D(
        gait_type="Bound", attitude_osc_amplitude=np.deg2rad(8.0), **COMMON_KW
    )
    planner = gen.planners[0]
    cmd = np.array(planner.generate_trajectory())

    stance_duration = planner.T * planner.stance_duty
    n_stance = int(stance_duration / planner.dt)
    v_hip_bound = np.linalg.norm(planner.velocity) + abs(
        np.deg2rad(8.0) * 2 * np.pi / planner.T
    ) * 0.1  # loose bound: base speed + peak oscillation-induced speed headroom

    positions = []
    for q in cmd[:n_stance]:
        contact = planner.kin.foot_rim_contact_fk(*q)
        positions.append(planner.kin.forward_kinematics(*q, alpha=contact[0], w=0.0))
    positions = np.array(positions)

    deltas = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    assert np.all(deltas < v_hip_bound * planner.dt * 5.0)  # generous slack for DLS/IK residuals
