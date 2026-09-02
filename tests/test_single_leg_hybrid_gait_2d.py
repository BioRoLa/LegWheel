import numpy as np
import pytest

from hybrid_note.scripts.hybrid_gait.single_leg_hybrid_gait import (
    HybridGait2DParameters,
    SingleLegHybridGait2D,
)


def test_default_flat_gait_rolls_then_returns_to_periodic_touchdown_pose():
    trajectory = SingleLegHybridGait2D().plan()
    p = trajectory.parameters

    assert trajectory.phase[0] == "stance"
    assert trajectory.phase[-1] == "swing"
    assert np.allclose(
        trajectory.contact_xy_world_m[trajectory.stance_mask, 1],
        p.current_ground_height_m,
        atol=1e-9,
    )
    assert np.isclose(trajectory.clearance_m.max(), p.swing_height_m, atol=1e-9)
    assert np.all(trajectory.clearance_m >= -1e-9)
    assert np.allclose(
        [trajectory.theta_rad[-1], trajectory.beta_rad[-1]],
        [trajectory.theta_rad[0], trajectory.beta_rad[0]],
        atol=1e-9,
    )
    assert np.allclose(
        trajectory.contact_xy_world_m[-1], trajectory.next_foothold_xy_world_m, atol=1e-9
    )


def test_explicit_raised_foothold_is_hit_without_changing_hip_height():
    parameters = HybridGait2DParameters(
        next_foothold_x_m=0.225,
        next_foothold_height_m=0.015,
        swing_height_m=0.035,
    )
    trajectory = SingleLegHybridGait2D(parameters).plan()

    assert np.allclose(trajectory.hip_xy_world_m[:, 1], parameters.hip_height_m)
    assert np.allclose(trajectory.contact_xy_world_m[-1], [0.225, 0.015], atol=1e-9)
    assert np.allclose(
        trajectory.contact_xy_world_m,
        trajectory.desired_contact_xy_world_m,
        atol=1e-9,
    )


def test_stance_workspace_guard_reports_excessive_requested_travel():
    parameters = HybridGait2DParameters(body_speed_mps=2.0)
    with pytest.raises(ValueError, match="Stance requires"):
        SingleLegHybridGait2D(parameters).plan()


def test_reverse_flat_gait_is_supported():
    trajectory = SingleLegHybridGait2D(
        HybridGait2DParameters(body_speed_mps=-0.10)
    ).plan()

    assert trajectory.touchdown_beta_rad < 0.0
    assert np.allclose(
        trajectory.contact_xy_world_m[trajectory.stance_mask, 1], 0.0, atol=1e-9
    )
    assert np.allclose(
        trajectory.contact_xy_world_m[-1], trajectory.next_foothold_xy_world_m, atol=1e-9
    )


def test_phase_boundary_is_kept_when_dt_does_not_divide_stance_time():
    parameters = HybridGait2DParameters(dt_s=0.03, stance_ratio=0.77)
    trajectory = SingleLegHybridGait2D(parameters).plan()

    boundary = np.flatnonzero(trajectory.stance_mask)[-1]
    assert trajectory.time_s[boundary] == parameters.stance_time_s
    assert np.isclose(trajectory.clearance_m[boundary], 0.0, atol=1e-9)
