import numpy as np
import pytest

from hybrid_note.scripts.hybrid_gait.multi_rim_hybrid_gait import (
    MultiRimHybridSwing2D,
    MultiRimSwing2DParameters,
)


def test_default_transition_lands_on_requested_rims_and_clears_ground():
    trajectory = MultiRimHybridSwing2D().plan()

    assert trajectory.lowest_contact_state[0] == "left_rim"
    assert trajectory.lowest_contact_state[-1] == "right_rim"
    assert set(trajectory.lowest_contact_state) == {"left_rim", "foot_rim", "right_rim"}
    assert np.allclose(
        trajectory.lowest_point_xy_world_m[0],
        trajectory.current_foothold_xy_world_m,
        atol=2e-5,
    )
    assert np.allclose(
        trajectory.lowest_point_xy_world_m[-1],
        trajectory.next_foothold_xy_world_m,
        atol=2e-5,
    )
    assert trajectory.clearance_m[1:-1].min() >= -2e-5
    assert np.isclose(trajectory.clearance_m.max(), 0.03, atol=2e-4)


@pytest.mark.parametrize("target_rim", ["left_rim", "right_rim"])
def test_foot_rim_can_swing_to_either_side_rim(target_rim):
    parameters = MultiRimSwing2DParameters(
        start_rim="foot_rim",
        target_rim=target_rim,
        dt_s=0.05,
        arc_samples=181,
    )
    trajectory = MultiRimHybridSwing2D(parameters).plan()

    assert trajectory.lowest_contact_state[0] == "foot_rim"
    assert trajectory.lowest_contact_state[-1] == target_rim
    assert trajectory.clearance_m[1:-1].min() >= -2e-5


def test_side_rim_reports_when_hip_is_too_high_for_flat_ground():
    parameters = MultiRimSwing2DParameters(hip_height_m=0.30)
    with pytest.raises(ValueError, match="Cannot place left_rim"):
        MultiRimHybridSwing2D(parameters).plan()


def test_same_start_and_target_rim_is_rejected():
    parameters = MultiRimSwing2DParameters(start_rim="foot_rim", target_rim="foot_rim")
    with pytest.raises(ValueError, match="must be different"):
        MultiRimHybridSwing2D(parameters)
