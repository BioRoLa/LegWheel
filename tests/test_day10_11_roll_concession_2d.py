"""Day 10--11 Step 4: the rolling side's contract, and the comparison rule.

Cheap tests only.  The expensive claims -- the excursion map itself, where roll
and swing cross, the ``L_top`` sensitivity -- are made by
``day10_11_step4_driver.py``, which has to read Day 6--7's 3.8 MB trajectory
file to make them.

What is worth pinning here is the arithmetic that turns a hip profile into a
number, the refusals that stop an extrapolation from being reported as a
measurement, and -- most of all -- the fact that the three sweeps being
compared were run on the *same* obstacle.  If that ever stops being true, every
number Step 4 produces silently becomes a comparison between two different
terrains.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_concession_2d import BodyRequirementKind
from hybrid_note.scripts.experiments.day10_11_roll_concession_2d import (
    G_M_PER_S2,
    REFERENCE_APPROACH_CLEARANCE_M,
    REFERENCE_TOP_LENGTH_M,
    RollTrajectory2D,
    compare_body_demand_2d,
    hip_excursion_2d,
    swing_obstacle_hip_profile_2d,
    swing_off_hip_profile_2d,
    swing_onto_hip_profile_2d,
)


# --------------------------------------------------------------------------
# The axis every number in this step sits on
# --------------------------------------------------------------------------


def test_roll_and_swing_were_measured_on_the_same_obstacle():
    """Step 4 adds up three sweeps.  They have to be the same column.

    Day 6--7's ``step11r`` froze ``obstacle_width_m = 0.35`` and Step 2 / Step 3
    use ``top_length_m = 0.35``.  Nothing enforces that agreement except this
    test, and without it the roll-vs-swing comparison is between two terrains.
    """

    from hybrid_note.scripts.experiments.day10_11_swing_off_sweep_2d import (
        SwingOffGridSettings2D,
    )
    from hybrid_note.scripts.experiments.day10_11_swing_sweep_2d import (
        SwingGridSettings2D,
    )

    assert SwingGridSettings2D().top_length_m == REFERENCE_TOP_LENGTH_M
    assert SwingOffGridSettings2D().top_length_m == REFERENCE_TOP_LENGTH_M
    assert SwingGridSettings2D().x_start_m == SwingOffGridSettings2D().x_start_m


def test_the_rolling_approach_clearance_is_the_one_day_6_7_froze():
    assert REFERENCE_APPROACH_CLEARANCE_M == pytest.approx(0.04)


# --------------------------------------------------------------------------
# Turning a profile into a number
# --------------------------------------------------------------------------


def test_a_straight_climb_is_measured_exactly():
    excursion = hip_excursion_2d([0.0, 1.0], [0.0, 0.2])
    assert excursion.peak_to_peak_m == pytest.approx(0.2)
    assert excursion.total_rise_m == pytest.approx(0.2)
    assert excursion.total_fall_m == pytest.approx(0.0)
    assert excursion.forward_distance_m == pytest.approx(1.0)
    assert excursion.hip_z_per_forward_distance == pytest.approx(0.2)
    assert excursion.net_change_m == pytest.approx(0.2)


def test_peak_to_peak_and_vertical_path_disagree_on_a_zigzag():
    """The whole reason ``hip_z_per_forward_distance`` uses the path.

    A body moved up, down and up again is moved three times; peak-to-peak sees
    one of those.  Step 4 compares a flat-topped trapezoid against a triangle,
    so the two definitions are not interchangeable.
    """

    excursion = hip_excursion_2d([0.0, 1.0, 2.0, 3.0], [0.0, 0.1, 0.0, 0.1])
    assert excursion.peak_to_peak_m == pytest.approx(0.1)
    assert excursion.total_vertical_path_m == pytest.approx(0.3)
    assert excursion.hip_z_per_forward_distance == pytest.approx(0.1)


def test_vertical_work_counts_only_the_rises():
    """No regeneration: a descent is not repaid, so it is not subtracted."""

    excursion = hip_excursion_2d([0.0, 1.0, 2.0], [0.0, 0.1, 0.0])
    assert excursion.total_rise_m == pytest.approx(0.1)
    assert excursion.vertical_work_per_distance_j_per_kg_m == pytest.approx(
        G_M_PER_S2 * 0.05
    )


def test_a_profile_that_does_not_return_is_flagged():
    assert hip_excursion_2d([0.0, 1.0], [0.0, 0.0]).is_posture_neutral
    assert not hip_excursion_2d([0.0, 1.0], [0.0, 0.04]).is_posture_neutral


def test_a_single_sample_is_not_a_profile():
    with pytest.raises(ValueError):
        hip_excursion_2d([0.0], [0.0])


# --------------------------------------------------------------------------
# The L_top extrapolation, and its refusals
# --------------------------------------------------------------------------


def _flat_topped(feasible: bool = True, flat_z: float = 0.2) -> RollTrajectory2D:
    """A trapezoid: climb, flat top, descend -- the measured rolling shape."""

    return RollTrajectory2D(
        obstacle_height_m=0.1,
        theta_climb_deg=40.0,
        feasible=feasible,
        hip_x_m=(0.0, 0.1, 0.4, 0.5),
        hip_z_m=(0.1, flat_z, flat_z, 0.1),
        stages=("ROLL_UP", "ROLL_UP", "WHEEL_TRANSITION", "ROLL_DOWN"),
        phases=("RIGHT_RIM_ROLL_UP", "RIGHT_RIM_TOP",
                "WHEEL_MODE_TOP_ROLL", "LEFT_RIM_ROLL_DOWN"),
    )


def test_a_longer_top_only_dilutes_the_metric():
    trajectory = _flat_topped()
    base = trajectory.per_forward_at_top_length(REFERENCE_TOP_LENGTH_M)
    longer = trajectory.per_forward_at_top_length(REFERENCE_TOP_LENGTH_M + 0.2)
    assert base == pytest.approx(trajectory.excursion.hip_z_per_forward_distance)
    assert longer < base


def test_the_extrapolation_refuses_below_the_measured_floor():
    """Day 6--7 Step 12R's minimum: shorter than that, the traversal fails.

    Reporting a number there would describe a motion that cannot happen.
    """

    trajectory = _flat_topped()
    assert trajectory.per_forward_at_top_length(0.22, minimum_top_length_m=0.28) is None
    assert trajectory.per_forward_at_top_length(0.30, minimum_top_length_m=0.28) is not None


def test_the_extrapolation_refuses_when_the_flat_top_is_not_flat():
    """The assumption is checked against the cell it is applied to."""

    sloped = RollTrajectory2D(
        obstacle_height_m=0.1, theta_climb_deg=40.0, feasible=True,
        hip_x_m=(0.0, 0.1, 0.4, 0.5), hip_z_m=(0.1, 0.2, 0.22, 0.1),
        stages=("ROLL_UP", "ROLL_UP", "WHEEL_TRANSITION", "ROLL_DOWN"),
        phases=("RIGHT_RIM_ROLL_UP", "WHEEL_MODE_TOP_ROLL",
                "WHEEL_MODE_TOP_ROLL", "LEFT_RIM_ROLL_DOWN"),
    )
    assert sloped.flat_top_hip_z_range_m > 1e-4
    assert sloped.per_forward_at_top_length(0.45) is None


def test_an_infeasible_cell_is_never_extrapolated():
    assert _flat_topped(feasible=False).per_forward_at_top_length(0.45) is None


# --------------------------------------------------------------------------
# The contract
# --------------------------------------------------------------------------


def test_a_feasible_rolling_cell_demands_a_trajectory_not_a_bound():
    cell = _flat_topped().as_cell_concession()
    assert cell.requirement_kind is BodyRequirementKind.TRACK
    assert cell.hip_z_travel_m == pytest.approx(0.1)


def test_a_matched_stage_excludes_the_flat_top():
    """``ROLL_UP`` vs ``SWING_UP`` is the pair; the top is on neither side."""

    trajectory = _flat_topped()
    ascent = trajectory.excursion_for_stages(["ROLL_UP"])
    assert ascent.forward_distance_m == pytest.approx(0.1)
    assert ascent.peak_to_peak_m == pytest.approx(0.1)


# --------------------------------------------------------------------------
# The swing side, reconstructed
# --------------------------------------------------------------------------


def test_the_ascent_lands_the_step_plus_the_lift_above_where_it_started():
    """``min_hip_lift`` is applied to the landing hip, so it adds to the rise."""

    profile = swing_onto_hip_profile_2d(
        height_m=0.10, theta_deg=60.0, approach_hip_x_m=0.0,
        landing_distance_m=0.16, min_hip_lift_m=0.02,
    )
    assert profile.excursion.net_change_m == pytest.approx(0.12, abs=1e-9)
    assert profile.excursion.total_fall_m == pytest.approx(0.0)


def test_the_descent_drops_the_step_less_whatever_the_hold_keeps():
    profile = swing_off_hip_profile_2d(
        height_m=0.10, theta_deg=60.0, takeoff_hip_x_m=0.37,
        landing_hip_x_m=0.60, min_hip_hold_fraction=0.25,
    )
    assert profile.excursion.net_change_m == pytest.approx(-0.075, abs=1e-9)


def test_crossing_the_top_adds_distance_and_no_vertical_path():
    """Which is exactly what ``WHEEL_MODE_TOP_ROLL`` does for the roll side."""

    up = swing_onto_hip_profile_2d(
        height_m=0.10, theta_deg=60.0, approach_hip_x_m=0.0,
        landing_distance_m=0.16, min_hip_lift_m=0.0,
    )
    down = swing_off_hip_profile_2d(
        height_m=0.10, theta_deg=60.0, takeoff_hip_x_m=0.37,
        landing_hip_x_m=0.60, min_hip_hold_fraction=0.0,
    )
    whole = swing_obstacle_hip_profile_2d(up, down)
    assert whole.total_vertical_path_m == pytest.approx(0.20, abs=1e-9)
    assert whole.forward_distance_m > (
        up.excursion.forward_distance_m + down.excursion.forward_distance_m
    )


def test_a_takeoff_behind_the_landing_is_not_a_crossing():
    up = swing_onto_hip_profile_2d(
        height_m=0.10, theta_deg=60.0, approach_hip_x_m=0.0,
        landing_distance_m=0.16, min_hip_lift_m=0.0,
    )
    down = swing_off_hip_profile_2d(
        height_m=0.10, theta_deg=60.0, takeoff_hip_x_m=0.20,
        landing_hip_x_m=0.60, min_hip_hold_fraction=0.0,
    )
    with pytest.raises(ValueError):
        swing_obstacle_hip_profile_2d(up, down)


# --------------------------------------------------------------------------
# Spec 5.3's rule
# --------------------------------------------------------------------------


class _Roll:
    """Minimal stand-in: the rule only reads ``requirement_kind``."""

    def __init__(self, kind):
        self.requirement_kind = kind


def test_the_rule_prefers_roll_only_when_it_moves_the_body_less():
    excursion = hip_excursion_2d([0.0, 1.0], [0.0, 0.05])
    verdict, _ = compare_body_demand_2d(
        _Roll(BodyRequirementKind.TRACK), 0.08, roll_excursion=excursion
    )
    assert verdict == -1


def test_a_tie_in_magnitude_goes_to_the_swing():
    """Because the swing may meet its bound with any shape, and roll may not.

    That freedom is real and unpriced, so equal magnitudes are not equal
    demands.  A rule that returned 0 here would hide the asymmetry the whole
    step exists to establish.
    """

    excursion = hip_excursion_2d([0.0, 1.0], [0.0, 0.08])
    verdict, reason = compare_body_demand_2d(
        _Roll(BodyRequirementKind.TRACK), 0.08, roll_excursion=excursion
    )
    assert verdict == 1
    assert "shape" in reason or "extremum" in reason


def test_there_is_nothing_to_compare_when_either_side_is_infeasible():
    excursion = hip_excursion_2d([0.0, 1.0], [0.0, 0.05])
    assert compare_body_demand_2d(
        _Roll(BodyRequirementKind.NONE), 0.08, roll_excursion=excursion
    )[0] is None
    assert compare_body_demand_2d(
        _Roll(BodyRequirementKind.TRACK), None, roll_excursion=excursion
    )[0] is None
