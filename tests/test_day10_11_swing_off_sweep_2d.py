"""Day 10--11 Step 3: the descent sweep's contract.

Cheap tests only.  The expensive claims -- the map itself, the 160 mm
docstring measurement, the ``arrival = ROLL_UP`` coupling -- are made by
``day10_11_step3_driver.py``, which has to run planner sweeps to make them.

What is worth pinning here is the shape of the search rather than its answers:
that a descent is priced in hip *hold* and never in hip lift, that the grid
result is comparable and cannot carry the greedy signature, and that the
duration ladder the descent needs (and the ascent did not) is actually walked.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_concession_2d import (
    BindingCeiling,
    BodyRequirementKind,
    ConcessionSource,
)
from hybrid_note.scripts.experiments.day10_11_swing_off_sweep_2d import (
    DEFAULT_HEIGHTS_MM,
    DEFAULT_TAKEOFF_DISTANCES_M,
    RollUpExitPose2D,
    SwingOffGridSettings2D,
    minimum_roll_up_descent_2d,
    minimum_swing_off_concession_2d,
    swing_off_rows,
)


# --------------------------------------------------------------------------
# Settings and axes
# --------------------------------------------------------------------------


def test_the_descent_shares_the_ascents_heights():
    """Both halves of a sequence have to be priced on one axis to be added up."""

    from hybrid_note.scripts.experiments.day10_11_swing_sweep_2d import (
        DEFAULT_HEIGHTS_MM as ASCENT_HEIGHTS,
    )

    assert DEFAULT_HEIGHTS_MM == ASCENT_HEIGHTS


def test_the_hold_ladder_is_fractions_and_spans_the_whole_drop():
    ladder = SwingOffGridSettings2D().hip_hold_ladder
    assert ladder[0] == 0.0
    assert ladder[-1] == 1.0
    assert all(0.0 <= f <= 1.0 for f in ladder)
    assert list(ladder) == sorted(ladder)


def test_the_duration_ladder_starts_at_no_extension():
    """A cell that needs no extra time must report ``duration_scale == 1.0``."""

    ladder = SwingOffGridSettings2D().duration_scale_ladder
    assert ladder[0] == 1.0
    assert list(ladder) == sorted(ladder)


def test_takeoff_distances_are_a_top_length_coordinate():
    """They have to be comparable with the rolling side's L_transition band."""

    assert min(DEFAULT_TAKEOFF_DISTANCES_M) < 0.20
    assert max(DEFAULT_TAKEOFF_DISTANCES_M) > 0.20


# --------------------------------------------------------------------------
# The concession a descent produces
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def easy_cell():
    """A low step from a normal takeoff -- feasible, and cheap to compute."""

    return minimum_swing_off_concession_2d(0.06, 0.16, SwingOffGridSettings2D())


def test_a_descent_is_priced_in_hip_hold_never_in_hip_lift(easy_cell):
    assert easy_cell.concession.direction == "off"
    assert easy_cell.concession.min_hip_lift_m is None
    assert easy_cell.concession.min_hip_hold_fraction is not None


def test_a_grid_descent_is_comparable_and_never_looks_greedy(easy_cell):
    assert easy_cell.concession.source is ConcessionSource.GRID_MINIMUM
    assert easy_cell.concession.is_comparable
    assert not easy_cell.concession.greedy_backtrack_suspected


def test_a_feasible_descent_has_no_binding_ceiling(easy_cell):
    assert easy_cell.concession.feasible
    assert easy_cell.concession.binding_ceiling is BindingCeiling.NONE
    assert easy_cell.concession.requirement_kind is BodyRequirementKind.LOWER_BOUND


def test_a_free_takeoff_leaves_on_the_foot_rim(easy_cell):
    """A standing pose always contacts at ``foot_rim, alpha = 0`` -- Step 0's 80/80.

    It matters here because Step 2b showed the rim the leg is on decides which
    alpha seams a swing has to cross.
    """

    assert easy_cell.takeoff_rim == "foot_rim"
    assert easy_cell.takeoff_alpha_deg == pytest.approx(0.0, abs=1e-6)


def test_the_row_reports_the_hold_in_millimetres_as_well_as_a_fraction(easy_cell):
    row = swing_off_rows([easy_cell])[0]
    assert row["min_hip_hold_mm"] == pytest.approx(
        row["min_hip_hold_fraction"] * row["obstacle_mm"]
    )


def test_the_takeoff_distance_is_measured_from_the_trailing_edge(easy_cell):
    settings = SwingOffGridSettings2D()
    x_max = settings.x_start_m + settings.top_length_m
    assert easy_cell.takeoff_hip_x_m == pytest.approx(x_max - easy_cell.takeoff_distance_m)


# --------------------------------------------------------------------------
# arrival = ROLL_UP
# --------------------------------------------------------------------------


def test_an_ascent_that_never_crested_produces_a_stance_refusal():
    """No swing is attempted, and the reason says so rather than blaming the swing."""

    pose = RollUpExitPose2D(
        height_m=0.12, theta_climb_deg=50.0, reached=False,
        feasible_traversal=False, note="the traversal never produced an accepted stage.",
    )
    cell = minimum_roll_up_descent_2d(pose, 0.32, SwingOffGridSettings2D())
    assert not cell.concession.feasible
    assert cell.concession.binding_ceiling is BindingCeiling.STANCE
    assert cell.evaluations == 0
    assert "ROLL_UP exit" in cell.concession.reason


def test_a_roll_up_arrival_is_labelled_and_keeps_the_climb_theta():
    """The cell has to say which ascent it belongs to; §2.7's rows are not interchangeable."""

    pose = RollUpExitPose2D(
        height_m=0.10, theta_climb_deg=60.0, reached=True, feasible_traversal=True,
        theta_rad=float(np.deg2rad(60.0)), beta_rad=float(np.deg2rad(-68.7)),
        hip_x_m=0.1346, hip_z_m=0.2697, rim="right_rim",
    )
    # One rung per ladder: this test is about the labelling and the derived
    # takeoff distance, not about whether the cell is feasible, and the full
    # grid costs minutes on a cell that fails.
    cheap = SwingOffGridSettings2D(
        hip_hold_ladder=(0.0,), touchdown_drop_ladder_m=(0.0,),
        duration_scale_ladder=(1.0,),
    )
    cell = minimum_roll_up_descent_2d(pose, 0.32, cheap)
    assert cell.arrival == "ROLL_UP"
    assert cell.theta_deg == pytest.approx(60.0)
    assert cell.top_length_m == pytest.approx(0.32)
    # The takeoff distance is derived, not chosen: it is whatever the top leaves.
    assert cell.takeoff_distance_m == pytest.approx(0.10 + 0.32 - 0.1346)
