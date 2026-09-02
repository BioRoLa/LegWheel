"""Day 10--11 Step 5: the ``SWING_OVER`` sweep's contract.

Cheap tests only.  The map itself -- which heights and top lengths an
over-swing clears -- is made by ``day10_11_step5_driver.py``.

The two claims worth pinning here are the ones the module's speed and its
fairness rest on: that the front-face bisection really is independent of the
top length (which is what makes the cache correct, not merely fast), and that
``#5`` is not quietly handed an easier corridor than ``#4``.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
    approach_hip_x_for_clearance_2d,
)
from hybrid_note.scripts.experiments.day10_11_swing_over_2d import (
    MIN_STANCE_HIP_M,
    SwingOverSettings2D,
    minimum_swing_over_2d,
)


# --------------------------------------------------------------------------
# The premise the cache rests on
# --------------------------------------------------------------------------


def test_the_front_face_bisection_ignores_the_top_length():
    """Why ``_approach_hip_x_m`` may drop ``top_length`` from its key.

    The take-off stance sits in front of the leading edge, so the bisection
    never looks at the trailing one.  If this ever stops holding, the cache
    returns a hip x for the wrong obstacle and every ``#5`` cell in that column
    is silently wrong -- so it is asserted rather than assumed.
    """

    theta = float(np.deg2rad(60.0))
    short = SharedTerrainSpec2D(height_m=0.06, top_length_m=0.05,
                                x_start_m=0.10, arc_samples=121)
    long = SharedTerrainSpec2D(height_m=0.06, top_length_m=0.35,
                               x_start_m=0.10, arc_samples=121)
    assert approach_hip_x_for_clearance_2d(short, theta, 0.04) == pytest.approx(
        approach_hip_x_for_clearance_2d(long, theta, 0.04), abs=1e-9
    )


# --------------------------------------------------------------------------
# Fairness against the other candidates
# --------------------------------------------------------------------------


def test_the_over_swing_gets_the_same_apex_corridor_as_the_descent():
    """A looser clearance would make ``#5`` win by being measured differently."""

    from hybrid_note.scripts.experiments.day10_11_swing_off_sweep_2d import (
        SwingOffGridSettings2D,
    )

    assert SwingOverSettings2D().apex_clearance_m == (
        SwingOffGridSettings2D().apex_clearance_m
    )


def test_the_over_swing_stands_on_the_same_terrain_column():
    from hybrid_note.scripts.experiments.day10_11_swing_sweep_2d import (
        SwingGridSettings2D,
    )

    assert SwingOverSettings2D().x_start_m == SwingGridSettings2D().x_start_m


# --------------------------------------------------------------------------
# The ladders
# --------------------------------------------------------------------------


def test_theta_is_walked_from_the_most_retracted_upward():
    """The first success wins, so the ladder has to be ordered by cost.

    A more extended stance holds the body higher for the whole crossing and
    lengthens the stride the gait must supply, so it is strictly worse.
    """

    ladder = SwingOverSettings2D().theta_ladder_deg
    assert list(ladder) == sorted(ladder)


def test_the_duration_ladder_starts_at_no_extension():
    ladder = SwingOverSettings2D().duration_scale_ladder
    assert ladder[0] == 1.0
    assert list(ladder) == sorted(ladder)


def test_the_clearance_ladder_is_a_repair_not_a_cost_axis():
    """It sits inside the theta loop, so it never trades against the stance."""

    settings = SwingOverSettings2D()
    assert len(settings.clearance_ladder_m) >= 2
    assert list(settings.clearance_ladder_m) == sorted(settings.clearance_ladder_m)


# --------------------------------------------------------------------------
# What an infeasible cell reports
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def impossible():
    """An obstacle far taller than the leg: no stance is even valid.

    One rung per ladder -- this is about what the refusal says, not about how
    long the search takes to give up.
    """

    cheap = SwingOverSettings2D(
        theta_ladder_deg=(60.0,), clearance_ladder_m=(0.04,),
        duration_scale_ladder=(1.0,),
    )
    return minimum_swing_over_2d(0.40, 0.10, cheap)


def test_an_impossible_cell_names_a_failure_and_claims_no_plan(impossible):
    assert not impossible.feasible
    assert impossible.failure is not None
    assert impossible.theta_deg is None
    assert impossible.stride_m is None


def test_a_stance_refusal_and_a_swing_failure_stay_distinguishable(impossible):
    """A stance that penetrates the front face is not the swing failing.

    Conflating them would put a collision count on the swing planner that it
    never earned.  The invariant is the pairing: ``STANCE_INVALID`` means the
    planner was never reached, and any other name means it was.
    """

    if impossible.failure == "STANCE_INVALID":
        assert impossible.evaluations == 0
    else:
        assert impossible.evaluations > 0
        assert impossible.failure.isupper()


def test_an_infeasible_cell_has_no_stance_price(impossible):
    assert impossible.stance_hip_above_min_m is None
    assert impossible.as_dict()["stance_hip_above_min_mm"] is None


def test_the_retracted_stance_height_is_the_wheel_radius():
    """``MIN_STANCE_HIP_M`` is the floor the stance price is measured from."""

    assert MIN_STANCE_HIP_M == pytest.approx(0.1438, abs=1e-4)
