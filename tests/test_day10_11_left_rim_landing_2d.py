"""Day 10--11 Step 2b: the invariants of landing in ``LEFT_RIM_READY``.

Two kinds of test, kept apart on purpose.

The contract tests run no geometry at all: they check that a *pinned* landing
is typed as one, and that the comparison refuses to rank it against an ordinary
swing's lower bound.  That refusal is the whole point of the new requirement
kind -- "the hip must be at exactly 143.8 mm" and "the hip must be at least
40 mm higher than usual" are not two numbers on one scale.

The geometry tests are deliberately coarse: a 5-degree beta step over the part
of the axis the window is known to live on.  They are here to catch a change
that moves or closes the window, not to reproduce the driver's sweep -- that is
``day10_11_step2b_driver.py``'s job, and it costs minutes rather than seconds.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_concession_2d import (
    BindingCeiling,
    BodyRequirementKind,
    ConcessionSource,
    SwingConcession2D,
    compare_concessions,
)
from hybrid_note.scripts.experiments.day10_11_left_rim_landing_2d import (
    LEFT_RIM_ALPHA_LIMITS_DEG,
    LEFT_RIM_READY_THETA_RAD,
    choose_landing_beta_2d,
    left_rim_beta_window_2d,
    left_rim_landing_scene_2d,
    left_rim_ready_from_landing_2d,
    predicted_pivot_deg_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import SharedTerrainSpec2D


# --------------------------------------------------------------------------
# The contract: a pinned landing is not a lower bound
# --------------------------------------------------------------------------


def _pinned(**overrides):
    payload = dict(
        feasible=True,
        direction="onto",
        obstacle_height_m=0.10,
        source=ConcessionSource.GRID_MINIMUM,
        binding_ceiling=BindingCeiling.NONE,
        min_liftoff_rise_m=0.0,
        min_clearance_m=0.004,
        pinned_hip_above_surface_m=0.1438,
    )
    payload.update(overrides)
    return SwingConcession2D(**payload)


def test_a_pinned_landing_reports_a_pinned_requirement():
    assert _pinned().requirement_kind is BodyRequirementKind.PINNED


def test_an_ordinary_landing_still_reports_a_lower_bound():
    concession = _pinned(pinned_hip_above_surface_m=None, min_hip_lift_m=0.04)
    assert concession.requirement_kind is BodyRequirementKind.LOWER_BOUND


def test_an_infeasible_pinned_landing_demands_nothing():
    concession = _pinned(
        feasible=False, binding_ceiling=BindingCeiling.FIT, min_liftoff_rise_m=None
    )
    assert concession.requirement_kind is BodyRequirementKind.NONE


def test_a_pinned_landing_refuses_a_hip_lift():
    """The knob does not exist here; accepting one would hide that."""

    with pytest.raises(ValueError, match="no hip-lift freedom"):
        _pinned(min_hip_lift_m=0.02)


def test_pinned_and_lower_bound_may_not_be_ranked_against_each_other():
    ordinary = _pinned(pinned_hip_above_surface_m=None, min_hip_lift_m=0.04)
    with pytest.raises(ValueError, match="different shapes"):
        compare_concessions(_pinned(), ordinary)


def test_two_pinned_landings_may_be_ranked():
    cheaper = _pinned(min_clearance_m=0.006)
    dearer = _pinned(min_clearance_m=0.002)
    assert compare_concessions(cheaper, dearer) == -1


# --------------------------------------------------------------------------
# The geometry: the window, the placement, and the precondition
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def window():
    spec = SharedTerrainSpec2D(height_m=0.10)
    return left_rim_beta_window_2d(
        spec, beta_range_deg=(-320.0, -175.0), sample_step_deg=5.0
    )


def test_the_left_rim_window_exists_and_is_contiguous(window):
    assert window.samples
    assert window.contiguous
    assert window.width_deg > 90.0


def test_the_landing_hip_height_does_not_depend_on_beta(window):
    """At 17 degrees the leg is very nearly a circle, which is why the hip is pinned."""

    heights = [sample.hip_above_top_m for sample in window.samples]
    assert max(heights) - min(heights) < 1e-4


def test_every_window_sample_sits_inside_the_left_rim_arc(window):
    low, high = LEFT_RIM_ALPHA_LIMITS_DEG
    assert all(low <= sample.alpha_deg <= high for sample in window.samples)


def test_the_landing_is_placed_where_it_was_asked_to_be(window):
    spec = SharedTerrainSpec2D(height_m=0.10)
    wanted = spec.x_max_m - 0.04
    _, realised = left_rim_landing_scene_2d(
        spec, contact_x_m=wanted, beta_rad=float(np.deg2rad(window.best_beta_deg))
    )
    assert abs(realised - wanted) < 1e-6


def test_a_swing_placed_pose_passes_step_8rs_own_precondition(window):
    """The precondition is Day 6--7's function, not a re-statement of it."""

    spec = SharedTerrainSpec2D(height_m=0.10)
    scene, _ = left_rim_landing_scene_2d(
        spec, contact_x_m=spec.x_max_m - 0.04,
        beta_rad=float(np.deg2rad(window.best_beta_deg)),
    )
    readiness = left_rim_ready_from_landing_2d(spec, scene, run_descent=False)
    assert readiness.ready
    assert readiness.contact_rim == "left_rim"
    assert np.isclose(scene.theta_rad, LEFT_RIM_READY_THETA_RAD, atol=1e-12)


def test_the_beta_choice_buys_the_pivot_before_it_buys_seam_margin(window):
    """Lexicographic, with no tunable weight -- physics first, robustness second."""

    required = 100.0
    choice = choose_landing_beta_2d(window, required)
    assert choice.sufficient
    assert choice.sample.rim_budget_deg >= required
    affordable = [s for s in window.samples if s.rim_budget_deg >= required]
    assert choice.sample.seam_margin_deg == max(s.seam_margin_deg for s in affordable)


def test_an_unaffordable_pivot_is_reported_as_the_rim_running_out(window):
    """Not as a swing failure: a rolling arrival would hit the same wall."""

    choice = choose_landing_beta_2d(window, 1000.0)
    assert not choice.sufficient
    assert "left-rim arc" in choice.reason


def test_a_taller_step_forces_the_landing_closer_to_the_seam(window):
    """The quantitative answer to spec §2.8(c), as a monotonicity."""

    margins = []
    for height_m in (0.06, 0.10, 0.14, 0.20):
        spec = SharedTerrainSpec2D(height_m=height_m)
        choice = choose_landing_beta_2d(window, predicted_pivot_deg_2d(spec))
        assert choice.sufficient
        margins.append(choice.sample.seam_margin_deg)
    assert margins == sorted(margins, reverse=True)
