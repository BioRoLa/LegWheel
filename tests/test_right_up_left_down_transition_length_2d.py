"""Day 6--7 Step 12R: obstacle-top length and transition distance."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

from hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d import (  # noqa: E402
    check_right_up_left_down_traversal,
)
from hybrid_note.scripts.experiments.right_up_left_down_sweep_2d import (  # noqa: E402
    SweepSettings2D,
)
from hybrid_note.scripts.experiments.right_up_left_down_transition_length_2d import (  # noqa: E402
    TransitionMeasurement2D,
    measure_transition_distances,
    measure_transition_from_rows,
    minimum_top_length_for_full_traversal,
    plot_top_length_sweep_2d,
    plot_transition_budget_2d,
    run_top_length_sweep_2d,
    write_transition_length_csv,
)

ARC_SAMPLES = 61
HEIGHT_M = 0.10
THETA_RAD = np.deg2rad(60.0)


@pytest.fixture(scope="module")
def settings() -> SweepSettings2D:
    return SweepSettings2D(arc_samples=ARC_SAMPLES, obstacle_width_m=0.35)


@pytest.fixture(scope="module")
def traversal(settings):
    return check_right_up_left_down_traversal(
        obstacle=settings.obstacle(HEIGHT_M),
        initial_state=settings.initial_state_for(HEIGHT_M, THETA_RAD),
        theta_climb=THETA_RAD,
        constraints=settings.constraints,
    )


@pytest.fixture(scope="module")
def measurement(traversal) -> TransitionMeasurement2D:
    return measure_transition_distances(traversal)


# --------------------------------------------------------------------------
# The measurement
# --------------------------------------------------------------------------


def test_transition_parts_add_up_to_the_whole(measurement):
    assert measurement.reached_left_rim_ready
    assert measurement.l_transition_m == pytest.approx(
        measurement.retract_forward_distance_m
        + measurement.wheel_mode_forward_distance_m
    )
    assert measurement.required_top_length_m == pytest.approx(
        measurement.leading_edge_margin_m + measurement.l_transition_m
    )


def test_measured_transition_agrees_with_the_handover_frame(traversal, measurement):
    """Two independent routes to L_transition must give the same number.

    The measurement reads phase boundaries off the trajectory; the traversal
    records the handover frame's own forward displacement.  Agreement is what
    makes the phase labels trustworthy as measurement points.
    """

    assert measurement.l_transition_m == pytest.approx(
        traversal.l_transition_m, abs=1e-9
    )


def test_every_distance_is_positive_and_fits_inside_the_top(measurement):
    assert measurement.leading_edge_margin_m > 0.0
    assert measurement.retract_forward_distance_m > 0.0
    assert measurement.wheel_mode_forward_distance_m > 0.0
    assert measurement.trailing_edge_entry_margin_m > 0.0
    assert measurement.required_top_length_m < measurement.obstacle_top_length_m
    assert measurement.spare_top_length_m == pytest.approx(
        measurement.trailing_edge_entry_margin_m
    )


def test_a_run_that_never_hands_over_reports_no_transition():
    """A missing measurement is reported as missing, not as zero."""

    rows = [
        {"phase": "APPROACH", "contact_x_m": -0.05},
        {"phase": "RIGHT_RIM_ROLL_UP", "contact_x_m": 0.10},
        {"phase": "RETRACT_TO_WHEEL", "contact_x_m": 0.12},
    ]
    measured = measure_transition_from_rows(
        rows, obstacle_height_m=0.10, theta_climb_deg=60.0,
        obstacle_x_start_m=0.10, obstacle_top_length_m=0.35,
        full_traversal_success=False, failure_stage="LEFT_RIM_TRANSITION_FAIL",
    )
    assert not measured.reached_left_rim_ready
    assert measured.l_transition_m is None
    assert measured.required_top_length_m is None
    assert measured.spare_top_length_m is None
    assert measured.leading_edge_margin_m == pytest.approx(0.02)


def test_measurement_rejects_a_foreign_object():
    with pytest.raises(TypeError):
        measure_transition_distances(object())


# --------------------------------------------------------------------------
# Requirement A is not gated by requirement C
# --------------------------------------------------------------------------


def test_roll_up_succeeds_on_a_top_far_too_short_to_traverse(settings, measurement):
    """The headline separation: a short top blocks C without blocking A.

    The obstacle here is barely half of the measured required_top_length, so
    the handover cannot possibly finish before the trailing corner -- yet the
    leg still climbs onto the top exactly as it does on a long obstacle.
    """

    short_m = 0.5 * measurement.required_top_length_m
    result = check_right_up_left_down_traversal(
        obstacle=settings.obstacle(HEIGHT_M).__class__(
            x_start_m=settings.obstacle_x_start_m, width_m=short_m,
            height_m=HEIGHT_M, arc_samples=ARC_SAMPLES,
        ),
        initial_state=settings.initial_state_for(HEIGHT_M, THETA_RAD),
        theta_climb=THETA_RAD,
        constraints=settings.constraints,
    )
    assert result.roll_up_success, "the roll-up must not depend on the top length"
    assert not result.full_success
    assert result.failure_stage is not None


def test_transition_distance_does_not_depend_on_where_the_corner_is(settings):
    """L_transition is a property of the leg, not of the obstacle length.

    The corner only decides where the motion stops, so two obstacles that both
    leave room for the handover must measure the same transition distance.
    """

    measured = []
    for width in (0.32, 0.40):
        obstacle = settings.obstacle(HEIGHT_M).__class__(
            x_start_m=settings.obstacle_x_start_m, width_m=width,
            height_m=HEIGHT_M, arc_samples=ARC_SAMPLES,
        )
        result = check_right_up_left_down_traversal(
            obstacle=obstacle,
            initial_state=settings.initial_state_for(HEIGHT_M, THETA_RAD),
            theta_climb=THETA_RAD, constraints=settings.constraints,
        )
        measured.append(measure_transition_distances(result))
    assert measured[0].reached_left_rim_ready
    assert measured[1].reached_left_rim_ready
    assert measured[0].l_transition_m == pytest.approx(
        measured[1].l_transition_m, abs=1e-9
    )
    assert measured[0].required_top_length_m == pytest.approx(
        measured[1].required_top_length_m, abs=1e-9
    )
    # Only the room left over differs.
    assert measured[1].trailing_edge_entry_margin_m > (
        measured[0].trailing_edge_entry_margin_m
    )


# --------------------------------------------------------------------------
# The top-length sweep
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def top_length_cases(settings, measurement):
    required = measurement.required_top_length_m
    return run_top_length_sweep_2d(
        [(HEIGHT_M, THETA_RAD)],
        (round(required - 0.05, 4), round(required + 0.02, 4)),
        settings=settings, max_workers=2, progress=False,
    )


def test_sweep_reports_one_row_per_top_length(top_length_cases):
    case = top_length_cases[0]
    assert len(case.rows) == 2
    assert [row["obstacle_top_length_m"] for row in case.rows] == sorted(
        row["obstacle_top_length_m"] for row in case.rows
    )
    required = {
        "obstacle_height_m", "theta_climb_deg", "L_transition_m",
        "obstacle_top_length_m", "full_traversal_success", "failure_stage",
        "failure_reason",
    }
    for row in case.rows:
        assert required <= set(row)


def test_a_top_shorter_than_required_cannot_complete(top_length_cases, measurement):
    """This is the operative statement of requirement C."""

    case = top_length_cases[0]
    too_short = [
        row for row in case.rows
        if row["obstacle_top_length_m"] < measurement.required_top_length_m
    ]
    assert too_short
    for row in too_short:
        assert not row["full_traversal_success"]
        # ...but the climb itself was never the problem.
        assert row["roll_up_success"]


def test_sweep_rejects_degenerate_input(settings):
    with pytest.raises(ValueError, match="at least one case"):
        run_top_length_sweep_2d([], (0.30,), settings=settings, progress=False)
    with pytest.raises(ValueError, match="at least one top length"):
        run_top_length_sweep_2d(
            [(HEIGHT_M, THETA_RAD)], (), settings=settings, progress=False
        )
    with pytest.raises(ValueError, match="must be positive"):
        run_top_length_sweep_2d(
            [(HEIGHT_M, THETA_RAD)], (-0.3,), settings=settings, progress=False
        )


def test_minimum_top_length_takes_the_largest_per_case_minimum():
    """One demo obstacle must satisfy every case shown on it."""

    class _Case:
        def __init__(self, height, theta, minimum):
            self.obstacle_height_m = height
            self.theta_climb_rad = theta
            self.measurement = None
            self._minimum = minimum

        theta_climb_deg = property(lambda self: float(np.rad2deg(self.theta_climb_rad)))
        minimum_top_length_m = property(lambda self: self._minimum)
        maximum_top_length_m = property(lambda self: None)

    summary = minimum_top_length_for_full_traversal(
        [_Case(0.06, THETA_RAD, 0.21), _Case(0.10, THETA_RAD, 0.27)]
    )
    assert summary["minimum_top_length_for_full_right_up_left_down_demo"] == 0.27
    assert summary["smallest_single_case_minimum_m"] == 0.21
    assert summary["cases_with_no_feasible_top_length"] == []

    unusable = minimum_top_length_for_full_traversal([_Case(0.16, THETA_RAD, None)])
    assert unusable["minimum_top_length_for_full_right_up_left_down_demo"] is None
    (height, theta_deg), = unusable["cases_with_no_feasible_top_length"]
    assert height == 0.16
    assert theta_deg == pytest.approx(60.0)


def test_csv_export(top_length_cases, tmp_path):
    paths = write_transition_length_csv(
        top_length_cases, tmp_path / "sweep.csv",
        measurement_path=tmp_path / "measurements.csv",
        minimum_path=tmp_path / "minimum.csv",
    )
    assert len(paths) == 3
    rows = sum(len(case.rows) for case in top_length_cases)
    assert len(paths[0].read_text(encoding="utf-8").splitlines()) == rows + 1


def test_plots_build(top_length_cases, measurement):
    ax = plot_transition_budget_2d([measurement])
    plt.close(ax.figure)
    ax = plot_top_length_sweep_2d(top_length_cases)
    plt.close(ax.figure)
    with pytest.raises(ValueError, match="no measurement reached"):
        plot_transition_budget_2d([])
