"""Day 12 Step 11: paper-oriented trajectory metrics.

Plan §18.  Most of what matters here is what the metrics **refuse** to say --
no energy, no CoM, no fabricated zero -- so most of these tests check those
refusals rather than the arithmetic.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_decision_map_2d import load_tables_2d
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import BODY_BASIS
from hybrid_note.scripts.experiments.day12_paper_metrics_2d import (
    COM_METRICS_ABSENT,
    ENERGY_WORDS,
    energy_vocabulary,
    metrics_rows,
    trajectory_metrics_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import COM_BASIS
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (
    plan_terrain_2d,
)
import hybrid_note.scripts.experiments.day12_paper_metrics_2d as module

NOTES = __import__("pathlib").Path(module.__file__).resolve().parents[2] / "notes"


@pytest.fixture(scope="module")
def tables():
    return load_tables_2d(NOTES / "day10-11", NOTES / "day6-7")


@pytest.fixture(scope="module")
def flat_metrics(tables):
    run = plan_terrain_2d(None, tables, samples=61)
    return run, trajectory_metrics_2d("flat", run.trajectory, run.plan,
                                      run.body, run.stability)


#: The platform the obstacle fixtures use, named so assertions about it read
#: as "of the obstacle's order" rather than as a bare literal.
OBSTACLE_HEIGHT_M: float = 0.04


@pytest.fixture(scope="module")
def obstacle_metrics(tables):
    terrain = SharedTerrainSpec2D(
        height_m=OBSTACLE_HEIGHT_M, top_length_m=0.40, x_start_m=1.00,
        obstacle_id="day12_platform")
    run = plan_terrain_2d(terrain, tables, samples=61)
    return run, trajectory_metrics_2d("40mm x 400mm", run.trajectory, run.plan,
                                      run.body, run.stability)


# --------------------------------------------------------------------------
# What these metrics refuse to say
# --------------------------------------------------------------------------


def test_the_module_uses_no_energy_vocabulary():
    """Plan §18: do not infer energy or cost of transport from these."""

    assert energy_vocabulary() == []


def test_the_energy_guard_would_actually_fire(tmp_path):
    """A guard that cannot fail is not a guard."""

    probe = tmp_path / "probe.py"
    probe.write_text("value = 1.0\ncost_of_transport = value / 2.0\n")
    assert energy_vocabulary(probe)


def test_cot_is_in_the_forbidden_vocabulary():
    assert "cot" in ENERGY_WORDS
    assert "cost of transport" in ENERGY_WORDS


def test_com_metrics_are_absent_with_a_reason(flat_metrics):
    """Not zero, and not quietly filled with the body centre's numbers."""

    _, metrics = flat_metrics
    assert metrics.com_z_peak_to_peak_m is None
    assert metrics.com_z_std_m is None
    assert "no whole-robot CoM model" in metrics.com_metrics_absent


def test_body_centre_and_com_stay_labelled_apart(flat_metrics):
    _, metrics = flat_metrics
    assert metrics.body_basis == BODY_BASIS
    assert metrics.com_basis == COM_BASIS
    assert metrics.body_basis != metrics.com_basis


def test_an_unmeasurable_body_excursion_is_none_not_zero(flat_metrics):
    """A 0 would read as "the body never moves"; None says "not measured".

    This used to be asserted on the flat run itself, which had no usable body
    heights because Step 5 was INFEASIBLE there.  The flat run is feasible now
    (log 1.6 and 1.8), so the unmeasurable case has to be constructed -- if it
    were left as it was it would still pass one day and silently stop
    exercising the rule it is named for.
    """

    from dataclasses import replace as _replace

    _, measured = flat_metrics
    unmeasurable = _replace(measured, usable_body_samples=0,
                            body_z_peak_to_peak_m=None, body_z_std_m=None)
    assert unmeasurable.usable_body_samples < 2
    assert unmeasurable.body_z_peak_to_peak_m is None
    assert unmeasurable.body_z_std_m is None
    assert unmeasurable.as_dict()["body_z_peak_to_peak_mm"] is None, (
        "and it must survive the trip to the row as None, not become 0")


def test_a_feasible_flat_run_does_report_a_body_excursion(flat_metrics):
    """The other half: once Step 5 succeeds the metric must appear."""

    _, metrics = flat_metrics
    assert metrics.usable_body_samples >= 2
    assert metrics.body_z_peak_to_peak_m is not None


# --------------------------------------------------------------------------
# The counts plan §18 turns on
# --------------------------------------------------------------------------


def test_flat_ground_has_only_nominal_recovery_swings(flat_metrics):
    _, metrics = flat_metrics
    assert metrics.nominal_recovery_swings > 0
    assert metrics.terrain_transition_swings == 0
    assert metrics.total_swing_segments == metrics.nominal_recovery_swings


def test_an_obstacle_adds_transition_swings_without_changing_the_nominal_ones(
        flat_metrics, obstacle_metrics):
    """Plan §18: the Hybrid-vs-Walk story turns on telling these apart."""

    _, flat = flat_metrics
    _, obstacle = obstacle_metrics
    # Two things this used to assert are no longer true, both because the
    # planner changed under it and both correctly:
    #
    #   * the crossing need not contain a terrain-transition *swing* at all --
    #     the rolling preference (log 1.7) makes that count zero;
    #   * the nominal recovery count is not preserved -- world registration
    #     (log 1.11) has each leg roll to where the obstacle really is, so the
    #     obstacle run has more cycles than the flat one, 18 against 8.
    #
    # What plan section 18 actually needs, and what holds under every one of
    # those rules, is that the two counts **partition** the swings: whatever a
    # crossing is made of, it is counted, and it is counted apart from the
    # nominal recoveries.
    assert obstacle.nominal_recovery_swings > flat.nominal_recovery_swings
    assert (obstacle.total_swing_segments
            == obstacle.nominal_recovery_swings
            + obstacle.terrain_transition_swings)
    assert (flat.total_swing_segments
            == flat.nominal_recovery_swings + flat.terrain_transition_swings)
    assert flat.terrain_transition_swings == 0
    assert flat.transition_roll_time_s == pytest.approx(0.0), (
        "a flat run crosses nothing, by either kind")


def test_the_two_crossing_kinds_are_counted_in_different_fields(
        obstacle_metrics):
    """A crossing is made of swings or of rolls, and the totals say which.

    Named for the property rather than for one strategy: this used to assert a
    ``SWING_SWING`` crossing specifically, and the project now prefers rolling
    (log 1.7), so the strategy-specific form was testing the decision rule
    instead of the counting.
    """

    _, metrics = obstacle_metrics
    swings = metrics.terrain_transition_swings
    rolling = metrics.transition_roll_time_s
    assert swings > 0 or rolling > 0.0, (
        "a crossing is made of something, and it must be counted somewhere")
    if swings == 0:
        assert rolling > 0.0
        assert metrics.transition_roll_distance_m is not None
    else:
        assert rolling == pytest.approx(0.0)
        assert metrics.transition_roll_distance_m == pytest.approx(0.0)


# --------------------------------------------------------------------------
# Leg-seconds, and the overlap
# --------------------------------------------------------------------------


def test_the_leg_seconds_partition_the_run(flat_metrics):
    """Four legs are always in exactly one kind, so the totals add to 4x."""

    _, metrics = flat_metrics
    total = metrics.foot_rim_roll_time_s + metrics.swing_time_s
    assert total == pytest.approx(4.0 * metrics.traversal_duration_s, rel=1e-6)


def test_leg_seconds_are_not_wall_clock(flat_metrics):
    """The distinction the metric exists for: an "any leg" wall-clock total
    would equal the whole run for every kind at once."""

    _, metrics = flat_metrics
    assert metrics.foot_rim_roll_time_s > metrics.traversal_duration_s


def test_the_distance_attributions_are_documented_as_overlapping():
    """The overlap is the whole reason these must not be added up."""

    # Normalised: the docstring wraps, so a raw substring search would be
    # testing the line width rather than the wording.
    source = " ".join(module._mode_time_and_distance.__doc__.split())
    assert "overlap" in source
    assert "must not be summed" in source


def test_traversal_distance_and_duration_are_positive(flat_metrics):
    _, metrics = flat_metrics
    assert metrics.traversal_distance_m > 0.0
    assert metrics.traversal_duration_s > 0.0


# --------------------------------------------------------------------------
# The handoff and stability numbers come from the steps that measured them
# --------------------------------------------------------------------------


def test_the_handoff_numbers_are_the_trajectory_s_own(flat_metrics):
    run, metrics = flat_metrics
    assert metrics.max_joint_discontinuity_rad == pytest.approx(
        run.trajectory.max_joint_discontinuity_rad)
    assert metrics.max_contact_handoff_gap_m == pytest.approx(
        run.trajectory.max_contact_gap_m)


def test_the_stability_numbers_are_step_6_s_own(flat_metrics):
    run, metrics = flat_metrics
    assert metrics.minimum_stability_margin_m == pytest.approx(
        run.stability.minimum_margin_m)
    assert metrics.mean_swing_stability_margin_m > \
        metrics.minimum_stability_margin_m


def test_the_hip_lift_grows_with_the_obstacle(flat_metrics, obstacle_metrics):
    _, flat = flat_metrics
    _, obstacle = obstacle_metrics
    assert obstacle.max_hip_lift_m > flat.max_hip_lift_m
    # Not pinned to the obstacle height any more: a swing crossing lifts the
    # hip by exactly the obstacle, a rolling one by whatever the climb costs
    # (37.80 mm for this 40 mm platform).  The property is that the obstacle
    # is what drives it, and that it is of the obstacle's order.
    assert 0.5 * OBSTACLE_HEIGHT_M < obstacle.max_hip_lift_m < 2.0 * OBSTACLE_HEIGHT_M


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------


def test_rows_are_csv_writable_with_one_header(flat_metrics, obstacle_metrics):
    rows = metrics_rows([flat_metrics[1], obstacle_metrics[1]])
    assert all(len(r) == len(rows[0]) for r in rows)


def test_the_notes_travel_in_the_table(flat_metrics):
    rows = metrics_rows([flat_metrics[1]])
    notes = [r for r in rows if r["row_kind"] == "note"]
    assert any(COM_METRICS_ABSENT in str(r.values()) for r in notes)


def test_the_exported_row_names_leg_seconds_as_such(flat_metrics):
    """A column called ``time_s`` would be read as wall clock."""

    row = flat_metrics[1].as_dict()
    assert "swing_leg_seconds" in row
    assert "foot_rim_roll_leg_seconds" in row
    assert "swing_time_s" not in row
