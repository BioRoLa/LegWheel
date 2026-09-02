"""Day 10--11 Step 1: the concession contract's invariants.

These are the rules the type enforces on its own, with no planner runs, so they
stay fast.  The expensive claim -- that every Day 8--9 showcase converts without
losing information -- is checked by ``day10_11_step1_driver.py``, which has to
run the showcases to make it.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_concession_2d import (
    BindingCeiling,
    BodyRequirementKind,
    ConcessionSource,
    RollCellConcession2D,
    RollConcession2D,
    SwingConcession2D,
    adjustment_strings_from_concession,
    compare_concessions,
    roll_concession_from_cells,
)


def _swing(
    *,
    feasible=True,
    direction="onto",
    hip_lift_m=0.04,
    hold_fraction=None,
    clearance_m=0.0015,
    ceiling=BindingCeiling.NONE,
    source=ConcessionSource.GRID_MINIMUM,
    height_m=0.14,
):
    return SwingConcession2D(
        feasible=feasible,
        direction=direction,
        obstacle_height_m=height_m,
        source=source,
        binding_ceiling=ceiling,
        min_hip_lift_m=hip_lift_m if direction == "onto" else None,
        min_hip_hold_fraction=hold_fraction if direction == "off" else None,
        min_clearance_m=clearance_m,
    )


def _roll_cell(theta_deg, *, feasible=True, hip_min=0.24, hip_max=0.30, margin=0.004):
    return RollCellConcession2D(
        feasible=feasible,
        obstacle_height_m=0.10,
        top_length_m=0.35,
        theta_climb_deg=theta_deg,
        approach_clearance_m=0.04,
        hip_z_min_m=hip_min,
        hip_z_max_m=hip_max,
        hip_z_start_m=hip_min,
        min_collision_margin_m=margin,
    )


# --------------------------------------------------------------------------
# Swing side
# --------------------------------------------------------------------------


def test_direction_decides_which_hip_knob_is_legal():
    with pytest.raises(ValueError, match="hip lift, not hip hold"):
        SwingConcession2D(
            feasible=True, direction="onto", obstacle_height_m=0.1,
            source=ConcessionSource.GRID_MINIMUM, binding_ceiling=BindingCeiling.NONE,
            min_hip_hold_fraction=0.5,
        )
    with pytest.raises(ValueError, match="hip hold, not hip lift"):
        SwingConcession2D(
            feasible=True, direction="off", obstacle_height_m=0.1,
            source=ConcessionSource.GRID_MINIMUM, binding_ceiling=BindingCeiling.NONE,
            min_hip_lift_m=0.04,
        )


def test_feasibility_and_binding_ceiling_must_agree():
    with pytest.raises(ValueError, match="no binding ceiling"):
        _swing(feasible=True, ceiling=BindingCeiling.REACH)
    with pytest.raises(ValueError, match="must say what stopped it"):
        _swing(feasible=False, ceiling=BindingCeiling.NONE)


def test_descent_hold_fraction_converts_to_metres():
    concession = _swing(
        direction="off", hip_lift_m=None, hold_fraction=0.5, height_m=0.16
    )
    assert concession.body_deviation_m == pytest.approx(0.08)


def test_a_greedy_ladder_result_is_not_comparable():
    """The 150 mm hole is a search artefact, so its number is not a minimum."""

    assert not _swing(source=ConcessionSource.GREEDY_LADDER).is_comparable
    assert _swing(source=ConcessionSource.GRID_MINIMUM).is_comparable


def test_the_greedy_hole_is_detected_by_its_signature():
    """A reach-shaped failure inside the fit stage is the 150 mm pathology."""

    from hybrid_note.scripts.experiments.day10_11_concession_2d import SwingFailure

    suspect = SwingConcession2D(
        feasible=False, direction="onto", obstacle_height_m=0.15,
        source=ConcessionSource.GREEDY_LADDER, binding_ceiling=BindingCeiling.FIT,
        min_hip_lift_m=0.04, min_liftoff_rise_m=0.02,
        failure=SwingFailure.IK_NOT_CONVERGED,
    )
    assert suspect.greedy_backtrack_suspected

    genuinely_stuck = SwingConcession2D(
        feasible=False, direction="onto", obstacle_height_m=0.20,
        source=ConcessionSource.GREEDY_LADDER, binding_ceiling=BindingCeiling.FIT,
        min_hip_lift_m=0.08, min_liftoff_rise_m=0.12,
        failure=SwingFailure.TERRAIN_COLLISION,
    )
    assert not genuinely_stuck.greedy_backtrack_suspected

    # A grid search cannot produce the signature, whatever it reports.
    from dataclasses import replace
    assert not replace(suspect, source=ConcessionSource.GRID_MINIMUM).greedy_backtrack_suspected


def test_unmeasured_stays_none_rather_than_zero():
    concession = _swing(clearance_m=None)
    assert concession.min_clearance_m is None
    assert concession.min_liftoff_rise_m is None
    assert concession.as_dict()["min_clearance_mm"] is None


@pytest.mark.parametrize("ceiling", [BindingCeiling.STANCE, BindingCeiling.REACH])
def test_branches_that_never_reached_the_repair_have_no_adjustments(ceiling):
    concession = _swing(feasible=False, ceiling=ceiling, hip_lift_m=0.12)
    assert adjustment_strings_from_concession(concession) == ()


def test_adjustment_strings_reproduce_the_showcase_wording():
    climb = SwingConcession2D(
        feasible=True, direction="onto", obstacle_height_m=0.16,
        source=ConcessionSource.GREEDY_LADDER, binding_ceiling=BindingCeiling.NONE,
        min_hip_lift_m=0.06, min_liftoff_rise_m=0.03, min_touchdown_drop_m=0.0,
        duration_scale=1.0,
    )
    assert adjustment_strings_from_concession(climb) == (
        "hip raised 60 mm", "lift-off raised 30 mm",
    )

    descent = SwingConcession2D(
        feasible=True, direction="off", obstacle_height_m=0.16,
        source=ConcessionSource.GREEDY_LADDER, binding_ceiling=BindingCeiling.NONE,
        min_hip_hold_fraction=0.5, min_liftoff_rise_m=0.0, min_touchdown_drop_m=0.0,
        duration_scale=1.5,
    )
    assert adjustment_strings_from_concession(descent, original_duration_s=0.6) == (
        "hip held above the landing pose by 80 mm",
        "swing lengthened 0.60 -> 0.90 s",
    )


# --------------------------------------------------------------------------
# Rolling side
# --------------------------------------------------------------------------


def test_rolling_asks_the_body_to_track_not_to_clear_a_bound():
    assert _roll_cell(60.0).requirement_kind is BodyRequirementKind.TRACK
    assert _swing().requirement_kind is BodyRequirementKind.LOWER_BOUND


def test_roll_cell_reports_travel_and_rise_separately():
    cell = RollCellConcession2D(
        feasible=True, obstacle_height_m=0.10, top_length_m=0.35,
        theta_climb_deg=60.0, approach_clearance_m=0.04,
        hip_z_min_m=0.20, hip_z_max_m=0.30, hip_z_start_m=0.25,
    )
    assert cell.hip_z_travel_m == pytest.approx(0.10)
    assert cell.hip_rise_above_start_m == pytest.approx(0.05)


def test_aggregation_takes_the_best_theta_and_records_the_window():
    cells = [
        _roll_cell(40.0, feasible=False),
        _roll_cell(50.0, hip_max=0.32),   # travel 0.08
        _roll_cell(60.0, hip_max=0.28),   # travel 0.04  <- best
        _roll_cell(70.0, hip_max=0.31),
    ]
    rolled = roll_concession_from_cells(cells)
    assert rolled.feasible
    assert rolled.best_theta_climb_deg == 60.0
    assert rolled.feasible_cell_count == 3
    assert rolled.feasible_theta_span_deg == pytest.approx(20.0)
    assert rolled.body_deviation_m == pytest.approx(0.04)


def test_a_comb_shaped_theta_window_is_not_reported_as_wide():
    """Day 6-7's own h = 0.12 row: feasible at 45 and 55 deg, infeasible at 50."""

    cells = [
        _roll_cell(40.0, feasible=False),
        _roll_cell(45.0, hip_max=0.28),
        _roll_cell(50.0, feasible=False),
        _roll_cell(55.0, hip_max=0.29),
        _roll_cell(60.0, feasible=False),
    ]
    rolled = roll_concession_from_cells(cells)

    # max - min still says 10 deg, which is exactly why it is not enough alone.
    assert rolled.feasible_theta_span_deg == pytest.approx(10.0)
    assert not rolled.theta_window_is_contiguous
    # The best theta stands alone, so nothing survives a theta error.
    assert rolled.best_theta_climb_deg == 45.0
    assert rolled.contiguous_span_around_best_deg == pytest.approx(0.0)


def test_a_genuine_window_reports_a_contiguous_span():
    cells = [
        _roll_cell(40.0, feasible=False),
        _roll_cell(45.0, hip_max=0.29),
        _roll_cell(50.0, hip_max=0.28),
        _roll_cell(55.0, hip_max=0.295),
        _roll_cell(60.0, feasible=False),
    ]
    rolled = roll_concession_from_cells(cells)
    assert rolled.theta_window_is_contiguous
    assert rolled.best_theta_climb_deg == 50.0
    assert rolled.contiguous_span_around_best_deg == pytest.approx(10.0)


def test_aggregation_refuses_to_mix_terrain_cells():
    mixed = [
        _roll_cell(60.0),
        RollCellConcession2D(
            feasible=True, obstacle_height_m=0.14, top_length_m=0.35,
            theta_climb_deg=60.0, approach_clearance_m=0.04,
        ),
    ]
    with pytest.raises(ValueError, match="aggregates over theta only"):
        roll_concession_from_cells(mixed)


def test_an_all_infeasible_column_is_infeasible_and_keeps_its_reasons():
    cells = [
        RollCellConcession2D(
            feasible=False, obstacle_height_m=0.16, top_length_m=0.35,
            theta_climb_deg=theta, approach_clearance_m=0.04,
            failure_stage="ROLL_DOWN",
        )
        for theta in (40.0, 60.0, 85.0)
    ]
    rolled = roll_concession_from_cells(cells)
    assert not rolled.feasible
    assert rolled.body_deviation_m is None
    assert rolled.failure_stages == ("ROLL_DOWN",)


# --------------------------------------------------------------------------
# Comparison
# --------------------------------------------------------------------------


def test_feasibility_outranks_a_smaller_demand():
    cheap_but_broken = _swing(feasible=False, ceiling=BindingCeiling.FIT, hip_lift_m=0.0)
    expensive_but_works = _swing(hip_lift_m=0.06)
    assert compare_concessions(expensive_but_works, cheap_but_broken) == -1


def test_smaller_body_demand_wins_then_wider_margin():
    assert compare_concessions(_swing(hip_lift_m=0.02), _swing(hip_lift_m=0.06)) == -1
    assert compare_concessions(
        _swing(hip_lift_m=0.04, clearance_m=0.005),
        _swing(hip_lift_m=0.04, clearance_m=0.001),
    ) == -1


def test_a_margin_floor_demotes_a_cell_that_only_just_passes():
    thin = _swing(hip_lift_m=0.0, clearance_m=0.0011)
    thick = _swing(hip_lift_m=0.06, clearance_m=0.004)
    assert compare_concessions(thin, thick) == -1
    assert compare_concessions(thin, thick, margin_floor_m=0.002) == 1


def test_ranking_a_bound_against_a_trajectory_is_refused_by_default():
    with pytest.raises(ValueError, match="Step 4 has to measure which is harder"):
        compare_concessions(_swing(), roll_concession_from_cells([_roll_cell(60.0)]))

    assert compare_concessions(
        _swing(), roll_concession_from_cells([_roll_cell(60.0)]),
        allow_cross_kind=True,
    ) in (-1, 0, 1)


# --------------------------------------------------------------------------
# Step 2: the grid search's contract with Step 1
# --------------------------------------------------------------------------


def test_a_grid_cell_is_comparable_and_never_carries_the_greedy_signature():
    """Step 2's whole point: escalate on the real verdict, not on a proxy.

    Kept small on purpose -- one cell, two-rung ladders, coarse sampling -- so
    it stays a contract test rather than a sweep.
    """

    from hybrid_note.scripts.experiments.day10_11_swing_sweep_2d import (
        SwingGridSettings2D,
        minimum_swing_onto_concession_2d,
    )

    settings = SwingGridSettings2D(
        hip_lift_ladder_m=(0.0, 0.04),
        liftoff_rise_ladder_m=(0.0,),
        sample_count=21,
        collision_arc_samples=41,
    )
    cell = minimum_swing_onto_concession_2d(0.06, 0.04, settings)

    assert cell.concession.source is ConcessionSource.GRID_MINIMUM
    assert cell.concession.is_comparable
    assert not cell.concession.greedy_backtrack_suspected
    assert cell.evaluations >= 1
    # Feasible or not, the cell has to say which ceiling bound it.
    assert cell.concession.feasible == (
        cell.concession.binding_ceiling is BindingCeiling.NONE
    )
