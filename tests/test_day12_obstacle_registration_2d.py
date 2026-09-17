"""Day 12 appendix B: registering the planned crossing against the platform.

The measurement is a subtraction, so the tests are mostly about it being the
*right* subtraction -- and about the two facts it exists to pin down: the
crossing does not register, and nothing in the plan ever stands on the
obstacle.  Both are properties of the frozen data; if either one ever changes,
these tests are how the change gets noticed rather than assumed.
"""

from pathlib import Path

import matplotlib
import pytest

matplotlib.use("Agg")

from hybrid_note.scripts.experiments.day10_11_composer_2d import (
    COMPOSER_FRAME_X_START_M,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    DEFAULT_ORDER,
    StrategyId,
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER
from hybrid_note.scripts.experiments.day12_obstacle_registration_2d import (
    plot_registration_2d,
    registration_report_2d,
    registration_rows,
    stance_on_top_seconds_2d,
)
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (
    plan_terrain_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import LegMode
from hybrid_note.scripts.experiments.day12_whole_body_animation_2d import (
    planned_surfaces_2d,
    viewing_heights_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    assemble_whole_body_2d,
)

NOTES = Path(__file__).resolve().parents[1] / "hybrid_note" / "notes"

#: The evaluation terrain is a *query*, not a rule (plan §0.1): it is written
#: here, in the test, and never inside the planner or the viewer.
TERRAIN = SharedTerrainSpec2D(height_m=0.04, top_length_m=0.40, x_start_m=1.00,
                              obstacle_id="day12_platform")


@pytest.fixture(scope="module")
def tables():
    return load_tables_2d(NOTES / "day10-11", NOTES / "day6-7")


@pytest.fixture(scope="module")
def crossing(tables):
    run = plan_terrain_2d(TERRAIN, tables, samples=61, reposition_unresolved=2)
    assert run.planned, "the 4 cm terrain assembles a trajectory"
    report = registration_report_2d(run.plan, run.body, TERRAIN)
    return run, report


# --------------------------------------------------------------------------
# The subtraction
# --------------------------------------------------------------------------


def test_the_implied_obstacle_is_the_composition_frame_carried_over(crossing):
    _, report = crossing
    for registration in report.registrations:
        assert registration.implied_x_start_entry_m == pytest.approx(
            registration.world_hip_start_m - registration.local_hip_start_m
            + COMPOSER_FRAME_X_START_M)
        assert registration.frame_x_start_m == COMPOSER_FRAME_X_START_M


def test_drift_is_exactly_what_the_schedule_failed_to_deliver(crossing):
    """The obstacle appears to move during a segment by precisely the body
    advance the segment asked for and did not get."""

    _, report = crossing
    for registration in report.registrations:
        assert registration.drift_m == pytest.approx(
            registration.delivered_advance_m - registration.demanded_advance_m)


def test_a_flat_run_has_nothing_to_register(tables):
    run = plan_terrain_2d(None, tables, samples=61)
    report = registration_report_2d(run.plan, run.body, None)
    assert report.registrations == ()
    assert report.spread_m == 0.0
    # No crossing is not the same as a registered one.
    assert not report.is_registrable
    assert report.distance_to_planned_terrain_m is None


# --------------------------------------------------------------------------
# The two facts
# --------------------------------------------------------------------------


def test_the_crossing_does_not_register(crossing):
    _, report = crossing
    assert report.registrations
    assert not report.is_registrable
    assert report.spread_m > report.tolerance_m
    # Not a rounding-scale disagreement: each leg contradicts *itself*.
    assert report.worst_within_leg_gap_m > 0.1
    assert report.worst_drift_m > 0.1


def _raised_stance_segments(run) -> int:
    raised = 0
    for leg in LEG_ORDER:
        for scheduled in run.plan.schedule.segments_of(leg):
            if scheduled.mode is not LegMode.STANCE:
                continue
            segment = run.plan.plans[leg].phased[scheduled.segment_index].segment
            if max(float(segment.start_contact.point_world_xz_m[1]),
                   float(segment.end_contact.point_world_xz_m[1])) > 1e-3:
                raised += 1
    return raised


def test_the_rolling_crossing_does_stand_on_the_obstacle(crossing):
    """Problem A6, dissolved by the rolling preference (log 1.7 / 1.8).

    A swing crossing touched down and lifted off at the same instant, so the
    robot never actually stood on the platform -- 0.000 s, and the missing
    160 mm was Step 7's unresolved ``TOP_REPOSITION``.  A rolling crossing
    carries the contact across the top instead, so the stance is there without
    a reposition segment: 6.4168 leg-seconds at this cell.
    """

    run, report = crossing
    assert report.stance_on_top_s > 0.0
    assert stance_on_top_seconds_2d(run.plan) == pytest.approx(
        report.stance_on_top_s)
    assert _raised_stance_segments(run) > 0


def test_a_swing_crossing_still_never_stands_on_the_obstacle(tables):
    """The other half, kept: A6 was real, and it is a property of the swing
    crossing rather than something that was fixed underneath it."""

    run = plan_terrain_2d(TERRAIN, tables, samples=61, reposition_unresolved=2,
                          order=DEFAULT_ORDER, body_tolerance_m=0.0)
    assert run.composed.strategy is StrategyId.SWING_SWING
    assert stance_on_top_seconds_2d(run.plan) == 0.0
    assert _raised_stance_segments(run) == 0


def test_the_raised_surface_reaches_the_drawing(crossing):
    """What the viewer draws comes from the segments, not from the terrain
    parameter: the obstacle's height shows up because a segment ends on it."""

    run, _ = crossing
    whole = assemble_whole_body_2d(run.plan, run.body, run.stability,
                                   samples=61, reposition_unresolved=2,
                                   use_generator_frames=True)
    surfaces = planned_surfaces_2d(run.plan, whole)
    assert len(surfaces) == len(whole.samples)

    raised = [surface for row in surfaces for surface in row.values()
              if surface.raised_z_m is not None]
    assert raised, "the crossing segments carry the obstacle's top"
    assert all(surface.raised_z_m == pytest.approx(TERRAIN.height_m)
               for surface in raised)
    ascending = [s for s in raised if s.raised_is_the_target]
    assert ascending and len(ascending) < len(raised)


def test_the_drawn_height_leaves_no_foot_under_its_own_surface(crossing):
    run, _ = crossing
    whole = assemble_whole_body_2d(run.plan, run.body, run.stability,
                                   samples=61, reposition_unresolved=2,
                                   use_generator_frames=True)
    heights = viewing_heights_2d(run.plan, whole,
                                 nominal_body_z_m=run.body.nominal_body_z_m)
    for height in heights:
        assert min(height.float_gap_m.values(), default=0.0) >= 0.0


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------


def test_the_rows_share_one_header(crossing):
    _, report = crossing
    rows = registration_rows(report)
    assert rows and rows[0]["row_kind"] == "summary"
    keys = list(rows[0])
    assert all(list(row) == keys for row in rows)
    assert {row["row_kind"] for row in rows} == {"summary", "leg", "segment"}


def test_the_chart_draws(crossing, tmp_path):
    _, report = crossing
    path = tmp_path / "registration.png"
    plot_registration_2d(report, path)
    assert path.exists() and path.stat().st_size > 0


def test_the_chart_refuses_a_run_with_no_crossing(tables, tmp_path):
    run = plan_terrain_2d(None, tables, samples=61)
    report = registration_report_2d(run.plan, run.body, None)
    with pytest.raises(ValueError):
        plot_registration_2d(report, tmp_path / "nothing.png")
