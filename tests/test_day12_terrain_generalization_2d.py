"""Day 12 Step 10: one planner, several terrains.

Plan §17.  The claim is that terrain is a *parameter*, so most of these tests
are about the planner's **shape** -- one entry point, no size branches -- and
are cheap.  The two obstacle runs that compose a crossing are expensive, so
they live in one module fixture.
"""

import inspect
import re

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    StrategyId,
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import SegmentKind
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import decide_2d
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (
    HYBRID_BODY_TOLERANCE_M,
    HYBRID_DECISION_ORDER,
    PLANNER_MODULES,
    Stage,
    comparison_rows,
    nominal_body_height_m,
    plan_terrain_2d,
    planner_size_literals,
)
import hybrid_note.scripts.experiments.day12_terrain_generalization_2d as module

NOTES = __import__("pathlib").Path(module.__file__).resolve().parents[2] / "notes"

#: The one match the gate reports that is **not** a terrain size, with why.
#: An allowlist with a written justification, not a silenced check: a new match
#: has to be argued for here before a test will accept it.
ALLOWED_SIZE_LITERALS = {
    "day12_timing_skeleton_2d.py": (
        "0.10 * schedule.timing.cycle_period_s -- a fraction of one gait "
        "cycle, not a length in metres"
    ),
}


@pytest.fixture(scope="module")
def tables():
    return load_tables_2d(NOTES / "day10-11", NOTES / "day6-7")


# --------------------------------------------------------------------------
# Requirement 8: no terrain-size logic in the planner
# --------------------------------------------------------------------------


def test_the_planner_modules_contain_no_evaluation_size(tables):
    """Plan §17 requirement 8, checked mechanically rather than by reading."""

    found = planner_size_literals()
    unexplained = {name: lines for name, lines in found.items()
                   if name not in ALLOWED_SIZE_LITERALS}
    assert unexplained == {}, (
        "an evaluation size appeared in a planner module; if it is not a "
        f"terrain size, justify it in ALLOWED_SIZE_LITERALS: {unexplained}")


def test_every_allowed_literal_still_exists_and_is_still_that_line():
    """An allowlist that outlives its reason is a silenced check."""

    found = planner_size_literals()
    for name in ALLOWED_SIZE_LITERALS:
        assert name in found, (
            f"{name} no longer matches; remove it from ALLOWED_SIZE_LITERALS "
            "rather than leaving a permanent exemption")
        assert any("cycle_period_s" in line for line in found[name])


def test_no_planner_module_is_missing_from_the_gate():
    for name in PLANNER_MODULES:
        assert (module.Path(module.__file__).resolve().parent / name).exists()


def test_the_entry_point_takes_the_terrain_as_a_parameter():
    """Not a global, not an experiment id: an argument."""

    signature = inspect.signature(plan_terrain_2d)
    assert list(signature.parameters)[0] == "terrain"
    annotation = signature.parameters["terrain"].annotation
    assert "SharedTerrainSpec2D" in str(annotation)
    assert "None" in str(annotation), "flat ground is the absence of one"


def test_the_source_has_no_branch_on_a_height_value():
    """A *comparison* against a height is the branch requirement 1 bans.

    Assignment is not: ``height_m = 0.0 if terrain is None else ...`` branches
    on whether an obstacle exists, not on how big one is.  The first version of
    this pattern included a bare ``=`` and flagged exactly that line -- the
    test was wrong, not the code.
    """

    source = inspect.getsource(module)
    comparison = re.compile(r"height_m\s*(?:[<>]=?|==|!=)\s*[0-9]")
    assert not comparison.search(source)
    assert not re.search(r"top_length_m\s*(?:[<>]=?|==|!=)\s*[0-9]", source)
    assert not re.search(r"if\s+height", source)


def test_the_nominal_body_height_comes_from_the_leg_not_a_platform():
    """It was obtained from a 4 cm platform once; the gate caught that."""

    assert nominal_body_height_m() == pytest.approx(0.1622826, abs=1e-6)
    source = inspect.getsource(module.nominal_body_height_m)
    assert "SharedTerrainSpec2D" not in source
    assert "hip_z_for_flat_stance" in source


# --------------------------------------------------------------------------
# Test A: flat ground (requirement 4)
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def flat(tables):
    return plan_terrain_2d(None, tables, samples=61)


def test_flat_ground_is_the_absence_of_an_obstacle_not_a_size(flat):
    assert flat.is_flat
    assert flat.terrain is None
    assert flat.height_m == 0.0


def test_flat_ground_runs_the_nominal_cycle(flat):
    """``FOOT_RIM_ROLL + RECOVERY_SWING``, and nothing terrain-specific."""

    kinds = {k for k in flat._segment_kinds()}
    assert kinds == {SegmentKind.FOOT_RIM_ROLL, SegmentKind.RECOVERY_SWING}


def test_flat_ground_has_recovery_swings_and_no_transition_swings(flat):
    """Plan §0.2 retracted "flat ground needs no swing"; §17 keeps the split."""

    assert flat.nominal_recovery_swings > 0
    assert flat.terrain_transition_swings == 0


def test_flat_ground_has_no_ascent_or_descent_primitive(flat):
    assert flat.ascent_primitive is None
    assert flat.descent_primitive is None


def test_flat_ground_still_gets_a_full_trajectory_and_verdict(flat):
    assert flat.planned
    assert flat.report is not None
    assert flat.trajectory is not None


# --------------------------------------------------------------------------
# Tests B and C: the same pipeline, only the parameters change
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def obstacle_runs(tables):
    out = {}
    for height_m in (0.04, 0.10):
        terrain = SharedTerrainSpec2D(
            height_m=height_m, top_length_m=0.40, x_start_m=1.00,
            obstacle_id="day12_platform")
        out[height_m] = plan_terrain_2d(terrain, tables, samples=61)
    return out


def test_both_obstacles_compose_and_get_a_trajectory(obstacle_runs):
    for run in obstacle_runs.values():
        assert run.planned
        assert run.composed is not None and run.composed.composed


def test_both_obstacles_use_the_same_primitives_chosen_by_day_10_11(
        obstacle_runs, tables):
    """Step 10 does not choose; ``decide_2d`` does, from frozen tables.

    Asserted against ``decide_2d`` itself rather than against a named
    strategy: this test exists to hold that Step 10 defers, and hard-coding
    the answer made it fail for the right reason when the rule changed (log
    1.7: the project moved to a rolling preference) instead of catching a
    Step 10 that had started choosing for itself.
    """

    for run in obstacle_runs.values():
        expected = decide_2d(run.terrain.height_m, run.terrain.top_length_m,
                             tables, order=HYBRID_DECISION_ORDER,
                             body_tolerance_m=HYBRID_BODY_TOLERANCE_M)
        assert run.composed.strategy is expected.winner
    # And they agree with each other, which is what "the same primitives"
    # meant before it was written as a literal.
    strategies = {run.composed.strategy for run in obstacle_runs.values()}
    assert len(strategies) == 1


def test_ascent_and_descent_stay_separately_reported(obstacle_runs):
    """Requirement 5: independently selected, so independently reported."""

    for run in obstacle_runs.values():
        assert run.ascent_primitive != run.descent_primitive


def test_a_crossing_run_rolls_further_than_a_flat_one(flat, obstacle_runs):
    """World registration means a leg rolls **to** the obstacle first (log 1.11).

    This has now been wrong twice for the same reason -- it asserted a number
    that a design change was supposed to move.  First it asserted the crossing
    added terrain-transition *swings* (the rolling preference made that zero,
    log 1.7); then it asserted the nominal recovery count was *unchanged*, and
    world registration made that false too: each leg now rolls from where it
    actually starts to where the obstacle actually is, so the obstacle run
    contains more nominal cycles than the flat one (18 against 8), and it
    should.

    So this asserts the direction and the reason, not the count.
    """

    for run in obstacle_runs.values():
        assert run.nominal_recovery_swings > flat.nominal_recovery_swings, (
            "a leg that has to reach the obstacle rolls more cycles to get "
            "there than one walking on the flat")
        assert run.composed is not None and run.composed.composed, (
            "there is still a crossing; it is simply not made of swings")


def test_changing_only_the_height_changes_only_the_terrain(obstacle_runs):
    """The two runs differ in their terrain and in nothing structural."""

    four_cm, ten_cm = obstacle_runs[0.04], obstacle_runs[0.10]
    assert four_cm.terrain.height_m != ten_cm.terrain.height_m
    assert four_cm.ascent_primitive == ten_cm.ascent_primitive
    assert four_cm.descent_primitive == ten_cm.descent_primitive
    assert (four_cm.terrain_transition_swings
            == ten_cm.terrain_transition_swings)


# --------------------------------------------------------------------------
# Test D: the challenge terrain refuses, with a reason (requirement 6)
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def challenge(tables):
    terrain = SharedTerrainSpec2D(
        height_m=0.19, top_length_m=0.40, x_start_m=1.00,
        obstacle_id="day12_platform")
    return plan_terrain_2d(terrain, tables, samples=61)


def test_the_challenge_terrain_is_refused_not_forced(challenge):
    assert not challenge.planned
    assert not challenge.feasible
    assert challenge.failures


def test_the_refusal_names_the_first_limiting_constraint(challenge):
    """Plan §17: record the first limiting constraint, not just "no"."""

    first = challenge.first_limiting_constraint
    assert first is not None
    assert first.stage is Stage.DECISION
    assert first.detail.strip()


def test_every_strategy_gets_its_own_reason(challenge):
    """Four refusals, one per strategy that was not available."""

    strategies = {f.strategy for f in challenge.failures if f.strategy}
    assert len(strategies) >= 3
    for failure in challenge.failures:
        assert failure.detail.strip()


def test_nothing_was_relaxed_to_make_it_pass(challenge):
    assert challenge.trajectory is None
    assert challenge.report is None
    assert challenge.max_body_lift_m is None


# --------------------------------------------------------------------------
# Requirement 7: the comparison table
# --------------------------------------------------------------------------


def test_the_comparison_table_has_every_column_the_plan_asks_for(
        flat, obstacle_runs, challenge):
    rows = comparison_rows([flat, *obstacle_runs.values(), challenge])
    terrain_rows = [r for r in rows if r["row_kind"] == "terrain"]
    assert len(terrain_rows) == 4
    for column in ("terrain", "feasible", "ascent_primitive",
                   "descent_primitive", "nominal_recovery_swings",
                   "terrain_transition_swings", "max_body_lift_mm",
                   "min_stability_margin_mm", "failure_reason"):
        assert column in terrain_rows[0]


def test_the_table_is_csv_writable(flat, obstacle_runs, challenge):
    rows = comparison_rows([flat, *obstacle_runs.values(), challenge])
    assert all(len(r) == len(rows[0]) for r in rows)


def test_the_swing_counts_are_reported_separately(flat, obstacle_runs):
    """Plan §18 needs nominal recovery apart from terrain-transition."""

    rows = comparison_rows([flat, *obstacle_runs.values()])
    terrain_rows = [r for r in rows if r["row_kind"] == "terrain"]
    assert terrain_rows[0]["terrain_transition_swings"] == 0
    assert terrain_rows[0]["nominal_recovery_swings"] > 0
    # Both counts are present on every row and are different fields -- which
    # is what "reported separately" asks for.  Their *values* depend on the
    # decision rule and are not this test's business.
    for row in terrain_rows:
        assert "terrain_transition_swings" in row
        assert "nominal_recovery_swings" in row
