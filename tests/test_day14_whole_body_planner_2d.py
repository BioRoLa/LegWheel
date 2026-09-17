"""Day 14 Step 1: the gait-first planner reproduces the frozen flat gait.

The claim under test: placing the nominal cycle on the body's clock, in the
gait's liftoff order, gives **the same** flat-ground trajectory Day 12 froze --
same joint angles at the same instants, same swing windows, same support
margin -- while the four legs now agree about the body by construction.

Expensive (one nominal run plus the frozen planner), so both runs live in one
module fixture at the coarse sample count the Day 12 tests use.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_decision_map_2d import load_tables_2d
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import SegmentKind
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (
    LIFTOFF_SEQUENCES,
)
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (
    plan_terrain_2d,
)
import hybrid_note.scripts.experiments.day12_terrain_generalization_2d as module
from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import (
    plan_flat_gait_first_2d,
)

NOTES = __import__("pathlib").Path(module.__file__).resolve().parents[2] / "notes"
SAMPLES = 61


@pytest.fixture(scope="module")
def gait_first():
    return plan_flat_gait_first_2d(cycles=2, samples=SAMPLES)


@pytest.fixture(scope="module")
def frozen():
    tables = load_tables_2d(NOTES / "day10-11", NOTES / "day6-7")
    return plan_terrain_2d(None, tables, samples=SAMPLES)


def test_flat_ground_passes_every_check(gait_first):
    run, plan = gait_first
    assert run.planned
    assert run.report.is_valid, [c.value for c in run.report.failed_checks()]
    assert run.feasible


def test_the_legs_lift_off_in_the_gaits_order_and_one_at_a_time(gait_first):
    _, plan = gait_first
    order = [e.leg for e in plan.swings]
    wave = LIFTOFF_SEQUENCES["project_walk"]
    # Eight swings: the wave order twice, starting wherever the clock starts.
    start = wave.index(order[0])
    expected = [wave[(start + i) % 4] for i in range(len(order))]
    assert order == expected
    assert plan.max_airborne_count == 1
    for a, b in zip(plan.swings, plan.swings[1:]):
        assert not a.overlaps(b)
        assert b.start_s >= a.end_s - 1e-9


def test_the_swing_windows_are_the_gaits_own(gait_first):
    """0.36 s airborne every 0.6 s, without any duty-window rule in the loop."""

    _, plan = gait_first
    for event in plan.swings:
        assert event.duration_s == pytest.approx(0.36, abs=1e-9)
        assert event.minimum_duration_s < event.duration_s
    starts = sorted(e.start_s for e in plan.swings)
    assert np.allclose(np.diff(starts), 0.6, atol=1e-9)
    assert plan.slowdowns == ()


def test_the_four_legs_agree_about_the_body_by_construction(gait_first):
    run, _ = gait_first
    assert run.body.world_x_spread_m is not None
    assert run.body.world_x_spread_m < 1e-9


def test_the_frozen_flat_trajectory_is_reproduced(gait_first, frozen):
    """Same instants, same joint angles, same body height, same margin.

    ``body_x`` differs by one constant: the frozen run integrates stance
    advances from zero, this one reads the world position off the clock.
    Everything the exporter writes is identical.
    """

    run, _ = gait_first
    ours, theirs = run.trajectory.samples, frozen.trajectory.samples
    assert len(ours) == len(theirs)
    offset = ours[0].body_position_world_m[0] - theirs[0].body_position_world_m[0]
    for a, b in zip(ours, theirs):
        assert a.time_s == pytest.approx(b.time_s, abs=1e-9)
        assert a.body_position_world_m[2] == pytest.approx(
            b.body_position_world_m[2], abs=1e-9)
        assert a.body_position_world_m[0] - b.body_position_world_m[0] == pytest.approx(
            offset, abs=1e-9)
        assert a.swing_leg == b.swing_leg
        for leg in LEG_ORDER:
            assert a.legs[leg].theta_rad == pytest.approx(b.legs[leg].theta_rad, abs=1e-9)
            assert a.legs[leg].beta_rad == pytest.approx(b.legs[leg].beta_rad, abs=1e-9)
            assert a.legs[leg].mode == b.legs[leg].mode
    assert run.stability.minimum_margin_m == pytest.approx(
        frozen.stability.minimum_margin_m, abs=1e-9)
    assert run.stability.minimum_margin_m == pytest.approx(0.0048394, abs=1e-6)


def test_the_plan_is_the_nominal_cycle_and_nothing_else(gait_first):
    run, _ = gait_first
    kinds = {k for k in run._segment_kinds()}
    assert kinds == {SegmentKind.FOOT_RIM_ROLL, SegmentKind.RECOVERY_SWING}
    assert run.nominal_recovery_swings == 8
    assert run.terrain_transition_swings == 0
