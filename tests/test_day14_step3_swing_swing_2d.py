"""Day 14 Step 3: four legs over a 40 x 400 mm platform, swing up / swing down.

Expensive (two to three planning passes, ~20 min), so one module fixture at a
coarse sample count, and the assertions are about the plan's *shape*:

* the four legs go over one at a time, in body travel, by construction;
* every landing is on the surface its transition names, inside its edges;
* the four hips stay on one plane (the only body constraint the owner keeps);
* every HARD check passes; advisory ones are reported.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import SegmentKind
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER, LegId
from hybrid_note.scripts.experiments.day14_leg_terrain_rule_2d import (
    plan_swing_swing_crossing_2d,
)
from hybrid_note.scripts.experiments.day14_whole_body_planner_2d import (
    refit_body_plane_2d,
)

SPEC = SharedTerrainSpec2D(height_m=0.04, top_length_m=0.40, x_start_m=1.0,
                           arc_samples=121)


@pytest.fixture(scope="module")
def crossing():
    run, plan, rule, passes = plan_swing_swing_crossing_2d(SPEC, samples=61, log=lambda m: None)
    run, plane = refit_body_plane_2d(run)
    return run, plan, rule, plane, passes


def test_the_four_legs_cross_one_at_a_time(crossing):
    run, plan, _, _, _ = crossing
    assert plan.max_airborne_count == 1
    for a, b in zip(plan.swings, plan.swings[1:]):
        assert not a.overlaps(b)
        assert b.start_s >= a.end_s - 1e-9


def test_every_leg_climbs_and_descends_exactly_once(crossing):
    _, plan, _, _, _ = crossing
    for leg in LEG_ORDER:
        kinds = [e.kind for e in plan.swings if e.leg is leg]
        assert kinds.count("SWING_UP") == 1, (leg, kinds)
        assert kinds.count("SWING_DOWN") == 1, (leg, kinds)
        assert kinds.index("SWING_UP") < kinds.index("SWING_DOWN")


def test_the_pair_takes_off_at_different_body_positions(crossing):
    """Same mount x, different contacts: the stagger that makes it possible."""

    _, plan, _, _, _ = crossing
    for pair in ((LegId.LF, LegId.RF), (LegId.LH, LegId.RH)):
        ups = {e.leg: e for e in plan.swings if e.kind == "SWING_UP" and e.leg in pair}
        assert set(ups) == set(pair)
        first, second = sorted(ups.values(), key=lambda e: e.body_x_start_m)
        assert second.body_x_start_m >= first.body_x_end_m - 1e-9


def test_every_landing_is_on_its_surface_inside_the_edges(crossing):
    _, _, rule, _, _ = crossing
    accepted = [t for t in rule.transitions if t.success]
    assert accepted
    for t in accepted:
        x, z = t.landing_contact_xz_m
        assert z == pytest.approx(t.landing_surface_z_m, abs=1e-4)
        if t.landing_surface_z_m > 0.0:
            assert SPEC.x_start_m + 0.01 <= x <= SPEC.x_max_m - 0.01
        else:
            assert not (SPEC.x_start_m - 0.01 < x < SPEC.x_max_m + 0.01)


def test_the_four_hips_stay_on_one_plane(crossing):
    _, _, _, plane, _ = crossing
    assert plane["coplanarity_residual_max_mm"] < 2.0
    assert plane["pitch_max_deg"] < 10.0
    # On a 400 mm top the front pair is down again before the rear pair
    # climbs, so only one axle is ever raised: the body centre rises by half
    # the block (measured 20.00 mm), not the whole of it.
    assert plane["body_z_max_mm"] - plane["body_z_min_mm"] == pytest.approx(20.0, abs=3.0)


def test_hard_checks_pass_and_advisory_ones_are_reported(crossing):
    run, _, _, _, _ = crossing
    assert run.planned
    assert run.report.is_executable, [c.value for c in run.report.hard_failed_checks()]
    # Advisory checks are reported, not enforced.
    assert set(c.value for c in run.report.advisory_failed_checks()) <= {
        "support_margin", "body_requirement_satisfied"}


def test_the_swings_are_counted_as_the_terrain_forced_them(crossing):
    run, _, _, _, _ = crossing
    assert run.terrain_transition_swings >= 8
    assert run.nominal_recovery_swings >= 4
    kinds = set(run._segment_kinds())
    assert {SegmentKind.SWING_UP, SegmentKind.SWING_DOWN, SegmentKind.FOOT_RIM_ROLL,
            SegmentKind.RECOVERY_SWING} <= kinds
