"""Day 12 Step 6: the three-leg support triangle and its stability margin.

Plan §13.  Requirement 10 asks for clearly inside / boundary / outside cases,
and those are tested directly on :class:`SupportTriangle2D` -- the geometry
lives there, so it can be checked against triangles chosen by hand rather than
against whatever the planner happens to produce.  The pipeline path is then
exercised once end to end on a real four-leg plan.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_composer_2d import ComposedSequence2D
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import StrategyId
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirement2D,
    BodyRequirementKind,
    FrameRef2D,
    MotionSegment2D,
    PointContact2D,
    RollSampling2D,
    RollingContact2D,
    SegmentKind,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    LegId,
    initialize_four_leg_state_2d,
    leg_mounts_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    BOUNDARY_TOLERANCE_S,
    COM_BASIS,
    DEGENERATE_AREA_M2,
    GAMMA_RAD,
    StabilitySample2D,
    SupportTriangle2D,
    SwingStability2D,
    contact_offset_from_hip_m,
    stability_rows,
    support_triangle_at,
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    LegMode,
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    build_leg_plan_2d,
    plan_four_legs_2d,
)

#: A right triangle with legs of 1 m: easy distances to check by hand.
UNIT = np.array([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)], dtype=float)


def _triangle(points=UNIT, swing=LegId.LF) -> SupportTriangle2D:
    return SupportTriangle2D(
        time_s=0.0, swing_leg=swing,
        support_legs=(LegId.RF, LegId.LH, LegId.RH), points_xy_m=points,
    )


# --------------------------------------------------------------------------
# Requirement 10: inside, boundary, outside
# --------------------------------------------------------------------------


def test_a_point_clearly_inside_has_a_positive_margin():
    margin = _triangle().signed_margin_m((0.2, 0.2))
    assert margin > 0.0
    # Nearest edge is a leg, at 0.2 -- not the hypotenuse, which is 0.6/sqrt(2).
    assert margin == pytest.approx(0.2)
    assert margin < 0.6 / np.sqrt(2)


def test_a_point_on_an_edge_has_zero_margin():
    assert _triangle().signed_margin_m((0.5, 0.0)) == pytest.approx(0.0)


def test_a_point_on_a_vertex_has_zero_margin():
    assert _triangle().signed_margin_m((0.0, 0.0)) == pytest.approx(0.0)


def test_a_point_clearly_outside_has_a_negative_margin():
    margin = _triangle().signed_margin_m((-0.3, 0.5))
    assert margin < 0.0
    assert margin == pytest.approx(-0.3)


def test_the_margin_does_not_depend_on_the_winding():
    """Listing the same three contacts in the other order is the same triangle."""

    forward = _triangle(UNIT).signed_margin_m((0.2, 0.2))
    reversed_ = _triangle(UNIT[::-1]).signed_margin_m((0.2, 0.2))
    assert forward == pytest.approx(reversed_)


def test_a_collinear_support_reports_no_margin_rather_than_a_distance():
    """A line has no interior; a distance to it would read as a real margin."""

    line = np.array([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], dtype=float)
    triangle = _triangle(line)
    assert triangle.is_degenerate
    assert triangle.signed_margin_m((0.5, 0.0)) is None


def test_a_fourth_contact_is_refused_rather_than_sliced_away():
    """Four points is a scheduling result to report, not a triangle to build."""

    square = np.array([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
    with pytest.raises(ValueError, match="three contacts"):
        _triangle(square)


def test_the_degenerate_threshold_is_far_below_a_real_triangle():
    real = _triangle(np.array([(0.0, -0.2), (0.5, 0.2), (-0.5, 0.2)]))
    assert real.area_m2 > 1000 * DEGENERATE_AREA_M2
    assert not real.is_degenerate


def test_an_unknown_margin_is_never_reported_as_stable():
    """Unknown is not the same as fine."""

    sample = StabilitySample2D(
        time_s=0.0, swing_leg=LegId.LF, support_legs=(), com_xy_m=(0.0, 0.0),
        margin_m=None, support_area_m2=0.0, is_degenerate=True,
    )
    assert sample.is_stable is False


# --------------------------------------------------------------------------
# Contacts, not hips (plan §13's "重要實作細節")
# --------------------------------------------------------------------------


def _point(x: float, hip_x: float) -> PointContact2D:
    return PointContact2D(
        rim="foot_rim", alpha_rad=0.0, point_world_xz_m=(x, 0.0),
        surface_id="ground", theta_rad=np.deg2rad(60.0), beta_rad=0.0,
        hip_xz_m=(hip_x, 0.2194),
    )


def _roll_segment(contact_x, hip_x) -> MotionSegment2D:
    return MotionSegment2D(
        kind=SegmentKind.FOOT_RIM_ROLL, phase_label="FOOT_RIM_ROLL",
        start_contact=_point(contact_x[0], hip_x[0]),
        end_contact=_point(contact_x[1], hip_x[1]),
        sampling=RollSampling2D(arc_samples=241, beta_step_rad=np.deg2rad(-1.0),
                                theta_step_rad=None),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.TRACK, x_range_m=(hip_x[0], hip_x[1]),
            hip_z_profile_m=np.array([0.2194, 0.2194])),
        frames=FrameRef2D(source_id="synthetic", indices=(0, 1)),
        rolling=RollingContact2D(
            rim="foot_rim", surface_ids=("ground",),
            alpha_range_rad=(np.deg2rad(-40.0), np.deg2rad(40.0)),
            beta_range_rad=(np.deg2rad(-40.0), np.deg2rad(40.0)),
            theta_range_rad=(np.deg2rad(60.0), np.deg2rad(60.0)),
            contact_start_xz_m=(contact_x[0], 0.0),
            contact_end_xz_m=(contact_x[1], 0.0)),
    )


def test_the_contact_offset_is_measured_from_the_hip_not_from_the_origin():
    """Plan §13: use contact points, not hip positions."""

    segment = _roll_segment(contact_x=(0.05, 0.35), hip_x=(0.0, 0.4))
    assert contact_offset_from_hip_m(segment, 0.0) == pytest.approx(0.05)
    assert contact_offset_from_hip_m(segment, 1.0) == pytest.approx(-0.05)


def test_shifting_the_whole_chain_does_not_move_the_offset():
    """The chain's x origin is arbitrary; the foot-under-hip offset is not."""

    here = _roll_segment(contact_x=(0.05, 0.35), hip_x=(0.0, 0.4))
    there = _roll_segment(contact_x=(9.05, 9.35), hip_x=(9.0, 9.4))
    for fraction in (0.0, 0.5, 1.0):
        assert contact_offset_from_hip_m(here, fraction) == pytest.approx(
            contact_offset_from_hip_m(there, fraction))


def test_a_rolling_contact_moves_while_the_swing_is_airborne():
    """Which is why the margin is sampled across the swing, not at liftoff."""

    segment = _roll_segment(contact_x=(0.05, 0.35), hip_x=(0.0, 0.4))
    offsets = [contact_offset_from_hip_m(segment, f)
               for f in np.linspace(0.0, 1.0, 5)]
    assert len(set(np.round(offsets, 9))) > 1


# --------------------------------------------------------------------------
# gamma stays 0 (requirement 8), and the lateral geometry comes from Step 2
# --------------------------------------------------------------------------


def test_gamma_is_zero_in_day_12():
    assert GAMMA_RAD == 0.0


def test_the_lateral_coordinates_are_step_2_s_mounting_offsets():
    mounts = {m.leg: m.offset_body_xyz_m for m in leg_mounts_2d(GAMMA_RAD)}
    lefts = [mounts[leg][1] for leg in (LegId.LF, LegId.LH)]
    rights = [mounts[leg][1] for leg in (LegId.RF, LegId.RH)]
    assert all(v > 0 for v in lefts), "+y is left"
    assert all(v < 0 for v in rights)
    for left, right in zip(lefts, rights):
        assert left + right == pytest.approx(0.0)


# --------------------------------------------------------------------------
# End to end on a real four-leg plan
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def stability():
    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="flat run, no crossing")
    plans = {leg: build_leg_plan_2d(leg, composed) for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, walk_timing_2d())
    state = initialize_four_leg_state_2d(SharedTerrainSpec2D(
        height_m=0.04, top_length_m=0.40, x_start_m=1.00,
        obstacle_id="day12_platform"))
    trajectory = body_trajectory_2d(
        four, nominal_body_z_m=float(state.body_position_world_m[2]),
        samples=121)
    return four, trajectory, swing_stability_2d(four, trajectory)


def test_every_swing_inside_the_covered_interval_is_evaluated(stability):
    four, _, result = stability
    airborne = [s for leg in LEG_ORDER
                for s in four.schedule.segments_of(leg)
                if s.mode is LegMode.AIRBORNE]
    lo, hi = four.schedule.covered_interval_s
    inside = [s for s in airborne
              if min(s.end_s, hi) - max(s.start_s, lo) > BOUNDARY_TOLERANCE_S]
    assert len(result.swings) == len(inside)


def test_a_swing_clipped_to_nothing_is_not_reported_as_a_failed_one(stability):
    """A zero-length clip is outside the run, not an unstable interval."""

    _, _, result = stability
    for swing in result.swings:
        assert swing.end_s - swing.start_s > BOUNDARY_TOLERANCE_S
        assert swing.minimum_margin_m is not None
        assert swing.unknown_samples == 0, "every sample is a real three-leg support"


def test_a_swing_is_sampled_across_its_whole_span_not_just_liftoff(stability):
    """Plan §13 requirement 5.

    Half-open: at ``end_s`` the next swing has lifted off, so this swing's
    three-leg support does not exist there.  The samples cover the span the
    support actually holds for.
    """

    _, _, result = stability
    for swing in result.swings:
        assert len(swing.samples) > 1
        assert swing.samples[0].time_s == pytest.approx(swing.start_s)
        assert swing.start_s <= swing.samples[-1].time_s < swing.end_s
        span = swing.end_s - swing.start_s
        assert swing.samples[-1].time_s > swing.end_s - 0.1 * span


def test_exactly_three_legs_support_each_sampled_instant(stability):
    _, _, result = stability
    for swing in result.swings:
        for sample in swing.samples:
            assert len(sample.support_legs) == 3
            assert swing.swing_leg not in sample.support_legs


def test_each_swing_stores_its_minimum_margin_and_when(stability):
    """Plan §13 requirement 6."""

    _, _, result = stability
    for swing in result.swings:
        row = swing.as_dict()
        assert row["minimum_stability_margin_mm"] is not None
        assert swing.start_s <= swing.worst_time_s <= swing.end_s


def test_some_swings_start_exactly_on_the_support_boundary(stability):
    """The headline Step 6 result, and it is symmetry, not rounding.

    At the start of the LH and RH swings the two diagonal support legs sit
    symmetrically about the body centre -- measured at (+239.2, -211.7) and
    (-239.2, +211.7) mm -- so the edge joining them passes exactly through it
    and the margin is exactly zero.  The other three swings never do better
    than 0.995 mm, so the walk is at best marginally stable throughout.
    """

    _, _, result = stability
    assert result.minimum_margin_m == pytest.approx(0.0, abs=1e-9)
    on_the_edge = [s for s in result.swings
                   if s.minimum_margin_m is not None
                   and abs(s.minimum_margin_m) < 1e-9]
    assert len(on_the_edge) == 2
    assert {s.swing_leg for s in on_the_edge} == {LegId.LH, LegId.RH}


def test_no_swing_ever_gets_more_than_a_millimetre_of_margin(stability):
    """Every swing, not only the two that touch zero."""

    _, _, result = stability
    for swing in result.swings:
        assert swing.minimum_margin_m < 0.0011
        assert not swing.is_stable


def test_the_margin_decays_through_the_swing_rather_than_holding(stability):
    """Which is exactly why plan §13 requirement 5 forbids checking liftoff."""

    _, _, result = stability
    swing = next(s for s in result.swings if s.swing_leg is LegId.RF)
    margins = [s.margin_m for s in swing.samples]
    assert margins[0] > margins[-1]
    assert margins[0] > 0.015, "it starts comfortably inside"
    assert all(a >= b - 1e-12 for a, b in zip(margins, margins[1:])), \
        "monotone: the body advances while the contacts stay put"


def test_checking_liftoff_alone_would_have_called_this_stable(stability):
    """The concrete cost of requirement 5, in millimetres."""

    _, _, result = stability
    swing = next(s for s in result.swings if s.swing_leg is LegId.RF)
    assert swing.samples[0].margin_m > 0.010, "liftoff alone clears a 10 mm floor"
    assert swing.minimum_margin_m < 0.0011
    assert not swing.is_stable


def test_the_traversal_stores_its_own_minimum_and_the_worst_swing(stability):
    _, _, result = stability
    assert result.minimum_margin_m is not None
    assert result.worst_swing is not None
    assert result.minimum_margin_m == pytest.approx(
        result.worst_swing.minimum_margin_m)


def test_a_margin_below_the_floor_marks_the_swing_infeasible(stability):
    """Plan §13 requirement 7.

    A floor of exactly zero is not used here: the boundary margins come out at
    a few times 1e-14 m, so ``> 0.0`` would pass on arithmetic noise.  A
    micrometre is still far below anything physical and is the honest way to
    say "this needs a margin, not merely a non-negative number".
    """

    four, trajectory, result = stability
    assert not result.is_stable
    assert len(result.unstable_swings) == len(result.swings)
    micron = swing_stability_2d(four, trajectory, margin_floor_m=1e-6)
    assert not micron.is_stable
    assert len(micron.unstable_swings) >= 2, "the two boundary swings at least"


def test_the_body_assumption_travels_with_the_answer(stability):
    """Step 5 said body_z is infeasible; a margin must not hide that."""

    _, trajectory, result = stability
    assert not trajectory.is_feasible
    assert "INFEASIBLE" in result.body_basis
    assert "Step 5" in result.body_basis


def test_the_projected_point_is_not_called_a_CoM(stability):
    _, _, result = stability
    rows = stability_rows(result)
    assert all(len(r) == len(rows[0]) for r in rows)
    assert rows[0]["com_basis"] == COM_BASIS
    assert "NOT a whole-robot CoM" in COM_BASIS


def test_the_rows_carry_the_three_row_kinds(stability):
    _, _, result = stability
    kinds = {r["row_kind"] for r in stability_rows(result)}
    assert kinds == {"traversal", "swing", "sample"}
