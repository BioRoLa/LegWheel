"""Day 12 Step 4: per-leg sequences on the common timeline.

Plan §11's eight requirements.  Like Step 3's tests these are mostly synthetic:
the claims here are about *labelling, carrying through and reporting*, and a
synthetic crossing exercises them without re-running Day 6--7's traversal.  The
one place a real composer would be needed -- "did the crossing's motion survive
unchanged" -- is checked by identity instead, which is a stronger claim than
comparing values.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_composer_2d import ComposedSequence2D
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    BLOCKED_PAIRS,
    StrategyId,
    Verdict,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirement2D,
    BodyRequirementKind,
    FrameRef2D,
    MotionSegment2D,
    MotionSequence2D,
    PointContact2D,
    RollSampling2D,
    RollingContact2D,
    SwingSampling2D,
    SegmentKind,
    SwingShaping2D,
    TransitionKind,
    TransitionRequirement2D,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER, LegId
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    LegMode,
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    STRATEGY_HALVES,
    AirborneOverrun2D,
    airborne_overruns_2d,
    TRANSITION_PHASE_OF_KIND,
    LegPlan2D,
    PhasedSegment2D,
    TransitionPhase,
    build_leg_plan_2d,
    debug_rows,
    phase_of_kind,
    plan_four_legs_2d,
)


# --------------------------------------------------------------------------
# A synthetic crossing
# --------------------------------------------------------------------------


def _point(x: float, z: float = 0.0) -> PointContact2D:
    return PointContact2D(
        rim="foot_rim", alpha_rad=0.0, point_world_xz_m=(x, z),
        surface_id="ground", theta_rad=np.deg2rad(60.0), beta_rad=0.0,
        hip_xz_m=(x, z + 0.2194),
    )


def _roll(kind: SegmentKind, start: int, frames: int) -> MotionSegment2D:
    return MotionSegment2D(
        kind=kind, phase_label=kind.value,
        start_contact=_point(0.0), end_contact=_point(0.1),
        sampling=RollSampling2D(arc_samples=241, beta_step_rad=np.deg2rad(-1.0),
                                theta_step_rad=None),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.TRACK, x_range_m=(0.0, 0.1),
            hip_z_profile_m=np.full(frames, 0.2194)),
        frames=FrameRef2D(source_id="day10_11",
                          indices=tuple(range(start, start + frames))),
        rolling=RollingContact2D(
            rim="foot_rim", surface_ids=("ground",),
            alpha_range_rad=(np.deg2rad(-40.0), np.deg2rad(40.0)),
            beta_range_rad=(np.deg2rad(-40.0), np.deg2rad(40.0)),
            theta_range_rad=(np.deg2rad(60.0), np.deg2rad(60.0)),
            contact_start_xz_m=(0.0, 0.0), contact_end_xz_m=(0.1, 0.0)),
    )


#: What Day 8--9 planned every crossing swing with, carried through Day 10--11
#: unchanged (``day10_11_step9_body_requirements.csv``).  Quoted, not invented:
#: it is the only real time anywhere in the chain.
PLANNED_SWING_S = 0.6


def _swing(kind: SegmentKind, start: int, frames: int,
           duration_s: float | None = PLANNED_SWING_S) -> MotionSegment2D:
    return MotionSegment2D(
        kind=kind, phase_label=kind.value,
        start_contact=_point(0.1), end_contact=_point(0.3),
        sampling=SwingSampling2D(arc_samples=241, sample_count=frames,
                                 leg_arc_samples=241,
                                 max_joint_step_rad=np.deg2rad(2.0)),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.LOWER_BOUND, x_range_m=(0.1, 0.3),
            hip_z_min_m=0.2194),
        frames=FrameRef2D(source_id="day10_11",
                          indices=tuple(range(start, start + frames))),
        duration_s=duration_s,
        swing_shaping=SwingShaping2D(
            apex_clearance_m=0.05, liftoff_rise_m=0.01, touchdown_drop_m=0.01,
            duration_scale=1.0, mid_fractions=(0.25, 0.5, 0.75)),
    )


def _sequence(*segments) -> MotionSequence2D:
    return MotionSequence2D(terrain_id="synthetic", segments=tuple(segments))


def _composed(strategy: StrategyId, *segments, **kwargs) -> ComposedSequence2D:
    return ComposedSequence2D(
        strategy=strategy, height_m=0.08, top_length_m=0.30,
        sequence=_sequence(*segments) if segments else None,
        refusal=kwargs.pop("refusal", None), **kwargs,
    )


def _swing_swing() -> ComposedSequence2D:
    return _composed(
        StrategyId.SWING_SWING,
        _swing(SegmentKind.SWING_UP, 0, 40),
        _swing(SegmentKind.SWING_DOWN, 40, 40),
    )


def _roll_roll() -> ComposedSequence2D:
    return _composed(
        StrategyId.ROLL_ROLL,
        _roll(SegmentKind.APPROACH, 0, 20),
        _roll(SegmentKind.ROLL_UP, 20, 60),
        _roll(SegmentKind.WHEEL_TRANSITION, 80, 30),
        _roll(SegmentKind.ROLL_DOWN, 110, 60),
    )


# --------------------------------------------------------------------------
# 1. the phase mapping is exhaustive and read off the kind, not guessed
# --------------------------------------------------------------------------


def test_every_terrain_transition_kind_has_a_phase():
    """A new transition kind must be given a phase, not defaulted into one."""

    missing = [k for k in SegmentKind
               if k.is_terrain_transition and k not in TRANSITION_PHASE_OF_KIND]
    assert missing == [], f"unmapped transition kinds: {missing}"


def test_no_nominal_kind_is_in_the_transition_map():
    for kind in TRANSITION_PHASE_OF_KIND:
        assert kind.is_terrain_transition


def test_ascent_and_descent_kinds_land_on_their_own_phase():
    for kind in (SegmentKind.APPROACH, SegmentKind.ROLL_UP, SegmentKind.SWING_UP):
        assert phase_of_kind(kind, after_transition=False) is TransitionPhase.ASCENT
    for kind in (SegmentKind.ROLL_DOWN, SegmentKind.SWING_DOWN,
                 SegmentKind.POST_TOUCHDOWN_ROLL):
        assert phase_of_kind(kind, after_transition=False) is TransitionPhase.DESCENT
    assert phase_of_kind(SegmentKind.WHEEL_TRANSITION,
                         after_transition=False) is TransitionPhase.ON_TOP


def test_swing_over_gets_its_own_phase_rather_than_a_fabricated_split():
    """``#5`` crosses in one primitive; inventing ASCENT/DESCENT would lie."""

    assert phase_of_kind(SegmentKind.SWING_OVER,
                         after_transition=False) is TransitionPhase.OVER


def test_a_nominal_kind_takes_the_phase_from_where_it_was_put():
    """The same FOOT_RIM_ROLL is 'before' or 'after' by position alone."""

    assert phase_of_kind(SegmentKind.FOOT_RIM_ROLL,
                         after_transition=False) is TransitionPhase.NOMINAL_BEFORE
    assert phase_of_kind(SegmentKind.FOOT_RIM_ROLL,
                         after_transition=True) is TransitionPhase.NOMINAL_AFTER
    assert phase_of_kind(SegmentKind.RECOVERY_SWING,
                         after_transition=True) is TransitionPhase.NOMINAL_AFTER


def test_phase_is_nominal_and_is_transition_partition_the_enum():
    for phase in TransitionPhase:
        assert phase.is_nominal != phase.is_transition


def test_every_strategy_has_both_halves_named():
    """Plan §11: ascent and descent stay separate decisions."""

    assert set(STRATEGY_HALVES) == set(StrategyId)
    for strategy, (up, down) in STRATEGY_HALVES.items():
        assert up and down
        if strategy is not StrategyId.SWING_OVER:
            assert ("UP" in up) and ("DOWN" in down)


# --------------------------------------------------------------------------
# 2. nominal runs are inserted outside the crossing (requirement 3)
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def swing_plan() -> LegPlan2D:
    return build_leg_plan_2d(LegId.LF, _swing_swing(),
                             cycles_before=1, cycles_after=1)


@pytest.fixture(scope="module")
def roll_plan() -> LegPlan2D:
    return build_leg_plan_2d(LegId.LF, _roll_roll())


@pytest.fixture(scope="module")
def blocked_plan() -> LegPlan2D:
    return build_leg_plan_2d(LegId.LF, _blocked())


def test_the_crossing_is_flanked_by_nominal_runs(swing_plan):
    phases = [p.phase for p in swing_plan.phased]
    assert phases[0] is TransitionPhase.NOMINAL_BEFORE
    assert phases[-1] is TransitionPhase.NOMINAL_AFTER
    assert TransitionPhase.ASCENT in phases
    assert TransitionPhase.DESCENT in phases


def test_a_nominal_run_is_a_whole_cycle_not_a_bare_roll(swing_plan):
    """Plan §0.2: nominal locomotion is FOOT_RIM_ROLL + RECOVERY_SWING."""

    for phase in (TransitionPhase.NOMINAL_BEFORE, TransitionPhase.NOMINAL_AFTER):
        kinds = [p.kind for p in swing_plan.segments_in(phase)]
        assert kinds == [SegmentKind.FOOT_RIM_ROLL, SegmentKind.RECOVERY_SWING]


def test_nominal_runs_can_be_asked_for_separately():
    plan = build_leg_plan_2d(LegId.RF, _swing_swing(),
                             cycles_before=2, cycles_after=0)
    assert len(plan.segments_in(TransitionPhase.NOMINAL_BEFORE)) == 4
    assert plan.segments_in(TransitionPhase.NOMINAL_AFTER) == ()


def test_the_chain_records_both_sources(swing_plan):
    """The crossing's frames and the flat runs' frames are not one numbering."""

    assert len(swing_plan.chain.sources) > 1
    assert "day10_11" in swing_plan.chain.sources


# --------------------------------------------------------------------------
# 3. the crossing's motion is carried through, not rewritten (requirement 4)
# --------------------------------------------------------------------------


def test_the_transition_segments_are_the_very_objects_day_10_11_produced():
    """Identity, not equality: nothing was rebuilt, so nothing can drift."""

    composed = _swing_swing()
    plan = build_leg_plan_2d(LegId.LF, composed, cycles_before=0, cycles_after=0)
    carried = [p.segment for p in plan.transition_segments]
    assert carried == list(composed.sequence.segments)
    for mine, theirs in zip(carried, composed.sequence.segments):
        assert mine is theirs


def test_sampling_and_body_requirement_are_read_through_not_copied():
    composed = _swing_swing()
    plan = build_leg_plan_2d(LegId.LF, composed, cycles_before=0, cycles_after=0)
    phased = plan.transition_segments[0]
    assert phased.segment.sampling is composed.sequence.segments[0].sampling
    assert phased.body_kind is BodyRequirementKind.LOWER_BOUND


def test_durations_are_still_unassigned_before_scheduling(swing_plan):
    """Step 4 does not invent time; Step 3's scheduler assigns it."""

    assert all(p.segment.duration_s is None
               for p in swing_plan.phased
               if p.kind.is_nominal_locomotion)


def test_phased_segment_reports_the_mode_the_scheduler_will_use(roll_plan):
    plan = roll_plan
    for phased in plan.transition_segments:
        assert phased.mode is LegMode.of(phased.kind)
    assert all(p.mode is LegMode.STANCE for p in plan.transition_segments)


# --------------------------------------------------------------------------
# 4. ascent and descent stay independent (requirement 2)
# --------------------------------------------------------------------------


def test_a_roll_roll_plan_names_both_halves(roll_plan):
    plan = roll_plan
    assert plan.ascent_strategy == "ROLL_UP"
    assert plan.descent_strategy == "ROLL_DOWN"
    assert plan.phases_are_separable


def test_a_swing_swing_plan_names_both_halves(swing_plan):
    assert swing_plan.ascent_strategy == "SWING_UP"
    assert swing_plan.descent_strategy == "SWING_DOWN"
    assert swing_plan.phases_are_separable


def test_swing_over_says_its_halves_are_not_separable():
    """The honest answer for ``#5``: there is no split to report."""

    composed = _composed(StrategyId.SWING_OVER,
                         _swing(SegmentKind.SWING_OVER, 0, 60))
    plan = build_leg_plan_2d(LegId.LF, composed,
                             cycles_before=0, cycles_after=0)
    assert plan.phases_are_separable is False
    assert plan.segments_in(TransitionPhase.OVER)
    assert plan.segments_in(TransitionPhase.ASCENT) == ()
    assert plan.segments_in(TransitionPhase.DESCENT) == ()


def test_the_obstacle_is_not_collapsed_into_one_decision(roll_plan):
    """Plan §11 forbids 'this obstacle = ROLL'; the phases must be addressable."""

    plan = roll_plan
    assert len(plan.segments_in(TransitionPhase.ASCENT)) == 2
    assert len(plan.segments_in(TransitionPhase.ON_TOP)) == 1
    assert len(plan.segments_in(TransitionPhase.DESCENT)) == 1


# --------------------------------------------------------------------------
# 5. an unresolved crossing stays unresolved (requirement 5)
# --------------------------------------------------------------------------


def _blocked() -> ComposedSequence2D:
    blocked = BLOCKED_PAIRS[StrategyId.ROLL_SWING]
    requirement = TransitionRequirement2D(
        kind=TransitionKind.TOP_REPOSITION,
        source_contact=_point(0.2, 0.16),
        target_condition="LEFT_RIM_READY on the top surface",
        evidence="the two primitives will not chain (spec 5.5).",
    )
    return ComposedSequence2D(
        strategy=StrategyId.ROLL_SWING, height_m=0.16, top_length_m=0.35,
        sequence=None, refusal=blocked.summary, verdict=blocked.verdict,
        unresolved=(requirement,),
    )


def test_a_blocked_pair_does_not_become_two_nominal_runs_touching(blocked_plan):
    """The hole must stay visible; flat runs must not close over it."""

    plan = blocked_plan
    assert plan.transition_segments == ()
    assert plan.unresolved
    assert plan.is_executable is False
    assert plan.refusal


def test_the_unresolved_requirement_is_carried_onto_the_chain(blocked_plan):
    plan = blocked_plan
    assert plan.chain.unresolved == plan.unresolved
    assert plan.chain.is_complete is False


def test_a_resolved_plan_has_nothing_unresolved(swing_plan):
    assert swing_plan.unresolved == ()
    assert swing_plan.refusal is None


# --------------------------------------------------------------------------
# 6. exit state / entry state (requirement 6)
# --------------------------------------------------------------------------


def test_chain_breaks_are_measured_and_reported_not_repaired(swing_plan):
    """A synthetic crossing does not join the flat runs; that must show."""

    plan = swing_plan
    assert plan.breaks, "a discontinuous chain must report breaks"
    assert plan.is_executable is False
    # Nothing was moved to make it fit: the segments are still the originals.
    assert len(plan.phased) == 2 + 2 + 2


def test_a_broken_chain_is_still_schedulable_so_the_break_can_be_seen():
    """Refusing to schedule would hide the break instead of reporting it."""

    plans = {leg: build_leg_plan_2d(leg, _swing_swing()) for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, walk_timing_2d())
    assert four.schedule.scheduled
    assert four.is_executable is False
    assert set(four.unresolved_legs) == set(LEG_ORDER)


# --------------------------------------------------------------------------
# 7. conflicts are reported, not fixed (requirement 7)
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def four_swing():
    plans = {leg: build_leg_plan_2d(leg, _swing_swing()) for leg in LEG_ORDER}
    return plan_four_legs_2d(plans, walk_timing_2d())


def test_the_plan_is_not_executable_while_a_conflict_stands(four_swing):
    if four_swing.schedule.conflicts:
        assert four_swing.is_executable is False


def test_executability_needs_both_legs_and_schedule(four_swing):
    """Conjunctive on purpose: either half alone would hide the other."""

    assert four_swing.is_executable == (
        all(p.is_executable for p in four_swing.plans.values())
        and not four_swing.schedule.conflicts
    )


def test_nothing_shortens_a_segment_to_make_it_fit(four_swing):
    """Scheduling assigns windows; it must not drop or split the plan."""

    for leg, plan in four_swing.plans.items():
        assert len(four_swing.schedule.segments_of(leg)) == len(plan.phased)


def test_the_schedule_still_marks_its_durations_as_assigned(four_swing):
    assert all(s.duration_is_assigned for s in four_swing.schedule.scheduled)


# --------------------------------------------------------------------------
# 8. the debug table (requirement 8)
# --------------------------------------------------------------------------


def test_debug_table_has_the_five_columns_the_plan_asks_for(four_swing):
    rows = debug_rows(four_swing)
    segments = [r for r in rows if r["row_kind"] == "segment"]
    assert segments
    for row in segments:
        for column in ("start_s", "end_s", "leg", "segment_kind",
                       "contact_state", "body_kind"):
            assert column in row


def test_debug_table_is_csv_writable_one_header_for_every_row(four_swing):
    """Day 10-11 trap 21: a heterogeneous row list needs a field union."""

    rows = debug_rows(four_swing)
    assert len({tuple(r) for r in rows}) == 1


def test_debug_table_carries_the_phase_beside_the_kind(four_swing):
    rows = [r for r in debug_rows(four_swing) if r["row_kind"] == "segment"]
    phases = {r["phase"] for r in rows}
    assert TransitionPhase.ASCENT.value in phases
    assert TransitionPhase.DESCENT.value in phases
    assert TransitionPhase.NOMINAL_BEFORE.value in phases


def test_debug_table_reports_the_breaks_it_found(four_swing):
    rows = debug_rows(four_swing)
    assert [r for r in rows if r["row_kind"] == "chain_break"]


def test_debug_table_reports_an_unresolved_crossing():
    plans = {leg: build_leg_plan_2d(leg, _blocked()) for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, walk_timing_2d())
    rows = debug_rows(four)
    assert [r for r in rows if r["row_kind"] == "unresolved_transition"]
    assert [r for r in rows if r["row_kind"] == "plan"][0]["is_executable"] is False


def test_contact_state_is_the_scheduler_s_mode_not_a_second_opinion(four_swing):
    by_key = {(s.leg.value, s.segment_index): s
              for s in four_swing.schedule.scheduled}
    for row in debug_rows(four_swing):
        if row["row_kind"] != "segment":
            continue
        scheduled = by_key[(row["leg"], row["segment_index"])]
        assert row["contact_state"] == scheduled.mode.value


# --------------------------------------------------------------------------
# 9. what the window rule would otherwise hide (requirement 7 again)
# --------------------------------------------------------------------------


def test_a_crossing_is_reported_as_compressed_not_quietly_squeezed(four_swing):
    """The swing window is sized for the recovery; the crossing shares it."""

    overruns = four_swing.airborne_overruns
    assert overruns, "a crossing packed into one swing window must be reported"
    for overrun in overruns:
        assert overrun.compression > 1.0
        assert "SWING_UP" in overrun.segment_kinds
        assert "SWING_DOWN" in overrun.segment_kinds


def test_the_compression_is_measured_against_the_planned_durations(four_swing):
    """Two 0.6 s swings cannot fit a 0.6 s window, whatever the frame counts."""

    overrun = four_swing.airborne_overruns[0]
    assert overrun.planned_s == pytest.approx(2 * PLANNED_SWING_S)
    assert overrun.compression == pytest.approx(
        overrun.planned_s / overrun.window_s)
    # The window is exactly one swing window, so the factor is exactly 2 --
    # and the untimed recovery still has to fit inside the same 0.6 s.
    assert overrun.window_s == pytest.approx(walk_timing_2d().swing_duration_s)
    assert overrun.compression == pytest.approx(2.0)


def test_the_recovery_sharing_the_window_is_counted_not_ignored(four_swing):
    """The untimed recovery is in the same run and also needs part of it."""

    assert four_swing.airborne_overruns[0].untimed_segments >= 1


def test_a_purely_nominal_plan_has_no_overrun():
    """The nominal cycle is what the window was sized for; it must not flag."""

    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="no crossing in this run",
    )
    plans = {leg: build_leg_plan_2d(leg, composed) for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, walk_timing_2d())
    assert airborne_overruns_2d(four) == ()


def test_an_overrun_alone_makes_the_plan_not_executable(four_swing):
    assert four_swing.airborne_overruns
    assert four_swing.is_executable is False


def test_the_debug_table_carries_the_overrun_rows(four_swing):
    rows = debug_rows(four_swing)
    overruns = [r for r in rows if r["row_kind"] == "airborne_overrun"]
    assert overruns
    assert float(overruns[0]["compression"]) > 1.0
