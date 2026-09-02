"""Day 12 Step 3: the four-leg timing skeleton.

Plan §10's eight requirements.  Most tests build synthetic chains, because the
claims are about *timing* and a synthetic chain exercises them in a second
rather than in a minute; one integration test uses the real Step 1 cycles so
the numbers in the module docstring are checked against what Step 1 produces.
"""

import numpy as np
import pytest

from legwheel.planners.gait_generator_3d import GAIT_LIBRARY

from hybrid_note.scripts.experiments.day10_11_concession_2d import BodyRequirementKind
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirement2D,
    FrameRef2D,
    MotionSegment2D,
    PointContact2D,
    RecoveryShaping2D,
    RollSampling2D,
    RollingContact2D,
    SegmentKind,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER, LegId
from hybrid_note.scripts.experiments.day12_segment_contract_2d import SegmentChain2D
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    FourLegSchedule2D,
    GaitTiming2D,
    LegMode,
    ScheduledSegment2D,
    rotation_rate_demand_2d,
    schedule_chains_2d,
    schedule_rows,
    walk_timing_2d,
)

#: Step 1's measured rotations, quoted so the ratio in the module docstring is
#: checked rather than restated.
STROKE_ROTATION_RAD = float(np.deg2rad(79.6875))
RECOVERY_ROTATION_RAD = float(np.deg2rad(280.3125))


# --------------------------------------------------------------------------
# Synthetic chains
# --------------------------------------------------------------------------


def _point(x: float, theta_deg: float = 60.0) -> PointContact2D:
    return PointContact2D(
        rim="foot_rim", alpha_rad=0.0, point_world_xz_m=(x, 0.0),
        surface_id="ground", theta_rad=np.deg2rad(theta_deg), beta_rad=0.0,
        hip_xz_m=(x, 0.2194),
    )


def _roll(start: int, frames: int, *, kind=SegmentKind.FOOT_RIM_ROLL,
          source="synthetic") -> MotionSegment2D:
    """``start`` is the first frame index; the caller keeps them disjoint."""

    return MotionSegment2D(
        kind=kind, phase_label=kind.value,
        start_contact=_point(0.0), end_contact=_point(0.2),
        sampling=RollSampling2D(arc_samples=241, beta_step_rad=np.deg2rad(-1.0),
                                theta_step_rad=None),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.TRACK, x_range_m=(0.0, 0.2),
            hip_z_profile_m=np.full(frames, 0.2194)),
        frames=FrameRef2D(source_id=source,
                          indices=tuple(range(start, start + frames))),
        rolling=RollingContact2D(
            rim="foot_rim", surface_ids=("ground",),
            alpha_range_rad=(np.deg2rad(-40.0), np.deg2rad(40.0)),
            beta_range_rad=(np.deg2rad(-40.0), np.deg2rad(40.0)),
            theta_range_rad=(np.deg2rad(60.0), np.deg2rad(60.0)),
            contact_start_xz_m=(0.0, 0.0), contact_end_xz_m=(0.2, 0.0)),
    )


def _recovery(start: int, frames: int, source="synthetic") -> MotionSegment2D:
    return MotionSegment2D(
        kind=SegmentKind.RECOVERY_SWING, phase_label="RECOVERY_SWING",
        start_contact=_point(0.2), end_contact=_point(0.5),
        sampling=RollSampling2D(arc_samples=241, beta_step_rad=np.deg2rad(-4.0),
                                theta_step_rad=np.deg2rad(2.0)),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.LOWER_BOUND, x_range_m=(0.2, 0.5),
            hip_z_min_m=0.2194),
        frames=FrameRef2D(source_id=source,
                          indices=tuple(range(start, start + frames))),
        recovery_shaping=RecoveryShaping2D(
            theta_compact_rad=np.deg2rad(17.0),
            theta_touchdown_rad=np.deg2rad(60.0),
            airborne_rotation_rad=RECOVERY_ROTATION_RAD,
            min_clearance_m=0.05, hip_advance_m=0.0),
    )


def _chain(leg: LegId, *specs) -> SegmentChain2D:
    """Build a chain from ``(builder, frames)`` pairs with disjoint frames."""

    segments, cursor = [], 0
    for builder, frames, *rest in specs:
        kwargs = rest[0] if rest else {}
        segments.append(builder(cursor, frames, **kwargs))
        cursor += frames
    return SegmentChain2D(leg_id=leg.value, segments=tuple(segments))


def _nominal_chain(leg: LegId, cycles: int = 2) -> SegmentChain2D:
    specs = []
    for _ in range(cycles):
        specs += [(_roll, 49), (_recovery, 119)]
    return _chain(leg, *specs)


def _four_nominal_chains(cycles: int = 2) -> dict[LegId, SegmentChain2D]:
    return {leg: _nominal_chain(leg, cycles) for leg in LEG_ORDER}


@pytest.fixture(scope="module")
def schedule() -> FourLegSchedule2D:
    return schedule_chains_2d(_four_nominal_chains())


# --------------------------------------------------------------------------
# §10(1): the existing walk definition, reused and verified
# --------------------------------------------------------------------------


def test_the_timing_comes_from_the_projects_own_gait_library():
    timing = walk_timing_2d()
    definition = GAIT_LIBRARY["Walk"]
    assert timing.stance_duty == definition["stance_duty"]
    assert list(timing.phase_offsets) == list(definition["phase_offsets"])
    assert timing.gait_name == "Walk"


def test_the_phase_offsets_are_indexed_by_the_project_leg_index():
    """``GAIT_LIBRARY`` lists ``[FL, FR, RR, RL]``.  Storing it in the plan's
    LF/RF/LH/RH reading order would swap the two hind legs."""

    timing = walk_timing_2d()
    assert timing.phase_offsets[LegId.LF.index] == 0.75
    assert timing.phase_offsets[LegId.RF.index] == 0.25
    assert timing.phase_offsets[LegId.RH.index] == 0.5
    assert timing.phase_offsets[LegId.LH.index] == 0.0


def test_the_four_swing_windows_tile_the_cycle_exactly_once():
    """The property that makes this gait the one-airborne-leg structure.

    Derived from the phase convention, not tabulated, so this is a check on
    the convention rather than on a copied table.
    """

    timing = walk_timing_2d()
    windows = sorted(timing.swing_window(leg) for leg in LEG_ORDER)
    assert windows[0][0] == pytest.approx(0.0)
    assert windows[-1][1] == pytest.approx(1.0)
    for before, after in zip(windows, windows[1:]):
        assert before[1] == pytest.approx(after[0])
    for lo, hi in windows:
        assert hi - lo == pytest.approx(1.0 - timing.stance_duty)


def test_the_swing_order_is_the_one_the_gait_file_claims():
    """That file's comment says ``FL -> RR -> FR -> RL``."""

    timing = walk_timing_2d()
    order = sorted(LEG_ORDER, key=lambda leg: timing.swing_window(leg)[0])
    assert [leg.value for leg in order] == ["LF", "RH", "RF", "LH"]


def test_a_duty_below_three_quarters_cannot_hold_one_leg_airborne():
    """A property of the number, not of the schedule: with four legs the
    airborne fraction must be at most a quarter."""

    assert walk_timing_2d().max_simultaneous_airborne == 1
    trot = GaitTiming2D(cycle_period_s=2.4, stance_duty=0.6,
                        phase_offsets=(0.0, 0.5, 0.0, 0.5), gait_name="Trot")
    assert trot.max_simultaneous_airborne == 2


# --------------------------------------------------------------------------
# The measurement this step exists to surface
# --------------------------------------------------------------------------


def test_time_proportional_to_rotation_would_make_the_constraint_unreachable():
    """Step 1's cycle rolls 79.69 deg and flies 280.31 deg.  If time followed
    rotation, stance would be 0.221 of the cycle and three legs would be
    airborne on average."""

    demand = rotation_rate_demand_2d(
        STROKE_ROTATION_RAD, RECOVERY_ROTATION_RAD, walk_timing_2d()
    )
    assert demand["rotation_proportional_duty"] == pytest.approx(0.2214, abs=1e-4)
    implied = GaitTiming2D(
        cycle_period_s=2.4, stance_duty=demand["rotation_proportional_duty"],
        phase_offsets=(0.75, 0.25, 0.5, 0.0),
    )
    assert implied.max_simultaneous_airborne > 1


def test_the_one_airborne_leg_constraint_costs_a_tenfold_beta_rate():
    """The price of the constraint, in the currency it is actually paid in."""

    demand = rotation_rate_demand_2d(
        STROKE_ROTATION_RAD, RECOVERY_ROTATION_RAD, walk_timing_2d()
    )
    assert demand["swing_to_stance_rate_ratio"] == pytest.approx(10.553, abs=1e-3)


def test_the_ratio_does_not_depend_on_the_cycle_period():
    """The period is the one number Step 3 had to choose; nothing may turn on
    it, or the choice would be a hidden result."""

    fast = rotation_rate_demand_2d(
        STROKE_ROTATION_RAD, RECOVERY_ROTATION_RAD, walk_timing_2d(0.8)
    )
    slow = rotation_rate_demand_2d(
        STROKE_ROTATION_RAD, RECOVERY_ROTATION_RAD, walk_timing_2d(9.0)
    )
    assert fast["swing_to_stance_rate_ratio"] == pytest.approx(
        slow["swing_to_stance_rate_ratio"]
    )


# --------------------------------------------------------------------------
# §10(3)(4)(6): the schedule
# --------------------------------------------------------------------------


def test_every_leg_has_a_mode_on_every_covered_interval(schedule):
    """§10(4): an explicit state on every timeline interval."""

    lo, hi = schedule.covered_interval_s
    for t in np.linspace(lo, hi, 61)[:-1]:
        for leg in schedule.legs:
            assert schedule.segment_at(leg, float(t)) is not None


def test_at_most_one_leg_is_airborne(schedule):
    assert schedule.max_airborne_count == 1
    assert schedule.one_leg_airborne_at_a_time
    assert schedule.conflicts == ()


def test_every_swing_leaves_exactly_three_supports(schedule):
    assert schedule.every_swing_has_three_supports
    lo, hi = schedule.covered_interval_s
    for t in np.linspace(lo, hi, 61)[:-1]:
        airborne = schedule.airborne_legs_at(float(t))
        support = schedule.support_legs_at(float(t))
        assert len(airborne) + len(support) == 4
        if airborne:
            assert len(support) == 3


def test_the_scheduler_exposes_what_the_plan_asks_for(schedule):
    """§10(6): active/swing leg, three support legs, segment index and kind
    per leg, start and end time of each segment."""

    lo, hi = schedule.covered_interval_s
    middle = 0.5 * (lo + hi)
    swing = schedule.swing_leg_at(middle)
    assert swing is None or swing in LEG_ORDER
    assert len(schedule.support_legs_at(middle)) in (3, 4)
    for segment in schedule.scheduled:
        assert isinstance(segment, ScheduledSegment2D)
        assert segment.end_s > segment.start_s
        assert isinstance(segment.segment_kind, SegmentKind)
        assert segment.segment_index >= 0


def test_the_swing_order_on_the_clock_matches_the_gait(schedule):
    lo, hi = schedule.covered_interval_s
    seen: list[LegId] = []
    for t in np.linspace(lo, hi, 400)[:-1]:
        airborne = schedule.airborne_legs_at(float(t))
        if airborne and (not seen or seen[-1] is not airborne[0]):
            seen.append(airborne[0])
    assert [leg.value for leg in seen[:4]] == ["LF", "RH", "RF", "LH"]


def test_one_leg_does_one_thing_at_a_time(schedule):
    for leg in schedule.legs:
        own = schedule.segments_of(leg)
        for before, after in zip(own, own[1:]):
            assert after.start_s == pytest.approx(before.end_s)


def test_stance_and_swing_windows_have_the_duty_durations(schedule):
    timing = schedule.timing
    for leg in schedule.legs:
        windows: dict[int, float] = {}
        for segment in schedule.segments_of(leg):
            windows[segment.window_index] = windows.get(
                segment.window_index, 0.0
            ) + segment.duration_s
        for index, total in windows.items():
            mode = next(
                s.mode for s in schedule.segments_of(leg)
                if s.window_index == index
            )
            expected = (
                timing.stance_duration_s if mode is LegMode.STANCE
                else timing.swing_duration_s
            )
            assert total == pytest.approx(expected)


# --------------------------------------------------------------------------
# Ragged ends are not gait errors
# --------------------------------------------------------------------------


def test_the_ragged_ends_are_reported_separately(schedule):
    """A phase offset is an offset in time, so the legs' chains do not start
    together.  Counting supports there would count legs that are not in the
    plan yet as airborne."""

    lo, hi = schedule.covered_interval_s
    assert lo > schedule.start_s
    assert hi < schedule.end_s
    assert len(schedule.ragged_intervals_s) == 2
    for start, end in schedule.ragged_intervals_s:
        assert end > start


def test_the_constraints_are_evaluated_on_the_covered_interval_only(schedule):
    """Outside it a leg has no segment at all, and 'not scheduled' is not
    'airborne'."""

    head_start, head_end = schedule.ragged_intervals_s[0]
    middle = 0.5 * (head_start + head_end)
    assert len(schedule.support_legs_at(middle)) < 4
    # ...and that does not make the schedule invalid.
    assert schedule.one_leg_airborne_at_a_time


# --------------------------------------------------------------------------
# Conflicts are reported, never repaired
# --------------------------------------------------------------------------


def test_a_leg_out_of_phase_produces_a_reported_conflict():
    """A chain that begins airborne puts its swing window where the gait did
    not plan one.  Step 3 must say so, not silently reshape it."""

    chains = _four_nominal_chains()
    chains[LegId.RF] = _chain(
        LegId.RF, (_recovery, 119), (_roll, 49), (_recovery, 119)
    )
    schedule = schedule_chains_2d(chains)
    assert schedule.max_airborne_count > 1
    assert not schedule.one_leg_airborne_at_a_time
    assert schedule.conflicts
    for conflict in schedule.conflicts:
        assert len(conflict.legs) > 1
        assert conflict.duration_s > 0.0


def test_asking_for_the_swing_leg_during_a_conflict_raises():
    """Handing back one of several would hide exactly what this module is for."""

    chains = _four_nominal_chains()
    chains[LegId.RF] = _chain(
        LegId.RF, (_recovery, 119), (_roll, 49), (_recovery, 119)
    )
    schedule = schedule_chains_2d(chains)
    conflict = schedule.conflicts[0]
    middle = 0.5 * (conflict.start_s + conflict.end_s)
    with pytest.raises(ValueError, match="legs are airborne"):
        schedule.swing_leg_at(middle)
    # The plural accessor still answers.
    assert len(schedule.airborne_legs_at(middle)) > 1


# --------------------------------------------------------------------------
# §10(5): arbitrary sequences, no obstacle object
# --------------------------------------------------------------------------


def test_a_terrain_transition_chain_schedules_without_any_obstacle_object():
    """The input is a SegmentChain2D, so a transition sequence works with no
    change to this module."""

    transition = _chain(
        LegId.LF,
        (_roll, 10, {"kind": SegmentKind.APPROACH}),
        (_roll, 40, {"kind": SegmentKind.ROLL_UP}),
        (_roll, 30, {"kind": SegmentKind.WHEEL_TRANSITION}),
        (_recovery, 60),
        (_roll, 49),
    )
    chains = _four_nominal_chains()
    chains[LegId.LF] = transition
    schedule = schedule_chains_2d(chains)
    kinds = {s.segment_kind for s in schedule.segments_of(LegId.LF)}
    assert SegmentKind.ROLL_UP in kinds and SegmentKind.WHEEL_TRANSITION in kinds
    # The three contact segments share one stance window.
    first_window = [
        s for s in schedule.segments_of(LegId.LF) if s.window_index == 0
    ]
    assert len(first_window) == 3
    assert sum(s.duration_s for s in first_window) == pytest.approx(
        schedule.timing.stance_duration_s
    )


def test_segments_in_one_run_split_the_window_by_frame_count():
    """The modelling choice, made explicit: frames are the only extensive
    quantity these segments carry."""

    chains = _four_nominal_chains()
    chains[LegId.LF] = _chain(
        LegId.LF,
        (_roll, 30, {"kind": SegmentKind.APPROACH}),
        (_roll, 10, {"kind": SegmentKind.ROLL_UP}),
        (_recovery, 60),
    )
    schedule = schedule_chains_2d(chains)
    window = [s for s in schedule.segments_of(LegId.LF) if s.window_index == 0]
    assert window[0].duration_s / window[1].duration_s == pytest.approx(3.0)


def test_a_schedule_needs_at_least_one_leg():
    with pytest.raises(ValueError, match="at least one leg"):
        schedule_chains_2d({})


# --------------------------------------------------------------------------
# §10(7): timing only, and time is an assigned quantity
# --------------------------------------------------------------------------


def test_every_duration_is_marked_as_assigned(schedule):
    """Day 10--11 trap 33: nothing upstream carries a duration, so every one of
    these is Step 3's own modelling decision."""

    for segment in schedule.scheduled:
        assert segment.duration_is_assigned
    for row in schedule.rows():
        assert row["duration_is_assigned"] is True


def test_the_input_chains_carry_no_duration():
    chain = _nominal_chain(LegId.LF)
    assert chain.total_duration_s is None
    assert len(chain.untimed_segments) == len(chain.segments)


def test_the_schedule_reports_no_support_polygon(schedule):
    """§10(7): stability is Step 6.  ``support_legs_at`` returns legs, and
    nothing here computes a polygon or a margin."""

    lo, hi = schedule.covered_interval_s
    support = schedule.support_legs_at(0.5 * (lo + hi))
    assert all(isinstance(leg, LegId) for leg in support)
    assert not hasattr(schedule, "stability_margin")
    assert not hasattr(schedule, "support_polygon")


def test_the_report_table_has_timing_segments_conflicts_and_summary(schedule):
    rows = schedule_rows(schedule)
    kinds = [row["row_kind"] for row in rows]
    assert kinds.count("timing") == 1
    assert kinds.count("summary") == 1
    assert kinds.count("segment") == len(schedule.scheduled)
    summary = next(r for r in rows if r["row_kind"] == "summary")
    assert summary["max_airborne_count"] == 1
    assert summary["ragged_interval_count"] == 2


# --------------------------------------------------------------------------
# Against the real Step 1 cycles
# --------------------------------------------------------------------------


def test_the_real_step1_cycles_schedule_with_one_leg_airborne():
    """The integration check: the chains Step 1 actually produces, on the
    timeline Step 3 actually builds."""

    from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
        cycle_segments_2d, run_nominal_cycles_2d,
    )

    cycles = run_nominal_cycles_2d(2)
    segments, offset = [], 0
    for cycle in cycles:
        pair = cycle_segments_2d(cycle, source_id="day12_step3",
                                 frame_offset=offset)
        segments.extend(pair)
        offset += sum(s.frames.frame_count for s in pair)

    schedule = schedule_chains_2d({
        leg: SegmentChain2D(leg_id=leg.value, segments=tuple(segments))
        for leg in LEG_ORDER
    })
    assert schedule.max_airborne_count == 1
    assert schedule.every_swing_has_three_supports
    assert schedule.conflicts == ()

    demand = rotation_rate_demand_2d(
        cycles[0].stroke.rotation_rad, cycles[0].recovery.rotation_rad,
        schedule.timing,
    )
    assert demand["swing_to_stance_rate_ratio"] == pytest.approx(10.553, abs=1e-3)
