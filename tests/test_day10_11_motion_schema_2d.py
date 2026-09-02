"""Day 10--11 Step 6: the segment schema's contract.

Two of the three completion criteria are round-trips over real data (Day 6--7
Step 10R's traversal, one Day 8--9 swing); the third asks for a **test that
proves** a segment missing any one sampling parameter cannot be rebuilt.  That
last one is not satisfied by asserting the field exists, so it is demonstrated:
each field is shown to change the trajectory it describes.
"""

import csv
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_concession_2d import BodyRequirementKind
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirement2D,
    FrameRef2D,
    MotionSegment2D,
    MotionSequence2D,
    PointContact2D,
    RollSampling2D,
    RollingContact2D,
    RollingMode,
    SegmentKind,
    SwingSampling2D,
    SwingShaping2D,
    TransitionKind,
    TransitionRequirement2D,
)
from hybrid_note.scripts.experiments.day10_11_sequence_builders_2d import (
    coverage_report_2d,
    handoff_report_2d,
    sequence_from_traversal_frames_2d,
)

DAY6_7 = (
    Path(__file__).resolve().parents[1]
    / "hybrid_note" / "notes" / "day6-7"
    / "day6_7_step10r_full_traversal_frames.csv"
)


# --------------------------------------------------------------------------
# The kinds the plan requires the schema to be able to express
# --------------------------------------------------------------------------


def test_the_schema_can_express_the_two_kinds_day_10_11_never_generates():
    """Research plan 10.5.4.

    A long path spends most of its distance in ``WHEEL_ROLL``, and a swing that
    lands mid-obstacle continues in ``POST_TOUCHDOWN_ROLL``.  Adding either
    after Day 12 would mean rewriting every consumer, so the schema carries
    them now even though nothing here produces one.
    """

    assert SegmentKind.WHEEL_ROLL in SegmentKind
    assert SegmentKind.POST_TOUCHDOWN_ROLL in SegmentKind
    assert SegmentKind.WHEEL_ROLL.is_rolling
    assert SegmentKind.POST_TOUCHDOWN_ROLL.is_rolling


def test_every_kind_is_either_a_swing_or_a_roll():
    for kind in SegmentKind:
        assert kind.is_swing != kind.is_rolling


def test_the_terrain_transition_swing_kinds_are_kept_apart():
    """Their target semantics differ (spec 5.4), so one SWING kind would lose it.

    Day 12 added two more swings.  ``RECOVERY_SWING`` does not weaken the
    claim: it is not a terrain-transition swing at all, and
    ``is_terrain_transition`` is what keeps it out.  ``TOP_REPOSITION_SWING``
    (Step 7) *is* one, and is listed here rather than folded into
    ``SWING_OVER``: that kind crosses the whole obstacle in one primitive,
    while this one relocates on top of it because two primitives would not
    chain.  The assertion is over the predicate rather than over ``is_swing``
    so that it keeps testing what it was written to test.
    """

    transitions = {k for k in SegmentKind if k.is_swing and k.is_terrain_transition}
    assert transitions == {
        SegmentKind.SWING_UP, SegmentKind.SWING_DOWN, SegmentKind.SWING_OVER,
        SegmentKind.TOP_REPOSITION_SWING,
    }
    assert SegmentKind.RECOVERY_SWING.is_swing
    assert not SegmentKind.RECOVERY_SWING.is_terrain_transition


# --------------------------------------------------------------------------
# What spec 2.5 said could not be expressed
# --------------------------------------------------------------------------


def _rolling(alpha_from, alpha_to, beta_from, beta_to) -> RollingContact2D:
    return RollingContact2D(
        rim="left_rim", surface_ids=("day6_7_obstacle_top",),
        alpha_range_rad=(np.deg2rad(alpha_from), np.deg2rad(alpha_to)),
        beta_range_rad=(np.deg2rad(beta_from), np.deg2rad(beta_to)),
        theta_range_rad=(np.deg2rad(17.0), np.deg2rad(17.0)),
        contact_start_xz_m=(0.45, 0.10), contact_end_xz_m=(0.45, 0.10),
    )


def test_a_moving_contact_is_a_range_not_a_point():
    """Spec 2.5's first gap: one rim plus one alpha cannot describe wheel mode."""

    rolling = _rolling(-179.4, -134.5, -225.6, -180.6)
    assert rolling.mode is RollingMode.SURFACE_ROLL
    assert not rolling.is_static


def test_a_corner_pivot_is_distinguished_from_a_roll():
    """The case ``alpha_range`` alone would record as a static pose.

    Step 10R spends 72 frames pinned at ``alpha = -134.5 deg`` on the trailing
    corner while ``beta`` sweeps 71 degrees.  Without the beta range the schema
    describes that as one frozen pose.
    """

    pivot = _rolling(-134.5, -134.5, -297.6, -226.6)
    assert pivot.mode is RollingMode.CORNER_PIVOT
    assert not pivot.is_static
    assert pivot.beta_sweep_rad == pytest.approx(np.deg2rad(71.0))


def test_a_segment_where_nothing_moves_is_flagged():
    assert _rolling(-134.5, -134.5, -226.6, -226.6).is_static


def test_a_swing_must_record_the_knobs_no_contact_state_mentions():
    """Spec 2.5's second gap: liftoff / touchdown / duration / clearance."""

    shaping = SwingShaping2D(
        apex_clearance_m=0.03, liftoff_rise_m=0.03, touchdown_drop_m=0.02,
        duration_scale=1.5, mid_fractions=(0.35, 0.65),
    )
    row = shaping.as_dict()
    assert row["liftoff_rise_mm"] == pytest.approx(30.0)
    assert row["touchdown_drop_mm"] == pytest.approx(20.0)
    assert row["apex_clearance_mm"] == pytest.approx(30.0)
    assert row["duration_scale"] == pytest.approx(1.5)


# --------------------------------------------------------------------------
# Sampling: the completion criterion
# --------------------------------------------------------------------------


def test_no_sampling_parameter_has_a_default():
    """Omitting one must be a construction error, not a silent fallback."""

    with pytest.raises(TypeError):
        SwingSampling2D(arc_samples=121, sample_count=31, leg_arc_samples=61)
    with pytest.raises(TypeError):
        SwingSampling2D(arc_samples=121, sample_count=31, max_joint_step_rad=0.17)
    with pytest.raises(TypeError):
        RollSampling2D(arc_samples=241, beta_step_rad=-0.017)


def test_the_joint_step_limit_is_meaningless_without_the_sample_count():
    """Day 8--9 Step 9: ``max_joint_step_rad`` is a **per-sample** limit.

    The same trajectory sampled twice as densely has half the step, so the two
    fields only mean something together.  Two segments with different pairs and
    the same product describe the same limit -- which is why recording one of
    them would not pin the case down.
    """

    coarse = SwingSampling2D(arc_samples=121, sample_count=16,
                             leg_arc_samples=61, max_joint_step_rad=0.20)
    fine = SwingSampling2D(arc_samples=121, sample_count=31,
                           leg_arc_samples=61, max_joint_step_rad=0.10)
    assert coarse.max_joint_step_rad != fine.max_joint_step_rad
    assert coarse.implied_joint_speed_limit_rad == pytest.approx(
        fine.implied_joint_speed_limit_rad
    )


def test_sample_count_changes_the_trajectory_it_describes():
    """A demonstration, not an assertion: the field is load-bearing.

    Two requests identical except in ``sample_count`` produce different joint
    steps, so a schema that dropped it could not tell the two apart.
    """

    from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
        flat_to_flat_swing_request_2d,
    )
    from hybrid_note.scripts.experiments.cartesian_swing_planner_2d import (
        generate_swing_2d,
    )

    steps = []
    for sample_count in (16, 31):
        request = flat_to_flat_swing_request_2d(
            step_length_m=0.20, sample_count=sample_count, arc_samples=121
        )
        plan = generate_swing_2d(request, arc_samples=61)
        observed = [
            s.joint_step_rad for s in plan.result.samples
            if s.joint_step_rad is not None
        ]
        steps.append(max(observed))
    assert steps[0] > steps[1] * 1.4


def test_arc_samples_changes_what_counts_as_a_rim_seam():
    """Day 6--7 trap 2: the seam width is a sampling artefact, not geometry.

    ``seam_bridge_for_sampling_m`` exists precisely because of this, and it
    moves seven-fold across the densities in use.  A sequence that recorded a
    rim hand-over without its ``arc_samples`` would not say whether that
    hand-over was legal.
    """

    from hybrid_note.scripts.experiments.right_up_left_down_sweep_2d import (
        seam_bridge_for_sampling_m,
    )

    coarse = seam_bridge_for_sampling_m(61)
    fine = seam_bridge_for_sampling_m(481)
    assert coarse > fine * 5.0


def test_leg_arc_samples_changes_the_clearance_the_segment_reports():
    """The collision sweep's density decides the number, so it is part of it.

    The shift is small -- micrometres on this swing -- but spec 6.2's whole
    point is that the clearances in play are millimetres and Step 5 measured a
    margin floor moving 58 cells.  A recorded clearance without its density is
    not reproducible, whatever its size.
    """

    from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
        flat_to_flat_swing_request_2d,
    )
    from hybrid_note.scripts.experiments.cartesian_swing_planner_2d import (
        generate_swing_2d,
    )

    request = flat_to_flat_swing_request_2d(
        step_length_m=0.20, sample_count=31, arc_samples=121
    )
    clearances = []
    for leg_arc_samples in (31, 241):
        plan = generate_swing_2d(request, arc_samples=leg_arc_samples)
        assert plan.valid
        clearances.append(plan.collision.minimum_clearance_m)
    assert clearances[0] != pytest.approx(clearances[1], abs=1e-9)


# --------------------------------------------------------------------------
# Body requirement
# --------------------------------------------------------------------------


def test_the_body_requirement_reuses_step_1s_kinds():
    """A second enum would break the link from Step 5's decision to Step 9."""

    requirement = BodyRequirement2D(
        kind=BodyRequirementKind.LOWER_BOUND, x_range_m=(0.0, 0.3),
        hip_z_min_m=0.28,
    )
    assert requirement.kind is BodyRequirementKind.LOWER_BOUND


def test_a_track_requirement_without_a_profile_is_refused():
    with pytest.raises(ValueError):
        BodyRequirement2D(kind=BodyRequirementKind.TRACK, x_range_m=(0.0, 0.3))


def test_a_bound_requirement_without_a_height_is_refused():
    with pytest.raises(ValueError):
        BodyRequirement2D(kind=BodyRequirementKind.LOWER_BOUND, x_range_m=(0.0, 0.3))


def test_a_single_frame_phase_may_carry_a_single_sample_profile():
    """Step 10R's ``RIGHT_RIM_FRONT_CONTACT`` is one frame and a real phase."""

    requirement = BodyRequirement2D(
        kind=BodyRequirementKind.TRACK, x_range_m=(0.1, 0.1),
        hip_z_profile_m=[0.2196],
    )
    assert requirement.hip_z_travel_m == pytest.approx(0.0)


# --------------------------------------------------------------------------
# Frames by reference
# --------------------------------------------------------------------------


def test_frames_are_referenced_by_explicit_index_not_by_a_range():
    """Step 10R's own output skips index 11 -- a frame it proposed and rejected.

    A ``(start, stop)`` pair would silently claim it.
    """

    reference = FrameRef2D(source_id="s", indices=(9, 10, 12, 13))
    assert reference.frame_count == 4
    assert not reference.is_contiguous


def test_frame_indices_must_be_ascending_and_unique():
    with pytest.raises(ValueError):
        FrameRef2D(source_id="s", indices=(3, 1))
    with pytest.raises(ValueError):
        FrameRef2D(source_id="s", indices=(1, 1))


# --------------------------------------------------------------------------
# Segment and sequence invariants
# --------------------------------------------------------------------------


def _point(alpha_deg=0.0) -> PointContact2D:
    return PointContact2D(
        rim="foot_rim", alpha_rad=np.deg2rad(alpha_deg),
        point_world_xz_m=(0.0, 0.0), surface_id="ground",
        theta_rad=np.deg2rad(60.0), beta_rad=0.0, hip_xz_m=(0.0, 0.2194),
    )


def _roll_segment(indices=(0, 1)) -> MotionSegment2D:
    return MotionSegment2D(
        kind=SegmentKind.WHEEL_ROLL,
        start_contact=_point(), end_contact=_point(5.0),
        sampling=RollSampling2D(arc_samples=241, beta_step_rad=-0.017,
                                theta_step_rad=None),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.TRACK, x_range_m=(0.0, 0.1),
            hip_z_profile_m=[0.1438, 0.1438],
        ),
        frames=FrameRef2D(source_id="src", indices=indices),
        rolling=_rolling(-90.0, -85.0, -10.0, 0.0),
    )


def test_a_rolling_segment_must_say_what_moved_during_it():
    with pytest.raises(ValueError):
        replace(_roll_segment(), rolling=None)


def test_a_rolling_segment_refuses_swing_sampling():
    with pytest.raises(TypeError):
        replace(
            _roll_segment(),
            sampling=SwingSampling2D(arc_samples=121, sample_count=31,
                                     leg_arc_samples=61, max_joint_step_rad=0.17),
        )


def test_a_sequence_refuses_two_segments_claiming_the_same_frame():
    with pytest.raises(ValueError):
        MotionSequence2D(
            terrain_id="t",
            segments=(_roll_segment((0, 1)), _roll_segment((1, 2))),
        )


def test_a_sequence_refuses_segments_from_different_frame_sources():
    other = replace(
        _roll_segment((2, 3)),
        frames=FrameRef2D(source_id="elsewhere", indices=(2, 3)),
    )
    with pytest.raises(ValueError):
        MotionSequence2D(terrain_id="t", segments=(_roll_segment((0, 1)), other))


def test_a_sequence_with_any_untimed_segment_reports_no_total_duration():
    """A partial sum would read as a real time for the whole traversal."""

    sequence = MotionSequence2D(terrain_id="t", segments=(_roll_segment(),))
    assert sequence.total_duration_s is None


# --------------------------------------------------------------------------
# Unresolved transitions -- spec 5.6
# --------------------------------------------------------------------------


def _requirement() -> TransitionRequirement2D:
    return TransitionRequirement2D(
        kind=TransitionKind.TOP_REPOSITION,
        source_contact=_point(),
        target_condition="LEFT_RIM_READY",
        evidence="Step 2b: the direct hand-over crosses the alpha = -40 deg seam.",
    )


def test_a_requirement_carries_no_trajectory_and_no_duration():
    """Spec 5.6.  Generating one would publish an assumption as a result.

    A single-leg model cannot say who holds the body up while this leg is in
    the air, so a mid-air theta / beta / duration written here would be an
    unverified assumption wearing the clothes of a measurement.
    """

    requirement = _requirement()
    assert not hasattr(requirement, "duration_s")
    assert not hasattr(requirement, "frames")
    assert requirement.requires_external_support
    assert requirement.resolved is False


def test_a_resolved_requirement_is_refused_because_it_is_a_segment():
    """Flipping the flag is not how one gets solved; replacing it is."""

    with pytest.raises(ValueError):
        TransitionRequirement2D(
            kind=TransitionKind.TOP_REPOSITION, source_contact=_point(),
            target_condition="LEFT_RIM_READY", evidence="x", resolved=True,
        )


def test_a_requirement_must_say_what_the_next_segment_needs_and_why():
    for field in ("target_condition", "evidence"):
        with pytest.raises(ValueError):
            TransitionRequirement2D(**{
                "kind": TransitionKind.TOP_REPOSITION,
                "source_contact": _point(),
                "target_condition": "LEFT_RIM_READY",
                "evidence": "measured",
                field: "   ",
            })


def test_a_sequence_carrying_an_unresolved_transition_is_not_complete():
    """The distinction Step 9's hand-off file turns on.

    A reader that looked only at ``segments`` would treat this as executable.
    ``is_complete`` is what stops "unsolved" from being read as "absent".
    """

    plan = MotionSequence2D(terrain_id="t", segments=(_roll_segment(),))
    holed = MotionSequence2D(
        terrain_id="t", segments=(_roll_segment(),),
        unresolved=(_requirement(),),
    )
    assert plan.is_complete
    assert not holed.is_complete


def test_requirements_and_segments_share_one_table_but_stay_labelled():
    """A file that listed only the segments would read as complete."""

    sequence = MotionSequence2D(
        terrain_id="t", segments=(_roll_segment(),),
        unresolved=(_requirement(),),
    )
    kinds = [row["row_kind"] for row in sequence.rows()]
    assert kinds == ["segment", "unresolved_transition"]
    unresolved_row = sequence.rows()[-1]
    assert unresolved_row["transition_kind"] == "TOP_REPOSITION"
    assert unresolved_row["resolved"] is False


# --------------------------------------------------------------------------
# Round trip 1: Day 6--7 Step 10R
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def traversal():
    with DAY6_7.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    sequence = sequence_from_traversal_frames_2d(
        rows, terrain_id="day6_7_obstacle", source_id=DAY6_7.name, arc_samples=241
    )
    return rows, sequence


def test_the_full_traversal_round_trips_without_losing_a_frame(traversal):
    rows, sequence = traversal
    report = coverage_report_2d(sequence, rows)
    assert report.lossless
    assert report.missing == ()
    assert report.duplicated == ()
    assert report.covered_frame_count == report.source_frame_count


def test_the_wheel_mode_segment_survives(traversal):
    """The one spec 2.5 says the original schema could not represent."""

    _, sequence = traversal
    wheel = [s for s in sequence.segments if s.phase_label == "WHEEL_MODE_TOP_ROLL"]
    assert len(wheel) == 1
    rolling = wheel[0].rolling
    assert rolling.mode is RollingMode.SURFACE_ROLL
    assert rolling.contact_start_xz_m[0] != rolling.contact_end_xz_m[0]


def test_the_corner_pivot_is_recorded_as_motion_not_as_a_pose(traversal):
    _, sequence = traversal
    pivots = [
        s for s in sequence.segments
        if s.rolling is not None and s.rolling.mode is RollingMode.CORNER_PIVOT
        and s.frames.frame_count > 1
    ]
    assert {s.phase_label for s in pivots} == {
        "LEFT_RIM_TRAILING_TRANSITION", "LEFT_RIM_ROLL_DOWN"
    }
    assert all(abs(s.rolling.beta_sweep_rad) > np.deg2rad(1.0) for s in pivots)


def test_the_coupled_retract_records_its_theta_step(traversal):
    """``RETRACT_TO_WHEEL`` moves theta and beta together, 1 degree each."""

    _, sequence = traversal
    retract = next(
        s for s in sequence.segments if s.phase_label == "RETRACT_TO_WHEEL"
    )
    assert retract.sampling.theta_step_rad is not None
    assert np.rad2deg(retract.sampling.theta_step_rad) == pytest.approx(-1.0)


def test_a_pure_roll_records_no_theta_step(traversal):
    _, sequence = traversal
    wheel = next(
        s for s in sequence.segments if s.phase_label == "WHEEL_MODE_TOP_ROLL"
    )
    assert wheel.sampling.theta_step_rad is None


def test_the_traversal_carries_no_duration_because_it_was_never_timed(traversal):
    """Quasi-static.  Inventing a number here would hide a real gap."""

    _, sequence = traversal
    assert all(s.duration_s is None for s in sequence.segments)
    assert sequence.total_duration_s is None


def test_the_joints_never_teleport_across_a_hand_over(traversal):
    """Contact points jump by up to 180 mm; the leg must not.

    A rim transfer moves the contact to a different part of the wheel, so a
    large contact jump is expected.  What would be a real discontinuity is a
    joint jump, and the largest here is under two degrees.
    """

    _, sequence = traversal
    reports = handoff_report_2d(sequence)
    assert max(abs(r.theta_jump_rad) for r in reports) < np.deg2rad(1.5)
    assert max(abs(r.beta_jump_rad) for r in reports) < np.deg2rad(2.0)
    assert max(r.contact_jump_m for r in reports) > 0.15


def test_the_seam_crossing_is_a_change_of_chart_not_a_motion(traversal):
    """``alpha`` wraps at ``+-180 deg``; the contact point barely moves.

    Unwrapped this hand-over reads as a 359 degree jump, which is why the
    report folds the difference into ``(-pi, pi]``.
    """

    _, sequence = traversal
    seam = next(
        r for r in handoff_report_2d(sequence) if r.to_phase == "LEFT_RIM_READY"
    )
    assert seam.rim_changed
    assert abs(seam.alpha_jump_rad) < np.deg2rad(5.0)
    assert seam.contact_jump_m < 0.01
