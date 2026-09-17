"""Day 12 Step 0: the frozen segment semantics and the chain contract.

Step 0's acceptance is three questions, and the note is explicit that the first
of them must be answerable without reading prose:

    FOOT_RIM_ROLL can be constructed
    FOOT_RIM_ROLL != WHEEL_ROLL
    one segment's end state can be passed to the next segment

The last one is where Day 10--11's schema stops being enough, so it is tested
against **real Day 6--7 frames** rather than against hand-built fixtures: the
chain Day 12 needs crosses frame sources, and a synthetic test would not have
found that :class:`MotionSequence2D` forbids it.
"""

import csv
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_concession_2d import BodyRequirementKind
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    WHEEL_MODE_THETA_RAD,
    BodyRequirement2D,
    FrameRef2D,
    MotionSegment2D,
    MotionSequence2D,
    PointContact2D,
    RollSampling2D,
    RollingContact2D,
    SegmentKind,
    TransitionKind,
    TransitionRequirement2D,
)
from hybrid_note.scripts.experiments.day10_11_sequence_builders_2d import (
    sequence_from_traversal_frames_2d,
)
from hybrid_note.scripts.experiments.day12_segment_contract_2d import (
    SEGMENT_SEMANTICS,
    BoundaryKind,
    ChainTolerance2D,
    SegmentChain2D,
    boundary_kind_2d,
    boundary_rows_2d,
    chain_boundaries_2d,
    entry_state_2d,
    exit_state_2d,
    segment_semantics_rows,
)

DAY6_7 = (
    Path(__file__).resolve().parents[1]
    / "hybrid_note" / "notes" / "day6-7"
    / "day6_7_step10r_full_traversal_frames.csv"
)

#: Day 6--7's approach stands at this theta.  Quoted, not chosen: the point of
#: the tests below is that the nominal posture is whatever the project already
#: uses, and specifically that it is nowhere near wheel mode.
NOMINAL_EXPANDED_THETA_RAD = float(np.deg2rad(60.0))


# --------------------------------------------------------------------------
# Fixtures -- one flat foot-rim roll, built twice from the same numbers
# --------------------------------------------------------------------------


def _rolling(
    *, rim="foot_rim", theta_rad=NOMINAL_EXPANDED_THETA_RAD,
    alpha_from=0.0, alpha_to=18.0, x_from=-0.10, x_to=-0.05,
) -> RollingContact2D:
    return RollingContact2D(
        rim=rim,
        surface_ids=("ground",),
        alpha_range_rad=(np.deg2rad(alpha_from), np.deg2rad(alpha_to)),
        beta_range_rad=(np.deg2rad(-18.0), 0.0),
        theta_range_rad=(theta_rad, theta_rad),
        contact_start_xz_m=(x_from, 0.0),
        contact_end_xz_m=(x_to, 0.0),
    )


def _point(
    *, x, rim="foot_rim", alpha_deg=0.0, theta_rad=NOMINAL_EXPANDED_THETA_RAD,
    beta_deg=0.0, hip_x=None, hip_z=0.2194, surface="ground",
) -> PointContact2D:
    return PointContact2D(
        rim=rim,
        alpha_rad=float(np.deg2rad(alpha_deg)),
        point_world_xz_m=(x, 0.0),
        surface_id=surface,
        theta_rad=float(theta_rad),
        beta_rad=float(np.deg2rad(beta_deg)),
        hip_xz_m=(x if hip_x is None else hip_x, hip_z),
    )


def _roll_segment(
    kind=SegmentKind.FOOT_RIM_ROLL, *, source="day12_flat", indices=(0, 1, 2),
    theta_rad=NOMINAL_EXPANDED_THETA_RAD, rim="foot_rim",
    x_from=-0.10, x_to=-0.05, start=None, end=None,
) -> MotionSegment2D:
    return MotionSegment2D(
        kind=kind,
        phase_label=kind.value,
        start_contact=start or _point(x=x_from, rim=rim, theta_rad=theta_rad),
        end_contact=end or _point(
            x=x_to, rim=rim, theta_rad=theta_rad, alpha_deg=18.0, beta_deg=-18.0
        ),
        sampling=RollSampling2D(
            arc_samples=241, beta_step_rad=float(np.deg2rad(-1.0)), theta_step_rad=None
        ),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.TRACK,
            x_range_m=(x_from, x_to),
            hip_z_profile_m=np.full(len(indices), 0.2194),
        ),
        frames=FrameRef2D(source_id=source, indices=indices),
        rolling=_rolling(rim=rim, theta_rad=theta_rad, x_from=x_from, x_to=x_to),
    )


# --------------------------------------------------------------------------
# 1. FOOT_RIM_ROLL can be constructed
# --------------------------------------------------------------------------


def test_a_foot_rim_roll_segment_can_be_built_at_the_nominal_expanded_posture():
    segment = _roll_segment()
    assert segment.kind is SegmentKind.FOOT_RIM_ROLL
    assert segment.rolling.rim.value == "foot_rim"
    # The whole reason the kind exists: theta is the expanded posture, and the
    # schema does not object.
    assert segment.rolling.theta_range_rad[0] > WHEEL_MODE_THETA_RAD


def test_a_foot_rim_roll_must_actually_roll_on_the_foot_rim():
    """Otherwise the name is decorative and a consumer cannot trust it."""

    with pytest.raises(ValueError, match="FOOT_RIM_ROLL asserts the foot rim"):
        _roll_segment(rim="left_rim")


def test_rolling_on_a_landed_rim_has_its_own_kind():
    """The rejection above must not read as "Day 12 cannot roll on the left rim"."""

    segment = _roll_segment(kind=SegmentKind.POST_TOUCHDOWN_ROLL, rim="left_rim")
    assert segment.rolling.rim.value == "left_rim"


# --------------------------------------------------------------------------
# 2. FOOT_RIM_ROLL != WHEEL_ROLL
# --------------------------------------------------------------------------


def test_the_two_flat_running_kinds_are_distinct_members():
    assert SegmentKind.FOOT_RIM_ROLL is not SegmentKind.WHEEL_ROLL
    assert SegmentKind.FOOT_RIM_ROLL.value != SegmentKind.WHEEL_ROLL.value


def test_only_wheel_roll_pins_theta_and_the_nominal_cycle_is_two_kinds():
    """The freeze, as a property rather than as a docstring.

    ``nominal`` was ``{FOOT_RIM_ROLL}`` until the Day 12 plan's 2026-09-01
    FINAL FREEZE (§0.2) defined one nominal cycle as a rolling stance plus an
    airborne recovery.  The single-member version encoded the retracted claim
    that flat ground needs no swing, so it is the assertion that had to move.
    """

    pinned = {k for k in SegmentKind if k.pins_theta}
    nominal = {k for k in SegmentKind if k.is_nominal_locomotion}
    assert pinned == {SegmentKind.WHEEL_ROLL}
    assert nominal == {SegmentKind.FOOT_RIM_ROLL, SegmentKind.RECOVERY_SWING}
    assert not pinned & nominal
    # One rolling half, one airborne half.
    assert {k.is_swing for k in nominal} == {False, True}


def test_the_nominal_recovery_swing_is_not_a_terrain_transition_swing():
    """Day 12 plan section 18: the metrics must count the two apart, because
    the Hybrid-vs-Walk comparison turns on telling the swings the gait always
    makes from the ones the terrain forced."""

    swings = {k for k in SegmentKind if k.is_swing}
    forced = {k for k in swings if k.is_terrain_transition}
    assert swings - forced == {SegmentKind.RECOVERY_SWING}
    assert forced == {
        SegmentKind.SWING_UP, SegmentKind.SWING_DOWN, SegmentKind.SWING_OVER,
        # Step 7's reposition is forced by the terrain too: it exists only
        # because two transition primitives would not chain directly.
        SegmentKind.TOP_REPOSITION_SWING,
    }


def test_nominal_and_terrain_transition_partition_everything_but_wheel_mode():
    """No kind may be both, and none may be neither -- except the two that
    genuinely are neither.

    ``WHEEL_ROLL`` is neither the nominal cycle nor a terrain transition, and
    Day 13 added a second such kind: ``BODY_HOLD`` is a deliberate pause, so
    the gait did not make it and the terrain did not force it.  The exceptions
    are listed rather than the assertion weakened -- a kind that falls into
    neither bucket by accident is exactly what this test is for."""

    neither = {SegmentKind.WHEEL_ROLL, SegmentKind.BODY_HOLD}
    for kind in SegmentKind:
        assert not (kind.is_nominal_locomotion and kind.is_terrain_transition)
        if kind not in neither:
            assert kind.is_nominal_locomotion or kind.is_terrain_transition


def test_a_wheel_roll_at_an_expanded_posture_is_refused():
    """An expanded flat run mislabelled WHEEL_ROLL is the exact confusion the
    2026-08-31 freeze exists to stop -- so the schema refuses it."""

    with pytest.raises(ValueError, match="true wheel mode"):
        _roll_segment(kind=SegmentKind.WHEEL_ROLL)


def test_a_wheel_roll_at_seventeen_degrees_is_accepted():
    segment = _roll_segment(kind=SegmentKind.WHEEL_ROLL, theta_rad=WHEEL_MODE_THETA_RAD)
    assert segment.kind.pins_theta
    assert not segment.kind.is_nominal_locomotion


def test_wheel_mode_theta_comes_from_the_project_configuration():
    """Not a number written into Day 12."""

    from legwheel.config import RobotParams

    assert WHEEL_MODE_THETA_RAD == pytest.approx(np.deg2rad(RobotParams.THETA0_DEG))


def test_the_semantics_table_covers_every_kind():
    """A kind added later without a stated meaning is how a label drifts."""

    assert {k for k, _ in SEGMENT_SEMANTICS} == set(SegmentKind)
    rows = segment_semantics_rows()
    assert sum(r["is_nominal_locomotion"] for r in rows) == 2   # the cycle
    assert sum(r["pins_theta_to_wheel_mode"] for r in rows) == 1
    # Everything but the nominal cycle's two, WHEEL_ROLL and BODY_HOLD.
    assert sum(r["is_terrain_transition"] for r in rows) == len(SegmentKind) - 4


def test_as_dict_carries_the_distinction_into_the_csv():
    """Note section 1.1: the two must not be conflated in a CSV either."""

    foot = _roll_segment().as_dict()
    wheel = _roll_segment(
        kind=SegmentKind.WHEEL_ROLL, theta_rad=WHEEL_MODE_THETA_RAD
    ).as_dict()
    assert (foot["kind"], foot["is_nominal_locomotion"], foot["pins_theta"],
            foot["is_terrain_transition"]) == ("FOOT_RIM_ROLL", True, False, False)
    assert (wheel["kind"], wheel["is_nominal_locomotion"], wheel["pins_theta"],
            wheel["is_terrain_transition"]) == ("WHEEL_ROLL", False, True, False)


# --------------------------------------------------------------------------
# 3. One segment's end state can be passed to the next
# --------------------------------------------------------------------------


def test_the_endpoint_contract_is_a_definite_pose():
    segment = _roll_segment()
    assert exit_state_2d(segment) is segment.end_contact
    assert entry_state_2d(segment) is segment.start_contact


def test_a_non_definite_endpoint_is_refused_by_the_chain_contract():
    """The schema's union still allows it; the chain contract does not."""

    segment = _roll_segment()
    loose = replace(segment, end_contact=segment.rolling)
    with pytest.raises(TypeError, match="non-definite end state"):
        exit_state_2d(loose)


def test_an_exit_state_handed_straight_to_the_next_segment_chains():
    first = _roll_segment(indices=(0, 1, 2), x_from=-0.10, x_to=-0.05)
    second = _roll_segment(
        indices=(3, 4, 5), x_from=-0.05, x_to=0.0,
        start=exit_state_2d(first),
    )
    chain = SegmentChain2D(leg_id="LF", segments=(first, second))
    assert chain.is_chained
    assert chain.breaks == ()
    assert chain.entry_state is first.start_contact
    assert chain.exit_state is second.end_contact


def test_a_joint_jump_across_a_handover_breaks_the_chain():
    """29 deg is the alpha = -40 seam jump Step 2b measured -- the one that
    refuted strategy #3.  A chain checker that let it through would let Day 12
    schedule a leg that teleports."""

    first = _roll_segment(source="day12_flat", indices=(0, 1, 2))
    jumped = replace(
        exit_state_2d(first),
        beta_rad=float(exit_state_2d(first).beta_rad + np.deg2rad(29.0)),
    )
    second = _roll_segment(source="day6_7_step10r", indices=(0, 1, 2), start=jumped)
    assert boundary_kind_2d(first, second) is BoundaryKind.HANDOVER
    _, breaks = chain_boundaries_2d((first, second))
    assert [b.reason for b in breaks] == ["beta_jump"]
    assert np.rad2deg(breaks[0].value) == pytest.approx(29.0, abs=1e-6)


def test_the_same_jump_inside_one_source_is_a_cut_overrun_instead():
    """Same number, different question: within one run a 29 deg step is not a
    teleport, it is a cut that skipped 29 degrees of motion the sampling says
    should have taken 29 frames.  Both are failures; naming them the same
    would send Day 12 looking in the wrong place."""

    first = _roll_segment(source="day12_flat", indices=(0, 1, 2))
    jumped = replace(
        exit_state_2d(first),
        beta_rad=float(exit_state_2d(first).beta_rad + np.deg2rad(29.0)),
    )
    second = _roll_segment(source="day12_flat", indices=(3, 4, 5), start=jumped)
    assert boundary_kind_2d(first, second) is BoundaryKind.CUT
    _, breaks = chain_boundaries_2d((first, second))
    assert [b.reason for b in breaks] == ["beta_step_overrun"]
    # 1 deg nominal beta step, gap of 1 frame, 1.5x slack.
    assert np.rad2deg(breaks[0].limit) == pytest.approx(1.5)


def test_a_large_contact_jump_alone_does_not_break_the_chain():
    """Day 10--11 trap 32: the +-180 deg seam moves the contact point 162 mm
    while the joints move about a degree.  That is a change of chart, not a
    motion, and a checker that failed on it would reject the hand-over Day
    6--7's traversal is built around."""

    first = _roll_segment(source="day12_flat", indices=(0, 1, 2))
    far = replace(
        exit_state_2d(first),
        point_world_xz_m=exit_state_2d(first).point_world_xz_m + np.array([0.162, 0.0]),
    )
    second = _roll_segment(source="day6_7_step10r", indices=(0, 1, 2), start=far)
    reports, breaks = chain_boundaries_2d((first, second))
    assert breaks == []
    assert reports[0].contact_jump_m == pytest.approx(0.162)

    # ...and a caller that does want it bounded can say so.
    _, strict = chain_boundaries_2d(
        (first, second), ChainTolerance2D(max_contact_jump_m=0.01)
    )
    assert [b.reason for b in strict] == ["contact_jump"]


# --------------------------------------------------------------------------
# 4. Why the chain is a new container: the sequence forbids what Day 12 needs
# --------------------------------------------------------------------------


def test_a_motion_sequence_refuses_segments_from_two_sources():
    """The finding that made Step 0 more than an enum addition."""

    flat = _roll_segment(source="day12_flat", indices=(0, 1, 2))
    transition = _roll_segment(
        kind=SegmentKind.ROLL_UP, source="day6_7_step10r", indices=(0, 1, 2),
        start=exit_state_2d(flat),
    )
    with pytest.raises(ValueError, match="same frame source"):
        MotionSequence2D(terrain_id="t", segments=(flat, transition))


def test_a_chain_accepts_them_and_records_where_each_came_from():
    flat = _roll_segment(source="day12_flat", indices=(0, 1, 2))
    transition = _roll_segment(
        kind=SegmentKind.ROLL_UP, source="day6_7_step10r", indices=(0, 1, 2),
        start=exit_state_2d(flat),
    )
    chain = SegmentChain2D(leg_id="LF", segments=(flat, transition))
    assert chain.sources == ("day12_flat", "day6_7_step10r")
    assert chain.is_chained


def test_a_chain_still_refuses_a_frame_reused_within_one_source():
    """Relaxing the cross-source rule must not relax the one that still means
    something: inside a source, a frame belongs to exactly one segment."""

    first = _roll_segment(source="day12_flat", indices=(0, 1, 2))
    second = _roll_segment(
        source="day12_flat", indices=(2, 3, 4), start=exit_state_2d(first)
    )
    with pytest.raises(ValueError, match="overlap on frames"):
        SegmentChain2D(leg_id="LF", segments=(first, second))


# --------------------------------------------------------------------------
# 5. Completeness and time are separate questions
# --------------------------------------------------------------------------


def _top_reposition() -> TransitionRequirement2D:
    return TransitionRequirement2D(
        kind=TransitionKind.TOP_REPOSITION,
        source_contact=_point(x=0.30, surface="obstacle_top"),
        target_condition="LEFT_RIM_READY",
        evidence="Day 10-11 Step 9: unresolved without four-leg support.",
    )


def test_a_chain_can_be_continuous_and_still_have_a_hole_in_it():
    chain = SegmentChain2D(
        leg_id="LF", segments=(_roll_segment(),), unresolved=(_top_reposition(),)
    )
    assert chain.is_chained
    assert not chain.is_complete


def test_a_quasi_static_chain_reports_no_duration_rather_than_a_partial_sum():
    """Trap 33: Day 6--7's rolling segments were never assigned time.  Step 3
    has to supply one, and that is a new modelling decision."""

    chain = SegmentChain2D(
        leg_id="LF",
        segments=(_roll_segment(indices=(0, 1, 2)),
                  _roll_segment(indices=(3, 4, 5), start=None)),
    )
    assert chain.total_duration_s is None
    assert chain.untimed_segments == (0, 1)


# --------------------------------------------------------------------------
# 6. Against real data
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def day6_7_rows() -> list[dict]:
    with DAY6_7.open(newline="") as handle:
        return list(csv.DictReader(handle))


def test_day6_7_approach_is_already_a_foot_rim_roll(day6_7_rows):
    """Step 0's central finding: the motion exists, only the name was missing.

    The APPROACH segment rolls the foot rim along flat ground at the expanded
    posture.  Relabelling it FOOT_RIM_ROLL changes nothing the schema checks --
    which is the evidence that the new kind needs no new geometry.
    """

    sequence = sequence_from_traversal_frames_2d(
        day6_7_rows, terrain_id="day6_7_obstacle",
        source_id="day6_7_step10r_full_traversal_frames.csv", arc_samples=241,
    )
    approach = sequence.segments[0]
    assert approach.kind is SegmentKind.APPROACH
    assert approach.rolling.rim.value == "foot_rim"
    assert approach.rolling.surface_ids == ("ground",)
    theta = approach.rolling.theta_range_rad
    assert theta[0] == pytest.approx(theta[1])
    assert theta[0] > WHEEL_MODE_THETA_RAD + np.deg2rad(30.0)

    relabelled = replace(approach, kind=SegmentKind.FOOT_RIM_ROLL)
    assert relabelled.kind.is_nominal_locomotion
    assert relabelled.rolling == approach.rolling


def test_a_flat_run_chains_onto_the_real_day6_7_traversal(day6_7_rows):
    """The Day 12 shape end to end: nominal roll, then a transition from a
    different frame source, with the boundary measured rather than assumed."""

    sequence = sequence_from_traversal_frames_2d(
        day6_7_rows, terrain_id="day6_7_obstacle",
        source_id="day6_7_step10r_full_traversal_frames.csv", arc_samples=241,
    )
    approach = sequence.segments[0]
    entry = entry_state_2d(approach)
    flat = _roll_segment(
        source="day12_flat_run", indices=(0, 1, 2),
        x_from=float(entry.point_world_xz_m[0]) - 0.05,
        end=entry,
    )
    chain = SegmentChain2D(leg_id="LF", segments=(flat, *sequence.segments))

    assert chain.sources == (
        "day12_flat_run", "day6_7_step10r_full_traversal_frames.csv"
    )
    assert len(chain.handoffs) == len(sequence.segments)
    # The hand-over into the traversal is exact: it *is* the traversal's own
    # first state, handed over rather than re-derived.
    first = chain.handoffs[0]
    assert first.theta_jump_rad == pytest.approx(0.0)
    assert first.beta_jump_rad == pytest.approx(0.0)
    assert first.contact_jump_m == pytest.approx(0.0)


def test_the_traversals_own_boundaries_are_cuts_and_all_pass(day6_7_rows):
    """The regression that corrected this module.

    Every one of Step 10R's nine boundaries is a cut -- consecutive frames of
    one run -- and each moves the leg by about one sampling step: up to 1.75
    deg of beta and 6.62 mm of hip.  The first version of the contract held
    these to a fixed 2 mm hip bound and failed five of them, which was the
    contract being wrong about what a boundary is, not the traversal being
    discontinuous.
    """

    sequence = sequence_from_traversal_frames_2d(
        day6_7_rows, terrain_id="day6_7_obstacle",
        source_id="day6_7_step10r_full_traversal_frames.csv", arc_samples=241,
    )
    segments = sequence.segments
    kinds = {
        boundary_kind_2d(b, a) for b, a in zip(segments, segments[1:])
    }
    assert kinds == {BoundaryKind.CUT}

    reports, breaks = chain_boundaries_2d(segments)
    assert breaks == []
    assert max(abs(r.theta_jump_rad) for r in reports) == pytest.approx(
        np.deg2rad(1.0)
    )
    assert max(abs(r.beta_jump_rad) for r in reports) == pytest.approx(
        np.deg2rad(1.75)
    )
    # The hip moves further than any fixed "continuity" bound would allow,
    # because it is moving, not jumping.
    assert max(r.hip_jump_m for r in reports) * 1e3 == pytest.approx(6.62, abs=0.01)
    # And the contact point moves further still, across the rim seams.
    assert max(r.contact_jump_m for r in reports) > 0.15


def test_the_boundary_table_labels_cuts_and_handovers_apart(day6_7_rows):
    sequence = sequence_from_traversal_frames_2d(
        day6_7_rows, terrain_id="day6_7_obstacle",
        source_id="day6_7_step10r_full_traversal_frames.csv", arc_samples=241,
    )
    entry = entry_state_2d(sequence.segments[0])
    flat = _roll_segment(
        source="day12_flat_run", indices=(0, 1, 2),
        x_from=float(entry.point_world_xz_m[0]) - 0.05, end=entry,
    )
    rows = boundary_rows_2d((flat, *sequence.segments))
    assert rows[0]["boundary_kind"] == "handover"
    assert {r["boundary_kind"] for r in rows[1:]} == {"cut"}
    assert all(r["continuous"] for r in rows)
