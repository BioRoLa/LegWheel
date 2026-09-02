"""Day 10--11 Step 9: the body-requirement timeline's contract.

Cheap tests only.  The delivered file itself is produced by
``day10_11_step9_driver.py``, which has to compose a full rolling traversal to
make it.  What is pinned here is what Step 9 decided:

    x is the independent variable, and rolling time stays missing
    a swing's endpoints are hard even though its interior is not
    the segment-level scalar is an envelope, not a timeline
    an unresolved transition keeps its own row and never fills in a pose
"""

import numpy as np
import pytest

from legwheel.planners.hybrid import RimId

from hybrid_note.scripts.experiments.day10_11_body_timeline_2d import (
    NOT_GENERATED,
    TIME_BASIS,
    TIME_BASIS_STATEMENT,
    UPPER_BOUND_STATUS,
    BodyTimeline2D,
    ConstraintClass,
    ContactPhase,
    RequirementBasis,
    constraint_class_of,
    provenance_rows_2d,
    reader_check_2d,
    timeline_from_composed_2d,
    timeline_rows_2d,
    union_rows,
)
from hybrid_note.scripts.experiments.day10_11_composer_2d import ComposedSequence2D
from hybrid_note.scripts.experiments.day10_11_concession_2d import (
    BodyRequirementKind,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    StrategyId,
    Verdict,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirement2D,
    FrameRef2D,
    MotionSegment2D,
    MotionSequence2D,
    PointContact2D,
    RollSampling2D,
    RollingContact2D,
    SegmentKind,
    SwingSampling2D,
    SwingShaping2D,
    TransitionKind,
    TransitionRequirement2D,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import write_rows_csv


# --------------------------------------------------------------------------
# Fixtures: a two-segment sequence built by hand, so nothing runs a planner
# --------------------------------------------------------------------------


def _contact(x: float, z: float, theta_deg: float = 60.0) -> PointContact2D:
    return PointContact2D(
        rim=RimId.FOOT, alpha_rad=0.0,
        point_world_xz_m=np.array([x, z]), surface_id="ground",
        theta_rad=float(np.deg2rad(theta_deg)), beta_rad=0.0,
        hip_xz_m=np.array([x, z + 0.22]),
    )


def _frame(index: int, x: float, hip_z: float, *, time_s=None) -> dict:
    row = {
        "index": index, "stage": "S", "phase": "S",
        "theta_deg": 60.0, "beta_deg": 0.0,
        "hip_x_m": x, "hip_z_m": hip_z,
        "active_rim": "foot_rim", "alpha_deg": 0.0,
        "contact_x_m": x, "contact_z_m": 0.0,
        "contact_surface": "ground", "accepted": "True",
    }
    if time_s is not None:
        row["time_s"] = time_s
    return row


def _rolling_segment(indices, hip_z) -> MotionSegment2D:
    return MotionSegment2D(
        kind=SegmentKind.ROLL_UP, phase_label="ROLL_UP",
        start_contact=_contact(0.0, 0.0), end_contact=_contact(0.1, 0.0),
        sampling=RollSampling2D(arc_samples=121,
                                beta_step_rad=float(np.deg2rad(-1.0)),
                                theta_step_rad=None),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.TRACK, x_range_m=(0.0, 0.1),
            hip_z_profile_m=np.asarray(hip_z, dtype=float),
        ),
        frames=FrameRef2D(source_id="unit", indices=tuple(indices)),
        duration_s=None,
        rolling=RollingContact2D(
            rim=RimId.FOOT, surface_ids=("ground",),
            alpha_range_rad=(0.0, 0.1), beta_range_rad=(-0.1, 0.0),
            theta_range_rad=(1.0, 1.1),
            contact_start_xz_m=np.array([0.0, 0.0]),
            contact_end_xz_m=np.array([0.1, 0.0]),
        ),
    )


def _swing_segment(indices, hip_z_min) -> MotionSegment2D:
    return MotionSegment2D(
        kind=SegmentKind.SWING_UP, phase_label="SWING_UP",
        start_contact=_contact(0.1, 0.0), end_contact=_contact(0.3, 0.08),
        sampling=SwingSampling2D(arc_samples=121, sample_count=len(indices),
                                 leg_arc_samples=61,
                                 max_joint_step_rad=float(np.deg2rad(5.0))),
        body_requirement=BodyRequirement2D(
            kind=BodyRequirementKind.LOWER_BOUND, x_range_m=(0.1, 0.3),
            hip_z_min_m=float(hip_z_min),
        ),
        frames=FrameRef2D(source_id="unit", indices=tuple(indices)),
        duration_s=0.6,
        swing_shaping=SwingShaping2D(apex_clearance_m=0.03, liftoff_rise_m=0.0,
                                     touchdown_drop_m=0.0, duration_scale=1.0,
                                     mid_fractions=(0.35, 0.65)),
    )


def _composed(*, unresolved=(), gap_on_top_m=None) -> ComposedSequence2D:
    """Two segments: a rolling stage that tracks, then a swing that rises."""

    rolling_z = [0.220, 0.221, 0.222]
    swing_z = [0.222, 0.262, 0.302]
    frames = (
        [_frame(0, 0.00, rolling_z[0]), _frame(1, 0.05, rolling_z[1]),
         _frame(2, 0.10, rolling_z[2])]
        + [_frame(3, 0.10, swing_z[0], time_s=0.0),
           _frame(4, 0.20, swing_z[1], time_s=0.3),
           _frame(5, 0.30, swing_z[2], time_s=0.6)]
    )
    sequence = MotionSequence2D(
        terrain_id="unit",
        segments=(_rolling_segment((0, 1, 2), rolling_z),
                  _swing_segment((3, 4, 5), max(swing_z))),
        unresolved=tuple(unresolved),
    )
    parameters = ()
    if gap_on_top_m is not None:
        parameters = (("gap_on_top_m", float(gap_on_top_m)),)
    return ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.080, top_length_m=0.240,
        sequence=sequence, refusal=None, parameters=parameters,
        collision_free=True, min_clearance_m=0.0031,
        verdict=Verdict.COMPOSED, frame_rows=tuple(frames),
    )


# --------------------------------------------------------------------------
# x is the independent variable, and rolling time stays missing
# --------------------------------------------------------------------------


def test_the_time_basis_is_x_and_the_file_says_why():
    assert TIME_BASIS == "x_progress"
    # The completion criterion is that the *file* states it, not the module.
    notes = " ".join(row["notes"] for row in provenance_rows_2d())
    assert TIME_BASIS_STATEMENT in notes
    assert "NEW MODELLING DECISION" in notes


def test_rolling_knots_carry_no_time_and_swing_knots_keep_theirs():
    timeline = timeline_from_composed_2d(_composed())
    rolling = [k for k in timeline.knots if not k.segment_kind.is_swing]
    swing = [k for k in timeline.knots if k.segment_kind.is_swing]
    assert rolling and swing
    # Day 6--7 never assigned time; a zero here would be an invented number.
    assert all(k.time_s is None for k in rolling)
    assert [k.time_s for k in swing] == [0.0, 0.3, 0.6]
    assert all(s.duration_s is None for s in timeline.segments
               if not s.kind.is_swing)


def test_x_monotonicity_is_checked_rather_than_assumed():
    timeline = timeline_from_composed_2d(_composed())
    assert timeline.x_is_monotonic
    backwards = timeline.knots[:2] + (timeline.knots[0],)
    assert not BodyTimeline2D(
        sequence_id="x", strategy=StrategyId.SWING_SWING, height_m=0.08,
        top_length_m=0.24, verdict=Verdict.COMPOSED, knots=backwards,
    ).x_is_monotonic


# --------------------------------------------------------------------------
# Hard versus preference
# --------------------------------------------------------------------------


def test_track_and_pinned_are_hard_and_lower_bound_is_a_preference():
    assert constraint_class_of(BodyRequirementKind.TRACK) is ConstraintClass.HARD
    assert constraint_class_of(BodyRequirementKind.PINNED) is ConstraintClass.HARD
    assert (constraint_class_of(BodyRequirementKind.LOWER_BOUND)
            is ConstraintClass.PREFERENCE)


def test_a_swings_endpoints_are_pinned_and_only_its_interior_is_a_preference():
    """Trap 16: touchdown theta is the IK's output, so the hip is not free."""

    timeline = timeline_from_composed_2d(_composed())
    swing = [k for k in timeline.knots if k.segment_kind.is_swing]
    assert [k.body_kind for k in swing] == [
        BodyRequirementKind.PINNED,
        BodyRequirementKind.LOWER_BOUND,
        BodyRequirementKind.PINNED,
    ]
    assert swing[0].basis is RequirementBasis.PINNED_CONTACT
    assert swing[1].basis is RequirementBasis.WITNESS_LOWER_BOUND
    # And the endpoints are the two instants the foot is actually down.
    assert [k.contact_phase for k in swing] == [
        ContactPhase.STANCE, ContactPhase.FLIGHT, ContactPhase.STANCE
    ]


def test_every_rolling_knot_is_an_exact_track():
    timeline = timeline_from_composed_2d(_composed())
    rolling = [k for k in timeline.knots if not k.segment_kind.is_swing]
    assert all(k.basis is RequirementBasis.EXACT_TRACK for k in rolling)
    assert all(k.constraint_class is ConstraintClass.HARD for k in rolling)
    assert all(k.contact_phase is ContactPhase.STANCE for k in rolling)


def test_the_upper_bound_is_recorded_as_not_measured_not_as_blank():
    knot = timeline_from_composed_2d(_composed()).knots[0].as_dict()
    assert knot["hip_z_upper_bound_mm"] == UPPER_BOUND_STATUS == "NOT_MEASURED"


# --------------------------------------------------------------------------
# The envelope is not the timeline
# --------------------------------------------------------------------------


def test_a_rolling_segment_has_no_scalar_envelope_at_all():
    """A ``TRACK`` requirement is the profile; ``max(profile)`` is not an
    envelope Step 6 ever claimed, and reporting one would invent an
    over-constraint that nobody imposed."""

    timeline = timeline_from_composed_2d(_composed())
    rolling = next(s for s in timeline.segments if not s.kind.is_swing)
    assert rolling.envelope_hip_z_m is None
    assert rolling.max_envelope_excess_m == 0.0
    assert rolling.as_dict()["segment_envelope_hip_z_mm"] == ""
    assert all(k.envelope_excess_m == 0.0 for k in timeline.knots
               if not k.segment_kind.is_swing)


def test_the_segment_scalar_over_constrains_the_body_and_the_amount_is_reported():
    timeline = timeline_from_composed_2d(_composed())
    swing = next(s for s in timeline.segments if s.kind.is_swing)
    # The scalar is the maximum of the planned hip trajectory (Step 6's
    # builder); at the lift-off end the real requirement is 80 mm lower.
    assert swing.envelope_hip_z_m == pytest.approx(0.302)
    assert swing.hip_z_low_m == pytest.approx(0.222)
    assert swing.max_envelope_excess_m == pytest.approx(0.080)
    # And the envelope is kept, because it is the number Step 5 decided on.
    assert swing.as_dict()["segment_envelope_hip_z_mm"] == pytest.approx(302.0)


def test_a_knots_requirement_is_its_own_height_not_the_segment_scalar():
    timeline = timeline_from_composed_2d(_composed())
    swing = [k for k in timeline.knots if k.segment_kind.is_swing]
    assert [k.hip_z_m for k in swing] == pytest.approx([0.222, 0.262, 0.302])
    assert [k.envelope_excess_m for k in swing] == pytest.approx([0.08, 0.04, 0.0])


# --------------------------------------------------------------------------
# Unsolved is not absent
# --------------------------------------------------------------------------


def _requirement() -> TransitionRequirement2D:
    return TransitionRequirement2D(
        kind=TransitionKind.TOP_REPOSITION,
        source_contact=_contact(0.2, 0.08),
        target_condition="LEFT_RIM_READY",
        evidence="Step 2b: the alpha = -40 deg seam.",
    )


def test_a_timeline_with_no_knots_writes_no_vacuous_checks():
    """``x is monotonic`` over an empty list is true and means nothing; a
    reader would take it as "checked, and fine"."""

    blocked = ComposedSequence2D(
        strategy=StrategyId.SWING_ROLL, height_m=0.140, top_length_m=0.225,
        sequence=None, refusal="will not chain",
        verdict=Verdict.DIRECT_HANDOFF_INFEASIBLE,
        unresolved=(_requirement(),),
    )
    head = timeline_from_composed_2d(blocked).rows()[0]
    assert head["x_is_monotonic"] == ""
    assert head["max_envelope_excess_mm"] == ""


def test_a_rolling_sequence_reports_no_envelope_rather_than_a_zero_one():
    timeline = timeline_from_composed_2d(_composed())
    rolling_only = BodyTimeline2D(
        sequence_id=timeline.sequence_id, strategy=StrategyId.ROLL_ROLL,
        height_m=0.14, top_length_m=0.225, verdict=Verdict.COMPOSED,
        segments=tuple(s for s in timeline.segments if not s.kind.is_swing),
        knots=tuple(k for k in timeline.knots if not k.segment_kind.is_swing),
    )
    assert rolling_only.rows()[0]["max_envelope_excess_mm"] == ""


def test_a_blocked_pair_becomes_a_timeline_with_no_knots_and_its_requirement():
    blocked = ComposedSequence2D(
        strategy=StrategyId.SWING_ROLL, height_m=0.140, top_length_m=0.225,
        sequence=None, refusal="will not chain",
        verdict=Verdict.DIRECT_HANDOFF_INFEASIBLE,
        unresolved=(_requirement(),),
    )
    timeline = timeline_from_composed_2d(blocked)
    assert timeline.knots == ()
    assert len(timeline.unresolved) == 1
    assert not timeline.is_executable
    assert timeline.verdict is Verdict.DIRECT_HANDOFF_INFEASIBLE


def test_an_unresolved_row_never_fills_in_a_pose_or_a_duration():
    blocked = ComposedSequence2D(
        strategy=StrategyId.SWING_ROLL, height_m=0.140, top_length_m=0.225,
        sequence=None, refusal="will not chain",
        verdict=Verdict.DIRECT_HANDOFF_INFEASIBLE,
        unresolved=(_requirement(),),
    )
    row = next(r for r in timeline_from_composed_2d(blocked).rows()
               if r["row_kind"] == "unresolved_transition")
    for column in ("theta_deg", "beta_deg", "segment_duration_s",
                   "hip_z_required_mm"):
        assert row[column] == NOT_GENERATED
    assert row["resolved"] is False
    assert row["constraint_class"] == ConstraintClass.UNRESOLVED.value
    # The route back travels with the refusal, as Step 9's prelude decided.
    assert row["multileg_route"]


def test_segments_and_unresolved_requirements_share_one_table():
    timeline = timeline_from_composed_2d(_composed(unresolved=(_requirement(),)))
    kinds = [r["row_kind"] for r in timeline.rows()]
    assert "segment" in kinds and "unresolved_transition" in kinds
    # A sequence carrying one is not executable however many segments it has.
    assert not timeline.is_executable


def test_an_uncovered_stretch_of_path_gets_its_own_row():
    timeline = timeline_from_composed_2d(_composed(gap_on_top_m=0.110))
    assert len(timeline.gaps) == 1
    assert timeline.gaps[0].length_m == pytest.approx(0.110)
    assert not timeline.is_executable
    row = next(r for r in timeline.rows() if r["row_kind"] == "unplanned_gap")
    assert row["constraint_class"] == ConstraintClass.UNRESOLVED.value


def test_a_gap_below_a_millimetre_is_rounding_not_a_hole():
    # Step 7's #4 cell has gap_on_top_m = -5.6e-17.
    assert timeline_from_composed_2d(_composed(gap_on_top_m=-5.6e-17)).gaps == ()


# --------------------------------------------------------------------------
# The file stands on its own
# --------------------------------------------------------------------------


def test_the_delivered_file_passes_a_reader_that_imports_nothing(tmp_path):
    path = tmp_path / "timeline.csv"
    write_rows_csv(path, timeline_rows_2d([timeline_from_composed_2d(_composed())]))
    check = reader_check_2d(path)
    assert check.passed, check.problems
    assert check.sequences == 1
    assert check.knots == 6
    assert check.hard_knots == 5      # 3 rolling + 2 swing endpoints
    assert check.preference_knots == 1


def test_the_reader_rejects_a_file_that_lost_its_provenance(tmp_path):
    rows = timeline_rows_2d([timeline_from_composed_2d(_composed())])
    stripped = [r for r in rows if r["row_kind"] != "provenance"]
    path = tmp_path / "no_provenance.csv"
    write_rows_csv(path, stripped)
    check = reader_check_2d(path)
    assert not check.passed
    assert any("time basis" in problem for problem in check.problems)


def test_the_reader_rejects_a_header_that_disagrees_with_its_rows(tmp_path):
    rows = timeline_rows_2d([timeline_from_composed_2d(_composed())])
    trimmed = [r for r in rows if not (r["row_kind"] == "knot"
                                       and r["frame_index"] == 4)]
    path = tmp_path / "short.csv"
    write_rows_csv(path, trimmed)
    check = reader_check_2d(path)
    assert not check.passed
    assert any("knots" in problem for problem in check.problems)


def test_no_row_is_entitled_to_physically_infeasible(tmp_path):
    rows = timeline_rows_2d([timeline_from_composed_2d(_composed())])
    rows[-1] = dict(rows[-1], verdict="PHYSICALLY_INFEASIBLE")
    path = tmp_path / "overclaimed.csv"
    write_rows_csv(path, union_rows(rows))
    assert any("PHYSICALLY_INFEASIBLE" in problem
               for problem in reader_check_2d(path).problems)


def test_union_rows_pads_every_column(tmp_path):
    padded = union_rows([{"a": 1}, {"b": 2}])
    assert set(padded[0]) == set(padded[1]) == {"a", "b"}
