"""Day 12 Step 7: resolving ``TOP_REPOSITION`` with the four legs in hand.

Plan §14.  The expensive part is the swing planner, so the ordering claims --
support first, then motion, then touchdown -- are tested on the cheap paths,
and the two real cases are run once each in a module fixture.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_composer_2d import (
    ComposedSequence2D,
    top_reposition_requirement_2d,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    BLOCKED_PAIRS,
    StrategyId,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    RimId,
    SegmentKind,
    TransitionKind,
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
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    DEFAULT_MARGIN_FLOOR_M,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import walk_timing_2d
from hybrid_note.scripts.experiments.day12_top_reposition_2d import (
    REPOSITION_TARGETS,
    TARGET_THETA_HEADROOM_RAD,
    RepositionOutcome,
    reposition_rows,
    resolve_top_reposition_2d,
    support_gate_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    build_leg_plan_2d,
    plan_four_legs_2d,
)

#: Coarser than the driver's 0.5 deg.  The LEFT_RIM_READY sweep builds one full
#: leg per beta and costs about ten minutes at 0.5 deg; nothing tested here
#: turns on that resolution -- the claims are "a left-rim landing exists" and
#: "the direct swing cannot reach it" -- so the tests say which grid they used
#: and use a cheaper one.
TEST_BETA_STEP_DEG = 2.0

#: Day 10--11 Step 9's own cells for the two blocked pairs.
BLOCKED_CELLS = {
    StrategyId.ROLL_SWING: (0.160, 0.350),
    StrategyId.SWING_ROLL: (0.140, 0.225),
}


@pytest.fixture(scope="module")
def context():
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
    return four, trajectory


# --------------------------------------------------------------------------
# The new segment kind is a kind, not a relabelled neighbour
# --------------------------------------------------------------------------


def test_the_reposition_swing_is_its_own_kind():
    """Folding it into SWING_OVER or RECOVERY_SWING would lose what it is."""

    kind = SegmentKind.TOP_REPOSITION_SWING
    assert kind.is_swing
    assert kind.is_terrain_transition, "the terrain is why it is in the plan"
    assert not kind.is_nominal_locomotion, "the gait does not always make it"
    assert kind is not SegmentKind.SWING_OVER


# --------------------------------------------------------------------------
# The target condition, made checkable (requirement 6)
# --------------------------------------------------------------------------


def test_both_blocked_pairs_have_a_checkable_target():
    assert set(REPOSITION_TARGETS) == set(BLOCKED_PAIRS)


def test_the_two_targets_are_genuinely_different_landings():
    """``#2`` wants the foot rim high; ``#3`` wants LEFT_RIM_READY."""

    roll_swing = REPOSITION_TARGETS[StrategyId.ROLL_SWING]
    swing_roll = REPOSITION_TARGETS[StrategyId.SWING_ROLL]
    assert roll_swing.rim is RimId.FOOT
    assert swing_roll.rim is RimId.LEFT
    assert roll_swing.theta_min_rad == pytest.approx(np.deg2rad(35.0))
    assert swing_roll.theta_min_rad < np.deg2rad(20.0) < swing_roll.theta_max_rad


def test_the_request_aims_above_the_floor_not_at_it():
    """A theta floor is a minimum, and the touchdown theta is the IK's output.

    Aiming at exactly 35 deg produced 34.99955 deg and failed by 0.00045 deg.
    The headroom is the honest reading of "at least"; the *check* still uses
    the floor itself.
    """

    assert TARGET_THETA_HEADROOM_RAD > 0.0
    assert TARGET_THETA_HEADROOM_RAD < np.deg2rad(10.0), "headroom, not a new target"


def test_a_wrong_rim_touchdown_is_rejected_with_a_reason():
    target = REPOSITION_TARGETS[StrategyId.SWING_ROLL]

    class _End:
        rim, theta_rad = RimId.FOOT, float(np.deg2rad(17.0))

    class _Segment:
        end_contact = _End()

    ok, why = target.accepts(_Segment())
    assert not ok
    assert "left_rim" in why and "foot_rim" in why


def test_a_theta_below_the_floor_is_rejected_with_the_numbers():
    target = REPOSITION_TARGETS[StrategyId.ROLL_SWING]

    class _End:
        rim, theta_rad = RimId.FOOT, float(np.deg2rad(30.0))

    class _Segment:
        end_contact = _End()

    ok, why = target.accepts(_Segment())
    assert not ok
    assert "30.00" in why and "35.00" in why


def test_a_satisfying_touchdown_is_accepted_and_says_why():
    target = REPOSITION_TARGETS[StrategyId.ROLL_SWING]

    class _End:
        rim, theta_rad = RimId.FOOT, float(np.deg2rad(37.0))

    class _Segment:
        end_contact = _End()

    ok, why = target.accepts(_Segment())
    assert ok
    assert "37.00" in why


# --------------------------------------------------------------------------
# Only TOP_REPOSITION, and only where one exists
# --------------------------------------------------------------------------


def test_a_strategy_that_was_never_blocked_is_refused(context):
    four, trajectory = context
    with pytest.raises(ValueError, match="not a blocked pair"):
        resolve_top_reposition_2d(four, trajectory, StrategyId.SWING_SWING,
                                  0.08, 0.30)


def test_the_requirement_that_is_resolved_is_day_10_11_s_own(context):
    four, trajectory = context
    requirement = top_reposition_requirement_2d(StrategyId.ROLL_SWING, 0.16, 0.35)
    assert requirement.kind is TransitionKind.TOP_REPOSITION
    assert requirement.requires_external_support
    assert not requirement.resolved


# --------------------------------------------------------------------------
# Support first (requirements 2, 3 and 4)
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def planning_floor(context):
    four, trajectory = context
    return [resolve_top_reposition_2d(four, trajectory, strategy, h, l,
                                      leg=LegId.LF)
            for strategy, (h, l) in BLOCKED_CELLS.items()]


def test_the_support_gate_uses_the_interval_the_schedule_already_assigned(context):
    four, trajectory = context
    gate = support_gate_2d(four, trajectory, LegId.LF)
    airborne = [s for s in four.schedule.segments_of(LegId.LF)
                if s.mode.value == "AIRBORNE"]
    assert gate.start_s == pytest.approx(airborne[0].start_s)


def test_both_cases_fail_the_support_gate_at_the_planning_floor(planning_floor):
    """Step 6 measured 0.995 mm against a 10 mm floor; this is that, applied."""

    for attempt in planning_floor:
        assert attempt.outcome is RepositionOutcome.SUPPORT_INSUFFICIENT
        assert not attempt.resolved
        assert not attempt.gate.passed
        assert attempt.gate.minimum_margin_m < DEFAULT_MARGIN_FLOOR_M


def test_a_failed_gate_stops_before_any_motion_is_generated(planning_floor):
    """Requirement 4: no ABAD correction, and no trajectory either.

    Generating the swing anyway would produce something that looks like an
    answer to a question that was not allowed to be asked.
    """

    for attempt in planning_floor:
        assert attempt.swing is None
        assert attempt.segment is None


def test_the_failure_reason_names_the_margin_and_the_floor(planning_floor):
    for attempt in planning_floor:
        assert "margin" in attempt.reason
        assert "floor" in attempt.reason


def test_the_original_failure_reason_is_never_overwritten(planning_floor):
    """Requirement 10.  Day 10-11's evidence survives Day 12's verdict."""

    for attempt in planning_floor:
        original = BLOCKED_PAIRS[attempt.strategy].evidence
        assert attempt.original_evidence == original
        assert attempt.requirement.evidence == original
        assert not attempt.requirement.resolved, "the record stays a requirement"


# --------------------------------------------------------------------------
# With support granted, the rest of the chain runs (requirements 5, 6, 7, 8)
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def relaxed(context):
    four, trajectory = context
    return {strategy: resolve_top_reposition_2d(
                four, trajectory, strategy, h, l, leg=LegId.LF,
                margin_floor_m=-1.0,
                landing_beta_step_deg=TEST_BETA_STEP_DEG)
            for strategy, (h, l) in BLOCKED_CELLS.items()}


def test_a_relaxed_run_is_stamped_so_it_cannot_be_read_as_the_answer(relaxed):
    for attempt in relaxed.values():
        assert attempt.relaxed_floor
        assert attempt.as_dict()["relaxed_floor"] is True


def test_case_2_resolves_once_the_support_gate_opens(relaxed):
    """Plan §14's acceptance: a real ``resolved = True`` in the four-leg context."""

    attempt = relaxed[StrategyId.ROLL_SWING]
    assert attempt.outcome is RepositionOutcome.RESOLVED
    assert attempt.resolved
    assert attempt.swing.valid, "collision and touchdown checks passed"
    assert attempt.segment is not None


def test_the_resolved_segment_is_the_new_kind_and_lands_where_it_must(relaxed):
    attempt = relaxed[StrategyId.ROLL_SWING]
    segment = attempt.segment
    assert segment.kind is SegmentKind.TOP_REPOSITION_SWING
    assert segment.end_contact.rim is RimId.FOOT
    assert segment.end_contact.theta_rad >= np.deg2rad(35.0)


def test_case_3_reaches_a_specific_motion_failure_not_a_shrug(relaxed):
    """LEFT_RIM_READY needs a beta far from the start's, and the direct swing
    cannot get there: ``JOINT_DISCONTINUITY``.  A real reason, not "no"."""

    attempt = relaxed[StrategyId.SWING_ROLL]
    assert attempt.outcome is RepositionOutcome.SWING_FAILED
    assert not attempt.resolved
    assert attempt.swing is not None and not attempt.swing.valid
    assert attempt.swing.failure.value != "NONE"
    assert attempt.swing.failure.value in attempt.reason


def test_every_outcome_is_a_real_verdict(planning_floor, relaxed):
    """No attempt is allowed to end in "unknown"."""

    for attempt in list(planning_floor) + list(relaxed.values()):
        assert attempt.outcome in set(RepositionOutcome)
        assert attempt.reason.strip()


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------


def test_rows_are_csv_writable_and_carry_both_row_kinds(planning_floor):
    rows = reposition_rows(planning_floor)
    assert all(len(r) == len(rows[0]) for r in rows)
    assert {r["row_kind"] for r in rows} == {"attempt", "support_gate"}


def test_rows_keep_the_original_evidence_beside_the_verdict(planning_floor):
    rows = [r for r in reposition_rows(planning_floor) if r["row_kind"] == "attempt"]
    for row in rows:
        assert row["original_evidence"]
        assert row["target_condition"]
        assert row["outcome"]
