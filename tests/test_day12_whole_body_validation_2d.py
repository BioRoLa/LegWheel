"""Day 12 Step 9: whole-body validation.

Plan §16.  Two kinds of test here: the real trajectory is validated once in a
module fixture, and the individual checks are exercised against hand-built
samples so that each one is shown to fire for its own reason rather than being
credited for a failure some other check found.
"""

import numpy as np
import pytest

from legwheel.config import RobotParams

from hybrid_note.scripts.experiments.day10_11_composer_2d import ComposedSequence2D
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import StrategyId
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    RimId,
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
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    LegMode,
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    TransitionPhase,
    build_leg_plan_2d,
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    LegSample2D,
    WholeBodySample2D,
    WholeBodyTrajectory2D,
    assemble_whole_body_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (
    BETA_GUARD_NOTE,
    BETA_IS_CONTINUOUS,
    BETA_WORKSPACE_GUARD_RAD,
    MOTOR_MAX_RATE_RAD_S,
    MOTOR_MAX_RPM,
    motor_rates_rad_s,
    DELEGATED_CHECKS,
    THETA_MAX_RAD,
    THETA_MIN_RAD,
    UNEVALUABLE_CHECKS,
    CheckId,
    joint_rates_2d,
    validate_whole_body_2d,
    validation_rows,
)


@pytest.fixture(scope="module")
def validated():
    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="flat run, no crossing")
    plans = {leg: build_leg_plan_2d(leg, composed) for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, walk_timing_2d())
    state = initialize_four_leg_state_2d(SharedTerrainSpec2D(
        height_m=0.04, top_length_m=0.40, x_start_m=1.00,
        obstacle_id="day12_platform"))
    body = body_trajectory_2d(
        four, nominal_body_z_m=float(state.body_position_world_m[2]),
        samples=121)
    stability = swing_stability_2d(four, body)
    whole = assemble_whole_body_2d(four, body, stability, samples=121,
                                   reposition_unresolved=2)
    return whole, body, stability, validate_whole_body_2d(whole, body, stability)


# --------------------------------------------------------------------------
# What the validator is, and is not
# --------------------------------------------------------------------------


def _same(a, b) -> bool:
    """Equality that treats two NaNs as the same value.

    Step 5 left most body heights as NaN, and ``nan != nan`` would make an
    unchanged trajectory look changed -- which would be this test reporting its
    own comparison rather than the code under test.
    """

    if isinstance(a, float) and isinstance(b, float):
        return (a == b) or (np.isnan(a) and np.isnan(b))
    return a == b


def test_the_validator_repairs_nothing(validated):
    """Plan §16: no silent repair.  The trajectory is unchanged afterwards."""

    whole, body, stability, _ = validated
    before = [s.as_dict() for s in whole.samples]
    identity = whole.samples
    validate_whole_body_2d(whole, body, stability)

    assert whole.samples is identity, "not even the container was replaced"
    after = [s.as_dict() for s in whole.samples]
    assert len(after) == len(before)
    for row_a, row_b in zip(before, after):
        assert set(row_a) == set(row_b)
        assert all(_same(row_a[k], row_b[k]) for k in row_a)


def test_a_check_that_was_not_run_is_not_a_pass(validated):
    """Delegated and unevaluable checks are listed, not omitted."""

    _, _, _, report = validated
    assert report.delegated == DELEGATED_CHECKS
    assert report.unevaluable == UNEVALUABLE_CHECKS
    assert set(report.delegated) & set(c.value for c in report.checks_run) == set()
    assert set(report.unevaluable) & set(c.value for c in report.checks_run) == set()


def test_every_delegated_check_names_who_owns_it(validated):
    _, _, _, report = validated
    for why in report.delegated.values():
        assert any(token in why for token in ("Day 8-9", "Day 6-7", "Step 7"))


def test_every_unevaluable_check_names_what_is_missing(validated):
    _, _, _, report = validated
    assert "FrameRef2D" in report.unevaluable["theta_rate_interior"]
    assert "joint_rate_limit" not in report.unevaluable, (
        "the motor speed is known now (330 rpm); it is a real check")


# --------------------------------------------------------------------------
# Which checks pass, and which fail, on the real trajectory
# --------------------------------------------------------------------------


def test_the_timing_checks_all_pass(validated):
    """Steps 3 and 8 got the schedule right; this is that, independently."""

    _, _, _, report = validated
    for check in (CheckId.TIME_STRICTLY_INCREASING,
                  CheckId.AT_MOST_ONE_AIRBORNE,
                  CheckId.THREE_SUPPORT_LEGS):
        assert report.failures_of(check) == ()


def test_theta_stays_within_the_motor_limits(validated):
    _, _, _, report = validated
    assert report.failures_of(CheckId.THETA_WITHIN_LIMITS) == ()
    assert THETA_MIN_RAD == pytest.approx(np.deg2rad(RobotParams.MIN_THETA_DEG))
    assert THETA_MAX_RAD == pytest.approx(np.deg2rad(RobotParams.MAX_THETA_DEG))


def test_joint_and_body_continuity_hold_within_segments(validated):
    _, _, _, report = validated
    assert report.failures_of(CheckId.JOINT_CONTINUITY) == ()
    assert report.failures_of(CheckId.BODY_CONTINUITY) == ()


def test_stance_and_contact_agree_everywhere(validated):
    _, _, _, report = validated
    assert report.failures_of(CheckId.STANCE_CONTACT_VALID) == ()


def test_the_trajectory_is_not_valid_and_says_why(validated):
    """Three checks fail, and each traces to a Step 4-7 result, not to Step 9."""

    _, _, _, report = validated
    assert not report.is_valid
    assert set(report.failed_checks()) == {
        CheckId.BODY_REQUIREMENT_SATISFIED,
        CheckId.SEGMENT_CHAINING,
        CheckId.SUPPORT_MARGIN,
    }


def test_the_legacy_beta_guard_is_measured_but_not_failed(validated):
    """The joint rotates continuously (project owner), so ``BETA_MAX_DEG`` and
    the Hybrid beta are describing different quantities.  The excursion is
    still counted -- the number must not vanish with the failure."""

    _, _, _, report = validated
    assert BETA_IS_CONTINUOUS
    assert report.failures_of(CheckId.BETA_WORKSPACE_GUARD) == ()
    assert report.beta_outside_legacy_guard > 0
    assert "continuous" in BETA_GUARD_NOTE
    assert BETA_WORKSPACE_GUARD_RAD == pytest.approx(
        np.deg2rad(RobotParams.BETA_MAX_DEG))


def test_turning_the_hardware_fact_off_brings_the_failures_back(validated):
    """The fact is a switch, not a deletion: a bounded joint still fails."""

    import hybrid_note.scripts.experiments.day12_whole_body_validation_2d as v

    whole, body, stability, _ = validated
    original = v.BETA_IS_CONTINUOUS
    try:
        v.BETA_IS_CONTINUOUS = False
        report = v.validate_whole_body_2d(whole, body, stability)
        failures = report.failures_of(CheckId.BETA_WORKSPACE_GUARD)
        assert failures
        assert all(f.segment_kind is SegmentKind.RECOVERY_SWING
                   for f in failures)
    finally:
        v.BETA_IS_CONTINUOUS = original


# --------------------------------------------------------------------------
# The motor budget the two joints share
# --------------------------------------------------------------------------


def test_the_motor_transform_is_the_project_s_own():
    """``phi_r = theta + beta - theta_0``, ``phi_l = -theta + beta + theta_0``
    (``legwheel/utils/utils.py``); the offsets drop out of the rates."""

    assert motor_rates_rad_s(1.0, 0.0) == (1.0, -1.0)
    assert motor_rates_rad_s(0.0, 1.0) == (1.0, 1.0)


def test_the_two_joints_share_one_speed_budget():
    """Both motors must stay inside the limit, so the rates add."""

    limit = MOTOR_MAX_RATE_RAD_S
    assert MOTOR_MAX_RPM == 330.0
    assert np.rad2deg(limit) == pytest.approx(1980.0)
    worst = max(abs(r) for r in motor_rates_rad_s(0.6 * limit, 0.6 * limit))
    assert worst > limit, "0.6 + 0.6 of the budget does not fit"


def test_the_motor_rate_check_passes_and_says_by_how_much(validated):
    """Step 3's 10.553x demand, finally judged: it fits comfortably."""

    _, _, _, report = validated
    assert report.failures_of(CheckId.MOTOR_RATE_LIMIT) == ()
    for rate in report.joint_rates:
        assert rate.motor_utilisation < 0.30
        assert rate.peak_motor_rate_rad_s < MOTOR_MAX_RATE_RAD_S


def test_there_is_room_left_for_theta_while_beta_is_at_its_peak(validated):
    """The headroom that makes theta-compensated rolling worth trying."""

    _, _, _, report = validated
    row = report.joint_rates[0].as_dict()
    assert row["theta_rate_headroom_deg_s"] > 1000.0


def test_the_chaining_failure_is_the_known_break_not_a_new_one(validated):
    """Trap 25: the two nominal runs are generated independently."""

    _, _, _, report = validated
    failures = report.failures_of(CheckId.SEGMENT_CHAINING)
    assert failures
    assert all("teleport" in f.detail for f in failures)
    assert max(f.value for f in failures) == pytest.approx(297.065, abs=0.01)


def test_the_body_failure_quotes_step_5(validated):
    _, _, _, report = validated
    failures = report.failures_of(CheckId.BODY_REQUIREMENT_SATISFIED)
    assert any("INFEASIBLE" in f.detail for f in failures)
    assert any("no body height at all" in f.detail for f in failures)


def test_the_margin_failure_quotes_step_6(validated):
    _, _, _, stability = validated[0], validated[1], validated[2], validated[3]
    failures = stability.failures_of(CheckId.SUPPORT_MARGIN)
    assert len(failures) == 5, "one per unstable swing"
    for failure in failures:
        assert failure.leg is not None
        assert failure.segment_index is not None


# --------------------------------------------------------------------------
# Every failure is structured (plan §16's explicit requirement)
# --------------------------------------------------------------------------


def test_every_failure_carries_the_fields_the_plan_asks_for(validated):
    _, _, _, report = validated
    for failure in report.failures:
        row = failure.as_dict()
        assert row["check"] and row["detail"]
        for key in ("time_s", "leg", "segment_index", "segment_kind",
                    "value", "limit"):
            assert key in row


def test_a_per_leg_failure_names_the_leg_and_the_segment(validated):
    _, _, _, report = validated
    per_leg = [f for f in report.failures if f.leg is not None]
    assert per_leg
    for failure in per_leg:
        assert failure.leg in LEG_ORDER
        assert failure.segment_index is not None


# --------------------------------------------------------------------------
# Each check fires for its own reason
# --------------------------------------------------------------------------


def _leg_sample(leg, *, theta_deg=60.0, beta_deg=0.0, mode=LegMode.STANCE,
                in_contact=None, segment_index=0):
    return LegSample2D(
        leg=leg, theta_rad=float(np.deg2rad(theta_deg)),
        beta_rad=float(np.deg2rad(beta_deg)), gamma_rad=0.0, mode=mode,
        rim=RimId.FOOT, alpha_rad=0.0, contact_world_xy_m=(0.0, 0.0),
        in_contact=(mode is LegMode.STANCE) if in_contact is None else in_contact,
        segment_index=segment_index, segment_kind=SegmentKind.FOOT_RIM_ROLL,
        phase=TransitionPhase.NOMINAL_BEFORE, arc_samples=241,
        rim_geometry_gap_m=0.0,
    )


def _sample(time_s, legs, swing=None, support=(), margin=0.02, body_z=0.162):
    return WholeBodySample2D(
        time_s=time_s, body_position_world_m=(0.0, 0.0, body_z),
        body_rpy_rad=(0.0, 0.0, 0.0), legs=legs, swing_leg=swing,
        support_legs=support, stability_margin_m=margin,
    )


def _trajectory(samples):
    return WholeBodyTrajectory2D(samples=tuple(samples), handoffs=())


def _clean_legs(**overrides):
    legs = {leg: _leg_sample(leg) for leg in LEG_ORDER}
    legs.update(overrides)
    return legs


def _report(samples, body, stability):
    return validate_whole_body_2d(_trajectory(samples), body, stability)


def test_two_airborne_legs_are_caught(validated):
    _, body, stability, _ = validated
    legs = _clean_legs(
        **{LegId.LF: _leg_sample(LegId.LF, mode=LegMode.AIRBORNE),
           LegId.RF: _leg_sample(LegId.RF, mode=LegMode.AIRBORNE)})
    report = _report([_sample(0.0, legs, swing=LegId.LF,
                              support=(LegId.LH, LegId.RH))], body, stability)
    assert report.failures_of(CheckId.AT_MOST_ONE_AIRBORNE)


def test_a_swing_with_two_supports_is_caught(validated):
    _, body, stability, _ = validated
    legs = _clean_legs(**{LegId.LF: _leg_sample(LegId.LF, mode=LegMode.AIRBORNE)})
    report = _report([_sample(0.0, legs, swing=LegId.LF,
                              support=(LegId.LH, LegId.RH))], body, stability)
    assert report.failures_of(CheckId.THREE_SUPPORT_LEGS)


def test_time_going_backwards_is_caught(validated):
    _, body, stability, _ = validated
    report = _report([_sample(1.0, _clean_legs()),
                      _sample(0.5, _clean_legs())], body, stability)
    assert report.failures_of(CheckId.TIME_STRICTLY_INCREASING)


def test_theta_beyond_the_motor_limit_is_caught(validated):
    _, body, stability, _ = validated
    legs = _clean_legs(**{LegId.LF: _leg_sample(LegId.LF, theta_deg=170.0)})
    report = _report([_sample(0.0, legs)], body, stability)
    failures = report.failures_of(CheckId.THETA_WITHIN_LIMITS)
    assert failures and failures[0].leg is LegId.LF


def test_a_stance_leg_not_in_contact_is_caught(validated):
    _, body, stability, _ = validated
    legs = _clean_legs(**{LegId.LF: _leg_sample(LegId.LF, in_contact=False)})
    report = _report([_sample(0.0, legs)], body, stability)
    assert report.failures_of(CheckId.STANCE_CONTACT_VALID)


def test_a_joint_teleport_inside_one_segment_is_caught(validated):
    _, body, stability, _ = validated
    first = _clean_legs()
    second = _clean_legs(**{LegId.LF: _leg_sample(LegId.LF, theta_deg=120.0)})
    report = _report([_sample(0.0, first), _sample(0.01, second)],
                     body, stability)
    assert report.failures_of(CheckId.JOINT_CONTINUITY)


def test_a_body_teleport_is_caught(validated):
    _, body, stability, _ = validated
    far = WholeBodySample2D(
        time_s=0.01, body_position_world_m=(1.0, 0.0, 0.162),
        body_rpy_rad=(0.0, 0.0, 0.0), legs=_clean_legs(), swing_leg=None,
        support_legs=(), stability_margin_m=0.02)
    report = _report([_sample(0.0, _clean_legs()), far], body, stability)
    assert report.failures_of(CheckId.BODY_CONTINUITY)


# --------------------------------------------------------------------------
# The rates Step 3 deferred to here
# --------------------------------------------------------------------------


def test_the_measured_beta_ratio_matches_step_3_s_prediction(validated):
    """Step 3 derived 10.553x from the duty; this measures it on the motion."""

    whole, _, _, report = validated
    assert len(report.joint_rates) == 4
    for rate in report.joint_rates:
        assert rate.airborne_to_stance_ratio == pytest.approx(10.553, abs=0.01)


def test_the_theta_rate_is_flagged_as_a_lower_bound(validated):
    """It reads 0 because Step 8 interpolates between endpoints that share a
    theta -- the recovery's retraction is not in the assembled samples."""

    _, _, _, report = validated
    for rate in report.joint_rates:
        assert rate.theta_rate_is_lower_bound
        assert rate.peak_theta_rate_rad_s == pytest.approx(0.0)
    assert "lower bound" in UNEVALUABLE_CHECKS["theta_rate_interior"].lower() \
        or "invisible" in UNEVALUABLE_CHECKS["theta_rate_interior"]


def test_the_rates_skip_the_boundary_revolution(validated):
    """Dividing a whole turn by a sample interval would invent a rate."""

    whole, _, _, _ = validated
    rates = joint_rates_2d(whole)
    for rate in rates:
        assert rate.peak_beta_rate_rad_s < np.deg2rad(1000.0)


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------


def test_rows_are_csv_writable_with_one_header(validated):
    _, _, _, report = validated
    rows = validation_rows(report)
    assert all(len(r) == len(rows[0]) for r in rows)


def test_rows_carry_every_row_kind(validated):
    _, _, _, report = validated
    kinds = {r["row_kind"] for r in validation_rows(report)}
    assert kinds == {"summary", "check", "failure", "delegated", "unevaluable",
                     "joint_rate", "assumption"}


def test_every_check_gets_a_verdict_row(validated):
    _, _, _, report = validated
    rows = [r for r in validation_rows(report) if r["row_kind"] == "check"]
    assert len(rows) == len(report.checks_run)
    assert sum(1 for r in rows if r["passed"]) == len(report.passed_checks())


def test_the_assumptions_travel_into_the_report(validated):
    whole, _, _, report = validated
    assert report.assumptions == whole.assumptions
    assert report.assumptions
