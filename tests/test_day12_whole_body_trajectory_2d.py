"""Day 12 Step 8: the complete synchronized four-leg trajectory.

Plan §15.  The assembly is cheap -- it interpolates finished results -- so the
whole thing is built once in a module fixture and the requirements are checked
against it.
"""

import numpy as np
import pytest

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
    BODY_BASIS,
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    initialize_four_leg_state_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    COM_BASIS,
    GAMMA_RAD,
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
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    FULL_TURN_RAD,
    assemble_whole_body_2d,
    assumptions_of,
    handoff_checks_2d,
    whole_body_rows,
)


@pytest.fixture(scope="module")
def assembled():
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
    result = assemble_whole_body_2d(four, body, stability, samples=121,
                                    reposition_unresolved=2)
    return four, body, stability, result


# --------------------------------------------------------------------------
# Requirements 1 and 2: every sample is complete
# --------------------------------------------------------------------------


def test_the_samples_cover_the_interval_all_four_legs_share(assembled):
    """Half-open, like Step 6's swing sampling: at the closing instant one
    leg's chain has run out while another's swing has begun, so that instant
    has no complete four-leg configuration."""

    four, _, _, result = assembled
    lo, hi = four.schedule.covered_interval_s
    assert result.samples[0].time_s == pytest.approx(lo)
    assert lo < result.samples[-1].time_s < hi
    assert result.samples[-1].time_s > hi - 0.05 * (hi - lo)


def test_every_sample_carries_body_pose_and_all_four_legs(assembled):
    _, _, _, result = assembled
    assert result.every_sample_has_four_legs
    for sample in result.samples:
        assert len(sample.body_position_world_m) == 3
        assert len(sample.body_rpy_rad) == 3
        assert set(sample.legs) == set(LEG_ORDER)


def test_every_leg_sample_has_theta_beta_gamma(assembled):
    _, _, _, result = assembled
    for sample in result.samples:
        for leg_sample in sample.legs.values():
            for value in (leg_sample.theta_rad, leg_sample.beta_rad,
                          leg_sample.gamma_rad):
                assert np.isfinite(value)


def test_gamma_is_zero_everywhere(assembled):
    """Day 12 keeps ABAD at zero; Step 8 must not quietly introduce one."""

    _, _, _, result = assembled
    for sample in result.samples:
        for leg_sample in sample.legs.values():
            assert leg_sample.gamma_rad == GAMMA_RAD == 0.0


def test_every_leg_sample_has_mode_contact_rim_and_alpha(assembled):
    _, _, _, result = assembled
    for sample in result.samples:
        for leg_sample in sample.legs.values():
            assert isinstance(leg_sample.mode, LegMode)
            assert isinstance(leg_sample.rim, RimId)
            assert np.isfinite(leg_sample.alpha_rad)
            assert len(leg_sample.contact_world_xy_m) == 2
            assert leg_sample.in_contact == (leg_sample.mode is LegMode.STANCE)


# --------------------------------------------------------------------------
# Requirement 3: the whole-body metadata
# --------------------------------------------------------------------------


def test_each_sample_names_the_swing_leg_and_the_support_legs(assembled):
    _, _, _, result = assembled
    for sample in result.samples:
        if sample.swing_leg is not None:
            assert sample.swing_leg not in sample.support_legs
            assert len(sample.support_legs) == 3


def test_each_sample_carries_a_stability_margin_where_one_exists(assembled):
    _, _, _, result = assembled
    with_margin = [s for s in result.samples if s.stability_margin_m is not None]
    assert with_margin, "the flat run has swings, so it has margins"


def test_each_leg_sample_carries_its_segment_identity(assembled):
    _, _, _, result = assembled
    for sample in result.samples:
        for leg_sample in sample.legs.values():
            assert leg_sample.segment_index >= 0
            assert isinstance(leg_sample.segment_kind, SegmentKind)
            assert leg_sample.phase is not None


# --------------------------------------------------------------------------
# Requirement 4: sampling parameters are carried, not re-derived
# --------------------------------------------------------------------------


def test_the_segments_own_arc_samples_are_carried_through(assembled):
    four, _, _, result = assembled
    sample = result.samples[len(result.samples) // 2]
    for leg, leg_sample in sample.legs.items():
        segment = four.plans[leg].phased[leg_sample.segment_index].segment
        assert leg_sample.arc_samples == segment.sampling.arc_samples


# --------------------------------------------------------------------------
# Requirement 5: the five handoff checks
# --------------------------------------------------------------------------


def test_every_boundary_of_every_leg_is_checked(assembled):
    four, _, _, result = assembled
    expected = sum(max(len(four.schedule.segments_of(leg)) - 1, 0)
                   for leg in LEG_ORDER)
    assert len(result.handoffs) == expected


def test_time_is_monotonic_at_every_handoff_and_across_the_samples(assembled):
    _, _, _, result = assembled
    assert result.time_is_monotonic
    assert all(h.time_is_monotonic for h in result.handoffs)


def test_a_recovery_s_full_turn_is_reported_as_a_turn_not_a_break(assembled):
    """beta is a revolution counter here and is never wrapped.

    Step 1 builds the recovery with ``beta_target = start.beta - 2*pi``, so the
    boundary after it is 360 deg raw.  Reporting only that would call a
    revolution a discontinuity; reporting only the wrapped value would hide
    that the leg turned.  Both are kept.
    """

    _, _, _, result = assembled
    turns = [h for h in result.handoffs if h.is_whole_turn]
    assert turns, "the nominal cycle has such boundaries"
    for handoff in turns:
        assert abs(abs(handoff.joint_jump_rad) - FULL_TURN_RAD) < 1e-6
        assert abs(handoff.joint_jump_wrapped_rad) < 1e-6
    assert result.max_joint_discontinuity_rad < 1e-6
    assert result.max_joint_jump_rad == pytest.approx(FULL_TURN_RAD)


def test_the_known_chain_break_shows_up_as_a_contact_gap(assembled):
    """Trap 25: the two nominal runs are generated independently, so they do
    not join.  Step 8 reports that; it does not stitch it."""

    _, _, _, result = assembled
    gaps = [h for h in result.handoffs if h.contact_jump_m > 0.1]
    assert gaps, "the independently generated runs do not chain"
    assert result.max_contact_gap_m == pytest.approx(0.297065, abs=1e-4)


# --------------------------------------------------------------------------
# Requirement 6: the 1.2 mm gap is quantified, not hidden
# --------------------------------------------------------------------------


def test_the_rim_geometry_gap_is_measured_at_every_handoff(assembled):
    _, _, _, result = assembled
    for handoff in result.handoffs:
        assert handoff.rim_geometry_gap_m >= 0.0
        assert np.isfinite(handoff.rim_geometry_gap_m)


def test_the_gap_is_zero_here_because_the_run_never_leaves_the_foot_rim(assembled):
    """Not "the 1.2 mm does not exist": it is 0 on the foot rim by construction
    and 1.2 mm on the upper tyres, and this run only uses the foot rim."""

    _, _, _, result = assembled
    assert result.max_rim_geometry_gap_m < 1e-9
    for sample in result.samples:
        for leg_sample in sample.legs.values():
            assert leg_sample.rim is RimId.FOOT
    assert "1.2 mm on the upper tyres" in result.as_dict()["rim_gap_note"]


# --------------------------------------------------------------------------
# The assumptions travel with the answer
# --------------------------------------------------------------------------


def test_the_unresolved_results_are_listed_on_the_trajectory(assembled):
    four, body, stability, result = assembled
    assert result.assumptions
    joined = " ".join(result.assumptions)
    assert "Step 5" in joined and "INFEASIBLE" in joined
    assert "Step 6" in joined and "unstable" in joined
    assert "Step 7" in joined and "TOP_REPOSITION" in joined


def test_an_assumption_is_only_listed_when_it_actually_holds(assembled):
    """The list is computed from the results, not written by hand."""

    four, body, stability, _ = assembled
    without = assumptions_of(four, body, stability, reposition_unresolved=0)
    assert not any("Step 7" in note for note in without)
    assert any("Step 5" in note for note in without)


def test_the_flat_run_has_no_step_4_overrun_to_report(assembled):
    """No crossing means no compressed airborne run -- and no invented one."""

    four, body, stability, result = assembled
    assert not four.airborne_overruns
    assert not any("Step 4" in note for note in result.assumptions)


def test_the_two_approximation_labels_are_carried(assembled):
    _, _, _, result = assembled
    assert result.body_basis == BODY_BASIS
    assert result.com_basis == COM_BASIS


# --------------------------------------------------------------------------
# Requirement 8: serialization
# --------------------------------------------------------------------------


def test_rows_are_csv_writable_with_one_header(assembled):
    _, _, _, result = assembled
    rows = whole_body_rows(result)
    assert all(len(r) == len(rows[0]) for r in rows)


def test_rows_carry_all_four_row_kinds(assembled):
    _, _, _, result = assembled
    kinds = {r["row_kind"] for r in whole_body_rows(result)}
    assert kinds == {"summary", "assumption", "handoff", "sample"}


def test_a_sample_row_has_a_column_per_leg(assembled):
    _, _, _, result = assembled
    row = next(r for r in whole_body_rows(result) if r["row_kind"] == "sample")
    for leg in LEG_ORDER:
        assert f"{leg.value}_theta_deg" in row
        assert f"{leg.value}_mode" in row
        assert f"{leg.value}_contact_x_mm" in row


def test_the_summary_reports_the_three_numbers_the_plan_asks_for(assembled):
    _, _, _, result = assembled
    summary = result.as_dict()
    for key in ("max_joint_jump_deg", "max_contact_gap_mm",
                "minimum_stability_margin_mm"):
        assert key in summary and summary[key] is not None


def test_infeasible_body_z_is_counted_not_silently_dropped(assembled):
    """Step 5 left most samples without a height; the summary says how many."""

    _, _, _, result = assembled
    summary = result.as_dict()
    assert summary["finite_body_z_samples"] < summary["samples"]
