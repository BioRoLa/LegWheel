"""Day 12 problem A4: the scan that says what the zero margin is.

The claims under test are the ones the resolution rests on:

* the project's gait sits at the **critical duty**, so its margin is zero by
  construction rather than by defect;
* its liftoff sequence is already the best of the six, so that axis is spent;
* a rolling stance leaves the support polygon a fraction of the hip's advance,
  and rolling less scales that fraction down instead of up;
* a fore-aft body shift would help enormously and cannot be installed, because
  the sign it needs reverses inside the cycle.

The pipeline half is built once, in a module fixture: assembling four leg plans
from the generator's frames is the expensive part of the whole Day 12 stack.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_composer_2d import ComposedSequence2D
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import StrategyId
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    HIP_TO_BODY_Z_M,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    LegId,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    NominalPosture2D,
)
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (
    COM_UNCERTAINTY_PER_AXIS_M,
    CRITICAL_STANCE_DUTY,
    HYBRID_MARGIN_FLOOR_M,
    HYBRID_STANCE_DUTY,
    MARGIN_LOST_PER_COM_OFFSET,
    LIFTOFF_SEQUENCES,
    LIFTOFF_SPACING,
    BodyOffsetProbe2D,
    FrameRateDemand2D,
    MarginScanPoint2D,
    RollingStride2D,
    best_body_offset_2d,
    best_within_motor_budget_2d,
    derived_margin_floor_m,
    frame_motor_rate_2d,
    hybrid_posture_2d,
    hybrid_timing_2d,
    liftoff_order_2d,
    phase_offsets_for_2d,
    resampled_motor_rate_rad_s,
    rolling_stride_2d,
    scan_point_2d,
    scan_rows,
    scan_support_margin_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    DEFAULT_MARGIN_FLOOR_M,
    support_triangle_at,
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    GaitTiming2D,
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    build_leg_plan_2d,
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (
    MOTOR_MAX_RATE_RAD_S,
)

#: What "the margin is zero" is allowed to mean numerically.  The contacts are
#: placed by a root solve on a half-metre geometry, so the residue lands around
#: a nanometre -- four orders below the smallest margin any *other* duty
#: produces (1.33 mm) and seven below the planning floor.  Asserting exact
#: equality would make the claim a float question instead of a gait one.
ZERO_TOL_M: float = 1e-6


# --------------------------------------------------------------------------
# Naming a gait by its liftoff order
# --------------------------------------------------------------------------


def test_the_offsets_produce_the_order_they_were_asked_for():
    for name, order in LIFTOFF_SEQUENCES.items():
        for duty in (0.75, 0.80, 0.9):
            timing = GaitTiming2D(
                cycle_period_s=2.4, stance_duty=duty,
                phase_offsets=phase_offsets_for_2d(order, duty),
                gait_name=name)
            assert liftoff_order_2d(timing) == order, name


def test_the_project_walk_entry_is_the_gait_library_gait():
    """The scan compares against ``GAIT_LIBRARY['Walk']``, so it has to *be* it."""

    library = walk_timing_2d()
    order = LIFTOFF_SEQUENCES["project_walk"]
    assert liftoff_order_2d(library) == order
    assert np.allclose(phase_offsets_for_2d(order, library.stance_duty),
                       library.phase_offsets)


def test_the_project_walk_sits_exactly_at_the_critical_duty():
    """The whole diagnosis: 0.75 is not near the critical duty, it *is* it."""

    assert walk_timing_2d().stance_duty == CRITICAL_STANCE_DUTY


def test_the_six_sequences_are_distinct_and_complete():
    orders = list(LIFTOFF_SEQUENCES.values())
    assert len(orders) == 6, "3! orderings once the first leg is fixed"
    assert len(set(orders)) == 6
    for order in orders:
        assert sorted(l.index for l in order) == [0, 1, 2, 3]
        assert order[0].index == 0, "cyclic: the front-left leg is fixed first"


def test_a_liftoff_order_that_repeats_a_leg_is_refused():
    front_left = LIFTOFF_SEQUENCES["project_walk"][0]
    with pytest.raises(ValueError):
        phase_offsets_for_2d((front_left,) * 4, 0.75)


def test_the_spacing_is_the_one_the_critical_duty_forces():
    """At the critical duty four swing windows tile the cycle exactly."""

    assert LIFTOFF_SPACING * 4 == pytest.approx(1.0)
    assert (1.0 - CRITICAL_STANCE_DUTY) == pytest.approx(LIFTOFF_SPACING)


# --------------------------------------------------------------------------
# What the rolling stance leaves behind
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def stride() -> RollingStride2D:
    """The stride of the posture the gait rolls in, not of the default one."""

    return rolling_stride_2d()


def test_the_stride_is_measured_on_the_posture_the_gait_actually_uses():
    """The two postures give different strides, so the default has to be the
    one that is flown -- otherwise the diagnostic describes another gait."""

    from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
        NominalPosture2D as _Fixed,
    )

    chosen = rolling_stride_2d()
    uncompensated = rolling_stride_2d(_Fixed())
    assert chosen.hip_advance_m != uncompensated.hip_advance_m
    assert chosen.contact_advance_m == pytest.approx(
        uncompensated.contact_advance_m, abs=1e-9), (
        "theta compensation moves the hip, not the contact")
    assert chosen.relative_stride_m > uncompensated.relative_stride_m
    assert rolling_stride_2d(hybrid_posture_2d()).hip_advance_m == pytest.approx(
        chosen.hip_advance_m)


def test_the_contact_advances_with_the_hip_so_the_polygon_sees_less(stride):
    assert stride.contact_advance_m > 0.0, "a rolling foot moves"
    assert stride.relative_stride_m == pytest.approx(
        stride.hip_advance_m - stride.contact_advance_m)
    assert stride.relative_stride_m < stride.hip_advance_m
    assert stride.stride_loss_ratio > 2.5, (
        "the measured cost of rolling: a planted foot would give the support "
        "polygon over two and a half times the excursion.  2.640x for the "
        "levelled posture the gait uses; the uncompensated posture reads "
        "3.140x and is not what is being flown")


def test_rolling_a_shorter_stroke_does_not_buy_the_stride_back(stride):
    """The answer to 'then just roll less'."""

    assert stride.relative_stride_is_proportional_to_roll
    for fraction, partial in zip(stride.partial_roll_fraction,
                                 stride.partial_relative_stride_m):
        assert partial < stride.relative_stride_m or fraction == 1.0
        assert partial / stride.relative_stride_m == pytest.approx(
            fraction, abs=0.10)


def test_a_stride_that_lost_everything_reports_infinite_loss_not_a_divide():
    degenerate = RollingStride2D(
        hip_advance_m=0.1, contact_advance_m=0.1, relative_stride_m=0.0,
        partial_relative_stride_m=(0.0,), partial_roll_fraction=(1.0,))
    assert degenerate.stride_loss_ratio == float("inf")
    assert degenerate.relative_stride_is_proportional_to_roll is False


# --------------------------------------------------------------------------
# Scoring a scan point
# --------------------------------------------------------------------------


def _point(**kwargs) -> MarginScanPoint2D:
    base = dict(sequence_name="project_walk", stance_duty=0.85,
                cycle_period_s=2.4, swing_window_s=0.36,
                min_margin_m=0.005, unstable_swings=5,
                peak_motor_rate_rad_s=0.5 * MOTOR_MAX_RATE_RAD_S)
    base.update(kwargs)
    return MarginScanPoint2D(**base)


def test_a_point_over_the_motor_budget_does_not_clear_however_good_the_margin():
    over = _point(min_margin_m=1.0,
                  peak_motor_rate_rad_s=2.0 * MOTOR_MAX_RATE_RAD_S)
    assert over.motor_is_within_budget is False
    assert over.clears(DEFAULT_MARGIN_FLOOR_M) is False


def test_an_unmeasured_motor_rate_is_not_a_pass():
    """Unknown is not the same as fine -- the same rule Step 6 uses."""

    unpriced = _point(min_margin_m=1.0, peak_motor_rate_rad_s=None)
    assert unpriced.motor_utilisation is None
    assert unpriced.motor_is_within_budget is None
    assert unpriced.clears(DEFAULT_MARGIN_FLOOR_M) is False


def test_a_margin_exactly_on_the_floor_does_not_clear_it():
    assert _point(min_margin_m=DEFAULT_MARGIN_FLOOR_M).clears(
        DEFAULT_MARGIN_FLOOR_M) is False


def test_best_within_budget_ignores_the_over_budget_maximum():
    points = (_point(stance_duty=0.85, min_margin_m=0.005),
              _point(stance_duty=0.90, min_margin_m=0.009,
                     peak_motor_rate_rad_s=1.2 * MOTOR_MAX_RATE_RAD_S))
    best = best_within_motor_budget_2d(points)
    assert best is not None and best.stance_duty == 0.85


def test_best_within_budget_is_none_when_nothing_was_priced():
    assert best_within_motor_budget_2d(
        (_point(peak_motor_rate_rad_s=None),)) is None


# --------------------------------------------------------------------------
# The scan itself
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def scanned():
    """Plans in the Day 13 configuration, scanned over both axes."""

    from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
        run_foot_rim_roll_2d,
    )
    from dataclasses import replace

    fixed = NominalPosture2D()
    held = float(max(f.hip_xz_m[1] for f in run_foot_rim_roll_2d(fixed).frames))
    levelled = replace(fixed, hold_hip_z_m=held)
    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="flat run, no crossing")
    plans = {leg: build_leg_plan_2d(leg, composed, posture=levelled,
                                    continuous_nominal=True)
             for leg in LEG_ORDER}
    body_z = held - HIP_TO_BODY_Z_M
    by_sequence = scan_support_margin_2d(
        plans, nominal_body_z_m=body_z, duties=(0.75,),
        sequences=tuple(LIFTOFF_SEQUENCES), samples=121)
    by_duty = scan_support_margin_2d(
        plans, nominal_body_z_m=body_z, duties=(0.75, 0.80, 0.85),
        sequences=("project_walk",), samples=121)
    return plans, body_z, by_sequence, by_duty


def test_the_margin_at_the_critical_duty_is_zero(scanned):
    _, _, _, by_duty = scanned
    critical = next(p for p in by_duty
                    if p.stance_duty == CRITICAL_STANCE_DUTY)
    assert critical.min_margin_m == pytest.approx(0.0, abs=ZERO_TOL_M), (
        "not 'small' -- the wave gait's margin is identically zero at 3/4")


def test_raising_the_duty_is_the_lever_that_changes_the_sign(scanned):
    _, _, _, by_duty = scanned
    ordered = sorted(by_duty, key=lambda p: p.stance_duty)
    margins = [p.min_margin_m for p in ordered]
    assert all(m is not None for m in margins)
    assert all(b > a for a, b in zip(margins, margins[1:])), (
        "monotone in duty, which is what makes it a usable lever")
    assert margins[0] == pytest.approx(0.0, abs=ZERO_TOL_M)
    assert margins[-1] > 0.0


def test_the_projects_own_sequence_is_the_only_non_negative_one(scanned):
    """The sequence axis is spent: there is nothing better to switch to."""

    _, _, by_sequence, _ = scanned
    project = next(p for p in by_sequence
                   if p.sequence_name == "project_walk")
    others = [p for p in by_sequence if p.sequence_name != "project_walk"]
    assert len(others) == 5
    assert project.min_margin_m == pytest.approx(0.0, abs=ZERO_TOL_M)
    for point in others:
        assert point.min_margin_m is not None
        assert point.min_margin_m < 0.0, (
            f"{point.sequence_name} would put the centre outside the polygon")


def test_the_ten_millimetre_floor_is_not_reachable_at_any_scanned_duty(scanned):
    """The finding that turns A4 into a decision instead of a bug."""

    _, _, _, by_duty = scanned
    for point in by_duty:
        assert point.min_margin_m is not None
        assert point.min_margin_m < DEFAULT_MARGIN_FLOOR_M


def test_the_body_shift_that_would_work_cannot_be_installed(scanned):
    plans, body_z, _, _ = scanned
    probe = best_body_offset_2d(plans, walk_timing_2d(),
                                nominal_body_z_m=body_z, search_steps=161,
                                samples=121)
    assert isinstance(probe, BodyOffsetProbe2D)
    assert probe.achievable_margin_m > 0.05, (
        "a fore-aft shift would buy an enormous margin ...")
    assert probe.offset_reverses_sign, (
        "... and the sign it needs reverses inside the cycle, so no constant "
        "CoM correction captures any of it")
    assert probe.required_offset_min_m < 0.0 < probe.required_offset_max_m


def test_rows_carry_every_point_and_share_one_header(scanned):
    _, _, by_sequence, by_duty = scanned
    rows = scan_rows(tuple(by_sequence) + tuple(by_duty), rolling_stride_2d())
    kinds = [row["row_kind"] for row in rows]
    assert kinds.count("rolling_stride") == 1
    assert kinds.count("scan_point") == len(by_sequence) + len(by_duty)
    header = list(rows[0])
    assert all(list(row) == header for row in rows)


# --------------------------------------------------------------------------
# The motor price, and the measurement that was giving the wrong one
# --------------------------------------------------------------------------


def test_every_scanned_point_is_priced_without_being_asked(scanned):
    """A margin without its cost is half a trade, so the price is not opt-in."""

    _, _, by_sequence, by_duty = scanned
    for point in tuple(by_sequence) + tuple(by_duty):
        assert point.peak_motor_rate_rad_s is not None
        assert point.motor_utilisation is not None
        assert point.peak_segment_kind == "RECOVERY_SWING", (
            "the recovery is where the duty lever bites")


def test_the_frame_rate_is_the_same_whatever_grid_is_asked_for(scanned):
    """The property that made this the measurement worth bounding duty with."""

    plans, body_z, _, _ = scanned
    coarse = scan_point_2d(plans, sequence_name="project_walk",
                           stance_duty=0.75, nominal_body_z_m=body_z,
                           resample_at=121, samples=121)
    fine = scan_point_2d(plans, sequence_name="project_walk",
                         stance_duty=0.75, nominal_body_z_m=body_z,
                         resample_at=961, samples=121)
    assert coarse.peak_motor_rate_rad_s == pytest.approx(
        fine.peak_motor_rate_rad_s, rel=1e-12)
    assert coarse.resampled_at_samples == 121
    assert fine.resampled_at_samples == 961


def test_the_resampled_rate_now_agrees_with_the_frame_rate(scanned):
    """The regression guard on the staircase.

    ``leg_sample_at`` used to snap to the nearest frame, which made the
    resampled signal a staircase: its finite difference was ``frame step /
    sample interval`` and grew without bound as the grid was refined (the same
    run read 48.1% at 241 samples and 127.7% at 1921).  With the frames
    interpolated the two measurements are measurements of one thing, and a
    finer grid may only approach the frame rate -- never exceed it.
    """

    plans, body_z, _, _ = scanned
    for grid in (121, 481, 1921):
        point = scan_point_2d(plans, sequence_name="project_walk",
                              stance_duty=0.75, nominal_body_z_m=body_z,
                              resample_at=grid, samples=121)
        assert point.resampled_motor_rate_rad_s <= (
            point.peak_motor_rate_rad_s * 1.02), (
            f"at {grid} samples the resampled rate "
            f"({point.resampled_motor_rate_rad_s:.2f}) exceeded the "
            f"frame-to-frame rate ({point.peak_motor_rate_rad_s:.2f}): the "
            "sampled signal is a staircase again")
        assert point.resampled_motor_rate_rad_s < MOTOR_MAX_RATE_RAD_S


def test_the_chosen_duty_is_the_project_walk_sequence_at_a_higher_duty():
    """``walk_timing_2d`` must stay untouched -- Day 12's numbers rest on it."""

    library = walk_timing_2d()
    chosen = hybrid_timing_2d()
    assert library.stance_duty == CRITICAL_STANCE_DUTY
    assert chosen.stance_duty == HYBRID_STANCE_DUTY > CRITICAL_STANCE_DUTY
    assert liftoff_order_2d(chosen) == liftoff_order_2d(library), (
        "only the duty differs: the sequence axis was already spent")
    assert chosen.cycle_period_s == library.cycle_period_s
    assert chosen.max_simultaneous_airborne == 1, (
        "still one leg in the air at a time, which is what makes the "
        "three-leg support the right model")


def test_the_chosen_duty_is_positive_margin_inside_the_motor_budget(scanned):
    """The claim log 1.6 rests on, re-measured through the public entry point."""

    plans, body_z, _, _ = scanned
    point = scan_point_2d(plans, sequence_name="project_walk",
                          stance_duty=HYBRID_STANCE_DUTY,
                          nominal_body_z_m=body_z, samples=121)
    assert point.min_margin_m is not None and point.min_margin_m > 0.0
    assert point.motor_is_within_budget is True
    assert point.min_cycle_period_s == pytest.approx(point.cycle_period_s), (
        "it runs at full speed -- no period stretch is needed")
    assert not point.clears(DEFAULT_MARGIN_FLOOR_M), (
        "and it still does not clear the 10 mm floor, which is the open "
        "question rather than a gait fault")


def test_the_peak_is_one_generator_step_over_one_frame_interval(scanned):
    """Name the quantity, so a later change to the step size is traceable."""

    plans, body_z, _, _ = scanned
    from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
        plan_four_legs_2d,
    )
    four = plan_four_legs_2d(plans, walk_timing_2d())
    demands = frame_motor_rate_2d(four)
    assert demands, "the plans carry frames, so there is something to measure"
    worst = max(demands, key=lambda d: d.peak_motor_rate_rad_s)
    assert isinstance(worst, FrameRateDemand2D)
    assert worst.segment_kind == "RECOVERY_SWING"
    rebuilt = max(abs(r) for r in __import__(
        "hybrid_note.scripts.experiments.day12_whole_body_validation_2d",
        fromlist=["motor_rates_rad_s"]).motor_rates_rad_s(
            worst.theta_step_rad / worst.frame_interval_s,
            worst.beta_step_rad / worst.frame_interval_s))
    assert rebuilt == pytest.approx(worst.peak_motor_rate_rad_s, rel=1e-12)


def test_a_segment_without_frames_is_skipped_not_scored_as_zero(scanned):
    """No frames is not a slow segment; a zero row would hide that."""

    plans, _, _, _ = scanned
    from dataclasses import replace as _replace
    from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
        plan_four_legs_2d,
    )
    stripped = {leg: _replace(plan, frames={}) for leg, plan in plans.items()}
    assert frame_motor_rate_2d(plan_four_legs_2d(stripped,
                                                 walk_timing_2d())) == ()


def test_an_over_budget_duty_reports_the_period_that_would_afford_it(scanned):
    """Over budget is slower, not impossible -- the rate scales with the period."""

    plans, body_z, _, _ = scanned
    fast = scan_point_2d(plans, sequence_name="project_walk", stance_duty=0.95,
                         nominal_body_z_m=body_z, samples=121)
    assert fast.motor_is_within_budget is False
    assert fast.min_cycle_period_s > fast.cycle_period_s
    assert fast.min_cycle_period_s == pytest.approx(
        fast.cycle_period_s * fast.motor_utilisation)
    inside = scan_point_2d(plans, sequence_name="project_walk",
                           stance_duty=0.95, nominal_body_z_m=body_z,
                           cycle_period_s=fast.min_cycle_period_s, samples=121)
    assert inside.motor_utilisation == pytest.approx(1.0, abs=1e-9)
    assert inside.min_margin_m == pytest.approx(fast.min_margin_m, rel=1e-9), (
        "slowing the gait buys motor headroom and changes no geometry")



# --------------------------------------------------------------------------
# The margin floor, and the chain it hangs on
# --------------------------------------------------------------------------


def test_the_floor_is_derived_from_the_com_uncertainty_not_picked():
    """The whole reason this floor is defensible where 10 mm was not."""

    assert derived_margin_floor_m() == pytest.approx(
        COM_UNCERTAINTY_PER_AXIS_M * MARGIN_LOST_PER_COM_OFFSET)
    assert derived_margin_floor_m(0.004) == pytest.approx(
        2.0 * derived_margin_floor_m(0.002)), "linear in the uncertainty"


def test_the_floor_in_use_rounds_up_from_the_derivation_never_down():
    """Rounding a safety floor down would quietly spend the safety."""

    assert HYBRID_MARGIN_FLOOR_M >= derived_margin_floor_m()
    assert HYBRID_MARGIN_FLOOR_M - derived_margin_floor_m() < 0.001, (
        "and not so far up that it stops being the derived number")


def test_the_day12_default_floor_is_left_untouched():
    """Day 12's frozen numbers were all measured against 10 mm."""

    assert DEFAULT_MARGIN_FLOOR_M == 0.010
    assert HYBRID_MARGIN_FLOOR_M < DEFAULT_MARGIN_FLOOR_M


def test_the_sensitivity_constant_matches_what_the_geometry_does(scanned):
    """``MARGIN_LOST_PER_COM_OFFSET`` is a measurement, so it must re-measure."""

    plans, body_z, _, _ = scanned
    four = plan_four_legs_2d(plans, hybrid_timing_2d())
    body = body_trajectory_2d(four, nominal_body_z_m=body_z, samples=121)
    stability = swing_stability_2d(four, body,
                                   margin_floor_m=HYBRID_MARGIN_FLOOR_M)

    cache = []
    for swing in stability.swings:
        for sample in swing.samples:
            body_x = float(np.interp(sample.time_s, body.time_s, body.body_x_m))
            cache.append((body_x, support_triangle_at(
                four, body_x, sample.time_s, swing_leg=swing.swing_leg)))

    def worst(offset_m: float) -> float:
        out = float("inf")
        for sign_x in (1.0, -1.0):
            for sign_y in (1.0, -1.0):
                for body_x, triangle in cache:
                    margin = triangle.signed_margin_m(
                        (body_x + sign_x * offset_m, sign_y * offset_m))
                    if margin is not None:
                        out = min(out, margin)
        return out

    nominal = worst(0.0)
    probe = 0.002
    measured = (nominal - worst(probe)) / probe
    assert measured == pytest.approx(MARGIN_LOST_PER_COM_OFFSET, rel=0.02), (
        f"the constant says {MARGIN_LOST_PER_COM_OFFSET:.3f} mm per mm, the "
        f"geometry says {measured:.3f}")


def test_the_chosen_gait_clears_its_own_floor(scanned):
    """The claim the export now rests on: no accepted failure is needed."""

    plans, body_z, _, _ = scanned
    four = plan_four_legs_2d(plans, hybrid_timing_2d())
    body = body_trajectory_2d(four, nominal_body_z_m=body_z, samples=121)
    stability = swing_stability_2d(four, body,
                                   margin_floor_m=HYBRID_MARGIN_FLOOR_M)
    assert stability.is_stable
    assert not stability.unstable_swings
    assert stability.minimum_margin_m > HYBRID_MARGIN_FLOOR_M


def test_the_same_gait_does_not_clear_the_old_floor(scanned):
    """So the pass is the floor's doing, and that is on the record."""

    plans, body_z, _, _ = scanned
    four = plan_four_legs_2d(plans, hybrid_timing_2d())
    body = body_trajectory_2d(four, nominal_body_z_m=body_z, samples=121)
    stability = swing_stability_2d(four, body,
                                   margin_floor_m=DEFAULT_MARGIN_FLOOR_M)
    assert not stability.is_stable


# --------------------------------------------------------------------------
# The one thing Day 12 borrows from the Walk planner
# --------------------------------------------------------------------------


def test_the_only_shared_input_with_the_walk_planner_is_pinned():
    """Day 12 reads exactly two numbers out of ``GAIT_LIBRARY["Walk"]``.

    Nothing else in ``legwheel.planners`` reaches Day 12: the four legs, the
    crossing, the schedule and the motor export are all built here.  But these
    two are shared, and the whole of log section 1.6 rests on the duty being
    **exactly** the critical 3/4 -- that is what makes Step 6's 0.000 mm the
    textbook answer rather than a defect, and what makes
    ``HYBRID_STANCE_DUTY`` a departure from the project's gait rather than an
    arbitrary number.

    So this is a **contract with the Walk planner**, not a sanity check.  If it
    fails, the Walk gait was retuned -- which is a legitimate thing to do,
    especially since a Walk at duty 3/4 has a zero support margin for exactly
    the same reason a Hybrid does -- and then:

    * log 1.6's diagnosis needs re-reading against the new duty;
    * ``HYBRID_STANCE_DUTY`` should be reconsidered, because a Hybrid-vs-Walk
      comparison at two different duties is confounded;
    * the numbers in log 1.6's scan table were measured at 0.75 and stay
      measured at 0.75.

    Failing loudly here is the point: the alternative is Day 12 quietly
    re-deriving its conclusions from a gait it was never checked against.
    """

    from legwheel.planners.gait_generator_3d import GAIT_LIBRARY

    walk = GAIT_LIBRARY["Walk"]
    assert walk["stance_duty"] == CRITICAL_STANCE_DUTY, (
        f"the Walk gait's duty moved from {CRITICAL_STANCE_DUTY} to "
        f"{walk['stance_duty']}.  Day 12's A4 analysis (log 1.6) is written "
        "against 3/4 being the critical duty; re-read it before trusting "
        "HYBRID_STANCE_DUTY, and note that a Walk at any duty of 3/4 has a "
        "zero support margin for the same reason the Hybrid did.")
    assert list(walk["phase_offsets"]) == [0.75, 0.25, 0.5, 0.0], (
        f"the Walk gait's phase offsets moved to {walk['phase_offsets']}.  "
        "LIFTOFF_SEQUENCES['project_walk'] names the liftoff order these "
        "encode (LF RH RF LH), and the scan in log 1.6 found it is the only "
        "one of the six with a non-negative margin -- re-run that scan.")
