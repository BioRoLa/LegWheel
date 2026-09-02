"""Day 13: generator-frame trajectories, and the motor command they become.

The point of all of this is one measured fact: a trajectory assembled from
segment endpoints has **no theta motion at all** through a recovery, because a
recovery's two endpoints share a theta.  Commanding that would hold the leg
extended straight through the swing.  These tests pin that difference and the
refusals that stop it reaching hardware.
"""

import numpy as np
import pytest

from hybrid_note.scripts.experiments.day10_11_composer_2d import ComposedSequence2D
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import StrategyId
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    HIP_TO_BODY_Z_M,
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    LegId,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    NominalPosture2D,
    run_foot_rim_roll_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import walk_timing_2d
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    build_leg_plan_2d,
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    assemble_whole_body_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (
    MOTOR_MAX_RATE_RAD_S,
    CheckId,
    validate_whole_body_2d,
)
from hybrid_note.scripts.experiments.day13_motor_export_2d import (
    NotExportable,
    command_rows,
    motor_command_2d,
)

from dataclasses import replace


@pytest.fixture(scope="module")
def built():
    fixed = NominalPosture2D()
    hip_z = [f.hip_xz_m[1] for f in run_foot_rim_roll_2d(fixed).frames]
    held = float(max(hip_z))
    levelled = replace(fixed, hold_hip_z_m=held)
    composed = ComposedSequence2D(
        strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
        sequence=None, refusal="flat run, no crossing")
    plans = {leg: build_leg_plan_2d(leg, composed, posture=levelled,
                                    continuous_nominal=True)
             for leg in LEG_ORDER}
    four = plan_four_legs_2d(plans, walk_timing_2d())
    body = body_trajectory_2d(four, nominal_body_z_m=held - HIP_TO_BODY_Z_M,
                              samples=121)
    stability = swing_stability_2d(four, body)
    interpolated = assemble_whole_body_2d(four, body, stability, samples=121)
    frames = assemble_whole_body_2d(four, body, stability, samples=121,
                                    use_generator_frames=True)
    return {
        "plans": plans, "four": four, "body": body, "stability": stability,
        "interpolated": interpolated, "frames": frames,
        "report_i": validate_whole_body_2d(interpolated, body, stability),
        "report_f": validate_whole_body_2d(frames, body, stability),
    }


def _theta_span_deg(whole):
    values = np.array([leg.theta_rad for s in whole.samples
                       for leg in s.legs.values()])
    return float(np.rad2deg(values.max() - values.min()))


# --------------------------------------------------------------------------
# The frames are there, and they are the segments' own
# --------------------------------------------------------------------------


def test_the_plan_keeps_the_frames_its_segments_reference(built):
    plan = built["plans"][LegId.LF]
    assert plan.frames
    for phased in plan.phased:
        reference = phased.segment.frames
        frames = plan.frames[reference.source_id]
        assert max(reference.indices) < len(frames)


def test_the_registry_is_not_the_deduplicated_cycle_frames(built):
    """``NominalCycle2D.frames`` drops the recovery's first frame as a
    duplicate; the FrameRef indices keep it.  Using the short list would shift
    every recovery frame by one."""

    plan = built["plans"][LegId.LF]
    frames = next(iter(plan.frames.values()))
    recovery = next(p for p in plan.phased if p.kind.is_swing)
    first = frames[recovery.segment.frames.indices[0]]
    assert not first.airborne, "the recovery's first frame is the liftoff"


# --------------------------------------------------------------------------
# The difference the frames make
# --------------------------------------------------------------------------


def test_endpoint_interpolation_loses_the_whole_retraction(built):
    """The measurement this entire piece of work exists for.

    Not *exactly* zero: theta compensation holds the hip to 0.000176 mm rather
    than to nothing, so the segment endpoints differ by about 1e-5 deg.  That
    is the levelling residual, not motion -- the retraction it should be
    showing is 55 degrees.
    """

    span = _theta_span_deg(built["interpolated"])
    assert span < 1e-3, "the endpoints carry no retraction"
    assert _theta_span_deg(built["frames"]) > 5e4 * max(span, 1e-9)


def test_generator_frames_contain_the_retraction(built):
    span = _theta_span_deg(built["frames"])
    assert span > 50.0
    values = np.array([leg.theta_rad for s in built["frames"].samples
                       for leg in s.legs.values()])
    assert np.rad2deg(values.min()) == pytest.approx(17.0, abs=0.01), \
        "the leg reaches wheel mode"


def test_only_the_frame_built_trajectory_says_so(built):
    assert built["frames"].from_generator_frames
    assert not built["interpolated"].from_generator_frames


def test_the_real_motor_rate_is_far_above_what_the_endpoints_suggested(built):
    """The interpolated theta rate was a lower bound, and it was a big one.

    The margin used to be asserted as ">2x", against a real rate of 951 deg/s.
    That 951 was itself inflated: ``leg_sample_at`` snapped to the nearest
    generator frame, so the sampled signal was a staircase and differencing it
    reported ``frame step / sample interval`` rather than the motion (log 1.6,
    trap 61).  With the frames interpolated the real rate is 855 deg/s and the
    ratio is 1.83x -- the claim this test exists for is unchanged, but the
    number it was calibrated against was not a physical one.
    """

    interp = built["report_i"].joint_rates[0]
    real = built["report_f"].joint_rates[0]
    assert interp.peak_theta_rate_rad_s < np.deg2rad(0.1), \
        "the endpoints report essentially no theta motion"
    assert real.peak_theta_rate_rad_s > np.deg2rad(400.0)
    assert real.peak_motor_rate_rad_s > 1.5 * interp.peak_motor_rate_rad_s
    assert real.peak_motor_rate_rad_s < MOTOR_MAX_RATE_RAD_S


def test_the_theta_lower_bound_flag_clears_once_the_frames_are_used(built):
    assert built["report_i"].joint_rates[0].theta_rate_is_lower_bound
    assert not built["report_f"].joint_rates[0].theta_rate_is_lower_bound


def test_the_measured_peak_is_not_the_two_peaks_added(built):
    """Combining peaks that need not co-occur overstated this by 1429 vs 951."""

    rate = built["report_f"].joint_rates[0]
    assert rate.peak_motor_rate_rad_s < rate.peak_motor_rate_upper_bound_rad_s


# --------------------------------------------------------------------------
# A boundary frame is allowed to disagree with its segment's label
# --------------------------------------------------------------------------


def test_the_liftoff_frame_is_marked_as_a_boundary(built):
    boundary = [leg for s in built["frames"].samples
                for leg in s.legs.values() if leg.is_segment_boundary_frame]
    assert boundary


def test_a_boundary_frame_may_be_in_contact_on_an_airborne_segment(built):
    """It is the instant of liftoff, shared with the stance segment before it."""

    from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import LegMode

    odd = [leg for s in built["frames"].samples for leg in s.legs.values()
           if leg.mode is LegMode.AIRBORNE and leg.in_contact]
    assert all(leg.is_segment_boundary_frame for leg in odd)
    assert built["report_f"].failures_of(CheckId.STANCE_CONTACT_VALID) == ()


# --------------------------------------------------------------------------
# The export, and the two refusals
# --------------------------------------------------------------------------


def test_an_interpolated_trajectory_is_refused(built):
    with pytest.raises(NotExportable, match="interpolating segment endpoints"):
        motor_command_2d(built["interpolated"], built["report_i"],
                         accept_failures=(CheckId.SUPPORT_MARGIN,))


def test_an_unnamed_failing_check_is_refused(built):
    with pytest.raises(NotExportable, match="support_margin"):
        motor_command_2d(built["frames"], built["report_f"])


def test_naming_the_failure_is_what_allows_it(built):
    command = motor_command_2d(built["frames"], built["report_f"],
                               accept_failures=(CheckId.SUPPORT_MARGIN,))
    assert command.accepted_failures == ("support_margin",)


@pytest.fixture(scope="module")
def command(built):
    return motor_command_2d(built["frames"], built["report_f"],
                            accept_failures=(CheckId.SUPPORT_MARGIN,))


def test_the_command_is_four_rows_in_the_project_s_leg_order(command):
    assert command.theta_rad.shape[0] == 4
    assert command.beta_rad.shape == command.theta_rad.shape
    assert {leg.index for leg in LEG_ORDER} == {0, 1, 2, 3}


def test_the_hind_legs_are_not_swapped(built, command):
    """LEG_ORDER reads LF RF LH RH but indexes 0 1 3 2 (trap 19)."""

    sample = built["frames"].samples[0]
    for leg in LEG_ORDER:
        assert command.theta_rad[leg.index, 0] == pytest.approx(
            sample.legs[leg].theta_rad)


def test_the_command_stays_inside_the_motor_budget(command):
    assert command.peak_motor_rate_rad_s() < MOTOR_MAX_RATE_RAD_S
    assert command.as_dict()["motor_utilisation"] < 1.0


def test_the_playback_rate_is_reported_because_the_csv_has_no_time(command):
    row = command.as_dict()
    assert row["playback_hz"] > 0
    assert row["duration_s"] == pytest.approx(
        (command.sample_count - 1) * command.dt_s)


def test_every_leg_reaches_wheel_mode_and_the_rolling_posture(command):
    for leg in LEG_ORDER:
        row = command.theta_rad[leg.index]
        assert np.rad2deg(row.min()) == pytest.approx(17.0, abs=0.01)
        assert np.rad2deg(row.max()) > 70.0


def test_rows_are_csv_writable_with_one_header(command):
    rows = command_rows(command)
    assert all(len(r) == len(rows[0]) for r in rows)
    assert {r["row_kind"] for r in rows} == {"command", "leg"}
