"""The flat-Walk approach splice: exact boundaries, and a launch from rest."""

from __future__ import annotations

import contextlib
import io

import numpy as np
import pytest

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk import (
    FlatHandoffError,
    LEG_ORDER,
    ObstacleWalkRequest,
    cycle_sample_count,
    flat_approach_segment,
    generate_obstacle_walk,
    handover_contact_height_error_m,
    launch_sample_index,
    walk_swing_order,
)


STAND_HEIGHT_M = 0.25
PERIOD_S = 2.0
DT_S = 0.02
VELOCITY_X_M_S = 0.1


def build_generator() -> GaitGenerator3D:
    with contextlib.redirect_stdout(io.StringIO()):
        return GaitGenerator3D(
            stand_height=STAND_HEIGHT_M,
            twist=[0.0, VELOCITY_X_M_S, 0.0],
            step_height=0.02,
            period=PERIOD_S,
            gait_type="Walk",
            dt=DT_S,
            stability_margin=0.0,
        )


def build_approach(cycles: int = 2, launch_cycles: int = 1):
    generator = build_generator()
    segment = flat_approach_segment(
        generator,
        cycles=cycles,
        launch_cycles=launch_cycles,
        handover_body_x_m=0.0,
        body_y_m=0.0,
        stand_height_m=STAND_HEIGHT_M,
        ground_height_m=0.0,
        ground_surface_id="ground",
        expected_first_swing_leg=walk_swing_order(generator)[0],
    )
    return generator, segment


def test_walk_has_no_all_stance_sample() -> None:
    """The premise the whole handoff design rests on."""

    generator = build_generator()
    generator.generate_full_gait(n_cycles=1)
    phase = np.asarray(generator.PHASE)
    assert np.all(phase.sum(axis=1) == 1)


def test_launch_index_starts_at_rest_and_reaches_nominal_rate() -> None:
    index = launch_sample_index(40)
    slope = np.diff(index)
    assert index[0] == 0.0
    assert index[-1] == pytest.approx(20.0)
    assert slope[0] == pytest.approx(0.0, abs=1e-3)
    assert slope[-1] == pytest.approx(1.0, rel=2e-2)
    assert np.all(np.diff(index) >= -1e-12), "the warp must not run the Walk backwards"


def test_launch_index_rejects_odd_sample_counts() -> None:
    with pytest.raises(FlatHandoffError):
        launch_sample_index(41)


def test_approach_ends_at_the_requested_handover_x() -> None:
    generator, segment = build_approach(cycles=2, launch_cycles=1)
    advance = VELOCITY_X_M_S * PERIOD_S * 2
    assert segment.body_pose_world[-1, 0] == pytest.approx(0.0, abs=1e-12)
    assert segment.body_pose_world[0, 0] == pytest.approx(-advance, abs=1e-12)
    # Re-timing changes the clock, never the distance the Walk covers.
    assert segment.body_pose_world[-1, 0] - segment.body_pose_world[0, 0] == pytest.approx(
        advance, abs=1e-12
    )


def test_launch_adds_exactly_its_own_cycles_of_wall_clock() -> None:
    _, plain = build_approach(cycles=2, launch_cycles=0)
    _, launched = build_approach(cycles=2, launch_cycles=1)
    extra = launched.sample_count - plain.sample_count
    assert extra == cycle_sample_count(build_generator())


def test_launch_starts_from_rest_and_recovers_the_nominal_rate() -> None:
    _, launched = build_approach(cycles=2, launch_cycles=1)
    _, plain = build_approach(cycles=2, launch_cycles=0)
    speed = np.abs(np.diff(launched.commands_rad, axis=0)) / DT_S
    nominal = float(np.max(np.abs(np.diff(plain.commands_rad, axis=0)) / DT_S))

    assert float(speed[0].max()) < 0.02 * nominal
    # The final row is a liftoff instant, where the Walk's own joint rates are
    # low, so "back to nominal" is a claim about the peak of the un-warped
    # stretch, not about the handover sample itself.
    warp_rows = cycle_sample_count(build_generator()) * 2
    assert float(speed[warp_rows:].max()) == pytest.approx(nominal, rel=1e-9)
    assert float(speed[:warp_rows].max()) < nominal


def test_handover_row_is_all_stance_and_on_the_ground() -> None:
    _, segment = build_approach()
    assert np.all(segment.phase[-1] == 0)
    assert np.all(segment.contact_active[-1])
    # Every other row still carries exactly one swinging leg: it is the Walk.
    assert np.all(segment.phase[:-1].sum(axis=1) == 1)
    assert handover_contact_height_error_m(segment, 0.0) < 1e-3


def test_handover_leg_matches_the_crawl_leg_order() -> None:
    generator, segment = build_approach()
    assert segment.final_state.next_swing_leg is walk_swing_order(generator)[0]


def test_mismatched_first_swing_leg_is_rejected() -> None:
    generator = build_generator()
    wrong = next(leg for leg in LEG_ORDER if leg is not walk_swing_order(generator)[0])
    with pytest.raises(FlatHandoffError, match="reorder the gait"):
        flat_approach_segment(
            generator,
            cycles=1,
            launch_cycles=0,
            handover_body_x_m=0.0,
            body_y_m=0.0,
            stand_height_m=STAND_HEIGHT_M,
            ground_height_m=0.0,
            ground_surface_id="ground",
            expected_first_swing_leg=wrong,
        )


def test_launch_cycles_may_span_the_whole_approach() -> None:
    _, segment = build_approach(cycles=1, launch_cycles=1)
    speed = np.abs(np.diff(segment.commands_rad, axis=0)) / DT_S
    assert float(speed[0].max()) < 1e-3
    assert np.all(segment.phase[:-1].sum(axis=1) == 1)


def test_original_launch_cycles_are_prepended_to_steady_approach() -> None:
    """Legacy mode walks *extra* reduced-speed cycles beyond the steady ones."""

    request = ObstacleWalkRequest(
        obstacle_x_start_m=1.0,
        obstacle_length_m=0.4,
        obstacle_height_m=0.04,
        flat_approach_cycles=2,
        flat_launch_cycles=3,
        flat_launch_mode="legacy",
    )
    assert request.flat_launch_cycles > request.flat_approach_cycles


def test_timewarp_mode_rejects_a_launch_longer_than_the_approach() -> None:
    """The warp re-times cycles the approach already counts, so it cannot exceed them."""

    with pytest.raises(ValueError, match="flat_launch_cycles"):
        ObstacleWalkRequest(
            obstacle_x_start_m=1.0,
            obstacle_length_m=0.4,
            obstacle_height_m=0.04,
            flat_approach_cycles=2,
            flat_launch_cycles=3,
        )


def test_legacy_launch_steps_the_joints_at_every_ramp_cycle_boundary() -> None:
    """Why ``timewarp`` is the default.

    ``LaunchController`` splices whole Walk cycles of different stride end to
    end, so the joints kink where two ramp cycles meet.  It is a position
    discontinuity, so it does not shrink with dt: the same radians land in
    whatever sample period the planner uses, and the exporter then resamples it
    to the controller's 1 ms.

    Its size scales with the stride difference between consecutive ramp cycles,
    so it grows with the period and shrinks with more ramp cycles -- measured
    second-difference ratios run from 2.2x at ``T=2 s, n_ramp=3`` to 13.5x at
    ``T=4 s, n_ramp=2``.  The per-sample step alone does not reveal it at the
    milder settings, because the Walk's own swing takes comparable steps; the
    second difference is what separates a kink from ordinary motion.

    This pins the defect rather than the fix, so it stays visible if anyone
    makes the legacy launch the default again.
    """

    from legwheel.planners.launch_controller import LaunchController

    n_ramp = 3
    with contextlib.redirect_stdout(io.StringIO()):
        launcher = LaunchController(
            gait_type="Walk",
            stand_height=STAND_HEIGHT_M,
            twist=[0.0, VELOCITY_X_M_S, 0.0],
            step_height=0.04,
            period=PERIOD_S,
            dt=DT_S,
            n_ramp=n_ramp,
            ramp_floor=0.1,
            stability_margin=0.0,
            stance_duty=0.75,
        )
        commands, _phase = launcher._generate_launch_sequence_with_phase()

    commands = np.asarray(commands, dtype=float)
    per_cycle = len(commands) // n_ramp
    kink = np.abs(np.diff(commands, n=2, axis=0))
    # A second difference straddles the splice from either side, so both rows
    # around each ramp-cycle boundary carry it.
    boundaries = [k * per_cycle - offset for k in range(1, n_ramp) for offset in (1, 2)]
    interior = np.delete(np.arange(len(kink)), boundaries)
    assert kink[boundaries].max() > 2.0 * kink[interior].max()


def test_timewarp_launch_has_no_such_boundary_step() -> None:
    _, segment = build_approach(cycles=2, launch_cycles=1)
    step = np.abs(np.diff(segment.commands_rad, axis=0))
    _, plain = build_approach(cycles=2, launch_cycles=0)
    nominal = float(np.max(np.abs(np.diff(plain.commands_rad, axis=0))))
    # Re-timing can only slow the Walk down, never command a larger step than
    # the Walk itself takes at nominal rate.
    assert float(step.max()) <= nominal * (1.0 + 1e-9)


def test_spliced_traversal_keeps_every_boundary_exact() -> None:
    request = ObstacleWalkRequest(
        obstacle_x_start_m=1.0,
        obstacle_length_m=0.4,
        obstacle_height_m=0.04,
        step_length_m=VELOCITY_X_M_S * PERIOD_S * 0.75,
        period_s=PERIOD_S,
        dt_s=DT_S,
        step_clearance_m=0.02,
        approach_distance_m=1.0,
        post_distance_m=0.3,
        flat_approach_cycles=3,
        flat_launch_cycles=1,
        flat_recovery_cycles=3,
        flat_landing_cycles=1,
    )
    result = generate_obstacle_walk(request)
    assert result.traversal_completed
    assert result.records[0].kind == "flat_walk"
    assert result.records[-1].kind == "flat_walk_recovery"
    assert result.recovery_handover_joint_snap_rad is not None
    assert result.recovery_handover_joint_snap_rad < 0.02
    assert result.recovery_handover_contact_snap_m is not None
    assert result.recovery_handover_contact_snap_m < 0.006
    assert result.maximum_boundary_joint_position_error_rad == 0.0
    assert result.maximum_boundary_joint_velocity_error_rad_s < 0.5
    # The default launch is the time warp, whose whole purpose is that the
    # trajectory leaves the controller's trigger row at rest: the robot is
    # stationary while it waits for the trigger, so a first frame that already
    # moves would demand a velocity step no prep ramp can absorb.
    first_step = np.abs(result.segment.commands_rad[1] - result.segment.commands_rad[0])
    assert float(first_step.max()) / DT_S < 0.05
