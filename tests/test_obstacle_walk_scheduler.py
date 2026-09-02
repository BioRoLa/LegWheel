"""Step 6 tests for deterministic, event-aligned obstacle scheduling."""

from dataclasses import replace

import numpy as np
import pytest

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk import (
    FlatSegmentRequest,
    ObstacleScheduleError,
    RectangleObstacle1D,
    ScheduleRejectReason,
    ScheduleRequestKind,
    SwingSegmentRequest,
    WalkTerrain1D,
    generate_flat_walk_segment,
    query_touchdown_surface,
    schedule_obstacle_walk,
    schedule_to_dict,
    slice_segment,
)


@pytest.fixture(scope="module")
def step6_context():
    generator = GaitGenerator3D(
        stand_height=0.25,
        twist=[0.0, 0.05, 0.0],
        step_height=0.04,
        period=1.0,
        gait_type="Walk",
        dt=0.02,
        stability_margin=0.0,
    )
    flat = generate_flat_walk_segment(generator, n_cycles=2)
    initial_state = slice_segment(flat, 0, 51).final_state
    return generator, initial_state


def _terrain(x_start=0.65, length=0.35, edge_margin=0.02):
    return WalkTerrain1D(
        RectangleObstacle1D(x_start, length, 0.04, edge_margin)
    )


def test_schedule_is_deterministic_and_contains_only_requests(step6_context):
    generator, initial_state = step6_context
    terrain = _terrain()

    first = schedule_obstacle_walk(generator, initial_state, terrain)
    second = schedule_obstacle_walk(generator, initial_state, terrain)

    assert first == second
    assert isinstance(first.requests[0], FlatSegmentRequest)
    assert first.requests[0].kind is ScheduleRequestKind.FLAT_APPROACH
    assert first.requests[0].end_at_liftoff
    assert first.requests[-1].kind is ScheduleRequestKind.FLAT_RECOVERY
    assert all(isinstance(item, SwingSegmentRequest) for item in first.requests[1:-1])
    assert not first.kinematic_feasibility_checked
    assert not first.full_geometry_collision_checked


def test_event_order_comes_from_existing_walk_phase_and_never_cuts_mid_swing(
    step6_context,
):
    generator, initial_state = step6_context
    schedule = schedule_obstacle_walk(generator, initial_state, _terrain())
    swings = schedule.swing_requests

    event_indices = [item.event_index for item in swings]
    assert np.all(np.diff(event_indices) == 1)
    assert schedule.requests[0].end_event_leg is swings[0].leg
    assert schedule.requests[0].end_time_s == pytest.approx(swings[0].liftoff_time_s)
    assert all(item.touchdown_time_s > item.liftoff_time_s for item in swings)
    assert all(
        item.touchdown_sample_offset > item.liftoff_sample_offset
        for item in swings
    )

    # A cyclic rotation is expected because the obstacle can be reached at any
    # point in the original FL -> RR -> FR -> RL Walk event cycle.
    base_order = ("FL", "RR", "FR", "RL")
    observed = [item.leg.value for item in swings]
    start = base_order.index(observed[0])
    expected = [base_order[(start + index) % 4] for index in range(len(observed))]
    assert observed == expected


def test_changed_walk_phase_offsets_change_event_order(step6_context):
    _, initial_state = step6_context
    generator = GaitGenerator3D(
        stand_height=0.25,
        twist=[0.0, 0.05, 0.0],
        step_height=0.04,
        period=1.0,
        gait_type="Walk",
        dt=0.02,
        stability_margin=0.0,
    )
    generator.phase_offsets = [0.0, 0.75, 0.5, 0.25]
    schedule = schedule_obstacle_walk(generator, initial_state, _terrain())

    custom_order = ("FR", "RR", "RL", "FL")
    observed = [item.leg.value for item in schedule.swing_requests]
    start = custom_order.index(observed[0])
    expected = [
        custom_order[(start + index) % 4] for index in range(len(observed))
    ]
    assert observed == expected


def test_every_scheduled_touchdown_is_legal_and_edge_safe(step6_context):
    generator, initial_state = step6_context
    terrain = _terrain()
    schedule = schedule_obstacle_walk(generator, initial_state, terrain)

    for request in schedule.swing_requests:
        result = query_touchdown_surface(terrain, request.touchdown_world_m[0])
        assert result.is_legal
        assert result.surface_id == request.target_surface_id
        assert request.touchdown_world_m[2] == pytest.approx(
            result.surface_height_world_m
        )
        if result.status.value == "obstacle_top":
            assert request.touchdown_world_m[0] >= terrain.obstacle.legal_top_x_min_m
            assert request.touchdown_world_m[0] <= terrain.obstacle.legal_top_x_max_m


def test_obstacle_start_changes_flat_approach_but_keeps_event_alignment(step6_context):
    generator, initial_state = step6_context
    near = schedule_obstacle_walk(generator, initial_state, _terrain(x_start=0.55))
    far = schedule_obstacle_walk(generator, initial_state, _terrain(x_start=0.85))

    assert far.requests[0].duration_s > near.requests[0].duration_s
    assert near.requests[0].duration_s / generator.dt == pytest.approx(
        round(near.requests[0].duration_s / generator.dt)
    )
    assert far.requests[0].duration_s / generator.dt == pytest.approx(
        round(far.requests[0].duration_s / generator.dt)
    )


def test_edge_margin_can_be_repaired_with_bounded_touchdown_bias(step6_context):
    generator, initial_state = step6_context
    baseline = schedule_obstacle_walk(generator, initial_state, _terrain())
    first_nominal_x = baseline.swing_requests[0].nominal_touchdown_world_m[0]
    edge_terrain = _terrain(
        x_start=first_nominal_x - 0.01,
        length=0.35,
        edge_margin=0.02,
    )

    repaired = schedule_obstacle_walk(
        generator,
        initial_state,
        edge_terrain,
        maximum_touchdown_bias_m=0.02,
    )
    request = repaired.swing_requests[0]

    assert request.kind is ScheduleRequestKind.STEP_UP
    assert request.touchdown_bias_x_m == pytest.approx(0.01)
    assert request.touchdown_world_m[0] == pytest.approx(
        edge_terrain.obstacle.legal_top_x_min_m
    )


def test_edge_margin_failure_reports_exact_event_and_leg(step6_context):
    generator, initial_state = step6_context
    baseline = schedule_obstacle_walk(generator, initial_state, _terrain())
    first_nominal_x = baseline.swing_requests[0].nominal_touchdown_world_m[0]
    edge_terrain = _terrain(
        x_start=first_nominal_x - 0.01,
        length=0.35,
        edge_margin=0.02,
    )

    with pytest.raises(ObstacleScheduleError) as caught:
        schedule_obstacle_walk(
            generator,
            initial_state,
            edge_terrain,
            maximum_touchdown_bias_m=0.0,
        )

    error = caught.value
    assert error.reason is ScheduleRejectReason.NO_LEGAL_TOUCHDOWN
    assert error.event_index is not None
    assert error.leg is not None
    assert error.nominal_touchdown_x_world_m == pytest.approx(first_nominal_x)


def test_short_obstacle_can_be_cleared_without_four_top_contacts(step6_context):
    generator, initial_state = step6_context
    terrain = _terrain(x_start=0.635, length=0.01, edge_margin=0.002)
    schedule = schedule_obstacle_walk(generator, initial_state, terrain)

    assert not schedule.all_four_top_observed
    assert schedule.maximum_top_contact_count < 4
    assert any(
        item.kind is ScheduleRequestKind.CLEAR_OVER
        for item in schedule.swing_requests
    )


def test_long_obstacle_can_contain_all_four_top_contacts(step6_context):
    generator, initial_state = step6_context
    schedule = schedule_obstacle_walk(
        generator,
        initial_state,
        _terrain(x_start=0.45, length=0.80),
    )

    assert schedule.all_four_top_observed
    assert schedule.maximum_top_contact_count == 4
    assert any(item.kind is ScheduleRequestKind.STEP_UP for item in schedule.swing_requests)
    assert any(item.kind is ScheduleRequestKind.STEP_DOWN for item in schedule.swing_requests)


def test_recovery_is_rounded_up_to_complete_gait_cycles(step6_context):
    generator, initial_state = step6_context
    schedule = schedule_obstacle_walk(
        generator,
        initial_state,
        _terrain(),
        post_distance_m=0.26,
    )
    recovery = schedule.requests[-1]

    assert isinstance(recovery, FlatSegmentRequest)
    assert recovery.aligned_cycle_count == 6
    assert recovery.duration_s == pytest.approx(6.0)
    assert schedule.scheduled_recovery_distance_m == pytest.approx(0.30)


def test_phase_mismatch_and_unsupported_motion_are_rejected(step6_context):
    generator, initial_state = step6_context
    bad_phase = replace(
        initial_state,
        phase=np.array([1, 0, 0, 0]),
        contact_active=np.array([False, True, True, True]),
    )
    with pytest.raises(ObstacleScheduleError) as phase_error:
        schedule_obstacle_walk(generator, bad_phase, _terrain())
    assert phase_error.value.reason is ScheduleRejectReason.INVALID_INITIAL_STATE

    turning = GaitGenerator3D(
        stand_height=0.25,
        twist=[0.1, 0.05, 0.0],
        step_height=0.04,
        period=1.0,
        gait_type="Walk",
        dt=0.02,
        stability_margin=0.0,
    )
    with pytest.raises(ObstacleScheduleError) as motion_error:
        schedule_obstacle_walk(turning, initial_state, _terrain())
    assert motion_error.value.reason is ScheduleRejectReason.UNSUPPORTED_MOTION


def test_event_budget_failure_is_explicit(step6_context):
    generator, initial_state = step6_context
    with pytest.raises(ObstacleScheduleError) as caught:
        schedule_obstacle_walk(
            generator,
            initial_state,
            _terrain(x_start=2.0),
            maximum_events=4,
        )

    assert caught.value.reason is ScheduleRejectReason.NO_OBSTACLE_INTERACTION


def test_json_safe_summary_keeps_prototype_validation_flags(step6_context):
    generator, initial_state = step6_context
    schedule = schedule_obstacle_walk(generator, initial_state, _terrain())
    summary = schedule_to_dict(schedule)

    assert summary["status"] == "deterministic segment-request schedule prototype"
    assert summary["kinematic_feasibility_checked"] is False
    assert summary["full_geometry_collision_checked"] is False
    assert summary["requests"][0]["kind"] == "flat_approach"
    assert summary["requests"][-1]["kind"] == "flat_recovery"
