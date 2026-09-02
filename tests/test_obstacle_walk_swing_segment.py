"""Step 4 tests for arbitrary-height, single-leg Walk swing segments."""

from dataclasses import replace

import numpy as np
import pytest

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk import (
    LegId,
    RectangleObstacle1D,
    SegmentType,
    SwingPlanningError,
    SwingRejectReason,
    WalkTerrain1D,
    concatenate_segments,
    generate_flat_walk_segment,
    generate_swing_segment,
    plot_swing_plan,
    slice_segment,
)


HEIGHT_M = 0.04
CLEARANCE_M = 0.03


@pytest.fixture(scope="module")
def step4_context():
    generator = GaitGenerator3D(
        stand_height=0.25,
        twist=[0.0, 0.05, 0.0],
        step_height=0.04,
        period=1.0,
        gait_type="Walk",
        dt=0.02,
        stability_margin=0.0,
    )
    complete_flat = generate_flat_walk_segment(generator, n_cycles=2)
    flat_before_swing = slice_segment(complete_flat, 0, 51)
    ground_state = flat_before_swing.final_state

    body_top = ground_state.body_pose_world.copy()
    body_top[2] += HEIGHT_M
    feet_top = ground_state.foot_contact_points_world_m.copy()
    feet_top[:, 2] += HEIGHT_M
    top_state = replace(
        ground_state,
        body_pose_world=body_top,
        foot_contact_points_world_m=feet_top,
        surface_ids=("obstacle_top",) * 4,
    )
    return generator, flat_before_swing, ground_state, top_state


def _flat_case(ground_state):
    foot = ground_state.foot_contact_points_world_m[0]
    terrain = WalkTerrain1D(
        RectangleObstacle1D(foot[0] + 0.30, 0.20, HEIGHT_M, 0.02)
    )
    target = np.array([foot[0] + 0.08, foot[1], 0.0])
    return terrain, target


def _step_up_case(ground_state):
    foot = ground_state.foot_contact_points_world_m[0]
    terrain = WalkTerrain1D(
        RectangleObstacle1D(foot[0] + 0.04, 0.25, HEIGHT_M, 0.02)
    )
    target = np.array([foot[0] + 0.10, foot[1], HEIGHT_M])
    return terrain, target


def _top_case(top_state):
    foot = top_state.foot_contact_points_world_m[0]
    terrain = WalkTerrain1D(RectangleObstacle1D(-0.30, 0.85, HEIGHT_M, 0.02))
    target = np.array([foot[0] + 0.08, foot[1], HEIGHT_M])
    return terrain, target


def _step_down_case(top_state):
    foot = top_state.foot_contact_points_world_m[0]
    obstacle_end = foot[0] + 0.03
    terrain = WalkTerrain1D(
        RectangleObstacle1D(-0.30, obstacle_end + 0.30, HEIGHT_M, 0.01)
    )
    target = np.array([foot[0] + 0.08, foot[1], 0.0])
    return terrain, target


@pytest.mark.parametrize(
    ("state_name", "case_factory", "expected_surface"),
    [
        ("ground", _flat_case, "ground"),
        ("ground", _step_up_case, "obstacle_top"),
        ("top", _top_case, "obstacle_top"),
        ("top", _step_down_case, "ground"),
    ],
)
def test_four_start_end_height_combinations_are_kinematically_inspectable(
    step4_context,
    state_name,
    case_factory,
    expected_surface,
):
    generator, _, ground_state, top_state = step4_context
    state = ground_state if state_name == "ground" else top_state
    terrain, target = case_factory(state)

    plan = generate_swing_segment(
        generator,
        state,
        LegId.FL,
        target,
        terrain,
        CLEARANCE_M,
    )

    assert plan.segment.segment_type is SegmentType.SWING
    assert plan.segment.swing_leg is LegId.FL
    assert plan.target_surface_id == expected_surface
    assert plan.maximum_tracking_error_m < 1e-3
    assert plan.achieved_apex_height_world_m >= plan.requested_apex_height_world_m - 1e-3
    np.testing.assert_allclose(plan.cartesian_actual_world_m[-1], target, atol=1e-3)
    np.testing.assert_array_equal(plan.segment.commands_rad[0], state.joint_position_rad)
    np.testing.assert_array_equal(plan.segment.phase[0], np.zeros(4))
    np.testing.assert_array_equal(plan.segment.phase[-1], np.zeros(4))
    np.testing.assert_array_equal(plan.segment.contact_active, plan.segment.phase == 0)
    assert not plan.full_geometry_collision_checked

    # Step 4 freezes the body and the three support-leg commands by design.
    np.testing.assert_array_equal(
        plan.segment.body_pose_world,
        np.repeat(state.body_pose_world[np.newaxis, :], plan.segment.sample_count, axis=0),
    )
    for support_leg in (1, 2, 3):
        np.testing.assert_array_equal(
            plan.segment.commands_rad[:, support_leg],
            np.repeat(
                state.joint_position_rad[support_leg][np.newaxis, :],
                plan.segment.sample_count,
                axis=0,
            ),
        )


def test_step_up_preserves_step2_shared_boundary_contract(step4_context):
    generator, flat_before_swing, ground_state, _ = step4_context
    terrain, target = _step_up_case(ground_state)
    plan = generate_swing_segment(
        generator,
        ground_state,
        LegId.FL,
        target,
        terrain,
        CLEARANCE_M,
    )

    result = concatenate_segments([flat_before_swing, plan.segment])
    report = result.boundary_reports[0]

    assert report.passed
    assert report.joint_position_max_rad == pytest.approx(0.0)
    np.testing.assert_allclose(
        (plan.segment.commands_rad[1, 0] - plan.segment.commands_rad[0, 0])
        / plan.segment.dt_s,
        (flat_before_swing.commands_rad[-1, 0] - flat_before_swing.commands_rad[-2, 0])
        / flat_before_swing.dt_s,
    )
    np.testing.assert_allclose(
        plan.segment.commands_rad[-1, 0] - plan.segment.commands_rad[-2, 0],
        0.0,
    )


def test_step_up_uses_terrain_aware_apex(step4_context):
    generator, _, ground_state, _ = step4_context
    terrain, target = _step_up_case(ground_state)
    plan = generate_swing_segment(
        generator,
        ground_state,
        LegId.FL,
        target,
        terrain,
        CLEARANCE_M,
    )

    assert plan.terrain_max_height_world_m == pytest.approx(HEIGHT_M)
    assert plan.requested_apex_height_world_m == pytest.approx(HEIGHT_M + CLEARANCE_M)


def test_illegal_edge_margin_touchdown_is_rejected(step4_context):
    generator, _, ground_state, _ = step4_context
    foot = ground_state.foot_contact_points_world_m[0]
    obstacle = RectangleObstacle1D(foot[0] + 0.04, 0.25, HEIGHT_M, 0.02)
    terrain = WalkTerrain1D(obstacle)
    target = np.array([obstacle.x_start_m + 0.01, foot[1], HEIGHT_M])

    with pytest.raises(SwingPlanningError) as caught:
        generate_swing_segment(generator, ground_state, "FL", target, terrain, CLEARANCE_M)

    assert caught.value.reason is SwingRejectReason.ILLEGAL_TOUCHDOWN


def test_target_height_must_match_queried_surface(step4_context):
    generator, _, ground_state, _ = step4_context
    terrain, target = _step_up_case(ground_state)
    target[2] = 0.0

    with pytest.raises(SwingPlanningError) as caught:
        generate_swing_segment(generator, ground_state, "FL", target, terrain, CLEARANCE_M)

    assert caught.value.reason is SwingRejectReason.TARGET_HEIGHT_MISMATCH


def test_unreachable_touchdown_has_clear_ik_reason(step4_context):
    generator, _, ground_state, _ = step4_context
    foot = ground_state.foot_contact_points_world_m[0]
    terrain = WalkTerrain1D(RectangleObstacle1D(foot[0] + 2.0, 0.20, HEIGHT_M, 0.02))
    target = np.array([foot[0] + 1.0, foot[1], 0.0])

    with pytest.raises(SwingPlanningError) as caught:
        generate_swing_segment(generator, ground_state, "FL", target, terrain, CLEARANCE_M)

    assert caught.value.reason is SwingRejectReason.IK_UNREACHABLE


def test_contact_path_clearance_failure_is_rejected(step4_context):
    generator, _, ground_state, _ = step4_context
    terrain, target = _step_up_case(ground_state)

    with pytest.raises(SwingPlanningError) as caught:
        generate_swing_segment(
            generator,
            ground_state,
            "FL",
            target,
            terrain,
            CLEARANCE_M,
            contact_path_clearance_tolerance_m=0.05,
        )

    assert caught.value.reason is SwingRejectReason.CONTACT_PATH_COLLISION
    assert caught.value.sample_index is not None


def test_optional_full_geometry_checker_can_reject_or_certify(step4_context):
    generator, _, ground_state, _ = step4_context
    terrain, target = _step_up_case(ground_state)

    def reject_sample_four(**kwargs):
        return "forced full-leg collision" if kwargs["sample_index"] == 4 else None

    with pytest.raises(SwingPlanningError) as caught:
        generate_swing_segment(
            generator,
            ground_state,
            "FL",
            target,
            terrain,
            CLEARANCE_M,
            full_geometry_collision_checker=reject_sample_four,
        )
    assert caught.value.reason is SwingRejectReason.FULL_GEOMETRY_COLLISION
    assert caught.value.sample_index == 4

    plan = generate_swing_segment(
        generator,
        ground_state,
        "FL",
        target,
        terrain,
        CLEARANCE_M,
        full_geometry_collision_checker=lambda **_: None,
    )
    assert plan.full_geometry_collision_checked


def test_step_up_and_step_down_plots_are_inspectable(step4_context, tmp_path):
    generator, _, ground_state, top_state = step4_context
    for label, state, case_factory in (
        ("step_up", ground_state, _step_up_case),
        ("step_down", top_state, _step_down_case),
    ):
        terrain, target = case_factory(state)
        plan = generate_swing_segment(
            generator,
            state,
            "FL",
            target,
            terrain,
            CLEARANCE_M,
        )
        output = plot_swing_plan(plan, terrain, tmp_path / f"{label}.png")
        assert output.is_file()
        assert output.stat().st_size > 10_000
