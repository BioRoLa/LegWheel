"""Step 5 tests for world-fixed, mixed-height Walk stance segments."""

from dataclasses import replace

import numpy as np
import pytest

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk import (
    BODY_TRAJECTORY_ASSUMPTION,
    LegId,
    RectangleObstacle1D,
    SegmentType,
    StancePlanningError,
    StanceRejectReason,
    WalkTerrain1D,
    concatenate_segments,
    generate_flat_walk_segment,
    generate_stance_segment,
    generate_swing_segment,
    plot_stance_plan,
    slice_segment,
)


HEIGHT_M = 0.04


@pytest.fixture(scope="module")
def step5_context():
    generator = GaitGenerator3D(
        stand_height=0.25,
        twist=[0.0, 0.05, 0.0],
        step_height=0.04,
        period=1.0,
        gait_type="Walk",
        dt=0.02,
        stability_margin=0.0,
    )
    flat = slice_segment(generate_flat_walk_segment(generator, n_cycles=2), 0, 51)
    ground = flat.final_state

    front_left = ground.foot_contact_points_world_m[0]
    mixed_terrain = WalkTerrain1D(
        RectangleObstacle1D(front_left[0] + 0.04, 0.25, HEIGHT_M, 0.02)
    )
    fl_target = np.array([front_left[0] + 0.10, front_left[1], HEIGHT_M])
    step_up_fl = generate_swing_segment(
        generator,
        ground,
        LegId.FL,
        fl_target,
        mixed_terrain,
        clearance_m=0.03,
    )
    one_top = step_up_fl.segment.final_state

    front_right = one_top.foot_contact_points_world_m[1]
    fr_target = np.array(
        [mixed_terrain.obstacle.legal_top_x_min_m + 0.01, front_right[1], HEIGHT_M]
    )
    step_up_fr = generate_swing_segment(
        generator,
        one_top,
        LegId.FR,
        fr_target,
        mixed_terrain,
        clearance_m=0.03,
    )
    two_top = step_up_fr.segment.final_state

    body_top = ground.body_pose_world.copy()
    body_top[2] += HEIGHT_M
    feet_top = ground.foot_contact_points_world_m.copy()
    feet_top[:, 2] += HEIGHT_M
    four_top = replace(
        ground,
        body_pose_world=body_top,
        foot_contact_points_world_m=feet_top,
        surface_ids=("obstacle_top",) * 4,
    )
    x_min = float(np.min(feet_top[:, 0])) - 0.10
    x_max = float(np.max(feet_top[:, 0])) + 0.10
    top_terrain = WalkTerrain1D(
        RectangleObstacle1D(x_min, x_max - x_min, HEIGHT_M, 0.02)
    )
    ground_terrain = WalkTerrain1D(
        RectangleObstacle1D(front_left[0] + 1.0, 0.20, HEIGHT_M, 0.02)
    )
    return {
        "generator": generator,
        "flat": flat,
        "ground": ground,
        "ground_terrain": ground_terrain,
        "step_up_fl": step_up_fl,
        "one_top": one_top,
        "two_top": two_top,
        "mixed_terrain": mixed_terrain,
        "four_top": four_top,
        "top_terrain": top_terrain,
    }


@pytest.mark.parametrize(
    ("state_name", "terrain_name", "top_count"),
    [
        ("ground", "ground_terrain", 0),
        ("one_top", "mixed_terrain", 1),
        ("two_top", "mixed_terrain", 2),
        ("four_top", "top_terrain", 4),
    ],
)
def test_ground_and_mixed_height_stance_contacts_remain_world_fixed(
    step5_context,
    state_name,
    terrain_name,
    top_count,
):
    generator = step5_context["generator"]
    state = step5_context[state_name]
    terrain = step5_context[terrain_name]
    target_body = state.body_pose_world.copy()
    target_body[0] += 0.01
    target_body[2] += 0.003
    target_body[4] += np.deg2rad(0.5)

    plan = generate_stance_segment(
        generator,
        state,
        target_body,
        terrain,
        motion_duration_s=0.30,
    )

    assert plan.segment.segment_type is SegmentType.STANCE
    assert plan.body_trajectory_assumption == BODY_TRAJECTORY_ASSUMPTION
    assert plan.maximum_contact_drift_m < 1e-3
    assert sum(item == "obstacle_top" for item in state.surface_ids) == top_count
    np.testing.assert_allclose(plan.segment.body_pose_world[-1], target_body)
    np.testing.assert_array_equal(plan.segment.phase, np.zeros_like(plan.segment.phase))
    np.testing.assert_array_equal(
        plan.segment.surface_ids,
        np.repeat(np.asarray(state.surface_ids)[None, :], plan.segment.sample_count, axis=0),
    )
    np.testing.assert_allclose(
        plan.contact_actual_world_m,
        np.repeat(
            plan.contact_targets_world_m[None, :, :],
            plan.segment.sample_count,
            axis=0,
        ),
        atol=1e-3,
    )
    # Endpoint holds make discrete entry and exit command velocities zero.
    np.testing.assert_array_equal(plan.segment.commands_rad[1], plan.segment.commands_rad[0])
    np.testing.assert_array_equal(plan.segment.commands_rad[-1], plan.segment.commands_rad[-2])
    assert not plan.full_geometry_collision_checked


def test_step4_swing_to_step5_stance_boundary_is_continuous(step5_context):
    generator = step5_context["generator"]
    swing = step5_context["step_up_fl"].segment
    state = swing.final_state
    target_body = state.body_pose_world.copy()
    target_body[0] += 0.005

    stance = generate_stance_segment(
        generator,
        state,
        target_body,
        step5_context["mixed_terrain"],
        motion_duration_s=0.30,
    ).segment
    assembled = concatenate_segments([swing, stance])

    assert assembled.boundary_reports[0].passed
    assert assembled.boundary_reports[0].joint_position_max_rad == pytest.approx(0.0)
    assert assembled.boundary_reports[0].support_foot_position_max_m == pytest.approx(0.0)


def test_infeasible_mixed_height_body_pose_is_rejected(step5_context):
    state = step5_context["one_top"]
    target_body = state.body_pose_world.copy()
    target_body[2] += 0.50

    with pytest.raises(StancePlanningError) as caught:
        generate_stance_segment(
            step5_context["generator"],
            state,
            target_body,
            step5_context["mixed_terrain"],
            motion_duration_s=0.30,
        )

    assert caught.value.reason in {
        StanceRejectReason.IK_UNREACHABLE,
        StanceRejectReason.JOINT_LIMIT,
    }


def test_joint_limit_checker_has_explicit_reason(step5_context, monkeypatch):
    import legwheel.planners.obstacle_walk.stance as stance_module

    state = step5_context["ground"]
    monkeypatch.setattr(stance_module.RobotParams, "MAX_THETA_DEG", 100.0)

    with pytest.raises(StancePlanningError) as caught:
        generate_stance_segment(
            step5_context["generator"],
            state,
            state.body_pose_world,
            step5_context["ground_terrain"],
            motion_duration_s=0.10,
        )

    assert caught.value.reason is StanceRejectReason.JOINT_LIMIT
    assert caught.value.leg_index is not None


def test_optional_full_geometry_checker_can_reject_or_certify(step5_context):
    state = step5_context["two_top"]

    def reject_rr_at_three(**kwargs):
        if kwargs["sample_index"] == 3 and kwargs["leg"] is LegId.RR:
            return "forced linkage collision"
        return None

    with pytest.raises(StancePlanningError) as caught:
        generate_stance_segment(
            step5_context["generator"],
            state,
            state.body_pose_world,
            step5_context["mixed_terrain"],
            motion_duration_s=0.10,
            full_geometry_collision_checker=reject_rr_at_three,
        )
    assert caught.value.reason is StanceRejectReason.FULL_GEOMETRY_COLLISION
    assert caught.value.sample_index == 3
    assert caught.value.leg_index == 2

    plan = generate_stance_segment(
        step5_context["generator"],
        state,
        state.body_pose_world,
        step5_context["mixed_terrain"],
        motion_duration_s=0.10,
        full_geometry_collision_checker=lambda **_: None,
    )
    assert plan.full_geometry_collision_checked


def test_mixed_height_stance_plot_is_inspectable(step5_context, tmp_path):
    state = step5_context["two_top"]
    target_body = state.body_pose_world.copy()
    target_body[0] += 0.01
    plan = generate_stance_segment(
        step5_context["generator"],
        state,
        target_body,
        step5_context["mixed_terrain"],
        motion_duration_s=0.30,
    )

    output = plot_stance_plan(plan, step5_context["mixed_terrain"], tmp_path / "stance.png")
    assert output.is_file()
    assert output.stat().st_size > 10_000
