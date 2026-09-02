import matplotlib.pyplot as plt
import numpy as np
import pytest
from pathlib import Path

from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (
    airborne_retract_reset_comparison_rows,
    airborne_retract_reset_frame_rows,
    build_single_leg_rolling_scene_2d,
    evaluate_right_rim_retract_readiness_2d,
    forward_rolling_frame_rows,
    forward_rolling_rows,
    plot_single_leg_rolling_scene_2d,
    plot_right_rim_roll_up_trajectory_2d,
    plot_right_rim_theta_feasibility_2d,
    right_rim_roll_up_rows,
    right_rim_theta_sweep_rows,
    run_fixed_theta_right_rim_roll_up_2d,
    run_forward_right_rim_roll_up_2d,
    run_retract_to_wheel_2d,
    run_retract_and_reset_comparison_2d,
    run_airborne_retract_and_foot_reset_comparison_2d,
    run_wheel_reset_roll_2d,
    simulate_theta_candidates_forward_rolling_2d,
    sweep_right_rim_roll_up_theta_2d,
    write_right_rim_theta_sweep_csv,
    wheel_reset_roll_rows,
    write_wheel_reset_roll_csv,
    retract_reset_comparison_rows,
    retract_reset_frame_rows,
    write_retract_reset_comparison_csv,
    write_airborne_retract_reset_comparison_csv,
)


def test_fixed_pose_scene_reuses_rim_geometry_and_rectangle_parameters():
    scene = build_single_leg_rolling_scene_2d(
        theta_rad=np.deg2rad(40.0),
        beta_rad=np.deg2rad(5.0),
        hip_x_m=0.35,
        hip_z_m=0.24,
        obstacle_x_start_m=0.10,
        obstacle_width_m=0.22,
        obstacle_height_m=0.06,
        arc_samples=31,
    )

    assert scene.gamma_rad == 0.0
    assert scene.obstacle_x_start_m == pytest.approx(0.10)
    assert scene.obstacle_width_m == pytest.approx(0.22)
    assert scene.obstacle_top_length_m == pytest.approx(0.22)
    assert scene.obstacle_height_m == pytest.approx(0.06)
    assert set(scene.geometry.surface_names) == {"foot_rim", "upper_tyre_l", "upper_tyre_r"}
    assert scene.geometry.points_world_xz_m.shape == (3 * 31, 2)
    np.testing.assert_allclose(
        scene.geometry.points_world_xz_m[0],
        scene.geometry.points_hip_xz_m[0] + [0.35, 0.24],
    )


def test_fixed_pose_scene_supports_flat_ground_only():
    scene = build_single_leg_rolling_scene_2d(
        theta_rad=np.deg2rad(17.0),
        beta_rad=0.0,
        hip_x_m=0.0,
        hip_z_m=0.25,
        obstacle_x_start_m=None,
    )

    assert scene.terrain.obstacle is None
    assert scene.obstacle_top_length_m is None


def test_step1_rejects_nonzero_gamma():
    with pytest.raises(ValueError, match="gamma = 0"):
        build_single_leg_rolling_scene_2d(
            theta_rad=np.deg2rad(40.0),
            beta_rad=0.0,
            hip_x_m=0.0,
            hip_z_m=0.25,
            gamma_rad=np.deg2rad(1.0),
        )


def test_scene_plot_contains_terrain_rim_and_hip_artists():
    scene = build_single_leg_rolling_scene_2d(
        theta_rad=np.deg2rad(40.0),
        beta_rad=0.0,
        hip_x_m=0.0,
        hip_z_m=0.24,
        arc_samples=21,
    )
    fig, ax = plt.subplots()
    assert plot_single_leg_rolling_scene_2d(scene, ax=ax) is ax
    labels = {handle.get_label() for handle in ax.get_legend().legend_handles}
    assert "ground" in labels
    assert "obstacle top" in labels
    assert "foot rim (F)" in labels
    assert "right upper tyre (R)" in labels
    assert "hip H" in labels
    plt.close(fig)


def test_step3_fixed_theta_roll_up_reaches_top_by_updating_beta_only():
    result = run_fixed_theta_right_rim_roll_up_2d(
        theta_climb_rad=np.deg2rad(60.0),
        initial_beta_rad=np.deg2rad(-20.0),
        hip_x_m=0.0,
        hip_z_m=0.24,
        obstacle_width_m=0.60,
        obstacle_height_m=0.10,
        arc_samples=241,
        max_steps=12,
    )

    assert result.success
    assert result.failure_reason is None
    assert len(result.frames) == 2
    assert all(
        frame.theta_rad == pytest.approx(np.deg2rad(60.0))
        for frame in result.frames
    )
    assert [frame.terrain_surface_id for frame in result.frames] == [
        "day6_7_step3_obstacle_front",
        "day6_7_step3_obstacle_top",
    ]
    assert all(frame.active_rim == "right_rim" for frame in result.frames)
    assert all(frame.accepted and frame.valid_contact and not frame.collision for frame in result.frames)
    assert result.frames[1].contact_point_world_xz_m[0] > result.frames[0].contact_point_world_xz_m[0]
    assert result.frames[0].beta_rad > result.frames[1].beta_rad


def test_step3_records_failure_reason_without_theta_update_or_recovery():
    result = run_fixed_theta_right_rim_roll_up_2d(
        theta_climb_rad=np.deg2rad(60.0),
        initial_beta_rad=np.deg2rad(-30.0),
        hip_x_m=0.0,
        hip_z_m=0.24,
        obstacle_width_m=0.60,
        obstacle_height_m=0.10,
        arc_samples=241,
        max_steps=12,
    )

    assert not result.success
    assert result.failure_reason == "NO_VALID_RIGHT_RIM_CONTACT"
    assert len(result.frames) == 1
    assert result.frames[0].failure_reason == result.failure_reason
    assert result.frames[0].theta_rad == pytest.approx(np.deg2rad(60.0))


def test_step3_rows_and_sequential_frame_visualization():
    result = run_fixed_theta_right_rim_roll_up_2d(
        theta_climb_rad=np.deg2rad(60.0),
        initial_beta_rad=np.deg2rad(-20.0),
        hip_x_m=0.0,
        hip_z_m=0.24,
        obstacle_width_m=0.60,
        obstacle_height_m=0.10,
        arc_samples=241,
    )
    rows = right_rim_roll_up_rows(result)
    figures = plot_right_rim_roll_up_trajectory_2d(result)

    assert len(rows) == len(result.frames) == len(figures)
    assert rows[0]["active_rim"] == "right_rim"
    assert rows[-1]["terrain_surface_id"].endswith("_top")
    assert all("collision" in row and "failure_reason" in row for row in rows)
    assert all("Step 3 frame" in figure.axes[0].get_title() for figure in figures)
    for figure in figures:
        plt.close(figure)


def test_step4_theta_sweep_reuses_fixed_theta_runner_and_reports_range(tmp_path):
    result = sweep_right_rim_roll_up_theta_2d(
        theta_min_rad=np.deg2rad(58.0),
        theta_max_rad=np.deg2rad(62.0),
        dtheta_rad=np.deg2rad(1.0),
        initial_beta_rad=np.deg2rad(-20.0),
        hip_x_m=0.0,
        hip_z_m=0.24,
        obstacle_width_m=0.60,
        obstacle_height_m=0.10,
        arc_samples=241,
    )

    assert [row.theta_climb_deg for row in result.rows] == pytest.approx([58, 59, 60, 61, 62])
    assert [row.roll_up_success for row in result.rows] == [False, True, True, False, False]
    assert result.minimum_feasible_theta_deg == pytest.approx(59.0)
    assert result.maximum_feasible_theta_deg == pytest.approx(60.0)
    assert result.feasible_theta_range_deg == pytest.approx((59.0, 60.0))
    assert result.rows[0].failure_reason == "NO_VALID_RIGHT_RIM_CONTACT"
    assert result.rows[2].final_rim == "right_rim"
    assert result.rows[2].final_contact_surface.endswith("_top")

    rows = right_rim_theta_sweep_rows(result)
    csv_path = write_right_rim_theta_sweep_csv(result, tmp_path / "theta_sweep.csv")
    assert Path(csv_path).read_text(encoding="utf-8").splitlines()[0].split(",") == list(rows[0].keys())
    assert len(Path(csv_path).read_text(encoding="utf-8").splitlines()) == 6

    figure, ax = plt.subplots()
    assert plot_right_rim_theta_feasibility_2d(result, ax=ax) is ax
    assert ax.get_xlabel() == r"$\theta_{climb}$ [deg]"
    plt.close(figure)


def test_step4_rejects_invalid_theta_sweep_definition():
    with pytest.raises(ValueError, match="dtheta_rad must be positive"):
        sweep_right_rim_roll_up_theta_2d(
            theta_min_rad=0.0,
            theta_max_rad=1.0,
            dtheta_rad=0.0,
            initial_beta_rad=0.0,
            hip_x_m=0.0,
            hip_z_m=0.24,
        )


def test_step4_5_advances_hip_and_keeps_only_continuous_legal_poses():
    result = run_forward_right_rim_roll_up_2d(
        candidate_theta_rad=np.deg2rad(60.0),
        initial_beta_rad=np.deg2rad(-20.0),
        hip_x_m=0.0,
        hip_z_m=0.24,
        dx_m=0.002,
        max_forward_steps=10,
        beta_step_rad=np.deg2rad(-5.0),
        arc_samples=61,
    )

    assert not result.success
    assert result.failure_reason == "MAX_FORWARD_STEPS_BEFORE_TOP_ROLL_COMPLETE"
    assert result.frames[0].hip_x_m == pytest.approx(0.0)
    assert result.frames[0].accepted
    assert result.frames[0].terrain_surface_id.endswith("_front")
    assert result.frames[0].hip_forward_progress_m == pytest.approx(0.0)
    assert result.frames[1].accepted
    assert any(frame.roll_phase == "LEADING_CORNER_TRANSITION" for frame in result.frames)
    assert result.frames[-2].roll_phase == "LEADING_CORNER_TO_TOP"
    assert result.frames[-1].roll_phase == "TOP_ROLL"
    assert result.frames[-1].top_roll_progress_m > 0.002
    assert result.frames[-1].top_roll_remaining_m == pytest.approx(
        0.02 - result.frames[-1].top_roll_progress_m
    )
    assert result.frames[-1].continuation_target_world_xz_m is not None
    assert result.frames[-1].continuation_error_m == pytest.approx(0.0, abs=1e-9)
    assert result.frames[-1].hip_forward_progress_m > 0.002
    assert result.frames[-1].top_contact_advance_m == pytest.approx(
        result.frames[-1].top_roll_progress_m
    )
    assert result.frames[-1].beta_rad < result.frames[0].beta_rad
    assert max(frame.scene.hip_pose.position_world_xz_m[1] for frame in result.frames) > 0.26
    assert all(frame.collision is False for frame in result.frames)
    readiness = evaluate_right_rim_retract_readiness_2d(result)
    assert not readiness.ready_to_retract
    assert readiness.failure_reason == "TOP_ROLL_NOT_COMPLETE"
    assert readiness.preview_frames == ()


def test_step4_5_candidate_summary_retains_failure_and_frame_records():
    result = simulate_theta_candidates_forward_rolling_2d(
        [np.deg2rad(60.0)],
        initial_beta_rad=np.deg2rad(-20.0),
        hip_x_m=0.0,
        hip_z_m=0.24,
        dx_m=0.005,
        max_forward_steps=1,
        arc_samples=61,
    )

    assert len(result.rows) == len(result.simulations) == 1
    assert result.rows[0].candidate_theta_deg == pytest.approx(60.0)
    assert not result.rows[0].continuous_roll_up_success
    assert result.rows[0].failure_reason == "MAX_FORWARD_STEPS_BEFORE_TOP_SUPPORT"
    assert len(forward_rolling_rows(result)) == 1
    assert len(forward_rolling_frame_rows(result.simulations[0])) == 2


def test_step4_5_success_requires_top_roll_distance_not_hip_crossing():
    result = run_forward_right_rim_roll_up_2d(
        candidate_theta_rad=np.deg2rad(60.0),
        initial_beta_rad=np.deg2rad(-20.0),
        hip_x_m=0.0,
        hip_z_m=0.24,
        dx_m=0.002,
        max_forward_steps=10,
        beta_step_rad=np.deg2rad(-5.0),
        top_roll_distance_m=0.0,
        arc_samples=61,
    )

    assert result.success
    assert result.final_frame.top_roll_complete
    assert result.final_frame.top_roll_progress_m == pytest.approx(0.0, abs=1e-12)
    assert result.final_frame.top_contact_advance_m == pytest.approx(0.0, abs=1e-12)
    assert result.final_frame.top_roll_remaining_m == pytest.approx(0.0, abs=1e-12)
    assert result.final_frame.roll_phase == "TOP_ROLL_COMPLETE"
    assert result.final_frame.hip_x_m > 0.10


def test_step4_5_top_roll_uses_no_slip_sample_advance_and_reaches_distance(tmp_path):
    result = run_forward_right_rim_roll_up_2d(
        candidate_theta_rad=np.deg2rad(60.0),
        initial_beta_rad=np.deg2rad(-20.0),
        hip_x_m=0.0,
        hip_z_m=0.24,
        dx_m=0.0002,
        max_forward_steps=20,
        beta_step_rad=np.deg2rad(-5.0),
        top_roll_distance_m=0.0002,
        theta_search_window_rad=np.deg2rad(3.0),
        beta_search_window_rad=np.deg2rad(5.0),
        arc_samples=61,
        contact_continuation_iterations=4,
        top_contact_sample_search_window=8,
        top_contact_candidate_limit=8,
    )

    assert result.success
    top_frames = [
        frame
        for frame in result.frames
        if frame.terrain_surface_id is not None
        and frame.terrain_surface_id.endswith("_top")
        and frame.top_roll_progress_m is not None
    ]
    assert len(top_frames) >= 2
    assert top_frames[0].top_roll_progress_m == pytest.approx(0.0, abs=1e-9)
    assert top_frames[-1].top_roll_progress_m >= 0.0002
    assert top_frames[-1].alpha_rad > top_frames[0].alpha_rad
    assert all(
        later.alpha_rad > earlier.alpha_rad
        for earlier, later in zip(top_frames, top_frames[1:])
    )
    assert all(frame.continuation_error_m is not None for frame in top_frames)
    assert all(frame.collision is False for frame in result.frames)
    readiness = evaluate_right_rim_retract_readiness_2d(
        result,
        retract_theta_target_rad=np.deg2rad(17.0),
        preview_steps=3,
        stable_window_frames=2,
        safety_margin_m=0.0002,
    )
    assert readiness.ready_to_retract
    assert readiness.preview_success
    assert readiness.no_slip_stable
    assert readiness.clearance_satisfied
    assert len(readiness.preview_frames) == 3
    assert all(frame.accepted and not frame.collision for frame in readiness.preview_frames)
    assert [frame.theta_rad for frame in readiness.preview_frames] == pytest.approx(
        [np.deg2rad(59.0), np.deg2rad(58.0), np.deg2rad(57.0)]
    )

    retract = run_retract_to_wheel_2d(
        result,
        theta_target_rad=np.deg2rad(17.0),
        theta_step_rad=np.deg2rad(5.0),
    )
    assert retract.success
    assert retract.failure_reason is None
    assert retract.final_frame.theta_rad == pytest.approx(np.deg2rad(17.0))
    assert all(
        later.theta_rad < earlier.theta_rad
        for earlier, later in zip(retract.frames, retract.frames[1:])
    )
    assert all(
        frame.accepted
        and frame.valid_contact
        and not frame.collision
        and frame.joint_limits_ok
        and frame.terrain_surface_id.endswith("_top")
        for frame in retract.frames
    )
    assert all(
        frame.contact_point_world_xz_m
        == pytest.approx(retract.frames[0].contact_point_world_xz_m)
        for frame in retract.frames
    )

    beta_limited = run_retract_to_wheel_2d(
        result,
        beta_min_rad=np.deg2rad(-30.0),
        beta_max_rad=np.deg2rad(30.0),
    )
    assert not beta_limited.success
    assert beta_limited.failure_reason == "ROLL_UP_END_STATE_JOINT_LIMIT_VIOLATION"
    assert beta_limited.failure_theta_rad == pytest.approx(result.final_theta_rad)
    assert beta_limited.failure_beta_rad == pytest.approx(result.final_beta_rad)

    reset = run_wheel_reset_roll_2d(
        retract,
        beta_step_rad=np.deg2rad(1.0),
        foot_ready_alpha_tolerance_rad=np.deg2rad(1.0),
        max_reset_rotation_rad=np.deg2rad(120.0),
        max_reset_forward_distance_m=0.40,
    )
    assert reset.success
    assert reset.failure_reason is None
    assert reset.required_reset_rotation_rad > 0.0
    assert reset.required_reset_forward_distance_m > 0.0
    assert reset.l_reset_m == pytest.approx(reset.required_reset_forward_distance_m)
    assert reset.final_frame.foot_rim_ready
    assert reset.final_frame.active_rim == "foot_rim"
    assert reset.final_frame.terrain_surface_id.endswith("_top")
    assert reset.final_frame.alpha_rad == pytest.approx(0.0, abs=np.deg2rad(1.0))
    assert reset.final_frame.beta_rad == pytest.approx(0.0, abs=np.deg2rad(1.0))
    assert any(frame.active_rim == "right_rim" for frame in reset.frames)
    assert any(frame.active_rim == "foot_rim" for frame in reset.frames)
    assert all(
        frame.accepted
        and frame.valid_contact
        and not frame.collision
        and frame.joint_limits_ok
        and frame.terrain_surface_id.endswith("_top")
        for frame in reset.frames
    )
    assert all(
        later.beta_rad > earlier.beta_rad
        for earlier, later in zip(reset.frames, reset.frames[1:])
    )
    assert all(
        later.contact_forward_displacement_m > earlier.contact_forward_displacement_m
        for earlier, later in zip(reset.frames, reset.frames[1:])
    )
    assert reset.final_frame.hip_forward_displacement_m == pytest.approx(
        reset.required_reset_forward_distance_m, abs=1e-8
    )

    rows = wheel_reset_roll_rows(reset)
    csv_path = write_wheel_reset_roll_csv(reset, tmp_path / "wheel_reset.csv")
    assert len(rows) == len(reset.frames)
    assert rows[-1]["foot_rim_ready"]
    assert Path(csv_path).read_text(encoding="utf-8").splitlines()[0].split(",") == list(rows[0].keys())

    rotation_limited = run_wheel_reset_roll_2d(
        retract,
        max_reset_rotation_rad=np.deg2rad(10.0),
    )
    assert not rotation_limited.success
    assert rotation_limited.failure_reason == "MAX_RESET_ROTATION_REACHED"
    assert rotation_limited.required_reset_rotation_rad is None
    assert rotation_limited.required_reset_forward_distance_m is None

    coupled = run_retract_and_reset_comparison_2d(
        result,
        theta_step_rad=np.deg2rad(2.0),
        beta_step_rad=np.deg2rad(1.0),
        beta_search_window_rad=np.deg2rad(5.0),
        max_rotation_rad=np.deg2rad(400.0),
        max_forward_distance_m=0.90,
        max_seam_bridge_m=0.015,
        obstacle_top_length_m=1.00,
    )
    assert coupled.selected_branch == "forward_continuation"
    assert len(coupled.branches) == 2
    forward = next(
        branch for branch in coupled.branches
        if branch.branch == "forward_continuation"
    )
    reverse = next(
        branch for branch in coupled.branches
        if branch.branch == "shortest_reverse"
    )
    assert not forward.direction_reversal
    assert reverse.direction_reversal
    assert forward.success
    assert not reverse.success
    assert reverse.failure_reason == "NO_SLIP_REQUIRES_NEGATIVE_X_MOTION"
    assert reverse.required_rotation_rad is None
    assert reverse.required_forward_distance_m is None
    assert any(frame.active_rim == "left_rim" for frame in forward.frames)
    assert forward.final_frame.foot_rim_ready
    assert forward.final_frame.active_rim == "foot_rim"
    assert forward.final_frame.theta_rad == pytest.approx(np.deg2rad(17.0))
    assert not reverse.final_frame.foot_rim_ready
    assert all(
        later.theta_rad <= earlier.theta_rad + 1e-12
        for branch in coupled.branches
        for earlier, later in zip(branch.frames, branch.frames[1:])
    )
    assert all(
        later.contact_forward_displacement_m
        >= earlier.contact_forward_displacement_m - 1e-12
        for branch in coupled.branches
        for earlier, later in zip(branch.frames, branch.frames[1:])
    )
    assert all(
        frame.accepted and frame.valid_contact and not frame.collision
        for branch in coupled.branches
        for frame in branch.frames
    )
    assert all(
        frame.step_hip_displacement_m >= -1e-6
        and frame.step_contact_displacement_m >= -1e-6
        and abs(frame.no_slip_tangent_residual_m) <= 1e-9
        for branch in coupled.branches
        for frame in branch.frames
    )
    summary_rows = retract_reset_comparison_rows(coupled)
    frame_rows = retract_reset_frame_rows(coupled)
    summary_csv, frame_csv = write_retract_reset_comparison_csv(
        coupled,
        tmp_path / "step6_5_summary.csv",
        tmp_path / "step6_5_frames.csv",
    )
    assert len(summary_rows) == 2
    assert len(frame_rows) == sum(len(branch.frames) for branch in coupled.branches)
    assert Path(summary_csv).exists()
    assert Path(frame_csv).exists()

    airborne = run_airborne_retract_and_foot_reset_comparison_2d(
        result,
        theta_step_rad=np.deg2rad(5.0),
        beta_step_rad=np.deg2rad(5.0),
        vertical_step_m=0.01,
        airborne_clearance_m=0.015,
        touchdown_contact_advance_m=0.05,
        obstacle_top_length_m=0.60,
    )
    assert airborne.selected_branch == "shortest_reverse"
    assert len(airborne.branches) == 2
    assert all(branch.success for branch in airborne.branches)
    assert all(branch.final_frame.foot_rim_ready for branch in airborne.branches)
    assert all(branch.final_frame.active_rim == "foot_rim" for branch in airborne.branches)
    assert all(branch.final_frame.terrain_surface_id.endswith("_top") for branch in airborne.branches)
    assert all(branch.final_frame.theta_rad == pytest.approx(np.deg2rad(17.0)) for branch in airborne.branches)
    assert all(
        frame.accepted and not frame.collision and frame.joint_limits_ok
        for branch in airborne.branches
        for frame in branch.frames
    )
    assert all(
        frame.minimum_clearance_above_top_m >= 0.015 - 1e-9
        for branch in airborne.branches
        for frame in branch.frames
        if frame.phase == "AIRBORNE_RETRACT_RESET"
    )
    assert all(
        not frame.contact_required
        for branch in airborne.branches
        for frame in branch.frames
        if frame.phase in {"LIFTOFF", "AIRBORNE_RETRACT_RESET", "FOOT_ALIGN_DESCENT"}
    )
    assert all(branch.other_leg_support_assumed for branch in airborne.branches)
    shortest = airborne.selected_result
    assert shortest.required_rotation_rad < next(
        branch.required_rotation_rad
        for branch in airborne.branches
        if branch.branch == "forward_continuation"
    )
    airborne_summary = airborne_retract_reset_comparison_rows(airborne)
    airborne_frames = airborne_retract_reset_frame_rows(airborne)
    airborne_summary_csv, airborne_frame_csv = write_airborne_retract_reset_comparison_csv(
        airborne,
        tmp_path / "step6_75_summary.csv",
        tmp_path / "step6_75_frames.csv",
    )
    assert len(airborne_summary) == 2
    assert len(airborne_frames) == sum(len(branch.frames) for branch in airborne.branches)
    assert Path(airborne_summary_csv).exists()
    assert Path(airborne_frame_csv).exists()


def test_step4_5_completion_uses_top_contact_progress_not_hip_progress():
    result = run_forward_right_rim_roll_up_2d(
        candidate_theta_rad=np.deg2rad(60.0),
        initial_beta_rad=np.deg2rad(-20.0),
        hip_x_m=0.0,
        hip_z_m=0.24,
        dx_m=0.002,
        max_forward_steps=10,
        beta_step_rad=np.deg2rad(-5.0),
        top_roll_distance_m=0.001,
        arc_samples=61,
    )

    assert result.success
    assert result.failure_reason is None
    assert result.final_frame.hip_forward_progress_m > 0.002
    assert result.final_frame.top_roll_remaining_m == pytest.approx(0.0)
    assert result.final_frame.roll_phase == "TOP_ROLL_COMPLETE"
    assert result.final_frame.top_contact_advance_m >= 0.001
    assert result.final_frame.top_roll_progress_m >= 0.001
