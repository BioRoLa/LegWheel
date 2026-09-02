"""Day 8--9 Step 5 tests: Cartesian path -> joint trajectory."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    SwingFailure,
    build_leg_on_surface_scene_2d,
    build_swing_request_2d,
    flat_to_flat_swing_request_2d,
    swing_result_frame_rows,
)
from hybrid_note.scripts.experiments.cartesian_swing_ik_2d import (
    THETA_MIN_RAD,
    rim_contact_point_world_xz_m,
)
from hybrid_note.scripts.experiments.cartesian_swing_path_2d import (
    generate_swing_path_2d,
    generate_terrain_aware_swing_path_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_trajectory_2d import (
    joint_step_rad,
    joint_trajectory_report_2d,
    solve_swing_joint_trajectory_2d,
)

ARC_SAMPLES = 121
THETA_RAD = np.deg2rad(60.0)
CLEARANCE_M = 0.03


def _obstacle(height_m: float, *, width_m: float = 0.35, x_start_m: float = 0.10) -> dict:
    return {
        "obstacle_x_start_m": x_start_m,
        "obstacle_width_m": width_m,
        "obstacle_height_m": height_m,
        "obstacle_id": "test_obstacle",
        "arc_samples": ARC_SAMPLES,
    }


def _case_request(height_m: float, *, descend: bool = False):
    if height_m == 0.0:
        return flat_to_flat_swing_request_2d(arc_samples=ARC_SAMPLES, clearance_m=CLEARANCE_M)
    obstacle = _obstacle(height_m)
    low = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.00, 0.00, **obstacle)
    high = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.20, height_m, **obstacle)
    scenes = (high, low) if descend else (low, high)
    return build_swing_request_2d(*scenes, clearance_m=CLEARANCE_M)


def _obstacle_between_request(clearance_m: float = CLEARANCE_M):
    obstacle = _obstacle(0.04, width_m=0.15)
    start = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.00, 0.00, **obstacle)
    target = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.40, 0.00, **obstacle)
    return build_swing_request_2d(start, target, clearance_m=clearance_m)


HEIGHT_CASES = [
    pytest.param(0.0, False, id="flat_to_flat"),
    pytest.param(0.02, False, id="flat_to_20mm"),
    pytest.param(0.04, False, id="flat_to_40mm"),
    pytest.param(0.04, True, id="40mm_to_flat"),
]


@pytest.mark.parametrize("height_m,descend", HEIGHT_CASES)
def test_every_touchdown_height_yields_a_continuous_joint_trajectory(height_m, descend):
    """Step 5's completion criterion, across the planning note's cases."""

    request = _case_request(height_m, descend=descend)

    solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(request))
    report = joint_trajectory_report_2d(solved)

    assert report["ik_converged_count"] == request.sample_count
    assert report["joint_limits_ok_count"] == request.sample_count
    assert report["max_ik_residual_um"] < 100.0
    assert report["max_joint_step_deg"] < report["max_joint_step_limit_deg"]
    assert solved.failure is SwingFailure.NOT_EVALUATED
    assert solved.failure_sample_index is None


def test_the_obstacle_between_case_also_solves():
    solved, _ = solve_swing_joint_trajectory_2d(
        generate_terrain_aware_swing_path_2d(_obstacle_between_request())
    )
    report = joint_trajectory_report_2d(solved)

    assert report["ik_converged_count"] == solved.request.sample_count
    assert report["max_joint_step_deg"] < report["max_joint_step_limit_deg"]


def test_the_first_solve_reproduces_the_start_contact_state():
    """Contract, path, FK and IK must agree at the one pose all four know."""

    for height_m, descend in ((0.0, False), (0.04, False), (0.04, True)):
        request = _case_request(height_m, descend=descend)
        solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(request))
        report = joint_trajectory_report_2d(solved)

        assert report["start_theta_error_deg"] == pytest.approx(0.0, abs=1e-6)
        assert report["start_beta_error_deg"] == pytest.approx(0.0, abs=1e-6)


def test_solved_joints_actually_place_the_requested_rim_point():
    request = _case_request(0.04)

    solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(request))

    for sample in solved.samples[::10]:
        fraction = sample.time_s / request.swing_duration_s
        hip_pose = request.hip_trajectory.pose_at(float(np.clip(fraction, 0.0, 1.0)))
        placed = rim_contact_point_world_xz_m(
            sample.theta_rad, sample.beta_rad, sample.rim, sample.alpha_rad, hip_pose
        )
        assert placed == pytest.approx(sample.position_world_xz_m, abs=1e-4)


def test_joint_trajectory_is_smooth_sample_to_sample():
    request = _case_request(0.04)

    solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(request))

    thetas = np.array([sample.theta_rad for sample in solved.samples])
    betas = np.array([sample.beta_rad for sample in solved.samples])
    # No branch flips: consecutive samples move by a fraction of a degree.
    assert np.max(np.abs(np.diff(thetas))) < np.deg2rad(2.0)
    assert np.max(np.abs(np.diff(betas))) < np.deg2rad(2.0)
    assert solved.samples[0].joint_step_rad is None
    assert all(sample.joint_step_rad is not None for sample in solved.samples[1:])


def test_an_unreachable_apex_is_reported_at_the_sample_where_it_fails():
    """A clearance the leg cannot reach must fail loudly, and locally."""

    request = _obstacle_between_request(clearance_m=0.10)

    solved, solutions = solve_swing_joint_trajectory_2d(
        generate_terrain_aware_swing_path_2d(request)
    )
    report = joint_trajectory_report_2d(solved)

    assert solved.failure is SwingFailure.IK_NOT_CONVERGED
    assert solved.failure_sample_index is not None
    assert 0 < solved.failure_sample_index < request.sample_count
    assert "pinned at its limit" in solved.failure_detail
    # The reason is theta running out, not a joint limit being violated: the
    # reported joints stay legal throughout.
    assert report["joint_limits_ok_count"] == request.sample_count
    assert report["theta_min_deg"] == pytest.approx(np.rad2deg(THETA_MIN_RAD), abs=1e-6)
    assert any(solution.theta_at_limit for solution in solutions)


def test_a_tight_continuity_limit_is_what_reports_a_discontinuity():
    request = _case_request(0.04)
    strict = replace(
        request,
        constraints=replace(request.constraints, max_joint_step_rad=np.deg2rad(0.2)),
    )

    solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(strict))

    assert solved.failure is SwingFailure.JOINT_DISCONTINUITY
    assert solved.failure_sample_index is not None
    assert "joint step" in solved.failure_detail


def test_a_solved_trajectory_is_still_not_a_valid_swing():
    """Collision and touchdown have not run; ``valid`` stays false."""

    solved, _ = solve_swing_joint_trajectory_2d(
        generate_terrain_aware_swing_path_2d(_case_request(0.04))
    )

    assert solved.valid is False
    assert solved.failure is SwingFailure.NOT_EVALUATED
    assert "Steps 6-8" in solved.failure_detail
    assert all(sample.collision_free is None for sample in solved.samples)
    assert all(sample.terrain_clearance_m is None for sample in solved.samples)


def test_step5_fills_the_schema_frozen_in_step1_without_changing_it():
    solved, _ = solve_swing_joint_trajectory_2d(
        generate_terrain_aware_swing_path_2d(_case_request(0.02))
    )

    row = swing_result_frame_rows(solved)[10]
    assert row["theta_deg"] is not None
    assert row["beta_deg"] is not None
    assert row["gamma_deg"] == pytest.approx(0.0)
    assert row["ik_converged"] is True
    assert row["ik_residual_mm"] is not None
    assert row["joint_limits_ok"] is True
    # Untouched by Step 5, still waiting for Step 6.
    assert row["collision_free"] is None
    assert row["terrain_clearance_mm"] is None


def test_joint_step_wraps_beta_but_not_theta():
    previous = np.array([np.deg2rad(60.0), np.deg2rad(179.0)])
    same_pose_other_branch = np.array([np.deg2rad(60.0), np.deg2rad(179.0 + 360.0)])
    real_move = np.array([np.deg2rad(60.0), np.deg2rad(-179.0)])

    assert joint_step_rad(previous, same_pose_other_branch) == pytest.approx(0.0, abs=1e-9)
    assert joint_step_rad(previous, real_move) == pytest.approx(np.deg2rad(2.0), abs=1e-9)
    assert joint_step_rad(previous, previous + np.array([0.1, 0.0])) == pytest.approx(0.1)


def test_step5_requires_a_cartesian_path_first():
    request = _case_request(0.0)
    broken = replace(
        request, target=replace(request.target, target_terrain_surface_id="nope")
    )

    with pytest.raises(ValueError, match="no samples"):
        solve_swing_joint_trajectory_2d(generate_swing_path_2d(broken))
