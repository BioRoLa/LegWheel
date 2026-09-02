"""Day 8--9 Step 1 tests: the Cartesian swing input/output contract."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from legwheel.config import RobotParams
from legwheel.planners.hybrid import ContactState, RimId, SwingTarget

from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    HipTrajectory2D,
    SwingFailure,
    SwingResult2D,
    SwingSample2D,
    SwingTarget2D,
    build_leg_on_surface_scene_2d,
    build_swing_request_2d,
    day6_7_roll_end_swing_start_2d,
    flat_to_flat_swing_request_2d,
    pending_swing_result_2d,
    rim_alpha_limits_rad,
    swing_request_rows,
    swing_result_frame_rows,
    swing_result_summary_row,
    validate_swing_request_2d,
    write_swing_result_csv,
)

ARC_SAMPLES = 121
THETA_RAD = np.deg2rad(60.0)
ROLL_END_CACHE = (
    Path(__file__).resolve().parents[1]
    / "hybrid_note/notes/day6-7/day6_7_roll_up_end_state.json"
)


def _obstacle(height_m: float) -> dict:
    return {
        "obstacle_x_start_m": 0.10,
        "obstacle_width_m": 0.35,
        "obstacle_height_m": height_m,
        "obstacle_id": "test_obstacle",
        "arc_samples": ARC_SAMPLES,
    }


def _height_case_request(height_m: float, *, descend: bool):
    obstacle = _obstacle(height_m)
    low = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.00, 0.00, **obstacle)
    high = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.20, height_m, **obstacle)
    return build_swing_request_2d(*((high, low) if descend else (low, high)))


def test_flat_to_flat_request_is_well_posed():
    request = flat_to_flat_swing_request_2d(arc_samples=ARC_SAMPLES)

    assert validate_swing_request_2d(request) == ()
    assert request.start.terrain_surface_id == "ground"
    assert request.target.target_terrain_surface_id == "ground"
    assert request.height_change_m == pytest.approx(0.0, abs=1e-9)
    assert request.horizontal_span_m > 0.0
    assert swing_request_rows(request)


@pytest.mark.parametrize("height_m", [0.02, 0.04])
@pytest.mark.parametrize("descend", [False, True])
def test_touchdown_height_only_changes_the_target(height_m: float, descend: bool):
    """The same builder covers flat->top and top->flat without new gait logic."""

    request = _height_case_request(height_m, descend=descend)

    assert validate_swing_request_2d(request) == ()
    expected_dz = -height_m if descend else height_m
    assert request.height_change_m == pytest.approx(expected_dz, abs=1e-6)
    landing_surface = "ground" if descend else "test_obstacle_top"
    assert request.target.target_terrain_surface_id == landing_surface
    assert request.target.touchdown_height_m == pytest.approx(
        0.0 if descend else height_m, abs=1e-6
    )


def test_start_state_bridges_to_the_day1_contact_contract():
    request = flat_to_flat_swing_request_2d(arc_samples=ARC_SAMPLES)

    contact = request.start.as_contact_state()
    assert isinstance(contact, ContactState)
    assert contact.rim is request.start.rim
    assert contact.point_world_m[0] == pytest.approx(request.start.contact_point_world_xz_m[0])
    assert contact.point_world_m[1] == pytest.approx(0.0)
    assert contact.point_world_m[2] == pytest.approx(request.start.contact_point_world_xz_m[1])

    target = request.target.as_swing_target()
    assert isinstance(target, SwingTarget)
    assert target.target_rim is request.target.target_rim
    assert target.clearance_m == pytest.approx(request.target.clearance_m)


def test_alpha_outside_its_own_rim_arc_is_refused_at_construction():
    minimum_rad, maximum_rad = rim_alpha_limits_rad(RimId.FOOT)
    assert maximum_rad < np.deg2rad(90.0)

    with pytest.raises(ValueError):
        SwingTarget2D([0.15, 0.0], RimId.FOOT, np.deg2rad(90.0), "ground")

    inside = SwingTarget2D([0.15, 0.0], RimId.FOOT, 0.5 * maximum_rad, "ground")
    assert inside.target_alpha_rad == pytest.approx(0.5 * maximum_rad)


def test_validation_reports_every_ill_posed_target():
    request = flat_to_flat_swing_request_2d(arc_samples=ARC_SAMPLES)

    unknown = replace(
        request, target=replace(request.target, target_terrain_surface_id="nope")
    )
    assert any("does not contain" in problem for problem in validate_swing_request_2d(unknown))

    off_surface = replace(
        request, target=replace(request.target, target_point_world_xz_m=[0.15, 0.04])
    )
    assert any("from surface" in problem for problem in validate_swing_request_2d(off_surface))

    obstacle_request = _height_case_request(0.04, descend=False)
    vertical = replace(
        obstacle_request,
        target=replace(
            obstacle_request.target,
            target_terrain_surface_id="test_obstacle_front",
            target_point_world_xz_m=[0.10, 0.02],
        ),
    )
    assert any("is vertical" in problem for problem in validate_swing_request_2d(vertical))

    beyond_limit = replace(
        request, start=replace(request.start, theta_rad=np.deg2rad(RobotParams.MAX_THETA_DEG + 10.0))
    )
    assert any("joint limits" in problem for problem in validate_swing_request_2d(beyond_limit))

    unchanged = replace(
        request,
        target=SwingTarget2D(
            request.start.contact_point_world_xz_m,
            request.start.rim,
            request.start.alpha_rad,
            request.start.terrain_surface_id,
        ),
    )
    assert any("nothing to relocate" in problem for problem in validate_swing_request_2d(unchanged))


def test_gamma_and_hip_pitch_stay_pinned_to_the_first_version_scope():
    request = flat_to_flat_swing_request_2d(arc_samples=ARC_SAMPLES)

    with pytest.raises(ValueError):
        replace(request.start, gamma_rad=0.1)

    tilted_hip = replace(request.start.hip_pose, pitch_world_hip_rad=0.05)
    with pytest.raises(ValueError):
        HipTrajectory2D(start_pose=tilted_hip)


def test_hip_trajectory_interpolates_between_the_two_scene_hips():
    obstacle = _obstacle(0.04)
    low = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.00, 0.00, **obstacle)
    high = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.20, 0.04, **obstacle)
    request = build_swing_request_2d(low, high)

    assert not request.hip_trajectory.is_stationary
    start_pose = request.hip_trajectory.pose_at(0.0)
    end_pose = request.hip_trajectory.pose_at(1.0)
    middle = request.hip_trajectory.pose_at(0.5)
    assert start_pose.position_world_xz_m == pytest.approx(low.hip_pose.position_world_xz_m)
    assert end_pose.position_world_xz_m == pytest.approx(high.hip_pose.position_world_xz_m)
    assert middle.position_world_xz_m == pytest.approx(
        0.5 * (low.hip_pose.position_world_xz_m + high.hip_pose.position_world_xz_m)
    )

    stationary = build_swing_request_2d(low, high, move_hip=False)
    assert stationary.hip_trajectory.is_stationary


def test_result_cannot_claim_validity_without_a_clean_failure_reason():
    request = flat_to_flat_swing_request_2d(arc_samples=ARC_SAMPLES)

    with pytest.raises(ValueError):
        SwingResult2D(request=request, valid=True, failure=SwingFailure.IK_NOT_CONVERGED)
    with pytest.raises(ValueError):
        SwingResult2D(request=request, valid=False, failure=SwingFailure.NONE)


def test_unevaluated_sample_fields_stay_none():
    sample = SwingSample2D(0, 0.0, [0.0, 0.0], [0.0, 0.0], 0.0, RimId.FOOT)
    row = sample.as_row()

    assert row["theta_deg"] is None
    assert row["ik_converged"] is None
    assert row["collision_free"] is None
    assert row["terrain_clearance_mm"] is None


def test_pending_result_writes_the_frozen_frame_schema(tmp_path):
    request = flat_to_flat_swing_request_2d(arc_samples=ARC_SAMPLES)
    result = pending_swing_result_2d(request)

    assert result.valid is False
    assert result.failure is SwingFailure.NOT_EVALUATED
    assert swing_result_frame_rows(result) == []

    summary = swing_result_summary_row(result)
    assert summary["failure"] == "NOT_EVALUATED"
    assert summary["final_contact_error_mm"] is None
    assert summary["sample_count"] == request.sample_count

    frames_path, summary_path = write_swing_result_csv(
        result, tmp_path / "frames.csv", tmp_path / "summary.csv"
    )
    header = frames_path.read_text(encoding="utf-8").splitlines()[0].split(",")
    assert header[:4] == ["index", "time_s", "x_m", "z_m"]
    assert "collision_free" in header
    assert summary_path.read_text(encoding="utf-8").splitlines()[0].startswith("valid,failure")


def test_day6_7_roll_end_becomes_a_swing_start_state():
    scene, start = day6_7_roll_end_swing_start_2d(ROLL_END_CACHE)

    assert start.rim is RimId.RIGHT
    assert start.terrain_surface_id.endswith("_top")
    assert start.contact_point_world_xz_m[1] == pytest.approx(
        scene.terrain.obstacle.height_m, abs=1e-6
    )
    assert start.theta_rad == pytest.approx(scene.theta_rad)
    assert start.beta_rad == pytest.approx(scene.beta_rad)
    assert start.rim_edge_margin_rad > 0.0


def test_scenes_from_different_terrains_are_refused():
    low = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.0, 0.0, **_obstacle(0.02))
    high = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.2, 0.04, **_obstacle(0.04))

    with pytest.raises(ValueError, match="same terrain"):
        build_swing_request_2d(low, high)
