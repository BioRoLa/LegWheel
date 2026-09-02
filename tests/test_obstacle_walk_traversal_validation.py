"""Step 8 tests: full leg-geometry collision and whole-trajectory validation."""

from __future__ import annotations

import json

import numpy as np
import pytest

from legwheel.planners.hybrid import TerrainProfile2D
from legwheel.planners.obstacle_walk import (
    LEG_ORDER,
    ObstacleWalkRequest,
    RectangleObstacle1D,
    WalkTerrain1D,
    build_collision_checker,
    build_terrain_profile_2d,
    generate_obstacle_walk,
)
from legwheel.planners.obstacle_walk.collision import (
    LegGeometrySampler,
    verify_geometry_matches_forward_kinematics,
)
from legwheel.planners.obstacle_walk.traversal import build_walk_generator
from legwheel.planners.obstacle_walk.validation import CheckStatus, validate_traversal


TRAVERSAL_REQUEST = ObstacleWalkRequest(
    obstacle_x_start_m=0.65,
    obstacle_length_m=0.30,
    obstacle_height_m=0.05,
    edge_margin_m=0.02,
    stand_height_m=0.25,
    step_length_m=0.135,
    period_s=2.0,
    dt_s=0.02,
    step_clearance_m=0.02,
    approach_distance_m=0.40,
    post_distance_m=0.10,
)
TERRAIN = WalkTerrain1D(
    RectangleObstacle1D(x_start_m=0.65, length_m=0.30, height_m=0.05, edge_margin_m=0.02)
)


@pytest.fixture(scope="module")
def generator():
    return build_walk_generator(TRAVERSAL_REQUEST)


@pytest.fixture(scope="module")
def traversal():
    return generate_obstacle_walk(TRAVERSAL_REQUEST)


def test_terrain_profile_2d_matches_the_step3_rectangle():
    profile = build_terrain_profile_2d(TERRAIN)
    assert isinstance(profile, TerrainProfile2D)
    assert profile.ground_height_m == TERRAIN.ground_height_m
    assert len(profile.obstacles) == 1
    obstacle = profile.obstacles[0]
    assert obstacle.x_min_m == pytest.approx(TERRAIN.obstacle.x_start_m)
    assert obstacle.x_max_m == pytest.approx(TERRAIN.obstacle.x_end_m)
    assert obstacle.height_m == pytest.approx(TERRAIN.obstacle.height_m)
    kinds = {surface.surface_id for surface in obstacle.surfaces(TERRAIN.ground_height_m)}
    assert kinds == {"obstacle_top", "obstacle_front", "obstacle_back"}


def test_sampled_geometry_converges_on_the_3d_forward_kinematics(generator):
    """A constant offset would mean a wrong frame; shrinking gaps mean sampling."""

    q = np.array([np.deg2rad(107.0), np.deg2rad(-5.0), 0.0])
    pose = np.array([0.0, 0.0, 0.25, 0.0, 0.0, 0.0])
    gaps = []
    for arc_samples in (12, 48, 192):
        sampler = LegGeometrySampler(
            list(generator.legs), list(generator.hip_positions), arc_samples=arc_samples
        )
        gaps.append(verify_geometry_matches_forward_kinematics(sampler, 0, q, pose))
    assert gaps[-1] < gaps[0]
    assert gaps[-1] < 1e-3


def test_every_leg_geometry_is_finite_and_spans_three_rim_surfaces(generator):
    sampler = LegGeometrySampler(
        list(generator.legs), list(generator.hip_positions), arc_samples=16
    )
    pose = np.array([0.3, 0.0, 0.26, 0.0, 0.0, 0.0])
    q = np.array([np.deg2rad(100.0), np.deg2rad(10.0), 0.0])
    for leg_index in range(4):
        geometry = sampler.sample(leg_index, q, pose)
        assert np.all(np.isfinite(geometry.points_world_xz_m))
        assert set(geometry.surface_names) == {"foot_rim", "upper_tyre_l", "upper_tyre_r"}
        assert len(geometry.link_segments_world_xz_m) >= 1
        for segment in geometry.link_segments_world_xz_m:
            assert segment.shape == (2, 2)
            assert np.all(np.isfinite(segment))


def test_collision_checker_accepts_a_leg_standing_clear_of_the_obstacle(generator):
    checker = build_collision_checker(generator, TERRAIN)
    q = np.array([np.deg2rad(107.0), np.deg2rad(-5.0), 0.0])
    message, depth = checker.evaluate("FL", q, np.array([0.0, 0.0, 0.25, 0.0, 0.0, 0.0]))
    assert message is None
    assert depth <= checker.penetration_tolerance_m


def test_collision_checker_reports_geometry_driven_into_the_terrain(generator):
    checker = build_collision_checker(generator, TERRAIN)
    q = np.array([np.deg2rad(107.0), np.deg2rad(-5.0), 0.0])
    # Drop the body far below the standing height: the leg must hit the ground.
    message, depth = checker.evaluate("FL", q, np.array([0.2, 0.0, 0.18, 0.0, 0.0, 0.0]))
    assert message is not None
    assert depth > checker.penetration_tolerance_m
    assert "penetrates" in message or "crosses" in message


def test_collision_checker_satisfies_the_step4_checker_protocol(generator):
    checker = build_collision_checker(generator, TERRAIN)
    outcome = checker(
        sample_index=0,
        leg=LEG_ORDER[0],
        joint_position_rad=np.array([np.deg2rad(107.0), np.deg2rad(-5.0), 0.0]),
        body_pose_world=np.array([0.0, 0.0, 0.25, 0.0, 0.0, 0.0]),
        terrain=TERRAIN,
    )
    assert outcome is None


def test_validation_reports_every_stage_and_check(traversal):
    report = validate_traversal(traversal, frame_stride=200)
    names = [check.name for check in report.checks]
    assert names == [
        "joint_position_limits",
        "joint_velocity",
        "joint_acceleration",
        "gait_phase_legality",
        "support_contact_drift",
        "quasi_static_support_polygon",
        "full_leg_geometry_collision",
    ]
    expected_stages = {
        "approach",
        "single_leg_step_up",
        "all_four_legs_on_top_surface",
        "all_four_top_simultaneously",
        "single_leg_step_down",
        "all_four_legs_back_on_ground",
        "traversal_completed",
        "recovery",
    }
    assert set(report.stage_reached) == expected_stages
    # The completion flag must never be true unless every check and stage is.
    assert report.offline_complete_traversal == (
        all(check.passed for check in report.checks)
        and all(report.stage_reached.values())
    )


def test_full_leg_geometry_never_enters_the_obstacle(traversal):
    report = validate_traversal(traversal, frame_stride=100)
    collision = next(
        check for check in report.checks if check.name == "full_leg_geometry_collision"
    )
    assert collision.status is CheckStatus.PASSED, collision.detail
    assert report.collision_leg_poses_checked == report.collision_frames_checked * 4
    assert report.minimum_obstacle_clearance_m is not None


def test_a_tightened_limit_fails_with_a_located_row_and_leg(traversal):
    report = validate_traversal(
        traversal, frame_stride=400, joint_velocity_limit_rad_s=1e-4
    )
    velocity = next(check for check in report.checks if check.name == "joint_velocity")
    assert velocity.status is CheckStatus.FAILED
    assert velocity.row is not None and 0 <= velocity.row < traversal.segment.sample_count
    assert velocity.leg in {leg.value for leg in LEG_ORDER}
    assert velocity.segment_index is not None
    assert report.offline_complete_traversal is False


def test_collision_sweep_can_be_skipped_and_is_recorded_as_such(traversal):
    report = validate_traversal(traversal, skip_collision=True)
    collision = next(
        check for check in report.checks if check.name == "full_leg_geometry_collision"
    )
    assert collision.status is CheckStatus.SKIPPED
    assert report.minimum_obstacle_clearance_m is None
    assert any("NOT evaluated" in note for note in report.notes)
    # A skipped collision check must never be reported as a complete traversal.
    assert report.offline_complete_traversal is False


def test_report_is_json_serialisable(traversal):
    report = validate_traversal(traversal, frame_stride=400)
    payload = json.loads(json.dumps(report.to_dict()))
    assert payload["checks"]
    assert "stage_reached" in payload
    assert payload["collision"]["frame_stride"] == 400


def test_frame_stride_must_be_a_positive_integer(traversal):
    for bad in (0, -1, 1.5):
        with pytest.raises((ValueError, TypeError)):
            validate_traversal(traversal, frame_stride=bad)
