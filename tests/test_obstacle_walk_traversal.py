"""Step 7 tests: obstacle-walk composition, validation and hardware export."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

from legwheel.config import RobotParams
from legwheel.planners.obstacle_walk.traversal import (
    choose_lateral_sway,
    support_polygon_margin_m,
)
from legwheel.planners.obstacle_walk import (
    CONTROLLER_DT_S,
    CONTROLLER_TRANSFORM_ROWS,
    LEG_ORDER,
    ContinuityTolerances,
    ExportPaths,
    LegId,
    ObstacleTraversalError,
    ObstacleWalkRequest,
    RectangleObstacle1D,
    TraversalRejectReason,
    WalkTerrain1D,
    build_prep_rows,
    build_walk_generator,
    generate_obstacle_walk,
    query_touchdown_surface,
    resample_for_csv_controller,
    to_hardware_order,
    touchdown_candidates,
    walk_swing_order,
    wheel_face_exclusion_m,
    write_obstacle_walk_csv,
)


# Kept deliberately small so the whole traversal is generated once per session.
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


@pytest.fixture(scope="module")
def traversal():
    return generate_obstacle_walk(TRAVERSAL_REQUEST)


def test_wheel_face_exclusion_matches_disc_geometry():
    radius = 0.145
    for height in (0.02, 0.05, 0.10):
        expected = math.sqrt(2.0 * radius * height - height * height)
        assert wheel_face_exclusion_m(radius, height) == pytest.approx(expected)
    # An obstacle taller than the wheel cannot exclude more than the radius.
    assert wheel_face_exclusion_m(radius, 10.0) == pytest.approx(radius)
    for bad in (0.0, -1.0, float("nan")):
        with pytest.raises(ValueError):
            wheel_face_exclusion_m(radius, bad)
        with pytest.raises(ValueError):
            wheel_face_exclusion_m(bad, 0.05)


def test_swing_order_comes_from_walk_phase_offsets():
    generator = build_walk_generator(TRAVERSAL_REQUEST)
    assert walk_swing_order(generator) == (LegId.FL, LegId.RR, LegId.FR, LegId.RL)
    generator.phase_offsets = [0.25, 0.75, 0.0, 0.5]
    reordered = walk_swing_order(generator)
    assert reordered != (LegId.FL, LegId.RR, LegId.FR, LegId.RL)
    assert set(reordered) == set(LEG_ORDER)


def test_touchdown_candidates_respect_terrain_and_wheel_exclusion():
    terrain = WalkTerrain1D(
        RectangleObstacle1D(
            x_start_m=0.65, length_m=0.30, height_m=0.05, edge_margin_m=0.02
        )
    )
    exclusion = 0.11
    for nominal in (0.30, 0.62, 0.70, 0.98, 1.05, 1.40):
        candidates = touchdown_candidates(terrain, exclusion, 1e-3, 1e-3, nominal)
        assert candidates
        distances = [abs(value - nominal) for value in candidates]
        assert distances == sorted(distances)
        for candidate in candidates:
            assert query_touchdown_surface(terrain, candidate).is_legal
            on_top = (
                terrain.obstacle.x_start_m <= candidate <= terrain.obstacle.x_end_m
            )
            if not on_top:
                assert (
                    candidate <= terrain.obstacle.x_start_m - exclusion
                    or candidate >= terrain.obstacle.x_end_m + exclusion
                )


def test_request_validation_rejects_bad_inputs():
    with pytest.raises(ValueError):
        ObstacleWalkRequest(0.5, 0.3, -0.05)
    with pytest.raises(ValueError):
        ObstacleWalkRequest(0.5, 0.3, 0.05, body_lift_ratio=1.5)
    with pytest.raises(ValueError):
        ObstacleWalkRequest(0.5, 0.3, 0.05, body_advance_fractions=(0.25, 1.0))
    with pytest.raises(ValueError):
        ObstacleWalkRequest(0.5, 0.3, 0.05, swing_duration_scales=())
    with pytest.raises(ValueError):
        ObstacleWalkRequest(0.5, 0.3, 0.05, maximum_events=0)


def test_stride_shorter_than_the_wheel_exclusion_is_rejected():
    request = ObstacleWalkRequest(
        obstacle_x_start_m=0.65,
        obstacle_length_m=0.30,
        obstacle_height_m=0.05,
        edge_margin_m=0.02,
        step_length_m=0.05,
        period_s=2.0,
        dt_s=0.02,
    )
    with pytest.raises(ObstacleTraversalError) as error:
        generate_obstacle_walk(request)
    assert error.value.reason is TraversalRejectReason.STEP_TOO_SHORT_FOR_OBSTACLE


def test_walk_velocity_guard_rejects_an_over_long_stride():
    request = ObstacleWalkRequest(
        obstacle_x_start_m=0.65,
        obstacle_length_m=0.30,
        obstacle_height_m=0.05,
        step_length_m=2.0,
        period_s=2.0,
        dt_s=0.02,
    )
    with pytest.raises(ObstacleTraversalError) as error:
        generate_obstacle_walk(request)
    assert error.value.reason is TraversalRejectReason.VELOCITY_GUARD_SCALED


def test_traversal_completes_every_stage(traversal):
    assert traversal.traversal_completed
    assert set(traversal.legs_that_reached_top) == set(LEG_ORDER)
    assert set(traversal.legs_that_returned_to_ground) == set(LEG_ORDER)
    assert traversal.maximum_top_contact_count >= 2
    assert all(traversal.stage_results.values())
    # Step 7 never runs a full leg/wheel/body collision model.
    assert traversal.full_geometry_collision_checked is False


def test_every_segment_boundary_passes_the_step2_validator(traversal):
    assert traversal.boundary_reports
    assert all(report.passed for report in traversal.boundary_reports)
    limits = ContinuityTolerances()
    assert (
        traversal.maximum_boundary_joint_position_error_rad
        <= limits.joint_position_rad
    )
    assert (
        traversal.maximum_boundary_joint_velocity_error_rad_s
        <= limits.joint_velocity_rad_s
    )


def test_exactly_one_leg_swings_at_a_time(traversal):
    swinging = traversal.segment.phase.sum(axis=1)
    assert set(np.unique(swinging)).issubset({0, 1})
    assert np.any(swinging == 1)
    assert np.array_equal(traversal.segment.contact_active, traversal.segment.phase == 0)


def test_body_stays_level_and_never_moves_backward(traversal):
    poses = traversal.segment.body_pose_world
    assert np.allclose(poses[:, 3:], 0.0)
    assert np.all(np.diff(poses[:, 0]) >= -1e-12)
    # Contact z values drift by a few tenths of a micrometre, so the level
    # standing height is only reproduced to that accuracy.
    assert np.all(poses[:, 2] >= TRAVERSAL_REQUEST.stand_height_m - 1e-5)
    assert np.max(poses[:, 2]) > TRAVERSAL_REQUEST.stand_height_m


def test_every_touchdown_is_terrain_legal_and_clear_of_the_faces(traversal):
    obstacle = traversal.terrain.obstacle
    exclusion = traversal.wheel_face_exclusion_m
    touchdowns = [
        record for record in traversal.records if record.touchdown_world_m is not None
    ]
    assert len(touchdowns) >= 8
    for record in touchdowns:
        x, _y, z = record.touchdown_world_m
        query = query_touchdown_surface(traversal.terrain, x)
        assert query.is_legal
        assert query.surface_id == record.touchdown_surface_id
        assert z == pytest.approx(float(query.surface_height_world_m), abs=1e-9)
        if record.touchdown_surface_id == traversal.terrain.ground_surface_id:
            assert (
                x <= obstacle.x_start_m - exclusion
                or x >= obstacle.x_end_m + exclusion
            )
        else:
            assert obstacle.legal_top_x_min_m <= x <= obstacle.legal_top_x_max_m
        assert abs(record.touchdown_bias_x_m) <= (
            TRAVERSAL_REQUEST.maximum_touchdown_bias_m + 1e-12
        )


def test_touchdowns_stay_on_fixed_symmetric_lateral_tracks(traversal):
    """Temporary body sway must never accumulate into world-frame footholds."""

    initial_y = traversal.segment.start_state.foot_contact_points_world_m[:, 1]
    touchdown_y: dict[LegId, list[float]] = {leg: [] for leg in LEG_ORDER}
    for record in traversal.records:
        if record.leg is None or record.touchdown_world_m is None:
            continue
        touchdown_y[record.leg].append(record.touchdown_world_m[1])
        leg_index = LEG_ORDER.index(record.leg)
        assert record.touchdown_world_m[1] == pytest.approx(initial_y[leg_index], abs=1e-8)

    assert all(touchdown_y.values())
    assert initial_y[LEG_ORDER.index(LegId.FL)] == pytest.approx(
        -initial_y[LEG_ORDER.index(LegId.FR)], abs=1e-8
    )
    assert initial_y[LEG_ORDER.index(LegId.RL)] == pytest.approx(
        -initial_y[LEG_ORDER.index(LegId.RR)], abs=1e-8
    )


def test_stance_holds_contacts_and_swings_track_their_targets(traversal):
    assert traversal.maximum_contact_drift_m <= (
        TRAVERSAL_REQUEST.contact_drift_tolerance_m
    )
    assert traversal.maximum_tracking_error_m <= TRAVERSAL_REQUEST.tracking_tolerance_m


def test_joint_commands_stay_inside_the_configured_limits(traversal):
    commands = traversal.segment.commands_rad
    assert np.all(np.isfinite(commands))
    theta = commands[:, :, 0]
    assert np.all(theta >= np.deg2rad(RobotParams.MIN_THETA_DEG) - 1e-9)
    assert np.all(theta <= np.deg2rad(RobotParams.MAX_THETA_DEG) + 1e-9)
    assert np.all(np.abs(commands[:, :, 1]) <= np.deg2rad(RobotParams.BETA_MAX_DEG) + 1e-9)
    assert np.all(np.abs(commands[:, :, 2]) <= np.deg2rad(RobotParams.GAMMA_MAX_DEG) + 1e-9)
    step = np.max(np.abs(np.diff(commands, axis=0)))
    # The guard is a rate, so it is checked against the period the assembled
    # trajectory actually carries, not the coarser one the crawl planned on.
    assert step <= (
        TRAVERSAL_REQUEST.joint_velocity_limit_rad_s * traversal.segment.dt_s + 1e-9
    )


def test_commands_keep_the_requested_joint_limit_margin(traversal):
    commands = traversal.segment.commands_rad
    margin = TRAVERSAL_REQUEST.joint_limit_margin_rad
    assert margin > 0.0
    assert np.all(
        commands[:, :, 0] >= np.deg2rad(RobotParams.MIN_THETA_DEG) + margin - 1e-12
    )
    assert np.all(
        commands[:, :, 0] <= np.deg2rad(RobotParams.MAX_THETA_DEG) - margin + 1e-12
    )
    assert np.all(
        np.abs(commands[:, :, 1])
        <= np.deg2rad(RobotParams.BETA_MAX_DEG) - margin + 1e-12
    )


def test_an_impossible_joint_limit_margin_is_reported(traversal):
    request = ObstacleWalkRequest(
        obstacle_x_start_m=0.65,
        obstacle_length_m=0.30,
        obstacle_height_m=0.05,
        edge_margin_m=0.02,
        step_length_m=0.135,
        period_s=2.0,
        dt_s=0.02,
        step_clearance_m=0.02,
        approach_distance_m=0.40,
        post_distance_m=0.10,
        joint_limit_margin_rad=0.60,
    )
    with pytest.raises(ObstacleTraversalError) as error:
        generate_obstacle_walk(request)
    assert error.value.reason is TraversalRejectReason.NO_FEASIBLE_TOUCHDOWN
    assert any("margin" in attempt for attempt in error.value.attempts)


def test_segment_records_tile_the_trajectory_rows(traversal):
    records = traversal.records
    assert records[0].start_row == 0
    for left, right in zip(records, records[1:]):
        # Adjacent segments share their boundary row exactly once.
        assert right.start_row == left.end_row
    assert records[-1].end_row == traversal.segment.sample_count - 1
    kinds = [record.kind for record in records]
    assert kinds[0::2] == ["stance_advance"] * (len(records) // 2)
    assert kinds[1::2] == ["swing"] * (len(records) // 2)


def test_generation_is_deterministic():
    first = generate_obstacle_walk(TRAVERSAL_REQUEST)
    second = generate_obstacle_walk(TRAVERSAL_REQUEST)
    assert np.array_equal(first.segment.commands_rad, second.segment.commands_rad)
    assert np.array_equal(first.segment.phase, second.segment.phase)
    assert [record.touchdown_world_m for record in first.records] == [
        record.touchdown_world_m for record in second.records
    ]


def test_hardware_order_matches_the_existing_exporter():
    module_path = (
        Path(__file__).resolve().parents[1]
        / "examples"
        / "gait"
        / "generate_hardware_csv.py"
    )
    spec = importlib.util.spec_from_file_location("_hardware_csv_reference", module_path)
    reference = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reference)
    rows = np.arange(24, dtype=float).reshape(2, 12)
    assert np.array_equal(to_hardware_order(rows), reference._to_hw_order(rows))


def test_prep_ramp_starts_at_home_and_ends_at_the_first_frame():
    frame = np.linspace(0.1, 1.2, 12)
    prep = build_prep_rows(frame, dt_s=0.02, prep_duration_s=1.0)
    assert prep.shape == (50, 12)
    home = np.zeros(12)
    home[[0, 2, 4, 6]] = np.deg2rad(RobotParams.THETA0_DEG)
    assert np.allclose(prep[0], home)
    assert np.allclose(prep[-1], frame)
    assert len(build_prep_rows(frame, dt_s=0.02, prep_duration_s=0.0)) == 0


def test_controller_resampling_preserves_planner_knots_and_phase_intervals():
    commands = np.vstack([np.zeros(12), np.ones(12), np.full(12, 0.5)])
    phase = np.array([[0, 0, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0]], dtype=np.int8)
    resampled, resampled_phase, ratio = resample_for_csv_controller(
        commands, phase, planner_dt_s=0.02
    )
    assert ratio == 20
    assert resampled.shape == (41, 12)
    assert np.allclose(resampled[::ratio], commands)
    assert np.array_equal(resampled_phase[:ratio], np.repeat(phase[:1], ratio, axis=0))
    assert np.array_equal(
        resampled_phase[ratio : 2 * ratio], np.repeat(phase[1:2], ratio, axis=0)
    )
    assert np.array_equal(resampled_phase[-1], phase[-1])


def test_controller_resampling_rejects_non_integral_planner_period():
    commands = np.zeros((2, 12))
    phase = np.zeros((2, 4), dtype=np.int8)
    with pytest.raises(ValueError, match="integer multiple"):
        resample_for_csv_controller(commands, phase, planner_dt_s=0.0015)


def test_export_writes_row_aligned_hardware_files(tmp_path, traversal):
    paths = write_obstacle_walk_csv(traversal, tmp_path / "obstacle_walk.csv")
    assert isinstance(paths, ExportPaths)
    commands = np.loadtxt(paths.csv_path, delimiter=",")
    assert commands.ndim == 2 and commands.shape[1] == 12
    with paths.csv_path.open(encoding="utf-8") as handle:
        first_line = handle.readline()
    assert not first_line.lstrip().startswith("#")
    float(first_line.split(",")[0])

    phase = np.loadtxt(paths.phase_csv_path, delimiter=",", skiprows=1)
    assert phase.shape == (len(commands), 4)
    assert set(np.unique(phase)).issubset({0.0, 1.0})

    prep_rows = paths.prep_row_count
    # The ratio belongs to the *assembled* segment, not to the crawl's planner
    # dt.  The flat sections are generated at the controller rate and the crawl
    # is resampled onto their grid before assembly, so by the time the exporter
    # sees the trajectory there is normally nothing left for it to resample.
    ratio = int(round(traversal.segment.dt_s / CONTROLLER_DT_S))
    expected_trajectory_rows = (traversal.segment.sample_count - 1) * ratio + 1
    assert prep_rows == CONTROLLER_TRANSFORM_ROWS
    assert paths.trajectory_row_count == expected_trajectory_rows
    assert paths.total_row_count == prep_rows + expected_trajectory_rows
    # Prep appears once, at the front only: every later row belongs to a segment.
    assert np.allclose(phase[:prep_rows], 0.0)
    expected_first = to_hardware_order(traversal.segment.to_planner_commands())[0]
    assert np.allclose(commands[prep_rows], expected_first, atol=1e-6)

    metadata = json.loads(paths.metadata_path.read_text(encoding="utf-8"))
    assert metadata["rows"]["prep_rows"] == prep_rows
    assert metadata["rows"]["trajectory_start_row"] == CONTROLLER_TRANSFORM_ROWS
    assert metadata["rows"]["planner_dt_s"] == pytest.approx(traversal.segment.dt_s)
    # The crawl's own planning period stays recoverable from the request block.
    assert metadata["request"]["dt_s"] == pytest.approx(TRAVERSAL_REQUEST.dt_s)
    assert metadata["rows"]["controller_dt_s"] == pytest.approx(CONTROLLER_DT_S)
    assert metadata["rows"]["resample_ratio"] == ratio
    assert metadata["rows"]["total_rows"] == len(commands)
    assert metadata["motion_model"]["periodic_rolling_walk_spliced"] is False
    assert metadata["segments"][0]["csv_start_row"] == prep_rows
    assert metadata["segments"][-1]["csv_end_row"] == len(commands) - 1

    report = json.loads(paths.validation_path.read_text(encoding="utf-8"))
    assert report["all_boundaries_passed"] is True
    assert report["traversal"]["traversal_completed"] is True
    assert report["not_checked"]["full_leg_wheel_body_collision_geometry"] is True
    assert report["worst_case"]["boundary_joint_position_error_rad"] < 1e-9


def test_metadata_maps_every_csv_row_back_to_a_segment(tmp_path, traversal):
    paths = write_obstacle_walk_csv(traversal, tmp_path / "rows.csv")
    metadata = json.loads(paths.metadata_path.read_text(encoding="utf-8"))
    covered = np.zeros(paths.trajectory_row_count, dtype=bool)
    for segment in metadata["segments"]:
        start = segment["csv_start_row"] - paths.prep_row_count
        end = segment["csv_end_row"] - paths.prep_row_count
        covered[start : end + 1] = True
    assert covered.all()


def test_hardware_export_rejects_transform_duration_mismatch(tmp_path, traversal):
    with pytest.raises(ValueError, match="exactly 5000 transform rows"):
        write_obstacle_walk_csv(
            traversal, tmp_path / "bad_prep.csv", prep_duration_s=0.5
        )


def test_support_polygon_margin_sign_convention():
    """Positive is inside; the diagonal of a rectangle gives ~zero margin."""

    square = np.array([[1.0, 1.0], [1.0, -1.0], [-1.0, -1.0], [-1.0, 1.0]])
    assert support_polygon_margin_m(np.array([0.0, 0.0]), square) == pytest.approx(1.0)
    assert support_polygon_margin_m(np.array([2.0, 0.0]), square) < 0.0
    triangle = np.array([[0.255, -0.2117], [-0.255, -0.2117], [-0.255, 0.2117]])
    assert support_polygon_margin_m(np.array([0.0, 0.0]), triangle) == pytest.approx(
        0.0, abs=1e-9
    )


def test_lateral_sway_takes_the_smallest_sufficient_shift():
    """A bigger shift would raise the margin further, so 'smallest' matters."""

    triangle = np.array([[0.30, -0.21, 0.0], [-0.25, -0.21, 0.0], [-0.25, 0.21, 0.0]])
    required = 0.02
    swayed_y, margin = choose_lateral_sway(
        0.0, 0.0, triangle, maximum_sway_m=0.09, candidate_count=33,
        required_margin_m=required,
    )
    assert margin >= required
    assert swayed_y < 0.0  # towards the two right-hand feet
    best_y, best_margin = choose_lateral_sway(
        0.0, 0.0, triangle, maximum_sway_m=0.09, candidate_count=33,
        required_margin_m=1e9,  # unreachable: forces the best-effort branch
    )
    assert best_margin >= margin
    assert abs(best_y) >= abs(swayed_y)


def test_lateral_sway_reports_the_best_margin_when_none_suffices():
    collinear = np.array([[0.0, -0.2, 0.0], [0.0, 0.0, 0.0], [0.0, 0.2, 0.0]])
    _y, margin = choose_lateral_sway(
        0.5, 0.0, collinear, maximum_sway_m=0.05, candidate_count=11,
        required_margin_m=0.02,
    )
    assert margin < 0.02
    assert np.isfinite(margin)
