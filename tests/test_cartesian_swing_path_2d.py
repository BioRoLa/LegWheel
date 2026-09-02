"""Day 8--9 Step 2 tests: the Cartesian quintic Bezier swing path."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from legwheel.planners.hybrid import RimId

from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    SwingFailure,
    SwingTarget2D,
    build_leg_on_surface_scene_2d,
    build_swing_request_2d,
    flat_to_flat_swing_request_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_path_2d import (
    generate_swing_path_2d,
    quintic_swing_control_points_2d,
    sample_quintic_swing_2d,
    swing_apex_height_m,
    swing_path_endpoint_report,
)

ARC_SAMPLES = 121
THETA_RAD = np.deg2rad(60.0)
CLEARANCE_M = 0.03


def _height_case_request(height_m: float, *, descend: bool = False):
    """The four planning-note cases: 0->0, 0->20, 0->40 and 40->0 mm."""

    if height_m == 0.0:
        return flat_to_flat_swing_request_2d(
            arc_samples=ARC_SAMPLES, clearance_m=CLEARANCE_M
        )
    obstacle = {
        "obstacle_x_start_m": 0.10,
        "obstacle_width_m": 0.35,
        "obstacle_height_m": height_m,
        "obstacle_id": "test_obstacle",
        "arc_samples": ARC_SAMPLES,
    }
    low = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.00, 0.00, **obstacle)
    high = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.20, height_m, **obstacle)
    scenes = (high, low) if descend else (low, high)
    return build_swing_request_2d(*scenes, clearance_m=CLEARANCE_M)


HEIGHT_CASES = [
    pytest.param(0.0, False, id="flat_to_flat"),
    pytest.param(0.02, False, id="flat_to_20mm"),
    pytest.param(0.04, False, id="flat_to_40mm"),
    pytest.param(0.04, True, id="40mm_to_flat"),
]


@pytest.mark.parametrize("height_m,descend", HEIGHT_CASES)
def test_one_generator_hits_every_endpoint_and_duration(height_m, descend):
    request = _height_case_request(height_m, descend=descend)

    report = swing_path_endpoint_report(generate_swing_path_2d(request))

    assert report["start_position_error_mm"] == pytest.approx(0.0, abs=1e-9)
    assert report["target_position_error_mm"] == pytest.approx(0.0, abs=1e-9)
    assert report["t_first_s"] == pytest.approx(0.0)
    assert report["t_last_s"] == pytest.approx(request.swing_duration_s)
    assert report["duration_error_s"] == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("height_m,descend", HEIGHT_CASES)
def test_first_version_endpoint_velocities_are_zero(height_m, descend):
    request = _height_case_request(height_m, descend=descend)

    report = swing_path_endpoint_report(generate_swing_path_2d(request))

    assert report["liftoff_speed_mps"] == pytest.approx(0.0, abs=1e-12)
    assert report["touchdown_speed_mps"] == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("height_m,descend", HEIGHT_CASES)
def test_apex_clears_the_higher_end_by_the_clearance(height_m, descend):
    request = _height_case_request(height_m, descend=descend)
    dense = replace(request, sample_count=4001)

    report = swing_path_endpoint_report(generate_swing_path_2d(dense))

    z_start = request.start.contact_point_world_xz_m[1]
    z_target = request.target.target_point_world_xz_m[1]
    expected = max(z_start, z_target) + CLEARANCE_M
    assert report["requested_apex_z_m"] == pytest.approx(expected)
    assert report["achieved_apex_z_m"] == pytest.approx(expected, abs=1e-9)
    assert 0.0 < report["apex_at_s"] < 1.0


@pytest.mark.parametrize("height_m,descend", HEIGHT_CASES)
def test_contact_point_never_travels_backwards(height_m, descend):
    request = _height_case_request(height_m, descend=descend)

    result = generate_swing_path_2d(request)
    report = swing_path_endpoint_report(result)

    assert report["x_monotone"]
    step_x = np.diff(result.positions_world_xz_m[:, 0])
    expected_sign = np.sign(request.horizontal_span_m)
    assert np.all(step_x * expected_sign >= -1e-12)


def test_apex_rule_uses_the_higher_end_not_the_target():
    """A descending swing must still clear the ledge it starts on."""

    descending = _height_case_request(0.04, descend=True)

    assert swing_apex_height_m(descending) == pytest.approx(0.04 + CLEARANCE_M)


def test_velocity_is_the_analytic_derivative_not_a_difference():
    request = replace(_height_case_request(0.04), sample_count=2001)

    result = generate_swing_path_2d(request)
    positions = result.positions_world_xz_m
    velocities = result.velocities_world_xz_mps
    times = result.time_s

    central = (positions[2:] - positions[:-2]) / (times[2:] - times[:-2])[:, None]
    assert np.allclose(central, velocities[1:-1], atol=1e-4)


def test_default_control_polygon_matches_the_planning_note():
    request = _height_case_request(0.04)
    start = request.start.contact_point_world_xz_m
    target = request.target.target_point_world_xz_m

    controls = quintic_swing_control_points_2d(start, target, swing_apex_height_m(request))

    assert controls.shape == (6, 2)
    assert controls[0] == pytest.approx(start)
    assert controls[5] == pytest.approx(target)
    # P1 = P0 and P4 = P5 are what make both endpoint velocities vanish.
    assert controls[1] == pytest.approx(controls[0])
    assert controls[4] == pytest.approx(controls[5])
    # The solved P2/P3 height overshoots the apex, because the curve only
    # approaches its interior control points.
    assert controls[2, 1] == pytest.approx(controls[3, 1])
    assert controls[2, 1] > swing_apex_height_m(request)


def test_liftoff_and_touchdown_knobs_change_the_endpoint_velocity():
    request = _height_case_request(0.04)

    shaped = generate_swing_path_2d(request, liftoff_fraction=0.1, touchdown_fraction=0.1)
    report = swing_path_endpoint_report(shaped)

    assert report["liftoff_speed_mps"] > 0.0
    assert report["touchdown_speed_mps"] > 0.0
    assert report["start_position_error_mm"] == pytest.approx(0.0, abs=1e-9)
    assert report["target_position_error_mm"] == pytest.approx(0.0, abs=1e-9)


def test_alpha_walks_from_the_start_contact_to_the_target_contact():
    request = _height_case_request(0.04)
    rotated_target = SwingTarget2D(
        request.target.target_point_world_xz_m,
        request.target.target_rim,
        np.deg2rad(30.0),
        request.target.target_terrain_surface_id,
        clearance_m=CLEARANCE_M,
    )
    request = replace(request, target=rotated_target)

    result = generate_swing_path_2d(request)

    assert result.samples[0].alpha_rad == pytest.approx(request.start.alpha_rad)
    assert result.samples[-1].alpha_rad == pytest.approx(np.deg2rad(30.0))
    alphas = np.array([sample.alpha_rad for sample in result.samples])
    assert np.all(np.diff(alphas) >= -1e-12)


def test_endpoint_rims_come_from_the_contact_states_not_from_alpha():
    """Alpha is ambiguous on an arc seam; the endpoints must not be.

    The +-40 deg boundary belongs to two arcs whose points are 45 mm apart, so
    deriving the rim from alpha there could plan a touchdown on a different rim
    than the target named -- silently, and 45 mm away.
    """

    request = _height_case_request(0.0)
    seam_target = SwingTarget2D(
        request.target.target_point_world_xz_m,
        RimId.RIGHT,
        np.deg2rad(40.0),
        request.target.target_terrain_surface_id,
        clearance_m=CLEARANCE_M,
    )
    request = replace(request, target=seam_target)

    result = generate_swing_path_2d(request)

    assert result.samples[0].rim is request.start.rim
    assert result.samples[-1].rim is RimId.RIGHT
    assert result.samples[-1].alpha_rad == pytest.approx(np.deg2rad(40.0))


def test_rim_identity_follows_alpha_across_the_arc_boundary():
    request = _height_case_request(0.04)
    right_rim_alpha = np.deg2rad(94.8)
    request = replace(
        request,
        start=replace(request.start, rim="right_rim", alpha_rad=right_rim_alpha),
    )

    result = generate_swing_path_2d(request)

    assert result.samples[0].rim.value == "right_rim"
    assert result.samples[-1].rim.value == "foot_rim"
    rims = [sample.rim.value for sample in result.samples]
    # Exactly one handover, and it happens where alpha crosses 40 deg.
    changes = [i for i in range(1, len(rims)) if rims[i] != rims[i - 1]]
    assert len(changes) == 1
    assert np.rad2deg(result.samples[changes[0]].alpha_rad) <= 40.0
    assert np.rad2deg(result.samples[changes[0] - 1].alpha_rad) > 40.0


def test_an_ill_posed_request_returns_a_reason_not_a_path():
    request = _height_case_request(0.0)
    broken = replace(
        request, target=replace(request.target, target_terrain_surface_id="nope")
    )

    result = generate_swing_path_2d(broken)

    assert result.samples == ()
    assert result.valid is False
    assert result.failure is SwingFailure.INVALID_REQUEST
    assert "does not contain" in result.failure_detail


def test_a_generated_path_is_not_yet_a_valid_swing():
    """Step 2 produces geometry, not feasibility; Steps 4-8 own ``valid``."""

    result = generate_swing_path_2d(_height_case_request(0.04))

    assert result.sample_count == result.request.sample_count
    assert result.valid is False
    assert result.failure is SwingFailure.NOT_EVALUATED
    assert all(sample.theta_rad is None for sample in result.samples)
    assert all(sample.collision_free is None for sample in result.samples)


def test_sampler_rejects_a_degenerate_time_or_polygon():
    controls = quintic_swing_control_points_2d([0.0, 0.0], [0.2, 0.0], 0.05)

    with pytest.raises(ValueError):
        sample_quintic_swing_2d(controls, duration_s=0.0, sample_count=101)
    with pytest.raises(ValueError):
        sample_quintic_swing_2d(controls, duration_s=0.6, sample_count=1)
    with pytest.raises(ValueError):
        sample_quintic_swing_2d(controls[:5], duration_s=0.6, sample_count=101)


# ---------------------------------------------------------------------------
# Step 3: terrain-aware apex
# ---------------------------------------------------------------------------

from hybrid_note.scripts.experiments.cartesian_swing_path_2d import (  # noqa: E402
    corridor_obstacle_top_surfaces_2d,
    generate_terrain_aware_swing_path_2d,
    path_point_clearance_report_2d,
    terrain_aware_apex_2d,
)

OBSTACLE_HEIGHT_M = 0.04


def _obstacle_between_request(*, obstacle_x_start_m: float = 0.10, width_m: float = 0.15):
    """Case E: both contacts on the ground, one obstacle in between."""

    obstacle = {
        "obstacle_x_start_m": obstacle_x_start_m,
        "obstacle_width_m": width_m,
        "obstacle_height_m": OBSTACLE_HEIGHT_M,
        "obstacle_id": "test_obstacle",
        "arc_samples": ARC_SAMPLES,
    }
    start = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.00, 0.00, **obstacle)
    target = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.40, 0.00, **obstacle)
    return build_swing_request_2d(start, target, clearance_m=CLEARANCE_M)


def test_corridor_selects_only_overlapping_obstacle_tops():
    request = _obstacle_between_request()
    terrain = request.terrain

    inside = corridor_obstacle_top_surfaces_2d(terrain, 0.0, 0.40)
    assert [surface.surface_id for surface in inside] == ["test_obstacle_top"]

    beyond = corridor_obstacle_top_surfaces_2d(terrain, 0.30, 0.40)
    assert beyond == ()

    widened = corridor_obstacle_top_surfaces_2d(terrain, 0.30, 0.40, margin_m=0.10)
    assert [surface.surface_id for surface in widened] == ["test_obstacle_top"]


def test_apex_rises_over_an_obstacle_between_two_equal_height_contacts():
    request = _obstacle_between_request()

    construction = terrain_aware_apex_2d(request)

    assert construction.obstacle_governs
    assert construction.governing_source == "test_obstacle_top"
    assert construction.base_height_m == pytest.approx(OBSTACLE_HEIGHT_M, abs=1e-6)
    assert construction.apex_height_m == pytest.approx(OBSTACLE_HEIGHT_M + CLEARANCE_M, abs=1e-6)
    # Step 2's rule would have stopped at the two ground contacts.
    assert swing_apex_height_m(request) == pytest.approx(CLEARANCE_M, abs=1e-6)


@pytest.mark.parametrize("height_m,descend", HEIGHT_CASES)
def test_apex_is_unchanged_when_no_obstacle_stands_above_both_ends(height_m, descend):
    request = _height_case_request(height_m, descend=descend)

    construction = terrain_aware_apex_2d(request)

    assert not construction.obstacle_governs
    assert construction.apex_height_m == pytest.approx(swing_apex_height_m(request))


def test_terrain_aware_path_lifts_the_contact_point_out_of_the_obstacle():
    request = _obstacle_between_request()

    step2 = path_point_clearance_report_2d(generate_swing_path_2d(request))
    step3 = path_point_clearance_report_2d(generate_terrain_aware_swing_path_2d(request))

    assert step2["penetrating_sample_count"] > 0
    assert step2["point_path_clears_terrain"] is False
    assert step3["penetrating_sample_count"] == 0
    assert step3["point_path_clears_terrain"] is True
    assert step3["minimum_obstacle_clearance_mm"] > 0.0


def test_the_apex_rule_does_not_deliver_the_clearance_along_the_whole_obstacle():
    """The heuristic clears the peak, not the corridor: keep that visible."""

    request = _obstacle_between_request()

    report = path_point_clearance_report_2d(generate_terrain_aware_swing_path_2d(request))

    # The closest approach happens at a leading obstacle edge, where the curve
    # is still climbing, and is well under the requested clearance.
    assert report["minimum_obstacle_clearance_mm"] < CLEARANCE_M * 1e3
    assert report["worst_obstacle_sample_x_m"] == pytest.approx(0.10, abs=0.02)


def test_terrain_aware_generator_keeps_the_step2_endpoint_guarantees():
    request = _obstacle_between_request()

    report = swing_path_endpoint_report(generate_terrain_aware_swing_path_2d(request))

    assert report["start_position_error_mm"] == pytest.approx(0.0, abs=1e-9)
    assert report["target_position_error_mm"] == pytest.approx(0.0, abs=1e-9)
    assert report["t_last_s"] == pytest.approx(request.swing_duration_s)
    assert report["liftoff_speed_mps"] == pytest.approx(0.0, abs=1e-12)
    assert report["touchdown_speed_mps"] == pytest.approx(0.0, abs=1e-12)
    assert report["achieved_apex_z_m"] == pytest.approx(
        terrain_aware_apex_2d(request).apex_height_m, abs=1e-5
    )


def test_terrain_aware_generator_still_rejects_an_ill_posed_request():
    request = _obstacle_between_request()
    broken = replace(
        request, target=replace(request.target, target_terrain_surface_id="nope")
    )

    result = generate_terrain_aware_swing_path_2d(broken)

    assert result.samples == ()
    assert result.failure is SwingFailure.INVALID_REQUEST
