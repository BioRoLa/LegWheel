"""Day 8--9 Step 8 tests: touchdown velocity, and the only path to ``valid``."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    SwingFailure,
    build_leg_on_surface_scene_2d,
    build_swing_request_2d,
    flat_to_flat_swing_request_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_path_2d import (
    generate_terrain_aware_swing_path_2d,
    swing_path_endpoint_report,
    terrain_aware_apex_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_trajectory_2d import (
    solve_swing_joint_trajectory_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_collision_2d import (
    check_swing_trajectory_collisions_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_touchdown_2d import (
    validate_swing_touchdown_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_velocity_2d import (
    validate_swing_touchdown_velocity_2d,
)

SAMPLE_COUNT = 15
ARC_SAMPLES = 61
LEG_ARC_SAMPLES = 241
THETA_RAD = np.deg2rad(60.0)
CLEARANCE_M = 0.03


def _clean_request(height_m: float = 0.04):
    """A swing that passes every earlier step, so Step 8 decides the outcome."""

    obstacle = {
        "obstacle_x_start_m": 0.20,
        "obstacle_width_m": 0.45,
        "obstacle_height_m": height_m,
        "obstacle_id": "test_obstacle",
        "arc_samples": LEG_ARC_SAMPLES,
    }
    low = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.00, 0.00, **obstacle)
    high = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.36, height_m, **obstacle)
    return replace(
        build_swing_request_2d(low, high, clearance_m=CLEARANCE_M), sample_count=SAMPLE_COUNT
    )


def _through_step7(request, **path_kwargs):
    solved, _ = solve_swing_joint_trajectory_2d(
        generate_terrain_aware_swing_path_2d(request, **path_kwargs)
    )
    checked, _, _ = check_swing_trajectory_collisions_2d(solved, arc_samples=ARC_SAMPLES)
    validated, _, _ = validate_swing_touchdown_2d(checked)
    return validated


def test_a_clean_swing_finally_becomes_valid():
    """The first and only place ``valid=True`` can be reached."""

    request = _clean_request()

    final, velocity = validate_swing_touchdown_velocity_2d(_through_step7(request))

    assert velocity.passed
    assert final.valid is True
    assert final.failure is SwingFailure.NONE
    assert final.failure_detail is None
    assert final.touchdown_normal_speed_mps is not None
    assert final.touchdown_tangential_speed_mps is not None


def test_the_first_version_lands_with_zero_normal_speed():
    request = _clean_request()

    _, velocity = validate_swing_touchdown_velocity_2d(_through_step7(request))

    assert velocity.normal_velocity_mps == pytest.approx(0.0, abs=1e-12)
    assert velocity.tangential_velocity_mps == pytest.approx(0.0, abs=1e-12)
    assert velocity.speed_mps == pytest.approx(0.0, abs=1e-12)
    assert velocity.surface_id == request.target.target_terrain_surface_id


@pytest.mark.parametrize("drop_m", [0.003, 0.006, 0.012])
def test_normal_speed_matches_the_bezier_endpoint_derivative(drop_m):
    """``P'(1) = 5 (P5 - P4)``, so raising P4 by d gives 5 d / T downward."""

    request = _clean_request()

    _, velocity = validate_swing_touchdown_velocity_2d(
        _through_step7(request, touchdown_drop_m=drop_m)
    )

    expected = 5.0 * drop_m / request.swing_duration_s
    assert velocity.normal_velocity_mps == pytest.approx(-expected, rel=1e-9)
    assert velocity.approaching is True


def test_a_hard_landing_is_rejected():
    request = _clean_request()
    limit = request.constraints.touchdown_normal_speed_max_mps
    # 5 d / T = 2 * limit
    drop_m = 2.0 * limit * request.swing_duration_s / 5.0

    final, velocity = validate_swing_touchdown_velocity_2d(
        _through_step7(request, touchdown_drop_m=drop_m)
    )

    assert velocity.normal_ok is False
    assert velocity.passed is False
    assert velocity.failure is SwingFailure.TOUCHDOWN_VELOCITY_TOO_HIGH
    assert final.valid is False
    assert final.failure is SwingFailure.TOUCHDOWN_VELOCITY_TOO_HIGH
    assert "exceeds" in final.failure_detail


def test_step8_refuses_to_judge_a_trajectory_that_skipped_step6():
    """``valid=True`` must not be reachable by calling this step early."""

    request = _clean_request()
    solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(request))

    with pytest.raises(ValueError, match="collision-checked"):
        validate_swing_touchdown_velocity_2d(solved)


def test_step8_refuses_to_judge_a_trajectory_that_skipped_step7():
    request = _clean_request()
    solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(request))
    checked, _, _ = check_swing_trajectory_collisions_2d(solved, arc_samples=ARC_SAMPLES)

    with pytest.raises(ValueError, match="touchdown contact state"):
        validate_swing_touchdown_velocity_2d(checked)


def test_an_earlier_failure_is_never_promoted_to_valid():
    """A swing that collides in flight stays invalid however softly it lands."""

    obstacle = {
        "obstacle_x_start_m": 0.10,
        "obstacle_width_m": 0.35,
        "obstacle_height_m": 0.04,
        "obstacle_id": "test_obstacle",
        "arc_samples": LEG_ARC_SAMPLES,
    }
    low = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.00, 0.00, **obstacle)
    high = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.20, 0.04, **obstacle)
    request = replace(
        build_swing_request_2d(low, high, clearance_m=CLEARANCE_M), sample_count=21
    )
    through_7 = _through_step7(request)
    assert through_7.failure is SwingFailure.TERRAIN_COLLISION

    final, velocity = validate_swing_touchdown_velocity_2d(through_7)

    assert velocity.passed is True          # the landing itself is gentle
    assert final.valid is False             # and the swing is still unusable
    assert final.failure is SwingFailure.TERRAIN_COLLISION


def test_tangential_component_is_measured_against_the_body():
    """The diagnostic a body-velocity-matched touchdown would later tune."""

    request = _clean_request()

    _, velocity = validate_swing_touchdown_velocity_2d(_through_step7(request))

    hip_speed = (
        request.hip_trajectory.displacement_world_xz_m[0] / request.swing_duration_s
    )
    assert velocity.hip_tangential_velocity_mps == pytest.approx(hip_speed)
    # The first version lands with a stationary contact point while the body
    # keeps moving, so the mismatch is the whole body speed.
    assert velocity.tangential_mismatch_mps == pytest.approx(-hip_speed)


def test_touchdown_drop_does_not_break_the_step2_guarantees():
    request = _clean_request()
    apex_m = terrain_aware_apex_2d(request).apex_height_m

    report = swing_path_endpoint_report(
        generate_terrain_aware_swing_path_2d(request, touchdown_drop_m=0.01)
    )

    assert report["start_position_error_mm"] == pytest.approx(0.0, abs=1e-9)
    assert report["target_position_error_mm"] == pytest.approx(0.0, abs=1e-9)
    assert report["achieved_apex_z_m"] == pytest.approx(apex_m, abs=1e-4)
    assert report["touchdown_speed_mps"] > 0.0


def test_liftoff_rise_gives_an_upward_departure():
    request = flat_to_flat_swing_request_2d(arc_samples=LEG_ARC_SAMPLES)
    request = replace(request, sample_count=SAMPLE_COUNT)

    result = generate_terrain_aware_swing_path_2d(request, liftoff_rise_m=0.01)

    liftoff_velocity = result.samples[0].velocity_world_xz_mps
    assert liftoff_velocity[1] == pytest.approx(
        5.0 * 0.01 / request.swing_duration_s, rel=1e-9
    )
    assert liftoff_velocity[0] == pytest.approx(0.0, abs=1e-12)
