"""Day 8--9 Step 7 tests: touchdown contact validation."""

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
    validate_swing_request_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_path_2d import (
    generate_terrain_aware_swing_path_2d,
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

SAMPLE_COUNT = 21
LEG_ARC_SAMPLES = 241
THETA_RAD = np.deg2rad(60.0)
CLEARANCE_M = 0.03


def _case_request(height_m: float, *, descend: bool = False):
    if height_m == 0.0:
        return replace(
            flat_to_flat_swing_request_2d(
                arc_samples=LEG_ARC_SAMPLES, clearance_m=CLEARANCE_M
            ),
            sample_count=SAMPLE_COUNT,
        )
    obstacle = {
        "obstacle_x_start_m": 0.10,
        "obstacle_width_m": 0.35,
        "obstacle_height_m": height_m,
        "obstacle_id": "test_obstacle",
        "arc_samples": LEG_ARC_SAMPLES,
    }
    low = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.00, 0.00, **obstacle)
    high = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.20, height_m, **obstacle)
    scenes = (high, low) if descend else (low, high)
    return replace(
        build_swing_request_2d(*scenes, clearance_m=CLEARANCE_M), sample_count=SAMPLE_COUNT
    )


def _solved(request):
    return solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(request))[0]


HEIGHT_CASES = [
    pytest.param(0.0, False, id="flat_to_flat"),
    pytest.param(0.02, False, id="flat_to_20mm"),
    pytest.param(0.04, False, id="flat_to_40mm"),
    pytest.param(0.04, True, id="40mm_to_flat"),
]


@pytest.mark.parametrize("height_m,descend", HEIGHT_CASES)
def test_each_case_lands_in_the_requested_contact_state(height_m, descend):
    request = _case_request(height_m, descend=descend)

    _, validation, candidate = validate_swing_touchdown_2d(_solved(request))

    assert validation.passed
    assert validation.failure is None
    assert validation.position_error_m < request.constraints.touchdown_position_tolerance_m
    assert validation.rim_match
    assert validation.surface_match
    assert validation.alpha_ok
    assert validation.penetration_free
    assert validation.joint_limits_ok
    assert candidate is not None
    assert candidate.rim is request.target.target_rim
    assert candidate.terrain_surface_id == request.target.target_terrain_surface_id


def test_touchdown_fills_the_result_fields_frozen_in_step1():
    request = _case_request(0.04)

    checked, validation, _ = validate_swing_touchdown_2d(_solved(request))

    assert checked.final_contact_error_m == pytest.approx(validation.position_error_m)
    assert checked.final_alpha_error_rad == pytest.approx(validation.alpha_error_rad)
    assert checked.final_rim is request.target.target_rim
    assert checked.final_terrain_surface_id == request.target.target_terrain_surface_id


def test_passing_touchdown_is_still_not_a_valid_swing():
    checked, validation, _ = validate_swing_touchdown_2d(_solved(_case_request(0.0)))

    assert validation.passed
    assert checked.valid is False
    assert checked.failure is SwingFailure.NOT_EVALUATED
    assert "Step 8" in checked.failure_detail


def test_landing_on_the_planned_point_but_not_the_planned_contact_is_rejected():
    """The counter-example Step 7 exists for.

    The requested rim point is placed to within a nanometre, on the requested
    rim -- and a *different* point of that rim is what actually touches the
    ground.  The pose is legal; the contact state is not the one the next
    rolling segment was planned against.
    """

    base = _case_request(0.0)
    request = replace(
        base,
        # Alpha has to sweep 0 -> 41 deg across the swing, so the trajectory
        # needs enough samples that Step 5 does not reject it for joint
        # discontinuity before Step 7 gets to look at the landing.
        sample_count=101,
        target=SwingTarget2D(
            base.target.target_point_world_xz_m,
            RimId.RIGHT,
            np.deg2rad(41.0),
            base.target.target_terrain_surface_id,
            clearance_m=CLEARANCE_M,
        ),
    )
    assert validate_swing_request_2d(request) == ()

    checked, validation, candidate = validate_swing_touchdown_2d(_solved(request))

    assert validation.position_error_m < 1e-6
    assert validation.rim_match
    assert validation.alpha_ok is False
    assert validation.alpha_error_rad > request.constraints.touchdown_alpha_tolerance_rad
    assert validation.passed is False
    assert validation.failure is SwingFailure.TOUCHDOWN_ALPHA_ERROR
    assert candidate.alpha_rad != pytest.approx(request.target.target_alpha_rad)
    # Step 5 rejects this same trajectory first, and keeps the headline: alpha
    # has to sweep across the +-40 deg rim seam, where the tyre surface has a
    # 45 mm physical gap, so the joints cannot follow it smoothly.  Step 7 is
    # an independent check, not a gate the earlier ones feed into -- which is
    # why the validation record is returned separately from the result.
    assert checked.failure is SwingFailure.JOINT_DISCONTINUITY
    assert "Also at touchdown" in checked.failure_detail
    assert "alpha error" in checked.failure_detail


def test_a_touchdown_across_a_rim_seam_is_rejected():
    """Landing on the far side of an arc seam is a re-grip, not a swing.

    Alpha = 40 deg is the boundary between the foot rim and the right upper
    tyre, and those two arcs are 45 mm apart in space.  Asking to touch down on
    the right rim there, from a path that approaches on the foot rim, forces a
    jump the joints cannot follow -- and the landing pose is buried in the
    ground.  Both steps say so.
    """

    base = _case_request(0.0)
    request = replace(
        base,
        sample_count=101,
        target=SwingTarget2D(
            base.target.target_point_world_xz_m,
            RimId.RIGHT,
            np.deg2rad(40.0),
            base.target.target_terrain_surface_id,
            clearance_m=CLEARANCE_M,
        ),
    )
    solved = _solved(request)

    # The path now ends on the rim the target named, so the position itself is
    # exact; what fails is everything around it.
    assert solved.samples[-1].rim is RimId.RIGHT
    assert solved.failure is SwingFailure.JOINT_DISCONTINUITY

    checked, validation, _ = validate_swing_touchdown_2d(solved)

    assert validation.position_error_m < 1e-6
    assert validation.passed is False
    assert validation.failure is SwingFailure.TERRAIN_COLLISION
    assert checked.failure is SwingFailure.JOINT_DISCONTINUITY


def test_step7_alone_can_reject_a_landing_the_earlier_steps_accept():
    """Not every bad touchdown is pre-empted: this one only Step 7 catches."""

    base = _case_request(0.0)
    request = replace(
        base,
        sample_count=101,
        target=SwingTarget2D(
            base.target.target_point_world_xz_m,
            RimId.FOOT,
            np.deg2rad(38.0),
            base.target.target_terrain_surface_id,
            clearance_m=CLEARANCE_M,
        ),
    )
    solved = _solved(request)
    assert solved.failure is SwingFailure.NOT_EVALUATED  # Steps 1-5 are happy

    checked, validation, _ = validate_swing_touchdown_2d(solved)

    assert validation.position_error_m < 1e-6
    assert validation.rim_match and validation.alpha_ok and validation.surface_match
    assert validation.penetration_free is False
    assert checked.failure is SwingFailure.TERRAIN_COLLISION
    assert checked.failure_sample_index == solved.samples[-1].index


def test_an_earlier_failure_keeps_the_headline():
    request = _case_request(0.04)
    collided, report, _ = check_swing_trajectory_collisions_2d(
        _solved(request), arc_samples=61
    )
    assert report.collision_free is False

    checked, validation, _ = validate_swing_touchdown_2d(collided)

    assert validation.passed  # the landing itself is fine
    assert checked.failure is SwingFailure.TERRAIN_COLLISION
    assert checked.failure_sample_index == report.first_collision_index


def test_touchdown_penetration_tolerance_defaults_to_the_contact_model():
    """Touchdown is supposed to be in contact, so the tolerance is the query's.

    Step 6 uses the IK's own accuracy (~10 um) because an airborne leg should
    not be touching anything at all.  A landing is different: the Day 3--5
    query calls anything within +-contact_tolerance of a surface "in contact",
    so it cannot tell "resting on it" from "just inside it", and the touchdown
    check must use that same resolution rather than a tighter one.
    """

    request = _case_request(0.0)
    solved = _solved(request)

    default = validate_swing_touchdown_2d(solved)[1]
    explicit = validate_swing_touchdown_2d(
        solved, penetration_tolerance_m=request.constraints.contact_tolerance_m
    )[1]

    assert default.penetration_detail == explicit.penetration_detail
    assert default.passed == explicit.passed is True

    # The looser tolerance is not a licence: asking for an off-centre foot-rim
    # point at the landing rotates the leg 4.3 mm into the ground, which is far
    # outside the contact model's resolution and is still rejected.
    buried = replace(
        request,
        sample_count=101,
        target=SwingTarget2D(
            request.target.target_point_world_xz_m,
            RimId.FOOT,
            np.deg2rad(38.0),
            request.target.target_terrain_surface_id,
            clearance_m=CLEARANCE_M,
        ),
    )
    buried_validation = validate_swing_touchdown_2d(_solved(buried))[1]
    assert buried_validation.penetration_free is False
    assert buried_validation.failure is SwingFailure.TERRAIN_COLLISION


def test_touchdown_measures_alpha_on_the_grid_the_target_was_defined_on():
    request = _case_request(0.02)

    fine = validate_swing_touchdown_2d(_solved(request), arc_samples=241)[1]
    coarse = validate_swing_touchdown_2d(_solved(request), arc_samples=61)[1]

    assert fine.alpha_error_rad == pytest.approx(0.0, abs=1e-12)
    # A coarser grid cannot represent the target alpha exactly, so it reports a
    # quantisation error rather than a planning error.
    assert coarse.alpha_error_rad >= fine.alpha_error_rad


def test_step7_requires_a_joint_solution():
    request = _case_request(0.0)
    path_only = generate_terrain_aware_swing_path_2d(request)

    with pytest.raises(ValueError, match="joint solution"):
        validate_swing_touchdown_2d(path_only)
