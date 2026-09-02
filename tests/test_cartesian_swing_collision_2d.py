"""Day 8--9 Step 6 tests: full-leg collision checking over a swing trajectory."""

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
from hybrid_note.scripts.experiments.cartesian_swing_path_2d import (
    generate_terrain_aware_swing_path_2d,
    path_point_clearance_report_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_trajectory_2d import (
    solve_swing_joint_trajectory_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_collision_2d import (
    check_swing_trajectory_collisions_2d,
    collision_frame_rows,
    swing_sample_scene_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_ik_2d import (
    rim_contact_point_world_xz_m,
)

# Kept small on purpose: the terrain query is per rim sample, so a full-detail
# check of a 101-sample trajectory takes tens of seconds.
SAMPLE_COUNT = 21
ARC_SAMPLES = 61
LEG_ARC_SAMPLES = 121
THETA_RAD = np.deg2rad(60.0)
CLEARANCE_M = 0.03


def _step_up_request(height_m: float = 0.04, *, sample_count: int = SAMPLE_COUNT):
    obstacle = {
        "obstacle_x_start_m": 0.10,
        "obstacle_width_m": 0.35,
        "obstacle_height_m": height_m,
        "obstacle_id": "test_obstacle",
        "arc_samples": LEG_ARC_SAMPLES,
    }
    low = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.00, 0.00, **obstacle)
    high = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, 0.20, height_m, **obstacle)
    request = build_swing_request_2d(low, high, clearance_m=CLEARANCE_M)
    return replace(request, sample_count=sample_count)


def _checked(request, *, arc_samples: int = ARC_SAMPLES):
    solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(request))
    return check_swing_trajectory_collisions_2d(solved, arc_samples=arc_samples)


def test_the_contact_point_clears_but_the_leg_body_does_not():
    """Step 6's completion criterion, and it is not a contrived case.

    The flat -> 40 mm swing that Steps 2--5 all accepted puts the planned
    contact point safely over the obstacle, and still drives the tyre into the
    obstacle's vertical front face.
    """

    request = _step_up_request()
    solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(request))

    # The point-level view -- everything Steps 2--3 could see -- is clean.
    point_report = path_point_clearance_report_2d(solved)
    assert point_report["penetrating_sample_count"] == 0
    assert point_report["point_path_clears_terrain"] is True

    checked, report, _ = check_swing_trajectory_collisions_2d(solved, arc_samples=ARC_SAMPLES)

    assert report.collision_free is False
    assert report.collision_type == "vertical_face_collision"
    assert report.collision_surface_id.endswith("_front")
    assert report.collision_depth_m > 0.0
    assert report.minimum_clearance_m < 0.0
    assert checked.failure is SwingFailure.TERRAIN_COLLISION
    assert checked.failure_sample_index == report.first_collision_index
    assert checked.valid is False


def test_a_flat_swing_is_collision_free():
    request = replace(
        flat_to_flat_swing_request_2d(arc_samples=LEG_ARC_SAMPLES, clearance_m=CLEARANCE_M),
        sample_count=SAMPLE_COUNT,
    )

    checked, report, _ = _checked(request)

    assert report.collision_free is True
    assert report.first_collision_index is None
    assert report.collision_type is None
    assert all(sample.collision_free for sample in checked.samples)
    assert checked.failure is SwingFailure.NOT_EVALUATED
    assert "Steps 7-8" in checked.failure_detail


def test_touching_endpoints_are_not_reported_as_collisions():
    """Both ends rest on the terrain by construction; that is not a collision."""

    request = replace(
        flat_to_flat_swing_request_2d(arc_samples=LEG_ARC_SAMPLES, clearance_m=CLEARANCE_M),
        sample_count=SAMPLE_COUNT,
    )

    checked, _, queries = _checked(request)

    assert checked.samples[0].collision_free is True
    assert checked.samples[-1].collision_free is True
    # They do register as contacts, which is exactly what a swing endpoint is.
    assert queries[0].candidates
    assert queries[-1].candidates


def test_report_indices_agree_with_the_per_sample_flags():
    checked, report, _ = _checked(_step_up_request())

    colliding = [sample.index for sample in checked.samples if sample.collision_free is False]
    assert colliding
    assert report.first_collision_index == min(colliding)
    assert report.checked_sample_count == len(checked.samples)

    clearances = [
        sample.terrain_clearance_m
        for sample in checked.samples
        if sample.terrain_clearance_m is not None
    ]
    assert report.minimum_clearance_m == pytest.approx(min(clearances))
    assert checked.minimum_terrain_clearance_m == pytest.approx(min(clearances))


def test_step6_fills_the_schema_frozen_in_step1_without_changing_it():
    checked, _, _ = _checked(_step_up_request(height_m=0.02))

    row = swing_result_frame_rows(checked)[5]
    assert row["collision_free"] in (True, False)
    assert row["terrain_clearance_mm"] is not None
    # Still owned by Steps 1--5.
    assert row["theta_deg"] is not None
    assert row["ik_converged"] is True


def test_rebuilt_scene_places_the_planned_contact_point():
    """The geometry Step 6 checks must be the geometry Step 5 solved."""

    request = _step_up_request()
    solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(request))

    for sample in solved.samples[::5]:
        scene = swing_sample_scene_2d(solved, sample, arc_samples=ARC_SAMPLES)
        assert scene.theta_rad == pytest.approx(sample.theta_rad)
        assert scene.beta_rad == pytest.approx(sample.beta_rad)
        placed = rim_contact_point_world_xz_m(
            scene.theta_rad, scene.beta_rad, sample.rim, sample.alpha_rad, scene.hip_pose
        )
        assert placed == pytest.approx(sample.position_world_xz_m, abs=1e-4)


def test_an_earlier_failure_is_not_overwritten_by_a_collision():
    """Step 6 adds evidence; it does not relabel a Step 5 failure."""

    request = _step_up_request()
    strict = replace(
        request,
        constraints=replace(request.constraints, max_joint_step_rad=np.deg2rad(0.2)),
    )
    solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(strict))
    assert solved.failure is SwingFailure.JOINT_DISCONTINUITY

    checked, report, _ = check_swing_trajectory_collisions_2d(solved, arc_samples=ARC_SAMPLES)

    assert checked.failure is SwingFailure.JOINT_DISCONTINUITY
    assert report.collision_free is False
    assert "Also:" in checked.failure_detail


def test_collision_frame_rows_line_up_with_the_queries():
    checked, _, queries = _checked(_step_up_request())

    rows = collision_frame_rows(checked, queries)

    assert len(rows) == len(checked.samples)
    assert rows[0]["index"] == 0
    assert any(row["vertical_face_collisions"] > 0 for row in rows)
    assert all("statuses" in row for row in rows)

    with pytest.raises(ValueError, match="same length"):
        collision_frame_rows(checked, queries[:-1])


def test_step6_requires_a_joint_solution():
    request = _step_up_request()
    path_only = generate_terrain_aware_swing_path_2d(request)

    with pytest.raises(ValueError, match="joint solution"):
        check_swing_trajectory_collisions_2d(path_only, arc_samples=ARC_SAMPLES)


def test_arc_sampling_density_changes_detail_but_not_the_verdict():
    """How far the default can be trusted, measured rather than assumed."""

    request = _step_up_request(sample_count=15)
    solved, _ = solve_swing_joint_trajectory_2d(generate_terrain_aware_swing_path_2d(request))

    coarse = check_swing_trajectory_collisions_2d(solved, arc_samples=61)[1]
    fine = check_swing_trajectory_collisions_2d(solved, arc_samples=121)[1]

    # Stable: whether the swing collides at all, and how deep the worst
    # overlap goes.
    assert coarse.collision_free == fine.collision_free is False
    assert coarse.minimum_clearance_m == pytest.approx(fine.minimum_clearance_m, abs=5e-5)
    # Not stable to the sample: the onset can move by one trajectory sample,
    # and when two overlaps are nearly equally deep, which one is reported as
    # primary can swap.  Both are recorded here so neither is quietly assumed.
    assert abs(coarse.first_collision_index - fine.first_collision_index) <= 1
