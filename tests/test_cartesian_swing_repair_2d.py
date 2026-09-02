"""Day 8--9: repairing a colliding swing with the endpoint knobs."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    SwingFailure,
    build_leg_on_surface_scene_2d,
    build_swing_request_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_planner_2d import (
    generate_swing_2d,
    minimum_duration_for_touchdown_drop_s,
    repair_swing_2d,
    feasible_regression_cases_2d,
)

SAMPLE_COUNT = 31
ARC_SAMPLES = 61
THETA_RAD = np.deg2rad(60.0)
CLEARANCE_M = 0.03

CLOSE_STEP = {
    "obstacle_x_start_m": 0.10,
    "obstacle_width_m": 0.35,
    "obstacle_height_m": 0.04,
    "obstacle_id": "repair_obstacle",
    "arc_samples": 241,
}
GAP_STEP = dict(CLOSE_STEP, obstacle_width_m=0.15)


def _request(start_hip_x, start_z, target_hip_x, target_z, terrain):
    start = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, start_hip_x, start_z, **terrain)
    target = build_leg_on_surface_scene_2d(THETA_RAD, 0.0, target_hip_x, target_z, **terrain)
    return replace(
        build_swing_request_2d(start, target, clearance_m=CLEARANCE_M),
        sample_count=SAMPLE_COUNT,
    )


def _ascend_into_a_close_step():
    return _request(0.00, 0.00, 0.20, 0.04, CLOSE_STEP)


def _descend_from_a_close_step():
    return _request(0.20, 0.04, 0.00, 0.00, CLOSE_STEP)


def test_a_liftoff_side_collision_is_repaired_by_rising_first():
    """Standing next to a step, the leg has to go up before it goes forward."""

    request = _ascend_into_a_close_step()
    assert generate_swing_2d(request, arc_samples=ARC_SAMPLES).failure is (
        SwingFailure.TERRAIN_COLLISION
    )

    repair = repair_swing_2d(request, arc_samples=ARC_SAMPLES)

    assert repair.repaired is True
    assert repair.plan.valid is True
    assert repair.liftoff_rise_m > 0.0
    assert repair.touchdown_drop_m == 0.0
    assert repair.duration_was_extended is False
    assert repair.attempts[0].collision_side == "liftoff"
    assert repair.plan.collision.minimum_clearance_m > 0.0


def test_a_touchdown_side_collision_is_repaired_by_coming_in_from_above():
    """The mirror case, and it costs time -- which must be reported, not hidden."""

    request = _descend_from_a_close_step()

    repair = repair_swing_2d(request, arc_samples=ARC_SAMPLES)

    assert repair.repaired is True
    assert repair.plan.valid is True
    assert repair.touchdown_drop_m > 0.0
    assert repair.liftoff_rise_m == 0.0
    assert repair.attempts[0].collision_side == "touchdown"
    # Landing from higher up is slower, and the swing had to be lengthened.
    assert repair.duration_was_extended is True
    assert repair.swing_duration_s == pytest.approx(
        minimum_duration_for_touchdown_drop_s(
            repair.touchdown_drop_m,
            request.constraints.touchdown_normal_speed_max_mps,
        )
    )
    assert repair.request.swing_duration_s == repair.swing_duration_s


def test_collision_geometry_does_not_depend_on_duration():
    """Why the repair can fix geometry first and buy back velocity afterwards.

    The control polygon is built from positions only, so stretching the swing
    in time cannot change what it hits -- it only changes how fast it arrives.
    """

    request = _descend_from_a_close_step()
    slow = replace(request, swing_duration_s=request.swing_duration_s * 4.0)

    fast_plan = generate_swing_2d(request, arc_samples=ARC_SAMPLES, touchdown_drop_m=0.01)
    slow_plan = generate_swing_2d(slow, arc_samples=ARC_SAMPLES, touchdown_drop_m=0.01)

    assert fast_plan.collision.collision_free == slow_plan.collision.collision_free
    assert fast_plan.collision.first_collision_index == (
        slow_plan.collision.first_collision_index
    )
    assert fast_plan.collision.minimum_clearance_m == pytest.approx(
        slow_plan.collision.minimum_clearance_m
    )
    assert np.allclose(
        fast_plan.result.positions_world_xz_m, slow_plan.result.positions_world_xz_m
    )


def test_repair_declines_a_swing_that_already_works():
    request = feasible_regression_cases_2d(sample_count=SAMPLE_COUNT)["C flat -> 40 mm top"]

    repair = repair_swing_2d(request, arc_samples=ARC_SAMPLES)

    assert repair.plan.valid is True
    assert repair.repaired is False          # nothing needed repairing
    assert repair.liftoff_rise_m == 0.0
    assert "nothing to repair" in repair.reason
    assert len(repair.attempts) == 1


def test_repair_refuses_a_failure_the_endpoint_knobs_cannot_reach():
    """An unreachable apex is an input problem, not a trajectory problem."""

    request = _request(0.00, 0.00, 0.40, 0.00, GAP_STEP)
    request = replace(request, target=replace(request.target, clearance_m=0.10))
    assert generate_swing_2d(request, arc_samples=ARC_SAMPLES).failure is (
        SwingFailure.IK_NOT_CONVERGED
    )

    repair = repair_swing_2d(request, arc_samples=ARC_SAMPLES)

    assert repair.repaired is False
    assert "not a collision" in repair.reason
    assert "hip trajectory" in repair.reason
    assert len(repair.attempts) == 1          # it did not even try


def test_refusing_to_lengthen_the_swing_reports_what_it_would_have_cost():
    request = _descend_from_a_close_step()

    repair = repair_swing_2d(request, arc_samples=ARC_SAMPLES, extend_duration=False)

    assert repair.repaired is False
    assert repair.duration_was_extended is False
    assert "collision-free" in repair.reason
    assert "touchdown speed limit" in repair.reason


def test_a_duration_budget_is_honoured():
    request = _descend_from_a_close_step()

    repair = repair_swing_2d(request, arc_samples=ARC_SAMPLES, max_duration_s=1.0)

    assert repair.repaired is False
    assert "beyond the" in repair.reason


def test_minimum_duration_follows_the_endpoint_derivative():
    limit = 0.05
    for drop_m in (0.006, 0.012, 0.03):
        assert minimum_duration_for_touchdown_drop_s(drop_m, limit) == pytest.approx(
            5.0 * drop_m / limit
        )
    with pytest.raises(ValueError):
        minimum_duration_for_touchdown_drop_s(-0.01, limit)


def test_the_ladder_escalates_until_it_clears():
    """The swing over a step needs far more rise than the swing onto one."""

    request = _request(0.00, 0.00, 0.40, 0.00, GAP_STEP)

    repair = repair_swing_2d(request, arc_samples=ARC_SAMPLES)

    assert repair.repaired is True
    assert repair.liftoff_rise_m >= 0.05
    clearances = [
        attempt.minimum_clearance_m
        for attempt in repair.attempts
        if attempt.minimum_clearance_m is not None
    ]
    # Each rung buys clearance; the last one crosses zero.
    assert clearances == sorted(clearances)
    assert clearances[0] < 0.0 < clearances[-1]
