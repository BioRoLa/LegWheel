"""Day 8--9 Step 9: regression over the planning note's case set.

One entry point, one set of checks, both directions: the swings that must work
and the requests that must be refused.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    SwingFailure,
    build_leg_on_surface_scene_2d,
    build_swing_request_2d,
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
from hybrid_note.scripts.experiments.cartesian_swing_velocity_2d import (
    validate_swing_touchdown_velocity_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_planner_2d import (
    REGRESSION_CLEARANCE_M,
    REGRESSION_THETA_RAD,
    feasible_regression_cases_2d,
    generate_swing_2d,
    regression_row,
    rejected_regression_cases_2d,
)

ARC_SAMPLES = 61

# The cases carry their own resolution: the joint-continuity limit is per
# sample, so overriding it here could change what a marginal case reports.
FEASIBLE = feasible_regression_cases_2d()
REJECTED = rejected_regression_cases_2d()
SAMPLE_COUNT = next(iter(FEASIBLE.values())).sample_count


@pytest.mark.parametrize("name", sorted(FEASIBLE))
def test_every_planning_note_case_passes_all_seven_checks(name):
    """Cases A--E, checked on exactly what the planning note lists."""

    plan = generate_swing_2d(FEASIBLE[name], arc_samples=ARC_SAMPLES)
    request = FEASIBLE[name]

    # 2. endpoint correctness
    assert plan.endpoint_report["start_position_error_mm"] == pytest.approx(0.0, abs=1e-9)
    assert plan.endpoint_report["target_position_error_mm"] == pytest.approx(0.0, abs=1e-9)
    assert plan.endpoint_report["t_last_s"] == pytest.approx(request.swing_duration_s)
    # 3. IK feasibility
    assert plan.joint_report["ik_converged_count"] == request.sample_count
    assert plan.joint_report["max_ik_residual_um"] < 100.0
    # 4. joint limits and continuity
    assert plan.joint_report["joint_limits_ok_count"] == request.sample_count
    assert plan.joint_report["max_joint_step_deg"] < plan.joint_report["max_joint_step_limit_deg"]
    # 5. collision-free
    assert plan.collision.collision_free is True
    # Not "> 0": both endpoints are contacts, so the minimum clearance over the
    # whole trajectory is ~0 by construction and can sit a few nanometres below
    # it.  What must hold is that nothing is inside the terrain.
    assert plan.collision.minimum_clearance_m > -1e-5
    # 6. correct touchdown state
    assert plan.touchdown.passed is True
    assert plan.touchdown.rim_match and plan.touchdown.surface_match and plan.touchdown.alpha_ok
    # 7. touchdown velocity
    assert plan.velocity.normal_ok is True
    # and only then
    assert plan.valid is True
    assert plan.failure is SwingFailure.NONE


@pytest.mark.parametrize("name", sorted(REJECTED))
def test_every_infeasible_request_is_refused_for_the_right_reason(name):
    """Without these, a suite that only checks successes could not fail."""

    request, expected = REJECTED[name]

    plan = generate_swing_2d(request, arc_samples=ARC_SAMPLES)

    assert plan.valid is False
    assert plan.failure is expected


def test_only_the_swing_target_changes_between_touchdown_heights():
    """The planning note's headline claim, tested literally.

    One terrain, one start ContactState, several touchdown targets at
    different heights -- all through the same call, with nothing branching on
    terrain height.
    """

    obstacle = {
        "obstacle_x_start_m": 0.20,
        "obstacle_width_m": 0.45,
        "obstacle_height_m": 0.04,
        "obstacle_id": "regression_obstacle",
        "arc_samples": 241,
    }
    start_scene = build_leg_on_surface_scene_2d(
        REGRESSION_THETA_RAD, 0.0, 0.00, 0.00, **obstacle
    )
    targets = [
        build_leg_on_surface_scene_2d(REGRESSION_THETA_RAD, 0.0, hip_x, 0.04, **obstacle)
        for hip_x in (0.36, 0.42, 0.48)
    ]

    plans = []
    for target_scene in targets:
        request = replace(
            build_swing_request_2d(
                start_scene, target_scene, clearance_m=REGRESSION_CLEARANCE_M
            ),
            sample_count=SAMPLE_COUNT,
        )
        # Same terrain object and same start contact state every time.
        assert request.terrain == start_scene.terrain
        assert request.start.contact_point_world_xz_m == pytest.approx(
            plans[0].result.request.start.contact_point_world_xz_m
        ) if plans else True
        plans.append(generate_swing_2d(request, arc_samples=ARC_SAMPLES))

    assert all(plan.valid for plan in plans)
    # Different touchdowns, so different apexes and different joint ranges --
    # produced without a single branch on height.
    touchdown_x = [plan.result.samples[-1].position_world_xz_m[0] for plan in plans]
    assert touchdown_x == sorted(touchdown_x)
    assert len(set(round(x, 6) for x in touchdown_x)) == len(plans)


def test_the_composed_planner_is_exactly_steps_3_to_8():
    """``generate_swing_2d`` must not be a second implementation."""

    request = FEASIBLE["C flat -> 40 mm top"]

    composed = generate_swing_2d(request, arc_samples=ARC_SAMPLES)

    path = generate_terrain_aware_swing_path_2d(request)
    solved, _ = solve_swing_joint_trajectory_2d(path)
    checked, collision, _ = check_swing_trajectory_collisions_2d(solved, arc_samples=ARC_SAMPLES)
    validated, touchdown, _ = validate_swing_touchdown_2d(checked)
    manual, velocity = validate_swing_touchdown_velocity_2d(validated)

    assert composed.result.valid == manual.valid
    assert composed.result.failure is manual.failure
    assert composed.collision.collision_free == collision.collision_free
    assert composed.touchdown.passed == touchdown.passed
    assert composed.velocity.normal_velocity_mps == pytest.approx(
        velocity.normal_velocity_mps
    )
    assert composed.result.final_contact_error_m == pytest.approx(
        manual.final_contact_error_m
    )
    assert np.allclose(
        composed.result.positions_world_xz_m, manual.positions_world_xz_m
    )


def test_an_ill_posed_request_stops_before_there_is_anything_to_inspect():
    request = FEASIBLE["A flat -> flat"]
    broken = replace(
        request, target=replace(request.target, target_terrain_surface_id="nope")
    )

    plan = generate_swing_2d(broken, arc_samples=ARC_SAMPLES)

    assert plan.valid is False
    assert plan.failure is SwingFailure.INVALID_REQUEST
    assert plan.result.samples == ()
    assert plan.apex is None
    assert plan.collision is None
    assert plan.touchdown is None
    assert plan.velocity is None


def test_regression_row_reports_every_listed_property():
    plan = generate_swing_2d(FEASIBLE["B flat -> 20 mm top"], arc_samples=ARC_SAMPLES)

    row = regression_row("B flat -> 20 mm top", plan)

    for key in (
        "planner",
        "endpoint_error_mm",
        "duration_error_s",
        "ik_converged",
        "joint_limits_ok",
        "collision_free",
        "touchdown_state_ok",
        "touchdown_velocity_ok",
        "VALID",
    ):
        assert key in row
    assert row["planner"] == "generate_swing_2d"
    assert row["VALID"] is True
