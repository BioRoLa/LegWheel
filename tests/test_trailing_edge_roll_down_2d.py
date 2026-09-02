"""Day 6--7 Step 7R regression tests: trailing-edge roll-down."""

from __future__ import annotations

import numpy as np
import pytest

from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (
    build_single_leg_rolling_scene_2d,
    run_forward_right_rim_roll_up_2d,
)
from hybrid_note.scripts.experiments.trailing_edge_roll_down_2d import (
    PHASE_CORNER,
    PHASE_GROUND_CONTACT,
    PHASE_GROUND_ROLL,
    PHASE_ROLL_DOWN,
    PHASE_TOP_ROLL,
    RollDownStartState2D,
    TrailingEdgeRollDownResult2D,
    _lowest_contact_sample,
    roll_down_frame_rows,
    roll_down_start_state_from_rolling_result,
    roll_down_summary_row,
    run_trailing_edge_roll_down_2d,
    write_trailing_edge_roll_down_csv,
)

ARC_SAMPLES = 61
OBSTACLE_HEIGHT_M = 0.10
OBSTACLE_X_START_M = 0.10


def _scene_kwargs(width_m: float) -> dict:
    return {
        "gamma_rad": 0.0,
        "ground_height_m": 0.0,
        "obstacle_x_start_m": OBSTACLE_X_START_M,
        "obstacle_width_m": width_m,
        "obstacle_height_m": OBSTACLE_HEIGHT_M,
        "obstacle_id": "step7r_test_obstacle",
        "arc_samples": ARC_SAMPLES,
    }


def _top_contact_start_state(
    *,
    contact_x_m: float,
    theta_deg: float = 60.0,
    beta_deg: float = -69.0,
    width_m: float = 0.03,
) -> RollDownStartState2D:
    """A legal obstacle-top rolling state built directly from the leg model.

    Step 7R only needs *a* legal obstacle-top rolling state as its entry, so
    the tests do not pay for a full Step 4.5 roll-up.
    """

    scene_kwargs = _scene_kwargs(width_m)
    theta = np.deg2rad(theta_deg)
    beta = np.deg2rad(beta_deg)
    template = build_single_leg_rolling_scene_2d(theta, beta, 0.0, 0.0, **scene_kwargs)
    sample = _lowest_contact_sample(template.geometry)
    local = template.geometry.points_hip_xz_m[sample]
    top_z = OBSTACLE_HEIGHT_M
    return RollDownStartState2D(
        theta_rad=theta,
        beta_rad=beta,
        active_sample_index=sample,
        contact_point_world_xz_m=(contact_x_m, top_z),
        hip_position_world_xz_m=(contact_x_m - local[0], top_z - local[1]),
        scene_kwargs=scene_kwargs,
        source_phase="TOP_ROLL_COMPLETE",
    )


def _run(start, **overrides) -> TrailingEdgeRollDownResult2D:
    kwargs = {
        "top_contact_step_m": 0.003,
        "ground_contact_step_m": 0.003,
        "pivot_beta_step_rad": np.deg2rad(3.0),
        "ground_roll_distance_m": 0.005,
        "max_steps": 200,
    }
    kwargs.update(overrides)
    return run_trailing_edge_roll_down_2d(start, **kwargs)


def test_start_state_requires_a_successful_roll_up():
    failed = run_forward_right_rim_roll_up_2d(
        candidate_theta_rad=np.deg2rad(60.0),
        initial_beta_rad=np.deg2rad(-20.0),
        hip_x_m=0.0,
        hip_z_m=0.24,
        max_forward_steps=1,
        arc_samples=31,
    )
    assert not failed.success
    with pytest.raises(ValueError, match="successful Step 4.5 roll-up"):
        roll_down_start_state_from_rolling_result(failed)


def test_trailing_edge_must_lie_ahead_of_the_entry_contact():
    # Entry contact already past the trailing edge of a 0.03 m obstacle top.
    start = _top_contact_start_state(contact_x_m=0.16, width_m=0.03)
    with pytest.raises(ValueError, match="trailing edge must lie ahead"):
        _run(start)


def test_roll_down_reaches_lower_ground_with_continuous_legal_contact():
    start = _top_contact_start_state(contact_x_m=0.115, width_m=0.03)
    result = _run(start)

    assert result.success
    assert result.top_roll_success
    assert result.roll_down_success
    assert result.ground_contact_success
    assert result.failure_phase is None

    accepted = result.accepted_frames
    assert len(accepted) == len(result.frames)
    assert not any(frame.collision for frame in result.frames)
    assert all(frame.valid_contact for frame in result.frames)
    # Never below the terrain at any accepted frame.
    assert min(frame.ground_clearance_m for frame in result.frames) >= 0.0

    # theta is untouched and beta stays continuous across every phase.
    thetas = {round(frame.theta_deg, 9) for frame in result.frames}
    assert len(thetas) == 1
    betas = np.asarray([frame.beta_rad for frame in result.frames])
    assert np.max(np.abs(np.diff(betas))) <= np.deg2rad(5.0)

    # The traversal genuinely ends on the lower ground, not on the obstacle.
    final = result.final_frame
    assert final.phase in (PHASE_GROUND_CONTACT, PHASE_GROUND_ROLL)
    assert final.contact_point_world_xz_m[1] == pytest.approx(0.0, abs=1e-6)
    assert final.contact_point_world_xz_m[0] > result.trailing_corner_world_xz_m[0]


def test_every_phase_of_the_primitive_is_reached():
    start = _top_contact_start_state(contact_x_m=0.115, width_m=0.03)
    result = _run(start)

    for phase in (PHASE_TOP_ROLL, PHASE_ROLL_DOWN, PHASE_GROUND_CONTACT):
        assert result.frames_in_phase(phase), f"phase {phase} never occurred"
    # Phases appear in primitive order and never repeat out of order.
    order = [PHASE_TOP_ROLL, PHASE_CORNER, PHASE_ROLL_DOWN,
             PHASE_GROUND_CONTACT, PHASE_GROUND_ROLL]
    seen = [frame.phase for frame in result.frames]
    ranks = [order.index(phase) for phase in seen if phase in order]
    assert ranks == sorted(ranks)


def test_corner_transition_pivots_instead_of_rolling():
    """The distinguishing physics of Step 7R.

    A sharp corner supports the leg at a point, so the corner transition is a
    rotation about a pinned material sample: the world contact point must not
    move while the hip keeps travelling.  This is what makes roll-down more
    than a mirrored roll-up.
    """

    start = _top_contact_start_state(contact_x_m=0.115, width_m=0.03)
    result = _run(start)

    pivot = [
        frame for frame in result.frames
        if frame.phase in (PHASE_CORNER, PHASE_ROLL_DOWN)
    ]
    assert len(pivot) > 5
    assert all(frame.corner_pinned for frame in pivot)

    advances = {round(frame.contact_advance_m, 12) for frame in pivot}
    assert len(advances) == 1, "the contact point moved during the corner pivot"

    samples = {frame.active_sample_index for frame in pivot}
    assert len(samples) == 1, "the pinned material sample changed during the pivot"

    contact_x = {round(frame.contact_point_world_xz_m[0], 9) for frame in pivot}
    assert contact_x == {round(result.trailing_corner_world_xz_m[0], 9)}

    hips = [frame.hip_x_m for frame in pivot]
    assert hips == sorted(hips)
    assert hips[-1] - hips[0] > 0.05
    drops = [frame.hip_z_m for frame in pivot]
    assert drops == sorted(drops, reverse=True)
    assert result.hip_drop_at_touchdown_m < 0.0


def test_corner_pivot_never_scrapes_the_back_face():
    start = _top_contact_start_state(contact_x_m=0.115, width_m=0.03)
    result = _run(start)

    assert result.minimum_back_face_clearance_m is not None
    assert result.minimum_back_face_clearance_m > 0.0


def test_a_long_obstacle_top_exhausts_the_finite_rim_arc():
    """Rim arc is a finite budget shared by the climb, the top and the pivot.

    The leg cannot roll past the seam between two rim arcs, so a top that is
    too long is reported as a rim failure rather than silently jumping rims.
    """

    start = _top_contact_start_state(contact_x_m=0.115, width_m=0.30)
    result = _run(start, max_steps=300)

    assert not result.success
    assert result.failure_reason in {
        "RIM_SEAM_REACHES_GROUND_FIRST",
        "RIM_ARC_EXHAUSTED_BEFORE_TRAILING_EDGE",
    }
    assert not result.ground_contact_success
    assert result.frames[-1].failure_reason == result.failure_reason
    assert not result.frames[-1].accepted


def test_rim_budget_decreases_monotonically_along_the_traversal():
    start = _top_contact_start_state(contact_x_m=0.115, width_m=0.03)
    result = _run(start)

    budget = [
        frame.remaining_rim_arc_rad
        for frame in result.frames
        if frame.remaining_rim_arc_rad is not None
    ]
    assert len(budget) == len(result.frames)
    assert np.all(np.diff(budget) <= 1e-9), "rim budget increased along the traversal"
    assert result.remaining_rim_arc_rad_at_touchdown < budget[0]


def test_frame_rows_and_csv_round_trip(tmp_path):
    start = _top_contact_start_state(contact_x_m=0.115, width_m=0.03)
    result = _run(start)

    rows = roll_down_frame_rows(result)
    assert len(rows) == len(result.frames)
    assert rows[0]["phase"] == PHASE_TOP_ROLL
    assert {"contact_advance_m", "remaining_rim_arc_deg", "corner_pinned"} <= set(rows[0])

    summary = roll_down_summary_row(result)
    assert summary["success"] is True
    assert summary["ground_contact_rim"] == "right_rim"

    summary_path, frames_path = write_trailing_edge_roll_down_csv(
        result, tmp_path / "summary.csv", tmp_path / "frames.csv"
    )
    assert summary_path.exists() and frames_path.exists()
    assert len(frames_path.read_text(encoding="utf-8").splitlines()) == len(rows) + 1
