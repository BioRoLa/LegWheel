"""Step 2 regression tests for segment slicing and continuity-checked assembly."""

from dataclasses import replace

import numpy as np
import pytest

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk import (
    ContinuityTolerances,
    SegmentContinuityError,
    concatenate_segments,
    generate_flat_walk_segment,
    slice_segment,
    validate_segment_boundary,
)


@pytest.fixture(scope="module")
def split_flat_walk():
    generator = GaitGenerator3D(
        stand_height=0.25,
        twist=[0.0, 0.05, 0.0],
        step_height=0.04,
        period=1.0,
        gait_type="Walk",
        dt=0.02,
        stability_margin=0.0,
    )
    complete = generate_flat_walk_segment(generator, n_cycles=2)
    # Both slices contain sample 50. It is the shared endpoint that assembly
    # validates and stores only once.
    left = slice_segment(complete, 0, 51)
    right = slice_segment(complete, 50, 100)
    return complete, left, right


def _replace_right_first_sample(right, *, field: str, value, state_updates: dict):
    array = np.array(getattr(right, field), copy=True)
    array[0] = value
    start_state = replace(right.start_state, **state_updates)
    return replace(right, **{field: array, "start_state": start_state})


def test_flat_walk_split_and_reassembly_is_lossless(split_flat_walk):
    complete, left, right = split_flat_walk

    result = concatenate_segments([left, right], dt=complete.dt_s)
    combined = result.segment

    assert combined.sample_count == complete.sample_count
    assert len(result.boundary_reports) == 1
    assert result.boundary_reports[0].passed
    np.testing.assert_array_equal(combined.commands_rad, complete.commands_rad)
    np.testing.assert_array_equal(combined.phase, complete.phase)
    np.testing.assert_array_equal(combined.body_pose_world, complete.body_pose_world)
    np.testing.assert_array_equal(
        combined.foot_contact_points_world_m,
        complete.foot_contact_points_world_m,
    )
    np.testing.assert_array_equal(combined.contact_active, complete.contact_active)
    np.testing.assert_array_equal(combined.gait_cycle_phase, complete.gait_cycle_phase)
    assert combined.surface_ids == complete.surface_ids


def test_boundary_report_records_actual_errors_and_limits(split_flat_walk):
    _, left, right = split_flat_walk

    report = validate_segment_boundary(left, right)

    assert report.passed
    assert report.joint_position_max_rad == pytest.approx(0.0)
    assert report.joint_velocity_max_rad_s == pytest.approx(0.16185825498850545)
    assert report.body_position_max_m == pytest.approx(0.0)
    assert report.body_orientation_max_rad == pytest.approx(0.0)
    assert report.support_foot_position_max_m == pytest.approx(0.0)
    assert report.phase_match
    assert report.contact_active_match
    assert report.active_surface_match


def test_joint_position_jump_is_rejected_with_leg_and_joint(split_flat_walk):
    _, left, right = split_flat_walk
    commands = right.commands_rad.copy()
    commands[0, 0, 0] += 0.01
    invalid = _replace_right_first_sample(
        right,
        field="commands_rad",
        value=commands[0],
        state_updates={"joint_position_rad": commands[0]},
    )

    with pytest.raises(SegmentContinuityError, match=r"0 -> 1.*FL\.theta.*0\.01 rad"):
        concatenate_segments([left, invalid])


def test_joint_velocity_jump_is_rejected_with_leg_and_joint(split_flat_walk):
    _, left, right = split_flat_walk
    commands = right.commands_rad.copy()
    commands[1, 1, 1] += 0.10
    invalid = replace(right, commands_rad=commands)

    with pytest.raises(SegmentContinuityError, match=r"FR\.beta.*rad/s"):
        concatenate_segments([left, invalid])


def test_phase_jump_is_rejected_with_leg_name(split_flat_walk):
    _, left, right = split_flat_walk
    phase = right.phase.copy()
    phase[0, 0] = 1 - phase[0, 0]
    active = right.contact_active.copy()
    active[0] = phase[0] == 0
    start = replace(
        right.start_state,
        phase=phase[0],
        contact_active=active[0],
    )
    invalid = replace(right, phase=phase, contact_active=active, start_state=start)

    with pytest.raises(SegmentContinuityError, match=r"phase mismatch for legs FL"):
        concatenate_segments([left, invalid])


def test_body_pose_jump_is_rejected_with_axis(split_flat_walk):
    _, left, right = split_flat_walk
    body_pose = right.body_pose_world.copy()
    body_pose[0, 0] += 0.02
    invalid = _replace_right_first_sample(
        right,
        field="body_pose_world",
        value=body_pose[0],
        state_updates={"body_pose_world": body_pose[0]},
    )

    with pytest.raises(SegmentContinuityError, match=r"body position x.*0\.02 m"):
        concatenate_segments([left, invalid])


def test_support_foot_jump_is_rejected_with_leg_name(split_flat_walk):
    _, left, right = split_flat_walk
    active_leg_index = int(np.flatnonzero(right.contact_active[0])[0])
    points = right.foot_contact_points_world_m.copy()
    points[0, active_leg_index, 0] += 0.015
    invalid = _replace_right_first_sample(
        right,
        field="foot_contact_points_world_m",
        value=points[0],
        state_updates={"foot_contact_points_world_m": points[0]},
    )

    leg_name = ("FL", "FR", "RR", "RL")[active_leg_index]
    with pytest.raises(
        SegmentContinuityError,
        match=rf"support foot {leg_name}.*0\.015 m",
    ):
        concatenate_segments([left, invalid])


def test_active_surface_jump_is_rejected_with_leg_name(split_flat_walk):
    _, left, right = split_flat_walk
    active_leg_index = int(np.flatnonzero(right.contact_active[0])[0])
    surface_rows = [list(row) for row in right.surface_ids]
    surface_rows[0][active_leg_index] = "unexpected_surface"
    start_surfaces = tuple(surface_rows[0])
    invalid = replace(
        right,
        surface_ids=surface_rows,
        start_state=replace(right.start_state, surface_ids=start_surfaces),
    )

    leg_name = ("FL", "FR", "RR", "RL")[active_leg_index]
    with pytest.raises(
        SegmentContinuityError,
        match=rf"active surface mismatch for legs {leg_name}",
    ):
        concatenate_segments([left, invalid])


def test_velocity_tolerance_is_configurable_and_reported(split_flat_walk):
    _, left, right = split_flat_walk
    strict = ContinuityTolerances(joint_velocity_rad_s=0.1)

    with pytest.raises(SegmentContinuityError) as caught:
        concatenate_segments([left, right], tolerances=strict)

    assert caught.value.report.joint_velocity_max_rad_s == pytest.approx(0.16185825498850545)
    assert caught.value.report.violations == (
        "joint velocity FL.beta error 0.161858 rad/s > 0.1 rad/s",
    )


def test_requested_dt_mismatch_is_rejected_before_assembly(split_flat_walk):
    _, left, right = split_flat_walk

    with pytest.raises(ValueError, match=r"segment 0 dt.*requested dt"):
        concatenate_segments([left, right], dt=0.01)
