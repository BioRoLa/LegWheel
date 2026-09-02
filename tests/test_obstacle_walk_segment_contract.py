"""Step 1 tests for WalkState, TrajectorySegment, and the flat adapter."""

from dataclasses import replace

import numpy as np
import pytest

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk import (
    CommandOrder,
    LegId,
    SegmentType,
    flat_walk_segment_from_generator,
    generate_flat_walk_segment,
)


def _flat_generator() -> GaitGenerator3D:
    return GaitGenerator3D(
        stand_height=0.25,
        twist=[0.0, 0.05, 0.0],
        step_height=0.04,
        period=1.0,
        gait_type="Walk",
        dt=0.02,
        stability_margin=0.0,
    )


@pytest.fixture(scope="module")
def flat_result():
    generator = _flat_generator()
    segment = generate_flat_walk_segment(generator, n_cycles=1)
    return generator, segment


def test_flat_adapter_round_trips_existing_planner_arrays(flat_result):
    generator, segment = flat_result

    assert segment.segment_type is SegmentType.FLAT
    assert segment.command_order is CommandOrder.PLANNER_LEG_MAJOR
    assert segment.sample_count == 50
    np.testing.assert_array_equal(segment.to_planner_commands(), generator.CMDS)
    np.testing.assert_array_equal(segment.to_phase_array(), generator.PHASE)
    np.testing.assert_array_equal(
        segment.start_state.joint_position_rad,
        generator.CMDS[0].reshape(4, 3),
    )
    np.testing.assert_array_equal(
        segment.final_state.joint_position_rad,
        generator.CMDS[-1].reshape(4, 3),
    )
    np.testing.assert_array_equal(
        segment.final_state.previous_joint_position_rad,
        generator.CMDS[-2].reshape(4, 3),
    )


def test_flat_adapter_reconstructs_synchronized_world_metadata(flat_result):
    _, segment = flat_result

    assert segment.phase.shape == (50, 4)
    assert segment.body_pose_world.shape == (50, 6)
    assert segment.foot_contact_points_world_m.shape == (50, 4, 3)
    assert segment.contact_active.shape == (50, 4)
    np.testing.assert_array_equal(segment.contact_active, segment.phase == 0)
    np.testing.assert_allclose(segment.body_pose_world[0], [0, 0, 0.25, 0, 0, 0])
    assert segment.body_pose_world[-1, 0] == pytest.approx(0.05 * 0.98)
    np.testing.assert_allclose(segment.body_pose_world[:, 1], 0.0)
    np.testing.assert_allclose(segment.body_pose_world[:, 2], 0.25)
    assert all(row == ("ground",) * 4 for row in segment.surface_ids)
    assert segment.final_state.next_swing_leg is LegId.FL


def test_flat_adapter_supports_an_explicit_initial_world_pose():
    generator = _flat_generator()
    initial = np.array([1.2, -0.4, 0.30, 0.0, 0.0, np.pi / 2])
    segment = generate_flat_walk_segment(
        generator,
        n_cycles=1,
        initial_body_pose_world=initial,
    )

    np.testing.assert_allclose(segment.body_pose_world[0], initial)
    assert segment.body_pose_world[-1, 0] == pytest.approx(initial[0], abs=1e-12)
    assert segment.body_pose_world[-1, 1] == pytest.approx(initial[1] + 0.05 * 0.98)


def test_segment_contract_is_immutable_but_round_trip_copies_are_writable(flat_result):
    _, segment = flat_result

    assert not segment.commands_rad.flags.writeable
    assert not segment.phase.flags.writeable
    assert not segment.body_pose_world.flags.writeable
    assert not segment.foot_contact_points_world_m.flags.writeable
    assert not segment.start_state.joint_position_rad.flags.writeable

    commands = segment.to_planner_commands()
    commands[0, 0] += 1.0
    assert commands[0, 0] != segment.commands_rad[0, 0, 0]


def test_adapter_requires_an_already_generated_walk():
    generator = _flat_generator()
    with pytest.raises(ValueError, match=r"generate_full_gait\(\)"):
        flat_walk_segment_from_generator(generator)

    trot = GaitGenerator3D(
        stand_height=0.25,
        twist=[0.0, 0.05, 0.0],
        gait_type="Trot",
        dt=0.02,
    )
    trot.generate_full_gait(n_cycles=1)
    with pytest.raises(ValueError, match="Walk"):
        flat_walk_segment_from_generator(trot)


def test_segment_rejects_boundary_state_drift(flat_result):
    _, segment = flat_result
    invalid_start = replace(
        segment.start_state,
        joint_position_rad=segment.start_state.joint_position_rad + 0.01,
    )

    with pytest.raises(ValueError, match="start_state joint position"):
        replace(segment, start_state=invalid_start)
