"""Adapter from the current periodic flat Walk generator to Step 1 segments."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk.types import (
    LEG_ORDER,
    LegId,
    SegmentType,
    TrajectorySegment,
    WalkState,
)


def _body_pose_trajectory(
    generator: GaitGenerator3D,
    count: int,
    initial_body_pose_world: NDArray[np.float64] | None,
) -> NDArray[np.float64]:
    if initial_body_pose_world is None:
        initial = np.array([0.0, 0.0, generator.stand_height, 0.0, 0.0, 0.0])
    else:
        initial = np.asarray(initial_body_pose_world, dtype=float)
    if initial.shape != (6,) or not np.all(np.isfinite(initial)):
        raise ValueError("initial_body_pose_world must contain six finite values.")

    time = np.arange(count, dtype=float) * generator.dt
    pose = np.repeat(initial[np.newaxis, :], count, axis=0)
    yaw0 = initial[5]
    yaw_rate = float(generator.omega_z)
    vx, vy = (float(value) for value in generator.v_com)

    if abs(yaw_rate) < 1e-12:
        c0, s0 = np.cos(yaw0), np.sin(yaw0)
        pose[:, 0] += (c0 * vx - s0 * vy) * time
        pose[:, 1] += (s0 * vx + c0 * vy) * time
    else:
        yaw = yaw0 + yaw_rate * time
        pose[:, 0] += (
            vx * (np.sin(yaw) - np.sin(yaw0))
            + vy * (np.cos(yaw) - np.cos(yaw0))
        ) / yaw_rate
        pose[:, 1] += (
            vx * (np.cos(yaw0) - np.cos(yaw))
            + vy * (np.sin(yaw) - np.sin(yaw0))
        ) / yaw_rate
        pose[:, 5] = yaw
    return pose


def _rotation_body_to_world(rpy: NDArray[np.float64]) -> NDArray[np.float64]:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _foot_points_world(
    generator: GaitGenerator3D,
    commands: NDArray[np.float64],
    body_pose_world: NDArray[np.float64],
) -> NDArray[np.float64]:
    count = len(commands)
    points = np.empty((count, 4, 3), dtype=float)
    for sample in range(count):
        rotation = _rotation_body_to_world(body_pose_world[sample, 3:])
        translation = body_pose_world[sample, :3]
        for leg_index, kinematics in enumerate(generator.legs):
            theta, beta, gamma = commands[sample, leg_index]
            alpha, width = kinematics.foot_rim_contact_fk(theta, beta, gamma)
            point_body = kinematics.forward_kinematics(
                theta,
                beta,
                gamma,
                alpha=alpha,
                w=width,
            )
            points[sample, leg_index] = translation + rotation @ point_body
    return points


def _next_swing_leg(phase_cycle: NDArray[np.int8], sample_index: int) -> LegId | None:
    count = len(phase_cycle)
    for offset in range(1, count + 1):
        previous = phase_cycle[(sample_index + offset - 1) % count]
        current = phase_cycle[(sample_index + offset) % count]
        starts = np.flatnonzero((previous == 0) & (current == 1))
        if len(starts):
            return LEG_ORDER[int(starts[0])]
    return None


def flat_walk_segment_from_generator(
    generator: GaitGenerator3D,
    *,
    initial_body_pose_world: NDArray[np.float64] | None = None,
    ground_surface_id: str = "ground",
) -> TrajectorySegment:
    """Wrap an already-generated ``GaitGenerator3D`` flat Walk as a segment.

    The body world trajectory is reconstructed from the generator's constant
    planar twist.  Foot points are reconstructed with current 3-D kinematics;
    they are not simulator measurements.  Step 1 assigns every selected
    surface to flat ``ground`` and does not perform terrain queries.
    """

    if not isinstance(generator, GaitGenerator3D):
        raise TypeError("generator must be a GaitGenerator3D instance.")
    if generator.gait_type != "Walk":
        raise ValueError("the Step 1 flat adapter accepts gait_type='Walk' only.")
    if not ground_surface_id:
        raise ValueError("ground_surface_id must not be empty.")
    if not hasattr(generator, "CMDS") or not hasattr(generator, "PHASE"):
        raise ValueError("generate_full_gait() must be called before adapting the generator.")
    if generator.n_cycles is None or generator.n_cycles <= 0:
        raise ValueError("generator.n_cycles must be a positive generated cycle count.")

    flat_commands = np.asarray(generator.CMDS, dtype=float)
    flat_phase = np.asarray(generator.PHASE)
    if flat_commands.ndim != 2 or flat_commands.shape[1] != 12:
        raise ValueError(f"generator.CMDS must have shape (N, 12); got {flat_commands.shape}.")
    if flat_phase.shape != (len(flat_commands), 4):
        raise ValueError(
            f"generator.PHASE must have shape ({len(flat_commands)}, 4); got {flat_phase.shape}."
        )

    commands = flat_commands.reshape(len(flat_commands), 4, 3)
    phase = flat_phase.astype(np.int8)
    count = len(commands)
    body_pose = _body_pose_trajectory(generator, count, initial_body_pose_world)
    foot_points = _foot_points_world(generator, commands, body_pose)
    active = phase == 0
    surfaces = tuple((ground_surface_id,) * 4 for _ in range(count))
    time = np.arange(count, dtype=float) * generator.dt

    cycle_count = count // generator.n_cycles
    if cycle_count * generator.n_cycles != count:
        raise ValueError("generated command count is not divisible by generator.n_cycles.")
    phase_cycle = phase[:cycle_count]
    gait_cycle_phase = (np.arange(count) % cycle_count) / cycle_count

    start_state = WalkState(
        joint_position_rad=commands[0],
        previous_joint_position_rad=None,
        body_pose_world=body_pose[0],
        foot_contact_points_world_m=foot_points[0],
        phase=phase[0],
        contact_active=active[0],
        surface_ids=surfaces[0],
        gait_cycle_phase=0.0,
        next_swing_leg=_next_swing_leg(phase_cycle, 0),
    )
    final_state = WalkState(
        joint_position_rad=commands[-1],
        previous_joint_position_rad=commands[-2] if count > 1 else None,
        body_pose_world=body_pose[-1],
        foot_contact_points_world_m=foot_points[-1],
        phase=phase[-1],
        contact_active=active[-1],
        surface_ids=surfaces[-1],
        gait_cycle_phase=((count - 1) % cycle_count) / cycle_count,
        next_swing_leg=_next_swing_leg(phase_cycle, cycle_count - 1),
    )
    return TrajectorySegment(
        time_s=time,
        commands_rad=commands,
        phase=phase,
        body_pose_world=body_pose,
        foot_contact_points_world_m=foot_points,
        contact_active=active,
        gait_cycle_phase=gait_cycle_phase,
        surface_ids=surfaces,
        start_state=start_state,
        final_state=final_state,
        dt_s=generator.dt,
        segment_type=SegmentType.FLAT,
    )


def generate_flat_walk_segment(
    generator: GaitGenerator3D,
    *,
    n_cycles: int = 1,
    initial_body_pose_world: NDArray[np.float64] | None = None,
    ground_surface_id: str = "ground",
) -> TrajectorySegment:
    """Generate a flat Walk with the existing planner and immediately adapt it."""

    if not isinstance(n_cycles, int) or isinstance(n_cycles, bool) or n_cycles <= 0:
        raise ValueError("n_cycles must be a positive integer.")
    generator.generate_full_gait(n_cycles=n_cycles)
    return flat_walk_segment_from_generator(
        generator,
        initial_body_pose_world=initial_body_pose_world,
        ground_surface_id=ground_surface_id,
    )
