"""Step 2 segment slicing, boundary validation, and lossless assembly."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from legwheel.planners.obstacle_walk.types import (
    JOINT_ORDER,
    LEG_ORDER,
    LegId,
    SegmentType,
    TrajectorySegment,
    WalkState,
)


@dataclass(frozen=True)
class ContinuityTolerances:
    """Centralized limits used for every segment boundary.

    The velocity quantity compares one backward and one forward finite
    difference around a shared sample.  It therefore includes normal discrete
    acceleration as well as a true velocity jump.
    """

    joint_position_rad: float = 1e-9
    joint_velocity_rad_s: float = 0.5
    body_position_m: float = 1e-9
    body_orientation_rad: float = 1e-9
    support_foot_position_m: float = 1e-9

    def __post_init__(self) -> None:
        for name, value in self.__dict__.items():
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative.")


@dataclass(frozen=True)
class BoundaryContinuityReport:
    """Inspectable evidence for one ``left[-1] == right[0]`` boundary."""

    left_segment_index: int
    right_segment_index: int
    joint_position_max_rad: float
    joint_position_leg: LegId
    joint_position_joint: str
    joint_velocity_max_rad_s: float
    joint_velocity_leg: LegId
    joint_velocity_joint: str
    body_position_max_m: float
    body_position_axis: str
    body_orientation_max_rad: float
    body_orientation_axis: str
    support_foot_position_max_m: float
    support_foot_leg: LegId | None
    phase_match: bool
    contact_active_match: bool
    active_surface_match: bool
    passed: bool
    violations: tuple[str, ...]


@dataclass(frozen=True)
class ConcatenationResult:
    """Combined segment plus all boundary validation reports."""

    segment: TrajectorySegment
    boundary_reports: tuple[BoundaryContinuityReport, ...]


class SegmentContinuityError(ValueError):
    """Raised before assembly when a boundary violates the contract."""

    def __init__(self, report: BoundaryContinuityReport):
        self.report = report
        detail = "; ".join(report.violations)
        super().__init__(
            f"segment boundary {report.left_segment_index} -> "
            f"{report.right_segment_index} failed: {detail}"
        )


def _worst_joint(error: np.ndarray) -> tuple[float, LegId, str]:
    leg_index, joint_index = np.unravel_index(int(np.argmax(error)), error.shape)
    return (
        float(error[leg_index, joint_index]),
        LEG_ORDER[leg_index],
        JOINT_ORDER[joint_index],
    )


def _worst_axis(error: np.ndarray, names: tuple[str, str, str]) -> tuple[float, str]:
    axis = int(np.argmax(error))
    return float(error[axis]), names[axis]


def validate_segment_boundary(
    left: TrajectorySegment,
    right: TrajectorySegment,
    *,
    left_segment_index: int = 0,
    right_segment_index: int = 1,
    tolerances: ContinuityTolerances | None = None,
) -> BoundaryContinuityReport:
    """Validate a shared endpoint without mutating either segment."""

    if not isinstance(left, TrajectorySegment) or not isinstance(right, TrajectorySegment):
        raise TypeError("left and right must be TrajectorySegment instances.")
    limits = tolerances or ContinuityTolerances()
    if not isinstance(limits, ContinuityTolerances):
        raise TypeError("tolerances must be ContinuityTolerances.")
    if left.sample_count < 2 or right.sample_count < 2:
        raise ValueError("boundary velocity validation requires at least two samples per segment.")
    if not np.isclose(left.dt_s, right.dt_s, rtol=0.0, atol=1e-15):
        raise ValueError(
            f"segment dt mismatch at {left_segment_index} -> {right_segment_index}: "
            f"{left.dt_s} s != {right.dt_s} s."
        )
    if left.command_order is not right.command_order:
        raise ValueError(
            f"segment command order mismatch at {left_segment_index} -> {right_segment_index}."
        )

    position_error = np.abs(left.commands_rad[-1] - right.commands_rad[0])
    q_error, q_leg, q_joint = _worst_joint(position_error)
    left_velocity = (left.commands_rad[-1] - left.commands_rad[-2]) / left.dt_s
    right_velocity = (right.commands_rad[1] - right.commands_rad[0]) / right.dt_s
    velocity_error = np.abs(left_velocity - right_velocity)
    qd_error, qd_leg, qd_joint = _worst_joint(velocity_error)

    body_position_error = np.abs(left.body_pose_world[-1, :3] - right.body_pose_world[0, :3])
    body_pos_error, body_pos_axis = _worst_axis(body_position_error, ("x", "y", "z"))
    body_orientation_error = np.abs(
        left.body_pose_world[-1, 3:] - right.body_pose_world[0, 3:]
    )
    body_rpy_error, body_rpy_axis = _worst_axis(
        body_orientation_error,
        ("roll", "pitch", "yaw"),
    )

    shared_active = left.contact_active[-1] & right.contact_active[0]
    support_errors = np.linalg.norm(
        left.foot_contact_points_world_m[-1] - right.foot_contact_points_world_m[0],
        axis=1,
    )
    if np.any(shared_active):
        masked = np.where(shared_active, support_errors, -np.inf)
        foot_index = int(np.argmax(masked))
        foot_error = float(masked[foot_index])
        foot_leg: LegId | None = LEG_ORDER[foot_index]
    else:
        foot_error = 0.0
        foot_leg = None

    phase_match = bool(np.array_equal(left.phase[-1], right.phase[0]))
    active_match = bool(np.array_equal(left.contact_active[-1], right.contact_active[0]))
    active_surface_match = all(
        not shared_active[index]
        or left.surface_ids[-1][index] == right.surface_ids[0][index]
        for index in range(4)
    )

    violations: list[str] = []
    if q_error > limits.joint_position_rad:
        violations.append(
            f"joint position {q_leg.value}.{q_joint} error {q_error:.6g} rad "
            f"> {limits.joint_position_rad:.6g} rad"
        )
    if qd_error > limits.joint_velocity_rad_s:
        violations.append(
            f"joint velocity {qd_leg.value}.{qd_joint} error {qd_error:.6g} rad/s "
            f"> {limits.joint_velocity_rad_s:.6g} rad/s"
        )
    if not phase_match:
        changed = [
            leg.value
            for index, leg in enumerate(LEG_ORDER)
            if left.phase[-1, index] != right.phase[0, index]
        ]
        violations.append(f"phase mismatch for legs {','.join(changed)}")
    if not active_match:
        changed = [
            leg.value
            for index, leg in enumerate(LEG_ORDER)
            if left.contact_active[-1, index] != right.contact_active[0, index]
        ]
        violations.append(f"contact_active mismatch for legs {','.join(changed)}")
    if body_pos_error > limits.body_position_m:
        violations.append(
            f"body position {body_pos_axis} error {body_pos_error:.6g} m "
            f"> {limits.body_position_m:.6g} m"
        )
    if body_rpy_error > limits.body_orientation_rad:
        violations.append(
            f"body orientation {body_rpy_axis} error {body_rpy_error:.6g} rad "
            f"> {limits.body_orientation_rad:.6g} rad"
        )
    if foot_error > limits.support_foot_position_m:
        assert foot_leg is not None
        violations.append(
            f"support foot {foot_leg.value} error {foot_error:.6g} m "
            f"> {limits.support_foot_position_m:.6g} m"
        )
    if not active_surface_match:
        changed = [
            leg.value
            for index, leg in enumerate(LEG_ORDER)
            if shared_active[index]
            and left.surface_ids[-1][index] != right.surface_ids[0][index]
        ]
        violations.append(f"active surface mismatch for legs {','.join(changed)}")

    return BoundaryContinuityReport(
        left_segment_index=left_segment_index,
        right_segment_index=right_segment_index,
        joint_position_max_rad=q_error,
        joint_position_leg=q_leg,
        joint_position_joint=q_joint,
        joint_velocity_max_rad_s=qd_error,
        joint_velocity_leg=qd_leg,
        joint_velocity_joint=qd_joint,
        body_position_max_m=body_pos_error,
        body_position_axis=body_pos_axis,
        body_orientation_max_rad=body_rpy_error,
        body_orientation_axis=body_rpy_axis,
        support_foot_position_max_m=foot_error,
        support_foot_leg=foot_leg,
        phase_match=phase_match,
        contact_active_match=active_match,
        active_surface_match=active_surface_match,
        passed=not violations,
        violations=tuple(violations),
    )


def _next_swing_leg(phase: np.ndarray, sample_index: int) -> LegId | None:
    for row in range(sample_index + 1, len(phase)):
        starts = np.flatnonzero((phase[row - 1] == 0) & (phase[row] == 1))
        if len(starts):
            return LEG_ORDER[int(starts[0])]
    return None


def _state_at(segment: TrajectorySegment, index: int) -> WalkState:
    previous = (
        segment.commands_rad[index - 1]
        if index > 0
        else segment.start_state.previous_joint_position_rad
    )
    next_swing = _next_swing_leg(segment.phase, index)
    if next_swing is None and index == segment.sample_count - 1:
        next_swing = segment.final_state.next_swing_leg
    return WalkState(
        joint_position_rad=segment.commands_rad[index],
        previous_joint_position_rad=previous,
        body_pose_world=segment.body_pose_world[index],
        foot_contact_points_world_m=segment.foot_contact_points_world_m[index],
        phase=segment.phase[index],
        contact_active=segment.contact_active[index],
        surface_ids=segment.surface_ids[index],
        gait_cycle_phase=float(segment.gait_cycle_phase[index]),
        next_swing_leg=next_swing,
    )


def slice_segment(segment: TrajectorySegment, start: int, stop: int) -> TrajectorySegment:
    """Return a local-time slice while preserving synchronized metadata."""

    if not isinstance(segment, TrajectorySegment):
        raise TypeError("segment must be a TrajectorySegment.")
    if not isinstance(start, int) or not isinstance(stop, int):
        raise TypeError("start and stop must be integers.")
    if start < 0 or stop > segment.sample_count or stop - start < 2:
        raise ValueError("slice must select at least two samples within the segment.")
    count = stop - start
    return TrajectorySegment(
        time_s=np.arange(count, dtype=float) * segment.dt_s,
        commands_rad=segment.commands_rad[start:stop],
        phase=segment.phase[start:stop],
        body_pose_world=segment.body_pose_world[start:stop],
        foot_contact_points_world_m=segment.foot_contact_points_world_m[start:stop],
        contact_active=segment.contact_active[start:stop],
        gait_cycle_phase=segment.gait_cycle_phase[start:stop],
        surface_ids=segment.surface_ids[start:stop],
        start_state=_state_at(segment, start),
        final_state=_state_at(segment, stop - 1),
        dt_s=segment.dt_s,
        segment_type=segment.segment_type,
        swing_leg=segment.swing_leg,
        command_order=segment.command_order,
    )


def concatenate_segments(
    segments: Sequence[TrajectorySegment],
    *,
    dt: float | None = None,
    tolerances: ContinuityTolerances | None = None,
) -> ConcatenationResult:
    """Validate and join segments, dropping each repeated right endpoint row."""

    items = tuple(segments)
    if not items:
        raise ValueError("segments must contain at least one TrajectorySegment.")
    if not all(isinstance(item, TrajectorySegment) for item in items):
        raise TypeError("segments must contain only TrajectorySegment instances.")
    limits = tolerances or ContinuityTolerances()
    expected_dt = items[0].dt_s if dt is None else float(dt)
    if not np.isfinite(expected_dt) or expected_dt <= 0.0:
        raise ValueError("dt must be finite and positive.")
    for index, item in enumerate(items):
        if not np.isclose(item.dt_s, expected_dt, rtol=0.0, atol=1e-15):
            raise ValueError(
                f"segment {index} dt {item.dt_s} s does not match requested dt {expected_dt} s."
            )

    reports: list[BoundaryContinuityReport] = []
    for index, (left, right) in enumerate(zip(items, items[1:])):
        report = validate_segment_boundary(
            left,
            right,
            left_segment_index=index,
            right_segment_index=index + 1,
            tolerances=limits,
        )
        reports.append(report)
        if not report.passed:
            raise SegmentContinuityError(report)

    def join_array(name: str) -> np.ndarray:
        arrays = [getattr(items[0], name)]
        arrays.extend(getattr(item, name)[1:] for item in items[1:])
        return np.concatenate(arrays, axis=0)

    commands = join_array("commands_rad")
    phase = join_array("phase")
    body_pose = join_array("body_pose_world")
    foot_points = join_array("foot_contact_points_world_m")
    contact_active = join_array("contact_active")
    gait_cycle_phase = join_array("gait_cycle_phase")
    surface_rows = list(items[0].surface_ids)
    for item in items[1:]:
        surface_rows.extend(item.surface_ids[1:])
    sample_count = len(commands)

    segment_types = {item.segment_type for item in items}
    combined_type = segment_types.pop() if len(segment_types) == 1 else SegmentType.COMPOSITE
    combined = TrajectorySegment(
        time_s=np.arange(sample_count, dtype=float) * expected_dt,
        commands_rad=commands,
        phase=phase,
        body_pose_world=body_pose,
        foot_contact_points_world_m=foot_points,
        contact_active=contact_active,
        gait_cycle_phase=gait_cycle_phase,
        surface_ids=surface_rows,
        start_state=items[0].start_state,
        final_state=items[-1].final_state,
        dt_s=expected_dt,
        segment_type=combined_type,
        command_order=items[0].command_order,
    )
    return ConcatenationResult(combined, tuple(reports))
