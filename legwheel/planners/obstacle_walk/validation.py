"""Step 8 whole-trajectory validation for a generated obstacle walk.

Step 7 validates each segment as it is built.  Step 8 re-checks the *assembled*
trajectory end to end, adding the two things Step 7 explicitly did not do:

* full leg-geometry collision against the obstacle (all three tyre arcs and the
  six linkage bars, not just the tracked contact point), and
* support-polygon quasi-static stability.

It also reports every traversal stage separately, so "one leg got up" can never
be mistaken for "the robot crossed the obstacle".
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Sequence

import numpy as np
from numpy.typing import NDArray

from legwheel.config import RobotParams
from legwheel.planners.gait_generator_3d import _hull_signed_margin
from legwheel.planners.obstacle_walk.collision import (
    DEFAULT_ARC_SAMPLES,
    LegGeometrySampler,
    RectangleGeometryCollisionChecker,
    build_terrain_profile_2d,
)
from legwheel.planners.obstacle_walk.traversal import ObstacleWalkResult, TraversalStage
from legwheel.planners.obstacle_walk.types import JOINT_ORDER, LEG_ORDER


class CheckStatus(str, Enum):
    PASSED = "passed"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass(frozen=True)
class CheckResult:
    """One named whole-trajectory check with its worst observed value."""

    name: str
    status: CheckStatus
    detail: str
    worst_value: float | None = None
    limit: float | None = None
    row: int | None = None
    leg: str | None = None
    segment_index: int | None = None

    @property
    def passed(self) -> bool:
        return self.status is not CheckStatus.FAILED


@dataclass(frozen=True)
class TraversalValidationReport:
    """Per-check and per-stage evidence for one assembled trajectory."""

    checks: tuple[CheckResult, ...]
    stage_reached: dict[str, bool]
    collision_frames_checked: int
    collision_leg_poses_checked: int
    frame_stride: int
    arc_samples: int
    minimum_obstacle_clearance_m: float | None
    minimum_stability_margin_m: float
    offline_complete_traversal: bool
    notes: tuple[str, ...] = field(default_factory=tuple)

    @property
    def passed(self) -> bool:
        return all(check.passed for check in self.checks)

    def failed_checks(self) -> tuple[CheckResult, ...]:
        return tuple(item for item in self.checks if not item.passed)

    def to_dict(self) -> dict[str, object]:
        return {
            "offline_complete_traversal": self.offline_complete_traversal,
            "all_checks_passed": self.passed,
            "stage_reached": dict(self.stage_reached),
            "collision": {
                "frames_checked": self.collision_frames_checked,
                "leg_poses_checked": self.collision_leg_poses_checked,
                "frame_stride": self.frame_stride,
                "rim_arc_samples_per_surface": self.arc_samples,
                "minimum_obstacle_clearance_m": self.minimum_obstacle_clearance_m,
            },
            "minimum_stability_margin_m": self.minimum_stability_margin_m,
            "checks": [
                {
                    "name": item.name,
                    "status": item.status.value,
                    "detail": item.detail,
                    "worst_value": item.worst_value,
                    "limit": item.limit,
                    "row": item.row,
                    "leg": item.leg,
                    "segment_index": item.segment_index,
                }
                for item in self.checks
            ],
            "notes": list(self.notes),
        }


def _segment_index_for_row(result: ObstacleWalkResult, row: int) -> int | None:
    for record in result.records:
        if record.start_row <= row <= record.end_row:
            return record.index
    return None


def _check_joint_limits(result: ObstacleWalkResult) -> CheckResult:
    commands = result.segment.commands_rad
    bounds = (
        (
            0,
            np.deg2rad(RobotParams.MIN_THETA_DEG),
            np.deg2rad(RobotParams.MAX_THETA_DEG),
        ),
        (1, -result.beta_limit_rad, result.beta_limit_rad),
        (
            2,
            -np.deg2rad(RobotParams.GAMMA_MAX_DEG),
            np.deg2rad(RobotParams.GAMMA_MAX_DEG),
        ),
    )
    worst_name, worst_slack, worst_row, worst_leg = "", np.inf, None, None
    for joint_index, lower, upper in bounds:
        values = commands[:, :, joint_index]
        slack = np.minimum(values - lower, upper - values)
        row, leg_index = np.unravel_index(int(np.argmin(slack)), slack.shape)
        if slack[row, leg_index] < worst_slack:
            worst_slack = float(slack[row, leg_index])
            worst_name = JOINT_ORDER[joint_index]
            worst_row, worst_leg = int(row), LEG_ORDER[int(leg_index)].value
    status = CheckStatus.PASSED if worst_slack >= 0.0 else CheckStatus.FAILED
    return CheckResult(
        name="joint_position_limits",
        status=status,
        detail=(
            f"closest approach to a joint limit is {worst_leg}.{worst_name} with "
            f"{worst_slack:.6g} rad of slack"
        ),
        worst_value=worst_slack,
        limit=0.0,
        row=worst_row,
        leg=worst_leg,
        segment_index=None if worst_row is None else _segment_index_for_row(result, worst_row),
    )


def _check_rate(
    result: ObstacleWalkResult,
    name: str,
    order: int,
    limit: float,
) -> CheckResult:
    dt = result.segment.dt_s
    values = np.abs(np.diff(result.segment.commands_rad, axis=0, n=order)) / dt**order
    if not len(values):
        return CheckResult(name, CheckStatus.SKIPPED, "trajectory is too short", None, limit)
    row, leg_index, joint_index = np.unravel_index(int(np.argmax(values)), values.shape)
    peak = float(values[row, leg_index, joint_index])
    status = CheckStatus.PASSED if peak <= limit else CheckStatus.FAILED
    return CheckResult(
        name=name,
        status=status,
        detail=(
            f"peak {peak:.6g} at {LEG_ORDER[int(leg_index)].value}."
            f"{JOINT_ORDER[int(joint_index)]}, limit {limit:.6g}"
        ),
        worst_value=peak,
        limit=limit,
        row=int(row) + order,
        leg=LEG_ORDER[int(leg_index)].value,
        segment_index=_segment_index_for_row(result, int(row) + order),
    )


def _check_gait_phase(result: ObstacleWalkResult) -> CheckResult:
    phase = result.segment.phase
    swinging = phase.sum(axis=1)
    bad = np.flatnonzero(swinging > 1)
    if len(bad):
        row = int(bad[0])
        return CheckResult(
            "gait_phase_legality",
            CheckStatus.FAILED,
            f"{int(swinging[row])} legs swing simultaneously",
            float(np.max(swinging)),
            1.0,
            row,
            None,
            _segment_index_for_row(result, row),
        )
    if not np.array_equal(result.segment.contact_active, phase == 0):
        return CheckResult(
            "gait_phase_legality",
            CheckStatus.FAILED,
            "contact_active disagrees with the stance/swing phase",
        )
    return CheckResult(
        "gait_phase_legality",
        CheckStatus.PASSED,
        f"at most one leg swings at a time over {len(phase)} rows",
        float(np.max(swinging)),
        1.0,
    )


def _check_support_contact_drift(result: ObstacleWalkResult, limit: float) -> CheckResult:
    points = result.segment.foot_contact_points_world_m
    active = result.segment.contact_active
    motion = np.linalg.norm(np.diff(points, axis=0), axis=2)
    # Only rows where a leg is in stance on both sides of the step may be held.
    held = active[:-1] & active[1:]
    drift = np.where(held, motion, 0.0)
    if not drift.size:
        return CheckResult("support_contact_drift", CheckStatus.SKIPPED, "no rows", None, limit)
    row, leg_index = np.unravel_index(int(np.argmax(drift)), drift.shape)
    peak = float(drift[row, leg_index])
    status = CheckStatus.PASSED if peak <= limit else CheckStatus.FAILED
    return CheckResult(
        "support_contact_drift",
        status,
        f"largest single-step motion of a held contact is {peak * 1e3:.4f} mm",
        peak,
        limit,
        int(row) + 1,
        LEG_ORDER[int(leg_index)].value,
        _segment_index_for_row(result, int(row) + 1),
    )


def _check_stability(result: ObstacleWalkResult, required_margin_m: float):
    """Quasi-static margin from the body-centre ground projection to the hull.

    The body origin is used as the centre-of-mass proxy, matching the existing
    Walk planner's ``stability_margin`` convention.  It is a static polygon
    test: no inertia, no contact forces, no friction.
    """

    poses = result.segment.body_pose_world
    points = result.segment.foot_contact_points_world_m
    active = result.segment.contact_active
    worst = np.inf
    worst_row = 0
    for row in range(len(poses)):
        support = points[row][active[row]]
        margin, _direction = _hull_signed_margin(poses[row, :2], support[:, :2])
        if margin < worst:
            worst = float(margin)
            worst_row = row
    status = CheckStatus.PASSED if worst >= required_margin_m else CheckStatus.FAILED
    return (
        CheckResult(
            "quasi_static_support_polygon",
            status,
            (
                f"smallest signed distance from the body-centre ground projection to "
                f"the support polygon is {worst * 1e3:.2f} mm"
            ),
            worst,
            required_margin_m,
            worst_row,
            None,
            _segment_index_for_row(result, worst_row),
        ),
        worst,
    )


def _check_full_geometry_collision(
    result: ObstacleWalkResult,
    frame_stride: int,
    arc_samples: int,
    penetration_tolerance_m: float,
):
    if result.walk_generator is None:
        raise ValueError(
            "the collision sweep needs result.walk_generator; regenerate the "
            "trajectory with the current generate_obstacle_walk()."
        )
    generator = result.walk_generator
    sampler = LegGeometrySampler(
        kinematics=list(generator.legs),
        hip_positions_body_m=list(generator.hip_positions),
        arc_samples=arc_samples,
    )
    checker = RectangleGeometryCollisionChecker(
        sampler=sampler,
        terrain_profile=build_terrain_profile_2d(result.terrain),
        penetration_tolerance_m=penetration_tolerance_m,
    )
    commands = result.segment.commands_rad
    poses = result.segment.body_pose_world
    rows = list(range(0, len(commands), max(1, frame_stride)))
    if rows[-1] != len(commands) - 1:
        rows.append(len(commands) - 1)
    worst_depth = 0.0
    worst_message: str | None = None
    worst_row: int | None = None
    worst_leg: str | None = None
    for row in rows:
        for leg_index, leg in enumerate(LEG_ORDER):
            message, depth = checker.evaluate(leg, commands[row, leg_index], poses[row])
            if depth > worst_depth:
                worst_depth, worst_message = depth, message
                worst_row, worst_leg = row, leg.value
    status = (
        CheckStatus.FAILED
        if worst_depth > penetration_tolerance_m
        else CheckStatus.PASSED
    )
    detail = (
        f"deepest penetration {worst_depth * 1e3:.4f} mm over {len(rows)} frames"
        if worst_message is None
        else worst_message
    )
    return (
        CheckResult(
            "full_leg_geometry_collision",
            status,
            detail,
            worst_depth,
            penetration_tolerance_m,
            worst_row,
            worst_leg,
            None if worst_row is None else _segment_index_for_row(result, worst_row),
        ),
        len(rows),
        len(rows) * 4,
        worst_depth,
    )


def _stage_reached(result: ObstacleWalkResult) -> dict[str, bool]:
    stages = {record.stage for record in result.records}
    return {
        TraversalStage.APPROACH.value: TraversalStage.APPROACH in stages,
        "single_leg_step_up": bool(result.legs_that_reached_top),
        "all_four_legs_on_top_surface": set(result.legs_that_reached_top) == set(LEG_ORDER),
        "all_four_top_simultaneously": result.all_four_top_observed,
        "single_leg_step_down": bool(result.legs_that_returned_to_ground),
        "all_four_legs_back_on_ground": (
            set(result.legs_that_returned_to_ground) == set(LEG_ORDER)
        ),
        "traversal_completed": result.traversal_completed,
        TraversalStage.RECOVERY.value: TraversalStage.RECOVERY in stages,
    }


def validate_traversal(
    result: ObstacleWalkResult,
    *,
    frame_stride: int = 1,
    arc_samples: int = DEFAULT_ARC_SAMPLES,
    penetration_tolerance_m: float = 1e-3,
    joint_velocity_limit_rad_s: float = 16.0,
    joint_acceleration_limit_rad_s2: float = 16000.0,
    support_contact_drift_limit_m: float = 2e-3,
    required_stability_margin_m: float = 0.0,
    skip_collision: bool = False,
) -> TraversalValidationReport:
    """Re-check an assembled obstacle walk end to end and report per stage.

    ``frame_stride`` decimates only the geometry collision sweep, which is by
    far the most expensive check; every other check always runs on every row.
    """

    if not isinstance(result, ObstacleWalkResult):
        raise TypeError("result must be an ObstacleWalkResult.")
    if not isinstance(frame_stride, int) or frame_stride < 1:
        raise ValueError("frame_stride must be a positive integer.")

    checks = [
        _check_joint_limits(result),
        _check_rate(result, "joint_velocity", 1, joint_velocity_limit_rad_s),
        _check_rate(result, "joint_acceleration", 2, joint_acceleration_limit_rad_s2),
        _check_gait_phase(result),
        _check_support_contact_drift(result, support_contact_drift_limit_m),
    ]
    stability_check, worst_margin = _check_stability(result, required_stability_margin_m)
    checks.append(stability_check)

    notes: list[str] = []
    minimum_clearance: float | None = None
    frames_checked = 0
    poses_checked = 0
    if skip_collision:
        checks.append(
            CheckResult(
                "full_leg_geometry_collision",
                CheckStatus.SKIPPED,
                "collision sweep was disabled by the caller",
            )
        )
        notes.append("full leg geometry collision was NOT evaluated")
    else:
        collision_check, frames_checked, poses_checked, worst_depth = (
            _check_full_geometry_collision(
                result, frame_stride, arc_samples, penetration_tolerance_m
            )
        )
        checks.append(collision_check)
        minimum_clearance = -worst_depth
        if frame_stride > 1:
            notes.append(
                f"collision was sampled every {frame_stride} rows; poses between "
                f"sampled frames were not evaluated"
            )
        notes.append(
            f"rim arcs are polylines of {arc_samples} samples per surface, so the "
            f"check is optimistic by the arc sagitta (well under 0.1 mm at this "
            f"resolution)"
        )

    stage_reached = _stage_reached(result)
    complete = all(check.passed for check in checks) and all(stage_reached.values())
    if not complete:
        notes.append(
            "not marked as an offline complete traversal: see failed checks and "
            "stage_reached"
        )
    return TraversalValidationReport(
        checks=tuple(checks),
        stage_reached=stage_reached,
        collision_frames_checked=frames_checked,
        collision_leg_poses_checked=poses_checked,
        frame_stride=frame_stride,
        arc_samples=arc_samples,
        minimum_obstacle_clearance_m=minimum_clearance,
        minimum_stability_margin_m=worst_margin,
        offline_complete_traversal=complete,
        notes=tuple(notes),
    )
