"""Step 4 arbitrary-touchdown single-leg swing segment prototype.

The implementation reuses the current 3-D Walk Bezier optimizer, the existing
rim material-point convention, and ``CorgiLegKinematics.inverse_kinematics``.
The body and the three support-leg commands remain fixed during this Step 4
primitive; mixed-height stance/body motion belongs to Step 5.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from legwheel.config import RobotParams
from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk.terrain import (
    TouchdownStatus,
    WalkTerrain1D,
    query_touchdown_surface,
)
from legwheel.planners.obstacle_walk.types import (
    LEG_ORDER,
    LegId,
    SegmentType,
    TrajectorySegment,
    WalkState,
)


class SwingRejectReason(str, Enum):
    INVALID_START_CONTACT = "INVALID_START_CONTACT"
    ILLEGAL_TOUCHDOWN = "ILLEGAL_TOUCHDOWN"
    TARGET_HEIGHT_MISMATCH = "TARGET_HEIGHT_MISMATCH"
    IK_UNREACHABLE = "IK_UNREACHABLE"
    JOINT_LIMIT = "JOINT_LIMIT"
    JOINT_DISCONTINUITY = "JOINT_DISCONTINUITY"
    CARTESIAN_TRACKING_ERROR = "CARTESIAN_TRACKING_ERROR"
    CONTACT_PATH_COLLISION = "CONTACT_PATH_COLLISION"
    FULL_GEOMETRY_COLLISION = "FULL_GEOMETRY_COLLISION"


class SwingPlanningError(ValueError):
    """A deterministic rejection with a stable machine-readable reason."""

    def __init__(self, reason: SwingRejectReason, message: str, *, sample_index: int | None = None):
        self.reason = SwingRejectReason(reason)
        self.sample_index = sample_index
        location = "" if sample_index is None else f" at sample {sample_index}"
        super().__init__(f"{self.reason.value}{location}: {message}")


class FullGeometryCollisionChecker(Protocol):
    """Optional Step 4 hook for a complete 3-D leg/wheel geometry checker.

    Return ``None`` when the sample is collision-free, otherwise return a
    human-readable collision description. The current Step 4 package does not
    pretend that its point-path check implements this full contract.
    """

    def __call__(
        self,
        *,
        sample_index: int,
        leg: LegId,
        joint_position_rad: NDArray[np.float64],
        body_pose_world: NDArray[np.float64],
        terrain: WalkTerrain1D,
    ) -> str | None: ...


def _readonly_array(value: object, shape: tuple[int | None, ...], name: str) -> NDArray[np.float64]:
    array = np.asarray(value, dtype=float)
    valid = array.ndim == len(shape) and all(
        expected is None or actual == expected
        for actual, expected in zip(array.shape, shape)
    )
    if not valid or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be finite with shape {shape}; got {array.shape}.")
    array = array.copy()
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class SwingPlanResult:
    """A generated segment plus Cartesian and validation evidence."""

    segment: TrajectorySegment
    swing_leg: LegId | str
    touchdown_world_m: NDArray[np.float64]
    target_surface_id: str
    rim_alpha_td_deg: float
    cartesian_target_world_m: NDArray[np.float64]
    cartesian_actual_world_m: NDArray[np.float64]
    tracking_error_m: NDArray[np.float64]
    terrain_max_height_world_m: float
    requested_apex_height_world_m: float
    achieved_apex_height_world_m: float
    full_geometry_collision_checked: bool

    def __post_init__(self) -> None:
        if not isinstance(self.segment, TrajectorySegment):
            raise TypeError("segment must be TrajectorySegment.")
        object.__setattr__(self, "swing_leg", LegId(self.swing_leg))
        object.__setattr__(
            self,
            "touchdown_world_m",
            _readonly_array(self.touchdown_world_m, (3,), "touchdown_world_m"),
        )
        count = self.segment.sample_count
        object.__setattr__(
            self,
            "cartesian_target_world_m",
            _readonly_array(
                self.cartesian_target_world_m,
                (count, 3),
                "cartesian_target_world_m",
            ),
        )
        object.__setattr__(
            self,
            "cartesian_actual_world_m",
            _readonly_array(
                self.cartesian_actual_world_m,
                (count, 3),
                "cartesian_actual_world_m",
            ),
        )
        object.__setattr__(
            self,
            "tracking_error_m",
            _readonly_array(self.tracking_error_m, (count,), "tracking_error_m"),
        )
        for name in (
            "rim_alpha_td_deg",
            "terrain_max_height_world_m",
            "requested_apex_height_world_m",
            "achieved_apex_height_world_m",
        ):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite.")
        if not isinstance(self.target_surface_id, str) or not self.target_surface_id:
            raise ValueError("target_surface_id must be a non-empty string.")

    @property
    def maximum_tracking_error_m(self) -> float:
        return float(np.max(self.tracking_error_m))


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


def _terrain_max_height(terrain: WalkTerrain1D, x_start: float, x_end: float) -> float:
    corridor_min, corridor_max = sorted((x_start, x_end))
    obstacle = terrain.obstacle
    overlaps_obstacle = (
        corridor_max >= obstacle.x_start_m and corridor_min <= obstacle.x_end_m
    )
    if overlaps_obstacle:
        return terrain.ground_height_m + obstacle.height_m
    return terrain.ground_height_m


def _solve_touchdown_material_point(
    generator: GaitGenerator3D,
    leg_index: int,
    start_q: NDArray[np.float64],
    target_body_m: NDArray[np.float64],
    tracking_tolerance_m: float,
) -> tuple[NDArray[np.float64], float, float]:
    kinematics = generator.legs[leg_index]
    alpha_deg, width_m = kinematics.foot_rim_contact_fk(*start_q)
    guess = start_q.copy()
    try:
        for _ in range(12):
            q_target = kinematics.inverse_kinematics(
                target_body_m,
                guess_q=guess,
                rim_point=(alpha_deg, width_m),
                tol=min(tracking_tolerance_m, 2e-4),
            )
            next_alpha_deg, next_width_m = kinematics.foot_rim_contact_fk(*q_target)
            if (
                abs(next_alpha_deg - alpha_deg) <= 1e-5
                and abs(next_width_m - width_m) <= 1e-7
            ):
                return q_target, float(next_alpha_deg), float(next_width_m)
            alpha_deg = float(next_alpha_deg)
            width_m = float(next_width_m)
            guess = q_target
    except RuntimeError as exc:
        raise SwingPlanningError(SwingRejectReason.IK_UNREACHABLE, str(exc)) from exc
    return q_target, float(alpha_deg), float(width_m)


def _check_joint_limits(commands: NDArray[np.float64], leg_index: int) -> None:
    """Reject poses outside the joint limits.

    ``BETA_MAX_DEG`` bounds the sagittal range over which the **foot rim** can
    still be the contact.  A Walk never rolls the contact onto the upper tyre
    rims, so past this angle the planned foot-rim contact simply does not
    exist and the pose is infeasible, not merely aggressive.  Widening it here
    would silently produce commands the gait cannot realise.
    """

    limits = (
        (0, np.deg2rad(RobotParams.MIN_THETA_DEG), np.deg2rad(RobotParams.MAX_THETA_DEG)),
        (1, -np.deg2rad(RobotParams.BETA_MAX_DEG), np.deg2rad(RobotParams.BETA_MAX_DEG)),
        (2, -np.deg2rad(RobotParams.GAMMA_MAX_DEG), np.deg2rad(RobotParams.GAMMA_MAX_DEG)),
    )
    for joint_index, lower, upper in limits:
        values = commands[:, leg_index, joint_index]
        invalid = np.flatnonzero((values < lower) | (values > upper))
        if len(invalid):
            sample = int(invalid[0])
            raise SwingPlanningError(
                SwingRejectReason.JOINT_LIMIT,
                f"{LEG_ORDER[leg_index].value}.{('theta', 'beta', 'gamma')[joint_index]} "
                f"is {values[sample]:.6g} rad outside [{lower:.6g}, {upper:.6g}] rad",
                sample_index=sample,
            )


def _check_contact_path(
    points_world_m: NDArray[np.float64],
    terrain: WalkTerrain1D,
    clearance_tolerance_m: float,
) -> None:
    obstacle = terrain.obstacle
    top_z = terrain.ground_height_m + obstacle.height_m
    count = len(points_world_m)

    # Sample 0 is the shared liftoff and sample 1 only extrapolates the
    # incoming joint velocity, so both are still at the liftoff contact.  The
    # final two samples are the planned touchdown and its zero-velocity hold.
    # Surface contact is legal at all four, so only the airborne interior of
    # the swing is checked against the obstacle top.
    for sample in range(2, count - 2):
        x, _, z = points_world_m[sample]
        inside_footprint = obstacle.x_start_m <= x <= obstacle.x_end_m
        if inside_footprint and z <= top_z + clearance_tolerance_m:
            raise SwingPlanningError(
                SwingRejectReason.CONTACT_PATH_COLLISION,
                f"tracked rim point z={z:.6g} m does not clear obstacle top "
                f"z={top_z:.6g} m by {clearance_tolerance_m:.6g} m",
                sample_index=sample,
            )

    for sample, (start, end) in enumerate(zip(points_world_m, points_world_m[1:])):
        dx = end[0] - start[0]
        if abs(dx) <= 1e-15:
            continue
        for edge_name, edge_x in (
            ("front", obstacle.x_start_m),
            ("back", obstacle.x_end_m),
        ):
            fraction = (edge_x - start[0]) / dx
            if 0.0 < fraction < 1.0:
                z_crossing = start[2] + fraction * (end[2] - start[2])
                if z_crossing <= top_z + clearance_tolerance_m:
                    raise SwingPlanningError(
                        SwingRejectReason.CONTACT_PATH_COLLISION,
                        f"tracked rim point crosses {edge_name} face at "
                        f"z={z_crossing:.6g} m, obstacle top is {top_z:.6g} m",
                        sample_index=sample,
                    )


def _walk_successor(leg: LegId) -> LegId:
    swing_order = (LegId.FL, LegId.RR, LegId.FR, LegId.RL)
    return swing_order[(swing_order.index(leg) + 1) % len(swing_order)]


def generate_swing_segment(
    generator: GaitGenerator3D,
    start_state: WalkState,
    swing_leg: LegId | str,
    touchdown_world_m: NDArray[np.float64],
    terrain: WalkTerrain1D,
    clearance_m: float,
    *,
    tracking_tolerance_m: float = 1e-3,
    touchdown_height_tolerance_m: float = 1e-3,
    maximum_joint_step_rad: float = 0.35,
    contact_path_clearance_tolerance_m: float = 1e-4,
    swing_duration_s: float | None = None,
    next_swing_leg: LegId | str | None = None,
    full_geometry_collision_checker: FullGeometryCollisionChecker | None = None,
) -> SwingPlanResult:
    """Generate one static-body, single-leg swing to a world-frame touchdown."""

    if not isinstance(generator, GaitGenerator3D) or generator.gait_type != "Walk":
        raise TypeError("generator must be a Walk GaitGenerator3D.")
    if not isinstance(start_state, WalkState):
        raise TypeError("start_state must be WalkState.")
    if not isinstance(terrain, WalkTerrain1D):
        raise TypeError("terrain must be WalkTerrain1D.")
    leg = LegId(swing_leg)
    leg_index = LEG_ORDER.index(leg)
    target_world = np.asarray(touchdown_world_m, dtype=float)
    if target_world.shape != (3,) or not np.all(np.isfinite(target_world)):
        raise ValueError("touchdown_world_m must contain three finite values.")
    for name, value in (
        ("clearance_m", clearance_m),
        ("tracking_tolerance_m", tracking_tolerance_m),
        ("touchdown_height_tolerance_m", touchdown_height_tolerance_m),
        ("maximum_joint_step_rad", maximum_joint_step_rad),
        ("contact_path_clearance_tolerance_m", contact_path_clearance_tolerance_m),
    ):
        if not np.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative.")
    if tracking_tolerance_m == 0.0 or maximum_joint_step_rad == 0.0:
        raise ValueError("tracking_tolerance_m and maximum_joint_step_rad must be positive.")
    if swing_duration_s is not None and (
        not np.isfinite(swing_duration_s) or swing_duration_s <= 0.0
    ):
        raise ValueError("swing_duration_s must be finite and positive when given.")
    if not np.all(start_state.phase == 0):
        raise SwingPlanningError(
            SwingRejectReason.INVALID_START_CONTACT,
            "Step 4 requires all four legs to be in stance at the shared start sample",
        )

    start_query = query_touchdown_surface(
        terrain,
        float(start_state.foot_contact_points_world_m[leg_index, 0]),
    )
    start_z = float(start_state.foot_contact_points_world_m[leg_index, 2])
    if (
        not start_query.is_legal
        or start_query.surface_id != start_state.surface_ids[leg_index]
        or abs(start_z - float(start_query.surface_height_world_m))
        > touchdown_height_tolerance_m
    ):
        raise SwingPlanningError(
            SwingRejectReason.INVALID_START_CONTACT,
            f"{leg.value} start contact does not match the queried terrain surface",
        )

    target_query = query_touchdown_surface(terrain, float(target_world[0]))
    if not target_query.is_legal:
        raise SwingPlanningError(
            SwingRejectReason.ILLEGAL_TOUCHDOWN,
            f"target x={target_world[0]:.6g} m was rejected: "
            f"{target_query.rejection_reason.value}",
        )
    target_height = float(target_query.surface_height_world_m)
    if abs(float(target_world[2]) - target_height) > touchdown_height_tolerance_m:
        raise SwingPlanningError(
            SwingRejectReason.TARGET_HEIGHT_MISMATCH,
            f"target z={target_world[2]:.6g} m does not match "
            f"surface z={target_height:.6g} m",
        )

    body_pose = start_state.body_pose_world
    rotation = _rotation_body_to_world(body_pose[3:])
    translation = body_pose[:3]
    target_body = rotation.T @ (target_world - translation)
    start_q = start_state.joint_position_rad[leg_index]
    touchdown_q, alpha_td_deg, width_td_m = _solve_touchdown_material_point(
        generator,
        leg_index,
        start_q,
        target_body,
        tracking_tolerance_m,
    )
    kinematics = generator.legs[leg_index]
    liftoff_world = translation + rotation @ kinematics.forward_kinematics(
        *start_q,
        alpha=alpha_td_deg,
        w=width_td_m,
    )

    # Preserve the incoming discrete joint velocity exactly at the first
    # interval. This is the Step 2 shared-boundary contract. If no prior sample
    # exists, start with a zero command velocity.
    if start_state.previous_joint_position_rad is None:
        continuity_q = start_q.copy()
    else:
        continuity_q = start_q + (
            start_q - start_state.previous_joint_position_rad[leg_index]
        )
    continuity_world = translation + rotation @ kinematics.forward_kinematics(
        *continuity_q,
        alpha=alpha_td_deg,
        w=width_td_m,
    )
    terrain_max = _terrain_max_height(terrain, liftoff_world[0], target_world[0])
    requested_apex = (
        max(liftoff_world[2], continuity_world[2], target_world[2], terrain_max)
        + clearance_m
    )
    step_height = requested_apex - continuity_world[2]

    planner = generator.planners[leg_index]
    # The Bezier profile is purely geometric, so the requested duration only
    # sets how finely it is sampled.  A longer duration lowers the per-sample
    # joint step of a tall step-up/step-down without changing its path.
    swing_duration = (
        planner.T * (1.0 - planner.stance_duty)
        if swing_duration_s is None
        else float(swing_duration_s)
    )
    profile_sample_count = max(3, int(round(swing_duration / planner.dt)))
    liftoff_swing = continuity_world[[0, 2, 1]]
    touchdown_swing = target_world[[0, 2, 1]]
    profile = planner.swing_planner.solveSwingTrajectory(
        liftoff_swing,
        touchdown_swing,
        step_height,
        np.zeros(3),
        np.zeros(3),
    )
    profile_target_swing = np.asarray(
        [
            profile.getFootendPoint(value)
            for value in np.linspace(0.0, 1.0, profile_sample_count)
        ]
    )
    profile_target_world = profile_target_swing[:, [0, 2, 1]]
    # Start sample, velocity-continuous sample, remaining Bezier samples, then
    # one repeated touchdown sample for zero outgoing command velocity.
    target_path_world = np.vstack(
        [
            liftoff_world,
            profile_target_world,
            target_world,
        ]
    )
    sample_count = len(target_path_world)

    commands = np.repeat(
        start_state.joint_position_rad[np.newaxis, :, :],
        sample_count,
        axis=0,
    )
    actual_path_world = np.empty((sample_count, 3), dtype=float)
    tracking_error = np.empty(sample_count, dtype=float)
    guess = start_q.copy()
    for sample, point_world in enumerate(target_path_world):
        if sample == 0:
            q = start_q.copy()
        elif sample == 1:
            q = continuity_q.copy()
        elif sample >= sample_count - 2:
            q = touchdown_q.copy()
        else:
            point_body = rotation.T @ (point_world - translation)
            try:
                q = kinematics.inverse_kinematics(
                    point_body,
                    guess_q=guess,
                    rim_point=(alpha_td_deg, width_td_m),
                    tol=min(tracking_tolerance_m, 2e-4),
                )
            except RuntimeError as exc:
                raise SwingPlanningError(
                    SwingRejectReason.IK_UNREACHABLE,
                    str(exc),
                    sample_index=sample,
                ) from exc
        commands[sample, leg_index] = q
        actual = translation + rotation @ kinematics.forward_kinematics(
            *q,
            alpha=alpha_td_deg,
            w=width_td_m,
        )
        actual_path_world[sample] = actual
        tracking_error[sample] = np.linalg.norm(actual - point_world)
        guess = q

    max_tracking_error = float(np.max(tracking_error))
    if max_tracking_error > tracking_tolerance_m:
        sample = int(np.argmax(tracking_error))
        raise SwingPlanningError(
            SwingRejectReason.CARTESIAN_TRACKING_ERROR,
            f"maximum error {max_tracking_error:.6g} m exceeds "
            f"{tracking_tolerance_m:.6g} m",
            sample_index=sample,
        )
    joint_steps = np.max(np.abs(np.diff(commands[:, leg_index], axis=0)), axis=1)
    if np.any(joint_steps > maximum_joint_step_rad):
        sample = int(np.argmax(joint_steps)) + 1
        raise SwingPlanningError(
            SwingRejectReason.JOINT_DISCONTINUITY,
            f"joint step {joint_steps[sample - 1]:.6g} rad exceeds "
            f"{maximum_joint_step_rad:.6g} rad",
            sample_index=sample,
        )
    _check_joint_limits(commands, leg_index)
    _check_contact_path(
        actual_path_world,
        terrain,
        contact_path_clearance_tolerance_m,
    )

    if full_geometry_collision_checker is not None:
        for sample in range(sample_count):
            collision = full_geometry_collision_checker(
                sample_index=sample,
                leg=leg,
                joint_position_rad=commands[sample, leg_index].copy(),
                body_pose_world=body_pose.copy(),
                terrain=terrain,
            )
            if collision is not None:
                raise SwingPlanningError(
                    SwingRejectReason.FULL_GEOMETRY_COLLISION,
                    collision,
                    sample_index=sample,
                )

    phase = np.zeros((sample_count, 4), dtype=np.int8)
    phase[1:-2, leg_index] = 1
    active = phase == 0
    body_poses = np.repeat(body_pose[np.newaxis, :], sample_count, axis=0)
    foot_points = np.repeat(
        start_state.foot_contact_points_world_m[np.newaxis, :, :],
        sample_count,
        axis=0,
    )
    for sample in range(1, sample_count):
        q = commands[sample, leg_index]
        alpha_low_deg, width = kinematics.foot_rim_contact_fk(*q)
        foot_points[sample, leg_index] = translation + rotation @ kinematics.forward_kinematics(
            *q,
            alpha=alpha_low_deg,
            w=width,
        )
    surface_rows = [list(start_state.surface_ids) for _ in range(sample_count)]
    for sample in range(1, sample_count):
        surface_rows[sample][leg_index] = str(target_query.surface_id)
    gait_phase = (
        start_state.gait_cycle_phase
        + np.arange(sample_count, dtype=float) * generator.dt / generator.T
    ) % 1.0
    final_state = WalkState(
        joint_position_rad=commands[-1],
        previous_joint_position_rad=commands[-2],
        body_pose_world=body_poses[-1],
        foot_contact_points_world_m=foot_points[-1],
        phase=phase[-1],
        contact_active=active[-1],
        surface_ids=surface_rows[-1],
        gait_cycle_phase=float(gait_phase[-1]),
        next_swing_leg=(
            _walk_successor(leg) if next_swing_leg is None else LegId(next_swing_leg)
        ),
    )
    segment = TrajectorySegment(
        time_s=np.arange(sample_count, dtype=float) * generator.dt,
        commands_rad=commands,
        phase=phase,
        body_pose_world=body_poses,
        foot_contact_points_world_m=foot_points,
        contact_active=active,
        gait_cycle_phase=gait_phase,
        surface_ids=surface_rows,
        start_state=start_state,
        final_state=final_state,
        dt_s=generator.dt,
        segment_type=SegmentType.SWING,
        swing_leg=leg,
    )
    return SwingPlanResult(
        segment=segment,
        swing_leg=leg,
        touchdown_world_m=target_world,
        target_surface_id=str(target_query.surface_id),
        rim_alpha_td_deg=alpha_td_deg,
        cartesian_target_world_m=target_path_world,
        cartesian_actual_world_m=actual_path_world,
        tracking_error_m=tracking_error,
        terrain_max_height_world_m=terrain_max,
        requested_apex_height_world_m=requested_apex,
        achieved_apex_height_world_m=float(np.max(actual_path_world[:, 2])),
        full_geometry_collision_checked=full_geometry_collision_checker is not None,
    )


def plot_swing_plan(
    plan: SwingPlanResult,
    terrain: WalkTerrain1D,
    output_path: str | Path,
) -> Path:
    """Save an inspectable X-Z plot of target and actual material-point paths."""

    if not isinstance(plan, SwingPlanResult):
        raise TypeError("plan must be SwingPlanResult.")
    if not isinstance(terrain, WalkTerrain1D):
        raise TypeError("terrain must be WalkTerrain1D.")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(9, 4.8))
    obstacle = terrain.obstacle
    axis.axhline(terrain.ground_height_m, color="black", linewidth=1.2, label="ground")
    axis.add_patch(
        Rectangle(
            (obstacle.x_start_m, terrain.ground_height_m),
            obstacle.length_m,
            obstacle.height_m,
            facecolor="0.75",
            edgecolor="0.25",
            label="obstacle",
        )
    )
    axis.plot(
        plan.cartesian_target_world_m[:, 0],
        plan.cartesian_target_world_m[:, 2],
        "--",
        color="tab:blue",
        label="Bezier target",
    )
    axis.plot(
        plan.cartesian_actual_world_m[:, 0],
        plan.cartesian_actual_world_m[:, 2],
        "o-",
        markersize=3,
        color="tab:orange",
        label="rim-point FK",
    )
    axis.scatter(
        [plan.touchdown_world_m[0]],
        [plan.touchdown_world_m[2]],
        marker="x",
        s=70,
        color="red",
        label="touchdown",
    )
    axis.axhline(
        plan.requested_apex_height_world_m,
        color="tab:green",
        linewidth=1,
        linestyle=":",
        label="requested apex",
    )
    axis.set_xlabel("world x (m)")
    axis.set_ylabel("world z (m)")
    axis.set_title(
        f"Step 4 {plan.swing_leg.value} swing, max IK/FK error "
        f"{plan.maximum_tracking_error_m * 1e3:.2f} mm"
    )
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return output
