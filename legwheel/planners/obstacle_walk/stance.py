"""Step 5 world-fixed, mixed-height stance segment prototype.

All four legs remain in stance.  Their selected lowest-rim contacts are held
fixed in the world while a user-specified body pose follows a cubic smoothstep
trajectory.  This is a quasi-static inverse-kinematics primitive; it is not the
continuous rolling-Jacobian stance law used by the periodic flat Walk planner.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from legwheel.config import RobotParams
from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk.swing import FullGeometryCollisionChecker
from legwheel.planners.obstacle_walk.terrain import WalkTerrain1D, query_touchdown_surface
from legwheel.planners.obstacle_walk.types import (
    JOINT_ORDER,
    LEG_ORDER,
    SegmentType,
    TrajectorySegment,
    WalkState,
)


BODY_TRAJECTORY_ASSUMPTION = "world_pose_cubic_smoothstep_with_endpoint_holds"
BRAKING_BODY_TRAJECTORY_ASSUMPTION = (
    "world_pose_quintic_hermite_from_incoming_velocity_to_rest"
)


class StanceRejectReason(str, Enum):
    INVALID_START_CONTACT = "INVALID_START_CONTACT"
    IK_UNREACHABLE = "IK_UNREACHABLE"
    JOINT_LIMIT = "JOINT_LIMIT"
    JOINT_DISCONTINUITY = "JOINT_DISCONTINUITY"
    CONTACT_DRIFT = "CONTACT_DRIFT"
    CONTACT_SURFACE_MISMATCH = "CONTACT_SURFACE_MISMATCH"
    FULL_GEOMETRY_COLLISION = "FULL_GEOMETRY_COLLISION"


class StancePlanningError(ValueError):
    """A deterministic Step 5 rejection with a stable reason."""

    def __init__(
        self,
        reason: StanceRejectReason,
        message: str,
        *,
        sample_index: int | None = None,
        leg_index: int | None = None,
    ):
        self.reason = StanceRejectReason(reason)
        self.sample_index = sample_index
        self.leg_index = leg_index
        location = ""
        if sample_index is not None:
            location += f" at sample {sample_index}"
        if leg_index is not None:
            location += f" for {LEG_ORDER[leg_index].value}"
        super().__init__(f"{self.reason.value}{location}: {message}")


def _readonly_array(
    value: object,
    shape: tuple[int | None, ...],
    name: str,
) -> NDArray[np.float64]:
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
class StancePlanResult:
    """Generated stance segment and inspectable Step 5 validation evidence."""

    segment: TrajectorySegment
    contact_targets_world_m: NDArray[np.float64]
    contact_actual_world_m: NDArray[np.float64]
    contact_drift_m: NDArray[np.float64]
    rim_alpha_deg: NDArray[np.float64]
    rim_width_m: NDArray[np.float64]
    requested_body_pose_world: NDArray[np.float64]
    body_trajectory_assumption: str
    requested_motion_duration_s: float
    full_geometry_collision_checked: bool

    def __post_init__(self) -> None:
        if not isinstance(self.segment, TrajectorySegment):
            raise TypeError("segment must be TrajectorySegment.")
        count = self.segment.sample_count
        object.__setattr__(
            self,
            "contact_targets_world_m",
            _readonly_array(
                self.contact_targets_world_m,
                (4, 3),
                "contact_targets_world_m",
            ),
        )
        object.__setattr__(
            self,
            "contact_actual_world_m",
            _readonly_array(
                self.contact_actual_world_m,
                (count, 4, 3),
                "contact_actual_world_m",
            ),
        )
        object.__setattr__(
            self,
            "contact_drift_m",
            _readonly_array(self.contact_drift_m, (count, 4), "contact_drift_m"),
        )
        object.__setattr__(
            self,
            "rim_alpha_deg",
            _readonly_array(self.rim_alpha_deg, (count, 4), "rim_alpha_deg"),
        )
        object.__setattr__(
            self,
            "rim_width_m",
            _readonly_array(self.rim_width_m, (count, 4), "rim_width_m"),
        )
        object.__setattr__(
            self,
            "requested_body_pose_world",
            _readonly_array(
                self.requested_body_pose_world,
                (6,),
                "requested_body_pose_world",
            ),
        )
        if not isinstance(self.body_trajectory_assumption, str) or not (
            self.body_trajectory_assumption
        ):
            raise ValueError("body_trajectory_assumption must be non-empty.")
        if (
            not np.isfinite(self.requested_motion_duration_s)
            or self.requested_motion_duration_s <= 0.0
        ):
            raise ValueError("requested_motion_duration_s must be finite and positive.")

    @property
    def maximum_contact_drift_m(self) -> float:
        return float(np.max(self.contact_drift_m))


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


def _shortest_angle_delta(start: NDArray[np.float64], end: NDArray[np.float64]):
    return (end - start + np.pi) % (2.0 * np.pi) - np.pi


def _body_pose_trajectory(
    start: NDArray[np.float64],
    target: NDArray[np.float64],
    motion_steps: int,
    motion_duration_s: float,
    initial_body_velocity_world_m_s: NDArray[np.float64] | None,
) -> tuple[NDArray[np.float64], bool]:
    """Return body poses and whether a leading zero-velocity hold was inserted.

    With no incoming body velocity the profile is the cubic smoothstep with an
    explicit hold at both endpoints, so the discrete incoming and outgoing body
    velocities are zero.  When the caller supplies an incoming world velocity
    the position profile becomes the quintic Hermite that starts at exactly
    that velocity and ends at rest, so the leading hold is dropped and the
    segment can absorb a moving handover without a velocity step.
    """

    progress = np.linspace(0.0, 1.0, motion_steps + 1)
    motion = np.empty((motion_steps + 1, 6), dtype=float)
    angle_delta = _shortest_angle_delta(start[3:], target[3:])
    if initial_body_velocity_world_m_s is None:
        smooth = 3.0 * progress**2 - 2.0 * progress**3
        motion[:, :3] = start[:3] + smooth[:, None] * (target[:3] - start[:3])
        motion[:, 3:] = start[3:] + smooth[:, None] * angle_delta
        return np.vstack((motion[0], motion, motion[-1])), True

    position_basis = 10.0 * progress**3 - 15.0 * progress**4 + 6.0 * progress**5
    velocity_basis = (
        progress - 6.0 * progress**3 + 8.0 * progress**4 - 3.0 * progress**5
    )
    motion[:, :3] = (
        start[:3]
        + position_basis[:, None] * (target[:3] - start[:3])
        + motion_duration_s
        * velocity_basis[:, None]
        * initial_body_velocity_world_m_s[np.newaxis, :]
    )
    motion[:, 3:] = start[3:] + position_basis[:, None] * angle_delta
    return np.vstack((motion, motion[-1])), False


def _solve_lowest_rim_contact(
    generator: GaitGenerator3D,
    leg_index: int,
    target_body_m: NDArray[np.float64],
    guess_q: NDArray[np.float64],
    tolerance_m: float,
    *,
    maximum_iterations: int = 40,
    continuation_steps: int = 8,
) -> tuple[NDArray[np.float64], float, float, NDArray[np.float64]]:
    """Place the *lowest* rim point of one leg at a body-frame target.

    The unknown is which rim point ends up lowest, so this alternates an IK
    solve at a fixed rim point with a re-query of the lowest point.  That
    fixed-point iteration diverges when the leg is tilted far enough that a
    single update swings ``alpha`` by tens of degrees, so the update is damped
    and the damping is halved whenever an iteration fails to improve.  If the
    damped iteration still stalls, the target is approached by continuation
    from the guess pose, which keeps every intermediate solve close to a pose
    that already worked.
    """

    kinematics = generator.legs[leg_index]

    def iterate(
        start_q: NDArray[np.float64], target: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], float, float, NDArray[np.float64]] | None:
        alpha_deg, width_m = kinematics.foot_rim_contact_fk(*start_q)
        q = np.asarray(start_q, dtype=float).copy()
        best: tuple[float, NDArray[np.float64], float, float, NDArray[np.float64]] | None
        best = None
        damping = 1.0
        for _ in range(maximum_iterations):
            q = kinematics.inverse_kinematics(
                target,
                guess_q=q,
                rim_point=(alpha_deg, width_m),
                tol=min(tolerance_m, 2e-4),
            )
            next_alpha, next_width = kinematics.foot_rim_contact_fk(*q)
            actual = kinematics.forward_kinematics(*q, alpha=next_alpha, w=next_width)
            error = float(np.linalg.norm(actual - target))
            if error <= tolerance_m:
                return q, float(next_alpha), float(next_width), actual
            if best is None or error < best[0]:
                best = (error, q.copy(), float(next_alpha), float(next_width), actual)
            else:
                # Not improving: the alpha update is overshooting, so take a
                # smaller share of it.
                damping = max(damping * 0.5, 0.05)
            alpha_deg = alpha_deg + damping * (float(next_alpha) - alpha_deg)
            width_m = width_m + damping * (float(next_width) - width_m)
        return None

    target = np.asarray(target_body_m, dtype=float)
    try:
        solved = iterate(guess_q, target)
        if solved is not None:
            return solved
        # Continuation: walk the target across from where the guess pose
        # actually is, so each solve starts from a nearby configuration.
        alpha_deg, width_m = kinematics.foot_rim_contact_fk(*guess_q)
        start_point = np.asarray(
            kinematics.forward_kinematics(*guess_q, alpha=alpha_deg, w=width_m),
            dtype=float,
        )
        q = np.asarray(guess_q, dtype=float).copy()
        solved = None
        for fraction in np.linspace(0.0, 1.0, max(2, continuation_steps + 1))[1:]:
            step_target = start_point + fraction * (target - start_point)
            solved = iterate(q, step_target)
            if solved is None:
                break
            q = solved[0]
        if solved is not None:
            return solved
    except RuntimeError as exc:
        raise StancePlanningError(
            StanceRejectReason.IK_UNREACHABLE,
            str(exc),
            leg_index=leg_index,
        ) from exc
    raise StancePlanningError(
        StanceRejectReason.IK_UNREACHABLE,
        "lowest-rim contact iteration did not converge",
        leg_index=leg_index,
    )


def _check_joint_limits(commands: NDArray[np.float64]) -> None:
    """Reject poses outside the joint limits; ``beta`` is bounded by the
    foot-rim contact range, see ``swing._check_joint_limits``."""

    limits = (
        (0, np.deg2rad(RobotParams.MIN_THETA_DEG), np.deg2rad(RobotParams.MAX_THETA_DEG)),
        (1, -np.deg2rad(RobotParams.BETA_MAX_DEG), np.deg2rad(RobotParams.BETA_MAX_DEG)),
        (2, -np.deg2rad(RobotParams.GAMMA_MAX_DEG), np.deg2rad(RobotParams.GAMMA_MAX_DEG)),
    )
    for leg_index in range(4):
        for joint_index, lower, upper in limits:
            values = commands[:, leg_index, joint_index]
            invalid = np.flatnonzero((values < lower) | (values > upper))
            if len(invalid):
                sample = int(invalid[0])
                raise StancePlanningError(
                    StanceRejectReason.JOINT_LIMIT,
                    f"{JOINT_ORDER[joint_index]}={values[sample]:.6g} rad is outside "
                    f"[{lower:.6g}, {upper:.6g}] rad",
                    sample_index=sample,
                    leg_index=leg_index,
                )


def generate_stance_segment(
    generator: GaitGenerator3D,
    start_state: WalkState,
    target_body_pose_world: NDArray[np.float64],
    terrain: WalkTerrain1D,
    motion_duration_s: float,
    *,
    contact_drift_tolerance_m: float = 1e-3,
    contact_height_tolerance_m: float = 1e-3,
    maximum_joint_step_rad: float = 0.35,
    initial_body_velocity_world_m_s: NDArray[np.float64] | None = None,
    full_geometry_collision_checker: FullGeometryCollisionChecker | None = None,
) -> StancePlanResult:
    """Move the body while holding four ground/top rim contacts world-fixed."""

    if not isinstance(generator, GaitGenerator3D) or generator.gait_type != "Walk":
        raise TypeError("generator must be a Walk GaitGenerator3D.")
    if not isinstance(start_state, WalkState):
        raise TypeError("start_state must be WalkState.")
    if not isinstance(terrain, WalkTerrain1D):
        raise TypeError("terrain must be WalkTerrain1D.")
    target_pose = np.asarray(target_body_pose_world, dtype=float)
    if target_pose.shape != (6,) or not np.all(np.isfinite(target_pose)):
        raise ValueError("target_body_pose_world must contain six finite values.")
    for name, value in (
        ("motion_duration_s", motion_duration_s),
        ("contact_drift_tolerance_m", contact_drift_tolerance_m),
        ("contact_height_tolerance_m", contact_height_tolerance_m),
        ("maximum_joint_step_rad", maximum_joint_step_rad),
    ):
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive.")
    if not np.all(start_state.phase == 0):
        raise StancePlanningError(
            StanceRejectReason.INVALID_START_CONTACT,
            "Step 5 requires all four legs to begin in stance",
        )

    contact_targets = start_state.foot_contact_points_world_m.copy()
    for leg_index, point in enumerate(contact_targets):
        query = query_touchdown_surface(terrain, float(point[0]))
        if (
            not query.is_legal
            or query.surface_id != start_state.surface_ids[leg_index]
            or abs(float(point[2]) - float(query.surface_height_world_m))
            > contact_height_tolerance_m
        ):
            raise StancePlanningError(
                StanceRejectReason.INVALID_START_CONTACT,
                "world contact does not match its terrain surface and height",
                leg_index=leg_index,
            )

    incoming_velocity = None
    if initial_body_velocity_world_m_s is not None:
        incoming_velocity = np.asarray(initial_body_velocity_world_m_s, dtype=float)
        if incoming_velocity.shape != (3,) or not np.all(np.isfinite(incoming_velocity)):
            raise ValueError(
                "initial_body_velocity_world_m_s must contain three finite values."
            )
    motion_steps = max(2, int(round(motion_duration_s / generator.dt)))
    body_poses, leading_hold = _body_pose_trajectory(
        start_state.body_pose_world,
        target_pose,
        motion_steps,
        motion_steps * generator.dt,
        incoming_velocity,
    )
    first_free_sample = 2 if leading_hold else 1
    count = len(body_poses)
    commands = np.empty((count, 4, 3), dtype=float)
    actual_contacts = np.empty((count, 4, 3), dtype=float)
    rim_alpha = np.empty((count, 4), dtype=float)
    rim_width = np.empty((count, 4), dtype=float)

    commands[0] = start_state.joint_position_rad
    guesses = start_state.joint_position_rad.copy()
    for sample, body_pose in enumerate(body_poses):
        rotation = _rotation_body_to_world(body_pose[3:])
        translation = body_pose[:3]
        for leg_index, target_world in enumerate(contact_targets):
            target_body = rotation.T @ (target_world - translation)
            if sample < first_free_sample:
                q = start_state.joint_position_rad[leg_index]
                kinematics = generator.legs[leg_index]
                alpha, width = kinematics.foot_rim_contact_fk(*q)
                actual_body = kinematics.forward_kinematics(
                    *q,
                    alpha=alpha,
                    w=width,
                )
            elif sample == count - 1:
                q = commands[sample - 1, leg_index]
                kinematics = generator.legs[leg_index]
                alpha, width = kinematics.foot_rim_contact_fk(*q)
                actual_body = kinematics.forward_kinematics(
                    *q,
                    alpha=alpha,
                    w=width,
                )
            else:
                q, alpha, width, actual_body = _solve_lowest_rim_contact(
                    generator,
                    leg_index,
                    target_body,
                    guesses[leg_index],
                    contact_drift_tolerance_m,
                )
            commands[sample, leg_index] = q
            actual_contacts[sample, leg_index] = translation + rotation @ actual_body
            rim_alpha[sample, leg_index] = alpha
            rim_width[sample, leg_index] = width
            guesses[leg_index] = q

    drift = np.linalg.norm(actual_contacts - contact_targets[np.newaxis, :, :], axis=2)
    if float(np.max(drift)) > contact_drift_tolerance_m:
        sample, leg_index = np.unravel_index(int(np.argmax(drift)), drift.shape)
        raise StancePlanningError(
            StanceRejectReason.CONTACT_DRIFT,
            f"drift {drift[sample, leg_index]:.6g} m exceeds "
            f"{contact_drift_tolerance_m:.6g} m",
            sample_index=int(sample),
            leg_index=int(leg_index),
        )

    for sample in range(count):
        for leg_index, point in enumerate(actual_contacts[sample]):
            query = query_touchdown_surface(terrain, float(point[0]))
            if (
                not query.is_legal
                or query.surface_id != start_state.surface_ids[leg_index]
                or abs(float(point[2]) - float(query.surface_height_world_m))
                > contact_height_tolerance_m
            ):
                raise StancePlanningError(
                    StanceRejectReason.CONTACT_SURFACE_MISMATCH,
                    "tracked lowest-rim point left its assigned terrain surface",
                    sample_index=sample,
                    leg_index=leg_index,
                )

    joint_steps = np.max(np.abs(np.diff(commands, axis=0)), axis=2)
    if np.any(joint_steps > maximum_joint_step_rad):
        step, leg_index = np.unravel_index(int(np.argmax(joint_steps)), joint_steps.shape)
        raise StancePlanningError(
            StanceRejectReason.JOINT_DISCONTINUITY,
            f"joint step {joint_steps[step, leg_index]:.6g} rad exceeds "
            f"{maximum_joint_step_rad:.6g} rad",
            sample_index=int(step + 1),
            leg_index=int(leg_index),
        )
    _check_joint_limits(commands)

    if full_geometry_collision_checker is not None:
        for sample in range(count):
            for leg_index, leg in enumerate(LEG_ORDER):
                collision = full_geometry_collision_checker(
                    sample_index=sample,
                    leg=leg,
                    joint_position_rad=commands[sample, leg_index].copy(),
                    body_pose_world=body_poses[sample].copy(),
                    terrain=terrain,
                )
                if collision is not None:
                    raise StancePlanningError(
                        StanceRejectReason.FULL_GEOMETRY_COLLISION,
                        collision,
                        sample_index=sample,
                        leg_index=leg_index,
                    )

    phase = np.zeros((count, 4), dtype=np.int8)
    active = np.ones((count, 4), dtype=bool)
    surfaces = tuple(start_state.surface_ids for _ in range(count))
    gait_phase = (
        start_state.gait_cycle_phase
        + np.arange(count, dtype=float) * generator.dt / generator.T
    ) % 1.0
    # Preserve the exact shared-boundary metadata supplied by start_state.
    # The independently recomputed FK value remains available in
    # StancePlanResult.contact_actual_world_m and is included in drift checks.
    segment_contacts = actual_contacts.copy()
    segment_contacts[0] = contact_targets
    final_state = WalkState(
        joint_position_rad=commands[-1],
        previous_joint_position_rad=commands[-2],
        body_pose_world=body_poses[-1],
        foot_contact_points_world_m=segment_contacts[-1],
        phase=phase[-1],
        contact_active=active[-1],
        surface_ids=surfaces[-1],
        gait_cycle_phase=float(gait_phase[-1]),
        next_swing_leg=start_state.next_swing_leg,
    )
    segment = TrajectorySegment(
        time_s=np.arange(count, dtype=float) * generator.dt,
        commands_rad=commands,
        phase=phase,
        body_pose_world=body_poses,
        foot_contact_points_world_m=segment_contacts,
        contact_active=active,
        gait_cycle_phase=gait_phase,
        surface_ids=surfaces,
        start_state=start_state,
        final_state=final_state,
        dt_s=generator.dt,
        segment_type=SegmentType.STANCE,
    )
    return StancePlanResult(
        segment=segment,
        contact_targets_world_m=contact_targets,
        contact_actual_world_m=actual_contacts,
        contact_drift_m=drift,
        rim_alpha_deg=rim_alpha,
        rim_width_m=rim_width,
        requested_body_pose_world=target_pose,
        body_trajectory_assumption=(
            BODY_TRAJECTORY_ASSUMPTION
            if incoming_velocity is None
            else BRAKING_BODY_TRAJECTORY_ASSUMPTION
        ),
        requested_motion_duration_s=motion_duration_s,
        full_geometry_collision_checked=full_geometry_collision_checker is not None,
    )


def plot_stance_plan(
    plan: StancePlanResult,
    terrain: WalkTerrain1D,
    output_path: str | Path,
) -> Path:
    """Save an X-Z plot of the body path and fixed support contacts."""

    if not isinstance(plan, StancePlanResult):
        raise TypeError("plan must be StancePlanResult.")
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
        plan.segment.body_pose_world[:, 0],
        plan.segment.body_pose_world[:, 2],
        "o-",
        markersize=3,
        color="tab:blue",
        label="body smoothstep",
    )
    for leg_index, leg in enumerate(LEG_ORDER):
        axis.scatter(
            [plan.contact_targets_world_m[leg_index, 0]],
            [plan.contact_targets_world_m[leg_index, 2]],
            marker="x",
            s=65,
            label=f"{leg.value} fixed contact",
        )
    axis.set_xlabel("world x (m)")
    axis.set_ylabel("world z (m)")
    axis.set_title(
        "Step 5 mixed-height stance, max contact drift "
        f"{plan.maximum_contact_drift_m * 1e3:.2f} mm"
    )
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", ncol=2)
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return output
