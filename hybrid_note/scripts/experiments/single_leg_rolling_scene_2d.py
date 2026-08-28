"""Day 6--7 fixed-pose, query, and roll-up experiments.

This module contains the small fixed-pose scene used by Day 6--7, the
terrain-aware query visualization, the fixed-theta right-rim roll-up test,
and the Step 4 brute-force theta sweep.  It is not a planner, FSM, recovery
controller, or optimization routine.

The leg geometry is reused from the existing ``PlotLeg`` model and
``sample_contact_geometry_points`` helper.  The terrain is the existing 2D
``TerrainProfile`` contract: flat ground with zero or one rectangular
obstacle.

Public angle inputs are radians, matching the existing kinematics API.  The
command-line entry point accepts degrees for convenience.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
import numpy as np

from legwheel.config import RobotParams
from legwheel.planners.hybrid import (
    ContactQueryResult2D,
    ContactStatus2D,
    HipPose2D,
    RectangleObstacle2D,
    SampledLegGeometry2D,
    TerrainProfile2D,
    candidate_status_2d,
    legacy_leg_link_segments_2d,
    plot_terrain_profile_2d,
    query_contact,
    RimId,
    sampled_leg_geometry_from_legacy_records,
)
from legwheel.visualization.plot_leg import PlotLeg

from ..kinematics.ground_contact_single_pose import sample_contact_geometry_points


RIM_SURFACE_COLORS = {
    "foot_rim": "#16a34a",
    "upper_tyre_l": "#2563eb",
    "upper_tyre_r": "#f97316",
}
RIM_SURFACE_LABELS = {
    "foot_rim": "foot rim (F)",
    "upper_tyre_l": "left upper tyre (L)",
    "upper_tyre_r": "right upper tyre (R)",
}
CONTACT_REGION_COLORS = {
    "foot_rim": RIM_SURFACE_COLORS["foot_rim"],
    "left_rim": RIM_SURFACE_COLORS["upper_tyre_l"],
    "right_rim": RIM_SURFACE_COLORS["upper_tyre_r"],
    "non_contact_region": "#6b7280",
}


def _finite_scalar(value: float, name: str) -> float:
    """Validate one finite scalar without silently accepting a vector."""

    array = np.asarray(value, dtype=float)
    if array.shape != () or not np.isfinite(array):
        raise ValueError(f"{name} must be one finite scalar.")
    return float(array)


@dataclass(frozen=True)
class SingleLegRollingScene2D:
    """All immutable data needed to redraw one fixed single-leg scene.

    ``geometry`` contains the three physical sampled tyre arcs in hip/world
    ``x-z`` coordinates.  The full linkage/wheel drawing is regenerated from
    the existing ``PlotLeg`` model by the plotting function.
    """

    theta_rad: float
    beta_rad: float
    gamma_rad: float
    hip_pose: HipPose2D
    terrain: TerrainProfile2D
    geometry: SampledLegGeometry2D

    def __post_init__(self) -> None:
        theta = _finite_scalar(self.theta_rad, "theta_rad")
        beta = _finite_scalar(self.beta_rad, "beta_rad")
        gamma = _finite_scalar(self.gamma_rad, "gamma_rad")
        if not np.isclose(gamma, 0.0, atol=1e-12):
            raise ValueError("Day 6--7 Step 1 only supports gamma = 0.")
        if not isinstance(self.hip_pose, HipPose2D):
            raise TypeError("hip_pose must be a HipPose2D.")
        if not np.isclose(self.hip_pose.pitch_world_hip_rad, 0.0, atol=1e-12):
            raise ValueError("Day 6--7 Step 1 only supports zero hip pitch.")
        if not isinstance(self.terrain, TerrainProfile2D):
            raise TypeError("terrain must be a TerrainProfile2D.")
        if not isinstance(self.geometry, SampledLegGeometry2D):
            raise TypeError("geometry must be a SampledLegGeometry2D.")
        object.__setattr__(self, "theta_rad", theta)
        object.__setattr__(self, "beta_rad", beta)
        object.__setattr__(self, "gamma_rad", gamma)

    @property
    def theta_deg(self) -> float:
        return float(np.rad2deg(self.theta_rad))

    @property
    def beta_deg(self) -> float:
        return float(np.rad2deg(self.beta_rad))

    @property
    def obstacle_x_start_m(self) -> float | None:
        obstacle = self.terrain.obstacle
        return None if obstacle is None else float(obstacle.x_min_m)

    @property
    def obstacle_width_m(self) -> float | None:
        obstacle = self.terrain.obstacle
        return None if obstacle is None else float(obstacle.width_m)

    @property
    def obstacle_top_length_m(self) -> float | None:
        """Alias for the rectangle width used as its top-surface length."""

        return self.obstacle_width_m

    @property
    def obstacle_height_m(self) -> float | None:
        obstacle = self.terrain.obstacle
        return None if obstacle is None else float(obstacle.height_m)


def build_single_leg_rolling_scene_2d(
    theta_rad: float,
    beta_rad: float,
    hip_x_m: float,
    hip_z_m: float,
    *,
    gamma_rad: float = 0.0,
    ground_height_m: float = 0.0,
    obstacle_x_start_m: float | None = 0.10,
    obstacle_width_m: float = 0.20,
    obstacle_height_m: float = 0.05,
    obstacle_id: str = "day6_7_obstacle",
    arc_samples: int = 121,
) -> SingleLegRollingScene2D:
    """Build one fixed-pose single-leg scene.

    Args:
        theta_rad: Existing LegWheel leg angle in radians.
        beta_rad: Existing LegWheel leg angle in radians.
        hip_x_m: Hip/origin ``x`` in world coordinates.
        hip_z_m: Hip/origin ``z`` in world coordinates.
        gamma_rad: Must remain exactly zero in this 2D Step-1 scene.
        ground_height_m: Height of the flat ground.
        obstacle_x_start_m: Leading/front x coordinate.  ``None`` means flat
            ground only.
        obstacle_width_m: Rectangle width, also the obstacle top length.
        obstacle_height_m: Rectangle height above the ground.
        obstacle_id: Stable terrain surface prefix.
        arc_samples: Samples per physical tyre arc for the rim markers.
    """

    theta = _finite_scalar(theta_rad, "theta_rad")
    beta = _finite_scalar(beta_rad, "beta_rad")
    gamma = _finite_scalar(gamma_rad, "gamma_rad")
    hip_x = _finite_scalar(hip_x_m, "hip_x_m")
    hip_z = _finite_scalar(hip_z_m, "hip_z_m")
    ground_height = _finite_scalar(ground_height_m, "ground_height_m")
    if not np.isclose(gamma, 0.0, atol=1e-12):
        raise ValueError("Day 6--7 Step 1 only supports gamma = 0.")
    if arc_samples < 2:
        raise ValueError("arc_samples must be at least 2.")

    if obstacle_x_start_m is None:
        terrain = TerrainProfile2D(ground_height_m=ground_height)
    else:
        obstacle_x_start = _finite_scalar(obstacle_x_start_m, "obstacle_x_start_m")
        obstacle_width = _finite_scalar(obstacle_width_m, "obstacle_width_m")
        obstacle_height = _finite_scalar(obstacle_height_m, "obstacle_height_m")
        obstacle = RectangleObstacle2D(
            obstacle_id=obstacle_id,
            x_min_m=obstacle_x_start,
            x_max_m=obstacle_x_start + obstacle_width,
            height_m=obstacle_height,
        )
        terrain = TerrainProfile2D(
            ground_height_m=ground_height,
            obstacles=(obstacle,),
        )

    hip_pose = HipPose2D([hip_x, hip_z], pitch_world_hip_rad=0.0)
    # Reuse the existing legacy sampler; it is the source of truth for the
    # physical foot/left/right tyre arcs and F/L/R/N semantic regions.
    records = sample_contact_geometry_points(
        theta=theta,
        beta=beta,
        arc_samples=arc_samples,
        include_reference_points=True,
    )
    geometry = sampled_leg_geometry_from_legacy_records(
        records,
        hip_pose,
        link_segments_hip_xz_m=legacy_leg_link_segments_2d(theta, beta),
    )
    return SingleLegRollingScene2D(
        theta_rad=theta,
        beta_rad=beta,
        gamma_rad=gamma,
        hip_pose=hip_pose,
        terrain=terrain,
        geometry=geometry,
    )


def _plot_x_limits(scene: SingleLegRollingScene2D) -> tuple[float, float]:
    points = scene.geometry.points_world_xz_m
    x_values = list(points[:, 0]) + [scene.hip_pose.position_world_xz_m[0]]
    if scene.terrain.obstacle is not None:
        x_values.extend(
            [scene.terrain.obstacle.x_min_m, scene.terrain.obstacle.x_max_m]
        )
    x_min = min(x_values)
    x_max = max(x_values)
    padding = max(0.08, 0.16 * max(x_max - x_min, 0.1))
    return x_min - padding, x_max + padding


def query_single_leg_rolling_scene_2d(
    scene: SingleLegRollingScene2D,
    *,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
) -> ContactQueryResult2D:
    """Run the existing terrain-aware query on one fixed scene."""

    if not isinstance(scene, SingleLegRollingScene2D):
        raise TypeError("scene must be a SingleLegRollingScene2D.")
    return query_contact(
        scene.geometry,
        scene.terrain,
        contact_tolerance_m=contact_tolerance_m,
        collision_tolerance_m=collision_tolerance_m,
    )


@dataclass(frozen=True)
class RightRimRollUpFrame2D:
    """One accepted or rejected fixed-theta beta-update frame."""

    step: int
    theta_rad: float
    beta_rad: float
    active_rim: str | None
    alpha_rad: float | None
    contact_point_world_xz_m: tuple[float, float] | None
    terrain_surface_id: str | None
    valid_contact: bool
    collision: bool
    status: ContactStatus2D
    accepted: bool
    failure_reason: str | None
    scene: SingleLegRollingScene2D
    query_result: ContactQueryResult2D


@dataclass(frozen=True)
class RightRimRollUpResult2D:
    """Fixed-theta right-rim roll-up result; no theta search or recovery."""

    theta_climb_rad: float
    initial_beta_rad: float
    beta_step_rad: float
    frames: tuple[RightRimRollUpFrame2D, ...]
    success: bool
    failure_reason: str | None

    @property
    def final_beta_rad(self) -> float:
        return self.frames[-1].beta_rad

    @property
    def final_frame(self) -> RightRimRollUpFrame2D:
        return self.frames[-1]


def _right_rim_candidate_kind(candidate, terrain: TerrainProfile2D) -> str | None:
    if candidate.rim is not RimId.RIGHT:
        return None
    kind = terrain.surface_by_id(candidate.terrain_surface_id).kind
    if kind.value == "obstacle_front":
        return "front"
    if kind.value == "obstacle_top":
        return "top"
    return None


def _select_active_right_rim_candidate(
    result: ContactQueryResult2D,
    terrain: TerrainProfile2D,
    previous_surface: str | None,
) -> tuple[object | None, str | None]:
    """Select one physical right-rim candidate for the fixed-theta log.

    This is a deterministic observation rule, not a planner: the first frame
    prefers the free-space side of the front face, then a top candidate is
    preferred once the front contact has been established.
    """

    candidates = [
        candidate
        for candidate in result.candidates
        if _right_rim_candidate_kind(candidate, terrain) is not None
    ]
    front = [candidate for candidate in candidates if _right_rim_candidate_kind(candidate, terrain) == "front"]
    top = [candidate for candidate in candidates if _right_rim_candidate_kind(candidate, terrain) == "top"]
    obstacle = terrain.obstacle

    if previous_surface is None:
        free_front = [
            candidate
            for candidate in front
            if candidate.terrain_gap_m >= 0.0
            and (obstacle is None or candidate.point_world_xz_m[0] <= obstacle.x_min_m)
        ]
        if free_front:
            return max(free_front, key=lambda candidate: candidate.point_world_xz_m[0]), "front"
        if front:
            return min(front, key=lambda candidate: abs(candidate.terrain_gap_m)), "front"
        return (max(top, key=lambda candidate: candidate.point_world_xz_m[0]), "top") if top else (None, None)

    if previous_surface == "front":
        if top:
            return max(top, key=lambda candidate: candidate.point_world_xz_m[0]), "top"
        if front:
            return max(front, key=lambda candidate: candidate.point_world_xz_m[0]), "front"
    elif previous_surface == "top" and top:
        return max(top, key=lambda candidate: candidate.point_world_xz_m[0]), "top"
    return (None, None)


def run_fixed_theta_right_rim_roll_up_2d(
    theta_climb_rad: float,
    initial_beta_rad: float,
    hip_x_m: float,
    hip_z_m: float,
    *,
    beta_step_rad: float = -np.deg2rad(1.0),
    max_steps: int = 12,
    gamma_rad: float = 0.0,
    ground_height_m: float = 0.0,
    obstacle_x_start_m: float = 0.10,
    obstacle_width_m: float = 0.60,
    obstacle_height_m: float = 0.10,
    obstacle_id: str = "day6_7_step3_obstacle",
    arc_samples: int = 241,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
    forward_progress_tolerance_m: float = 2e-4,
) -> RightRimRollUpResult2D:
    """Run one fixed-theta beta-only right-rim roll-up experiment.

    The initial beta and signed beta step are explicit inputs.  Every frame
    rebuilds the existing fixed-pose scene and calls ``query_contact``.  The
    run stops at the first legal right-rim top contact; it never changes theta,
    retracts, swings, or sweeps obstacle height/theta.
    """

    theta = _finite_scalar(theta_climb_rad, "theta_climb_rad")
    initial_beta = _finite_scalar(initial_beta_rad, "initial_beta_rad")
    beta_step = _finite_scalar(beta_step_rad, "beta_step_rad")
    if np.isclose(beta_step, 0.0, atol=1e-15):
        raise ValueError("beta_step_rad must be non-zero.")
    if max_steps < 0:
        raise ValueError("max_steps must be non-negative.")
    if forward_progress_tolerance_m < 0.0 or not np.isfinite(forward_progress_tolerance_m):
        raise ValueError("forward_progress_tolerance_m must be finite and non-negative.")

    frames = []
    previous_surface = None
    previous_point_x = None
    success = False
    failure_reason = None

    for step in range(max_steps + 1):
        beta = initial_beta + step * beta_step
        scene = build_single_leg_rolling_scene_2d(
            theta,
            beta,
            hip_x_m,
            hip_z_m,
            gamma_rad=gamma_rad,
            ground_height_m=ground_height_m,
            obstacle_x_start_m=obstacle_x_start_m,
            obstacle_width_m=obstacle_width_m,
            obstacle_height_m=obstacle_height_m,
            obstacle_id=obstacle_id,
            arc_samples=arc_samples,
        )
        query_result = query_single_leg_rolling_scene_2d(
            scene,
            contact_tolerance_m=contact_tolerance_m,
            collision_tolerance_m=collision_tolerance_m,
        )
        candidate, surface = _select_active_right_rim_candidate(
            query_result,
            scene.terrain,
            previous_surface,
        )
        contact_point = (
            None
            if candidate is None
            else (
                float(candidate.point_world_xz_m[0]),
                float(candidate.point_world_xz_m[1]),
            )
        )
        alpha_rad = None if candidate is None else float(candidate.alpha_rad)
        active_rim = None if candidate is None else candidate.rim.value
        status = (
            ContactStatus2D.NO_CONTACT
            if candidate is None
            else candidate_status_2d(candidate, scene.terrain)
        )
        reason = None

        if query_result.collision:
            reason = "INVALID_COLLISION_OR_PENETRATION"
        elif candidate is None:
            reason = "NO_VALID_RIGHT_RIM_CONTACT"
        elif previous_surface is None and surface != "front":
            reason = "TOP_CONTACT_BEFORE_FRONT_CONTACT"
        elif previous_surface == "top" and surface != "top":
            reason = "ILLEGAL_RIM_TRANSITION_FROM_TOP"
        elif previous_point_x is not None and contact_point[0] < previous_point_x - forward_progress_tolerance_m:
            reason = "FORWARD_PROGRESS_REVERSED"
        elif any(
            item.rim is not RimId.RIGHT
            and item.terrain_surface_id != scene.terrain.ground_surface_id
            for item in query_result.candidates
        ):
            reason = "OTHER_RIM_OBSTACLE_CONTACT"

        accepted = reason is None
        frame = RightRimRollUpFrame2D(
            step=step,
            theta_rad=theta,
            beta_rad=beta,
            active_rim=active_rim,
            alpha_rad=alpha_rad,
            contact_point_world_xz_m=contact_point,
            terrain_surface_id=None if candidate is None else candidate.terrain_surface_id,
            valid_contact=candidate is not None,
            collision=query_result.collision,
            status=status,
            accepted=accepted,
            failure_reason=reason,
            scene=scene,
            query_result=query_result,
        )
        frames.append(frame)

        if not accepted:
            failure_reason = reason
            break
        previous_surface = surface
        previous_point_x = contact_point[0]
        if surface == "top":
            success = True
            break

    if not success and failure_reason is None:
        failure_reason = "MAX_BETA_STEPS_REACHED_BEFORE_TOP_CONTACT"
    return RightRimRollUpResult2D(
        theta_climb_rad=theta,
        initial_beta_rad=initial_beta,
        beta_step_rad=beta_step,
        frames=tuple(frames),
        success=success,
        failure_reason=None if success else failure_reason,
    )


def right_rim_roll_up_rows(result: RightRimRollUpResult2D) -> list[dict]:
    """Return one table row per fixed-theta roll-up frame."""

    if not isinstance(result, RightRimRollUpResult2D):
        raise TypeError("result must be RightRimRollUpResult2D.")
    return [
        {
            "step": frame.step,
            "theta_rad": frame.theta_rad,
            "theta_deg": np.rad2deg(frame.theta_rad),
            "beta_rad": frame.beta_rad,
            "beta_deg": np.rad2deg(frame.beta_rad),
            "active_rim": frame.active_rim,
            "alpha_rad": frame.alpha_rad,
            "alpha_deg": None if frame.alpha_rad is None else np.rad2deg(frame.alpha_rad),
            "contact_point_world_xz_m": frame.contact_point_world_xz_m,
            "terrain_surface_id": frame.terrain_surface_id,
            "valid_contact": frame.valid_contact,
            "collision": frame.collision,
            "status": frame.status.value,
            "accepted": frame.accepted,
            "failure_reason": frame.failure_reason,
        }
        for frame in result.frames
    ]


@dataclass(frozen=True)
class RightRimThetaSweepRow2D:
    """One theta value and the result of its fixed-theta roll-up test."""

    theta_climb_rad: float
    roll_up_success: bool
    failure_reason: str | None
    final_beta_rad: float
    final_rim: str | None
    final_contact_surface: str | None

    @property
    def theta_climb_deg(self) -> float:
        return float(np.rad2deg(self.theta_climb_rad))

    @property
    def final_beta_deg(self) -> float:
        return float(np.rad2deg(self.final_beta_rad))

    def as_dict(self) -> dict:
        """Return a CSV/table-friendly row with degrees and radians."""

        return {
            "theta_climb_rad": self.theta_climb_rad,
            "theta_climb_deg": self.theta_climb_deg,
            "roll_up_success": self.roll_up_success,
            "failure_reason": self.failure_reason,
            "final_beta_rad": self.final_beta_rad,
            "final_beta_deg": self.final_beta_deg,
            "final_rim": self.final_rim,
            "final_contact_surface": self.final_contact_surface,
        }


@dataclass(frozen=True)
class RightRimThetaSweepResult2D:
    """Brute-force fixed-initial-condition theta sweep result."""

    theta_min_rad: float
    theta_max_rad: float
    dtheta_rad: float
    rows: tuple[RightRimThetaSweepRow2D, ...]

    @property
    def feasible_rows(self) -> tuple[RightRimThetaSweepRow2D, ...]:
        return tuple(row for row in self.rows if row.roll_up_success)

    @property
    def minimum_feasible_theta_rad(self) -> float | None:
        feasible = self.feasible_rows
        return None if not feasible else min(row.theta_climb_rad for row in feasible)

    @property
    def maximum_feasible_theta_rad(self) -> float | None:
        feasible = self.feasible_rows
        return None if not feasible else max(row.theta_climb_rad for row in feasible)

    @property
    def feasible_theta_range_rad(self) -> tuple[float, float] | None:
        minimum = self.minimum_feasible_theta_rad
        maximum = self.maximum_feasible_theta_rad
        return None if minimum is None or maximum is None else (minimum, maximum)

    @property
    def minimum_feasible_theta_deg(self) -> float | None:
        value = self.minimum_feasible_theta_rad
        return None if value is None else float(np.rad2deg(value))

    @property
    def maximum_feasible_theta_deg(self) -> float | None:
        value = self.maximum_feasible_theta_rad
        return None if value is None else float(np.rad2deg(value))

    @property
    def feasible_theta_range_deg(self) -> tuple[float, float] | None:
        value = self.feasible_theta_range_rad
        return None if value is None else tuple(float(np.rad2deg(item)) for item in value)


@dataclass(frozen=True)
class ForwardRollingFrame2D:
    """One forward-rolling simulation frame after a local pose search."""

    step: int
    hip_x_m: float
    hip_forward_progress_m: float
    theta_rad: float
    beta_rad: float
    active_rim: str | None
    alpha_rad: float | None
    contact_point_world_xz_m: tuple[float, float] | None
    continuation_target_world_xz_m: tuple[float, float] | None
    continuation_error_m: float | None
    leading_edge_clearance_m: float | None
    top_roll_progress_m: float | None
    top_contact_advance_m: float | None
    top_roll_complete: bool
    top_roll_remaining_m: float | None
    roll_phase: str
    terrain_surface_id: str | None
    contact_phase: str | None
    valid_contact: bool
    collision: bool
    status: ContactStatus2D
    accepted: bool
    failure_reason: str | None
    scene: SingleLegRollingScene2D
    query_result: ContactQueryResult2D


@dataclass(frozen=True)
class ForwardRollingResult2D:
    """One candidate-theta forward simulation result."""

    candidate_theta_rad: float
    initial_beta_rad: float
    dx_m: float
    top_roll_distance_m: float
    frames: tuple[ForwardRollingFrame2D, ...]
    success: bool
    failure_reason: str | None

    @property
    def final_frame(self) -> ForwardRollingFrame2D:
        return self.frames[-1]

    @property
    def final_hip_x_m(self) -> float:
        return self.final_frame.hip_x_m

    @property
    def final_theta_rad(self) -> float:
        return self.final_frame.theta_rad

    @property
    def final_beta_rad(self) -> float:
        return self.final_frame.beta_rad


@dataclass(frozen=True)
class RetractPreviewFrame2D:
    """One hypothetical fixed-top-contact step toward the retract target."""

    step: int
    theta_rad: float
    beta_rad: float
    hip_position_world_xz_m: tuple[float, float]
    active_sample_index: int
    alpha_rad: float
    contact_point_world_xz_m: tuple[float, float]
    collision: bool
    accepted: bool
    failure_reason: str | None
    scene: SingleLegRollingScene2D
    query_result: ContactQueryResult2D


@dataclass(frozen=True)
class RetractReadinessResult2D:
    """Phase-D guard result; preview frames are not committed motion."""

    retract_theta_target_rad: float
    preview_step_rad: float
    stable_window_frames: int
    required_leading_edge_clearance_m: float
    measured_leading_edge_clearance_m: float | None
    rolling_complete: bool
    stable_top_contact: bool
    no_slip_stable: bool
    collision_free: bool
    clearance_satisfied: bool
    preview_success: bool
    ready_to_retract: bool
    failure_reason: str | None
    max_no_slip_error_m: float | None
    preview_frames: tuple[RetractPreviewFrame2D, ...]

    @property
    def retract_theta_target_deg(self) -> float:
        return float(np.rad2deg(self.retract_theta_target_rad))

    def as_dict(self) -> dict:
        return {
            "ready_to_retract": self.ready_to_retract,
            "failure_reason": self.failure_reason,
            "rolling_complete": self.rolling_complete,
            "stable_top_contact": self.stable_top_contact,
            "no_slip_stable": self.no_slip_stable,
            "collision_free": self.collision_free,
            "clearance_satisfied": self.clearance_satisfied,
            "preview_success": self.preview_success,
            "stable_window_frames": self.stable_window_frames,
            "required_leading_edge_clearance_m": self.required_leading_edge_clearance_m,
            "measured_leading_edge_clearance_m": self.measured_leading_edge_clearance_m,
            "max_no_slip_error_m": self.max_no_slip_error_m,
            "retract_theta_target_deg": self.retract_theta_target_deg,
            "preview_steps_completed": len(self.preview_frames),
        }


@dataclass(frozen=True)
class RetractToWheelFrame2D:
    """One committed Step-5 frame while theta retracts toward 17 degrees."""

    step: int
    theta_rad: float
    beta_rad: float
    hip_position_world_xz_m: tuple[float, float]
    active_rim: str | None
    active_sample_index: int | None
    alpha_rad: float | None
    contact_point_world_xz_m: tuple[float, float] | None
    terrain_surface_id: str | None
    valid_contact: bool
    collision: bool
    joint_limits_ok: bool
    accepted: bool
    failure_reason: str | None
    scene: SingleLegRollingScene2D
    query_result: ContactQueryResult2D


@dataclass(frozen=True)
class RetractToWheelResult2D:
    """Full fixed-contact retract segment attached to one roll-up result."""

    rolling_result: ForwardRollingResult2D
    theta_target_rad: float
    theta_step_rad: float
    beta_min_rad: float
    beta_max_rad: float
    frames: tuple[RetractToWheelFrame2D, ...]
    success: bool
    failure_theta_rad: float | None
    failure_beta_rad: float | None
    failure_reason: str | None

    @property
    def final_frame(self) -> RetractToWheelFrame2D:
        return self.frames[-1]

    @property
    def theta_target_deg(self) -> float:
        return float(np.rad2deg(self.theta_target_rad))


@dataclass(frozen=True)
class WheelResetRollFrame2D:
    """One legacy unsigned-arc frame during the sequential Step-6 reset."""

    step: int
    beta_rad: float
    reset_rotation_rad: float
    active_rim: str | None
    active_sample_index: int | None
    alpha_rad: float | None
    contact_point_world_xz_m: tuple[float, float] | None
    terrain_surface_id: str | None
    hip_position_world_xz_m: tuple[float, float]
    hip_forward_displacement_m: float
    contact_forward_displacement_m: float
    valid_contact: bool
    collision: bool
    joint_limits_ok: bool
    foot_rim_ready: bool
    accepted: bool
    failure_reason: str | None
    scene: SingleLegRollingScene2D
    query_result: ContactQueryResult2D


@dataclass(frozen=True)
class WheelResetRollResult2D:
    """Step-6 right-rim to foot-rim reset result and measured L_reset."""

    retract_result: RetractToWheelResult2D
    beta_step_rad: float
    foot_ready_alpha_target_rad: float
    foot_ready_alpha_tolerance_rad: float
    max_reset_rotation_rad: float
    max_reset_forward_distance_m: float
    frames: tuple[WheelResetRollFrame2D, ...]
    success: bool
    required_reset_rotation_rad: float | None
    required_reset_forward_distance_m: float | None
    failure_reason: str | None

    @property
    def final_frame(self) -> WheelResetRollFrame2D:
        return self.frames[-1]

    @property
    def l_reset_m(self) -> float | None:
        return self.required_reset_forward_distance_m

    @property
    def required_reset_rotation_deg(self) -> float | None:
        value = self.required_reset_rotation_rad
        return None if value is None else float(np.rad2deg(value))


@dataclass(frozen=True)
class RetractResetFrame2D:
    """One committed Step-6.5 frame with simultaneous retract and reset."""

    step: int
    branch: str
    theta_rad: float
    beta_rad: float
    beta_unwrapped_rad: float
    accumulated_rotation_rad: float
    active_rim: str | None
    active_sample_index: int | None
    alpha_rad: float | None
    contact_point_world_xz_m: tuple[float, float] | None
    terrain_surface_id: str | None
    hip_position_world_xz_m: tuple[float, float]
    hip_forward_displacement_m: float
    contact_forward_displacement_m: float
    step_hip_displacement_m: float
    step_contact_displacement_m: float
    no_slip_tangent_residual_m: float
    valid_contact: bool
    collision: bool
    joint_limits_ok: bool
    foot_rim_ready: bool
    accepted: bool
    failure_reason: str | None
    scene: SingleLegRollingScene2D
    query_result: ContactQueryResult2D


@dataclass(frozen=True)
class RetractResetBranchResult2D:
    """One Step-6.5 rotation-direction branch."""

    rolling_result: ForwardRollingResult2D
    branch: str
    beta_direction: int
    direction_reversal: bool
    theta_target_rad: float
    foot_ready_alpha_target_rad: float
    foot_ready_alpha_tolerance_rad: float
    max_rotation_rad: float
    max_forward_distance_m: float
    frames: tuple[RetractResetFrame2D, ...]
    success: bool
    required_rotation_rad: float | None
    required_forward_distance_m: float | None
    failure_reason: str | None

    @property
    def final_frame(self) -> RetractResetFrame2D:
        return self.frames[-1]

    @property
    def required_rotation_deg(self) -> float | None:
        value = self.required_rotation_rad
        return None if value is None else float(np.rad2deg(value))

    @property
    def l_reset_m(self) -> float | None:
        return self.required_forward_distance_m


@dataclass(frozen=True)
class RetractResetComparison2D:
    """Both Step-6.5 branches and the deterministic kinematic selection."""

    rolling_result: ForwardRollingResult2D
    branches: tuple[RetractResetBranchResult2D, ...]
    selected_branch: str | None
    selection_basis: str

    @property
    def selected_result(self) -> RetractResetBranchResult2D | None:
        return next(
            (item for item in self.branches if item.branch == self.selected_branch),
            None,
        )


@dataclass(frozen=True)
class AirborneRetractResetFrame2D:
    """One Step-6.75 frame from top-contact release through touchdown.

    Contact is required only for ``TAKEOFF_CONTACT`` and ``TOUCHDOWN``.  The
    intermediate phases deliberately represent an unloaded leg; collision and
    joint-limit checks remain mandatory at every sampled configuration.
    """

    step: int
    branch: str
    phase: str
    theta_rad: float
    beta_rad: float
    beta_unwrapped_rad: float
    accumulated_rotation_rad: float
    hip_position_world_xz_m: tuple[float, float]
    hip_forward_displacement_m: float
    hip_vertical_displacement_m: float
    active_rim: str | None
    active_sample_index: int | None
    alpha_rad: float | None
    contact_point_world_xz_m: tuple[float, float] | None
    terrain_surface_id: str | None
    contact_required: bool
    valid_contact: bool
    collision: bool
    joint_limits_ok: bool
    minimum_clearance_above_top_m: float
    foot_rim_ready: bool
    accepted: bool
    failure_reason: str | None
    scene: SingleLegRollingScene2D
    query_result: ContactQueryResult2D


@dataclass(frozen=True)
class AirborneRetractResetBranchResult2D:
    """One airborne beta-direction branch after a successful Step-4.5 roll-up."""

    rolling_result: ForwardRollingResult2D
    branch: str
    beta_direction: int
    direction_reversal: bool
    theta_target_rad: float
    beta_target_rad: float
    airborne_clearance_m: float
    touchdown_contact_advance_m: float
    other_leg_support_assumed: bool
    frames: tuple[AirborneRetractResetFrame2D, ...]
    success: bool
    required_rotation_rad: float | None
    required_hip_forward_distance_m: float | None
    maximum_hip_lift_m: float | None
    failure_reason: str | None

    @property
    def final_frame(self) -> AirborneRetractResetFrame2D:
        return self.frames[-1]

    @property
    def required_rotation_deg(self) -> float | None:
        value = self.required_rotation_rad
        return None if value is None else float(np.rad2deg(value))


@dataclass(frozen=True)
class AirborneRetractResetComparison2D:
    """Both Step-6.75 airborne branches and their kinematic selection."""

    rolling_result: ForwardRollingResult2D
    branches: tuple[AirborneRetractResetBranchResult2D, ...]
    selected_branch: str | None
    selection_basis: str
    stability_scope: str

    @property
    def selected_result(self) -> AirborneRetractResetBranchResult2D | None:
        return next(
            (item for item in self.branches if item.branch == self.selected_branch),
            None,
        )


@dataclass(frozen=True)
class ForwardRollingSweepRow2D:
    """Summary row for one theta candidate after forward simulation."""

    candidate_theta_rad: float
    continuous_roll_up_success: bool
    failure_reason: str | None
    final_hip_x_m: float
    final_theta_rad: float
    final_beta_rad: float
    final_hip_forward_progress_m: float
    final_top_roll_progress_m: float | None
    final_top_contact_advance_m: float | None
    final_top_roll_remaining_m: float | None
    final_roll_phase: str
    final_continuation_target_world_xz_m: tuple[float, float] | None
    final_continuation_error_m: float | None
    final_rim: str | None
    final_contact_surface: str | None
    frames: int

    @property
    def candidate_theta_deg(self) -> float:
        return float(np.rad2deg(self.candidate_theta_rad))

    @property
    def final_theta_deg(self) -> float:
        return float(np.rad2deg(self.final_theta_rad))

    @property
    def final_beta_deg(self) -> float:
        return float(np.rad2deg(self.final_beta_rad))

    def as_dict(self) -> dict:
        return {
            "candidate_theta_rad": self.candidate_theta_rad,
            "candidate_theta_deg": self.candidate_theta_deg,
            "continuous_roll_up_success": self.continuous_roll_up_success,
            "failure_reason": self.failure_reason,
            "final_hip_x_m": self.final_hip_x_m,
            "final_theta_rad": self.final_theta_rad,
            "final_theta_deg": self.final_theta_deg,
            "final_beta_rad": self.final_beta_rad,
            "final_beta_deg": self.final_beta_deg,
            "final_hip_forward_progress_m": self.final_hip_forward_progress_m,
            "final_top_roll_progress_m": self.final_top_roll_progress_m,
            "final_top_contact_advance_m": self.final_top_contact_advance_m,
            "final_top_roll_remaining_m": self.final_top_roll_remaining_m,
            "final_roll_phase": self.final_roll_phase,
            "final_continuation_target_world_xz_m": self.final_continuation_target_world_xz_m,
            "final_continuation_error_m": self.final_continuation_error_m,
            "final_rim": self.final_rim,
            "final_contact_surface": self.final_contact_surface,
            "frames": self.frames,
        }


@dataclass(frozen=True)
class ForwardRollingSweepResult2D:
    """Forward-simulation results for all Step-4 theta candidates."""

    rows: tuple[ForwardRollingSweepRow2D, ...]
    simulations: tuple[ForwardRollingResult2D, ...]

    @property
    def feasible_rows(self) -> tuple[ForwardRollingSweepRow2D, ...]:
        return tuple(row for row in self.rows if row.continuous_roll_up_success)


def _forward_contact_phase(
    candidate,
    terrain: TerrainProfile2D,
    *,
    corner_tolerance_m: float,
) -> str | None:
    """Classify right-rim contact as front, corner, or top for logging."""

    kind = _right_rim_candidate_kind(candidate, terrain)
    if kind is None:
        return None
    obstacle = terrain.obstacle
    if obstacle is not None:
        top_z = terrain.ground_height_m + obstacle.height_m
        point = candidate.point_world_xz_m
        if (
            abs(point[0] - obstacle.x_min_m) <= corner_tolerance_m
            and abs(point[1] - top_z) <= corner_tolerance_m
        ):
            return "corner"
    return kind


def _forward_roll_phase(
    *,
    accepted: bool,
    surface: str | None,
    contact_phase: str | None,
    top_roll_complete: bool,
) -> str:
    """Return the motion-primitive phase, separate from terrain geometry."""

    if not accepted:
        return "FAILED"
    if surface == "top":
        return "TOP_ROLL_COMPLETE" if top_roll_complete else "TOP_ROLL"
    if contact_phase == "corner":
        return "LEADING_CORNER_TRANSITION"
    return "FRONT_FACE_CONTACT"


def _local_search_values(center: float, window: float, step: float) -> tuple[float, ...]:
    """Return deterministic, center-first values for a local brute-force search."""

    if window < 0.0 or step <= 0.0:
        raise ValueError("local search window must be non-negative and step positive.")
    if np.isclose(window, 0.0, atol=1e-15):
        return (center,)
    count = int(np.floor(window / step + 1e-12))
    offsets = [0.0]
    for index in range(1, count + 1):
        offsets.extend((index * step, -index * step))
    return tuple(center + offset for offset in offsets)


def _select_continuation_candidate(
    result: ContactQueryResult2D,
    terrain: TerrainProfile2D,
    previous_surface: str | None,
    target_world_xz_m: np.ndarray,
    preferred_kind: str,
    previous_sample_index: int | None,
) -> tuple[object | None, str | None, float | None]:
    """Select the right-rim candidate nearest to a continuation target.

    The query still returns every candidate.  This helper only adds the local
    motion-primitive selection rule: keep the physical right rim, prefer the
    requested front/top surface, and stay near the previous rim sample and
    target point.
    """

    target = np.asarray(target_world_xz_m, dtype=float)
    if target.shape != (2,) or not np.all(np.isfinite(target)):
        raise ValueError("target_world_xz_m must be a finite [x, z] point.")
    options = []
    for candidate in result.candidates:
        kind = _right_rim_candidate_kind(candidate, terrain)
        if kind != preferred_kind:
            continue
        if previous_surface == "top" and kind != "top":
            continue
        point_error = float(np.linalg.norm(candidate.point_world_xz_m - target))
        sample_error = (
            0
            if previous_sample_index is None
            else abs(candidate.sample_index - previous_sample_index)
        )
        score = (
            point_error,
            sample_error,
            candidate.surface_distance_m,
            -candidate.edge_margin_rad,
            candidate.sample_index,
        )
        options.append((score, candidate, kind, point_error))
    if not options:
        return None, None, None
    _, candidate, kind, point_error = min(options, key=lambda item: item[0])
    return candidate, kind, point_error


def _continuation_target(
    previous_surface: str,
    previous_contact_point: np.ndarray,
    terrain: TerrainProfile2D,
    *,
    corner_tolerance_m: float,
    contact_tolerance_m: float,
    collision_tolerance_m: float,
    top_contact_step_m: float,
) -> tuple[np.ndarray, str]:
    """Return the next world contact target and requested terrain kind.

    The rectangular obstacle has a vertical front and a horizontal top.  A
    front contact keeps the previous z anchor; once it is within the leading
    corner tolerance, the target moves to a small clearance inside the top
    span.  Top contact then advances along +x by ``top_contact_step_m``.
    """

    obstacle = terrain.obstacle
    if obstacle is None:
        raise ValueError("contact continuation requires one rectangular obstacle.")
    top_z = terrain.ground_height_m + obstacle.height_m
    point = np.asarray(previous_contact_point, dtype=float)
    if previous_surface == "top":
        return np.array([point[0] + top_contact_step_m, top_z], dtype=float), "top"
    if point[1] >= top_z - corner_tolerance_m:
        top_entry_x = obstacle.x_min_m + max(contact_tolerance_m, collision_tolerance_m)
        return np.array([top_entry_x, top_z], dtype=float), "top"
    front_clearance = 0.5 * contact_tolerance_m
    return np.array([obstacle.x_min_m - front_clearance, point[1]], dtype=float), "front"


def _solve_contact_continuation_pose_2d(
    previous_theta: float,
    previous_beta: float,
    current_hip_x: float,
    hip_z: float,
    sample_index: int,
    target_world_xz_m: np.ndarray,
    *,
    theta_limit_rad: float,
    beta_limit_rad: float,
    iterations: int,
    position_tolerance_m: float,
    derivative_step_rad: float,
    scene_kwargs: dict,
) -> tuple[float, float, float] | None:
    """Solve a local two-angle pose continuation for one rim sample.

    This is a small geometric continuation solve, not a global optimizer.  It
    uses the existing leg geometry model to finite-difference the selected
    sample position, then leaves final contact/collision acceptance to the
    terrain-aware query in the caller.
    """

    if iterations <= 0 or position_tolerance_m < 0.0 or derivative_step_rad <= 0.0:
        raise ValueError("continuation solver settings are invalid.")
    if theta_limit_rad < 0.0 or beta_limit_rad < 0.0:
        raise ValueError("continuation angle limits must be non-negative.")
    target = np.asarray(target_world_xz_m, dtype=float)
    if target.shape != (2,) or not np.all(np.isfinite(target)):
        raise ValueError("target_world_xz_m must be a finite [x, z] point.")

    previous = np.array([previous_theta, previous_beta], dtype=float)
    pose = previous.copy()

    def sample_point(pose_value: np.ndarray) -> np.ndarray:
        scene = build_single_leg_rolling_scene_2d(
            pose_value[0],
            pose_value[1],
            current_hip_x,
            hip_z,
            **scene_kwargs,
        )
        if sample_index >= len(scene.geometry.points_world_xz_m):
            raise IndexError("continuation sample index is outside the current geometry.")
        return np.asarray(scene.geometry.points_world_xz_m[sample_index], dtype=float)

    def residual(pose_value: np.ndarray) -> np.ndarray:
        return sample_point(pose_value) - target

    for _ in range(iterations):
        current_residual = residual(pose)
        error = float(np.linalg.norm(current_residual))
        if error <= position_tolerance_m:
            return float(pose[0]), float(pose[1]), error

        jacobian = np.empty((2, 2), dtype=float)
        for column in range(2):
            perturbed = pose.copy()
            perturbed[column] += derivative_step_rad
            jacobian[:, column] = (residual(perturbed) - current_residual) / derivative_step_rad
        try:
            delta = np.linalg.solve(jacobian, -current_residual)
        except np.linalg.LinAlgError:
            return None
        if not np.all(np.isfinite(delta)):
            return None

        scale = max(
            1.0,
            abs(delta[0]) / max(theta_limit_rad, 1e-12),
            abs(delta[1]) / max(beta_limit_rad, 1e-12),
        )
        delta = delta / scale
        improved = False
        for damping in (1.0, 0.5, 0.25, 0.125):
            trial = pose + damping * delta
            if (
                abs(trial[0] - previous[0]) > theta_limit_rad + 1e-12
                or abs(trial[1] - previous[1]) > beta_limit_rad + 1e-12
            ):
                continue
            trial_error = float(np.linalg.norm(residual(trial)))
            if trial_error < error:
                pose = trial
                improved = True
                break
        if not improved:
            break

    final_error = float(np.linalg.norm(residual(pose)))
    if final_error <= position_tolerance_m:
        return float(pose[0]), float(pose[1]), final_error
    return None


def _ordered_nearby_sample_indices(
    previous_sample_index: int,
    sample_count: int,
    search_window: int,
) -> tuple[int, ...]:
    """Return sampled rim indices from the previous sample outwards."""

    if previous_sample_index < 0 or previous_sample_index >= sample_count:
        raise ValueError("previous_sample_index is outside the sampled geometry.")
    if search_window < 0:
        raise ValueError("search_window must be non-negative.")
    indices = []
    for offset in range(search_window + 1):
        offsets = (0,) if offset == 0 else (-offset, offset)
        for signed_offset in offsets:
            index = previous_sample_index + signed_offset
            if 0 <= index < sample_count and index not in indices:
                indices.append(index)
    return tuple(indices)


def _top_contact_pose_options_2d(
    previous_theta: float,
    previous_beta: float,
    current_hip_x: float,
    hip_z: float,
    previous_sample_index: int,
    target_world_xz_m: np.ndarray,
    *,
    sample_search_window: int,
    candidate_limit: int,
    theta_limit_rad: float,
    beta_limit_rad: float,
    iterations: int,
    position_tolerance_m: float,
    derivative_step_rad: float,
    scene_kwargs: dict,
) -> tuple[tuple[float, float, float, int], ...]:
    """Propose continuous theta/beta poses for a top-contact target.

    The existing sampled leg geometry is used for every local solve.  Each
    nearby rim sample is treated as a possible continuation of the same
    physical right rim; the caller still runs the complete terrain-aware
    query before accepting any returned pose.
    """

    if candidate_limit <= 0:
        raise ValueError("candidate_limit must be positive.")
    sample_count = int(3 * scene_kwargs["arc_samples"])
    options = []
    for sample_index in _ordered_nearby_sample_indices(
        previous_sample_index,
        sample_count,
        sample_search_window,
    ):
        solved = _solve_contact_continuation_pose_2d(
            previous_theta,
            previous_beta,
            current_hip_x,
            hip_z,
            sample_index,
            target_world_xz_m,
            theta_limit_rad=theta_limit_rad,
            beta_limit_rad=beta_limit_rad,
            iterations=iterations,
            position_tolerance_m=position_tolerance_m,
            derivative_step_rad=derivative_step_rad,
            scene_kwargs=scene_kwargs,
        )
        if solved is None:
            continue
        theta, beta, error = solved
        options.append((theta, beta, error, sample_index))

    options.sort(
        key=lambda item: (
            abs(item[3] - previous_sample_index),
            item[2],
            abs(item[0] - previous_theta),
            abs(item[1] - previous_beta),
        )
    )
    return tuple(options[:candidate_limit])


def _forward_candidate_is_legal(
    query_result: ContactQueryResult2D,
    candidate,
    terrain: TerrainProfile2D,
) -> bool:
    """Apply the Step3 contact/collision acceptance rules to one pose."""

    if query_result.collision or candidate is None:
        return False
    if any(
        item.rim is not RimId.RIGHT
        and item.terrain_surface_id != terrain.ground_surface_id
        for item in query_result.candidates
    ):
        return False
    return _right_rim_candidate_kind(candidate, terrain) is not None


def _run_pose_ik_forward_right_rim_roll_up_2d(
    candidate_theta_rad: float,
    initial_beta_rad: float,
    hip_x_m: float,
    hip_z_m: float,
    *,
    dx_m: float = 0.005,
    max_forward_steps: int = 30,
    beta_step_rad: float = -np.deg2rad(1.0),
    theta_search_window_rad: float = np.deg2rad(1.0),
    theta_search_step_rad: float = np.deg2rad(1.0),
    beta_search_window_rad: float = np.deg2rad(2.0),
    beta_search_step_rad: float = np.deg2rad(1.0),
    gamma_rad: float = 0.0,
    ground_height_m: float = 0.0,
    obstacle_x_start_m: float = 0.10,
    obstacle_width_m: float = 0.60,
    obstacle_height_m: float = 0.10,
    obstacle_id: str = "day6_7_step4_5_obstacle",
    arc_samples: int = 241,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
    corner_tolerance_m: float = 2e-3,
    top_roll_distance_m: float = 0.02,
    leading_edge_tolerance_m: float = 1e-6,
    top_contact_step_ratio: float = 1.0,
    contact_continuation_iterations: int = 8,
    contact_continuation_tolerance_m: float = 2e-4,
    contact_continuation_derivative_step_rad: float = 1e-4,
    top_contact_sample_search_window: int = 12,
    top_contact_candidate_limit: int = 8,
) -> ForwardRollingResult2D:
    """Simulate forward hip motion with local theta/beta contact search.

    Frame zero validates the Step-4 candidate's initial pose.  Subsequent
    frames advance ``hip_x`` by ``dx_m`` and brute-force poses around the
    previous accepted ``(theta, beta)``.  A pose is accepted only if the
    existing terrain-aware query reports a legal right-rim contact, no
    collision, no other obstacle-rim contact, and a continuous parameter
    update.  Front/corner poses first attempt a local contact-continuation
    solve around the previous right-rim sample.  During top contact, the
    next world-frame right-rim target is solved explicitly for nearby sampled
    rim points; the terrain-aware query remains the final acceptance gate.
    Once top contact is first reached, the simulation continues until
    the right-rim contact point advances along the horizontal obstacle top by
    ``top_roll_distance_m`` while retaining legal top contact.  Hip position is
    still the forward-motion input and is logged separately as
    ``hip_forward_progress_m``.
    """

    candidate_theta = _finite_scalar(candidate_theta_rad, "candidate_theta_rad")
    initial_beta = _finite_scalar(initial_beta_rad, "initial_beta_rad")
    hip_x = _finite_scalar(hip_x_m, "hip_x_m")
    _finite_scalar(hip_z_m, "hip_z_m")
    dx = _finite_scalar(dx_m, "dx_m")
    if dx <= 0.0:
        raise ValueError("dx_m must be positive for +x forward simulation.")
    if max_forward_steps < 0:
        raise ValueError("max_forward_steps must be non-negative.")
    corner_tolerance = _finite_scalar(corner_tolerance_m, "corner_tolerance_m")
    leading_edge_tolerance = _finite_scalar(
        leading_edge_tolerance_m, "leading_edge_tolerance_m"
    )
    top_roll_distance = _finite_scalar(top_roll_distance_m, "top_roll_distance_m")
    top_step_ratio = _finite_scalar(top_contact_step_ratio, "top_contact_step_ratio")
    continuation_tolerance = _finite_scalar(
        contact_continuation_tolerance_m, "contact_continuation_tolerance_m"
    )
    continuation_derivative_step = _finite_scalar(
        contact_continuation_derivative_step_rad,
        "contact_continuation_derivative_step_rad",
    )
    if (
        corner_tolerance < 0.0
        or leading_edge_tolerance < 0.0
        or top_roll_distance < 0.0
        or top_step_ratio <= 0.0
        or continuation_tolerance < 0.0
        or continuation_derivative_step <= 0.0
        or contact_continuation_iterations <= 0
        or top_contact_sample_search_window < 0
        or top_contact_candidate_limit <= 0
    ):
        raise ValueError("invalid contact-continuation or top-roll settings.")

    obstacle_x_start = _finite_scalar(obstacle_x_start_m, "obstacle_x_start_m")
    frames = []
    previous_theta = candidate_theta
    previous_beta = initial_beta
    previous_surface = None
    previous_phase = None
    previous_contact_point = None
    previous_sample_index = None
    top_entry_contact_x = None
    failure_reason = None
    success = False

    scene_kwargs = {
        "gamma_rad": gamma_rad,
        "ground_height_m": ground_height_m,
        "obstacle_x_start_m": obstacle_x_start_m,
        "obstacle_width_m": obstacle_width_m,
        "obstacle_height_m": obstacle_height_m,
        "obstacle_id": obstacle_id,
        "arc_samples": arc_samples,
    }

    for step in range(max_forward_steps + 1):
        current_hip_x = hip_x + step * dx
        continuation_target = None
        preferred_kind = None
        pose_options = []
        top_ik_attempted = False
        top_ik_solution_count = 0
        if step == 0:
            pose_options.append((candidate_theta, initial_beta, "initial", None))
        else:
            if previous_surface is not None and previous_contact_point is not None:
                continuation_scene = build_single_leg_rolling_scene_2d(
                    previous_theta,
                    previous_beta,
                    current_hip_x,
                    hip_z_m,
                    **scene_kwargs,
                )
                continuation_target, preferred_kind = _continuation_target(
                    previous_surface,
                    previous_contact_point,
                    continuation_scene.terrain,
                    corner_tolerance_m=corner_tolerance,
                    contact_tolerance_m=contact_tolerance_m,
                    collision_tolerance_m=collision_tolerance_m,
                    top_contact_step_m=dx * top_step_ratio,
                )
                if previous_sample_index is not None:
                    solved_pose = _solve_contact_continuation_pose_2d(
                        previous_theta,
                        previous_beta,
                        current_hip_x,
                        hip_z_m,
                        previous_sample_index,
                        continuation_target,
                        theta_limit_rad=theta_search_window_rad,
                        beta_limit_rad=beta_search_window_rad,
                        iterations=contact_continuation_iterations,
                        position_tolerance_m=continuation_tolerance,
                        derivative_step_rad=continuation_derivative_step,
                        scene_kwargs=scene_kwargs,
                    )
                    if solved_pose is not None and preferred_kind != "top":
                        pose_options.append(
                            (solved_pose[0], solved_pose[1], "continuation", None)
                        )
                if preferred_kind == "top" and previous_sample_index is not None:
                    top_ik_attempted = True
                    top_options = _top_contact_pose_options_2d(
                        previous_theta,
                        previous_beta,
                        current_hip_x,
                        hip_z_m,
                        previous_sample_index,
                        continuation_target,
                        sample_search_window=top_contact_sample_search_window,
                        candidate_limit=top_contact_candidate_limit,
                        theta_limit_rad=theta_search_window_rad,
                        beta_limit_rad=beta_search_window_rad,
                        iterations=contact_continuation_iterations,
                        position_tolerance_m=continuation_tolerance,
                        derivative_step_rad=continuation_derivative_step,
                        scene_kwargs=scene_kwargs,
                    )
                    top_ik_solution_count = len(top_options)
                    pose_options.extend(
                        (theta, beta, "top_contact_ik", sample_index)
                        for theta, beta, _, sample_index in top_options
                    )
            if preferred_kind != "top":
                pose_options.extend(
                    (theta, beta, "local_grid", None)
                    for theta in _local_search_values(
                        previous_theta, theta_search_window_rad, theta_search_step_rad
                    )
                    for beta in _local_search_values(
                        previous_beta, beta_search_window_rad, beta_search_step_rad
                    )
                )

        accepted_option = None
        rejected_query_result = None
        rejected_scene = None
        top_ik_collision_seen = False
        for theta, beta, source, expected_sample_index in pose_options:
            scene = build_single_leg_rolling_scene_2d(
                theta,
                beta,
                current_hip_x,
                hip_z_m,
                gamma_rad=gamma_rad,
                ground_height_m=ground_height_m,
                obstacle_x_start_m=obstacle_x_start_m,
                obstacle_width_m=obstacle_width_m,
                obstacle_height_m=obstacle_height_m,
                obstacle_id=obstacle_id,
                arc_samples=arc_samples,
            )
            query_result = query_single_leg_rolling_scene_2d(
                scene,
                contact_tolerance_m=contact_tolerance_m,
                collision_tolerance_m=collision_tolerance_m,
            )
            if continuation_target is None or preferred_kind is None:
                candidate, surface = _select_active_right_rim_candidate(
                    query_result,
                    scene.terrain,
                    previous_surface,
                )
                continuation_error = None
            elif source == "top_contact_ik":
                top_candidates = [
                    item
                    for item in query_result.candidates
                    if _right_rim_candidate_kind(item, scene.terrain) == "top"
                    and (
                        expected_sample_index is None
                        or abs(item.sample_index - expected_sample_index) <= 1
                    )
                ]
                if top_candidates:
                    candidate = min(
                        top_candidates,
                        key=lambda item: (
                            float(np.linalg.norm(item.point_world_xz_m - continuation_target)),
                            abs(item.sample_index - expected_sample_index),
                        ),
                    )
                    surface = "top"
                    continuation_error = float(
                        np.linalg.norm(candidate.point_world_xz_m - continuation_target)
                    )
                else:
                    candidate, surface, continuation_error = None, None, None
            else:
                candidate, surface, continuation_error = _select_continuation_candidate(
                    query_result,
                    scene.terrain,
                    previous_surface,
                    continuation_target,
                    preferred_kind,
                    previous_sample_index,
                )
            if not _forward_candidate_is_legal(query_result, candidate, scene.terrain):
                if source == "top_contact_ik" and query_result.collision:
                    top_ik_collision_seen = True
                rejected_query_result = query_result
                rejected_scene = scene
                continue
            phase = _forward_contact_phase(
                candidate,
                scene.terrain,
                corner_tolerance_m=corner_tolerance,
            )
            kind = _right_rim_candidate_kind(candidate, scene.terrain)
            if previous_surface is None and kind != "front":
                continue
            if previous_surface == "top" and kind != "top":
                continue
            if previous_phase == "top" and phase not in {"top", "corner"}:
                continue
            # The local windows are also the continuity bound.  This explicit
            # check keeps the contract clear if search construction changes.
            if abs(theta - previous_theta) > theta_search_window_rad + 1e-12:
                continue
            if abs(beta - previous_beta) > beta_search_window_rad + 1e-12:
                continue
            score = (
                (0.0 if continuation_error is None else continuation_error / max(continuation_tolerance, 1e-12))
                + abs(theta - previous_theta) / max(theta_search_step_rad, 1e-12)
                + abs(beta - previous_beta) / max(beta_search_step_rad, 1e-12)
                + (0.25 if previous_surface == "front" and kind == "top" else 0.0)
            )
            option = (
                score,
                theta,
                beta,
                candidate,
                surface,
                phase,
                scene,
                query_result,
                continuation_error,
            )
            if accepted_option is None or option[0] < accepted_option[0]:
                accepted_option = option
                if source == "top_contact_ik":
                    # Nearby samples are ordered by continuation distance.
                    # Once one has passed the full query, keeping searching
                    # only adds cost and can select a less continuous rim
                    # sample for the same world target.
                    break

        if accepted_option is None:
            if step == 0:
                failure_reason = "INITIAL_CONDITION_INVALID"
            elif top_ik_attempted and top_ik_solution_count == 0:
                failure_reason = "NO_TOP_CONTACT_IK_SOLUTION"
            elif top_ik_attempted and top_ik_collision_seen:
                failure_reason = "TOP_CONTACT_IK_COLLISION_BLOCKED"
            elif rejected_query_result is not None and rejected_query_result.collision:
                failure_reason = "NO_LEGAL_CONTINUOUS_POSE_DUE_TO_COLLISION"
            else:
                failure_reason = "NO_LEGAL_CONTINUOUS_CONTACT_POSE"
            scene = rejected_scene
            query_result = rejected_query_result
            if scene is None or query_result is None:
                # This only occurs for an empty option set, which validation
                # prevents, but keeps the returned record total and explicit.
                scene = build_single_leg_rolling_scene_2d(
                    previous_theta,
                    previous_beta,
                    current_hip_x,
                    hip_z_m,
                    gamma_rad=gamma_rad,
                    ground_height_m=ground_height_m,
                    obstacle_x_start_m=obstacle_x_start_m,
                    obstacle_width_m=obstacle_width_m,
                    obstacle_height_m=obstacle_height_m,
                    obstacle_id=obstacle_id,
                    arc_samples=arc_samples,
                )
                query_result = query_single_leg_rolling_scene_2d(
                    scene,
                    contact_tolerance_m=contact_tolerance_m,
                    collision_tolerance_m=collision_tolerance_m,
                )
            frames.append(
                ForwardRollingFrame2D(
                    step=step,
                    hip_x_m=current_hip_x,
                    hip_forward_progress_m=current_hip_x - hip_x,
                    theta_rad=previous_theta,
                    beta_rad=previous_beta,
                    active_rim=None,
                    alpha_rad=None,
                    contact_point_world_xz_m=None,
                    continuation_target_world_xz_m=(
                        None
                        if continuation_target is None
                        else tuple(float(value) for value in continuation_target)
                    ),
                    continuation_error_m=None,
                    leading_edge_clearance_m=None,
                    top_roll_progress_m=None,
                    top_contact_advance_m=None,
                    top_roll_complete=False,
                    top_roll_remaining_m=None,
                    roll_phase="FAILED",
                    terrain_surface_id=None,
                    contact_phase=None,
                    valid_contact=False,
                    collision=query_result.collision,
                    status=query_result.primary_status,
                    accepted=False,
                    failure_reason=failure_reason,
                    scene=scene,
                    query_result=query_result,
                )
            )
            break

        (
            _,
            theta,
            beta,
            candidate,
            surface,
            phase,
            scene,
            query_result,
            continuation_error,
        ) = accepted_option
        contact_point = (
            float(candidate.point_world_xz_m[0]),
            float(candidate.point_world_xz_m[1]),
        )
        leading_edge_clearance = contact_point[0] - obstacle_x_start
        if surface == "top" and top_entry_contact_x is None:
            top_entry_contact_x = contact_point[0]
        top_contact_advance = (
            None
            if top_entry_contact_x is None
            else contact_point[0] - top_entry_contact_x
        )
        # The current obstacle top is horizontal, so its tangent direction is
        # +x. Keep both names: top_contact_advance_m is the raw geometric
        # measurement, while top_roll_progress_m is the completion metric.
        top_roll_progress = top_contact_advance
        top_roll_complete = (
            surface == "top"
            and top_roll_progress is not None
            and top_roll_progress >= top_roll_distance - leading_edge_tolerance
        )
        top_roll_remaining = (
            None
            if top_roll_progress is None
            else max(top_roll_distance - top_roll_progress, 0.0)
        )
        roll_phase = _forward_roll_phase(
            accepted=True,
            surface=surface,
            contact_phase=phase,
            top_roll_complete=top_roll_complete,
        )
        frame = ForwardRollingFrame2D(
            step=step,
            hip_x_m=current_hip_x,
            hip_forward_progress_m=current_hip_x - hip_x,
            theta_rad=float(theta),
            beta_rad=float(beta),
            active_rim=candidate.rim.value,
            alpha_rad=float(candidate.alpha_rad),
            contact_point_world_xz_m=contact_point,
            continuation_target_world_xz_m=(
                None
                if continuation_target is None
                else tuple(float(value) for value in continuation_target)
            ),
            continuation_error_m=continuation_error,
            leading_edge_clearance_m=leading_edge_clearance,
            top_roll_progress_m=top_roll_progress,
            top_contact_advance_m=top_contact_advance,
            top_roll_complete=top_roll_complete,
            top_roll_remaining_m=top_roll_remaining,
            roll_phase=roll_phase,
            terrain_surface_id=candidate.terrain_surface_id,
            contact_phase=phase,
            valid_contact=True,
            collision=False,
            status=candidate_status_2d(candidate, scene.terrain),
            accepted=True,
            failure_reason=None,
            scene=scene,
            query_result=query_result,
        )
        frames.append(frame)
        previous_theta = float(theta)
        previous_beta = float(beta)
        previous_surface = surface
        previous_phase = phase
        previous_contact_point = np.asarray(contact_point, dtype=float)
        previous_sample_index = candidate.sample_index
        if top_roll_complete:
            success = True
            break

    if not success and failure_reason is None:
        failure_reason = "MAX_FORWARD_STEPS_BEFORE_TOP_ROLL_COMPLETE"
    return ForwardRollingResult2D(
        candidate_theta_rad=candidate_theta,
        initial_beta_rad=initial_beta,
        dx_m=dx,
        top_roll_distance_m=top_roll_distance,
        frames=tuple(frames),
        success=success,
        failure_reason=None if success else failure_reason,
    )


def _translated_scene_with_sample_on_target_2d(
    template: SingleLegRollingScene2D,
    sample_index: int,
    target_world_xz_m,
) -> SingleLegRollingScene2D:
    """Translate an already sampled scene so one point reaches the target."""

    target = np.asarray(target_world_xz_m, dtype=float)
    hip_position = target - template.geometry.points_hip_xz_m[sample_index]
    hip_pose = HipPose2D(hip_position, pitch_world_hip_rad=0.0)
    return replace(
        template,
        hip_pose=hip_pose,
        geometry=replace(template.geometry, hip_pose=hip_pose),
    )


def _right_rim_sample_indices(geometry: SampledLegGeometry2D) -> tuple[int, ...]:
    return tuple(
        index
        for index, region in enumerate(geometry.contact_regions)
        if region == "right_rim"
    )


def _candidate_near_sample_and_target(
    result: ContactQueryResult2D,
    terrain: TerrainProfile2D,
    sample_index: int,
    target_world_xz_m,
    *,
    preferred_kind: str | None = None,
):
    target = np.asarray(target_world_xz_m, dtype=float)
    options = []
    for candidate in result.candidates:
        kind = _right_rim_candidate_kind(candidate, terrain)
        if kind is None or (preferred_kind is not None and kind != preferred_kind):
            continue
        options.append(
            (
                abs(candidate.sample_index - sample_index),
                float(np.linalg.norm(candidate.point_world_xz_m - target)),
                candidate.surface_distance_m,
                candidate,
            )
        )
    return None if not options else min(options, key=lambda item: item[:3])[-1]


def run_forward_right_rim_roll_up_2d(
    candidate_theta_rad: float,
    initial_beta_rad: float,
    hip_x_m: float,
    hip_z_m: float,
    *,
    dx_m: float = 0.005,
    max_forward_steps: int = 80,
    beta_step_rad: float = -np.deg2rad(5.0),
    theta_search_window_rad: float = np.deg2rad(2.0),
    theta_search_step_rad: float = np.deg2rad(1.0),
    beta_search_window_rad: float = np.deg2rad(2.0),
    beta_search_step_rad: float = np.deg2rad(1.0),
    gamma_rad: float = 0.0,
    ground_height_m: float = 0.0,
    obstacle_x_start_m: float = 0.10,
    obstacle_width_m: float = 0.60,
    obstacle_height_m: float = 0.10,
    obstacle_id: str = "day6_7_step4_5_obstacle",
    arc_samples: int = 241,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
    corner_tolerance_m: float = 2e-3,
    top_roll_distance_m: float = 0.02,
    leading_edge_tolerance_m: float = 1e-6,
    top_contact_step_ratio: float = 1.0,
    contact_continuation_iterations: int = 8,
    contact_continuation_tolerance_m: float = 2e-4,
    contact_continuation_derivative_step_rad: float = 1e-4,
    top_contact_sample_search_window: int = 12,
    top_contact_candidate_limit: int = 8,
    top_beta_step_rad: float = -np.deg2rad(1.0),
) -> ForwardRollingResult2D:
    """Contact-driven right-rim roll around the corner and along the top.

    Unlike the earlier pose-IK prototype, this routine does not prescribe
    equal increments for both hip x and contact x.  It first keeps a sampled
    right-rim point on the leading corner while changing beta; hip x/z are
    reconstructed from that contact constraint.  Once the right rim is the
    lowest collision-free support, a later right-rim sample must become the
    active support.  The average sampled rim arc length defines the matching
    world-frame contact advance (no slip); only then is the hip pose
    reconstructed. ``dx_m`` is a preferred arc increment, not a prescribed
    contact translation. Every accepted frame still passes the existing
    terrain-aware query.

    ``contact_continuation_*`` and top candidate-limit arguments remain in the
    public signature for notebook compatibility; this contact-driven version
    does not use the old fixed-hip Newton solve.
    """

    del contact_continuation_iterations
    del contact_continuation_derivative_step_rad
    del top_contact_candidate_limit
    candidate_theta = _finite_scalar(candidate_theta_rad, "candidate_theta_rad")
    initial_beta = _finite_scalar(initial_beta_rad, "initial_beta_rad")
    initial_hip_x = _finite_scalar(hip_x_m, "hip_x_m")
    initial_hip_z = _finite_scalar(hip_z_m, "hip_z_m")
    dx = _finite_scalar(dx_m, "dx_m")
    corner_beta_step = _finite_scalar(beta_step_rad, "beta_step_rad")
    top_beta_step = _finite_scalar(top_beta_step_rad, "top_beta_step_rad")
    top_roll_distance = _finite_scalar(top_roll_distance_m, "top_roll_distance_m")
    top_step_ratio = _finite_scalar(top_contact_step_ratio, "top_contact_step_ratio")
    continuation_tolerance = _finite_scalar(
        contact_continuation_tolerance_m, "contact_continuation_tolerance_m"
    )
    if dx <= 0.0:
        raise ValueError("dx_m must be positive for +x forward simulation.")
    if max_forward_steps < 0:
        raise ValueError("max_forward_steps must be non-negative.")
    if corner_beta_step >= 0.0 or top_beta_step >= 0.0:
        raise ValueError("corner and top beta steps must both be negative.")
    if top_roll_distance < 0.0 or top_step_ratio <= 0.0:
        raise ValueError("top-roll distance and step ratio are invalid.")
    if theta_search_window_rad < 0.0 or theta_search_step_rad <= 0.0:
        raise ValueError("theta search settings are invalid.")
    if beta_search_window_rad < 0.0 or beta_search_step_rad <= 0.0:
        raise ValueError("beta search settings are invalid.")
    if top_contact_sample_search_window < 0 or continuation_tolerance < 0.0:
        raise ValueError("contact continuation settings are invalid.")

    obstacle_x_start = _finite_scalar(obstacle_x_start_m, "obstacle_x_start_m")
    top_z = _finite_scalar(ground_height_m, "ground_height_m") + _finite_scalar(
        obstacle_height_m, "obstacle_height_m"
    )
    corner_target = np.array([obstacle_x_start, top_z], dtype=float)
    scene_kwargs = {
        "gamma_rad": gamma_rad,
        "ground_height_m": ground_height_m,
        "obstacle_x_start_m": obstacle_x_start_m,
        "obstacle_width_m": obstacle_width_m,
        "obstacle_height_m": obstacle_height_m,
        "obstacle_id": obstacle_id,
        "arc_samples": arc_samples,
    }
    frames: list[ForwardRollingFrame2D] = []

    def append_frame(
        scene,
        query_result,
        candidate,
        *,
        target,
        roll_phase,
        accepted=True,
        failure_reason=None,
        top_entry_x=None,
    ):
        contact_point = None
        surface = None
        contact_phase = None
        continuation_error = None
        alpha = None
        active_rim = None
        terrain_surface_id = None
        status = query_result.primary_status
        if candidate is not None:
            contact_point = tuple(float(value) for value in candidate.point_world_xz_m)
            surface = _right_rim_candidate_kind(candidate, scene.terrain)
            contact_phase = _forward_contact_phase(
                candidate, scene.terrain, corner_tolerance_m=corner_tolerance_m
            )
            continuation_error = float(
                np.linalg.norm(candidate.point_world_xz_m - np.asarray(target, dtype=float))
            )
            alpha = float(candidate.alpha_rad)
            active_rim = candidate.rim.value
            terrain_surface_id = candidate.terrain_surface_id
            status = candidate_status_2d(candidate, scene.terrain)
        clearance = None if contact_point is None else contact_point[0] - obstacle_x_start
        top_progress = (
            None
            if top_entry_x is None or contact_point is None
            else max(contact_point[0] - top_entry_x, 0.0)
        )
        complete = bool(
            accepted
            and surface == "top"
            and top_progress is not None
            and top_progress >= top_roll_distance - leading_edge_tolerance_m
        )
        remaining = (
            None if top_progress is None else max(top_roll_distance - top_progress, 0.0)
        )
        hip = scene.hip_pose.position_world_xz_m
        frames.append(
            ForwardRollingFrame2D(
                step=len(frames),
                hip_x_m=float(hip[0]),
                hip_forward_progress_m=float(hip[0] - initial_hip_x),
                theta_rad=float(scene.theta_rad),
                beta_rad=float(scene.beta_rad),
                active_rim=active_rim,
                alpha_rad=alpha,
                contact_point_world_xz_m=contact_point,
                continuation_target_world_xz_m=tuple(float(value) for value in target),
                continuation_error_m=continuation_error,
                leading_edge_clearance_m=clearance,
                top_roll_progress_m=top_progress,
                top_contact_advance_m=top_progress,
                top_roll_complete=complete,
                top_roll_remaining_m=remaining,
                roll_phase=roll_phase,
                terrain_surface_id=terrain_surface_id,
                contact_phase=contact_phase,
                valid_contact=bool(accepted and candidate is not None),
                collision=query_result.collision,
                status=status,
                accepted=accepted,
                failure_reason=failure_reason,
                scene=scene,
                query_result=query_result,
            )
        )
        return complete

    initial_scene = build_single_leg_rolling_scene_2d(
        candidate_theta,
        initial_beta,
        initial_hip_x,
        initial_hip_z,
        **scene_kwargs,
    )
    initial_query = query_single_leg_rolling_scene_2d(
        initial_scene,
        contact_tolerance_m=contact_tolerance_m,
        collision_tolerance_m=collision_tolerance_m,
    )
    initial_candidate, initial_surface = _select_active_right_rim_candidate(
        initial_query, initial_scene.terrain, None
    )
    if (
        initial_surface != "front"
        or not _forward_candidate_is_legal(
            initial_query, initial_candidate, initial_scene.terrain
        )
    ):
        append_frame(
            initial_scene,
            initial_query,
            initial_candidate,
            target=corner_target,
            roll_phase="FAILED",
            accepted=False,
            failure_reason="INITIAL_CONDITION_INVALID",
        )
        return ForwardRollingResult2D(
            candidate_theta,
            initial_beta,
            dx,
            top_roll_distance,
            tuple(frames),
            False,
            "INITIAL_CONDITION_INVALID",
        )

    append_frame(
        initial_scene,
        initial_query,
        initial_candidate,
        target=corner_target,
        roll_phase="FRONT_FACE_CONTACT",
    )
    previous_theta = candidate_theta
    previous_beta = initial_beta
    previous_sample = initial_candidate.sample_index
    support_scene = None
    support_query = None
    support_candidate = None

    # Pivot the same right rim around the sharp leading corner.  The target is
    # fixed; hip x and z therefore emerge from beta/sample continuation.
    while len(frames) <= max_forward_steps:
        desired_beta = previous_beta + corner_beta_step
        options = []
        # Corner pivot keeps the Step-4 candidate theta fixed.  Beta and the
        # active sample provide the roll around the sharp edge.
        for theta in (previous_theta,):
            template = build_single_leg_rolling_scene_2d(
                theta, desired_beta, 0.0, 0.0, **scene_kwargs
            )
            right_indices = _right_rim_sample_indices(template.geometry)
            nearby = sorted(
                right_indices,
                key=lambda index: abs(index - previous_sample),
            )[:3]
            global_min_z = float(np.min(template.geometry.points_hip_xz_m[:, 1]))
            lowest_right = min(
                right_indices,
                key=lambda index: template.geometry.points_hip_xz_m[index, 1],
            )
            if lowest_right not in nearby:
                nearby.append(lowest_right)
            for sample_index in nearby:
                scene = _translated_scene_with_sample_on_target_2d(
                    template,
                    sample_index,
                    corner_target,
                )
                query_result = query_single_leg_rolling_scene_2d(
                    scene,
                    contact_tolerance_m=contact_tolerance_m,
                    collision_tolerance_m=collision_tolerance_m,
                )
                support_ready = bool(
                    template.geometry.points_hip_xz_m[sample_index, 1]
                    <= global_min_z + 1e-8
                )
                candidate = _candidate_near_sample_and_target(
                    query_result,
                    scene.terrain,
                    sample_index,
                    corner_target,
                    preferred_kind="top" if support_ready else None,
                )
                if not _forward_candidate_is_legal(
                    query_result, candidate, scene.terrain
                ):
                    continue
                hip = scene.hip_pose.position_world_xz_m
                score = (
                    0 if support_ready else 1,
                    abs(sample_index - previous_sample),
                    abs(theta - previous_theta),
                    -float(hip[0]),
                )
                options.append(
                    (
                        score,
                        theta,
                        desired_beta,
                        sample_index,
                        scene,
                        query_result,
                        candidate,
                        support_ready,
                    )
                )
        if not options:
            reason = "NO_LEGAL_CORNER_CONTACT_CONTINUATION"
            append_frame(
                frames[-1].scene,
                frames[-1].query_result,
                None,
                target=corner_target,
                roll_phase="FAILED",
                accepted=False,
                failure_reason=reason,
            )
            return ForwardRollingResult2D(
                candidate_theta, initial_beta, dx, top_roll_distance,
                tuple(frames), False, reason
            )
        (
            _, previous_theta, previous_beta, previous_sample,
            scene, query_result, candidate, support_ready,
        ) = min(options, key=lambda item: item[0])
        append_frame(
            scene,
            query_result,
            candidate,
            target=corner_target,
            roll_phase=("LEADING_CORNER_TO_TOP" if support_ready else "LEADING_CORNER_TRANSITION"),
            top_entry_x=obstacle_x_start if support_ready else None,
        )
        if support_ready:
            support_scene, support_query, support_candidate = scene, query_result, candidate
            break

    if support_scene is None:
        reason = "MAX_FORWARD_STEPS_BEFORE_TOP_SUPPORT"
        return ForwardRollingResult2D(
            candidate_theta, initial_beta, dx, top_roll_distance,
            tuple(frames), False, reason
        )

    top_entry_x = obstacle_x_start
    if top_roll_distance <= leading_edge_tolerance_m:
        last = frames.pop()
        append_frame(
            last.scene,
            last.query_result,
            support_candidate,
            target=corner_target,
            roll_phase="TOP_ROLL_COMPLETE",
            top_entry_x=top_entry_x,
        )
        return ForwardRollingResult2D(
            candidate_theta, initial_beta, dx, top_roll_distance,
            tuple(frames), True, None
        )

    # Roll without slip along the top.  The next world contact displacement is
    # not prescribed directly: a later material rim sample must become the
    # lowest support, and its rim arc length from the previous sample defines
    # the +x displacement.  Hip x/z are then reconstructed from that contact.
    while len(frames) <= max_forward_steps:
        previous_progress = frames[-1].top_roll_progress_m or 0.0
        previous_contact_x = frames[-1].contact_point_world_xz_m[0]
        failure_target = np.array([previous_contact_x + dx, top_z], dtype=float)
        desired_beta = previous_beta + top_beta_step
        angle_options = []
        theta_values = _local_search_values(
            previous_theta, theta_search_window_rad, theta_search_step_rad
        )
        beta_values = [desired_beta]
        beta_search_count = int(
            np.floor(beta_search_window_rad / beta_search_step_rad + 1e-12)
        )
        for index in range(1, beta_search_count + 1):
            beta_values.append(desired_beta + index * top_beta_step)
        if not any(np.isclose(previous_beta, item, atol=1e-12) for item in beta_values):
            beta_values.append(previous_beta)
        for offset in _local_search_values(0.0, beta_search_window_rad, beta_search_step_rad):
            value = desired_beta + offset
            if not any(np.isclose(value, item, atol=1e-12) for item in beta_values):
                beta_values.append(value)
        angle_pairs = [(previous_theta, beta) for beta in beta_values]
        angle_pairs.extend(
            (theta, beta)
            for theta in theta_values
            if theta != previous_theta
            for beta in beta_values
        )
        for theta, beta in angle_pairs:
            template = build_single_leg_rolling_scene_2d(
                theta, beta, 0.0, 0.0, **scene_kwargs
            )
            right_indices = _right_rim_sample_indices(template.geometry)
            sample_index = min(
                right_indices,
                key=lambda index: template.geometry.points_hip_xz_m[index, 1],
            )
            global_min_z = float(np.min(template.geometry.points_hip_xz_m[:, 1]))
            if (
                sample_index <= previous_sample
                or template.geometry.points_hip_xz_m[sample_index, 1]
                > global_min_z + 1e-8
            ):
                continue
            sample_slice = slice(previous_sample, sample_index + 1)
            previous_arc_points = frames[-1].scene.geometry.points_hip_xz_m[
                sample_slice
            ]
            next_arc_points = template.geometry.points_hip_xz_m[sample_slice]
            previous_arc_length = float(
                np.sum(np.linalg.norm(np.diff(previous_arc_points, axis=0), axis=1))
            )
            next_arc_length = float(
                np.sum(np.linalg.norm(np.diff(next_arc_points, axis=0), axis=1))
            )
            rolling_arc_increment = 0.5 * (previous_arc_length + next_arc_length)
            if rolling_arc_increment <= 1e-12:
                continue
            next_progress = previous_progress + rolling_arc_increment
            target = np.array(
                [previous_contact_x + rolling_arc_increment, top_z], dtype=float
            )
            scene = _translated_scene_with_sample_on_target_2d(
                template, sample_index, target
            )
            query_result = query_single_leg_rolling_scene_2d(
                scene,
                contact_tolerance_m=contact_tolerance_m,
                collision_tolerance_m=collision_tolerance_m,
            )
            candidate = _candidate_near_sample_and_target(
                query_result,
                scene.terrain,
                sample_index,
                target,
                preferred_kind="top",
            )
            if (
                not _forward_candidate_is_legal(query_result, candidate, scene.terrain)
                or abs(candidate.sample_index - sample_index) > 1
            ):
                continue
            error = float(np.linalg.norm(candidate.point_world_xz_m - target))
            if error > max(continuation_tolerance, contact_tolerance_m):
                continue
            score = (
                abs(rolling_arc_increment - dx * top_step_ratio),
                abs(beta - desired_beta) / max(beta_search_step_rad, 1e-12),
                abs(theta - previous_theta) / max(theta_search_step_rad, 1e-12),
                abs(sample_index - previous_sample),
                error,
            )
            angle_options.append(
                (
                    score,
                    theta,
                    beta,
                    sample_index,
                    scene,
                    query_result,
                    candidate,
                    target,
                    next_progress,
                )
            )
            # Pair ordering already prefers the smallest continuous beta
            # update.  The first legal sample advance is the local no-slip
            # continuation; do not evaluate visibly different alternatives.
            break
        if not angle_options:
            reason = "NO_LEGAL_TOP_CONTACT_CONTINUATION"
            append_frame(
                frames[-1].scene,
                frames[-1].query_result,
                None,
                target=failure_target,
                roll_phase="FAILED",
                accepted=False,
                failure_reason=reason,
                top_entry_x=top_entry_x,
            )
            return ForwardRollingResult2D(
                candidate_theta, initial_beta, dx, top_roll_distance,
                tuple(frames), False, reason
            )
        (
            _, previous_theta, previous_beta, previous_sample,
            scene, query_result, candidate, target, next_progress,
        ) = min(angle_options, key=lambda item: item[0])
        complete = append_frame(
            scene,
            query_result,
            candidate,
            target=target,
            roll_phase=(
                "TOP_ROLL_COMPLETE"
                if next_progress >= top_roll_distance - leading_edge_tolerance_m
                else "TOP_ROLL"
            ),
            top_entry_x=top_entry_x,
        )
        if complete:
            return ForwardRollingResult2D(
                candidate_theta, initial_beta, dx, top_roll_distance,
                tuple(frames), True, None
            )

    reason = "MAX_FORWARD_STEPS_BEFORE_TOP_ROLL_COMPLETE"
    return ForwardRollingResult2D(
        candidate_theta, initial_beta, dx, top_roll_distance,
        tuple(frames), False, reason
    )


def _frame_right_rim_sample_index(frame: ForwardRollingFrame2D) -> int | None:
    if frame.alpha_rad is None:
        return None
    right_indices = _right_rim_sample_indices(frame.scene.geometry)
    if not right_indices:
        return None
    return min(
        right_indices,
        key=lambda index: abs(
            frame.scene.geometry.alpha_rad[index] - frame.alpha_rad
        ),
    )


def _sampled_rim_arc_length_between_frames(
    first: ForwardRollingFrame2D,
    second: ForwardRollingFrame2D,
) -> float | None:
    first_index = _frame_right_rim_sample_index(first)
    second_index = _frame_right_rim_sample_index(second)
    if first_index is None or second_index is None or second_index <= first_index:
        return None
    sample_slice = slice(first_index, second_index + 1)
    first_points = first.scene.geometry.points_hip_xz_m[sample_slice]
    second_points = second.scene.geometry.points_hip_xz_m[sample_slice]
    first_length = float(
        np.sum(np.linalg.norm(np.diff(first_points, axis=0), axis=1))
    )
    second_length = float(
        np.sum(np.linalg.norm(np.diff(second_points, axis=0), axis=1))
    )
    return 0.5 * (first_length + second_length)


def evaluate_right_rim_retract_readiness_2d(
    rolling_result: ForwardRollingResult2D,
    *,
    retract_theta_target_rad: float = np.deg2rad(17.0),
    preview_step_rad: float = np.deg2rad(1.0),
    preview_steps: int = 5,
    stable_window_frames: int = 4,
    safety_margin_m: float | None = None,
    beta_search_window_rad: float = np.deg2rad(5.0),
    beta_search_step_rad: float = np.deg2rad(1.0),
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
) -> RetractReadinessResult2D:
    """Evaluate the top-roll guard and a short retract-to-17 preview.

    Preview poses keep the final right-rim world contact fixed on the obstacle
    top while theta moves toward the requested target.  Beta and hip x/z are
    reconstructed locally, and every preview pose passes through the existing
    terrain-aware query.  The returned poses are diagnostic only; this
    function does not append them to the commanded rolling trajectory.
    """

    if not isinstance(rolling_result, ForwardRollingResult2D):
        raise TypeError("rolling_result must be ForwardRollingResult2D.")
    target_theta = _finite_scalar(
        retract_theta_target_rad, "retract_theta_target_rad"
    )
    theta_step = _finite_scalar(preview_step_rad, "preview_step_rad")
    beta_window = _finite_scalar(beta_search_window_rad, "beta_search_window_rad")
    beta_step = _finite_scalar(beta_search_step_rad, "beta_search_step_rad")
    if theta_step <= 0.0 or beta_window < 0.0 or beta_step <= 0.0:
        raise ValueError("preview and beta-search steps must be positive.")
    if preview_steps <= 0 or stable_window_frames < 2:
        raise ValueError("preview_steps must be positive and stable window at least 2.")

    final = rolling_result.final_frame
    right_indices = _right_rim_sample_indices(final.scene.geometry)
    adjacent_lengths = np.linalg.norm(
        np.diff(final.scene.geometry.points_hip_xz_m[list(right_indices)], axis=0),
        axis=1,
    )
    sample_arc_resolution = (
        0.0 if len(adjacent_lengths) == 0 else float(np.median(adjacent_lengths))
    )
    required_clearance = (
        max(
            rolling_result.top_roll_distance_m,
            2.0 * contact_tolerance_m,
            2.0 * sample_arc_resolution,
        )
        if safety_margin_m is None
        else _finite_scalar(safety_margin_m, "safety_margin_m")
    )
    if required_clearance < 0.0:
        raise ValueError("safety_margin_m must be non-negative.")

    rolling_complete = bool(rolling_result.success and final.top_roll_complete)
    measured_clearance = final.leading_edge_clearance_m
    clearance_satisfied = bool(
        measured_clearance is not None
        and measured_clearance >= required_clearance - 1e-9
    )
    top_frames = [
        frame
        for frame in rolling_result.frames
        if frame.top_roll_progress_m is not None
        and frame.terrain_surface_id is not None
        and frame.terrain_surface_id.endswith("_top")
    ]
    stable_frames = top_frames[-stable_window_frames:]
    stable_top_contact = bool(
        len(stable_frames) == stable_window_frames
        and all(
            frame.accepted
            and frame.valid_contact
            and frame.active_rim == RimId.RIGHT.value
            for frame in stable_frames
        )
    )
    collision_free = bool(
        len(stable_frames) == stable_window_frames
        and all(not frame.collision for frame in stable_frames)
    )
    no_slip_errors = []
    monotonic = len(stable_frames) == stable_window_frames
    for first, second in zip(stable_frames, stable_frames[1:]):
        if (
            first.alpha_rad is None
            or second.alpha_rad is None
            or second.alpha_rad <= first.alpha_rad
            or first.contact_point_world_xz_m is None
            or second.contact_point_world_xz_m is None
        ):
            monotonic = False
            continue
        arc_length = _sampled_rim_arc_length_between_frames(first, second)
        if arc_length is None:
            monotonic = False
            continue
        contact_advance = (
            second.contact_point_world_xz_m[0]
            - first.contact_point_world_xz_m[0]
        )
        if contact_advance <= 0.0:
            monotonic = False
        no_slip_errors.append(abs(contact_advance - arc_length))
    max_no_slip_error = max(no_slip_errors) if no_slip_errors else None
    no_slip_stable = bool(
        monotonic
        and len(no_slip_errors) == stable_window_frames - 1
        and max_no_slip_error is not None
        and max_no_slip_error <= max(contact_tolerance_m, 1e-6)
    )

    prerequisite_reason = None
    if not rolling_complete:
        prerequisite_reason = "TOP_ROLL_NOT_COMPLETE"
    elif not clearance_satisfied:
        prerequisite_reason = "INSUFFICIENT_LEADING_EDGE_CLEARANCE"
    elif not stable_top_contact:
        prerequisite_reason = "TOP_CONTACT_NOT_STABLE"
    elif not collision_free:
        prerequisite_reason = "RECENT_TOP_CONTACT_COLLISION"
    elif not no_slip_stable:
        prerequisite_reason = "RECENT_TOP_ROLL_NOT_NO_SLIP"

    preview_frames: list[RetractPreviewFrame2D] = []
    preview_success = False
    failure_reason = prerequisite_reason
    if prerequisite_reason is None:
        anchor = np.asarray(final.contact_point_world_xz_m, dtype=float)
        previous_theta = final.theta_rad
        previous_beta = final.beta_rad
        previous_sample = _frame_right_rim_sample_index(final)
        direction = -1.0 if target_theta < previous_theta else 1.0
        scene_kwargs = {
            "gamma_rad": final.scene.gamma_rad,
            "ground_height_m": final.scene.terrain.ground_height_m,
            "obstacle_x_start_m": final.scene.terrain.obstacle.x_min_m,
            "obstacle_width_m": final.scene.terrain.obstacle.width_m,
            "obstacle_height_m": final.scene.terrain.obstacle.height_m,
            "obstacle_id": final.scene.terrain.obstacle.obstacle_id,
            "arc_samples": len(final.scene.geometry.points_hip_xz_m) // 3,
        }
        for step in range(1, preview_steps + 1):
            theta = previous_theta + direction * min(
                theta_step, abs(target_theta - previous_theta)
            )
            if np.isclose(theta, previous_theta, atol=1e-12):
                preview_success = True
                break
            options = []
            for beta in _local_search_values(previous_beta, beta_window, beta_step):
                template = build_single_leg_rolling_scene_2d(
                    theta, beta, 0.0, 0.0, **scene_kwargs
                )
                sample_indices = _right_rim_sample_indices(template.geometry)
                sample_index = min(
                    sample_indices,
                    key=lambda index: template.geometry.points_hip_xz_m[index, 1],
                )
                if (
                    template.geometry.points_hip_xz_m[sample_index, 1]
                    > np.min(template.geometry.points_hip_xz_m[:, 1]) + 1e-8
                ):
                    continue
                scene = _translated_scene_with_sample_on_target_2d(
                    template, sample_index, anchor
                )
                query_result = query_single_leg_rolling_scene_2d(
                    scene,
                    contact_tolerance_m=contact_tolerance_m,
                    collision_tolerance_m=collision_tolerance_m,
                )
                candidate = _candidate_near_sample_and_target(
                    query_result,
                    scene.terrain,
                    sample_index,
                    anchor,
                    preferred_kind="top",
                )
                if not _forward_candidate_is_legal(
                    query_result, candidate, scene.terrain
                ):
                    continue
                options.append(
                    (
                        abs(beta - previous_beta),
                        0 if previous_sample is None else abs(sample_index - previous_sample),
                        beta,
                        sample_index,
                        scene,
                        query_result,
                        candidate,
                    )
                )
            if not options:
                failure_reason = f"RETRACT_PREVIEW_FAILED_AT_STEP_{step}"
                break
            (
                _, _, previous_beta, previous_sample,
                scene, query_result, candidate,
            ) = min(options, key=lambda item: item[:2])
            previous_theta = theta
            preview_frames.append(
                RetractPreviewFrame2D(
                    step=step,
                    theta_rad=float(theta),
                    beta_rad=float(previous_beta),
                    hip_position_world_xz_m=tuple(
                        float(value) for value in scene.hip_pose.position_world_xz_m
                    ),
                    active_sample_index=int(previous_sample),
                    alpha_rad=float(candidate.alpha_rad),
                    contact_point_world_xz_m=tuple(
                        float(value) for value in candidate.point_world_xz_m
                    ),
                    collision=query_result.collision,
                    accepted=True,
                    failure_reason=None,
                    scene=scene,
                    query_result=query_result,
                )
            )
        else:
            preview_success = True
        if preview_success:
            failure_reason = None

    ready = bool(
        rolling_complete
        and stable_top_contact
        and no_slip_stable
        and collision_free
        and clearance_satisfied
        and preview_success
    )
    return RetractReadinessResult2D(
        retract_theta_target_rad=target_theta,
        preview_step_rad=theta_step,
        stable_window_frames=stable_window_frames,
        required_leading_edge_clearance_m=required_clearance,
        measured_leading_edge_clearance_m=measured_clearance,
        rolling_complete=rolling_complete,
        stable_top_contact=stable_top_contact,
        no_slip_stable=no_slip_stable,
        collision_free=collision_free,
        clearance_satisfied=clearance_satisfied,
        preview_success=preview_success,
        ready_to_retract=ready,
        failure_reason=failure_reason,
        max_no_slip_error_m=max_no_slip_error,
        preview_frames=tuple(preview_frames),
    )


def retract_preview_rows(result: RetractReadinessResult2D) -> list[dict]:
    """Return a notebook-friendly table for the hypothetical preview."""

    if not isinstance(result, RetractReadinessResult2D):
        raise TypeError("result must be RetractReadinessResult2D.")
    return [
        {
            "step": frame.step,
            "theta_deg": float(np.rad2deg(frame.theta_rad)),
            "beta_deg": float(np.rad2deg(frame.beta_rad)),
            "hip_x_m": frame.hip_position_world_xz_m[0],
            "hip_z_m": frame.hip_position_world_xz_m[1],
            "active_sample_index": frame.active_sample_index,
            "alpha_rad": frame.alpha_rad,
            "contact_point_world_xz_m": frame.contact_point_world_xz_m,
            "collision": frame.collision,
            "accepted": frame.accepted,
            "failure_reason": frame.failure_reason,
        }
        for frame in result.preview_frames
    ]


def plot_retract_preview_final_pose_2d(
    result: RetractReadinessResult2D,
    *,
    ax=None,
    show: bool = False,
):
    """Plot the last hypothetical preview pose and its hip path."""

    if not isinstance(result, RetractReadinessResult2D):
        raise TypeError("result must be RetractReadinessResult2D.")
    if not result.preview_frames:
        raise ValueError("result has no retract preview frames to plot.")
    if ax is None:
        _, ax = plt.subplots(figsize=(10, 5.5))
    final = result.preview_frames[-1]
    plot_single_leg_rolling_scene_2d(
        final.scene, ax=ax, query_result=final.query_result
    )
    hip_path = np.asarray(
        [frame.hip_position_world_xz_m for frame in result.preview_frames],
        dtype=float,
    )
    ax.plot(
        hip_path[:, 0], hip_path[:, 1], "o--", color="#7c3aed",
        linewidth=1.5, markersize=4, label="retract preview hip path",
    )
    ax.set_title(
        "Phase D retract preview (hypothetical only): "
        f"ready={result.ready_to_retract}, final preview theta="
        f"{np.rad2deg(final.theta_rad):.1f} deg"
    )
    ax.legend(fontsize=8, loc="best")
    if show:
        plt.show()
    return ax


def run_retract_to_wheel_2d(
    rolling_result: ForwardRollingResult2D,
    *,
    theta_target_rad: float = np.deg2rad(17.0),
    theta_step_rad: float = np.deg2rad(1.0),
    beta_search_window_rad: float = np.deg2rad(5.0),
    beta_search_step_rad: float = np.deg2rad(1.0),
    beta_min_rad: float = -np.pi,
    beta_max_rad: float = np.pi,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
) -> RetractToWheelResult2D:
    """Commit a fixed-top-contact retract segment down to 17 degrees.

    The successful roll-up final frame is frame zero.  No additional wheel
    rolling is allowed: every retract pose keeps that final world contact
    point fixed.  Theta decreases by at most ``theta_step_rad``; beta is
    searched locally and hip x/z are reconstructed from the active lowest
    right-rim sample.  Foot-rim reset and re-extension are out of scope.
    """

    if not isinstance(rolling_result, ForwardRollingResult2D):
        raise TypeError("rolling_result must be ForwardRollingResult2D.")
    theta_target = _finite_scalar(theta_target_rad, "theta_target_rad")
    theta_step = _finite_scalar(theta_step_rad, "theta_step_rad")
    beta_window = _finite_scalar(beta_search_window_rad, "beta_search_window_rad")
    beta_step = _finite_scalar(beta_search_step_rad, "beta_search_step_rad")
    beta_min = _finite_scalar(beta_min_rad, "beta_min_rad")
    beta_max = _finite_scalar(beta_max_rad, "beta_max_rad")
    theta_joint_min = np.deg2rad(RobotParams.MIN_THETA_DEG)
    theta_joint_max = np.deg2rad(RobotParams.MAX_THETA_DEG)
    if theta_step <= 0.0 or beta_window < 0.0 or beta_step <= 0.0:
        raise ValueError("theta/beta search steps are invalid.")
    if beta_min >= beta_max:
        raise ValueError("beta_min_rad must be smaller than beta_max_rad.")
    if not (theta_joint_min - 1e-12 <= theta_target <= theta_joint_max + 1e-12):
        raise ValueError("theta_target_rad violates RobotParams theta limits.")

    final = rolling_result.final_frame
    start_is_valid = bool(
        rolling_result.success
        and final.top_roll_complete
        and final.accepted
        and final.valid_contact
        and not final.collision
        and final.active_rim == RimId.RIGHT.value
        and final.terrain_surface_id is not None
        and final.terrain_surface_id.endswith("_top")
        and final.contact_point_world_xz_m is not None
    )
    start_joint_limits_ok = bool(
        theta_joint_min - 1e-12 <= final.theta_rad <= theta_joint_max + 1e-12
        and beta_min - 1e-12 <= final.beta_rad <= beta_max + 1e-12
    )
    start_sample = _frame_right_rim_sample_index(final)
    start_candidate = (
        None
        if start_sample is None or final.contact_point_world_xz_m is None
        else _candidate_near_sample_and_target(
            final.query_result,
            final.scene.terrain,
            start_sample,
            final.contact_point_world_xz_m,
            preferred_kind="top",
        )
    )
    frames = [
        RetractToWheelFrame2D(
            step=0,
            theta_rad=final.theta_rad,
            beta_rad=final.beta_rad,
            hip_position_world_xz_m=tuple(
                float(value) for value in final.scene.hip_pose.position_world_xz_m
            ),
            active_rim=final.active_rim,
            active_sample_index=start_sample,
            alpha_rad=final.alpha_rad,
            contact_point_world_xz_m=final.contact_point_world_xz_m,
            terrain_surface_id=final.terrain_surface_id,
            valid_contact=final.valid_contact,
            collision=final.collision,
            joint_limits_ok=start_joint_limits_ok,
            accepted=start_is_valid and start_joint_limits_ok,
            failure_reason=(
                None
                if start_is_valid and start_joint_limits_ok
                else (
                    "ROLL_UP_END_STATE_INVALID"
                    if not start_is_valid
                    else "ROLL_UP_END_STATE_JOINT_LIMIT_VIOLATION"
                )
            ),
            scene=final.scene,
            query_result=final.query_result,
        )
    ]
    if not frames[0].accepted:
        reason = frames[0].failure_reason
        return RetractToWheelResult2D(
            rolling_result, theta_target, theta_step, beta_min, beta_max,
            tuple(frames), False, final.theta_rad, final.beta_rad, reason
        )
    if theta_target > final.theta_rad + 1e-12:
        raise ValueError("Step 5 requires theta_target_rad <= roll-up final theta.")
    if np.isclose(theta_target, final.theta_rad, atol=1e-12):
        return RetractToWheelResult2D(
            rolling_result, theta_target, theta_step, beta_min, beta_max,
            tuple(frames), True, None, None, None
        )

    anchor = np.asarray(final.contact_point_world_xz_m, dtype=float)
    previous_theta = final.theta_rad
    previous_beta = final.beta_rad
    previous_sample = start_sample
    obstacle = final.scene.terrain.obstacle
    scene_kwargs = {
        "gamma_rad": final.scene.gamma_rad,
        "ground_height_m": final.scene.terrain.ground_height_m,
        "obstacle_x_start_m": obstacle.x_min_m,
        "obstacle_width_m": obstacle.width_m,
        "obstacle_height_m": obstacle.height_m,
        "obstacle_id": obstacle.obstacle_id,
        "arc_samples": len(final.scene.geometry.points_hip_xz_m) // 3,
    }
    failure_theta = None
    failure_beta = None
    failure_reason = None

    while previous_theta > theta_target + 1e-12:
        theta = max(theta_target, previous_theta - theta_step)
        if not (theta_joint_min - 1e-12 <= theta <= theta_joint_max + 1e-12):
            failure_theta = theta
            failure_beta = previous_beta
            failure_reason = "THETA_JOINT_LIMIT_VIOLATION"
            break
        options = []
        collision_seen = False
        beta_within_limits_seen = False
        last_beta = previous_beta
        for beta in _local_search_values(previous_beta, beta_window, beta_step):
            last_beta = beta
            if not (beta_min - 1e-12 <= beta <= beta_max + 1e-12):
                continue
            beta_within_limits_seen = True
            template = build_single_leg_rolling_scene_2d(
                theta, beta, 0.0, 0.0, **scene_kwargs
            )
            right_samples = _right_rim_sample_indices(template.geometry)
            sample_index = min(
                right_samples,
                key=lambda index: template.geometry.points_hip_xz_m[index, 1],
            )
            if (
                template.geometry.points_hip_xz_m[sample_index, 1]
                > np.min(template.geometry.points_hip_xz_m[:, 1]) + 1e-8
            ):
                continue
            scene = _translated_scene_with_sample_on_target_2d(
                template, sample_index, anchor
            )
            query_result = query_single_leg_rolling_scene_2d(
                scene,
                contact_tolerance_m=contact_tolerance_m,
                collision_tolerance_m=collision_tolerance_m,
            )
            collision_seen = collision_seen or query_result.collision
            candidate = _candidate_near_sample_and_target(
                query_result,
                scene.terrain,
                sample_index,
                anchor,
                preferred_kind="top",
            )
            if not _forward_candidate_is_legal(
                query_result, candidate, scene.terrain
            ):
                continue
            options.append(
                (
                    abs(beta - previous_beta),
                    0 if previous_sample is None else abs(sample_index - previous_sample),
                    beta,
                    sample_index,
                    scene,
                    query_result,
                    candidate,
                )
            )
            # Center-first beta ordering makes the first legal option the
            # smallest local joint update.
            break
        if not options:
            failure_theta = theta
            failure_beta = last_beta if beta_within_limits_seen else previous_beta
            failure_reason = (
                "BETA_JOINT_LIMIT_VIOLATION"
                if not beta_within_limits_seen
                else (
                    "RETRACT_COLLISION_BLOCKED"
                    if collision_seen
                    else "NO_LEGAL_RIGHT_RIM_TOP_CONTACT"
                )
            )
            break
        (
            _, _, previous_beta, previous_sample,
            scene, query_result, candidate,
        ) = min(options, key=lambda item: item[:2])
        previous_theta = theta
        frames.append(
            RetractToWheelFrame2D(
                step=len(frames),
                theta_rad=float(theta),
                beta_rad=float(previous_beta),
                hip_position_world_xz_m=tuple(
                    float(value) for value in scene.hip_pose.position_world_xz_m
                ),
                active_rim=candidate.rim.value,
                active_sample_index=int(candidate.sample_index),
                alpha_rad=float(candidate.alpha_rad),
                contact_point_world_xz_m=tuple(
                    float(value) for value in candidate.point_world_xz_m
                ),
                terrain_surface_id=candidate.terrain_surface_id,
                valid_contact=True,
                collision=False,
                joint_limits_ok=True,
                accepted=True,
                failure_reason=None,
                scene=scene,
                query_result=query_result,
            )
        )

    success = bool(
        failure_reason is None
        and np.isclose(frames[-1].theta_rad, theta_target, atol=1e-12)
        and all(
            frame.accepted
            and frame.valid_contact
            and not frame.collision
            and frame.joint_limits_ok
            for frame in frames
        )
    )
    if not success and failure_reason is None:
        failure_theta = frames[-1].theta_rad
        failure_beta = frames[-1].beta_rad
        failure_reason = "THETA_TARGET_NOT_REACHED"
    return RetractToWheelResult2D(
        rolling_result=rolling_result,
        theta_target_rad=theta_target,
        theta_step_rad=theta_step,
        beta_min_rad=beta_min,
        beta_max_rad=beta_max,
        frames=tuple(frames),
        success=success,
        failure_theta_rad=failure_theta,
        failure_beta_rad=failure_beta,
        failure_reason=None if success else failure_reason,
    )


def retract_to_wheel_rows(result: RetractToWheelResult2D) -> list[dict]:
    """Return every committed Step-5 retract frame as table rows."""

    if not isinstance(result, RetractToWheelResult2D):
        raise TypeError("result must be RetractToWheelResult2D.")
    return [
        {
            "step": frame.step,
            "theta_rad": frame.theta_rad,
            "theta_deg": float(np.rad2deg(frame.theta_rad)),
            "beta_rad": frame.beta_rad,
            "beta_deg": float(np.rad2deg(frame.beta_rad)),
            "hip_x_m": frame.hip_position_world_xz_m[0],
            "hip_z_m": frame.hip_position_world_xz_m[1],
            "active_rim": frame.active_rim,
            "active_sample_index": frame.active_sample_index,
            "alpha_rad": frame.alpha_rad,
            "contact_point_world_xz_m": frame.contact_point_world_xz_m,
            "terrain_surface_id": frame.terrain_surface_id,
            "valid_contact": frame.valid_contact,
            "collision": frame.collision,
            "joint_limits_ok": frame.joint_limits_ok,
            "accepted": frame.accepted,
            "failure_reason": frame.failure_reason,
        }
        for frame in result.frames
    ]


def write_retract_to_wheel_csv(
    result: RetractToWheelResult2D,
    path: str | Path,
) -> Path:
    """Save the Step-5 retract trajectory to CSV."""

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rows = retract_to_wheel_rows(result)
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return output_path


def animate_roll_up_and_retract_2d(
    retract_result: RetractToWheelResult2D,
    *,
    interval_ms: int = 160,
    repeat: bool = False,
    show: bool = False,
):
    """Animate successful roll-up followed by the committed retract segment."""

    if not isinstance(retract_result, RetractToWheelResult2D):
        raise TypeError("retract_result must be RetractToWheelResult2D.")
    if interval_ms <= 0:
        raise ValueError("interval_ms must be positive.")
    rolling_frames = list(retract_result.rolling_result.frames)
    retract_frames = list(retract_result.frames[1:])
    combined = [
        (
            "RIGHT_RIM_ROLL_UP",
            frame.scene,
            frame.query_result,
            frame.theta_rad,
            frame.beta_rad,
            frame.contact_point_world_xz_m,
        )
        for frame in rolling_frames
    ] + [
        (
            "RETRACT_TO_WHEEL",
            frame.scene,
            frame.query_result,
            frame.theta_rad,
            frame.beta_rad,
            frame.contact_point_world_xz_m,
        )
        for frame in retract_frames
    ]
    if not combined:
        raise ValueError("combined trajectory is empty.")
    trajectory_points = np.vstack(
        [
            np.vstack(
                (scene.geometry.points_world_xz_m, scene.hip_pose.position_world_xz_m[None, :])
            )
            for _, scene, _, _, _, _ in combined
        ]
    )
    x_pad = max(0.035, 0.08 * float(np.ptp(trajectory_points[:, 0])))
    z_pad = max(0.035, 0.10 * float(np.ptp(trajectory_points[:, 1])))
    x_limits = (
        float(np.min(trajectory_points[:, 0]) - x_pad),
        float(np.max(trajectory_points[:, 0]) + x_pad),
    )
    z_limits = (
        min(
            combined[0][1].terrain.ground_height_m - 0.025,
            float(np.min(trajectory_points[:, 1]) - 0.025),
        ),
        float(np.max(trajectory_points[:, 1]) + z_pad),
    )
    figure, ax = plt.subplots(figsize=(11, 5.5))

    def draw_frame(index: int):
        phase, scene, query_result, theta, beta, contact = combined[index]
        ax.clear()
        plot_single_leg_rolling_scene_2d(scene, ax=ax, query_result=query_result)
        ax.set_xlim(*x_limits)
        ax.set_ylim(*z_limits)
        contact_trace = [
            item[5] for item in combined[: index + 1] if item[5] is not None
        ]
        if contact_trace:
            points = np.asarray(contact_trace, dtype=float)
            ax.plot(
                points[:, 0], points[:, 1], color="#0f766e", linewidth=2,
                marker="o", markersize=3, label="contact trace", zorder=13,
            )
        ax.set_title(
            f"Step 5 combined trajectory: {phase}  "
            f"frame {index}/{len(combined) - 1}\n"
            f"theta={np.rad2deg(theta):.1f} deg, "
            f"beta={np.rad2deg(beta):.1f} deg, "
            f"collision={query_result.collision}"
        )
        ax.text(
            0.01, 0.02,
            f"phase={phase}\n"
            f"retract target={retract_result.theta_target_deg:.1f} deg\n"
            f"retract success={retract_result.success}\n"
            f"failure={retract_result.failure_reason or 'none'}",
            transform=ax.transAxes,
            fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.86, "edgecolor": "0.75"},
            zorder=16,
        )
        return tuple(ax.lines)

    animation = FuncAnimation(
        figure,
        draw_frame,
        frames=len(combined),
        interval=interval_ms,
        repeat=repeat,
        blit=False,
    )
    draw_frame(0)
    if show:
        plt.show()
    return animation


def _wheel_reset_contour_indices(
    geometry: SampledLegGeometry2D,
) -> tuple[int, ...]:
    """Ordered forward-reset contour: right rim toward the foot-rim center."""

    right = sorted(_right_rim_sample_indices(geometry), reverse=True)
    foot = sorted(
        (
            index
            for index, region in enumerate(geometry.contact_regions)
            if region == "foot_rim"
        ),
        reverse=True,
    )
    return tuple(right + foot)


def _top_candidate_near_reset_sample(
    result: ContactQueryResult2D,
    terrain: TerrainProfile2D,
    sample_index: int,
    target_world_xz_m,
):
    target = np.asarray(target_world_xz_m, dtype=float)
    options = []
    for candidate in result.candidates:
        if candidate.rim not in {RimId.RIGHT, RimId.FOOT}:
            continue
        surface = terrain.surface_by_id(candidate.terrain_surface_id)
        if surface.kind.value != "obstacle_top":
            continue
        options.append(
            (
                abs(candidate.sample_index - sample_index),
                float(np.linalg.norm(candidate.point_world_xz_m - target)),
                candidate.surface_distance_m,
                candidate,
            )
        )
    return None if not options else min(options, key=lambda item: item[:3])[-1]


def _wheel_reset_candidate_is_legal(
    query_result: ContactQueryResult2D,
    candidate,
    terrain: TerrainProfile2D,
) -> bool:
    """Allow the planned right/foot top transition, but no other obstacle rim."""

    if query_result.collision or candidate is None:
        return False
    if candidate.rim not in {RimId.RIGHT, RimId.FOOT}:
        return False
    if terrain.surface_by_id(candidate.terrain_surface_id).kind.value != "obstacle_top":
        return False
    return not any(
        item.rim not in {RimId.RIGHT, RimId.FOOT}
        and item.terrain_surface_id != terrain.ground_surface_id
        for item in query_result.candidates
    )


def run_wheel_reset_roll_2d(
    retract_result: RetractToWheelResult2D,
    *,
    beta_step_rad: float = np.deg2rad(1.0),
    foot_ready_alpha_target_rad: float = 0.0,
    foot_ready_alpha_tolerance_rad: float = np.deg2rad(1.0),
    max_reset_rotation_rad: float = np.deg2rad(120.0),
    max_reset_forward_distance_m: float = 0.40,
    beta_min_rad: float = -np.pi,
    beta_max_rad: float = np.pi,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
) -> WheelResetRollResult2D:
    """Legacy fixed-theta pose chain retained for sequential comparison.

    This pre-Step-6.5 routine uses unsigned contour advance and therefore is
    not a physically verified +x no-slip trajectory when beta increases.
    Do not use it as the executable rolling result; use
    ``run_retract_and_reset_comparison_2d``.  Recorded frames occur when the
    sampled lowest support advances along the ordered right-to-foot contour.
    The world contact advances by the corresponding canonical contour arc
    length, providing the sampled no-slip definition of ``L_reset``.  The
    1.2-mm gap between the two existing sampled rim arcs is included as the
    first-version right/foot seam bridge; no new leg geometry is invented.
    """

    if not isinstance(retract_result, RetractToWheelResult2D):
        raise TypeError("retract_result must be RetractToWheelResult2D.")
    beta_step = _finite_scalar(beta_step_rad, "beta_step_rad")
    alpha_target = _finite_scalar(
        foot_ready_alpha_target_rad, "foot_ready_alpha_target_rad"
    )
    alpha_tolerance = _finite_scalar(
        foot_ready_alpha_tolerance_rad, "foot_ready_alpha_tolerance_rad"
    )
    max_rotation = _finite_scalar(max_reset_rotation_rad, "max_reset_rotation_rad")
    max_distance = _finite_scalar(
        max_reset_forward_distance_m, "max_reset_forward_distance_m"
    )
    beta_min = _finite_scalar(beta_min_rad, "beta_min_rad")
    beta_max = _finite_scalar(beta_max_rad, "beta_max_rad")
    if beta_step <= 0.0 or alpha_tolerance < 0.0:
        raise ValueError("beta_step_rad must be positive and alpha tolerance non-negative.")
    if max_rotation <= 0.0 or max_distance <= 0.0:
        raise ValueError("maximum reset rotation/distance must be positive.")
    if beta_min >= beta_max:
        raise ValueError("beta_min_rad must be smaller than beta_max_rad.")

    retract_final = retract_result.final_frame
    theta_target = np.deg2rad(RobotParams.MIN_THETA_DEG)
    start_valid = bool(
        retract_result.success
        and np.isclose(retract_final.theta_rad, theta_target, atol=1e-12)
        and retract_final.accepted
        and retract_final.valid_contact
        and not retract_final.collision
        and retract_final.active_rim == RimId.RIGHT.value
        and retract_final.terrain_surface_id is not None
        and retract_final.terrain_surface_id.endswith("_top")
        and retract_final.contact_point_world_xz_m is not None
    )
    start_sample = retract_final.active_sample_index
    start_beta = retract_final.beta_rad
    start_hip = retract_final.scene.hip_pose.position_world_xz_m
    start_contact = retract_final.contact_point_world_xz_m
    start_joint_limits_ok = beta_min <= start_beta <= beta_max
    frames = [
        WheelResetRollFrame2D(
            step=0,
            beta_rad=start_beta,
            reset_rotation_rad=0.0,
            active_rim=retract_final.active_rim,
            active_sample_index=start_sample,
            alpha_rad=retract_final.alpha_rad,
            contact_point_world_xz_m=start_contact,
            terrain_surface_id=retract_final.terrain_surface_id,
            hip_position_world_xz_m=tuple(float(value) for value in start_hip),
            hip_forward_displacement_m=0.0,
            contact_forward_displacement_m=0.0,
            valid_contact=retract_final.valid_contact,
            collision=retract_final.collision,
            joint_limits_ok=start_joint_limits_ok,
            foot_rim_ready=False,
            accepted=start_valid and start_joint_limits_ok,
            failure_reason=(
                None
                if start_valid and start_joint_limits_ok
                else (
                    "RETRACT_END_STATE_INVALID"
                    if not start_valid
                    else "RESET_START_BETA_JOINT_LIMIT_VIOLATION"
                )
            ),
            scene=retract_final.scene,
            query_result=retract_final.query_result,
        )
    ]
    if not frames[0].accepted:
        return WheelResetRollResult2D(
            retract_result, beta_step, alpha_target, alpha_tolerance,
            max_rotation, max_distance, tuple(frames), False, None, None,
            frames[0].failure_reason,
        )

    obstacle = retract_final.scene.terrain.obstacle
    arc_samples = len(retract_final.scene.geometry.points_hip_xz_m) // 3
    scene_kwargs = {
        "gamma_rad": retract_final.scene.gamma_rad,
        "ground_height_m": retract_final.scene.terrain.ground_height_m,
        "obstacle_x_start_m": obstacle.x_min_m,
        "obstacle_width_m": obstacle.width_m,
        "obstacle_height_m": obstacle.height_m,
        "obstacle_id": obstacle.obstacle_id,
        "arc_samples": arc_samples,
    }
    canonical = build_single_leg_rolling_scene_2d(
        theta_target, 0.0, 0.0, 0.0, **scene_kwargs
    )
    contour = _wheel_reset_contour_indices(canonical.geometry)
    contour_position = {sample_index: index for index, sample_index in enumerate(contour)}
    if start_sample not in contour_position:
        return WheelResetRollResult2D(
            retract_result, beta_step, alpha_target, alpha_tolerance,
            max_rotation, max_distance, tuple(frames), False, None, None,
            "RESET_START_SAMPLE_NOT_ON_RIGHT_FOOT_CONTOUR",
        )
    previous_position = contour_position[start_sample]
    previous_beta = start_beta
    previous_contact_x = float(start_contact[0])
    failure_reason = None

    while previous_beta - start_beta < max_rotation - 1e-12:
        beta = previous_beta + beta_step
        reset_rotation = beta - start_beta
        if reset_rotation > max_rotation + 1e-12:
            failure_reason = "MAX_RESET_ROTATION_REACHED"
            break
        if not (beta_min - 1e-12 <= beta <= beta_max + 1e-12):
            failure_reason = "RESET_BETA_JOINT_LIMIT_VIOLATION"
            break
        template = build_single_leg_rolling_scene_2d(
            theta_target, beta, 0.0, 0.0, **scene_kwargs
        )
        physical_samples = [
            index
            for index, region in enumerate(template.geometry.contact_regions)
            if region in {"right_rim", "foot_rim", "left_rim"}
        ]
        sample_index = min(
            physical_samples,
            key=lambda index: template.geometry.points_hip_xz_m[index, 1],
        )
        if sample_index not in contour_position:
            failure_reason = "RESET_LEFT_ALLOWED_RIGHT_FOOT_CONTOUR"
            break
        position = contour_position[sample_index]
        previous_beta = beta
        if position <= previous_position:
            # Beta moved less than one sampled contact interval.  Keep
            # integrating beta until a new material support sample appears.
            continue
        canonical_points = canonical.geometry.points_hip_xz_m[
            list(contour[previous_position : position + 1])
        ]
        arc_increment = float(
            np.sum(np.linalg.norm(np.diff(canonical_points, axis=0), axis=1))
        )
        contact_distance = frames[-1].contact_forward_displacement_m + arc_increment
        if contact_distance > max_distance + 1e-12:
            failure_reason = "MAX_RESET_FORWARD_DISTANCE_REACHED"
            break
        target = np.array(
            [previous_contact_x + arc_increment, start_contact[1]], dtype=float
        )
        if target[0] > obstacle.x_max_m - contact_tolerance_m:
            failure_reason = "OBSTACLE_TOP_LENGTH_EXCEEDED"
            break
        scene = _translated_scene_with_sample_on_target_2d(
            template, sample_index, target
        )
        query_result = query_single_leg_rolling_scene_2d(
            scene,
            contact_tolerance_m=contact_tolerance_m,
            collision_tolerance_m=collision_tolerance_m,
        )
        candidate = _top_candidate_near_reset_sample(
            query_result, scene.terrain, sample_index, target
        )
        if not _wheel_reset_candidate_is_legal(
            query_result, candidate, scene.terrain
        ):
            failure_reason = (
                "WHEEL_RESET_COLLISION_BLOCKED"
                if query_result.collision
                else "NO_LEGAL_RIGHT_FOOT_TOP_CONTACT"
            )
            break
        foot_ready = bool(
            candidate.rim is RimId.FOOT
            and abs(candidate.alpha_rad - alpha_target) <= alpha_tolerance + 1e-12
        )
        hip = scene.hip_pose.position_world_xz_m
        frames.append(
            WheelResetRollFrame2D(
                step=len(frames),
                beta_rad=float(beta),
                reset_rotation_rad=float(reset_rotation),
                active_rim=candidate.rim.value,
                active_sample_index=int(candidate.sample_index),
                alpha_rad=float(candidate.alpha_rad),
                contact_point_world_xz_m=tuple(
                    float(value) for value in candidate.point_world_xz_m
                ),
                terrain_surface_id=candidate.terrain_surface_id,
                hip_position_world_xz_m=tuple(float(value) for value in hip),
                hip_forward_displacement_m=float(hip[0] - start_hip[0]),
                contact_forward_displacement_m=float(contact_distance),
                valid_contact=True,
                collision=False,
                joint_limits_ok=True,
                foot_rim_ready=foot_ready,
                accepted=True,
                failure_reason=None,
                scene=scene,
                query_result=query_result,
            )
        )
        previous_position = position
        previous_contact_x = float(candidate.point_world_xz_m[0])
        if foot_ready:
            return WheelResetRollResult2D(
                retract_result=retract_result,
                beta_step_rad=beta_step,
                foot_ready_alpha_target_rad=alpha_target,
                foot_ready_alpha_tolerance_rad=alpha_tolerance,
                max_reset_rotation_rad=max_rotation,
                max_reset_forward_distance_m=max_distance,
                frames=tuple(frames),
                success=True,
                required_reset_rotation_rad=float(reset_rotation),
                required_reset_forward_distance_m=float(contact_distance),
                failure_reason=None,
            )

    if failure_reason is None:
        failure_reason = "MAX_RESET_ROTATION_REACHED"
    return WheelResetRollResult2D(
        retract_result, beta_step, alpha_target, alpha_tolerance,
        max_rotation, max_distance, tuple(frames), False, None, None,
        failure_reason,
    )


def wheel_reset_roll_rows(result: WheelResetRollResult2D) -> list[dict]:
    """Return every accepted Step-6 frame as notebook/CSV rows."""

    if not isinstance(result, WheelResetRollResult2D):
        raise TypeError("result must be WheelResetRollResult2D.")
    return [
        {
            "step": frame.step,
            "theta_deg": RobotParams.MIN_THETA_DEG,
            "beta_rad": frame.beta_rad,
            "beta_deg": float(np.rad2deg(frame.beta_rad)),
            "reset_rotation_rad": frame.reset_rotation_rad,
            "reset_rotation_deg": float(np.rad2deg(frame.reset_rotation_rad)),
            "active_rim": frame.active_rim,
            "active_sample_index": frame.active_sample_index,
            "alpha_rad": frame.alpha_rad,
            "contact_point_world_xz_m": frame.contact_point_world_xz_m,
            "terrain_surface_id": frame.terrain_surface_id,
            "hip_x_m": frame.hip_position_world_xz_m[0],
            "hip_z_m": frame.hip_position_world_xz_m[1],
            "hip_forward_displacement_m": frame.hip_forward_displacement_m,
            "contact_forward_displacement_m": frame.contact_forward_displacement_m,
            "valid_contact": frame.valid_contact,
            "collision": frame.collision,
            "joint_limits_ok": frame.joint_limits_ok,
            "foot_rim_ready": frame.foot_rim_ready,
            "accepted": frame.accepted,
            "failure_reason": frame.failure_reason,
        }
        for frame in result.frames
    ]


def write_wheel_reset_roll_csv(
    result: WheelResetRollResult2D,
    path: str | Path,
) -> Path:
    """Save the Step-6 reset trajectory to CSV."""

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rows = wheel_reset_roll_rows(result)
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return output_path


def plot_wheel_reset_roll_trajectory_2d(
    result: WheelResetRollResult2D,
    *,
    ax=None,
    show: bool = False,
):
    """Plot reset start, contact/hip paths, and foot-rim-ready final frame."""

    if not isinstance(result, WheelResetRollResult2D):
        raise TypeError("result must be WheelResetRollResult2D.")
    if ax is None:
        _, ax = plt.subplots(figsize=(10, 5.5))
    final = result.final_frame
    plot_single_leg_rolling_scene_2d(final.scene, ax=ax, query_result=final.query_result)
    contact = np.asarray(
        [frame.contact_point_world_xz_m for frame in result.frames], dtype=float
    )
    hips = np.asarray(
        [frame.hip_position_world_xz_m for frame in result.frames], dtype=float
    )
    ax.plot(contact[:, 0], contact[:, 1], color="#0f766e", linewidth=2, label="reset contact path")
    ax.plot(hips[:, 0], hips[:, 1], "--", color="#7c3aed", linewidth=1.5, label="reset hip path")
    ax.scatter(*contact[0], color="#2563eb", s=70, marker="o", label="reset start", zorder=15)
    final_label = "foot-rim ready" if result.success else "reset stopped"
    final_color = "#16a34a" if result.success else "#dc2626"
    ax.scatter(*contact[-1], color=final_color, s=90, marker="*", label=final_label, zorder=15)
    ax.set_title(
        "Legacy Step 6 unsigned-arc pose chain: "
        f"success={result.success}, rotation="
        f"{result.required_reset_rotation_deg if result.success else float('nan'):.1f} deg, "
        f"L_reset={result.l_reset_m if result.success else float('nan'):.4f} m"
    )
    ax.legend(fontsize=8, loc="best")
    if show:
        plt.show()
    return ax


def animate_wheel_reset_roll_2d(
    result: WheelResetRollResult2D,
    *,
    interval_ms: int = 140,
    repeat: bool = False,
    show: bool = False,
):
    """Animate Step 6 and label reset start / foot-rim-ready frames."""

    if not isinstance(result, WheelResetRollResult2D):
        raise TypeError("result must be WheelResetRollResult2D.")
    if interval_ms <= 0:
        raise ValueError("interval_ms must be positive.")
    points = np.vstack(
        [
            np.vstack((frame.scene.geometry.points_world_xz_m, np.asarray(frame.hip_position_world_xz_m)[None, :]))
            for frame in result.frames
        ]
    )
    x_pad = max(0.035, 0.08 * float(np.ptp(points[:, 0])))
    z_pad = max(0.035, 0.10 * float(np.ptp(points[:, 1])))
    x_limits = (float(np.min(points[:, 0]) - x_pad), float(np.max(points[:, 0]) + x_pad))
    z_limits = (
        min(result.frames[0].scene.terrain.ground_height_m - 0.025, float(np.min(points[:, 1]) - 0.025)),
        float(np.max(points[:, 1]) + z_pad),
    )
    figure, ax = plt.subplots(figsize=(11, 5.5))

    def draw_frame(index: int):
        frame = result.frames[index]
        ax.clear()
        plot_single_leg_rolling_scene_2d(frame.scene, ax=ax, query_result=frame.query_result)
        ax.set_xlim(*x_limits)
        ax.set_ylim(*z_limits)
        trace = np.asarray(
            [item.contact_point_world_xz_m for item in result.frames[: index + 1]],
            dtype=float,
        )
        ax.plot(trace[:, 0], trace[:, 1], color="#0f766e", linewidth=2, marker="o", markersize=3, zorder=13)
        label = "RESET START" if index == 0 else ("FOOT-RIM READY" if frame.foot_rim_ready else "WHEEL_RESET_ROLL")
        ax.set_title(
            f"Legacy Step 6 (not signed no-slip): {label}  "
            f"frame {index}/{len(result.frames) - 1}\n"
            f"theta=17.0 deg, beta={np.rad2deg(frame.beta_rad):.1f} deg, "
            f"rim={frame.active_rim}, alpha={frame.alpha_rad:.3f} rad"
        )
        ax.text(
            0.01, 0.02,
            f"rotation={np.rad2deg(frame.reset_rotation_rad):.1f} deg\n"
            f"contact advance={frame.contact_forward_displacement_m:.4f} m\n"
            f"hip advance={frame.hip_forward_displacement_m:.4f} m\n"
            f"ready={frame.foot_rim_ready}, collision={frame.collision}",
            transform=ax.transAxes,
            fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.86, "edgecolor": "0.75"},
            zorder=16,
        )
        return tuple(ax.lines)

    animation = FuncAnimation(
        figure, draw_frame, frames=len(result.frames), interval=interval_ms,
        repeat=repeat, blit=False,
    )
    draw_frame(0)
    if show:
        plt.show()
    return animation


def _retract_reset_contour_indices(
    geometry: SampledLegGeometry2D,
    beta_direction: int,
) -> tuple[int, ...]:
    """Return the material-contact order for one Step-6.5 branch."""

    groups = {
        name: sorted(
            index
            for index, region in enumerate(geometry.contact_regions)
            if region == name
        )
        for name in ("foot_rim", "left_rim", "right_rim")
    }
    if beta_direction > 0:
        # Shortest reverse branch: right(+alpha -> +40) -> foot(+40 -> 0).
        return tuple(reversed(groups["right_rim"])) + tuple(
            reversed(groups["foot_rim"])
        )
    # Forward continuation: right(+40 -> +180), wrap to left(-180 -> -40),
    # then foot(-40 -> 0).  The two sampled endpoint bridges are explicit.
    return (
        tuple(groups["right_rim"])
        + tuple(groups["left_rim"])
        + tuple(groups["foot_rim"])
    )


def _top_candidate_near_retract_reset_sample(
    result: ContactQueryResult2D,
    terrain: TerrainProfile2D,
    sample_index: int,
    target_world_xz_m,
    allowed_rims: set[RimId],
):
    target = np.asarray(target_world_xz_m, dtype=float)
    options = []
    for candidate in result.candidates:
        if candidate.rim not in allowed_rims:
            continue
        if terrain.surface_by_id(candidate.terrain_surface_id).kind.value != "obstacle_top":
            continue
        options.append(
            (
                abs(candidate.sample_index - sample_index),
                float(np.linalg.norm(candidate.point_world_xz_m - target)),
                candidate.surface_distance_m,
                candidate,
            )
        )
    return None if not options else min(options, key=lambda item: item[:3])[-1]


def _retract_reset_candidate_is_legal(
    query_result: ContactQueryResult2D,
    candidate,
    terrain: TerrainProfile2D,
    allowed_rims: set[RimId],
) -> bool:
    if query_result.collision or candidate is None or candidate.rim not in allowed_rims:
        return False
    if terrain.surface_by_id(candidate.terrain_surface_id).kind.value != "obstacle_top":
        return False
    return not any(
        item.rim not in allowed_rims
        and item.terrain_surface_id != terrain.ground_surface_id
        for item in query_result.candidates
    )


def _crossed_contour_seam_too_wide(
    geometry: SampledLegGeometry2D,
    contour: tuple[int, ...],
    previous_position: int,
    current_position: int,
    max_seam_bridge_m: float,
) -> bool:
    for position in range(previous_position, current_position):
        first = contour[position]
        second = contour[position + 1]
        if geometry.contact_regions[first] == geometry.contact_regions[second]:
            continue
        gap = float(
            np.linalg.norm(
                geometry.points_hip_xz_m[first]
                - geometry.points_hip_xz_m[second]
            )
        )
        if gap > max_seam_bridge_m + 1e-12:
            return True
    return False


def _scene_with_horizontal_no_slip_support_2d(
    previous_frame: RetractResetFrame2D,
    template: SingleLegRollingScene2D,
    sample_index: int,
    top_height_m: float,
) -> tuple[SingleLegRollingScene2D, np.ndarray, float]:
    """Place the next support while the previous material point has zero x slip.

    The previous active sample is a material point shared by both sampled
    configurations.  Its world-x coordinate is held fixed across the update:

        Hx_next + r_x(q_next, i_prev)
        = Hx_prev + r_x(q_prev, i_prev)

    The new lowest support sample determines hip z.  This finite-difference
    constraint retains the sign of beta rotation and includes theta-induced
    shape deformation; it does not turn unsigned contour length into forced
    +x motion.
    """

    previous_sample = previous_frame.active_sample_index
    if previous_sample is None:
        raise ValueError("previous_frame must contain an active material sample.")
    previous_hip = np.asarray(previous_frame.hip_position_world_xz_m, dtype=float)
    previous_local_x = previous_frame.scene.geometry.points_hip_xz_m[
        previous_sample, 0
    ]
    previous_material_world_x = float(previous_hip[0] + previous_local_x)
    next_previous_local_x = template.geometry.points_hip_xz_m[previous_sample, 0]
    hip_x = previous_material_world_x - next_previous_local_x
    hip_z = float(top_height_m - template.geometry.points_hip_xz_m[sample_index, 1])
    hip_pose = HipPose2D([hip_x, hip_z], pitch_world_hip_rad=0.0)
    scene = replace(
        template,
        hip_pose=hip_pose,
        geometry=replace(template.geometry, hip_pose=hip_pose),
    )
    support_point = scene.geometry.points_world_xz_m[sample_index]
    residual = float(
        hip_x + next_previous_local_x - previous_material_world_x
    )
    return scene, support_point.copy(), residual


def run_retract_and_reset_branch_2d(
    rolling_result: ForwardRollingResult2D,
    *,
    branch: str,
    theta_target_rad: float = np.deg2rad(17.0),
    theta_step_rad: float = np.deg2rad(1.0),
    beta_step_rad: float = np.deg2rad(1.0),
    beta_search_window_rad: float = np.deg2rad(5.0),
    foot_ready_alpha_target_rad: float = 0.0,
    foot_ready_alpha_tolerance_rad: float = np.deg2rad(1.0),
    stop_at: str = "foot_rim_ready",
    max_rotation_rad: float = np.deg2rad(400.0),
    max_forward_distance_m: float = 0.55,
    max_seam_bridge_m: float = 5e-3,
    obstacle_top_length_m: float | None = None,
    beta_min_rad: float = -4.0 * np.pi,
    beta_max_rad: float = 4.0 * np.pi,
    max_steps: int = 800,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
    forward_progress_tolerance_m: float = 1e-6,
    no_slip_tolerance_m: float = 1e-9,
) -> RetractResetBranchResult2D:
    """Run one coupled retract/reset branch from the successful roll-up end.

    ``forward_continuation`` keeps the roll-up beta direction (negative).
    ``shortest_reverse`` changes beta in the positive direction.  Theta and
    beta normally advance together.  If a sampled rim seam is still open
    while retracting, beta is held and theta continues closing the geometry;
    this prevents inventing a long virtual rolling surface.

    ``stop_at`` selects where this one trajectory is cut.  The
    retract-to-wheel state and the left-rim handover are milestones along the
    same no-slip motion as the original foot-rim reset, so they are exposed
    as stopping points rather than simulated a second time.

    ``"foot_rim_ready"``
        Original Step 6.5 behaviour and the default, so existing results and
        the Step 6.75 comparison are unchanged.
    ``"theta_target"``
        Stop as soon as theta reaches its target while still holding legal
        obstacle-top contact: the retract-to-wheel state.
    ``"left_rim_ready"``
        Stop once theta is at target and the active contact has handed over
        to the left rim, ready for a left-rim trailing-edge descent.
    ``"trailing_corner"``
        Roll until the contact reaches the trailing corner and stop there
        successfully, instead of reporting the corner as a top-length
        overrun.  Whether the configuration is *usable* at the corner is a
        separate question for the caller: reaching the corner is a property
        of the terrain, being ready to descend is a property of the leg.
    """

    if not isinstance(rolling_result, ForwardRollingResult2D):
        raise TypeError("rolling_result must be ForwardRollingResult2D.")
    directions = {"forward_continuation": -1, "shortest_reverse": 1}
    if branch not in directions:
        raise ValueError(f"branch must be one of {tuple(directions)}.")
    stop_options = (
        "foot_rim_ready",
        "theta_target",
        "left_rim_ready",
        "trailing_corner",
    )
    if stop_at not in stop_options:
        raise ValueError(f"stop_at must be one of {stop_options}.")
    beta_direction = directions[branch]
    direction_reversal = beta_direction > 0
    theta_target = _finite_scalar(theta_target_rad, "theta_target_rad")
    theta_step = _finite_scalar(theta_step_rad, "theta_step_rad")
    beta_step = _finite_scalar(beta_step_rad, "beta_step_rad")
    beta_search_window = _finite_scalar(
        beta_search_window_rad, "beta_search_window_rad"
    )
    alpha_target = _finite_scalar(
        foot_ready_alpha_target_rad, "foot_ready_alpha_target_rad"
    )
    alpha_tolerance = _finite_scalar(
        foot_ready_alpha_tolerance_rad, "foot_ready_alpha_tolerance_rad"
    )
    max_rotation = _finite_scalar(max_rotation_rad, "max_rotation_rad")
    max_distance = _finite_scalar(
        max_forward_distance_m, "max_forward_distance_m"
    )
    seam_limit = _finite_scalar(max_seam_bridge_m, "max_seam_bridge_m")
    top_length = (
        None
        if obstacle_top_length_m is None
        else _finite_scalar(obstacle_top_length_m, "obstacle_top_length_m")
    )
    beta_min = _finite_scalar(beta_min_rad, "beta_min_rad")
    beta_max = _finite_scalar(beta_max_rad, "beta_max_rad")
    forward_tolerance = _finite_scalar(
        forward_progress_tolerance_m, "forward_progress_tolerance_m"
    )
    no_slip_tolerance = _finite_scalar(
        no_slip_tolerance_m, "no_slip_tolerance_m"
    )
    if theta_step <= 0.0 or beta_step <= 0.0 or beta_search_window < beta_step:
        raise ValueError(
            "theta/beta steps must be positive and beta search window >= beta step."
        )
    if alpha_tolerance < 0.0 or seam_limit < 0.0:
        raise ValueError("alpha tolerance and seam bridge must be non-negative.")
    if forward_tolerance < 0.0 or no_slip_tolerance < 0.0:
        raise ValueError("motion tolerances must be non-negative.")
    if max_rotation <= 0.0 or max_distance <= 0.0 or max_steps <= 0:
        raise ValueError("Step-6.5 limits must be positive.")
    if top_length is not None and top_length <= 0.0:
        raise ValueError("obstacle_top_length_m must be positive when provided.")
    if beta_min >= beta_max:
        raise ValueError("beta_min_rad must be smaller than beta_max_rad.")

    start = rolling_result.final_frame
    theta_joint_min = np.deg2rad(RobotParams.MIN_THETA_DEG)
    theta_joint_max = np.deg2rad(RobotParams.MAX_THETA_DEG)
    start_sample = _frame_right_rim_sample_index(start)
    start_valid = bool(
        rolling_result.success
        and start.top_roll_complete
        and start.accepted
        and start.valid_contact
        and not start.collision
        and start.active_rim == RimId.RIGHT.value
        and start.contact_point_world_xz_m is not None
        and start.terrain_surface_id is not None
        and start.terrain_surface_id.endswith("_top")
        and start_sample is not None
    )
    start_joint_limits_ok = bool(
        theta_joint_min - 1e-12 <= start.theta_rad <= theta_joint_max + 1e-12
        and beta_min - 1e-12 <= start.beta_rad <= beta_max + 1e-12
    )
    start_hip = tuple(
        float(value) for value in start.scene.hip_pose.position_world_xz_m
    )
    frames = [
        RetractResetFrame2D(
            step=0,
            branch=branch,
            theta_rad=start.theta_rad,
            beta_rad=start.beta_rad,
            beta_unwrapped_rad=start.beta_rad,
            accumulated_rotation_rad=0.0,
            active_rim=start.active_rim,
            active_sample_index=start_sample,
            alpha_rad=start.alpha_rad,
            contact_point_world_xz_m=start.contact_point_world_xz_m,
            terrain_surface_id=start.terrain_surface_id,
            hip_position_world_xz_m=start_hip,
            hip_forward_displacement_m=0.0,
            contact_forward_displacement_m=0.0,
            step_hip_displacement_m=0.0,
            step_contact_displacement_m=0.0,
            no_slip_tangent_residual_m=0.0,
            valid_contact=start.valid_contact,
            collision=start.collision,
            joint_limits_ok=start_joint_limits_ok,
            foot_rim_ready=False,
            accepted=start_valid and start_joint_limits_ok,
            failure_reason=(
                None
                if start_valid and start_joint_limits_ok
                else (
                    "ROLL_UP_END_STATE_INVALID"
                    if not start_valid
                    else "ROLL_UP_END_STATE_JOINT_LIMIT_VIOLATION"
                )
            ),
            scene=start.scene,
            query_result=start.query_result,
        )
    ]
    if not frames[0].accepted:
        return RetractResetBranchResult2D(
            rolling_result, branch, beta_direction, direction_reversal,
            theta_target, alpha_target, alpha_tolerance, max_rotation,
            max_distance, tuple(frames), False, None, None,
            frames[0].failure_reason,
        )
    if theta_target > start.theta_rad + 1e-12:
        raise ValueError("theta_target_rad must not exceed roll-up final theta.")

    obstacle = start.scene.terrain.obstacle
    scene_kwargs = {
        "gamma_rad": start.scene.gamma_rad,
        "ground_height_m": start.scene.terrain.ground_height_m,
        "obstacle_x_start_m": obstacle.x_min_m,
        "obstacle_width_m": obstacle.width_m if top_length is None else top_length,
        "obstacle_height_m": obstacle.height_m,
        "obstacle_id": obstacle.obstacle_id,
        "arc_samples": len(start.scene.geometry.points_hip_xz_m) // 3,
    }
    contour = _retract_reset_contour_indices(start.scene.geometry, beta_direction)
    contour_position = {sample: position for position, sample in enumerate(contour)}
    if start_sample not in contour_position:
        return RetractResetBranchResult2D(
            rolling_result, branch, beta_direction, direction_reversal,
            theta_target, alpha_target, alpha_tolerance, max_rotation,
            max_distance, tuple(frames), False, None, None,
            "START_SAMPLE_NOT_ON_BRANCH_CONTOUR",
        )
    allowed_rims = (
        {RimId.RIGHT, RimId.FOOT}
        if beta_direction > 0
        else {RimId.RIGHT, RimId.LEFT, RimId.FOOT}
    )
    previous_theta = start.theta_rad
    previous_beta = start.beta_rad
    previous_position = contour_position[start_sample]
    previous_contact_x = float(start.contact_point_world_xz_m[0])
    failure_reason = None

    for _ in range(max_steps):
        theta = max(theta_target, previous_theta - theta_step)
        beta = previous_beta + beta_direction * beta_step
        rotation = (
            frames[-1].accumulated_rotation_rad + abs(beta - previous_beta)
        )
        if rotation > max_rotation + 1e-12:
            failure_reason = "MAX_COUPLED_RESET_ROTATION_REACHED"
            break
        if not (beta_min - 1e-12 <= beta <= beta_max + 1e-12):
            failure_reason = "COUPLED_RESET_BETA_JOINT_LIMIT_VIOLATION"
            break
        template = build_single_leg_rolling_scene_2d(
            theta, beta, 0.0, 0.0, **scene_kwargs
        )
        physical_samples = [
            index
            for index, region in enumerate(template.geometry.contact_regions)
            if region in {"right_rim", "left_rim", "foot_rim"}
        ]
        sample_index = min(
            physical_samples,
            key=lambda index: template.geometry.points_hip_xz_m[index, 1],
        )
        position = contour_position.get(sample_index)
        if position is None:
            failure_reason = "BRANCH_LEFT_ORDERED_PHYSICAL_RIM_CONTOUR"
            break

        if _crossed_contour_seam_too_wide(
            template.geometry, contour, previous_position, position, seam_limit
        ):
            if theta >= previous_theta - 1e-12:
                failure_reason = "RIM_SEAM_NOT_CLOSED_AT_THETA_TARGET"
                break
            # Continue retracting but hold beta/contact until the sampled seam
            # is geometrically close enough to represent a rim transition.
            beta = previous_beta
            rotation = frames[-1].accumulated_rotation_rad
            template = build_single_leg_rolling_scene_2d(
                theta, beta, 0.0, 0.0, **scene_kwargs
            )
            sample_index = min(
                physical_samples,
                key=lambda index: template.geometry.points_hip_xz_m[index, 1],
            )
            position = contour_position.get(sample_index)
            if position is None:
                failure_reason = "RETRACT_HOLD_LEFT_BRANCH_CONTOUR"
                break

        top_height = float(start.contact_point_world_xz_m[1])

        def evaluate_pose(candidate_template, candidate_sample, candidate_position):
            candidate_scene, candidate_target, residual = (
                _scene_with_horizontal_no_slip_support_2d(
                    frames[-1], candidate_template, candidate_sample, top_height
                )
            )
            candidate_query = query_single_leg_rolling_scene_2d(
                candidate_scene,
                contact_tolerance_m=contact_tolerance_m,
                collision_tolerance_m=collision_tolerance_m,
            )
            contact_candidate = _top_candidate_near_retract_reset_sample(
                candidate_query,
                candidate_scene.terrain,
                candidate_sample,
                candidate_target,
                allowed_rims,
            )
            hip = candidate_scene.hip_pose.position_world_xz_m
            previous_hip = np.asarray(frames[-1].hip_position_world_xz_m)
            step_hip_dx = float(hip[0] - previous_hip[0])
            step_contact_dx = float(candidate_target[0] - previous_contact_x)
            forward_valid = bool(
                step_hip_dx >= -forward_tolerance
                and step_contact_dx >= -forward_tolerance
            )
            no_slip_valid = abs(residual) <= no_slip_tolerance
            geometry_valid = _retract_reset_candidate_is_legal(
                candidate_query,
                contact_candidate,
                candidate_scene.terrain,
                allowed_rims,
            )
            return (
                candidate_scene,
                candidate_target,
                candidate_query,
                contact_candidate,
                step_hip_dx,
                step_contact_dx,
                residual,
                bool(
                    candidate_position is not None
                    and geometry_valid
                    and forward_valid
                    and no_slip_valid
                ),
                forward_valid,
                no_slip_valid,
            )

        evaluated = evaluate_pose(template, sample_index, position)
        attempted = [evaluated]
        if not evaluated[7]:
            # First local alternative: continue retracting while beta is held.
            if theta < previous_theta - 1e-12:
                hold_template = build_single_leg_rolling_scene_2d(
                    theta, previous_beta, 0.0, 0.0, **scene_kwargs
                )
                hold_sample = min(
                    physical_samples,
                    key=lambda index: hold_template.geometry.points_hip_xz_m[index, 1],
                )
                hold_position = contour_position.get(hold_sample)
                hold_evaluated = evaluate_pose(
                    hold_template, hold_sample, hold_position
                )
                attempted.append(hold_evaluated)
                if hold_evaluated[7]:
                    beta = previous_beta
                    rotation = frames[-1].accumulated_rotation_rad
                    template = hold_template
                    sample_index = hold_sample
                    position = hold_position
                    evaluated = hold_evaluated

        if not evaluated[7] and theta < previous_theta - 1e-12:
            # Second alternative: a one-step configuration correction opposite
            # to the branch direction.  It still must satisfy signed no-slip
            # and may not move hip/contact backward.
            adjustment_beta = previous_beta - beta_direction * beta_step
            adjustment_rotation = (
                frames[-1].accumulated_rotation_rad
                + abs(adjustment_beta - previous_beta)
            )
            if (
                adjustment_rotation <= max_rotation + 1e-12
                and beta_min - 1e-12 <= adjustment_beta <= beta_max + 1e-12
            ):
                adjustment_template = build_single_leg_rolling_scene_2d(
                    theta, adjustment_beta, 0.0, 0.0, **scene_kwargs
                )
                adjustment_sample = min(
                    physical_samples,
                    key=lambda index: adjustment_template.geometry.points_hip_xz_m[
                        index, 1
                    ],
                )
                adjustment_position = contour_position.get(adjustment_sample)
                adjustment_evaluated = evaluate_pose(
                    adjustment_template,
                    adjustment_sample,
                    adjustment_position,
                )
                attempted.append(adjustment_evaluated)
                if adjustment_evaluated[7]:
                    beta = adjustment_beta
                    rotation = adjustment_rotation
                    template = adjustment_template
                    sample_index = adjustment_sample
                    position = adjustment_position
                    evaluated = adjustment_evaluated

        if not evaluated[7] and theta < previous_theta - 1e-12:
            # Broader local continuation search.  Branch-direction updates are
            # tested first, then opposite corrections.  Every option still
            # obeys the same signed no-slip and nonnegative +x constraints.
            search_count = int(
                np.floor(beta_search_window / beta_step + 1e-12)
            )
            signed_steps = [
                beta_direction * count for count in range(2, search_count + 1)
            ] + [
                -beta_direction * count for count in range(2, search_count + 1)
            ]
            for signed_step in signed_steps:
                search_beta = previous_beta + signed_step * beta_step
                search_rotation = (
                    frames[-1].accumulated_rotation_rad
                    + abs(search_beta - previous_beta)
                )
                if (
                    search_rotation > max_rotation + 1e-12
                    or not beta_min - 1e-12 <= search_beta <= beta_max + 1e-12
                ):
                    continue
                search_template = build_single_leg_rolling_scene_2d(
                    theta, search_beta, 0.0, 0.0, **scene_kwargs
                )
                search_sample = min(
                    physical_samples,
                    key=lambda index: search_template.geometry.points_hip_xz_m[
                        index, 1
                    ],
                )
                search_position = contour_position.get(search_sample)
                search_evaluated = evaluate_pose(
                    search_template, search_sample, search_position
                )
                attempted.append(search_evaluated)
                if not search_evaluated[7]:
                    continue
                beta = search_beta
                rotation = search_rotation
                template = search_template
                sample_index = search_sample
                position = search_position
                evaluated = search_evaluated
                break

        if not evaluated[7]:
            if any(not item[8] for item in attempted):
                failure_reason = "NO_SLIP_REQUIRES_NEGATIVE_X_MOTION"
            elif any(not item[9] for item in attempted):
                failure_reason = "NO_SLIP_TANGENT_RESIDUAL_EXCEEDED"
            elif any(item[2].collision for item in attempted):
                failure_reason = "COUPLED_RESET_COLLISION_BLOCKED"
            else:
                failure_reason = "NO_LEGAL_BRANCH_TOP_CONTACT"
            break

        (
            scene,
            target,
            query_result,
            candidate,
            step_hip_dx,
            step_contact_dx,
            no_slip_residual,
            _,
            _,
            _,
        ) = evaluated
        contact_distance = float(target[0] - start.contact_point_world_xz_m[0])
        if contact_distance > max_distance + 1e-12:
            failure_reason = "MAX_COUPLED_RESET_FORWARD_DISTANCE_REACHED"
            break
        active_obstacle_x_min = scene_kwargs["obstacle_x_start_m"]
        active_obstacle_x_max = (
            active_obstacle_x_min + scene_kwargs["obstacle_width_m"]
        )
        if target[0] < active_obstacle_x_min - contact_tolerance_m:
            failure_reason = "NO_SLIP_MOVED_BEHIND_OBSTACLE_LEADING_EDGE"
            break
        if target[0] > active_obstacle_x_max - contact_tolerance_m:
            if stop_at == "trailing_corner" and len(frames) > 1:
                # The no-slip advance is dictated by the rim geometry, so the
                # contact lands within one step of the corner rather than
                # exactly on it.  The last accepted frame is the arrival
                # state; its residual gap is left for the caller to report.
                arrival = frames[-1]
                return RetractResetBranchResult2D(
                    rolling_result=rolling_result,
                    branch=branch,
                    beta_direction=beta_direction,
                    direction_reversal=direction_reversal,
                    theta_target_rad=theta_target,
                    foot_ready_alpha_target_rad=alpha_target,
                    foot_ready_alpha_tolerance_rad=alpha_tolerance,
                    max_rotation_rad=max_rotation,
                    max_forward_distance_m=max_distance,
                    frames=tuple(frames),
                    success=True,
                    required_rotation_rad=float(
                        arrival.accumulated_rotation_rad
                    ),
                    required_forward_distance_m=float(
                        arrival.contact_forward_displacement_m
                    ),
                    failure_reason=None,
                )
            failure_reason = "OBSTACLE_TOP_LENGTH_EXCEEDED"
            break
        theta_at_target = bool(np.isclose(theta, theta_target, atol=1e-12))
        foot_ready = bool(
            theta_at_target
            and candidate.rim is RimId.FOOT
            and abs(candidate.alpha_rad - alpha_target)
            <= alpha_tolerance + 1e-12
        )
        if stop_at == "foot_rim_ready":
            stop_now = foot_ready
        elif stop_at == "theta_target":
            stop_now = theta_at_target
        elif stop_at == "left_rim_ready":
            stop_now = bool(theta_at_target and candidate.rim is RimId.LEFT)
        else:
            # "trailing_corner": only arriving at the corner ends this run, and
            # that is handled by the corner check above.  Do not let the rim
            # handover terminate it early.
            stop_now = False
        hip = scene.hip_pose.position_world_xz_m
        frames.append(
            RetractResetFrame2D(
                step=len(frames),
                branch=branch,
                theta_rad=float(theta),
                beta_rad=float(beta),
                beta_unwrapped_rad=float(beta),
                accumulated_rotation_rad=float(rotation),
                active_rim=candidate.rim.value,
                active_sample_index=int(candidate.sample_index),
                alpha_rad=float(candidate.alpha_rad),
                contact_point_world_xz_m=tuple(
                    float(value) for value in candidate.point_world_xz_m
                ),
                terrain_surface_id=candidate.terrain_surface_id,
                hip_position_world_xz_m=tuple(float(value) for value in hip),
                hip_forward_displacement_m=float(hip[0] - start_hip[0]),
                contact_forward_displacement_m=float(contact_distance),
                step_hip_displacement_m=float(step_hip_dx),
                step_contact_displacement_m=float(step_contact_dx),
                no_slip_tangent_residual_m=float(no_slip_residual),
                valid_contact=True,
                collision=False,
                joint_limits_ok=True,
                foot_rim_ready=foot_ready,
                accepted=True,
                failure_reason=None,
                scene=scene,
                query_result=query_result,
            )
        )
        previous_theta = theta
        previous_beta = beta
        previous_position = contour_position.get(candidate.sample_index, position)
        previous_contact_x = float(candidate.point_world_xz_m[0])
        if stop_now:
            return RetractResetBranchResult2D(
                rolling_result=rolling_result,
                branch=branch,
                beta_direction=beta_direction,
                direction_reversal=direction_reversal,
                theta_target_rad=theta_target,
                foot_ready_alpha_target_rad=alpha_target,
                foot_ready_alpha_tolerance_rad=alpha_tolerance,
                max_rotation_rad=max_rotation,
                max_forward_distance_m=max_distance,
                frames=tuple(frames),
                success=True,
                required_rotation_rad=float(rotation),
                required_forward_distance_m=float(contact_distance),
                failure_reason=None,
            )
    else:
        failure_reason = "MAX_COUPLED_RESET_STEPS_REACHED"

    return RetractResetBranchResult2D(
        rolling_result, branch, beta_direction, direction_reversal,
        theta_target, alpha_target, alpha_tolerance, max_rotation,
        max_distance, tuple(frames), False, None, None, failure_reason,
    )


def run_retract_and_reset_comparison_2d(
    rolling_result: ForwardRollingResult2D,
    **branch_kwargs,
) -> RetractResetComparison2D:
    """Run both Step-6.5 branches and select the shorter feasible rotation.

    This is deliberately a kinematic selection, not an HT04 electrical-energy
    claim.  A later motor model may replace the selection basis without
    changing the two physical branch trajectories.
    """

    branches = tuple(
        run_retract_and_reset_branch_2d(
            rolling_result, branch=branch, **branch_kwargs
        )
        for branch in ("forward_continuation", "shortest_reverse")
    )
    feasible = [item for item in branches if item.success]
    selected = min(
        feasible,
        key=lambda item: (
            item.required_rotation_rad,
            item.required_forward_distance_m,
            item.direction_reversal,
        ),
        default=None,
    )
    return RetractResetComparison2D(
        rolling_result=rolling_result,
        branches=branches,
        selected_branch=None if selected is None else selected.branch,
        selection_basis=(
            "signed horizontal no-slip + nonnegative +x motion; then minimum "
            "feasible accumulated |delta beta| with distance tie-break"
        ),
    )


def retract_reset_comparison_rows(
    result: RetractResetComparison2D,
) -> list[dict]:
    """Return one Step-6.5 summary row per direction branch."""

    if not isinstance(result, RetractResetComparison2D):
        raise TypeError("result must be RetractResetComparison2D.")
    rows = []
    for branch in result.branches:
        final = branch.final_frame
        rows.append(
            {
                "branch": branch.branch,
                "selected": branch.branch == result.selected_branch,
                "success": branch.success,
                "direction_reversal": branch.direction_reversal,
                "required_rotation_rad": branch.required_rotation_rad,
                "required_rotation_deg": branch.required_rotation_deg,
                "L_reset_m": branch.l_reset_m,
                "frames": len(branch.frames),
                "final_theta_deg": float(np.rad2deg(final.theta_rad)),
                "final_beta_unwrapped_deg": float(
                    np.rad2deg(final.beta_unwrapped_rad)
                ),
                "final_rim": final.active_rim,
                "final_alpha_deg": (
                    None
                    if final.alpha_rad is None
                    else float(np.rad2deg(final.alpha_rad))
                ),
                "final_surface": final.terrain_surface_id,
                "hip_forward_displacement_m": final.hip_forward_displacement_m,
                "contact_forward_displacement_m": (
                    final.contact_forward_displacement_m
                ),
                "minimum_step_hip_displacement_m": min(
                    frame.step_hip_displacement_m for frame in branch.frames
                ),
                "minimum_step_contact_displacement_m": min(
                    frame.step_contact_displacement_m for frame in branch.frames
                ),
                "max_no_slip_tangent_residual_m": max(
                    abs(frame.no_slip_tangent_residual_m)
                    for frame in branch.frames
                ),
                "failure_reason": branch.failure_reason,
                "selection_basis": result.selection_basis,
            }
        )
    return rows


def retract_reset_frame_rows(
    result: RetractResetComparison2D,
) -> list[dict]:
    """Return all Step-6.5 frames from both branches."""

    if not isinstance(result, RetractResetComparison2D):
        raise TypeError("result must be RetractResetComparison2D.")
    rows = []
    for branch in result.branches:
        for frame in branch.frames:
            rows.append(
                {
                    "branch": branch.branch,
                    "selected": branch.branch == result.selected_branch,
                    "step": frame.step,
                    "theta_rad": frame.theta_rad,
                    "theta_deg": float(np.rad2deg(frame.theta_rad)),
                    "beta_rad": frame.beta_rad,
                    "beta_unwrapped_deg": float(
                        np.rad2deg(frame.beta_unwrapped_rad)
                    ),
                    "accumulated_rotation_rad": frame.accumulated_rotation_rad,
                    "accumulated_rotation_deg": float(
                        np.rad2deg(frame.accumulated_rotation_rad)
                    ),
                    "active_rim": frame.active_rim,
                    "active_sample_index": frame.active_sample_index,
                    "alpha_rad": frame.alpha_rad,
                    "contact_point_world_xz_m": frame.contact_point_world_xz_m,
                    "terrain_surface_id": frame.terrain_surface_id,
                    "hip_x_m": frame.hip_position_world_xz_m[0],
                    "hip_z_m": frame.hip_position_world_xz_m[1],
                    "hip_forward_displacement_m": frame.hip_forward_displacement_m,
                    "contact_forward_displacement_m": (
                        frame.contact_forward_displacement_m
                    ),
                    "step_hip_displacement_m": frame.step_hip_displacement_m,
                    "step_contact_displacement_m": (
                        frame.step_contact_displacement_m
                    ),
                    "no_slip_tangent_residual_m": (
                        frame.no_slip_tangent_residual_m
                    ),
                    "valid_contact": frame.valid_contact,
                    "collision": frame.collision,
                    "joint_limits_ok": frame.joint_limits_ok,
                    "foot_rim_ready": frame.foot_rim_ready,
                    "accepted": frame.accepted,
                    "failure_reason": frame.failure_reason,
                }
            )
    return rows


def write_retract_reset_comparison_csv(
    result: RetractResetComparison2D,
    summary_path: str | Path,
    frame_path: str | Path,
) -> tuple[Path, Path]:
    """Save Step-6.5 branch summaries and both full trajectories."""

    outputs = []
    for path, rows in (
        (summary_path, retract_reset_comparison_rows(result)),
        (frame_path, retract_reset_frame_rows(result)),
    ):
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        outputs.append(output_path)
    return tuple(outputs)


def plot_retract_reset_comparison_2d(
    result: RetractResetComparison2D,
    *,
    axes=None,
    show: bool = False,
):
    """Plot the final pose and contact/hip traces for both Step-6.5 branches."""

    if not isinstance(result, RetractResetComparison2D):
        raise TypeError("result must be RetractResetComparison2D.")
    if axes is None:
        _, axes = plt.subplots(1, len(result.branches), figsize=(15, 5.5))
    axes = np.atleast_1d(axes)
    if len(axes) != len(result.branches):
        raise ValueError("axes must contain one axis per branch.")
    for ax, branch in zip(axes, result.branches):
        final = branch.final_frame
        plot_single_leg_rolling_scene_2d(
            final.scene, ax=ax, query_result=final.query_result
        )
        contact = np.asarray(
            [frame.contact_point_world_xz_m for frame in branch.frames],
            dtype=float,
        )
        hips = np.asarray(
            [frame.hip_position_world_xz_m for frame in branch.frames],
            dtype=float,
        )
        ax.plot(
            contact[:, 0], contact[:, 1], color="#0f766e", linewidth=2,
            label="contact path",
        )
        ax.plot(
            hips[:, 0], hips[:, 1], "--", color="#7c3aed", linewidth=1.5,
            label="hip path",
        )
        ax.scatter(
            *contact[0], color="#2563eb", s=55, marker="o",
            label="roll-up end", zorder=15,
        )
        ax.scatter(
            *contact[-1],
            color="#16a34a" if branch.success else "#dc2626",
            s=80,
            marker="*",
            label="foot-rim ready" if branch.success else "branch stopped",
            zorder=15,
        )
        selected = " [SELECTED]" if branch.branch == result.selected_branch else ""
        ax.set_title(
            f"{branch.branch}{selected}\n"
            f"success={branch.success}, rotation="
            f"{branch.required_rotation_deg if branch.success else float('nan'):.1f} deg, "
            f"L={branch.l_reset_m if branch.success else float('nan'):.3f} m"
        )
        ax.legend(fontsize=7, loc="best")
    if show:
        plt.show()
    return axes


def animate_retract_and_reset_branch_2d(
    result: RetractResetBranchResult2D,
    *,
    interval_ms: int = 100,
    frame_stride: int = 1,
    repeat: bool = False,
    show: bool = False,
):
    """Animate one Step-6.5 branch, optionally decimating display frames."""

    if not isinstance(result, RetractResetBranchResult2D):
        raise TypeError("result must be RetractResetBranchResult2D.")
    if interval_ms <= 0 or frame_stride <= 0:
        raise ValueError("interval_ms and frame_stride must be positive.")
    display_frames = list(result.frames[::frame_stride])
    if display_frames[-1] is not result.frames[-1]:
        display_frames.append(result.frames[-1])
    points = np.vstack(
        [
            np.vstack(
                (
                    frame.scene.geometry.points_world_xz_m,
                    np.asarray(frame.hip_position_world_xz_m)[None, :],
                )
            )
            for frame in display_frames
        ]
    )
    x_pad = max(0.035, 0.06 * float(np.ptp(points[:, 0])))
    z_pad = max(0.035, 0.10 * float(np.ptp(points[:, 1])))
    x_limits = (
        float(np.min(points[:, 0]) - x_pad),
        float(np.max(points[:, 0]) + x_pad),
    )
    z_limits = (
        min(
            display_frames[0].scene.terrain.ground_height_m - 0.025,
            float(np.min(points[:, 1]) - 0.025),
        ),
        float(np.max(points[:, 1]) + z_pad),
    )
    figure, ax = plt.subplots(figsize=(11, 5.5))

    def draw_frame(index: int):
        frame = display_frames[index]
        ax.clear()
        plot_single_leg_rolling_scene_2d(
            frame.scene, ax=ax, query_result=frame.query_result
        )
        ax.set_xlim(*x_limits)
        ax.set_ylim(*z_limits)
        trace = np.asarray(
            [item.contact_point_world_xz_m for item in display_frames[: index + 1]],
            dtype=float,
        )
        ax.plot(trace[:, 0], trace[:, 1], color="#0f766e", linewidth=2)
        phase = (
            "FOOT-RIM READY"
            if frame.foot_rim_ready
            else (
                "BRANCH FAILED"
                if index == len(display_frames) - 1 and not result.success
                else "RETRACT + RESET"
            )
        )
        ax.set_title(
            f"Step 6.5 {result.branch}: {phase}\n"
            f"theta={np.rad2deg(frame.theta_rad):.1f} deg, "
            f"beta(unwrapped)={np.rad2deg(frame.beta_unwrapped_rad):.1f} deg, "
            f"rim={frame.active_rim}"
        )
        ax.text(
            0.01,
            0.02,
            f"rotation={np.rad2deg(frame.accumulated_rotation_rad):.1f} deg\n"
            f"contact advance={frame.contact_forward_displacement_m:.4f} m\n"
            f"no-slip residual={frame.no_slip_tangent_residual_m:.2e} m\n"
            f"theta target={np.rad2deg(result.theta_target_rad):.1f} deg\n"
            f"ready={frame.foot_rim_ready}, collision={frame.collision}\n"
            f"failure={result.failure_reason or 'none'}",
            transform=ax.transAxes,
            fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.86, "edgecolor": "0.75"},
            zorder=16,
        )
        return tuple(ax.lines)

    animation = FuncAnimation(
        figure,
        draw_frame,
        frames=len(display_frames),
        interval=interval_ms,
        repeat=repeat,
        blit=False,
    )
    draw_frame(0)
    if show:
        plt.show()
    return animation


def _minimum_scene_height_above_obstacle_top_2d(
    scene: SingleLegRollingScene2D,
) -> float:
    """Return a conservative whole-leg vertical clearance above obstacle top."""

    obstacle = scene.terrain.obstacle
    if obstacle is None:
        raise ValueError("Step 6.75 requires one rectangular obstacle.")
    z_values = list(scene.geometry.points_world_xz_m[:, 1])
    for segment in scene.geometry.link_segments_world_xz_m:
        z_values.extend(segment[:, 1])
    top_height = scene.terrain.ground_height_m + obstacle.height_m
    return float(min(z_values) - top_height)


def _select_foot_top_candidate_2d(
    query_result: ContactQueryResult2D,
    terrain: TerrainProfile2D,
    target_world_xz_m: np.ndarray,
    *,
    alpha_target_rad: float,
    alpha_tolerance_rad: float,
):
    options = []
    for candidate in query_result.candidates:
        if candidate.rim is not RimId.FOOT:
            continue
        if terrain.surface_by_id(candidate.terrain_surface_id).kind.value != "obstacle_top":
            continue
        alpha_error = abs(candidate.alpha_rad - alpha_target_rad)
        if alpha_error > alpha_tolerance_rad + 1e-12:
            continue
        options.append(
            (
                float(np.linalg.norm(candidate.point_world_xz_m - target_world_xz_m)),
                alpha_error,
                candidate.surface_distance_m,
                candidate,
            )
        )
    return None if not options else min(options, key=lambda item: item[:3])[-1]


def _airborne_beta_target_2d(start_beta_rad: float, beta_direction: int) -> float:
    """Return the next equivalent foot-down beta in one requested direction."""

    period = 2.0 * np.pi
    if beta_direction < 0:
        target = np.floor((start_beta_rad - 1e-12) / period) * period
        if target >= start_beta_rad - 1e-12:
            target -= period
    elif beta_direction > 0:
        target = np.ceil((start_beta_rad + 1e-12) / period) * period
        if target <= start_beta_rad + 1e-12:
            target += period
    else:
        raise ValueError("beta_direction must be -1 or +1.")
    return float(target)


def run_airborne_retract_and_foot_reset_branch_2d(
    rolling_result: ForwardRollingResult2D,
    *,
    branch: str,
    theta_target_rad: float = np.deg2rad(17.0),
    theta_step_rad: float = np.deg2rad(2.0),
    beta_step_rad: float = np.deg2rad(2.0),
    vertical_step_m: float = 0.005,
    airborne_clearance_m: float = 0.015,
    touchdown_contact_advance_m: float = 0.05,
    touchdown_edge_margin_m: float = 0.015,
    foot_ready_alpha_target_rad: float = 0.0,
    foot_ready_alpha_tolerance_rad: float = np.deg2rad(1.0),
    max_rotation_rad: float = np.deg2rad(400.0),
    max_hip_lift_m: float = 0.30,
    obstacle_top_length_m: float | None = None,
    beta_min_rad: float = -4.0 * np.pi,
    beta_max_rad: float = 4.0 * np.pi,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
    other_leg_support_assumed: bool = True,
) -> AirborneRetractResetBranchResult2D:
    """Retract and rotate an unloaded leg, then land its foot rim on the top.

    The routine starts at a successful Step-4.5 top-contact frame.  It first
    raises the hip until every sampled rim and linkage endpoint is above the
    obstacle top by ``airborne_clearance_m``.  Theta and beta are then changed
    simultaneously while hip x moves toward a user-defined touchdown target.
    The final configuration is lowered onto the obstacle and must pass the
    existing terrain-aware query as a foot-rim top contact.

    This is a discretely checked single-leg kinematic trajectory.  The other
    three legs carrying the body is an explicit assumption, not a stability or
    dynamics result.
    """

    if not isinstance(rolling_result, ForwardRollingResult2D):
        raise TypeError("rolling_result must be ForwardRollingResult2D.")
    directions = {"forward_continuation": -1, "shortest_reverse": 1}
    if branch not in directions:
        raise ValueError(f"branch must be one of {tuple(directions)}.")
    beta_direction = directions[branch]
    theta_target = _finite_scalar(theta_target_rad, "theta_target_rad")
    theta_step = _finite_scalar(theta_step_rad, "theta_step_rad")
    beta_step = _finite_scalar(beta_step_rad, "beta_step_rad")
    vertical_step = _finite_scalar(vertical_step_m, "vertical_step_m")
    clearance = _finite_scalar(airborne_clearance_m, "airborne_clearance_m")
    touchdown_advance = _finite_scalar(
        touchdown_contact_advance_m, "touchdown_contact_advance_m"
    )
    edge_margin = _finite_scalar(touchdown_edge_margin_m, "touchdown_edge_margin_m")
    alpha_target = _finite_scalar(
        foot_ready_alpha_target_rad, "foot_ready_alpha_target_rad"
    )
    alpha_tolerance = _finite_scalar(
        foot_ready_alpha_tolerance_rad, "foot_ready_alpha_tolerance_rad"
    )
    max_rotation = _finite_scalar(max_rotation_rad, "max_rotation_rad")
    max_lift = _finite_scalar(max_hip_lift_m, "max_hip_lift_m")
    beta_min = _finite_scalar(beta_min_rad, "beta_min_rad")
    beta_max = _finite_scalar(beta_max_rad, "beta_max_rad")
    if min(theta_step, beta_step, vertical_step, max_rotation, max_lift) <= 0.0:
        raise ValueError("Step-6.75 step sizes and limits must be positive.")
    if min(clearance, touchdown_advance, edge_margin, alpha_tolerance) < 0.0:
        raise ValueError("Step-6.75 distances and tolerances must be non-negative.")
    if beta_min >= beta_max:
        raise ValueError("beta_min_rad must be smaller than beta_max_rad.")

    start = rolling_result.final_frame
    start_sample = _frame_right_rim_sample_index(start)
    theta_joint_min = np.deg2rad(RobotParams.MIN_THETA_DEG)
    theta_joint_max = np.deg2rad(RobotParams.MAX_THETA_DEG)
    start_valid = bool(
        rolling_result.success
        and start.top_roll_complete
        and start.accepted
        and start.valid_contact
        and not start.collision
        and start.active_rim == RimId.RIGHT.value
        and start_sample is not None
        and start.contact_point_world_xz_m is not None
        and start.terrain_surface_id is not None
        and start.terrain_surface_id.endswith("_top")
    )
    start_joint_ok = bool(
        theta_joint_min - 1e-12 <= start.theta_rad <= theta_joint_max + 1e-12
        and beta_min - 1e-12 <= start.beta_rad <= beta_max + 1e-12
    )
    obstacle = start.scene.terrain.obstacle
    if obstacle is None:
        raise ValueError("Step 6.75 requires one rectangular obstacle.")
    top_length = (
        obstacle.width_m
        if obstacle_top_length_m is None
        else _finite_scalar(obstacle_top_length_m, "obstacle_top_length_m")
    )
    if top_length <= 0.0:
        raise ValueError("obstacle_top_length_m must be positive.")
    scene_kwargs = {
        "gamma_rad": start.scene.gamma_rad,
        "ground_height_m": start.scene.terrain.ground_height_m,
        "obstacle_x_start_m": obstacle.x_min_m,
        "obstacle_width_m": top_length,
        "obstacle_height_m": obstacle.height_m,
        "obstacle_id": obstacle.obstacle_id,
        "arc_samples": len(start.scene.geometry.points_hip_xz_m) // 3,
    }
    top_height = scene_kwargs["ground_height_m"] + scene_kwargs["obstacle_height_m"]
    start_hip = np.asarray(start.scene.hip_pose.position_world_xz_m, dtype=float)
    beta_target = _airborne_beta_target_2d(start.beta_rad, beta_direction)
    required_rotation = abs(beta_target - start.beta_rad)

    frames: list[AirborneRetractResetFrame2D] = []

    def append_frame(
        scene: SingleLegRollingScene2D,
        phase: str,
        accumulated_rotation_rad: float,
        *,
        contact_required: bool,
        contact_candidate=None,
        failure_reason: str | None = None,
    ) -> bool:
        query_result = query_single_leg_rolling_scene_2d(
            scene,
            contact_tolerance_m=contact_tolerance_m,
            collision_tolerance_m=collision_tolerance_m,
        )
        joint_ok = bool(
            theta_joint_min - 1e-12 <= scene.theta_rad <= theta_joint_max + 1e-12
            and beta_min - 1e-12 <= scene.beta_rad <= beta_max + 1e-12
        )
        contact_ok = bool(contact_candidate is not None) if contact_required else True
        accepted = bool(
            not query_result.collision
            and joint_ok
            and contact_ok
            and failure_reason is None
        )
        candidate = contact_candidate
        hip = scene.hip_pose.position_world_xz_m
        minimum_clearance = _minimum_scene_height_above_obstacle_top_2d(scene)
        foot_ready = bool(
            phase == "TOUCHDOWN"
            and accepted
            and candidate is not None
            and candidate.rim is RimId.FOOT
            and abs(candidate.alpha_rad - alpha_target) <= alpha_tolerance + 1e-12
        )
        frames.append(
            AirborneRetractResetFrame2D(
                step=len(frames),
                branch=branch,
                phase=phase,
                theta_rad=float(scene.theta_rad),
                beta_rad=float(scene.beta_rad),
                beta_unwrapped_rad=float(scene.beta_rad),
                accumulated_rotation_rad=float(accumulated_rotation_rad),
                hip_position_world_xz_m=tuple(float(value) for value in hip),
                hip_forward_displacement_m=float(hip[0] - start_hip[0]),
                hip_vertical_displacement_m=float(hip[1] - start_hip[1]),
                active_rim=None if candidate is None else candidate.rim.value,
                active_sample_index=(
                    None if candidate is None else int(candidate.sample_index)
                ),
                alpha_rad=None if candidate is None else float(candidate.alpha_rad),
                contact_point_world_xz_m=(
                    None
                    if candidate is None
                    else tuple(float(value) for value in candidate.point_world_xz_m)
                ),
                terrain_surface_id=(
                    None if candidate is None else candidate.terrain_surface_id
                ),
                contact_required=contact_required,
                valid_contact=bool(candidate is not None),
                collision=query_result.collision,
                joint_limits_ok=joint_ok,
                minimum_clearance_above_top_m=minimum_clearance,
                foot_rim_ready=foot_ready,
                accepted=accepted,
                failure_reason=failure_reason,
                scene=scene,
                query_result=query_result,
            )
        )
        return accepted

    start_candidate = next(
        (
            item
            for item in start.query_result.candidates
            if item.rim is RimId.RIGHT
            and item.terrain_surface_id == start.terrain_surface_id
            and item.sample_index == start_sample
        ),
        None,
    )
    if not start_valid or not start_joint_ok or start_candidate is None:
        append_frame(
            start.scene,
            "TAKEOFF_CONTACT",
            0.0,
            contact_required=True,
            contact_candidate=start_candidate,
            failure_reason=(
                "ROLL_UP_END_STATE_INVALID"
                if not start_valid or start_candidate is None
                else "ROLL_UP_END_STATE_JOINT_LIMIT_VIOLATION"
            ),
        )
        return AirborneRetractResetBranchResult2D(
            rolling_result, branch, beta_direction, beta_direction > 0,
            theta_target, beta_target, clearance, touchdown_advance,
            bool(other_leg_support_assumed), tuple(frames), False, None, None,
            None, frames[-1].failure_reason,
        )
    append_frame(
        start.scene,
        "TAKEOFF_CONTACT",
        0.0,
        contact_required=True,
        contact_candidate=start_candidate,
    )
    if theta_target > start.theta_rad + 1e-12:
        raise ValueError("theta_target_rad must not exceed roll-up final theta.")
    if required_rotation > max_rotation + 1e-12:
        return replace(
            AirborneRetractResetBranchResult2D(
                rolling_result, branch, beta_direction, beta_direction > 0,
                theta_target, beta_target, clearance, touchdown_advance,
                bool(other_leg_support_assumed), tuple(frames), False, None, None,
                None, "MAX_AIRBORNE_ROTATION_EXCEEDED",
            )
        )

    touchdown_x = float(start.contact_point_world_xz_m[0] + touchdown_advance)
    top_x_min = obstacle.x_min_m + edge_margin
    top_x_max = obstacle.x_min_m + top_length - edge_margin
    if not top_x_min <= touchdown_x <= top_x_max:
        return AirborneRetractResetBranchResult2D(
            rolling_result, branch, beta_direction, beta_direction > 0,
            theta_target, beta_target, clearance, touchdown_advance,
            bool(other_leg_support_assumed), tuple(frames), False, None, None,
            None, "TOUCHDOWN_TARGET_OUTSIDE_OBSTACLE_TOP",
        )

    final_template = build_single_leg_rolling_scene_2d(
        theta_target, beta_target, 0.0, 0.0, **scene_kwargs
    )
    foot_indices = [
        index
        for index, region in enumerate(final_template.geometry.contact_regions)
        if region == RimId.FOOT.value
    ]
    if not foot_indices:
        raise RuntimeError("existing sampled geometry contains no foot-rim samples.")
    landing_sample = min(
        foot_indices,
        key=lambda index: (
            final_template.geometry.points_hip_xz_m[index, 1],
            abs(final_template.geometry.alpha_rad[index] - alpha_target),
        ),
    )
    landing_local = final_template.geometry.points_hip_xz_m[landing_sample]
    landing_hip = np.array(
        [touchdown_x - landing_local[0], top_height - landing_local[1]],
        dtype=float,
    )

    start_template = build_single_leg_rolling_scene_2d(
        start.theta_rad, start.beta_rad, start_hip[0], 0.0, **scene_kwargs
    )
    start_local_min = _minimum_scene_height_above_obstacle_top_2d(start_template)
    lifted_start_z = max(
        start_hip[1] + clearance,
        clearance - start_local_min,
    )
    if lifted_start_z - start_hip[1] > max_lift + 1e-12:
        return AirborneRetractResetBranchResult2D(
            rolling_result, branch, beta_direction, beta_direction > 0,
            theta_target, beta_target, clearance, touchdown_advance,
            bool(other_leg_support_assumed), tuple(frames), False, None, None,
            None, "MAX_HIP_LIFT_EXCEEDED_AT_TAKEOFF",
        )

    lift_steps = max(1, int(np.ceil((lifted_start_z - start_hip[1]) / vertical_step)))
    for index in range(1, lift_steps + 1):
        ratio = index / lift_steps
        hip_z = float(start_hip[1] + ratio * (lifted_start_z - start_hip[1]))
        scene = build_single_leg_rolling_scene_2d(
            start.theta_rad, start.beta_rad, start_hip[0], hip_z, **scene_kwargs
        )
        if not append_frame(
            scene, "LIFTOFF", 0.0, contact_required=False,
        ):
            frames[-1] = replace(
                frames[-1], failure_reason="LIFTOFF_COLLISION_OR_JOINT_LIMIT"
            )
            return AirborneRetractResetBranchResult2D(
                rolling_result, branch, beta_direction, beta_direction > 0,
                theta_target, beta_target, clearance, touchdown_advance,
                bool(other_leg_support_assumed), tuple(frames), False, None, None,
                None, "LIFTOFF_COLLISION_OR_JOINT_LIMIT",
            )

    motion_steps = max(
        1,
        int(np.ceil(abs(start.theta_rad - theta_target) / theta_step)),
        int(np.ceil(required_rotation / beta_step)),
    )
    for index in range(1, motion_steps + 1):
        ratio = index / motion_steps
        theta = float(start.theta_rad + ratio * (theta_target - start.theta_rad))
        beta = float(start.beta_rad + ratio * (beta_target - start.beta_rad))
        hip_x = float(start_hip[0] + ratio * (landing_hip[0] - start_hip[0]))
        template = build_single_leg_rolling_scene_2d(
            theta, beta, hip_x, 0.0, **scene_kwargs
        )
        local_min = _minimum_scene_height_above_obstacle_top_2d(template)
        hip_z = float(clearance - local_min)
        if hip_z - start_hip[1] > max_lift + 1e-12:
            append_frame(
                build_single_leg_rolling_scene_2d(
                    theta, beta, hip_x, hip_z, **scene_kwargs
                ),
                "AIRBORNE_RETRACT_RESET",
                abs(beta - start.beta_rad),
                contact_required=False,
                failure_reason="MAX_HIP_LIFT_EXCEEDED_DURING_AIRBORNE_SWEEP",
            )
            return AirborneRetractResetBranchResult2D(
                rolling_result, branch, beta_direction, beta_direction > 0,
                theta_target, beta_target, clearance, touchdown_advance,
                bool(other_leg_support_assumed), tuple(frames), False, None, None,
                None, "MAX_HIP_LIFT_EXCEEDED_DURING_AIRBORNE_SWEEP",
            )
        scene = build_single_leg_rolling_scene_2d(
            theta, beta, hip_x, hip_z, **scene_kwargs
        )
        if not append_frame(
            scene,
            "AIRBORNE_RETRACT_RESET",
            abs(beta - start.beta_rad),
            contact_required=False,
        ):
            frames[-1] = replace(
                frames[-1],
                failure_reason="AIRBORNE_SWEEP_COLLISION_OR_JOINT_LIMIT",
            )
            return AirborneRetractResetBranchResult2D(
                rolling_result, branch, beta_direction, beta_direction > 0,
                theta_target, beta_target, clearance, touchdown_advance,
                bool(other_leg_support_assumed), tuple(frames), False, None, None,
                None, "AIRBORNE_SWEEP_COLLISION_OR_JOINT_LIMIT",
            )

    lifted_landing_z = frames[-1].hip_position_world_xz_m[1]
    touchdown_steps = max(
        1, int(np.ceil((lifted_landing_z - landing_hip[1]) / vertical_step))
    )
    target = np.array([touchdown_x, top_height], dtype=float)
    for index in range(1, touchdown_steps + 1):
        ratio = index / touchdown_steps
        hip_z = float(lifted_landing_z + ratio * (landing_hip[1] - lifted_landing_z))
        scene = build_single_leg_rolling_scene_2d(
            theta_target, beta_target, landing_hip[0], hip_z, **scene_kwargs
        )
        query_result = query_single_leg_rolling_scene_2d(
            scene,
            contact_tolerance_m=contact_tolerance_m,
            collision_tolerance_m=collision_tolerance_m,
        )
        is_final = index == touchdown_steps
        candidate = (
            _select_foot_top_candidate_2d(
                query_result,
                scene.terrain,
                target,
                alpha_target_rad=alpha_target,
                alpha_tolerance_rad=alpha_tolerance,
            )
            if is_final
            else None
        )
        phase = "TOUCHDOWN" if is_final else "FOOT_ALIGN_DESCENT"
        failure = "FOOT_RIM_TOUCHDOWN_INVALID" if is_final else "DESCENT_COLLISION"
        if not append_frame(
            scene,
            phase,
            required_rotation,
            contact_required=is_final,
            contact_candidate=candidate,
        ):
            frames[-1] = replace(frames[-1], failure_reason=failure)
            return AirborneRetractResetBranchResult2D(
                rolling_result, branch, beta_direction, beta_direction > 0,
                theta_target, beta_target, clearance, touchdown_advance,
                bool(other_leg_support_assumed), tuple(frames), False, None, None,
                None, failure,
            )

    success = frames[-1].foot_rim_ready
    maximum_lift = max(frame.hip_vertical_displacement_m for frame in frames)
    return AirborneRetractResetBranchResult2D(
        rolling_result=rolling_result,
        branch=branch,
        beta_direction=beta_direction,
        direction_reversal=beta_direction > 0,
        theta_target_rad=theta_target,
        beta_target_rad=beta_target,
        airborne_clearance_m=clearance,
        touchdown_contact_advance_m=touchdown_advance,
        other_leg_support_assumed=bool(other_leg_support_assumed),
        frames=tuple(frames),
        success=success,
        required_rotation_rad=required_rotation if success else None,
        required_hip_forward_distance_m=(
            frames[-1].hip_forward_displacement_m if success else None
        ),
        maximum_hip_lift_m=maximum_lift if success else None,
        failure_reason=None if success else "FOOT_RIM_TOUCHDOWN_INVALID",
    )


def run_airborne_retract_and_foot_reset_comparison_2d(
    rolling_result: ForwardRollingResult2D,
    **branch_kwargs,
) -> AirborneRetractResetComparison2D:
    """Run both airborne rotation directions and select the shorter success."""

    branches = tuple(
        run_airborne_retract_and_foot_reset_branch_2d(
            rolling_result, branch=branch, **branch_kwargs
        )
        for branch in ("forward_continuation", "shortest_reverse")
    )
    feasible = [item for item in branches if item.success]
    selected = min(
        feasible,
        key=lambda item: (
            item.required_rotation_rad,
            item.maximum_hip_lift_m,
            item.direction_reversal,
        ),
        default=None,
    )
    return AirborneRetractResetComparison2D(
        rolling_result=rolling_result,
        branches=branches,
        selected_branch=None if selected is None else selected.branch,
        selection_basis=(
            "collision-free sampled airborne sweep and valid foot-rim top "
            "touchdown; then minimum accumulated |delta beta| with hip-lift tie-break"
        ),
        stability_scope=(
            "single-leg 2D kinematics only; other three legs supporting the body "
            "is assumed and whole-body stability/dynamics are not evaluated"
        ),
    )


def airborne_retract_reset_comparison_rows(
    result: AirborneRetractResetComparison2D,
) -> list[dict]:
    """Return one Step-6.75 summary row per airborne rotation branch."""

    if not isinstance(result, AirborneRetractResetComparison2D):
        raise TypeError("result must be AirborneRetractResetComparison2D.")
    rows = []
    for branch in result.branches:
        final = branch.final_frame
        rows.append(
            {
                "branch": branch.branch,
                "selected": branch.branch == result.selected_branch,
                "success": branch.success,
                "direction_reversal": branch.direction_reversal,
                "required_rotation_rad": branch.required_rotation_rad,
                "required_rotation_deg": branch.required_rotation_deg,
                "required_hip_forward_distance_m": (
                    branch.required_hip_forward_distance_m
                ),
                "maximum_hip_lift_m": branch.maximum_hip_lift_m,
                "airborne_clearance_m": branch.airborne_clearance_m,
                "touchdown_contact_advance_m": (
                    branch.touchdown_contact_advance_m
                ),
                "frames": len(branch.frames),
                "final_phase": final.phase,
                "final_theta_deg": float(np.rad2deg(final.theta_rad)),
                "final_beta_unwrapped_deg": float(
                    np.rad2deg(final.beta_unwrapped_rad)
                ),
                "final_rim": final.active_rim,
                "final_alpha_deg": (
                    None
                    if final.alpha_rad is None
                    else float(np.rad2deg(final.alpha_rad))
                ),
                "final_surface": final.terrain_surface_id,
                "foot_rim_ready": final.foot_rim_ready,
                "minimum_sampled_airborne_clearance_m": (
                    min(airborne_clearances)
                    if (
                        airborne_clearances := [
                            frame.minimum_clearance_above_top_m
                            for frame in branch.frames
                            if frame.phase == "AIRBORNE_RETRACT_RESET"
                        ]
                    )
                    else None
                ),
                "collision_free": all(not frame.collision for frame in branch.frames),
                "other_leg_support_assumed": branch.other_leg_support_assumed,
                "failure_reason": branch.failure_reason,
                "selection_basis": result.selection_basis,
                "stability_scope": result.stability_scope,
            }
        )
    return rows


def airborne_retract_reset_frame_rows(
    result: AirborneRetractResetComparison2D,
) -> list[dict]:
    """Return every sampled Step-6.75 frame from both branches."""

    if not isinstance(result, AirborneRetractResetComparison2D):
        raise TypeError("result must be AirborneRetractResetComparison2D.")
    rows = []
    for branch in result.branches:
        for frame in branch.frames:
            rows.append(
                {
                    "branch": branch.branch,
                    "selected": branch.branch == result.selected_branch,
                    "step": frame.step,
                    "phase": frame.phase,
                    "theta_rad": frame.theta_rad,
                    "theta_deg": float(np.rad2deg(frame.theta_rad)),
                    "beta_rad": frame.beta_rad,
                    "beta_unwrapped_deg": float(
                        np.rad2deg(frame.beta_unwrapped_rad)
                    ),
                    "accumulated_rotation_deg": float(
                        np.rad2deg(frame.accumulated_rotation_rad)
                    ),
                    "hip_x_m": frame.hip_position_world_xz_m[0],
                    "hip_z_m": frame.hip_position_world_xz_m[1],
                    "hip_forward_displacement_m": frame.hip_forward_displacement_m,
                    "hip_vertical_displacement_m": frame.hip_vertical_displacement_m,
                    "active_rim": frame.active_rim,
                    "active_sample_index": frame.active_sample_index,
                    "alpha_rad": frame.alpha_rad,
                    "contact_point_world_xz_m": frame.contact_point_world_xz_m,
                    "terrain_surface_id": frame.terrain_surface_id,
                    "contact_required": frame.contact_required,
                    "valid_contact": frame.valid_contact,
                    "collision": frame.collision,
                    "joint_limits_ok": frame.joint_limits_ok,
                    "minimum_clearance_above_top_m": (
                        frame.minimum_clearance_above_top_m
                    ),
                    "foot_rim_ready": frame.foot_rim_ready,
                    "accepted": frame.accepted,
                    "failure_reason": frame.failure_reason,
                }
            )
    return rows


def write_airborne_retract_reset_comparison_csv(
    result: AirborneRetractResetComparison2D,
    summary_path: str | Path,
    frame_path: str | Path,
) -> tuple[Path, Path]:
    """Save Step-6.75 branch summaries and all sampled frames."""

    outputs = []
    for path, rows in (
        (summary_path, airborne_retract_reset_comparison_rows(result)),
        (frame_path, airborne_retract_reset_frame_rows(result)),
    ):
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        outputs.append(output_path)
    return tuple(outputs)


def plot_airborne_retract_reset_comparison_2d(
    result: AirborneRetractResetComparison2D,
    *,
    axes=None,
    show: bool = False,
):
    """Plot both Step-6.75 hip paths and their final touchdown poses."""

    if not isinstance(result, AirborneRetractResetComparison2D):
        raise TypeError("result must be AirborneRetractResetComparison2D.")
    if axes is None:
        _, axes = plt.subplots(1, len(result.branches), figsize=(15, 5.5))
    axes = np.atleast_1d(axes)
    if len(axes) != len(result.branches):
        raise ValueError("axes must contain one axis per branch.")
    for ax, branch in zip(axes, result.branches):
        final = branch.final_frame
        plot_single_leg_rolling_scene_2d(
            final.scene, ax=ax, query_result=final.query_result
        )
        hips = np.asarray(
            [frame.hip_position_world_xz_m for frame in branch.frames], dtype=float
        )
        ax.plot(
            hips[:, 0], hips[:, 1], "--", color="#7c3aed", linewidth=2,
            label="airborne hip path",
        )
        takeoff = branch.frames[0].contact_point_world_xz_m
        touchdown = final.contact_point_world_xz_m
        if takeoff is not None:
            ax.scatter(
                *takeoff, color="#2563eb", s=55, marker="o",
                label="right-rim takeoff", zorder=15,
            )
        if touchdown is not None:
            ax.scatter(
                *touchdown, color="#16a34a", s=85, marker="*",
                label="foot-rim touchdown", zorder=15,
            )
        selected = " [SELECTED]" if branch.branch == result.selected_branch else ""
        ax.set_title(
            f"Step 6.75 {branch.branch}{selected}\n"
            f"success={branch.success}, rotation="
            f"{branch.required_rotation_deg if branch.success else float('nan'):.1f} deg"
        )
        ax.legend(fontsize=7, loc="best")
    if show:
        plt.show()
    return axes


def animate_roll_up_and_airborne_reset_2d(
    result: AirborneRetractResetBranchResult2D,
    *,
    interval_ms: int = 100,
    roll_frame_stride: int = 1,
    airborne_frame_stride: int = 1,
    repeat: bool = False,
    show: bool = False,
):
    """Animate successful Step 4.5 roll-up followed by one Step-6.75 branch."""

    if not isinstance(result, AirborneRetractResetBranchResult2D):
        raise TypeError("result must be AirborneRetractResetBranchResult2D.")
    if min(interval_ms, roll_frame_stride, airborne_frame_stride) <= 0:
        raise ValueError("animation interval and frame strides must be positive.")
    roll_frames = list(result.rolling_result.frames[::roll_frame_stride])
    if roll_frames[-1] is not result.rolling_result.frames[-1]:
        roll_frames.append(result.rolling_result.frames[-1])
    airborne_frames = list(result.frames[1::airborne_frame_stride])
    if not airborne_frames or airborne_frames[-1] is not result.frames[-1]:
        airborne_frames.append(result.frames[-1])
    display_frames = [("ROLL_UP", frame) for frame in roll_frames]
    display_frames.extend(("STEP_6_75", frame) for frame in airborne_frames)
    all_points = np.vstack(
        [
            np.vstack(
                (
                    frame.scene.geometry.points_world_xz_m,
                    frame.scene.hip_pose.position_world_xz_m[None, :],
                )
            )
            for _, frame in display_frames
        ]
    )
    x_pad = max(0.04, 0.08 * float(np.ptp(all_points[:, 0])))
    z_pad = max(0.04, 0.08 * float(np.ptp(all_points[:, 1])))
    x_limits = (
        float(np.min(all_points[:, 0]) - x_pad),
        float(np.max(all_points[:, 0]) + x_pad),
    )
    z_limits = (
        min(
            display_frames[0][1].scene.terrain.ground_height_m - 0.025,
            float(np.min(all_points[:, 1]) - z_pad),
        ),
        float(np.max(all_points[:, 1]) + z_pad),
    )
    figure, ax = plt.subplots(figsize=(11, 5.5))

    def draw_frame(index: int):
        source, frame = display_frames[index]
        ax.clear()
        plot_single_leg_rolling_scene_2d(
            frame.scene, ax=ax, query_result=frame.query_result
        )
        ax.set_xlim(*x_limits)
        ax.set_ylim(*z_limits)
        hip_trace = np.asarray(
            [
                item.scene.hip_pose.position_world_xz_m
                for _, item in display_frames[: index + 1]
            ],
            dtype=float,
        )
        ax.plot(
            hip_trace[:, 0], hip_trace[:, 1], "--", color="#7c3aed",
            linewidth=1.6, label="hip path",
        )
        phase = frame.roll_phase if source == "ROLL_UP" else frame.phase
        theta = frame.theta_rad
        beta = frame.beta_rad
        active_rim = frame.active_rim
        collision = frame.collision
        ax.set_title(
            f"Step 4.5 + 6.75 {result.branch}: {phase}\n"
            f"theta={np.rad2deg(theta):.1f} deg, "
            f"beta={np.rad2deg(beta):.1f} deg, rim={active_rim or 'airborne'}"
        )
        if source == "ROLL_UP":
            detail = (
                f"top-roll progress={frame.top_roll_progress_m or 0.0:.4f} m\n"
                f"contact required=True\ncollision={collision}"
            )
        else:
            detail = (
                f"phase={frame.phase}\n"
                f"hip dx={frame.hip_forward_displacement_m:.4f} m, "
                f"hip dz={frame.hip_vertical_displacement_m:.4f} m\n"
                f"rotation={np.rad2deg(frame.accumulated_rotation_rad):.1f} deg\n"
                f"top clearance={frame.minimum_clearance_above_top_m:.4f} m\n"
                f"contact required={frame.contact_required}, "
                f"foot ready={frame.foot_rim_ready}\n"
                f"collision={collision}, failure={result.failure_reason or 'none'}"
            )
        ax.text(
            0.01, 0.02, detail, transform=ax.transAxes, fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.86, "edgecolor": "0.75"},
            zorder=16,
        )
        return tuple(ax.lines)

    animation = FuncAnimation(
        figure,
        draw_frame,
        frames=len(display_frames),
        interval=interval_ms,
        repeat=repeat,
        blit=False,
    )
    draw_frame(0)
    if show:
        plt.show()
    return animation


def simulate_theta_candidates_forward_rolling_2d(
    theta_candidates_rad,
    initial_beta_rad: float,
    hip_x_m: float,
    hip_z_m: float,
    **simulation_kwargs,
) -> ForwardRollingSweepResult2D:
    """Run Step4 candidate thetas through the forward simulation."""

    candidates = tuple(float(_finite_scalar(value, "theta_candidate_rad")) for value in theta_candidates_rad)
    if not candidates:
        raise ValueError("theta_candidates_rad must contain at least one theta.")
    simulations = tuple(
        run_forward_right_rim_roll_up_2d(
            candidate_theta_rad=theta,
            initial_beta_rad=initial_beta_rad,
            hip_x_m=hip_x_m,
            hip_z_m=hip_z_m,
            **simulation_kwargs,
        )
        for theta in candidates
    )
    rows = tuple(
        ForwardRollingSweepRow2D(
            candidate_theta_rad=simulation.candidate_theta_rad,
            continuous_roll_up_success=simulation.success,
            failure_reason=simulation.failure_reason,
            final_hip_x_m=simulation.final_hip_x_m,
            final_theta_rad=simulation.final_theta_rad,
            final_beta_rad=simulation.final_beta_rad,
            final_hip_forward_progress_m=simulation.final_frame.hip_forward_progress_m,
            final_top_roll_progress_m=simulation.final_frame.top_roll_progress_m,
            final_top_contact_advance_m=simulation.final_frame.top_contact_advance_m,
            final_top_roll_remaining_m=simulation.final_frame.top_roll_remaining_m,
            final_roll_phase=simulation.final_frame.roll_phase,
            final_continuation_target_world_xz_m=(
                simulation.final_frame.continuation_target_world_xz_m
            ),
            final_continuation_error_m=simulation.final_frame.continuation_error_m,
            final_rim=simulation.final_frame.active_rim,
            final_contact_surface=simulation.final_frame.terrain_surface_id,
            frames=len(simulation.frames),
        )
        for simulation in simulations
    )
    return ForwardRollingSweepResult2D(rows=rows, simulations=simulations)


def forward_rolling_rows(result: ForwardRollingSweepResult2D) -> list[dict]:
    """Return one summary row per Step-4 theta candidate."""

    if not isinstance(result, ForwardRollingSweepResult2D):
        raise TypeError("result must be ForwardRollingSweepResult2D.")
    return [row.as_dict() for row in result.rows]


def forward_rolling_frame_rows(result: ForwardRollingResult2D) -> list[dict]:
    """Return every accepted/rejected frame of one forward simulation."""

    if not isinstance(result, ForwardRollingResult2D):
        raise TypeError("result must be ForwardRollingResult2D.")
    return [
        {
            "step": frame.step,
            "hip_x_m": frame.hip_x_m,
            "hip_z_m": float(frame.scene.hip_pose.position_world_xz_m[1]),
            "hip_forward_progress_m": frame.hip_forward_progress_m,
            "theta_rad": frame.theta_rad,
            "theta_deg": np.rad2deg(frame.theta_rad),
            "beta_rad": frame.beta_rad,
            "beta_deg": np.rad2deg(frame.beta_rad),
            "active_rim": frame.active_rim,
            "alpha_rad": frame.alpha_rad,
            "contact_point_world_xz_m": frame.contact_point_world_xz_m,
            "continuation_target_world_xz_m": frame.continuation_target_world_xz_m,
            "continuation_error_m": frame.continuation_error_m,
            "leading_edge_clearance_m": frame.leading_edge_clearance_m,
            "top_roll_progress_m": frame.top_roll_progress_m,
            "top_contact_advance_m": frame.top_contact_advance_m,
            "top_roll_complete": frame.top_roll_complete,
            "top_roll_remaining_m": frame.top_roll_remaining_m,
            "roll_phase": frame.roll_phase,
            "terrain_surface_id": frame.terrain_surface_id,
            "contact_phase": frame.contact_phase,
            "valid_contact": frame.valid_contact,
            "collision": frame.collision,
            "status": frame.status.value,
            "accepted": frame.accepted,
            "failure_reason": frame.failure_reason,
        }
        for frame in result.frames
    ]


def write_forward_rolling_csv(
    result: ForwardRollingSweepResult2D,
    path: str | Path,
) -> Path:
    """Write the forward-simulation summary rows to CSV."""

    if not isinstance(result, ForwardRollingSweepResult2D):
        raise TypeError("result must be ForwardRollingSweepResult2D.")
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rows = forward_rolling_rows(result)
    fieldnames = list(rows[0].keys())
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return output_path


def plot_forward_rolling_trajectory_2d(
    result: ForwardRollingSweepResult2D,
    *,
    ax=None,
    show: bool = False,
):
    """Plot world-frame contact paths and hip forward motion."""

    if not isinstance(result, ForwardRollingSweepResult2D):
        raise TypeError("result must be ForwardRollingSweepResult2D.")
    if ax is None:
        _, ax = plt.subplots(figsize=(10, 5.5))
    if not result.simulations:
        return ax
    reference_scene = result.simulations[0].final_frame.scene
    x_values = [frame.hip_x_m for simulation in result.simulations for frame in simulation.frames]
    x_values.extend(
        point[0]
        for simulation in result.simulations
        for frame in simulation.frames
        if (point := frame.contact_point_world_xz_m) is not None
    )
    x_min = min(x_values) - 0.05
    x_max = max(x_values) + 0.08
    plot_terrain_profile_2d(reference_scene.terrain, ax=ax, x_limits_m=(x_min, x_max))
    for simulation in result.simulations:
        points = [frame.contact_point_world_xz_m for frame in simulation.frames if frame.contact_point_world_xz_m is not None]
        if not points:
            continue
        points_array = np.asarray(points, dtype=float)
        color = "#16a34a" if simulation.success else "#dc2626"
        label = f"theta={np.rad2deg(simulation.candidate_theta_rad):.1f} deg"
        ax.plot(points_array[:, 0], points_array[:, 1], color=color, linewidth=2.0, marker="o", markersize=3.5, label=label)
    for simulation in result.simulations:
        hip_points = np.asarray(
            [frame.scene.hip_pose.position_world_xz_m for frame in simulation.frames],
            dtype=float,
        )
        color = "#16a34a" if simulation.success else "#dc2626"
        ax.plot(
            hip_points[:, 0],
            hip_points[:, 1],
            linestyle="--",
            color=color,
            linewidth=1.2,
            alpha=0.7,
            label=f"hip path ({np.rad2deg(simulation.candidate_theta_rad):.1f} deg)",
        )
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world z [m]")
    ax.set_title("Day 6--7 Step 4.5: forward rolling contact trajectory")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(fontsize=8, loc="best")
    if show:
        plt.show()
    return ax


def animate_forward_rolling_2d(
    result: ForwardRollingResult2D,
    *,
    interval_ms: int = 700,
    repeat: bool = False,
    show: bool = False,
):
    """Animate the recorded forward-rolling frames of one theta candidate.

    The animation includes accepted contact frames and the first rejected
    frame, so a short roll-up attempt can visibly end at its collision or
    continuity failure instead of hiding the failure state.
    """

    if not isinstance(result, ForwardRollingResult2D):
        raise TypeError("result must be ForwardRollingResult2D.")
    if not result.frames:
        raise ValueError("result must contain at least one frame.")
    if interval_ms <= 0:
        raise ValueError("interval_ms must be positive.")

    figure, ax = plt.subplots(figsize=(11, 5.5))
    trajectory_points = np.vstack(
        [
            np.vstack(
                (
                    frame.scene.geometry.points_world_xz_m,
                    frame.scene.hip_pose.position_world_xz_m[None, :],
                )
            )
            for frame in result.frames
        ]
    )
    x_span = float(np.ptp(trajectory_points[:, 0]))
    z_span = float(np.ptp(trajectory_points[:, 1]))
    animation_x_limits = (
        float(np.min(trajectory_points[:, 0]) - max(0.035, 0.08 * x_span)),
        float(np.max(trajectory_points[:, 0]) + max(0.035, 0.08 * x_span)),
    )
    animation_z_limits = (
        min(
            result.frames[0].scene.terrain.ground_height_m - 0.025,
            float(np.min(trajectory_points[:, 1]) - 0.025),
        ),
        float(np.max(trajectory_points[:, 1]) + max(0.035, 0.10 * z_span)),
    )

    def draw_frame(frame_index: int):
        frame = result.frames[frame_index]
        ax.clear()
        plot_single_leg_rolling_scene_2d(
            frame.scene,
            ax=ax,
            query_result=frame.query_result,
        )
        # Keep one trajectory-wide zoom instead of fitting the long obstacle
        # top in every frame; this makes the leg, hip origin, and rim motion
        # readable without a distracting axis jump between frames.
        ax.set_xlim(*animation_x_limits)
        ax.set_ylim(*animation_z_limits)
        trace = [
            item.contact_point_world_xz_m
            for item in result.frames[: frame_index + 1]
            if item.contact_point_world_xz_m is not None
        ]
        if trace:
            points = np.asarray(trace, dtype=float)
            ax.plot(
                points[:, 0],
                points[:, 1],
                color="#0f766e",
                linewidth=2.0,
                marker="o",
                markersize=4.0,
                label="contact-point trace",
                zorder=13,
            )
        ax.set_title(
            f"Step 4.5 forward rolling: frame {frame.step} / {len(result.frames) - 1}\n"
            f"hip=({frame.hip_x_m:.4f}, "
            f"{frame.scene.hip_pose.position_world_xz_m[1]:.4f}) m, "
            f"theta={np.rad2deg(frame.theta_rad):.2f} deg, "
            f"beta={np.rad2deg(frame.beta_rad):.2f} deg"
        )
        clearance_text = (
            f"{frame.leading_edge_clearance_m * 1e3:.2f} mm"
            if frame.leading_edge_clearance_m is not None
            else "none"
        )
        top_progress_text = (
            f"{frame.top_roll_progress_m * 1e3:.2f} mm"
            if frame.top_roll_progress_m is not None
            else "none"
        )
        continuation_error_text = (
            f"{frame.continuation_error_m * 1e3:.2f} mm"
            if frame.continuation_error_m is not None
            else "none"
        )
        top_remaining_text = (
            f"{frame.top_roll_remaining_m * 1e3:.2f} mm"
            if frame.top_roll_remaining_m is not None
            else "none"
        )
        ax.text(
            0.01,
            0.02,
            f"contact phase={frame.contact_phase or 'none'}, "
            f"roll phase={frame.roll_phase}\n"
            f"accepted={frame.accepted}, collision={frame.collision}\n"
            f"hip progress={frame.hip_forward_progress_m * 1e3:.2f} mm, "
            f"hip rise={(frame.scene.hip_pose.position_world_xz_m[1] - result.frames[0].scene.hip_pose.position_world_xz_m[1]) * 1e3:.2f} mm\n"
            f"leading clearance={clearance_text}\n"
            f"top-contact progress={top_progress_text}, "
            f"target={result.top_roll_distance_m * 1e3:.2f} mm, "
            f"remaining={top_remaining_text}\n"
            f"continuation error={continuation_error_text}\n"
            f"complete={frame.top_roll_complete}\n"
            f"failure={frame.failure_reason or 'none'}",
            transform=ax.transAxes,
            fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.86, "edgecolor": "0.75"},
            zorder=16,
        )
        return tuple(ax.lines)

    animation = FuncAnimation(
        figure,
        draw_frame,
        frames=len(result.frames),
        interval=interval_ms,
        repeat=repeat,
        blit=False,
    )
    # Keep the first frame available when the animation is displayed in a
    # notebook backend that does not immediately render the HTML animation.
    draw_frame(0)
    if show:
        plt.show()
    return animation


def sweep_right_rim_roll_up_theta_2d(
    theta_min_rad: float,
    theta_max_rad: float,
    dtheta_rad: float,
    initial_beta_rad: float,
    hip_x_m: float,
    hip_z_m: float,
    *,
    beta_step_rad: float = -np.deg2rad(1.0),
    max_steps: int = 12,
    gamma_rad: float = 0.0,
    ground_height_m: float = 0.0,
    obstacle_x_start_m: float = 0.10,
    obstacle_width_m: float = 0.60,
    obstacle_height_m: float = 0.10,
    obstacle_id: str = "day6_7_step4_obstacle",
    arc_samples: int = 241,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
    forward_progress_tolerance_m: float = 2e-4,
) -> RightRimThetaSweepResult2D:
    """Brute-force theta-climb sweep using the existing Step3 runner.

    Every theta uses the same initial beta, hip pose, beta update, terrain,
    and query tolerances.  The function only aggregates the existing
    fixed-theta roll-up result; it does not optimize, recover, retract, or
    change obstacle height.
    """

    theta_min = _finite_scalar(theta_min_rad, "theta_min_rad")
    theta_max = _finite_scalar(theta_max_rad, "theta_max_rad")
    dtheta = _finite_scalar(dtheta_rad, "dtheta_rad")
    if theta_max < theta_min:
        raise ValueError("theta_max_rad must be greater than or equal to theta_min_rad.")
    if dtheta <= 0.0:
        raise ValueError("dtheta_rad must be positive.")

    span = theta_max - theta_min
    count = int(np.floor(span / dtheta + 1e-12))
    theta_values = [theta_min + index * dtheta for index in range(count + 1)]
    if not np.isclose(theta_values[-1], theta_max, atol=1e-12):
        # Include the requested upper bound even when the interval is not an
        # exact multiple of dtheta; the last interval may be shorter.
        theta_values.append(theta_max)
    else:
        theta_values[-1] = theta_max

    rows = []
    for theta in theta_values:
        roll_up = run_fixed_theta_right_rim_roll_up_2d(
            theta_climb_rad=theta,
            initial_beta_rad=initial_beta_rad,
            hip_x_m=hip_x_m,
            hip_z_m=hip_z_m,
            beta_step_rad=beta_step_rad,
            max_steps=max_steps,
            gamma_rad=gamma_rad,
            ground_height_m=ground_height_m,
            obstacle_x_start_m=obstacle_x_start_m,
            obstacle_width_m=obstacle_width_m,
            obstacle_height_m=obstacle_height_m,
            obstacle_id=obstacle_id,
            arc_samples=arc_samples,
            contact_tolerance_m=contact_tolerance_m,
            collision_tolerance_m=collision_tolerance_m,
            forward_progress_tolerance_m=forward_progress_tolerance_m,
        )
        final_frame = roll_up.final_frame
        rows.append(
            RightRimThetaSweepRow2D(
                theta_climb_rad=theta,
                roll_up_success=roll_up.success,
                failure_reason=roll_up.failure_reason,
                final_beta_rad=roll_up.final_beta_rad,
                final_rim=final_frame.active_rim,
                final_contact_surface=final_frame.terrain_surface_id,
            )
        )

    return RightRimThetaSweepResult2D(
        theta_min_rad=theta_min,
        theta_max_rad=theta_max,
        dtheta_rad=dtheta,
        rows=tuple(rows),
    )


def right_rim_theta_sweep_rows(result: RightRimThetaSweepResult2D) -> list[dict]:
    """Return theta-sweep rows for pandas, display, or CSV export."""

    if not isinstance(result, RightRimThetaSweepResult2D):
        raise TypeError("result must be RightRimThetaSweepResult2D.")
    return [row.as_dict() for row in result.rows]


def write_right_rim_theta_sweep_csv(
    result: RightRimThetaSweepResult2D,
    path: str | Path,
) -> Path:
    """Write one CSV row per theta value and return the written path."""

    if not isinstance(result, RightRimThetaSweepResult2D):
        raise TypeError("result must be RightRimThetaSweepResult2D.")
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rows = right_rim_theta_sweep_rows(result)
    fieldnames = list(rows[0].keys()) if rows else [
        "theta_climb_rad", "theta_climb_deg", "roll_up_success",
        "failure_reason", "final_beta_rad", "final_beta_deg",
        "final_rim", "final_contact_surface",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return output_path


def plot_right_rim_theta_feasibility_2d(
    result: RightRimThetaSweepResult2D,
    *,
    ax=None,
    show: bool = False,
):
    """Plot theta against binary fixed-theta roll-up feasibility."""

    if not isinstance(result, RightRimThetaSweepResult2D):
        raise TypeError("result must be RightRimThetaSweepResult2D.")
    if ax is None:
        _, ax = plt.subplots(figsize=(9, 4.5))
    theta_deg = [row.theta_climb_deg for row in result.rows]
    feasible = [row.roll_up_success for row in result.rows]
    ax.scatter(
        [theta for theta, ok in zip(theta_deg, feasible) if ok],
        [1] * sum(feasible),
        color="#16a34a",
        s=58,
        label="roll-up success",
        zorder=3,
    )
    ax.scatter(
        [theta for theta, ok in zip(theta_deg, feasible) if not ok],
        [0] * sum(not ok for ok in feasible),
        color="#dc2626",
        marker="x",
        s=70,
        linewidths=1.8,
        label="roll-up failure",
        zorder=3,
    )
    ax.set_xlabel(r"$\theta_{climb}$ [deg]")
    ax.set_ylabel("roll-up feasible")
    ax.set_yticks([0, 1], labels=["failure", "success"])
    ax.set_ylim(-0.25, 1.25)
    ax.grid(axis="y", alpha=0.3)
    ax.set_title("Day 6--7 Step 4: fixed-condition theta-climb feasibility")
    ax.legend(loc="best")
    if show:
        plt.show()
    return ax


def plot_right_rim_roll_up_trajectory_2d(
    result: RightRimRollUpResult2D,
    *,
    show: bool = False,
) -> tuple[object, ...]:
    """Create one independent figure per recorded frame, in sequence."""

    if not isinstance(result, RightRimRollUpResult2D):
        raise TypeError("result must be RightRimRollUpResult2D.")
    figures = []
    for frame in result.frames:
        figure, ax = plt.subplots(figsize=(11, 5.5))
        plot_single_leg_rolling_scene_2d(
            frame.scene,
            ax=ax,
            query_result=frame.query_result,
        )
        ax.set_title(
            f"Step 3 frame {frame.step}: beta={np.rad2deg(frame.beta_rad):.2f} deg\n"
            f"{frame.status.value}, accepted={frame.accepted}"
        )
        ax.text(
            0.01,
            0.02,
            f"failure_reason={frame.failure_reason or 'none'}",
            transform=ax.transAxes,
            fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "0.75"},
        )
        figure.tight_layout()
        figures.append(figure)
    if show:
        plt.show()
    return tuple(figures)


def plot_single_leg_rolling_scene_2d(
    scene: SingleLegRollingScene2D,
    *,
    ax=None,
    show_non_contact_endpoints: bool = True,
    query_result: ContactQueryResult2D | None = None,
):
    """Draw terrain, reused leg geometry, rims, and optional query overlays."""

    if not isinstance(scene, SingleLegRollingScene2D):
        raise TypeError("scene must be a SingleLegRollingScene2D.")
    if ax is None:
        _, ax = plt.subplots(figsize=(11, 5.5))

    x_limits = _plot_x_limits(scene)
    plot_terrain_profile_2d(scene.terrain, ax=ax, x_limits_m=x_limits)

    # PlotLeg owns the existing full linkage/wheel visualization.  The 2D
    # scene fixes the hip pitch to zero, so translating its origin is enough.
    leg = PlotLeg()
    leg.plot_leg(
        scene.theta_rad,
        scene.beta_rad,
        scene.hip_pose.position_world_xz_m,
        ax,
    )

    points_world = scene.geometry.points_world_xz_m
    surface_names = np.asarray(scene.geometry.surface_names)
    contact_regions = np.asarray(scene.geometry.contact_regions)

    for surface_name, color in RIM_SURFACE_COLORS.items():
        mask = surface_names == surface_name
        if not np.any(mask):
            continue
        ax.plot(
            points_world[mask, 0],
            points_world[mask, 1],
            color=color,
            marker=".",
            markersize=2.2,
            linewidth=1.0,
            alpha=0.95,
            zorder=5,
            label=RIM_SURFACE_LABELS[surface_name],
        )

    if show_non_contact_endpoints:
        for region, color in CONTACT_REGION_COLORS.items():
            if region == "non_contact_region":
                mask = contact_regions == region
                if np.any(mask):
                    ax.scatter(
                        points_world[mask, 0],
                        points_world[mask, 1],
                        s=28,
                        color=color,
                        marker="o",
                        zorder=6,
                        label="open gap endpoints (N)",
                    )

    hip = scene.hip_pose.position_world_xz_m
    ax.scatter(
        hip[0],
        hip[1],
        marker="X",
        s=90,
        color="black",
        edgecolor="white",
        linewidth=0.7,
        zorder=8,
        label="hip H",
    )
    ax.annotate("H", xy=hip, xytext=(6, 7), textcoords="offset points", fontsize=10)

    if query_result is not None:
        if not isinstance(query_result, ContactQueryResult2D):
            raise TypeError("query_result must be a ContactQueryResult2D or None.")
        for candidate in query_result.candidates:
            point = candidate.point_world_xz_m
            terrain_point = candidate.terrain_point_world_xz_m
            status = candidate_status_2d(candidate, scene.terrain)
            color = "#059669" if status is not ContactStatus2D.VALID_OTHER_RIM_CONTACT else "#0891b2"
            ax.plot(
                [point[0], terrain_point[0]],
                [point[1], terrain_point[1]],
                linestyle="--",
                color=color,
                linewidth=1.0,
                alpha=0.75,
                zorder=8,
            )
            ax.scatter(
                point[0],
                point[1],
                marker="o",
                s=74,
                facecolor=color,
                edgecolor="white",
                linewidth=0.8,
                zorder=9,
                label="valid contact candidate"
                if candidate is query_result.candidates[0]
                else None,
            )
            ax.annotate(
                f"{status.value}\n{candidate.rim.value}\n"
                f"{candidate.terrain_surface_id}, gap={candidate.terrain_gap_m * 1e3:.2f} mm",
                point,
                xytext=(5, 7),
                textcoords="offset points",
                fontsize=7,
                color="#065f46",
                zorder=12,
            )

        for penetration in query_result.geometry_penetrations:
            point = penetration.point_world_xz_m
            ax.scatter(
                point[0],
                point[1],
                marker="x",
                s=88,
                color="#7c3aed",
                linewidths=2.2,
                label="invalid geometry penetration"
                if penetration is query_result.geometry_penetrations[0]
                else None,
                zorder=11,
            )
            ax.annotate(
                f"{ContactStatus2D.INVALID_GEOMETRY_PENETRATION.value}\n"
                f"{penetration.rim.value}, pen={penetration.penetration_depth_m * 1e3:.2f} mm",
                point,
                xytext=(5, -22),
                textcoords="offset points",
                fontsize=7,
                color="#5b21b6",
                zorder=12,
            )

        for collision in query_result.collisions:
            point = collision.point_world_xz_m
            ax.scatter(
                point[0],
                point[1],
                marker="x",
                s=88,
                color="#7c3aed",
                linewidths=2.2,
                label="invalid rim penetration"
                if collision is query_result.collisions[0]
                else None,
                zorder=11,
            )
            ax.annotate(
                f"{ContactStatus2D.INVALID_GEOMETRY_PENETRATION.value}\n"
                f"{collision.rim.value}, pen={collision.penetration_depth_m * 1e3:.2f} mm\n"
                f"{collision.terrain_surface_id}",
                point,
                xytext=(5, -22),
                textcoords="offset points",
                fontsize=7,
                color="#5b21b6",
                zorder=12,
            )

        for link_collision in query_result.link_collisions:
            segment = link_collision.segment_world_xz_m
            point = link_collision.point_world_xz_m
            ax.plot(
                segment[:, 0],
                segment[:, 1],
                color="#dc2626",
                linewidth=5.0,
                alpha=0.85,
                label="invalid link collision"
                if link_collision is query_result.link_collisions[0]
                else None,
                zorder=10,
            )
            ax.scatter(
                point[0],
                point[1],
                marker="X",
                s=86,
                color="#dc2626",
                edgecolor="white",
                linewidth=0.8,
                zorder=12,
            )
            ax.annotate(
                f"{ContactStatus2D.INVALID_LINK_COLLISION.value}\n"
                f"{link_collision.geometry_id}, pen={link_collision.penetration_depth_m * 1e3:.2f} mm",
                point,
                xytext=(5, 7),
                textcoords="offset points",
                fontsize=7,
                color="#991b1b",
                zorder=12,
            )

        ax.text(
            0.01,
            0.99,
            "status: " + ", ".join(status.value for status in query_result.statuses)
            + f"\nvalid_contact={query_result.valid_contact}, collision={query_result.collision}",
            transform=ax.transAxes,
            va="top",
            fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "0.75"},
            zorder=13,
        )

    # Terrain labels are already drawn next to their finite segments.  These
    # proxy handles keep the legend explicit without changing terrain data.
    legend_handles = [
        Line2D([], [], color="#111827", linewidth=2, label="ground"),
        Line2D([], [], color="#dc2626", linewidth=3, label="obstacle vertical face(s)"),
        Line2D([], [], color="#16a34a", linewidth=3, label="obstacle top"),
        Line2D([], [], color="#111827", linewidth=1.2, label="linkage / wheel geometry"),
    ]
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(legend_handles + handles, [item.get_label() for item in legend_handles] + labels,
              fontsize=8, loc="best")

    y_values = list(points_world[:, 1]) + [hip[1], scene.terrain.ground_height_m]
    if scene.terrain.obstacle is not None:
        y_values.append(scene.terrain.ground_height_m + scene.terrain.obstacle.height_m)
    y_min, y_max = min(y_values), max(y_values)
    y_padding = max(0.04, 0.12 * max(y_max - y_min, 0.1))
    ax.set_xlim(*x_limits)
    ax.set_ylim(y_min - y_padding, y_max + y_padding)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(
        "Day 6--7 Step 1: fixed 2D single-leg + rectangular obstacle scene\n"
        f"theta={scene.theta_deg:.1f} deg, beta={scene.beta_deg:.1f} deg, gamma=0 deg"
        + (
            "\nquery: " + ", ".join(status.value for status in query_result.statuses)
            if query_result is not None
            else ""
        )
    )
    ax.grid(True, alpha=0.22)
    return ax


def build_and_plot_single_leg_rolling_scene_2d(
    theta_rad: float,
    beta_rad: float,
    hip_x_m: float,
    hip_z_m: float,
    *,
    ax=None,
    query_result: ContactQueryResult2D | None = None,
    **scene_kwargs,
) -> tuple[SingleLegRollingScene2D, object]:
    """Convenience helper: build one scene and return ``(scene, axes)``."""

    scene = build_single_leg_rolling_scene_2d(
        theta_rad,
        beta_rad,
        hip_x_m,
        hip_z_m,
        **scene_kwargs,
    )
    return scene, plot_single_leg_rolling_scene_2d(scene, ax=ax, query_result=query_result)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Draw the Day 6--7 fixed single-leg 2D scene with its contact query."
    )
    parser.add_argument("--theta-deg", type=float, default=40.0)
    parser.add_argument("--beta-deg", type=float, default=0.0)
    parser.add_argument("--hip-x-m", type=float, default=0.0)
    parser.add_argument("--hip-z-m", type=float, default=0.24)
    parser.add_argument("--obstacle-x-start-m", type=float, default=0.10)
    parser.add_argument("--obstacle-width-m", type=float, default=0.20)
    parser.add_argument("--obstacle-height-m", type=float, default=0.05)
    parser.add_argument("--arc-samples", type=int, default=121)
    parser.add_argument(
        "--flat-ground-only",
        action="store_true",
        help="Do not add the rectangular obstacle.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional PNG path; without it, open an interactive Matplotlib window.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    scene = build_single_leg_rolling_scene_2d(
        theta_rad=np.deg2rad(args.theta_deg),
        beta_rad=np.deg2rad(args.beta_deg),
        hip_x_m=args.hip_x_m,
        hip_z_m=args.hip_z_m,
        obstacle_x_start_m=None if args.flat_ground_only else args.obstacle_x_start_m,
        obstacle_width_m=args.obstacle_width_m,
        obstacle_height_m=args.obstacle_height_m,
        arc_samples=args.arc_samples,
    )
    query_result = query_single_leg_rolling_scene_2d(scene)
    ax = plot_single_leg_rolling_scene_2d(scene, query_result=query_result)
    if args.output is None:
        plt.show()
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        ax.figure.savefig(args.output, dpi=160, bbox_inches="tight")
        print(f"Saved Day 6--7 Step-2 scene + query: {args.output}")
        print(
            f"theta={scene.theta_deg:.3f} deg, beta={scene.beta_deg:.3f} deg, "
            f"gamma={scene.gamma_rad:.3f} rad, "
            f"status={query_result.status.value}, collision={query_result.collision}"
        )


if __name__ == "__main__":
    main()


__all__ = [
    "AirborneRetractResetFrame2D",
    "AirborneRetractResetBranchResult2D",
    "AirborneRetractResetComparison2D",
    "ForwardRollingFrame2D",
    "ForwardRollingResult2D",
    "ForwardRollingSweepResult2D",
    "ForwardRollingSweepRow2D",
    "RetractPreviewFrame2D",
    "RetractReadinessResult2D",
    "RetractToWheelFrame2D",
    "RetractToWheelResult2D",
    "RetractResetFrame2D",
    "RetractResetBranchResult2D",
    "RetractResetComparison2D",
    "WheelResetRollFrame2D",
    "WheelResetRollResult2D",
    "RightRimRollUpFrame2D",
    "RightRimRollUpResult2D",
    "RightRimThetaSweepRow2D",
    "RightRimThetaSweepResult2D",
    "SingleLegRollingScene2D",
    "build_and_plot_single_leg_rolling_scene_2d",
    "build_single_leg_rolling_scene_2d",
    "plot_single_leg_rolling_scene_2d",
    "plot_right_rim_roll_up_trajectory_2d",
    "plot_right_rim_theta_feasibility_2d",
    "plot_forward_rolling_trajectory_2d",
    "plot_retract_preview_final_pose_2d",
    "plot_retract_reset_comparison_2d",
    "plot_wheel_reset_roll_trajectory_2d",
    "forward_rolling_frame_rows",
    "forward_rolling_rows",
    "animate_forward_rolling_2d",
    "animate_roll_up_and_airborne_reset_2d",
    "animate_roll_up_and_retract_2d",
    "animate_wheel_reset_roll_2d",
    "animate_retract_and_reset_branch_2d",
    "evaluate_right_rim_retract_readiness_2d",
    "query_single_leg_rolling_scene_2d",
    "right_rim_roll_up_rows",
    "retract_preview_rows",
    "retract_to_wheel_rows",
    "retract_reset_comparison_rows",
    "retract_reset_frame_rows",
    "airborne_retract_reset_comparison_rows",
    "airborne_retract_reset_frame_rows",
    "wheel_reset_roll_rows",
    "right_rim_theta_sweep_rows",
    "run_fixed_theta_right_rim_roll_up_2d",
    "run_forward_right_rim_roll_up_2d",
    "run_retract_to_wheel_2d",
    "run_retract_and_reset_branch_2d",
    "run_retract_and_reset_comparison_2d",
    "run_airborne_retract_and_foot_reset_branch_2d",
    "run_airborne_retract_and_foot_reset_comparison_2d",
    "run_wheel_reset_roll_2d",
    "simulate_theta_candidates_forward_rolling_2d",
    "sweep_right_rim_roll_up_theta_2d",
    "write_forward_rolling_csv",
    "write_retract_to_wheel_csv",
    "write_retract_reset_comparison_csv",
    "write_airborne_retract_reset_comparison_csv",
    "write_wheel_reset_roll_csv",
    "write_right_rim_theta_sweep_csv",
    "plot_airborne_retract_reset_comparison_2d",
]
