"""Day 8--9 Step 1: the Cartesian swing input/output contract (2D, gamma = 0).

This module deliberately contains **no** trajectory generation, no Bezier, no
IK and no collision query.  It only freezes the data that flows into and out
of ``generate_swing()`` so that Steps 2--8 can be written against a stable
interface:

    SwingStartState2D + SwingTarget2D + terrain + hip trajectory
        -> SwingRequest2D
        -> (Steps 2--8)
        -> SwingResult2D

Scope of the first version, following the Day 8--9 planning note:

* planar sagittal geometry in world ``[x, z]`` metres, ``+x`` forward and
  ``+z`` up, matching the Day 3--7 ``*_2D`` contract;
* ``gamma = 0`` and zero hip pitch;
* the touchdown surface is a horizontal terrain surface.

Rim/alpha semantics, terrain surfaces, contact candidates and leg geometry are
reused from ``legwheel.planners.hybrid``; the Day 6--7 scene builder is reused
for the test-case helpers at the bottom of this file.  Nothing here redefines
robot geometry.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Sequence

import numpy as np
from numpy.typing import NDArray

from legwheel.config import RobotParams
from legwheel.planners.hybrid import (
    ContactCandidate2D,
    ContactState,
    HipPose2D,
    RimId,
    SurfaceOrientation,
    SwingTarget,
    TerrainProfile2D,
    TerrainSurface2D,
    legacy_rim_edge_margin_rad,
)
from legwheel.planners.hybrid.geometry_2d import LEGACY_SURFACE_ALPHA_LIMITS_DEG


# ---------------------------------------------------------------------------
# Shared conventions
# ---------------------------------------------------------------------------

#: Semantic rim identifier -> legacy tyre-arc surface name used by the existing
#: sampler and by ``legacy_rim_edge_margin_rad``.
RIM_TO_LEGACY_SURFACE: dict[RimId, str] = {
    RimId.FOOT: "foot_rim",
    RimId.LEFT: "upper_tyre_l",
    RimId.RIGHT: "upper_tyre_r",
}

LEGACY_SURFACE_TO_RIM: dict[str, RimId] = {
    value: key for key, value in RIM_TO_LEGACY_SURFACE.items()
}


def legacy_surface_name_for_rim(rim: RimId) -> str:
    """Return the legacy tyre-arc surface name owning one semantic rim."""

    return RIM_TO_LEGACY_SURFACE[RimId(rim)]


def rim_alpha_limits_rad(rim: RimId) -> tuple[float, float]:
    """Return the ``(alpha_min, alpha_max)`` arc bounds of one rim."""

    minimum_deg, maximum_deg = LEGACY_SURFACE_ALPHA_LIMITS_DEG[legacy_surface_name_for_rim(rim)]
    return float(np.deg2rad(minimum_deg)), float(np.deg2rad(maximum_deg))


def rim_for_alpha_rad(alpha_rad: float) -> RimId:
    """Return the rim that owns one global alpha value.

    The three tyre arcs partition alpha contiguously, so a swing that walks
    alpha from one contact state to another crosses rim identities on its own.
    A value landing exactly on a shared arc boundary is the same physical
    point on either arc; it is reported as the foot rim.
    """

    alpha = _finite_scalar(alpha_rad, "alpha_rad")
    for rim in (RimId.FOOT, RimId.LEFT, RimId.RIGHT):
        minimum_rad, maximum_rad = rim_alpha_limits_rad(rim)
        if minimum_rad - 1e-12 <= alpha <= maximum_rad + 1e-12:
            return rim
    raise ValueError(
        f"alpha={np.rad2deg(alpha):.3f} deg lies outside every rim arc "
        "([-180, 180] deg)."
    )


def _finite_scalar(value: float, name: str) -> float:
    number = float(value)
    if not np.isfinite(number):
        raise ValueError(f"{name} must be finite.")
    return number


def _finite_xz_point(value, name: str) -> NDArray[np.float64]:
    point = np.asarray(value, dtype=float)
    if point.shape != (2,):
        raise ValueError(f"{name} must be a world [x, z] pair; got shape {point.shape}.")
    if not np.all(np.isfinite(point)):
        raise ValueError(f"{name} must contain only finite values.")
    point = point.copy()
    point.setflags(write=False)
    return point


def _optional_finite(value, name: str) -> float | None:
    """Allow ``None`` for a quantity that a later step has not evaluated yet."""

    if value is None:
        return None
    return _finite_scalar(value, name)


# ---------------------------------------------------------------------------
# Failure vocabulary
# ---------------------------------------------------------------------------


class SwingFailure(str, Enum):
    """Why a swing is not usable, or ``NONE`` when it is.

    The vocabulary is fixed here in Step 1 so that Steps 5--8 report failures
    in one language instead of inventing per-step strings.
    """

    NOT_EVALUATED = "NOT_EVALUATED"
    NONE = "NONE"

    INVALID_REQUEST = "INVALID_REQUEST"
    IK_NOT_CONVERGED = "IK_NOT_CONVERGED"
    IK_RESIDUAL_TOO_LARGE = "IK_RESIDUAL_TOO_LARGE"
    JOINT_LIMIT_VIOLATION = "JOINT_LIMIT_VIOLATION"
    JOINT_DISCONTINUITY = "JOINT_DISCONTINUITY"
    TERRAIN_COLLISION = "TERRAIN_COLLISION"
    TOUCHDOWN_POSITION_ERROR = "TOUCHDOWN_POSITION_ERROR"
    TOUCHDOWN_RIM_MISMATCH = "TOUCHDOWN_RIM_MISMATCH"
    TOUCHDOWN_ALPHA_ERROR = "TOUCHDOWN_ALPHA_ERROR"
    TOUCHDOWN_SURFACE_MISMATCH = "TOUCHDOWN_SURFACE_MISMATCH"
    TOUCHDOWN_VELOCITY_TOO_HIGH = "TOUCHDOWN_VELOCITY_TOO_HIGH"


# ---------------------------------------------------------------------------
# Inputs
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SwingStartState2D:
    """Where the swing starts: a contact state **plus** its leg configuration.

    The Day 1 ``ContactState`` intentionally carries only world contact data
    (rim / alpha / point / surface).  A swing additionally needs the joint
    configuration that realises that contact, because Step 5 seeds the first
    IK solve with it and Step 6 rebuilds the full leg geometry from it.  Rather
    than widen the frozen Day 1 contract, this type composes the two; use
    :meth:`as_contact_state` to hand the pure contact part back to code that
    speaks the Day 1 interface.
    """

    contact_point_world_xz_m: NDArray[np.float64]
    rim: RimId
    alpha_rad: float
    terrain_surface_id: str
    theta_rad: float
    beta_rad: float
    hip_pose: HipPose2D
    gamma_rad: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "rim", RimId(self.rim))
        object.__setattr__(
            self,
            "contact_point_world_xz_m",
            _finite_xz_point(self.contact_point_world_xz_m, "contact_point_world_xz_m"),
        )
        for name in ("alpha_rad", "theta_rad", "beta_rad", "gamma_rad"):
            object.__setattr__(self, name, _finite_scalar(getattr(self, name), name))
        if not self.terrain_surface_id:
            raise ValueError("terrain_surface_id must not be empty.")
        if not np.isclose(self.gamma_rad, 0.0, atol=1e-12):
            raise ValueError("The Day 8--9 first version only supports gamma = 0.")
        if not isinstance(self.hip_pose, HipPose2D):
            raise TypeError("hip_pose must be a HipPose2D.")
        if not np.isclose(self.hip_pose.pitch_world_hip_rad, 0.0, atol=1e-12):
            raise ValueError("The Day 8--9 first version only supports zero hip pitch.")
        # Raises when alpha falls outside the arc that the rim actually owns.
        legacy_rim_edge_margin_rad(self.legacy_surface_name, self.alpha_rad)

    @property
    def legacy_surface_name(self) -> str:
        return legacy_surface_name_for_rim(self.rim)

    @property
    def rim_edge_margin_rad(self) -> float:
        """Angular distance from alpha to the nearest end of its own rim arc."""

        return legacy_rim_edge_margin_rad(self.legacy_surface_name, self.alpha_rad)

    @property
    def alpha_deg(self) -> float:
        return float(np.rad2deg(self.alpha_rad))

    @property
    def theta_deg(self) -> float:
        return float(np.rad2deg(self.theta_rad))

    @property
    def beta_deg(self) -> float:
        return float(np.rad2deg(self.beta_rad))

    @property
    def joint_angles_rad(self) -> NDArray[np.float64]:
        return np.array([self.theta_rad, self.beta_rad, self.gamma_rad], dtype=float)

    def as_contact_state(self) -> ContactState:
        """Return the Day 1 world-frame ``ContactState`` for this start state."""

        x, z = self.contact_point_world_xz_m
        return ContactState(
            rim=self.rim,
            alpha_rad=self.alpha_rad,
            point_world_m=(float(x), 0.0, float(z)),
            terrain_surface_id=self.terrain_surface_id,
        )

    def as_dict(self) -> dict:
        return {
            "contact_x_m": float(self.contact_point_world_xz_m[0]),
            "contact_z_m": float(self.contact_point_world_xz_m[1]),
            "rim": self.rim.value,
            "alpha_deg": self.alpha_deg,
            "terrain_surface_id": self.terrain_surface_id,
            "theta_deg": self.theta_deg,
            "beta_deg": self.beta_deg,
            "gamma_deg": float(np.rad2deg(self.gamma_rad)),
            "hip_x_m": float(self.hip_pose.position_world_xz_m[0]),
            "hip_z_m": float(self.hip_pose.position_world_xz_m[1]),
            "rim_edge_margin_deg": float(np.rad2deg(self.rim_edge_margin_rad)),
        }


def swing_start_state_from_contact_candidate(
    candidate: ContactCandidate2D,
    *,
    theta_rad: float,
    beta_rad: float,
    hip_pose: HipPose2D,
    gamma_rad: float = 0.0,
) -> SwingStartState2D:
    """Promote one contact candidate into a swing start state.

    This is the seam with Day 6--7: a rolling propagation ends on some
    ``ContactCandidate2D``, and the pose that produced it is known, so the
    roll-end state converts into a swing start state without re-deriving any
    geometry.
    """

    if not isinstance(candidate, ContactCandidate2D):
        raise TypeError("candidate must be a ContactCandidate2D.")
    return SwingStartState2D(
        contact_point_world_xz_m=candidate.point_world_xz_m,
        rim=candidate.rim,
        alpha_rad=candidate.alpha_rad,
        terrain_surface_id=candidate.terrain_surface_id,
        theta_rad=theta_rad,
        beta_rad=beta_rad,
        hip_pose=hip_pose,
        gamma_rad=gamma_rad,
    )


@dataclass(frozen=True)
class SwingTarget2D:
    """The requested touchdown **contact state**, not just a foothold.

    ``target_terrain_surface_id`` is what makes Step 7 checkable: the swing has
    to land on the intended surface, not merely at the intended height.
    """

    target_point_world_xz_m: NDArray[np.float64]
    target_rim: RimId
    target_alpha_rad: float
    target_terrain_surface_id: str
    clearance_m: float = 0.03

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_rim", RimId(self.target_rim))
        object.__setattr__(
            self,
            "target_point_world_xz_m",
            _finite_xz_point(self.target_point_world_xz_m, "target_point_world_xz_m"),
        )
        object.__setattr__(
            self, "target_alpha_rad", _finite_scalar(self.target_alpha_rad, "target_alpha_rad")
        )
        object.__setattr__(self, "clearance_m", _finite_scalar(self.clearance_m, "clearance_m"))
        if self.clearance_m < 0.0:
            raise ValueError("clearance_m must be non-negative.")
        if not self.target_terrain_surface_id:
            raise ValueError("target_terrain_surface_id must not be empty.")
        legacy_rim_edge_margin_rad(self.legacy_surface_name, self.target_alpha_rad)

    @property
    def legacy_surface_name(self) -> str:
        return legacy_surface_name_for_rim(self.target_rim)

    @property
    def target_alpha_deg(self) -> float:
        return float(np.rad2deg(self.target_alpha_rad))

    @property
    def touchdown_height_m(self) -> float:
        """The touchdown terrain height is just the target ``z``; see note 7.2."""

        return float(self.target_point_world_xz_m[1])

    def as_swing_target(self) -> SwingTarget:
        """Return the Day 1 world-frame ``SwingTarget`` for this request."""

        x, z = self.target_point_world_xz_m
        return SwingTarget(
            target_position_world_m=(float(x), 0.0, float(z)),
            target_rim=self.target_rim,
            target_alpha_rad=self.target_alpha_rad,
            clearance_m=self.clearance_m,
        )

    def as_dict(self) -> dict:
        return {
            "target_x_m": float(self.target_point_world_xz_m[0]),
            "target_z_m": float(self.target_point_world_xz_m[1]),
            "target_rim": self.target_rim.value,
            "target_alpha_deg": self.target_alpha_deg,
            "target_terrain_surface_id": self.target_terrain_surface_id,
            "clearance_m": float(self.clearance_m),
        }


@dataclass(frozen=True)
class HipTrajectory2D:
    """Hip motion during the swing, as a function of normalised time ``s``.

    The first version is a straight line between two hip poses; a constant hip
    is the ``end_pose is None`` case.  Keeping it behind ``pose_at`` means a
    later body-trajectory model can replace the interpolation without changing
    any caller.
    """

    start_pose: HipPose2D
    end_pose: HipPose2D | None = None

    def __post_init__(self) -> None:
        for name in ("start_pose", "end_pose"):
            pose = getattr(self, name)
            if pose is None:
                continue
            if not isinstance(pose, HipPose2D):
                raise TypeError(f"{name} must be a HipPose2D.")
            if not np.isclose(pose.pitch_world_hip_rad, 0.0, atol=1e-12):
                raise ValueError("The Day 8--9 first version only supports zero hip pitch.")

    @property
    def is_stationary(self) -> bool:
        if self.end_pose is None:
            return True
        return bool(
            np.allclose(
                self.start_pose.position_world_xz_m,
                self.end_pose.position_world_xz_m,
                atol=1e-12,
            )
        )

    @property
    def displacement_world_xz_m(self) -> NDArray[np.float64]:
        if self.end_pose is None:
            return np.zeros(2, dtype=float)
        return np.asarray(
            self.end_pose.position_world_xz_m - self.start_pose.position_world_xz_m, dtype=float
        )

    def pose_at(self, s: float) -> HipPose2D:
        """Return the hip pose at normalised swing time ``s`` in ``[0, 1]``."""

        fraction = _finite_scalar(s, "s")
        if not 0.0 - 1e-12 <= fraction <= 1.0 + 1e-12:
            raise ValueError("s must lie in [0, 1].")
        if self.end_pose is None:
            return self.start_pose
        fraction = float(np.clip(fraction, 0.0, 1.0))
        position = (
            self.start_pose.position_world_xz_m
            + fraction * self.displacement_world_xz_m
        )
        return HipPose2D(position, pitch_world_hip_rad=0.0)


@dataclass(frozen=True)
class SwingConstraints2D:
    """Every tolerance and limit the swing checks use, in one place.

    Defaults come from the existing robot configuration and the Day 3--7
    query tolerances, so constructing this with no arguments reproduces the
    conventions the earlier steps were verified with.
    """

    theta_min_rad: float = float(np.deg2rad(RobotParams.MIN_THETA_DEG))
    theta_max_rad: float = float(np.deg2rad(RobotParams.MAX_THETA_DEG))
    #: Continuity limit **per sample**, so it is coupled to ``sample_count``:
    #: the same trajectory sampled twice as densely has half the step.  A case
    #: whose verdict depends on this limit therefore has to pin its own
    #: ``sample_count`` to be reproducible.  The sampling-independent view is
    #: the joint *speed*, which the Step 5 report derives from it.
    max_joint_step_rad: float = float(np.deg2rad(10.0))

    ik_position_tolerance_m: float = 1e-3

    touchdown_position_tolerance_m: float = 2e-3
    touchdown_alpha_tolerance_rad: float = float(np.deg2rad(2.0))
    touchdown_normal_speed_max_mps: float = 0.05

    contact_tolerance_m: float = 1e-3
    collision_tolerance_m: float = 1e-3
    surface_position_tolerance_m: float = 1e-3
    min_terrain_clearance_m: float = 0.0

    def __post_init__(self) -> None:
        for name in (
            "theta_min_rad",
            "theta_max_rad",
            "max_joint_step_rad",
            "ik_position_tolerance_m",
            "touchdown_position_tolerance_m",
            "touchdown_alpha_tolerance_rad",
            "touchdown_normal_speed_max_mps",
            "contact_tolerance_m",
            "collision_tolerance_m",
            "surface_position_tolerance_m",
            "min_terrain_clearance_m",
        ):
            object.__setattr__(self, name, _finite_scalar(getattr(self, name), name))
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} must be non-negative.")
        if self.theta_max_rad <= self.theta_min_rad:
            raise ValueError("theta_max_rad must be greater than theta_min_rad.")
        if self.max_joint_step_rad <= 0.0:
            raise ValueError("max_joint_step_rad must be positive.")

    def as_dict(self) -> dict:
        return {
            "theta_min_deg": float(np.rad2deg(self.theta_min_rad)),
            "theta_max_deg": float(np.rad2deg(self.theta_max_rad)),
            "max_joint_step_deg": float(np.rad2deg(self.max_joint_step_rad)),
            "ik_position_tolerance_mm": self.ik_position_tolerance_m * 1e3,
            "touchdown_position_tolerance_mm": self.touchdown_position_tolerance_m * 1e3,
            "touchdown_alpha_tolerance_deg": float(
                np.rad2deg(self.touchdown_alpha_tolerance_rad)
            ),
            "touchdown_normal_speed_max_mps": self.touchdown_normal_speed_max_mps,
            "contact_tolerance_mm": self.contact_tolerance_m * 1e3,
            "collision_tolerance_mm": self.collision_tolerance_m * 1e3,
            "surface_position_tolerance_mm": self.surface_position_tolerance_m * 1e3,
            "min_terrain_clearance_mm": self.min_terrain_clearance_m * 1e3,
        }


@dataclass(frozen=True)
class SwingRequest2D:
    """One complete, self-describing swing planning problem."""

    start: SwingStartState2D
    target: SwingTarget2D
    terrain: TerrainProfile2D
    hip_trajectory: HipTrajectory2D
    swing_duration_s: float = 0.6
    sample_count: int = 101
    constraints: SwingConstraints2D = field(default_factory=SwingConstraints2D)

    def __post_init__(self) -> None:
        if not isinstance(self.start, SwingStartState2D):
            raise TypeError("start must be a SwingStartState2D.")
        if not isinstance(self.target, SwingTarget2D):
            raise TypeError("target must be a SwingTarget2D.")
        if not isinstance(self.terrain, TerrainProfile2D):
            raise TypeError("terrain must be a TerrainProfile2D.")
        if not isinstance(self.hip_trajectory, HipTrajectory2D):
            raise TypeError("hip_trajectory must be a HipTrajectory2D.")
        if not isinstance(self.constraints, SwingConstraints2D):
            raise TypeError("constraints must be a SwingConstraints2D.")
        object.__setattr__(
            self, "swing_duration_s", _finite_scalar(self.swing_duration_s, "swing_duration_s")
        )
        if self.swing_duration_s <= 0.0:
            raise ValueError("swing_duration_s must be positive.")
        if int(self.sample_count) < 2:
            raise ValueError("sample_count must be at least 2.")
        object.__setattr__(self, "sample_count", int(self.sample_count))

    @property
    def horizontal_span_m(self) -> float:
        """Signed forward distance from the start contact to the target."""

        return float(
            self.target.target_point_world_xz_m[0] - self.start.contact_point_world_xz_m[0]
        )

    @property
    def height_change_m(self) -> float:
        """Touchdown height minus start height; ``0`` for a flat-to-flat swing."""

        return float(
            self.target.target_point_world_xz_m[1] - self.start.contact_point_world_xz_m[1]
        )

    @property
    def is_rim_transition(self) -> bool:
        return self.start.rim is not self.target.target_rim

    def as_dict(self) -> dict:
        rows = {}
        rows.update({f"start_{k}": v for k, v in self.start.as_dict().items()})
        rows.update(self.target.as_dict())
        rows.update(
            {
                "swing_duration_s": self.swing_duration_s,
                "sample_count": self.sample_count,
                "horizontal_span_m": self.horizontal_span_m,
                "height_change_m": self.height_change_m,
                "rim_transition": self.is_rim_transition,
                "hip_stationary": self.hip_trajectory.is_stationary,
            }
        )
        return rows


# ---------------------------------------------------------------------------
# Outputs
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SwingSample2D:
    """One trajectory sample and everything later steps record about it.

    Step 2 fills in only the Cartesian fields.  Step 5 adds the joint solution,
    Step 6 the collision outcome.  Fields a step has not evaluated stay
    ``None`` instead of a silently plausible number.
    """

    index: int
    time_s: float
    position_world_xz_m: NDArray[np.float64]
    velocity_world_xz_mps: NDArray[np.float64]
    alpha_rad: float
    rim: RimId

    theta_rad: float | None = None
    beta_rad: float | None = None
    gamma_rad: float | None = None
    ik_converged: bool | None = None
    ik_residual_m: float | None = None
    joint_step_rad: float | None = None
    joint_limits_ok: bool | None = None
    collision_free: bool | None = None
    terrain_clearance_m: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "rim", RimId(self.rim))
        if int(self.index) < 0:
            raise ValueError("index must be non-negative.")
        object.__setattr__(self, "index", int(self.index))
        object.__setattr__(self, "time_s", _finite_scalar(self.time_s, "time_s"))
        object.__setattr__(self, "alpha_rad", _finite_scalar(self.alpha_rad, "alpha_rad"))
        object.__setattr__(
            self,
            "position_world_xz_m",
            _finite_xz_point(self.position_world_xz_m, "position_world_xz_m"),
        )
        object.__setattr__(
            self,
            "velocity_world_xz_mps",
            _finite_xz_point(self.velocity_world_xz_mps, "velocity_world_xz_mps"),
        )
        for name in (
            "theta_rad",
            "beta_rad",
            "gamma_rad",
            "ik_residual_m",
            "joint_step_rad",
            "terrain_clearance_m",
        ):
            object.__setattr__(self, name, _optional_finite(getattr(self, name), name))

    @property
    def speed_mps(self) -> float:
        return float(np.linalg.norm(self.velocity_world_xz_mps))

    def as_row(self) -> dict:
        return {
            "index": self.index,
            "time_s": self.time_s,
            "x_m": float(self.position_world_xz_m[0]),
            "z_m": float(self.position_world_xz_m[1]),
            "vx_mps": float(self.velocity_world_xz_mps[0]),
            "vz_mps": float(self.velocity_world_xz_mps[1]),
            "speed_mps": self.speed_mps,
            "rim": self.rim.value,
            "alpha_deg": float(np.rad2deg(self.alpha_rad)),
            "theta_deg": None if self.theta_rad is None else float(np.rad2deg(self.theta_rad)),
            "beta_deg": None if self.beta_rad is None else float(np.rad2deg(self.beta_rad)),
            "gamma_deg": None if self.gamma_rad is None else float(np.rad2deg(self.gamma_rad)),
            "ik_converged": self.ik_converged,
            "ik_residual_mm": (
                None if self.ik_residual_m is None else self.ik_residual_m * 1e3
            ),
            "joint_step_deg": (
                None if self.joint_step_rad is None else float(np.rad2deg(self.joint_step_rad))
            ),
            "joint_limits_ok": self.joint_limits_ok,
            "collision_free": self.collision_free,
            "terrain_clearance_mm": (
                None if self.terrain_clearance_m is None else self.terrain_clearance_m * 1e3
            ),
        }


@dataclass(frozen=True)
class SwingResult2D:
    """What ``generate_swing()`` returns: the trajectory plus why to trust it.

    ``valid`` is never true unless ``failure is SwingFailure.NONE``; the two
    are kept together so a caller can branch on the flag and still report the
    reason without re-deriving it.
    """

    request: SwingRequest2D
    samples: tuple[SwingSample2D, ...] = ()
    valid: bool = False
    failure: SwingFailure = SwingFailure.NOT_EVALUATED
    failure_sample_index: int | None = None
    failure_detail: str | None = None

    minimum_terrain_clearance_m: float | None = None
    final_contact_error_m: float | None = None
    final_alpha_error_rad: float | None = None
    final_rim: RimId | None = None
    final_terrain_surface_id: str | None = None
    touchdown_normal_speed_mps: float | None = None
    touchdown_tangential_speed_mps: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.request, SwingRequest2D):
            raise TypeError("request must be a SwingRequest2D.")
        samples = tuple(self.samples)
        if not all(isinstance(item, SwingSample2D) for item in samples):
            raise TypeError("samples must contain SwingSample2D values.")
        object.__setattr__(self, "samples", samples)
        object.__setattr__(self, "failure", SwingFailure(self.failure))
        if self.final_rim is not None:
            object.__setattr__(self, "final_rim", RimId(self.final_rim))
        if self.valid and self.failure is not SwingFailure.NONE:
            raise ValueError("A valid swing must report SwingFailure.NONE.")
        if not self.valid and self.failure is SwingFailure.NONE:
            raise ValueError("SwingFailure.NONE must not be paired with valid=False.")
        if self.failure_sample_index is not None:
            index = int(self.failure_sample_index)
            if index < 0:
                raise ValueError("failure_sample_index must be non-negative.")
            object.__setattr__(self, "failure_sample_index", index)
        for name in (
            "minimum_terrain_clearance_m",
            "final_contact_error_m",
            "final_alpha_error_rad",
            "touchdown_normal_speed_mps",
            "touchdown_tangential_speed_mps",
        ):
            object.__setattr__(self, name, _optional_finite(getattr(self, name), name))

    @property
    def sample_count(self) -> int:
        return len(self.samples)

    @property
    def time_s(self) -> NDArray[np.float64]:
        return np.array([sample.time_s for sample in self.samples], dtype=float)

    @property
    def positions_world_xz_m(self) -> NDArray[np.float64]:
        if not self.samples:
            return np.zeros((0, 2), dtype=float)
        return np.array([sample.position_world_xz_m for sample in self.samples], dtype=float)

    @property
    def velocities_world_xz_mps(self) -> NDArray[np.float64]:
        if not self.samples:
            return np.zeros((0, 2), dtype=float)
        return np.array([sample.velocity_world_xz_mps for sample in self.samples], dtype=float)

    @property
    def apex_height_m(self) -> float | None:
        if not self.samples:
            return None
        return float(np.max(self.positions_world_xz_m[:, 1]))

    def summary_row(self) -> dict:
        row = {
            "valid": self.valid,
            "failure": self.failure.value,
            "failure_sample_index": self.failure_sample_index,
            "failure_detail": self.failure_detail,
            "sample_count": self.sample_count,
            "apex_z_m": self.apex_height_m,
            "minimum_terrain_clearance_mm": (
                None
                if self.minimum_terrain_clearance_m is None
                else self.minimum_terrain_clearance_m * 1e3
            ),
            "final_contact_error_mm": (
                None if self.final_contact_error_m is None else self.final_contact_error_m * 1e3
            ),
            "final_alpha_error_deg": (
                None
                if self.final_alpha_error_rad is None
                else float(np.rad2deg(self.final_alpha_error_rad))
            ),
            "final_rim": None if self.final_rim is None else self.final_rim.value,
            "final_terrain_surface_id": self.final_terrain_surface_id,
            "touchdown_normal_speed_mps": self.touchdown_normal_speed_mps,
            "touchdown_tangential_speed_mps": self.touchdown_tangential_speed_mps,
        }
        row.update(self.request.as_dict())
        return row


def pending_swing_result_2d(
    request: SwingRequest2D,
    *,
    failure: SwingFailure = SwingFailure.NOT_EVALUATED,
    failure_detail: str | None = None,
) -> SwingResult2D:
    """Return the output object for a request no planner has solved yet.

    Step 1 ships this so the notebook can exercise the full output contract
    before Step 2 exists, and so later steps have a well-defined object to
    return when they reject a request outright.
    """

    return SwingResult2D(
        request=request,
        samples=(),
        valid=False,
        failure=failure,
        failure_detail=failure_detail,
    )


# ---------------------------------------------------------------------------
# Request validation
# ---------------------------------------------------------------------------


def _horizontal_surface_problems(
    surface: TerrainSurface2D,
    point_world_xz_m: NDArray[np.float64],
    *,
    label: str,
    position_tolerance_m: float,
) -> list[str]:
    problems: list[str] = []
    if surface.orientation is not SurfaceOrientation.HORIZONTAL:
        problems.append(
            f"{label} surface {surface.surface_id!r} is {surface.orientation.value}; "
            "the first version only supports contact on a horizontal surface."
        )
        return problems
    x, z = float(point_world_xz_m[0]), float(point_world_xz_m[1])
    height_error = abs(z - surface.position_m)
    if height_error > position_tolerance_m:
        problems.append(
            f"{label} z={z:.6f} m is {height_error * 1e3:.2f} mm from surface "
            f"{surface.surface_id!r} at z={surface.position_m:.6f} m."
        )
    if not surface.span_min_m - position_tolerance_m <= x <= surface.span_max_m + position_tolerance_m:
        problems.append(
            f"{label} x={x:.6f} m lies outside the span of surface "
            f"{surface.surface_id!r} ([{surface.span_min_m:.6f}, {surface.span_max_m:.6f}] m)."
        )
    return problems


def validate_swing_request_2d(request: SwingRequest2D) -> tuple[str, ...]:
    """Return every reason this request is not a well-posed swing problem.

    An empty tuple means the request is internally consistent: both contact
    states name a real horizontal terrain surface they actually lie on, both
    alphas belong to their own rim arc, and the start pose respects the joint
    limits.  It does **not** mean a feasible trajectory exists -- that is what
    Steps 2--8 decide.
    """

    if not isinstance(request, SwingRequest2D):
        raise TypeError("request must be a SwingRequest2D.")

    problems: list[str] = []
    terrain = request.terrain
    constraints = request.constraints
    tolerance = constraints.surface_position_tolerance_m

    for label, surface_id, point in (
        ("start contact", request.start.terrain_surface_id, request.start.contact_point_world_xz_m),
        (
            "target contact",
            request.target.target_terrain_surface_id,
            request.target.target_point_world_xz_m,
        ),
    ):
        try:
            surface = terrain.surface_by_id(surface_id)
        except KeyError:
            problems.append(
                f"{label} names terrain surface {surface_id!r}, which this terrain "
                f"does not contain (known: {list(terrain.surface_ids)})."
            )
            continue
        problems.extend(
            _horizontal_surface_problems(
                surface, point, label=label, position_tolerance_m=tolerance
            )
        )

    if not constraints.theta_min_rad <= request.start.theta_rad <= constraints.theta_max_rad:
        problems.append(
            f"start theta={request.start.theta_deg:.2f} deg is outside the joint limits "
            f"[{np.rad2deg(constraints.theta_min_rad):.2f}, "
            f"{np.rad2deg(constraints.theta_max_rad):.2f}] deg."
        )

    if np.allclose(
        request.start.contact_point_world_xz_m,
        request.target.target_point_world_xz_m,
        atol=1e-9,
    ) and not request.is_rim_transition and np.isclose(
        request.start.alpha_rad, request.target.target_alpha_rad, atol=1e-9
    ):
        problems.append(
            "start and target describe the same contact state; there is nothing to relocate."
        )

    return tuple(problems)


def swing_request_rows(request: SwingRequest2D) -> list[dict]:
    """Flatten a request into ``field / value`` rows for display or CSV."""

    sections = (
        ("start", request.start.as_dict()),
        ("target", request.target.as_dict()),
        ("constraints", request.constraints.as_dict()),
        (
            "swing",
            {
                "swing_duration_s": request.swing_duration_s,
                "sample_count": request.sample_count,
                "horizontal_span_m": request.horizontal_span_m,
                "height_change_m": request.height_change_m,
                "rim_transition": request.is_rim_transition,
                "hip_stationary": request.hip_trajectory.is_stationary,
            },
        ),
    )
    return [
        {"section": section, "field": key, "value": value}
        for section, values in sections
        for key, value in values.items()
    ]


# ---------------------------------------------------------------------------
# Tables and CSV
# ---------------------------------------------------------------------------


def swing_result_frame_rows(result: SwingResult2D) -> list[dict]:
    if not isinstance(result, SwingResult2D):
        raise TypeError("result must be a SwingResult2D.")
    return [sample.as_row() for sample in result.samples]


def swing_result_summary_row(result: SwingResult2D) -> dict:
    if not isinstance(result, SwingResult2D):
        raise TypeError("result must be a SwingResult2D.")
    return result.summary_row()


def _write_rows_csv(path, rows: Sequence[dict], *, fieldnames: Sequence[str] | None = None) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    names = list(fieldnames) if fieldnames is not None else list(rows[0]) if rows else []
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=names)
        writer.writeheader()
        writer.writerows(rows)
    return output


def write_swing_result_csv(
    result: SwingResult2D,
    frames_path,
    summary_path,
) -> tuple[Path, Path]:
    """Write the per-sample frames and the one-line summary, Day 6--7 style."""

    frame_rows = swing_result_frame_rows(result)
    frame_fieldnames = list(frame_rows[0]) if frame_rows else list(_EMPTY_FRAME_FIELDS)
    frames = _write_rows_csv(frames_path, frame_rows, fieldnames=frame_fieldnames)
    summary = _write_rows_csv(summary_path, [swing_result_summary_row(result)])
    return frames, summary


#: Frame CSV header used when a result carries no samples yet, so an empty
#: Step 1 export still documents the schema Steps 2--8 will fill in.
_EMPTY_FRAME_FIELDS = (
    "index",
    "time_s",
    "x_m",
    "z_m",
    "vx_mps",
    "vz_mps",
    "speed_mps",
    "rim",
    "alpha_deg",
    "theta_deg",
    "beta_deg",
    "gamma_deg",
    "ik_converged",
    "ik_residual_mm",
    "joint_step_deg",
    "joint_limits_ok",
    "collision_free",
    "terrain_clearance_mm",
)


# ---------------------------------------------------------------------------
# Test-case builders (reuse the Day 6--7 scene; no new geometry)
# ---------------------------------------------------------------------------

from legwheel.planners.hybrid import query_contact  # noqa: E402  (grouped with the builders)

from .single_leg_rolling_scene_2d import (  # noqa: E402
    SingleLegRollingScene2D,
    build_single_leg_rolling_scene_2d,
)


def _lowest_rim_offset_m(theta_rad: float, beta_rad: float, arc_samples: int) -> float:
    """Lowest sampled rim height of one pose, measured from the hip origin."""

    probe = build_single_leg_rolling_scene_2d(
        theta_rad,
        beta_rad,
        0.0,
        0.0,
        obstacle_x_start_m=None,
        arc_samples=arc_samples,
    )
    return float(np.min(probe.geometry.points_world_xz_m[:, 1]))


def build_leg_on_surface_scene_2d(
    theta_rad: float,
    beta_rad: float,
    hip_x_m: float,
    support_height_m: float,
    *,
    ground_height_m: float = 0.0,
    obstacle_x_start_m: float | None = None,
    obstacle_width_m: float = 0.20,
    obstacle_height_m: float = 0.05,
    obstacle_id: str = "day8_9_obstacle",
    arc_samples: int = 241,
    surface_offset_m: float = 1e-9,
) -> SingleLegRollingScene2D:
    """Drop one pose onto ``support_height_m`` by solving only for ``hip_z``.

    The hip height is chosen so the lowest sampled rim point sits on the
    requested support height, lifted by ``surface_offset_m``.  That lift is the
    same 1 nm convention Day 6--7 uses: resting a sampled point *exactly* on a
    surface leaves the sign of a 1e-17 rounding error to decide whether the
    terrain query calls it a contact or a penetration.

    Which terrain surface the point lands on is the caller's choice of
    ``hip_x_m`` and obstacle span -- this helper does not search for a
    foothold, it only removes the "guess a hip height" step from building a
    legal test case.
    """

    lowest_offset = _lowest_rim_offset_m(theta_rad, beta_rad, arc_samples)
    hip_z_m = (
        _finite_scalar(support_height_m, "support_height_m")
        + _finite_scalar(surface_offset_m, "surface_offset_m")
        - lowest_offset
    )
    return build_single_leg_rolling_scene_2d(
        theta_rad,
        beta_rad,
        hip_x_m,
        hip_z_m,
        ground_height_m=ground_height_m,
        obstacle_x_start_m=obstacle_x_start_m,
        obstacle_width_m=obstacle_width_m,
        obstacle_height_m=obstacle_height_m,
        obstacle_id=obstacle_id,
        arc_samples=arc_samples,
    )


def select_active_contact_candidate_2d(candidates) -> ContactCandidate2D:
    """Pick the supporting contact out of a set of candidates.

    The rule is deliberately simple and stated once: the candidate whose
    signed normal gap is smallest, tie-broken by the larger rim-edge margin so
    a point in the middle of an arc wins over one at its seam.  Step 7 needs
    the same rule as the scene builders -- "which contact is the leg actually
    standing on" has to mean one thing across the whole pipeline.
    """

    candidates = tuple(candidates)
    if not candidates:
        raise ValueError("no contact candidates to choose from.")
    return min(candidates, key=lambda item: (abs(item.terrain_gap_m), -item.edge_margin_rad))


def active_contact_candidate_2d(
    scene: SingleLegRollingScene2D,
    *,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
    prefer_sample_index: int | None = None,
) -> ContactCandidate2D:
    """Return the supporting contact of one scene, using the Day 3--5 query.

    Selection is deliberately simple and explicit: the candidate whose signed
    normal gap is smallest, tie-broken by the larger rim-edge margin.  A pose
    with no candidate, or one the query already calls a collision, raises
    instead of returning a contact the rest of the pipeline would trust.
    """

    if not isinstance(scene, SingleLegRollingScene2D):
        raise TypeError("scene must be a SingleLegRollingScene2D.")
    result = query_contact(
        scene.geometry,
        scene.terrain,
        contact_tolerance_m=contact_tolerance_m,
        collision_tolerance_m=collision_tolerance_m,
    )
    if result.link_collisions:
        raise ValueError("this pose has a link/terrain collision; it is not a support state.")
    if result.geometry_penetrations:
        raise ValueError("this pose penetrates the terrain; it is not a support state.")
    if not result.candidates:
        raise ValueError(
            "no rim sample is within the contact tolerance of any terrain surface."
        )
    if prefer_sample_index is not None:
        matches = [
            item for item in result.candidates if item.sample_index == int(prefer_sample_index)
        ]
        if not matches:
            raise ValueError(
                f"sample index {int(prefer_sample_index)} is not a contact candidate here."
            )
        return matches[0]
    return select_active_contact_candidate_2d(result.candidates)


def swing_start_state_from_scene_2d(
    scene: SingleLegRollingScene2D,
    *,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
    prefer_sample_index: int | None = None,
) -> SwingStartState2D:
    """Build a swing start state from a posed, terrain-supported scene."""

    candidate = active_contact_candidate_2d(
        scene,
        contact_tolerance_m=contact_tolerance_m,
        collision_tolerance_m=collision_tolerance_m,
        prefer_sample_index=prefer_sample_index,
    )
    return swing_start_state_from_contact_candidate(
        candidate,
        theta_rad=scene.theta_rad,
        beta_rad=scene.beta_rad,
        hip_pose=scene.hip_pose,
        gamma_rad=scene.gamma_rad,
    )


def swing_target_from_scene_2d(
    scene: SingleLegRollingScene2D,
    *,
    clearance_m: float = 0.03,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
    prefer_sample_index: int | None = None,
) -> SwingTarget2D:
    """Derive a touchdown request from a pose that already achieves it.

    Deriving the target from a real configuration keeps the Step 1 test cases
    honest: the requested ``(position, rim, alpha)`` is known to be reachable,
    so a later IK or collision failure is a planner result rather than an
    impossible request.
    """

    candidate = active_contact_candidate_2d(
        scene,
        contact_tolerance_m=contact_tolerance_m,
        collision_tolerance_m=collision_tolerance_m,
        prefer_sample_index=prefer_sample_index,
    )
    return SwingTarget2D(
        target_point_world_xz_m=candidate.point_world_xz_m,
        target_rim=candidate.rim,
        target_alpha_rad=candidate.alpha_rad,
        target_terrain_surface_id=candidate.terrain_surface_id,
        clearance_m=clearance_m,
    )


def build_swing_request_2d(
    start_scene: SingleLegRollingScene2D,
    target_scene: SingleLegRollingScene2D,
    *,
    clearance_m: float = 0.03,
    swing_duration_s: float = 0.6,
    sample_count: int = 101,
    constraints: SwingConstraints2D | None = None,
    move_hip: bool = True,
    start_sample_index: int | None = None,
    target_sample_index: int | None = None,
) -> SwingRequest2D:
    """Compose a request from a start scene and a touchdown scene.

    Both scenes must describe the same terrain: the swing is a relocation on
    one world, not a jump between two of them.  ``move_hip`` drives the hip
    from the start scene's hip to the target scene's hip during the swing,
    which is what makes the touchdown pose reachable at the end of the motion.
    """

    for name, scene in (("start_scene", start_scene), ("target_scene", target_scene)):
        if not isinstance(scene, SingleLegRollingScene2D):
            raise TypeError(f"{name} must be a SingleLegRollingScene2D.")
    if start_scene.terrain != target_scene.terrain:
        raise ValueError("start and target scenes must share the same terrain.")

    limits = SwingConstraints2D() if constraints is None else constraints
    start = swing_start_state_from_scene_2d(
        start_scene,
        contact_tolerance_m=limits.contact_tolerance_m,
        collision_tolerance_m=limits.collision_tolerance_m,
        prefer_sample_index=start_sample_index,
    )
    target = swing_target_from_scene_2d(
        target_scene,
        clearance_m=clearance_m,
        contact_tolerance_m=limits.contact_tolerance_m,
        collision_tolerance_m=limits.collision_tolerance_m,
        prefer_sample_index=target_sample_index,
    )
    hip_trajectory = HipTrajectory2D(
        start_pose=start_scene.hip_pose,
        end_pose=target_scene.hip_pose if move_hip else None,
    )
    return SwingRequest2D(
        start=start,
        target=target,
        terrain=start_scene.terrain,
        hip_trajectory=hip_trajectory,
        swing_duration_s=swing_duration_s,
        sample_count=sample_count,
        constraints=limits,
    )


def flat_to_flat_swing_request_2d(
    *,
    theta_rad: float = float(np.deg2rad(60.0)),
    beta_rad: float = 0.0,
    start_hip_x_m: float = 0.0,
    step_length_m: float = 0.15,
    target_theta_rad: float | None = None,
    target_beta_rad: float | None = None,
    ground_height_m: float = 0.0,
    clearance_m: float = 0.03,
    swing_duration_s: float = 0.6,
    sample_count: int = 101,
    arc_samples: int = 241,
    constraints: SwingConstraints2D | None = None,
) -> SwingRequest2D:
    """The Step 1 reference case: flat ground to flat ground, ``z_TD = 0``.

    Both ends are produced by dropping a pose onto the ground, so the request
    describes two contact states the leg can actually hold.
    """

    start_scene = build_leg_on_surface_scene_2d(
        theta_rad,
        beta_rad,
        start_hip_x_m,
        ground_height_m,
        ground_height_m=ground_height_m,
        obstacle_x_start_m=None,
        arc_samples=arc_samples,
    )
    target_scene = build_leg_on_surface_scene_2d(
        theta_rad if target_theta_rad is None else target_theta_rad,
        beta_rad if target_beta_rad is None else target_beta_rad,
        start_hip_x_m + step_length_m,
        ground_height_m,
        ground_height_m=ground_height_m,
        obstacle_x_start_m=None,
        arc_samples=arc_samples,
    )
    return build_swing_request_2d(
        start_scene,
        target_scene,
        clearance_m=clearance_m,
        swing_duration_s=swing_duration_s,
        sample_count=sample_count,
        constraints=constraints,
    )


def day6_7_roll_end_swing_start_2d(
    path,
    *,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
) -> tuple[SingleLegRollingScene2D, SwingStartState2D]:
    """Read a Day 6--7 roll-end cache and express it as a swing start state.

    This is the seam described in the planning note: the state a rolling
    propagation stops in is the most realistic input a swing can have.  The
    cache stores the pose and the active rim sample, so the contact is
    recovered by re-running the same terrain query rather than by trusting a
    stored contact point.

    Returns the rebuilt scene as well, because plotting and later collision
    checks need the full leg geometry, not only the contact state.
    """

    import json

    state = json.loads(Path(path).read_text(encoding="utf-8"))
    hip_x_m, hip_z_m = state["hip_position_world_xz_m"]
    scene = build_single_leg_rolling_scene_2d(
        state["theta_rad"],
        state["beta_rad"],
        hip_x_m,
        hip_z_m,
        **state.get("scene_kwargs", {}),
    )
    start = swing_start_state_from_scene_2d(
        scene,
        contact_tolerance_m=contact_tolerance_m,
        collision_tolerance_m=collision_tolerance_m,
        prefer_sample_index=state.get("active_sample_index"),
    )
    cached_point = np.asarray(state["contact_point_world_xz_m"], dtype=float)
    drift_m = float(np.linalg.norm(start.contact_point_world_xz_m - cached_point))
    if drift_m > contact_tolerance_m:
        raise ValueError(
            f"rebuilt contact point differs from the cached one by {drift_m * 1e3:.3f} mm; "
            "the cache and the current geometry model disagree."
        )
    return scene, start
