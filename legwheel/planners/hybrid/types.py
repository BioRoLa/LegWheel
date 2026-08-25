"""Core data contracts for terrain-aware hybrid gait planning.

Coordinate contract
-------------------
All positions in these public planning types are in the world frame ``{W}``,
in metres, with ``+x`` forward, ``+y`` left, and ``+z`` upward when the robot
body is at its reference orientation.  Angles are in radians.

These types deliberately contain no contact-query or gait-execution logic.
They are the stable interface between those modules.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable, Sequence

import numpy as np
from numpy.typing import NDArray

Vector3 = NDArray[np.float64]


def _vector3(value: Iterable[float], field_name: str) -> Vector3:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"{field_name} must contain exactly three values; got {vector.shape}.")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{field_name} must contain only finite values.")
    vector = vector.copy()
    vector.setflags(write=False)
    return vector


def _array(
    value: object,
    field_name: str,
    *,
    shape: tuple[int | None, ...],
) -> NDArray[np.float64]:
    """Return a finite, immutable float array with a partly variable shape."""

    array = np.asarray(value, dtype=float)
    valid_shape = array.ndim == len(shape) and all(
        expected is None or actual == expected
        for actual, expected in zip(array.shape, shape)
    )
    if not valid_shape:
        expected = tuple("N" if item is None else item for item in shape)
        raise ValueError(f"{field_name} must have shape {expected}; got {array.shape}.")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{field_name} must contain only finite values.")
    array = array.copy()
    array.setflags(write=False)
    return array


class RimId(str, Enum):
    """Stable semantic rim names; independent of legacy numeric indices."""

    FOOT = "foot_rim"
    LEFT = "left_rim"
    RIGHT = "right_rim"


class MotionMode(str, Enum):
    """Per-leg motion primitive selected during offline planning."""

    ROLL = "ROLL"
    SWING = "SWING"


@dataclass(frozen=True)
class ContactState:
    """One selected terrain contact state in world coordinates."""

    rim: RimId
    alpha_rad: float
    point_world_m: Vector3
    terrain_surface_id: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "rim", RimId(self.rim))
        object.__setattr__(self, "point_world_m", _vector3(self.point_world_m, "point_world_m"))
        if not np.isfinite(self.alpha_rad):
            raise ValueError("alpha_rad must be finite.")
        if not self.terrain_surface_id:
            raise ValueError("terrain_surface_id must not be empty.")


@dataclass(frozen=True)
class ContactCandidate:
    """A sampled rim/terrain contact considered by the contact query."""

    rim: RimId
    alpha_rad: float
    point_world_m: Vector3
    terrain_gap_m: float
    edge_margin_m: float
    collision_free: bool
    terrain_surface_id: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "rim", RimId(self.rim))
        object.__setattr__(self, "point_world_m", _vector3(self.point_world_m, "point_world_m"))
        for name in ("alpha_rad", "terrain_gap_m", "edge_margin_m"):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite.")
        if self.edge_margin_m < 0.0:
            raise ValueError("edge_margin_m must be non-negative.")
        if not self.terrain_surface_id:
            raise ValueError("terrain_surface_id must not be empty.")


@dataclass(frozen=True)
class SwingTarget:
    """World-frame touchdown request passed to the Cartesian swing planner."""

    target_position_world_m: Vector3
    target_rim: RimId
    target_alpha_rad: float
    clearance_m: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "target_position_world_m",
            _vector3(self.target_position_world_m, "target_position_world_m"),
        )
        object.__setattr__(self, "target_rim", RimId(self.target_rim))
        if not np.isfinite(self.target_alpha_rad):
            raise ValueError("target_alpha_rad must be finite.")
        if not np.isfinite(self.clearance_m) or self.clearance_m < 0.0:
            raise ValueError("clearance_m must be finite and non-negative.")


@dataclass(frozen=True)
class RectangularObstacle:
    """Axis-aligned box on top of the terrain base plane, in ``{W}``."""

    surface_id: str
    min_xy_world_m: NDArray[np.float64]
    max_xy_world_m: NDArray[np.float64]
    height_m: float

    def __post_init__(self) -> None:
        minimum = np.asarray(self.min_xy_world_m, dtype=float)
        maximum = np.asarray(self.max_xy_world_m, dtype=float)
        if minimum.shape != (2,) or maximum.shape != (2,):
            raise ValueError("Obstacle min/max XY values must each have shape (2,).")
        if not np.all(np.isfinite(minimum)) or not np.all(np.isfinite(maximum)):
            raise ValueError("Obstacle bounds must be finite.")
        if np.any(maximum <= minimum):
            raise ValueError("Obstacle max_xy_world_m must be greater than min_xy_world_m.")
        if not np.isfinite(self.height_m) or self.height_m <= 0.0:
            raise ValueError("Obstacle height_m must be finite and positive.")
        if not self.surface_id:
            raise ValueError("surface_id must not be empty.")
        minimum, maximum = minimum.copy(), maximum.copy()
        minimum.setflags(write=False)
        maximum.setflags(write=False)
        object.__setattr__(self, "min_xy_world_m", minimum)
        object.__setattr__(self, "max_xy_world_m", maximum)


@dataclass(frozen=True)
class TerrainProfile:
    """Day 1 terrain contract: a base plane plus axis-aligned boxes."""

    ground_height_m: float = 0.0
    obstacles: tuple[RectangularObstacle, ...] = field(default_factory=tuple)
    ground_surface_id: str = "ground"

    def __post_init__(self) -> None:
        if not np.isfinite(self.ground_height_m):
            raise ValueError("ground_height_m must be finite.")
        if not self.ground_surface_id:
            raise ValueError("ground_surface_id must not be empty.")
        obstacles = tuple(self.obstacles)
        if not all(isinstance(item, RectangularObstacle) for item in obstacles):
            raise TypeError("obstacles must contain only RectangularObstacle values.")
        ids = [self.ground_surface_id, *(item.surface_id for item in obstacles)]
        if len(ids) != len(set(ids)):
            raise ValueError("Terrain surface IDs must be unique.")
        object.__setattr__(self, "obstacles", obstacles)


@dataclass(frozen=True)
class InitialRobotState:
    """Robot state at the beginning of an offline planning request."""

    body_position_world_m: Vector3
    body_rpy_rad: Vector3
    joint_position_rad: NDArray[np.float64]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "body_position_world_m",
            _vector3(self.body_position_world_m, "body_position_world_m"),
        )
        object.__setattr__(self, "body_rpy_rad", _vector3(self.body_rpy_rad, "body_rpy_rad"))
        object.__setattr__(
            self,
            "joint_position_rad",
            _array(self.joint_position_rad, "joint_position_rad", shape=(4, 3)),
        )


@dataclass(frozen=True)
class TraversalGoal:
    """Requested world-frame body path for the complete traversal."""

    body_path_world_m: NDArray[np.float64]

    def __post_init__(self) -> None:
        path = _array(self.body_path_world_m, "body_path_world_m", shape=(None, 3))
        if len(path) < 2:
            raise ValueError("body_path_world_m must contain at least start and goal points.")
        object.__setattr__(self, "body_path_world_m", path)


@dataclass(frozen=True)
class GaitConstraints:
    """Algorithm-independent limits applied to offline trajectory planning."""

    minimum_clearance_m: float = 0.01
    minimum_stability_margin_m: float = 0.0
    sample_period_s: float = 0.01
    maximum_swing_count: int | None = None

    def __post_init__(self) -> None:
        for name in ("minimum_clearance_m", "minimum_stability_margin_m"):
            value = getattr(self, name)
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative.")
        if not np.isfinite(self.sample_period_s) or self.sample_period_s <= 0.0:
            raise ValueError("sample_period_s must be finite and positive.")
        if self.maximum_swing_count is not None and self.maximum_swing_count < 0:
            raise ValueError("maximum_swing_count must be non-negative or None.")


@dataclass(frozen=True)
class HybridPlanningRequest:
    """Complete, deterministic input to the offline hybrid-gait planner."""

    terrain: TerrainProfile
    initial_state: InitialRobotState
    goal: TraversalGoal
    constraints: GaitConstraints = field(default_factory=GaitConstraints)

    def __post_init__(self) -> None:
        if not isinstance(self.terrain, TerrainProfile):
            raise TypeError("terrain must be a TerrainProfile.")
        if not isinstance(self.initial_state, InitialRobotState):
            raise TypeError("initial_state must be an InitialRobotState.")
        if not isinstance(self.goal, TraversalGoal):
            raise TypeError("goal must be a TraversalGoal.")
        if not isinstance(self.constraints, GaitConstraints):
            raise TypeError("constraints must be GaitConstraints.")


@dataclass(frozen=True)
class HybridTrajectory:
    """Synchronized full-traversal result produced before robot execution.

    Array shapes use ``N`` time samples, four legs, and three joints ordered as
    ``theta, beta, gamma``.  Contact metadata remains present during swing and
    describes the selected target contact; ``contact_active`` distinguishes
    physical contact from a planned target.
    """

    time_s: NDArray[np.float64]
    body_pose_world: NDArray[np.float64]
    joint_position_rad: NDArray[np.float64]
    modes: Sequence[Sequence[MotionMode | str]]
    rims: Sequence[Sequence[RimId | str]]
    alpha_rad: NDArray[np.float64]
    foothold_world_m: NDArray[np.float64]
    contact_active: NDArray[np.bool_]
    swing_phase: NDArray[np.float64]
    stability_margin_m: NDArray[np.float64]

    def __post_init__(self) -> None:
        time = _array(self.time_s, "time_s", shape=(None,))
        if len(time) == 0 or time[0] < 0.0 or np.any(np.diff(time) <= 0.0):
            raise ValueError("time_s must be non-empty, non-negative, and strictly increasing.")
        count = len(time)
        object.__setattr__(self, "time_s", time)
        for name, shape in (
            ("body_pose_world", (count, 6)),
            ("joint_position_rad", (count, 4, 3)),
            ("alpha_rad", (count, 4)),
            ("foothold_world_m", (count, 4, 3)),
            ("swing_phase", (count, 4)),
            ("stability_margin_m", (count,)),
        ):
            object.__setattr__(self, name, _array(getattr(self, name), name, shape=shape))

        active = np.asarray(self.contact_active, dtype=bool)
        if active.shape != (count, 4):
            raise ValueError(f"contact_active must have shape ({count}, 4); got {active.shape}.")
        active = active.copy()
        active.setflags(write=False)
        object.__setattr__(self, "contact_active", active)

        phases = self.swing_phase
        if np.any((phases < 0.0) | (phases > 1.0)):
            raise ValueError("swing_phase values must lie within [0, 1].")
        object.__setattr__(self, "modes", self._enum_grid(self.modes, MotionMode, count, "modes"))
        object.__setattr__(self, "rims", self._enum_grid(self.rims, RimId, count, "rims"))

    @staticmethod
    def _enum_grid(values, enum_type, count: int, field_name: str) -> tuple[tuple[Enum, ...], ...]:
        rows = tuple(tuple(enum_type(value) for value in row) for row in values)
        if len(rows) != count or any(len(row) != 4 for row in rows):
            raise ValueError(f"{field_name} must have shape ({count}, 4).")
        return rows

    @property
    def sample_count(self) -> int:
        return len(self.time_s)
