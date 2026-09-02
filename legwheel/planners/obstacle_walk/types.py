"""Data contracts for continuous, segment-based offline Walk planning.

Coordinate and ordering contract
--------------------------------
World positions use metres in ``{W}``, with ``+x`` forward, ``+y`` left, and
``+z`` upward.  Body pose rows are ``[x, y, z, roll, pitch, yaw]`` and angles
are radians.  Joint arrays are always planner leg-major order
``[FL, FR, RR, RL] x [theta, beta, gamma]``.  Hardware CSV reordering happens
only at the final exporter boundary.

``foot_contact_points_world_m`` stores the selected lowest rim point for every
leg.  It is an active terrain contact only where ``contact_active`` is true;
during swing it is kinematic metadata, not a claim of physical contact.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Sequence

import numpy as np
from numpy.typing import NDArray


class LegId(str, Enum):
    FL = "FL"
    FR = "FR"
    RR = "RR"
    RL = "RL"


LEG_ORDER = (LegId.FL, LegId.FR, LegId.RR, LegId.RL)
JOINT_ORDER = ("theta", "beta", "gamma")


class CommandOrder(str, Enum):
    """The sole command order accepted inside a trajectory segment."""

    PLANNER_LEG_MAJOR = "FL_FR_RR_RL__theta_beta_gamma"


class SegmentType(str, Enum):
    """Implemented segment semantics; later steps may extend this enum."""

    FLAT = "flat"
    SWING = "swing"
    STANCE = "stance"
    COMPOSITE = "composite"


def _finite_array(
    value: object,
    field_name: str,
    shape: tuple[int | None, ...],
) -> NDArray[np.float64]:
    array = np.asarray(value, dtype=float)
    valid_shape = array.ndim == len(shape) and all(
        expected is None or actual == expected
        for actual, expected in zip(array.shape, shape)
    )
    if not valid_shape:
        expected_shape = tuple("N" if item is None else item for item in shape)
        raise ValueError(f"{field_name} must have shape {expected_shape}; got {array.shape}.")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{field_name} must contain only finite values.")
    array = array.copy()
    array.setflags(write=False)
    return array


def _phase_array(
    value: object,
    field_name: str,
    shape: tuple[int | None, ...],
) -> NDArray[np.int8]:
    raw = np.asarray(value)
    valid_shape = raw.ndim == len(shape) and all(
        expected is None or actual == expected
        for actual, expected in zip(raw.shape, shape)
    )
    if not valid_shape:
        expected_shape = tuple("N" if item is None else item for item in shape)
        raise ValueError(f"{field_name} must have shape {expected_shape}; got {raw.shape}.")
    if not np.all(np.isin(raw, (0, 1))):
        raise ValueError(f"{field_name} values must be stance=0 or swing=1.")
    phase = raw.astype(np.int8, copy=True)
    phase.setflags(write=False)
    return phase


def _bool_array(
    value: object,
    field_name: str,
    shape: tuple[int | None, ...],
) -> NDArray[np.bool_]:
    raw = np.asarray(value)
    valid_shape = raw.ndim == len(shape) and all(
        expected is None or actual == expected
        for actual, expected in zip(raw.shape, shape)
    )
    if not valid_shape:
        expected_shape = tuple("N" if item is None else item for item in shape)
        raise ValueError(f"{field_name} must have shape {expected_shape}; got {raw.shape}.")
    if not np.all(np.isin(raw, (False, True, 0, 1))):
        raise ValueError(f"{field_name} must contain only boolean values.")
    array = raw.astype(bool, copy=True)
    array.setflags(write=False)
    return array


def _surface_row(values: Sequence[str], field_name: str) -> tuple[str, str, str, str]:
    row = tuple(values)
    if len(row) != 4 or any(not isinstance(item, str) or not item for item in row):
        raise ValueError(f"{field_name} must contain four non-empty surface IDs.")
    return row  # type: ignore[return-value]


def _surface_grid(
    values: Sequence[Sequence[str]], count: int
) -> tuple[tuple[str, str, str, str], ...]:
    rows = tuple(_surface_row(row, "surface_ids") for row in values)
    if len(rows) != count:
        raise ValueError(f"surface_ids must have shape ({count}, 4); got ({len(rows)}, 4).")
    return rows


@dataclass(frozen=True)
class WalkState:
    """All state needed to begin the next offline Walk segment."""

    joint_position_rad: NDArray[np.float64]
    previous_joint_position_rad: NDArray[np.float64] | None
    body_pose_world: NDArray[np.float64]
    foot_contact_points_world_m: NDArray[np.float64]
    phase: NDArray[np.int8]
    contact_active: NDArray[np.bool_]
    surface_ids: Sequence[str]
    gait_cycle_phase: float
    next_swing_leg: LegId | str | None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "joint_position_rad",
            _finite_array(self.joint_position_rad, "joint_position_rad", (4, 3)),
        )
        if self.previous_joint_position_rad is not None:
            object.__setattr__(
                self,
                "previous_joint_position_rad",
                _finite_array(
                    self.previous_joint_position_rad,
                    "previous_joint_position_rad",
                    (4, 3),
                ),
            )
        object.__setattr__(
            self,
            "body_pose_world",
            _finite_array(self.body_pose_world, "body_pose_world", (6,)),
        )
        object.__setattr__(
            self,
            "foot_contact_points_world_m",
            _finite_array(
                self.foot_contact_points_world_m,
                "foot_contact_points_world_m",
                (4, 3),
            ),
        )
        phase = _phase_array(self.phase, "phase", (4,))
        active = _bool_array(self.contact_active, "contact_active", (4,))
        if not np.array_equal(active, phase == 0):
            raise ValueError("contact_active must equal (phase == 0) for the Step 1 flat Walk.")
        object.__setattr__(self, "phase", phase)
        object.__setattr__(self, "contact_active", active)
        object.__setattr__(self, "surface_ids", _surface_row(self.surface_ids, "surface_ids"))
        if not np.isfinite(self.gait_cycle_phase) or not 0.0 <= self.gait_cycle_phase < 1.0:
            raise ValueError("gait_cycle_phase must be finite and lie in [0, 1).")
        if self.next_swing_leg is not None:
            object.__setattr__(self, "next_swing_leg", LegId(self.next_swing_leg))


@dataclass(frozen=True)
class TrajectorySegment:
    """Synchronized samples and boundary states for one Walk segment."""

    time_s: NDArray[np.float64]
    commands_rad: NDArray[np.float64]
    phase: NDArray[np.int8]
    body_pose_world: NDArray[np.float64]
    foot_contact_points_world_m: NDArray[np.float64]
    contact_active: NDArray[np.bool_]
    gait_cycle_phase: NDArray[np.float64]
    surface_ids: Sequence[Sequence[str]]
    start_state: WalkState
    final_state: WalkState
    dt_s: float
    segment_type: SegmentType | str = SegmentType.FLAT
    swing_leg: LegId | str | None = None
    command_order: CommandOrder | str = CommandOrder.PLANNER_LEG_MAJOR

    def __post_init__(self) -> None:
        time = _finite_array(self.time_s, "time_s", (None,))
        if len(time) == 0:
            raise ValueError("time_s must contain at least one sample.")
        if not np.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive.")
        if time[0] != 0.0 or (len(time) > 1 and not np.allclose(np.diff(time), self.dt_s)):
            raise ValueError("time_s must start at zero and advance uniformly by dt_s.")
        count = len(time)
        commands = _finite_array(self.commands_rad, "commands_rad", (count, 4, 3))
        phase = _phase_array(self.phase, "phase", (count, 4))
        body_pose = _finite_array(self.body_pose_world, "body_pose_world", (count, 6))
        foot_points = _finite_array(
            self.foot_contact_points_world_m,
            "foot_contact_points_world_m",
            (count, 4, 3),
        )
        active = _bool_array(self.contact_active, "contact_active", (count, 4))
        if not np.array_equal(active, phase == 0):
            raise ValueError("contact_active must equal (phase == 0) for the Step 1 flat Walk.")
        gait_phase = _finite_array(self.gait_cycle_phase, "gait_cycle_phase", (count,))
        if np.any((gait_phase < 0.0) | (gait_phase >= 1.0)):
            raise ValueError("gait_cycle_phase values must lie in [0, 1).")
        surfaces = _surface_grid(self.surface_ids, count)

        if not isinstance(self.start_state, WalkState) or not isinstance(
            self.final_state, WalkState
        ):
            raise TypeError("start_state and final_state must be WalkState instances.")
        for label, state, index in (
            ("start_state", self.start_state, 0),
            ("final_state", self.final_state, -1),
        ):
            if not np.array_equal(state.joint_position_rad, commands[index]):
                raise ValueError(f"{label} joint position does not match its boundary sample.")
            if not np.array_equal(state.phase, phase[index]):
                raise ValueError(f"{label} phase does not match its boundary sample.")
            if not np.array_equal(state.body_pose_world, body_pose[index]):
                raise ValueError(f"{label} body pose does not match its boundary sample.")
            if not np.array_equal(state.foot_contact_points_world_m, foot_points[index]):
                raise ValueError(f"{label} foot points do not match its boundary sample.")
            if state.surface_ids != surfaces[index]:
                raise ValueError(f"{label} surface IDs do not match its boundary sample.")
            if state.gait_cycle_phase != gait_phase[index]:
                raise ValueError(f"{label} gait cycle phase does not match its boundary sample.")

        object.__setattr__(self, "time_s", time)
        object.__setattr__(self, "commands_rad", commands)
        object.__setattr__(self, "phase", phase)
        object.__setattr__(self, "body_pose_world", body_pose)
        object.__setattr__(self, "foot_contact_points_world_m", foot_points)
        object.__setattr__(self, "contact_active", active)
        object.__setattr__(self, "gait_cycle_phase", gait_phase)
        object.__setattr__(self, "surface_ids", surfaces)
        object.__setattr__(self, "segment_type", SegmentType(self.segment_type))
        object.__setattr__(self, "command_order", CommandOrder(self.command_order))
        if self.swing_leg is not None:
            object.__setattr__(self, "swing_leg", LegId(self.swing_leg))

    @property
    def sample_count(self) -> int:
        return len(self.time_s)

    def to_planner_commands(self) -> NDArray[np.float64]:
        """Return a writable ``(N, 12)`` copy in planner leg-major order."""

        return self.commands_rad.reshape(self.sample_count, 12).copy()

    def to_phase_array(self) -> NDArray[np.int8]:
        """Return a writable ``(N, 4)`` copy for the existing phase exporter."""

        return self.phase.copy()
