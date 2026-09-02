"""Step 6 deterministic event scheduler for one straight-X obstacle.

The scheduler predicts Walk liftoff/touchdown events from the existing
``GaitGenerator3D`` phase offsets.  It emits segment *requests* only; it does
not run IK, generate trajectory rows, assemble segments, or export a CSV.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TypeAlias

import numpy as np

from legwheel.planners.gait_generator_3d import GaitGenerator3D
from legwheel.planners.obstacle_walk.terrain import (
    TouchdownStatus,
    WalkTerrain1D,
    query_touchdown_surface,
)
from legwheel.planners.obstacle_walk.types import LEG_ORDER, LegId, WalkState


class ScheduleRequestKind(str, Enum):
    FLAT_APPROACH = "flat_approach"
    STEP_UP = "step_up"
    TOP_SWING = "top_swing"
    STEP_DOWN = "step_down"
    CLEAR_OVER = "clear_over"
    GROUND_SWING = "ground_swing"
    FLAT_RECOVERY = "flat_recovery"


class ScheduleRejectReason(str, Enum):
    INVALID_INITIAL_STATE = "INVALID_INITIAL_STATE"
    UNSUPPORTED_MOTION = "UNSUPPORTED_MOTION"
    OBSTACLE_NOT_AHEAD = "OBSTACLE_NOT_AHEAD"
    NO_LEGAL_TOUCHDOWN = "NO_LEGAL_TOUCHDOWN"
    NO_OBSTACLE_INTERACTION = "NO_OBSTACLE_INTERACTION"
    MAX_EVENTS_EXCEEDED = "MAX_EVENTS_EXCEEDED"


class ObstacleScheduleError(ValueError):
    """A deterministic scheduler rejection with event/leg context."""

    def __init__(
        self,
        reason: ScheduleRejectReason,
        message: str,
        *,
        event_index: int | None = None,
        leg: LegId | str | None = None,
        nominal_touchdown_x_world_m: float | None = None,
    ):
        self.reason = ScheduleRejectReason(reason)
        self.event_index = event_index
        self.leg = None if leg is None else LegId(leg)
        self.nominal_touchdown_x_world_m = nominal_touchdown_x_world_m
        location = ""
        if event_index is not None:
            location += f" at event {event_index}"
        if self.leg is not None:
            location += f" for {self.leg.value}"
        super().__init__(f"{self.reason.value}{location}: {message}")


@dataclass(frozen=True)
class FlatSegmentRequest:
    """A flat segment bounded by a legal gait event."""

    kind: ScheduleRequestKind | str
    start_time_s: float
    end_time_s: float
    duration_s: float
    end_event_leg: LegId | str | None
    end_at_liftoff: bool
    aligned_cycle_count: int | None = None

    def __post_init__(self) -> None:
        kind = ScheduleRequestKind(self.kind)
        if kind not in {
            ScheduleRequestKind.FLAT_APPROACH,
            ScheduleRequestKind.FLAT_RECOVERY,
        }:
            raise ValueError("FlatSegmentRequest kind must be a flat request.")
        object.__setattr__(self, "kind", kind)
        for name in ("start_time_s", "end_time_s", "duration_s"):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) < 0.0:
                raise ValueError(f"{name} must be finite and non-negative.")
        if not np.isclose(self.end_time_s - self.start_time_s, self.duration_s):
            raise ValueError("flat request times must agree with duration_s.")
        if self.end_event_leg is not None:
            object.__setattr__(self, "end_event_leg", LegId(self.end_event_leg))
        if self.aligned_cycle_count is not None and self.aligned_cycle_count < 0:
            raise ValueError("aligned_cycle_count must be non-negative.")


@dataclass(frozen=True)
class SwingSegmentRequest:
    """One existing Walk swing event with a terrain-selected touchdown."""

    kind: ScheduleRequestKind | str
    event_index: int
    leg: LegId | str
    liftoff_time_s: float
    touchdown_time_s: float
    liftoff_sample_offset: int
    touchdown_sample_offset: int
    gait_cycle_phase_at_liftoff: float
    nominal_touchdown_world_m: tuple[float, float, float]
    touchdown_world_m: tuple[float, float, float]
    touchdown_bias_x_m: float
    from_surface_id: str
    target_surface_id: str

    def __post_init__(self) -> None:
        kind = ScheduleRequestKind(self.kind)
        if kind in {
            ScheduleRequestKind.FLAT_APPROACH,
            ScheduleRequestKind.FLAT_RECOVERY,
        }:
            raise ValueError("SwingSegmentRequest kind must be a swing request.")
        object.__setattr__(self, "kind", kind)
        object.__setattr__(self, "leg", LegId(self.leg))
        if self.event_index < 0:
            raise ValueError("event_index must be non-negative.")
        if self.liftoff_sample_offset < 0:
            raise ValueError("liftoff_sample_offset must be non-negative.")
        if self.touchdown_sample_offset <= self.liftoff_sample_offset:
            raise ValueError("touchdown must occur after liftoff.")
        if not 0.0 <= self.gait_cycle_phase_at_liftoff < 1.0:
            raise ValueError("gait_cycle_phase_at_liftoff must lie in [0, 1).")
        if not np.isfinite(self.liftoff_time_s) or not np.isfinite(
            self.touchdown_time_s
        ):
            raise ValueError("event times must be finite.")
        if self.touchdown_time_s <= self.liftoff_time_s:
            raise ValueError("touchdown_time_s must be after liftoff_time_s.")
        for name in ("nominal_touchdown_world_m", "touchdown_world_m"):
            point = tuple(float(value) for value in getattr(self, name))
            if len(point) != 3 or not np.all(np.isfinite(point)):
                raise ValueError(f"{name} must contain three finite values.")
            object.__setattr__(self, name, point)
        if not np.isfinite(self.touchdown_bias_x_m):
            raise ValueError("touchdown_bias_x_m must be finite.")
        if not self.from_surface_id or not self.target_surface_id:
            raise ValueError("surface IDs must be non-empty.")


SegmentRequest: TypeAlias = FlatSegmentRequest | SwingSegmentRequest


@dataclass(frozen=True)
class ObstacleWalkSchedule:
    """Deterministic request list and evidence, not generated joint motion."""

    requests: tuple[SegmentRequest, ...]
    swing_requests: tuple[SwingSegmentRequest, ...]
    cycle_sample_count: int
    first_transition_event_index: int
    completion_event_index: int
    all_four_top_observed: bool
    maximum_top_contact_count: int
    requested_post_distance_m: float
    scheduled_recovery_distance_m: float
    kinematic_feasibility_checked: bool = False
    full_geometry_collision_checked: bool = False

    def __post_init__(self) -> None:
        if not self.requests or not self.swing_requests:
            raise ValueError("schedule must contain requests and swing requests.")
        if self.cycle_sample_count <= 0:
            raise ValueError("cycle_sample_count must be positive.")
        if self.first_transition_event_index < 0:
            raise ValueError("first_transition_event_index must be non-negative.")
        if self.completion_event_index < self.first_transition_event_index:
            raise ValueError("completion_event_index precedes the first transition.")
        if not 0 <= self.maximum_top_contact_count <= 4:
            raise ValueError("maximum_top_contact_count must lie in [0, 4].")
        if self.all_four_top_observed != (self.maximum_top_contact_count == 4):
            raise ValueError("all_four_top_observed disagrees with contact count.")


def _phase_cycle(generator: GaitGenerator3D) -> np.ndarray:
    cycle_count = int(round(generator.T / generator.dt))
    if cycle_count < 4 or not np.isclose(
        cycle_count * generator.dt,
        generator.T,
        rtol=0.0,
        atol=1e-9,
    ):
        raise ValueError("period must contain an integer number of at least four dt samples.")
    stance_count = int(round(generator.stance_duty * cycle_count))
    base = np.zeros(cycle_count, dtype=np.int8)
    base[stance_count:] = 1
    phase = np.empty((cycle_count, 4), dtype=np.int8)
    for leg_index, offset in enumerate(generator.phase_offsets):
        shift = int(offset * cycle_count)
        phase[:, leg_index] = base[(np.arange(cycle_count) + shift) % cycle_count]
    return phase


def _touchdown_delta_samples(
    phase_cycle: np.ndarray,
    liftoff_cycle_index: int,
    leg_index: int,
) -> int:
    count = len(phase_cycle)
    for delta in range(1, count + 1):
        previous = phase_cycle[(liftoff_cycle_index + delta - 1) % count, leg_index]
        current = phase_cycle[(liftoff_cycle_index + delta) % count, leg_index]
        if previous == 1 and current == 0:
            return delta
    raise RuntimeError("phase cycle contains a liftoff without touchdown.")


def _touchdown_templates_body(generator: GaitGenerator3D) -> np.ndarray:
    points = np.empty((4, 3), dtype=float)
    for leg_index, planner in enumerate(generator.planners):
        q_touchdown = planner._level_touchdown_q()
        alpha_deg, width_m = planner.kin.foot_rim_contact_fk(*q_touchdown)
        points[leg_index] = planner.kin.forward_kinematics(
            *q_touchdown,
            alpha=alpha_deg,
            w=width_m,
        )
    return points


def _classify_swing(
    from_surface: str,
    target_surface: str,
    ground_surface: str,
    top_surface: str,
    corridor_overlaps_obstacle: bool,
) -> ScheduleRequestKind:
    if from_surface == ground_surface and target_surface == top_surface:
        return ScheduleRequestKind.STEP_UP
    if from_surface == top_surface and target_surface == top_surface:
        return ScheduleRequestKind.TOP_SWING
    if from_surface == top_surface and target_surface == ground_surface:
        return ScheduleRequestKind.STEP_DOWN
    if corridor_overlaps_obstacle:
        return ScheduleRequestKind.CLEAR_OVER
    return ScheduleRequestKind.GROUND_SWING


def _validate_scheduler_inputs(
    generator: GaitGenerator3D,
    initial_state: WalkState,
    terrain: WalkTerrain1D,
) -> None:
    if not isinstance(generator, GaitGenerator3D) or generator.gait_type != "Walk":
        raise TypeError("generator must be a Walk GaitGenerator3D.")
    if not isinstance(initial_state, WalkState):
        raise TypeError("initial_state must be WalkState.")
    if not isinstance(terrain, WalkTerrain1D):
        raise TypeError("terrain must be WalkTerrain1D.")
    unsupported = (
        abs(float(generator.omega_z)) > 1e-12
        or abs(float(generator.v_com[1])) > 1e-12
        or np.max(np.abs(initial_state.body_pose_world[3:])) > 1e-12
    )
    if unsupported:
        raise ObstacleScheduleError(
            ScheduleRejectReason.UNSUPPORTED_MOTION,
            "Step 6 supports straight +world-X Walk with fixed world y and zero body RPY",
        )
    if float(generator.v_com[0]) <= 0.0:
        raise ObstacleScheduleError(
            ScheduleRejectReason.UNSUPPORTED_MOTION,
            "forward velocity must be positive",
        )
    if any(item != terrain.ground_surface_id for item in initial_state.surface_ids):
        raise ObstacleScheduleError(
            ScheduleRejectReason.INVALID_INITIAL_STATE,
            "first scheduler version requires four initial ground contacts",
        )
    if not np.all(initial_state.phase == 0):
        raise ObstacleScheduleError(
            ScheduleRejectReason.INVALID_INITIAL_STATE,
            "initial state must be an all-stance event boundary, not mid-swing",
        )
    for leg_index, point in enumerate(initial_state.foot_contact_points_world_m):
        query = query_touchdown_surface(terrain, float(point[0]))
        if (
            not query.is_legal
            or query.surface_id != initial_state.surface_ids[leg_index]
            or abs(float(point[2]) - float(query.surface_height_world_m)) > 1e-3
        ):
            raise ObstacleScheduleError(
                ScheduleRejectReason.INVALID_INITIAL_STATE,
                "initial foot contact does not match the terrain surface",
                leg=LEG_ORDER[leg_index],
            )
    if terrain.obstacle.x_start_m <= float(
        np.max(initial_state.foot_contact_points_world_m[:, 0])
    ):
        raise ObstacleScheduleError(
            ScheduleRejectReason.OBSTACLE_NOT_AHEAD,
            "obstacle front must be ahead of every initial foot contact",
        )


def schedule_obstacle_walk(
    generator: GaitGenerator3D,
    initial_state: WalkState,
    terrain: WalkTerrain1D,
    *,
    post_distance_m: float = 0.30,
    maximum_touchdown_bias_m: float = 0.04,
    maximum_events: int = 512,
) -> ObstacleWalkSchedule:
    """Return deterministic event-aligned requests for one rectangular obstacle.

    Event times and touchdown positions use the nominal periodic Walk clock.
    Step 7 must generate each requested segment from the previous final state
    and rerun kinematic, continuity, and collision validation.
    """

    _validate_scheduler_inputs(generator, initial_state, terrain)
    for name, value in (
        ("post_distance_m", post_distance_m),
        ("maximum_touchdown_bias_m", maximum_touchdown_bias_m),
    ):
        if not np.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative.")
    if not isinstance(maximum_events, int) or isinstance(maximum_events, bool):
        raise TypeError("maximum_events must be an integer.")
    if maximum_events <= 0:
        raise ValueError("maximum_events must be positive.")

    phase_cycle = _phase_cycle(generator)
    cycle_count = len(phase_cycle)
    current_cycle_index = int(round(initial_state.gait_cycle_phase * cycle_count)) % cycle_count
    if not np.array_equal(phase_cycle[current_cycle_index], initial_state.phase):
        raise ObstacleScheduleError(
            ScheduleRejectReason.INVALID_INITIAL_STATE,
            "gait_cycle_phase does not match the existing Walk phase pattern",
        )

    touchdown_body = _touchdown_templates_body(generator)
    velocity_x = float(generator.v_com[0])
    obstacle = terrain.obstacle
    current_contact_x = initial_state.foot_contact_points_world_m[:, 0].copy()
    current_surfaces = list(initial_state.surface_ids)
    swing_requests: list[SwingSegmentRequest] = []
    first_transition_event: int | None = None
    first_liftoff_time = 0.0
    first_liftoff_leg: LegId | None = None
    completion_event: int | None = None
    completion_time = 0.0
    maximum_top_count = 0

    event_index = 0
    sample_offset = 0
    while event_index < maximum_events:
        sample_offset += 1
        previous_index = (current_cycle_index + sample_offset - 1) % cycle_count
        cycle_index = (current_cycle_index + sample_offset) % cycle_count
        liftoff_indices = np.flatnonzero(
            (phase_cycle[previous_index] == 0) & (phase_cycle[cycle_index] == 1)
        )
        if len(liftoff_indices) == 0:
            continue
        if len(liftoff_indices) != 1:
            raise ObstacleScheduleError(
                ScheduleRejectReason.INVALID_INITIAL_STATE,
                "Walk phase pattern produced simultaneous liftoffs",
                event_index=event_index,
            )

        leg_index = int(liftoff_indices[0])
        leg = LEG_ORDER[leg_index]
        touchdown_delta = _touchdown_delta_samples(phase_cycle, cycle_index, leg_index)
        touchdown_sample = sample_offset + touchdown_delta
        liftoff_time = sample_offset * generator.dt
        touchdown_time = touchdown_sample * generator.dt
        nominal_x = (
            float(initial_state.body_pose_world[0])
            + velocity_x * touchdown_time
            + float(touchdown_body[leg_index, 0])
        )
        nominal_y = float(initial_state.body_pose_world[1]) + float(
            touchdown_body[leg_index, 1]
        )
        nominal_query = query_touchdown_surface(terrain, nominal_x)
        target_x = nominal_x
        if nominal_query.status is TouchdownStatus.NO_LEGAL_TOUCHDOWN:
            target_x = float(
                np.clip(
                    nominal_x,
                    obstacle.legal_top_x_min_m,
                    obstacle.legal_top_x_max_m,
                )
            )
            bias = target_x - nominal_x
            if abs(bias) > maximum_touchdown_bias_m + 1e-12:
                raise ObstacleScheduleError(
                    ScheduleRejectReason.NO_LEGAL_TOUCHDOWN,
                    f"nearest legal top requires x bias {bias:.6g} m, limit is "
                    f"{maximum_touchdown_bias_m:.6g} m",
                    event_index=event_index,
                    leg=leg,
                    nominal_touchdown_x_world_m=nominal_x,
                )
            target_query = query_touchdown_surface(terrain, target_x)
        else:
            bias = 0.0
            target_query = nominal_query
        if not target_query.is_legal:
            raise ObstacleScheduleError(
                ScheduleRejectReason.NO_LEGAL_TOUCHDOWN,
                "touchdown adjustment did not produce a legal surface",
                event_index=event_index,
                leg=leg,
                nominal_touchdown_x_world_m=nominal_x,
            )

        from_surface = current_surfaces[leg_index]
        target_surface = str(target_query.surface_id)
        corridor_min = min(float(current_contact_x[leg_index]), target_x)
        corridor_max = max(float(current_contact_x[leg_index]), target_x)
        overlaps = (
            corridor_max >= obstacle.x_start_m
            and corridor_min <= obstacle.x_end_m
        )
        kind = _classify_swing(
            from_surface,
            target_surface,
            terrain.ground_surface_id,
            obstacle.top_surface_id,
            overlaps,
        )
        interacts = kind in {
            ScheduleRequestKind.STEP_UP,
            ScheduleRequestKind.TOP_SWING,
            ScheduleRequestKind.STEP_DOWN,
            ScheduleRequestKind.CLEAR_OVER,
        }
        if first_transition_event is None and interacts:
            first_transition_event = event_index
            first_liftoff_time = liftoff_time
            first_liftoff_leg = leg

        if first_transition_event is not None:
            target_z = float(target_query.surface_height_world_m)
            nominal_z = target_z
            swing_requests.append(
                SwingSegmentRequest(
                    kind=kind,
                    event_index=event_index,
                    leg=leg,
                    liftoff_time_s=liftoff_time,
                    touchdown_time_s=touchdown_time,
                    liftoff_sample_offset=sample_offset,
                    touchdown_sample_offset=touchdown_sample,
                    gait_cycle_phase_at_liftoff=cycle_index / cycle_count,
                    nominal_touchdown_world_m=(nominal_x, nominal_y, nominal_z),
                    touchdown_world_m=(target_x, nominal_y, target_z),
                    touchdown_bias_x_m=bias,
                    from_surface_id=from_surface,
                    target_surface_id=target_surface,
                )
            )

        current_contact_x[leg_index] = target_x
        current_surfaces[leg_index] = target_surface
        top_count = sum(item == obstacle.top_surface_id for item in current_surfaces)
        maximum_top_count = max(maximum_top_count, top_count)

        if (
            first_transition_event is not None
            and all(item == terrain.ground_surface_id for item in current_surfaces)
            and float(np.min(current_contact_x)) > obstacle.x_end_m
        ):
            completion_event = event_index
            completion_time = touchdown_time
            break
        event_index += 1

    if first_transition_event is None:
        raise ObstacleScheduleError(
            ScheduleRejectReason.NO_OBSTACLE_INTERACTION,
            f"no swing interacted with the obstacle within {maximum_events} events",
        )
    if completion_event is None:
        raise ObstacleScheduleError(
            ScheduleRejectReason.MAX_EVENTS_EXCEEDED,
            f"traversal did not finish within {maximum_events} events",
            event_index=event_index,
        )
    assert first_liftoff_leg is not None

    approach = FlatSegmentRequest(
        kind=ScheduleRequestKind.FLAT_APPROACH,
        start_time_s=0.0,
        end_time_s=first_liftoff_time,
        duration_s=first_liftoff_time,
        end_event_leg=first_liftoff_leg,
        end_at_liftoff=True,
    )
    recovery_cycles = (
        0
        if post_distance_m == 0.0
        else int(np.ceil(post_distance_m / (velocity_x * generator.T)))
    )
    recovery_duration = recovery_cycles * generator.T
    recovery = FlatSegmentRequest(
        kind=ScheduleRequestKind.FLAT_RECOVERY,
        start_time_s=completion_time,
        end_time_s=completion_time + recovery_duration,
        duration_s=recovery_duration,
        end_event_leg=None,
        end_at_liftoff=False,
        aligned_cycle_count=recovery_cycles,
    )
    requests: tuple[SegmentRequest, ...] = (
        approach,
        *swing_requests,
        recovery,
    )
    return ObstacleWalkSchedule(
        requests=requests,
        swing_requests=tuple(swing_requests),
        cycle_sample_count=cycle_count,
        first_transition_event_index=first_transition_event,
        completion_event_index=completion_event,
        all_four_top_observed=maximum_top_count == 4,
        maximum_top_contact_count=maximum_top_count,
        requested_post_distance_m=post_distance_m,
        scheduled_recovery_distance_m=recovery_duration * velocity_x,
    )


def schedule_to_dict(schedule: ObstacleWalkSchedule) -> dict[str, object]:
    """Convert a schedule to JSON-safe evidence without generating motion."""

    if not isinstance(schedule, ObstacleWalkSchedule):
        raise TypeError("schedule must be ObstacleWalkSchedule.")
    request_rows: list[dict[str, object]] = []
    for request in schedule.requests:
        if isinstance(request, FlatSegmentRequest):
            request_rows.append(
                {
                    "kind": request.kind.value,
                    "start_time_s": request.start_time_s,
                    "end_time_s": request.end_time_s,
                    "duration_s": request.duration_s,
                    "end_event_leg": (
                        None
                        if request.end_event_leg is None
                        else request.end_event_leg.value
                    ),
                    "end_at_liftoff": request.end_at_liftoff,
                    "aligned_cycle_count": request.aligned_cycle_count,
                }
            )
        else:
            request_rows.append(
                {
                    "kind": request.kind.value,
                    "event_index": request.event_index,
                    "leg": request.leg.value,
                    "liftoff_time_s": request.liftoff_time_s,
                    "touchdown_time_s": request.touchdown_time_s,
                    "liftoff_sample_offset": request.liftoff_sample_offset,
                    "touchdown_sample_offset": request.touchdown_sample_offset,
                    "gait_cycle_phase_at_liftoff": (
                        request.gait_cycle_phase_at_liftoff
                    ),
                    "nominal_touchdown_world_m": list(
                        request.nominal_touchdown_world_m
                    ),
                    "touchdown_world_m": list(request.touchdown_world_m),
                    "touchdown_bias_x_m": request.touchdown_bias_x_m,
                    "from_surface_id": request.from_surface_id,
                    "target_surface_id": request.target_surface_id,
                }
            )
    return {
        "status": "deterministic segment-request schedule prototype",
        "cycle_sample_count": schedule.cycle_sample_count,
        "first_transition_event_index": schedule.first_transition_event_index,
        "completion_event_index": schedule.completion_event_index,
        "all_four_top_observed": schedule.all_four_top_observed,
        "maximum_top_contact_count": schedule.maximum_top_contact_count,
        "requested_post_distance_m": schedule.requested_post_distance_m,
        "scheduled_recovery_distance_m": schedule.scheduled_recovery_distance_m,
        "kinematic_feasibility_checked": schedule.kinematic_feasibility_checked,
        "full_geometry_collision_checked": schedule.full_geometry_collision_checked,
        "requests": request_rows,
    }
