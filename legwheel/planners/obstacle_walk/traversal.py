"""Step 7 offline obstacle-walk traversal composer.

This module consumes the Step 3 terrain query, the Step 4 arbitrary-touchdown
swing, the Step 5 world-fixed mixed-height stance and the Step 2 continuity
validator, and emits one continuous ``TrajectorySegment`` that approaches a
single rectangular obstacle, climbs it, walks along its top, steps back down
and recovers on flat ground.

Motion contract
---------------
The generated motion is a **quasi-static crawl**, not the periodic rolling
Walk of ``GaitGenerator3D``:

* Exactly one leg swings at a time, in the leg order taken from the existing
  Walk ``phase_offsets``.
* The body advances only while all four legs are in stance, and it is at rest
  at every stance/swing boundary.
* Every crawl stance contact is held world-fixed.  The periodic Walk instead rolls
  the wheel through stance, so the two stance laws are different and a
  periodic-Walk segment cannot be spliced in at an arbitrary sample without a
  joint velocity step.  The approach is produced by the repository's original
  ``LaunchController`` followed by steady Walk and hands off only at a liftoff
  sample; the crawl stance absorbs that incoming velocity.
* The body stays level (``roll = pitch = yaw = 0``); only its height follows
  the support heights.

Foothold legality
-----------------
Step 3 answers a *point* query.  A wheel of outer radius ``R`` standing on the
ground next to a box of height ``H`` also has rim material inside the box
whenever the contact is closer than ``sqrt(2 R H - H^2)`` to a vertical face,
so Step 7 additionally excludes that band from every ground touchdown.  This
is a wheel-disc model of the rim, not a full linkage/body collision check.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Sequence

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import brentq

from legwheel.config import RobotParams
from legwheel.planners.gait_generator_3d import GaitGenerator3D, _hull_signed_margin
from legwheel.planners.obstacle_walk.assembly import (
    BoundaryContinuityReport,
    ContinuityTolerances,
    concatenate_segments,
)
from legwheel.planners.obstacle_walk.scheduler import (
    ObstacleScheduleError,
    # Reused so the Step 6 reference schedule and Step 7 agree on exactly the
    # same discrete Walk phase pattern.
    _phase_cycle,
    schedule_obstacle_walk,
    schedule_to_dict,
)
from legwheel.planners.obstacle_walk.handoff import (
    FlatHandoffError,
    blend_final_row_to,
    flat_approach_segment,
    flat_recovery_segment,
    handover_contact_height_error_m,
    legacy_launch_flat_approach_segment,
    resample_segment,
)
from legwheel.planners.obstacle_walk.stance import (
    StancePlanningError,
    # Reused so Step 5 stays the single implementation of the world-fixed
    # lowest-rim contact solve.
    _solve_lowest_rim_contact,
    generate_stance_segment,
)
from legwheel.planners.obstacle_walk.swing import (
    FullGeometryCollisionChecker,
    SwingPlanningError,
    generate_swing_segment,
)
from legwheel.planners.obstacle_walk.terrain import (
    RectangleObstacle1D,
    WalkTerrain1D,
    query_touchdown_surface,
)
from legwheel.planners.obstacle_walk.types import (
    LEG_ORDER,
    LegId,
    TrajectorySegment,
    WalkState,
)


BODY_POSE_POLICY = "level_body_min_contact_plus_stand_height_plus_lift_ratio_times_span"
STANCE_CONTACT_MODEL = "world_fixed_lowest_rim_contact_quasi_static"


class TraversalStage(str, Enum):
    APPROACH = "approach"
    STEP_UP = "step_up"
    TOP_SUPPORT = "top_support"
    STEP_DOWN = "step_down"
    RECOVERY = "recovery"


class TraversalRejectReason(str, Enum):
    UNSUPPORTED_REQUEST = "UNSUPPORTED_REQUEST"
    VELOCITY_GUARD_SCALED = "VELOCITY_GUARD_SCALED"
    STEP_TOO_SHORT_FOR_OBSTACLE = "STEP_TOO_SHORT_FOR_OBSTACLE"
    INITIAL_STATE_INFEASIBLE = "INITIAL_STATE_INFEASIBLE"
    FLAT_HANDOFF_INFEASIBLE = "FLAT_HANDOFF_INFEASIBLE"
    NO_FEASIBLE_TOUCHDOWN = "NO_FEASIBLE_TOUCHDOWN"
    MAX_EVENTS_EXCEEDED = "MAX_EVENTS_EXCEEDED"


class ObstacleTraversalError(ValueError):
    """A deterministic Step 7 rejection carrying the event that failed."""

    def __init__(
        self,
        reason: TraversalRejectReason,
        message: str,
        *,
        event_index: int | None = None,
        leg: LegId | str | None = None,
        attempts: Sequence[str] = (),
    ):
        self.reason = TraversalRejectReason(reason)
        self.event_index = event_index
        self.leg = None if leg is None else LegId(leg)
        self.attempts = tuple(attempts)
        location = ""
        if event_index is not None:
            location += f" at event {event_index}"
        if self.leg is not None:
            location += f" for {self.leg.value}"
        detail = "".join(f"\n  tried {item}" for item in self.attempts)
        super().__init__(f"{self.reason.value}{location}: {message}{detail}")


@dataclass(frozen=True)
class ObstacleWalkRequest:
    """Every input needed to reproduce one obstacle-walk trajectory."""

    obstacle_x_start_m: float
    obstacle_length_m: float
    obstacle_height_m: float
    # Lateral extent.  The planner is sagittal 2-D and never reads this; it is
    # carried so the exported metadata can describe the box the scene must
    # build, which is otherwise unrecoverable from the CSV.
    obstacle_width_m: float = 0.8
    edge_margin_m: float = 0.02
    stand_height_m: float = 0.25
    step_length_m: float = 0.15
    period_s: float = 4.0
    # Flat Walk speed is deliberately independent of the obstacle crawl's
    # step_length_m.  The known flat baseline uses 0.03 m/s and T=4 s, while
    # the 4 cm box still needs a 0.15 m crawl step to clear its wheel/edge
    # exclusion band.
    flat_walk_velocity_m_s: float | None = None
    # Swing height used only by the existing periodic flat Walk.  Keep the
    # original generator's normal 0.04 m default; the earlier 0.12 m value was
    # copied from one experimental filename and produced an excessive theta
    # excursion in this splice.
    flat_walk_step_height_m: float = 0.04
    # Stance duty of the periodic flat Walk, and therefore of the crawl's swing
    # timing.  The gait table's 0.75 makes the four swings tile the cycle
    # exactly, which leaves no four-leg overlap *and* puts the CoM on the
    # support-triangle edge for the two rear swings; 0.85 breaks that symmetry
    # and is the value the flat-Walk hardware runs use.  None keeps the table
    # default.
    stance_duty: float | None = None
    # Crawl swing duration.  ``None`` uses ``period_s / 4``, which pairs each
    # swing with an equally long stance advance and is what the crawl used
    # before ``stance_duty`` became configurable.
    crawl_swing_seconds: float | None = None
    # Sample period of the flat Walk sections.  They are the Walk's own samples
    # at the rate the flat-Walk hardware runs use, so that the spliced flat
    # stretches are the same trajectory the operator compares against rather
    # than a coarse plan interpolated up afterwards.  The crawl keeps ``dt_s``
    # and is resampled onto this grid before assembly.
    flat_walk_dt_s: float = 0.001
    # The obstacle search is intentionally coarse; the exporter resamples its
    # accepted result to the controller's fixed 1 ms contract.  The old 1 ms
    # planner default made the lowest-rim solve change branch at the first
    # obstacle event and was not a runnable default.
    dt_s: float = 0.02
    # 0.03 is the geometry-checked nominal clearance assuming a level body, but
    # a hardware run measured the flat Walk's own tilt (duty 0.85, no lateral
    # sway compensation) at up to 14 deg -- worth ~60 mm at the foot, twice the
    # nominal clearance -- so 0.05 buys margin against that tilt clipping the
    # obstacle.  0.02 was separately measured to let the rim clip the
    # top-front corner even with a level body.
    step_clearance_m: float = 0.05
    body_lift_ratio: float = 0.6
    approach_distance_m: float = 1.0
    post_distance_m: float = 0.30
    maximum_touchdown_bias_m: float = 0.16
    ground_face_safety_m: float = 0.001
    top_edge_safety_m: float = 0.001
    body_advance_fractions: tuple[float, ...] = (1.0, 0.75, 0.5, 0.25, 0.0)
    swing_duration_scales: tuple[float, ...] = (1.0, 1.5, 2.0)
    # Planning guard on the per-sample joint step, expressed as a rate so it
    # is dt independent.  The default is the peak the existing flat Walk
    # itself reaches at dt = 0.001 s (about 16 rad/s), so Step 7 never asks
    # for faster joints than the baseline gait already commands.  It is not a
    # hardware limit.
    joint_velocity_limit_rad_s: float = 16.0
    contact_drift_tolerance_m: float = 1e-3
    tracking_tolerance_m: float = 1e-3
    # Step 4 and Step 5 accept any pose inside the raw RobotParams limits, so a
    # world-fixed stance can legally sweep a leg to exactly beta = +-40 deg and
    # leave the following swing with no room at all.  Step 7 keeps this margin
    # in hand and backs the body advance off instead.
    joint_limit_margin_rad: float = 0.02
    # A straight crawl whose feet sit under the hips is only marginally stable:
    # lifting one leg leaves a triangle whose diagonal passes almost exactly
    # through the body centre.  The body therefore sways sideways during the
    # stance that precedes each swing, which is what every static crawl does.
    maximum_lateral_sway_m: float = 0.09
    required_stability_margin_m: float = 0.02
    lateral_sway_candidates: int = 33
    maximum_events: int = 400
    # Whole periods of the *existing* periodic flat Walk prepended ahead of the
    # crawl.  They end at a liftoff sample, which is the only kind of sample the
    # crawl can take over from; see ``handoff``.  Zero keeps the pure crawl.
    flat_approach_cycles: int = 0
    # Original LaunchController ramp cycles prepended before the requested
    # number of steady approach cycles.  These are velocity-scaled Walk cycles,
    # not a custom time warp and not a zero-joint-rate launch.
    flat_launch_cycles: int = 1
    flat_launch_ramp_floor: float = 0.1
    # Which launch the flat approach uses.  "timewarp" replays the Walk's own
    # joint path against a monotone clock that starts at rest, so it covers
    # exactly cycles * v_x * T and is continuous everywhere.  "legacy" uses the
    # repository's LaunchController, which splices whole cycles of different
    # stride and therefore steps the joints at every ramp-cycle boundary.
    flat_launch_mode: str = "timewarp"
    # Whole periods of the periodic flat Walk that replace the post-obstacle
    # crawl.  When this is non-zero ``post_distance_m`` no longer sets the
    # recovery distance -- these cycles do -- and the crawl only walks far
    # enough past the obstacle for the Walk's own footprint to be legal ground.
    flat_recovery_cycles: int = 0
    flat_recovery_launch_cycles: int = 1
    # Periods at the very end replayed under the closing time warp, so the file
    # finishes stationary the way the pure crawl always did.
    flat_landing_cycles: int = 1
    # Largest joint difference the recovery splice will close by hand.  It
    # exists because the crawl's exit pose and the Walk's own pose are solved
    # independently; anything larger means the crawl missed the footprint.
    recovery_snap_limit_rad: float = 0.02
    # During the final all-stance reconciliation, the two valid lowest-rim IK
    # branches can differ even though their final contact points coincide.
    # Bound the temporary contact motion explicitly instead of hiding it in a
    # one-row joint snap.
    recovery_contact_blend_limit_m: float = 0.006

    def __post_init__(self) -> None:
        positive = (
            "obstacle_length_m",
            "obstacle_height_m",
            "stand_height_m",
            "step_length_m",
            "period_s",
            "dt_s",
            "step_clearance_m",
            "flat_walk_step_height_m",
            "approach_distance_m",
            "joint_velocity_limit_rad_s",
            "contact_drift_tolerance_m",
            "tracking_tolerance_m",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")
        non_negative = (
            "joint_limit_margin_rad",
            "maximum_lateral_sway_m",
            "required_stability_margin_m",
            "edge_margin_m",
            "post_distance_m",
            "maximum_touchdown_bias_m",
            "ground_face_safety_m",
            "top_edge_safety_m",
            "recovery_snap_limit_rad",
            "recovery_contact_blend_limit_m",
        )
        for name in non_negative:
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative.")
        if not np.isfinite(self.flat_walk_dt_s) or self.flat_walk_dt_s <= 0.0:
            raise ValueError("flat_walk_dt_s must be finite and positive.")
        ratio = self.dt_s / self.flat_walk_dt_s
        if not np.isclose(ratio, round(ratio), rtol=0.0, atol=1e-9) or round(ratio) < 1:
            raise ValueError(
                "dt_s must be an integer multiple of flat_walk_dt_s so the crawl can be "
                "resampled onto the flat Walk's grid without moving its knots."
            )
        if self.crawl_swing_seconds is not None and (
            not np.isfinite(self.crawl_swing_seconds) or self.crawl_swing_seconds <= 0.0
        ):
            raise ValueError("crawl_swing_seconds must be finite and positive when given.")
        if self.stance_duty is not None and not 0.0 < self.stance_duty < 1.0:
            raise ValueError("stance_duty must lie in (0, 1) when given.")
        if not np.isfinite(self.obstacle_width_m) or self.obstacle_width_m <= 0.0:
            raise ValueError("obstacle_width_m must be finite and positive.")
        if self.flat_walk_velocity_m_s is not None and (
            not np.isfinite(self.flat_walk_velocity_m_s)
            or self.flat_walk_velocity_m_s <= 0.0
        ):
            raise ValueError("flat_walk_velocity_m_s must be finite and positive.")
        if not np.isfinite(self.obstacle_x_start_m):
            raise ValueError("obstacle_x_start_m must be finite.")
        if not 0.0 <= self.body_lift_ratio <= 1.0:
            raise ValueError("body_lift_ratio must lie in [0, 1].")
        fractions = tuple(float(value) for value in self.body_advance_fractions)
        if not fractions or any(not 0.0 <= value <= 1.0 for value in fractions):
            raise ValueError("body_advance_fractions must be non-empty values in [0, 1].")
        if list(fractions) != sorted(fractions, reverse=True):
            raise ValueError("body_advance_fractions must be ordered from most to least advance.")
        object.__setattr__(self, "body_advance_fractions", fractions)
        scales = tuple(float(value) for value in self.swing_duration_scales)
        if not scales or any(not np.isfinite(value) or value <= 0.0 for value in scales):
            raise ValueError("swing_duration_scales must be non-empty positive values.")
        object.__setattr__(self, "swing_duration_scales", scales)
        if not isinstance(self.maximum_events, int) or self.maximum_events <= 0:
            raise ValueError("maximum_events must be a positive integer.")
        if (
            not isinstance(self.lateral_sway_candidates, int)
            or self.lateral_sway_candidates < 1
        ):
            raise ValueError("lateral_sway_candidates must be a positive integer.")
        if (
            not isinstance(self.flat_approach_cycles, int)
            or isinstance(self.flat_approach_cycles, bool)
            or self.flat_approach_cycles < 0
        ):
            raise ValueError("flat_approach_cycles must be a non-negative integer.")
        if (
            not isinstance(self.flat_launch_cycles, int)
            or isinstance(self.flat_launch_cycles, bool)
            or self.flat_launch_cycles < 0
        ):
            raise ValueError("flat_launch_cycles must be a non-negative integer.")
        if self.flat_launch_mode not in ("timewarp", "legacy"):
            raise ValueError('flat_launch_mode must be "timewarp" or "legacy".')
        if (
            self.flat_launch_mode == "timewarp"
            and self.flat_approach_cycles
            and self.flat_launch_cycles > self.flat_approach_cycles
        ):
            raise ValueError(
                "with flat_launch_mode='timewarp', flat_launch_cycles must not exceed "
                "flat_approach_cycles; the warp has to reach nominal rate by the "
                "handover, not after it."
            )
        for name in (
            "flat_recovery_cycles",
            "flat_recovery_launch_cycles",
            "flat_landing_cycles",
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer.")
        if self.flat_recovery_cycles and (
            self.flat_recovery_launch_cycles + self.flat_landing_cycles
            > self.flat_recovery_cycles
        ):
            raise ValueError(
                "flat_recovery_launch_cycles + flat_landing_cycles must fit inside "
                "flat_recovery_cycles; the recovery Walk has to reach nominal rate "
                "before it starts slowing down again."
            )
        if not np.isfinite(self.flat_launch_ramp_floor) or not (
            0.0 < self.flat_launch_ramp_floor <= 1.0
        ):
            raise ValueError("flat_launch_ramp_floor must lie in (0, 1].")

    @property
    def beta_limit_rad(self) -> float:
        """Sagittal bound over which the foot rim can still be the contact.

        A Walk never rolls onto the upper tyre rims, so this is a hard
        feasibility bound for this gait, not a tunable guard.
        """

        return float(np.deg2rad(RobotParams.BETA_MAX_DEG))

    @property
    def forward_velocity_m_s(self) -> float:
        """Velocity of the periodic flat Walk sections.

        ``None`` preserves the historical coupled API for callers that only
        provide crawl stride and period.  The CLI now passes the flat velocity
        explicitly, so its known baseline is independent of obstacle stride.
        """

        if self.flat_walk_velocity_m_s is not None:
            return float(self.flat_walk_velocity_m_s)
        return self.step_length_m / (self.period_s * 0.75)


@dataclass(frozen=True)
class SegmentRecord:
    """Row range and per-segment evidence for one generated segment."""

    index: int
    kind: str
    stage: TraversalStage
    event_index: int | None
    leg: LegId | None
    start_row: int
    end_row: int
    sample_count: int
    body_pose_start_world: tuple[float, ...]
    body_pose_end_world: tuple[float, ...]
    body_advance_m: float
    touchdown_world_m: tuple[float, float, float] | None
    touchdown_surface_id: str | None
    touchdown_bias_x_m: float | None
    from_surface_id: str | None
    maximum_contact_drift_m: float | None
    maximum_tracking_error_m: float | None
    requested_apex_height_world_m: float | None
    achieved_apex_height_world_m: float | None
    rejected_candidates: tuple[str, ...] = ()


@dataclass(frozen=True)
class ObstacleWalkResult:
    """The assembled trajectory plus everything needed for a report."""

    segment: TrajectorySegment
    records: tuple[SegmentRecord, ...]
    boundary_reports: tuple[BoundaryContinuityReport, ...]
    request: ObstacleWalkRequest
    terrain: WalkTerrain1D
    forward_velocity_m_s: float
    swing_order: tuple[LegId, ...]
    wheel_face_exclusion_m: float
    wheel_outer_radius_m: float
    beta_limit_rad: float
    maximum_abs_beta_rad: float
    # Support-polygon margin during each swing, i.e. while only three legs
    # carry the robot.  This is the binding stability interval.
    swing_stability_margins_m: tuple[float, ...]
    legs_that_reached_top: tuple[LegId, ...]
    legs_that_returned_to_ground: tuple[LegId, ...]
    maximum_top_contact_count: int
    all_four_top_observed: bool
    traversal_completed: bool
    recovery_distance_m: float
    # Kept so Step 8 can rebuild the exact leg geometry the trajectory was
    # planned with, instead of constructing a second generator that might
    # differ.
    walk_generator: GaitGenerator3D | None = None
    body_pose_policy: str = BODY_POSE_POLICY
    stance_contact_model: str = STANCE_CONTACT_MODEL
    full_geometry_collision_checked: bool = False
    # Non-None only when a recovery Walk was spliced on: the joint and contact
    # residual that splice had to close during its final settling stance.  See
    # ``handoff.blend_final_row_to``.
    recovery_handover_joint_snap_rad: float | None = None
    recovery_handover_contact_snap_m: float | None = None
    stage_results: dict[str, bool] = field(default_factory=dict)
    # Step 6's nominal periodic-Walk prediction, kept for comparison only.  It
    # is not the plan that was executed: the crawl re-derives every touchdown
    # from the actual previous final state and from the stricter wheel-face
    # foothold rule, so its event times and footholds legitimately differ.
    reference_schedule: dict[str, object] | None = None
    reference_schedule_error: str | None = None

    @property
    def minimum_swing_stability_margin_m(self) -> float:
        if not self.swing_stability_margins_m:
            return 0.0
        return min(self.swing_stability_margins_m)

    @property
    def maximum_boundary_joint_position_error_rad(self) -> float:
        if not self.boundary_reports:
            return 0.0
        return max(report.joint_position_max_rad for report in self.boundary_reports)

    @property
    def maximum_boundary_joint_velocity_error_rad_s(self) -> float:
        if not self.boundary_reports:
            return 0.0
        return max(report.joint_velocity_max_rad_s for report in self.boundary_reports)

    @property
    def maximum_contact_drift_m(self) -> float:
        values = [
            record.maximum_contact_drift_m
            for record in self.records
            if record.maximum_contact_drift_m is not None
        ]
        return max(values) if values else 0.0

    @property
    def maximum_tracking_error_m(self) -> float:
        values = [
            record.maximum_tracking_error_m
            for record in self.records
            if record.maximum_tracking_error_m is not None
        ]
        return max(values) if values else 0.0


def wheel_face_exclusion_m(wheel_outer_radius_m: float, obstacle_height_m: float) -> float:
    """Ground stand-off that keeps the wheel disc clear of a vertical face.

    A disc of radius ``R`` whose lowest point rests on the ground spans
    ``sqrt(2 R h - h^2)`` horizontally at height ``h``.  A ground contact
    closer than that to a face of height ``H`` therefore puts rim material
    inside the obstacle.  Obstacles taller than the wheel use the full radius.
    """

    if not np.isfinite(wheel_outer_radius_m) or wheel_outer_radius_m <= 0.0:
        raise ValueError("wheel_outer_radius_m must be finite and positive.")
    if not np.isfinite(obstacle_height_m) or obstacle_height_m <= 0.0:
        raise ValueError("obstacle_height_m must be finite and positive.")
    height = min(float(obstacle_height_m), float(wheel_outer_radius_m))
    return math.sqrt(max(2.0 * wheel_outer_radius_m * height - height * height, 0.0))


def build_walk_generator(request: ObstacleWalkRequest) -> GaitGenerator3D:
    """Create the Walk generator whose leg order and kinematics Step 7 reuses."""

    import contextlib
    import io

    velocity = request.forward_velocity_m_s
    with contextlib.redirect_stdout(io.StringIO()):
        generator = GaitGenerator3D(
            stand_height=request.stand_height_m,
            twist=[0.0, velocity, 0.0],
            step_height=request.flat_walk_step_height_m,
            period=request.period_s,
            gait_type="Walk",
            dt=request.dt_s,
            stability_margin=0.0,
            stance_duty=request.stance_duty,
        )
    if not np.isclose(float(generator.v_com[0]), velocity, rtol=1e-6, atol=1e-9):
        raise ObstacleTraversalError(
            TraversalRejectReason.VELOCITY_GUARD_SCALED,
            f"the Walk workspace guard scaled v_x from {velocity:.6g} m/s to "
            f"{float(generator.v_com[0]):.6g} m/s; reduce step_length_m or raise period_s",
        )
    return generator


def build_flat_walk_generator(request: ObstacleWalkRequest) -> GaitGenerator3D:
    """The Walk generator for the spliced flat sections, at their own dt.

    Identical to :func:`build_walk_generator` except for the sample period, so
    the flat stretches carry the Walk's native samples instead of a coarse plan
    interpolated up by the exporter.
    """

    import dataclasses

    return build_walk_generator(
        dataclasses.replace(request, dt_s=request.flat_walk_dt_s)
    )


def walk_swing_order(generator: GaitGenerator3D) -> tuple[LegId, ...]:
    """Leg order taken from the existing Walk phase offsets, never hard-coded."""

    cycle_samples = int(round(generator.T / generator.dt))
    stance_samples = int(round(generator.stance_duty * cycle_samples))
    liftoff_index = [
        (stance_samples - int(offset * cycle_samples)) % cycle_samples
        for offset in generator.phase_offsets
    ]
    if len(set(liftoff_index)) != 4:
        raise ObstacleTraversalError(
            TraversalRejectReason.UNSUPPORTED_REQUEST,
            "the Walk phase offsets do not give four distinct liftoff events",
        )
    return tuple(LEG_ORDER[index] for index in np.argsort(liftoff_index))


def _neutral_leg_pose(
    generator: GaitGenerator3D,
    leg_index: int,
    stand_height_m: float,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Upright ``beta = gamma = 0`` pose whose lowest rim point is the contact.

    The Walk's own ``_level_touchdown_q`` is the *touchdown* extreme of a
    rolling stance, so it is not centred for a world-fixed stance sweep.  The
    upright pose is, which is why Step 7 measures its stride from here.
    """

    kinematics = generator.legs[leg_index]

    def lowest_z(theta: float) -> float:
        alpha, width = kinematics.foot_rim_contact_fk(theta, 0.0, 0.0)
        return float(
            kinematics.forward_kinematics(theta, 0.0, 0.0, alpha=alpha, w=width)[2]
        )

    lower, upper = np.deg2rad(18.0), np.deg2rad(159.5)
    if (lowest_z(lower) + stand_height_m) * (lowest_z(upper) + stand_height_m) > 0.0:
        raise ObstacleTraversalError(
            TraversalRejectReason.UNSUPPORTED_REQUEST,
            f"stand_height_m={stand_height_m:.4g} m is outside the upright leg range",
        )
    theta = brentq(lambda value: lowest_z(value) + stand_height_m, lower, upper)
    q = np.array([theta, 0.0, 0.0], dtype=float)
    alpha, width = kinematics.foot_rim_contact_fk(*q)
    point = np.asarray(
        kinematics.forward_kinematics(*q, alpha=alpha, w=width), dtype=float
    )
    return q, point


def _rotation_free_contact(body_pose: NDArray[np.float64], point_body: NDArray[np.float64]):
    """Level-body world contact; Step 7 never pitches or rolls the body."""

    return body_pose[:3] + point_body


def build_initial_state(
    generator: GaitGenerator3D,
    request: ObstacleWalkRequest,
    swing_order: Sequence[LegId],
    neutral_points_body_m: NDArray[np.float64],
    neutral_q_rad: Sequence[NDArray[np.float64]],
    body_x_m: float,
    ground_height_m: float,
    ground_surface_id: str,
    gait_cycle_phase: float,
) -> WalkState:
    """Synthesize the steady crawl posture that precedes the first swing.

    Leg ``swing_order[k]`` is placed so that after its own swing it lands one
    half stride ahead of the upright pose, and every later body advance moves
    it back by a quarter stride.  Starting from this posture the crawl is
    already periodic, so no settling cycles are needed.
    """

    stride = request.step_length_m
    advance = stride / 4.0
    body_pose = np.array(
        [float(body_x_m), 0.0, ground_height_m + request.stand_height_m, 0.0, 0.0, 0.0]
    )
    commands = np.empty((4, 3), dtype=float)
    contacts = np.empty((4, 3), dtype=float)
    for position, leg in enumerate(swing_order):
        leg_index = LEG_ORDER.index(leg)
        neutral = neutral_points_body_m[leg_index]
        target = np.array(
            [
                neutral[0] + stride / 2.0 - (3 - position) * advance,
                neutral[1],
                ground_height_m - body_pose[2],
            ]
        )
        q = np.asarray(neutral_q_rad[leg_index], dtype=float)
        actual = neutral
        # Continuation: walk the target across in small steps so the lowest-rim
        # iteration always starts from a nearby pose.
        for fraction in np.linspace(0.0, 1.0, 13)[1:]:
            step_target = neutral + fraction * (target - neutral)
            try:
                q, _alpha, _width, actual = _solve_lowest_rim_contact(
                    generator, leg_index, step_target, q, 1e-6
                )
            except StancePlanningError as exc:
                raise ObstacleTraversalError(
                    TraversalRejectReason.INITIAL_STATE_INFEASIBLE,
                    f"the initial crawl posture is unreachable: {exc}",
                    leg=leg,
                ) from exc
        commands[leg_index] = q
        contacts[leg_index] = _rotation_free_contact(body_pose, np.asarray(actual, float))
    return WalkState(
        joint_position_rad=commands,
        previous_joint_position_rad=None,
        body_pose_world=body_pose,
        foot_contact_points_world_m=contacts,
        phase=np.zeros(4, dtype=np.int8),
        contact_active=np.ones(4, dtype=bool),
        surface_ids=(ground_surface_id,) * 4,
        gait_cycle_phase=float(gait_cycle_phase),
        next_swing_leg=swing_order[0],
    )


def initial_gait_cycle_phase(generator: GaitGenerator3D, first_swing_leg: LegId) -> float:
    """All-stance cycle position immediately before ``first_swing_leg`` lifts.

    Only bookkeeping for the Step 6 reference comparison; the crawl itself
    never reads the periodic phase array.  A Walk with ``stance_duty = 0.75``
    tiles its four swings across the whole cycle, so whether an all-stance
    sample exists at all depends on how ``T / dt`` rounds.  When none exists
    the cycle origin is returned and the reference schedule simply reports why
    it could not consume the state.
    """

    cycle = _phase_cycle(generator)
    cycle_samples = len(cycle)
    all_stance = np.flatnonzero(~cycle.astype(bool).any(axis=1))
    if not len(all_stance):
        return 0.0
    leg_index = LEG_ORDER.index(first_swing_leg)
    for index in all_stance:
        if cycle[(int(index) + 1) % cycle_samples, leg_index] == 1:
            return int(index) / cycle_samples
    return int(all_stance[0]) / cycle_samples


def touchdown_candidates(
    terrain: WalkTerrain1D,
    exclusion_m: float,
    ground_safety_m: float,
    top_safety_m: float,
    nominal_x_m: float,
) -> tuple[float, ...]:
    """Legal touchdown x values nearest ``nominal_x_m``, closest first.

    Legality here is Step 3's surface query *plus* the wheel-face exclusion
    band on both ground sides of the obstacle.
    """

    obstacle = terrain.obstacle
    before = min(nominal_x_m, obstacle.x_start_m - exclusion_m - ground_safety_m)
    after = max(nominal_x_m, obstacle.x_end_m + exclusion_m + ground_safety_m)
    top = float(
        np.clip(
            nominal_x_m,
            obstacle.legal_top_x_min_m + top_safety_m,
            obstacle.legal_top_x_max_m - top_safety_m,
        )
    )
    unique: list[float] = []
    for value in (before, after, top):
        if not any(abs(value - kept) <= 1e-12 for kept in unique):
            unique.append(float(value))
    return tuple(sorted(unique, key=lambda value: abs(value - nominal_x_m)))


def _body_pose_target(
    contact_heights_m: NDArray[np.float64],
    body_x_m: float,
    body_y_m: float,
    request: ObstacleWalkRequest,
) -> NDArray[np.float64]:
    lowest = float(np.min(contact_heights_m))
    span = float(np.max(contact_heights_m) - lowest)
    height = lowest + request.stand_height_m + request.body_lift_ratio * span
    return np.array([float(body_x_m), float(body_y_m), height, 0.0, 0.0, 0.0])


def support_polygon_margin_m(
    body_xy_m: NDArray[np.float64], support_points_world_m: NDArray[np.float64]
) -> float:
    """Signed distance from the body-centre ground projection to the hull.

    Positive is inside.  The body origin stands in for the centre of mass, the
    same convention the existing Walk planner's ``stability_margin`` uses.
    """

    margin, _direction = _hull_signed_margin(
        np.asarray(body_xy_m, dtype=float)[:2],
        np.asarray(support_points_world_m, dtype=float)[:, :2],
    )
    return float(margin)


def choose_lateral_sway(
    body_x_m: float,
    reference_y_m: float,
    support_points_world_m: NDArray[np.float64],
    maximum_sway_m: float,
    candidate_count: int,
    required_margin_m: float,
) -> tuple[float, float]:
    """Return ``(body_y, margin)`` for the *smallest* sufficient sideways shift.

    Maximising the margin would swing the body as far as the limit allows,
    which pushes the stance legs far off their neutral lateral position and
    makes the world-fixed contact solve much harder.  The robot only needs
    enough sway to clear ``required_margin_m``, so candidates are tried in
    order of increasing displacement and the first sufficient one wins.  If
    none suffices the best available margin is returned so the caller can
    reject the candidate with a meaningful number.
    """

    def margin_at(candidate_y: float) -> float:
        return support_polygon_margin_m(
            np.array([body_x_m, candidate_y]), support_points_world_m
        )

    if maximum_sway_m <= 0.0:
        return reference_y_m, margin_at(reference_y_m)
    offsets = np.linspace(-maximum_sway_m, maximum_sway_m, max(3, candidate_count))
    order = np.argsort(np.abs(offsets), kind="stable")
    best_y, best_margin = reference_y_m, -np.inf
    for index in order:
        candidate_y = reference_y_m + float(offsets[index])
        margin = margin_at(candidate_y)
        if margin > best_margin:
            best_margin, best_y = margin, candidate_y
        if margin >= required_margin_m:
            return candidate_y, margin
    return best_y, best_margin


def generate_obstacle_walk(
    request: ObstacleWalkRequest,
    *,
    generator: GaitGenerator3D | None = None,
    full_geometry_collision_checker: FullGeometryCollisionChecker | None = None,
    tolerances: ContinuityTolerances | None = None,
) -> ObstacleWalkResult:
    """Generate and validate one complete offline obstacle-walk trajectory."""

    if not isinstance(request, ObstacleWalkRequest):
        raise TypeError("request must be an ObstacleWalkRequest.")
    generator = generator or build_walk_generator(request)
    if generator.gait_type != "Walk":
        raise ObstacleTraversalError(
            TraversalRejectReason.UNSUPPORTED_REQUEST,
            "Step 7 supports gait_type='Walk' only",
        )

    terrain = WalkTerrain1D(
        RectangleObstacle1D(
            x_start_m=request.obstacle_x_start_m,
            length_m=request.obstacle_length_m,
            height_m=request.obstacle_height_m,
            edge_margin_m=request.edge_margin_m,
        )
    )
    obstacle = terrain.obstacle
    wheel_radius = float(generator.legs[0].solver.foot_radius)
    exclusion = wheel_face_exclusion_m(wheel_radius, request.obstacle_height_m)
    minimum_stride = exclusion + request.edge_margin_m
    if request.step_length_m < minimum_stride:
        raise ObstacleTraversalError(
            TraversalRejectReason.STEP_TOO_SHORT_FOR_OBSTACLE,
            f"step_length_m={request.step_length_m:.4g} m cannot cross from the last legal "
            f"ground foothold to the first legal top foothold, which needs at least "
            f"{minimum_stride:.4g} m (wheel-face exclusion {exclusion:.4g} m + edge margin "
            f"{request.edge_margin_m:.4g} m)",
        )

    swing_order = walk_swing_order(generator)
    neutral_points = np.empty((4, 3), dtype=float)
    neutral_q: list[NDArray[np.float64]] = []
    for leg_index in range(4):
        q, point = _neutral_leg_pose(generator, leg_index, request.stand_height_m)
        neutral_q.append(q)
        neutral_points[leg_index] = point

    start_body_x = request.obstacle_x_start_m - request.approach_distance_m
    # approach_distance_m is the whole start-COM-to-obstacle gap.  The flat Walk
    # consumes its front portion and the crawl covers what is left, so the robot
    # starts the same distance from the obstacle however the two are split.
    launch_scales = np.linspace(
        request.flat_launch_ramp_floor, 1.0, request.flat_launch_cycles
    )
    # The time warp only changes the clock, so it covers exactly the cycles it
    # replays.  The LaunchController instead prepends whole extra cycles walked
    # at reduced speed, which is why it needs so much more room.
    flat_distance = float(generator.v_com[0]) * generator.T * (
        request.flat_approach_cycles
        if request.flat_launch_mode == "timewarp"
        else request.flat_approach_cycles + float(np.sum(launch_scales))
    )
    if flat_distance >= request.approach_distance_m:
        raise ObstacleTraversalError(
            TraversalRejectReason.FLAT_HANDOFF_INFEASIBLE,
            f"{request.flat_approach_cycles} flat Walk cycles cover {flat_distance:.4g} m, "
            f"which leaves no approach for the crawl inside approach_distance_m="
            f"{request.approach_distance_m:.4g} m",
        )
    handover_body_x = start_body_x + flat_distance
    flat_approach: TrajectorySegment | None = None
    if request.flat_approach_cycles > 0:
        # A second generator: generate_full_gait() caches CMDS/PHASE on the
        # instance, and the crawl keeps using `generator` purely as kinematics.
        try:
            if request.flat_launch_mode == "timewarp":
                flat_approach = flat_approach_segment(
                    build_flat_walk_generator(request),
                    cycles=request.flat_approach_cycles,
                    launch_cycles=request.flat_launch_cycles,
                    handover_body_x_m=handover_body_x,
                    body_y_m=0.0,
                    stand_height_m=request.stand_height_m,
                    ground_height_m=terrain.ground_height_m,
                    ground_surface_id=terrain.ground_surface_id,
                    expected_first_swing_leg=swing_order[0],
                )
            else:
                flat_approach = legacy_launch_flat_approach_segment(
                    build_flat_walk_generator(request),
                    steady_cycles=request.flat_approach_cycles,
                    launch_cycles=request.flat_launch_cycles,
                    ramp_floor=request.flat_launch_ramp_floor,
                    handover_body_x_m=handover_body_x,
                    body_y_m=0.0,
                    stand_height_m=request.stand_height_m,
                    ground_height_m=terrain.ground_height_m,
                    ground_surface_id=terrain.ground_surface_id,
                    expected_first_swing_leg=swing_order[0],
                )
        except FlatHandoffError as exc:
            raise ObstacleTraversalError(
                TraversalRejectReason.FLAT_HANDOFF_INFEASIBLE, str(exc)
            ) from exc
        height_error = handover_contact_height_error_m(
            flat_approach, terrain.ground_height_m
        )
        # generate_stance_segment applies its own contact_height_tolerance_m, whose
        # default the request cannot raise, so the stricter of the two decides.
        handover_height_tolerance = min(request.contact_drift_tolerance_m, 1e-3)
        if height_error > handover_height_tolerance:
            raise ObstacleTraversalError(
                TraversalRejectReason.FLAT_HANDOFF_INFEASIBLE,
                f"at the handover sample the flat Walk's own rim contacts sit "
                f"{height_error * 1e3:.3f} mm off the ground, beyond the "
                f"{handover_height_tolerance * 1e3:.3f} mm the crawl's stance accepts; "
                f"a finer dt makes the Walk's own IK land closer to the ground",
            )
        initial_state = flat_approach.final_state
    else:
        initial_state = build_initial_state(
            generator,
            request,
            swing_order,
            neutral_points,
            neutral_q,
            body_x_m=start_body_x,
            ground_height_m=terrain.ground_height_m,
            ground_surface_id=terrain.ground_surface_id,
            gait_cycle_phase=initial_gait_cycle_phase(generator, swing_order[0]),
        )
    state = initial_state

    stride = request.step_length_m
    advance = stride / 4.0
    joint_step_limit = request.joint_velocity_limit_rad_s * generator.dt
    # The crawl is quasi-static: it swings one leg while the body is at rest, so
    # its swing duration is a free parameter.  Deriving it from the Walk's duty
    # was accidental coupling -- raising the duty to 0.85 for the flat Walk's
    # stability would otherwise shorten every crawl swing by 40% and push the
    # per-sample joint step to 95% of its budget for no reason.
    swing_duration = (
        generator.T / 4.0
        if request.crawl_swing_seconds is None
        else request.crawl_swing_seconds
    )
    stance_duration = generator.T / 4.0

    segments: list[TrajectorySegment] = []
    records: list[SegmentRecord] = []
    # The crawl's first stance must not step away from the Walk's body motion,
    # so it starts at exactly the Walk's world velocity and brakes to rest.
    handover_velocity: NDArray[np.float64] | None = None
    if flat_approach is not None:
        records.append(_flat_record(len(records), flat_approach, segments, "flat_walk"))
        segments.append(flat_approach)
        handover_velocity = np.array(
            [float(generator.v_com[0]), float(generator.v_com[1]), 0.0]
        )
    body_x = float(state.body_pose_world[0])
    body_y_reference = float(state.body_pose_world[1])
    swing_stability_margins: list[float] = []
    reached_top: list[LegId] = []
    returned_to_ground: list[LegId] = []
    maximum_top_count = 0
    traversal_completed = False
    completion_body_x: float | None = None
    recovery_events = 0
    required_recovery_events = 0
    flat_recovery: TrajectorySegment | None = None
    resync_targets: NDArray[np.float64] | None = None
    resync_body_x: float | None = None
    resync_events_left = 0

    for event in range(request.maximum_events):
        leg = swing_order[event % 4]
        leg_index = LEG_ORDER.index(leg)
        from_surface = state.surface_ids[leg_index]
        chosen = None
        attempts: list[str] = []
        # During the resync the touchdown is not chosen at all: it is the
        # recovery Walk's own foot point for this leg, so the crawl arrives on
        # the Walk's footprint instead of its own.
        forced_touchdown = (
            None
            if resync_targets is None or resync_events_left <= 0
            else np.asarray(resync_targets[leg_index], dtype=float)
        )
        for fraction in request.body_advance_fractions:
            trial_body_x = body_x + fraction * advance
            nominal_x = trial_body_x + float(neutral_points[leg_index, 0]) + stride / 2.0
            candidates = (
                (float(forced_touchdown[0]),)
                if forced_touchdown is not None
                else touchdown_candidates(
                    terrain,
                    exclusion,
                    request.ground_face_safety_m,
                    request.top_edge_safety_m,
                    nominal_x,
                )
            )
            for candidate_x in candidates:
                bias = candidate_x - nominal_x
                # A forced touchdown answers to the Walk's geometry, not to the
                # crawl's stride, so the stride-relative bias bound does not
                # apply to it; the swing's own reach check still does.
                if forced_touchdown is None and abs(bias) > request.maximum_touchdown_bias_m:
                    continue
                query = query_touchdown_surface(terrain, candidate_x)
                if not query.is_legal:
                    attempts.append(
                        f"advance={fraction:.2f} x={candidate_x:.4f}: "
                        f"{query.rejection_reason.value}"
                    )
                    continue
                surface_height = float(query.surface_height_world_m)
                # The body pose must suit the support set that is held through
                # the stance *and* the surface the swing leg is about to reach,
                # so both the leg's current and target heights are included.
                heights = np.append(
                    state.foot_contact_points_world_m[:, 2], surface_height
                )
                # The three legs that stay down carry the robot through the
                # whole swing, during which the body is stationary, so the
                # stance that precedes it must already place the body inside
                # their triangle.
                support = np.delete(
                    state.foot_contact_points_world_m, leg_index, axis=0
                )
                swayed_y, margin = choose_lateral_sway(
                    trial_body_x,
                    body_y_reference,
                    support,
                    request.maximum_lateral_sway_m,
                    request.lateral_sway_candidates,
                    request.required_stability_margin_m,
                )
                if margin < request.required_stability_margin_m:
                    attempts.append(
                        f"advance={fraction:.2f} x={candidate_x:.4f} "
                        f"surface={query.surface_id} stability: best support-polygon "
                        f"margin {margin * 1e3:.1f} mm < "
                        f"{request.required_stability_margin_m * 1e3:.1f} mm"
                    )
                    continue
                target_pose = _body_pose_target(heights, trial_body_x, swayed_y, request)
                try:
                    stance = generate_stance_segment(
                        generator,
                        state,
                        target_pose,
                        terrain,
                        stance_duration,
                        contact_drift_tolerance_m=request.contact_drift_tolerance_m,
                        maximum_joint_step_rad=joint_step_limit,
                        initial_body_velocity_world_m_s=(
                            handover_velocity if event == 0 else None
                        ),
                        full_geometry_collision_checker=full_geometry_collision_checker,
                    )
                except StancePlanningError as exc:
                    attempts.append(
                        f"advance={fraction:.2f} x={candidate_x:.4f} "
                        f"surface={query.surface_id} stance: {exc}"
                    )
                    continue
                margin_violation = _joint_limit_margin_violation(
                    stance.segment.commands_rad,
                    request.joint_limit_margin_rad,
                    request.beta_limit_rad,
                )
                if margin_violation is not None:
                    attempts.append(
                        f"advance={fraction:.2f} x={candidate_x:.4f} "
                        f"surface={query.surface_id} stance margin: {margin_violation}"
                    )
                    continue
                swing = None
                # A taller step-up or step-down needs a longer swing to stay
                # under the per-sample joint step; the Bezier path itself is
                # unchanged, only its sampling.
                for scale in request.swing_duration_scales:
                    try:
                        swing = generate_swing_segment(
                            generator,
                            stance.segment.final_state,
                            leg,
                            (
                                forced_touchdown
                                if forced_touchdown is not None
                                else np.array(
                                    [
                                        candidate_x,
                                        # Body sway is temporary; keep the
                                        # touchdown on its fixed world track.
                                        body_y_reference
                                        + float(neutral_points[leg_index, 1]),
                                        surface_height,
                                    ]
                                )
                            ),
                            terrain,
                            request.step_clearance_m,
                            tracking_tolerance_m=request.tracking_tolerance_m,
                            maximum_joint_step_rad=joint_step_limit,
                            swing_duration_s=swing_duration * scale,
                            next_swing_leg=swing_order[(event + 1) % 4],
                            full_geometry_collision_checker=(
                                full_geometry_collision_checker
                            ),
                        )
                        margin_violation = _joint_limit_margin_violation(
                            swing.segment.commands_rad,
                            request.joint_limit_margin_rad,
                            request.beta_limit_rad,
                        )
                        if margin_violation is not None:
                            attempts.append(
                                f"advance={fraction:.2f} x={candidate_x:.4f} "
                                f"surface={query.surface_id} swing_scale={scale:g} "
                                f"margin: {margin_violation}"
                            )
                            swing = None
                            continue
                        break
                    except SwingPlanningError as exc:
                        attempts.append(
                            f"advance={fraction:.2f} x={candidate_x:.4f} "
                            f"surface={query.surface_id} swing_scale={scale:g}: {exc}"
                        )
                        swing = None
                if swing is None:
                    continue
                chosen = (
                    trial_body_x, candidate_x, bias, query, stance, swing, margin
                )
                break
            if chosen is not None:
                break

        if chosen is None:
            raise ObstacleTraversalError(
                TraversalRejectReason.NO_FEASIBLE_TOUCHDOWN,
                "no legal touchdown produced a feasible stance/swing pair",
                event_index=event,
                leg=leg,
                attempts=attempts,
            )

        trial_body_x, candidate_x, bias, query, stance, swing, margin = chosen
        stage = _classify_stage(
            from_surface,
            str(query.surface_id),
            terrain.ground_surface_id,
            traversal_completed,
            bool(reached_top),
        )
        records.append(
            _stance_record(
                len(records),
                event,
                leg,
                stage,
                stance,
                trial_body_x - body_x,
                segments,
            )
        )
        segments.append(stance.segment)
        records.append(
            _swing_record(
                len(records),
                event,
                leg,
                stage,
                swing,
                candidate_x,
                bias,
                from_surface,
                str(query.surface_id),
                tuple(attempts),
                segments,
            )
        )
        segments.append(swing.segment)

        body_x = trial_body_x
        swing_stability_margins.append(float(margin))
        state = swing.segment.final_state
        if str(query.surface_id) == obstacle.top_surface_id and leg not in reached_top:
            reached_top.append(leg)
        top_count = sum(
            item == obstacle.top_surface_id for item in state.surface_ids
        )
        maximum_top_count = max(maximum_top_count, top_count)
        if (
            reached_top
            and from_surface == obstacle.top_surface_id
            and str(query.surface_id) == terrain.ground_surface_id
            and leg not in returned_to_ground
        ):
            returned_to_ground.append(leg)

        if (
            not traversal_completed
            and reached_top
            and top_count == 0
            and float(np.min(state.foot_contact_points_world_m[:, 0])) > obstacle.x_end_m
        ):
            traversal_completed = True
            completion_body_x = body_x
            required_recovery_events = (
                0
                if request.post_distance_m == 0.0
                else 4 * int(math.ceil(request.post_distance_m / stride))
            )
            if request.flat_recovery_cycles > 0:
                # The recovery Walk's distance replaces post_distance_m, so the
                # crawl only has to reach the first body x at which the Walk's
                # whole footprint is legal ground.
                required_recovery_events = 0
                # The Walk's rearmost touchdown sits about half a stride behind
                # its neutral foot, and every ground contact must clear the
                # wheel-face exclusion band behind the obstacle.
                rear_offset = float(np.min(neutral_points[:, 0])) - stride / 2.0
                clearance_body_x = (
                    obstacle.x_end_m
                    + exclusion
                    + request.ground_face_safety_m
                    - rear_offset
                )
                resync_body_x = max(body_x + 4.0 * advance, clearance_body_x)

        if traversal_completed and request.flat_recovery_cycles > 0:
            if resync_targets is None:
                assert resync_body_x is not None
                if body_x + 4.0 * advance < resync_body_x:
                    continue  # still walking the crawl clear of the obstacle
                try:
                    flat_recovery = flat_recovery_segment(
                        build_flat_walk_generator(request),
                        cycles=request.flat_recovery_cycles,
                        launch_cycles=request.flat_recovery_launch_cycles,
                        landing_cycles=request.flat_landing_cycles,
                        start_body_x_m=resync_body_x,
                        body_y_m=body_y_reference,
                        stand_height_m=request.stand_height_m,
                        ground_height_m=terrain.ground_height_m,
                        ground_surface_id=terrain.ground_surface_id,
                        first_swing_leg=swing_order[(event + 1) % 4],
                    )
                except FlatHandoffError as exc:
                    raise ObstacleTraversalError(
                        TraversalRejectReason.FLAT_HANDOFF_INFEASIBLE,
                        f"the recovery Walk could not be placed: {exc}",
                        event_index=event,
                    ) from exc
                resync_targets = np.asarray(
                    flat_recovery.start_state.foot_contact_points_world_m, dtype=float
                )
                resync_events_left = 4
                continue
            resync_events_left -= 1
            if resync_events_left <= 0:
                break
            continue

        if traversal_completed:
            if recovery_events >= required_recovery_events:
                break
            recovery_events += 1
    else:
        raise ObstacleTraversalError(
            TraversalRejectReason.MAX_EVENTS_EXCEEDED,
            f"the traversal did not finish within {request.maximum_events} events",
            event_index=request.maximum_events - 1,
        )

    recovery_joint_snap_rad: float | None = None
    recovery_contact_snap_m: float | None = None
    if flat_recovery is not None:
        target = flat_recovery.start_state
        try:
            exit_stance = generate_stance_segment(
                generator,
                state,
                np.asarray(target.body_pose_world, dtype=float),
                terrain,
                stance_duration,
                contact_drift_tolerance_m=request.contact_drift_tolerance_m,
                maximum_joint_step_rad=joint_step_limit,
                full_geometry_collision_checker=full_geometry_collision_checker,
            )
        except StancePlanningError as exc:
            raise ObstacleTraversalError(
                TraversalRejectReason.FLAT_HANDOFF_INFEASIBLE,
                f"the crawl cannot settle onto the recovery Walk's pose: {exc}",
            ) from exc
        snapped, recovery_joint_snap_rad, recovery_contact_snap_m = blend_final_row_to(
            generator, exit_stance.segment, target
        )
        if recovery_joint_snap_rad > request.recovery_snap_limit_rad:
            raise ObstacleTraversalError(
                TraversalRejectReason.FLAT_HANDOFF_INFEASIBLE,
                f"the crawl's exit pose differs from the recovery Walk's by "
                f"{recovery_joint_snap_rad:.3e} rad, beyond the "
                f"{request.recovery_snap_limit_rad:.3e} rad this splice will close; "
                "the crawl did not actually land on the Walk's footprint",
            )
        if recovery_contact_snap_m > request.recovery_contact_blend_limit_m:
            raise ObstacleTraversalError(
                TraversalRejectReason.FLAT_HANDOFF_INFEASIBLE,
                f"the crawl-to-Walk settling blend moves a stance contact by "
                f"{recovery_contact_snap_m * 1e3:.3f} mm, beyond the explicit "
                f"{request.recovery_contact_blend_limit_m * 1e3:.3f} mm handoff budget",
            )
        records.append(
            _stance_record(
                len(records),
                None,
                None,
                TraversalStage.RECOVERY,
                exit_stance,
                float(target.body_pose_world[0]) - body_x,
                segments,
                kind="stance_recovery_handover",
            )
        )
        segments.append(snapped)
        records.append(
            _flat_record(
                len(records),
                flat_recovery,
                segments,
                "flat_walk_recovery",
                TraversalStage.RECOVERY,
            )
        )
        segments.append(flat_recovery)
        body_x = float(flat_recovery.body_pose_world[-1, 0])

    # The flat sections are already on the controller's grid; the crawl was
    # planned coarse and is brought up to it here, so assembly sees one clock
    # and the exporter has nothing left to resample.
    if len(records) != len(segments):
        raise AssertionError(
            f"{len(records)} records for {len(segments)} segments; the row ranges "
            "below assume one record per segment, in order"
        )
    segments = [resample_segment(item, request.flat_walk_dt_s) for item in segments]
    # Every record's row range was measured on the coarse plan, so re-derive it
    # from the segments that actually reach the CSV.
    records = [
        dataclasses.replace(
            record,
            start_row=(max(_row_offset(segments[:index]) - 1, 0) if index else 0),
            end_row=(max(_row_offset(segments[:index]) - 1, 0) if index else 0)
            + segments[index].sample_count
            - 1,
            sample_count=segments[index].sample_count,
        )
        for index, record in enumerate(records)
    ]

    combined = concatenate_segments(
        segments,
        dt=request.flat_walk_dt_s,
        tolerances=tolerances or ContinuityTolerances(),
    )
    reference_schedule: dict[str, object] | None = None
    reference_schedule_error: str | None = None
    try:
        reference_schedule = schedule_to_dict(
            schedule_obstacle_walk(
                generator,
                initial_state,
                terrain,
                post_distance_m=request.post_distance_m,
                maximum_touchdown_bias_m=request.maximum_touchdown_bias_m,
            )
        )
    except (ObstacleScheduleError, ValueError) as exc:
        all_stance_samples = int(
            np.count_nonzero(~_phase_cycle(generator).astype(bool).any(axis=1))
        )
        reference_schedule_error = (
            f"{exc} (the periodic Walk cycle at T={generator.T:g} s, "
            f"dt={generator.dt:g} s, stance_duty={generator.stance_duty:g} contains "
            f"{all_stance_samples} all-stance samples, so a four-leg-stance crawl "
            f"boundary need not exist in it)"
        )
    recovery_distance = 0.0 if completion_body_x is None else body_x - completion_body_x
    stage_results = {
        TraversalStage.APPROACH.value: True,
        TraversalStage.STEP_UP.value: bool(reached_top),
        TraversalStage.TOP_SUPPORT.value: maximum_top_count >= 2,
        TraversalStage.STEP_DOWN.value: bool(returned_to_ground),
        TraversalStage.RECOVERY.value: traversal_completed
        and recovery_events >= required_recovery_events,
    }
    return ObstacleWalkResult(
        segment=combined.segment,
        records=tuple(records),
        boundary_reports=combined.boundary_reports,
        request=request,
        terrain=terrain,
        forward_velocity_m_s=float(generator.v_com[0]),
        swing_order=swing_order,
        wheel_face_exclusion_m=exclusion,
        wheel_outer_radius_m=wheel_radius,
        walk_generator=generator,
        beta_limit_rad=request.beta_limit_rad,
        maximum_abs_beta_rad=float(np.max(np.abs(combined.segment.commands_rad[:, :, 1]))),
        swing_stability_margins_m=tuple(swing_stability_margins),
        legs_that_reached_top=tuple(reached_top),
        legs_that_returned_to_ground=tuple(returned_to_ground),
        maximum_top_contact_count=maximum_top_count,
        all_four_top_observed=maximum_top_count == 4,
        traversal_completed=traversal_completed,
        recovery_distance_m=recovery_distance,
        full_geometry_collision_checked=full_geometry_collision_checker is not None,
        recovery_handover_joint_snap_rad=recovery_joint_snap_rad,
        recovery_handover_contact_snap_m=recovery_contact_snap_m,
        stage_results=stage_results,
        reference_schedule=reference_schedule,
        reference_schedule_error=reference_schedule_error,
    )



def _joint_limit_margin_violation(
    commands_rad: NDArray[np.float64], margin_rad: float, beta_limit_rad: float
) -> str | None:
    """Report the first joint that leaves the limits minus ``margin_rad``."""

    if margin_rad <= 0.0:
        return None
    limits = (
        (
            0,
            "theta",
            np.deg2rad(RobotParams.MIN_THETA_DEG) + margin_rad,
            np.deg2rad(RobotParams.MAX_THETA_DEG) - margin_rad,
        ),
        (1, "beta", -beta_limit_rad + margin_rad, beta_limit_rad - margin_rad),
        (2, "gamma", -np.deg2rad(RobotParams.GAMMA_MAX_DEG) + margin_rad,
         np.deg2rad(RobotParams.GAMMA_MAX_DEG) - margin_rad),
    )
    for joint_index, name, lower, upper in limits:
        values = commands_rad[:, :, joint_index]
        outside = np.argwhere((values < lower) | (values > upper))
        if len(outside):
            sample, leg_index = (int(value) for value in outside[0])
            return (
                f"{LEG_ORDER[leg_index].value}.{name}="
                f"{values[sample, leg_index]:.6g} rad at sample {sample} leaves the "
                f"limits with {margin_rad:.6g} rad of margin"
            )
    return None


def _classify_stage(
    from_surface: str,
    target_surface: str,
    ground_surface: str,
    traversal_completed: bool,
    any_leg_on_top: bool,
) -> TraversalStage:
    if traversal_completed:
        return TraversalStage.RECOVERY
    if from_surface == ground_surface and target_surface != ground_surface:
        return TraversalStage.STEP_UP
    if from_surface != ground_surface and target_surface == ground_surface:
        return TraversalStage.STEP_DOWN
    if any_leg_on_top:
        return TraversalStage.TOP_SUPPORT
    return TraversalStage.APPROACH


def _row_offset(segments: Sequence[TrajectorySegment]) -> int:
    """First combined row index of the segment that would be appended next."""

    if not segments:
        return 0
    return sum(item.sample_count for item in segments) - (len(segments) - 1)


def _flat_record(
    record_index: int,
    segment: TrajectorySegment,
    segments: Sequence[TrajectorySegment],
    kind: str,
    stage: TraversalStage = TraversalStage.APPROACH,
) -> SegmentRecord:
    """Row range for a spliced periodic-Walk segment.

    It carries no event, leg or touchdown: the periodic Walk places all four
    feet on its own schedule, and this composer does not re-derive them.
    """

    start_row = max(_row_offset(segments) - 1, 0) if segments else 0
    count = segment.sample_count
    return SegmentRecord(
        index=record_index,
        kind=kind,
        stage=stage,
        event_index=None,
        leg=None,
        start_row=start_row,
        end_row=start_row + count - 1,
        sample_count=count,
        body_pose_start_world=tuple(float(v) for v in segment.body_pose_world[0]),
        body_pose_end_world=tuple(float(v) for v in segment.body_pose_world[-1]),
        body_advance_m=float(
            segment.body_pose_world[-1, 0] - segment.body_pose_world[0, 0]
        ),
        touchdown_world_m=None,
        touchdown_surface_id=None,
        touchdown_bias_x_m=None,
        from_surface_id=None,
        maximum_contact_drift_m=None,
        maximum_tracking_error_m=None,
        requested_apex_height_world_m=None,
        achieved_apex_height_world_m=None,
    )


def _stance_record(
    record_index: int,
    event: int | None,
    leg: LegId | None,
    stage: TraversalStage,
    stance,
    body_advance_m: float,
    segments: Sequence[TrajectorySegment],
    kind: str = "stance_advance",
) -> SegmentRecord:
    start_row = max(_row_offset(segments) - 1, 0) if segments else 0
    count = stance.segment.sample_count
    return SegmentRecord(
        index=record_index,
        kind=kind,
        stage=stage,
        event_index=event,
        leg=leg,
        start_row=start_row,
        end_row=start_row + count - 1,
        sample_count=count,
        body_pose_start_world=tuple(float(v) for v in stance.segment.body_pose_world[0]),
        body_pose_end_world=tuple(float(v) for v in stance.segment.body_pose_world[-1]),
        body_advance_m=float(body_advance_m),
        touchdown_world_m=None,
        touchdown_surface_id=None,
        touchdown_bias_x_m=None,
        from_surface_id=None,
        maximum_contact_drift_m=float(stance.maximum_contact_drift_m),
        maximum_tracking_error_m=None,
        requested_apex_height_world_m=None,
        achieved_apex_height_world_m=None,
    )


def _swing_record(
    record_index: int,
    event: int,
    leg: LegId,
    stage: TraversalStage,
    swing,
    touchdown_x: float,
    bias: float,
    from_surface: str,
    target_surface: str,
    attempts: tuple[str, ...],
    segments: Sequence[TrajectorySegment],
) -> SegmentRecord:
    start_row = max(_row_offset(segments) - 1, 0)
    count = swing.segment.sample_count
    return SegmentRecord(
        index=record_index,
        kind="swing",
        stage=stage,
        event_index=event,
        leg=leg,
        start_row=start_row,
        end_row=start_row + count - 1,
        sample_count=count,
        body_pose_start_world=tuple(float(v) for v in swing.segment.body_pose_world[0]),
        body_pose_end_world=tuple(float(v) for v in swing.segment.body_pose_world[-1]),
        body_advance_m=0.0,
        touchdown_world_m=tuple(float(v) for v in swing.touchdown_world_m),
        touchdown_surface_id=target_surface,
        touchdown_bias_x_m=float(bias),
        from_surface_id=from_surface,
        maximum_contact_drift_m=None,
        maximum_tracking_error_m=float(swing.maximum_tracking_error_m),
        requested_apex_height_world_m=float(swing.requested_apex_height_world_m),
        achieved_apex_height_world_m=float(swing.achieved_apex_height_world_m),
        rejected_candidates=attempts,
    )
