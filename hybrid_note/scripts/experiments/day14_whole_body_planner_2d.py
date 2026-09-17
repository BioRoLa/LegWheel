"""Day 14: the gait-first whole-body planner.

One sentence: **the body's clock comes first; every leg is generated against
where the body carries its hip; liftoffs are placed one at a time, in the
gait's order, and never overlap.**

This is the rewrite Day 13 log 33.5 concluded was needed.  The Day 12/13
pipeline planned each leg on its own and inferred time from position
afterwards, which made a leg's time identical to its claim about the body and
put the two legs of a pair -- same ``mount_x`` -- in the air together.  Here
nothing is inferred afterwards: :class:`GaitClock2D` maps body x to time both
ways, a stance stroke is asked for exactly the distance the body will carry
the hip before the leg's turn, and a swing is placed as an interval of body
travel that must not overlap the previous one.

What is reused
--------------

The generators are Day 12's, unchanged: ``run_foot_rim_roll_2d`` for a stance
stroke (it has always taken ``max_distance_m``), ``run_recovery_swing_2d`` for
every airborne motion (it has always taken an explicit landing), and the
segment builders ``roll_segment_2d`` / ``swing_segment_2d``.  The output is the
Day 12 contract -- ``LegPlan2D`` per leg, one ``FourLegSchedule2D``, and then
``body_trajectory_2d`` / ``swing_stability_2d`` / ``assemble_whole_body_2d`` /
``validate_whole_body_2d`` exactly as ``plan_terrain_2d`` runs them -- so the
exporter, the metrics and the validator do not know which planner produced
the trajectory.

What decides a leg's next move
------------------------------

A :class:`LegRule2D`.  The flat rule (:class:`FlatRule2D`) is the nominal
cycle: a full stroke, then a nominal recovery, ``cycles`` times, and it is what
Step 1 measures against the frozen Day 12 flat numbers.  The terrain rule
(Step 3, a separate module) answers from where the leg actually is relative to
the obstacle.  The loop below does not know the difference.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from typing import Protocol

import numpy as np

from hybrid_note.scripts.experiments.day10_11_composer_2d import ComposedSequence2D
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import StrategyId
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    FrameRef2D,
    MotionSegment2D,
    SegmentKind,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    HIP_TO_BODY_Z_M,
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    LegId,
    leg_mounts_2d,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    CycleFrame2D,
    NominalCycle2D,
    NominalPosture2D,
    RecoveryConfig2D,
    RecoverySwing2D,
    RollStroke2D,
    roll_segment_2d,
    run_foot_rim_roll_2d,
    run_nominal_cycles_2d,
    run_recovery_swing_2d,
    swing_segment_2d,
    translate_cycle_2d,
)
from hybrid_note.scripts.experiments.day12_segment_contract_2d import (
    ChainTolerance2D,
    SegmentChain2D,
    chain_boundaries_2d,
)
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (
    HYBRID_MARGIN_FLOOR_M,
    hybrid_body_z_m,
    hybrid_posture_2d,
    hybrid_timing_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    GAMMA_RAD,
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_terrain_generalization_2d import (
    Stage,
    TerrainFailure2D,
    TerrainRun2D,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    FourLegSchedule2D,
    GaitTiming2D,
    LegMode,
    ScheduledSegment2D,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    _hold_segment_2d,
    FourLegPlan2D,
    LegPlan2D,
    PhasedSegment2D,
    TransitionPhase,
    _cached_nominal_cycles,
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    assemble_whole_body_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (
    validate_whole_body_2d,
)
from hybrid_note.scripts.experiments.day12_world_registration_2d import (
    SpeedZone2D,
    body_speed_m_s,
    swing_hip_advance_m,
)
from hybrid_note.scripts.experiments.day14_gait_clock_2d import (
    GaitClock2D,
    SwingEvent2D,
    minimum_swing_duration_s,
)
from hybrid_note.scripts.experiments.day14_nominal_transitions_2d import (
    transition_segment_2d,
)

__all__ = [
    "StanceAction2D",
    "SwingAction2D",
    "LegProgress2D",
    "LegRule2D",
    "FlatRule2D",
    "PlannerRefusal2D",
    "GaitFirstPlan2D",
    "plan_gait_first_2d",
    "truncate_stroke_2d",
    "plan_flat_gait_first_2d",
    "plan_terrain_gait_first_2d",
    "run_through_day12_2d",
    "refit_body_plane_2d",
    "leg_hip_z_at",
    "hybrid_clock_2d",
]


# --------------------------------------------------------------------------
# What a rule may ask a leg to do next
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class StanceAction2D:
    """Roll on the current rim for at most ``max_distance_m`` of contact.

    ``None`` means the whole usable arc, which is the nominal stroke.  A rule
    that already holds the stroke (the flat rule, whose strokes are one cached
    cycle translated) hands it over in ``stroke`` and the loop places it rather
    than regenerating it -- bit-identical to Day 12, and 30 s cheaper per leg.
    """

    max_distance_m: float | None = None
    kind: SegmentKind = SegmentKind.FOOT_RIM_ROLL
    phase_label: str = "NOMINAL_FOOT_RIM_ROLL"
    phase: TransitionPhase = TransitionPhase.NOMINAL_BEFORE
    stroke: RollStroke2D | None = None


@dataclass(frozen=True)
class SwingAction2D:
    """Leave the ground from the stroke's end and land where this says.

    Every field with a ``None`` default falls back to the nominal recovery's own
    answer inside ``run_recovery_swing_2d`` -- one turn on from the arc start,
    the posture's stance theta, the held hip height.  A terrain rule overrides
    them to land on an obstacle top, on the ground past a trailing edge, or on
    another rim.
    """

    kind: SegmentKind = SegmentKind.RECOVERY_SWING
    phase_label: str = "NOMINAL_RECOVERY_SWING"
    phase: TransitionPhase = TransitionPhase.NOMINAL_BEFORE
    beta_target_rad: float | None = None
    theta_touchdown_rad: float | None = None
    hip_z_touchdown_m: float | None = None
    #: How far the hip travels while airborne.  ``None`` keeps the config's,
    #: which for the nominal gait is what closes the cycle
    #: (``swing_hip_advance_m``).
    hip_advance_m: float | None = None
    lift_hip_before_rotation: bool = False
    swing: RecoverySwing2D | None = None


@dataclass
class LegProgress2D:
    """Where one leg is in its own plan.  Mutable; the loop owns it."""

    leg: LegId
    mount_x_m: float
    #: The stroke the leg is standing at the end of, awaiting its liftoff.
    stroke: RollStroke2D | None = None
    stroke_action: StanceAction2D | None = None
    strokes_done: int = 0
    swings_done: int = 0
    finished: bool = False
    #: How much arc the loop cut off this leg's pending stroke to let it lift
    #: before another leg's long swing, in metres of hip travel.
    cut_hip_m: float = 0.0
    #: Set when the loop cut this leg's stroke: its next swing must land at or
    #: before this body x, because another leg lifts off there.
    landing_cap_body_x_m: float | None = None
    #: The swing already generated for the pending stroke, so that a loop
    #: iteration that only cut *another* leg does not ask the rule again.
    pending_swing: tuple | None = None

    @property
    def stroke_start_body_x_m(self) -> float | None:
        if self.stroke is None:
            return None
        return float(self.stroke.start.hip_xz_m[0]) - self.mount_x_m

    @property
    def liftoff_body_x_m(self) -> float | None:
        if self.stroke is None:
            return None
        return float(self.stroke.end.hip_xz_m[0]) - self.mount_x_m


class LegRule2D(Protocol):
    """What the loop asks; the rule does not see the other legs' state.

    It does not need to: the loop serialises the swings.  What the rule
    decides is *this* leg's next stroke length and its next landing, from the
    terrain under it and where its contact is.
    """

    def first_stroke(self, progress: LegProgress2D, *,
                     hip_x_m: float) -> StanceAction2D: ...

    def swing_after(self, progress: LegProgress2D, *,
                    landing_cap_body_x_m: float | None = None,
                    next_liftoffs_body_x_m: Sequence[float] = ()) -> SwingAction2D: ...

    def stroke_after(self, progress: LegProgress2D,
                     landing: CycleFrame2D) -> StanceAction2D | None: ...


# --------------------------------------------------------------------------
# The flat rule: the nominal cycle, ``cycles`` times
# --------------------------------------------------------------------------


@dataclass
class FlatRule2D:
    """Full stroke, nominal recovery, repeat.  The Day 12 flat gait.

    One nominal run is generated at the origin (cached by
    ``run_nominal_cycles_2d``'s own translation rule) and each leg gets it
    translated to its own hip x.  Translation invariance of the flat cycle was
    measured to 0.0000 um in Day 12 Step 1, so this is the same plan the frozen
    numbers were measured on, placed in the world.
    """

    cycles: int
    posture: NominalPosture2D
    config: RecoveryConfig2D
    _template: tuple[NominalCycle2D, ...] = field(default_factory=tuple)
    _per_leg: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.cycles < 1:
            raise ValueError("ask for at least one cycle.")
        # Step 4's own cache, keyed on the configuration: the flat run is the
        # most expensive thing here (one full stroke generation, ~40 s) and it
        # is the same run every time.
        self._template = tuple(_cached_nominal_cycles(
            self.cycles, self.posture, self.config))
        if any(not c.success for c in self._template):
            bad = next(c for c in self._template if not c.success)
            raise ValueError(
                "the nominal cycle does not complete: "
                f"stroke={bad.stroke.stop_reason}, "
                f"recovery={bad.recovery.failure_reason}")

    def _cycles_for(self, progress: LegProgress2D, hip_x_m: float):
        if progress.leg not in self._per_leg:
            dx = float(hip_x_m) - float(self._template[0].stroke.start.hip_xz_m[0])
            self._per_leg[progress.leg] = tuple(
                translate_cycle_2d(c, dx, 0.0) for c in self._template)
        return self._per_leg[progress.leg]

    def first_stroke(self, progress: LegProgress2D, *,
                     hip_x_m: float) -> StanceAction2D:
        cycle = self._cycles_for(progress, hip_x_m)[0]
        return StanceAction2D(stroke=cycle.stroke)

    def swing_after(self, progress: LegProgress2D, *,
                    landing_cap_body_x_m: float | None = None,
                    next_liftoffs_body_x_m: Sequence[float] = ()) -> SwingAction2D:
        cycle = self._per_leg[progress.leg][progress.swings_done]
        return SwingAction2D(swing=cycle.recovery)

    def stroke_after(self, progress: LegProgress2D,
                     landing: CycleFrame2D) -> StanceAction2D | None:
        if progress.swings_done >= self.cycles:
            return None
        cycle = self._per_leg[progress.leg][progress.swings_done]
        return StanceAction2D(stroke=cycle.stroke)


# --------------------------------------------------------------------------
# The plan
# --------------------------------------------------------------------------


class PlannerRefusal2D(RuntimeError):
    """The loop could not place a motion.  The message says which and why."""


def _trace_enabled() -> bool:
    """``DAY14_TRACE=1`` prints every placement decision of the loop."""

    import os
    return bool(os.environ.get("DAY14_TRACE"))


#: Two swings whose body-x windows overlap by less than this are not two legs
#: in the air: a swing flown with the body standing still is given 0.1 mm of
#: hip travel so the clock can hold time for it, and that is the whole overlap.
SERIAL_TOLERANCE_M: float = 1e-3

#: The share of the motor budget a swing may plan for, per frame step.
#: Measured on the frozen flat export: the nominal 0.36 s swing has per-frame
#: steps worth 72% of the budget, yet the 1 kHz command peaks at 95.0%,
#: because a 200 Hz sample that straddles a phase corner (retract -> rotate)
#: carries both joints' steps.  That 1.32x spike scales with the base rate,
#: so a swing planned at 95% per frame exported at 117-120% (measured on the
#: 40 mm crossing).  0.74 keeps the export peak under 98% and leaves the
#: nominal 0.36 s swing untouched (its minimum becomes 0.350 s).
SWING_UTILISATION: float = 0.74

#: A swing flown with the body standing still is given this much hip travel,
#: so the body's clock can hold time for it.
PAUSE_ADVANCE_M: float = 1e-4


@dataclass
class _LegBuild:
    leg: LegId
    mount_x_m: float
    source_id: str
    segments: list = field(default_factory=list)
    phases: list = field(default_factory=list)
    labels: list = field(default_factory=list)
    frames: list = field(default_factory=list)
    #: How each segment's times are read off the final clock: ``None`` for a
    #: stroke (from its hip x), ``("dwell", order)`` for a swing flown with the
    #: body standing still, ``("travel", x_lift, x_land)`` for the rest.
    timings: list = field(default_factory=list)
    #: Strokes and swings in placement order, written once the clock is final.
    pending: list = field(default_factory=list)

    def add_roll(self, stroke: RollStroke2D, action: StanceAction2D,
                 cut_at_hip_x_m: Sequence[float] = (), frame_at=None) -> None:
        # A stroke of one frame is a leg that landed and has nowhere to roll
        # before its next liftoff -- a standing pose, which the following swing
        # starts from.  The schema (rightly) refuses a rolling segment with a
        # zero beta step, so it is not written as one.
        if len(stroke.frames) < 2:
            return
        # Frames are uniform in hip x, and a consumer spreads a segment's
        # frames uniformly in *time*.  Those agree only while the body speed
        # is constant across the segment, so a stroke that spans a speed-zone
        # boundary is written as several segments cut at the boundary --
        # ``CUT`` boundaries of one run, which the chain contract allows.
        frames = list(stroke.frames)
        if frame_at is not None:
            # An exact frame at each boundary (unless one is already within
            # 50 um of it), so the cut is where the clock's speed changes.
            for x in sorted(set(float(v) for v in cut_at_hip_x_m)):
                hips = [float(f.hip_xz_m[0]) for f in frames]
                if not (hips[0] < x < hips[-1]) or min(abs(h - x) for h in hips) < 5e-5:
                    continue
                j = next(k for k, h in enumerate(hips) if h > x)
                try:
                    frames.insert(j, frame_at(stroke, x))
                except PlannerRefusal2D:
                    # A climb's poses cannot be interpolated; the cut falls
                    # on the nearest of its own frames (<= 2 mm off).
                    continue
        hips = [float(f.hip_xz_m[0]) for f in frames]
        stroke = replace(stroke, frames=tuple(frames))
        cut_indices = {
            int(np.argmin(np.abs(np.asarray(hips) - float(x))))
            for x in cut_at_hip_x_m if hips[0] < float(x) < hips[-1]}
        # A frame made between two of the roll's own (``index == -1``) breaks
        # the uniform beta step a segment's frames are read with, so it and
        # its neighbours bound two-frame segments of their own.
        for i, f in enumerate(frames):
            if f.index == -1:
                cut_indices.update({i - 1, i, i + 1})
        # A climb is several stages with different step sizes (the approach
        # in 5 mm of hip, the roll-up in 2 mm, the top in 0.5 deg of beta);
        # each stage is its own segment so its frames play at their own
        # uniform pace.  Measured: one segment for all of it put the
        # climbing leg 22.6 mm away from where the body's clock had it.
        for i in range(1, len(frames)):
            if frames[i].phase != frames[i - 1].phase:
                cut_indices.add(i - 1)
        # A stroke levelled on a rising axle takes uneven hip steps (9-15 mm
        # in one stroke, measured at 40 mm), and a segment's frames are
        # played uniformly in time: written as one segment, the leg's hip
        # was 6.4 mm from the body's.  Such a stroke is written frame by
        # frame, each two-frame segment timed off the clock exactly.
        steps = [b - a for a, b in zip(hips, hips[1:]) if b - a > 1e-9]
        if steps and max(steps) > 1.02 * min(steps):
            cut_indices.update(range(1, len(hips) - 1))
        bounds = [0] + sorted(i for i in cut_indices if 0 < i < len(hips) - 1) + [len(hips) - 1]
        for a, b in zip(bounds, bounds[1:]):
            if b <= a:
                continue
            piece = replace(stroke, frames=tuple(stroke.frames[a:b + 1]))
            self.segments.append(roll_segment_2d(
                piece, source_id=self.source_id, frame_offset=len(self.frames),
                kind=action.kind, phase_label=action.phase_label))
            self.phases.append(action.phase)
            self.labels.append("gait_first")
            self.timings.append(None)
            self.frames.extend(piece.frames)

    def add_swing(self, swing: RecoverySwing2D, action: SwingAction2D,
                  timing: tuple) -> None:
        if timing and timing[0] == "phased":
            # The fold, the rotation and the extension are their own
            # segments, so each can be timed as a dwell or as travel.  The
            # shared boundary frames are copied, as with cut strokes.
            for part in timing[1:]:
                lo, hi = part[-1]
                piece = replace(swing, frames=tuple(swing.frames[lo:hi + 1]))
                self._add_swing_piece(piece, action, part[:-1])
            return
        self._add_swing_piece(swing, action, timing)

    def _add_swing_piece(self, swing: RecoverySwing2D, action: SwingAction2D,
                         timing: tuple) -> None:
        # A nominal recovery is written as the RECOVERY_SWING the schema knows
        # (RollSampling2D + RecoveryShaping2D).  A terrain transition flown the
        # same way is written as the terrain kind it *is* -- SWING_UP and the
        # rest carry Day 8--9's SwingSampling2D, so the metrics count nominal
        # and terrain-forced swings apart (Day 12 plan section 18).
        if action.kind is SegmentKind.RECOVERY_SWING:
            segment = swing_segment_2d(
                swing, source_id=self.source_id, frame_offset=len(self.frames),
                kind=action.kind, phase_label=action.phase_label)
        else:
            segment = transition_segment_2d(
                swing, kind=action.kind, source_id=self.source_id,
                frame_offset=len(self.frames), phase_label=action.phase_label)
        self.segments.append(segment)
        self.phases.append(action.phase)
        self.labels.append("gait_first")
        self.timings.append(tuple(timing))
        self.frames.extend(swing.frames)


@dataclass(frozen=True)
class GaitFirstPlan2D:
    """The four leg plans, the schedule they were placed on, and the events."""

    plans: dict
    schedule: FourLegSchedule2D
    clock: GaitClock2D
    swings: tuple[SwingEvent2D, ...]
    #: Every time the loop had to slow the body for a swing, as a record.
    slowdowns: tuple[dict, ...]
    #: Every stroke the loop cut short so its leg could lift before another
    #: leg's long swing.
    cuts: tuple[dict, ...] = ()

    @property
    def max_airborne_count(self) -> int:
        return self.schedule.max_airborne_count

    def as_dict(self) -> dict:
        return {
            "legs": len(self.plans),
            "segments": len(self.schedule.scheduled),
            "swings": len(self.swings),
            "slowdowns": len(self.slowdowns),
            "cuts": len(self.cuts),
            "max_airborne_count": self.max_airborne_count,
            "start_s": self.schedule.start_s,
            "end_s": self.schedule.end_s,
            **{f"clock_{k}": v for k, v in self.clock.as_dict().items()},
        }


def hybrid_clock_2d(
    timing: GaitTiming2D | None = None,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    *,
    origin_x_m: float = 0.0,
    reference_cycle: NominalCycle2D | None = None,
) -> tuple[GaitClock2D, NominalPosture2D, RecoveryConfig2D]:
    """The Hybrid gait's clock, posture and recovery config, consistently.

    ``config.hip_advance_m`` is the value that closes the cycle at this duty
    (``swing_hip_advance_m``), and the body speed is one cycle over one period.
    Both are derived, so a caller cannot hand the planner a speed the gait does
    not actually produce.
    """

    timing = hybrid_timing_2d() if timing is None else timing
    posture = hybrid_posture_2d() if posture is None else posture
    if config is None:
        config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))
    if reference_cycle is None:
        reference_cycle = _cached_nominal_cycles(1, posture, config)[0]
    if not reference_cycle.success:
        raise ValueError("the nominal cycle does not complete; no speed to take.")
    # ``body_speed_m_s``'s own definition -- one cycle's hip advance over one
    # period -- read off a cycle already generated rather than generating one.
    advance = float(reference_cycle.recovery.end.hip_xz_m[0]
                    - reference_cycle.stroke.start.hip_xz_m[0])
    clock = GaitClock2D(timing=timing,
                        speed_m_s=advance / float(timing.cycle_period_s),
                        origin_x_m=float(origin_x_m))
    return clock, posture, config


def plan_gait_first_2d(
    rule: LegRule2D,
    clock: GaitClock2D,
    posture: NominalPosture2D,
    config: RecoveryConfig2D,
    *,
    tolerance: ChainTolerance2D | None = None,
    swing_utilisation: float = SWING_UTILISATION,
    long_swing_speed_scale: float = 1.0,
    crossing_speed_scale: float = 1.0,
    crossing_margin_m: float = 0.10,
    phase_dwell_min_s: float = 0.0,
) -> GaitFirstPlan2D:
    """Place every leg's strokes and swings on the body's clock, in order.

    The loop is in **body x**, not time: a swing is an interval of body travel,
    two swings may not overlap, and time is read off the clock only when the
    schedule is written out.  Slowing the body for a swing that needs more
    time than its hip travel allows is a zone on the clock, applied to every
    leg alike.
    """

    mounts = {m.leg: float(m.offset_body_xyz_m[0]) for m in leg_mounts_2d(GAMMA_RAD)}
    builds = {leg: _LegBuild(leg, mounts[leg], f"day14_gait_first:{leg.value}")
              for leg in LEG_ORDER}
    progress = {leg: LegProgress2D(leg, mounts[leg]) for leg in LEG_ORDER}
    order = clock.liftoff_order
    rank = {leg: i for i, leg in enumerate(order)}

    # Every leg begins a full stroke where its phase puts it.  Strokes are
    # written out only when the leg actually lifts off, because until then the
    # loop may still shorten one (see below).
    for leg in LEG_ORDER:
        t0 = clock.chain_start_time_s(leg)
        hip_x0 = clock.body_x_at_time(t0) + mounts[leg]
        action = rule.first_stroke(progress[leg], hip_x_m=hip_x0)
        stroke = _stance_stroke(action, posture, hip_x_m=hip_x0,
                                start_beta_rad=None)
        if not stroke.success:
            raise PlannerRefusal2D(
                f"{leg.value}: first stroke refused: {stroke.stop_reason}")
        progress[leg].stroke = stroke
        progress[leg].stroke_action = action
        progress[leg].strokes_done = 1

    swings: list[SwingEvent2D] = []
    slowdowns: list[dict] = []
    cuts: list[dict] = []
    last_landing_body_x: float | None = None
    nominal_advance = float(config.hip_advance_m)
    guard = 0
    trace = _trace_enabled()
    frame_at = getattr(rule, "frame_at_hip_x", None)
    latest_pause = getattr(rule, "latest_pause_hip_x_m", None)
    is_uncuttable = getattr(rule, "stroke_is_uncuttable", None)
    paused_at: dict = {}

    while True:
        guard += 1
        if guard > 10000:
            raise PlannerRefusal2D("the placement loop did not terminate.")
        # A leg that was cut for another goes first among legs due at the
        # same body x: its swing was made to land by the other's takeoff, and
        # if the other went first instead it would find it in its window
        # again, cut it by nothing, and never advance (measured: the loop
        # spun on exactly that tie).
        # Liftoffs are rounded to a micron: a leg cut to another's takeoff
        # gets its liftoff back through mount offsets (x + mount - mount),
        # and a last-bit difference put it *after* that leg in this order
        # (measured at 100 mm, axle rise 50: RH re-placed its recovery over
        # the two fronts it had just cut, and refused).
        due = [(round(p.liftoff_body_x_m, 6), 0 if p.landing_cap_body_x_m is not None else 1,
                rank[p.leg], p.leg)
               for p in progress.values() if not p.finished and p.stroke is not None]
        if not due:
            break
        due.sort()
        leg = due[0][3]
        prog = progress[leg]
        x_lift = float(prog.liftoff_body_x_m)   # the rounding is for the order only
        build = builds[leg]

        # At most one leg airborne, by construction: this swing may not begin
        # before the previous one has landed.  Nothing here can move this
        # liftoff later -- the leg is at the end of the stroke its rule gave
        # it -- so this is a refusal, with the numbers.
        if (last_landing_body_x is not None
                and x_lift < last_landing_body_x - SERIAL_TOLERANCE_M):
            raise PlannerRefusal2D(
                f"{leg.value} reaches its liftoff at body x {x_lift * 1e3:.3f} mm "
                f"while the previous swing lands at {last_landing_body_x * 1e3:.3f} mm; "
                "two legs would be airborne, and this leg's stroke has already "
                "been cut as far as it can be.")

        cached = prog.pending_swing
        if (cached is not None and cached[0] is prog.stroke
                and cached[1] == prog.landing_cap_body_x_m):
            action, swing = cached[2], cached[3]
        else:
            action = rule.swing_after(
                prog, landing_cap_body_x_m=prog.landing_cap_body_x_m,
                next_liftoffs_body_x_m=sorted(
                    p.liftoff_body_x_m for p in progress.values()
                    if p.leg is not leg and not p.finished and p.stroke is not None
                    and p.liftoff_body_x_m > x_lift + 1e-9))
            swing = _airborne_swing(action, prog.stroke, config)
            if not swing.success:
                raise PlannerRefusal2D(
                    f"{leg.value}: swing {action.kind.value} refused: "
                    f"{swing.failure_reason}")
            prog.pending_swing = (prog.stroke, prog.landing_cap_body_x_m, action, swing)
        x_land = float(swing.end.hip_xz_m[0]) - mounts[leg]
        if trace:
            others = ", ".join(
                f"{p.leg.value}@{p.liftoff_body_x_m * 1e3:.1f}"
                f"{'(cap ' + format(p.landing_cap_body_x_m * 1e3, '.1f') + ')' if p.landing_cap_body_x_m is not None else ''}"
                for p in progress.values()
                if p.leg is not leg and not p.finished and p.stroke is not None)
            print(f"    trace[{guard}] {leg.value} {action.kind.value} lift {x_lift * 1e3:.1f} "
                  f"land {x_land * 1e3:.1f} (stroke from {prog.stroke_start_body_x_m * 1e3:.1f}, "
                  f"cap {'-' if prog.landing_cap_body_x_m is None else format(prog.landing_cap_body_x_m * 1e3, '.1f')}, "
                  f"last landing {'-' if last_landing_body_x is None else format(last_landing_body_x * 1e3, '.1f')}) "
                  f"others: {others}")
        if prog.landing_cap_body_x_m is not None:
            if x_land > prog.landing_cap_body_x_m + SERIAL_TOLERANCE_M:
                raise PlannerRefusal2D(
                    f"{leg.value} was cut to lift before body x "
                    f"{prog.landing_cap_body_x_m * 1e3:.3f} mm but its "
                    f"{action.kind.value} lands at {x_land * 1e3:.3f} mm.")
            prog.landing_cap_body_x_m = None

        # Would this swing swallow another leg's liftoff?  Then that leg goes
        # FIRST: its pending stroke is cut so that a nominal-length swing from
        # the cut lands by the time this one lifts off.  Lifting early is the
        # one phase knob a leg has (a stroke can be shortened, never
        # lengthened past the rim), and it is what "the whole machine decides"
        # means here: the long swing keeps its geometry, the other legs make
        # room for it in body travel, and nothing is retimed after the fact.
        # A leg due at the *same* body x counts too: two legs cannot both lift
        # where the body stands, so the other one goes first, with a swing
        # that lands by this one's takeoff (down to a body pause).
        blockers = [p for p in progress.values()
                    if p.leg is not leg and not p.finished and p.stroke is not None
                    and x_lift - SERIAL_TOLERANCE_M <= p.liftoff_body_x_m
                    < x_land - SERIAL_TOLERANCE_M]
        uncuttable = [p for p in blockers
                      if p.stroke is not None
                      and (p.stroke.stop_reason == "RIGHT_RIM_TOP"
                           or (is_uncuttable is not None and is_uncuttable(p)))]
        edge_blockers: set = set()
        if uncuttable:
            # A climbing leg's stroke is the corner's geometry and cannot be
            # cut for anyone; this leg yields instead: it swings in place
            # where it still can, lands, and re-plans after the climb.
            # Measured at 100 mm: RF's climb landing could not be below the
            # climber's takeoff (the axle is too low mid-climb), so RF's
            # window covered the climber's takeoff every time.
            floor = -np.inf if last_landing_body_x is None else float(last_landing_body_x)
            pause_x = x_lift
            if action.kind.value in ("SWING_DOWN", "TOP_REPOSITION_SWING"):
                # A leg on the top yields by landing *by* the other leg's
                # takeoff, not necessarily in place: a descent from short of
                # the edge has no landing in place (measured at 100 mm, axle
                # rise 60: LF 47 mm short of the edge, capped at its own
                # liftoff, found none), but one that lands at the edge does.
                pause_x = min(u.liftoff_body_x_m for u in uncuttable)
                pause_x = max(pause_x, x_lift)
                edge = float(getattr(getattr(rule, "spec", None), "x_max_m", np.inf))
                margin = float(getattr(rule, "descend_in_place_margin_m", 0.010))
                if (action.kind.value == "SWING_DOWN"
                        and pause_x + prog.mount_x_m < edge - margin - 1e-9):
                    # No descent lands under that cap (the rim needs the hip
                    # within ~12 mm of the trailing edge, measured); the leg
                    # hops in place instead and rolls on to the edge after
                    # the other leg's swing (measured on a 900 mm top: RF
                    # 155 mm short of the edge was capped 18 mm short of it).
                    pause_x = x_lift
            if latest_pause is not None:
                latest_hip = latest_pause(prog)
                if latest_hip is not None:
                    pause_x = min(pause_x, float(latest_hip) - prog.mount_x_m)
            if pause_x < max(prog.stroke_start_body_x_m, floor) - 1e-9:
                edge_only = all(u.stroke is not None
                                and u.stroke.stop_reason != "RIGHT_RIM_TOP"
                                for u in uncuttable)
                if not edge_only:
                    raise PlannerRefusal2D(
                        f"{leg.value} must wait for {uncuttable[0].leg.value} (its "
                        f"{action.kind.value} [{x_lift * 1e3:.3f}, {x_land * 1e3:.3f}] mm covers "
                        f"its liftoff at {uncuttable[0].liftoff_body_x_m * 1e3:.3f} mm, which cannot be cut) but has "
                        f"no legal place to pause at or before {pause_x * 1e3:.3f} mm.")
                # The legs in the way stand at the top's trailing edge, and
                # this leg cannot pause for them; then they go first after
                # all, each descending in place at this takeoff (a stroke at
                # the edge is uncuttable for a *pause*, which would restart
                # it to the same edge, not for a descent a few mm early:
                # measured at 100 mm, axle rise 50, LH's climb began 3 mm
                # before the fronts reached the edge).
                edge_blockers = {u.leg for u in uncuttable}
                if trace:
                    print(f"      {leg.value} cannot pause for "
                          f"{', '.join(u.leg.value for u in uncuttable)} at the edge; "
                          f"they descend in place at {x_lift * 1e3:.1f} mm")
                uncuttable = []
        if uncuttable:
            key = (leg, round(pause_x, 9))
            if paused_at.get(key, 0) >= 1:
                raise PlannerRefusal2D(
                    f"{leg.value} already paused at {pause_x * 1e3:.3f} mm for "
                    f"{uncuttable[0].leg.value}'s climb and is still in its way.")
            paused_at[key] = paused_at.get(key, 0) + 1
            before = prog.liftoff_body_x_m
            if pause_x < x_lift - 1e-12:
                prog.stroke = truncate_stroke_2d(
                    prog.stroke, pause_x + prog.mount_x_m,
                    min_hip_x_m=max(prog.stroke_start_body_x_m, floor) + prog.mount_x_m,
                    hard_max_hip_x_m=pause_x + prog.mount_x_m, frame_at=frame_at)
            prog.landing_cap_body_x_m = float(pause_x)
            prog.pending_swing = None
            prog.cut_hip_m += float(before - prog.liftoff_body_x_m)
            if trace:
                print(f"      yield {leg.value}: liftoff {before * 1e3:.1f} -> "
                      f"{prog.liftoff_body_x_m * 1e3:.1f}, lands by {pause_x * 1e3:.1f} for "
                      f"{uncuttable[0].leg.value}'s climb")
            cuts.append({"leg": leg.value, "for": uncuttable[0].leg.value,
                         "for_kind": "RIGHT_RIM_ROLL_UP",
                         "liftoff_was_mm": before * 1e3,
                         "liftoff_now_mm": prog.liftoff_body_x_m * 1e3,
                         "cut_hip_mm": (before - prog.liftoff_body_x_m) * 1e3,
                         "landing_cap_mm": pause_x * 1e3,
                         "room_mm": (pause_x - prog.liftoff_body_x_m) * 1e3})
            continue
        if blockers:
            blockers.sort(key=lambda p: p.liftoff_body_x_m)
            floor = -np.inf if last_landing_body_x is None else float(last_landing_body_x)
            room = x_lift - floor
            # One blocker with room for a swing lifts as late as the room
            # allows and lands at this takeoff.  Several blockers, or no
            # room, swing in place *at* this takeoff, one after another with
            # the body standing still (a dwell on the clock), and this leg
            # lifts once they have all landed.  Cuts are exact (the rule
            # makes the stance frame at the cut), so no frame need exist.
            def latest_landing(other):
                # Not past where the rule says this leg's swing can still
                # land legally (before the face: the arc-start pose needs
                # room), whatever its advance.
                if latest_pause is None:
                    return x_lift
                latest_hip = latest_pause(other)
                if latest_hip is None:
                    return x_lift
                return min(x_lift, float(latest_hip) - other.mount_x_m)

            if (len(blockers) == 1 and room >= PAUSE_ADVANCE_M - 1e-9
                    and blockers[0].leg not in edge_blockers):
                cap = latest_landing(blockers[0])
                x_cut = min(blockers[0].liftoff_body_x_m, max(floor, cap - nominal_advance))
                if x_cut > cap:
                    x_cut = cap
                plan_cuts = [(blockers[0], x_cut, cap)]
            else:
                plan_cuts = []
                for other in blockers:
                    x_pause = latest_landing(other)
                    plan_cuts.append((other, x_pause, x_pause))
            for other, x_cut, cap in plan_cuts:
                key = (other.leg, round(x_lift, 9))
                if abs(x_cut - cap) < 1e-12:
                    if paused_at.get(key, 0) >= 1:
                        raise PlannerRefusal2D(
                            f"{other.leg.value} cannot stay in stance through {leg.value}'s "
                            f"{action.kind.value} [{x_lift * 1e3:.3f}, {x_land * 1e3:.3f}] mm: "
                            f"after swinging in place at {x_lift * 1e3:.3f} mm its next stroke "
                            f"ends at {other.liftoff_body_x_m * 1e3:.3f} mm, still inside.")
                    paused_at[key] = paused_at.get(key, 0) + 1
                start = other.stroke_start_body_x_m
                # The floor is the previous landing, which a slot-aware
                # search puts within SERIAL_TOLERANCE_M of this takeoff, not
                # exactly on it (measured at 100 mm: RF's climb landed 0.5 mm
                # past LF's liftoff, and LH's pause there was refused).
                if x_cut < max(start, floor) - SERIAL_TOLERANCE_M:
                    raise PlannerRefusal2D(
                        f"{other.leg.value} lifts off at body x {other.liftoff_body_x_m * 1e3:.3f} mm, "
                        f"inside {leg.value}'s {action.kind.value} "
                        f"[{x_lift * 1e3:.3f}, {x_land * 1e3:.3f}] mm, and cannot lift "
                        f"at {x_cut * 1e3:.3f} mm: its stroke starts at "
                        f"{start * 1e3:.3f} mm and the previous swing lands at "
                        f"{floor * 1e3:.3f} mm ({len(blockers)} legs to fit in).")
                before = other.liftoff_body_x_m
                if (other.landing_cap_body_x_m is not None
                        and abs(other.landing_cap_body_x_m - cap) < 1e-9
                        and abs(before - x_cut) < 1e-9):
                    raise PlannerRefusal2D(
                        f"{other.leg.value} was already cut to lift at body x "
                        f"{before * 1e3:.3f} mm for {leg.value}'s {action.kind.value} "
                        f"[{x_lift * 1e3:.3f}, {x_land * 1e3:.3f}] mm and is still inside it; "
                        "the cut cannot be repeated.")
                other.stroke = truncate_stroke_2d(
                    other.stroke, x_cut + other.mount_x_m,
                    min_hip_x_m=min(x_cut, max(start, floor)) + other.mount_x_m,
                    hard_max_hip_x_m=cap + other.mount_x_m, frame_at=frame_at)
                other.landing_cap_body_x_m = float(cap)
                other.pending_swing = None
                other.cut_hip_m += float(before - other.liftoff_body_x_m)
                if trace:
                    print(f"      cut {other.leg.value}: liftoff {before * 1e3:.1f} -> "
                          f"{other.liftoff_body_x_m * 1e3:.1f} (floor {floor * 1e3:.1f}, "
                          f"wanted {x_cut * 1e3:.1f}), lands by {cap * 1e3:.1f}")
                cuts.append({"leg": other.leg.value, "for": leg.value,
                             "for_kind": action.kind.value,
                             "liftoff_was_mm": before * 1e3,
                             "liftoff_now_mm": other.liftoff_body_x_m * 1e3,
                             "cut_hip_mm": (before - other.liftoff_body_x_m) * 1e3,
                             "landing_cap_mm": cap * 1e3,
                             "room_mm": (cap - other.liftoff_body_x_m) * 1e3})
            continue  # the cut legs are now due first

        minimum = minimum_swing_duration_s(swing.frames, utilisation=swing_utilisation)
        fold, rotate, extend = swing_phase_ranges_2d(swing.frames)
        if x_land - x_lift > 1e-9 and (fold is not None or extend is not None):
            # A transition that folds and extends with the body standing
            # still: the fold is a dwell at the takeoff, the rotation travels
            # the window, the extension is a dwell at the landing.
            timing = ["phased"]
            if fold is not None:
                d = max(float(phase_dwell_min_s),
                        minimum_swing_duration_s(swing.frames[fold[0]:fold[1] + 1],
                                                 utilisation=swing_utilisation))
                clock, _, _ = clock.with_dwell(x_lift, d)
                timing.append(("dwell", max(dw.order for dw in clock.dwells), fold))
                slowdowns.append({"leg": leg.value, "kind": action.kind.value + ":fold",
                                  "body_x_start_mm": x_lift * 1e3, "body_x_end_mm": x_lift * 1e3,
                                  "nominal_s": 0.0, "minimum_s": d, "speed_mm_s": 0.0, "dwell_s": d})
            start_s = clock.time_at_body_x(x_lift, side="leave")
            end_s = clock.time_at_body_x(x_land)
            d_rot = minimum_swing_duration_s(swing.frames[rotate[0]:rotate[1] + 1],
                                             utilisation=swing_utilisation)
            if (long_swing_speed_scale > 1.0 + 1e-9
                    and x_land - x_lift > nominal_advance + 1e-9):
                # A swing longer than nominal keeps three legs on the ground
                # for longer than the gait ever does; the body may cross that
                # stretch faster (up to the scale, never below the motors'
                # minimum time), so the stance is short in *time*.
                wanted = max(d_rot, (x_land - x_lift) / (clock.speed_m_s * long_swing_speed_scale))
                if end_s - start_s > wanted + 1e-9:
                    speed = (x_land - x_lift) / wanted
                    clock = clock.with_zone(SpeedZone2D(x_lift, x_land, speed))
                    slowdowns.append({"leg": leg.value, "kind": action.kind.value + ":rotate",
                                      "body_x_start_mm": x_lift * 1e3, "body_x_end_mm": x_land * 1e3,
                                      "nominal_s": end_s - start_s, "minimum_s": d_rot,
                                      "speed_mm_s": speed * 1e3, "dwell_s": 0.0})
                    end_s = clock.time_at_body_x(x_land)
            if end_s - start_s + 1e-12 < d_rot:
                speed = (x_land - x_lift) / d_rot
                clock = clock.with_zone(SpeedZone2D(x_lift, x_land, speed))
                slowdowns.append({"leg": leg.value, "kind": action.kind.value + ":rotate",
                                  "body_x_start_mm": x_lift * 1e3, "body_x_end_mm": x_land * 1e3,
                                  "nominal_s": end_s - start_s, "minimum_s": d_rot,
                                  "speed_mm_s": speed * 1e3, "dwell_s": 0.0})
            timing.append(("travel", float(x_lift), float(x_land), rotate))
            if extend is not None:
                d = max(float(phase_dwell_min_s),
                        minimum_swing_duration_s(swing.frames[extend[0]:extend[1] + 1],
                                                 utilisation=swing_utilisation))
                clock, _, end_s = clock.with_dwell(x_land, d)
                timing.append(("dwell", max(dw.order for dw in clock.dwells), extend))
                slowdowns.append({"leg": leg.value, "kind": action.kind.value + ":extend",
                                  "body_x_start_mm": x_land * 1e3, "body_x_end_mm": x_land * 1e3,
                                  "nominal_s": 0.0, "minimum_s": d, "speed_mm_s": 0.0, "dwell_s": d})
            else:
                end_s = clock.time_at_body_x(x_land)
            timing = tuple(timing)
        elif x_land - x_lift <= 1e-9:
            # Flown with the body standing still: a dwell on the clock, after
            # any pauses already held at this x.  Recorded as a slowdown to
            # zero speed.
            clock, start_s, end_s = clock.with_dwell(x_lift, minimum)
            timing = ("dwell", max(d.order for d in clock.dwells))
            slowdowns.append({
                "leg": leg.value, "kind": action.kind.value,
                "body_x_start_mm": x_lift * 1e3, "body_x_end_mm": x_land * 1e3,
                "nominal_s": 0.0, "minimum_s": minimum, "speed_mm_s": 0.0,
                "dwell_s": minimum,
            })
        else:
            start_s = clock.time_at_body_x(x_lift, side="leave")
            end_s = clock.time_at_body_x(x_land)
            if end_s - start_s + 1e-12 < minimum:
                # The hip travel does not give the motors enough time: slow
                # the body over exactly this stretch, for every leg.
                speed = (x_land - x_lift) / minimum
                clock = clock.with_zone(SpeedZone2D(x_lift, x_land, speed))
                slowdowns.append({
                    "leg": leg.value, "kind": action.kind.value,
                    "body_x_start_mm": x_lift * 1e3, "body_x_end_mm": x_land * 1e3,
                    "nominal_s": end_s - start_s, "minimum_s": minimum,
                    "speed_mm_s": speed * 1e3, "dwell_s": 0.0,
                })
                end_s = clock.time_at_body_x(x_land)
            timing = ("travel", float(x_lift), float(x_land))
        if isinstance(timing, tuple) and timing and timing[0] == "phased":
            # the swing's start is its first phase's start
            first = timing[1]
            start_s = (clock.dwell_window_s(first[1])[0] if first[0] == "dwell"
                       else clock.time_at_body_x(first[1], side="leave"))
        build.pending.append((prog.stroke, prog.stroke_action, None))
        build.pending.append((None, None, (swing, action, timing)))
        swings.append(SwingEvent2D(
            leg=leg, body_x_start_m=x_lift, body_x_end_m=x_land,
            start_s=start_s, end_s=end_s, kind=action.kind.value,
            minimum_duration_s=minimum))
        last_landing_body_x = x_land
        prog.swings_done += 1

        following = rule.stroke_after(prog, swing.end)
        if following is None:
            prog.finished = True
            prog.stroke = None
            prog.stroke_action = None
            continue
        stroke = _stance_stroke(following, posture,
                                hip_x_m=float(swing.end.hip_xz_m[0]),
                                start_beta_rad=float(swing.end.beta_rad))
        if not stroke.success:
            raise PlannerRefusal2D(
                f"{leg.value}: stroke after {action.kind.value} refused: "
                f"{stroke.stop_reason}")
        prog.stroke = stroke
        prog.stroke_action = following
        prog.pending_swing = None
        prog.strokes_done += 1

    if crossing_speed_scale < 1.0 - 1e-9:
        # The owner's direction: slow the body over the crossing only, the
        # flat approach and exit at the gait's own pace.  The crossing is
        # from the first terrain swing's takeoff to the last one's landing
        # (plus a margin); zones the loop already placed there keep their
        # own speeds, the gaps between them get the scaled speed.
        terrain = [e for e in swings if e.kind not in ("RECOVERY_SWING",)]
        if terrain:
            x0 = min(e.body_x_start_m for e in terrain) - float(crossing_margin_m)
            x1 = max(e.body_x_end_m for e in terrain) + float(crossing_margin_m)
            cursor = x0
            for zone in sorted(clock.zones, key=lambda z: z.x_start_m):
                if zone.x_end_m <= x0 or zone.x_start_m >= x1:
                    continue
                if zone.x_start_m > cursor + 1e-9:
                    clock = clock.with_zone(SpeedZone2D(
                        cursor, zone.x_start_m, clock.speed_m_s * crossing_speed_scale))
                cursor = max(cursor, zone.x_end_m)
            if x1 > cursor + 1e-9:
                clock = clock.with_zone(SpeedZone2D(cursor, x1, clock.speed_m_s * crossing_speed_scale))
            slowdowns.append({"leg": "body", "kind": "CROSSING_SLOW",
                              "body_x_start_mm": x0 * 1e3, "body_x_end_mm": x1 * 1e3,
                              "nominal_s": 0.0, "minimum_s": 0.0,
                              "speed_mm_s": clock.speed_m_s * crossing_speed_scale * 1e3,
                              "dwell_s": 0.0})

    # Now that every zone and dwell is known, write the strokes cut at their
    # boundaries: a segment's frames are played uniformly in time, so a
    # stroke must not span a change of body speed, and it must not span a
    # body stop either.
    boundaries = sorted({z.x_start_m for z in clock.zones} | {z.x_end_m for z in clock.zones}
                        | {d.x_m for d in clock.dwells})
    for build in builds.values():
        for stroke, stance_action, airborne in build.pending:
            if airborne is not None:
                build.add_swing(*airborne)
            else:
                build.add_roll(stroke, stance_action,
                               cut_at_hip_x_m=[x + build.mount_x_m for x in boundaries],
                               frame_at=frame_at)
    plans, scheduled = _write_out(builds, clock, tolerance)
    return GaitFirstPlan2D(
        plans=plans, clock=clock, swings=tuple(swings), slowdowns=tuple(slowdowns),
        cuts=tuple(cuts),
        schedule=FourLegSchedule2D(timing=clock.timing, scheduled=tuple(scheduled)))


def swing_phase_ranges_2d(frames):
    """``(fold, rotate, extend)`` frame index ranges (inclusive) of a swing
    whose hip stands still while it folds and while it extends, or ``None``
    for a phase in which the hip moves (a nominal recovery: all ``None``
    but the rotation).

    The fold is the leading run of frames at the takeoff hip x (the takeoff
    frame itself included), the extension the trailing run at the landing
    hip x, and the rotation everything between, sharing its boundary frames
    with both.
    """

    hips = [float(f.hip_xz_m[0]) for f in frames]
    n = len(hips)
    if n < 3 or abs(hips[-1] - hips[0]) <= 1e-9:
        return None, (0, n - 1), None
    a = 0
    while a + 1 < n and abs(hips[a + 1] - hips[0]) <= 1e-9:
        a += 1
    b = n - 1
    while b - 1 > a and abs(hips[b - 1] - hips[-1]) <= 1e-9:
        b -= 1
    fold = (0, a) if a >= 1 else None
    extend = (b, n - 1) if b <= n - 2 else None
    return fold, (a, b), extend


def truncate_stroke_2d(stroke: RollStroke2D, max_hip_x_m: float,
                       min_hip_x_m: float | None = None,
                       hard_max_hip_x_m: float | None = None,
                       frame_at=None) -> RollStroke2D:
    """The same stroke, ended at the last frame whose hip is at or before ``max_hip_x_m``.

    With ``frame_at(stroke, hip_x_m)`` (the rule's exact stance pose between
    two frames) the stroke is ended at ``max_hip_x_m`` itself, and the floor
    and hard maximum are moot.

    A stroke is generated step by step and ``max_distance_m`` only stops it
    earlier, so a prefix of a full stroke's frames *is* the shorter stroke the
    generator would have produced -- no regeneration, no new geometry.  At
    least the first frame is kept: a one-frame stroke is a leg that lifts off
    where it landed.

    Frames are a roll step apart (6.4 mm of hip), so the last frame at or
    before ``max_hip_x_m`` can sit a step *below* a floor the caller must
    respect (the previous swing's landing).  ``min_hip_x_m`` then moves the
    cut up to the first frame at or above the floor, as long as that is still
    at or before ``hard_max_hip_x_m``; otherwise there is no frame in the
    window and a ``PlannerRefusal2D`` says so.
    """

    hips = [float(f.hip_xz_m[0]) for f in stroke.frames]
    if frame_at is not None:
        x = float(max_hip_x_m)
        if x >= hips[-1] - 1e-9:
            return stroke
        x = max(x, hips[0])
        prefix = [f for f, h in zip(stroke.frames, hips) if h <= x + 1e-9]
        if not prefix or float(prefix[-1].hip_xz_m[0]) < x - 1e-9:
            prefix.append(frame_at(stroke, x))
        end = prefix[-1]
        scene = stroke.posture.scene(float(end.beta_rad), float(end.hip_xz_m[0]),
                                     float(end.hip_xz_m[1]), theta_rad=float(end.theta_rad))
        return replace(stroke, frames=tuple(prefix), success=True,
                       stop_reason="CUT_FOR_ANOTHER_SWING", final_scene=scene)
    count = sum(1 for h in hips if h <= float(max_hip_x_m) + 1e-12)
    if count == 0:
        count = 1
    if min_hip_x_m is not None and hips[count - 1] < float(min_hip_x_m) - 1e-12:
        above = [i for i, h in enumerate(hips) if h >= float(min_hip_x_m) - 1e-12]
        if not above:
            raise PlannerRefusal2D(
                f"no frame of the stroke reaches hip x {min_hip_x_m * 1e3:.3f} mm.")
        count = above[0] + 1
        limit = max_hip_x_m if hard_max_hip_x_m is None else hard_max_hip_x_m
        if hips[count - 1] > float(limit) + 1e-12:
            raise PlannerRefusal2D(
                f"no frame of the stroke lies between hip x {min_hip_x_m * 1e3:.3f} "
                f"and {limit * 1e3:.3f} mm (the roll step is "
                f"{(hips[1] - hips[0]) * 1e3 if len(hips) > 1 else 0:.1f} mm).")
    frames = list(stroke.frames[:count])
    end = frames[-1]
    scene = stroke.posture.scene(float(end.beta_rad), float(end.hip_xz_m[0]),
                                 float(end.hip_xz_m[1]), theta_rad=float(end.theta_rad))
    return replace(stroke, frames=tuple(frames), success=True,
                   stop_reason="CUT_FOR_ANOTHER_SWING" if len(frames) < len(stroke.frames)
                   else stroke.stop_reason,
                   final_scene=scene)


def _stance_stroke(action: StanceAction2D, posture: NominalPosture2D, *,
                   hip_x_m: float, start_beta_rad: float | None) -> RollStroke2D:
    if action.stroke is not None:
        return action.stroke
    return run_foot_rim_roll_2d(posture, start_beta_rad=start_beta_rad,
                                hip_x_m=float(hip_x_m),
                                max_distance_m=action.max_distance_m)


def _airborne_swing(action: SwingAction2D, stroke: RollStroke2D,
                    config: RecoveryConfig2D) -> RecoverySwing2D:
    if action.swing is not None:
        return action.swing
    if action.hip_advance_m is not None:
        config = replace(config, hip_advance_m=float(action.hip_advance_m))
    return run_recovery_swing_2d(
        stroke, config,
        beta_target_rad=action.beta_target_rad,
        theta_touchdown_rad=action.theta_touchdown_rad,
        hip_z_touchdown_m=action.hip_z_touchdown_m,
        lift_hip_before_rotation=action.lift_hip_before_rotation)


def _write_out(builds: dict, clock: GaitClock2D,
               tolerance: ChainTolerance2D | None):
    """Segments and frames into ``LegPlan2D``; times off the clock."""

    plans: dict = {}
    scheduled: list[ScheduledSegment2D] = []
    tol = ChainTolerance2D() if tolerance is None else tolerance
    for leg in LEG_ORDER:
        build = builds[leg]
        timed = _fill_holds(build, _segment_times(build, clock))
        segments = tuple(build.segments)
        for index in range(1, len(segments)):
            if timed[index][0] < timed[index - 1][1] - 1e-12:
                lines = []
                for k in range(max(0, index - 4), min(len(segments), index + 3)):
                    seg = segments[k]
                    lines.append(
                        f"      [{k}] {seg.kind.value:22s} {seg.phase_label:18s} "
                        f"hip x {float(seg.start_contact.hip_xz_m[0]) * 1e3:7.1f}->{float(seg.end_contact.hip_xz_m[0]) * 1e3:7.1f} "
                        f"t {timed[k][0]:8.4f}..{timed[k][1]:8.4f} timing {build.timings[k]}")
                raise PlannerRefusal2D(
                    f"{leg.value} segment {index} starts at {timed[index][0]:.4f} s before segment "
                    f"{index - 1} ends at {timed[index - 1][1]:.4f} s:\n" + "\n".join(lines))
        phased = tuple(PhasedSegment2D(seg, phase, label)
                       for seg, phase, label in
                       zip(segments, build.phases, build.labels))
        chain = SegmentChain2D(
            leg_id=leg.value, segments=segments, tolerance=tol, unresolved=(),
            notes="Day 14: generated on the body's clock, in liftoff order.")
        _, breaks = chain_boundaries_2d(segments, tol)
        plans[leg] = LegPlan2D(
            leg=leg, phased=phased, chain=chain, strategy=None,
            ascent_strategy=None, descent_strategy=None, unresolved=(),
            refusal=None, breaks=tuple(breaks),
            frames={build.source_id: tuple(build.frames)},
            notes="gait-first")
        for index, segment in enumerate(segments):
            start_s, end_s = timed[index]
            if end_s < start_s - 1e-12:
                raise PlannerRefusal2D(
                    f"{leg.value} segment {index} ({segment.kind.value}) runs "
                    "the hip backwards; the body does not reverse.")
            scheduled.append(ScheduledSegment2D(
                leg=leg, window_index=index, segment_index=index,
                segment_kind=segment.kind, phase_label=segment.phase_label,
                mode=LegMode.AIRBORNE if segment.kind.is_swing else LegMode.STANCE,
                start_s=float(start_s), end_s=float(max(end_s, start_s)),
                frame_count=segment.frames.frame_count,
                duration_is_assigned=True))
    return plans, scheduled


def _segment_times(build: _LegBuild, clock: GaitClock2D) -> list[tuple[float, float]]:
    """Each segment's ``(start_s, end_s)`` off the final clock.

    A stroke begins when the body *leaves* its first hip x and ends when it
    *arrives* at its last, so a stroke ending where the body dwells ends
    before the pauses held there and the next one begins after them.  A
    swing flown in place is its own dwell; any other swing runs from leaving
    its takeoff to arriving at its landing.
    """

    out = []
    for segment, timing in zip(build.segments, build.timings):
        if timing is None:
            start_s = clock.time_at_body_x(
                float(segment.start_contact.hip_xz_m[0]) - build.mount_x_m, side="leave")
            end_s = clock.time_at_body_x(
                float(segment.end_contact.hip_xz_m[0]) - build.mount_x_m)
        elif timing[0] == "dwell":
            start_s, end_s = clock.dwell_window_s(int(timing[1]))
        else:
            start_s = clock.time_at_body_x(float(timing[1]), side="leave")
            end_s = clock.time_at_body_x(float(timing[2]))
        out.append((float(start_s), float(end_s)))
    return out


def _fill_holds(build: _LegBuild, timed: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Insert a ``BODY_HOLD`` wherever the leg stands between two segments.

    The body dwelling while another leg swings in place leaves every stance
    leg (and a leg that has just landed and waits for the next pause) with
    a gap in time between two of its segments.  The pose is the same on both
    sides; the hold says so explicitly, so the trajectory sampler finds a
    segment at every instant instead of dropping the leg.
    """

    segments, phases, labels, timings, out = [], [], [], [], []
    for index, segment in enumerate(build.segments):
        start_s, end_s = timed[index]
        if out and start_s > out[-1][1] + 1e-9:
            hold = _hold_segment_2d(segments[-1], start_s - out[-1][1], "BODY_HOLD")
            # The chain contract gives every frame to exactly one segment, so
            # the hold gets its own copy of the frame it holds.
            held = build.frames[hold.frames.indices[0]]
            build.frames.append(held)
            hold = replace(hold, frames=FrameRef2D(source_id=hold.frames.source_id,
                                                   indices=(len(build.frames) - 1,)))
            segments.append(hold)
            phases.append(phases[-1])
            labels.append("gait_first")
            timings.append(("hold",))
            out.append((out[-1][1], start_s))
        segments.append(segment)
        phases.append(build.phases[index])
        labels.append(build.labels[index])
        timings.append(build.timings[index])
        out.append((start_s, end_s))
    build.segments[:] = segments
    build.phases[:] = phases
    build.labels[:] = labels
    build.timings[:] = timings
    return out


# --------------------------------------------------------------------------
# Through the Day 12 tail: body, stability, assembly, validation
# --------------------------------------------------------------------------


def run_through_day12_2d(
    plan: GaitFirstPlan2D,
    *,
    nominal_body_z_m: float,
    samples: int,
    margin_floor_m: float,
    terrain=None,
    composed: ComposedSequence2D | None = None,
) -> TerrainRun2D:
    """Body trajectory, support margin, assembly and validation, as Day 12 does them.

    ``world_registered=True`` because every leg here *is* in the world: the
    four hips read one clock, so the body x is the median of what they say and
    ``world_x_spread_m`` measures how far they disagree -- by construction it
    should be zero, and the test pins that rather than assuming it.
    """

    four = plan_four_legs_2d(plan.plans, plan.clock.timing, schedule=plan.schedule)
    body = body_trajectory_2d(four, nominal_body_z_m=float(nominal_body_z_m),
                              samples=int(samples), world_registered=True)
    stability = swing_stability_2d(four, body, margin_floor_m=float(margin_floor_m))
    trajectory = assemble_whole_body_2d(four, body, stability, samples=int(samples),
                                        use_generator_frames=True)
    report = validate_whole_body_2d(trajectory, body, stability)
    failures = [TerrainFailure2D(stage=Stage.VALIDATION,
                                 detail=f"{check.value}: {report.failures_of(check)[0].detail}")
                for check in report.failed_checks()]
    if composed is None:
        composed = ComposedSequence2D(
            strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
            sequence=None, refusal="flat ground: there is no crossing to compose")
    height = 0.0 if terrain is None else float(terrain.height_m)
    top = 0.0 if terrain is None else float(terrain.top_length_m)
    return TerrainRun2D(
        height_m=height, top_length_m=top, terrain=terrain, plan=four,
        body=body, stability=stability, trajectory=trajectory, report=report,
        composed=composed, failures=tuple(failures),
        swing_waits=tuple(plan.slowdowns))


# --------------------------------------------------------------------------
# The body plane, read off the four hips
# --------------------------------------------------------------------------


def leg_hip_z_at(four: FourLegPlan2D, leg: LegId, time_s: float) -> float | None:
    """This leg's hip height at ``time_s``, interpolated between its frames.

    The same lookup ``leg_sample_at`` makes for theta and beta, applied to the
    hip z the generator recorded; ``LegSample2D`` does not carry it.
    """

    from hybrid_note.scripts.experiments.day12_support_stability_2d import segment_at

    scheduled = segment_at(four.schedule.segments_of(leg), float(time_s))
    if scheduled is None:
        return None
    leg_plan = four.plans[leg]
    segment = leg_plan.phased[scheduled.segment_index].segment
    frames = leg_plan.frames.get(segment.frames.source_id)
    if not frames:
        return None
    span = scheduled.end_s - scheduled.start_s
    fraction = float(np.clip(0.0 if span <= 0.0 else
                             (time_s - scheduled.start_s) / span, 0.0, 1.0))
    indices = segment.frames.indices
    place = fraction * (len(indices) - 1)
    low = int(np.clip(np.floor(place), 0, len(indices) - 1))
    high = int(min(low + 1, len(indices) - 1))
    blend = float(place - low)
    a, b = frames[indices[low]], frames[indices[high]]
    return float(a.hip_xz_m[1] + (b.hip_xz_m[1] - a.hip_xz_m[1]) * blend)


def leg_hip_state_at(four: FourLegPlan2D, leg: LegId, time_s: float) -> dict | None:
    """Hip x and z, the segment kind and label, and where in the segment
    ``time_s`` falls -- for diagnosing where two hips of one axle disagree."""

    from hybrid_note.scripts.experiments.day12_support_stability_2d import segment_at

    scheduled = segment_at(four.schedule.segments_of(leg), float(time_s))
    if scheduled is None:
        return None
    leg_plan = four.plans[leg]
    segment = leg_plan.phased[scheduled.segment_index].segment
    frames = leg_plan.frames.get(segment.frames.source_id)
    span = scheduled.end_s - scheduled.start_s
    fraction = float(np.clip(0.0 if span <= 0.0 else
                             (time_s - scheduled.start_s) / span, 0.0, 1.0))
    indices = segment.frames.indices
    place = fraction * (len(indices) - 1)
    low = int(np.clip(np.floor(place), 0, len(indices) - 1))
    high = int(min(low + 1, len(indices) - 1))
    blend = float(place - low)
    a, b = frames[indices[low]], frames[indices[high]]
    return {"hip_x_mm": round(float(a.hip_xz_m[0] + (b.hip_xz_m[0] - a.hip_xz_m[0]) * blend) * 1e3, 2),
            "hip_z_mm": round(float(a.hip_xz_m[1] + (b.hip_xz_m[1] - a.hip_xz_m[1]) * blend) * 1e3, 2),
            "kind": segment.kind.value, "label": segment.phase_label,
            "segment": scheduled.segment_index, "window_s": (round(scheduled.start_s, 4), round(scheduled.end_s, 4)),
            "frames": len(indices), "fraction": round(fraction, 3),
            "hip_x_ends_mm": (round(float(frames[indices[0]].hip_xz_m[0]) * 1e3, 2),
                              round(float(frames[indices[-1]].hip_xz_m[0]) * 1e3, 2)),
            "hip_x_uniform_mm": [round(float(frames[i].hip_xz_m[0]) * 1e3, 1) for i in indices][:12]}


def refit_body_plane_2d(run: TerrainRun2D) -> tuple[TerrainRun2D, dict]:
    """Body height and pitch from the four hips, sample by sample.

    ``body_trajectory_2d`` merges the legs' *demands*; a held stroke demands
    nothing, so on an obstacle its body height stays nominal while the hips
    of the legs on the top are somewhere else.  The gait-first planner knows
    every hip, so the body is read off them: a rigid frame carries the four
    hips on one plane, and in the sagittal model that plane is the line
    ``hip_z = z0 + slope * mount_x``.  The residual of that fit is the one
    body constraint the project owner keeps (2026-09-07: "只要四個點可以維持是
    一個平面就可以了") and it is reported, never hidden.
    """

    four = run.plan
    mounts = {m.leg: float(m.offset_body_xyz_m[0]) for m in leg_mounts_2d(GAMMA_RAD)}
    samples = []
    worst_residual = 0.0
    worst_at: dict = {}
    pitches = []
    for sample in run.trajectory.samples:
        xs, zs = [], []
        for leg in LEG_ORDER:
            if leg not in four.plans:
                continue
            hip_z = leg_hip_z_at(four, leg, sample.time_s)
            if hip_z is None:
                continue
            xs.append(mounts[leg])
            zs.append(hip_z)
        if len(xs) < 2 or np.ptp(xs) < 1e-9:
            samples.append(sample)
            continue
        slope, intercept = np.polyfit(np.asarray(xs), np.asarray(zs), 1)
        errors = np.abs(np.asarray(zs) - (slope * np.asarray(xs) + intercept))
        residual = float(np.max(errors))
        if residual > worst_residual:
            worst_residual = residual
            legs_here = [leg for leg in LEG_ORDER if leg in four.plans
                         and leg_hip_z_at(four, leg, sample.time_s) is not None]
            worst_at = {"time_s": float(sample.time_s),
                        "body_x_mm": round(float(sample.body_position_world_m[0]) * 1e3, 2),
                        "hips_mm": {leg.value: round(z * 1e3, 2) for leg, z in zip(legs_here, zs)},
                        "legs": {leg.value: leg_hip_state_at(four, leg, sample.time_s)
                                 for leg in legs_here}}
        pitch = float(np.arctan(slope))
        pitches.append(pitch)
        x, y, _ = sample.body_position_world_m
        samples.append(replace(
            sample,
            body_position_world_m=(x, y, float(intercept) - HIP_TO_BODY_Z_M),
            body_rpy_rad=(0.0, pitch, 0.0)))
    trajectory = replace(run.trajectory, samples=tuple(samples),
                         assumptions=run.trajectory.assumptions + (
                             "Day 14: body z and pitch are the plane fitted "
                             "through the four hips' own frames, not the merged "
                             "demand", ))
    report = {
        "coplanarity_residual_max_mm": worst_residual * 1e3,
        "coplanarity_worst_at": worst_at,
        "pitch_min_deg": float(np.rad2deg(min(pitches))) if pitches else 0.0,
        "pitch_max_deg": float(np.rad2deg(max(pitches))) if pitches else 0.0,
        "body_z_min_mm": float(min(s.body_position_world_m[2] for s in samples)) * 1e3,
        "body_z_max_mm": float(max(s.body_position_world_m[2] for s in samples)) * 1e3,
    }
    return replace(run, trajectory=trajectory), report


def plan_terrain_gait_first_2d(
    rule: LegRule2D,
    *,
    terrain,
    strategy: StrategyId,
    timing: GaitTiming2D | None = None,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    origin_x_m: float = 0.0,
    long_swing_speed_scale: float = 1.0,
    crossing_speed_scale: float = 1.0,
    phase_dwell_min_s: float = 0.0,
    samples: int = 121,
    margin_floor_m: float = HYBRID_MARGIN_FLOOR_M,
    nominal_body_z_m: float | None = None,
) -> tuple[TerrainRun2D, GaitFirstPlan2D]:
    """One terrain on the gait-first planner, through the Day 12 tail."""

    timing = hybrid_timing_2d() if timing is None else timing
    posture = hybrid_posture_2d() if posture is None else posture
    if config is None:
        config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))
    clock, _, _ = hybrid_clock_2d(timing, posture, config, origin_x_m=origin_x_m)
    plan = plan_gait_first_2d(rule, clock, posture, config,
                              long_swing_speed_scale=long_swing_speed_scale,
                              crossing_speed_scale=crossing_speed_scale,
                              phase_dwell_min_s=phase_dwell_min_s)
    if nominal_body_z_m is None:
        nominal_body_z_m = (hybrid_body_z_m() if posture.hold_hip_z_m is None
                            else float(posture.hold_hip_z_m) - HIP_TO_BODY_Z_M)
    composed = ComposedSequence2D(
        strategy=strategy, height_m=float(terrain.height_m),
        top_length_m=float(terrain.top_length_m), sequence=None, refusal=None,
        notes="Day 14 gait-first: per-leg transitions on the body's clock; "
              "there is no single composed single-leg sequence.")
    run = run_through_day12_2d(plan, nominal_body_z_m=nominal_body_z_m,
                               samples=samples, margin_floor_m=margin_floor_m,
                               terrain=terrain, composed=composed)
    return run, plan


def plan_flat_gait_first_2d(
    *,
    cycles: int = 2,
    timing: GaitTiming2D | None = None,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    origin_x_m: float = 0.0,
    long_swing_speed_scale: float = 1.0,
    crossing_speed_scale: float = 1.0,
    phase_dwell_min_s: float = 0.0,
    samples: int = 121,
    margin_floor_m: float = HYBRID_MARGIN_FLOOR_M,
    nominal_body_z_m: float | None = None,
) -> tuple[TerrainRun2D, GaitFirstPlan2D]:
    """Flat ground on the gait-first planner: the Step 1 regression target."""

    timing = hybrid_timing_2d() if timing is None else timing
    posture = hybrid_posture_2d() if posture is None else posture
    if config is None:
        config = RecoveryConfig2D(hip_advance_m=swing_hip_advance_m(timing, posture))
    rule = FlatRule2D(cycles=int(cycles), posture=posture, config=config)
    clock, _, _ = hybrid_clock_2d(timing, posture, config, origin_x_m=origin_x_m,
                                  reference_cycle=rule._template[0])
    plan = plan_gait_first_2d(rule, clock, posture, config,
                              long_swing_speed_scale=long_swing_speed_scale,
                              crossing_speed_scale=crossing_speed_scale,
                              phase_dwell_min_s=phase_dwell_min_s)
    if nominal_body_z_m is None:
        nominal_body_z_m = (hybrid_body_z_m() if posture.hold_hip_z_m is None
                            else float(posture.hold_hip_z_m) - HIP_TO_BODY_Z_M)
    run = run_through_day12_2d(plan, nominal_body_z_m=nominal_body_z_m,
                               samples=samples, margin_floor_m=margin_floor_m)
    return run, plan
