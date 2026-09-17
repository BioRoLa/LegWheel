"""Day 12 Step 4: put the Day 10--11 per-leg sequences on the common timeline.

Plan §11.  Everything about *what* a leg does over the obstacle was decided in
Day 10--11; this step decides only *when*, and it reports what does not fit
rather than adjusting anything.

**Calling ``compose_2d`` is not re-deciding.**  ``decide_2d`` is a pure lookup
over tables frozen by earlier steps, and ``compose_2d`` turns the already-made
choice into motion.  Re-deciding would look like a fresh rule here picking ROLL
or SWING; there is none.  What this module adds is placement in time.

**Ascent and descent stay separate** (plan §11 "重要").  Each segment carries a
:class:`TransitionPhase`, and a plan reports its ascent and descent strategies
independently.  ``#5 SWING_OVER`` is the one case where they genuinely are a
single decision -- it is labelled ``OVER`` and
:attr:`LegPlan2D.phases_are_separable` says so, instead of the code pretending
to a split the primitive does not have.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from functools import lru_cache
from enum import Enum
from pathlib import Path
from typing import Sequence

import numpy as np

from hybrid_note.scripts.experiments.day10_11_composer_2d import (
    ComposedSequence2D,
    compose_2d,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    DecisionTables2D,
    StrategyId,
    load_tables_2d,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirementKind,
    FrameRef2D,
    MotionSegment2D,
    PointContact2D,
    SegmentKind,
    TransitionRequirement2D,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    NominalPosture2D,
    RecoveryConfig2D,
    cycle_segments_2d,
    run_nominal_cycles_2d,
)
from hybrid_note.scripts.experiments.day12_segment_contract_2d import (
    ChainBreak2D,
    ChainTolerance2D,
    SegmentChain2D,
    chain_boundaries_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    FourLegSchedule2D,
    GaitTiming2D,
    LegMode,
    ScheduledSegment2D,
    schedule_chains_2d,
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER, LegId


# --------------------------------------------------------------------------
# Which part of the obstacle a segment belongs to
# --------------------------------------------------------------------------


class TransitionPhase(str, Enum):
    """Where a segment sits in the crossing.

    Read off :class:`SegmentKind`, which already names the phase exactly --
    there is nothing to infer from geometry, and inferring it would be a second
    source of truth that can disagree with the first.
    """

    NOMINAL_BEFORE = "NOMINAL_BEFORE"
    ASCENT = "ASCENT"
    ON_TOP = "ON_TOP"
    DESCENT = "DESCENT"
    #: ``#5`` crosses in one primitive.  Not a missing split: an absent one.
    OVER = "OVER"
    NOMINAL_AFTER = "NOMINAL_AFTER"

    @property
    def is_nominal(self) -> bool:
        return self in (TransitionPhase.NOMINAL_BEFORE,
                        TransitionPhase.NOMINAL_AFTER)

    @property
    def is_transition(self) -> bool:
        return not self.is_nominal


#: Every terrain-transition kind, mapped to the phase it belongs to.  A kind
#: missing from here is a mapping bug, not a default: see :func:`phase_of_kind`.
TRANSITION_PHASE_OF_KIND: dict[SegmentKind, TransitionPhase] = {
    SegmentKind.APPROACH: TransitionPhase.ASCENT,
    SegmentKind.ROLL_UP: TransitionPhase.ASCENT,
    SegmentKind.SWING_UP: TransitionPhase.ASCENT,
    SegmentKind.WHEEL_TRANSITION: TransitionPhase.ON_TOP,
    SegmentKind.ROLL_DOWN: TransitionPhase.DESCENT,
    SegmentKind.SWING_DOWN: TransitionPhase.DESCENT,
    SegmentKind.POST_TOUCHDOWN_ROLL: TransitionPhase.DESCENT,
    SegmentKind.SWING_OVER: TransitionPhase.OVER,
    SegmentKind.TOP_REPOSITION_SWING: TransitionPhase.ON_TOP,
}


def phase_of_kind(kind: SegmentKind, *, after_transition: bool) -> TransitionPhase:
    """The phase a segment of ``kind`` belongs to.

    ``after_transition`` disambiguates the *nominal* kinds only: the same
    ``FOOT_RIM_ROLL`` is the approach run before the obstacle and the departure
    run after it, and which one it is depends on where it was inserted, not on
    anything the segment carries.
    """

    kind = SegmentKind(kind)
    if kind.is_terrain_transition:
        try:
            return TRANSITION_PHASE_OF_KIND[kind]
        except KeyError:  # pragma: no cover - guarded by a test
            raise KeyError(
                f"{kind.value} is a terrain-transition kind with no phase; add "
                "it to TRANSITION_PHASE_OF_KIND rather than defaulting it."
            ) from None
    return (TransitionPhase.NOMINAL_AFTER if after_transition
            else TransitionPhase.NOMINAL_BEFORE)


#: Which strategy each half of a pair came from.  The ``StrategyId`` name is a
#: pair, and Step 4 must be able to answer for the halves separately (plan §11).
STRATEGY_HALVES: dict[StrategyId, tuple[str, str]] = {
    StrategyId.ROLL_ROLL: ("ROLL_UP", "ROLL_DOWN"),
    StrategyId.ROLL_SWING: ("ROLL_UP", "SWING_DOWN"),
    StrategyId.SWING_ROLL: ("SWING_UP", "ROLL_DOWN"),
    StrategyId.SWING_SWING: ("SWING_UP", "SWING_DOWN"),
    StrategyId.SWING_OVER: ("SWING_OVER", "SWING_OVER"),
}


# --------------------------------------------------------------------------
# A segment with its label -- the segment itself is never copied
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class PhasedSegment2D:
    """A :class:`MotionSegment2D` plus where Step 4 put it.

    The segment is held by reference.  Its sampling and its body requirement
    are plan §11 requirement 4's "preserve", and the way to preserve them is to
    not rewrite them: everything below reads through to ``segment``.
    """

    segment: MotionSegment2D
    phase: TransitionPhase
    #: Which generator the frames came from, e.g. ``"nominal"`` or the strategy.
    source_label: str

    @property
    def kind(self) -> SegmentKind:
        return self.segment.kind

    @property
    def mode(self) -> LegMode:
        return LegMode.of(self.segment.kind)

    @property
    def body_kind(self) -> BodyRequirementKind | None:
        requirement = self.segment.body_requirement
        return None if requirement is None else requirement.kind

    def as_dict(self) -> dict:
        return {
            "phase": self.phase.value,
            "source_label": self.source_label,
            "segment_kind": self.kind.value,
            "phase_label": self.segment.phase_label,
            "mode": self.mode.value,
            "frame_count": self.segment.frames.frame_count,
            "frame_source": self.segment.frames.source_id,
            "body_kind": None if self.body_kind is None else self.body_kind.value,
            "duration_s": self.segment.duration_s,
        }


# --------------------------------------------------------------------------
# One leg's plan
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class LegPlan2D:
    """What one leg intends to do, end to end, before any time is assigned."""

    leg: LegId
    phased: tuple[PhasedSegment2D, ...]
    chain: SegmentChain2D
    strategy: StrategyId | None
    ascent_strategy: str | None
    descent_strategy: str | None
    unresolved: tuple[TransitionRequirement2D, ...] = ()
    refusal: str | None = None
    breaks: tuple[ChainBreak2D, ...] = ()
    #: ``source_id -> the frames that source's ``FrameRef2D`` indices point at``.
    #: Empty for a source whose frames were not kept.  Step 8 uses this to build
    #: a trajectory out of the generator's own frames instead of interpolating
    #: between segment endpoints -- the difference matters the moment anything
    #: downstream has to command a motor, because a ``RECOVERY_SWING``'s two
    #: endpoints share a theta and its retraction lives entirely in between.
    frames: dict = field(default_factory=dict)
    notes: str = ""

    @property
    def phases_are_separable(self) -> bool:
        """False for ``#5``, whose crossing is a single primitive.

        Not a defect.  Reporting a fabricated ASCENT/DESCENT split for it would
        be, which is what plan §11's "保留 ascent / descent" is guarding.
        """

        if self.strategy is StrategyId.SWING_OVER:
            return False
        return not any(p.phase is TransitionPhase.OVER for p in self.phased)

    @property
    def is_executable(self) -> bool:
        """A plan with an unresolved requirement or a broken join is not one.

        Plan §11 requirements 5 and 6, as one flag that cannot be satisfied by
        looking at ``phased`` alone.
        """

        return not self.unresolved and not self.breaks and self.refusal is None

    def segments_in(self, phase: TransitionPhase) -> tuple[PhasedSegment2D, ...]:
        return tuple(p for p in self.phased if p.phase is phase)

    @property
    def transition_segments(self) -> tuple[PhasedSegment2D, ...]:
        return tuple(p for p in self.phased if p.phase.is_transition)

    def as_dict(self) -> dict:
        return {
            "leg": self.leg.value,
            "strategy": None if self.strategy is None else self.strategy.value,
            "ascent_strategy": self.ascent_strategy,
            "descent_strategy": self.descent_strategy,
            "phases_are_separable": self.phases_are_separable,
            "segments": len(self.phased),
            "transition_segments": len(self.transition_segments),
            "nominal_segments": len(self.phased) - len(self.transition_segments),
            "sources": ",".join(self.chain.sources),
            "unresolved_transitions": len(self.unresolved),
            "chain_breaks": len(self.breaks),
            "is_executable": self.is_executable,
            "refusal": self.refusal,
            "notes": self.notes,
        }


# --------------------------------------------------------------------------
# Building a leg plan
# --------------------------------------------------------------------------


@lru_cache(maxsize=32)
def _cached_nominal_cycles(cycles: int, posture, config) -> tuple:
    """The nominal run for one configuration, generated once.

    ``run_nominal_cycles_2d`` is deterministic in its arguments and is by far
    the most expensive thing Step 4 does, and on flat ground all four legs ask
    for the identical run.  ``NominalPosture2D`` and ``RecoveryConfig2D`` are
    frozen and hashable, so the cache is keyed on the actual configuration
    rather than only on "the default" -- which is what lets a levelled posture
    be compared against the fixed-theta one without paying four times for each.
    """

    return tuple(run_nominal_cycles_2d(cycles, posture, config))


def _nominal_phased(
    cycles: int,
    *,
    phase: TransitionPhase,
    source_id: str,
    posture: NominalPosture2D | None,
    config: RecoveryConfig2D | None,
) -> tuple[list[PhasedSegment2D], tuple]:
    """``cycles`` nominal cycles, labelled ``phase`` (plan §11 requirement 3).

    Returns the segments **and the frames they reference**.  A
    ``MotionSegment2D`` deliberately holds a ``FrameRef2D`` rather than the
    frames themselves (spec 5.4), so the frames have to be kept somewhere -- and
    a consumer that has to drive hardware needs them, not the endpoints.
    """

    if cycles <= 0:
        return [], ()
    out: list[PhasedSegment2D] = []
    frames: list = []
    offset = 0
    for cycle in _cached_nominal_cycles(cycles, posture, config):
        pair = cycle_segments_2d(cycle, source_id=source_id, frame_offset=offset)
        offset += sum(s.frames.frame_count for s in pair)
        # NOT ``cycle.frames``: that drops the recovery's first frame as a
        # duplicate of the stroke's last, while ``cycle_segments_2d`` indexes
        # ``n_roll + n_rec`` frames and keeps it.  Using the de-duplicated list
        # would shift every recovery frame by one.
        frames.extend(cycle.stroke.frames)
        frames.extend(cycle.recovery.frames)
        out += [PhasedSegment2D(s, phase, "nominal") for s in pair]
    return out, tuple(frames)


def build_leg_plan_2d(
    leg: LegId,
    composed: ComposedSequence2D,
    *,
    cycles_before: int = 1,
    cycles_after: int = 1,
    continuous_nominal: bool = False,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    tolerance: ChainTolerance2D | None = None,
) -> LegPlan2D:
    """nominal run -> the Day 10--11 crossing -> nominal run, as one chain.

    A refused or blocked ``composed`` does **not** become an empty crossing that
    the nominal runs quietly close over (plan §11 requirement 5): its
    ``unresolved`` requirements and its refusal are carried onto the plan, and
    :attr:`LegPlan2D.is_executable` is False.
    """

    ascent, descent = (None, None)
    if composed.strategy is not None:
        ascent, descent = STRATEGY_HALVES[composed.strategy]

    label = composed.strategy.value if composed.strategy else "unknown"

    registry: dict = {}

    if continuous_nominal and composed.sequence is None:
        # Day 13: one run, split by phase, instead of two runs that start from
        # the same posture and therefore do not join.  Trap 25 named this as
        # "not yet chained" rather than a defect, and this is the chaining --
        # ``run_nominal_cycles_2d`` already carries beta and hip_x from one
        # cycle to the next, so asking it for the total is all it takes.
        #
        # Only where there is no crossing.  Continuing *through* a crossing
        # needs the crossing's own exit state and a common world x, which is a
        # separate gap (see LegPlan2D.notes).
        source = f"day12_step4_nominal:{leg.value}"
        run, run_frames = _nominal_phased(
            cycles_before + cycles_after,
            phase=TransitionPhase.NOMINAL_BEFORE,
            source_id=source, posture=posture, config=config,
        )
        registry[source] = run_frames
        split = 2 * cycles_before  # two segments per cycle
        phased = list(run[:split]) + [
            PhasedSegment2D(p.segment, TransitionPhase.NOMINAL_AFTER,
                            p.source_label)
            for p in run[split:]
        ]
    else:
        before_source = f"day12_step4_before:{leg.value}"
        phased, before_frames = _nominal_phased(
            cycles_before, phase=TransitionPhase.NOMINAL_BEFORE,
            source_id=before_source, posture=posture, config=config,
        )
        if before_frames:
            registry[before_source] = before_frames
        if composed.sequence is not None:
            phased += [
                PhasedSegment2D(
                    segment,
                    phase_of_kind(segment.kind, after_transition=False),
                    label,
                )
                for segment in composed.sequence.segments
            ]
        after_source = f"day12_step4_after:{leg.value}"
        after, after_frames = _nominal_phased(
            cycles_after, phase=TransitionPhase.NOMINAL_AFTER,
            source_id=after_source, posture=posture, config=config,
        )
        if after_frames:
            registry[after_source] = after_frames
        phased += after

    segments = tuple(p.segment for p in phased)
    chain = SegmentChain2D(
        leg_id=leg.value, segments=segments,
        tolerance=ChainTolerance2D() if tolerance is None else tolerance,
        unresolved=composed.unresolved,
        notes=(
            "Day 12 Step 4: Day 10-11's crossing between two nominal runs; the "
            "crossing's motion is carried through unchanged."
        ),
    )
    _, breaks = chain_boundaries_2d(segments, chain.tolerance)
    return LegPlan2D(
        leg=leg, phased=tuple(phased), chain=chain,
        strategy=composed.strategy,
        ascent_strategy=ascent, descent_strategy=descent,
        unresolved=composed.unresolved, refusal=composed.refusal,
        breaks=tuple(breaks), frames=registry,
        notes=composed.notes,
    )


# --------------------------------------------------------------------------
# What the scheduler's window rule hides, measured
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class AirborneOverrun2D:
    """An airborne run whose window is shorter than the motion already planned.

    Step 3 gives **one** swing window to a maximal airborne run and splits it
    between the segments by frame count.  That is right for a nominal recovery,
    which is what duty 0.75 sized the window against.  A crossing puts
    ``SWING_UP`` / ``SWING_DOWN`` into the same run -- and those segments are
    **not** untimed: Day 8--9 planned each with a duration, which Day 10--11
    carried through unchanged.  The window rule then silently *compresses* that
    planned motion instead of saying it does not fit.

    Plan §11 requirement 7 is exactly about not hiding that, so it is measured
    against the only real times in the whole chain: the planned durations
    themselves.  ``compression`` is how much faster than planned the crossing
    would have to be executed.
    """

    leg: LegId
    start_s: float
    end_s: float
    window_s: float
    segment_kinds: tuple[str, ...]
    #: Sum of the durations the timed segments in this run were planned with.
    planned_s: float
    #: How many segments in the run had no planned duration.  Their time is
    #: Step 3's to assign, and they are *also* sharing this window.
    untimed_segments: int

    @property
    def compression(self) -> float:
        """> 1 means the window is shorter than the planned motion alone."""

        if self.window_s <= 0.0:
            return float("inf")
        return float(self.planned_s / self.window_s)

    def as_dict(self) -> dict:
        return {
            "leg": self.leg.value,
            "start_s": self.start_s,
            "end_s": self.end_s,
            "window_s": self.window_s,
            "segment_kinds": ",".join(self.segment_kinds),
            "planned_s": self.planned_s,
            "untimed_segments": self.untimed_segments,
            "compression": self.compression,
        }


#: Below this the window carries the planned motion, give or take rounding.
OVERRUN_TOLERANCE: float = 1.0


def airborne_overruns_2d(plan: "FourLegPlan2D") -> tuple[AirborneOverrun2D, ...]:
    """Every airborne run a swing window compresses rather than fits."""

    out: list[AirborneOverrun2D] = []
    for leg, leg_plan in plan.plans.items():
        scheduled = sorted(plan.schedule.segments_of(leg),
                           key=lambda s: s.segment_index)
        run: list = []
        for item in list(scheduled) + [None]:
            if item is not None and item.mode is LegMode.AIRBORNE:
                run.append(item)
                continue
            if run:
                durations = [leg_plan.phased[s.segment_index].segment.duration_s
                             for s in run]
                planned = sum(d for d in durations if d is not None)
                window = float(run[-1].end_s - run[0].start_s)
                if planned > window * OVERRUN_TOLERANCE:
                    out.append(AirborneOverrun2D(
                        leg=leg, start_s=run[0].start_s, end_s=run[-1].end_s,
                        window_s=window,
                        segment_kinds=tuple(s.segment_kind.value for s in run),
                        planned_s=float(planned),
                        untimed_segments=sum(1 for d in durations if d is None),
                    ))
                run = []
    return tuple(out)



# --------------------------------------------------------------------------
# Four legs, scheduled
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class FourLegPlan2D:
    """The four leg plans and the one schedule they were placed on."""

    plans: dict[LegId, LegPlan2D]
    schedule: FourLegSchedule2D
    timing: GaitTiming2D

    @property
    def is_executable(self) -> bool:
        """Every leg resolved **and** the schedule free of timing conflicts.

        Deliberately conjunctive: a leg whose motion is fully generated can
        still be unschedulable against the others, and one flag that hid either
        half would be the "silently invent" failure plan §11 forbids.
        """

        return (all(p.is_executable for p in self.plans.values())
                and not self.schedule.conflicts
                and not self.airborne_overruns)

    @property
    def airborne_overruns(self) -> tuple[AirborneOverrun2D, ...]:
        return airborne_overruns_2d(self)

    @property
    def unresolved_legs(self) -> tuple[LegId, ...]:
        return tuple(l for l in LEG_ORDER
                     if l in self.plans and not self.plans[l].is_executable)

    def as_dict(self) -> dict:
        lo, hi = self.schedule.covered_interval_s
        return {
            "legs": len(self.plans),
            "cycle_period_s": self.timing.cycle_period_s,
            "stance_duty": self.timing.stance_duty,
            "segments": len(self.schedule.scheduled),
            "start_s": self.schedule.start_s,
            "end_s": self.schedule.end_s,
            "covered_from_s": lo,
            "covered_to_s": hi,
            "max_airborne_count": self.schedule.max_airborne_count,
            "one_leg_airborne_at_a_time": self.schedule.one_leg_airborne_at_a_time,
            "conflict_count": len(self.schedule.conflicts),
            "airborne_overrun_count": len(self.airborne_overruns),
            "max_compression": max(
                (o.compression for o in self.airborne_overruns), default=1.0),
            "unresolved_legs": ",".join(l.value for l in self.unresolved_legs),
            "is_executable": self.is_executable,
        }


def plan_four_legs_2d(
    plans: dict[LegId, LegPlan2D],
    timing: GaitTiming2D | None = None,
    *,
    schedule: FourLegSchedule2D | None = None,
) -> FourLegPlan2D:
    """Place the leg plans on one timeline.  Nothing here repairs a conflict.

    Terrain-transition airborne runs are longer than one swing window, so a
    crossing is *expected* to produce conflicts (plan §11 requirement 7).  They
    are reported; choosing between a longer cycle, a leg spanning two windows,
    or a different duty is a later decision, not one to make silently here.

    ``schedule`` lets the caller supply a timeline built some other way.  The
    default -- one stance window per contact run, one swing window per airborne
    run -- is right exactly while the four legs have the **same** chain, which
    is true on flat ground and false the moment world registration gives one
    leg two more cycles than another to reach the obstacle.  Rather than decide
    here which is which, the caller that knows says so (log 1.15).
    """

    timing = walk_timing_2d() if timing is None else timing
    if schedule is None:
        schedule = schedule_chains_2d({l: p.chain for l, p in plans.items()},
                                      timing)
    return FourLegPlan2D(plans=dict(plans), schedule=schedule, timing=timing)


# --------------------------------------------------------------------------
# Plan §11 requirement 8: the debug table
# --------------------------------------------------------------------------


def debug_rows(plan: FourLegPlan2D) -> list[dict]:
    """``time interval | leg | segment kind | contact/airborne | body requirement``.

    One row per scheduled segment, plus the plan-level and conflict rows, so a
    reader of the file alone can see the unresolved and conflicting parts
    without having to already know they exist.
    """

    rows: list[dict] = [{"row_kind": "plan", **plan.as_dict()}]
    for leg in LEG_ORDER:
        if leg in plan.plans:
            rows.append({"row_kind": "leg", **plan.plans[leg].as_dict()})

    phase_by_index: dict[tuple[str, int], PhasedSegment2D] = {}
    for leg, leg_plan in plan.plans.items():
        for i, phased in enumerate(leg_plan.phased):
            phase_by_index[(leg.value, i)] = phased

    for scheduled in plan.schedule.scheduled:
        phased = phase_by_index.get((scheduled.leg.value, scheduled.segment_index))
        rows.append({
            "row_kind": "segment",
            "start_s": scheduled.start_s,
            "end_s": scheduled.end_s,
            "duration_s": scheduled.duration_s,
            "duration_is_assigned": scheduled.duration_is_assigned,
            "leg": scheduled.leg.value,
            "segment_index": scheduled.segment_index,
            "segment_kind": scheduled.segment_kind.value,
            "contact_state": scheduled.mode.value,
            "phase": None if phased is None else phased.phase.value,
            "source_label": None if phased is None else phased.source_label,
            "body_kind": (
                None if phased is None or phased.body_kind is None
                else phased.body_kind.value
            ),
            "frame_count": scheduled.frame_count,
        })

    for leg, leg_plan in plan.plans.items():
        for requirement in leg_plan.unresolved:
            rows.append({
                "row_kind": "unresolved_transition",
                "leg": leg.value,
                **{k: v for k, v in requirement.as_dict().items()},
            })
        for chain_break in leg_plan.breaks:
            rows.append({
                "row_kind": "chain_break", "leg": leg.value,
                **chain_break.as_dict(),
            })
    for conflict in plan.schedule.conflicts:
        rows.append({"row_kind": "conflict", **conflict.as_dict()})
    for overrun in plan.airborne_overruns:
        rows.append({"row_kind": "airborne_overrun", **overrun.as_dict()})

    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]


# --------------------------------------------------------------------------
# Day 13: holding still, so that one leg can wait for another
# --------------------------------------------------------------------------


def _hold_segment_2d(after: MotionSegment2D, duration_s: float,
                     phase_label: str) -> MotionSegment2D:
    """A ``BODY_HOLD`` that keeps exactly the contact ``after`` ended on.

    Both endpoints are that same contact, so the segment moves nothing and
    :func:`day12_body_trajectory_2d._hip_x_at` reports a constant hip through
    it.  Its ``FrameRef2D`` points at the frame it is holding -- the last one
    of ``after`` -- rather than at a frame invented for the purpose.
    """

    end = after.end_contact
    if not isinstance(end, PointContact2D):
        raise TypeError(
            "a hold can only follow a segment that ends on one definite "
            f"contact point; {after.kind.value} does not."
        )
    return MotionSegment2D(
        kind=SegmentKind.BODY_HOLD,
        start_contact=end,
        end_contact=end,
        sampling=None,
        body_requirement=after.body_requirement,
        frames=FrameRef2D(source_id=after.frames.source_id,
                          indices=(after.frames.indices[-1],)),
        duration_s=float(duration_s),
        phase_label=phase_label,
    )


def insert_holds_2d(plan: FourLegPlan2D,
                    waits: Sequence[dict]) -> FourLegPlan2D:
    """Turn scheduled waits into real ``BODY_HOLD`` segments.

    A wait left as a *gap* in the schedule is what produced the body-velocity
    jumps of log 21: ``body_trajectory_2d`` drops a leg with no active segment,
    the body x is re-solved from the legs that remain, and it steps.  A hold
    keeps the leg in that set at a fixed hip, so nothing is re-solved.

    Both halves are updated together: the segment goes into the leg's
    ``phased`` **and** onto the schedule, because ``segment_index`` indexes
    ``phased`` and ten call sites dereference it.  A schedule-only hold would
    break every one of them.
    """

    if not waits:
        return plan

    by_leg: dict[LegId, list[dict]] = {}
    for wait in waits:
        leg = next(l for l in LEG_ORDER if l.value == wait["leg"])
        by_leg.setdefault(leg, []).append(wait)

    plans = dict(plan.plans)
    scheduled = list(plan.schedule.scheduled)

    for leg, leg_waits in by_leg.items():
        leg_plan = plans[leg]
        phased = list(leg_plan.phased)
        own = sorted([s for s in scheduled if s.leg is leg],
                     key=lambda s: s.start_s)
        others = [s for s in scheduled if s.leg is not leg]

        for wait in sorted(leg_waits, key=lambda w: -float(w["from_s"])):
            at = float(wait["from_s"])
            before = [s for s in own if s.end_s <= at + 1e-9]
            if not before:
                raise ValueError(
                    f"{leg.value} has no segment before {at:.6f} s to hold "
                    "the contact of; a hold needs a contact to keep."
                )
            anchor = before[-1]
            held = _hold_segment_2d(
                phased[anchor.segment_index].segment,
                duration_s=float(wait["delay_s"]),
                phase_label=f"HOLD_FOR_{wait.get('reason', 'SWING_CLEARANCE')}",
            )
            index = anchor.segment_index + 1
            phased.insert(index, PhasedSegment2D(
                segment=held,
                phase=phased[anchor.segment_index].phase,
                source_label="day13_hold",
            ))
            # Every later segment of this leg shifts one index along.
            own = [
                replace(s, segment_index=s.segment_index + 1,
                        window_index=s.window_index + 1)
                if s.segment_index >= index else s
                for s in own
            ]
            own.append(ScheduledSegment2D(
                leg=leg, window_index=index, segment_index=index,
                segment_kind=SegmentKind.BODY_HOLD,
                phase_label=held.phase_label, mode=LegMode.STANCE,
                start_s=at, end_s=at + float(wait["delay_s"]),
                frame_count=1, duration_is_assigned=True,
            ))
            own.sort(key=lambda s: s.start_s)

        plans[leg] = replace(leg_plan, phased=tuple(phased))
        scheduled = others + own

    return FourLegPlan2D(
        plans=plans,
        schedule=FourLegSchedule2D(timing=plan.schedule.timing,
                                   scheduled=tuple(scheduled)),
        timing=plan.timing,
    )
