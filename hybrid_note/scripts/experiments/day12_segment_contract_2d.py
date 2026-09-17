"""Day 12 Step 0: the segment contract a four-leg timeline can actually chain.

Step 0's job is to freeze semantics, not to build motion.  Two things had to be
settled before any of Day 12's later steps can be written down honestly.

**1. What the nominal flat-ground motion is called.**  Day 10--11's
``SegmentKind.WHEEL_ROLL`` is documented as propulsion at ``theta = 17 deg``,
and the 2026-08-31 Hybrid-gait freeze says the nominal flat run is *not* wheel
mode: the leg stays expanded and the foot rim rolls.  Day 6--7 already
generates exactly that motion -- its ``APPROACH`` stage rolls the foot rim
along flat ground at ``theta = 60 deg`` -- but ``APPROACH`` is named for the
obstacle it is approaching, and nominal locomotion has no obstacle.  So
``FOOT_RIM_ROLL`` joins the schema beside both, with each of the three keeping
its own claim.  That change lives in ``day10_11_motion_schema_2d`` because an
``Enum`` that already has members cannot be extended from outside.

**2. What a chain of segments is, once the segments come from different runs.**
This is the one place the Day 10--11 schema genuinely cannot go.
:class:`MotionSequence2D` requires *every* segment to reference one
``frames.source_id`` and requires frame indices to be unique across the whole
sequence -- both are real guarantees for a sequence cut out of a single
traversal, and Step 7 relies on them.  A Day 12 leg trajectory is not that: it
is a flat roll (a Day 12 generator), then a Day 6--7 traversal, then possibly a
Day 8--9 swing, each numbering its frames from zero.  Relaxing the sequence's
invariant would remove a guarantee Day 10--11 earned; so Day 12 gets its own
container, and the invariant that still applies -- indices unique *within* a
source -- is kept.

**What "chainable" means here, and the distinction the data forced.**
The first version of this module held every boundary to one tolerance, and Day
6--7's own traversal failed it -- which turned out to be the module being
wrong, not the traversal.  A boundary is one of two things, and they are not
the same question:

``BoundaryKind.CUT``
    consecutive frames of one continuously validated run, which the Day 10--11
    builder cut at a ``(stage, phase)`` change.  The "jump" is one rolling step
    of **real motion** -- measured across Step 10R's ten segments: up to 1.75
    deg of beta and 6.6 mm of hip.  Continuity is not in question; what is
    worth asking is whether the cut skipped motion, so the joints are checked
    against the segment's own sampling step.

``BoundaryKind.HANDOVER``
    two independently generated motions meeting, which is new in Day 12 and
    which nothing has ever checked.  Here the leg really could teleport, so the
    joints and the hip are held to fixed bounds.

The **contact point** is reported at every boundary and bounded at none of them
by default: Day 10--11 trap 32 measured 180.5 mm of contact-point travel across
the ``+-180 deg`` rim seam while the joints moved about a degree, because that
seam is a change of chart rather than a motion.  A checker that failed on
contact jumps would reject the hand-over Day 6--7's traversal is built around.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum

import numpy as np

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    MotionSegment2D,
    PointContact2D,
    SegmentKind,
    TransitionRequirement2D,
)
from hybrid_note.scripts.experiments.day10_11_sequence_builders_2d import (
    HandoffReport2D,
    handoff_between_2d,
)

__all__ = [
    "BoundaryKind",
    "boundary_kind_2d",
    "boundary_rows_2d",
    "ChainTolerance2D",
    "ChainBreak2D",
    "SegmentChain2D",
    "entry_state_2d",
    "exit_state_2d",
    "chain_boundaries_2d",
    "segment_semantics_rows",
    "SEGMENT_SEMANTICS",
]


# --------------------------------------------------------------------------
# The chainable common state
# --------------------------------------------------------------------------


def entry_state_2d(segment: MotionSegment2D) -> PointContact2D:
    """The state a segment must be handed to start.

    Spec 5.4 typed the endpoints as ``ContactState | RollingContact``, and the
    schema keeps that union because the spec allows it.  **Nothing produces the
    second reading**: both Day 10--11 builders write a
    :class:`PointContact2D` at each end, because a hand-over check compares
    joint values and a moving contact has none.  Day 12 promotes that from "how
    it happens to be" to the contract, so a later producer that emits the other
    form fails here instead of at the point where a leg is scheduled.
    """

    return _definite(segment.start_contact, segment, "start")


def exit_state_2d(segment: MotionSegment2D) -> PointContact2D:
    """The state a segment hands to the next one.  See :func:`entry_state_2d`."""

    return _definite(segment.end_contact, segment, "end")


def _definite(contact, segment: MotionSegment2D, which: str) -> PointContact2D:
    if not isinstance(contact, PointContact2D):
        raise TypeError(
            f"{segment.kind.value} has a non-definite {which} state "
            f"({type(contact).__name__}); a chainable endpoint is a "
            "PointContact2D, because the next segment is entered from one "
            "pose, not from a range of them."
        )
    return contact


# --------------------------------------------------------------------------
# What counts as chained
# --------------------------------------------------------------------------


class BoundaryKind(str, Enum):
    """The two ways two segments can meet.  They are **not** the same check.

    Measured, not assumed.  Day 6--7's traversal is one continuously validated
    run that the Day 10--11 builder *cuts* at ``(stage, phase)`` changes, so
    consecutive segments hold consecutive frames and the "jump" between them is
    one rolling step of real motion -- up to 1.75 deg of beta and 6.6 mm of
    hip, measured across the ten segments of Step 10R.  A Day 12 chain also has
    boundaries of a completely different sort: a flat run generated here meets
    a traversal generated there, and nothing has ever checked that the second
    starts where the first stopped.

    Holding both to one tolerance gets one of them wrong in each direction: a
    tolerance loose enough for a cut would let a hand-over teleport, and a
    tolerance tight enough for a hand-over would reject the traversal's own
    boundaries.  This is the same trap the Day 10--11 log names repeatedly --
    a quantity designed for one situation applied to another.
    """

    #: Consecutive frames of one source run.  Continuity was established when
    #: the run was generated; what is worth checking here is that the cut did
    #: not silently skip motion.
    CUT = "cut"
    #: Two independently generated motions meeting.  Continuity is an open
    #: question and this is where it gets answered.
    HANDOVER = "handover"


def boundary_kind_2d(before: MotionSegment2D, after: MotionSegment2D) -> BoundaryKind:
    """A cut is same-source and forward in frame index; everything else is a
    hand-over.  A backwards or repeated index within one source is not a cut
    even though the source matches -- the chain would be replaying frames."""

    same_source = before.frames.source_id == after.frames.source_id
    forward = after.frames.indices[0] > before.frames.indices[-1]
    return BoundaryKind.CUT if (same_source and forward) else BoundaryKind.HANDOVER


def _frame_gap(before: MotionSegment2D, after: MotionSegment2D) -> int:
    return int(after.frames.indices[0] - before.frames.indices[-1])


@dataclass(frozen=True)
class ChainTolerance2D:
    """How much a boundary may move and still be one motion.

    Every default is a measurement, not a preference.

    ``handover_joint_jump_rad`` (2 deg)
        The largest joint motion Day 6--7's traversal makes in a single step is
        ``beta`` 1.75 deg, so 2 deg admits a hand-over that is off by one step
        of the generating grid.  It refuses the 29 deg jump Step 2b measured
        across the ``alpha = -40 deg`` seam -- the jump that refuted strategy
        #3 -- which is the discrimination this number has to make.

    ``handover_hip_jump_m`` (10 mm)
        Same calibration on the body side: the largest hip motion the traversal
        makes in one step is 6.6 mm (the ``APPROACH`` cut).  Ten admits that
        and refuses a body teleport.

    ``cut_step_slack``
        A cut's joint jump is compared against the segment's **own** sampling
        step times the frame gap.  The slack is there because
        ``RollSampling2D.beta_step_rad`` is the *nominal* increment and a stage
        that stops on an event truncates its last one, so the observed jump can
        sit anywhere up to the nominal.  1.5 makes this a "same order as one
        step" test, which is what it can honestly be.

    ``max_contact_jump_m``
        ``None``, and that is the point: Day 10--11 trap 32 measured 180.5 mm
        of contact-point travel across the ``+-180 deg`` rim seam while the
        joints moved about a degree.  That seam is a change of chart, not a
        motion.  The value is always reported; bounding it is opt-in.

    **A cut's hip jump is deliberately not checked.**  With the foot on a
    surface, ``hip_z`` and ``theta`` determine each other -- that is Day 10--11
    trap 43, the measurement that promoted ``PINNED`` from a special case to
    every swing endpoint.  So a cut whose joints moved at most one step has a
    hip that moved at most one step's worth, and a separate hip bound would add
    no information while introducing a constant nobody measured.
    """

    handover_joint_jump_rad: float = float(np.deg2rad(2.0))
    handover_hip_jump_m: float = 10.0e-3
    cut_step_slack: float = 1.5
    max_contact_jump_m: float | None = None

    def __post_init__(self) -> None:
        for name in ("handover_joint_jump_rad", "handover_hip_jump_m",
                     "cut_step_slack"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")
            object.__setattr__(self, name, value)
        if self.max_contact_jump_m is not None:
            value = float(self.max_contact_jump_m)
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError("max_contact_jump_m must be positive when given.")
            object.__setattr__(self, "max_contact_jump_m", value)

    def cut_budget_rad(
        self, before: MotionSegment2D, after: MotionSegment2D, coordinate: str
    ) -> float:
        """How far ``coordinate`` may move across a cut of this frame gap.

        The step of **either** neighbour is allowed, because the motion across
        a cut belongs to neither segment exclusively: Day 6--7's
        ``RIGHT_RIM_TOP -> RETRACT_TO_WHEEL`` boundary moves theta by one
        degree, and only the second of the two records a theta step at all.
        """

        gap = max(1, _frame_gap(before, after))
        steps = []
        for segment in (before, after):
            sampling = segment.sampling
            step = getattr(sampling, f"{coordinate}_step_rad", None)
            if step is not None:
                steps.append(abs(float(step)))
        if not steps:
            return 0.0
        return float(max(steps) * gap * self.cut_step_slack)


@dataclass(frozen=True)
class ChainBreak2D:
    """One boundary that is not a continuation, with the number that says so."""

    index: int
    boundary: BoundaryKind
    report: HandoffReport2D
    reason: str
    value: float
    limit: float

    def as_dict(self) -> dict:
        return {
            "boundary_index": self.index,
            "boundary_kind": self.boundary.value,
            "reason": self.reason,
            "value": self.value,
            "limit": self.limit,
            **self.report.as_dict(),
        }


def chain_boundaries_2d(
    segments: Sequence[MotionSegment2D],
    tolerance: ChainTolerance2D | None = None,
) -> tuple[list[HandoffReport2D], list[ChainBreak2D]]:
    """Measure every boundary, and name the ones that are not continuations.

    Returns the reports for all boundaries and a break record for each check
    that failed, so a caller can see what a boundary did as well as whether it
    passed.  Nothing is repaired here: a chain with breaks stays a chain with
    breaks, and the scheduler decides what that means.
    """

    tol = ChainTolerance2D() if tolerance is None else tolerance
    reports: list[HandoffReport2D] = []
    breaks: list[ChainBreak2D] = []
    for i, (before, after) in enumerate(zip(segments, segments[1:])):
        entry_state_2d(after)
        exit_state_2d(before)
        report = handoff_between_2d(before, after)
        reports.append(report)
        kind = boundary_kind_2d(before, after)

        checks: list[tuple[str, float, float]] = []
        if kind is BoundaryKind.HANDOVER:
            checks += [
                ("theta_jump", abs(report.theta_jump_rad),
                 tol.handover_joint_jump_rad),
                ("beta_jump", abs(report.beta_jump_rad),
                 tol.handover_joint_jump_rad),
                ("hip_jump", abs(report.hip_jump_m), tol.handover_hip_jump_m),
            ]
        else:
            # A cut is checked against its own sampling: the question is
            # whether the cut skipped motion, not whether the leg teleported.
            # See ChainTolerance2D on why the hip is left out.
            for coordinate, value in (
                ("theta", abs(report.theta_jump_rad)),
                ("beta", abs(report.beta_jump_rad)),
            ):
                budget = tol.cut_budget_rad(before, after, coordinate)
                if budget > 0.0 or value > 0.0:
                    checks.append((f"{coordinate}_step_overrun", value, budget))
        if tol.max_contact_jump_m is not None:
            checks.append(
                ("contact_jump", abs(report.contact_jump_m), tol.max_contact_jump_m)
            )
        for reason, value, limit in checks:
            if value > limit:
                breaks.append(ChainBreak2D(i, kind, report, reason, value, limit))
    return reports, breaks


def boundary_rows_2d(
    segments: Sequence[MotionSegment2D],
    tolerance: ChainTolerance2D | None = None,
) -> list[dict]:
    """Every boundary as a row, cuts and hand-overs labelled apart."""

    reports, breaks = chain_boundaries_2d(segments, tolerance)
    broken = {b.index for b in breaks}
    return [
        {
            "boundary_index": i,
            "boundary_kind": boundary_kind_2d(before, after).value,
            "frame_gap": _frame_gap(before, after),
            "source_before": before.frames.source_id,
            "source_after": after.frames.source_id,
            "continuous": i not in broken,
            **report.as_dict(),
        }
        for i, (report, (before, after)) in enumerate(
            zip(reports, zip(segments, segments[1:]))
        )
    ]


# --------------------------------------------------------------------------
# The chain
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SegmentChain2D:
    """One leg's ordered motions, possibly drawn from several frame sources.

    This is :class:`MotionSequence2D` minus the single-source rule and plus the
    continuity check -- **not** a replacement for it.  A sequence cut out of
    one traversal should still be a ``MotionSequence2D``; a chain is what Day
    12 assembles *from* such sequences plus its own flat runs.

    ``unresolved`` is carried through unchanged from Day 10--11 for the same
    reason it exists there: a chain that still needs a ``TOP_REPOSITION``
    nobody has generated is not an executable plan, and
    :attr:`is_complete` is what refuses to let a reader of ``segments`` alone
    conclude otherwise.
    """

    leg_id: str
    segments: tuple[MotionSegment2D, ...]
    tolerance: ChainTolerance2D = ChainTolerance2D()
    unresolved: tuple[TransitionRequirement2D, ...] = ()
    notes: str = ""

    def __post_init__(self) -> None:
        if not str(self.leg_id).strip():
            raise ValueError("leg_id must not be empty.")
        segments = tuple(self.segments)
        if not segments:
            raise ValueError("a chain has at least one segment.")
        # Indices stay unique *within* a source.  Across sources they collide
        # by construction -- every generator numbers its own frames from zero
        # -- and that collision is meaningless, not a conflict.
        seen: dict[str, set[int]] = defaultdict(set)
        for segment in segments:
            ref = segment.frames
            overlap = seen[ref.source_id] & set(ref.indices)
            if overlap:
                raise ValueError(
                    f"segments of leg {self.leg_id!r} overlap on frames "
                    f"{sorted(overlap)[:5]} of source {ref.source_id!r}: within "
                    "one source a frame belongs to exactly one segment."
                )
            seen[ref.source_id] |= set(ref.indices)
        object.__setattr__(self, "segments", segments)
        object.__setattr__(self, "unresolved", tuple(self.unresolved))

    # -- what it is made of --------------------------------------------------

    @property
    def sources(self) -> tuple[str, ...]:
        """Every frame source this chain draws on, in first-use order."""

        out: list[str] = []
        for segment in self.segments:
            if segment.frames.source_id not in out:
                out.append(segment.frames.source_id)
        return tuple(out)

    @property
    def entry_state(self) -> PointContact2D:
        return entry_state_2d(self.segments[0])

    @property
    def exit_state(self) -> PointContact2D:
        return exit_state_2d(self.segments[-1])

    # -- whether it holds together -------------------------------------------

    @property
    def handoffs(self) -> tuple[HandoffReport2D, ...]:
        return tuple(chain_boundaries_2d(self.segments, self.tolerance)[0])

    @property
    def breaks(self) -> tuple[ChainBreak2D, ...]:
        return tuple(chain_boundaries_2d(self.segments, self.tolerance)[1])

    @property
    def is_chained(self) -> bool:
        """Every boundary is a continuation.  Says nothing about completeness."""

        return not self.breaks

    @property
    def is_complete(self) -> bool:
        """Chained **and** carrying no unresolved transition.

        Kept apart from :attr:`is_chained` on purpose: Day 12's whole reason
        for existing is that a chain can be perfectly continuous and still have
        a hole in it where a ``TOP_REPOSITION`` has to go.
        """

        return self.is_chained and not self.unresolved

    # -- what it does with time ----------------------------------------------

    @property
    def total_duration_s(self) -> float | None:
        """``None`` if any segment has no time -- a partial sum would mislead.

        Day 6--7's rolling traversal is quasi-static and every rolling segment
        it produced carries ``duration_s = None`` (trap 33).  Day 12 Step 3 has
        to supply a timeline, and that is a **new modelling decision**; until
        it does, this stays ``None`` rather than summing the swings alone.
        """

        if any(s.duration_s is None for s in self.segments):
            return None
        return float(sum(s.duration_s for s in self.segments))

    @property
    def untimed_segments(self) -> tuple[int, ...]:
        """Which segments Step 3 will have to assign a duration to."""

        return tuple(i for i, s in enumerate(self.segments) if s.duration_s is None)

    # -- reporting ------------------------------------------------------------

    def rows(self) -> list[dict]:
        """Segments, boundaries and holes in one table, kept apart by ``row_kind``."""

        rows = [
            {"leg_id": self.leg_id, "row_kind": "segment", "chain_index": i,
             "source_id": s.frames.source_id, **s.as_dict()}
            for i, s in enumerate(self.segments)
        ]
        rows += [
            {"leg_id": self.leg_id, "row_kind": "handoff", "chain_index": i,
             **h.as_dict()}
            for i, h in enumerate(self.handoffs)
        ]
        rows += [
            {"leg_id": self.leg_id, "row_kind": "chain_break", "chain_index": b.index,
             **b.as_dict()}
            for b in self.breaks
        ]
        rows += [
            {"leg_id": self.leg_id, "row_kind": "unresolved_transition",
             "chain_index": len(self.segments) + i, **t.as_dict()}
            for i, t in enumerate(self.unresolved)
        ]
        return rows


# --------------------------------------------------------------------------
# The freeze, as a table
# --------------------------------------------------------------------------

#: What each segment kind claims, so Step 0's freeze can be read as data rather
#: than as prose.  The columns that matter for Day 12: exactly one kind pins
#: theta to wheel mode and it is not nominal, and the nominal cycle is two
#: kinds -- one rolling, one airborne.
SEGMENT_SEMANTICS: tuple[tuple[SegmentKind, str], ...] = (
    (SegmentKind.FOOT_RIM_ROLL,
     "Nominal cycle, stance half: expanded posture, foot rim rolling along the "
     "usable arc.  A finite stroke, not an indefinite one."),
    (SegmentKind.RECOVERY_SWING,
     "Nominal cycle, airborne half: liftoff, retract to a compact posture, "
     "carry on in the same forward rotation sense, extend into the next "
     "touchdown.  Not a terrain-transition swing."),
    (SegmentKind.TOP_REPOSITION_SWING,
     "Day 12 Step 7: the airborne relocation on top of the obstacle that Day "
     "10-11 recorded as an unresolved TOP_REPOSITION.  A terrain-transition "
     "swing, generated by the same Day 8-9 planner as every other swing."),
    (SegmentKind.WHEEL_ROLL,
     "True wheel mode at theta = 17 deg.  Only inside a transition that needs "
     "it; never the nominal flat run."),
    (SegmentKind.APPROACH,
     "The run-up to a known transition.  Same motion as FOOT_RIM_ROLL, but its "
     "name asserts an obstacle ahead."),
    (SegmentKind.ROLL_UP, "Rolling onto the leading edge."),
    (SegmentKind.WHEEL_TRANSITION, "Retract to wheel mode on the top."),
    (SegmentKind.ROLL_DOWN, "Rolling off the trailing edge."),
    (SegmentKind.POST_TOUCHDOWN_ROLL,
     "Rolling on whichever rim a swing just landed on, bounded by the arc that "
     "remains."),
    (SegmentKind.SWING_UP, "Airborne onto the top."),
    (SegmentKind.SWING_DOWN, "Airborne off the top."),
    (SegmentKind.SWING_OVER, "Airborne clear over the whole obstacle."),
    (SegmentKind.BODY_HOLD,
     "The whole machine stands still: every leg keeps the contact it has and "
     "the body does not advance.  Costs time and no rim arc.  Neither nominal "
     "locomotion nor a terrain transition -- the gait does not pause with "
     "nothing in the way, and the terrain did not force it."),
)


def segment_semantics_rows() -> list[dict]:
    return [
        {
            "kind": kind.value,
            "family": "swing" if kind.is_swing else "rolling",
            "pins_theta_to_wheel_mode": kind.pins_theta,
            "is_nominal_locomotion": kind.is_nominal_locomotion,
            "is_terrain_transition": kind.is_terrain_transition,
            "meaning": meaning,
        }
        for kind, meaning in SEGMENT_SEMANTICS
    ]
