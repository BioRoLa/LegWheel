"""Day 10--11 Step 9: the body-requirement timeline handed to Day 12.

Step 7 produced sequences; Step 9 turns them into the one thing the quadruped
stage actually consumes: **what each sequence asks of the body, along the
path**, with the hard requirements separated from the preferences and with the
transitions nobody has solved kept visible as their own rows.

Three decisions shape this module, and each of them is a decision rather than
a detail.

``x`` is the independent variable
    Day 6--7's traversal is quasi-static: every rolling frame has
    ``duration_s = None`` because the traversal was never assigned time (spec
    Step 9, implementation-log trap 33).  Choosing ``t`` would mean inventing
    a rolling timing law here -- a **new modelling decision**, not a
    measurement -- and it would land in exactly the slot a real number has to
    occupy later.  So the timeline is parameterised by hip ``x``, which every
    segment already has, and the times that *do* exist (the swings carry one
    from Day 8--9) are carried through unchanged rather than being thrown away
    or extended over the rolling segments.  :func:`x_is_monotonic_2d` is what
    makes this legitimate, and it is checked rather than assumed.

The segment-level bound is an envelope, not a timeline
    ``MotionSegment2D.body_requirement`` carries **one** ``hip_z_min_m`` for a
    whole swing -- the maximum of the hip trajectory (Step 6's builder).  Over
    an ascent that rises 80 mm, quoting that scalar at the lift-off end
    over-constrains the body by the full 80 mm.  This module therefore emits a
    requirement **per knot**, and reports ``envelope_excess_mm`` so the size of
    what the scalar was hiding is visible rather than merely fixed.

A swing's endpoints are ``PINNED``, its interior is a ``LOWER_BOUND``
    Step 1 named ``PINNED`` for Step 2b's landing, where a fully specified
    touchdown contact state leaves the hip no freedom.  The same argument
    applies to **every** swing endpoint here: at lift-off and at touchdown the
    foot is on a surface, so the hip height and the joint angles are in
    one-to-one correspondence (trap 16 -- touchdown ``theta`` is the IK's
    *output*; move the hip and the plan stays valid but lands in a different
    pose).  A reader who treated a whole swing as a preference would raise the
    hip at touchdown and silently change the landing the sweeps priced.  So
    the endpoints are hard and only the interior is a preference.

Nothing here runs a planner.  It reads composed sequences and writes a table.
"""

from __future__ import annotations

import csv
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np

from hybrid_note.scripts.experiments.day10_11_concession_2d import (
    BodyRequirementKind,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    BLOCKED_PAIRS,
    StrategyId,
    Verdict,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    MotionSegment2D,
    SegmentKind,
    TransitionRequirement2D,
)


#: What the timeline is parameterised by.  See the module docstring: the
#: alternative would have required inventing rolling timings.
TIME_BASIS = "x_progress"

#: Said in full, once, in the file itself -- the completion criterion asks for
#: the rolling time source to be written in the delivered file, not only here.
TIME_BASIS_STATEMENT = (
    "The independent variable is hip x [mm], not time.  Day 6-7's rolling "
    "traversal is quasi-static and carries no duration (every rolling segment "
    "has duration_s = None), so any rolling timing would be a NEW MODELLING "
    "DECISION rather than a measurement.  Times that do exist are carried "
    "through unchanged: each swing segment has the duration Day 8-9 planned "
    "it with, in segment_duration_s, and its per-sample time in knot_time_s.  "
    "Rolling rows leave both blank -- blank means 'never assigned', not zero."
)

#: A tolerance for "the same x".  Two knots at a segment boundary share an x;
#: floating point makes the difference ~1e-13 m.
X_TOLERANCE_M = 1e-9


class ConstraintClass(str, Enum):
    """Whether violating a row breaks the sequence or merely wastes effort."""

    #: Violate it and the geometry the sequence was checked against no longer
    #: holds: the contact moves, or the landing becomes a different landing.
    HARD = "HARD"
    #: The body may exceed it freely.  Meeting it exactly is simply the
    #: cheapest way to satisfy the segment.
    PREFERENCE = "PREFERENCE"
    #: There is nothing to satisfy yet -- the motion has not been generated.
    UNRESOLVED = "UNRESOLVED"


class ContactPhase(str, Enum):
    """Whether this leg is carrying at this knot.  Day 12's duty needs it."""

    STANCE = "STANCE"
    FLIGHT = "FLIGHT"


class RequirementBasis(str, Enum):
    """Where a knot's number comes from, so its strength is not overstated."""

    #: A rolling frame: the hip height is an output of the contact geometry.
    #: There is no freedom at all, and the value is exact.
    EXACT_TRACK = "EXACT_TRACK"
    #: A swing endpoint: the foot is on a surface, so hip height and joint
    #: angles determine each other.
    PINNED_CONTACT = "PINNED_CONTACT"
    #: A swing interior: the planned trajectory is a **witness** that this
    #: height works.  It is sufficient, and it was not shown to be necessary
    #: -- only the segment-level minimum (Step 2 / Step 3's grid search) was.
    WITNESS_LOWER_BOUND = "WITNESS_LOWER_BOUND"


#: What a reader loses by violating each class.  Written into the file so the
#: distinction survives without this module.
VIOLATION_EFFECT = {
    ConstraintClass.HARD: (
        "the contact geometry the sequence was checked against no longer "
        "holds; the sequence is invalid."
    ),
    ConstraintClass.PREFERENCE: (
        "nothing breaks; the body is simply higher than this segment needed."
    ),
    ConstraintClass.UNRESOLVED: (
        "nothing to satisfy: the motion does not exist yet."
    ),
}

_CONSTRAINT_OF_KIND = {
    BodyRequirementKind.TRACK: ConstraintClass.HARD,
    BodyRequirementKind.PINNED: ConstraintClass.HARD,
    BodyRequirementKind.LOWER_BOUND: ConstraintClass.PREFERENCE,
    BodyRequirementKind.NONE: ConstraintClass.PREFERENCE,
}


def constraint_class_of(kind: BodyRequirementKind) -> ConstraintClass:
    """Step 9's completion criterion, as a function rather than as prose."""

    return _CONSTRAINT_OF_KIND[BodyRequirementKind(kind)]


#: Whether the hip may go **higher** than a row asks is not a Day 10--11
#: measurement: raising it lengthens the leg's reach demand, and no sweep here
#: looked for the height at which the IK stops converging.  Recorded as
#: ``NOT_MEASURED`` rather than left blank, which would read as "unbounded".
UPPER_BOUND_STATUS = "NOT_MEASURED"


# --------------------------------------------------------------------------
# Knots
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class BodyKnot2D:
    """One sample of the body requirement, at one hip ``x``."""

    segment_index: int
    segment_kind: SegmentKind
    frame_index: int
    x_m: float
    #: What the plan's hip did here.  Its *meaning* is set by ``basis``.
    hip_z_m: float
    body_kind: BodyRequirementKind
    basis: RequirementBasis
    contact_phase: ContactPhase
    contact_xz_m: tuple[float, float] | None
    rim: str
    alpha_rad: float | None
    theta_rad: float | None
    beta_rad: float | None
    #: The source's own time, where it had one.  ``None`` on every rolling
    #: knot: Day 6--7 never assigned time.
    time_s: float | None
    #: The scalar the segment carries minus this knot's requirement -- how
    #: much the Step 6/7 envelope over-constrains the body right here.
    envelope_excess_m: float

    @property
    def constraint_class(self) -> ConstraintClass:
        return constraint_class_of(self.body_kind)

    def as_dict(self) -> dict:
        def deg(value):
            return None if value is None else float(np.rad2deg(value))

        return {
            "segment_index": self.segment_index,
            "segment_kind": self.segment_kind.value,
            "frame_index": self.frame_index,
            "x_mm": self.x_m * 1e3,
            "hip_z_required_mm": self.hip_z_m * 1e3,
            "body_kind": self.body_kind.value,
            "constraint_class": self.constraint_class.value,
            "requirement_basis": self.basis.value,
            "hip_z_upper_bound_mm": UPPER_BOUND_STATUS,
            "envelope_excess_mm": self.envelope_excess_m * 1e3,
            "contact_phase": self.contact_phase.value,
            "contact_x_mm": (
                None if self.contact_xz_m is None else self.contact_xz_m[0] * 1e3
            ),
            "contact_z_mm": (
                None if self.contact_xz_m is None else self.contact_xz_m[1] * 1e3
            ),
            "rim": self.rim,
            "alpha_deg": deg(self.alpha_rad),
            "theta_deg": deg(self.theta_rad),
            "beta_deg": deg(self.beta_rad),
            "knot_time_s": self.time_s,
        }


@dataclass(frozen=True)
class TimelineSegment2D:
    """A segment as the timeline sees it: a span of ``x`` and a class."""

    index: int
    kind: SegmentKind
    phase_label: str
    x_range_m: tuple[float, float]
    body_kind: BodyRequirementKind
    #: The scalar Step 6/7 carried.  Kept because it **is** the number Step 5
    #: made its decision on (Step 2 / Step 3's grid minimum) -- it is a correct
    #: envelope and a misleading timeline, so both are written.  ``None`` on a
    #: rolling segment, which never had one: a ``TRACK`` requirement is the
    #: profile itself.
    envelope_hip_z_m: float | None
    hip_z_low_m: float
    hip_z_high_m: float
    frame_count: int
    duration_s: float | None
    contact_phase: ContactPhase
    #: Set on a swing: its two endpoints are hard even though its interior is
    #: not.  ``None`` on a rolling segment, where every knot is already hard.
    endpoint_body_kind: BodyRequirementKind | None

    @property
    def constraint_class(self) -> ConstraintClass:
        return constraint_class_of(self.body_kind)

    @property
    def max_envelope_excess_m(self) -> float:
        """Zero where there is no envelope -- not the profile's own range."""

        if self.envelope_hip_z_m is None:
            return 0.0
        return float(self.envelope_hip_z_m - self.hip_z_low_m)

    def as_dict(self) -> dict:
        return {
            "segment_index": self.index,
            "segment_kind": self.kind.value,
            "phase_label": self.phase_label,
            "x_from_mm": self.x_range_m[0] * 1e3,
            "x_to_mm": self.x_range_m[1] * 1e3,
            "body_kind": self.body_kind.value,
            "constraint_class": self.constraint_class.value,
            "endpoint_body_kind": (
                "" if self.endpoint_body_kind is None
                else self.endpoint_body_kind.value
            ),
            "segment_envelope_hip_z_mm": (
                "" if self.envelope_hip_z_m is None
                else self.envelope_hip_z_m * 1e3
            ),
            "hip_z_low_mm": self.hip_z_low_m * 1e3,
            "hip_z_high_mm": self.hip_z_high_m * 1e3,
            "max_envelope_excess_mm": self.max_envelope_excess_m * 1e3,
            "frame_count": self.frame_count,
            "segment_duration_s": self.duration_s,
            "contact_phase": self.contact_phase.value,
            "violation_effect": VIOLATION_EFFECT[self.constraint_class],
        }


@dataclass(frozen=True)
class TimelineGap2D:
    """An interval of the path that **no segment covers**.

    ``#4`` on a top longer than ``landing + takeoff`` has one: the composer
    measures it as ``gap_on_top_m`` and refuses to paper over it.  A timeline
    that simply ran the two swings together would hide it.
    """

    x_range_m: tuple[float, float]
    reason: str

    @property
    def length_m(self) -> float:
        return float(self.x_range_m[1] - self.x_range_m[0])

    def as_dict(self) -> dict:
        return {
            "x_from_mm": self.x_range_m[0] * 1e3,
            "x_to_mm": self.x_range_m[1] * 1e3,
            "gap_length_mm": self.length_m * 1e3,
            "constraint_class": ConstraintClass.UNRESOLVED.value,
            "violation_effect": VIOLATION_EFFECT[ConstraintClass.UNRESOLVED],
            "notes": self.reason,
        }


# --------------------------------------------------------------------------
# The timeline
# --------------------------------------------------------------------------


#: Written on every unresolved row, in the columns a resolved motion would
#: have filled.  Blank there would read as zero; this reads as what it is.
NOT_GENERATED = "NOT_GENERATED"


@dataclass(frozen=True)
class BodyTimeline2D:
    """One strategy at one cell: its body requirement, or its hole."""

    sequence_id: str
    strategy: StrategyId
    height_m: float
    top_length_m: float
    verdict: Verdict
    segments: tuple[TimelineSegment2D, ...] = ()
    knots: tuple[BodyKnot2D, ...] = ()
    unresolved: tuple[TransitionRequirement2D, ...] = ()
    gaps: tuple[TimelineGap2D, ...] = ()
    #: The smallest terrain clearance measured anywhere along the sequence.
    #: This is the geometric slack a body deviation eats into, which is the
    #: only tracking-tolerance number Day 10--11 actually has.
    clearance_budget_m: float | None = None
    notes: str = ""

    @property
    def is_executable(self) -> bool:
        """A timeline Day 12 can act on: it has motion and no holes."""

        return bool(self.knots) and not self.unresolved and not self.gaps

    @property
    def x_range_m(self) -> tuple[float, float] | None:
        if not self.knots:
            return None
        return (self.knots[0].x_m, self.knots[-1].x_m)

    @property
    def x_is_monotonic(self) -> bool:
        """Whether ``x`` may be used as the independent variable at all."""

        xs = [k.x_m for k in self.knots]
        return all(b - a >= -X_TOLERANCE_M for a, b in zip(xs, xs[1:]))

    @property
    def max_knot_spacing_m(self) -> float | None:
        """How coarse the polyline is -- Day 12 interpolates between knots."""

        if len(self.knots) < 2:
            return None
        xs = [k.x_m for k in self.knots]
        return float(max(b - a for a, b in zip(xs, xs[1:])))

    @property
    def max_envelope_excess_m(self) -> float:
        return float(max((k.envelope_excess_m for k in self.knots), default=0.0))

    def knots_of_class(self, klass: ConstraintClass) -> tuple[BodyKnot2D, ...]:
        return tuple(k for k in self.knots if k.constraint_class is klass)

    # -- rows ------------------------------------------------------------

    def _head(self) -> dict:
        return {
            "sequence_id": self.sequence_id,
            "strategy": self.strategy.value,
            "verdict": self.verdict.value,
            "obstacle_mm": self.height_m * 1e3,
            "top_length_mm": self.top_length_m * 1e3,
        }

    def rows(self) -> list[dict]:
        """Sequence, segments, knots, gaps and unresolved requirements.

        One table on purpose.  A hand-off that put the unsolved transitions in
        a second file would be read as complete by anyone who opened only the
        first, which is precisely the collapse of "unsolved" into "absent"
        that spec 5.6 exists to prevent.
        """

        head = self._head()
        rows: list[dict] = [{
            "row_kind": "sequence", **head,
            "executable": self.is_executable,
            "segments": len(self.segments),
            "knots": len(self.knots),
            "hard_knots": len(self.knots_of_class(ConstraintClass.HARD)),
            "preference_knots": len(
                self.knots_of_class(ConstraintClass.PREFERENCE)),
            "unresolved_transitions": len(self.unresolved),
            "unplanned_gaps": len(self.gaps),
            "x_from_mm": None if not self.knots else self.x_range_m[0] * 1e3,
            "x_to_mm": None if not self.knots else self.x_range_m[1] * 1e3,
            # Blank, not True, on a sequence with no knots: a vacuous truth
            # here would read as "the timeline was checked and it is fine".
            "x_is_monotonic": "" if not self.knots else self.x_is_monotonic,
            "max_knot_spacing_mm": (
                None if self.max_knot_spacing_m is None
                else self.max_knot_spacing_m * 1e3
            ),
            # Blank where no segment carried a scalar at all, so "no envelope
            # exists here" is not written as "the envelope hides 0 mm".
            "max_envelope_excess_mm": (
                "" if all(s.envelope_hip_z_m is None for s in self.segments)
                else self.max_envelope_excess_m * 1e3
            ),
            "clearance_budget_mm": (
                None if self.clearance_budget_m is None
                else self.clearance_budget_m * 1e3
            ),
            "time_basis": TIME_BASIS,
            "notes": self.notes,
        }]
        rows += [{"row_kind": "segment", **head, **s.as_dict()}
                 for s in self.segments]
        rows += [{"row_kind": "knot", **head, **k.as_dict()}
                 for k in self.knots]
        rows += [{"row_kind": "unplanned_gap", **head, **g.as_dict()}
                 for g in self.gaps]
        for requirement in self.unresolved:
            row = {
                "row_kind": "unresolved_transition", **head,
                "constraint_class": ConstraintClass.UNRESOLVED.value,
                "violation_effect": VIOLATION_EFFECT[ConstraintClass.UNRESOLVED],
                # The columns a generated motion would have filled.  Spec Step
                # 9: do NOT guess the reposition's theta / beta / duration.
                "hip_z_required_mm": NOT_GENERATED,
                "theta_deg": NOT_GENERATED,
                "beta_deg": NOT_GENERATED,
                "segment_duration_s": NOT_GENERATED,
                "x_from_mm": NOT_GENERATED,
                "x_to_mm": NOT_GENERATED,
            }
            row.update(requirement.as_dict())
            blocked = BLOCKED_PAIRS.get(self.strategy)
            if blocked is not None:
                row["single_leg_fix"] = blocked.single_leg_fix or ""
                row["multileg_route"] = blocked.multileg_route or ""
            rows.append(row)
        return rows


# --------------------------------------------------------------------------
# Building one from a composed sequence
# --------------------------------------------------------------------------


def _rim_of(row: dict) -> str:
    return str(row.get("active_rim") or "")


def _float_or_none(value) -> float | None:
    if value is None or value == "":
        return None
    return float(value)


def _segment_envelope_m(segment: MotionSegment2D) -> float | None:
    """The single scalar Step 6/7 carried for this segment, if it carried one.

    ``None`` on a rolling segment, and that is the point: a ``TRACK``
    requirement **is** the profile, so it has no envelope to be misread as one.
    Taking ``max(profile)`` here would manufacture a claim Step 6 never made
    and then report the body as over-constrained by it.
    """

    if segment.body_requirement.kind is BodyRequirementKind.TRACK:
        return None
    return float(segment.body_requirement.hip_z_min_m)


def _knots_for_segment(
    segment: MotionSegment2D,
    index: int,
    frames_by_index: dict[int, dict],
) -> list[BodyKnot2D]:
    """One knot per frame, with the endpoint rule applied to swings."""

    envelope = _segment_envelope_m(segment)
    indices = segment.frames.indices
    knots: list[BodyKnot2D] = []
    for position, frame_index in enumerate(indices):
        row = frames_by_index.get(frame_index)
        if row is None:
            raise KeyError(
                f"segment {index} references frame {frame_index}, which is not "
                "in the frame rows: the timeline must not invent one."
            )
        is_endpoint = position in (0, len(indices) - 1)
        if segment.kind.is_swing:
            body_kind = (
                BodyRequirementKind.PINNED if is_endpoint
                else BodyRequirementKind.LOWER_BOUND
            )
            basis = (
                RequirementBasis.PINNED_CONTACT if is_endpoint
                else RequirementBasis.WITNESS_LOWER_BOUND
            )
            phase = ContactPhase.STANCE if is_endpoint else ContactPhase.FLIGHT
        else:
            body_kind = BodyRequirementKind.TRACK
            basis = RequirementBasis.EXACT_TRACK
            phase = ContactPhase.STANCE
        hip_z = float(row["hip_z_m"])
        contact_x = _float_or_none(row.get("contact_x_m"))
        contact_z = _float_or_none(row.get("contact_z_m"))
        knots.append(BodyKnot2D(
            segment_index=index,
            segment_kind=segment.kind,
            frame_index=int(frame_index),
            x_m=float(row["hip_x_m"]),
            hip_z_m=hip_z,
            body_kind=body_kind,
            basis=basis,
            contact_phase=phase,
            contact_xz_m=(
                None if contact_x is None or contact_z is None
                else (contact_x, contact_z)
            ),
            rim=_rim_of(row),
            alpha_rad=(
                None if _float_or_none(row.get("alpha_deg")) is None
                else float(np.deg2rad(float(row["alpha_deg"])))
            ),
            theta_rad=(
                None if _float_or_none(row.get("theta_deg")) is None
                else float(np.deg2rad(float(row["theta_deg"])))
            ),
            beta_rad=(
                None if _float_or_none(row.get("beta_deg")) is None
                else float(np.deg2rad(float(row["beta_deg"])))
            ),
            time_s=_float_or_none(row.get("time_s")),
            envelope_excess_m=(
                0.0 if envelope is None else float(envelope - hip_z)
            ),
        ))
    return knots


def timeline_from_composed_2d(
    result,
    *,
    sequence_id: str | None = None,
) -> BodyTimeline2D:
    """Derive the body-requirement timeline of a composed sequence.

    ``result`` is Step 7's :class:`ComposedSequence2D`.  A blocked pair -- one
    with no sequence -- comes back as a timeline with no knots and its
    unresolved requirements intact, so the two cases share one reader.
    """

    strategy = result.strategy
    sequence_id = sequence_id or _sequence_id(
        strategy, result.height_m, result.top_length_m
    )
    if result.sequence is None:
        return BodyTimeline2D(
            sequence_id=sequence_id,
            strategy=strategy,
            height_m=result.height_m,
            top_length_m=result.top_length_m,
            verdict=(
                result.verdict if result.verdict is not None
                else Verdict.OUT_OF_ENVELOPE
            ),
            unresolved=tuple(result.unresolved),
            notes=result.refusal or result.notes,
        )

    frames_by_index = {int(r["index"]): r for r in result.frame_rows}
    segments: list[TimelineSegment2D] = []
    knots: list[BodyKnot2D] = []
    for index, segment in enumerate(result.sequence.segments):
        segment_knots = _knots_for_segment(segment, index, frames_by_index)
        knots.extend(segment_knots)
        heights = [k.hip_z_m for k in segment_knots]
        segments.append(TimelineSegment2D(
            index=index,
            kind=segment.kind,
            phase_label=segment.phase_label,
            x_range_m=(min(k.x_m for k in segment_knots),
                       max(k.x_m for k in segment_knots)),
            body_kind=segment.body_requirement.kind,
            envelope_hip_z_m=_segment_envelope_m(segment),
            hip_z_low_m=float(min(heights)),
            hip_z_high_m=float(max(heights)),
            frame_count=len(segment_knots),
            duration_s=segment.duration_s,
            contact_phase=(
                ContactPhase.FLIGHT if segment.kind.is_swing
                else ContactPhase.STANCE
            ),
            endpoint_body_kind=(
                BodyRequirementKind.PINNED if segment.kind.is_swing else None
            ),
        ))

    parameters = dict(result.parameters)
    gaps: list[TimelineGap2D] = []
    gap_on_top = float(parameters.get("gap_on_top_m", 0.0))
    if gap_on_top > 1e-3:
        # The composer reports where this is: between the ascent's landing and
        # the descent's take-off, on the top.
        landing_end = segments[0].x_range_m[1]
        gaps.append(TimelineGap2D(
            x_range_m=(landing_end, landing_end + gap_on_top),
            reason=(
                "crossing the top between the two swings: no planner in this "
                "project can generate it yet (the engine needs a "
                "'forward_distance' stop condition; research plan Day 15-16)."
            ),
        ))

    return BodyTimeline2D(
        sequence_id=sequence_id,
        strategy=strategy,
        height_m=result.height_m,
        top_length_m=result.top_length_m,
        verdict=(
            result.verdict if result.verdict is not None else Verdict.COMPOSED
        ),
        segments=tuple(segments),
        knots=tuple(knots),
        unresolved=tuple(result.sequence.unresolved),
        gaps=tuple(gaps),
        clearance_budget_m=result.min_clearance_m,
        notes=result.notes,
    )


def _sequence_id(strategy: StrategyId, height_m: float, top_length_m: float) -> str:
    tag = strategy.value.split()[0].lstrip("#")
    name = "_".join(strategy.value.split()[1:]).replace("+_", "").lower()
    return f"s{tag}_{name}_h{height_m * 1e3:.0f}_L{top_length_m * 1e3:.0f}"


# --------------------------------------------------------------------------
# The file
# --------------------------------------------------------------------------


def provenance_rows_2d(extra: Sequence[tuple[str, str]] = ()) -> list[dict]:
    """The notes the completion criteria require to live in the file itself."""

    notes: list[tuple[str, str]] = [
        ("time_basis", TIME_BASIS_STATEMENT),
        ("hard_vs_preference", (
            "constraint_class = HARD covers body_kind TRACK (a rolling "
            "segment: the hip height is an output of the contact geometry) "
            "and PINNED (a swing endpoint: the foot is on a surface, so hip "
            "height and joint angles determine each other).  "
            f"HARD -- {VIOLATION_EFFECT[ConstraintClass.HARD]}  "
            "constraint_class = PREFERENCE covers LOWER_BOUND (a swing "
            f"interior).  PREFERENCE -- {VIOLATION_EFFECT[ConstraintClass.PREFERENCE]}"
        )),
        ("upper_bound", (
            "hip_z_upper_bound_mm is NOT_MEASURED everywhere.  Raising the "
            "hip increases the leg's reach demand and no Day 10-11 sweep "
            "looked for where the IK stops converging.  NOT_MEASURED is not "
            "'unbounded'."
        )),
        ("envelope_vs_timeline", (
            "segment_envelope_hip_z_mm is the single scalar Step 6/7 carried "
            "for a segment (the maximum of the planned hip trajectory), and "
            "it is the number Step 5 decided on.  It is a correct ENVELOPE "
            "and a misleading TIMELINE: applied at a swing's lift-off end it "
            "over-constrains the body by max_envelope_excess_mm.  Use the "
            "per-knot hip_z_required_mm along the path; use the envelope only "
            "when comparing whole strategies.  It is BLANK on a rolling "
            "segment, which never had one: a TRACK requirement is the profile "
            "itself, and quoting max(profile) as its envelope would "
            "manufacture a claim Step 6 never made."
        )),
        ("unresolved_rows", (
            "row_kind = unresolved_transition marks a motion that is NEEDED "
            "and NOT YET SOLVED -- it is not the same as 'no such motion is "
            "needed'.  Its theta / beta / duration columns say NOT_GENERATED "
            "on purpose: guessing them would publish an assumption as a "
            "result (spec 5.6).  row_kind = unplanned_gap marks a stretch of "
            "path no segment covers."
        )),
        ("verdict_vocabulary", (
            "verdict follows spec 5.5.  DIRECT_HANDOFF_INFEASIBLE and "
            "REQUIRES_MULTILEG_REPOSITION are statements about the CURRENT "
            "single-leg primitive set, not about the robot.  No row in this "
            "file is PHYSICALLY_INFEASIBLE."
        )),
        ("scope", (
            "Offline, single-leg, 2D sagittal, one known rectangular "
            "obstacle.  Geometry, kinematics and sampled collision checks "
            "only: no dynamics, no friction, no load."
        )),
    ]
    notes.extend(extra)
    return [
        {"row_kind": "provenance", "sequence_id": "", "strategy": "",
         "verdict": "", "notes": f"[{key}] {text}"}
        for key, text in notes
    ]


def union_rows(rows: Iterable[dict]) -> list[dict]:
    """Pad every row to the union of the columns.  Implementation-log trap 21."""

    rows = [dict(r) for r in rows]
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    return [{key: row.get(key, "") for key in columns} for row in rows]


def timeline_rows_2d(
    timelines: Sequence[BodyTimeline2D],
    extra_provenance: Sequence[tuple[str, str]] = (),
) -> list[dict]:
    """The delivered table: provenance first, then one block per sequence."""

    rows = provenance_rows_2d(extra_provenance)
    for timeline in timelines:
        rows.extend(timeline.rows())
    return union_rows(rows)


# --------------------------------------------------------------------------
# Reading it back with nothing but the standard library
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class ReaderCheck2D:
    """Whether the delivered file stands on its own.

    Step 9's first completion criterion is that Day 12 can read this file and
    nothing else.  The honest way to check that is to read it with no project
    import at all -- which is what :func:`reader_check_2d` does -- rather than
    to assert it in prose.
    """

    path: Path
    sequences: int
    executable_sequences: int
    knots: int
    hard_knots: int
    preference_knots: int
    unresolved_rows: int
    gap_rows: int
    problems: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.problems

    def as_dict(self) -> dict:
        return {
            "check": "reader_check",
            "file": self.path.name,
            "sequences": self.sequences,
            "executable_sequences": self.executable_sequences,
            "knots": self.knots,
            "hard_knots": self.hard_knots,
            "preference_knots": self.preference_knots,
            "unresolved_rows": self.unresolved_rows,
            "gap_rows": self.gap_rows,
            "passed": self.passed,
            "problems": " | ".join(self.problems),
        }


def reader_check_2d(path: Path | str) -> ReaderCheck2D:
    """Re-read the delivered CSV the way Day 12 would, and audit it.

    Deliberately uses only :mod:`csv`: if this function needed anything from
    ``hybrid_note`` or ``legwheel``, the file would not be self-contained and
    the criterion would be false however loudly the driver claimed it.
    """

    path = Path(path)
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    problems: list[str] = []
    kinds = {r["row_kind"] for r in rows}
    if "provenance" not in kinds:
        problems.append("no provenance rows: the file does not state its own basis.")
    if not any("[time_basis]" in r.get("notes", "") for r in rows):
        problems.append("the time basis is not written in the file.")
    if not any("[hard_vs_preference]" in r.get("notes", "") for r in rows):
        problems.append("hard vs preference is not defined in the file.")

    sequence_rows = [r for r in rows if r["row_kind"] == "sequence"]
    knot_rows = [r for r in rows if r["row_kind"] == "knot"]
    gap_rows = [r for r in rows if r["row_kind"] == "unplanned_gap"]
    unresolved_rows = [r for r in rows if r["row_kind"] == "unresolved_transition"]

    if not sequence_rows:
        problems.append("no sequence rows.")

    for head in sequence_rows:
        sequence_id = head["sequence_id"]
        segments = [r for r in rows
                    if r["row_kind"] == "segment" and r["sequence_id"] == sequence_id]
        knots = [r for r in knot_rows if r["sequence_id"] == sequence_id]
        if int(head["segments"]) != len(segments):
            problems.append(
                f"{sequence_id}: header claims {head['segments']} segments, "
                f"{len(segments)} rows present."
            )
        if int(head["knots"]) != len(knots):
            problems.append(
                f"{sequence_id}: header claims {head['knots']} knots, "
                f"{len(knots)} rows present."
            )
        for segment in segments:
            covered = [k for k in knots
                       if k["segment_index"] == segment["segment_index"]]
            if int(segment["frame_count"]) != len(covered):
                problems.append(
                    f"{sequence_id} segment {segment['segment_index']}: "
                    f"{segment['frame_count']} frames claimed, "
                    f"{len(covered)} knots present."
                )
        xs = [float(k["x_mm"]) for k in knots]
        if any(b - a < -1e-6 for a, b in zip(xs, xs[1:])):
            problems.append(f"{sequence_id}: x is not monotonic; it cannot be "
                            "the independent variable.")
        classes = {k["constraint_class"] for k in knots}
        if not classes <= {"HARD", "PREFERENCE"}:
            problems.append(f"{sequence_id}: knot rows carry a class outside "
                            f"HARD / PREFERENCE: {sorted(classes)}.")
        for knot in knots:
            if knot["hip_z_upper_bound_mm"] != UPPER_BOUND_STATUS:
                problems.append(
                    f"{sequence_id}: a knot claims an upper bound Day 10-11 "
                    "never measured."
                )
                break

    for row in unresolved_rows:
        if row.get("resolved", "").strip() not in ("False", "false"):
            problems.append(
                f"{row['sequence_id']}: an unresolved transition is marked "
                "resolved; a resolved transition is a segment, not a "
                "requirement."
            )
        for column in ("theta_deg", "beta_deg", "segment_duration_s"):
            if row.get(column) != NOT_GENERATED:
                problems.append(
                    f"{row['sequence_id']}: unresolved row fills {column}; "
                    "Day 10-11 must not guess a reposition's pose or timing."
                )
    if any(r["verdict"] == "PHYSICALLY_INFEASIBLE" for r in rows):
        problems.append("a row claims PHYSICALLY_INFEASIBLE; Day 10-11 has no "
                        "cell entitled to it (spec 5.5).")

    return ReaderCheck2D(
        path=path,
        sequences=len(sequence_rows),
        executable_sequences=sum(
            1 for r in sequence_rows if r["executable"] == "True"
        ),
        knots=len(knot_rows),
        hard_knots=sum(1 for r in knot_rows if r["constraint_class"] == "HARD"),
        preference_knots=sum(
            1 for r in knot_rows if r["constraint_class"] == "PREFERENCE"
        ),
        unresolved_rows=len(unresolved_rows),
        gap_rows=len(gap_rows),
        problems=tuple(problems),
    )


def read_timeline_rows_2d(path: Path | str) -> list[dict]:
    """The delivered file as plain rows, for anything that only plots it."""

    with Path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))
