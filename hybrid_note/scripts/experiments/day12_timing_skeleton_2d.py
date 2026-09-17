"""Day 12 Step 3: the four-leg timing skeleton.

Plan §10.  One common timeline for LF/RF/LH/RH, built by **reusing** the
project's existing walk-gait phase definition rather than inventing one.  It
does timing and nothing else: no support polygon (Step 6), no body trajectory
(Step 5), no terrain decision anywhere.

Why the existing walk definition is the right thing to reuse
------------------------------------------------------------

``GAIT_LIBRARY["Walk"]`` in ``gait_generator_3d`` already *is* the
one-airborne-leg structure::

    phase_offsets = [0.75, 0.25, 0.5, 0.0]     indexed [FL, FR, RR, RL]
    stance_duty   = 0.75

The phase semantics are given in that file: ``phase_offset`` is where on the
``[stance | swing]`` curve a leg starts, so leg ``i``'s phase at time ``t`` is
``(t/T + phi_i) mod 1``, and it is airborne when that reaches ``stance_duty``.
Working it through gives four swing windows that tile the cycle exactly once::

    LF (FL)  t/T in [0.00, 0.25)
    RH (RR)  t/T in [0.25, 0.50)
    RF (FR)  t/T in [0.50, 0.75)
    LH (RL)  t/T in [0.75, 1.00)

Exactly one leg airborne at any instant, in the order that file's own comment
claims.  :func:`walk_timing_2d` reads the numbers out of ``GAIT_LIBRARY``; the
tiling is checked rather than assumed.

The measurement that makes this step more than plumbing
--------------------------------------------------------

Step 1's nominal cycle turns the leg-wheel 79.69 deg while rolling and 280.31
deg while airborne.  **If time were proportional to rotation**, stance would be
only ``79.69/360 = 0.221`` of the cycle and 3.1 legs would be airborne on
average -- the one-airborne-leg constraint would be unreachable, not merely
tight.

Time is not proportional to rotation, and it does not have to be: the recovery
is airborne with no contact constraint, and the rolling stroke is quasi-static.
So the duty is a **modelling freedom**, and the four-leg constraint is what
spends it.  With four legs the airborne fraction must be at most ``1/4``, so
``stance_duty >= 0.75`` -- which is exactly what the existing walk uses.  At
that duty::

    rolling   79.69 deg / (0.75 T)  =  106.25 deg / T
    recovery  280.31 deg / (0.25 T) = 1121.24 deg / T
    ratio                              10.55x

So *"at most one airborne leg" is equivalent to requiring the recovery to sweep
beta about 10.6 times faster than the rolling stroke does.*
:func:`rotation_rate_demand_2d` computes it for any timing.  Step 3 **records**
that ratio and does not judge it: whether the joints can do it is Step 9's
validation, and deciding it here would be a feasibility claim this step has no
evidence for.

Time is this step's own modelling decision
-------------------------------------------

Day 6--7's traversal is quasi-static and Step 1's cycle advances in ``beta``,
so every segment reaching this module carries ``duration_s = None`` (Day 10--11
trap 33).  Every duration below is **assigned here**, not measured, and
:attr:`ScheduledSegment2D.duration_is_assigned` says so on every row so that no
consumer can read one as data.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from itertools import groupby

import numpy as np

from legwheel.planners.gait_generator_3d import GAIT_LIBRARY

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    MotionSegment2D,
    SegmentKind,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER, LegId
from hybrid_note.scripts.experiments.day12_segment_contract_2d import SegmentChain2D

__all__ = [
    "LegMode",
    "GaitTiming2D",
    "walk_timing_2d",
    "PhaseWindow2D",
    "ScheduledSegment2D",
    "TimingConflict2D",
    "FourLegSchedule2D",
    "schedule_chains_2d",
    "rotation_rate_demand_2d",
    "schedule_rows",
    "plot_timeline_2d",
]


class LegMode(str, Enum):
    """What a leg is doing over one timeline interval.

    Two values, because Step 3's only question is whether the leg is carrying
    load.  ``SUPPORT`` versus ``ROLLING`` is a distinction the *segment kind*
    already carries, and duplicating it here would create a second place for it
    to be wrong.
    """

    #: In terrain contact -- rolling or otherwise, but on the ground.
    STANCE = "STANCE"
    #: Off the ground.
    AIRBORNE = "AIRBORNE"

    @classmethod
    def of(cls, kind: SegmentKind) -> "LegMode":
        return cls.AIRBORNE if kind.is_swing else cls.STANCE


# --------------------------------------------------------------------------
# Timing
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class GaitTiming2D:
    """The common timeline's parameters.

    ``phase_offsets`` is indexed by the **project's** leg index (``0: FL,
    1: FR, 2: RR, 3: RL``), which is the order ``GAIT_LIBRARY`` uses.  Storing
    it in the plan's LF/RF/LH/RH reading order instead would swap the two hind
    legs -- the same trap Step 2's ``joint_position_rad`` has.
    """

    cycle_period_s: float
    stance_duty: float
    phase_offsets: tuple[float, float, float, float]
    gait_name: str = "Walk"

    def __post_init__(self) -> None:
        if not np.isfinite(self.cycle_period_s) or self.cycle_period_s <= 0.0:
            raise ValueError("cycle_period_s must be finite and positive.")
        if not 0.0 < self.stance_duty < 1.0:
            raise ValueError("stance_duty must be in the open interval (0, 1).")
        offsets = tuple(float(v) for v in self.phase_offsets)
        if len(offsets) != 4:
            raise ValueError("phase_offsets has one entry per leg, by leg index.")
        if any(not 0.0 <= v < 1.0 for v in offsets):
            raise ValueError("each phase offset is a fraction in [0, 1).")
        object.__setattr__(self, "phase_offsets", offsets)

    @property
    def stance_duration_s(self) -> float:
        return float(self.cycle_period_s * self.stance_duty)

    @property
    def swing_duration_s(self) -> float:
        return float(self.cycle_period_s * (1.0 - self.stance_duty))

    @property
    def max_simultaneous_airborne(self) -> int:
        """How many legs the duty alone allows to be airborne at once.

        ``ceil`` of the airborne fraction times four: at ``duty = 0.75`` it is
        one, and the schedule can then be built so that it *is* one.  A duty
        below 0.75 makes "at most one airborne" impossible for any phasing,
        which is a property of the number rather than of the schedule.
        """

        return int(np.ceil(4.0 * (1.0 - self.stance_duty) - 1e-9))

    def phase_at(self, leg: LegId, time_s: float) -> float:
        """Leg ``leg``'s position in its own cycle at ``time_s``, in ``[0, 1)``."""

        offset = self.phase_offsets[leg.index]
        return float((time_s / self.cycle_period_s + offset) % 1.0)

    def mode_at(self, leg: LegId, time_s: float) -> LegMode:
        return (
            LegMode.STANCE if self.phase_at(leg, time_s) < self.stance_duty
            else LegMode.AIRBORNE
        )

    def swing_window(self, leg: LegId) -> tuple[float, float]:
        """When in the first cycle this leg is airborne, as ``t/T`` fractions.

        Derived from the phase convention rather than tabulated, so the four
        windows tiling the cycle is a consequence that can be tested.
        """

        start = (self.stance_duty - self.phase_offsets[leg.index]) % 1.0
        return (start, start + (1.0 - self.stance_duty))

    def as_dict(self) -> dict:
        return {
            "gait": self.gait_name,
            "cycle_period_s": self.cycle_period_s,
            "stance_duty": self.stance_duty,
            "stance_duration_s": self.stance_duration_s,
            "swing_duration_s": self.swing_duration_s,
            **{
                f"phase_offset_{leg.value}": self.phase_offsets[leg.index]
                for leg in LEG_ORDER
            },
        }


def walk_timing_2d(cycle_period_s: float = 2.4) -> GaitTiming2D:
    """The project's own walk gait, read out of ``GAIT_LIBRARY``.

    ``cycle_period_s`` defaults to ``TrajectoryParams.PERIOD``.  It is the one
    number with no existing Day 12 meaning: Step 1's cycle has no duration at
    all, so *some* period has to be chosen here, and the project's own gait
    period is a better default than a fresh invention.  Nothing below depends
    on its value -- every result is either a fraction of it or a ratio.
    """

    definition = GAIT_LIBRARY["Walk"]
    return GaitTiming2D(
        cycle_period_s=float(cycle_period_s),
        stance_duty=float(definition["stance_duty"]),
        phase_offsets=tuple(float(v) for v in definition["phase_offsets"]),
        gait_name="Walk",
    )


def rotation_rate_demand_2d(
    stroke_rotation_rad: float,
    recovery_rotation_rad: float,
    timing: GaitTiming2D,
) -> dict:
    """What the duty demands of the recovery's angular rate.

    The number this step exists to surface: holding one leg airborne at a time
    is not free, and this is its price in ``beta`` speed.
    """

    stance_rate = float(stroke_rotation_rad) / timing.stance_duration_s
    swing_rate = float(recovery_rotation_rad) / timing.swing_duration_s
    return {
        "stroke_rotation_deg": float(np.rad2deg(stroke_rotation_rad)),
        "recovery_rotation_deg": float(np.rad2deg(recovery_rotation_rad)),
        "stance_duty": timing.stance_duty,
        "stance_rate_deg_per_s": float(np.rad2deg(stance_rate)),
        "swing_rate_deg_per_s": float(np.rad2deg(swing_rate)),
        "swing_to_stance_rate_ratio": float(swing_rate / stance_rate),
        "rotation_proportional_duty": float(
            stroke_rotation_rad / (stroke_rotation_rad + recovery_rotation_rad)
        ),
    }


# --------------------------------------------------------------------------
# The schedule
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class PhaseWindow2D:
    """One stance or swing window of one leg, before segments are placed in it."""

    leg: LegId
    index: int
    mode: LegMode
    start_s: float
    end_s: float

    @property
    def duration_s(self) -> float:
        return float(self.end_s - self.start_s)


@dataclass(frozen=True)
class ScheduledSegment2D:
    """One segment, given a start and an end on the common timeline."""

    leg: LegId
    window_index: int
    segment_index: int
    segment_kind: SegmentKind
    phase_label: str
    mode: LegMode
    start_s: float
    end_s: float
    frame_count: int
    #: Always ``True`` in Day 12.  Kept as a field rather than as prose because
    #: a consumer reading ``duration_s`` has no other way to tell an assigned
    #: number from a measured one, and Day 6--7 / Step 1 measured none.
    duration_is_assigned: bool = True

    @property
    def duration_s(self) -> float:
        return float(self.end_s - self.start_s)

    def covers(self, time_s: float) -> bool:
        return self.start_s <= time_s < self.end_s

    def as_dict(self) -> dict:
        return {
            "leg": self.leg.value,
            "leg_index": self.leg.index,
            "window_index": self.window_index,
            "segment_index": self.segment_index,
            "segment_kind": self.segment_kind.value,
            "phase_label": self.phase_label,
            "mode": self.mode.value,
            "start_s": self.start_s,
            "end_s": self.end_s,
            "duration_s": self.duration_s,
            "duration_is_assigned": self.duration_is_assigned,
            "frame_count": self.frame_count,
            "is_nominal_locomotion": self.segment_kind.is_nominal_locomotion,
            "is_terrain_transition": self.segment_kind.is_terrain_transition,
        }


@dataclass(frozen=True)
class TimingConflict2D:
    """An interval where more than one leg is airborne.

    Reported, never repaired.  A leg in a terrain transition can need a longer
    airborne run than one swing window holds, and shrinking it here would be
    Step 3 quietly overruling the motion Day 10--11 selected.
    """

    start_s: float
    end_s: float
    legs: tuple[LegId, ...]

    @property
    def duration_s(self) -> float:
        return float(self.end_s - self.start_s)

    def as_dict(self) -> dict:
        return {
            "start_s": self.start_s,
            "end_s": self.end_s,
            "duration_s": self.duration_s,
            "airborne_count": len(self.legs),
            "legs": ", ".join(leg.value for leg in self.legs),
        }


@dataclass(frozen=True)
class FourLegSchedule2D:
    """Four legs on one timeline.  Timing only."""

    timing: GaitTiming2D
    scheduled: tuple[ScheduledSegment2D, ...]

    def __post_init__(self) -> None:
        scheduled = tuple(self.scheduled)
        if not scheduled:
            raise ValueError("a schedule has at least one segment.")
        for leg in {s.leg for s in scheduled}:
            own = [s for s in scheduled if s.leg is leg]
            for before, after in zip(own, own[1:]):
                if after.start_s < before.end_s - 1e-12:
                    raise ValueError(
                        f"leg {leg.value} has overlapping segments at "
                        f"{before.end_s:.6f} s; one leg does one thing at a time."
                    )
        object.__setattr__(self, "scheduled", scheduled)

    # -- what the scheduler must expose (plan §10 requirement 6) ------------

    @property
    def legs(self) -> tuple[LegId, ...]:
        return tuple(leg for leg in LEG_ORDER if any(s.leg is leg for s in self.scheduled))

    @property
    def start_s(self) -> float:
        return float(min(s.start_s for s in self.scheduled))

    @property
    def end_s(self) -> float:
        return float(max(s.end_s for s in self.scheduled))

    def segments_of(self, leg: LegId) -> tuple[ScheduledSegment2D, ...]:
        return tuple(s for s in self.scheduled if s.leg is leg)

    def segment_at(self, leg: LegId, time_s: float) -> ScheduledSegment2D | None:
        for s in self.segments_of(leg):
            if s.covers(time_s):
                return s
        return None

    def airborne_legs_at(self, time_s: float) -> tuple[LegId, ...]:
        return tuple(
            leg for leg in self.legs
            if (s := self.segment_at(leg, time_s)) is not None
            and s.mode is LegMode.AIRBORNE
        )

    def swing_leg_at(self, time_s: float) -> LegId | None:
        """The single airborne leg, or ``None``.

        Raises when more than one leg is airborne rather than returning the
        first: a caller asking for *the* swing leg has assumed there is one,
        and quietly handing back one of several would hide the conflict this
        module exists to surface.
        """

        airborne = self.airborne_legs_at(time_s)
        if len(airborne) > 1:
            raise ValueError(
                f"{len(airborne)} legs are airborne at {time_s:.6f} s "
                f"({', '.join(l.value for l in airborne)}); ask "
                "airborne_legs_at, and see .conflicts."
            )
        return airborne[0] if airborne else None

    def support_legs_at(self, time_s: float) -> tuple[LegId, ...]:
        """The legs in contact.  **Not** a support polygon -- that is Step 6."""

        return tuple(
            leg for leg in self.legs
            if (s := self.segment_at(leg, time_s)) is not None
            and s.mode is LegMode.STANCE
        )

    # -- the constraint ----------------------------------------------------

    @property
    def boundaries_s(self) -> tuple[float, ...]:
        """Every instant at which any leg changes segment.

        The schedule is piecewise constant between these, so checking them (and
        the midpoints between them) is exhaustive -- no sampling rate to choose
        and nothing to miss between samples.
        """

        edges = sorted({s.start_s for s in self.scheduled}
                       | {s.end_s for s in self.scheduled})
        return tuple(edges)

    @property
    def conflicts(self) -> tuple[TimingConflict2D, ...]:
        lo_covered, hi_covered = self.covered_interval_s
        edges = [
            t for t in self.boundaries_s
            if lo_covered - 1e-12 <= t <= hi_covered + 1e-12
        ]
        edges = sorted({lo_covered, hi_covered, *edges})
        found = []
        for lo, hi in zip(edges, edges[1:]):
            if hi - lo <= 1e-12:
                continue
            airborne = self.airborne_legs_at(0.5 * (lo + hi))
            if len(airborne) > 1:
                found.append(TimingConflict2D(lo, hi, airborne))
        # Merge runs with the same leg set so one long conflict is one record.
        merged: list[TimingConflict2D] = []
        for conflict in found:
            if (merged and merged[-1].legs == conflict.legs
                    and abs(merged[-1].end_s - conflict.start_s) <= 1e-12):
                merged[-1] = TimingConflict2D(
                    merged[-1].start_s, conflict.end_s, conflict.legs
                )
            else:
                merged.append(conflict)
        return tuple(merged)

    @property
    def max_airborne_count(self) -> int:
        """The most legs airborne at once, over the covered interval.

        Also restricted to :attr:`covered_interval_s`: on a ragged edge a leg
        that has not started is not on the ground either, so counting there
        would compare against a robot that is not fully in the plan.
        """

        return max(
            (len(self.airborne_legs_at(t)) for t in self._covered_midpoints()),
            default=0,
        )

    @property
    def one_leg_airborne_at_a_time(self) -> bool:
        return self.max_airborne_count <= 1

    @property
    def covered_interval_s(self) -> tuple[float, float]:
        """The interval over which **every** scheduled leg has a segment.

        The legs start their chains at different absolute times, because a
        phase offset is exactly an offset in time.  So the overall span has
        ragged ends where one or two legs have not begun or have already
        finished, and a support count taken there is counting legs that are
        not in the plan yet rather than legs that are in the air.

        This is the interval the four-leg constraints are meaningful on.  The
        ragged parts are not hidden -- :attr:`ragged_intervals_s` returns them.
        """

        starts = []
        ends = []
        for leg in self.legs:
            own = self.segments_of(leg)
            starts.append(min(s.start_s for s in own))
            ends.append(max(s.end_s for s in own))
        return (float(max(starts)), float(min(ends)))

    @property
    def ragged_intervals_s(self) -> tuple[tuple[float, float], ...]:
        """The head and tail where not all legs are scheduled.  May be empty."""

        lo, hi = self.covered_interval_s
        out = []
        if lo > self.start_s + 1e-12:
            out.append((self.start_s, lo))
        if hi < self.end_s - 1e-12:
            out.append((hi, self.end_s))
        return tuple(out)

    def _covered_midpoints(self) -> list[float]:
        lo, hi = self.covered_interval_s
        edges = [t for t in self.boundaries_s if lo - 1e-12 <= t <= hi + 1e-12]
        edges = sorted({lo, hi, *edges})
        return [
            0.5 * (a + b) for a, b in zip(edges, edges[1:]) if b - a > 1e-12
        ]

    @property
    def every_swing_has_three_supports(self) -> bool:
        """Whether each airborne interval leaves exactly three legs down.

        Evaluated on :attr:`covered_interval_s` only.  Distinct from
        :attr:`one_leg_airborne_at_a_time`: one airborne leg still fails this
        if another leg is unscheduled, and conflating the two would report a
        ragged edge as a gait error.
        """

        for middle in self._covered_midpoints():
            if self.airborne_legs_at(middle) and len(
                self.support_legs_at(middle)
            ) != 3:
                return False
        return True

    def rows(self) -> list[dict]:
        return [s.as_dict() for s in self.scheduled]


# --------------------------------------------------------------------------
# Building it
# --------------------------------------------------------------------------


def _runs(segments: Sequence[MotionSegment2D]) -> list[list[tuple[int, MotionSegment2D]]]:
    """Maximal runs of segments that share an airborne/contact mode."""

    indexed = list(enumerate(segments))
    return [
        list(group)
        for _, group in groupby(indexed, key=lambda item: LegMode.of(item[1].kind))
    ]


def schedule_chains_2d(
    chains: dict[LegId, SegmentChain2D],
    timing: GaitTiming2D | None = None,
) -> FourLegSchedule2D:
    """Map per-leg segment chains onto one timeline.

    The input is a :class:`SegmentChain2D` per leg -- Step 0's container -- so
    an arbitrary sequence works and no obstacle object is involved (plan §10
    requirement 5).

    **How a segment gets a duration.**  Each chain is cut into maximal contact
    and airborne runs; a contact run is given one stance window and an airborne
    run one swing window, and the segments inside a run split that window in
    proportion to their **frame counts**.  Frames are the only extensive
    quantity these segments carry, and using them is a modelling choice, not a
    measurement: a segment's frames come from its own sampling, so a densely
    sampled stage gets more of the window than a sparsely sampled one of the
    same physical extent.  Step 5 or Step 8 may replace the split; the run
    boundaries are what Step 3 is actually fixing.

    A leg's first window starts at the time its own phase offset puts it, so
    the four legs interleave exactly as the walk gait says.
    """

    timing = walk_timing_2d() if timing is None else timing
    if not chains:
        raise ValueError("schedule at least one leg.")

    scheduled: list[ScheduledSegment2D] = []
    for leg in LEG_ORDER:
        chain = chains.get(leg)
        if chain is None:
            continue
        runs = _runs(chain.segments)
        # Where this leg's own cycle starts on the common clock.  A phase
        # offset of phi means the leg is already phi of the way through its
        # cycle at t = 0, so its cycle began that long ago.
        cursor = -float(timing.phase_offsets[leg.index]) * timing.cycle_period_s
        for window_index, run in enumerate(runs):
            mode = LegMode.of(run[0][1].kind)
            duration = (
                timing.stance_duration_s if mode is LegMode.STANCE
                else timing.swing_duration_s
            )
            total_frames = sum(s.frames.frame_count for _, s in run)
            offset = 0.0
            for position, (segment_index, segment) in enumerate(run):
                share = segment.frames.frame_count / total_frames
                span = duration * share
                start = cursor + offset
                # The last segment of a run ends exactly on the window edge,
                # so accumulated rounding cannot leak into the next window.
                end = (
                    cursor + duration if position == len(run) - 1
                    else start + span
                )
                scheduled.append(ScheduledSegment2D(
                    leg=leg,
                    window_index=window_index,
                    segment_index=segment_index,
                    segment_kind=segment.kind,
                    phase_label=segment.phase_label,
                    mode=mode,
                    start_s=start,
                    end_s=end,
                    frame_count=segment.frames.frame_count,
                ))
                offset += span
            cursor += duration

    return FourLegSchedule2D(timing=timing, scheduled=tuple(scheduled))


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------


def schedule_rows(schedule: FourLegSchedule2D) -> list[dict]:
    """Timing parameters, every scheduled segment, and any conflicts."""

    rows = [{"row_kind": "timing", **schedule.timing.as_dict()}]
    rows += [{"row_kind": "segment", **row} for row in schedule.rows()]
    rows += [
        {"row_kind": "conflict", **conflict.as_dict()}
        for conflict in schedule.conflicts
    ]
    lo, hi = schedule.covered_interval_s
    rows.append({
        "row_kind": "summary",
        "start_s": schedule.start_s,
        "end_s": schedule.end_s,
        "covered_from_s": lo,
        "covered_to_s": hi,
        "ragged_interval_count": len(schedule.ragged_intervals_s),
        "max_airborne_count": schedule.max_airborne_count,
        "one_leg_airborne_at_a_time": schedule.one_leg_airborne_at_a_time,
        "every_swing_has_three_supports": schedule.every_swing_has_three_supports,
        "conflict_count": len(schedule.conflicts),
    })
    fields: list[str] = []
    for row in rows:
        fields += [key for key in row if key not in fields]
    return [{key: row.get(key) for key in fields} for row in rows]


#: One colour per mode, plus a distinct one for terrain-transition segments so
#: the plot shows what the gait always does apart from what a terrain forced.
_MODE_COLORS = {
    (LegMode.STANCE, False): "#2a6f4e",
    (LegMode.STANCE, True): "#7fb069",
    (LegMode.AIRBORNE, False): "#b06000",
    (LegMode.AIRBORNE, True): "#c5221f",
}


def plot_timeline_2d(schedule: FourLegSchedule2D, path=None, *, ax=None):
    """The four-leg timeline (plan §10 requirement 8)."""

    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    created = ax is None
    if created:
        figure, axes = plt.subplots(
            2, 1, figsize=(12.0, 6.0), sharex=True,
            gridspec_kw={"height_ratios": [3.0, 1.0]},
        )
    else:
        figure, axes = ax.figure, np.atleast_1d(ax)
    lanes, counts = axes[0], axes[-1]

    legs = schedule.legs
    rows = {leg: len(legs) - 1 - i for i, leg in enumerate(legs)}
    for segment in schedule.scheduled:
        colour = _MODE_COLORS[
            (segment.mode, segment.segment_kind.is_terrain_transition)
        ]
        y = rows[segment.leg]
        lanes.barh(y, segment.duration_s, left=segment.start_s, height=0.62,
                   color=colour, edgecolor="white", linewidth=0.8)
        if segment.duration_s > 0.10 * schedule.timing.cycle_period_s:
            lanes.text(
                segment.start_s + segment.duration_s / 2.0, y,
                segment.segment_kind.value.replace("_", "\n"),
                ha="center", va="center", fontsize=6.0, color="white",
            )
    for conflict in schedule.conflicts:
        lanes.axvspan(conflict.start_s, conflict.end_s, color="#c5221f",
                      alpha=0.18, lw=0, zorder=0)

    lanes.set_yticks([rows[leg] for leg in legs])
    lanes.set_yticklabels([leg.value for leg in legs])
    lanes.set_ylim(-0.6, len(legs) - 0.4)
    lanes.grid(axis="x", alpha=0.25)
    lanes.legend(
        handles=[
            Patch(color=_MODE_COLORS[(LegMode.STANCE, False)], label="stance (nominal)"),
            Patch(color=_MODE_COLORS[(LegMode.AIRBORNE, False)], label="airborne (nominal recovery)"),
            Patch(color=_MODE_COLORS[(LegMode.STANCE, True)], label="stance (terrain transition)"),
            Patch(color=_MODE_COLORS[(LegMode.AIRBORNE, True)], label="airborne (terrain transition)"),
        ],
        fontsize=7.5, ncol=4, loc="lower left", bbox_to_anchor=(0.0, 1.02),
        frameon=False,
    )
    timing = schedule.timing
    lanes.set_title(
        f"{timing.gait_name}: duty {timing.stance_duty:.2f}, "
        f"T = {timing.cycle_period_s:.2f} s   |   "
        f"max airborne = {schedule.max_airborne_count}, "
        f"conflicts = {len(schedule.conflicts)}",
        fontsize=9, pad=30, loc="right",
    )

    edges = schedule.boundaries_s
    times = [0.5 * (lo + hi) for lo, hi in zip(edges, edges[1:]) if hi - lo > 1e-12]
    counts.step(
        [edges[0]] + [t for t in times] + [edges[-1]],
        [0] + [len(schedule.airborne_legs_at(t)) for t in times] + [0],
        where="mid", color="#111", lw=1.4,
    )
    counts.axhline(1.0, color="#c5221f", ls="--", lw=1.0,
                   label="Day 12 limit: one airborne leg")
    # The ragged ends are shaded in both panels: the constraints are evaluated
    # on the covered interval, so a count drawn outside it must not read as a
    # result.
    for lo_ragged, hi_ragged in schedule.ragged_intervals_s:
        for panel in (lanes, counts):
            panel.axvspan(lo_ragged, hi_ragged, color="#888", alpha=0.13, lw=0,
                          zorder=0)
    covered_lo, covered_hi = schedule.covered_interval_s
    counts.annotate(
        "not all legs scheduled", xy=(covered_lo, 3.4),
        xytext=(-4, 0), textcoords="offset points", ha="right", va="center",
        fontsize=7, color="#555",
    )
    counts.set_ylabel("airborne\nlegs")
    counts.set_xlabel("time  [s]")
    counts.set_yticks([0, 1, 2, 3, 4])
    counts.grid(alpha=0.25)
    counts.legend(fontsize=7.5, loc="upper right")

    if created:
        figure.tight_layout()
        if path is not None:
            figure.savefig(path, dpi=150)
            plt.close(figure)
    return figure
