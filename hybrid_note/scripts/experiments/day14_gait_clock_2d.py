"""Day 14: the gait clock -- the **only** clock the whole-body planner has.

Why this exists
---------------

Day 12/13 assigned time *after* the four legs had each been planned on their
own: ``world_schedule_2d`` read a segment's start time off its hip position,
``start_s = (hip_x - mount - origin) / speed``.  That makes a leg's time
identical to its claim about where the body is, and the two legs of a pair
share a ``mount_x`` -- so their crossings landed on the same instant and they
always swung together.  Five ways of retiming after the fact were measured and
all failed (Day 13 log 25, 33.5); the last one moved the four legs' agreement
about the body from 0.000 mm to 122.5 mm.

The project owner's direction (2026-09-07): the whole machine decides.  So the
body's progress along its path is fixed **first**, as a function of time, and
every leg is generated against it.  A stance leg rolls exactly as far as the
body carries its hip; a swing occupies a known interval of body travel.  Time
is never inferred from a leg afterwards.

What the clock knows
--------------------

* the gait timing (``GaitTiming2D``: period, duty, phase offsets), which fixes
  the liftoff **order** and the nominal swing window;
* the nominal body speed, which is not a free parameter -- it is one cycle's
  hip advance over one period (``body_speed_m_s``), and it is what makes a
  full stroke last exactly ``duty * T``;
* optional speed zones (``SpeedZone2D``), applied to the **body's** x, so that a
  swing which needs more time than its hip travel allows can slow the machine
  down for exactly that stretch -- every leg reads the same slower clock and
  the four still agree about the body by construction.

The mapping between body x and time is monotone and piecewise linear, and it
is defined on *both* sides of the origin: the flat gait starts each leg's chain
at ``-phase * T``, before the common origin, and the frozen Day 12 numbers were
measured that way.  ``time_at_body_x_2d`` clamps to zero below the origin,
which is right for its callers and wrong here, so the clock carries its own
inverse pair.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace

import numpy as np

from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER, LegId
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import GaitTiming2D
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (
    MOTOR_MAX_RATE_RAD_S,
    motor_rates_rad_s,
)
from hybrid_note.scripts.experiments.day12_world_registration_2d import SpeedZone2D

__all__ = [
    "Dwell2D",
    "GaitClock2D",
    "SwingEvent2D",
    "liftoff_order_2d",
    "minimum_swing_duration_s",
    "frame_hip_x",
]


def liftoff_order_2d(timing: GaitTiming2D) -> tuple[LegId, ...]:
    """The order the legs lift off in, read off the timing's swing windows.

    Not tabulated: ``GaitTiming2D.swing_window`` derives each leg's window from
    its phase offset, so the order is a consequence of the offsets and a test
    can check it against the gait file's own claim (FL -> RR -> FR -> RL).
    """

    starts = [(timing.swing_window(leg)[0], leg) for leg in LEG_ORDER]
    starts.sort(key=lambda pair: pair[0])
    return tuple(leg for _, leg in starts)


@dataclass(frozen=True)
class Dwell2D:
    """The body stands at ``x_m`` for ``duration_s`` while one leg swings in place.

    A leg that must clear the way for another leg's long swing, with no body
    travel left to do it in, swings with the body standing still.  That is a
    real motion of the machine (the stance legs hold, one leg turns), so the
    clock carries it as time passing at one body x -- not as a fake 0.1 mm of
    travel, which put the waiting leg's swing and the long swing at the same
    instant once the times were read off the clock.
    """

    x_m: float
    duration_s: float
    #: Placement order, so a swing can find its own dwell after the clock
    #: has grown further.
    order: int = 0

    def __post_init__(self) -> None:
        if not np.isfinite(self.duration_s) or self.duration_s < 0.0:
            raise ValueError("a dwell lasts a finite, non-negative time.")


@dataclass(frozen=True)
class GaitClock2D:
    """Body progress as a function of time, and back again."""

    timing: GaitTiming2D
    #: The nominal body speed.  One cycle's hip advance over one period.
    speed_m_s: float
    #: Body x at ``t = 0``.
    origin_x_m: float = 0.0
    #: Stretches of body x the machine covers slower (or faster) than nominal.
    zones: tuple[SpeedZone2D, ...] = ()
    #: Where the body stands still, in placement order (stable in x).
    dwells: tuple[Dwell2D, ...] = ()

    def __post_init__(self) -> None:
        if not np.isfinite(self.speed_m_s) or self.speed_m_s <= 0.0:
            raise ValueError("the body speed must be finite and positive.")
        ordered = tuple(sorted(self.zones, key=lambda z: z.x_start_m))
        for a, b in zip(ordered, ordered[1:]):
            if b.x_start_m < a.x_end_m - 1e-12:
                raise ValueError(
                    f"speed zones overlap: [{a.x_start_m}, {a.x_end_m}) and "
                    f"[{b.x_start_m}, {b.x_end_m}).")
        object.__setattr__(self, "zones", ordered)
        # A stable sort: dwells at one x keep the order they were placed in,
        # which is the order the legs swing in there.
        object.__setattr__(self, "dwells", tuple(sorted(self.dwells, key=lambda d: d.x_m)))

    # -- the two directions ------------------------------------------------

    def time_at_body_x(self, body_x_m: float, *, side: str = "arrive") -> float:
        """When the body is at ``body_x_m``.  Negative before the origin.

        Where the body dwells, ``side="arrive"`` is the moment it gets there
        and ``side="leave"`` the moment it moves on -- a stance segment ends
        at the first and begins at the second, and a swing that takes off
        from a dwell x takes off after the pauses held there.
        """

        x = float(body_x_m)
        held = 0.0
        for dwell in self.dwells:
            if dwell.x_m < x - 1e-12 or (side == "leave" and abs(dwell.x_m - x) <= 1e-12):
                held += dwell.duration_s
            elif dwell.x_m > x + 1e-12:
                break
        return self._travel_time_at_body_x(x) + held

    def _travel_time_at_body_x(self, x: float) -> float:
        """Time spent *moving* to reach ``x``: the zones alone."""

        origin = float(self.origin_x_m)
        if x <= origin:
            # Zones are only ever placed ahead of the origin; behind it the
            # body ran at its nominal speed, which is what the flat gait's
            # negative chain starts mean.
            return (x - origin) / self.speed_m_s
        total = 0.0
        cursor = origin
        for zone in self.zones:
            lo, hi = max(zone.x_start_m, cursor), min(zone.x_end_m, x)
            if hi <= lo:
                continue
            total += (lo - cursor) / self.speed_m_s
            total += (hi - lo) / zone.speed_m_s
            cursor = hi
            if cursor >= x:
                return float(total)
        return float(total + (x - cursor) / self.speed_m_s)

    def body_x_at_time(self, time_s: float) -> float:
        """Where the body is at ``time_s``.  The exact inverse of the above."""

        t = float(time_s)
        held = 0.0
        for dwell in self.dwells:
            arrive = self._travel_time_at_body_x(dwell.x_m) + held
            if t <= arrive:
                break
            if t <= arrive + dwell.duration_s:
                return float(dwell.x_m)
            held += dwell.duration_s
        return self._travel_x_at_time(t - held)

    def _travel_x_at_time(self, t: float) -> float:
        origin = float(self.origin_x_m)
        if t <= 0.0:
            return origin + t * self.speed_m_s
        elapsed = 0.0
        cursor = origin
        for zone in self.zones:
            if zone.x_start_m > cursor:
                gap = (zone.x_start_m - cursor) / self.speed_m_s
                if elapsed + gap >= t:
                    return cursor + (t - elapsed) * self.speed_m_s
                elapsed += gap
                cursor = zone.x_start_m
            span = (zone.x_end_m - cursor) / zone.speed_m_s
            if elapsed + span >= t:
                return cursor + (t - elapsed) * zone.speed_m_s
            elapsed += span
            cursor = zone.x_end_m
        return cursor + (t - elapsed) * self.speed_m_s

    def speed_at_body_x(self, body_x_m: float) -> float:
        x = float(body_x_m)
        for zone in self.zones:
            if zone.x_start_m <= x < zone.x_end_m:
                return float(zone.speed_m_s)
        return float(self.speed_m_s)

    def with_zone(self, zone: SpeedZone2D) -> "GaitClock2D":
        """The same clock with one more zone.  Overlaps are refused, not merged."""

        return replace(self, zones=self.zones + (zone,))

    def with_dwell(self, x_m: float, duration_s: float) -> tuple["GaitClock2D", float, float]:
        """The same clock with the body standing at ``x_m`` for ``duration_s``
        more, after whatever it already stands there for.  Returns the clock
        and the new dwell's ``(start_s, end_s)``.
        """

        start = self.time_at_body_x(float(x_m), side="leave")
        order = 1 + max((d.order for d in self.dwells), default=-1)
        clock = replace(self, dwells=self.dwells + (
            Dwell2D(float(x_m), float(duration_s), order),))
        return clock, float(start), float(start + duration_s)

    def dwell_window_s(self, order: int) -> tuple[float, float]:
        """``(start_s, end_s)`` of the dwell placed ``order``-th, on this clock."""

        for dwell in self.dwells:
            if dwell.order == order:
                before = sum(d.duration_s for d in self.dwells
                             if abs(d.x_m - dwell.x_m) <= 1e-12 and d.order < order)
                start = self.time_at_body_x(dwell.x_m) + before
                return float(start), float(start + dwell.duration_s)
        raise KeyError(f"no dwell with order {order}.")

    @property
    def dwell_time_s(self) -> float:
        return float(sum(d.duration_s for d in self.dwells))

    # -- what the gait says ------------------------------------------------

    @property
    def liftoff_order(self) -> tuple[LegId, ...]:
        return liftoff_order_2d(self.timing)

    def chain_start_time_s(self, leg: LegId) -> float:
        """When this leg's first full stroke began, given its phase at ``t = 0``.

        A phase offset of ``phi`` means the leg is ``phi`` of the way through
        its cycle at the origin, so its cycle began ``phi * T`` earlier.  This is
        exactly what ``schedule_chains_2d`` does, and it is what lets the
        flat-ground plan reproduce every frozen Day 12 number.
        """

        return -float(self.timing.phase_offsets[leg.index]) * float(
            self.timing.cycle_period_s)

    @property
    def nominal_swing_duration_s(self) -> float:
        return float(self.timing.swing_duration_s)

    @property
    def nominal_liftoff_spacing_s(self) -> float:
        """How far apart consecutive liftoffs are in the wave gait: ``T / 4``."""

        return float(self.timing.cycle_period_s) / 4.0

    def as_dict(self) -> dict:
        return {
            "speed_mm_s": self.speed_m_s * 1e3,
            "origin_x_mm": self.origin_x_m * 1e3,
            "cycle_period_s": self.timing.cycle_period_s,
            "stance_duty": self.timing.stance_duty,
            "liftoff_order": ",".join(l.value for l in self.liftoff_order),
            "zones": len(self.zones),
            "dwells": len(self.dwells),
            "dwell_time_s": self.dwell_time_s,
        }


# --------------------------------------------------------------------------
# A swing on the clock
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SwingEvent2D:
    """One airborne interval, in body x **and** in time.

    The body-x interval is what the planner serialises on: two swings whose
    body-x intervals overlap would put two legs in the air at once, whatever
    the speed profile does, so the rule "at most one airborne" is a rule about
    these intervals and is enforced when they are placed, not checked after.
    """

    leg: LegId
    body_x_start_m: float
    body_x_end_m: float
    start_s: float
    end_s: float
    kind: str
    #: The shortest time the motors allow this swing to take.
    minimum_duration_s: float

    @property
    def hip_advance_m(self) -> float:
        return float(self.body_x_end_m - self.body_x_start_m)

    @property
    def duration_s(self) -> float:
        return float(self.end_s - self.start_s)

    def overlaps(self, other: "SwingEvent2D", *, tolerance_m: float = 1e-9) -> bool:
        return (self.body_x_start_m < other.body_x_end_m - tolerance_m
                and other.body_x_start_m < self.body_x_end_m - tolerance_m)

    def as_dict(self) -> dict:
        return {
            "leg": self.leg.value,
            "kind": self.kind,
            "body_x_start_mm": self.body_x_start_m * 1e3,
            "body_x_end_mm": self.body_x_end_m * 1e3,
            "hip_advance_mm": self.hip_advance_m * 1e3,
            "start_s": self.start_s,
            "end_s": self.end_s,
            "duration_s": self.duration_s,
            "minimum_duration_s": self.minimum_duration_s,
        }


# --------------------------------------------------------------------------
# How long a swing must take
# --------------------------------------------------------------------------


def frame_hip_x(frame) -> float:
    return float(frame.hip_xz_m[0])


def minimum_swing_duration_s(
    frames: Sequence,
    *,
    rate_limit_rad_s: float = MOTOR_MAX_RATE_RAD_S,
    utilisation: float = 0.95,
) -> float:
    """The shortest time these frames can be played without exceeding the motors.

    The frames are spread evenly in time, as ``leg_sample_at`` plays them, so
    the joint rate between two neighbours is their joint step over one
    ``duration / (N - 1)``.  Each of the two motors (``phi_r = theta + beta``,
    ``phi_l = beta - theta``) is checked on its own, which is how the hardware
    is built and how ``day12_whole_body_validation_2d`` checks it; the shorthand
    ``|dtheta| + |dbeta|`` overstates when the joints move oppositely.

    ``utilisation`` keeps the same headroom the flat gait runs with (its peak is
    95.0% of the budget at the nominal window).  It is a planning margin, not a
    hardware fact.
    """

    if len(frames) < 2:
        return 0.0
    if not 0.0 < utilisation <= 1.0:
        raise ValueError("utilisation is a fraction in (0, 1].")
    worst = 0.0
    for a, b in zip(frames, frames[1:]):
        dtheta = float(b.theta_rad - a.theta_rad)
        dbeta = float(b.beta_rad - a.beta_rad)
        right, left = motor_rates_rad_s(dtheta, dbeta)
        worst = max(worst, abs(right), abs(left))
    steps = len(frames) - 1
    return float(steps * worst / (rate_limit_rad_s * utilisation))
