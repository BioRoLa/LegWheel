"""Day 12 Step 6: the three-leg support triangle and its stability margin.

Plan §13.  While one leg is airborne the other three carry the robot; this step
builds their support polygon in the **horizontal** plane, projects the body
centre onto it, and reports a signed margin over the whole swing -- not only at
liftoff, because a rolling support leg's contact point moves while the swing is
in the air.

**Three labels this module refuses to drop.**

``contact, not hip``      Plan §13 says the triangle is built from the actual
                          world contact points.  The lateral term is the
                          mounting offset Step 2 measured; the longitudinal one
                          is the contact's offset *from its own hip*, which is
                          real geometry and does not depend on the chain's
                          arbitrary x origin.
``body centre, not CoM``  There is no whole-robot mass model here, so the
                          projected point is the body centre and every row says
                          so (plan §13 requirement 10).
``body_z is infeasible``  Step 5 found the four-leg nominal cycle cannot hold a
                          single body height.  The margin is a horizontal
                          question and ``body_x`` is still well defined, so it
                          can be computed -- but the assumption travels with
                          the result instead of being quietly forgotten.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import MotionSegment2D
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    BODY_BASIS,
    BodyTrajectory2D,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    LegId,
    leg_mounts_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    LegMode,
    ScheduledSegment2D,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import FourLegPlan2D

#: What the projected point is.  Plan §13 requirement 10 asks for the
#: approximation to be isolated and labelled rather than silently called a CoM.
COM_BASIS = (
    "body centre projected on the horizontal plane -- "
    "quasi-static body-frame approximation, NOT a whole-robot CoM"
)

#: Day 12 keeps the ABAD joints at zero (plan §13 requirement 8), so each leg's
#: sagittal plane stays at its own mounting ``y`` and the horizontal geometry is
#: fixed laterally.  Anything that wants to move it is Day 13--14's business.
GAMMA_RAD: float = 0.0

#: Three points closer to collinear than this have no usable interior.  It is
#: an area, in m^2: the nominal support triangle is about 0.05 m^2, so this is
#: four orders of magnitude below a real one.
DEGENERATE_AREA_M2: float = 1e-6

#: The margin a swing must keep to count as stable.  A **planning** floor, not
#: a measured physical limit -- it is an argument everywhere it is used, and
#: this is only the default.
DEFAULT_MARGIN_FLOOR_M: float = 0.010


# --------------------------------------------------------------------------
# Where a leg is actually touching
# --------------------------------------------------------------------------


def contact_offset_from_hip_m(segment: MotionSegment2D, fraction: float) -> float:
    """The contact's ``x`` **relative to its own hip**, part way along a segment.

    Origin-free on purpose.  Step 4 hands every leg the same chain, whose
    absolute ``x`` starts at zero, so absolute contact positions do not share a
    frame -- but where the foot is relative to the hip is real geometry, and
    that is what places the contact once the hip is placed.
    """

    fraction = float(np.clip(fraction, 0.0, 1.0))
    start = (float(segment.start_contact.point_world_xz_m[0])
             - float(segment.start_contact.hip_xz_m[0]))
    end = (float(segment.end_contact.point_world_xz_m[0])
           - float(segment.end_contact.hip_xz_m[0]))
    return start + fraction * (end - start)


def _cross2(a, b) -> float:
    """The 2D cross product.  NumPy 2.0 deprecated ``np.cross`` on 2-vectors."""

    return float(a[0] * b[1] - a[1] * b[0])


#: Boundary tolerance, in seconds.  Window edges are computed as sums of
#: ``phase_offset * period``, so two legs' shared instant comes out as 0.6 for
#: one and 0.5999999999999999 for the other.  Without a tolerance that gap puts
#: a leg one segment behind and produces a two-leg "support" -- a scheduling
#: artefact that would read as a gait fault.  A microsecond is far below any
#: timing this planner assigns and far above the arithmetic.
BOUNDARY_TOLERANCE_S: float = 1e-6


def segment_at(scheduled: Sequence[ScheduledSegment2D],
               time_s: float) -> ScheduledSegment2D | None:
    """The one segment a leg is in at ``time_s``.

    Segments own ``[start, end)``, with the last one owning its own end.  At a
    boundary both neighbours would otherwise claim the instant, and picking
    either by hand gets it wrong in one direction: a leg at the *end* of its
    airborne segment is touching down, so it is a support leg, while a leg at
    the *start* of one has lifted off.  The half-open rule says both correctly
    and gives exactly one segment per leg per instant.
    """

    eps = BOUNDARY_TOLERANCE_S
    last = None
    for segment in scheduled:
        if segment.start_s - eps <= time_s < segment.end_s - eps:
            return segment
        if segment.start_s - eps <= time_s <= segment.end_s + eps:
            last = segment
    return last


@dataclass(frozen=True)
class SupportTriangle2D:
    """The three support contacts in the horizontal plane, at one instant."""

    time_s: float
    swing_leg: LegId | None
    support_legs: tuple[LegId, ...]
    #: ``(3, 2)`` world ``xy`` of the contacts, in ``support_legs`` order.
    points_xy_m: np.ndarray

    def __post_init__(self) -> None:
        points = np.asarray(self.points_xy_m, dtype=float).reshape(-1, 2)
        if len(points) > 3:
            # Slicing to the first three would compute a triangle from four
            # contacts and call the result a support polygon.  Day 12's first
            # version is three-leg support (plan §13); anything else is a
            # scheduling question, not a geometry one.
            raise ValueError(
                f"a three-leg support has three contacts, got {len(points)}: "
                "more than one leg is on the ground beyond the three, which is "
                "a scheduling result to report, not a polygon to build."
            )
        points.setflags(write=False)
        object.__setattr__(self, "points_xy_m", points)
        object.__setattr__(self, "support_legs", tuple(self.support_legs))

    @property
    def area_m2(self) -> float:
        """Unsigned area.  Zero when the contacts are collinear."""

        if len(self.points_xy_m) < 3:
            return 0.0
        a, b, c = self.points_xy_m[:3]
        return abs(_cross2(b - a, c - a)) / 2.0

    @property
    def is_degenerate(self) -> bool:
        return (len(self.points_xy_m) < 3
                or self.area_m2 < DEGENERATE_AREA_M2)

    def signed_margin_m(self, point_xy_m) -> float | None:
        """Distance from ``point_xy_m`` to the nearest edge; positive inside.

        ``None`` on a degenerate triangle: a collinear support has no interior,
        and returning a distance to a line would read as a real margin.
        """

        if self.is_degenerate:
            return None
        point = np.asarray(point_xy_m, dtype=float).reshape(2)
        vertices = self.points_xy_m[:3]
        # Orient consistently so "inside" is the same sign for both windings.
        a, b, c = vertices
        winding = _cross2(b - a, c - a)
        if winding < 0.0:
            vertices = vertices[::-1]

        distances = []
        for i in range(3):
            edge_a = vertices[i]
            edge_b = vertices[(i + 1) % 3]
            edge = edge_b - edge_a
            length = float(np.hypot(*edge))
            if length <= 0.0:
                return None
            # Positive when the point is on the interior side of this edge.
            distances.append(_cross2(edge, point - edge_a) / length)
        return float(min(distances))

    def as_dict(self) -> dict:
        return {
            "time_s": self.time_s,
            "swing_leg": None if self.swing_leg is None else self.swing_leg.value,
            "support_legs": ",".join(l.value for l in self.support_legs),
            "area_mm2": self.area_m2 * 1e6,
            "is_degenerate": self.is_degenerate,
        }


def support_triangle_at(
    plan: FourLegPlan2D,
    body_x_m: float,
    time_s: float,
    *,
    swing_leg: LegId | None = None,
    mounts: dict[LegId, np.ndarray] | None = None,
) -> SupportTriangle2D:
    """The support contacts at ``time_s``, in world ``xy``.

    ``swing_leg`` names the leg in the air for the interval being evaluated, and
    the triangle is then literally plan §13's "the other three legs" -- fixed
    for the whole swing.  Re-deciding who is airborne at *each* instant instead
    would swap the support set at the swing's closing sample, because that
    instant is the touchdown the next swing already owns, and the interval
    would end up reporting a triangle that was never the one under test.
    Passing ``None`` falls back to reading the airborne leg off the schedule,
    which is what a one-off query at a chosen instant wants.

    ``gamma = 0`` (requirement 8), so a leg's sagittal plane sits at its own
    mounting ``y`` and the lateral coordinate needs no model beyond Step 2's
    measured offsets.
    """

    if mounts is None:
        mounts = {m.leg: m.offset_body_xyz_m for m in leg_mounts_2d(GAMMA_RAD)}

    swing: LegId | None = swing_leg
    legs: list[LegId] = []
    points: list[tuple[float, float]] = []
    for leg in LEG_ORDER:
        if leg not in plan.plans or leg is swing_leg:
            continue
        scheduled = segment_at(plan.schedule.segments_of(leg), time_s)
        if scheduled is None:
            continue
        if scheduled.mode is LegMode.AIRBORNE:
            # With an explicit swing leg this means a *second* leg is in the
            # air, which is not a three-leg support at all.  Dropping it leaves
            # two contacts, and two contacts produce no margin -- reported, not
            # silently completed into a triangle.
            if swing_leg is None:
                swing = leg
            continue
        segment = plan.plans[leg].phased[scheduled.segment_index].segment
        span = scheduled.end_s - scheduled.start_s
        fraction = 0.0 if span <= 0.0 else (time_s - scheduled.start_s) / span
        offset = contact_offset_from_hip_m(segment, fraction)
        mount = mounts[leg]
        legs.append(leg)
        points.append((body_x_m + float(mount[0]) + offset, float(mount[1])))

    return SupportTriangle2D(
        time_s=float(time_s), swing_leg=swing, support_legs=tuple(legs),
        points_xy_m=np.array(points, dtype=float).reshape(-1, 2),
    )


def _sample_at(plan, mounts, body_x_m: float, time_s: float,
               swing_leg: LegId | None = None) -> StabilitySample2D:
    """One instant, with "not three supports" reported rather than forced."""

    triangle = support_triangle_at(plan, body_x_m, time_s,
                                   swing_leg=swing_leg, mounts=mounts)
    com = (body_x_m, 0.0)
    margin = (triangle.signed_margin_m(com)
              if len(triangle.support_legs) == 3 else None)
    return StabilitySample2D(
        time_s=float(time_s), swing_leg=triangle.swing_leg,
        support_legs=triangle.support_legs, com_xy_m=com,
        margin_m=margin, support_area_m2=triangle.area_m2,
        is_degenerate=triangle.is_degenerate,
    )


# --------------------------------------------------------------------------
# Sampling a swing
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class StabilitySample2D:
    """One instant: who is where, and how much margin there is."""

    time_s: float
    swing_leg: LegId | None
    support_legs: tuple[LegId, ...]
    com_xy_m: tuple[float, float]
    margin_m: float | None
    support_area_m2: float
    is_degenerate: bool

    @property
    def is_stable(self) -> bool:
        """A ``None`` margin is never stable: unknown is not the same as fine."""

        return self.margin_m is not None and self.margin_m > 0.0

    def as_dict(self) -> dict:
        return {
            "time_s": self.time_s,
            "swing_leg": None if self.swing_leg is None else self.swing_leg.value,
            "support_legs": ",".join(l.value for l in self.support_legs),
            "support_count": len(self.support_legs),
            "com_x_mm": self.com_xy_m[0] * 1e3,
            "com_y_mm": self.com_xy_m[1] * 1e3,
            "margin_mm": None if self.margin_m is None else self.margin_m * 1e3,
            "support_area_mm2": self.support_area_m2 * 1e6,
            "is_degenerate": self.is_degenerate,
            "is_stable": self.is_stable,
            "com_basis": COM_BASIS,
        }


@dataclass(frozen=True)
class SwingStability2D:
    """One airborne interval, evaluated across its whole span."""

    swing_leg: LegId
    segment_index: int
    segment_kind: str
    start_s: float
    end_s: float
    samples: tuple[StabilitySample2D, ...]
    margin_floor_m: float

    @property
    def support_legs(self) -> tuple[LegId, ...]:
        return self.samples[0].support_legs if self.samples else ()

    @property
    def minimum_margin_m(self) -> float | None:
        margins = [s.margin_m for s in self.samples if s.margin_m is not None]
        if not margins or any(s.margin_m is None for s in self.samples):
            # One unknown instant makes the minimum unknown; reporting the
            # minimum of the rest would overstate what was checked.
            return None if not margins else min(margins)
        return min(margins)

    @property
    def worst_time_s(self) -> float | None:
        worst = None
        for sample in self.samples:
            if sample.margin_m is None:
                continue
            if worst is None or sample.margin_m < worst[1]:
                worst = (sample.time_s, sample.margin_m)
        return None if worst is None else worst[0]

    @property
    def unknown_samples(self) -> int:
        return sum(1 for s in self.samples if s.margin_m is None)

    @property
    def is_stable(self) -> bool:
        """Every sampled instant clears the floor, and none is unknown."""

        if not self.samples or self.unknown_samples:
            return False
        return all(s.margin_m > self.margin_floor_m for s in self.samples)

    def as_dict(self) -> dict:
        minimum = self.minimum_margin_m
        return {
            "swing_leg": self.swing_leg.value,
            "support_legs": ",".join(l.value for l in self.support_legs),
            "segment_index": self.segment_index,
            "segment_kind": self.segment_kind,
            "start_s": self.start_s,
            "end_s": self.end_s,
            "samples": len(self.samples),
            "unknown_samples": self.unknown_samples,
            "minimum_stability_margin_mm": (
                None if minimum is None else minimum * 1e3),
            "worst_time_s": self.worst_time_s,
            "margin_floor_mm": self.margin_floor_m * 1e3,
            "stable": self.is_stable,
            "com_basis": COM_BASIS,
        }


@dataclass(frozen=True)
class TraversalStability2D:
    """Every swing in the run, plus the assumption the answer rests on."""

    swings: tuple[SwingStability2D, ...]
    margin_floor_m: float
    body_basis: str

    @property
    def minimum_margin_m(self) -> float | None:
        margins = [s.minimum_margin_m for s in self.swings
                   if s.minimum_margin_m is not None]
        return min(margins) if margins else None

    @property
    def worst_swing(self) -> SwingStability2D | None:
        candidates = [s for s in self.swings if s.minimum_margin_m is not None]
        if not candidates:
            return None
        return min(candidates, key=lambda s: s.minimum_margin_m)

    @property
    def unstable_swings(self) -> tuple[SwingStability2D, ...]:
        return tuple(s for s in self.swings if not s.is_stable)

    @property
    def is_stable(self) -> bool:
        return bool(self.swings) and not self.unstable_swings

    def as_dict(self) -> dict:
        minimum = self.minimum_margin_m
        worst = self.worst_swing
        return {
            "swings": len(self.swings),
            "unstable_swings": len(self.unstable_swings),
            "minimum_stability_margin_mm": (
                None if minimum is None else minimum * 1e3),
            "worst_swing_leg": None if worst is None else worst.swing_leg.value,
            "worst_time_s": None if worst is None else worst.worst_time_s,
            "margin_floor_mm": self.margin_floor_m * 1e3,
            "stable": self.is_stable,
            "gamma_deg": float(np.rad2deg(GAMMA_RAD)),
            "com_basis": COM_BASIS,
            "body_basis": self.body_basis,
        }


def swing_stability_2d(
    plan: FourLegPlan2D,
    trajectory: BodyTrajectory2D,
    *,
    margin_floor_m: float = DEFAULT_MARGIN_FLOOR_M,
    samples_per_swing: int = 21,
) -> TraversalStability2D:
    """Evaluate every airborne interval across its whole span.

    Plan §13 requirement 5: a rolling support leg's contact moves while the
    swing is in the air, so checking liftoff alone would miss the instant the
    triangle is narrowest.
    """

    mounts = {m.leg: m.offset_body_xyz_m for m in leg_mounts_2d(GAMMA_RAD)}
    times = trajectory.time_s
    body_x = trajectory.body_x_m
    lo, hi = plan.schedule.covered_interval_s

    swings: list[SwingStability2D] = []
    for leg in LEG_ORDER:
        if leg not in plan.plans:
            continue
        for scheduled in plan.schedule.segments_of(leg):
            if scheduled.mode is not LegMode.AIRBORNE:
                continue
            start = max(scheduled.start_s, lo)
            end = min(scheduled.end_s, hi)
            # A swing clipped to nothing by the covered interval is not a swing
            # that was checked; evaluating it would add a row of "n/a" that
            # reads as a failure rather than as an interval outside the run.
            if end - start <= BOUNDARY_TOLERANCE_S:
                continue

            # Half-open, matching who is actually in the air: at ``end`` the
            # next swing has already lifted off, so this swing's three-leg
            # support no longer exists there.  Sampling it would end every
            # swing with an unknown margin and make ``is_stable`` false for a
            # bookkeeping reason instead of a geometric one.
            grid = np.linspace(start, end, int(samples_per_swing) + 1)[:-1]
            samples: list[StabilitySample2D] = []
            for time_s in grid:
                # body_x comes from Step 5; body_z there is infeasible, which
                # is why the basis travels on the result.
                x = float(np.interp(time_s, times, body_x))
                samples.append(
                    _sample_at(plan, mounts, x, float(time_s), swing_leg=leg))

            swings.append(SwingStability2D(
                swing_leg=leg, segment_index=scheduled.segment_index,
                segment_kind=scheduled.segment_kind.value,
                start_s=float(start), end_s=float(end),
                samples=tuple(samples), margin_floor_m=float(margin_floor_m),
            ))

    body_basis = (
        f"body_x from Step 5; body_z INFEASIBLE (Step 5, "
        f"{len(trajectory.conflicts)} conflicts) -- {BODY_BASIS}"
        if not trajectory.is_feasible
        else f"body_x/body_z from Step 5 -- {BODY_BASIS}"
    )
    return TraversalStability2D(
        swings=tuple(swings), margin_floor_m=float(margin_floor_m),
        body_basis=body_basis,
    )


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------


def stability_rows(stability: TraversalStability2D) -> list[dict]:
    """Traversal summary, one row per swing, one row per sampled instant."""

    rows: list[dict] = [{"row_kind": "traversal", **stability.as_dict()}]
    for swing in stability.swings:
        rows.append({"row_kind": "swing", **swing.as_dict()})
    for swing in stability.swings:
        for sample in swing.samples:
            rows.append({"row_kind": "sample",
                         "segment_index": swing.segment_index, **sample.as_dict()})

    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]


def plot_stability_2d(
    plan: FourLegPlan2D,
    trajectory: BodyTrajectory2D,
    stability: TraversalStability2D,
    *,
    path=None,
):
    """Support triangle at the worst instant, and the margin over time."""

    import matplotlib.pyplot as plt

    fig, (left, right) = plt.subplots(1, 2, figsize=(12.0, 5.0),
                                      gridspec_kw={"width_ratios": [1.0, 1.4]})

    worst = stability.worst_swing
    if worst is not None and worst.worst_time_s is not None:
        time_s = worst.worst_time_s
        x = float(np.interp(time_s, trajectory.time_s, trajectory.body_x_m))
        triangle = support_triangle_at(plan, x, time_s)
        points = triangle.points_xy_m
        closed = np.vstack([points, points[:1]])
        left.plot(closed[:, 0] * 1e3, closed[:, 1] * 1e3, color="#2a6f4e",
                  lw=1.6, label="support triangle")
        left.fill(points[:, 0] * 1e3, points[:, 1] * 1e3, color="#2a6f4e",
                  alpha=0.12)
        for leg, point in zip(triangle.support_legs, points):
            left.scatter([point[0] * 1e3], [point[1] * 1e3], s=45,
                         color="#2a6f4e", zorder=3)
            left.annotate(leg.value, (point[0] * 1e3, point[1] * 1e3),
                          textcoords="offset points", xytext=(6, 6), fontsize=8)
        left.scatter([x * 1e3], [0.0], s=70, marker="x", color="#c5221f",
                     zorder=4, label="body centre (not CoM)")
        if triangle.swing_leg is not None:
            left.set_title(f"worst instant  t = {time_s:.3f} s   "
                           f"swing = {triangle.swing_leg.value}",
                           fontsize=9, loc="right")
        left.set_xlabel("x [mm]"); left.set_ylabel("y [mm]")
        left.legend(loc="upper left", fontsize=8, frameon=False)
        left.set_aspect("equal", adjustable="datalim")
        left.grid(alpha=0.25)

    colours = {leg: c for leg, c in zip(
        LEG_ORDER, ("#c5221f", "#b06000", "#2a6f4e", "#1f4e9c"))}
    seen: set = set()
    for swing in stability.swings:
        times = [s.time_s for s in swing.samples]
        margins = [np.nan if s.margin_m is None else s.margin_m * 1e3
                   for s in swing.samples]
        right.plot(times, margins, color=colours[swing.swing_leg], lw=1.6,
                   marker="o", ms=2.5,
                   label=(None if swing.swing_leg in seen
                          else f"swing {swing.swing_leg.value}"))
        seen.add(swing.swing_leg)
    right.axhline(stability.margin_floor_m * 1e3, color="#c5221f", lw=1.0,
                  ls="--", label="margin floor")
    right.axhline(0.0, color="#333333", lw=1.0)
    right.set_xlabel("time [s]"); right.set_ylabel("stability margin [mm]")
    right.set_title(f"{COM_BASIS}", fontsize=8, loc="right", pad=26)
    right.legend(loc="upper left", bbox_to_anchor=(0.0, 1.02), ncol=5,
                 fontsize=8, frameon=False)
    right.grid(alpha=0.25)

    fig.tight_layout()
    if path is not None:
        fig.savefig(path, dpi=150)
        plt.close(fig)
    return fig
