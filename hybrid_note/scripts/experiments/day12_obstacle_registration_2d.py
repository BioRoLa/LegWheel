"""Day 12 appendix B: where in the world does the planned crossing land?

Day 12 assembles a four-leg trajectory from two kinds of piece that were
generated in **different x frames**:

* the nominal cycles, whose ``hip_x`` starts at zero for every leg;
* the Day 10--11 crossing, whose segments carry absolute contacts in the
  composition frame -- the one whose obstacle starts at
  :data:`COMPOSER_FRAME_X_START_M`.

Step 5 says so in as many words and takes the only safe route: ``body_x`` is
integrated from the **increments** of the stance legs, because "the absolute
values do not share an origin".  That keeps the body continuous, and it throws
the crossing's own frame away.  Nothing downstream puts it back: the terrain
reaches :func:`plan_terrain_2d` as a *decision* input only, and Step 8 places
every contact relative to the hip.

So the assembled trajectory contains ``SWING_UP`` and ``SWING_DOWN`` **without
containing an obstacle**, and asking "does the planned crossing land on the
platform?" has no answer until the frames are lined up.  This module lines them
up and reports what comes out.

The measurement
---------------

One crossing segment knows two things about its own frame: the hip position it
starts at, and the hip position it ends at.  The assembled trajectory knows
where that leg's hip actually is at those two instants.  The difference is an
offset, and the offset carries the composition frame's obstacle into the world:

    implied_x_start = world_hip_x - local_hip_x + COMPOSER_FRAME_X_START_M

Evaluate it at the segment's entry and again at its exit.  If the crossing
fits the schedule it was given, the two agree and every leg agrees with every
other leg, and then there is one platform.  What actually comes out is in the
Day 12 log; the module states no verdict of its own beyond
:attr:`RegistrationReport2D.is_registrable`, which is a comparison against a
tolerance the caller can see and change.

**This module plans nothing and repairs nothing.**  It is a measurement of an
inconsistency that already exists in the frozen data.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np

from hybrid_note.scripts.experiments.day10_11_composer_2d import (
    COMPOSER_FRAME_X_START_M,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import SegmentKind
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    BodyTrajectory2D,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    LegId,
    leg_mounts_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import GAMMA_RAD
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import LegMode
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    FourLegPlan2D,
    TransitionPhase,
)

#: The pipeline's own contact tolerance (Day 10--11 scenes, Day 12 postures).
#: Used as the default agreement bound so that "registrable" means the same
#: thing here as "in contact" does everywhere else, rather than a number
#: invented for this measurement.
REGISTRATION_TOLERANCE_M: float = 1e-3

PHASE_COLORS: dict[str, str] = {
    TransitionPhase.ASCENT.value: "#1a4d8f",
    TransitionPhase.ON_TOP.value: "#7a3f9d",
    TransitionPhase.DESCENT.value: "#b06000",
    TransitionPhase.OVER.value: "#2a6f4e",
}


@dataclass(frozen=True)
class CrossingRegistration2D:
    """One crossing segment, and the obstacle position it implies."""

    leg: LegId
    segment_index: int
    kind: SegmentKind
    phase: TransitionPhase
    start_s: float
    end_s: float
    #: The segment's own frame.
    local_hip_start_m: float
    local_hip_end_m: float
    #: Where the assembled trajectory actually puts that hip.
    world_hip_start_m: float
    world_hip_end_m: float
    frame_x_start_m: float = COMPOSER_FRAME_X_START_M

    @property
    def demanded_advance_m(self) -> float:
        """How far the segment's own frame moves the hip."""

        return self.local_hip_end_m - self.local_hip_start_m

    @property
    def delivered_advance_m(self) -> float:
        """How far the schedule actually moves it in the same window."""

        return self.world_hip_end_m - self.world_hip_start_m

    @property
    def advance_ratio(self) -> float | None:
        if abs(self.delivered_advance_m) < 1e-12:
            return None
        return self.demanded_advance_m / self.delivered_advance_m

    @property
    def implied_x_start_entry_m(self) -> float:
        return (self.world_hip_start_m - self.local_hip_start_m
                + self.frame_x_start_m)

    @property
    def implied_x_start_exit_m(self) -> float:
        return (self.world_hip_end_m - self.local_hip_end_m
                + self.frame_x_start_m)

    @property
    def drift_m(self) -> float:
        """How much the implied obstacle moves *during* the segment.

        Zero would mean the body advanced exactly as much as the segment's own
        frame assumes.  Anything else is the same mismatch seen as a distance:
        the obstacle cannot be in two places, so the segment and the schedule
        disagree about one of them.
        """

        return self.implied_x_start_exit_m - self.implied_x_start_entry_m

    def as_dict(self) -> dict:
        ratio = self.advance_ratio
        return {
            "leg": self.leg.value,
            "segment_index": self.segment_index,
            "kind": self.kind.value,
            "phase": self.phase.value,
            "start_s": self.start_s,
            "end_s": self.end_s,
            "local_hip_start_mm": self.local_hip_start_m * 1e3,
            "local_hip_end_mm": self.local_hip_end_m * 1e3,
            "world_hip_start_mm": self.world_hip_start_m * 1e3,
            "world_hip_end_mm": self.world_hip_end_m * 1e3,
            "demanded_advance_mm": self.demanded_advance_m * 1e3,
            "delivered_advance_mm": self.delivered_advance_m * 1e3,
            "advance_ratio": None if ratio is None else ratio,
            "implied_x_start_entry_mm": self.implied_x_start_entry_m * 1e3,
            "implied_x_start_exit_mm": self.implied_x_start_exit_m * 1e3,
            "drift_mm": self.drift_m * 1e3,
            "frame_x_start_mm": self.frame_x_start_m * 1e3,
        }


@dataclass(frozen=True)
class RegistrationReport2D:
    """Every crossing segment's implied obstacle, and how far apart they are."""

    registrations: tuple[CrossingRegistration2D, ...]
    #: The terrain the run was planned *for*.  ``None`` for a flat run, which
    #: has no crossing and therefore nothing to register.
    terrain: SharedTerrainSpec2D | None
    #: Scheduled time with a foot in **stance** on a raised surface.  A
    #: crossing that never stands on the obstacle reads 0.0 s, and that is a
    #: fact about the plan rather than about this measurement.
    stance_on_top_s: float = 0.0
    tolerance_m: float = REGISTRATION_TOLERANCE_M

    @property
    def implied_x_starts_m(self) -> tuple[float, ...]:
        return tuple(r.implied_x_start_entry_m for r in self.registrations)

    @property
    def spread_m(self) -> float:
        """How far apart the implied obstacles are, over every segment."""

        values = self.implied_x_starts_m
        return 0.0 if len(values) < 2 else max(values) - min(values)

    @property
    def worst_drift_m(self) -> float:
        return max((abs(r.drift_m) for r in self.registrations), default=0.0)

    @property
    def worst_advance_ratio(self) -> float | None:
        ratios = [abs(r.advance_ratio) for r in self.registrations
                  if r.advance_ratio is not None]
        return max(ratios) if ratios else None

    def of_leg(self, leg: LegId) -> tuple[CrossingRegistration2D, ...]:
        return tuple(r for r in self.registrations if r.leg is leg)

    def within_leg_gap_m(self, leg: LegId) -> float:
        """One leg's own disagreement -- its ascent against its descent."""

        values = [r.implied_x_start_entry_m for r in self.of_leg(leg)]
        return 0.0 if len(values) < 2 else max(values) - min(values)

    @property
    def worst_within_leg_gap_m(self) -> float:
        return max((self.within_leg_gap_m(leg) for leg in LEG_ORDER),
                   default=0.0)

    @property
    def is_registrable(self) -> bool:
        """One obstacle position fits every crossing segment, to tolerance."""

        if not self.registrations:
            return False
        return (self.spread_m <= self.tolerance_m
                and self.worst_drift_m <= self.tolerance_m)

    @property
    def distance_to_planned_terrain_m(self) -> float | None:
        """From the terrain the run was planned for, to the nearest implied
        obstacle.  ``None`` on flat ground, where there is no terrain."""

        if self.terrain is None or not self.registrations:
            return None
        target = float(self.terrain.x_start_m)
        return min(abs(value - target) for value in self.implied_x_starts_m)

    def as_dict(self) -> dict:
        return {
            "crossing_segments": len(self.registrations),
            "is_registrable": self.is_registrable,
            "tolerance_mm": self.tolerance_m * 1e3,
            "implied_spread_mm": self.spread_m * 1e3,
            "worst_within_leg_gap_mm": self.worst_within_leg_gap_m * 1e3,
            "worst_drift_mm": self.worst_drift_m * 1e3,
            "worst_advance_ratio": self.worst_advance_ratio,
            "planned_terrain_x_start_mm": (
                None if self.terrain is None
                else float(self.terrain.x_start_m) * 1e3),
            "planned_terrain_top_length_mm": (
                None if self.terrain is None
                else float(self.terrain.top_length_m) * 1e3),
            "distance_to_planned_terrain_mm": (
                None if self.distance_to_planned_terrain_m is None
                else self.distance_to_planned_terrain_m * 1e3),
            "composition_frame_x_start_mm": COMPOSER_FRAME_X_START_M * 1e3,
            "stance_on_top_s": self.stance_on_top_s,
        }


def crossing_registrations_2d(
    plan: FourLegPlan2D,
    body: BodyTrajectory2D,
    *,
    frame_x_start_m: float = COMPOSER_FRAME_X_START_M,
) -> tuple[CrossingRegistration2D, ...]:
    """Line up every crossing segment's own frame with the assembled world.

    Which segments are "the crossing" is read from
    :attr:`TransitionPhase.is_transition`, not from a list of kinds: Day 12
    already maps every kind to a phase, and a second list would be a second
    place to forget a strategy.
    """

    mounts = {m.leg: m.offset_body_xyz_m for m in leg_mounts_2d(GAMMA_RAD)}
    times, body_x = body.time_s, body.body_x_m

    out: list[CrossingRegistration2D] = []
    for leg in LEG_ORDER:
        if leg not in plan.plans:
            continue
        leg_plan = plan.plans[leg]
        for scheduled in plan.schedule.segments_of(leg):
            phased = leg_plan.phased[scheduled.segment_index]
            if not phased.phase.is_transition:
                continue
            segment = phased.segment
            mount_x = float(mounts[leg][0])
            out.append(CrossingRegistration2D(
                leg=leg,
                segment_index=scheduled.segment_index,
                kind=segment.kind,
                phase=phased.phase,
                start_s=float(scheduled.start_s),
                end_s=float(scheduled.end_s),
                local_hip_start_m=float(segment.start_contact.hip_xz_m[0]),
                local_hip_end_m=float(segment.end_contact.hip_xz_m[0]),
                world_hip_start_m=float(
                    np.interp(scheduled.start_s, times, body_x)) + mount_x,
                world_hip_end_m=float(
                    np.interp(scheduled.end_s, times, body_x)) + mount_x,
                frame_x_start_m=float(frame_x_start_m),
            ))
    return tuple(out)


def stance_on_top_seconds_2d(
    plan: FourLegPlan2D, *, tolerance_m: float = REGISTRATION_TOLERANCE_M,
) -> float:
    """How long any leg is scheduled to **stand** on a raised surface.

    Read off the schedule and the segments' own contacts, so a crossing whose
    weight-bearing segment on the obstacle was never planned -- Step 7's
    unresolved ``TOP_REPOSITION`` is exactly that -- comes out as 0.0 s rather
    than as an absence nobody looked for.
    """

    total = 0.0
    for leg in LEG_ORDER:
        if leg not in plan.plans:
            continue
        for scheduled in plan.schedule.segments_of(leg):
            if scheduled.mode is not LegMode.STANCE:
                continue
            segment = plan.plans[leg].phased[scheduled.segment_index].segment
            raised = max(float(segment.start_contact.point_world_xz_m[1]),
                         float(segment.end_contact.point_world_xz_m[1]))
            if raised > tolerance_m:
                total += float(scheduled.end_s - scheduled.start_s)
    return total


def registration_report_2d(
    plan: FourLegPlan2D,
    body: BodyTrajectory2D,
    terrain: SharedTerrainSpec2D | None,
    *,
    frame_x_start_m: float = COMPOSER_FRAME_X_START_M,
    tolerance_m: float = REGISTRATION_TOLERANCE_M,
) -> RegistrationReport2D:
    return RegistrationReport2D(
        registrations=crossing_registrations_2d(
            plan, body, frame_x_start_m=frame_x_start_m),
        terrain=terrain,
        stance_on_top_s=stance_on_top_seconds_2d(plan, tolerance_m=tolerance_m),
        tolerance_m=float(tolerance_m),
    )


def registration_rows(report: RegistrationReport2D) -> list[dict]:
    rows: list[dict] = [{"row_kind": "summary", **report.as_dict()}]
    for leg in LEG_ORDER:
        if not report.of_leg(leg):
            continue
        rows.append({"row_kind": "leg", "leg": leg.value,
                     "crossing_segments": len(report.of_leg(leg)),
                     "within_leg_gap_mm": report.within_leg_gap_m(leg) * 1e3})
    rows.extend({"row_kind": "segment", **r.as_dict()}
                for r in report.registrations)

    # One header for every row kind, as Step 8 writes its table.
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]


# --------------------------------------------------------------------------
# The chart
# --------------------------------------------------------------------------


def plot_registration_2d(report: RegistrationReport2D, path: Path) -> None:
    """One row per crossing segment, on a world-x axis.

    A bar is where that segment thinks the obstacle is when it *starts*; the
    hollow bar is where the same segment thinks it is when it *ends*.  If the
    plan placed one obstacle, every bar would be the same bar.
    """

    if not report.registrations:
        raise ValueError("nothing to plot: this run has no crossing segments.")

    top_length_m = (0.0 if report.terrain is None
                    else float(report.terrain.top_length_m))
    figure, ax = plt.subplots(
        figsize=(13.0, 0.68 * len(report.registrations) + 3.0))

    for row, registration in enumerate(report.registrations):
        colour = PHASE_COLORS.get(registration.phase.value, "#4a5568")
        entry = registration.implied_x_start_entry_m * 1e3
        exit_ = registration.implied_x_start_exit_m * 1e3
        width = top_length_m * 1e3
        ax.barh(row, width, left=entry, height=0.52, color=colour, alpha=0.75,
                edgecolor="white", zorder=3)
        ax.barh(row, width, left=exit_, height=0.52, facecolor="none",
                edgecolor=colour, lw=1.3, ls="--", zorder=4)
        ax.annotate("", xy=(exit_, row), xytext=(entry, row),
                    arrowprops=dict(arrowstyle="->", color=colour, lw=1.2),
                    zorder=5)
        # Over the arrow, not beside the bar: a row whose bars sit at the
        # left edge would otherwise write its label on top of the tick labels.
        ax.text(0.5 * (entry + exit_), row - 0.36,
                f"drift {registration.drift_m * 1e3:+.0f} mm",
                va="center", ha="center", fontsize=8.5, color=colour, zorder=6)

    if report.terrain is not None:
        start = float(report.terrain.x_start_m) * 1e3
        ax.axvspan(start, start + top_length_m * 1e3, color="#8a8577",
                   alpha=0.30, zorder=1)
        ax.text(start + top_length_m * 1e3 - 10.0, -0.75,
                f"the platform this run was planned for  "
                f"({start:.0f} .. {start + top_length_m * 1e3:.0f} mm)",
                fontsize=9, color="#5a5648", va="center", ha="right", zorder=6)

    # The numbers ride on the tick labels rather than beside the bars: text
    # placed in data coordinates runs off the frame as soon as a bar moves.
    labels = []
    for registration in report.registrations:
        ratio = registration.advance_ratio
        labels.append(
            f"{registration.leg.value}  {registration.kind.value}"
            f"   t {registration.start_s:.2f}-{registration.end_s:.2f} s\n"
            f"needs {registration.demanded_advance_m * 1e3:.0f} mm of body "
            f"advance, gets {registration.delivered_advance_m * 1e3:.0f}"
            + ("" if ratio is None else f"   ({ratio:.1f}x)"))
    ax.set_yticks(range(len(report.registrations)))
    ax.set_yticklabels(labels, fontsize=8.5)
    ax.set_ylim(len(report.registrations) - 0.4, -1.05)
    ax.set_xlabel("world x  [mm]", fontsize=9)
    ax.grid(axis="x", color="#ddd", lw=0.6, zorder=0)
    ax.set_axisbelow(True)

    summary = report.as_dict()
    figure.suptitle(
        "Day 12 appendix B -- where each planned crossing thinks the obstacle is\n"
        f"solid = implied at segment entry, dashed = at its exit;  "
        f"spread {summary['implied_spread_mm']:.0f} mm,  "
        f"worst one leg disagreeing with itself "
        f"{summary['worst_within_leg_gap_mm']:.0f} mm,  "
        f"registrable = {summary['is_registrable']}",
        fontsize=11,
    )
    figure.tight_layout(rect=(0, 0, 1, 0.93))
    figure.savefig(path, dpi=140)
    plt.close(figure)
