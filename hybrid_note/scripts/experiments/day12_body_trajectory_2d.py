"""Day 12 Step 5: one body trajectory from the four legs' body requirements.

Plan §12.  Day 10--11 said what each leg needs of the body; Step 4 put those
needs on a common clock.  This step merges them into a single ``body_x(t)`` /
``body_z(t)`` -- **deterministically**, with no weighted optimizer (plan §12
requirement 4), and refusing rather than averaging when two hard requirements
disagree (requirement 5).

**The output is a body-frame quantity, not a whole-robot CoM.**  This 2D
pipeline has no mass model, so every row is labelled
``quasi-static body-frame approximation``: plan §12 names that distinction
explicitly, and writing "CoM" here would promote an approximation into a
measurement.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Sequence

import numpy as np

from legwheel.config import RobotParams

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirement2D,
    BodyRequirementKind,
    MotionSegment2D,
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
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    FourLegPlan2D,
    TransitionPhase,
)

#: What every body row is, and is not.  Plan §12 asks for this label in the
#: log and the paper note; carrying it on the data means it cannot be dropped
#: on the way there.
BODY_BASIS = "quasi-static body-frame approximation (no whole-robot CoM model)"

#: The leg planes hang this far *above* the body origin (Step 2, trap 17), so
#: a hip-height requirement is a body-height requirement shifted by it.
HIP_TO_BODY_Z_M: float = float(RobotParams.ABAD_AXIS_OFFSET)

#: Two hard requirements closer than this are the same requirement seen twice.
#: One rolling step of Day 6--7's traversal moves the hip 6.62 mm, so a
#: millimetre is comfortably inside "the same instant", and well below any
#: disagreement worth calling a conflict.
HARD_AGREEMENT_M: float = 1e-3


class BodyDriver(str, Enum):
    """Why ``body_z`` is where it is at one sample."""

    HARD = "hard"
    LOWER_BOUND = "lower_bound"
    NOMINAL = "nominal"
    INFEASIBLE = "infeasible"


# --------------------------------------------------------------------------
# Reading one leg's demand at one instant
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class LegDemand2D:
    """What one leg asks of the body at one time, and where the ask came from."""

    leg: LegId
    segment_index: int
    segment_kind: str
    phase: TransitionPhase | None
    kind: BodyRequirementKind
    #: The body height this demands.  A bound for LOWER_BOUND, a value for HARD.
    body_z_m: float
    mode: LegMode

    @property
    def is_hard(self) -> bool:
        return self.kind in (BodyRequirementKind.TRACK, BodyRequirementKind.PINNED)

    def as_dict(self) -> dict:
        return {
            "leg": self.leg.value,
            "segment_index": self.segment_index,
            "segment_kind": self.segment_kind,
            "phase": None if self.phase is None else self.phase.value,
            "body_kind": self.kind.value,
            "is_hard": self.is_hard,
            "body_z_mm": self.body_z_m * 1e3,
            "mode": self.mode.value,
        }


def _profile_at(requirement: BodyRequirement2D, fraction: float) -> float | None:
    """Sample a TRACK profile by position along the segment.

    The profile is indexed by the segment's own frames, and Step 3 gave the
    segment a window in proportion to those frames, so a linear read across the
    window is the same traversal the frames describe -- not a re-timing.
    """

    profile = requirement.hip_z_profile_m
    if profile is None or len(profile) == 0:
        return None
    if len(profile) == 1:
        return float(profile[0])
    position = float(np.clip(fraction, 0.0, 1.0)) * (len(profile) - 1)
    low = int(np.floor(position))
    high = min(low + 1, len(profile) - 1)
    weight = position - low
    return float(profile[low] * (1.0 - weight) + profile[high] * weight)


def leg_demand_at(
    scheduled: ScheduledSegment2D,
    segment: MotionSegment2D,
    time_s: float,
    *,
    phase: TransitionPhase | None = None,
) -> LegDemand2D | None:
    """One leg's demand at ``time_s``, or ``None`` when it asks for nothing."""

    requirement = segment.body_requirement
    if requirement is None or requirement.kind is BodyRequirementKind.NONE:
        return None

    span = scheduled.end_s - scheduled.start_s
    fraction = 0.0 if span <= 0.0 else (time_s - scheduled.start_s) / span

    if requirement.kind is BodyRequirementKind.TRACK:
        hip_z = _profile_at(requirement, fraction)
    elif requirement.kind is BodyRequirementKind.PINNED:
        # Only the endpoints are hard on a PINNED segment (Day 10-11 Step 9
        # section C); the interior is not, so it is not reported as one.  The
        # height itself is the requirement's own ``hip_z_min_m`` -- on a PINNED
        # requirement that field is a value, not a floor.
        if 0.0 < fraction < 1.0:
            return None
        hip_z = requirement.hip_z_min_m
    else:
        hip_z = requirement.hip_z_min_m
    if hip_z is None:
        return None

    return LegDemand2D(
        leg=scheduled.leg, segment_index=scheduled.segment_index,
        segment_kind=scheduled.segment_kind.value, phase=phase,
        kind=requirement.kind, body_z_m=float(hip_z) - HIP_TO_BODY_Z_M,
        mode=scheduled.mode,
    )


# --------------------------------------------------------------------------
# The merge
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class BodyConflict2D:
    """Two hard requirements that cannot both hold.  Plan §12 requirement 5."""

    time_s: float
    leg_a: LegId
    leg_b: LegId
    body_z_a_m: float
    body_z_b_m: float

    @property
    def disagreement_m(self) -> float:
        return abs(float(self.body_z_a_m - self.body_z_b_m))

    def as_dict(self) -> dict:
        return {
            "time_s": self.time_s,
            "leg_a": self.leg_a.value, "leg_b": self.leg_b.value,
            "body_z_a_mm": self.body_z_a_m * 1e3,
            "body_z_b_mm": self.body_z_b_m * 1e3,
            "disagreement_mm": self.disagreement_m * 1e3,
        }


@dataclass(frozen=True)
class BodySample2D:
    """One instant of the body trajectory, with the reason for its height."""

    time_s: float
    body_x_m: float
    body_z_m: float
    driver: BodyDriver
    driver_leg: LegId | None
    driver_segment_index: int | None
    driver_segment_kind: str | None
    active_hard: int
    active_lower_bounds: int
    #: Every leg that spoke at this instant, kept so a reader can see what was
    #: merged rather than only what won.
    demands: tuple[LegDemand2D, ...] = ()

    def as_dict(self) -> dict:
        return {
            "time_s": self.time_s,
            "body_x_mm": self.body_x_m * 1e3,
            "body_z_mm": self.body_z_m * 1e3,
            "driver": self.driver.value,
            "driver_leg": None if self.driver_leg is None else self.driver_leg.value,
            "driver_segment_index": self.driver_segment_index,
            "driver_segment_kind": self.driver_segment_kind,
            "active_hard": self.active_hard,
            "active_lower_bounds": self.active_lower_bounds,
            "basis": BODY_BASIS,
        }


@dataclass(frozen=True)
class BodyTrajectory2D:
    """``body_x(t)`` and ``body_z(t)`` for all four legs at once."""

    samples: tuple[BodySample2D, ...]
    conflicts: tuple[BodyConflict2D, ...]
    nominal_body_z_m: float
    #: Worst disagreement between the stance legs about where the body is,
    #: over the run.  ``None`` unless the plan was world-registered, because
    #: without a shared origin the four absolute positions are not comparable
    #: and a number here would only invite reading one anyway.
    world_x_spread_m: float | None = None
    #: Plan §12 requirement 6: the symmetric first test leaves these alone, and
    #: they are carried on the trajectory so "unchanged" is checkable, not
    #: assumed.
    body_y_m: float = 0.0
    body_rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def is_feasible(self) -> bool:
        return not self.conflicts

    @property
    def body_z_m(self) -> np.ndarray:
        return np.array([s.body_z_m for s in self.samples], dtype=float)

    @property
    def body_x_m(self) -> np.ndarray:
        return np.array([s.body_x_m for s in self.samples], dtype=float)

    @property
    def time_s(self) -> np.ndarray:
        return np.array([s.time_s for s in self.samples], dtype=float)

    @property
    def body_z_travel_m(self) -> float:
        """Plan §12 requirement 8: the vertical variation, for evaluation."""

        heights = self.body_z_m
        heights = heights[np.isfinite(heights)]
        if heights.size == 0:
            return 0.0
        return float(heights.max() - heights.min())

    @property
    def max_body_z_step_m(self) -> float:
        """The largest jump between neighbouring samples -- requirement 6."""

        steps = np.abs(np.diff(self.body_z_m))
        steps = steps[np.isfinite(steps)]
        if steps.size == 0:
            return 0.0
        return float(steps.max())

    def concession_intervals(self) -> tuple[tuple[float, float, LegId], ...]:
        """The spans where a leg, not the nominal posture, set ``body_z``.

        This is plan §12's acceptance in one call: every body_z change can be
        named with the leg and the segment that asked for it.
        """

        out: list[tuple[float, float, LegId]] = []
        for sample in self.samples:
            if sample.driver is BodyDriver.NOMINAL or sample.driver_leg is None:
                continue
            if out and out[-1][2] is sample.driver_leg:
                out[-1] = (out[-1][0], sample.time_s, out[-1][2])
            else:
                out.append((sample.time_s, sample.time_s, sample.driver_leg))
        return tuple(out)

    def _finite_extreme(self, reduce) -> float | None:
        heights = self.body_z_m
        heights = heights[np.isfinite(heights)]
        return None if heights.size == 0 else float(reduce(heights) * 1e3)

    def as_dict(self) -> dict:
        return {
            "samples": len(self.samples),
            "nominal_body_z_mm": self.nominal_body_z_m * 1e3,
            "body_z_min_mm": self._finite_extreme(np.min),
            "body_z_max_mm": self._finite_extreme(np.max),
            "feasible_samples": int(np.isfinite(self.body_z_m).sum()),
            "max_disagreement_mm": (
                max((c.disagreement_m for c in self.conflicts), default=0.0) * 1e3),
            "body_z_travel_mm": self.body_z_travel_m * 1e3,
            "max_body_z_step_mm": self.max_body_z_step_m * 1e3,
            "body_y_mm": self.body_y_m * 1e3,
            "body_roll_deg": float(np.rad2deg(self.body_rpy_rad[0])),
            "body_pitch_deg": float(np.rad2deg(self.body_rpy_rad[1])),
            "body_yaw_deg": float(np.rad2deg(self.body_rpy_rad[2])),
            "conflict_count": len(self.conflicts),
            "is_feasible": self.is_feasible,
            "basis": BODY_BASIS,
        }


#: The largest body pitch this model will fit, in radians.
#:
#: Day 13.  A rigid body may tilt, so two legs wanting different body heights
#: is a slope, not a conflict; the project owner removed the level-body
#: restriction on 2026-09-07 ("why force the body to be level?").
#:
#: The bound is derived rather than chosen.  A front leg standing on an
#: obstacle of height ``h`` while a rear leg is still on the ground tilts the
#: body by ``arctan(h / wheelbase)``, so the pitch a crossing asks for is fixed
#: by the geometry: 4.5 deg at 40 mm, 11.1 at 100, **15.35 at 140**.  A first
#: version capped this at a round 15 deg and refused 140 mm by 0.35 deg, which
#: is a limit inventing an infeasibility rather than reporting one.
#:
#: So the cap is the pitch the tallest swept obstacle implies, plus a little:
#: 200 mm over the 510 mm wheelbase is 21.4 deg, and 25 deg covers it while
#: still refusing a fit that has gone wrong.  It is a guard on the *fit*, not a
#: claim about the machine -- what the robot can actually tolerate is a
#: question about mass and friction that this model does not carry.
MAX_BODY_PITCH_RAD: float = float(np.deg2rad(25.0))

#: How far a leg's demand may sit off the fitted plane and still be on it.
#:
#: Same role as ``HARD_AGREEMENT_M`` had for a single height: below this the
#: demands are one plane seen four times.  Above it they are not a plane at
#: all -- no pitch satisfies them -- and that is a real conflict.
PLANE_RESIDUAL_M: float = 2e-3


def _pitched_body_2d(hard: Sequence[LegDemand2D], time_s: float):
    """Fit ``body_z(x) = z0 + x * tan(pitch)`` through the hard demands.

    Returns ``(body_z_at_origin, pitch_rad, driver)`` when a plane explains
    them, or ``None`` when it does not and the caller should report a conflict.

    ``None`` is returned rather than a bad fit in three cases, each of which a
    pitch genuinely cannot resolve: fewer than two distinct mount positions to
    fit through, a residual larger than :data:`PLANE_RESIDUAL_M`, and a pitch
    beyond :data:`MAX_BODY_PITCH_RAD`.
    """

    mounts = {m.leg: float(m.offset_body_xyz_m[0])
              for m in leg_mounts_2d(0.0)}
    xs = np.array([mounts[d.leg] for d in hard], dtype=float)
    zs = np.array([d.body_z_m for d in hard], dtype=float)
    if len(hard) == 1:
        return float(zs[0]), 0.0, BodyDriver.HARD
    if np.ptp(xs) < 1e-9:
        # Same mount x: no pitch can separate them, so this is the one case
        # that stays a conflict on height alone.
        return None if np.ptp(zs) > HARD_AGREEMENT_M else (
            float(zs.mean()), 0.0, BodyDriver.HARD)
    slope, intercept = np.polyfit(xs, zs, 1)
    residual = float(np.max(np.abs(zs - (slope * xs + intercept))))
    if residual > PLANE_RESIDUAL_M:
        return None
    pitch = float(np.arctan(slope))
    if abs(pitch) > MAX_BODY_PITCH_RAD:
        return None
    return float(intercept), pitch, BodyDriver.HARD


def merge_demands(
    demands: Sequence[LegDemand2D], nominal_body_z_m: float, time_s: float,
) -> tuple[float, BodyDriver, LegDemand2D | None, tuple[BodyConflict2D, ...]]:
    """The whole merge rule, in one place.

    Hard requirements win outright; two that disagree are a refusal, never an
    average (plan §12 requirement 5).  Otherwise the body sits at the highest
    active lower bound, or at nominal when nothing asks -- "the minimum body
    motion that satisfies all active legs" (requirement 3).
    """

    hard = [d for d in demands if d.is_hard]
    if hard:
        # Two legs asking for different body heights is a **slope**, not a
        # conflict.  The body is a rigid frame that can pitch: four hips define
        # a plane, and a front leg on an obstacle with a rear leg still on the
        # ground is exactly the case a plane describes and a single height
        # cannot.  Day 12 pinned ``body_rpy`` to zero and so had to call the
        # disagreement infeasible; the project owner removed that restriction
        # on 2026-09-07 ("why force the body to be level?").
        #
        # So fit a line through the demands against each leg's mount x and let
        # the body pitch.  What remains a conflict is what a *plane cannot
        # satisfy*: legs at the same mount x asking for different heights,
        # which no pitch can reconcile, and a residual too large to be a fit.
        pitched = _pitched_body_2d(hard, time_s)
        if pitched is not None:
            body_z, pitch_rad, driver = pitched
            return body_z, driver, hard[0], ()
        conflicts = tuple(
            BodyConflict2D(time_s, hard[0].leg, other.leg,
                           hard[0].body_z_m, other.body_z_m)
            for other in hard[1:]
            if abs(other.body_z_m - hard[0].body_z_m) > HARD_AGREEMENT_M
        )
        if conflicts:
            return float("nan"), BodyDriver.INFEASIBLE, None, conflicts
        return hard[0].body_z_m, BodyDriver.HARD, hard[0], ()

    bounds = [d for d in demands if d.kind is BodyRequirementKind.LOWER_BOUND]
    if bounds:
        highest = max(bounds, key=lambda d: d.body_z_m)
        if highest.body_z_m > nominal_body_z_m:
            return highest.body_z_m, BodyDriver.LOWER_BOUND, highest, ()
    return nominal_body_z_m, BodyDriver.NOMINAL, None, ()


def _hip_x_at(scheduled: ScheduledSegment2D, segment: MotionSegment2D,
              time_s: float) -> float:
    span = scheduled.end_s - scheduled.start_s
    fraction = 0.0 if span <= 0.0 else float(
        np.clip((time_s - scheduled.start_s) / span, 0.0, 1.0))
    start = float(segment.start_contact.hip_xz_m[0])
    end = float(segment.end_contact.hip_xz_m[0])
    return start + fraction * (end - start)


def body_trajectory_2d(
    plan: FourLegPlan2D,
    *,
    nominal_body_z_m: float,
    samples: int = 241,
    body_x_start_m: float = 0.0,
    world_registered: bool = False,
) -> BodyTrajectory2D:
    """Merge the four legs' requirements over the interval all four cover.

    ``body_x`` is integrated from the **advance of the stance legs**, not read
    off their absolute ``hip_x``: Step 4 gives every leg the same chain, whose
    ``hip_x`` starts at zero, so the absolute values do not share an origin.
    The increments do, and using them keeps ``body_x`` continuous without
    quietly averaging four disagreeing absolute positions.

    ``world_registered`` says that premise no longer holds -- the plan came
    from ``world_leg_plans_2d``, where each leg's chain was placed at that
    leg's own position in the world, so the absolute values **do** share an
    origin and integrating increments would throw that origin away again
    (problem C4, log 1.13).  Then ``body_x`` is read off the stance legs
    directly, and how far the legs disagree about it is measured rather than
    averaged into silence: see :attr:`BodyTrajectory2D.world_x_spread_m`.
    """

    lo, hi = plan.schedule.covered_interval_s
    grid = np.linspace(lo, hi, int(samples))

    by_leg: dict[LegId, tuple[ScheduledSegment2D, ...]] = {
        leg: tuple(plan.schedule.segments_of(leg)) for leg in plan.plans
    }

    mounts = {m.leg: float(m.offset_body_xyz_m[0])
              for m in leg_mounts_2d(0.0)}

    out: list[BodySample2D] = []
    conflicts: list[BodyConflict2D] = []
    body_x = float(body_x_start_m)
    previous_hip_x: dict[LegId, float] = {}
    world_spreads: list[float] = []

    for time_s in grid:
        demands: list[LegDemand2D] = []
        advances: list[float] = []
        absolute: list[float] = []
        for leg, scheduled_all in by_leg.items():
            leg_plan = plan.plans[leg]
            active = [s for s in scheduled_all
                      if s.start_s <= time_s <= s.end_s]
            if not active:
                previous_hip_x.pop(leg, None)
                continue
            scheduled = active[0]
            phased = leg_plan.phased[scheduled.segment_index]
            demand = leg_demand_at(scheduled, phased.segment, float(time_s),
                                   phase=phased.phase)
            if demand is not None:
                demands.append(demand)

            hip_x = _hip_x_at(scheduled, phased.segment, float(time_s))
            if scheduled.mode is LegMode.STANCE:
                if leg in previous_hip_x:
                    advances.append(hip_x - previous_hip_x[leg])
                previous_hip_x[leg] = hip_x
                # What this stance leg says the body's x is, if the chains
                # really do share a world origin.
                absolute.append(hip_x - mounts[leg])
            else:
                previous_hip_x.pop(leg, None)

        body_z, driver, winner, found = merge_demands(
            demands, nominal_body_z_m, float(time_s))
        conflicts.extend(found)
        if world_registered and absolute:
            # The legs are supposed to agree; the median is what to use and
            # the spread is what says whether trusting it was warranted.
            body_x = float(np.median(absolute))
            world_spreads.append(float(max(absolute) - min(absolute)))
        else:
            # A stance leg rolls the body forward; a leg that just lifted off
            # contributes nothing, which is why only STANCE increments are used.
            body_x += float(np.median(advances)) if advances else 0.0

        out.append(BodySample2D(
            time_s=float(time_s), body_x_m=body_x, body_z_m=body_z,
            driver=driver,
            driver_leg=None if winner is None else winner.leg,
            driver_segment_index=None if winner is None else winner.segment_index,
            driver_segment_kind=None if winner is None else winner.segment_kind,
            active_hard=sum(1 for d in demands if d.is_hard),
            active_lower_bounds=sum(
                1 for d in demands if d.kind is BodyRequirementKind.LOWER_BOUND),
            demands=tuple(demands),
        ))

    return BodyTrajectory2D(
        samples=tuple(out), conflicts=tuple(conflicts),
        nominal_body_z_m=float(nominal_body_z_m),
        world_x_spread_m=(max(world_spreads) if world_spreads else None),
    )


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------


def body_rows(trajectory: BodyTrajectory2D) -> list[dict]:
    """The trajectory, its conflicts and its summary, as one writable table."""

    rows: list[dict] = [{"row_kind": "summary", **trajectory.as_dict()}]
    rows += [{"row_kind": "sample", **s.as_dict()} for s in trajectory.samples]
    rows += [{"row_kind": "conflict", **c.as_dict()}
             for c in trajectory.conflicts]
    for start, end, leg in trajectory.concession_intervals():
        rows.append({"row_kind": "concession", "time_s": start,
                     "end_s": end, "driver_leg": leg.value})

    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]


def plot_body_trajectory_2d(trajectory: BodyTrajectory2D, path=None):
    """``body_z(t)`` with the leg that asked for each concession named."""

    import matplotlib.pyplot as plt

    fig, (top, bottom) = plt.subplots(
        2, 1, figsize=(11.0, 6.4), sharex=True,
        gridspec_kw={"height_ratios": [2.0, 1.0]},
    )
    time_s = trajectory.time_s
    top.plot(time_s, trajectory.body_z_m * 1e3, color="#1a1a1a", lw=1.6,
             label="body_z")
    top.axhline(trajectory.nominal_body_z_m * 1e3, color="#888888", lw=1.0,
                ls="--", label="nominal body_z")

    colours = {BodyDriver.HARD: "#c5221f", BodyDriver.LOWER_BOUND: "#b06000",
               BodyDriver.NOMINAL: "#cccccc",
               BodyDriver.INFEASIBLE: "#6a1b9a"}
    for driver, colour in colours.items():
        mask = np.array([s.driver is driver for s in trajectory.samples])
        if mask.any():
            top.scatter(time_s[mask], trajectory.body_z_m[mask] * 1e3, s=8,
                        color=colour, zorder=3, label=driver.value)
    top.set_ylabel("body_z [mm]")
    top.legend(loc="upper left", bbox_to_anchor=(0.0, 1.02), ncol=5,
               fontsize=8, frameon=False)
    top.set_title(f"Step 5 body trajectory  --  {BODY_BASIS}",
                  loc="right", fontsize=9, pad=30)
    top.grid(alpha=0.25)

    bottom.plot(time_s, trajectory.body_x_m * 1e3, color="#2a6f4e", lw=1.6)
    bottom.set_ylabel("body_x [mm]")
    bottom.set_xlabel("time [s]")
    bottom.grid(alpha=0.25)

    fig.tight_layout()
    if path is not None:
        fig.savefig(path, dpi=150)
        plt.close(fig)
    return fig
