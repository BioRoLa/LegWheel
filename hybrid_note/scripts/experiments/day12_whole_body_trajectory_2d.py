"""Day 12 Step 8: the complete synchronized four-leg trajectory.

Plan §15.  Every ingredient already exists -- Step 3's timeline, Step 4's
per-leg segments, Step 5's body trajectory, Step 6's margins, Step 7's
reposition verdicts -- and this step **aligns them onto one set of samples**.

**It calls no planner.**  Plan §15 requirement 7 forbids runtime replanning,
and the way to satisfy that is not to have anything here that could replan: the
module reads finished results and interpolates within them.

**Four things are still unresolved, and they travel with the output.**  Steps
4--7 each ended in a real "no", and a trajectory assembled on top of them is a
trajectory *under those assumptions*.  :attr:`WholeBodyTrajectory2D.assumptions`
carries them so that "Step 8 finished" cannot be read as "this works".
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from hybrid_note.scripts.experiments.cartesian_swing_ik_2d import (
    rim_point_model_gap_2d,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    MotionSegment2D,
    RimId,
    SegmentKind,
)
from hybrid_note.scripts.experiments.day10_11_sequence_builders_2d import (
    handoff_between_2d,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    BODY_BASIS,
    BodyTrajectory2D,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    LegId,
    leg_mounts_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    COM_BASIS,
    GAMMA_RAD,
    TraversalStability2D,
    contact_offset_from_hip_m,
    segment_at,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    LegMode,
    ScheduledSegment2D,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    FourLegPlan2D,
    TransitionPhase,
)


# --------------------------------------------------------------------------
# One leg at one instant
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class LegSample2D:
    """Plan §15 requirements 1 and 2, for one leg."""

    leg: LegId
    theta_rad: float
    beta_rad: float
    gamma_rad: float
    mode: LegMode
    rim: RimId
    alpha_rad: float
    contact_world_xy_m: tuple[float, float]
    in_contact: bool
    segment_index: int
    segment_kind: SegmentKind
    phase: TransitionPhase
    #: Requirement 4: the segment's own sampling, carried not re-derived.
    arc_samples: int | None
    #: Requirement 6, measured at this pose with the existing function.
    rim_geometry_gap_m: float
    #: True when this sample is a **generator frame**, false when it is an
    #: interpolation between the segment's endpoints.  A ``RECOVERY_SWING``'s
    #: endpoints share a theta and its whole retraction lives in between, so an
    #: interpolated sample is a summary of the motion, not the motion: it is
    #: fine for reading, and wrong for driving a motor.
    from_generator_frame: bool = False
    #: True at a segment's first or last frame.  Those instants are shared with
    #: the neighbouring segment -- a recovery's first frame **is** the stroke's
    #: last, the moment of liftoff, still in contact -- so the segment's mode
    #: and the frame's contact state legitimately disagree there.
    is_segment_boundary_frame: bool = False

    def as_dict(self) -> dict:
        return {
            "leg": self.leg.value,
            "theta_deg": float(np.rad2deg(self.theta_rad)),
            "beta_deg": float(np.rad2deg(self.beta_rad)),
            "gamma_deg": float(np.rad2deg(self.gamma_rad)),
            "mode": self.mode.value,
            "rim": self.rim.value,
            "alpha_deg": float(np.rad2deg(self.alpha_rad)),
            "contact_x_mm": self.contact_world_xy_m[0] * 1e3,
            "contact_y_mm": self.contact_world_xy_m[1] * 1e3,
            "in_contact": self.in_contact,
            "segment_index": self.segment_index,
            "segment_kind": self.segment_kind.value,
            "phase": self.phase.value,
            "arc_samples": self.arc_samples,
            "rim_geometry_gap_mm": self.rim_geometry_gap_m * 1e3,
            "from_generator_frame": self.from_generator_frame,
            "is_segment_boundary_frame": self.is_segment_boundary_frame,
        }


def _lerp(a: float, b: float, t: float) -> float:
    return float(a + (b - a) * float(np.clip(t, 0.0, 1.0)))


def leg_sample_at(
    scheduled: ScheduledSegment2D,
    segment: MotionSegment2D,
    phase: TransitionPhase,
    time_s: float,
    body_x_m: float,
    mount_xy_m,
    frames=None,
) -> LegSample2D:
    """One leg's state at ``time_s``.

    **With ``frames``, this reads the generator's own frame** -- the pose that
    was actually planned and collision-checked -- via the segment's
    ``FrameRef2D``.  That is what anything driving hardware needs: a
    ``RECOVERY_SWING`` starts and ends at the same theta and retracts to the
    compact posture entirely in between, so a trajectory built from its
    endpoints would command the leg to stay extended right through the swing.

    Without ``frames`` it falls back to interpolating the two endpoints, which
    is the Day 12 behaviour and is honest about itself through
    :attr:`LegSample2D.from_generator_frame`.  Neither path runs a planner, so
    plan §15 requirement 7 holds either way.

    **Between two frames it interpolates, and it must.**  This used to snap to
    the nearest frame, which made the sampled signal a *staircase*: whole
    stretches of identical poses separated by one 4 deg jump of
    ``beta_step_rad``.  Two things went wrong with that.  A rate read off it is
    ``frame step / sample interval`` -- a number that grows as the grid is
    refined and reports the discretisation rather than the gait (the same run
    read 48.1% of the motor budget at 241 samples and 127.7% at 1921).  And the
    exported command *was* that staircase, so the robot would have been asked
    to make each 4 deg step inside one playback tick instead of moving
    smoothly across it.  Interpolating fixes both at once, and the rate then
    agrees with ``frame_motor_rate_2d`` at every sample count.

    The continuous quantities are interpolated; ``rim`` and the airborne flag
    are **categorical** and take the nearer frame, because half of a rim is not
    a rim.  Interpolation does put the leg through poses that were not each
    individually collision-checked -- the generator's own step sizes (2 deg in
    ``theta``, 4 deg in ``beta``) bound how far from a checked pose any of them
    can be.
    """

    span = scheduled.end_s - scheduled.start_s
    fraction = float(np.clip(
        0.0 if span <= 0.0 else (time_s - scheduled.start_s) / span, 0.0, 1.0))
    start, end = segment.start_contact, segment.end_contact
    sampling = segment.sampling

    if frames:
        indices = segment.frames.indices
        place = fraction * (len(indices) - 1)
        low = int(np.clip(np.floor(place), 0, len(indices) - 1))
        high = int(min(low + 1, len(indices) - 1))
        blend = float(place - low)
        first, second = frames[indices[low]], frames[indices[high]]
        # The nearer frame owns everything that cannot be averaged.
        nearest = first if blend < 0.5 else second

        theta = _lerp(first.theta_rad, second.theta_rad, blend)
        beta = _lerp(first.beta_rad, second.beta_rad, blend)
        alpha = _lerp(0.0 if first.alpha_rad is None else first.alpha_rad,
                      0.0 if second.alpha_rad is None else second.alpha_rad,
                      blend)
        rim = RimId(nearest.rim) if nearest.rim else start.rim
        if first.contact_xz_m is None or second.contact_xz_m is None:
            # One end without a contact leaves nothing to interpolate between,
            # so fall back to the segment's own contact rather than inventing
            # the missing end.
            offset = contact_offset_from_hip_m(segment, fraction)
        else:
            offset = _lerp(
                float(first.contact_xz_m[0] - first.hip_xz_m[0]),
                float(second.contact_xz_m[0] - second.hip_xz_m[0]), blend)
        contact_xy = (body_x_m + float(mount_xy_m[0]) + offset,
                      float(mount_xy_m[1]))
        return LegSample2D(
            leg=scheduled.leg, theta_rad=theta, beta_rad=beta,
            gamma_rad=GAMMA_RAD, mode=scheduled.mode, rim=rim, alpha_rad=alpha,
            contact_world_xy_m=contact_xy,
            in_contact=not bool(nearest.airborne),
            segment_index=scheduled.segment_index,
            segment_kind=scheduled.segment_kind, phase=phase,
            arc_samples=getattr(sampling, "arc_samples", None),
            rim_geometry_gap_m=float(
                rim_point_model_gap_2d(theta, beta, rim, alpha)),
            from_generator_frame=True,
            # Inside the first or the last frame interval.  Snapping used to
            # make this "the nearest frame is the first or the last"; when the
            # interpolation went in this was narrowed to *exactly* the first
            # frame, which quietly stopped ``stance_contact_valid`` from
            # excusing the liftoff instant and made it fail at fine sampling
            # only.  Both bounds are a fixed fraction of the segment, so this
            # does not depend on the sample count.
            is_segment_boundary_frame=(low == 0
                                       or high == len(indices) - 1),
        )

    theta = _lerp(start.theta_rad, end.theta_rad, fraction)
    beta = _lerp(start.beta_rad, end.beta_rad, fraction)
    alpha = _lerp(start.alpha_rad, end.alpha_rad, fraction)
    rim = start.rim if fraction < 0.5 else end.rim

    offset = contact_offset_from_hip_m(segment, fraction)
    contact_xy = (body_x_m + float(mount_xy_m[0]) + offset, float(mount_xy_m[1]))

    return LegSample2D(
        leg=scheduled.leg, theta_rad=theta, beta_rad=beta, gamma_rad=GAMMA_RAD,
        mode=scheduled.mode, rim=rim, alpha_rad=alpha,
        contact_world_xy_m=contact_xy,
        in_contact=scheduled.mode is LegMode.STANCE,
        segment_index=scheduled.segment_index,
        segment_kind=scheduled.segment_kind, phase=phase,
        arc_samples=getattr(sampling, "arc_samples", None),
        rim_geometry_gap_m=float(rim_point_model_gap_2d(theta, beta, rim, alpha)),
        from_generator_frame=False,
    )


# --------------------------------------------------------------------------
# One instant, whole body
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class WholeBodySample2D:
    """Plan §15's "every frame at least" list, in one object."""

    time_s: float
    body_position_world_m: tuple[float, float, float]
    body_rpy_rad: tuple[float, float, float]
    legs: dict[LegId, LegSample2D]
    swing_leg: LegId | None
    support_legs: tuple[LegId, ...]
    stability_margin_m: float | None

    @property
    def max_rim_geometry_gap_m(self) -> float:
        """The **maximum**, not the spread: the gap is 0 on the foot rim and
        1.2 mm on the upper tyres, so a range would read as smaller than it is
        (Day 10--11 made the same point)."""

        return max((s.rim_geometry_gap_m for s in self.legs.values()), default=0.0)

    def as_dict(self) -> dict:
        row = {
            "time_s": self.time_s,
            "body_x_mm": self.body_position_world_m[0] * 1e3,
            "body_y_mm": self.body_position_world_m[1] * 1e3,
            "body_z_mm": self.body_position_world_m[2] * 1e3,
            "body_roll_deg": float(np.rad2deg(self.body_rpy_rad[0])),
            "body_pitch_deg": float(np.rad2deg(self.body_rpy_rad[1])),
            "body_yaw_deg": float(np.rad2deg(self.body_rpy_rad[2])),
            "swing_leg": None if self.swing_leg is None else self.swing_leg.value,
            "support_legs": ",".join(l.value for l in self.support_legs),
            "stability_margin_mm": (None if self.stability_margin_m is None
                                    else self.stability_margin_m * 1e3),
            "max_rim_geometry_gap_mm": self.max_rim_geometry_gap_m * 1e3,
        }
        for leg in LEG_ORDER:
            sample = self.legs.get(leg)
            if sample is None:
                continue
            row.update({f"{leg.value}_{k}": v
                        for k, v in sample.as_dict().items() if k != "leg"})
        return row


# --------------------------------------------------------------------------
# Handoffs (requirement 5)
# --------------------------------------------------------------------------


#: One full turn.  A ``RECOVERY_SWING`` ends a revolution behind where the next
#: stroke starts, by construction (Step 1: ``beta_target = start.beta - 2*pi``),
#: so ``beta`` is a revolution counter and is **never wrapped** anywhere in this
#: tree.  A 360 deg beta jump at such a boundary is therefore the revolution
#: itself, not a discontinuity -- which is why both readings are reported.
FULL_TURN_RAD: float = float(2.0 * np.pi)


def _wrapped_to_pi(angle_rad: float) -> float:
    return float((float(angle_rad) + np.pi) % (2.0 * np.pi) - np.pi)


@dataclass(frozen=True)
class HandoffCheck2D:
    """One segment boundary of one leg, measured five ways.

    The measurement itself is Step 0's :func:`handoff_between_2d`, not a second
    implementation: Day 12 already factored it out precisely so that chains
    spanning several frame sources could be checked with the same code the
    single-source sequences use.
    """

    leg: LegId
    from_index: int
    to_index: int
    from_kind: SegmentKind
    to_kind: SegmentKind
    time_s: float
    time_is_monotonic: bool
    #: Raw, unwrapped.  Carries whole revolutions, so it is the honest answer to
    #: "how far did the joint travel".
    joint_jump_rad: float
    #: The same jump brought into ``(-pi, pi]``.  This is the answer to "is the
    #: pose discontinuous", which is a different question: a recovery's full
    #: turn is 360 deg raw and 0 deg wrapped, and it is not a discontinuity.
    joint_jump_wrapped_rad: float
    body_jump_m: float
    contact_jump_m: float
    rim_geometry_gap_m: float
    rim_changed: bool
    #: Whether the contact moved to a **different terrain surface** -- ground
    #: to the obstacle top, or back.  This is a third kind of boundary, next to
    #: Step 0's ``CUT`` and ``HANDOVER``, and it needs its own reading for the
    #: same reason those two do (trap 1).
    #:
    #: When the surface changes, the contact point *has* to jump: a different
    #: point of the leg is now touching a different thing.  Measured on the
    #: crossing, the two surface changes jump the contact 92.782 mm and
    #: 108.512 mm while moving the hip only 3.238 mm and 1.570 mm -- the leg is
    #: where it was.  Every same-surface boundary jumps at most 5.855 mm.
    surface_changed: bool = False

    @property
    def is_surface_transfer(self) -> bool:
        """A contact jump that is a transfer, not a break.

        The sibling of :attr:`is_whole_turn`: both name a discontinuity that is
        real in the number and not a defect in the motion.
        """

        return self.surface_changed

    @property
    def is_whole_turn(self) -> bool:
        """A jump that is a revolution, not a break."""

        return (abs(abs(self.joint_jump_rad) - FULL_TURN_RAD) < 1e-6
                and abs(self.joint_jump_wrapped_rad) < 1e-6)

    def as_dict(self) -> dict:
        return {
            "leg": self.leg.value,
            "from_segment": self.from_index,
            "to_segment": self.to_index,
            "from_kind": self.from_kind.value,
            "to_kind": self.to_kind.value,
            "time_s": self.time_s,
            "time_is_monotonic": self.time_is_monotonic,
            "joint_jump_deg": float(np.rad2deg(self.joint_jump_rad)),
            "joint_jump_wrapped_deg": float(
                np.rad2deg(self.joint_jump_wrapped_rad)),
            "is_whole_turn": self.is_whole_turn,
            "body_jump_mm": self.body_jump_m * 1e3,
            "contact_jump_mm": self.contact_jump_m * 1e3,
            "rim_changed": self.rim_changed,
            "rim_geometry_gap_mm": self.rim_geometry_gap_m * 1e3,
        }


def _surfaces_of(segment: MotionSegment2D) -> tuple[str, ...]:
    """Which terrain surfaces this segment rolls on; empty when airborne."""

    rolling = getattr(segment, "rolling", None)
    if rolling is None or not getattr(rolling, "surface_ids", None):
        return ()
    return tuple(str(s) for s in rolling.surface_ids)


def _surface_changed_2d(before: MotionSegment2D, after: MotionSegment2D) -> bool:
    """Did the contact move to a different terrain surface across this seam?

    ``False`` whenever either side is airborne: a swing has no surface, and
    calling that a transfer would exempt every liftoff and touchdown from the
    chaining check -- which is exactly the check they most need.
    """

    a, b = _surfaces_of(before), _surfaces_of(after)
    if not a or not b:
        return False
    return set(a) != set(b)


def handoff_checks_2d(plan: FourLegPlan2D) -> list[HandoffCheck2D]:
    """Every segment boundary, measured.  Nothing is repaired.

    The rim-geometry gap is included because plan §15 requirement 5 lists it as
    one of the five things a handoff has to be checked for, and requirement 6
    says to quantify it rather than tune the geometry model until it stops
    showing.  It is measured with ``rim_point_model_gap_2d`` -- the function
    Day 8--9 already used -- so there is one definition of it in the tree.
    """

    out: list[HandoffCheck2D] = []
    for leg in LEG_ORDER:
        if leg not in plan.plans:
            continue
        leg_plan = plan.plans[leg]
        scheduled = sorted(plan.schedule.segments_of(leg),
                           key=lambda s: s.segment_index)
        for previous, following in zip(scheduled, scheduled[1:]):
            before = leg_plan.phased[previous.segment_index].segment
            after = leg_plan.phased[following.segment_index].segment
            report = handoff_between_2d(before, after)

            raw = max((report.theta_jump_rad, report.beta_jump_rad), key=abs)
            wrapped = max(
                (_wrapped_to_pi(report.theta_jump_rad),
                 _wrapped_to_pi(report.beta_jump_rad)), key=abs)

            exit_state, entry_state = before.end_contact, after.start_contact
            gap = max(
                rim_point_model_gap_2d(exit_state.theta_rad, exit_state.beta_rad,
                                       exit_state.rim, exit_state.alpha_rad),
                rim_point_model_gap_2d(entry_state.theta_rad,
                                       entry_state.beta_rad, entry_state.rim,
                                       entry_state.alpha_rad),
            )
            out.append(HandoffCheck2D(
                leg=leg, from_index=previous.segment_index,
                to_index=following.segment_index,
                from_kind=previous.segment_kind, to_kind=following.segment_kind,
                time_s=float(following.start_s),
                time_is_monotonic=following.start_s >= previous.end_s - 1e-9,
                joint_jump_rad=float(raw),
                joint_jump_wrapped_rad=float(wrapped),
                body_jump_m=float(report.hip_jump_m),
                contact_jump_m=float(report.contact_jump_m),
                rim_geometry_gap_m=float(gap),
                rim_changed=bool(report.rim_changed),
                surface_changed=_surface_changed_2d(before, after),
            ))
    return out


# --------------------------------------------------------------------------
# The trajectory
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class WholeBodyTrajectory2D:
    """One synchronized trajectory, with what it rests on written on it."""

    samples: tuple[WholeBodySample2D, ...]
    handoffs: tuple[HandoffCheck2D, ...]
    #: The unresolved results this trajectory was assembled on top of.  Not
    #: decoration: without them "assembled" reads as "works".
    assumptions: tuple[str, ...] = ()
    body_basis: str = BODY_BASIS
    com_basis: str = COM_BASIS

    @property
    def max_joint_jump_rad(self) -> float:
        """Raw, revolutions included."""

        return max((abs(h.joint_jump_rad) for h in self.handoffs), default=0.0)

    @property
    def max_joint_discontinuity_rad(self) -> float:
        """Wrapped, so a recovery's full turn does not read as a break."""

        return max((abs(h.joint_jump_wrapped_rad) for h in self.handoffs),
                   default=0.0)

    @property
    def max_contact_gap_m(self) -> float:
        return max((h.contact_jump_m for h in self.handoffs), default=0.0)

    @property
    def max_rim_geometry_gap_m(self) -> float:
        return max((h.rim_geometry_gap_m for h in self.handoffs), default=0.0)

    @property
    def minimum_stability_margin_m(self) -> float | None:
        margins = [s.stability_margin_m for s in self.samples
                   if s.stability_margin_m is not None]
        return min(margins) if margins else None

    @property
    def time_is_monotonic(self) -> bool:
        times = [s.time_s for s in self.samples]
        return all(b > a for a, b in zip(times, times[1:]))

    @property
    def every_sample_has_four_legs(self) -> bool:
        return all(len(s.legs) == 4 for s in self.samples)

    @property
    def from_generator_frames(self) -> bool:
        """True only when **every** leg sample is a real generator frame."""

        return bool(self.samples) and all(
            leg.from_generator_frame
            for sample in self.samples for leg in sample.legs.values()
        )

    def as_dict(self) -> dict:
        minimum = self.minimum_stability_margin_m
        return {
            "samples": len(self.samples),
            "handoffs": len(self.handoffs),
            "max_joint_jump_deg": float(np.rad2deg(self.max_joint_jump_rad)),
            "max_joint_discontinuity_deg": float(
                np.rad2deg(self.max_joint_discontinuity_rad)),
            "whole_turn_handoffs": sum(1 for h in self.handoffs if h.is_whole_turn),
            "max_contact_gap_mm": self.max_contact_gap_m * 1e3,
            "max_rim_geometry_gap_mm": self.max_rim_geometry_gap_m * 1e3,
            "rim_gap_note": (
                "0 here because this run never leaves the foot rim, where the "
                "gap is 0 by construction; it is 1.2 mm on the upper tyres"),
            "minimum_stability_margin_mm": (None if minimum is None
                                            else minimum * 1e3),
            "finite_body_z_samples": int(sum(
                1 for s in self.samples
                if np.isfinite(s.body_position_world_m[2]))),
            "time_is_monotonic": self.time_is_monotonic,
            "every_sample_has_four_legs": self.every_sample_has_four_legs,
            "from_generator_frames": self.from_generator_frames,
            "unresolved_assumptions": len(self.assumptions),
            "body_basis": self.body_basis,
            "com_basis": self.com_basis,
        }


def _margin_at(stability: TraversalStability2D, time_s: float) -> float | None:
    for swing in stability.swings:
        if swing.start_s <= time_s <= swing.end_s:
            candidates = [s for s in swing.samples if s.margin_m is not None]
            if not candidates:
                return None
            nearest = min(candidates, key=lambda s: abs(s.time_s - time_s))
            return nearest.margin_m
    return None


def assumptions_of(
    plan: FourLegPlan2D,
    trajectory: BodyTrajectory2D,
    stability: TraversalStability2D,
    reposition_unresolved: int = 0,
) -> tuple[str, ...]:
    """The Steps 4--7 verdicts this assembly sits on top of, as text."""

    out: list[str] = []
    if plan.airborne_overruns:
        worst = max(o.compression for o in plan.airborne_overruns)
        out.append(f"Step 4: {len(plan.airborne_overruns)} airborne run(s) are "
                   f"compressed up to {worst:.3f}x -- the swing window is "
                   f"shorter than the motion already planned into it")
    if not trajectory.is_feasible:
        worst = max((c.disagreement_m for c in trajectory.conflicts), default=0.0)
        out.append(f"Step 5: body_z is INFEASIBLE -- {len(trajectory.conflicts)} "
                   f"conflicts, worst {worst * 1e3:.3f} mm apart")
    if not stability.is_stable:
        minimum = stability.minimum_margin_m
        shown = "unknown" if minimum is None else f"{minimum * 1e3:.3f} mm"
        out.append(f"Step 6: {len(stability.unstable_swings)} of "
                   f"{len(stability.swings)} swings are unstable -- minimum "
                   f"margin {shown}")
    if reposition_unresolved:
        out.append(f"Step 7: {reposition_unresolved} TOP_REPOSITION "
                   f"requirement(s) remain unresolved")
    return tuple(out)


def assemble_whole_body_2d(
    plan: FourLegPlan2D,
    trajectory: BodyTrajectory2D,
    stability: TraversalStability2D,
    *,
    samples: int = 241,
    reposition_unresolved: int = 0,
    use_generator_frames: bool = False,
) -> WholeBodyTrajectory2D:
    """Align Steps 3--7's finished results onto one set of samples.

    ``use_generator_frames`` reads each leg's pose out of the frames its
    segments reference instead of interpolating the segment endpoints.  Off by
    default, because every frozen Day 12 number was measured with the
    interpolation -- and on for anything that has to become a motor command.
    """

    mounts = {m.leg: m.offset_body_xyz_m for m in leg_mounts_2d(GAMMA_RAD)}
    lo, hi = plan.schedule.covered_interval_s
    # Half-open, as Step 6 samples a swing.  At ``hi`` exactly, one leg's chain
    # has run out while another's swing has already begun, so that instant has
    # no complete four-leg configuration -- it is the edge of the data, not a
    # gait fault, and sampling it would put a two-leg "support" in the output.
    grid = np.linspace(lo, hi, int(samples) + 1)[:-1]
    times, body_x, body_z = (trajectory.time_s, trajectory.body_x_m,
                             trajectory.body_z_m)

    out: list[WholeBodySample2D] = []
    for time_s in grid:
        x = float(np.interp(time_s, times, body_x))
        z = float(np.interp(time_s, times, body_z))

        legs: dict[LegId, LegSample2D] = {}
        swing: LegId | None = None
        support: list[LegId] = []
        for leg in LEG_ORDER:
            if leg not in plan.plans:
                continue
            # Step 6's lookup, not a second copy: the boundary tolerance and
            # the half-open ownership rule are exactly what a re-implementation
            # gets wrong, and this module already got it wrong once.
            scheduled = segment_at(plan.schedule.segments_of(leg), float(time_s))
            if scheduled is None:
                continue
            leg_plan = plan.plans[leg]
            phased = leg_plan.phased[scheduled.segment_index]
            sample = leg_sample_at(
                scheduled, phased.segment, phased.phase, float(time_s), x,
                mounts[leg],
                frames=(leg_plan.frames.get(phased.segment.frames.source_id)
                        if use_generator_frames else None),
            )
            legs[leg] = sample
            if sample.mode is LegMode.AIRBORNE:
                swing = leg
            else:
                support.append(leg)

        out.append(WholeBodySample2D(
            time_s=float(time_s),
            body_position_world_m=(x, float(trajectory.body_y_m), z),
            body_rpy_rad=tuple(float(v) for v in trajectory.body_rpy_rad),
            legs=legs, swing_leg=swing, support_legs=tuple(support),
            stability_margin_m=_margin_at(stability, float(time_s)),
        ))

    return WholeBodyTrajectory2D(
        samples=tuple(out), handoffs=tuple(handoff_checks_2d(plan)),
        assumptions=assumptions_of(plan, trajectory, stability,
                                   reposition_unresolved),
    )


# --------------------------------------------------------------------------
# Output (requirement 8)
# --------------------------------------------------------------------------


def whole_body_rows(result: WholeBodyTrajectory2D) -> list[dict]:
    """Summary, assumptions, handoffs and samples, as one writable table."""

    rows: list[dict] = [{"row_kind": "summary", **result.as_dict()}]
    rows += [{"row_kind": "assumption", "note": note}
             for note in result.assumptions]
    rows += [{"row_kind": "handoff", **h.as_dict()} for h in result.handoffs]
    rows += [{"row_kind": "sample", **s.as_dict()} for s in result.samples]

    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]
