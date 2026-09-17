"""Day 12 problem A5: put the crossing where the obstacle actually is.

The audit found that a crossing assembles into a trajectory that contains no
obstacle -- the implied platform position scattered over 753.8 mm and the
nearest one sat 251.5 mm from the platform the run was planned for.  Log
section 1.10 measured why, and it is two separate things:

**1. Nothing shares a frame.**  ``build_leg_plan_2d`` hands **all four legs the
same chain** -- 14 segments, every one starting at hip ``x = 0`` -- and then
the schedule runs them at four different times.  That is four legs each
crossing their own obstacle, not four legs taking turns over one.  Meanwhile
Step 2 does compute a world origin (body at 145 mm, platform at 1000 mm) and
nothing downstream ever reads it.

**2. The chain is not joined at either end of the crossing.**  Entering it the
hip jumps 1014 mm and ``theta`` jumps -32.5 deg; leaving it the hip jumps
627 mm and ``theta`` jumps +55.5 deg.  Changing ``cycles_before`` does not move
those numbers, because they are not accumulated error -- the crossing simply
starts in its own coordinates *and its own posture*.

So registering the crossing takes both halves:

``rebase``       an offset in hip ``x`` and a whole number of ``beta`` turns,
                 which is bookkeeping and exact.
``transition``   an actual motion from the posture the nominal run lands in to
                 the posture the crossing starts from, and back.  That is a
                 new primitive, and :func:`run_posture_transition_2d` is it --
                 the recovery swing's own three ramps (retract, rotate,
                 extend), aimed at a destination instead of at the next
                 nominal touchdown.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, replace

import numpy as np

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    MotionSegment2D,
    MotionSequence2D,
    PointContact2D,
    RollingContact2D,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    LegId,
    initialize_four_leg_state_2d,
    leg_mounts_2d,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    NominalCycle2D,
    NominalPosture2D,
    RecoveryConfig2D,
    RecoverySwing2D,
    RollStroke2D,
    nominal_stroke_2d,
    run_foot_rim_roll_2d,
    run_nominal_cycles_2d,
    run_recovery_swing_2d,
    cycle_segments_2d,
    roll_segment_2d,
    standing_pose_penetration_m,
    standing_stroke_2d,
    swing_segment_2d,
    theta_for_hip_z_2d,
    HipZProfile2D,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    HIP_TO_BODY_Z_M,
)
from hybrid_note.scripts.experiments.day10_11_composer_2d import ComposedSequence2D
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import SegmentKind
from hybrid_note.scripts.experiments.day12_segment_contract_2d import (
    ChainTolerance2D,
    SegmentChain2D,
    chain_boundaries_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    LegPlan2D,
    PhasedSegment2D,
    STRATEGY_HALVES,
    TransitionPhase,
    phase_of_kind,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import GAMMA_RAD
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    FourLegSchedule2D,
    GaitTiming2D,
    LegMode,
    ScheduledSegment2D,
)

TWO_PI = 2.0 * np.pi


# --------------------------------------------------------------------------
# The missing motion
# --------------------------------------------------------------------------


def run_posture_transition_2d(
    stroke: RollStroke2D,
    theta_target_rad: float,
    beta_target_rad: float,
    config: RecoveryConfig2D | None = None,
    hip_z_target_m: float | None = None,
    lift_hip_before_rotation: bool = False,
) -> RecoverySwing2D:
    """Swing the leg from where a stroke left it to a posture something needs.

    Identical in shape to :func:`run_recovery_swing_2d` -- retract to the
    compact posture, carry the rotation forward, extend into the destination --
    and it *is* that function, with its two implicit destinations made
    explicit.  Written this way on purpose: the clearance rules for the three
    ramps were got wrong once already and a second copy would get them wrong
    again in a new way.

    ``beta_target_rad`` must be **forward** of where the stroke ended, because
    the leg rotates one way.  :func:`forward_beta_for_orientation` is how a
    caller turns "the crossing wants this orientation" into a target that is.
    """

    return run_recovery_swing_2d(
        stroke, config,
        beta_target_rad=float(beta_target_rad),
        theta_touchdown_rad=float(theta_target_rad),
        hip_z_touchdown_m=hip_z_target_m,
        lift_hip_before_rotation=bool(lift_hip_before_rotation),
    )


def standing_stroke_at_2d(
    posture: NominalPosture2D,
    theta_rad: float,
    beta_rad: float,
    hip_x_m: float,
    hip_z_m: float,
    *,
    max_lift_m: float | None = None,
) -> tuple[RollStroke2D, float]:
    """A standing pose, nudged up past a rounding-level ground penetration.

    Returns the stroke **and the lift it took**, so the number is on the record
    at every call site rather than hidden inside one.

    The crossing's own exit pose misses Day 12's ground by 0.0001 mm -- a tenth
    of a micron, which is float noise on a half-metre geometry and not a
    modelling disagreement.  ``max_lift_m`` is what keeps it that way: it
    defaults to the posture's own ``collision_tolerance_m`` and **raises** past
    it, so a real geometric conflict cannot be quietly lifted out of sight.
    """

    limit = (float(posture.collision_tolerance_m) if max_lift_m is None
             else float(max_lift_m))
    lift = standing_pose_penetration_m(posture, theta_rad, beta_rad,
                                       hip_x_m, hip_z_m)
    if lift > limit:
        raise ValueError(
            f"the pose penetrates the ground by {lift * 1e3:.4f} mm, past the "
            f"{limit * 1e3:.4f} mm this is allowed to absorb.  That is a "
            "geometric conflict between the crossing and the nominal scene, "
            "not a rounding edge, and lifting it away would hide it."
        )
    return (standing_stroke_2d(posture, theta_rad, beta_rad, hip_x_m,
                               hip_z_m + lift),
            float(lift))


def forward_beta_for_orientation(current_beta_rad: float,
                                 wanted_orientation_rad: float) -> float:
    """The next ``beta`` forward of ``current`` with the wanted orientation.

    ``beta`` is a revolution counter, so an orientation names a whole family of
    values a turn apart.  The leg rotates in one direction, so exactly one of
    them is reachable without going backwards, and this is it: never equal to
    ``current`` (a transition that rotates nothing still has to be a rotation
    of one full turn, or the ramp has no room to move).
    """

    current = float(current_beta_rad)
    wanted = float(wanted_orientation_rad)
    # Forward is decreasing, matching the recovery's own sense.
    turns = np.ceil((current - wanted) / TWO_PI)
    target = wanted + turns * TWO_PI
    if target >= current - 1e-12:
        target -= TWO_PI
    return float(target)


# --------------------------------------------------------------------------
# Where each leg is, in the world
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class LegApproach2D:
    """How far one leg has to roll before it meets the obstacle."""

    leg: LegId
    #: Gait phase at t = 0, which is what fixes where this leg's contact starts.
    phase: float
    start_contact_x_m: float
    obstacle_x_m: float
    #: Whole nominal strokes that fit in the gap, and what is left over.
    whole_strokes: int
    partial_distance_m: float
    #: How far the contact moves over one whole **cycle** -- stroke *and*
    #: recovery.  Not the stroke's own advance: the recovery's touchdown puts
    #: the foot down further forward again, and dividing the gap by the
    #: stroke's 202.458 mm instead of the cycle's 325.916 mm overshot the
    #: platform by 370-620 mm depending on the leg.
    cycle_contact_advance_m: float
    #: Where the crossing's first contact sits relative to its own hip.  The
    #: hip the approach has to stop short of is ``obstacle_x - this``, and
    #: leaving it off meant the walk aimed at one hip and the entry transition
    #: was sized against another.  Zero for this crossing, which is exactly why
    #: it went unnoticed.
    landing_contact_offset_m: float = 0.0
    #: How much further **this leg** rolls before it starts its crossing.
    #:
    #: **Measured, and NOT adopted.  Leave it at zero.**  Staggering does
    #: separate a pair in time, but it does so by crossing at different x --
    #: the robot goes over the obstacle skewed, and the project owner requires
    #: a straight-line crossing.  Day 13 replaced it with event-driven
    #: scheduling, which constrains *time* and leaves position alone.
    #: Kept rather than deleted so the next person to have this idea can see
    #: it was already measured, and why it was dropped (log 19.1).
    #:
    #: It is also **incomplete**: it moves where the approach aims without
    #: moving the crossing sequence's own world registration, so a non-zero
    #: value makes registration demand a ~291 mm *reverse* hip advance and the
    #: leg comes back with a refusal and no segments (log 18.6).  Anyone
    #: reviving this must shift both.
    #:
    #: Day 13 B3.  ``at_most_one_airborne`` fails because
    #: :func:`world_schedule_2d` takes a segment's time from its *position*,
    #: and the two legs of a pair share a ``mount_x`` -- so their crossings
    #: occupy identical time intervals no matter what the gait's phase offsets
    #: say.  (On flat ground the four swing windows do tile the cycle; this is
    #: a crossing-scheduling effect, not a gait one.)  Giving one leg of a pair
    #: a different landing x is what separates them in time.
    #:
    #: Kept separate from ``landing_contact_offset_m`` on purpose: that one is
    #: *derived* from the crossing sequence's own geometry (contact minus hip
    #: at entry) and is the same for every leg because every leg runs the same
    #: sequence.  This one is a free per-leg choice.  Zero reproduces Day 12.
    crossing_stagger_m: float = 0.0

    @property
    def target_hip_x_m(self) -> float:
        """The hip the approach must stop at or short of."""

        return float(self.obstacle_x_m - self.landing_contact_offset_m
                     + self.crossing_stagger_m)

    @property
    def distance_m(self) -> float:
        return float(self.obstacle_x_m - self.start_contact_x_m)

    @property
    def reaches_the_obstacle(self) -> bool:
        """False when the obstacle is behind this leg to start with.

        Reported rather than clamped: a leg that starts past the platform is a
        scene-setup answer, not something to fix by rolling zero strokes.
        """

        return self.distance_m > 0.0

    def as_dict(self) -> dict:
        return {
            "leg": self.leg.value,
            "phase": self.phase,
            "start_contact_x_mm": self.start_contact_x_m * 1e3,
            "obstacle_x_mm": self.obstacle_x_m * 1e3,
            "distance_mm": self.distance_m * 1e3,
            "whole_strokes": self.whole_strokes,
            "partial_distance_mm": self.partial_distance_m * 1e3,
            "cycle_contact_advance_mm": self.cycle_contact_advance_m * 1e3,
            "reaches_the_obstacle": self.reaches_the_obstacle,
        }


def leg_start_contact_x_2d(
    leg: LegId,
    body_x_m: float,
    timing: GaitTiming2D,
    stroke: RollStroke2D,
    mounts: dict[LegId, np.ndarray] | None = None,
) -> float:
    """Where this leg's contact sits at ``t = 0``, in world ``x``.

    The gait phase is what places it.  Step 2's initial state puts all four
    contacts level with their own hips, which is **not** a state the walk gait
    can start from -- it wants the four legs spread across the stroke by their
    phase offsets (audit item C3).  This reads the phase instead, so the four
    legs start where the schedule is about to assume they are.
    """

    if mounts is None:
        mounts = {m.leg: m.offset_body_xyz_m for m in leg_mounts_2d(GAMMA_RAD)}
    phase = timing.phase_at(leg, 0.0)
    fraction = float(min(phase / timing.stance_duty, 1.0))
    first, last = stroke.frames[0], stroke.frames[-1]
    offset_start = float(first.contact_xz_m[0] - first.hip_xz_m[0])
    offset_end = float(last.contact_xz_m[0] - last.hip_xz_m[0])
    offset = offset_start + fraction * (offset_end - offset_start)
    return float(body_x_m + float(mounts[leg][0]) + offset)


def cycle_contact_advance_m(
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
) -> float:
    """How far the contact moves over one whole nominal cycle.

    **Not** the stroke's own advance.  A cycle is a stroke *and* a recovery,
    and the recovery puts the foot down ahead of where it lifted off, so the
    contact gains the recovery's share too -- 325.916 mm against the stroke's
    202.458 mm.  Counting cycles with the stroke's number is what put every
    leg 370-620 mm past the platform on the first assembly.
    """

    cycles = run_nominal_cycles_2d(1, posture, config)
    if not cycles or not cycles[0].recovery.success:
        raise ValueError("the nominal cycle does not complete; nothing to count.")
    cycle = cycles[0]
    return float(cycle.recovery.end.contact_xz_m[0]
                 - cycle.stroke.frames[0].contact_xz_m[0])


def stroke_hip_per_contact_ratio(
    posture: NominalPosture2D | None = None,
) -> float:
    """How much hip travel one metre of contact travel costs, while rolling.

    325.916 / 202.458 = 1.610 for the levelled posture.  Needed because the
    partial stroke is asked for in **contact** distance while what has to land
    in the right place is the **hip**.
    """

    stroke = nominal_stroke_2d(posture)
    contact = float(stroke.contact_advance_m)
    if contact <= 0.0:
        raise ValueError("a stroke that advances no contact has no ratio.")
    return float(stroke.hip_advance_m / contact)


def leg_approaches_2d(
    terrain: SharedTerrainSpec2D,
    timing: GaitTiming2D,
    posture: NominalPosture2D | None = None,
    body_x_m: float | None = None,
    config: RecoveryConfig2D | None = None,
    landing_contact_offset_m: float = 0.0,
    crossing_stagger_m: dict[LegId, float] | None = None,
) -> dict[LegId, LegApproach2D]:
    """Each leg's roll to the obstacle, split into whole strokes and a remainder.

    ``body_x_m`` defaults to Step 2's own initial state -- the world origin
    that already exists and that nothing downstream has ever read.

    ``crossing_stagger_m`` moves a single leg's landing further along, so the
    two legs of a pair need not cross at the same x.  ``None`` (the default)
    means no stagger and reproduces Day 12 exactly.  See
    :attr:`LegApproach2D.crossing_stagger_m` for why this is what
    ``at_most_one_airborne`` needs.
    """

    posture = NominalPosture2D() if posture is None else posture
    stroke = nominal_stroke_2d(posture)
    advance = cycle_contact_advance_m(posture, config)
    if advance <= 0.0:
        raise ValueError("a cycle that advances nothing cannot reach anything.")

    if body_x_m is None:
        body_x_m = float(
            initialize_four_leg_state_2d(terrain).body_position_world_m[0])
    mounts = {m.leg: m.offset_body_xyz_m for m in leg_mounts_2d(GAMMA_RAD)}

    # Aim the **hip**, not the contact.
    #
    # The entry transition changes ``theta`` on its way (72.49 deg to 40.00),
    # so the contact sits somewhere else under the hip when it lands than when
    # it lifted off -- up to 58 mm of difference, and it differs per leg.
    # Aiming the contact left two of the four legs needing a *negative* hip
    # advance to correct, which is a correction the body cannot make.  The hip
    # is what the transition's ``hip_advance_m`` controls, so the hip is what
    # the approach has to place.
    ratio = stroke_hip_per_contact_ratio(posture)
    base_target_hip_x = float(terrain.x_start_m) - float(landing_contact_offset_m)
    stagger = dict(crossing_stagger_m or {})

    out: dict[LegId, LegApproach2D] = {}
    for leg in LEG_ORDER:
        # Per leg, not hoisted: B3 needs the two legs of a pair to aim at
        # different hip x, which is the only thing that separates their
        # crossings in a schedule whose time comes from position.
        leg_stagger = float(stagger.get(leg, 0.0))
        target_hip_x = base_target_hip_x + leg_stagger
        start = leg_start_contact_x_2d(leg, body_x_m, timing, stroke, mounts)
        start_hip = float(body_x_m) + float(mounts[leg][0])
        distance = target_hip_x - start_hip
        whole = int(np.floor(distance / advance)) if distance > 0.0 else 0
        remainder_hip = distance - whole * advance if distance > 0.0 else 0.0
        remainder = remainder_hip / ratio
        # Two things bound what the partial stroke can close.
        #
        # A stroke cannot be longer than a stroke: the remainder runs up to a
        # whole cycle (325.916 mm) but one rolling stroke only advances the
        # contact 202.458 mm, so the band between them is out of reach here.
        # And a stroke lands on a roll step, so asking for exactly the
        # remainder overshoots by up to one step -- and an overshoot cannot be
        # taken back, because the body does not reverse.
        #
        # So aim short by a step and hand the rest to the entry transition,
        # whose ``hip_advance_m`` moves the touchdown one-for-one.
        reachable = min(remainder, float(stroke.contact_advance_m))
        partial = max(0.0, reachable - float(posture.roll_step_m))
        out[leg] = LegApproach2D(
            leg=leg, phase=float(timing.phase_at(leg, 0.0)),
            start_contact_x_m=start, obstacle_x_m=float(terrain.x_start_m),
            whole_strokes=max(whole, 0), partial_distance_m=float(partial),
            cycle_contact_advance_m=advance,
            landing_contact_offset_m=float(landing_contact_offset_m),
            crossing_stagger_m=leg_stagger,
        )
    return out


# --------------------------------------------------------------------------
# Moving the crossing onto the leg
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class CrossingRebase2D:
    """The offset that carries a crossing's own frame onto a leg's chain."""

    hip_x_offset_m: float
    beta_offset_rad: float
    #: The state the chain is in when the crossing is spliced in.
    from_theta_rad: float
    from_beta_rad: float
    from_hip_x_m: float
    #: The state the crossing wants.
    to_theta_rad: float
    to_beta_rad: float
    to_hip_x_m: float

    @property
    def beta_turns(self) -> float:
        return float(self.beta_offset_rad / TWO_PI)

    def as_dict(self) -> dict:
        return {
            "hip_x_offset_mm": self.hip_x_offset_m * 1e3,
            "beta_offset_deg": float(np.rad2deg(self.beta_offset_rad)),
            "beta_turns": self.beta_turns,
            "from_theta_deg": float(np.rad2deg(self.from_theta_rad)),
            "to_theta_deg": float(np.rad2deg(self.to_theta_rad)),
            "from_beta_deg": float(np.rad2deg(self.from_beta_rad)),
            "to_beta_deg": float(np.rad2deg(self.to_beta_rad)),
            "from_hip_x_mm": self.from_hip_x_m * 1e3,
            "to_hip_x_mm": self.to_hip_x_m * 1e3,
        }


def crossing_rebase_2d(
    from_theta_rad: float,
    from_beta_rad: float,
    from_hip_x_m: float,
    to_theta_rad: float,
    to_beta_rad: float,
    to_hip_x_m: float,
) -> CrossingRebase2D:
    """What to add to the crossing's coordinates so it continues the chain.

    Pure bookkeeping, and that is the whole point of it: the **transition** is
    what physically brings the leg to the crossing's orientation, so by the
    time this runs ``from_beta_rad`` is already there and the offset is just
    the difference in the counters.

    Running :func:`forward_beta_for_orientation` here instead -- which is what
    the first version did -- advances a turn that the transition has already
    turned, and the seam comes out exactly 360 deg wide.  That function refuses
    to return its own input on purpose, because "rotate to this orientation"
    must be a rotation; "line these two counters up" must not be.
    """

    return CrossingRebase2D(
        hip_x_offset_m=float(from_hip_x_m - to_hip_x_m),
        beta_offset_rad=float(from_beta_rad - to_beta_rad),
        from_theta_rad=float(from_theta_rad), from_beta_rad=float(from_beta_rad),
        from_hip_x_m=float(from_hip_x_m),
        to_theta_rad=float(to_theta_rad), to_beta_rad=float(to_beta_rad),
        to_hip_x_m=float(to_hip_x_m),
    )


def approach_rows(approaches: dict[LegId, LegApproach2D]) -> list[dict]:
    return [{"row_kind": "approach", **approaches[leg].as_dict()}
            for leg in LEG_ORDER if leg in approaches]


# --------------------------------------------------------------------------
# The run up to the obstacle
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class ApproachRun2D:
    """One leg's nominal run, ending where the crossing has to start.

    ``cycles`` are whole nominal cycles; ``partial`` is the rolling stroke that
    closes what is left of the gap and has **no recovery after it**, because
    the entry transition is what leaves the ground next.
    """

    cycles: tuple[NominalCycle2D, ...]
    partial: RollStroke2D | None
    approach: LegApproach2D

    @property
    def success(self) -> bool:
        if any(not c.recovery.success for c in self.cycles):
            return False
        return self.partial is None or self.partial.success

    @property
    def failure_reason(self) -> str | None:
        for index, cycle in enumerate(self.cycles):
            if not cycle.recovery.success:
                return f"cycle {index}: {cycle.recovery.failure_reason}"
        if self.partial is not None and not self.partial.success:
            return f"partial stroke: {self.partial.stop_reason}"
        return None

    @property
    def end_frame(self):
        """The pose the run finishes in -- what the entry transition starts from."""

        if self.partial is not None and self.partial.frames:
            return self.partial.end
        if self.cycles:
            return self.cycles[-1].recovery.end
        return None

    @property
    def delivered_contact_x_m(self) -> float | None:
        """Where the contact actually ended up, against where it was aimed."""

        frame = self.end_frame
        return None if frame is None else float(frame.contact_xz_m[0])

    @property
    def end_hip_x_m(self) -> float | None:
        frame = self.end_frame
        return None if frame is None else float(frame.hip_xz_m[0])

    @property
    def residual_m(self) -> float | None:
        """How far short of the obstacle the **contact** stopped.

        Reported for reading; it is *not* what the transition should be given.
        Use :meth:`hip_advance_for_landing_m`, and see why there.
        """

        delivered = self.delivered_contact_x_m
        if delivered is None:
            return None
        return float(self.approach.obstacle_x_m - delivered)

    def hip_advance_for_landing_m(self,
                                  landing_contact_offset_m: float) -> float | None:
        """The hip advance that lands the **contact** on the obstacle.

        Handing the transition the contact-side residual is wrong and lands
        between 56 mm short and 62 mm long depending on the leg.  The reason is
        that the transition changes ``theta`` on the way -- 72.49 deg down to
        40.00 -- so the contact sits somewhere else under the hip when it lands
        than it did when it lifted off.  What the hip advance controls is the
        **hip**, so the sum has to be done there:

        ``contact_on_landing = hip_at_liftoff + advance + offset_on_landing``

        ``landing_contact_offset_m`` is that offset, read off the segment the
        transition is aiming at rather than assumed -- it happens to be 0.00 mm
        for this crossing's first segment, and assuming that would work here
        and break silently on the next crossing.
        """

        hip_x = self.end_hip_x_m
        if hip_x is None:
            return None
        return float(self.approach.obstacle_x_m
                     - float(landing_contact_offset_m) - hip_x)

    def as_dict(self) -> dict:
        frame = self.end_frame
        return {
            "leg": self.approach.leg.value,
            "cycles": len(self.cycles),
            "has_partial": self.partial is not None,
            "partial_requested_mm": self.approach.partial_distance_m * 1e3,
            "partial_delivered_mm": (
                None if self.partial is None
                else self.partial.contact_advance_m * 1e3),
            "end_contact_x_mm": (None if frame is None
                                 else float(frame.contact_xz_m[0]) * 1e3),
            "end_theta_deg": (None if frame is None
                              else float(np.rad2deg(frame.theta_rad))),
            "end_beta_deg": (None if frame is None
                             else float(np.rad2deg(frame.beta_rad))),
            "residual_mm": (None if self.residual_m is None
                            else self.residual_m * 1e3),
            "success": self.success,
            "failure_reason": self.failure_reason,
        }


def phase_start_beta_rad(phase: float, timing: GaitTiming2D,
                         posture: NominalPosture2D | None = None) -> float:
    """Where in its stroke a leg at ``phase`` already is.

    The gait's four legs are a quarter-cycle apart, and that quarter is
    *inside* a stroke, not a whole number of strokes.  Starting every leg at
    the arc's own beginning throws it away -- all four then lift off together
    and the walk stops being a walk (log 1.15: every leg began at
    ``beta = 39.84 deg``).
    """

    stroke = nominal_stroke_2d(posture)
    beta_start = float(stroke.frames[0].beta_rad)
    beta_end = float(stroke.frames[-1].beta_rad)

    # A leg at ``phase == stance_duty`` is exactly at liftoff, and a stroke
    # that starts there has nothing left to roll -- the segment comes out with
    # a zero ``beta_step`` and the schema refuses it, correctly.  Leave it one
    # roll step of stroke so the leg is *about* to lift off rather than
    # already gone: that is the same instant to within one step, and it keeps
    # the phase meaning instead of silently restarting the leg a whole
    # recovery earlier or later.
    steps = max(1, len(stroke.frames) - 1)
    ceiling = 1.0 - 1.0 / steps
    fraction = float(min(max(phase / timing.stance_duty, 0.0), ceiling))
    return beta_start + fraction * (beta_end - beta_start)


def phase_start_pose_2d(phase: float, timing: GaitTiming2D,
                        posture: NominalPosture2D | None = None
                        ) -> tuple[float, float, float]:
    """``(theta, beta, hip_z)`` a leg at ``phase`` is standing in.

    :func:`phase_start_beta_rad` gives the orientation; it is not enough on its
    own to *aim* a transition at, because the levelled posture solves theta
    against the held hip height at every beta, so a leg part way through the
    stroke is at a different theta from one at the arc's start.  Aiming at the
    arc start's theta and the phase's beta asks for a pose that is not on the
    stroke at all.
    """

    posture = NominalPosture2D() if posture is None else posture
    beta = phase_start_beta_rad(phase, timing, posture)
    stroke = nominal_stroke_2d(posture)
    hip_z = float(stroke.frames[0].hip_xz_m[1])
    theta = theta_for_hip_z_2d(posture, beta, hip_z)
    if theta is None:
        raise ValueError(
            f"no theta holds the hip at {hip_z * 1e3:.3f} mm at phase {phase}.")
    return float(theta), float(beta), hip_z


def resume_phases_2d(timing: GaitTiming2D,
                     posture: NominalPosture2D | None = None,
                     config: RecoveryConfig2D | None = None
                     ) -> dict[LegId, float]:
    """What phase each leg must resume at for the four to interleave again.

    **Why the crossing needs this at all.**  A leg enters the crossing at a
    fixed pose and leaves it at a fixed pose, so whatever phase it had going in
    is gone coming out.  Worse, the crossing is a fixed hip distance, so under
    position scheduling every leg spends the same time in it -- and the two
    legs that share a ``mount_x`` (the front pair, and the rear pair) reach the
    obstacle at the same body position and therefore at the same *instant*.
    They come out of the crossing in lockstep, and their recovery swings then
    happen together (log 1.16).

    **What fixes it.**  Only the relative phases matter, and those follow from
    ``mount_x`` alone: leg *i* leaves the crossing at body time
    ``K - mount_i / speed`` for one common ``K``, because all four legs leave it
    at the same *hip*.  So the phase a leg has to resume at is fixed up to one
    free constant shared by all four -- and that constant is what this picks,
    by placing the one unreachable band (a stroke can only be *shortened*, so a
    leg cannot resume later than ``stance_duty``) in the widest gap between the
    four demands.
    """

    posture = NominalPosture2D() if posture is None else posture
    cycle_hip = float(body_speed_m_s(timing, posture, config)
                      * timing.cycle_period_s)
    if cycle_hip <= 0.0:
        raise ValueError("a cycle that advances no hip has no phase to keep.")
    mounts = {m.leg: float(m.offset_body_xyz_m[0])
              for m in leg_mounts_2d(GAMMA_RAD)}
    duty = float(timing.stance_duty)

    # What each leg would need, before the common shift.
    #
    # The sign on the phase is not a convention to pick: the flat gait's own
    # schedule puts a leg's swing **earlier** the larger its phase (LF at 0.75
    # swings first, LH at 0.0 last), so a resume phase that reads ``+ phase``
    # walks the four legs round in the opposite order -- still a quarter cycle
    # apart, still stable-looking, and not the gait (log 1.19).
    demand = {leg: math.fmod(mounts[leg] / cycle_hip
                             - float(timing.phase_at(leg, 0.0)) + 2.0, 1.0)
              for leg in LEG_ORDER}

    # Put the unreachable band in the widest gap.  The band is ``(duty, 1)``:
    # a leg resumes by starting its stroke part way along, which can only make
    # its next liftoff *earlier*, never later.
    values = sorted(demand.values())
    best_gap, best_mid = -1.0, 0.0
    for index, low in enumerate(values):
        wraps = index + 1 == len(values)
        high = values[(index + 1) % len(values)] + (1.0 if wraps else 0.0)
        gap = high - low
        if gap > best_gap:
            best_gap, best_mid = gap, 0.5 * (low + high)
    band_mid = 0.5 * (duty + 1.0)
    shift = band_mid - best_mid

    out: dict[LegId, float] = {}
    for leg in LEG_ORDER:
        want = math.fmod(demand[leg] + shift + 2.0, 1.0)
        if want > duty + 1e-12:
            raise ValueError(
                f"{leg.value} would have to resume at phase {want:.4f}, past "
                f"the duty {duty:.4f}; a stroke cannot be lengthened.")
        out[leg] = float(duty - want)
    return out


def run_resumed_cycles_2d(
    cycles: int,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    *,
    hip_x_m: float,
    start_beta_rad: float,
    arc_start_beta_rad: float | None = None,
) -> list[NominalCycle2D]:
    """Nominal cycles whose **first** stroke begins part way along the arc.

    :func:`run_nominal_cycles_2d` cannot do this: it builds one cycle and
    translates it, so a partial first cycle would make every later one partial
    too.  And the recovery after a partial stroke has to be told where to land
    -- ``recovery_beta_target_2d`` aims one turn back from its own stroke's
    start, which for a partial stroke is the middle of the arc and not a ground
    contact at all (log 1.16).
    """

    posture = NominalPosture2D() if posture is None else posture
    out: list[NominalCycle2D] = []
    beta = float(start_beta_rad)
    hip_x = float(hip_x_m)
    for index in range(int(cycles)):
        stroke = run_foot_rim_roll_2d(posture, start_beta_rad=beta,
                                      hip_x_m=hip_x)
        partial = index == 0 and arc_start_beta_rad is not None
        recovery = run_recovery_swing_2d(
            stroke, config,
            beta_target_rad=(float(arc_start_beta_rad) - TWO_PI if partial
                             else None))
        out.append(NominalCycle2D(stroke, recovery))
        if not recovery.success:
            break
        beta = float(recovery.end.beta_rad)
        hip_x = float(recovery.end.hip_xz_m[0])
    return out


def run_approach_2d(
    approach: LegApproach2D,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    *,
    start_contact_x_m: float | None = None,
    start_beta_rad: float | None = None,
    max_cycles: int = 24,
) -> ApproachRun2D:
    """Roll this leg from where it starts to where the crossing begins.

    **Measured, not predicted.**  The first two attempts at this computed how
    many cycles would fit and then built that many; both were wrong, once by
    counting a stroke's contact advance instead of a cycle's and once by
    hitting a remainder no single stroke could close (log 1.11).  This walks
    instead: add a cycle while another whole one still fits, then close what is
    left.  The arithmetic that used to have to be right in advance is now just
    a stopping condition.

    ``start_beta_rad`` starts the leg part way through a stroke, which is what
    the gait phase means.  ``None`` starts at the arc's beginning.
    """

    posture = NominalPosture2D() if posture is None else posture
    start_x = (float(approach.start_contact_x_m)
               if start_contact_x_m is None else float(start_contact_x_m))
    target_hip = approach.target_hip_x_m

    # Where the first stroke's hip has to be for its contact to land on
    # ``start_x``.  Generated once at the origin and read off, because the
    # offset depends on where in the stroke the leg starts.
    probe = run_foot_rim_roll_2d(posture, start_beta_rad=start_beta_rad,
                                 hip_x_m=0.0)
    if not probe.success:
        return ApproachRun2D((), None, approach)
    hip_x0 = start_x - float(probe.frames[0].contact_xz_m[0])

    # Where a full stroke begins.  A leg that started part way through one
    # still has to *land* at the beginning of the next, because what follows is
    # a full stroke -- and ``recovery_beta_target_2d`` cannot know that: it
    # aims one turn back from wherever its own stroke started, which for a
    # partial stroke is the middle of the arc.  Landing there with the hip at
    # the levelled height is not a ground contact at all, and all four legs
    # failed with ``TOUCHDOWN_IS_NOT_A_VALID_GROUND_CONTACT`` until this was
    # passed explicitly (log 1.16).
    arc_start_beta = float(nominal_stroke_2d(posture).frames[0].beta_rad)

    cycles: list[NominalCycle2D] = []
    beta = start_beta_rad
    hip_x = hip_x0
    for index in range(int(max_cycles)):
        stroke = run_foot_rim_roll_2d(posture, start_beta_rad=beta,
                                      hip_x_m=hip_x)
        partial_start = (index == 0 and start_beta_rad is not None
                         and abs(float(stroke.start.beta_rad)
                                 - arc_start_beta) > 1e-9)
        recovery = run_recovery_swing_2d(
            stroke, config,
            beta_target_rad=(arc_start_beta - TWO_PI if partial_start
                             else None))
        if not recovery.success:
            cycles.append(NominalCycle2D(stroke, recovery))
            return ApproachRun2D(tuple(cycles), None, approach)
        landed = float(recovery.end.hip_xz_m[0])
        if landed > target_hip:
            break  # this cycle would overshoot; close the gap with a partial
        cycles.append(NominalCycle2D(stroke, recovery))
        beta = float(recovery.end.beta_rad)
        hip_x = landed

    partial = None
    remaining_hip = target_hip - hip_x
    if remaining_hip > 0.0:
        ratio = stroke_hip_per_contact_ratio(posture)
        # Aim short by a roll step: the transition can carry the hip forward
        # and nothing can carry it back.
        request = max(0.0, remaining_hip / ratio - float(posture.roll_step_m))
        # Then **check** it, because the conversion above cannot be trusted to
        # the millimetre.  ``max_distance_m`` is a contact distance and what
        # has to stop short is the hip; 1.610 is the whole stroke's average
        # ratio and the local one drifts either side of it, and the stroke
        # stops on the first step at or past the request rather than before
        # it.  Together those left RF overshooting by 0.553 mm -- and an
        # overshoot is fatal, not approximate: the leg was refused outright
        # with "the approach overshot ... the body does not reverse", so the
        # run came back with three legs and no fourth (log 1.19).
        while request > 0.0:
            attempt = run_foot_rim_roll_2d(
                posture, start_beta_rad=beta, hip_x_m=hip_x,
                max_distance_m=request)
            partial = attempt
            if not attempt.success or not attempt.frames:
                break
            if float(attempt.end.hip_xz_m[0]) <= target_hip:
                break
            partial = None
            request -= float(posture.roll_step_m)
    return ApproachRun2D(tuple(cycles), partial, approach)


def approach_run_rows(runs: dict[LegId, ApproachRun2D]) -> list[dict]:
    return [{"row_kind": "approach_run", **runs[leg].as_dict()}
            for leg in LEG_ORDER if leg in runs]


# --------------------------------------------------------------------------
# Carrying the crossing onto the leg's own coordinates
# --------------------------------------------------------------------------


def _shift_xz(value, dx_m: float):
    return None if value is None else (float(value[0]) + dx_m, float(value[1]))


def _shift_point(contact: PointContact2D, dx_m: float,
                 dbeta_rad: float) -> PointContact2D:
    return replace(
        contact,
        point_world_xz_m=_shift_xz(contact.point_world_xz_m, dx_m),
        hip_xz_m=_shift_xz(contact.hip_xz_m, dx_m),
        beta_rad=float(contact.beta_rad) + dbeta_rad,
    )


def _shift_rolling(contact: RollingContact2D, dx_m: float,
                   dbeta_rad: float) -> RollingContact2D:
    lo, hi = contact.beta_range_rad
    return replace(
        contact,
        contact_start_xz_m=_shift_xz(contact.contact_start_xz_m, dx_m),
        contact_end_xz_m=_shift_xz(contact.contact_end_xz_m, dx_m),
        beta_range_rad=(float(lo) + dbeta_rad, float(hi) + dbeta_rad),
    )


def _shift_contact(contact, dx_m: float, dbeta_rad: float):
    if isinstance(contact, PointContact2D):
        return _shift_point(contact, dx_m, dbeta_rad)
    if isinstance(contact, RollingContact2D):
        return _shift_rolling(contact, dx_m, dbeta_rad)
    raise TypeError(f"cannot rebase a {type(contact).__name__}")


def rebase_segment_2d(segment: MotionSegment2D, dx_m: float,
                      dbeta_rad: float) -> MotionSegment2D:
    """One crossing segment, moved onto the leg's world x and beta counter.

    ``theta`` is untouched: it is a joint angle, not a coordinate.  So is the
    contact's ``z`` -- the obstacle is where it is, and sliding a crossing
    sideways must not also slide it up.
    """

    return replace(
        segment,
        start_contact=_shift_contact(segment.start_contact, dx_m, dbeta_rad),
        end_contact=_shift_contact(segment.end_contact, dx_m, dbeta_rad),
        rolling=(None if segment.rolling is None
                 else _shift_contact(segment.rolling, dx_m, dbeta_rad)),
    )


def rebase_sequence_2d(sequence: MotionSequence2D, rebase: CrossingRebase2D,
                       ) -> MotionSequence2D:
    """The whole crossing, carried onto one leg's chain."""

    return replace(sequence, segments=tuple(
        rebase_segment_2d(seg, rebase.hip_x_offset_m, rebase.beta_offset_rad)
        for seg in sequence.segments))


# --------------------------------------------------------------------------
# One leg's whole world-registered chain
# --------------------------------------------------------------------------

#: What the two crossing transitions are called in the chain.  They are
#: ``RECOVERY_SWING`` motions -- the same generator, the same three (or four)
#: ramps -- but naming them so lets a reader of the segment table see *why*
#: the leg left the ground, which is the whole point of the phase labels.
ENTRY_TRANSITION_LABEL = "CROSSING_ENTRY_TRANSITION"
EXIT_TRANSITION_LABEL = "CROSSING_EXIT_TRANSITION"


@dataclass(frozen=True)
class WorldLegChain2D:
    """One leg's segments, placed in the world, with the crossing registered."""

    leg: LegId
    segments: tuple
    frames: tuple
    source_id: str
    approach: ApproachRun2D
    rebase: CrossingRebase2D | None
    entry_hip_advance_m: float
    exit_lift_m: float
    landing_contact_x_m: float | None
    success: bool
    failure_reason: str | None = None

    def as_dict(self) -> dict:
        return {
            "leg": self.leg.value,
            "segments": len(self.segments),
            "frames": len(self.frames),
            "approach_cycles": len(self.approach.cycles),
            "entry_hip_advance_mm": self.entry_hip_advance_m * 1e3,
            "exit_lift_um": self.exit_lift_m * 1e6,
            "landing_contact_x_mm": (None if self.landing_contact_x_m is None
                                     else self.landing_contact_x_m * 1e3),
            "obstacle_x_mm": self.approach.approach.obstacle_x_m * 1e3,
            "landing_error_mm": (
                None if self.landing_contact_x_m is None
                else (self.landing_contact_x_m
                      - self.approach.approach.obstacle_x_m) * 1e3),
            "rebase_dx_mm": (None if self.rebase is None
                             else self.rebase.hip_x_offset_m * 1e3),
            "rebase_turns": (None if self.rebase is None
                             else self.rebase.beta_turns),
            "success": self.success,
            "failure_reason": self.failure_reason,
        }


def build_world_leg_chain_2d(
    leg: LegId,
    sequence: MotionSequence2D,
    approach: LegApproach2D,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    *,
    cycles_after: int = 1,
    timing: GaitTiming2D | None = None,
    resume_phase: float | None = None,
) -> WorldLegChain2D:
    """Roll to the obstacle, transition in, cross it, transition out, carry on.

    Every piece here was measured on its own first (log 1.11); this is the
    assembly and nothing more.  The three joins it has to make come out at
    0.000000 mm and 0.000000 deg, against the 1014 mm / 1040 deg and
    627 mm / 325 deg they were before any of this existed.
    """

    posture = NominalPosture2D() if posture is None else posture
    source = f"day12_world:{leg.value}"
    entry = sequence.segments[0].start_contact
    exit_c = sequence.segments[-1].end_contact
    landing_offset = float(entry.point_world_xz_m[0] - entry.hip_xz_m[0])

    run = run_approach_2d(
        approach, posture, config,
        start_beta_rad=(None if timing is None else
                        phase_start_beta_rad(approach.phase, timing, posture)))
    if not run.success:
        return WorldLegChain2D(leg, (), (), source, run, None, 0.0, 0.0, None,
                               False, f"approach: {run.failure_reason}")

    segments: list = []
    frames: list = []

    def add_roll(stroke, label):
        segments.append(roll_segment_2d(stroke, source_id=source,
                                        frame_offset=len(frames),
                                        phase_label=label))
        frames.extend(stroke.frames)

    def add_swing(swing, kind, label):
        segments.append(swing_segment_2d(swing, source_id=source,
                                         frame_offset=len(frames),
                                         kind=kind, phase_label=label))
        frames.extend(swing.frames)

    for cycle in run.cycles:
        add_roll(cycle.stroke, "NOMINAL_FOOT_RIM_ROLL")
        add_swing(cycle.recovery, SegmentKind.RECOVERY_SWING,
                  "NOMINAL_RECOVERY_SWING")
    if run.partial is not None:
        add_roll(run.partial, "APPROACH_FOOT_RIM_ROLL")

    # -- into the crossing --------------------------------------------------
    advance = run.hip_advance_for_landing_m(landing_offset)
    if advance is None or advance < 0.0:
        return WorldLegChain2D(
            leg, tuple(segments), tuple(frames), source, run, None, 0.0, 0.0,
            None, False,
            f"the approach overshot: it would need a hip advance of "
            f"{0.0 if advance is None else advance * 1e3:.3f} mm, and the body "
            "does not reverse")
    entry_config = replace(RecoveryConfig2D() if config is None else config,
                           hip_advance_m=float(advance))
    # Where the approach actually left the leg.
    #
    # With a partial stroke that is the stroke itself.  Without one the leg is
    # standing where its last **recovery** put it -- and using the cycle's
    # stroke there restarts the transition a whole recovery earlier, so the
    # transition's window overlapped the recovery's and the schedule refused
    # the leg outright ("LH has overlapping segments").
    if run.partial is not None:
        last_stroke = run.partial
    else:
        landed_frame = run.cycles[-1].recovery.end
        last_stroke, _stand_lift = standing_stroke_at_2d(
            posture, float(landed_frame.theta_rad),
            float(landed_frame.beta_rad),
            float(landed_frame.hip_xz_m[0]), float(landed_frame.hip_xz_m[1]))
        if not last_stroke.success:
            return WorldLegChain2D(
                leg, tuple(segments), tuple(frames), source, run, None,
                float(advance), 0.0, None, False,
                f"the approach's landing pose is not a standing one: "
                f"{last_stroke.stop_reason}")
    entry_target = forward_beta_for_orientation(float(run.end_frame.beta_rad),
                                                float(entry.beta_rad))
    entry_swing = run_posture_transition_2d(
        last_stroke, float(entry.theta_rad), entry_target, entry_config,
        hip_z_target_m=float(entry.hip_xz_m[1]))
    if not entry_swing.success:
        return WorldLegChain2D(
            leg, tuple(segments), tuple(frames), source, run, None,
            float(advance), 0.0, None, False,
            f"entry transition: {entry_swing.failure_reason}")
    add_swing(entry_swing, SegmentKind.RECOVERY_SWING, ENTRY_TRANSITION_LABEL)
    landed = entry_swing.frames[-1]

    # -- the crossing, on this leg's coordinates ----------------------------
    rebase = crossing_rebase_2d(
        from_theta_rad=float(landed.theta_rad),
        from_beta_rad=float(landed.beta_rad),
        from_hip_x_m=float(landed.hip_xz_m[0]),
        to_theta_rad=float(entry.theta_rad), to_beta_rad=float(entry.beta_rad),
        to_hip_x_m=float(entry.hip_xz_m[0]))
    crossing = rebase_sequence_2d(sequence, rebase)
    segments.extend(crossing.segments)

    # -- back out of it -----------------------------------------------------
    moved_exit = crossing.segments[-1].end_contact
    stand, lift = standing_stroke_at_2d(
        posture, float(moved_exit.theta_rad), float(moved_exit.beta_rad),
        float(moved_exit.hip_xz_m[0]), float(moved_exit.hip_xz_m[1]))
    if not stand.success:
        return WorldLegChain2D(
            leg, tuple(segments), tuple(frames), source, run, rebase,
            float(advance), float(lift), None, False,
            f"crossing exit is not a standing pose: {stand.stop_reason}")
    # What pose the crossing hands the gait back in.
    #
    # The arc's start is the obvious answer and it is what costs the gait its
    # phase: every leg then leaves the crossing in the *same* state, and the
    # two that share a ``mount_x`` leave it at the same instant as well, so
    # their recoveries happen together from there on (log 1.16).  Handing each
    # leg back part way along the stroke instead is what puts the quarter-cycle
    # between them again -- ``resume_phase`` says how far along.
    nominal_start = nominal_stroke_2d(posture).frames[0]
    if resume_phase is None or timing is None:
        resume_theta = float(nominal_start.theta_rad)
        resume_beta = float(nominal_start.beta_rad)
        resume_hip_z = float(nominal_start.hip_xz_m[1])
    else:
        resume_theta, resume_beta, resume_hip_z = phase_start_pose_2d(
            float(resume_phase), timing, posture)
    partial_resume = abs(resume_beta - float(nominal_start.beta_rad)) > 1e-9
    exit_target = forward_beta_for_orientation(float(moved_exit.beta_rad),
                                               resume_beta)
    exit_swing = run_posture_transition_2d(
        stand, resume_theta, exit_target, config,
        hip_z_target_m=resume_hip_z,
        lift_hip_before_rotation=True)
    if not exit_swing.success:
        return WorldLegChain2D(
            leg, tuple(segments), tuple(frames), source, run, rebase,
            float(advance), float(lift), None, False,
            f"exit transition: {exit_swing.failure_reason}")
    add_swing(exit_swing, SegmentKind.RECOVERY_SWING, EXIT_TRANSITION_LABEL)

    # -- and on ------------------------------------------------------------
    # Carry the revolution counter, not just the position: the leg has been
    # turning the whole time and the run that resumes has to say so.
    landed_out = exit_swing.frames[-1]
    if partial_resume:
        # The arc's start, expressed in the counter the leg is actually on.
        # Beta is a revolution counter, so "the arc start" is a whole number of
        # turns away from the raw value the nominal stroke reports.
        turns = round((float(exit_target) - resume_beta) / TWO_PI)
        after = run_resumed_cycles_2d(
            cycles_after, posture, config,
            hip_x_m=float(landed_out.hip_xz_m[0]),
            start_beta_rad=float(landed_out.beta_rad),
            arc_start_beta_rad=(float(nominal_start.beta_rad)
                                + turns * TWO_PI))
    else:
        after = run_nominal_cycles_2d(
            cycles_after, posture, config,
            hip_x_m=float(landed_out.hip_xz_m[0]),
            start_beta_rad=float(landed_out.beta_rad))
    for cycle in after:
        if not cycle.recovery.success:
            break
        add_roll(cycle.stroke, "NOMINAL_FOOT_RIM_ROLL")
        add_swing(cycle.recovery, SegmentKind.RECOVERY_SWING,
                  "NOMINAL_RECOVERY_SWING")

    return WorldLegChain2D(
        leg, tuple(segments), tuple(frames), source, run, rebase,
        float(advance), float(lift),
        float(landed.contact_xz_m[0]), True, None)


def chain_rows(chains: dict[LegId, WorldLegChain2D]) -> list[dict]:
    return [{"row_kind": "world_chain", **chains[leg].as_dict()}
            for leg in LEG_ORDER if leg in chains]


# --------------------------------------------------------------------------
# Into the Step 4 contract
# --------------------------------------------------------------------------

#: What phase the two transitions are reported under.  They belong to the
#: crossing -- a leg only makes them because there is an obstacle -- so they
#: are labelled with the crossing's halves rather than as nominal recoveries,
#: and a reader of the phase column can see the crossing begin and end.
ENTRY_TRANSITION_PHASE = TransitionPhase.ASCENT
EXIT_TRANSITION_PHASE = TransitionPhase.DESCENT


def world_leg_plan_2d(
    leg: LegId,
    composed: ComposedSequence2D,
    approach: LegApproach2D,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    *,
    cycles_after: int = 1,
    tolerance: ChainTolerance2D | None = None,
    timing: GaitTiming2D | None = None,
    resume_phase: float | None = None,
) -> LegPlan2D:
    """:func:`build_world_leg_chain_2d`, in the shape Step 4 hands on.

    The same ``LegPlan2D`` every other Day 12 step consumes, so nothing
    downstream has to know the crossing was registered rather than pasted.
    A chain that failed to build comes back as a plan carrying its refusal --
    not as an exception and not as a shorter chain that looks complete.
    """

    ascent, descent = (None, None)
    if composed.strategy is not None:
        ascent, descent = STRATEGY_HALVES[composed.strategy]

    if composed.sequence is None:
        return LegPlan2D(
            leg=leg, phased=(), chain=None, strategy=composed.strategy,
            ascent_strategy=ascent, descent_strategy=descent,
            unresolved=composed.unresolved,
            refusal=composed.refusal or "no crossing to register",
            breaks=(), frames={}, notes=composed.notes)

    chain = build_world_leg_chain_2d(leg, composed.sequence, approach, posture,
                                     config, cycles_after=cycles_after,
                                     timing=timing, resume_phase=resume_phase)
    if not chain.success:
        return LegPlan2D(
            leg=leg, phased=(), chain=None, strategy=composed.strategy,
            ascent_strategy=ascent, descent_strategy=descent,
            unresolved=composed.unresolved,
            refusal=f"world registration failed -- {chain.failure_reason}",
            breaks=(), frames={}, notes=composed.notes)

    phased = []
    for segment in chain.segments:
        label = segment.phase_label or ""
        if label == ENTRY_TRANSITION_LABEL:
            phase = ENTRY_TRANSITION_PHASE
        elif label == EXIT_TRANSITION_LABEL:
            phase = EXIT_TRANSITION_PHASE
        elif label.startswith("NOMINAL") or label.startswith("APPROACH_FOOT"):
            phase = TransitionPhase.NOMINAL_BEFORE
        else:
            phase = phase_of_kind(segment.kind, after_transition=False)
        phased.append(PhasedSegment2D(segment, phase, "world"))

    segments = tuple(p.segment for p in phased)
    segment_chain = SegmentChain2D(
        leg_id=leg.value, segments=segments,
        tolerance=ChainTolerance2D() if tolerance is None else tolerance,
        unresolved=composed.unresolved,
        notes=("Day 12 A5: the crossing registered onto this leg's own world "
               "position, with a generated transition at each end."),
    )
    _, breaks = chain_boundaries_2d(segments, segment_chain.tolerance)
    return LegPlan2D(
        leg=leg, phased=tuple(phased), chain=segment_chain,
        strategy=composed.strategy, ascent_strategy=ascent,
        descent_strategy=descent, unresolved=composed.unresolved,
        refusal=composed.refusal, breaks=tuple(breaks),
        frames={chain.source_id: chain.frames}, notes=composed.notes)


def crossing_rebase_dx_2d(composed: ComposedSequence2D,
                          terrain: SharedTerrainSpec2D) -> float:
    """How far the crossing sequence is shifted to sit on the real obstacle.

    The same for every leg: the approach aims the **hip** at
    ``obstacle_x - landing_offset``, and that target is obstacle-relative, not
    mount-relative.  So the crossing occupies one hip-x window, and all four
    legs pass through it -- at different times, because their hips arrive at
    different times, which is exactly where the front/rear overlap comes from.
    """

    entry = composed.sequence.segments[0].start_contact
    landing_offset = float(entry.point_world_xz_m[0] - entry.hip_xz_m[0])
    target_hip_x = float(terrain.x_start_m) - landing_offset
    return target_hip_x - float(entry.hip_xz_m[0])


def crossing_body_profile_2d(
    composed: ComposedSequence2D,
    terrain: SharedTerrainSpec2D,
    nominal_body_z_m: float,
    *,
    samples: int = 601,
) -> HipZProfile2D:
    """The body height the crossing demands, as a function of **body x**.

    Built from the crossing sequence itself rather than from a first build
    pass: the sequence replays recorded frames, so its hip heights do not
    depend on anything the four-leg assembly decides.  That is what keeps this
    one pass instead of two.

    **Where two legs disagree.**  The front pair and the rear pair are both in
    the crossing for 148 mm of body travel (the crossing is 658 mm of hip
    travel against a 510 mm wheelbase), and there they demand different
    heights -- up to the crossing's own 54.241 mm span (log 1.21-10, 1.21-11).
    No single body height satisfies both; that is a real conflict and Step 5
    still reports it.  For *generation* a single number is needed anyway, and
    this takes the **highest** of the demands that are actually being made.

    That is a choice between two crossing legs, and only between them.  It is
    deliberately **not** a maximum against the nominal height: the whole
    crossing runs *below* the flat stance (body 86.6-140.9 mm against 162.3),
    so maxing against nominal erases every demand and hands back a flat
    profile -- which is what the first version of this did, and the profile
    came out 162.2818 mm from end to end.
    """

    dx = crossing_rebase_dx_2d(composed, terrain)
    mounts = [float(m.offset_body_xyz_m[0]) for m in leg_mounts_2d(GAMMA_RAD)]

    knots: list[tuple[float, float]] = []
    for segment in composed.sequence.segments:
        for contact in (segment.start_contact, segment.end_contact):
            hip_x = float(contact.hip_xz_m[0]) + dx
            body_z = float(contact.hip_xz_m[1]) - HIP_TO_BODY_Z_M
            knots.append((hip_x, body_z))
    knots.sort()
    hip_xs = np.array([k[0] for k in knots], dtype=float)
    body_zs = np.array([k[1] for k in knots], dtype=float)

    # One body-x grid wide enough to hold every leg's crossing window.
    low = min(hip_xs) - max(mounts) - 0.05
    high = max(hip_xs) - min(mounts) + 0.05
    grid = np.linspace(low, high, int(samples))

    demanded = np.full_like(grid, float(nominal_body_z_m))
    claimed = np.zeros_like(grid, dtype=bool)
    for mount in mounts:
        # This leg is crossing while its hip is inside the window, which in
        # body coordinates is the window shifted by its own mount.
        inside = (grid >= hip_xs[0] - mount) & (grid <= hip_xs[-1] - mount)
        if not inside.any():
            continue
        here = np.interp(grid[inside] + mount, hip_xs, body_zs)
        # Take the higher of the *crossing* demands where two overlap, and the
        # crossing demand outright where only one leg is in it.  Comparing
        # against the nominal height instead would throw the whole crossing
        # away, because all of it is below the flat stance.
        previous = np.where(claimed[inside], demanded[inside], -np.inf)
        demanded[inside] = np.maximum(previous, here)
        claimed[inside] = True

    return HipZProfile2D(tuple(float(v) for v in grid),
                         tuple(float(v) for v in demanded))


def leg_hip_z_profile_2d(body_profile: HipZProfile2D,
                         leg: LegId) -> HipZProfile2D:
    """The body profile as **this leg's hip** sees it.

    ``hip_x = body_x + mount_x`` and ``hip_z = body_z + HIP_TO_BODY_Z_M``, so
    the conversion is two shifts.  It is a per-leg view of one shared body
    trajectory -- which is the whole point: the legs follow the body, the body
    does not follow the legs.
    """

    mount = float({m.leg: m.offset_body_xyz_m[0]
                   for m in leg_mounts_2d(GAMMA_RAD)}[leg])
    return HipZProfile2D(
        tuple(x + mount for x in body_profile.hip_x_m),
        tuple(z + HIP_TO_BODY_Z_M for z in body_profile.hip_z_m))


def world_leg_plans_2d(
    composed: ComposedSequence2D,
    terrain: SharedTerrainSpec2D,
    timing: GaitTiming2D,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    *,
    cycles_after: int = 1,
    follow_body: bool = False,
    crossing_stagger_m: dict[LegId, float] | None = None,
) -> dict[LegId, LegPlan2D]:
    """All four legs, each registered to where **it** meets the obstacle.

    ``follow_body`` generates the nominal stance segments against the body
    height the crossing actually demands, instead of against the flat held
    height.  **Default off, and it is off because it does not work yet** --
    a position-dependent posture breaks the translation invariance three
    callers rely on (log 1.23).  Left wired so the next attempt starts from a
    measurement rather than from scratch.
    """

    if composed.sequence is None:
        return {leg: world_leg_plan_2d(leg, composed,
                                       _flat_approach_2d(leg, terrain))
                for leg in LEG_ORDER}
    entry = composed.sequence.segments[0].start_contact
    landing_offset = float(entry.point_world_xz_m[0] - entry.hip_xz_m[0])
    approaches = leg_approaches_2d(terrain, timing, posture, config=config,
                                   landing_contact_offset_m=landing_offset,
                                   crossing_stagger_m=crossing_stagger_m)
    phases = resume_phases_2d(timing, posture, config)

    # The stance legs follow the body; the body does not follow them.
    #
    # Without this every nominal leg is generated at the flat held height while
    # the crossing leg drags the body down to 183 mm -- the three legs still on
    # the ground are then standing 36 mm above where the body says they are.
    # ``body_profile`` says where the body actually goes, and each leg is
    # generated against its own view of it (log 1.23).
    postures = {leg: posture for leg in LEG_ORDER}
    if follow_body:
        base = NominalPosture2D() if posture is None else posture
        nominal_body_z = float(base.held_hip_z_at(0.0) or
                               nominal_stroke_2d(base).frames[0].hip_xz_m[1]
                               ) - HIP_TO_BODY_Z_M
        body_profile = crossing_body_profile_2d(composed, terrain,
                                                nominal_body_z)
        postures = {leg: replace(base, hold_hip_z_m=None,
                                 hold_hip_z_profile=leg_hip_z_profile_2d(
                                     body_profile, leg))
                    for leg in LEG_ORDER}

    return {leg: world_leg_plan_2d(leg, composed, approaches[leg],
                                   postures[leg],
                                   config, cycles_after=cycles_after,
                                   timing=timing, resume_phase=phases[leg])
            for leg in LEG_ORDER}


def _flat_approach_2d(leg: LegId,
                      terrain: SharedTerrainSpec2D) -> LegApproach2D:
    """A placeholder approach for a run with no crossing to register."""

    return LegApproach2D(
        leg=leg, phase=0.0, start_contact_x_m=0.0,
        obstacle_x_m=float(terrain.x_start_m) if terrain else 0.0,
        whole_strokes=0, partial_distance_m=0.0,
        cycle_contact_advance_m=1.0)


# --------------------------------------------------------------------------
# Time from position
# --------------------------------------------------------------------------


def body_speed_m_s(timing: GaitTiming2D,
                   posture: NominalPosture2D | None = None,
                   config: RecoveryConfig2D | None = None) -> float:
    """How fast the body travels, from the gait's own cycle.

    One nominal cycle advances the hip by the stroke's advance plus the
    recovery's, and the gait says a cycle takes ``cycle_period_s``.  That is
    the whole definition -- there is no separate speed to choose.
    """

    cycles = run_nominal_cycles_2d(1, posture, config)
    if not cycles or not cycles[0].recovery.success:
        raise ValueError("the nominal cycle does not complete; no speed to take.")
    cycle = cycles[0]
    advance = float(cycle.recovery.end.hip_xz_m[0]
                    - cycle.stroke.start.hip_xz_m[0])
    return advance / float(timing.cycle_period_s)


def swing_hip_advance_m(timing: GaitTiming2D,
                        posture: NominalPosture2D | None = None) -> float:
    """What ``RecoveryConfig2D.hip_advance_m`` has to be for the gait to close.

    Zero -- the default -- makes a recovery take **no time at all** once time
    is read from position, because the hip does not move during it.  The body
    does move: it moves by exactly one swing window's worth, and the three
    legs still on the ground are what move it.  So this is not a free
    parameter, it is the value that makes the cycle add up:

    ``stroke_advance + swing_advance = cycle_period * body_speed``

    which resolves to ``stroke_advance * (1 - duty) / duty``.  At duty 0.85
    that is 57.515 mm, and it reproduces the gait's own stance and swing
    durations to the millisecond (log 1.15).
    """

    stroke = nominal_stroke_2d(posture)
    duty = float(timing.stance_duty)
    return float(stroke.hip_advance_m * (1.0 - duty) / duty)


@dataclass(frozen=True)
class SpeedZone2D:
    """The body travels at ``speed_m_s`` while its x is inside ``[x0, x1)``.

    Day 13.  ``world_schedule_2d`` maps position to time with a single speed,
    so a plan crosses an obstacle at exactly the speed it walks flat ground.
    The project owner has no speed requirement overall but does need the
    approach run at a normal pace: creeping the whole way is both slow and, on
    a real machine, more likely to tip than a normal walk.

    Slowing the body is safe in a way that retiming one *leg* is not (log 25).
    The map is applied to the **body's** x, so every leg reads the same clock
    off the same body: the four still agree about where the body is, and
    ``world_x_spread_m`` stays at zero.  What changes is only how long the body
    takes to get from one x to the next.
    """

    x_start_m: float
    x_end_m: float
    speed_m_s: float

    def __post_init__(self) -> None:
        if not np.isfinite(self.speed_m_s) or self.speed_m_s <= 0.0:
            raise ValueError("a zone's speed must be finite and positive.")
        if self.x_end_m <= self.x_start_m:
            raise ValueError("a zone covers a positive span of x.")


def time_at_body_x_2d(body_x_m: float, origin_m: float, base_speed_m_s: float,
                      zones: Sequence[SpeedZone2D] = ()) -> float:
    """When the body reaches ``body_x_m``, integrating 1/speed over the path.

    With no zones this is exactly ``(x - origin) / speed``, so an empty
    ``zones`` reproduces every Day 12 number.  With zones it is the same
    integral taken piecewise -- continuous in ``x`` by construction, because
    each piece starts where the previous one ended, so no zone boundary can
    introduce a jump in time or a gap in the body's motion.
    """

    x = float(body_x_m)
    origin = float(origin_m)
    if x <= origin:
        return 0.0
    ordered = sorted(zones, key=lambda z: z.x_start_m)
    for a, b in zip(ordered, ordered[1:]):
        if b.x_start_m < a.x_end_m - 1e-12:
            raise ValueError(
                f"speed zones overlap: [{a.x_start_m}, {a.x_end_m}) and "
                f"[{b.x_start_m}, {b.x_end_m}).")
    total = 0.0
    cursor = origin
    for zone in ordered:
        lo, hi = max(zone.x_start_m, cursor), min(zone.x_end_m, x)
        if hi <= lo:
            continue
        total += (lo - cursor) / base_speed_m_s      # base speed up to the zone
        total += (hi - lo) / zone.speed_m_s          # the zone itself
        cursor = hi
        if cursor >= x:
            return float(total)
    return float(total + (x - cursor) / base_speed_m_s)


def whole_body_schedule_2d(
    plans: dict[LegId, LegPlan2D],
    timing: GaitTiming2D,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    *,
    body_x_start_m: float | None = None,
    speed_zones: Sequence[SpeedZone2D] = (),
    max_airborne: int = 1,
) -> tuple[FourLegSchedule2D, tuple[dict, ...]]:
    """Schedule the four legs **as one machine**, not four legs sharing a clock.

    Day 13, at the project owner's direction: *"although it began as one-leg
    planning, we are at the whole machine now, so the whole machine should be
    what decides -- judging when a leg may swing from position alone is plainly
    unworkable."*

    :func:`world_schedule_2d` derives each segment's time from its own hip
    position.  That was the right fix for what it addressed (index scheduling
    stretched one leg's run to 14.2 s against another's 7.6 s and left the four
    disagreeing about the body by 602.6 mm, log 1.14), and it does guarantee
    the thing that matters: every leg reads the same clock off the same body.

    But it also makes a leg's *time* identical to its *claim about where the
    body is*, and the two legs of a pair share a ``mount_x``.  Their crossings
    therefore occupy the same interval, they always swing together, and
    ``at_most_one_airborne`` cannot hold -- measured 75 coincident samples on
    the 100 mm swing crossing.  Four attempts to fix that inside position
    scheduling all failed for one reason: any retiming of one leg makes it
    disagree with the other three (log 25).

    The way out is not to retime one leg but to move **the body's clock**.  The
    body's progress along its path is still the single source of time -- so the
    four still agree, by construction -- but the *rate* at which that progress
    is spent may vary, and a leg that would be the second one airborne holds
    the whole machine until the first is down.

    Concretely: walk the timeline forward; whenever releasing the next swing
    would put more than ``max_airborne`` legs in the air, delay the body's
    clock (not one leg's) until the earlier swing has landed.  Every leg sees
    the same delay, so ``world_x_spread_m`` is untouched -- which is exactly
    what the four failed attempts could not preserve.

    Returns the schedule and one record per hold, so a caller can report what
    the coordination cost rather than discovering it in the trajectory.
    """

    base = world_schedule_2d(plans, timing, posture, config,
                             body_x_start_m=body_x_start_m,
                             speed_zones=speed_zones)
    scheduled = sorted(base.scheduled, key=lambda s: (s.start_s, s.leg.value))
    holds: list[dict] = []

    # Airborne intervals, in the order they begin.
    swings = [s for s in scheduled if s.mode is LegMode.AIRBORNE]
    if not swings:
        return base, ()

    # Walk the swings in time.  ``shift`` accumulates the delay applied to the
    # whole machine so far; ``landed`` is when the currently airborne legs come
    # down, in shifted time.
    shift = 0.0
    airborne_until: list[tuple[float, LegId]] = []
    for swing in swings:
        start = swing.start_s + shift
        end = swing.end_s + shift
        airborne_until = [(t, l) for t, l in airborne_until
                          if t > start + 1e-9 and l is not swing.leg]
        if len(airborne_until) >= max_airborne:
            # Hold the machine until the earliest of them has landed.
            release = min(t for t, _ in airborne_until)
            extra = release - start
            if extra > 1e-9:
                shift += extra
                start += extra
                end += extra
                holds.append({
                    "leg": swing.leg.value,
                    "delay_s": float(extra),
                    "from_s": float(swing.start_s + shift - extra),
                    "waited_for": ", ".join(
                        l.value for t, l in airborne_until if t <= release + 1e-9),
                })
                airborne_until = [(t, l) for t, l in airborne_until
                                  if t > start + 1e-9]
        airborne_until.append((end, swing.leg))

    if not holds:
        return base, ()

    # Re-time everything against the same accumulated shifts.  A segment's
    # delay is the shift in force when it starts, so the machine's whole
    # timeline stretches together and no leg moves relative to another.
    cuts = sorted((h["from_s"], h["delay_s"]) for h in holds)

    def shifted(t: float) -> float:
        out = t
        for cut, delay in cuts:
            if t >= cut - 1e-9:
                out += delay
        return out

    retimed = tuple(
        replace(seg, start_s=shifted(seg.start_s), end_s=shifted(seg.end_s))
        for seg in base.scheduled
    )
    return (FourLegSchedule2D(timing=base.timing, scheduled=retimed),
            tuple(holds))


def world_schedule_2d(
    plans: dict[LegId, LegPlan2D],
    timing: GaitTiming2D,
    posture: NominalPosture2D | None = None,
    config: RecoveryConfig2D | None = None,
    *,
    body_x_start_m: float | None = None,
    speed_zones: Sequence[SpeedZone2D] = (),
) -> FourLegSchedule2D:
    """Give every segment the time the **body** takes to cover its hip travel.

    Step 3 assigns time by segment index: each leg's chain is laid onto the
    gait's stance/swing windows in order, which is right only while all four
    legs have the same chain.  World registration breaks that -- a leg two
    strokes further from the obstacle has two more cycles, so the four chains
    are 17/17/21/21 segments long and laying them on a shared cycle stretches
    one leg's run to 14.2 s against another's 7.6 s (log 1.14).  The four then
    disagree about where the body is by 602.6 mm.

    Here time comes from position instead: the body advances at one speed, and
    a segment lasts exactly as long as the body takes to carry its hip from
    where the segment starts to where it ends.  Every leg is then reading the
    same clock off the same body, which is what "four legs, one machine" has
    to mean.
    """

    speed = body_speed_m_s(timing, posture, config)
    if speed <= 0.0:
        raise ValueError("a body that does not advance cannot time anything.")

    starts = [float(p.phased[0].segment.start_contact.hip_xz_m[0])
              - _mount_x(leg)
              for leg, p in plans.items() if p.phased]
    if not starts:
        raise ValueError("no leg has any segment to schedule.")
    origin = min(starts) if body_x_start_m is None else float(body_x_start_m)

    scheduled: list[ScheduledSegment2D] = []
    for leg, plan in plans.items():
        mount = _mount_x(leg)
        for index, phased in enumerate(plan.phased):
            segment = phased.segment
            body_at_start = (float(segment.start_contact.hip_xz_m[0]) - mount)
            body_at_end = (float(segment.end_contact.hip_xz_m[0]) - mount)
            start_s = time_at_body_x_2d(body_at_start, origin, speed,
                                        speed_zones)
            end_s = time_at_body_x_2d(body_at_end, origin, speed, speed_zones)
            if end_s < start_s:
                raise ValueError(
                    f"{leg.value} segment {index} ({segment.kind.value}) moves "
                    "the hip backwards; the body does not reverse.")
            scheduled.append(ScheduledSegment2D(
                leg=leg, window_index=index, segment_index=index,
                segment_kind=segment.kind, phase_label=segment.phase_label,
                mode=(LegMode.AIRBORNE if segment.kind.is_swing
                      else LegMode.STANCE),
                start_s=float(start_s), end_s=float(end_s),
                frame_count=len(segment.frames.indices),
                duration_is_assigned=True,
            ))

    # Close the gaps between a leg's consecutive segments.
    #
    # Each segment is timed from its **own** endpoints, so a boundary where the
    # generator's last frame and the next segment's first frame sit a grid step
    # apart becomes a hole in the timeline: measured at 0.66-4.32 mm of hip
    # travel per boundary, 20.65 mm over one leg's 19 segments, i.e. 17-20 ms
    # of wall clock each (log 27).
    #
    # Those holes are not idle time.  The hip really does cover that distance;
    # the generator simply did not emit a frame for it, and ``segment_chaining``
    # correctly passes them because they are one step of the generating grid,
    # well inside its measured 10 mm hand-over tolerance.  What is wrong is
    # recording them as *nothing*: a leg with no active segment is dropped by
    # ``body_trajectory_2d``, and ``planner_rows_2d`` then exported it at
    # ``theta = 0`` -- a fully folded leg, 17 deg below the joint minimum, which
    # is what made the legs snap in simulation.
    #
    # So the previous segment is held until the next one starts.  This adds no
    # motion and moves no hip: it says the leg stays where the segment left it
    # until the next segment picks it up, which is what physically happens.
    by_leg: dict[LegId, list[ScheduledSegment2D]] = {}
    for seg in scheduled:
        by_leg.setdefault(seg.leg, []).append(seg)
    closed: list[ScheduledSegment2D] = []
    for leg, own in by_leg.items():
        own.sort(key=lambda s: (s.start_s, s.segment_index))
        for current, following in zip(own, own[1:]):
            if following.start_s > current.end_s + 1e-12:
                current = replace(current, end_s=float(following.start_s))
            closed.append(current)
        closed.append(own[-1])
    return FourLegSchedule2D(timing=timing, scheduled=tuple(closed))


def airborne_overlaps_2d(
    schedule: FourLegSchedule2D, *, tolerance_s: float = 1e-9
) -> tuple[tuple[LegId, LegId, float, float], ...]:
    """Every interval where two legs are airborne at once.

    Reported rather than judged: ``(leg_a, leg_b, start_s, end_s)``.  This is
    what ``at_most_one_airborne`` fails on, expressed as intervals so a fix can
    be sized against it instead of against a count of failed samples.
    """

    windows: dict[LegId, list[list[float]]] = {}
    for seg in schedule.scheduled:
        if seg.mode is not LegMode.AIRBORNE:
            continue
        own = windows.setdefault(seg.leg, [])
        if own and seg.start_s <= own[-1][1] + tolerance_s:
            own[-1][1] = max(own[-1][1], float(seg.end_s))
        else:
            own.append([float(seg.start_s), float(seg.end_s)])

    out: list[tuple[LegId, LegId, float, float]] = []
    legs = [leg for leg in LEG_ORDER if leg in windows]
    for i, first in enumerate(legs):
        for second in legs[i + 1:]:
            for a0, a1 in windows[first]:
                for b0, b1 in windows[second]:
                    lo, hi = max(a0, b0), min(a1, b1)
                    if hi - lo > tolerance_s:
                        out.append((first, second, lo, hi))
    return tuple(sorted(out, key=lambda row: row[2]))


def delay_overlapping_swings_2d(
    schedule: FourLegSchedule2D, *, tolerance_s: float = 1e-9
) -> tuple[FourLegSchedule2D, tuple[dict, ...]]:
    """**Measured, and it does not work.**  Kept as the record of why.

    The aim was Day 13's: ``at_most_one_airborne`` fails because
    :func:`world_schedule_2d` takes a segment's time from its *position*, and
    the two legs of a pair share a ``mount_x``, so they get identical times and
    always swing together.  On the evaluation terrain that is four overlaps --
    two of 0.360 s (the ``RECOVERY_SWING`` of ``CROSSING_EXIT_TRANSITION``) and
    two slivers of 0.033 / 0.019 s where one leg leaves before the other lands.

    Four ways of delaying a swing were measured and all four fail (log 25.3):

    ``crossing_stagger_m``
        cross at different x.  Loses two legs outright, and skews the crossing,
        which the project owner rules out.
    translate one leg's remaining schedule
        ``world_x_spread_m`` goes 0.000 -> 84.068 mm and the body steps between
        -167 and +507 mm/s.
    stretch that leg's stance instead
        identical 84.068 mm: same arc over more time decouples its contact from
        the body just as thoroughly.
    pause the whole machine
        keeps the spread at 0.000 mm, but shifting every leg by one constant
        cannot change any leg-to-leg relation: twelve iterations later the four
        overlaps are the same widths, merely translated.

    The two requirements are in direct conflict under position scheduling,
    because there ``start_s = (hip_x - mount - origin) / speed`` makes a leg's
    *time* identical to its claim about the *body's position*.  Changing the
    relative timing of two legs necessarily makes them disagree about the body.

    So this is not a variant that has yet to be found: the fix has to change
    what produces the schedule -- the crossing sequence itself (Day 13 B2), or
    the position-derived timing that log 1.14 introduced to cure a 602.6 mm
    disagreement.  Neither is a scheduling-layer change.

    Raises ``NotImplementedError`` rather than returning a schedule that looks
    plausible; :func:`airborne_overlaps_2d` is the part worth keeping, and it
    is what any future attempt should be measured against.
    """

    raise NotImplementedError(
        "delaying a swing cannot fix at_most_one_airborne under position "
        "scheduling; see this function's docstring and log 25 for the four "
        "measured attempts and why they share a cause."
    )


def _mount_x(leg: LegId) -> float:
    return float({m.leg: m.offset_body_xyz_m[0]
                  for m in leg_mounts_2d(GAMMA_RAD)}[leg])
