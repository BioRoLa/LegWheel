"""Day 12 problem A4: why the support margin is zero, and what moves it.

Step 6 reported a minimum stability margin of **0.000 mm** and called all five
swings unstable.  The audit (log section 1.5) listed it as the one thing still
blocking a flat-ground run, with the instruction to *scan* rather than guess a
parameter.  This module is that scan.

**What the zero actually is.**  It is not a narrow stance and it is not a
missing ABAD joint.  A quadruped wave gait's longitudinal stability margin is
proportional to ``stance_duty - 3/4``; at exactly ``3/4`` it is zero by
construction, and ``GAIT_LIBRARY["Walk"]`` uses exactly ``0.75``.  Step 6
measured the critical duty, correctly.

**The three levers, and which of them exist here.**

``liftoff sequence``   Already optimal.  :data:`LIFTOFF_SEQUENCES` holds all
                      six distinct quadruped orders; the project's own gait is
                      the only one with a non-negative margin, and the other
                      five are -5 to -21 mm.  There is nothing to win here.
``stance duty``        Real, and the only one that changes the sign.  It is
                      bounded above by the motor budget: a shorter swing window
                      is the same recovery in less time.
``rolling stride``     Does **not** exist, and this is the finding.  What the
                      support polygon sees is the contact's excursion *relative
                      to its own hip* -- and a rolling stance spends most of the
                      hip's advance moving the contact forward too.  See
                      :class:`RollingStride2D`: 297.065 mm of hip advance leaves
                      only 94.607 mm of relative stride, and shortening the roll
                      scales both together, so it buys nothing.

That third one is why raising the duty helps so much less here than it would in
a walk: the margin is proportional to the relative stride, and rolling costs a
factor of about 3.1 of it.

**The bound on the duty, measured correctly.**  Buying margin costs swing time,
and a shorter swing is the same recovery driven faster, so the motor budget is
what caps the duty.  Measuring that demand by finite-differencing the sampled
whole-body trajectory does **not** work: ``leg_sample_at`` reads the nearest
generator frame, so the sampled signal is a staircase and its finite difference
is ``frame step / sample interval`` -- a number that grows as the grid is
refined and reports the discretisation instead of the motion.  The same gait
reads 48.1% at 241 samples and 127.7% at 1921.  :func:`frame_motor_rate_2d`
measures it between **consecutive generator frames at their assigned times**
instead, which is both sample-independent and what the hardware would execute.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from functools import lru_cache
from typing import Iterable, Sequence

import numpy as np

from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    HIP_TO_BODY_Z_M,
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import (
    LEG_ORDER,
    LegId,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    NominalPosture2D,
    nominal_stroke_2d,
    run_foot_rim_roll_2d,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    DEFAULT_MARGIN_FLOOR_M,
    support_triangle_at,
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import GaitTiming2D
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    FourLegPlan2D,
    LegPlan2D,
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    assemble_whole_body_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (
    MOTOR_MAX_RATE_RAD_S,
    joint_rates_2d,
    motor_rates_rad_s,
)

#: The duty at which a quadruped wave gait's stability margin is exactly zero.
#: ``GAIT_LIBRARY["Walk"]`` uses this value, so Step 6's 0.000 mm is the
#: textbook answer to the gait it was given -- not a defect in the geometry.
CRITICAL_STANCE_DUTY: float = 0.75

#: The stance duty this project runs the Hybrid gait at, chosen 2026-09-02
#: from the scan in this module (log section 1.6).  It is **not** a tuned
#: number and not a default carried over from anywhere: at
#: :data:`CRITICAL_STANCE_DUTY` the margin is identically zero, the margin
#: climbs monotonically above it, and 0.85 is where it stops being free --
#: 4.839 mm of margin for 72.0% of the motor budget at full speed.  Going
#: further buys little (0.875 gives 5.869 mm for 86.4%) and the planning floor
#: is out of reach either way, so this is the point where the trade turns.
HYBRID_STANCE_DUTY: float = 0.85

#: What the project owner's own measurement of the centre of mass leaves
#: uncertain, per axis, in metres (2026-09-02).
#:
#: The measurement itself is "the CoM really is at the centre of the robot".
#: That is a **fact input**, the same kind as the 330 rpm motor rating and the
#: continuous beta -- it is not derivable from anything in this repository,
#: which carries no mass model at all (``legwheel`` has no mass, no inertia;
#: ``COM_BIAS_X/Y`` are declared 0.0 as an assumption, and the reference URDF
#: has no ``<mass>`` tag).
#:
#: What the statement does **not** carry is a precision, so one is assumed
#: here rather than pretended away.  2 mm is a fair-to-slightly-conservative
#: reading of a balance or two-scale measurement, which resolves to about
#: 1 mm on this 22.2 kg machine.  **If the real measurement was cruder than
#: this, the floor below is too small** -- and the arithmetic to redo it is
#: one multiplication.
COM_UNCERTAINTY_PER_AXIS_M: float = 0.002

#: Margin lost per metre of centre-of-mass offset when **both** axes are off
#: at once, at :data:`HYBRID_STANCE_DUTY`.  Measured, and linear to four
#: figures across 0.5-5 mm: 0.6664 for a fore-aft offset alone, 0.7456 for a
#: lateral one alone, and this for both together.  The worst case is the one
#: that belongs in a floor, because nothing says the error picks one axis.
MARGIN_LOST_PER_COM_OFFSET: float = 1.412

#: The stability margin a swing must keep, for **this** gait.
#:
#: Derived, not chosen: it is exactly the margin the assumed CoM uncertainty
#: can eat, rounded up to a round number.  ``DEFAULT_MARGIN_FLOOR_M`` (10 mm)
#: is left alone -- every frozen Day 12 number was measured against it, and
#: log section 1.6 showed no duty and no cycle period can reach it.
#:
#: **What this floor does and does not cover.**  It covers the CoM being
#: somewhere other than where it was measured.  It does not cover dynamics,
#: contact forces, friction, terrain irregularity or joint error, because
#: Day 12 models none of them and inventing a number for them would make this
#: constant look better-founded than it is.  At duty 0.85 the margin is
#: 4.8394 mm, so what is left over for all of those together is about 1.8 mm.
HYBRID_MARGIN_FLOOR_M: float = 0.003

#: How far apart, in cycle fractions, the four liftoffs are placed.  Four
#: equally spaced liftoffs is what "wave gait" means; at the critical duty the
#: four swing windows tile the cycle exactly and no other spacing is even
#: available, so this is a definition rather than a tuned number.
LIFTOFF_SPACING: float = 0.25

_BY_INDEX = {leg.index: leg for leg in LEG_ORDER}
_FL = _BY_INDEX[0]
_FR = _BY_INDEX[1]
_RR = _BY_INDEX[2]
_RL = _BY_INDEX[3]

#: Every distinct quadruped liftoff order, up to the choice of which leg goes
#: first (the gait is cyclic, so fixing the front-left leg at position 0 loses
#: nothing).  ``project_walk`` is the one ``GAIT_LIBRARY["Walk"]`` encodes.
LIFTOFF_SEQUENCES: dict[str, tuple[LegId, LegId, LegId, LegId]] = {
    "project_walk": (_FL, _RR, _FR, _RL),
    "front_pair_first": (_FL, _FR, _RR, _RL),
    "front_pair_then_cross": (_FL, _FR, _RL, _RR),
    "diagonal_then_ipsilateral": (_FL, _RR, _RL, _FR),
    "ipsilateral_first": (_FL, _RL, _FR, _RR),
    "ipsilateral_then_cross": (_FL, _RL, _RR, _FR),
}


def phase_offsets_for_2d(
    order: Sequence[LegId],
    stance_duty: float,
    *,
    spacing: float = LIFTOFF_SPACING,
) -> tuple[float, float, float, float]:
    """``GaitTiming2D`` phase offsets that lift the legs off in ``order``.

    ``GaitTiming2D`` stores a phase *offset* per leg and derives the swing
    window as ``(stance_duty - offset) mod 1``.  Asking for a liftoff order is
    the natural way to state a gait, so this inverts that relation instead of
    leaving callers to hand-solve four modular equations -- which is how a scan
    over sequences silently ends up comparing a gait to itself.
    """

    order = tuple(order)
    if sorted(leg.index for leg in order) != [0, 1, 2, 3]:
        raise ValueError("a liftoff order names each of the four legs once.")
    offsets = [0.0, 0.0, 0.0, 0.0]
    for position, leg in enumerate(order):
        offsets[leg.index] = float((stance_duty - position * spacing) % 1.0)
    return tuple(offsets)  # type: ignore[return-value]


@lru_cache(maxsize=1)
def hybrid_posture_2d() -> NominalPosture2D:
    """The posture the Hybrid rolls in: ``theta`` modulated to hold the hip level.

    Day 13 measured this against the fixed-``theta`` alternative -- hip bob
    17.2871 mm down to 0.000176 mm, the contact advance identical to the
    micrometre, and Step 5's body-height conflicts from 227 to zero.  The
    height held is the **highest** point of the uncompensated arc, so the
    compensation only ever tucks the leg up and never asks for a reach the
    rim did not already have.

    Cached because it costs a full rolling stroke to find that height, and it
    is the same number every time.
    """

    fixed = NominalPosture2D()
    held = max(float(f.hip_xz_m[1]) for f in run_foot_rim_roll_2d(fixed).frames)
    return replace(fixed, hold_hip_z_m=float(held))


def hybrid_body_z_m() -> float:
    """Body height that puts the hip at the held height."""

    posture = hybrid_posture_2d()
    assert posture.hold_hip_z_m is not None
    return float(posture.hold_hip_z_m - HIP_TO_BODY_Z_M)


def derived_margin_floor_m(
    com_uncertainty_per_axis_m: float = COM_UNCERTAINTY_PER_AXIS_M,
) -> float:
    """The floor the assumed CoM uncertainty implies, before rounding.

    Exists so the chain from "how well do we know the CoM" to "how much margin
    must the gait keep" is executable rather than a comment.  Feeding it the
    real precision of a better measurement is the whole update.
    """

    return float(com_uncertainty_per_axis_m * MARGIN_LOST_PER_COM_OFFSET)


def hybrid_timing_2d(cycle_period_s: float = 2.4) -> GaitTiming2D:
    """The project's own walk sequence, at the duty section 1.6 chose.

    ``walk_timing_2d`` is deliberately left alone: it reads
    ``GAIT_LIBRARY["Walk"]`` verbatim, every frozen Day 12 number was measured
    with it, and it stays the answer to "what gait does the project have".
    This is the answer to "what gait does the Hybrid run", and the only thing
    that differs is the duty.
    """

    return GaitTiming2D(
        cycle_period_s=float(cycle_period_s),
        stance_duty=HYBRID_STANCE_DUTY,
        phase_offsets=phase_offsets_for_2d(LIFTOFF_SEQUENCES["project_walk"],
                                           HYBRID_STANCE_DUTY),
        gait_name=f"Walk@duty{HYBRID_STANCE_DUTY:g}",
    )


def liftoff_order_2d(timing: GaitTiming2D) -> tuple[LegId, ...]:
    """Read the liftoff order back off a timing, for checking a claim about it."""

    starts = [(timing.swing_window(leg)[0], leg) for leg in LEG_ORDER]
    starts.sort(key=lambda pair: pair[0])
    return tuple(leg for _, leg in starts)


# --------------------------------------------------------------------------
# What the support polygon actually sees
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class RollingStride2D:
    """Hip advance, contact advance, and the difference the polygon lives on.

    A planted foot gives the support polygon the *whole* hip advance: the hip
    moves and the contact does not.  A rolling foot gives it only what is left
    over.  :attr:`stride_loss_ratio` is how much of the stride the rolling
    stance spends on itself.
    """

    hip_advance_m: float
    contact_advance_m: float
    #: Hip advance minus contact advance: the contact's excursion relative to
    #: its own hip, which is the only part the support polygon responds to.
    relative_stride_m: float
    #: ``relative_stride_m`` at each fraction of the roll, to show whether
    #: rolling a shorter stroke would buy any of it back.  It does not.
    partial_relative_stride_m: tuple[float, ...]
    partial_roll_fraction: tuple[float, ...]

    @property
    def stride_loss_ratio(self) -> float:
        """How many times larger a planted-foot stride would be."""

        if self.relative_stride_m == 0.0:
            return float("inf")
        return float(self.hip_advance_m / self.relative_stride_m)

    @property
    def relative_stride_is_proportional_to_roll(self) -> bool:
        """True when a partial roll keeps the same relative-stride fraction.

        The question a "just roll less" proposal has to answer.  When this is
        True the relative stride scales with the roll and shortening the stroke
        shrinks the margin instead of growing it.
        """

        if not self.partial_roll_fraction:
            return False
        fractions = np.asarray(self.partial_roll_fraction, dtype=float)
        strides = np.asarray(self.partial_relative_stride_m, dtype=float)
        if self.relative_stride_m == 0.0:
            return False
        return bool(np.max(np.abs(strides / self.relative_stride_m - fractions))
                    < 0.10)

    def as_dict(self) -> dict:
        return {
            "hip_advance_mm": self.hip_advance_m * 1e3,
            "contact_advance_mm": self.contact_advance_m * 1e3,
            "relative_stride_mm": self.relative_stride_m * 1e3,
            "stride_loss_ratio": self.stride_loss_ratio,
            "relative_stride_is_proportional_to_roll":
                self.relative_stride_is_proportional_to_roll,
        }


def rolling_stride_2d(posture: NominalPosture2D | None = None,
                      *, fractions: int = 5) -> RollingStride2D:
    """Measure the rolling stroke's hip advance against its contact advance.

    Defaults to :func:`hybrid_posture_2d` -- **the posture the gait actually
    rolls in**.  It defaulted to the uncompensated ``NominalPosture2D`` when
    this module was written, which quoted a stride the chosen gait does not
    have: 297.065 mm of hip advance and a 3.140x loss, against the levelled
    posture's 325.916 mm and 2.640x.  The margins in the scan were always
    measured with the levelled posture, so only this diagnostic disagreed with
    the gait it was describing.
    """

    stroke = nominal_stroke_2d(hybrid_posture_2d() if posture is None
                               else posture)
    hip = np.array([f.hip_xz_m[0] for f in stroke.frames], dtype=float)
    contact = np.array([f.contact_xz_m[0] for f in stroke.frames], dtype=float)
    hip = hip - hip[0]
    contact = contact - contact[0]

    partial_f: list[float] = []
    partial_s: list[float] = []
    for k in range(1, int(fractions) + 1):
        fraction = k / float(fractions)
        index = int(round(fraction * (len(hip) - 1)))
        partial_f.append(float(fraction))
        partial_s.append(float(hip[index] - contact[index]))

    return RollingStride2D(
        hip_advance_m=float(hip[-1]),
        contact_advance_m=float(contact[-1]),
        relative_stride_m=float(hip[-1] - contact[-1]),
        partial_relative_stride_m=tuple(partial_s),
        partial_roll_fraction=tuple(partial_f),
    )


# --------------------------------------------------------------------------
# What the motors are actually asked for
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class FrameRateDemand2D:
    """The worst step between two consecutive generator frames, in one segment.

    This is the rate a controller playing the plan back would have to produce:
    the frames are the poses that were planned, and the schedule says how long
    the segment has, so the interval between two of them is a real time and the
    difference across it is a real motion.

    It is deliberately **not** a finite difference of the resampled trajectory.
    That signal is a staircase -- ``leg_sample_at`` snaps to the nearest frame
    -- and differencing a staircase reports ``step / sample interval``, which
    says more about the grid than about the gait.
    """

    leg: LegId
    segment_index: int
    segment_kind: str
    frame_count: int
    frame_interval_s: float
    theta_step_rad: float
    beta_step_rad: float
    peak_motor_rate_rad_s: float

    @property
    def motor_utilisation(self) -> float:
        return float(self.peak_motor_rate_rad_s / MOTOR_MAX_RATE_RAD_S)

    def as_dict(self) -> dict:
        return {
            "leg": self.leg.value,
            "segment_index": self.segment_index,
            "segment_kind": self.segment_kind,
            "frame_count": self.frame_count,
            "frame_interval_ms": self.frame_interval_s * 1e3,
            "theta_step_deg": float(np.rad2deg(self.theta_step_rad)),
            "beta_step_deg": float(np.rad2deg(self.beta_step_rad)),
            "peak_motor_rate_deg_s": float(
                np.rad2deg(self.peak_motor_rate_rad_s)),
            "motor_utilisation": self.motor_utilisation,
        }


def frame_motor_rate_2d(plan: FourLegPlan2D) -> tuple[FrameRateDemand2D, ...]:
    """The worst frame-to-frame motor demand of every scheduled segment.

    Segments whose frames were never registered, or that carry fewer than two,
    are skipped rather than reported as zero: no frames is not a slow segment,
    and a zero row would drag a maximum down without saying so.
    """

    out: list[FrameRateDemand2D] = []
    for leg, leg_plan in plan.plans.items():
        for scheduled in plan.schedule.segments_of(leg):
            phased = leg_plan.phased[scheduled.segment_index]
            frames = leg_plan.frames.get(phased.segment.frames.source_id)
            indices = phased.segment.frames.indices
            if not frames or len(indices) < 2:
                continue
            interval = ((scheduled.end_s - scheduled.start_s)
                        / (len(indices) - 1))
            if interval <= 0.0:
                continue
            theta = np.array([float(frames[i].theta_rad) for i in indices])
            beta = np.array([float(frames[i].beta_rad) for i in indices])
            d_theta, d_beta = np.diff(theta), np.diff(beta)
            rates = np.array([
                max(abs(r) for r in motor_rates_rad_s(a / interval,
                                                      b / interval))
                for a, b in zip(d_theta, d_beta)])
            worst = int(np.argmax(rates))
            out.append(FrameRateDemand2D(
                leg=leg, segment_index=scheduled.segment_index,
                segment_kind=scheduled.segment_kind.value,
                frame_count=len(indices), frame_interval_s=float(interval),
                theta_step_rad=float(d_theta[worst]),
                beta_step_rad=float(d_beta[worst]),
                peak_motor_rate_rad_s=float(rates[worst]),
            ))
    return tuple(out)


def resampled_motor_rate_rad_s(plan, trajectory, stability,
                               *, samples: int) -> float:
    """The finite difference of the resampled trajectory -- the wrong number.

    Kept so the defect can be demonstrated rather than asserted: call it at two
    sample counts and watch the answer move.
    """

    whole = assemble_whole_body_2d(plan, trajectory, stability,
                                   samples=int(samples),
                                   use_generator_frames=True)
    rates = joint_rates_2d(whole)
    return max((r.peak_motor_rate_rad_s for r in rates), default=0.0)


# --------------------------------------------------------------------------
# The scan
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class MarginScanPoint2D:
    """One gait, its margin, and what that margin costs the motors."""

    sequence_name: str
    stance_duty: float
    cycle_period_s: float
    swing_window_s: float
    min_margin_m: float | None
    unstable_swings: int
    #: Measured frame to frame (:func:`frame_motor_rate_2d`), so it does not
    #: depend on any sampling grid.  ``None`` only when no segment carried
    #: frames to measure.
    peak_motor_rate_rad_s: float | None = None
    #: The same demand read off the resampled trajectory, when it was asked
    #: for.  Carried purely so the two can be printed side by side: it is a
    #: function of ``resampled_at_samples`` and is not a physical rate.
    resampled_motor_rate_rad_s: float | None = None
    resampled_at_samples: int | None = None
    #: The segment the peak is in.  It has been ``RECOVERY_SWING`` at every
    #: duty scanned so far, which is why the duty is the lever that moves it.
    peak_segment_kind: str | None = None

    @property
    def motor_utilisation(self) -> float | None:
        if self.peak_motor_rate_rad_s is None:
            return None
        return float(self.peak_motor_rate_rad_s / MOTOR_MAX_RATE_RAD_S)

    @property
    def min_cycle_period_s(self) -> float | None:
        """The fastest this gait can be run without exceeding 330 rpm.

        Every rate here is a frame step over a frame interval, and the interval
        scales with the cycle period -- so the period is a real lever on the
        motor budget, and a duty that is over budget at one period is inside it
        at a slower one.  This is what the trade actually costs.
        """

        utilisation = self.motor_utilisation
        if utilisation is None:
            return None
        return float(self.cycle_period_s * max(utilisation, 1.0))

    @property
    def motor_is_within_budget(self) -> bool | None:
        utilisation = self.motor_utilisation
        return None if utilisation is None else bool(utilisation <= 1.0)

    def clears(self, margin_floor_m: float) -> bool:
        """Margin over the floor **and** motors inside their budget.

        An unknown motor rate is not a pass: a duty is only usable when both
        halves of the trade have been measured.
        """

        if self.min_margin_m is None or self.min_margin_m <= margin_floor_m:
            return False
        return self.motor_is_within_budget is True

    def as_dict(self) -> dict:
        utilisation = self.motor_utilisation
        return {
            "sequence_name": self.sequence_name,
            "stance_duty": self.stance_duty,
            "cycle_period_s": self.cycle_period_s,
            "swing_window_s": self.swing_window_s,
            "min_margin_mm": (None if self.min_margin_m is None
                              else self.min_margin_m * 1e3),
            "unstable_swings": self.unstable_swings,
            "peak_motor_rate_deg_s": (
                None if self.peak_motor_rate_rad_s is None
                else float(np.rad2deg(self.peak_motor_rate_rad_s))),
            "motor_limit_deg_s": float(np.rad2deg(MOTOR_MAX_RATE_RAD_S)),
            "motor_utilisation": utilisation,
            "motor_is_within_budget": self.motor_is_within_budget,
            "min_cycle_period_s": self.min_cycle_period_s,
            "peak_segment_kind": self.peak_segment_kind,
            "resampled_motor_rate_deg_s": (
                None if self.resampled_motor_rate_rad_s is None
                else float(np.rad2deg(self.resampled_motor_rate_rad_s))),
            "resampled_at_samples": self.resampled_at_samples,
        }


def scan_point_2d(
    plans: dict[LegId, LegPlan2D],
    *,
    sequence_name: str,
    stance_duty: float,
    nominal_body_z_m: float,
    cycle_period_s: float = 2.4,
    margin_floor_m: float = DEFAULT_MARGIN_FLOOR_M,
    resample_at: int | None = None,
    samples: int = 241,
) -> MarginScanPoint2D:
    """Plan, evaluate and price one gait.

    The motor price is always measured, because it is cheap once the plans
    exist and because a margin without its cost is half a trade.  ``resample_at``
    additionally computes the sampling-dependent number, for showing what that
    measurement does.
    """

    order = LIFTOFF_SEQUENCES[sequence_name]
    timing = GaitTiming2D(
        cycle_period_s=float(cycle_period_s),
        stance_duty=float(stance_duty),
        phase_offsets=phase_offsets_for_2d(order, stance_duty),
        gait_name=f"{sequence_name}@duty{stance_duty:g}",
    )
    four = plan_four_legs_2d(plans, timing)
    body = body_trajectory_2d(four, nominal_body_z_m=float(nominal_body_z_m),
                              samples=samples)
    stability = swing_stability_2d(four, body, margin_floor_m=margin_floor_m)

    demands = frame_motor_rate_2d(four)
    worst = (max(demands, key=lambda d: d.peak_motor_rate_rad_s)
             if demands else None)

    resampled: float | None = None
    if resample_at is not None:
        resampled = resampled_motor_rate_rad_s(four, body, stability,
                                               samples=resample_at)

    return MarginScanPoint2D(
        sequence_name=sequence_name,
        stance_duty=float(stance_duty),
        cycle_period_s=float(cycle_period_s),
        swing_window_s=timing.swing_duration_s,
        min_margin_m=stability.minimum_margin_m,
        unstable_swings=len(stability.unstable_swings),
        peak_motor_rate_rad_s=(None if worst is None
                               else worst.peak_motor_rate_rad_s),
        resampled_motor_rate_rad_s=resampled,
        resampled_at_samples=resample_at,
        peak_segment_kind=None if worst is None else worst.segment_kind,
    )


def scan_support_margin_2d(
    plans: dict[LegId, LegPlan2D],
    *,
    nominal_body_z_m: float,
    duties: Iterable[float] = (0.75, 0.775, 0.80, 0.825, 0.85, 0.875, 0.90),
    sequences: Iterable[str] = ("project_walk",),
    cycle_period_s: float = 2.4,
    margin_floor_m: float = DEFAULT_MARGIN_FLOOR_M,
    resample_at: int | None = None,
    samples: int = 241,
) -> tuple[MarginScanPoint2D, ...]:
    """The grid."""

    return tuple(
        scan_point_2d(plans, sequence_name=name, stance_duty=duty,
                      nominal_body_z_m=nominal_body_z_m,
                      cycle_period_s=cycle_period_s,
                      margin_floor_m=margin_floor_m,
                      resample_at=resample_at, samples=samples)
        for name in sequences
        for duty in duties
    )


def best_within_motor_budget_2d(
    points: Sequence[MarginScanPoint2D],
) -> MarginScanPoint2D | None:
    """The largest margin the motors can actually pay for.

    Reported separately from ``clears(floor)`` on purpose: this is what the
    machine *can* do, and whether that is enough is the floor's question, not
    the geometry's.
    """

    usable = [p for p in points
              if p.min_margin_m is not None and p.motor_is_within_budget is True]
    if not usable:
        return None
    return max(usable, key=lambda p: p.min_margin_m)


# --------------------------------------------------------------------------
# The body-offset probe
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class BodyOffsetProbe2D:
    """How much margin a fore-aft body shift could buy, and what it would cost.

    Shifting the body forward while the contacts stay planted is the same thing
    as moving the evaluated centre backward through the polygon, so this is a
    one-dimensional search at every sampled instant.

    The answer is large and **unusable as stated**: the required shift reverses
    sign between a front-leg swing and a hind-leg swing, so no constant offset
    -- no re-mounting of the battery, no CoM correction -- captures any of it.
    It would have to be a surge the body performs twice per cycle, and in a
    rolling stance the hip's fore-aft position is a consequence of ``beta``,
    not a free variable.  Recorded because "just move the CoM" is the first
    thing anyone proposes, and this is the measurement that answers it.
    """

    achievable_margin_m: float
    nominal_margin_m: float
    required_offset_min_m: float
    required_offset_max_m: float
    #: True when the required offset changes sign across the cycle.
    offset_reverses_sign: bool

    def as_dict(self) -> dict:
        return {
            "achievable_margin_mm": self.achievable_margin_m * 1e3,
            "nominal_margin_mm": self.nominal_margin_m * 1e3,
            "required_offset_min_mm": self.required_offset_min_m * 1e3,
            "required_offset_max_mm": self.required_offset_max_m * 1e3,
            "offset_reverses_sign": self.offset_reverses_sign,
        }


def best_body_offset_2d(
    plans: dict[LegId, LegPlan2D],
    timing: GaitTiming2D,
    *,
    nominal_body_z_m: float,
    search_m: float = 0.200,
    search_steps: int = 401,
    samples: int = 241,
) -> BodyOffsetProbe2D:
    """Search, at every sampled instant, the fore-aft shift that maximises the
    margin -- then report whether one offset could serve the whole cycle."""

    four = plan_four_legs_2d(plans, timing)
    body = body_trajectory_2d(four, nominal_body_z_m=float(nominal_body_z_m),
                              samples=samples)
    stability = swing_stability_2d(four, body)
    deltas = np.linspace(-float(search_m), float(search_m), int(search_steps))

    achievable = float("inf")
    nominal = float("inf")
    required: list[float] = []
    for swing in stability.swings:
        for sample in swing.samples:
            body_x = float(np.interp(sample.time_s, body.time_s, body.body_x_m))
            triangle = support_triangle_at(four, body_x, sample.time_s,
                                           swing_leg=swing.swing_leg)
            margins = [triangle.signed_margin_m((body_x + d, 0.0))
                       for d in deltas]
            usable = [(-np.inf if m is None else m) for m in margins]
            best = int(np.argmax(usable))
            achievable = min(achievable, float(usable[best]))
            required.append(float(deltas[best]))
            if sample.margin_m is not None:
                nominal = min(nominal, float(sample.margin_m))

    lo, hi = min(required), max(required)
    return BodyOffsetProbe2D(
        achievable_margin_m=achievable,
        nominal_margin_m=nominal,
        required_offset_min_m=lo,
        required_offset_max_m=hi,
        offset_reverses_sign=bool(lo < 0.0 < hi),
    )


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------


def scan_rows(
    points: Sequence[MarginScanPoint2D],
    stride: RollingStride2D | None = None,
    probe: BodyOffsetProbe2D | None = None,
    demands: Sequence[FrameRateDemand2D] = (),
) -> list[dict]:
    rows: list[dict] = []
    if stride is not None:
        rows.append({"row_kind": "rolling_stride", **stride.as_dict()})
    if probe is not None:
        rows.append({"row_kind": "body_offset_probe", **probe.as_dict()})
    for point in points:
        rows.append({"row_kind": "scan_point", **point.as_dict()})
    for demand in demands:
        rows.append({"row_kind": "frame_rate_demand", **demand.as_dict()})

    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]
