"""Day 12 Step 9: validate the whole-body trajectory, not the primitives.

Plan §16.  Every earlier step validated its own piece; this one asks whether the
assembled four-leg trajectory holds together, and returns **structured failures**
-- time, leg, segment index and kind, the value and the limit it broke -- rather
than a boolean.

**It repairs nothing.**  Plan §16 is explicit about that, and the way to
guarantee it is to have no writes here at all: every function takes a finished
trajectory and returns records.

**A check that is not run is not a pass.**  Some of plan §16's list was already
validated upstream, per frame, by the generator that produced the motion --
re-deriving it here would mean re-running planners, which Step 8 deliberately
does not do.  Those appear in :data:`DELEGATED_CHECKS` with the step that owns
them, so a reader can tell "checked elsewhere" from "checked and passed" from
"not checked".
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from legwheel.config import RobotParams

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    BodyRequirementKind,
    SegmentKind,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import BodyTrajectory2D
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER, LegId
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    TraversalStability2D,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import LegMode
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import FourLegPlan2D
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    WholeBodyTrajectory2D,
)

#: Motor travel, from the project's own constants.
THETA_MIN_RAD: float = float(np.deg2rad(RobotParams.MIN_THETA_DEG))
THETA_MAX_RAD: float = float(np.deg2rad(RobotParams.MAX_THETA_DEG))

#: ``RobotParams.BETA_MAX_DEG`` is labelled "sagittal swing geometric limit" and
#: is used by ``gait_generator_3d`` and ``obstacle_walk``, which treat beta as a
#: bounded swing.  The Hybrid nominal cycle treats it as a **revolution
#: counter** -- Step 1 builds the recovery with ``beta_target = start.beta -
#: 2*pi``.
BETA_WORKSPACE_GUARD_RAD: float = float(np.deg2rad(RobotParams.BETA_MAX_DEG))

#: **Hardware fact, supplied by the project owner (2026-09-01): the leg rotation
#: joint can rotate continuously.**  Recorded here rather than inferred: nothing
#: in this repository states it, and ``BETA_MAX_DEG`` says the opposite for the
#: planners that use beta as a bounded swing.
#:
#: With this, exceeding ``BETA_MAX_DEG`` is **not a violation** for the Hybrid
#: gait -- the two planners simply mean different things by ``beta``.  The check
#: still measures how far outside the legacy guard the motion goes, because a
#: consumer that shares code with those planners needs to know, but it no longer
#: reports a failure.  Set this to ``False`` and the failures come back.
BETA_IS_CONTINUOUS: bool = True

#: One motor's maximum speed, as given for this robot: 330 rpm.
#: **Taken as the joint-side speed.**  If 330 rpm is the motor's no-load figure
#: and there is a reduction or a load derating, the real budget is smaller and
#: this constant is the thing to correct -- it is one number in one place for
#: exactly that reason.
MOTOR_MAX_RPM: float = 330.0
MOTOR_MAX_RATE_RAD_S: float = float(MOTOR_MAX_RPM * 2.0 * np.pi / 60.0)

#: The project's own motor transform (``legwheel/utils/utils.py``)::
#:
#:     phi_r =  theta + beta - theta_0
#:     phi_l = -theta + beta + theta_0
#:
#: The offsets are constants, so the rates are ``phi_r' = theta' + beta'`` and
#: ``phi_l' = beta' - theta'``.  Both motors must stay inside the limit, which
#: makes the feasible set ``|theta'| + |beta'| <= MOTOR_MAX_RATE_RAD_S`` -- a
#: budget the two joints **share**, not one limit each.
def motor_rates_rad_s(theta_rate_rad_s: float,
                      beta_rate_rad_s: float) -> tuple[float, float]:
    """``(phi_r_dot, phi_l_dot)`` from the joint rates."""

    return (float(theta_rate_rad_s + beta_rate_rad_s),
            float(beta_rate_rad_s - theta_rate_rad_s))


#: A joint jump larger than this **across a segment boundary** is a
#: discontinuity rather than motion.  A boundary jump is a property of the two
#: segments, not of any grid, so a fixed angle is the right shape here.
MAX_JOINT_STEP_RAD: float = float(np.deg2rad(30.0))

#: What counts as a teleport *within* a segment, as a **rate**.
#:
#: This used to be ``MAX_JOINT_STEP_RAD`` applied to the per-sample step, and
#: that made the verdict a property of the sample count: the same flat-ground
#: trajectory failed at 61 and 121 samples, passed at 181 and 241, and its
#: implied rate was 1425.69 deg/s at *every* one of them.  A per-step angle
#: cannot be a limit on a signal that is resampled -- halve the grid spacing
#: and the step halves with it.
#:
#: Twice the motor limit, because anything a motor could actually be asked for
#: is already ``MOTOR_RATE_LIMIT``'s business; this is the separate question of
#: whether the trajectory contains a discontinuity that no rate explains.
TELEPORT_RATE_RAD_S: float = 2.0 * MOTOR_MAX_RATE_RAD_S

#: Likewise for the body between neighbouring samples.
MAX_BODY_STEP_M: float = 0.050


class CheckId(str, Enum):
    """One row of plan §16's list."""

    TIME_STRICTLY_INCREASING = "time_strictly_increasing"
    AT_MOST_ONE_AIRBORNE = "at_most_one_airborne"
    THREE_SUPPORT_LEGS = "three_support_legs"
    THETA_WITHIN_LIMITS = "theta_within_limits"
    BETA_WORKSPACE_GUARD = "beta_workspace_guard"
    JOINT_CONTINUITY = "joint_continuity"
    BODY_CONTINUITY = "body_continuity"
    BODY_REQUIREMENT_SATISFIED = "body_requirement_satisfied"
    STANCE_CONTACT_VALID = "stance_contact_valid"
    SEGMENT_CHAINING = "segment_chaining"
    SUPPORT_MARGIN = "support_margin"
    MOTOR_RATE_LIMIT = "motor_rate_limit"


class CheckSeverity(str, Enum):
    """Day 14: what a failed check is a fact about.

    ``HARD``
        the hardware or the geometry -- a joint outside its travel, a motor
        past its rate, a leg through the terrain, two legs airborne, a
        teleport between segments.  A trajectory failing one of these is not
        a command the robot can execute.
    ``ADVISORY``
        a statement of this 2D quasi-static model that the robot need not
        share.  ``support_margin`` rests on a **guessed** +/-2 mm centre-of-mass
        uncertainty and a model with no mass, contact force or dynamics; it
        cannot say the machine tips, only that it is outside a planning
        floor.  ``body_requirement_satisfied`` compares the legs' *demands*
        on the body, which the gait-first planner replaces by the plane the
        four hips actually lie on.  Both are computed and reported; neither
        blocks a trajectory (project owner, 2026-09-07).
    """

    HARD = "HARD"
    ADVISORY = "ADVISORY"


CHECK_SEVERITY: dict[CheckId, CheckSeverity] = {
    CheckId.TIME_STRICTLY_INCREASING: CheckSeverity.HARD,
    CheckId.AT_MOST_ONE_AIRBORNE: CheckSeverity.HARD,
    CheckId.THREE_SUPPORT_LEGS: CheckSeverity.HARD,
    CheckId.THETA_WITHIN_LIMITS: CheckSeverity.HARD,
    CheckId.BETA_WORKSPACE_GUARD: CheckSeverity.HARD,
    CheckId.JOINT_CONTINUITY: CheckSeverity.HARD,
    CheckId.BODY_CONTINUITY: CheckSeverity.HARD,
    CheckId.BODY_REQUIREMENT_SATISFIED: CheckSeverity.ADVISORY,
    CheckId.STANCE_CONTACT_VALID: CheckSeverity.HARD,
    CheckId.SEGMENT_CHAINING: CheckSeverity.HARD,
    CheckId.SUPPORT_MARGIN: CheckSeverity.ADVISORY,
    CheckId.MOTOR_RATE_LIMIT: CheckSeverity.HARD,
}


#: How far outside the legacy guard the motion goes, and why that is reported
#: rather than failed.  Kept as a note so the number does not vanish along with
#: the failure.
BETA_GUARD_NOTE = (
    "the Hybrid recovery sweeps beta a full turn, far outside "
    f"RobotParams.BETA_MAX_DEG = {RobotParams.BETA_MAX_DEG} deg.  That guard "
    "belongs to the planners which treat beta as a bounded sagittal swing; the "
    "leg rotation joint is continuous (project owner, 2026-09-01), so the two "
    "are describing different quantities.  Reported, not failed."
)


#: Checks plan §16 lists that a **generator already ran, per frame**, and that
#: Step 9 does not repeat.  Listed, not omitted: a missing check must not read
#: as a passing one.
DELEGATED_CHECKS: dict[str, str] = {
    "ik_residual": (
        "Day 8-9's swing planner solves and reports the IK per frame; Step 8 "
        "interpolates between validated endpoints and runs no IK of its own."
    ),
    "swing_collision_free": (
        "generate_swing_2d collision-checks the whole swing trajectory "
        "(Step 7 requirement 7); re-deriving it needs the planner Step 8 "
        "deliberately does not call."
    ),
    "terrain_penetration_per_frame": (
        "Day 6-7 and Day 8-9 checked terrain clearance frame by frame when the "
        "segments were generated; the assembled trajectory carries endpoints, "
        "not those frames."
    ),
    "expected_touchdown_surface": (
        "Step 7's RepositionTarget2D checks the touchdown rim and theta where a "
        "target is specified; no other segment here specifies one."
    ),
}


#: Checks plan §16 lists that **cannot be decided here**, with what is missing.
#: Distinct from :data:`DELEGATED_CHECKS`: those were run elsewhere, these were
#: run nowhere.  Both are listed so that neither reads as a pass.
UNEVALUABLE_CHECKS: dict[str, str] = {
    "theta_rate_interior": (
        "Step 8 interpolates linearly between segment endpoints, and a "
        "RECOVERY_SWING starts and ends at the same theta -- so its interior "
        "retraction toward the compact posture is invisible here and the "
        "measured peak theta rate comes out at 0.  That is a property of the "
        "assembly, not of the motion: the real frames exist behind each "
        "segment's FrameRef2D, and reading them needs the generator Step 8 "
        "does not call.  The beta rates are unaffected, because beta advances "
        "monotonically between the endpoints."
    ),
}


@dataclass(frozen=True)
class JointRateMeasurement2D:
    """Peak joint rates, split by mode.  Plan §16's missing half.

    Reported rather than judged: see :data:`UNEVALUABLE_CHECKS`.  The rates are
    a consequence of the duty Step 3 had to pick, so they are a property of the
    schedule as much as of the motion.

    **The theta rate is a lower bound, not the demand.**  Step 8 interpolates
    between segment endpoints and a recovery's two endpoints share a theta, so
    the retraction in between does not appear.  ``theta_rate_is_lower_bound``
    says so on the record rather than leaving a 0 to be misread.
    """

    leg: LegId
    peak_theta_rate_rad_s: float
    peak_beta_rate_rad_s: float
    peak_stance_beta_rate_rad_s: float
    peak_airborne_beta_rate_rad_s: float
    #: The worst motor rate **actually asked for**, measured step by step.
    #: Not ``motor_rates_rad_s(peak_theta, peak_beta)``: that combines two peaks
    #: that need not occur at the same instant and overstated this trajectory by
    #: 1429 deg/s against a true 951.
    peak_motor_rate_rad_s: float = 0.0
    #: True while theta comes from endpoint interpolation; see
    #: ``UNEVALUABLE_CHECKS["theta_rate_interior"]``.
    theta_rate_is_lower_bound: bool = True

    @property
    def peak_motor_rate_upper_bound_rad_s(self) -> float:
        """The two peaks combined -- a bound, kept because it is the number a
        conservative sizing calculation wants."""

        return max(abs(r) for r in motor_rates_rad_s(
            self.peak_theta_rate_rad_s, self.peak_beta_rate_rad_s))

    @property
    def motor_utilisation(self) -> float:
        """Fraction of ``MOTOR_MAX_RATE_RAD_S`` the peak asks for."""

        return float(self.peak_motor_rate_rad_s / MOTOR_MAX_RATE_RAD_S)

    @property
    def airborne_to_stance_ratio(self) -> float | None:
        if self.peak_stance_beta_rate_rad_s <= 0.0:
            return None
        return float(self.peak_airborne_beta_rate_rad_s
                     / self.peak_stance_beta_rate_rad_s)

    def as_dict(self) -> dict:
        ratio = self.airborne_to_stance_ratio
        return {
            "leg": self.leg.value,
            "peak_theta_rate_deg_s": float(
                np.rad2deg(self.peak_theta_rate_rad_s)),
            "peak_beta_rate_deg_s": float(np.rad2deg(self.peak_beta_rate_rad_s)),
            "peak_stance_beta_rate_deg_s": float(
                np.rad2deg(self.peak_stance_beta_rate_rad_s)),
            "peak_airborne_beta_rate_deg_s": float(
                np.rad2deg(self.peak_airborne_beta_rate_rad_s)),
            "airborne_to_stance_ratio": ratio,
            "theta_rate_is_lower_bound": self.theta_rate_is_lower_bound,
            "peak_motor_rate_deg_s": float(
                np.rad2deg(self.peak_motor_rate_rad_s)),
            "peak_motor_rate_upper_bound_deg_s": float(
                np.rad2deg(self.peak_motor_rate_upper_bound_rad_s)),
            "motor_limit_deg_s": float(np.rad2deg(MOTOR_MAX_RATE_RAD_S)),
            "motor_utilisation": self.motor_utilisation,
            "theta_rate_headroom_deg_s": float(np.rad2deg(
                MOTOR_MAX_RATE_RAD_S - abs(self.peak_beta_rate_rad_s))),
        }


def joint_rates_2d(result: WholeBodyTrajectory2D) -> tuple[JointRateMeasurement2D, ...]:
    """Peak joint rates per leg, measured on the assembled trajectory.

    Only within-segment steps are used: a segment boundary carries a whole
    revolution (Step 8), and dividing that by a sample interval would report a
    rate no joint is being asked for.
    """

    out: list[JointRateMeasurement2D] = []
    for leg in LEG_ORDER:
        theta_rates: list[float] = []
        beta_rates: list[float] = []
        stance_rates: list[float] = []
        airborne_rates: list[float] = []
        motor_peak = 0.0
        for previous, following in zip(result.samples, result.samples[1:]):
            before, after = previous.legs.get(leg), following.legs.get(leg)
            if before is None or after is None:
                continue
            if before.segment_index != after.segment_index:
                continue
            dt = following.time_s - previous.time_s
            if dt <= 0.0:
                continue
            theta_rate = abs(after.theta_rad - before.theta_rad) / dt
            beta_rate = abs(after.beta_rad - before.beta_rad) / dt
            theta_rates.append(theta_rate)
            beta_rates.append(beta_rate)
            (airborne_rates if after.mode is LegMode.AIRBORNE
             else stance_rates).append(beta_rate)
            signed_theta = (after.theta_rad - before.theta_rad) / dt
            signed_beta = (after.beta_rad - before.beta_rad) / dt
            motor_peak = max(
                motor_peak,
                *(abs(r) for r in motor_rates_rad_s(signed_theta, signed_beta)),
            )
        out.append(JointRateMeasurement2D(
            leg=leg,
            peak_theta_rate_rad_s=max(theta_rates, default=0.0),
            peak_beta_rate_rad_s=max(beta_rates, default=0.0),
            peak_stance_beta_rate_rad_s=max(stance_rates, default=0.0),
            peak_airborne_beta_rate_rad_s=max(airborne_rates, default=0.0),
            peak_motor_rate_rad_s=float(motor_peak),
            theta_rate_is_lower_bound=not all(
                leg_sample.from_generator_frame
                for sample in result.samples
                for leg_sample in (sample.legs.get(leg),) if leg_sample
            ),
        ))
    return tuple(out)


@dataclass(frozen=True)
class ValidationFailure2D:
    """One structured failure.  Plan §16 asks for exactly these fields."""

    check: CheckId
    detail: str
    time_s: float | None = None
    leg: LegId | None = None
    segment_index: int | None = None
    segment_kind: SegmentKind | None = None
    value: float | None = None
    limit: float | None = None

    def as_dict(self) -> dict:
        return {
            "check": self.check.value,
            "time_s": self.time_s,
            "leg": None if self.leg is None else self.leg.value,
            "segment_index": self.segment_index,
            "segment_kind": (None if self.segment_kind is None
                             else self.segment_kind.value),
            "value": self.value,
            "limit": self.limit,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class ValidationReport2D:
    """What held, what did not, and what was checked somewhere else."""

    failures: tuple[ValidationFailure2D, ...]
    checks_run: tuple[CheckId, ...]
    delegated: dict[str, str]
    #: Listed, not silently absent: these could not be decided at all.
    unevaluable: dict[str, str] = None
    #: Measured, but not a failure -- see :data:`BETA_GUARD_NOTE`.
    beta_outside_legacy_guard: int = 0
    #: Measured but not judged -- there is no limit to judge them against.
    joint_rates: tuple[JointRateMeasurement2D, ...] = ()
    #: Carried from the trajectory: the unresolved results it was built on.
    assumptions: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.unevaluable is None:
            object.__setattr__(self, "unevaluable", dict(UNEVALUABLE_CHECKS))

    @property
    def is_valid(self) -> bool:
        return not self.failures

    def failed_checks(self) -> tuple[CheckId, ...]:
        seen: list[CheckId] = []
        for failure in self.failures:
            if failure.check not in seen:
                seen.append(failure.check)
        return tuple(seen)

    def passed_checks(self) -> tuple[CheckId, ...]:
        failed = set(self.failed_checks())
        return tuple(c for c in self.checks_run if c not in failed)

    def failures_of(self, check: CheckId) -> tuple[ValidationFailure2D, ...]:
        return tuple(f for f in self.failures if f.check is check)

    # -- Day 14: hard facts apart from model statements ----------------------

    def hard_failed_checks(self) -> tuple[CheckId, ...]:
        return tuple(c for c in self.failed_checks()
                     if CHECK_SEVERITY[c] is CheckSeverity.HARD)

    def advisory_failed_checks(self) -> tuple[CheckId, ...]:
        return tuple(c for c in self.failed_checks()
                     if CHECK_SEVERITY[c] is CheckSeverity.ADVISORY)

    @property
    def is_executable(self) -> bool:
        """No HARD failure.  Advisory ones are reported, not enforced."""

        return not self.hard_failed_checks()

    def as_dict(self) -> dict:
        return {
            "checks_run": len(self.checks_run),
            "hard_checks_failed": len(self.hard_failed_checks()),
            "advisory_checks_failed": len(self.advisory_failed_checks()),
            "is_executable": self.is_executable,
            "checks_passed": len(self.passed_checks()),
            "checks_failed": len(self.failed_checks()),
            "failures": len(self.failures),
            "delegated_checks": len(self.delegated),
            "unevaluable_checks": len(self.unevaluable),
            "beta_outside_legacy_guard": self.beta_outside_legacy_guard,
            "beta_guard_note": BETA_GUARD_NOTE,
            "unresolved_assumptions": len(self.assumptions),
            "is_valid": self.is_valid,
        }


# --------------------------------------------------------------------------
# The checks
# --------------------------------------------------------------------------


def _check_timing(result: WholeBodyTrajectory2D) -> list[ValidationFailure2D]:
    out: list[ValidationFailure2D] = []
    for previous, following in zip(result.samples, result.samples[1:]):
        if following.time_s <= previous.time_s:
            out.append(ValidationFailure2D(
                check=CheckId.TIME_STRICTLY_INCREASING,
                time_s=following.time_s, value=following.time_s,
                limit=previous.time_s,
                detail="time did not increase between neighbouring samples",
            ))
    for sample in result.samples:
        airborne = [l for l, s in sample.legs.items()
                    if s.mode is LegMode.AIRBORNE]
        if len(airborne) > 1:
            out.append(ValidationFailure2D(
                check=CheckId.AT_MOST_ONE_AIRBORNE, time_s=sample.time_s,
                value=float(len(airborne)), limit=1.0,
                detail=f"airborne: {', '.join(l.value for l in airborne)}",
            ))
        if sample.swing_leg is not None and len(sample.support_legs) != 3:
            out.append(ValidationFailure2D(
                check=CheckId.THREE_SUPPORT_LEGS, time_s=sample.time_s,
                leg=sample.swing_leg, value=float(len(sample.support_legs)),
                limit=3.0,
                detail=("a swing needs exactly three intended support legs; "
                        f"got {', '.join(l.value for l in sample.support_legs)}"),
            ))
    return out


def _check_kinematics(result: WholeBodyTrajectory2D) -> list[ValidationFailure2D]:
    out: list[ValidationFailure2D] = []
    for sample in result.samples:
        for leg, leg_sample in sample.legs.items():
            if not (THETA_MIN_RAD - 1e-9 <= leg_sample.theta_rad
                    <= THETA_MAX_RAD + 1e-9):
                out.append(ValidationFailure2D(
                    check=CheckId.THETA_WITHIN_LIMITS, time_s=sample.time_s,
                    leg=leg, segment_index=leg_sample.segment_index,
                    segment_kind=leg_sample.segment_kind,
                    value=float(np.rad2deg(leg_sample.theta_rad)),
                    limit=float(RobotParams.MAX_THETA_DEG),
                    detail=(f"theta outside "
                            f"[{RobotParams.MIN_THETA_DEG}, "
                            f"{RobotParams.MAX_THETA_DEG}] deg"),
                ))
            outside = abs(leg_sample.beta_rad) > BETA_WORKSPACE_GUARD_RAD + 1e-9
            if outside and not BETA_IS_CONTINUOUS:
                out.append(ValidationFailure2D(
                    check=CheckId.BETA_WORKSPACE_GUARD, time_s=sample.time_s,
                    leg=leg, segment_index=leg_sample.segment_index,
                    segment_kind=leg_sample.segment_kind,
                    value=float(np.rad2deg(leg_sample.beta_rad)),
                    limit=float(RobotParams.BETA_MAX_DEG),
                    detail=("beta is outside the sagittal-swing guard and the "
                            "joint is recorded as NOT continuous"),
                ))

    for previous, following in zip(result.samples, result.samples[1:]):
        span_s = following.time_s - previous.time_s
        if span_s <= 0.0:
            continue  # time_strictly_increasing owns this
        for leg in LEG_ORDER:
            before, after = previous.legs.get(leg), following.legs.get(leg)
            if before is None or after is None:
                continue
            if before.segment_index != after.segment_index:
                continue  # a boundary; handoffs are checked separately
            step = max(abs(after.theta_rad - before.theta_rad),
                       abs(after.beta_rad - before.beta_rad))
            # As a rate, so the verdict belongs to the trajectory and not to
            # the grid it was sampled on.
            rate = step / span_s
            if rate > TELEPORT_RATE_RAD_S:
                out.append(ValidationFailure2D(
                    check=CheckId.JOINT_CONTINUITY, time_s=following.time_s,
                    leg=leg, segment_index=after.segment_index,
                    segment_kind=after.segment_kind,
                    value=float(np.rad2deg(rate)),
                    limit=float(np.rad2deg(TELEPORT_RATE_RAD_S)),
                    detail=("joint moved faster within a single segment than "
                            "any rate explains"),
                ))
    return out


def _check_body(result: WholeBodyTrajectory2D,
                trajectory: BodyTrajectory2D) -> list[ValidationFailure2D]:
    out: list[ValidationFailure2D] = []
    for previous, following in zip(result.samples, result.samples[1:]):
        before = np.asarray(previous.body_position_world_m, dtype=float)
        after = np.asarray(following.body_position_world_m, dtype=float)
        if not (np.all(np.isfinite(before)) and np.all(np.isfinite(after))):
            continue  # an infeasible height is reported by its own check
        step = float(np.linalg.norm(after - before))
        if step > MAX_BODY_STEP_M:
            out.append(ValidationFailure2D(
                check=CheckId.BODY_CONTINUITY, time_s=following.time_s,
                value=step * 1e3, limit=MAX_BODY_STEP_M * 1e3,
                detail="body jumped between neighbouring samples",
            ))

    if not trajectory.is_feasible:
        worst = max((c.disagreement_m for c in trajectory.conflicts), default=0.0)
        out.append(ValidationFailure2D(
            check=CheckId.BODY_REQUIREMENT_SATISFIED,
            time_s=trajectory.conflicts[0].time_s if trajectory.conflicts else None,
            value=worst * 1e3, limit=0.0,
            detail=(f"{len(trajectory.conflicts)} instants have two hard body "
                    f"requirements that disagree; Step 5 returned INFEASIBLE "
                    f"rather than averaging them"),
        ))
    unusable = sum(1 for s in result.samples
                   if not np.isfinite(s.body_position_world_m[2]))
    if unusable:
        out.append(ValidationFailure2D(
            check=CheckId.BODY_REQUIREMENT_SATISFIED,
            value=float(unusable), limit=0.0,
            detail=(f"{unusable} of {len(result.samples)} samples have no "
                    f"body height at all"),
        ))
    return out


def _check_contact(result: WholeBodyTrajectory2D) -> list[ValidationFailure2D]:
    """Segment mode against frame contact state, in the segment's interior.

    A segment's **boundary** frames are shared with its neighbour: a recovery's
    first frame is the stroke's last, the instant of liftoff, and the foot is
    still down.  Requiring agreement there would fail a correct trajectory for
    a labelling reason, so the boundary frames are excluded -- and only the
    boundary frames, which is why the flag is set where the frame is read
    rather than guessed from the time.
    """

    out: list[ValidationFailure2D] = []
    for sample in result.samples:
        for leg, leg_sample in sample.legs.items():
            if leg_sample.is_segment_boundary_frame:
                continue
            if leg_sample.mode is LegMode.STANCE and not leg_sample.in_contact:
                out.append(ValidationFailure2D(
                    check=CheckId.STANCE_CONTACT_VALID, time_s=sample.time_s,
                    leg=leg, segment_index=leg_sample.segment_index,
                    segment_kind=leg_sample.segment_kind,
                    detail="a stance leg is not marked as in contact",
                ))
            if leg_sample.mode is LegMode.AIRBORNE and leg_sample.in_contact:
                out.append(ValidationFailure2D(
                    check=CheckId.STANCE_CONTACT_VALID, time_s=sample.time_s,
                    leg=leg, segment_index=leg_sample.segment_index,
                    segment_kind=leg_sample.segment_kind,
                    detail="an airborne leg is marked as in contact",
                ))
    return out


def _check_chaining(result: WholeBodyTrajectory2D,
                    max_contact_gap_m: float) -> list[ValidationFailure2D]:
    """``end(k) -> start(k+1)``; plan §16 says a teleport is not allowed.

    **What "teleport" means depends on the boundary.**  Day 12's trap 1 is that
    a boundary has more than one kind and one threshold cannot read them all;
    this is that trap one level deeper.

    While the contact stays on one surface, the contact point is the right
    thing to measure: if it moves, the foot slid or the leg jumped.

    When the contact **transfers to a different surface** -- ground to the
    obstacle top, or back down -- the contact point *must* jump, because a
    different point of the leg is now touching a different thing.  Measured on
    the 40 mm crossing, the two transfers move the contact 92.782 mm and
    108.512 mm while moving the hip 3.238 mm and 1.570 mm.  The leg did not go
    anywhere.  Reading those as teleports is reading the wrong quantity, and it
    is what made ``segment_chaining`` fail on a crossing that is fine.

    So at a transfer the question becomes "did the **leg** jump", and the hip
    is what answers it.  The tolerance is not relaxed -- the same number is
    applied to a different quantity.
    """

    out: list[ValidationFailure2D] = []
    for handoff in result.handoffs:
        if handoff.surface_changed:
            if handoff.body_jump_m > max_contact_gap_m:
                out.append(ValidationFailure2D(
                    check=CheckId.SEGMENT_CHAINING, time_s=handoff.time_s,
                    leg=handoff.leg, segment_index=handoff.to_index,
                    segment_kind=handoff.to_kind,
                    value=handoff.body_jump_m * 1e3,
                    limit=max_contact_gap_m * 1e3,
                    detail=(f"the leg teleports between segment "
                            f"{handoff.from_index} and {handoff.to_index}: the "
                            f"contact transfers to another surface, which moves "
                            f"the contact point legitimately, but the hip moved "
                            f"{handoff.body_jump_m * 1e3:.3f} mm with it"),
                ))
            continue
        if handoff.contact_jump_m > max_contact_gap_m:
            out.append(ValidationFailure2D(
                check=CheckId.SEGMENT_CHAINING, time_s=handoff.time_s,
                leg=handoff.leg, segment_index=handoff.to_index,
                segment_kind=handoff.to_kind,
                value=handoff.contact_jump_m * 1e3,
                limit=max_contact_gap_m * 1e3,
                detail=(f"contact teleports between segment "
                        f"{handoff.from_index} and {handoff.to_index}"),
            ))
        # The revolution is not a discontinuity; the wrapped value is.
        if abs(handoff.joint_jump_wrapped_rad) > MAX_JOINT_STEP_RAD:
            out.append(ValidationFailure2D(
                check=CheckId.SEGMENT_CHAINING, time_s=handoff.time_s,
                leg=handoff.leg, segment_index=handoff.to_index,
                segment_kind=handoff.to_kind,
                value=float(np.rad2deg(handoff.joint_jump_wrapped_rad)),
                limit=float(np.rad2deg(MAX_JOINT_STEP_RAD)),
                detail="joint state does not chain across the boundary",
            ))
        if not handoff.time_is_monotonic:
            out.append(ValidationFailure2D(
                check=CheckId.SEGMENT_CHAINING, time_s=handoff.time_s,
                leg=handoff.leg, segment_index=handoff.to_index,
                segment_kind=handoff.to_kind,
                detail="the next segment starts before the previous one ends",
            ))
    return out


def _check_motor_rates(result: WholeBodyTrajectory2D) -> list[ValidationFailure2D]:
    """Both motors, against the shared speed budget.

    **Necessary, not sufficient.**  Step 8 interpolates between segment
    endpoints, and a recovery's endpoints share a theta, so the measured
    ``theta'`` is a lower bound (``theta_rate_interior``).  A pass here means
    "the part of the motion this assembly can see fits"; the interior of a
    recovery needs the generator's own frames to check.
    """

    out: list[ValidationFailure2D] = []
    for previous, following in zip(result.samples, result.samples[1:]):
        dt = following.time_s - previous.time_s
        if dt <= 0.0:
            continue
        for leg in LEG_ORDER:
            before, after = previous.legs.get(leg), following.legs.get(leg)
            if before is None or after is None:
                continue
            if before.segment_index != after.segment_index:
                continue  # a boundary carries a whole revolution, not a rate
            theta_rate = (after.theta_rad - before.theta_rad) / dt
            beta_rate = (after.beta_rad - before.beta_rad) / dt
            for name, rate in zip(("phi_r", "phi_l"),
                                  motor_rates_rad_s(theta_rate, beta_rate)):
                if abs(rate) > MOTOR_MAX_RATE_RAD_S:
                    out.append(ValidationFailure2D(
                        check=CheckId.MOTOR_RATE_LIMIT, time_s=following.time_s,
                        leg=leg, segment_index=after.segment_index,
                        segment_kind=after.segment_kind,
                        value=float(np.rad2deg(abs(rate))),
                        limit=float(np.rad2deg(MOTOR_MAX_RATE_RAD_S)),
                        detail=f"{name} exceeds {MOTOR_MAX_RPM:.0f} rpm",
                    ))
    return out


def _check_stability(stability: TraversalStability2D) -> list[ValidationFailure2D]:
    out: list[ValidationFailure2D] = []
    for swing in stability.unstable_swings:
        minimum = swing.minimum_margin_m
        out.append(ValidationFailure2D(
            check=CheckId.SUPPORT_MARGIN, time_s=swing.worst_time_s,
            leg=swing.swing_leg, segment_index=swing.segment_index,
            segment_kind=SegmentKind(swing.segment_kind),
            value=None if minimum is None else minimum * 1e3,
            limit=swing.margin_floor_m * 1e3,
            detail=("the CoM projection is not far enough inside the support "
                    "triangle over the whole swing"),
        ))
    return out


def validate_whole_body_2d(
    result: WholeBodyTrajectory2D,
    trajectory: BodyTrajectory2D,
    stability: TraversalStability2D,
    *,
    max_contact_gap_m: float = 0.010,
) -> ValidationReport2D:
    """Run every check Step 9 owns.  Nothing is repaired, ever."""

    failures: list[ValidationFailure2D] = []
    failures += _check_timing(result)
    failures += _check_kinematics(result)
    failures += _check_body(result, trajectory)
    failures += _check_contact(result)
    failures += _check_chaining(result, max_contact_gap_m)
    failures += _check_motor_rates(result)
    failures += _check_stability(stability)

    outside = sum(
        1 for sample in result.samples for s in sample.legs.values()
        if abs(s.beta_rad) > BETA_WORKSPACE_GUARD_RAD + 1e-9
    )
    return ValidationReport2D(
        failures=tuple(failures), checks_run=tuple(CheckId),
        beta_outside_legacy_guard=outside,
        delegated=dict(DELEGATED_CHECKS),
        unevaluable=dict(UNEVALUABLE_CHECKS),
        joint_rates=joint_rates_2d(result),
        assumptions=result.assumptions,
    )


def validation_rows(report: ValidationReport2D) -> list[dict]:
    """Summary, per-check verdicts, failures and delegations, as one table."""

    rows: list[dict] = [{"row_kind": "summary", **report.as_dict()}]
    failed = set(report.failed_checks())
    for check in report.checks_run:
        rows.append({
            "row_kind": "check", "check": check.value,
            "passed": check not in failed,
            "failures": len(report.failures_of(check)),
        })
    rows += [{"row_kind": "failure", **f.as_dict()} for f in report.failures]
    rows += [{"row_kind": "delegated", "check": name, "detail": why}
             for name, why in report.delegated.items()]
    rows += [{"row_kind": "unevaluable", "check": name, "detail": why}
             for name, why in report.unevaluable.items()]
    rows += [{"row_kind": "joint_rate", **r.as_dict()}
             for r in report.joint_rates]
    rows += [{"row_kind": "assumption", "detail": note}
             for note in report.assumptions]

    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]
