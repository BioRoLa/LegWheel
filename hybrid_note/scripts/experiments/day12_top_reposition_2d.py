"""Day 12 Step 7: resolve ``TOP_REPOSITION`` in the four-leg context.

Plan §14.  Day 10--11 left these unresolved on purpose -- a single-leg planner
cannot know whether the other three legs can carry the body -- and Day 12 is
the first place that question can be asked.

**Nothing here generates a trajectory of its own.**  Plan §14 says not to write
a second swing algorithm for repositioning, so the airborne motion comes from
the same chain everything else uses::

    standing_scene_2d  ->  build_swing_request_2d  ->  generate_swing_2d
                       ->  segment_from_swing_plan_2d

``generate_swing_2d`` already runs the terrain collision and touchdown
validation over the whole trajectory, which is requirement 7 satisfied by reuse
rather than by a second implementation.

**A resolved requirement is not a flag.**  ``TransitionRequirement2D`` refuses
``resolved=True`` -- "a resolved transition is a segment, not a requirement" --
so the output here is an *attempt record* carrying both the original
requirement (requirement 10's traceability) and the segment that solves it, if
one was generated.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from hybrid_note.scripts.experiments.cartesian_swing_contract_2d import (
    SwingConstraints2D,
    SwingRequest2D,
    build_swing_request_2d,
)
from hybrid_note.scripts.experiments.cartesian_swing_planner_2d import (
    SwingPlan2D,
    generate_swing_2d,
)
from hybrid_note.scripts.experiments.day10_11_composer_2d import (
    top_reposition_requirement_2d,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    BLOCKED_PAIRS,
    StrategyId,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import (
    MotionSegment2D,
    RimId,
    SegmentKind,
    TransitionKind,
    TransitionRequirement2D,
)
from hybrid_note.scripts.experiments.day10_11_sequence_builders_2d import (
    segment_from_swing_plan_2d,
)
from hybrid_note.scripts.experiments.day10_11_left_rim_landing_2d import (
    LEFT_RIM_READY_THETA_RAD,
    choose_landing_beta_2d,
    left_rim_beta_window_2d,
    left_rim_landing_scene_2d,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
    standing_scene_2d,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import BodyTrajectory2D
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LegId
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    DEFAULT_MARGIN_FLOOR_M,
    SwingStability2D,
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import LegMode
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import FourLegPlan2D

#: Arc resolution for the reposition scenes.  The same number Day 10--11 used
#: when it built the unresolved record, so both describe the same geometry.
SWING_ARC_SAMPLES: int = 721


class RepositionOutcome(str, Enum):
    """Why an attempt ended where it did.  Every value is a real answer."""

    #: The three remaining legs cannot hold the body over the interval.
    SUPPORT_INSUFFICIENT = "support_insufficient"
    #: Support was fine; the swing planner could not produce the motion.
    SWING_FAILED = "swing_failed"
    #: The swing landed, but not in a state the next primitive can start from.
    TOUCHDOWN_UNSUITABLE = "touchdown_unsuitable"
    #: Generated, collision-checked and landed in a usable state.
    RESOLVED = "resolved"


@dataclass(frozen=True)
class RepositionTarget2D:
    """``target_condition`` made checkable.

    Day 10--11 wrote the condition in words, which is right for a record that
    nobody could yet test.  Step 7 has to actually decide, so each blocked pair
    gets the same condition as a predicate over the swing's final sample.
    """

    strategy: StrategyId
    rim: RimId
    theta_min_rad: float | None = None
    theta_max_rad: float | None = None

    def accepts(self, segment: MotionSegment2D) -> tuple[bool, str]:
        """``(ok, reason)`` -- the reason is filled in on both outcomes."""

        end = segment.end_contact
        if end.rim is not self.rim:
            return False, (f"touchdown is on {end.rim.value}, and the next "
                           f"primitive starts from {self.rim.value}")
        theta_deg = float(np.rad2deg(end.theta_rad))
        if self.theta_min_rad is not None and end.theta_rad < self.theta_min_rad:
            return False, (f"touchdown theta {theta_deg:.2f} deg is below the "
                           f"{np.rad2deg(self.theta_min_rad):.2f} deg floor")
        if self.theta_max_rad is not None and end.theta_rad > self.theta_max_rad:
            return False, (f"touchdown theta {theta_deg:.2f} deg is above the "
                           f"{np.rad2deg(self.theta_max_rad):.2f} deg ceiling")
        return True, (f"touchdown on {end.rim.value} at theta "
                      f"{theta_deg:.2f} deg satisfies the condition")


#: Day 10--11's ``BLOCKED_TARGET_CONDITION``, as predicates.  The numbers are
#: quoted from there, not chosen here: ``#2`` needs the foot rim above Step 3
#: F's 35 deg floor, ``#3`` needs LEFT_RIM_READY, which is the left rim at the
#: 17 deg the Day 6--7 retract reaches (Day 12 trap 13: that 17 deg is a
#: configurable recovery parameter, not the wheel-mode angle).
REPOSITION_TARGETS: dict[StrategyId, RepositionTarget2D] = {
    StrategyId.ROLL_SWING: RepositionTarget2D(
        strategy=StrategyId.ROLL_SWING, rim=RimId.FOOT,
        theta_min_rad=float(np.deg2rad(35.0)),
    ),
    StrategyId.SWING_ROLL: RepositionTarget2D(
        strategy=StrategyId.SWING_ROLL, rim=RimId.LEFT,
        theta_min_rad=float(np.deg2rad(15.0)),
        theta_max_rad=float(np.deg2rad(25.0)),
    ),
}

#: Beta step for the LEFT_RIM_READY sweep, in degrees.  Day 10--11's own
#: default is 0.5, and the driver keeps it.  It is a parameter because the
#: sweep builds one full leg per beta and costs about ten minutes at that step:
#: a test that only needs "a left-rim landing exists and the swing cannot reach
#: it" can say so with a coarser grid without weakening the claim, as long as
#: it says which grid it used.
DEFAULT_LANDING_BETA_STEP_DEG: float = 0.5

#: How far above a ``theta_min`` floor the *request* aims.  The floor is a
#: minimum, and the touchdown theta is the IK's output, not the request's
#: input (Day 10-11 trap 16): asking for exactly the floor produced
#: 34.99955 deg against a 35 deg floor and failed by 0.00045 deg.  Aiming a
#: little above it is the honest reading of "at least"; the *check* still uses
#: the floor itself.
TARGET_THETA_HEADROOM_RAD: float = float(np.deg2rad(2.0))


# --------------------------------------------------------------------------
# The support gate
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SupportGate2D:
    """Whether the other three legs can hold the body over the interval."""

    leg: LegId
    start_s: float
    end_s: float
    stability: SwingStability2D | None
    margin_floor_m: float

    @property
    def passed(self) -> bool:
        return self.stability is not None and self.stability.is_stable

    @property
    def minimum_margin_m(self) -> float | None:
        return None if self.stability is None else self.stability.minimum_margin_m

    def reason(self) -> str:
        if self.stability is None:
            return ("no scheduled airborne interval for this leg, so no "
                    "three-leg support to check")
        minimum = self.minimum_margin_m
        if self.passed:
            return (f"the other three legs hold with at least "
                    f"{minimum * 1e3:.3f} mm of margin")
        shown = "unknown" if minimum is None else f"{minimum * 1e3:.3f} mm"
        return (f"minimum support margin over the interval is {shown}, below "
                f"the {self.margin_floor_m * 1e3:.1f} mm floor")

    def as_dict(self) -> dict:
        minimum = self.minimum_margin_m
        return {
            "leg": self.leg.value,
            "start_s": self.start_s,
            "end_s": self.end_s,
            "margin_floor_mm": self.margin_floor_m * 1e3,
            "minimum_margin_mm": None if minimum is None else minimum * 1e3,
            "support_legs": ("" if self.stability is None else
                             ",".join(l.value for l in self.stability.support_legs)),
            "passed": self.passed,
            "reason": self.reason(),
        }


def support_gate_2d(
    plan: FourLegPlan2D,
    trajectory: BodyTrajectory2D,
    leg: LegId,
    *,
    margin_floor_m: float = DEFAULT_MARGIN_FLOOR_M,
) -> SupportGate2D:
    """Run Step 6's check over the interval this leg is already scheduled airborne.

    Plan §14 requirement 2 is "schedule the target leg as airborne while the
    other three remain support contacts" -- and Step 3 already did exactly that,
    so the interval is read off the schedule rather than invented here.  The
    first airborne window is used: repositioning happens once, and picking the
    best of several would be choosing the answer.
    """

    airborne = [s for s in plan.schedule.segments_of(leg)
                if s.mode is LegMode.AIRBORNE]
    if not airborne:
        return SupportGate2D(leg=leg, start_s=float("nan"), end_s=float("nan"),
                             stability=None, margin_floor_m=margin_floor_m)

    result = swing_stability_2d(plan, trajectory, margin_floor_m=margin_floor_m)
    mine = [s for s in result.swings if s.swing_leg is leg]
    if not mine:
        return SupportGate2D(leg=leg, start_s=float("nan"), end_s=float("nan"),
                             stability=None, margin_floor_m=margin_floor_m)
    swing = mine[0]
    return SupportGate2D(leg=leg, start_s=swing.start_s, end_s=swing.end_s,
                         stability=swing, margin_floor_m=margin_floor_m)


# --------------------------------------------------------------------------
# One attempt
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class TopRepositionAttempt2D:
    """What Day 12 could and could not do with one unresolved requirement."""

    strategy: StrategyId
    height_m: float
    top_length_m: float
    leg: LegId
    #: Day 10--11's record, unchanged.  Plan §14 requirement 10.
    requirement: TransitionRequirement2D
    gate: SupportGate2D
    outcome: RepositionOutcome
    reason: str
    swing: SwingPlan2D | None = None
    segment: MotionSegment2D | None = None
    target: RepositionTarget2D | None = None
    #: Marks an attempt run with a floor other than the planning default, so a
    #: relaxed exploration can never be read as the answer.
    relaxed_floor: bool = False

    @property
    def resolved(self) -> bool:
        return self.outcome is RepositionOutcome.RESOLVED

    @property
    def original_evidence(self) -> str:
        """Why Day 10--11 could not do this alone.  Never overwritten."""

        return self.requirement.evidence

    def as_dict(self) -> dict:
        return {
            "strategy": self.strategy.value,
            "obstacle_mm": self.height_m * 1e3,
            "top_length_mm": self.top_length_m * 1e3,
            "leg": self.leg.value,
            "transition_kind": self.requirement.kind.value,
            "requires_external_support": self.requirement.requires_external_support,
            "outcome": self.outcome.value,
            "resolved": self.resolved,
            "reason": self.reason,
            "relaxed_floor": self.relaxed_floor,
            "support_passed": self.gate.passed,
            "support_minimum_margin_mm": self.gate.as_dict()["minimum_margin_mm"],
            "support_reason": self.gate.reason(),
            "swing_valid": None if self.swing is None else self.swing.valid,
            "swing_failure": (None if self.swing is None
                              else self.swing.failure.value),
            "segment_kind": (None if self.segment is None
                             else self.segment.kind.value),
            "touchdown_rim": (None if self.segment is None
                              else self.segment.end_contact.rim.value),
            "touchdown_theta_deg": (
                None if self.segment is None
                else float(np.rad2deg(self.segment.end_contact.theta_rad))),
            "target_condition": self.requirement.target_condition,
            "original_evidence": self.original_evidence,
        }


def _target_scene(spec: SharedTerrainSpec2D, strategy: StrategyId,
                  target: RepositionTarget2D, contact_x_m: float,
                  target_theta_rad: float | None,
                  landing_beta_step_deg: float = DEFAULT_LANDING_BETA_STEP_DEG):
    """The touchdown pose the next primitive needs, built by existing code.

    The two blocked pairs need genuinely different landings, and neither is
    invented here:

    ``#2``  a foot-rim stance above Step 3 F's 35 deg floor -- an ordinary
            standing pose, so ``standing_scene_2d`` builds it.
    ``#3``  ``LEFT_RIM_READY``, which a standing pose **cannot** express: the
            left rim only carries at particular ``beta``.  Day 10--11 already
            has the construction, so this calls it rather than approximating
            the condition with a foot-rim pose that would fail the check for
            the wrong reason.

    Returns ``(scene, note)`` or ``(None, why not)``.
    """

    if target.rim is RimId.LEFT:
        window = left_rim_beta_window_2d(
            spec, theta_rad=LEFT_RIM_READY_THETA_RAD,
            sample_step_deg=float(landing_beta_step_deg))
        choice = choose_landing_beta_2d(window, required_budget_deg=None)
        if choice.beta_deg is None:
            return None, (f"no LEFT_RIM_READY landing exists here "
                      f"(beta step {landing_beta_step_deg} deg): {choice.reason}")
        scene, realised = left_rim_landing_scene_2d(
            spec, contact_x_m=float(contact_x_m),
            beta_rad=float(np.deg2rad(choice.beta_deg)),
            theta_rad=LEFT_RIM_READY_THETA_RAD,
        )
        return scene, (f"LEFT_RIM_READY at beta = {choice.beta_deg:.1f} deg, "
                       f"contact placed at x = {realised * 1e3:.1f} mm")

    if target_theta_rad is None:
        target_theta_rad = (
            target.theta_min_rad + TARGET_THETA_HEADROOM_RAD
            if target.theta_min_rad is not None else float(np.deg2rad(60.0))
        )
    scene = standing_scene_2d(
        spec, float(target_theta_rad), hip_x_m=float(contact_x_m),
        support_height_m=spec.top_z_m,
    )
    return scene, (f"foot-rim stance at theta = "
                   f"{np.rad2deg(target_theta_rad):.2f} deg")


def _reposition_request(
    strategy: StrategyId, height_m: float, top_length_m: float,
    target: RepositionTarget2D,
    *, reposition_distance_m: float, target_theta_rad: float | None,
    swing_duration_s: float,
    landing_beta_step_deg: float = DEFAULT_LANDING_BETA_STEP_DEG,
) -> tuple[SwingRequest2D | None, str]:
    """A start scene and a target scene, handed to the existing builder.

    Both scenes share one terrain, which is what ``build_swing_request_2d``
    requires: the reposition is a relocation on the platform, not a jump
    between two worlds.
    """

    spec = SharedTerrainSpec2D(
        height_m=float(height_m), top_length_m=float(top_length_m),
        x_start_m=0.10, arc_samples=SWING_ARC_SAMPLES,
    )
    start_theta = float(np.deg2rad(60.0))
    start_hip_x = spec.x_start_m + 0.12
    start_scene = standing_scene_2d(spec, start_theta, hip_x_m=start_hip_x,
                                    support_height_m=spec.top_z_m)
    target_scene, note = _target_scene(
        spec, strategy, target,
        contact_x_m=start_hip_x + float(reposition_distance_m),
        target_theta_rad=target_theta_rad,
        landing_beta_step_deg=landing_beta_step_deg,
    )
    if target_scene is None:
        return None, note
    return build_swing_request_2d(
        start_scene, target_scene,
        clearance_m=0.03, swing_duration_s=float(swing_duration_s),
        sample_count=101, constraints=SwingConstraints2D(),
    ), note


def resolve_top_reposition_2d(
    plan: FourLegPlan2D,
    trajectory: BodyTrajectory2D,
    strategy: StrategyId,
    height_m: float,
    top_length_m: float,
    *,
    leg: LegId = LegId.LF,
    margin_floor_m: float = DEFAULT_MARGIN_FLOOR_M,
    reposition_distance_m: float = 0.06,
    target_theta_rad: float | None = None,
    landing_beta_step_deg: float = DEFAULT_LANDING_BETA_STEP_DEG,
    requirement: TransitionRequirement2D | None = None,
) -> TopRepositionAttempt2D:
    """Try to resolve one ``TOP_REPOSITION`` with the four legs in hand.

    The order is plan §14's: support first, motion second.  Generating the
    swing before checking the support would produce a trajectory that looks
    like an answer to a question nobody was allowed to ask.
    """

    if strategy not in BLOCKED_PAIRS:
        raise ValueError(
            f"{strategy.value} is not a blocked pair; there is no "
            "TOP_REPOSITION to resolve here."
        )
    if requirement is None:
        requirement = top_reposition_requirement_2d(strategy, height_m,
                                                    top_length_m)
    if requirement.kind is not TransitionKind.TOP_REPOSITION:
        raise ValueError("this resolver only handles TOP_REPOSITION.")

    target = REPOSITION_TARGETS[strategy]
    relaxed = margin_floor_m != DEFAULT_MARGIN_FLOOR_M

    gate = support_gate_2d(plan, trajectory, leg, margin_floor_m=margin_floor_m)
    if not gate.passed:
        # Requirement 4: stop here.  No ABAD correction, no second try with a
        # kinder floor -- either would turn "cannot" into "not yet asked
        # nicely".
        return TopRepositionAttempt2D(
            strategy=strategy, height_m=height_m, top_length_m=top_length_m,
            leg=leg, requirement=requirement, gate=gate,
            outcome=RepositionOutcome.SUPPORT_INSUFFICIENT,
            reason=gate.reason(), target=target, relaxed_floor=relaxed,
        )

    request, note = _reposition_request(
        strategy, height_m, top_length_m, target,
        reposition_distance_m=reposition_distance_m,
        target_theta_rad=target_theta_rad,
        swing_duration_s=max(gate.end_s - gate.start_s, 1e-3),
        landing_beta_step_deg=landing_beta_step_deg,
    )
    if request is None:
        return TopRepositionAttempt2D(
            strategy=strategy, height_m=height_m, top_length_m=top_length_m,
            leg=leg, requirement=requirement, gate=gate,
            outcome=RepositionOutcome.TOUCHDOWN_UNSUITABLE,
            reason=f"support holds, but {note}",
            target=target, relaxed_floor=relaxed,
        )
    swing = generate_swing_2d(request, arc_samples=SWING_ARC_SAMPLES)
    if not swing.valid:
        return TopRepositionAttempt2D(
            strategy=strategy, height_m=height_m, top_length_m=top_length_m,
            leg=leg, requirement=requirement, gate=gate,
            outcome=RepositionOutcome.SWING_FAILED,
            reason=(f"support holds, but the swing planner refused: "
                    f"{swing.failure.value}"),
            swing=swing, target=target, relaxed_floor=relaxed,
        )

    segment = segment_from_swing_plan_2d(
        swing, request, kind=SegmentKind.TOP_REPOSITION_SWING,
        source_id=f"day12_step7:{strategy.name}", arc_samples=SWING_ARC_SAMPLES,
        leg_arc_samples=SWING_ARC_SAMPLES, apex_clearance_m=0.03,
        liftoff_rise_m=0.0, touchdown_drop_m=0.0,
    )

    accepted, why = target.accepts(segment)
    if not accepted:
        return TopRepositionAttempt2D(
            strategy=strategy, height_m=height_m, top_length_m=top_length_m,
            leg=leg, requirement=requirement, gate=gate,
            outcome=RepositionOutcome.TOUCHDOWN_UNSUITABLE,
            reason=f"support holds and the swing is valid, but {why}",
            swing=swing, segment=segment, target=target, relaxed_floor=relaxed,
        )

    return TopRepositionAttempt2D(
        strategy=strategy, height_m=height_m, top_length_m=top_length_m,
        leg=leg, requirement=requirement, gate=gate,
        outcome=RepositionOutcome.RESOLVED,
        reason=(f"{gate.reason()}; the Day 8-9 swing is collision-free, lands "
                f"as a {note}, and {why}"),
        swing=swing, segment=segment, target=target, relaxed_floor=relaxed,
    )


def reposition_rows(attempts) -> list[dict]:
    """The attempts and their support gates, as one writable table."""

    rows: list[dict] = []
    for attempt in attempts:
        rows.append({"row_kind": "attempt", **attempt.as_dict()})
        rows.append({"row_kind": "support_gate", **attempt.gate.as_dict()})

    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]
