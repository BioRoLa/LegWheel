"""Day 12 Step 10: one planner entry point, several terrains.

Plan §17.  The claim being tested is **not** "the planner clears 4 cm": it is
that the same whole-body Hybrid planner takes a rectangular terrain as a
*parameter* and returns either a trajectory or a structured infeasibility --
with no branch anywhere on the experiment's sizes.

:func:`plan_terrain_2d` is that entry point.  Flat ground is not a special
case in it: it is the same call with no obstacle, and it runs the same nominal
``FOOT_RIM_ROLL + RECOVERY_SWING`` cycle every other terrain runs between its
transitions.

**Nothing here reads a height and decides anything.**  The evaluation sizes
(flat, 4, 10, 19 cm x 40 cm) appear only in drivers and tests, as queries.
:func:`planner_size_literals` exists so a test can check that mechanically
rather than by reading the code and hoping.
"""

from __future__ import annotations

import re
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np

from hybrid_note.scripts.experiments.day10_11_composer_2d import (
    ComposedSequence2D,
    compose_2d,
)
from hybrid_note.scripts.experiments.day10_11_decision_map_2d import (
    Availability,
    DecisionTables2D,
    StrategyId,
    DEFAULT_ORDER,
    decide_2d,
)
from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import SegmentKind
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
)
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    BodyTrajectory2D,
    body_trajectory_2d,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER, LegId
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import NominalPosture2D
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    HIP_TO_BODY_Z_M,
)
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    DEFAULT_MARGIN_FLOOR_M,
    TraversalStability2D,
    swing_stability_2d,
)
from hybrid_note.scripts.experiments.day12_nominal_cycle_2d import (
    RecoveryConfig2D,
)
from hybrid_note.scripts.experiments.day12_support_margin_scan_2d import (
    HYBRID_MARGIN_FLOOR_M,
    hybrid_body_z_m,
    hybrid_posture_2d,
    hybrid_timing_2d,
)
from hybrid_note.scripts.experiments.day12_world_registration_2d import (
    SpeedZone2D,
    delay_overlapping_swings_2d,
    whole_body_schedule_2d,
    swing_hip_advance_m,
    world_leg_plans_2d,
    world_schedule_2d,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import (
    GaitTiming2D,
    walk_timing_2d,
)
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import (
    STRATEGY_HALVES,
    FourLegPlan2D,
    build_leg_plan_2d,
    insert_holds_2d,
    plan_four_legs_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    WholeBodyTrajectory2D,
    assemble_whole_body_2d,
)
from hybrid_note.scripts.experiments.day12_whole_body_validation_2d import (
    ValidationReport2D,
    validate_whole_body_2d,
)

#: The Day 12 modules that must contain no terrain-size logic.  Listed
#: explicitly so that adding a module to the pipeline without adding it here is
#: a visible omission rather than a silent gap.
PLANNER_MODULES: tuple[str, ...] = (
    "day12_nominal_cycle_2d.py",
    "day12_four_leg_state_2d.py",
    "day12_timing_skeleton_2d.py",
    "day12_transition_mapping_2d.py",
    "day12_body_trajectory_2d.py",
    "day12_support_stability_2d.py",
    "day12_whole_body_trajectory_2d.py",
    "day12_whole_body_validation_2d.py",
    "day12_terrain_generalization_2d.py",
)


def nominal_body_height_m(posture: NominalPosture2D | None = None) -> float:
    """The body height of the nominal stance, from the leg geometry alone.

    Step 2 measured that the nominal stance is on the **lower ground** and so
    does not depend on the platform -- which means it must not be obtained by
    building a platform.  An earlier version of this module did exactly that,
    with a 4 cm one, and :func:`planner_size_literals` caught it: a size the
    planner never needed had become a constant inside it.

    Solved instead from the sampled leg at the nominal posture, minus the
    offset between the leg plane and the body origin (Step 2, trap 17).
    """

    posture = NominalPosture2D() if posture is None else posture
    return float(posture.hip_z_for_flat_stance(0.0) - HIP_TO_BODY_Z_M)


class Stage(str, Enum):
    """Where a terrain stopped being planned."""

    DECISION = "decision"
    COMPOSITION = "composition"
    ASSEMBLY = "assembly"
    VALIDATION = "validation"


@dataclass(frozen=True)
class TerrainFailure2D:
    """One structured reason.  Plan §17: never relax a constraint to pass."""

    stage: Stage
    detail: str
    strategy: StrategyId | None = None
    limiter: str | None = None

    def as_dict(self) -> dict:
        return {
            "stage": self.stage.value,
            "strategy": None if self.strategy is None else self.strategy.value,
            "limiter": self.limiter,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class TerrainRun2D:
    """One terrain through the whole pipeline, or the reason it stopped."""

    height_m: float
    top_length_m: float
    #: ``None`` means flat ground -- the absence of an obstacle, not a size.
    terrain: SharedTerrainSpec2D | None
    plan: FourLegPlan2D | None = None
    body: BodyTrajectory2D | None = None
    stability: TraversalStability2D | None = None
    trajectory: WholeBodyTrajectory2D | None = None
    report: ValidationReport2D | None = None
    composed: ComposedSequence2D | None = None
    failures: tuple[TerrainFailure2D, ...] = ()
    #: One record per wait inserted by ``event_driven`` scheduling: which leg
    #: was held, for how long, and from when.  Empty when the option is off,
    #: which is the default.  Reported rather than folded away because a
    #: trajectory that pauses is a different thing to command than one that
    #: does not, even though no position in it changed (log 20).
    swing_waits: tuple[dict, ...] = ()

    @property
    def is_flat(self) -> bool:
        return self.terrain is None

    @property
    def planned(self) -> bool:
        """A trajectory was assembled.  Not the same as valid."""

        return self.trajectory is not None

    @property
    def feasible(self) -> bool:
        """Assembled **and** it passes Step 9.  Both, or it is not feasible."""

        return self.planned and self.report is not None and self.report.is_valid

    @property
    def ascent_primitive(self) -> str | None:
        if self.is_flat:
            return None
        if self.composed is None or self.composed.strategy is None:
            return None
        return STRATEGY_HALVES[self.composed.strategy][0]

    @property
    def descent_primitive(self) -> str | None:
        if self.is_flat:
            return None
        if self.composed is None or self.composed.strategy is None:
            return None
        return STRATEGY_HALVES[self.composed.strategy][1]

    def _segment_kinds(self) -> list[SegmentKind]:
        if self.plan is None:
            return []
        return [p.kind for leg_plan in self.plan.plans.values()
                for p in leg_plan.phased]

    @property
    def nominal_recovery_swings(self) -> int:
        """Plan §18 counts these apart from the ones the terrain forced."""

        return sum(1 for k in self._segment_kinds()
                   if k is SegmentKind.RECOVERY_SWING)

    @property
    def terrain_transition_swings(self) -> int:
        return sum(1 for k in self._segment_kinds()
                   if k.is_swing and k.is_terrain_transition)

    @property
    def usable_body_samples(self) -> int:
        """How many samples actually have a body height.

        Step 5 returns ``NaN`` where two hard requirements disagree, and
        ``body_z_travel_m`` skips those -- so with one finite sample it reports
        a travel of 0.  That 0 would read as "the body never lifts", which is
        the opposite of what it means.
        """

        if self.body is None:
            return 0
        return int(np.isfinite(self.body.body_z_m).sum())

    @property
    def max_body_lift_m(self) -> float | None:
        """``None`` when there are too few usable heights to measure a lift."""

        if self.body is None or self.usable_body_samples < 2:
            return None
        return self.body.body_z_travel_m

    @property
    def min_stability_margin_m(self) -> float | None:
        return None if self.stability is None else self.stability.minimum_margin_m

    @property
    def first_limiting_constraint(self) -> TerrainFailure2D | None:
        """Plan §17's ask for the challenge terrain: what stopped it first."""

        return self.failures[0] if self.failures else None

    def as_dict(self) -> dict:
        first = self.first_limiting_constraint
        lift = self.max_body_lift_m
        margin = self.min_stability_margin_m
        return {
            "terrain": "flat" if self.is_flat else
                       f"{self.height_m * 1e3:.0f}mm x {self.top_length_m * 1e3:.0f}mm",
            "height_mm": self.height_m * 1e3,
            "top_length_mm": self.top_length_m * 1e3,
            "planned": self.planned,
            "feasible": self.feasible,
            "ascent_primitive": self.ascent_primitive,
            "descent_primitive": self.descent_primitive,
            "nominal_recovery_swings": self.nominal_recovery_swings,
            "terrain_transition_swings": self.terrain_transition_swings,
            "max_body_lift_mm": None if lift is None else lift * 1e3,
            "usable_body_samples": self.usable_body_samples,
            "min_stability_margin_mm": None if margin is None else margin * 1e3,
            "failed_checks": (None if self.report is None
                              else len(self.report.failed_checks())),
            "failure_stage": None if first is None else first.stage.value,
            "failure_reason": None if first is None else first.detail,
        }


#: The decision rule this project crosses obstacles with, chosen 2026-09-02
#: (log section 1.7, problems B1/B2).
#:
#: ``DEFAULT_ORDER`` puts ``margin`` ahead of ``roll_preference``, which means
#: a tie on body deviation is settled by clearance and the rolling preference
#: -- the spec's own "terrain-transition swing only when required" -- never
#: fires at all.  Moving it ahead of ``margin`` is what makes the preference
#: reachable; ``margin`` stays as the final tie-break.
HYBRID_DECISION_ORDER: tuple[str, ...] = (
    "feasible", "body", "roll_preference", "margin")

#: How much more body deviation a rolling crossing may cost and still be
#: preferred, in metres.
#:
#: Measured, not picked.  Across the map, the cells where preferring rolling
#: changes the winner cost a median of 14.29 mm of extra body deviation -- the
#: structural overhead of climbing at a rolling contact -- with a long tail out
#: to 104.46 mm.  A 15 mm band therefore admits the overhead and refuses the
#: tail, and the sweep shows it is where the curve turns:
#:
#: ``` text
#:   tol 10 mm -> rolling wins 18.0% of solvable cells
#:   tol 15 mm -> 48.8%,  worst extra body 14.86 mm,  #5 keeps all 16 cells
#:   tol 20 mm -> 48.8%,  no more rolling, and #5 drops from 16 cells to 10
#: ```
#:
#: It is a **design choice** and has to be declared as one in any write-up:
#: nothing measures whether 15 mm of body deviation is worth a rolling
#: contact, because the cost that would settle it is energy, and the plan
#: forbids inferring energy here.
HYBRID_BODY_TOLERANCE_M: float = 0.015


def plan_terrain_2d(
    terrain: SharedTerrainSpec2D | None,
    tables: DecisionTables2D,
    *,
    timing: GaitTiming2D | None = None,
    nominal_body_z_m: float | None = None,
    order: Sequence[str] = HYBRID_DECISION_ORDER,
    body_tolerance_m: float = HYBRID_BODY_TOLERANCE_M,
    posture: NominalPosture2D | None = None,
    margin_floor_m: float = HYBRID_MARGIN_FLOOR_M,
    samples: int = 121,
    reposition_unresolved: int = 0,
    legacy_configuration: bool = False,
    world_registered: bool = True,
    crossing_stagger_m: dict[LegId, float] | None = None,
    event_driven: bool = False,
    speed_zones: Sequence[SpeedZone2D] = (),
    strategy: StrategyId | None = None,
    whole_body: bool = False,
) -> TerrainRun2D:
    """**The** entry point.  One terrain in, a trajectory or a reason out.

    Flat ground is ``terrain=None``: the absence of an obstacle, expressed by
    not having one rather than by a size comparison.  Every other path through
    this function is identical for every obstacle -- the terrain only ever
    travels as data, into Day 10--11's decision tables and into the scene.
    """

    # The chosen configuration, in one place.  ``legacy_configuration`` runs
    # the Day 12 originals instead -- duty 0.75, a fixed ``theta`` and the
    # 10 mm floor -- which is what every frozen Day 12 number was measured
    # with and is therefore how they are reproduced.
    if legacy_configuration:
        timing = walk_timing_2d() if timing is None else timing
        posture = NominalPosture2D() if posture is None else posture
        margin_floor_m = DEFAULT_MARGIN_FLOOR_M
        continuous = False
        use_frames = False
    else:
        timing = hybrid_timing_2d() if timing is None else timing
        posture = hybrid_posture_2d() if posture is None else posture
        continuous = True
        use_frames = True
    height_m = 0.0 if terrain is None else float(terrain.height_m)
    top_length_m = 0.0 if terrain is None else float(terrain.top_length_m)

    failures: list[TerrainFailure2D] = []
    composed: ComposedSequence2D | None = None

    if terrain is None:
        composed = ComposedSequence2D(
            strategy=StrategyId.SWING_SWING, height_m=0.0, top_length_m=0.0,
            sequence=None,
            refusal="flat ground: there is no crossing to compose",
        )
    else:
        decision = decide_2d(height_m, top_length_m, tables, order=order,
                             body_tolerance_m=body_tolerance_m)
        if decision.winner is None:
            # Plan §17: record the first limiting constraint, per strategy, so
            # a refusal names what stopped it rather than only that it stopped.
            for cell in decision.cells:
                if cell.availability is Availability.FEASIBLE:
                    continue
                failures.append(TerrainFailure2D(
                    stage=Stage.DECISION, strategy=cell.strategy,
                    limiter=getattr(cell.limiter, "value", None),
                    detail=(f"{cell.availability.value}: {cell.reason}"),
                ))
            if not failures:
                failures.append(TerrainFailure2D(
                    stage=Stage.DECISION,
                    detail="no strategy is available at this cell.",
                ))
            return TerrainRun2D(height_m=height_m, top_length_m=top_length_m,
                                terrain=terrain, failures=tuple(failures))

        # The same rule the decision above was read with.  Letting this
        # re-decide under a different one is how a run reports one strategy
        # and composes another.
        # ``strategy`` forces one instead of taking the decision rule's pick.
        # The rule ranks by body deviation, so it prefers SWING wherever both
        # work -- at 40 and 100 mm ROLL is feasible and loses by only 14.3 and
        # 14.9 mm.  That is a preference, not a necessity, and a caller with a
        # reason to want contact all the way up (less chance of tipping, no leg
        # raised near the face of the obstacle) should be able to say so.
        composed = compose_2d(height_m, top_length_m, tables, order=order,
                              body_tolerance_m=body_tolerance_m,
                              strategy=strategy)
        if not composed.composed:
            failures.append(TerrainFailure2D(
                stage=Stage.COMPOSITION, strategy=composed.strategy,
                detail=composed.refusal or "the crossing did not compose.",
            ))

    # ``world_registered`` is problem A5 (log 1.9-1.11).  Without it every leg
    # is handed the *same* crossing chain, all four starting at hip x = 0, and
    # the four of them then run it at four different times -- four legs each
    # crossing their own obstacle rather than four taking turns over one.  With
    # it, each leg rolls to where the obstacle actually is and the crossing is
    # rebased onto that.  It costs generation time, which is why it is a switch
    # and not a rewrite: ``False`` reproduces every number frozen before it.
    swing_waits: tuple[dict, ...] = ()
    if world_registered and composed.sequence is not None and terrain is not None:
        # The recovery has to carry the hip forward, and by exactly one swing
        # window's worth.  Zero -- the generator's default -- is a statement
        # about Step 1 not modelling the body; under position scheduling it
        # becomes a *claim* that a recovery takes no time at all, because a hip
        # that does not move needs no time to be carried (log 1.15, C5).
        config = RecoveryConfig2D(
            hip_advance_m=swing_hip_advance_m(timing, posture))
        # ``crossing_stagger_m`` is Day 13 B3, **measured and not adopted**:
        # it separates a pair in time only by crossing at different x, which
        # makes the robot go over the obstacle skewed.  Left reachable so the
        # measurement can be reproduced; see its field docs for why a non-zero
        # value currently loses the leg entirely (log 18.6, 19.1).
        plans = world_leg_plans_2d(composed, terrain, timing, posture, config,
                                   cycles_after=1,
                                   crossing_stagger_m=crossing_stagger_m)
        # And time comes from position, not from segment index: the four
        # chains are no longer the same length, so laying them on a shared
        # cycle stretches one leg's run against another's and the four end up
        # disagreeing about where the body is by 602.6 mm (log 1.14-1.15).
        # ``speed_zones`` lets the body walk the approach at its normal pace
        # and slow only where it matters.  It is applied to the *body's* x, so
        # all four legs read the same clock off the same body and still agree
        # about where it is -- unlike retiming one leg, which does not (log 25).
        if whole_body:
            # Day 13: schedule the four legs as one machine.  Position
            # scheduling makes a leg's time its claim about the body, so a
            # pair sharing a mount_x always swings together; holding the
            # *body's* clock delays every leg equally and so cannot make them
            # disagree (see whole_body_schedule_2d).
            schedule, swing_waits = whole_body_schedule_2d(
                plans, timing, posture, config, speed_zones=speed_zones)
            swing_waits = tuple(swing_waits)
        else:
            schedule = world_schedule_2d(plans, timing, posture, config,
                                         speed_zones=speed_zones)
        # Position scheduling has one consequence it cannot express, and
        # Day 13 measured that it cannot be fixed here either: the two legs of
        # a pair share a ``mount_x``, so their segments get identical times and
        # they always swing together.  Four ways of delaying a swing were tried
        # and all four fail for one reason -- under position scheduling a leg's
        # time *is* its claim about the body's position, so retiming two legs
        # relative to each other makes them disagree about the body (log 25).
        # ``event_driven`` therefore refuses rather than returning a schedule
        # that looks plausible.
        if event_driven:
            schedule, waits = delay_overlapping_swings_2d(schedule)
            swing_waits = tuple(waits)
    else:
        plans = {leg: build_leg_plan_2d(leg, composed, posture=posture,
                                        continuous_nominal=continuous)
                 for leg in LEG_ORDER}
        # Flat ground keeps the index schedule, and correctly: it is right
        # exactly while all four legs have the same chain, which on flat they
        # do.  Every frozen flat number was measured with it.
        schedule = None
    four = plan_four_legs_2d(plans, timing, schedule=schedule)
    # No hold segment is inserted for a *stretched* wait: the leg stays inside
    # the stance segment it already had, which is the whole point -- it keeps
    # contributing a continuous hip x, so ``body_x = median(...)`` never has to
    # re-solve (log 23).  ``insert_holds_2d`` remains for a wait that has no
    # stance to stretch, which the scheduler reports rather than assumes.
    # ``whole_body`` holds are already in the schedule's times: the scheduler
    # shifted the machine's whole timeline, so re-inserting them here would
    # apply the same delay twice and overlap a leg with itself (caught by
    # FourLegSchedule2D's "one leg does one thing at a time" invariant).
    holds = [w for w in swing_waits
             if not w.get("stretched_kind") and not w.get("waited_for")]
    if holds:
        four = insert_holds_2d(four, holds)

    if nominal_body_z_m is None:
        nominal_body_z_m = (nominal_body_height_m(posture)
                            if legacy_configuration else hybrid_body_z_m())

    body = body_trajectory_2d(four, nominal_body_z_m=nominal_body_z_m,
                              samples=samples,
                              world_registered=bool(
                                  world_registered and composed.sequence
                                  is not None and terrain is not None))
    stability = swing_stability_2d(four, body, margin_floor_m=margin_floor_m)
    trajectory = assemble_whole_body_2d(
        four, body, stability, samples=samples,
        reposition_unresolved=reposition_unresolved,
        use_generator_frames=use_frames)
    report = validate_whole_body_2d(trajectory, body, stability)

    for check in report.failed_checks():
        first = report.failures_of(check)[0]
        failures.append(TerrainFailure2D(
            stage=Stage.VALIDATION,
            detail=f"{check.value}: {first.detail}",
        ))

    return TerrainRun2D(
        height_m=height_m, top_length_m=top_length_m, terrain=terrain,
        plan=four, body=body, stability=stability, trajectory=trajectory,
        report=report, composed=composed, failures=tuple(failures),
        swing_waits=swing_waits,
    )


# --------------------------------------------------------------------------
# The generalization gate itself (requirement 8)
# --------------------------------------------------------------------------

#: Matches a decimal literal that could be one of the evaluation sizes, in
#: metres or millimetres.  Deliberately broad: it is better to have to justify
#: a match than to miss a hard-coded size.
_SIZE_LITERAL = re.compile(
    r"(?<![\w.])(0\.04|0\.10|0\.19|0\.40|40\.0|100\.0|190\.0|400\.0)(?![\w.])"
)


def planner_size_literals(root: Path | None = None) -> dict[str, list[str]]:
    """Every evaluation-size literal found in the planner modules.

    Plan §17 requirement 8 asks for a test that changing the obstacle size does
    not change the code path.  Reading the modules and hoping is not a test;
    this is the mechanical version, and it is deliberately dumb -- it reports
    matches and lets the test decide, rather than encoding exceptions here.
    """

    root = Path(__file__).resolve().parent if root is None else Path(root)
    found: dict[str, list[str]] = {}
    for name in PLANNER_MODULES:
        path = root / name
        if not path.exists():
            found[name] = ["MODULE MISSING"]
            continue
        hits = [
            line.strip()
            for line in path.read_text().splitlines()
            if _SIZE_LITERAL.search(line) and not line.strip().startswith("#")
        ]
        if hits:
            found[name] = hits
    return found


def comparison_rows(runs) -> list[dict]:
    """Plan §17 requirement 7's table, plus every structured failure."""

    rows: list[dict] = [{"row_kind": "terrain", **run.as_dict()} for run in runs]
    for run in runs:
        for failure in run.failures:
            rows.append({
                "row_kind": "failure",
                "terrain": run.as_dict()["terrain"],
                **failure.as_dict(),
            })

    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]
