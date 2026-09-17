"""Day 10--11 Step 5: the decision map on ``(h, L_top)``.

Spec section "Step 5".  The output is a **pure function**
``decide_2d(h, L_top) -> Decision2D``: given terrain geometry, which pair of
primitives to use, with what internal parameters, and what it then demands of
the body trajectory.

Three things make this more than a table lookup.

**1. ``theta_climb`` is decided by ``L_top``, not chosen freely.**
Day 6--7 Step 12R measured ``required_top_length`` for all 70 cells and it is a
function of ``theta_climb`` **alone** -- across every height the spread is under
3 um at eight of the ten thetas.  It falls monotonically, 269.4 mm at 40 deg to
205.0 mm at 85 deg.  Step 4 measured the other half: hip excursion *rises*
monotonically with theta, 14.2 mm of overhead at 40 deg to 49.4 mm at 85 deg.
So the two halves of the same knob point in opposite directions, and ``L_top``
is what resolves them:

    the cheapest feasible rolling plan is the SMALLEST theta whose
    required_top_length still fits on the top.

A shorter top forces a more extended climb, which costs body excursion, in
steps -- so rolling's cost is a **staircase in L_top**, not a constant.

**2. "not measured" is a third state.**  Day 6--7 swept h = 40..160 mm; Step 2
and Step 3 swept 20..200 mm.  Outside the overlap a strategy has no data, and
saying "infeasible" there would manufacture a region boundary out of a gap in
the experiments.  :class:`Availability` keeps the two apart.

**3. The lexicographic order is a parameter.**  The spec's last completion
criterion asks whether re-ordering it moves the region boundaries, so the order
has to be something the driver can vary rather than something the code assumes.
"""

from __future__ import annotations

import csv
from collections import defaultdict
from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import numpy as np

from hybrid_note.scripts.experiments.day10_11_concession_2d import BodyRequirementKind

#: Step 2's main grid fixes where an ascent lands, measured from the leading
#: edge.  Every ``#4`` top-length bound is conditional on it, so it is a
#: parameter rather than a constant baked into the rule.
SWING_LANDING_DISTANCE_M = 0.16

#: Step 2's close-out *did* sweep the landing distance (0.10--0.22 m) and found
#: 0.10 m feasible at h = 100 / 150 / 200 mm with **no change in hip lift**.
#: So ``#4``'s minimum top length is 60 mm shorter than the main grid implies --
#: a real region boundary, verified at three heights only, which is why the
#: driver reports it as a sensitivity beside the as-swept bound rather than
#: replacing it.
SWING_LANDING_DISTANCE_CLOSEOUT_M = 0.10

#: Day 6--7's ``step11r`` and Step 2 / Step 3 were all run at this top length.
#: Step 4's excursion numbers therefore need re-basing when ``L_top`` differs.
REFERENCE_TOP_LENGTH_M = 0.35

#: Whether ``required_top_length_m`` can be used as a feasibility bound.
#:
#: It cannot, unqualified.  It is reported by a traversal that *succeeded*: the
#: length that run consumed, not a precondition.  Two independent measurements
#: bracket how wrong it is, and they disagree about the size:
#:
#: ``theta = 40 deg`` (Day 6--7 Step 12R's own top-length sweep)
#:     ``L`` exactly at the requirement (269.4 mm) **failed**
#:     (``LEFT_RIM_HAS_NOT_TAKEN_OVER_AT_CORNER``); the next grid point,
#:     279.4 mm, succeeded.  So the true minimum is in ``(269.4, 279.4]`` and
#:     the requirement is optimistic by at most 10 mm -- a grid resolution,
#:     not a measured shortfall.
#:
#: ``theta = 70 deg, h = 140 mm`` (Step 7, bracketing directly)
#:     220 mm failed with the same reason and 225 mm composed, so the true
#:     minimum is in ``(220, 225]`` and the requirement, 222.5 mm, **is inside
#:     that bracket**.
#:
#: So a flat margin would be fitting one sweep's grid spacing to every theta,
#: and the second measurement says that overshoots.  The rule instead is:
#: **use the measured minimum where Step 12R measured one** (theta = 40 / 60 /
#: 85), and elsewhere use the requirement and mark the cell as carrying an
#: unvalidated bound.  ``top_length_bound_measured`` is that mark.
#:
#: **This makes the bound non-monotonic in theta**, and visibly so: theta = 60's
#: measured 247.2 mm sits above theta = 55's unvalidated 243.0 mm, even though
#: a more extended climb needs a *shorter* top.  That is the two sources
#: showing through -- a measured minimum includes the shortfall and a
#: requirement does not -- not a physical reversal.  It is left visible rather
#: than smoothed, because smoothing it would mean inventing the very margin the
#: second measurement rules out.


class StrategyId(str, Enum):
    """The 2x2 + 1 of spec 2.6.  Two are refuted; they stay so the map can say so."""

    ROLL_ROLL = "#1 ROLL_UP + ROLL_DOWN"
    ROLL_SWING = "#2 ROLL_UP + SWING_DOWN"
    SWING_ROLL = "#3 SWING_UP + ROLL_DOWN"
    SWING_SWING = "#4 SWING_UP + SWING_DOWN"
    SWING_OVER = "#5 SWING_OVER"


class Verdict(str, Enum):
    """How strong a claim a refusal is entitled to make.  Spec 5.5.

    Day 10--11's negative results are almost all **conditional**, and writing
    them as "the robot cannot do this" over-generalises -- most damagingly
    toward the multi-leg stage, which is exactly where they are most likely to
    be resolved.  A cell can be both ``DIRECT_HANDOFF_INFEASIBLE`` and
    ``REQUIRES_MULTILEG_REPOSITION``; only ``PHYSICALLY_INFEASIBLE`` needs
    evidence this project does not have.

    ``OUT_OF_ENVELOPE`` and ``NOT_MEASURED`` were added after the first pass
    labelled **every** refusal ``DIRECT_HANDOFF_INFEASIBLE``.  Most refusals in
    the map are neither: ``#1`` on a short top has not failed at a hand-over,
    it has run out of terrain.  Spec 5.5's original four were written about the
    ``#2`` / ``#3`` question, and stretching them over the whole map is the
    same over-generalisation pointed the other way.
    """

    #: A complete, collision-free sequence exists from the current planner.
    COMPOSED = "COMPOSED"
    #: The two contact primitives cannot be **handed over directly**.  A claim
    #: about the current primitive set, not about the robot.
    DIRECT_HANDOFF_INFEASIBLE = "DIRECT_HANDOFF_INFEASIBLE"
    #: May work once another leg can carry the body while this one lifts off
    #: and re-places itself (``TOP_REPOSITION``, spec 5.6).  **Unverified** --
    #: a single-leg 2D model has no way to check it.
    REQUIRES_MULTILEG_REPOSITION = "REQUIRES_MULTILEG_REPOSITION"
    #: The strategy is fine; **this terrain** is outside where it works, or
    #: outside what was swept.  A short top for ``#1``, a stride too long for
    #: ``#5``.  Not a hand-over failure and not a limit of the robot.
    OUT_OF_ENVELOPE = "OUT_OF_ENVELOPE"
    #: Nobody measured this cell.  Distinct from every kind of "no".
    NOT_MEASURED = "NOT_MEASURED"
    #: Reserved for reach / joint-limit / collision / support evidence.
    #: **Nothing in Day 10--11 currently qualifies**, which is the point.
    PHYSICALLY_INFEASIBLE = "PHYSICALLY_INFEASIBLE"


@dataclass(frozen=True)
class BlockedPair2D:
    """Why a pair will not chain, and what would be needed to revisit it.

    Replaces the bare string this used to be.  A string invites "refuted" to
    be quoted on its own; a record forces the caller to carry the verdict and
    the route back with it.
    """

    verdict: Verdict
    #: What was actually measured, and where.
    evidence: str
    #: What would have to change.  ``None`` when nothing in reach would.
    single_leg_fix: str | None
    multileg_route: str | None

    def as_dict(self) -> dict:
        return {
            "verdict": self.verdict.value,
            "evidence": self.evidence,
            "single_leg_fix": self.single_leg_fix or "",
            "multileg_route": self.multileg_route or "",
        }

    @property
    def summary(self) -> str:
        parts = [f"{self.verdict.value}: {self.evidence}"]
        if self.single_leg_fix:
            parts.append(f"single-leg fix: {self.single_leg_fix}")
        if self.multileg_route:
            parts.append(f"multi-leg route (UNVERIFIED): {self.multileg_route}")
        return "  ".join(parts)


#: Spec 5.5's restatement of the two pairs that will not chain.  **Neither is
#: ``PHYSICALLY_INFEASIBLE``**; both keep a route back.
BLOCKED_PAIRS: dict[StrategyId, BlockedPair2D] = {
    StrategyId.ROLL_SWING: BlockedPair2D(
        verdict=Verdict.DIRECT_HANDOFF_INFEASIBLE,
        evidence=(
            "Step 3 D/E -- (a) handing over directly from the right rim crosses "
            "the alpha = +40 deg rim seam; (b) the Day 6-7 retract fall-back "
            "stops at theta = 17 deg, below the descent's measured 35 deg floor."
        ),
        single_leg_fix=(
            "RETRACT_FOR_SWING_DOWN -- a retract that stops at theta >= 35 deg "
            "with the contact back on the foot rim.  Step 8 priced it: at "
            "h = 160 mm rolling up asks the body for 80.1 mm against the swing "
            "ascent's 220.0 mm, a factor of 2.7."
        ),
        multileg_route=(
            "TOP_REPOSITION under multi-leg support: release the right-rim "
            "contact and re-extend in the air (spec 5.6)."
        ),
    ),
    StrategyId.SWING_ROLL: BlockedPair2D(
        verdict=Verdict.DIRECT_HANDOFF_INFEASIBLE,
        evidence=(
            "Step 2b -- a swing lands on the foot rim at alpha ~ 0 deg, while "
            "roll-down needs the left rim near the trailing edge; connecting "
            "them directly crosses the alpha = -40 deg seam, a ~29 deg joint "
            "discontinuity that denser sampling converges onto rather than away."
        ),
        single_leg_fix=None,
        multileg_route=(
            "SWING_UP -> a safe foot-rim landing on the top -> TOP_REPOSITION "
            "under multi-leg support -> LEFT_RIM_READY -> ROLL_DOWN (spec 5.6)."
        ),
    ),
}

#: Kept so existing readers of the plain-string form keep working; the record
#: above is what new code should use.
REFUTATIONS: dict[StrategyId, str] = {
    strategy: blocked.summary for strategy, blocked in BLOCKED_PAIRS.items()
}


class Availability(str, Enum):
    """Feasible, infeasible, never measured, or blocked at the hand-over.

    Separate from :class:`Verdict` on purpose: this says what the *map* can do
    with a cell, while ``Verdict`` says how strong a claim the refusal earns.
    """

    FEASIBLE = "feasible"
    INFEASIBLE = "infeasible"
    NOT_MEASURED = "not measured"
    #: Renamed from ``REFUTED``.  "Refuted" reads as a claim about the robot;
    #: what was actually shown is that the two primitives will not chain.
    HANDOFF_BLOCKED = "handoff blocked"


class Limiter(str, Enum):
    """What stopped a strategy, so a region boundary can be attributed."""

    NONE = "none"
    ASCENT = "ascent"
    DESCENT = "descent"
    TOP_LENGTH = "top length"
    STRIDE = "stride"
    HANDOFF_BLOCKED = "handoff blocked"
    NO_DATA = "no data"


# --------------------------------------------------------------------------
# Tables read back off the earlier steps
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class RollThetaRow2D:
    height_m: float
    theta_climb_deg: float
    feasible: bool
    hip_z_travel_m: float | None
    min_collision_margin_m: float | None
    #: What a successful traversal consumed.  Available for all 70 cells, and
    #: **optimistic** as a bound -- see ``INFERRED_TOP_LENGTH_MARGIN_M``.
    required_top_length_m: float | None
    #: The shortest top Step 12R's sweep actually completed on.  Measured at
    #: three thetas only, so it is ``None`` for the rest.
    minimum_feasible_top_length_m: float | None = None


@dataclass(frozen=True)
class SwingUpRow2D:
    height_m: float
    approach_clearance_m: float
    feasible: bool
    min_hip_lift_m: float | None
    min_clearance_m: float | None
    #: The repairs the sweep needed on top of the body knob.  Step 7 found
    #: they are **not optional**: rebuilding a cell without them reproduces a
    #: ``TERRAIN_COLLISION`` the sweep had already repaired away, so a decision
    #: that omits them is not enough to regenerate the motion it chose.
    min_liftoff_rise_m: float = 0.0
    min_touchdown_drop_m: float = 0.0
    duration_scale: float = 1.0


@dataclass(frozen=True)
class SwingDownRow2D:
    height_m: float
    takeoff_distance_m: float
    feasible: bool
    min_hip_hold_fraction: float | None
    min_clearance_m: float | None
    min_liftoff_rise_m: float = 0.0
    min_touchdown_drop_m: float = 0.0
    duration_scale: float = 1.0


@dataclass(frozen=True)
class SwingOverRow2D:
    height_m: float
    top_length_m: float
    #: ``None`` on an infeasible row: no theta on the ladder worked, so there
    #: is no stance to report.  Every field below it is ``None`` for the same
    #: reason, which is why they are all optional.
    theta_deg: float | None
    feasible: bool
    approach_clearance_m: float | None
    stride_m: float | None
    #: How high the stance holds the hip above the most retracted one.  Kept
    #: beside the cell but **not** folded into ``body_deviation_m``: see
    #: :func:`swing_over_cell_2d`.
    stance_hip_above_min_m: float | None
    min_clearance_m: float | None
    failure: str | None
    duration_scale: float = 1.0


def _flag(text: str) -> bool:
    return text == "True"


def _num(text: str) -> float | None:
    text = (text or "").strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def _rows(path: Path) -> list[dict]:
    with Path(path).open(encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


@dataclass(frozen=True)
class DecisionTables2D:
    """Everything Steps 2--4 measured, keyed for the decision function."""

    roll: tuple[RollThetaRow2D, ...]
    swing_up: tuple[SwingUpRow2D, ...]
    swing_down: tuple[SwingDownRow2D, ...]
    swing_over: tuple[SwingOverRow2D, ...] = ()

    #: Heights each side actually swept, so ``NOT_MEASURED`` is decidable.
    def heights_for(self, strategy: StrategyId) -> set[float]:
        if strategy is StrategyId.ROLL_ROLL:
            return {round(r.height_m, 6) for r in self.roll}
        if strategy is StrategyId.SWING_SWING:
            return ({round(r.height_m, 6) for r in self.swing_up}
                    & {round(r.height_m, 6) for r in self.swing_down})
        if strategy is StrategyId.SWING_OVER:
            return {round(r.height_m, 6) for r in self.swing_over}
        return set()

    def required_top_length_for_theta(self) -> dict[float, float]:
        """``theta -> required_top_length``.  A function of theta alone.

        Verified rather than assumed: the caller can compare the spread across
        heights, which the driver prints.  Where several heights disagree
        slightly, the **largest** is kept -- a bound that is too generous
        refuses a plan that might work, which is the safe direction.

        This is what a successful traversal *consumed*.  Use
        :meth:`top_length_bound_for_theta` to decide feasibility.
        """

        grouped: dict[float, list[float]] = defaultdict(list)
        for row in self.roll:
            if row.required_top_length_m is not None:
                grouped[round(row.theta_climb_deg, 3)].append(row.required_top_length_m)
        return {theta: max(values) for theta, values in sorted(grouped.items())}

    def top_length_bound_for_theta(self) -> dict[float, tuple[float, bool]]:
        """``theta -> (shortest top that works, whether that was measured)``.

        The measured minimum where Step 12R covered that theta; the computed
        requirement elsewhere, flagged as unvalidated.  See the module header
        for why a flat margin is **not** used: the two direct brackets disagree
        about the size of the shortfall, and one of them puts the requirement
        inside the bracket.
        """

        measured = {
            round(row.theta_climb_deg, 3): row.minimum_feasible_top_length_m
            for row in self.roll
            if row.minimum_feasible_top_length_m is not None
        }
        out: dict[float, tuple[float, bool]] = {}
        for theta, required in self.required_top_length_for_theta().items():
            if theta in measured:
                out[theta] = (measured[theta], True)
            else:
                out[theta] = (required, False)
        return out


def load_tables_2d(day10_11_dir: Path, day6_7_dir: Path,
                   swing_over_csv: Path | None = None) -> DecisionTables2D:
    """Read Steps 2/3/4 and Day 6--7 back in.  No planner runs here."""

    day10_11_dir, day6_7_dir = Path(day10_11_dir), Path(day6_7_dir)

    # -- rolling: feasibility x theta, hip travel, and the top-length demand --
    travel: dict[tuple[float, float], dict] = {}
    for row in _rows(day10_11_dir / "day10_11_step4_roll_concession.csv"):
        if row["section"] != "cell":
            continue
        travel[(round(float(row["obstacle_mm"]) / 1e3, 6),
                round(float(row["theta_climb_deg"]), 3))] = row

    required: dict[tuple[float, float], float | None] = {}
    for row in _rows(day6_7_dir / "day6_7_step12r_transition_measurements_all_cells.csv"):
        required[(round(float(row["obstacle_height_m"]), 6),
                  round(float(row["theta_climb_deg"]), 3))] = _num(
            row["required_top_length_m"])

    # The measured minimum, which is a function of theta alone exactly as the
    # computed requirement is -- 279.4 / 247.2 / 215.0 mm at 40 / 60 / 85 deg,
    # identical across the heights the sweep covered.
    measured_minimum: dict[float, float] = {}
    for row in _rows(day6_7_dir / "day6_7_step12r_minimum_top_length.csv"):
        value = _num(row["minimum_feasible_top_length_m"])
        if value is not None:
            key = round(float(row["theta_climb_deg"]), 3)
            measured_minimum[key] = max(measured_minimum.get(key, 0.0), value)

    roll: list[RollThetaRow2D] = []
    for row in _rows(day6_7_dir / "day6_7_step11r_feasibility_sweep.csv"):
        key = (round(float(row["obstacle_height_m"]), 6),
               round(float(row["theta_climb_deg"]), 3))
        cell = travel.get(key, {})
        roll.append(RollThetaRow2D(
            height_m=key[0],
            theta_climb_deg=key[1],
            feasible=_flag(row["feasible"]),
            hip_z_travel_m=(
                None if not cell or not cell.get("hip_z_travel_mm")
                else float(cell["hip_z_travel_mm"]) / 1e3
            ),
            min_collision_margin_m=_num(row["minimum_collision_margin_m"]),
            required_top_length_m=required.get(key),
            minimum_feasible_top_length_m=measured_minimum.get(key[1]),
        ))

    swing_up = [
        SwingUpRow2D(
            height_m=round(float(row["obstacle_mm"]) / 1e3, 6),
            approach_clearance_m=float(row["approach_clearance_mm"]) / 1e3,
            feasible=_flag(row["feasible"]),
            min_hip_lift_m=(
                None if not _flag(row["feasible"])
                else float(row["min_hip_lift_mm"]) / 1e3
            ),
            min_clearance_m=(
                None if _num(row["min_clearance_mm"]) is None
                else _num(row["min_clearance_mm"]) / 1e3
            ),
            min_liftoff_rise_m=(_num(row["min_liftoff_rise_mm"]) or 0.0) / 1e3,
            min_touchdown_drop_m=(_num(row["min_touchdown_drop_mm"]) or 0.0) / 1e3,
            duration_scale=_num(row["duration_scale"]) or 1.0,
        )
        for row in _rows(day10_11_dir / "day10_11_step2_swing_onto_sweep.csv")
    ]

    swing_down = [
        SwingDownRow2D(
            height_m=round(float(row["obstacle_mm"]) / 1e3, 6),
            takeoff_distance_m=float(row["takeoff_distance_m"]),
            feasible=_flag(row["feasible"]),
            min_hip_hold_fraction=(
                None if not _flag(row["feasible"])
                else float(row["min_hip_hold_fraction"])
            ),
            min_clearance_m=(
                None if _num(row["min_clearance_mm"]) is None
                else _num(row["min_clearance_mm"]) / 1e3
            ),
            min_liftoff_rise_m=(_num(row["min_liftoff_rise_mm"]) or 0.0) / 1e3,
            min_touchdown_drop_m=(_num(row["min_touchdown_drop_mm"]) or 0.0) / 1e3,
            duration_scale=_num(row["duration_scale"]) or 1.0,
        )
        for row in _rows(day10_11_dir / "day10_11_step3_swing_off_sweep.csv")
        if row["section"] == "map"
    ]

    over: list[SwingOverRow2D] = []
    if swing_over_csv is not None and Path(swing_over_csv).exists():
        for row in _rows(Path(swing_over_csv)):
            over.append(SwingOverRow2D(
                height_m=round(float(row["obstacle_mm"]) / 1e3, 6),
                top_length_m=float(row["top_length_m"]),
                theta_deg=_num(row["theta_deg"]),
                feasible=_flag(row["feasible"]),
                approach_clearance_m=(
                    None if _num(row.get("approach_clearance_mm", "")) is None
                    else _num(row["approach_clearance_mm"]) / 1e3
                ),
                stride_m=(
                    None if _num(row.get("stride_mm", "")) is None
                    else _num(row["stride_mm"]) / 1e3
                ),
                stance_hip_above_min_m=(
                    None if _num(row.get("stance_hip_above_min_mm", "")) is None
                    else _num(row["stance_hip_above_min_mm"]) / 1e3
                ),
                min_clearance_m=(
                    None if _num(row.get("min_clearance_mm", "")) is None
                    else _num(row["min_clearance_mm"]) / 1e3
                ),
                failure=(row.get("failure") or None),
                duration_scale=_num(row.get("duration_scale", "")) or 1.0,
            ))

    return DecisionTables2D(
        roll=tuple(roll), swing_up=tuple(swing_up),
        swing_down=tuple(swing_down), swing_over=tuple(over),
    )


# --------------------------------------------------------------------------
# One strategy in one cell
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class StrategyCell2D:
    """What one strategy costs at one ``(h, L_top)``, or why it cannot be used."""

    strategy: StrategyId
    height_m: float
    top_length_m: float
    availability: Availability
    limiter: Limiter
    reason: str
    #: How strong a claim this cell's refusal earns (spec 5.5).  ``COMPOSED``
    #: for a feasible cell; ``PHYSICALLY_INFEASIBLE`` is deliberately never
    #: produced here, because nothing in Day 10--11 has the evidence for it.
    verdict: Verdict | None = None

    #: The magnitude Step 4's rule compares: how far the body must move
    #: vertically, at this strategy's own minimum.
    body_deviation_m: float | None = None
    requirement_kind: BodyRequirementKind = BodyRequirementKind.NONE
    min_clearance_m: float | None = None

    #: Whatever internal freedom the strategy resolved, so the decision is
    #: reproducible and Step 7 can rebuild the motion from it.
    parameters: tuple[tuple[str, float], ...] = ()

    @property
    def feasible(self) -> bool:
        return self.availability is Availability.FEASIBLE

    @property
    def effective_verdict(self) -> Verdict:
        """The spec 5.5 label, derived when a caller did not set one.

        A cell that was measured and failed is **not** promoted to
        ``PHYSICALLY_INFEASIBLE``: the sweeps show a planner refusing, not the
        robot's limits.  Where no label fits, the cell keeps ``None`` rather
        than borrowing one.
        """

        if self.verdict is not None:
            return self.verdict
        return {
            Availability.FEASIBLE: Verdict.COMPOSED,
            Availability.HANDOFF_BLOCKED: Verdict.DIRECT_HANDOFF_INFEASIBLE,
            Availability.NOT_MEASURED: Verdict.NOT_MEASURED,
            # A strategy that needs a longer top, or a stride it cannot reach,
            # has **not** failed at a hand-over.  Labelling it
            # ``DIRECT_HANDOFF_INFEASIBLE`` would be the same over-generalisation
            # spec 5.5 exists to stop, pointed the other way.
            Availability.INFEASIBLE: Verdict.OUT_OF_ENVELOPE,
        }[self.availability]

    def as_dict(self) -> dict:
        row = {
            "strategy": self.strategy.value,
            "obstacle_mm": self.height_m * 1e3,
            "top_length_m": self.top_length_m,
            "availability": self.availability.value,
            "verdict": self.effective_verdict.value,
            "limiter": self.limiter.value,
            "feasible": self.feasible,
            "body_deviation_mm": (
                None if self.body_deviation_m is None
                else self.body_deviation_m * 1e3
            ),
            "requirement_kind": self.requirement_kind.value,
            "min_clearance_mm": (
                None if self.min_clearance_m is None else self.min_clearance_m * 1e3
            ),
            "reason": self.reason,
        }
        row.update({name: value for name, value in self.parameters})
        return row


#: Whether a height between two swept heights may borrow the harder of them.
#:
#: Day 13.  Every cell used to look its height up by exact float equality, so
#: 130 mm was refused even though 120 and 140 both work -- the map answered for
#: the heights that happened to be swept and for nothing in between, which
#: makes it a lookup table rather than a strategy for crossing obstacles.
#:
#: Interpolating is sound in one direction only.  If the sweep found a height
#: feasible on both sides of ``h``, the leg's reach, clearance and rim budget at
#: ``h`` lie between two measured feasible cases, so *reading the worse
#: neighbour* is a conservative answer rather than an invented one.  Going
#: outside the swept range is not: nothing measured bounds it, and guessing
#: there would manufacture an envelope out of missing experiments.  So this
#: brackets, and refuses to extrapolate.
INTERPOLATION_NOTE: str = (
    "height not swept directly; answered from the nearest swept height above "
    "it, which bounds the demand because reach and clearance vary monotonically "
    "with height between two measured points"
)


#: How much untraversed top ``#4`` may leave, in metres.
#:
#: ``#4`` is exactly two swings, so any top they do not between them cover has
#: to be crossed by a segment nobody generated.  A few millimetres is the
#: generating grid meeting itself -- one roll step is 4 mm and the hand-over
#: tolerance is 10 mm -- so 20 mm admits a coarse boundary and refuses a tear.
TOP_BRIDGE_TOLERANCE_M: float = 0.020


#: Whether a swept height's own result is healthy enough to answer for its
#: neighbours.
#:
#: Measured 2026-09-07, after a first version of this reached too far and I
#: mis-diagnosed why.  Bracketing 130 mm from 140 mm gives 24 Step 9 failures
#: -- identical to the 120 and 140 mm cells it sits between.  Bracketing 190 mm
#: from 200 mm gives 182, including ``segment_chaining: contact teleports``.
#:
#: The tempting reading is that the interpolation over-reached.  It did not:
#: **180 and 200 mm, both swept directly, give the same 182 failures with the
#: same 8 chaining breaks.**  The interpolation faithfully passed on the
#: quality of its source; the source is what is broken.  A gap-width limit was
#: tried first and cannot tell the two cases apart -- both gaps are 20 mm.
#:
#: So nothing is limited here.  A height between two swept heights is answered
#: from the harder of them, and if that answer is poor it is poor for the same
#: reason the measured neighbour is.  Hiding that behind a refusal would make
#: the map claim ignorance where it actually has a bad measurement, which is
#: the more misleading of the two.


def bracketing_height_m(height_m: float, swept: set[float]) -> float | None:
    """The swept height to answer for ``height_m`` with, or ``None``.

    Returns the nearest swept height **at or above** ``height_m`` -- the harder
    of the two neighbours, so a cell answered this way is never more optimistic
    than something that was actually measured.

    ``None`` in two cases, both of which stay ``NOT_MEASURED``:

    * ``height_m`` is outside the swept range.  An extrapolated envelope
      boundary would be an artefact of where the sweeps stopped, not a fact
      about the robot.
    Note that a *poor* answer is still returned when the neighbour's own
    result is poor: see the note above this function.  This function reports
    what the sweeps support, not whether the sweeps were any good.
    """

    if not swept:
        return None
    key = round(float(height_m), 6)
    if key in swept:
        return key
    lo = [h for h in swept if h < key]
    hi = [h for h in swept if h > key]
    if not lo or not hi:
        return None          # outside the measured range: do not guess
    return min(hi)


def _not_measured(strategy, height_m, top_length_m, what: str) -> StrategyCell2D:
    return StrategyCell2D(
        strategy=strategy, height_m=height_m, top_length_m=top_length_m,
        availability=Availability.NOT_MEASURED, limiter=Limiter.NO_DATA,
        reason=f"no data: {what} was never swept at this height.",
    )


def roll_roll_cell_2d(
    height_m: float, top_length_m: float, tables: DecisionTables2D
) -> StrategyCell2D:
    """``#1``: the smallest theta whose required top length still fits.

    Both halves of ``theta_climb`` are monotone and they oppose each other --
    excursion rises with theta, required top length falls with it -- so the
    optimum is always at the constraint boundary, and no search is needed.
    """

    key = bracketing_height_m(height_m, tables.heights_for(StrategyId.ROLL_ROLL))
    if key is None:
        return _not_measured(StrategyId.ROLL_ROLL, height_m, top_length_m,
                             "the rolling traversal")
    interpolated = key != round(height_m, 6)

    candidates = [
        row for row in tables.roll
        if round(row.height_m, 6) == key and row.feasible
    ]
    if not candidates:
        return StrategyCell2D(
            strategy=StrategyId.ROLL_ROLL, height_m=height_m,
            top_length_m=top_length_m, availability=Availability.INFEASIBLE,
            limiter=Limiter.ASCENT,
            reason="no theta_climb completes the traversal at this height.",
        )

    bounds = tables.top_length_bound_for_theta()
    fitting = [
        row for row in candidates
        if bounds.get(round(row.theta_climb_deg, 3)) is not None
        and bounds[round(row.theta_climb_deg, 3)][0] <= top_length_m
    ]
    if not fitting:
        cheapest = min(
            (bounds[round(row.theta_climb_deg, 3)][0] for row in candidates
             if bounds.get(round(row.theta_climb_deg, 3)) is not None),
            default=None,
        )
        return StrategyCell2D(
            strategy=StrategyId.ROLL_ROLL, height_m=height_m,
            top_length_m=top_length_m, availability=Availability.INFEASIBLE,
            limiter=Limiter.TOP_LENGTH,
            reason=(
                "every feasible theta needs a longer top; the least demanding "
                f"needs {cheapest * 1e3:.1f} mm."
                if cheapest is not None
                else "no feasible theta has a measured top-length demand."
            ),
        )

    best = min(
        fitting,
        key=lambda row: (
            row.hip_z_travel_m if row.hip_z_travel_m is not None else np.inf,
            row.theta_climb_deg,
        ),
    )
    bound, measured = bounds[round(best.theta_climb_deg, 3)]
    return StrategyCell2D(
        strategy=StrategyId.ROLL_ROLL, height_m=height_m, top_length_m=top_length_m,
        availability=Availability.FEASIBLE, limiter=Limiter.NONE,
        reason=(
            f"theta_climb = {best.theta_climb_deg:.0f} deg, the smallest that "
            f"fits (needs {bound * 1e3:.1f} mm of top, "
            f"{'measured' if measured else 'inferred'})."
        ),
        body_deviation_m=best.hip_z_travel_m,
        requirement_kind=BodyRequirementKind.TRACK,
        min_clearance_m=best.min_collision_margin_m,
        parameters=(
            ("theta_climb_deg", best.theta_climb_deg),
            ("required_top_length_m", bound),
            ("top_length_bound_measured", 1.0 if measured else 0.0),
            ("feasible_theta_count", float(len(fitting))),
        ),
    )


def swing_swing_cell_2d(
    height_m: float, top_length_m: float, tables: DecisionTables2D,
    *, landing_distance_m: float = SWING_LANDING_DISTANCE_M,
) -> StrategyCell2D:
    """``#4``: an ascent that lands, and a descent that still has room to leave.

    The coupling is geometric.  The ascent lands ``landing_distance_m`` past
    the leading edge and the descent leaves ``takeoff`` short of the trailing
    one, so the two only both fit when
    ``takeoff <= L_top - landing_distance_m``.  Step 3 already measured which
    takeoffs work; this only intersects that with what the top can hold.
    """

    key = bracketing_height_m(height_m, tables.heights_for(StrategyId.SWING_SWING))
    if key is None:
        return _not_measured(StrategyId.SWING_SWING, height_m, top_length_m,
                             "the swing pair")
    interpolated = key != round(height_m, 6)

    ups = [r for r in tables.swing_up if round(r.height_m, 6) == key and r.feasible]
    if not ups:
        return StrategyCell2D(
            strategy=StrategyId.SWING_SWING, height_m=height_m,
            top_length_m=top_length_m, availability=Availability.INFEASIBLE,
            limiter=Limiter.ASCENT,
            reason="no approach clearance produces a feasible ascent.",
        )
    best_up = min(ups, key=lambda r: (r.min_hip_lift_m, -(r.min_clearance_m or 0.0)))

    downs = [r for r in tables.swing_down if round(r.height_m, 6) == key and r.feasible]
    if not downs:
        return StrategyCell2D(
            strategy=StrategyId.SWING_SWING, height_m=height_m,
            top_length_m=top_length_m, availability=Availability.INFEASIBLE,
            limiter=Limiter.DESCENT,
            reason="no takeoff distance produces a feasible descent.",
        )

    room_m = top_length_m - landing_distance_m
    fitting = [r for r in downs if r.takeoff_distance_m <= room_m + 1e-12]
    if not fitting:
        shortest = min(r.takeoff_distance_m for r in downs)
        return StrategyCell2D(
            strategy=StrategyId.SWING_SWING, height_m=height_m,
            top_length_m=top_length_m, availability=Availability.INFEASIBLE,
            limiter=Limiter.TOP_LENGTH,
            reason=(
                f"the ascent lands {landing_distance_m * 1e3:.0f} mm in and the "
                f"shortest feasible takeoff is {shortest * 1e3:.0f} mm, so the "
                f"top must be at least "
                f"{(landing_distance_m + shortest) * 1e3:.0f} mm."
            ),
        )
    # Choose a takeoff that actually **reaches** the ascent's landing point.
    #
    # ``#4`` is two swings and nothing between them, so the top between where
    # the ascent lands and where the descent leaves is crossed by no segment at
    # all.  The cheapest takeoff by hip hold is the shortest one (80 mm at every
    # height), which on a 400 mm top leaves 160 mm uncrossed -- and that is
    # exactly the tear Step 9 reports at 200 mm as eight ``segment_chaining``
    # breaks among 182 failures.
    #
    # The sweep measured longer takeoffs too, and a longer one closes the gap:
    # 240 mm is feasible at every height up to 100 mm, which leaves nothing
    # uncrossed on a 400 mm top.  So prefer the takeoffs that bridge, and fall
    # back to the cheapest only when none of them do -- at 200 mm the longest
    # feasible takeoff is 120 mm, so 120 mm of top stays uncrossed no matter
    # what is chosen, and the cell is honestly blocked rather than quietly
    # producing a sequence that tears.
    #
    # An earlier version of this refused on the gap alone, which was wrong
    # twice over: the gap is identical at every height, so it also killed the
    # low cells that work, and it never tried the longer takeoffs the sweep had
    # already measured.
    bridging = [r for r in fitting
                if top_length_m - landing_distance_m - r.takeoff_distance_m
                <= TOP_BRIDGE_TOLERANCE_M]
    if bridging:
        best_down = min(
            bridging,
            key=lambda r: (r.min_hip_hold_fraction, -(r.min_clearance_m or 0.0)),
        )
    else:
        longest = max(r.takeoff_distance_m for r in fitting)
        uncrossed_m = top_length_m - landing_distance_m - longest
        return StrategyCell2D(
            strategy=StrategyId.SWING_SWING, height_m=height_m,
            top_length_m=top_length_m,
            availability=Availability.HANDOFF_BLOCKED,
            limiter=Limiter.HANDOFF_BLOCKED,
            reason=(
                f"the ascent lands {landing_distance_m * 1e3:.0f} mm in and the "
                f"longest feasible takeoff is {longest * 1e3:.0f} mm, so "
                f"{uncrossed_m * 1e3:.0f} mm of top is crossed by neither "
                "swing and #4 has no segment that traverses it (Day 12's "
                "unresolved TOP_REPOSITION)."
            ),
            parameters=(
                ("uncrossed_top_m", uncrossed_m),
                ("longest_takeoff_m", longest),
                ("landing_distance_m", landing_distance_m),
            ),
        )

    # Composition, spec task 3: item-wise maximum of the two demands.  Both are
    # "how high the body must be", so the pair's demand is the larger -- and the
    # ascent's ``h + lift`` dominates because a hold can never exceed ``h``.
    ascent_demand_m = height_m + best_up.min_hip_lift_m
    descent_demand_m = best_down.min_hip_hold_fraction * height_m
    clearances = [c for c in (best_up.min_clearance_m, best_down.min_clearance_m)
                  if c is not None]
    return StrategyCell2D(
        strategy=StrategyId.SWING_SWING, height_m=height_m, top_length_m=top_length_m,
        availability=Availability.FEASIBLE, limiter=Limiter.NONE,
        reason=(
            f"lift {best_up.min_hip_lift_m * 1e3:.0f} mm at c = "
            f"{best_up.approach_clearance_m * 1e3:.0f} mm; takeoff "
            f"{best_down.takeoff_distance_m * 1e3:.0f} mm at hold "
            f"{best_down.min_hip_hold_fraction:.3f}."
        ),
        body_deviation_m=max(ascent_demand_m, descent_demand_m),
        requirement_kind=BodyRequirementKind.LOWER_BOUND,
        min_clearance_m=min(clearances) if clearances else None,
        parameters=(
            ("approach_clearance_m", best_up.approach_clearance_m),
            ("min_hip_lift_m", best_up.min_hip_lift_m),
            ("takeoff_distance_m", best_down.takeoff_distance_m),
            ("min_hip_hold_fraction", best_down.min_hip_hold_fraction),
            # The repairs the sweep needed.  Not decorative: Step 7 found that
            # rebuilding without them reproduces a collision the sweep had
            # already repaired away.
            ("ascent_liftoff_rise_m", best_up.min_liftoff_rise_m),
            ("ascent_touchdown_drop_m", best_up.min_touchdown_drop_m),
            ("ascent_duration_scale", best_up.duration_scale),
            ("descent_liftoff_rise_m", best_down.min_liftoff_rise_m),
            ("descent_touchdown_drop_m", best_down.min_touchdown_drop_m),
            ("descent_duration_scale", best_down.duration_scale),
        ),
    )


def swing_over_cell_2d(
    height_m: float, top_length_m: float, tables: DecisionTables2D
) -> StrategyCell2D:
    """``#5``: one swing that never touches the top.

    The only strategy for which a **longer** top is worse -- it has to be
    spanned in a single stride.  Its internal freedom is ``theta``: a more
    extended stance raises the hip at both ends at once, which is the only way
    a straight-line ``HipTrajectory2D`` can put the foot high enough to clear
    the obstacle mid-flight.

    **Between the swept top lengths this closes over monotonically**, because
    the measurement says it may.  The sweep's feasible set is a strict prefix
    at every height (h = 20 mm clears all ten tops, 40 mm the first nine, 60 mm
    the first eight, 80 mm the first six) and the theta it needs never falls as
    the top grows.  A shorter top is a shorter stride at the same height, so
    that nesting is the physics rather than a coincidence -- and without the
    closure the map would show ``#5`` as isolated dots at the ten swept
    columns, which reads as a scattered result instead of a region.
    """

    key = round(height_m, 6)
    if key not in tables.heights_for(StrategyId.SWING_OVER):
        return _not_measured(StrategyId.SWING_OVER, height_m, top_length_m,
                             "the over-swing")

    at_height = [r for r in tables.swing_over if round(r.height_m, 6) == key]
    feasible = [r for r in at_height if r.feasible]
    if not feasible:
        failures = sorted({r.failure for r in at_height if r.failure})
        return StrategyCell2D(
            strategy=StrategyId.SWING_OVER, height_m=height_m,
            top_length_m=top_length_m, availability=Availability.INFEASIBLE,
            limiter=Limiter.STRIDE,
            reason=(
                "no theta spans this obstacle at any measured top length: "
                f"{', '.join(failures) or 'no reason recorded'}."
            ),
        )

    widest = max(r.top_length_m for r in feasible)
    if top_length_m > widest + 1e-12:
        return StrategyCell2D(
            strategy=StrategyId.SWING_OVER, height_m=height_m,
            top_length_m=top_length_m, availability=Availability.INFEASIBLE,
            limiter=Limiter.STRIDE,
            reason=(
                f"the widest top an over-swing clears at this height is "
                f"{widest * 1e3:.0f} mm."
            ),
        )

    # Conservative within the closure: take the cheapest stance among the swept
    # tops that are at least as wide as this one, so an interpolated cell never
    # claims a more retracted stance than a measured wider top needed.
    covering = [r for r in feasible if r.top_length_m >= top_length_m - 1e-12]
    best = min(covering, key=lambda r: (r.theta_deg, -(r.min_clearance_m or 0.0)))
    exact = any(abs(r.top_length_m - top_length_m) < 1e-9 for r in feasible)

    # Both ends stand on the lower ground at the same theta, so the straight-
    # line hip trajectory is horizontal: the crossing asks the body to move
    # vertically **not at all**.  That is a real property of the strategy, and
    # it is why ``#5`` wins the body comparison wherever it is feasible.
    #
    # It is not free.  Being in that stance holds the hip
    # ``stance_hip_above_min_m`` higher than the most retracted pose, for the
    # whole crossing.  That is deliberately **not** folded into the comparison:
    # whether it is already paid depends on the gait between obstacles, which
    # is Day 15--16's question, not this one.  It is reported instead.
    return StrategyCell2D(
        strategy=StrategyId.SWING_OVER, height_m=height_m, top_length_m=top_length_m,
        availability=Availability.FEASIBLE, limiter=Limiter.NONE,
        reason=(
            f"theta = {best.theta_deg:.0f} deg"
            + ("" if exact else f" (from the {best.top_length_m * 1e3:.0f} mm column)")
            + f"; zero hip excursion, but the stance holds the hip "
            f"{(best.stance_hip_above_min_m or 0.0) * 1e3:.0f} mm above the most "
            "retracted one throughout."
        ),
        body_deviation_m=0.0,
        requirement_kind=BodyRequirementKind.LOWER_BOUND,
        min_clearance_m=best.min_clearance_m,
        parameters=(
            ("theta_deg", best.theta_deg),
            ("stride_m", best.stride_m if best.stride_m is not None else float("nan")),
            ("stance_hip_above_min_m",
             best.stance_hip_above_min_m if best.stance_hip_above_min_m is not None
             else float("nan")),
            ("measured_exactly", 1.0 if exact else 0.0),
            # The stance the sweep actually succeeded from.  Without it a
            # rebuild stands somewhere else and collides.
            ("approach_clearance_m",
             best.approach_clearance_m if best.approach_clearance_m is not None
             else float("nan")),
            ("duration_scale", best.duration_scale),
        ),
    )


def blocked_pair_cell_2d(
    strategy: StrategyId, height_m: float, top_length_m: float
) -> StrategyCell2D:
    """A pair whose two primitives will not chain.  Spec 5.5.

    Was ``refuted_cell_2d``.  The old name and its ``REFUTED`` availability
    both read as claims about the robot; what was measured is that the current
    primitives will not hand over, which is a claim about the primitive set.
    """

    blocked = BLOCKED_PAIRS[strategy]
    return StrategyCell2D(
        strategy=strategy, height_m=height_m, top_length_m=top_length_m,
        availability=Availability.HANDOFF_BLOCKED,
        limiter=Limiter.HANDOFF_BLOCKED,
        reason=blocked.summary,
        verdict=blocked.verdict,
    )


#: The previous name, kept so nothing outside this module breaks on the rename.
refuted_cell_2d = blocked_pair_cell_2d


# --------------------------------------------------------------------------
# The decision
# --------------------------------------------------------------------------

#: Spec task 4.  First version is lexicographic with no weights, so a region
#: boundary is decided by the data rather than a tuning knob.
DEFAULT_ORDER: tuple[str, ...] = ("feasible", "body", "margin", "roll_preference")

#: Spec 6.2 asks for a margin floor, and this is where it goes -- but the
#: **default is to trust the planner's own verdict**, for a measured reason.
#:
#: All three sweeps report a handful of feasible plans whose minimum clearance
#: is a few times ``1e-7 m``: the tightest point of a swing is usually the
#: touchdown, where the clearance is zero by construction.  Those numbers are
#: a thousand times finer than the planner's own ``collision_tolerance_m`` of
#: ``1e-3 m``, so a floor at exactly ``0.0`` would overturn the model's verdict
#: with a sharper threshold than the model itself uses -- and it did: it opened
#: a false "no strategy works" band at h = 60 and 80 mm.
#:
#: ``None`` means "the planner already judged this".  The driver reports
#: explicit floors as a sensitivity, which is what 6.2 is actually for.
DEFAULT_MARGIN_FLOOR_M: float | None = None

#: The tie-break of spec task 4 item 4, as an explicit ranking.
ROLL_PREFERENCE: dict[StrategyId, int] = {
    StrategyId.ROLL_ROLL: 0,
    StrategyId.ROLL_SWING: 1,
    StrategyId.SWING_ROLL: 2,
    StrategyId.SWING_SWING: 3,
    StrategyId.SWING_OVER: 4,
}


def _passes_margin(cell: StrategyCell2D, margin_floor_m: float | None) -> bool:
    if not cell.feasible:
        return False
    if margin_floor_m is None or cell.min_clearance_m is None:
        return True
    return cell.min_clearance_m >= margin_floor_m


#: How much worse than the best feasible body deviation a strategy may be and
#: still count as tied on ``body``.
#:
#: **Zero reproduces the original rule exactly**, and not approximately: the
#: body term becomes ``max(0, body - best)``, which is a monotone shift of
#: ``body`` and therefore sorts identically.  A test holds that.
#:
#: It exists because a strict ``body`` comparison decides the whole map on
#: differences of a few millimetres, and ``roll_preference`` -- the spec's own
#: "terrain-transition swing only when required" -- then never fires at all.
#: See the Day 12 log, section 1.7.
DEFAULT_BODY_TOLERANCE_M: float = 0.0


def _sort_key(
    cell: StrategyCell2D,
    order: Sequence[str],
    margin_floor_m: float | None,
    body_reference_m: float | None = None,
    body_tolerance_m: float = DEFAULT_BODY_TOLERANCE_M,
):
    feasible = _passes_margin(cell, margin_floor_m)
    parts = []
    for name in order:
        if name == "feasible":
            parts.append(0 if feasible else 1)
        elif name == "body":
            if not (feasible and cell.body_deviation_m is not None):
                parts.append(np.inf)
            elif body_reference_m is None:
                parts.append(cell.body_deviation_m)
            else:
                # Everything within the tolerance of the best ties at 0 and
                # falls through to the next key; everything beyond it keeps
                # ordering by how far beyond.
                parts.append(max(0.0, cell.body_deviation_m
                                 - (body_reference_m + body_tolerance_m)))
        elif name == "margin":
            parts.append(
                -cell.min_clearance_m
                if (feasible and cell.min_clearance_m is not None) else np.inf
            )
        elif name == "roll_preference":
            parts.append(ROLL_PREFERENCE[cell.strategy])
        else:
            raise ValueError(f"unknown ordering key: {name!r}")
    return tuple(parts)


@dataclass(frozen=True)
class Decision2D:
    """The pure function's output: what to do at one ``(h, L_top)``, and why."""

    height_m: float
    top_length_m: float
    cells: tuple[StrategyCell2D, ...]
    order: tuple[str, ...]
    winner: StrategyId | None
    #: Every strategy that could have been used, so the map can shade the
    #: region where the choice is a preference rather than a necessity.
    feasible_strategies: tuple[StrategyId, ...] = ()
    #: True when the winner was not decided by body cost -- the spec asks for
    #: these cells to be marked on the figure.
    decided_by_tie_break: bool = False
    #: How much body deviation the runner-up would have cost.  This is the
    #: quantity the "both feasible" region exists to show.
    cost_gap_m: float | None = None

    @property
    def winner_cell(self) -> StrategyCell2D | None:
        if self.winner is None:
            return None
        return next(c for c in self.cells if c.strategy is self.winner)

    def as_dict(self) -> dict:
        winner = self.winner_cell
        return {
            "obstacle_mm": self.height_m * 1e3,
            "top_length_m": self.top_length_m,
            "winner": None if self.winner is None else self.winner.value,
            "feasible_count": len(self.feasible_strategies),
            "feasible_strategies": ", ".join(
                s.value for s in self.feasible_strategies
            ),
            "decided_by_tie_break": self.decided_by_tie_break,
            "cost_gap_mm": None if self.cost_gap_m is None else self.cost_gap_m * 1e3,
            "winner_body_deviation_mm": (
                None if winner is None or winner.body_deviation_m is None
                else winner.body_deviation_m * 1e3
            ),
            "winner_requirement_kind": (
                None if winner is None else winner.requirement_kind.value
            ),
            "winner_reason": None if winner is None else winner.reason,
            "limiters": "; ".join(
                f"{c.strategy.value}={c.limiter.value}" for c in self.cells
                if not c.feasible
            ),
        }


def decide_2d(
    height_m: float,
    top_length_m: float,
    tables: DecisionTables2D,
    *,
    order: Sequence[str] = DEFAULT_ORDER,
    margin_floor_m: float | None = DEFAULT_MARGIN_FLOOR_M,
    landing_distance_m: float = SWING_LANDING_DISTANCE_M,
    body_tolerance_m: float = DEFAULT_BODY_TOLERANCE_M,
) -> Decision2D:
    """``(h, L_top) -> (ascent, descent) + internal parameters + body demand``.

    Pure: every input is an argument, and ``tables`` is frozen data read off
    the earlier steps.  Nothing here runs a planner or touches the filesystem.
    """

    cells = (
        roll_roll_cell_2d(height_m, top_length_m, tables),
        refuted_cell_2d(StrategyId.ROLL_SWING, height_m, top_length_m),
        refuted_cell_2d(StrategyId.SWING_ROLL, height_m, top_length_m),
        swing_swing_cell_2d(height_m, top_length_m, tables,
                            landing_distance_m=landing_distance_m),
        swing_over_cell_2d(height_m, top_length_m, tables),
    )
    order = tuple(order)
    usable = [c.body_deviation_m for c in cells
              if _passes_margin(c, margin_floor_m)
              and c.body_deviation_m is not None]
    reference = min(usable) if usable else None
    ranked = sorted(cells, key=lambda c: _sort_key(
        c, order, margin_floor_m, reference, body_tolerance_m))
    feasible = tuple(
        c.strategy for c in ranked if _passes_margin(c, margin_floor_m)
    )
    if not feasible:
        return Decision2D(height_m, top_length_m, cells, order, None)

    best, *rest = [c for c in ranked if c.strategy in feasible]
    runner_up = rest[0] if rest else None
    gap = None
    tie_break = False
    if runner_up is not None:
        if (best.body_deviation_m is not None
                and runner_up.body_deviation_m is not None):
            gap = runner_up.body_deviation_m - best.body_deviation_m
        # "Decided by tie-break" means every key *before* the roll preference
        # left the top two equal, so the preference alone picked the winner.
        # Checking only the body cost would under-report: two cells can tie on
        # body and still be separated by margin, which is not a tie-break.
        # These are exactly the cells a different lexicographic order can move,
        # which is what the spec asks to have marked on the figure.
        prefix = tuple(k for k in order if k != "roll_preference")
        if prefix and prefix != order:
            tie_break = (
                _sort_key(best, prefix, margin_floor_m, reference,
                          body_tolerance_m)
                == _sort_key(runner_up, prefix, margin_floor_m, reference,
                             body_tolerance_m)
            )
    return Decision2D(
        height_m=height_m, top_length_m=top_length_m, cells=cells, order=order,
        winner=best.strategy, feasible_strategies=feasible,
        decided_by_tie_break=tie_break, cost_gap_m=gap,
    )
