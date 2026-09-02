"""Day 10--11 Step 1: what each primitive asks of the body, as a comparable number.

Day 8--9 reports the price of a swing as ``adjustments: tuple[str, ...]`` --
``("hip raised 60 mm", "lift-off raised 30 mm")``.  That is the right thing for
a showcase to print and the wrong thing to sort a decision on.  Step 1 turns it
into fields.

Two things this module refuses to paper over.

**A greedy search result is not a minimum.**  ``swing_onto_step_2d`` walks a hip
ladder until a cheap reachability probe passes, then repairs the trajectory --
and the repair can invalidate the reach the ladder stopped at, with no
backtracking.  Measured on 2026-08-29: 140 mm passes with ``hip +40``, 160 mm
passes with ``hip +60``, and **150 mm fails**.  So every concession carries a
:class:`ConcessionSource` saying how its numbers were obtained.  Step 2 replaces
``GREEDY_LADDER`` values with ``GRID_MINIMUM`` ones; until then, a hole in the
map may be an artefact of the search order rather than of the geometry.

**Roll and swing do not ask for the same *kind* of thing.**  A swing wants the
hip at least so high at one moment -- a lower bound.  A roll has no such
freedom: its hip height is an output of ``theta_climb`` and the contact
geometry, so what it asks for is that the body *follow* a particular height
profile -- a trajectory.  Collapsing both to one scalar and calling the smaller
one cheaper would be assuming the answer to the question Step 4 exists to ask.
:attr:`BodyRequirementKind` keeps the distinction in the type, and
:func:`compare_concessions` refuses to rank across kinds without being told how.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Sequence

import numpy as np

from .cartesian_swing_contract_2d import SwingFailure
from .cartesian_swing_planner_2d import StepSwingShowcase2D
from .right_up_left_down_full_traversal_2d import RollingTraversalResult2D

__all__ = [
    "BindingCeiling",
    "ConcessionSource",
    "BodyRequirementKind",
    "SwingConcession2D",
    "swing_concession_from_showcase",
    "adjustment_strings_from_concession",
    "RollCellConcession2D",
    "roll_cell_concession_from_result",
    "RollConcession2D",
    "roll_concession_from_cells",
    "compare_concessions",
    "concession_rows",
]


class BindingCeiling(str, Enum):
    """Which limit decided this cell, and therefore what the body is asked for.

    Day 8--9 §28 describes two ceilings that bind in order.  Running the
    showcase over a wide height range surfaces a third that the two-ceiling
    story does not cover: the leg cannot legally *stand* at one endpoint, so
    no swing is attempted at all.  It is kept separate because it asks for a
    different fix -- approach or landing distance, not hip height or control
    points.
    """

    NONE = "none"
    #: Neither endpoint pose is legal; no trajectory was attempted.
    STANCE = "stance"
    #: Ceiling 1.  The apex is out of the leg's range even at the top of the
    #: hip ladder; theta bottoms out against its limit.
    REACH = "reach"
    #: Ceiling 2.  The reach probe passed and the repair ran, but the result
    #: is still not usable.  This names the *stage* that was reached, not the
    #: cause -- ``SwingConcession2D.failure`` carries the cause, and the pair
    #: ``FIT`` + a reach-shaped failure is the greedy-ladder signature; see
    #: ``SwingConcession2D.greedy_backtrack_suspected``.
    FIT = "fit"
    NOT_EVALUATED = "not_evaluated"


class ConcessionSource(str, Enum):
    """How the numbers in a concession were obtained.

    This is not bookkeeping.  A ``GREEDY_LADDER`` value answers "what did the
    search settle for", a ``GRID_MINIMUM`` value answers "what is the least the
    body must give" -- and only the second one may be compared across cells.
    """

    GREEDY_LADDER = "greedy_ladder"
    GRID_MINIMUM = "grid_minimum"
    NOT_SEARCHED = "not_searched"


class BodyRequirementKind(str, Enum):
    """The shape of the demand a primitive places on the body trajectory."""

    #: "the hip must be at least this high at this moment" -- one inequality.
    LOWER_BOUND = "lower_bound"
    #: "the hip must be at exactly this height at this moment" -- an equality.
    #: Measured in Step 2b: a swing whose touchdown contact state is fully
    #: specified (rim, alpha *and* theta, as ``LEFT_RIM_READY`` requires) has
    #: no hip freedom left at the endpoint.  Raising the hip does not make the
    #: landing easier, it makes it a *different* landing -- one at a larger
    #: theta, which fails the precondition.  This is strictly harder than a
    #: lower bound and must not be silently ranked against one.
    PINNED = "pinned"
    #: "the hip must follow this height profile" -- a trajectory to track.
    TRACK = "track"
    NONE = "none"


#: Failures whose cause is the leg not *reaching*, as opposed to not *fitting*.
#: Used to spot a reach-shaped failure that surfaced in the fit stage, which is
#: the signature of the greedy ladder having been overrun by the repair.
_REACH_FAILURES = frozenset({
    SwingFailure.IK_NOT_CONVERGED,
    SwingFailure.IK_RESIDUAL_TOO_LARGE,
    SwingFailure.JOINT_LIMIT_VIOLATION,
})


def _optional(value) -> float | None:
    if value is None:
        return None
    value = float(value)
    return value if np.isfinite(value) else None


# --------------------------------------------------------------------------
# Swing side
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SwingConcession2D:
    """What one swing costs the body, in fields rather than in prose.

    Every field that was not measured is ``None``, never ``0.0`` -- the same
    contract rule Day 8--9 Step 1 froze.  ``0.0`` here means "measured, and the
    answer was nothing"; that distinction is what lets Step 5 tell a free cell
    from an unknown one.
    """

    feasible: bool
    direction: str
    obstacle_height_m: float
    source: ConcessionSource
    binding_ceiling: BindingCeiling

    #: The shared Day 10--11 approach axis.  ``None`` until Step 2 sweeps it;
    #: the Day 8--9 showcase holds it fixed and does not report it.
    approach_clearance_m: float | None = None

    #: Climb: how much higher the hip must be.  Descent: what fraction of the
    #: step height the hip must *refuse* to drop.  Exactly one is not ``None``.
    min_hip_lift_m: float | None = None
    min_hip_hold_fraction: float | None = None

    min_liftoff_rise_m: float | None = None
    min_touchdown_drop_m: float | None = None
    #: ``swing_duration_s / original_duration_s``; 1.0 when nothing was extended.
    duration_scale: float | None = None

    min_clearance_m: float | None = None
    theta_min_deg: float | None = None
    failure: SwingFailure | None = None
    reason: str | None = None

    #: Set by Step 2b.  ``None`` means the endpoint left the hip free in the
    #: usual way, so the demand is a lower bound.  A number means the touchdown
    #: contact state fixed the hip height at exactly this much above the
    #: landing surface, and the demand is an equality -- see
    #: :class:`BodyRequirementKind`.
    pinned_hip_above_surface_m: float | None = None

    def __post_init__(self) -> None:
        if self.direction not in ("onto", "off"):
            raise ValueError('direction must be "onto" or "off".')
        if self.direction == "onto" and self.min_hip_hold_fraction is not None:
            raise ValueError("a climb is priced in hip lift, not hip hold.")
        if self.direction == "off" and self.min_hip_lift_m is not None:
            raise ValueError("a descent is priced in hip hold, not hip lift.")
        if self.feasible and self.binding_ceiling is not BindingCeiling.NONE:
            raise ValueError("a feasible swing has no binding ceiling.")
        if not self.feasible and self.binding_ceiling is BindingCeiling.NONE:
            raise ValueError("an infeasible swing must say what stopped it.")
        if (
            self.pinned_hip_above_surface_m is not None
            and self.min_hip_lift_m not in (None, 0.0)
        ):
            raise ValueError(
                "a pinned landing has no hip-lift freedom; a non-zero lift "
                "would land the leg at a different theta, which is a "
                "different contact state."
            )

    @property
    def requirement_kind(self) -> BodyRequirementKind:
        """What shape of demand this swing places on the body trajectory.

        A swing never asks for a trajectory.  It usually asks for an
        inequality -- but Step 2b found the exception: when the touchdown
        contact state pins theta as well as the contact point, the hip height
        is an output of the request rather than a knob, and the demand becomes
        an equality.
        """

        if not self.feasible:
            return BodyRequirementKind.NONE
        if self.pinned_hip_above_surface_m is not None:
            return BodyRequirementKind.PINNED
        return BodyRequirementKind.LOWER_BOUND

    @property
    def body_deviation_m(self) -> float | None:
        """The hip displacement this swing demands, in metres.

        For a descent the demand is stated as a fraction of the step height,
        because that is the whole distance the body would otherwise fall; this
        converts it so the two directions can be added along a sequence.
        """

        if self.direction == "onto":
            return self.min_hip_lift_m
        if self.min_hip_hold_fraction is None:
            return None
        return float(self.min_hip_hold_fraction * self.obstacle_height_m)

    @property
    def greedy_backtrack_suspected(self) -> bool:
        """Whether this cell looks like the 150 mm hole rather than real geometry.

        Reaching the fit stage means the reach probe passed.  Failing *there*
        with a reach-shaped cause therefore means the repair moved the
        trajectory back out of range after the hip ladder had already stopped
        escalating -- and the search does not go back.

        Measured on 2026-08-29: 140 mm passes at ``hip +40``, 160 mm passes at
        ``hip +60``, and 150 mm lands exactly here.  A ``GRID_MINIMUM`` search
        cannot produce this combination, so Step 2 can assert it never does.
        """

        return (
            self.source is ConcessionSource.GREEDY_LADDER
            and self.binding_ceiling is BindingCeiling.FIT
            and self.failure in _REACH_FAILURES
        )

    @property
    def is_comparable(self) -> bool:
        """Whether this value may be ranked against another cell's.

        A greedy-ladder number says where a search stopped, which is not a
        property of the terrain.  Comparing two of them compares two search
        histories.
        """

        return self.source is ConcessionSource.GRID_MINIMUM

    def as_dict(self) -> dict:
        return {
            "direction": self.direction,
            "obstacle_mm": self.obstacle_height_m * 1e3,
            "approach_clearance_mm": (
                None if self.approach_clearance_m is None
                else self.approach_clearance_m * 1e3
            ),
            "feasible": self.feasible,
            "source": self.source.value,
            "binding_ceiling": self.binding_ceiling.value,
            "min_hip_lift_mm": (
                None if self.min_hip_lift_m is None else self.min_hip_lift_m * 1e3
            ),
            "min_hip_hold_fraction": self.min_hip_hold_fraction,
            "body_deviation_mm": (
                None if self.body_deviation_m is None else self.body_deviation_m * 1e3
            ),
            "min_liftoff_rise_mm": (
                None if self.min_liftoff_rise_m is None else self.min_liftoff_rise_m * 1e3
            ),
            "min_touchdown_drop_mm": (
                None if self.min_touchdown_drop_m is None
                else self.min_touchdown_drop_m * 1e3
            ),
            "duration_scale": self.duration_scale,
            "min_clearance_mm": (
                None if self.min_clearance_m is None else self.min_clearance_m * 1e3
            ),
            "theta_min_deg": self.theta_min_deg,
            "pinned_hip_above_surface_mm": (
                None if self.pinned_hip_above_surface_m is None
                else self.pinned_hip_above_surface_m * 1e3
            ),
            "requirement_kind": self.requirement_kind.value,
            "failure": None if self.failure is None else self.failure.value,
            "greedy_backtrack_suspected": self.greedy_backtrack_suspected,
            "reason": self.reason,
        }


def _binding_ceiling_from_showcase(showcase: StepSwingShowcase2D) -> BindingCeiling:
    """Read which ceiling stopped a showcase off the branch it returned from.

    The three branches are distinguishable without parsing prose:

    * no ``plan`` at all -> neither endpoint pose was legal (``STANCE``);
    * a ``plan`` but no ``adjustments`` and the hip at the top of its ladder
      -> the reach probe never passed (``REACH``);
    * anything else infeasible -> the repair ran and did not clear it (``FIT``).
    """

    if showcase.feasible:
        return BindingCeiling.NONE
    if showcase.plan is None:
        return BindingCeiling.STANCE
    if not showcase.adjustments:
        return BindingCeiling.REACH
    return BindingCeiling.FIT


def swing_concession_from_showcase(
    showcase: StepSwingShowcase2D,
    *,
    approach_clearance_m: float | None = None,
    source: ConcessionSource = ConcessionSource.GREEDY_LADDER,
) -> SwingConcession2D:
    """Re-express a Day 8--9 showcase as a concession.

    Defaults to ``GREEDY_LADDER`` because that is what the showcase actually
    is.  Step 2 passes ``GRID_MINIMUM`` after searching the full grid.
    """

    if not isinstance(showcase, StepSwingShowcase2D):
        raise TypeError("showcase must be a StepSwingShowcase2D.")

    climbing = showcase.direction == "onto"
    #: The showcase stores the descent knob already multiplied by the step
    #: height, so converting back is division -- and the height is never zero,
    #: because ``swing_onto_step_2d`` / ``swing_off_step_2d`` both reject it.
    hold_fraction = (
        None if climbing else float(showcase.hip_lift_m / showcase.obstacle_height_m)
    )
    attempted = showcase.plan is not None
    return SwingConcession2D(
        feasible=bool(showcase.feasible),
        direction=showcase.direction,
        obstacle_height_m=float(showcase.obstacle_height_m),
        source=source,
        binding_ceiling=_binding_ceiling_from_showcase(showcase),
        approach_clearance_m=_optional(approach_clearance_m),
        min_hip_lift_m=float(showcase.hip_lift_m) if climbing else None,
        min_hip_hold_fraction=hold_fraction,
        min_liftoff_rise_m=float(showcase.liftoff_rise_m) if attempted else None,
        min_touchdown_drop_m=float(showcase.touchdown_drop_m) if attempted else None,
        duration_scale=(
            float(showcase.swing_duration_s / showcase.original_duration_s)
            if attempted and showcase.original_duration_s > 0.0 else None
        ),
        min_clearance_m=_optional(showcase.minimum_clearance_m),
        theta_min_deg=_optional(showcase.theta_min_deg),
        failure=None if showcase.plan is None else showcase.plan.failure,
        reason=showcase.reason,
    )


def adjustment_strings_from_concession(
    concession: SwingConcession2D,
    *,
    original_duration_s: float | None = None,
) -> tuple[str, ...]:
    """Rebuild the showcase's ``adjustments`` tuple from the numeric fields.

    This is the losslessness proof, and it is deliberately a reconstruction
    rather than a parser.  If every string the showcase printed can be
    regenerated from the fields, then no information lives only in the prose --
    which is the thing Step 1 has to establish before Step 5 sorts on the
    fields alone.

    ``original_duration_s`` is needed only to reproduce the duration string,
    since a scale factor alone cannot say what it scaled from.
    """

    if concession.binding_ceiling in (BindingCeiling.STANCE, BindingCeiling.REACH):
        # Neither branch appends anything: the stance branch returns before the
        # ladder runs, and the reach branch returns from inside it.
        return ()

    parts: list[str] = []
    label = "hip raised" if concession.direction == "onto" else (
        "hip held above the landing pose by"
    )
    hip_m = (
        concession.min_hip_lift_m if concession.direction == "onto"
        else (
            None if concession.min_hip_hold_fraction is None
            else concession.min_hip_hold_fraction * concession.obstacle_height_m
        )
    )
    if hip_m is not None and hip_m > 0.0:
        parts.append(f"{label} {hip_m * 1e3:.0f} mm")
    if concession.min_liftoff_rise_m:
        parts.append(f"lift-off raised {concession.min_liftoff_rise_m * 1e3:.0f} mm")
    if concession.min_touchdown_drop_m:
        parts.append(
            f"touchdown approached from {concession.min_touchdown_drop_m * 1e3:.0f} mm up"
        )
    if (
        concession.duration_scale is not None
        and original_duration_s is not None
        and concession.duration_scale > 1.0 + 1e-12
    ):
        parts.append(
            f"swing lengthened {original_duration_s:.2f} -> "
            f"{original_duration_s * concession.duration_scale:.2f} s"
        )
    return tuple(parts)


# --------------------------------------------------------------------------
# Rolling side
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class RollCellConcession2D:
    """One ``(h, L_top, theta_climb, c)`` cell of the rolling side.

    The hip range is the whole point.  ``TraversalInitialState2D`` leaves
    ``hip_z_m = None`` on purpose: standing on the lower ground is the initial
    condition, and the hip height that realises it -- and every height after it
    -- follows from ``theta_climb`` and the contact geometry.  So the rolling
    side has no hip knob to price; what it has is a height profile the body has
    no choice but to follow, and its extent is what Step 4 compares against the
    swing side's lower bound.
    """

    feasible: bool
    obstacle_height_m: float
    top_length_m: float
    theta_climb_deg: float
    approach_clearance_m: float

    hip_z_min_m: float | None = None
    hip_z_max_m: float | None = None
    hip_z_start_m: float | None = None
    l_transition_m: float | None = None
    min_collision_margin_m: float | None = None
    frame_count: int = 0
    phases_visited: tuple[str, ...] = ()
    failure_stage: str | None = None
    failure_reason: str | None = None

    @property
    def requirement_kind(self) -> BodyRequirementKind:
        return BodyRequirementKind.NONE if not self.feasible else BodyRequirementKind.TRACK

    @property
    def hip_z_travel_m(self) -> float | None:
        """How far the hip is carried vertically over the whole traversal."""

        if self.hip_z_min_m is None or self.hip_z_max_m is None:
            return None
        return float(self.hip_z_max_m - self.hip_z_min_m)

    @property
    def hip_rise_above_start_m(self) -> float | None:
        """How far above its standing height the hip is taken.

        This is the closest rolling analogue of the swing side's hip lift, and
        it is *not* the same quantity: the swing may satisfy its bound however
        it likes, while this one is prescribed at every instant.  Step 4 has to
        say which is harder; Step 1 only has to keep them apart.
        """

        if self.hip_z_max_m is None or self.hip_z_start_m is None:
            return None
        return float(self.hip_z_max_m - self.hip_z_start_m)

    def as_dict(self) -> dict:
        return {
            "obstacle_mm": self.obstacle_height_m * 1e3,
            "top_length_m": self.top_length_m,
            "theta_climb_deg": self.theta_climb_deg,
            "approach_clearance_mm": self.approach_clearance_m * 1e3,
            "feasible": self.feasible,
            "hip_z_start_m": self.hip_z_start_m,
            "hip_z_min_m": self.hip_z_min_m,
            "hip_z_max_m": self.hip_z_max_m,
            "hip_z_travel_mm": (
                None if self.hip_z_travel_m is None else self.hip_z_travel_m * 1e3
            ),
            "hip_rise_above_start_mm": (
                None if self.hip_rise_above_start_m is None
                else self.hip_rise_above_start_m * 1e3
            ),
            "l_transition_m": self.l_transition_m,
            "min_collision_margin_mm": (
                None if self.min_collision_margin_m is None
                else self.min_collision_margin_m * 1e3
            ),
            "frame_count": self.frame_count,
            "phases_visited": " -> ".join(self.phases_visited),
            "requirement_kind": self.requirement_kind.value,
            "failure_stage": self.failure_stage,
            "failure_reason": self.failure_reason,
        }


def roll_cell_concession_from_result(
    result: RollingTraversalResult2D,
    *,
    top_length_m: float,
    approach_clearance_m: float,
) -> RollCellConcession2D:
    """Price one rolling cell off a traversal that has already been run."""

    accepted = [frame for frame in result.trajectory if frame.accepted]
    hip_z = [frame.hip_position_world_xz_m[1] for frame in accepted]
    return RollCellConcession2D(
        feasible=bool(result.full_success),
        obstacle_height_m=float(result.obstacle.height_m),
        top_length_m=float(top_length_m),
        theta_climb_deg=float(np.rad2deg(result.theta_climb_rad)),
        approach_clearance_m=float(approach_clearance_m),
        hip_z_min_m=float(min(hip_z)) if hip_z else None,
        hip_z_max_m=float(max(hip_z)) if hip_z else None,
        hip_z_start_m=float(hip_z[0]) if hip_z else None,
        l_transition_m=_optional(result.l_transition_m),
        min_collision_margin_m=_optional(result.minimum_collision_margin_m),
        frame_count=len(accepted),
        phases_visited=tuple(result.phases_visited),
        failure_stage=result.failure_stage,
        failure_reason=result.failure_reason,
    )


@dataclass(frozen=True)
class RollConcession2D:
    """One ``(h, L_top)`` after taking the best over the rolling side's own knob.

    ``theta_climb`` is rolling's internal freedom, so a fair comparison against
    swing takes the best value of it -- exactly as the swing side is allowed to
    choose its approach clearance.  The width of the feasible window is kept
    too: a cell that works for one theta out of ten is feasible on paper and
    fragile in practice, and Step 5 needs to be able to see that.
    """

    feasible: bool
    obstacle_height_m: float
    top_length_m: float
    approach_clearance_m: float
    feasible_cell_count: int
    evaluated_cell_count: int
    best_theta_climb_deg: float | None = None
    min_theta_climb_deg: float | None = None
    max_theta_climb_deg: float | None = None
    #: ``max - min`` over the feasible thetas.  On its own this **overstates**
    #: robustness, because the window can have holes -- Day 6--7's own h = 0.12
    #: row is feasible at 45 and 55 deg and infeasible at 50.  Read it together
    #: with :attr:`theta_window_is_contiguous`.
    feasible_theta_span_deg: float = 0.0
    #: Every theta that worked, so the shape of the window survives aggregation.
    feasible_theta_degs: tuple[float, ...] = ()
    #: Every theta that was tried, feasible or not.
    evaluated_theta_degs: tuple[float, ...] = ()
    hip_z_travel_m: float | None = None
    hip_rise_above_start_m: float | None = None
    l_transition_m: float | None = None
    min_collision_margin_m: float | None = None
    failure_stages: tuple[str, ...] = ()

    @property
    def requirement_kind(self) -> BodyRequirementKind:
        return BodyRequirementKind.NONE if not self.feasible else BodyRequirementKind.TRACK

    @property
    def theta_window_is_contiguous(self) -> bool:
        """Whether every evaluated theta inside the window actually works.

        Day 6--7's h = 0.12 row is the counterexample this exists for: feasible
        at 45 and 55 deg, infeasible at 50, so a 10 deg "span" describes a comb
        rather than a window.  A tie-break that prefers the wider span would
        pick the comb, which is the opposite of robust.
        """

        if not self.feasible_theta_degs:
            return False
        low, high = self.feasible_theta_degs[0], self.feasible_theta_degs[-1]
        inside = [
            theta for theta in self.evaluated_theta_degs if low <= theta <= high
        ]
        return len(inside) == len(self.feasible_theta_degs)

    @property
    def contiguous_span_around_best_deg(self) -> float:
        """The widest hole-free run of feasible thetas containing the best one.

        This is the number to tie-break on: it is what survives if the robot
        cannot hit ``theta_climb`` exactly.
        """

        if self.best_theta_climb_deg is None or not self.evaluated_theta_degs:
            return 0.0
        feasible = set(self.feasible_theta_degs)
        ordered = list(self.evaluated_theta_degs)
        try:
            index = ordered.index(self.best_theta_climb_deg)
        except ValueError:
            return 0.0
        low = high = index
        while low - 1 >= 0 and ordered[low - 1] in feasible:
            low -= 1
        while high + 1 < len(ordered) and ordered[high + 1] in feasible:
            high += 1
        return float(ordered[high] - ordered[low])

    @property
    def body_deviation_m(self) -> float | None:
        """The rolling side's hip demand, in metres.

        Reported as the *travel* rather than a bound, and paired with
        ``requirement_kind = TRACK`` so nobody reads it as one.
        """

        return self.hip_z_travel_m

    def as_dict(self) -> dict:
        return {
            "obstacle_mm": self.obstacle_height_m * 1e3,
            "top_length_m": self.top_length_m,
            "approach_clearance_mm": self.approach_clearance_m * 1e3,
            "feasible": self.feasible,
            "feasible_cells": f"{self.feasible_cell_count} / {self.evaluated_cell_count}",
            "best_theta_climb_deg": self.best_theta_climb_deg,
            "feasible_theta_span_deg": self.feasible_theta_span_deg,
            "theta_window_is_contiguous": self.theta_window_is_contiguous,
            "contiguous_span_around_best_deg": self.contiguous_span_around_best_deg,
            "feasible_theta_degs": ", ".join(f"{v:g}" for v in self.feasible_theta_degs),
            "hip_z_travel_mm": (
                None if self.hip_z_travel_m is None else self.hip_z_travel_m * 1e3
            ),
            "hip_rise_above_start_mm": (
                None if self.hip_rise_above_start_m is None
                else self.hip_rise_above_start_m * 1e3
            ),
            "body_deviation_mm": (
                None if self.body_deviation_m is None else self.body_deviation_m * 1e3
            ),
            "l_transition_m": self.l_transition_m,
            "min_collision_margin_mm": (
                None if self.min_collision_margin_m is None
                else self.min_collision_margin_m * 1e3
            ),
            "requirement_kind": self.requirement_kind.value,
            "failure_stages": ", ".join(self.failure_stages),
        }


def roll_concession_from_cells(
    cells: Sequence[RollCellConcession2D],
) -> RollConcession2D:
    """Take the best over ``theta_climb`` for one ``(h, L_top)``.

    "Best" is the smallest hip travel among the feasible cells, which is the
    same ordering Step 5 will apply across primitives.  Ties fall to the wider
    margin, for the reason §6.2 of the spec gives: the clearances in play are
    millimetres, so a cell that only just passes is not equivalent to one that
    passes comfortably.
    """

    cells = tuple(cells)
    if not cells:
        raise ValueError("no cells to aggregate.")
    heights = {round(cell.obstacle_height_m, 12) for cell in cells}
    tops = {round(cell.top_length_m, 12) for cell in cells}
    clearances = {round(cell.approach_clearance_m, 12) for cell in cells}
    if len(heights) != 1 or len(tops) != 1 or len(clearances) != 1:
        raise ValueError(
            "roll_concession_from_cells aggregates over theta only; "
            "height, top length and clearance must be constant."
        )

    feasible = [cell for cell in cells if cell.feasible]
    thetas = sorted(cell.theta_climb_deg for cell in feasible)
    best = min(
        feasible,
        key=lambda cell: (
            cell.hip_z_travel_m if cell.hip_z_travel_m is not None else np.inf,
            -(cell.min_collision_margin_m if cell.min_collision_margin_m is not None else -np.inf),
        ),
        default=None,
    )
    return RollConcession2D(
        feasible=bool(feasible),
        obstacle_height_m=cells[0].obstacle_height_m,
        top_length_m=cells[0].top_length_m,
        approach_clearance_m=cells[0].approach_clearance_m,
        feasible_cell_count=len(feasible),
        evaluated_cell_count=len(cells),
        best_theta_climb_deg=None if best is None else best.theta_climb_deg,
        min_theta_climb_deg=thetas[0] if thetas else None,
        max_theta_climb_deg=thetas[-1] if thetas else None,
        feasible_theta_span_deg=float(thetas[-1] - thetas[0]) if thetas else 0.0,
        feasible_theta_degs=tuple(thetas),
        evaluated_theta_degs=tuple(sorted(cell.theta_climb_deg for cell in cells)),
        hip_z_travel_m=None if best is None else best.hip_z_travel_m,
        hip_rise_above_start_m=None if best is None else best.hip_rise_above_start_m,
        l_transition_m=None if best is None else best.l_transition_m,
        min_collision_margin_m=None if best is None else best.min_collision_margin_m,
        failure_stages=tuple(sorted({
            cell.failure_stage for cell in cells
            if not cell.feasible and cell.failure_stage
        })),
    )


# --------------------------------------------------------------------------
# Comparison
# --------------------------------------------------------------------------


def compare_concessions(
    left,
    right,
    *,
    margin_floor_m: float = 0.0,
    allow_cross_kind: bool = False,
) -> int:
    """Lexicographic ordering: feasibility, then body demand, then margin.

    Returns ``-1`` when ``left`` is preferred, ``1`` when ``right`` is, ``0``
    when they tie.  No weights: the spec's first version fixes the order rather
    than tuning coefficients, so a region boundary is decided by the data and
    not by a knob.

    ``margin_floor_m`` implements §6.2 -- a cell whose clearance is under the
    floor is not treated as feasible, because the clearances in play are
    millimetres and Day 8--9 Step 9 showed they move with sampling density.

    Cross-kind comparison raises unless ``allow_cross_kind`` is set.  Ranking a
    hip *bound* against a hip *trajectory* by magnitude alone presumes the
    answer to the open question in spec §5.3; Step 4 has to supply a rule
    before this becomes meaningful.
    """

    def key(item):
        feasible = bool(item.feasible)
        margin = getattr(item, "min_clearance_m", None)
        if margin is None:
            margin = getattr(item, "min_collision_margin_m", None)
        if feasible and margin is not None and margin < margin_floor_m:
            feasible = False
        deviation = item.body_deviation_m
        return (
            0 if feasible else 1,
            deviation if (feasible and deviation is not None) else np.inf,
            -(margin if (feasible and margin is not None) else -np.inf),
        )

    if not allow_cross_kind:
        kinds = {left.requirement_kind, right.requirement_kind} - {BodyRequirementKind.NONE}
        if len(kinds) > 1:
            raise ValueError(
                "refusing to rank body demands of different shapes "
                f"({sorted(kind.value for kind in kinds)}); "
                "see spec §5.3 -- Step 4 has to measure which is harder first. "
                "Pass allow_cross_kind=True once that rule exists."
            )

    left_key, right_key = key(left), key(right)
    return -1 if left_key < right_key else (1 if left_key > right_key else 0)


def concession_rows(items: Iterable) -> list[dict]:
    return [item.as_dict() for item in items]
