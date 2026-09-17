"""Day 12 Step 11: paper-oriented trajectory metrics.

Plan §18.  These are **mechanism** metrics read off the assembled trajectory --
distances, counts, excursions, margins.  They are not an energy result and
cannot become one here.

Three rules the plan states outright, enforced rather than remembered:

``no energy, no COT``     Plan §18: "不要在沒有 energy experiment 前直接宣稱
                          therefore COT is lower".  Nothing in this module
                          computes work, power or cost of transport, and
                          :func:`energy_vocabulary` lets a test say so.
``body centre != CoM``    Body-centre metrics and whole-robot CoM metrics are
                          separate fields.  The CoM ones are **absent with a
                          reason**, because this pipeline has no mass model --
                          not zero, and not quietly filled with the body's.
``a metric that cannot   Step 5 left almost every body height as ``NaN``.  A
be measured is None``     peak-to-peak of one sample is 0, and a 0 there reads
                          as "the body never moves".  Those metrics come back
                          ``None`` with the count of usable samples beside
                          them.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from hybrid_note.scripts.experiments.day10_11_motion_schema_2d import SegmentKind
from hybrid_note.scripts.experiments.day12_body_trajectory_2d import (
    BODY_BASIS,
    BodyTrajectory2D,
)
from hybrid_note.scripts.experiments.day12_four_leg_state_2d import LEG_ORDER
from hybrid_note.scripts.experiments.day12_support_stability_2d import (
    COM_BASIS,
    TraversalStability2D,
)
from hybrid_note.scripts.experiments.day12_timing_skeleton_2d import LegMode
from hybrid_note.scripts.experiments.day12_transition_mapping_2d import FourLegPlan2D
from hybrid_note.scripts.experiments.day12_whole_body_trajectory_2d import (
    WholeBodyTrajectory2D,
)

#: Why the whole-robot CoM metrics plan §18 asks for are not here.
COM_METRICS_ABSENT = (
    "no whole-robot CoM model exists in this pipeline, so CoM_z peak-to-peak "
    "and RMS are not reported.  The body-centre metrics beside them are a "
    "different quantity and are labelled as such; filling the CoM fields with "
    "them would publish an approximation as a measurement."
)

#: Words that would mean this module had started inferring energy.  Plan §18
#: forbids that until there is an experiment; :func:`energy_vocabulary` makes
#: the prohibition checkable instead of a comment nobody re-reads.
ENERGY_WORDS: tuple[str, ...] = (
    "cost_of_transport", "cost of transport", "cot", "joule", "watt",
    "energy", "power", "work_done", "efficiency",
)


def energy_vocabulary(path: Path | None = None) -> list[str]:
    """Lines in this module that use energy vocabulary.  Should be empty.

    The docstrings that *forbid* energy claims necessarily name them, so the
    scan skips comments and docstrings and looks at code only.
    """

    path = Path(__file__) if path is None else Path(path)
    out: list[str] = []
    in_docstring = False
    in_word_list = False
    for line in path.read_text().splitlines():
        stripped = line.strip()
        # The list of forbidden words necessarily contains them.
        if stripped.startswith("ENERGY_WORDS"):
            in_word_list = True
            continue
        if in_word_list:
            if stripped.startswith(")"):
                in_word_list = False
            continue
        if stripped.count('"""') == 1:
            in_docstring = not in_docstring
            continue
        if in_docstring or stripped.startswith("#") or stripped.startswith('"""'):
            continue
        lowered = stripped.lower()
        for word in ENERGY_WORDS:
            if re.search(rf"(?<![\w]){re.escape(word)}(?![\w])", lowered):
                out.append(stripped)
                break
    return out


@dataclass(frozen=True)
class TrajectoryMetrics2D:
    """Plan §18's list.  Every unmeasurable entry is ``None`` with a reason."""

    terrain: str
    # -- traversal ---------------------------------------------------------
    traversal_distance_m: float | None
    traversal_duration_s: float | None
    # -- swings, counted apart (plan §18's central requirement) -------------
    total_swing_segments: int
    nominal_recovery_swings: int
    terrain_transition_swings: int
    #: Leg-seconds, not wall clock.  See :func:`_mode_time_and_distance`.
    swing_time_s: float
    swing_distance_m: float | None
    # -- what carried the body ---------------------------------------------
    #: Leg-seconds.  These categories overlap and must not be summed.
    foot_rim_roll_time_s: float
    foot_rim_roll_distance_m: float | None
    transition_roll_time_s: float
    transition_roll_distance_m: float | None
    # -- body centre (NOT CoM) ---------------------------------------------
    body_z_peak_to_peak_m: float | None
    body_z_std_m: float | None
    usable_body_samples: int
    total_samples: int
    # -- stability ---------------------------------------------------------
    minimum_stability_margin_m: float | None
    mean_swing_stability_margin_m: float | None
    # -- handoffs ----------------------------------------------------------
    max_hip_lift_m: float | None
    max_joint_discontinuity_rad: float
    max_contact_handoff_gap_m: float
    # -- provenance --------------------------------------------------------
    body_basis: str = BODY_BASIS
    com_basis: str = COM_BASIS
    com_metrics_absent: str = COM_METRICS_ABSENT

    @property
    def com_z_peak_to_peak_m(self) -> None:
        """Deliberately ``None``.  See :data:`COM_METRICS_ABSENT`."""

        return None

    @property
    def com_z_std_m(self) -> None:
        return None

    def as_dict(self) -> dict:
        def mm(value):
            return None if value is None else value * 1e3

        return {
            "terrain": self.terrain,
            "traversal_distance_mm": mm(self.traversal_distance_m),
            "traversal_duration_s": self.traversal_duration_s,
            "total_swing_segments": self.total_swing_segments,
            "nominal_recovery_swings": self.nominal_recovery_swings,
            "terrain_transition_swings": self.terrain_transition_swings,
            "swing_leg_seconds": self.swing_time_s,
            "swing_distance_mm": mm(self.swing_distance_m),
            "foot_rim_roll_leg_seconds": self.foot_rim_roll_time_s,
            "foot_rim_roll_distance_mm": mm(self.foot_rim_roll_distance_m),
            "transition_roll_leg_seconds": self.transition_roll_time_s,
            "transition_roll_distance_mm": mm(self.transition_roll_distance_m),
            "body_z_peak_to_peak_mm": mm(self.body_z_peak_to_peak_m),
            "body_z_std_mm": mm(self.body_z_std_m),
            "usable_body_samples": self.usable_body_samples,
            "total_samples": self.total_samples,
            "com_z_peak_to_peak_mm": self.com_z_peak_to_peak_m,
            "com_z_std_mm": self.com_z_std_m,
            "minimum_stability_margin_mm": mm(self.minimum_stability_margin_m),
            "mean_swing_stability_margin_mm": mm(
                self.mean_swing_stability_margin_m),
            "max_hip_lift_mm": mm(self.max_hip_lift_m),
            "max_joint_discontinuity_deg": float(
                np.rad2deg(self.max_joint_discontinuity_rad)),
            "max_contact_handoff_gap_mm": mm(self.max_contact_handoff_gap_m),
            "body_basis": self.body_basis,
            "com_basis": self.com_basis,
            "com_metrics_absent": self.com_metrics_absent,
        }


def _mode_time_and_distance(result: WholeBodyTrajectory2D, predicate):
    """``(leg_seconds, body_distance)`` for the samples where ``predicate`` holds.

    **Time is leg-seconds**, summed over the four legs -- not wall clock.  An
    "any leg qualifies" wall-clock measure is useless here: at every instant
    some leg is rolling and some leg is airborne, so every such total comes out
    equal to the whole run.  Leg-seconds says what it means and adds up: the
    four legs' totals across all kinds sum to ``4 x duration``.

    **Distance is body advance while at least one stance leg is in that kind**,
    and the categories therefore **overlap** -- two legs can be in different
    kinds at once.  They are attributions, not a partition, and must not be
    summed.  Reported this way rather than split by a rule nobody measured.
    """

    leg_seconds = 0.0
    distance_m = 0.0
    for previous, following in zip(result.samples, result.samples[1:]):
        dt = following.time_s - previous.time_s
        if dt <= 0.0:
            continue
        matching = [s for s in previous.legs.values() if predicate(s)]
        leg_seconds += dt * len(matching)
        if any(s.mode is LegMode.STANCE for s in matching):
            distance_m += abs(following.body_position_world_m[0]
                              - previous.body_position_world_m[0])
    return leg_seconds, distance_m


def trajectory_metrics_2d(
    terrain: str,
    result: WholeBodyTrajectory2D,
    plan: FourLegPlan2D,
    body: BodyTrajectory2D,
    stability: TraversalStability2D,
) -> TrajectoryMetrics2D:
    """Plan §18's metrics, from the assembled trajectory."""

    samples = result.samples
    body_x = np.array([s.body_position_world_m[0] for s in samples], dtype=float)
    body_z = np.array([s.body_position_world_m[2] for s in samples], dtype=float)
    finite = body_z[np.isfinite(body_z)]

    distance = (float(body_x[-1] - body_x[0]) if len(body_x) >= 2 else None)
    duration = (float(samples[-1].time_s - samples[0].time_s)
                if len(samples) >= 2 else None)

    kinds = [p.kind for leg_plan in plan.plans.values() for p in leg_plan.phased]
    swing_time, swing_distance = _mode_time_and_distance(
        result, lambda s: s.mode is LegMode.AIRBORNE)
    roll_time, roll_distance = _mode_time_and_distance(
        result, lambda s: s.segment_kind is SegmentKind.FOOT_RIM_ROLL)
    transition_time, transition_distance = _mode_time_and_distance(
        result,
        lambda s: s.segment_kind.is_rolling and s.segment_kind.is_terrain_transition,
    )

    swing_margins = [sample.margin_m
                     for swing in stability.swings
                     for sample in swing.samples
                     if sample.margin_m is not None]

    hip_lifts = [
        abs(float(segment.end_contact.hip_xz_m[1]
                  - segment.start_contact.hip_xz_m[1]))
        for leg_plan in plan.plans.values()
        for p in leg_plan.phased
        for segment in (p.segment,)
    ]

    # Fewer than two usable heights cannot describe an excursion; reporting 0
    # would read as "the body never moves", which is the opposite.
    measurable = finite.size >= 2
    return TrajectoryMetrics2D(
        terrain=terrain,
        traversal_distance_m=distance,
        traversal_duration_s=duration,
        total_swing_segments=sum(1 for k in kinds if k.is_swing),
        nominal_recovery_swings=sum(
            1 for k in kinds if k is SegmentKind.RECOVERY_SWING),
        terrain_transition_swings=sum(
            1 for k in kinds if k.is_swing and k.is_terrain_transition),
        swing_time_s=swing_time,
        swing_distance_m=swing_distance,
        foot_rim_roll_time_s=roll_time,
        foot_rim_roll_distance_m=roll_distance,
        transition_roll_time_s=transition_time,
        transition_roll_distance_m=transition_distance,
        body_z_peak_to_peak_m=(float(finite.max() - finite.min())
                               if measurable else None),
        body_z_std_m=float(finite.std()) if measurable else None,
        usable_body_samples=int(finite.size),
        total_samples=len(samples),
        minimum_stability_margin_m=stability.minimum_margin_m,
        mean_swing_stability_margin_m=(float(np.mean(swing_margins))
                                       if swing_margins else None),
        max_hip_lift_m=max(hip_lifts) if hip_lifts else None,
        max_joint_discontinuity_rad=result.max_joint_discontinuity_rad,
        max_contact_handoff_gap_m=result.max_contact_gap_m,
    )


def metrics_rows(metrics) -> list[dict]:
    """One row per terrain, plus the two provenance notes as their own rows."""

    rows: list[dict] = [{"row_kind": "metrics", **m.as_dict()} for m in metrics]
    rows.append({"row_kind": "note", "terrain": "-",
                 "com_metrics_absent": COM_METRICS_ABSENT})
    rows.append({"row_kind": "note", "terrain": "-",
                 "body_basis": BODY_BASIS})

    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    return [{key: row.get(key, "") for key in keys} for row in rows]
