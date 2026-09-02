"""Day 10--11 Step 4: the rolling side, in the currency Step 5 has to add up.

Spec sections 5.2 and 5.3.  Two jobs, and the second is the one with research
value:

1.  Fill in ``RollConcession2D`` for every Day 6--7 cell -- taking the best over
    ``theta_climb``, which is rolling's own internal freedom, exactly as the
    swing side is allowed to choose its approach clearance.

2.  Answer the open question 5.3 left: **rolling is not "zero body concession".**
    Its hip height is an *output* of theta and the contact geometry, so what
    rolling asks of the body is a whole trajectory rather than a bound.  Which
    of the two is harder is not knowable a priori, and Step 5 cannot rank a
    ``TRACK`` against a ``LOWER_BOUND`` until this step supplies a rule.

**Nothing here re-runs a traversal.**  ``day6_7_step11r_sweep_trajectories.csv``
already carries ``hip_x_m`` / ``hip_z_m`` for all 70 cells, stage and phase
labelled, with an ``accepted`` flag.  Re-running would cost ~264 s per cell
(implementation log, trap 5) to reproduce numbers that are already on disk.

**The one thing that is derived rather than measured** is the ``L_top``
dependence, and it is licensed by a property of the data rather than assumed:
``WHEEL_MODE_TOP_ROLL`` and ``LEFT_RIM_READY`` hold the hip height constant to
**0.02 mm** over the whole phase, so making the top longer or shorter only adds
or removes flat forward distance.  :meth:`RollTrajectory2D.per_forward_at_top_length`
uses exactly that and refuses to extrapolate below the measured minimum.
"""

from __future__ import annotations

import csv
from collections import defaultdict
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from hybrid_note.scripts.experiments.day10_11_concession_2d import (
    BodyRequirementKind,
    RollCellConcession2D,
    RollConcession2D,
    roll_concession_from_cells,
)
from hybrid_note.scripts.experiments.day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
    standing_scene_2d,
)

#: Standard gravity.  Only ever multiplies a height, so the vertical-work
#: figures are per unit mass -- this step has no mass model and must not
#: pretend to one.
G_M_PER_S2 = 9.80665

#: The top length every Day 10--11 measurement was taken at.  Day 6--7's
#: ``step11r`` sweep used ``obstacle_width_m = 0.35``, and Step 2 / Step 3's
#: grid settings use ``top_length_m = 0.35`` with the same ``x_start_m = 0.10``.
#: So roll and swing are already on the same column and the primary comparison
#: needs no rescaling at all.
REFERENCE_TOP_LENGTH_M = 0.35

#: Day 6--7's ``step11r`` sweep froze these.  Recorded here so the concessions
#: this module builds carry the same approach currency Step 0 aligned.
REFERENCE_APPROACH_CLEARANCE_M = 0.04

#: The phases whose hip height is flat, so a longer top only buys forward
#: distance.  Measured, not assumed: see the module docstring.
FLAT_TOP_PHASES = ("WHEEL_MODE_TOP_ROLL", "LEFT_RIM_READY")


# --------------------------------------------------------------------------
# The excursion metrics
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class HipExcursion2D:
    """How much a hip profile moves the body up and down, four ways.

    The spec (task 4) asks for peak-to-peak, RMS, vertical displacement per
    forward distance, and vertical work per forward distance.  Two of those
    need a definition rather than a formula, so both are fixed here:

    ``hip_z_per_forward_distance``
        Uses the **total vertical path** ``sum |dz|``, not the peak-to-peak.
        A profile that goes up, down and up again moves the body three times;
        peak-to-peak would count it once.  Peak-to-peak is kept alongside as
        its own field, so a reader who wants the envelope still has it.

    ``vertical_work_per_distance``
        Counts only the **rises** (``sum max(dz, 0)``), because a legged robot
        without regeneration does not get the descents back.  Units are
        J / (kg m) -- energy per unit mass per metre travelled, which is the
        vertical component of cost of transport and therefore the field that
        connects to the paper's story.

    Every field is a plain measurement of the samples handed in.  Nothing here
    knows whether the profile came from a rolling traversal or a swing.
    """

    #: ``max - min``: the envelope the body has to fit inside.
    peak_to_peak_m: float
    #: RMS about this profile's own mean, so a profile that sits high but flat
    #: scores low.  This is the field that separates "carried high" from
    #: "moved a lot", which peak-to-peak conflates.
    rms_m: float
    #: ``sum max(dz, 0)`` -- what has to be paid for.
    total_rise_m: float
    #: ``sum max(-dz, 0)`` -- what is given back only with regeneration.
    total_fall_m: float
    #: ``sum |dx|``.  Path length, not net displacement: a profile that
    #: reverses does more work per metre of progress, and should score worse.
    forward_distance_m: float
    #: ``z[-1] - z[0]``.  **Not** zero in general, and that matters: a rolling
    #: traversal ends in wheel mode at ``theta = 17 deg`` while it started at
    #: ``theta_climb``, so it does not return to its own starting posture.
    net_change_m: float
    sample_count: int

    @property
    def total_vertical_path_m(self) -> float:
        return float(self.total_rise_m + self.total_fall_m)

    @property
    def hip_z_per_forward_distance(self) -> float | None:
        """Dimensionless, so cells of different length compare directly."""

        if self.forward_distance_m <= 0.0:
            return None
        return float(self.total_vertical_path_m / self.forward_distance_m)

    @property
    def rise_per_forward_distance(self) -> float | None:
        if self.forward_distance_m <= 0.0:
            return None
        return float(self.total_rise_m / self.forward_distance_m)

    @property
    def vertical_work_per_distance_j_per_kg_m(self) -> float | None:
        """``g * rise / distance``.  The vertical part of cost of transport."""

        per_forward = self.rise_per_forward_distance
        if per_forward is None:
            return None
        return float(G_M_PER_S2 * per_forward)

    @property
    def is_posture_neutral(self) -> bool:
        """Whether the profile ends at the height it started.

        A profile that does not is not wrong, but it cannot be compared with
        one that does without saying so: half of its excursion is a change of
        stance rather than a cost of crossing the obstacle.
        """

        return bool(abs(self.net_change_m) <= 1e-3)

    def as_dict(self, prefix: str = "") -> dict:
        per_forward = self.hip_z_per_forward_distance
        work = self.vertical_work_per_distance_j_per_kg_m
        return {
            f"{prefix}hip_z_peak_to_peak_mm": self.peak_to_peak_m * 1e3,
            f"{prefix}hip_z_rms_mm": self.rms_m * 1e3,
            f"{prefix}hip_z_total_rise_mm": self.total_rise_m * 1e3,
            f"{prefix}hip_z_total_fall_mm": self.total_fall_m * 1e3,
            f"{prefix}hip_z_net_change_mm": self.net_change_m * 1e3,
            f"{prefix}forward_distance_mm": self.forward_distance_m * 1e3,
            f"{prefix}hip_z_per_forward_distance": per_forward,
            f"{prefix}hip_vertical_work_per_dist": work,
            f"{prefix}posture_neutral": self.is_posture_neutral,
            f"{prefix}sample_count": self.sample_count,
        }


def hip_excursion_2d(
    hip_x_m: Sequence[float], hip_z_m: Sequence[float]
) -> HipExcursion2D:
    """Measure one hip profile.  Two samples is enough (a straight line)."""

    x = np.asarray(hip_x_m, dtype=float)
    z = np.asarray(hip_z_m, dtype=float)
    if x.shape != z.shape:
        raise ValueError("hip_x_m and hip_z_m must have the same length.")
    if x.size < 2:
        raise ValueError("a hip profile needs at least two samples.")

    dz = np.diff(z)
    dx = np.diff(x)
    return HipExcursion2D(
        peak_to_peak_m=float(z.max() - z.min()),
        rms_m=float(np.sqrt(np.mean((z - z.mean()) ** 2))),
        total_rise_m=float(np.clip(dz, 0.0, None).sum()),
        total_fall_m=float(np.clip(-dz, 0.0, None).sum()),
        forward_distance_m=float(np.abs(dx).sum()),
        net_change_m=float(z[-1] - z[0]),
        sample_count=int(x.size),
    )


# --------------------------------------------------------------------------
# The rolling side, read back off Day 6--7's trajectories
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class RollTrajectory2D:
    """One Day 6--7 ``(h, theta_climb)`` cell's accepted hip profile."""

    obstacle_height_m: float
    theta_climb_deg: float
    feasible: bool
    hip_x_m: tuple[float, ...]
    hip_z_m: tuple[float, ...]
    stages: tuple[str, ...]
    phases: tuple[str, ...]
    l_transition_m: float | None = None
    min_collision_margin_m: float | None = None
    failure_stage: str | None = None
    failure_reason: str | None = None

    @property
    def excursion(self) -> HipExcursion2D:
        return hip_excursion_2d(self.hip_x_m, self.hip_z_m)

    def excursion_for_stages(self, stages: Iterable[str]) -> HipExcursion2D | None:
        """The profile restricted to some stages, for a matched comparison.

        Whole-traversal numbers are not directly comparable with a single
        swing: the traversal also crosses the top, which the swing side never
        pays for.  ``ROLL_UP`` against ``SWING_UP`` is the matched pair.
        """

        wanted = set(stages)
        index = [i for i, stage in enumerate(self.stages) if stage in wanted]
        if len(index) < 2:
            return None
        return hip_excursion_2d(
            [self.hip_x_m[i] for i in index], [self.hip_z_m[i] for i in index]
        )

    def excursion_for_phases(self, phases: Iterable[str]) -> HipExcursion2D | None:
        wanted = set(phases)
        index = [i for i, phase in enumerate(self.phases) if phase in wanted]
        if len(index) < 2:
            return None
        return hip_excursion_2d(
            [self.hip_x_m[i] for i in index], [self.hip_z_m[i] for i in index]
        )

    @property
    def flat_top_forward_distance_m(self) -> float:
        """How much of the forward travel is spent flat on the top."""

        excursion = self.excursion_for_phases(FLAT_TOP_PHASES)
        return 0.0 if excursion is None else excursion.forward_distance_m

    @property
    def flat_top_hip_z_range_m(self) -> float:
        """Evidence for the extrapolation, carried with it.

        If this is not ~0 the flat-top assumption does not hold for this cell
        and :meth:`per_forward_at_top_length` must not be used on it.
        """

        excursion = self.excursion_for_phases(FLAT_TOP_PHASES)
        return 0.0 if excursion is None else excursion.peak_to_peak_m

    def per_forward_at_top_length(
        self,
        top_length_m: float,
        *,
        minimum_top_length_m: float | None = None,
        flatness_tolerance_m: float = 1e-4,
    ) -> float | None:
        """``hip_z_per_forward_distance`` if the top were a different length.

        Only the flat run changes: it adds no vertical path, so the numerator
        is untouched and the denominator moves by ``top_length - 0.35``.

        Refuses rather than guesses in two cases -- a top shorter than the
        traversal is known to need (``minimum_top_length_m``, from Day 6--7
        Step 12R), and a cell whose flat phases are not actually flat.
        """

        if not self.feasible:
            return None
        if self.flat_top_hip_z_range_m > flatness_tolerance_m:
            return None
        if minimum_top_length_m is not None and top_length_m < minimum_top_length_m:
            return None
        excursion = self.excursion
        distance = excursion.forward_distance_m + (
            float(top_length_m) - REFERENCE_TOP_LENGTH_M
        )
        if distance <= 0.0:
            return None
        return float(excursion.total_vertical_path_m / distance)

    def as_cell_concession(
        self,
        *,
        top_length_m: float = REFERENCE_TOP_LENGTH_M,
        approach_clearance_m: float = REFERENCE_APPROACH_CLEARANCE_M,
    ) -> RollCellConcession2D:
        """Fill in Step 1's contract from the recorded profile."""

        return RollCellConcession2D(
            feasible=self.feasible,
            obstacle_height_m=self.obstacle_height_m,
            top_length_m=float(top_length_m),
            theta_climb_deg=self.theta_climb_deg,
            approach_clearance_m=float(approach_clearance_m),
            hip_z_min_m=float(min(self.hip_z_m)),
            hip_z_max_m=float(max(self.hip_z_m)),
            hip_z_start_m=float(self.hip_z_m[0]),
            l_transition_m=self.l_transition_m,
            min_collision_margin_m=self.min_collision_margin_m,
            frame_count=len(self.hip_x_m),
            phases_visited=tuple(dict.fromkeys(self.phases)),
            failure_stage=self.failure_stage,
            failure_reason=self.failure_reason,
        )


def _float_or_none(text: str) -> float | None:
    text = (text or "").strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def load_roll_trajectories_2d(
    trajectory_csv: Path, sweep_csv: Path
) -> dict[tuple[float, float], RollTrajectory2D]:
    """Read Day 6--7's recorded traversals back in, keyed by ``(h, theta)``.

    Only ``accepted`` frames are kept, which is the same filter
    ``roll_cell_concession_from_result`` applies to a live result -- a rejected
    frame is a pose the traversal proposed and then refused, so pricing the
    body for it would charge for a motion that never happens.
    """

    summary: dict[tuple[float, float], dict] = {}
    with Path(sweep_csv).open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            key = (
                round(float(row["obstacle_height_m"]), 6),
                round(float(row["theta_climb_deg"]), 3),
            )
            summary[key] = row

    frames: dict[tuple[float, float], list[dict]] = defaultdict(list)
    with Path(trajectory_csv).open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row["accepted"] != "True":
                continue
            key = (
                round(float(row["obstacle_height_m"]), 6),
                round(float(row["theta_climb_deg"]), 3),
            )
            frames[key].append(row)

    out: dict[tuple[float, float], RollTrajectory2D] = {}
    for key, rows in frames.items():
        rows.sort(key=lambda row: int(row["index"]))
        meta = summary.get(key, {})
        out[key] = RollTrajectory2D(
            obstacle_height_m=key[0],
            theta_climb_deg=key[1],
            feasible=meta.get("feasible") == "True",
            hip_x_m=tuple(float(row["hip_x_m"]) for row in rows),
            hip_z_m=tuple(float(row["hip_z_m"]) for row in rows),
            stages=tuple(row["stage"] for row in rows),
            phases=tuple(row["phase"] for row in rows),
            l_transition_m=_float_or_none(meta.get("L_transition_m", "")),
            min_collision_margin_m=_float_or_none(
                meta.get("minimum_collision_margin_m", "")
            ),
            failure_stage=(meta.get("failure_stage") or None),
            failure_reason=(meta.get("failure_reason") or None),
        )
    return out


def roll_concessions_by_height_2d(
    trajectories: dict[tuple[float, float], RollTrajectory2D],
    *,
    top_length_m: float = REFERENCE_TOP_LENGTH_M,
    approach_clearance_m: float = REFERENCE_APPROACH_CLEARANCE_M,
) -> dict[float, RollConcession2D]:
    """Take the best over ``theta_climb`` for each height, via Step 1's rule."""

    grouped: dict[float, list[RollCellConcession2D]] = defaultdict(list)
    for (height_m, _theta), trajectory in sorted(trajectories.items()):
        grouped[height_m].append(
            trajectory.as_cell_concession(
                top_length_m=top_length_m, approach_clearance_m=approach_clearance_m
            )
        )
    return {
        height: roll_concession_from_cells(cells)
        for height, cells in sorted(grouped.items())
    }


# --------------------------------------------------------------------------
# The swing side, in the same currency
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SwingHipProfile2D:
    """A swing's hip profile, rebuilt from the knob the sweep reported.

    ``HipTrajectory2D`` is a straight line between two hip poses (Day 8--9's
    first version), so the profile is fully determined by its endpoints and
    the body demand is applied to the far one: ``min_hip_lift`` raises the
    landing hip, ``min_hip_hold_fraction`` refuses part of the drop.  This is
    a reconstruction, not a re-plan, and it is exact for that model.
    """

    direction: str
    obstacle_height_m: float
    theta_deg: float
    start_hip_xz_m: tuple[float, float]
    end_hip_xz_m: tuple[float, float]
    knob_m: float

    @property
    def excursion(self) -> HipExcursion2D:
        return hip_excursion_2d(
            [self.start_hip_xz_m[0], self.end_hip_xz_m[0]],
            [self.start_hip_xz_m[1], self.end_hip_xz_m[1]],
        )


def _spec_for(height_m: float) -> SharedTerrainSpec2D:
    return SharedTerrainSpec2D(
        height_m=float(height_m),
        top_length_m=REFERENCE_TOP_LENGTH_M,
        x_start_m=0.10,
        arc_samples=121,
    )


def _standing_hip_xz(
    height_m: float, theta_deg: float, hip_x_m: float, support_height_m: float
) -> tuple[float, float]:
    scene = standing_scene_2d(
        _spec_for(height_m),
        float(np.deg2rad(theta_deg)),
        hip_x_m=float(hip_x_m),
        support_height_m=float(support_height_m),
    )
    position = scene.hip_pose.position_world_xz_m
    return (float(position[0]), float(position[1]))


def swing_onto_hip_profile_2d(
    *,
    height_m: float,
    theta_deg: float,
    approach_hip_x_m: float,
    landing_distance_m: float,
    min_hip_lift_m: float,
) -> SwingHipProfile2D:
    """SWING_UP: lower ground -> the top, with the lift on the landing hip."""

    spec = _spec_for(height_m)
    start = _standing_hip_xz(height_m, theta_deg, approach_hip_x_m, 0.0)
    end = _standing_hip_xz(
        height_m, theta_deg, spec.x_start_m + landing_distance_m, spec.top_z_m
    )
    return SwingHipProfile2D(
        direction="onto",
        obstacle_height_m=float(height_m),
        theta_deg=float(theta_deg),
        start_hip_xz_m=start,
        end_hip_xz_m=(end[0], end[1] + float(min_hip_lift_m)),
        knob_m=float(min_hip_lift_m),
    )


def swing_off_hip_profile_2d(
    *,
    height_m: float,
    theta_deg: float,
    takeoff_hip_x_m: float,
    landing_hip_x_m: float,
    min_hip_hold_fraction: float,
) -> SwingHipProfile2D:
    """SWING_DOWN: the top -> lower ground, holding part of the drop."""

    spec = _spec_for(height_m)
    start = _standing_hip_xz(height_m, theta_deg, takeoff_hip_x_m, spec.top_z_m)
    end = _standing_hip_xz(height_m, theta_deg, landing_hip_x_m, 0.0)
    hold_m = float(min_hip_hold_fraction) * float(height_m)
    return SwingHipProfile2D(
        direction="off",
        obstacle_height_m=float(height_m),
        theta_deg=float(theta_deg),
        start_hip_xz_m=start,
        end_hip_xz_m=(end[0], end[1] + hold_m),
        knob_m=hold_m,
    )


def swing_obstacle_hip_profile_2d(
    up: SwingHipProfile2D, down: SwingHipProfile2D
) -> HipExcursion2D:
    """One whole obstacle by swing: up, across the top, down.

    The rolling traversal crosses the top as part of itself, so a comparison
    against a single swing would charge rolling for forward distance the swing
    never travels.  The segment between the two swings is the leg standing on
    the top and moving from where it landed to where it takes off -- flat, so
    it adds forward distance and no vertical path, which is precisely the
    ``WHEEL_MODE_TOP_ROLL`` phase's role on the rolling side.

    The two swings must be at the same height, and the crossing is only
    well-posed when the takeoff is behind the landing.
    """

    if abs(up.obstacle_height_m - down.obstacle_height_m) > 1e-12:
        raise ValueError("the two swings must cross the same obstacle.")
    landing_x, landing_z = up.end_hip_xz_m
    takeoff_x, takeoff_z = down.start_hip_xz_m
    if takeoff_x < landing_x:
        raise ValueError(
            "the takeoff hip is behind the landing hip: there is no top to cross."
        )
    return hip_excursion_2d(
        [up.start_hip_xz_m[0], landing_x, takeoff_x, down.end_hip_xz_m[0]],
        [up.start_hip_xz_m[1], landing_z, takeoff_z, down.end_hip_xz_m[1]],
    )


# --------------------------------------------------------------------------
# The rule spec 5.3 asks Step 4 to supply
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class BodyDemandComparison2D:
    """One matched roll-vs-swing comparison, with its own caveats attached."""

    label: str
    obstacle_height_m: float
    roll_excursion: HipExcursion2D
    swing_excursion: HipExcursion2D
    roll_theta_climb_deg: float | None = None
    swing_theta_deg: float | None = None

    @property
    def roll_is_cheaper(self) -> bool | None:
        left = self.roll_excursion.hip_z_per_forward_distance
        right = self.swing_excursion.hip_z_per_forward_distance
        if left is None or right is None:
            return None
        return bool(left < right)

    @property
    def ratio(self) -> float | None:
        """``roll / swing`` in the dimensionless currency.  < 1 favours roll."""

        left = self.roll_excursion.hip_z_per_forward_distance
        right = self.swing_excursion.hip_z_per_forward_distance
        if left is None or right is None or right == 0.0:
            return None
        return float(left / right)

    @property
    def posture_caveat(self) -> str | None:
        """Whether the two sides return to the posture they started in.

        Day 6--7's traversal ends in wheel mode at ``theta = 17 deg`` after
        starting at ``theta_climb``, so part of its excursion is a change of
        stance rather than a cost of crossing.  The swing sweeps start and end
        at the same theta.  Comparing them without saying this would charge
        rolling for something the comparison is not about.
        """

        problems = []
        if not self.roll_excursion.is_posture_neutral:
            problems.append(
                f"roll ends {self.roll_excursion.net_change_m * 1e3:+.1f} mm "
                "from where it started"
            )
        if not self.swing_excursion.is_posture_neutral:
            problems.append(
                f"swing ends {self.swing_excursion.net_change_m * 1e3:+.1f} mm "
                "from where it started"
            )
        return "; ".join(problems) if problems else None

    def as_dict(self) -> dict:
        row = {
            "comparison": self.label,
            "obstacle_mm": self.obstacle_height_m * 1e3,
            "roll_theta_climb_deg": self.roll_theta_climb_deg,
            "swing_theta_deg": self.swing_theta_deg,
            "roll_is_cheaper": self.roll_is_cheaper,
            "ratio_roll_over_swing": self.ratio,
            "posture_caveat": self.posture_caveat,
        }
        row.update(self.roll_excursion.as_dict("roll_"))
        row.update(self.swing_excursion.as_dict("swing_"))
        return row


def compare_body_demand_2d(
    roll: RollConcession2D,
    swing_body_demand_m: float | None,
    *,
    roll_excursion: HipExcursion2D,
) -> tuple[int | None, str]:
    """Rank a ``TRACK`` against a ``LOWER_BOUND``.  Spec 5.3's missing rule.

    ``compare_concessions`` refuses cross-kind comparison because ranking a
    hip *bound* against a hip *trajectory* by magnitude presumes the answer.
    The rule this step supplies is:

    ``a LOWER_BOUND of L is realised by a family of body trajectories, and the
    cheapest member of that family has vertical excursion exactly L.``

    So the two are comparable **at their own minima**: rolling's excursion is
    prescribed and therefore already minimal, and the swing's minimum is its
    bound.  Both answer "the least this primitive can ask of the body".

    What the magnitude does **not** capture, and callers must not forget, is
    that rolling additionally fixes the *shape* at every instant while the
    swing only fixes an extremum.  That freedom is real and unpriced here, so
    a tie in magnitude is a win for the swing.  The return value says so.

    Returns ``(-1 | 0 | 1 | None, reason)``: ``-1`` prefers roll, ``1`` prefers
    swing, ``0`` a genuine tie in both magnitude and freedom (impossible while
    rolling is ``TRACK``), and ``None`` when there is nothing to compare.
    """

    if roll.requirement_kind is not BodyRequirementKind.TRACK:
        return None, "the rolling cell is not feasible, so it demands nothing."
    if swing_body_demand_m is None:
        return None, "the swing side has no feasible plan at this cell."

    roll_m = roll_excursion.peak_to_peak_m
    swing_m = float(swing_body_demand_m)
    if roll_m < swing_m:
        return -1, (
            f"roll moves the hip {roll_m * 1e3:.1f} mm against the swing's "
            f"minimum of {swing_m * 1e3:.1f} mm."
        )
    if roll_m > swing_m:
        return 1, (
            f"roll moves the hip {roll_m * 1e3:.1f} mm, more than the swing's "
            f"minimum of {swing_m * 1e3:.1f} mm -- and the swing is free to "
            "meet its bound with any shape, which rolling is not."
        )
    return 1, (
        "the magnitudes tie, which the swing wins: it fixes one extremum "
        "while rolling prescribes the whole profile."
    )
