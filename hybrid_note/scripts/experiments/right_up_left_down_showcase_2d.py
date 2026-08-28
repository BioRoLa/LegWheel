"""Day 6--7 final showcase: pick parameters, get advice, run the traversal.

This is the demonstration front end for the whole Day 6--7 rolling line.  It
adds no physics: the traversal is Step 10R's
``check_right_up_left_down_traversal`` unchanged.  What it adds is the thing a
demo actually needs -- telling you whether the parameters you chose are inside
what Steps 11R and 12R already measured, *before* you spend three minutes
finding out that they are not.

The advice is read from the persisted sweep results rather than recomputed, so
it always reflects the runs that were actually made:

* which ``theta_climb`` values completed a full traversal at a given obstacle
  height  (Step 11R);
* how much obstacle top the transition needs at a given ``theta_climb``
  (Step 12R).

Advice is never a veto.  A combination outside the measured grid is reported as
untested, not refused -- the traversal still runs and answers for itself.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from hybrid_note.scripts.experiments.right_up_left_down_full_traversal_2d import (  # noqa: E402
    ObstacleSpec2D,
    RollingTraversalResult2D,
    animate_full_traversal_2d,
    check_right_up_left_down_traversal,
    plot_traversal_key_frames_2d,
)
from hybrid_note.scripts.experiments.right_up_left_down_sweep_2d import (  # noqa: E402
    SweepSettings2D,
)
from hybrid_note.scripts.experiments.right_up_left_down_transition_length_2d import (  # noqa: E402
    measure_transition_distances,
)

__all__ = [
    "Day67Evidence2D",
    "ShowcaseAdvice2D",
    "ShowcaseResult2D",
    "advise_showcase_parameters",
    "run_showcase_traversal_2d",
    "showcase_summary_lines",
    "plot_day6_7_evidence_panel_2d",
    "showcase_animation",
    "showcase_key_frames",
    "EVIDENCE_ARC_SAMPLES",
    "SWEEP_CSV_NAME",
    "TRANSITION_CSV_NAME",
]

SWEEP_CSV_NAME = "day6_7_step11r_feasibility_sweep.csv"
TRANSITION_CSV_NAME = "day6_7_step12r_transition_measurements_all_cells.csv"

# Verified once in section 0 of the dashboard; quoted here so the summary can
# explain *why* tall obstacles fail without re-deriving it.
WHEEL_RADIUS_AT_17_DEG_M = 0.1450

# Every case in Step 12R needed exactly this much top beyond the point where
# LEFT_RIM_READY lands, to still complete.
MEASURED_TOP_LENGTH_MARGIN_M = 0.010

# Every Step 11R / 12R number was measured at this rim sampling.  Below it the
# APPROACH can stop short of the front face for reasons that are sampling, not
# geometry (see the Step 11R section on the h = 0.04 APPROACH_FAIL cells), so
# the evidence stops being a reliable predictor.
EVIDENCE_ARC_SAMPLES = 121


def _as_float(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _clean_deg(value) -> float:
    """Degrees as written, not as a float round trip left them.

    Angles reach the CSVs through ``deg2rad`` and back, so 60 deg can arrive as
    59.99999999999999.  Rounding here keeps table lookups and membership tests
    from silently missing.
    """

    return float(round(float(value), 6))


def _as_bool(value) -> bool:
    return str(value).strip().lower() == "true"


@dataclass(frozen=True)
class Day67Evidence2D:
    """What Steps 11R and 12R measured, loaded from their CSV output."""

    feasibility_rows: tuple[dict, ...]
    transition_rows: tuple[dict, ...]

    @classmethod
    def load(cls, day6_7_dir) -> "Day67Evidence2D":
        directory = Path(day6_7_dir)

        def read(name):
            path = directory / name
            if not path.exists():
                raise FileNotFoundError(
                    f"{path} is missing; run the Step 11R / 12R sections first."
                )
            with path.open(newline="", encoding="utf-8") as handle:
                return tuple(dict(row) for row in csv.DictReader(handle))

        return cls(read(SWEEP_CSV_NAME), read(TRANSITION_CSV_NAME))

    @property
    def swept_heights_m(self) -> tuple[float, ...]:
        return tuple(sorted({
            float(row["obstacle_height_m"]) for row in self.feasibility_rows
        }))

    @property
    def swept_theta_deg(self) -> tuple[float, ...]:
        return tuple(sorted({
            _clean_deg(row["theta_climb_deg"]) for row in self.feasibility_rows
        }))

    def feasible_theta_deg(self, obstacle_height_m: float) -> tuple[float, ...]:
        """theta_climb values that completed a full traversal at this height."""

        return tuple(sorted(
            _clean_deg(row["theta_climb_deg"])
            for row in self.feasibility_rows
            if _as_bool(row["feasible"])
            and np.isclose(float(row["obstacle_height_m"]), obstacle_height_m)
        ))

    def outcome_at(self, obstacle_height_m: float, theta_climb_deg: float):
        """``(feasible, failure_stage, failure_reason)`` or ``None`` if untested."""

        for row in self.feasibility_rows:
            if np.isclose(float(row["obstacle_height_m"]), obstacle_height_m) and (
                np.isclose(float(row["theta_climb_deg"]), theta_climb_deg)
            ):
                return (
                    _as_bool(row["feasible"]),
                    row.get("failure_stage") or None,
                    row.get("failure_reason") or None,
                )
        return None

    def required_top_length_by_theta(self) -> dict[float, float]:
        """Mean measured ``required_top_length`` per theta, feasible cases only."""

        buckets: dict[float, list[float]] = {}
        for row in self.transition_rows:
            if not _as_bool(row.get("full_traversal_success")):
                continue
            value = _as_float(row.get("required_top_length_m"))
            if value is None:
                continue
            buckets.setdefault(_clean_deg(row["theta_climb_deg"]), []).append(value)
        return {
            theta: float(np.mean(values)) for theta, values in sorted(buckets.items())
        }

    def required_top_length_m(self, theta_climb_deg: float) -> float | None:
        """Interpolate the measured requirement; refuse to extrapolate.

        ``required_top_length`` was measured to be a function of
        ``theta_climb`` alone -- it came out identical across six obstacle
        heights -- so one curve covers every height.  Outside the measured
        theta range this returns ``None`` rather than a guess.
        """

        table = self.required_top_length_by_theta()
        if not table:
            return None
        thetas = np.array(sorted(table))
        values = np.array([table[theta] for theta in thetas])
        if theta_climb_deg < thetas[0] or theta_climb_deg > thetas[-1]:
            return None
        return float(np.interp(theta_climb_deg, thetas, values))

    def suggested_top_length_m(self, theta_climb_deg: float) -> float | None:
        required = self.required_top_length_m(theta_climb_deg)
        return None if required is None else required + MEASURED_TOP_LENGTH_MARGIN_M


@dataclass(frozen=True)
class ShowcaseAdvice2D:
    """Whether the chosen parameters sit inside what was already measured."""

    obstacle_height_m: float
    theta_climb_deg: float
    obstacle_top_length_m: float
    required_top_length_m: float | None
    suggested_top_length_m: float | None
    feasible_theta_at_this_height_deg: tuple[float, ...]
    previously_measured_outcome: tuple | None
    warnings: tuple[str, ...]

    @property
    def top_length_is_long_enough(self) -> bool | None:
        if self.required_top_length_m is None:
            return None
        return bool(self.obstacle_top_length_m >= self.required_top_length_m)

    @property
    def looks_promising(self) -> bool:
        """No warning fired -- not a guarantee, just nothing known to be wrong."""

        return not self.warnings

    def report(self) -> str:
        lines = [
            f"obstacle height     : {self.obstacle_height_m:.3f} m",
            f"theta_climb         : {self.theta_climb_deg:.1f} deg",
            f"obstacle top length : {self.obstacle_top_length_m:.3f} m",
        ]
        if self.required_top_length_m is not None:
            lines.append(
                f"required top length : {self.required_top_length_m:.3f} m "
                f"(measured, Step 12R)"
            )
            lines.append(
                f"suggested top length: {self.suggested_top_length_m:.3f} m "
                f"(= required + {MEASURED_TOP_LENGTH_MARGIN_M * 1e3:.0f} mm)"
            )
        if self.feasible_theta_at_this_height_deg:
            values = ", ".join(
                f"{value:.0f}" for value in self.feasible_theta_at_this_height_deg
            )
            lines.append(f"theta feasible here : {values} deg  (Step 11R)")
        if self.previously_measured_outcome is not None:
            feasible, stage, reason = self.previously_measured_outcome
            lines.append(
                "this exact cell     : "
                + ("completed in Step 11R" if feasible
                   else f"failed in Step 11R ({stage}: {reason})")
            )
        lines.extend(f"WARNING: {item}" for item in self.warnings)
        if not self.warnings:
            lines.append("no warnings -- inside the measured envelope")
        return "\n".join(lines)


def advise_showcase_parameters(
    evidence: Day67Evidence2D,
    *,
    obstacle_height_m: float,
    theta_climb_deg: float,
    obstacle_top_length_m: float,
    arc_samples: int | None = None,
) -> ShowcaseAdvice2D:
    """Check a parameter choice against the Step 11R / 12R evidence.

    Warnings describe what is *known*; none of them stop the run.  A parameter
    set that was never swept is reported as untested rather than as bad.
    """

    if not isinstance(evidence, Day67Evidence2D):
        raise TypeError("evidence must be a Day67Evidence2D.")
    required = evidence.required_top_length_m(theta_climb_deg)
    suggested = evidence.suggested_top_length_m(theta_climb_deg)
    feasible_theta = evidence.feasible_theta_deg(obstacle_height_m)
    outcome = evidence.outcome_at(obstacle_height_m, theta_climb_deg)

    warnings: list[str] = []
    if required is not None and obstacle_top_length_m < required:
        warnings.append(
            f"the top is shorter than the measured requirement "
            f"({obstacle_top_length_m:.3f} < {required:.3f} m): the left rim "
            f"cannot take over before the trailing corner.  The roll-up will "
            f"still succeed -- this blocks the handover, not the climb."
        )
    elif suggested is not None and obstacle_top_length_m < suggested:
        warnings.append(
            f"the top clears the requirement but not the {MEASURED_TOP_LENGTH_MARGIN_M * 1e3:.0f} mm "
            f"margin every measured case needed ({obstacle_top_length_m:.3f} < "
            f"{suggested:.3f} m); the corner arrival may be too tight."
        )
    if required is None:
        warnings.append(
            f"theta_climb {theta_climb_deg:.1f} deg is outside the measured "
            f"range {min(evidence.required_top_length_by_theta() or [0]):.0f}-"
            f"{max(evidence.required_top_length_by_theta() or [0]):.0f} deg, so "
            f"no top-length requirement is known for it."
        )
    if obstacle_height_m not in evidence.swept_heights_m:
        warnings.append(
            f"obstacle height {obstacle_height_m:.3f} m was not swept in Step 11R."
        )
    elif not feasible_theta:
        warnings.append(
            f"no theta_climb completed a full traversal at height "
            f"{obstacle_height_m:.3f} m in Step 11R."
        )
    elif outcome is not None and not any(
        np.isclose(theta_climb_deg, value) for value in feasible_theta
    ):
        warnings.append(
            f"theta_climb {theta_climb_deg:.1f} deg did not complete at this "
            f"height in Step 11R; feasible values were "
            f"{', '.join(f'{v:.0f}' for v in feasible_theta)} deg."
        )
    if arc_samples is not None and arc_samples < EVIDENCE_ARC_SAMPLES:
        warnings.append(
            f"arc_samples={arc_samples} is below the {EVIDENCE_ARC_SAMPLES} "
            f"every measured result used.  The advice above may not hold: on a "
            f"coarse rim the APPROACH can stop short of the front face, which "
            f"is a sampling artefact rather than an infeasible traversal."
        )
    if obstacle_height_m > WHEEL_RADIUS_AT_17_DEG_M:
        warnings.append(
            f"the obstacle is taller than the theta=17 deg wheel radius "
            f"({WHEEL_RADIUS_AT_17_DEG_M:.4f} m); the corner pivot needs more "
            f"than 90 deg of left rim and is unlikely to have the budget."
        )
    return ShowcaseAdvice2D(
        obstacle_height_m=float(obstacle_height_m),
        theta_climb_deg=float(theta_climb_deg),
        obstacle_top_length_m=float(obstacle_top_length_m),
        required_top_length_m=required,
        suggested_top_length_m=suggested,
        feasible_theta_at_this_height_deg=feasible_theta,
        previously_measured_outcome=outcome,
        warnings=tuple(warnings),
    )


@dataclass(frozen=True)
class ShowcaseResult2D:
    """One showcase run: the advice, the traversal, and its measurements."""

    advice: ShowcaseAdvice2D
    traversal: RollingTraversalResult2D
    measurement: object
    settings: SweepSettings2D

    @property
    def success(self) -> bool:
        return bool(self.traversal.full_success)


def run_showcase_traversal_2d(
    evidence: Day67Evidence2D | None = None,
    *,
    obstacle_height_m: float = 0.10,
    theta_climb_deg: float = 60.0,
    obstacle_top_length_m: float | None = None,
    arc_samples: int = 121,
    obstacle_x_start_m: float = 0.10,
    approach_start_clearance_m: float = 0.04,
    release_theta: bool = False,
    ground_roll_distance_m: float = 0.02,
    verbose: bool = True,
) -> ShowcaseResult2D:
    """Run the full Day 6--7 traversal for one parameter choice.

    ``obstacle_top_length_m = None`` asks for the length Step 12R measured as
    sufficient for this ``theta_climb``, which is the sensible default for a
    demonstration: the shortest obstacle the traversal is known to clear.
    """

    if evidence is not None and not isinstance(evidence, Day67Evidence2D):
        raise TypeError("evidence must be a Day67Evidence2D or None.")
    theta_climb_rad = float(np.deg2rad(theta_climb_deg))

    if obstacle_top_length_m is None:
        suggested = (
            None if evidence is None
            else evidence.suggested_top_length_m(theta_climb_deg)
        )
        if suggested is None:
            raise ValueError(
                "obstacle_top_length_m must be given when the measured "
                "requirement for this theta_climb is unknown."
            )
        obstacle_top_length_m = suggested

    settings = SweepSettings2D(
        obstacle_x_start_m=obstacle_x_start_m,
        obstacle_width_m=float(obstacle_top_length_m),
        arc_samples=arc_samples,
        approach_start_clearance_m=approach_start_clearance_m,
        release_theta=release_theta,
        ground_roll_distance_m=ground_roll_distance_m,
    )
    advice = (
        None if evidence is None
        else advise_showcase_parameters(
            evidence,
            obstacle_height_m=obstacle_height_m,
            theta_climb_deg=theta_climb_deg,
            obstacle_top_length_m=obstacle_top_length_m,
            arc_samples=arc_samples,
        )
    )
    if verbose and advice is not None:
        print(advice.report(), flush=True)
        print("-" * 68, flush=True)

    obstacle = ObstacleSpec2D(
        x_start_m=obstacle_x_start_m,
        width_m=float(obstacle_top_length_m),
        height_m=float(obstacle_height_m),
        arc_samples=arc_samples,
    )
    traversal = check_right_up_left_down_traversal(
        obstacle=obstacle,
        initial_state=settings.initial_state_for(obstacle_height_m, theta_climb_rad),
        theta_climb=theta_climb_rad,
        constraints=settings.constraints,
    )
    measurement = measure_transition_distances(traversal, theta_climb_deg)
    result = ShowcaseResult2D(advice, traversal, measurement, settings)
    if verbose:
        print("\n".join(showcase_summary_lines(result)), flush=True)
    return result


def showcase_summary_lines(result: ShowcaseResult2D) -> list[str]:
    """A compact plain-text verdict for one showcase run."""

    if not isinstance(result, ShowcaseResult2D):
        raise TypeError("result must be a ShowcaseResult2D.")
    traversal = result.traversal
    measurement = result.measurement
    lines = [
        ("FULL TRAVERSAL COMPLETED" if result.success
         else f"DID NOT COMPLETE  ({traversal.failure_stage}: "
              f"{traversal.failure_reason})"),
        f"  frames                     : {len(traversal.trajectory)}",
        f"  phases                     : {' -> '.join(traversal.phases_visited)}",
    ]
    if measurement.l_transition_m is not None:
        lines += [
            f"  L_transition               : {measurement.l_transition_m:.4f} m",
            f"    retract to 17 deg        : "
            f"{measurement.retract_forward_distance_m:.4f} m",
            f"    wheel-mode roll          : "
            f"{measurement.wheel_mode_forward_distance_m:.4f} m",
            f"  leading-edge margin        : "
            f"{measurement.leading_edge_margin_m:.4f} m",
            f"  trailing-edge entry margin : "
            f"{measurement.trailing_edge_entry_margin_m:.4f} m",
        ]
    if traversal.minimum_collision_margin_m is not None:
        lines.append(
            f"  minimum collision margin   : "
            f"{traversal.minimum_collision_margin_m * 1e3:.2f} mm"
        )
    descent = traversal.descent_result
    if descent is not None and descent.pivot_rotation_rad is not None:
        lines.append(
            f"  corner pivot rotation      : "
            f"{np.rad2deg(descent.pivot_rotation_rad):.1f} deg"
        )
    final = traversal.final_state
    if final is not None and final.contact_point_world_xz_m is not None:
        lines.append(
            f"  final contact              : x={final.contact_point_world_xz_m[0]:.4f} m "
            f"on {final.terrain_surface_id} ({final.active_rim})"
        )
    return lines


def plot_day6_7_evidence_panel_2d(evidence: Day67Evidence2D, *, axes=None):
    """One figure summarising what Day 6--7 established.

    Left: which theta completed at each height (Step 11R).  Right: how much
    obstacle top the transition needs, against theta (Step 12R).
    """

    if not isinstance(evidence, Day67Evidence2D):
        raise TypeError("evidence must be a Day67Evidence2D.")
    if axes is None:
        _, axes = plt.subplots(1, 2, figsize=(13.4, 4.8))
    left, right = axes

    heights = evidence.swept_heights_m
    thetas = evidence.swept_theta_deg
    for height in heights:
        feasible = set(evidence.feasible_theta_deg(height))
        for theta in thetas:
            ok = theta in feasible
            left.plot([theta], [height], "o" if ok else "x",
                      color="#15803d" if ok else "#dc2626",
                      markersize=7 if ok else 6, markeredgewidth=1.8)
    left.plot([], [], "o", color="#15803d", label="full traversal completed")
    left.plot([], [], "x", color="#dc2626", label="did not complete")
    left.axhline(WHEEL_RADIUS_AT_17_DEG_M, color="#2563eb", linestyle="--",
                 linewidth=1.5,
                 label=f"wheel radius at 17 deg = {WHEEL_RADIUS_AT_17_DEG_M:.4f} m")
    left.set_xlabel("theta_climb [deg]")
    left.set_ylabel("obstacle height [m]")
    left.set_title("Step 11R: full-traversal feasibility")
    left.grid(True, alpha=0.3)
    left.legend(fontsize=8, loc="upper left")

    table = evidence.required_top_length_by_theta()
    if table:
        theta_values = np.array(sorted(table))
        required = np.array([table[value] for value in theta_values])
        right.plot(theta_values, required, "o-", color="#2563eb", linewidth=2.0,
                   label="required top length (LEFT_RIM_READY lands here)")
        right.plot(theta_values, required + MEASURED_TOP_LENGTH_MARGIN_M, "s--",
                   color="#16a34a", linewidth=1.7, markersize=5,
                   label=f"minimum that completed "
                         f"(+{MEASURED_TOP_LENGTH_MARGIN_M * 1e3:.0f} mm)")
        right.fill_between(theta_values, required,
                           required + MEASURED_TOP_LENGTH_MARGIN_M,
                           color="#16a34a", alpha=0.15)
    right.set_xlabel("theta_climb [deg]")
    right.set_ylabel("obstacle top length [m]")
    right.set_title(
        "Step 12R: obstacle top the transition needs\n"
        "(a shorter top blocks the handover, not the roll-up)"
    )
    right.grid(True, alpha=0.3)
    right.legend(fontsize=8)
    left.figure.tight_layout()
    return axes


def showcase_animation(result: ShowcaseResult2D, **kwargs):
    """Animate a showcase run with the Step 10R animator."""

    if not isinstance(result, ShowcaseResult2D):
        raise TypeError("result must be a ShowcaseResult2D.")
    return animate_full_traversal_2d(result.traversal, **kwargs)


def showcase_key_frames(result: ShowcaseResult2D, **kwargs):
    """Key-frame figure for a showcase run."""

    if not isinstance(result, ShowcaseResult2D):
        raise TypeError("result must be a ShowcaseResult2D.")
    return plot_traversal_key_frames_2d(result.traversal, **kwargs)
