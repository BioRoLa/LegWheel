"""Day 6--7 Step 12R: TOP_LENGTH_AND_TRANSITION_DISTANCE.

What this measures
------------------
After the right-rim roll-up has already succeeded, how much *obstacle-top
forward distance* does the leg still need in order to

    roll-up exit state -> retract theta to 17 deg -> wheel-mode forward roll
    -> LEFT_RIM_READY

That distance is ``L_transition``.  It is a property of the leg and the
retract/roll schedule, not of the obstacle: the transition trajectory does not
know where the trailing corner is, the corner only decides where the motion is
cut off.  So one run per (height, theta) measures it, and the top length that a
*continuous* traversal needs follows from it.

Three different questions, deliberately kept apart
--------------------------------------------------
A. **Local upward-edge roll-up feasibility.**  Can the leg get onto the top at
   all?  This is decided at the leading edge, by theta_climb and the obstacle
   height.  It does *not* depend on the obstacle top length, and this module
   demonstrates that rather than assuming it.

B. **Local downward-edge roll-down feasibility.**  Can the leg get off the
   trailing edge?  Decided at the trailing edge, by the rim budget still left
   when it arrives.

C. **Full rectangular-obstacle traversal transition-length requirement.**  Is
   the top long enough to fit the leading-edge margin plus ``L_transition``
   before the trailing corner arrives?

C is *not* a precondition for A.  An obstacle whose top is too short for C can
still be climbed (A) -- the leg simply cannot complete this particular
right-up / left-down traversal on it, because there is no room to hand the
contact over to the left rim before the trailing edge.  Reading C as "the
obstacle must be at least this long before roll-up may start" inverts the
result.

Not included, deliberately: no foot-rim ``L_reset`` criterion (that belonged to
the superseded reset branch), no Step 6.75 fallback, no swing, and no planner
motion selection.  Every number here comes from the Step 10R traversal.
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
    PHASE_LEFT_RIM_READY,
    PHASE_RETRACT_TO_WHEEL,
    PHASE_WHEEL_MODE_TOP_ROLL,
    ObstacleSpec2D,
    RollingTraversalResult2D,
    check_right_up_left_down_traversal,
    traversal_frame_rows,
)
from hybrid_note.scripts.experiments.right_up_left_down_sweep_2d import (  # noqa: E402
    SweepSettings2D,
    _failure_label,
)

__all__ = [
    "TransitionMeasurement2D",
    "measure_transition_distances",
    "measure_transition_from_rows",
    "TopLengthCase2D",
    "run_top_length_sweep_2d",
    "minimum_top_length_for_full_traversal",
    "transition_measurement_rows",
    "write_transition_length_csv",
    "plot_transition_budget_2d",
    "plot_top_length_sweep_2d",
]


@dataclass(frozen=True)
class TransitionMeasurement2D:
    """Where the obstacle top is spent, for one (height, theta_climb) case.

    All distances are contact-point travel along the obstacle top, measured in
    world x from the leading edge.  ``None`` anywhere means the run never got
    that far, which is reported rather than filled in.
    """

    obstacle_height_m: float
    theta_climb_deg: float
    obstacle_x_start_m: float
    obstacle_top_length_m: float

    # Where the roll-up handed over, measured from the leading edge.  This is
    # top length the transition can no longer use.
    leading_edge_margin_m: float | None
    # Contact travel while theta falls 60 -> 17 deg, still on the right rim.
    retract_forward_distance_m: float | None
    # Contact travel at theta = 17 deg up to the right-to-left handover.
    wheel_mode_forward_distance_m: float | None
    # The two above, together: roll-up exit -> LEFT_RIM_READY.
    l_transition_m: float | None
    # Top still unused between LEFT_RIM_READY and the trailing corner.
    trailing_edge_entry_margin_m: float | None
    # leading_edge_margin + L_transition: the shortest top on which the
    # handover can still finish before the trailing corner.
    required_top_length_m: float | None

    reached_left_rim_ready: bool
    full_traversal_success: bool
    failure_stage: str | None
    failure_reason: str | None

    @property
    def spare_top_length_m(self) -> float | None:
        """How much longer the top is than the handover strictly needs."""

        if self.required_top_length_m is None:
            return None
        return float(self.obstacle_top_length_m - self.required_top_length_m)

    def as_row(self) -> dict:
        return {
            "obstacle_height_m": self.obstacle_height_m,
            "theta_climb_deg": self.theta_climb_deg,
            "obstacle_top_length_m": self.obstacle_top_length_m,
            "leading_edge_margin_m": self.leading_edge_margin_m,
            "retract_forward_distance_m": self.retract_forward_distance_m,
            "wheel_mode_forward_distance_m": self.wheel_mode_forward_distance_m,
            "L_transition_m": self.l_transition_m,
            "trailing_edge_entry_margin_m": self.trailing_edge_entry_margin_m,
            "required_top_length_m": self.required_top_length_m,
            "spare_top_length_m": self.spare_top_length_m,
            "reached_left_rim_ready": self.reached_left_rim_ready,
            "full_traversal_success": self.full_traversal_success,
            "failure_stage": self.failure_stage,
            "failure_reason": self.failure_reason,
        }


def _first_contact_x(rows, phase: str) -> float | None:
    for row in rows:
        if row["phase"] == phase and row["contact_x_m"] is not None:
            return float(row["contact_x_m"])
    return None


def measure_transition_from_rows(
    rows,
    *,
    obstacle_height_m: float,
    theta_climb_deg: float,
    obstacle_x_start_m: float,
    obstacle_top_length_m: float,
    full_traversal_success: bool,
    failure_stage: str | None = None,
    failure_reason: str | None = None,
) -> TransitionMeasurement2D:
    """Measure the transition from a Step 10R trajectory in flat-row form.

    Phase boundaries carry the milestones, so the distances are read off the
    contact x at the first frame of each phase rather than recomputed:
    ``RETRACT_TO_WHEEL`` starts at the roll-up exit, ``WHEEL_MODE_TOP_ROLL``
    starts where theta first reaches its target, and ``LEFT_RIM_READY`` starts
    at the right-to-left handover.
    """

    rows = list(rows)
    x_start = float(obstacle_x_start_m)
    x_exit = _first_contact_x(rows, PHASE_RETRACT_TO_WHEEL)
    x_wheel = _first_contact_x(rows, PHASE_WHEEL_MODE_TOP_ROLL)
    x_ready = _first_contact_x(rows, PHASE_LEFT_RIM_READY)

    leading = None if x_exit is None else x_exit - x_start
    retract = None if (x_exit is None or x_wheel is None) else x_wheel - x_exit
    wheel = None if (x_wheel is None or x_ready is None) else x_ready - x_wheel
    transition = None if (x_exit is None or x_ready is None) else x_ready - x_exit
    trailing = (
        None if x_ready is None
        else (x_start + obstacle_top_length_m) - x_ready
    )
    required = None if x_ready is None else x_ready - x_start
    return TransitionMeasurement2D(
        obstacle_height_m=float(obstacle_height_m),
        theta_climb_deg=float(theta_climb_deg),
        obstacle_x_start_m=x_start,
        obstacle_top_length_m=float(obstacle_top_length_m),
        leading_edge_margin_m=leading,
        retract_forward_distance_m=retract,
        wheel_mode_forward_distance_m=wheel,
        l_transition_m=transition,
        trailing_edge_entry_margin_m=trailing,
        required_top_length_m=required,
        reached_left_rim_ready=x_ready is not None,
        full_traversal_success=bool(full_traversal_success),
        failure_stage=failure_stage,
        failure_reason=failure_reason,
    )


def measure_transition_distances(
    result: RollingTraversalResult2D,
    theta_climb_deg: float | None = None,
) -> TransitionMeasurement2D:
    """Measure the transition of one Step 10R traversal result."""

    if not isinstance(result, RollingTraversalResult2D):
        raise TypeError("result must be a RollingTraversalResult2D.")
    obstacle = result.obstacle
    measurement = measure_transition_from_rows(
        traversal_frame_rows(result),
        obstacle_height_m=obstacle.height_m,
        theta_climb_deg=(
            float(np.rad2deg(result.theta_climb_rad))
            if theta_climb_deg is None else theta_climb_deg
        ),
        obstacle_x_start_m=obstacle.x_start_m,
        obstacle_top_length_m=obstacle.width_m,
        full_traversal_success=result.full_success,
        failure_stage=_failure_label(result),
        failure_reason=result.failure_reason,
    )
    # Cross-check against the transition stage's own bookkeeping: the phase
    # boundaries and the handover frame must agree about L_transition.
    if (
        result.l_transition_m is not None
        and measurement.l_transition_m is not None
        and not np.isclose(
            result.l_transition_m, measurement.l_transition_m, atol=5e-3
        )
    ):
        raise AssertionError(
            "phase-boundary L_transition "
            f"{measurement.l_transition_m:.6f} disagrees with the handover "
            f"frame's {result.l_transition_m:.6f}."
        )
    return measurement


# --------------------------------------------------------------------------
# Top-length sweep: "is there enough transition space", not "may we start"
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class TopLengthCase2D:
    """One (height, theta_climb) case and the top lengths tried on it."""

    obstacle_height_m: float
    theta_climb_rad: float
    rows: tuple[dict, ...]
    measurement: TransitionMeasurement2D | None

    @property
    def theta_climb_deg(self) -> float:
        return float(np.rad2deg(self.theta_climb_rad))

    @property
    def successful_top_lengths_m(self) -> tuple[float, ...]:
        return tuple(
            row["obstacle_top_length_m"]
            for row in self.rows if row["full_traversal_success"]
        )

    @property
    def minimum_top_length_m(self) -> float | None:
        lengths = self.successful_top_lengths_m
        return min(lengths) if lengths else None

    @property
    def maximum_top_length_m(self) -> float | None:
        lengths = self.successful_top_lengths_m
        return max(lengths) if lengths else None


def _run_top_length_case(task):
    """Worker: one full Step 10R traversal at one obstacle top length."""

    height_m, theta_rad, top_length_m, settings = task
    obstacle = ObstacleSpec2D(
        x_start_m=settings.obstacle_x_start_m,
        width_m=float(top_length_m),
        height_m=float(height_m),
        gamma_rad=settings.gamma_rad,
        obstacle_id=settings.obstacle_id,
        arc_samples=settings.arc_samples,
    )
    result = check_right_up_left_down_traversal(
        obstacle=obstacle,
        initial_state=settings.initial_state_for(height_m, theta_rad),
        theta_climb=theta_rad,
        constraints=settings.constraints,
    )
    measurement = measure_transition_distances(
        result, theta_climb_deg=float(np.rad2deg(theta_rad))
    )
    row = {
        "obstacle_height_m": float(height_m),
        "theta_climb_deg": float(np.rad2deg(theta_rad)),
        "obstacle_top_length_m": float(top_length_m),
        "L_transition_m": measurement.l_transition_m,
        "required_top_length_m": measurement.required_top_length_m,
        "leading_edge_margin_m": measurement.leading_edge_margin_m,
        "retract_forward_distance_m": measurement.retract_forward_distance_m,
        "wheel_mode_forward_distance_m": measurement.wheel_mode_forward_distance_m,
        "trailing_edge_entry_margin_m": measurement.trailing_edge_entry_margin_m,
        "reached_left_rim_ready": measurement.reached_left_rim_ready,
        # The requirement C question: was the top long enough to finish?
        "full_traversal_success": bool(result.full_success),
        # The requirement A question, answered separately on the same run.
        "roll_up_success": bool(result.roll_up_success),
        "failure_stage": _failure_label(result),
        "failure_reason": result.failure_reason,
        "minimum_collision_margin_m": result.minimum_collision_margin_m,
    }
    return row


def run_top_length_sweep_2d(
    cases,
    top_lengths_m,
    *,
    settings: SweepSettings2D | None = None,
    max_workers: int | None = None,
    progress: bool = True,
) -> list[TopLengthCase2D]:
    """Run the full traversal at several top lengths, for several cases.

    ``cases`` is a sequence of ``(obstacle_height_m, theta_climb_rad)``.
    ``top_lengths_m`` may be one sequence shared by every case, or a mapping
    from case to its own sequence -- useful because each case's interesting
    range sits around its own ``required_top_length_m``.
    """

    import os
    from concurrent.futures import ProcessPoolExecutor

    settings = SweepSettings2D() if settings is None else settings
    cases = [(float(height), float(theta)) for height, theta in cases]
    if not cases:
        raise ValueError("at least one case is required.")
    if isinstance(top_lengths_m, dict):
        lengths_by_case = {case: tuple(float(v) for v in top_lengths_m[case])
                           for case in cases}
    else:
        shared = tuple(float(value) for value in top_lengths_m)
        lengths_by_case = {case: shared for case in cases}
    if any(not lengths for lengths in lengths_by_case.values()):
        raise ValueError("every case needs at least one top length.")
    if any(value <= 0.0 for lengths in lengths_by_case.values() for value in lengths):
        raise ValueError("top lengths must be positive.")

    tasks = [
        (height, theta, length, settings)
        for (height, theta) in cases
        for length in lengths_by_case[(height, theta)]
    ]
    if max_workers is None:
        max_workers = max(1, min(8, (os.cpu_count() or 2) // 2))

    if max_workers == 1:
        results = [_run_top_length_case(task) for task in tasks]
    else:
        with ProcessPoolExecutor(max_workers=max_workers) as pool:
            results = list(pool.map(_run_top_length_case, tasks))
    if progress:
        for row in results:
            verdict = (
                "FULL" if row["full_traversal_success"]
                else (row["failure_stage"] or "FAILED")
            )
            print(
                f"h={row['obstacle_height_m']:.3f} "
                f"theta={row['theta_climb_deg']:5.1f} "
                f"top={row['obstacle_top_length_m']:.3f}  {verdict:26s} "
                f"L_transition={row['L_transition_m']}",
                flush=True,
            )

    by_case: list[TopLengthCase2D] = []
    for height, theta in cases:
        rows = [
            row for row in results
            if np.isclose(row["obstacle_height_m"], height)
            and np.isclose(row["theta_climb_deg"], float(np.rad2deg(theta)))
        ]
        rows.sort(key=lambda row: row["obstacle_top_length_m"])
        measured = [row for row in rows if row["reached_left_rim_ready"]]
        measurement = None
        if measured:
            source = measured[0]
            measurement = TransitionMeasurement2D(
                obstacle_height_m=height,
                theta_climb_deg=float(np.rad2deg(theta)),
                obstacle_x_start_m=settings.obstacle_x_start_m,
                obstacle_top_length_m=source["obstacle_top_length_m"],
                leading_edge_margin_m=source["leading_edge_margin_m"],
                retract_forward_distance_m=source["retract_forward_distance_m"],
                wheel_mode_forward_distance_m=source["wheel_mode_forward_distance_m"],
                l_transition_m=source["L_transition_m"],
                trailing_edge_entry_margin_m=source["trailing_edge_entry_margin_m"],
                required_top_length_m=source["required_top_length_m"],
                reached_left_rim_ready=True,
                full_traversal_success=source["full_traversal_success"],
                failure_stage=source["failure_stage"],
                failure_reason=source["failure_reason"],
            )
        by_case.append(
            TopLengthCase2D(height, theta, tuple(rows), measurement)
        )
    return by_case


def minimum_top_length_for_full_traversal(cases) -> dict:
    """The demo answer: the shortest top on which a full traversal completed.

    Reported per case and overall.  The overall figure is the *largest* of the
    per-case minima, because a single demo obstacle has to satisfy every case
    it is meant to show -- taking the smallest would name a length that most
    of the cases cannot actually finish on.
    """

    per_case = []
    for case in cases:
        per_case.append(
            {
                "obstacle_height_m": case.obstacle_height_m,
                "theta_climb_deg": case.theta_climb_deg,
                "required_top_length_m": (
                    None if case.measurement is None
                    else case.measurement.required_top_length_m
                ),
                "minimum_feasible_top_length_m": case.minimum_top_length_m,
                "maximum_feasible_top_length_m": case.maximum_top_length_m,
            }
        )
    minima = [
        row["minimum_feasible_top_length_m"] for row in per_case
        if row["minimum_feasible_top_length_m"] is not None
    ]
    return {
        "per_case": per_case,
        "minimum_top_length_for_full_right_up_left_down_demo": (
            max(minima) if minima else None
        ),
        "smallest_single_case_minimum_m": min(minima) if minima else None,
        "cases_with_no_feasible_top_length": [
            (row["obstacle_height_m"], row["theta_climb_deg"])
            for row in per_case
            if row["minimum_feasible_top_length_m"] is None
        ],
    }


def transition_measurement_rows(measurements) -> list[dict]:
    return [item.as_row() for item in measurements]


def write_transition_length_csv(
    cases,
    sweep_path,
    *,
    measurement_path=None,
    minimum_path=None,
) -> list[Path]:
    """Write the top-length sweep, the per-case measurements and the minima."""

    written = []

    def dump(path, payload):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(payload[0]))
            writer.writeheader()
            writer.writerows(payload)
        written.append(path)

    dump(sweep_path, [row for case in cases for row in case.rows])
    if measurement_path is not None:
        rows = [case.measurement.as_row() for case in cases if case.measurement]
        if rows:
            dump(measurement_path, rows)
    if minimum_path is not None:
        dump(minimum_path, minimum_top_length_for_full_traversal(cases)["per_case"])
    return written


# --------------------------------------------------------------------------
# Figures
# --------------------------------------------------------------------------


def plot_transition_budget_2d(measurements, *, ax=None):
    """Stacked view of where each case spends the obstacle top.

    Left to right: the leading-edge margin the roll-up already used, then the
    retract, then the wheel-mode roll up to LEFT_RIM_READY.  Everything past
    the bar is top the traversal did not need.
    """

    measurements = [item for item in measurements if item.reached_left_rim_ready]
    if not measurements:
        raise ValueError("no measurement reached LEFT_RIM_READY.")
    if ax is None:
        _, ax = plt.subplots(figsize=(9.6, 0.62 * len(measurements) + 2.6))
    labels = [
        f"h={item.obstacle_height_m:.2f}, theta={item.theta_climb_deg:.0f}"
        for item in measurements
    ]
    positions = np.arange(len(measurements))
    segments = (
        ("leading-edge margin (roll-up)", "leading_edge_margin_m", "#94a3b8"),
        ("retract to 17 deg", "retract_forward_distance_m", "#c026d3"),
        ("wheel-mode roll to LEFT_RIM_READY", "wheel_mode_forward_distance_m", "#db2777"),
    )
    left = np.zeros(len(measurements))
    for label, field, colour in segments:
        widths = np.array([getattr(item, field) or 0.0 for item in measurements])
        ax.barh(positions, widths, left=left, color=colour, label=label,
                height=0.62, edgecolor="white")
        left = left + widths
    for position, item in zip(positions, measurements):
        ax.plot([item.obstacle_top_length_m], [position], "|", color="#111827",
                markersize=18, markeredgewidth=2.2)
        ax.text(item.obstacle_top_length_m + 0.006, position,
                f"top={item.obstacle_top_length_m:.2f}", va="center", fontsize=8)
    ax.plot([], [], "|", color="#111827", markersize=12, markeredgewidth=2.2,
            label="trailing corner (obstacle top length)")
    ax.set_yticks(positions)
    ax.set_yticklabels(labels, fontsize=8.5)
    ax.set_xlabel("distance along the obstacle top from the leading edge [m]")
    ax.set_title(
        "Step 12R: where the obstacle top is spent\n"
        "(bar end = LEFT_RIM_READY, i.e. the shortest top that still works)"
    )
    # Below the axes: the bars run the full width, so any in-axes corner
    # would sit on top of one of them.
    ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.16),
              ncol=2, frameon=False)
    ax.grid(True, axis="x", alpha=0.3)
    ax.set_ylim(-0.7, len(measurements) - 0.3)
    ax.figure.tight_layout()
    return ax


def plot_top_length_sweep_2d(cases, *, ax=None):
    """Full-traversal outcome against obstacle top length, per case.

    The vertical marker is the measured ``required_top_length``: the point
    where LEFT_RIM_READY lands.  Lengths to its left cannot finish the
    handover before the trailing corner, however healthy the roll-up was.
    """

    cases = list(cases)
    if not cases:
        raise ValueError("no cases to plot.")
    if ax is None:
        _, ax = plt.subplots(figsize=(10.0, 0.62 * len(cases) + 2.8))
    for position, case in enumerate(cases):
        for row in case.rows:
            ok = row["full_traversal_success"]
            ax.plot([row["obstacle_top_length_m"]], [position],
                    "o" if ok else "x",
                    color="#15803d" if ok else "#dc2626",
                    markersize=7 if ok else 6.5, markeredgewidth=1.9)
        if case.measurement and case.measurement.required_top_length_m:
            ax.axvline(case.measurement.required_top_length_m, ymin=0, ymax=0,
                       color="#2563eb")
            ax.plot([case.measurement.required_top_length_m], [position], "|",
                    color="#2563eb", markersize=20, markeredgewidth=2.4)
    ax.plot([], [], "o", color="#15803d", label="full traversal completed")
    ax.plot([], [], "x", color="#dc2626", label="did not complete")
    ax.plot([], [], "|", color="#2563eb", markersize=12, markeredgewidth=2.4,
            label="required_top_length (LEFT_RIM_READY)")
    ax.set_yticks(range(len(cases)))
    ax.set_yticklabels(
        [f"h={case.obstacle_height_m:.2f}, theta={case.theta_climb_deg:.0f}"
         for case in cases], fontsize=8.5
    )
    ax.set_xlabel("obstacle top length [m]")
    ax.set_title(
        "Step 12R: does the top provide enough transition space?\n"
        "(this is requirement C -- it does not gate the roll-up, requirement A)"
    )
    # Below the axes: the longest-top data points reach the lower-right corner.
    ax.legend(fontsize=8.5, loc="upper center", bbox_to_anchor=(0.5, -0.16),
              ncol=3, frameon=False)
    ax.set_ylim(-0.6, len(cases) - 0.4)
    ax.grid(True, axis="x", alpha=0.3)
    ax.figure.tight_layout()
    return ax
