"""Day 10--11 Step 2: the ascent swing sweep, over the axis Day 8--9 held fixed.

Day 8--9 §26.5(6) found that swing feasibility is decided mostly by *where the
leg starts*, not by how high it lands: the same five cases collide three times
from 0.10 m and pass five times from 0.20 m.  And yet ``swing_onto_step_2d``
holds ``approach_distance_m`` fixed and never searches it; on failure it only
prints ``start_knob_name`` telling the caller to change it.  This module makes
it an axis.

**The effect reproduces; the mechanism §26.5(6) proposed does not.**  That note
attributed it to the wheel radius (0.145 m) exceeding the contact-point-to-step
distance.  Measured here at h = 150 mm, that distance runs from 176 mm at
c = 20 mm to 319 mm at c = 160 mm -- never below 145 mm anywhere on the axis --
and cells still fail at c = 20 mm.  What repairs a tight approach is
``liftoff_rise`` (50 mm at c = 20 mm for every step from 60 mm up, 0 mm from
c = 60 mm on), and that knob raises the leg vertically at the *start* of the
swing.  So the binding problem is the leg body's swept volume while it is still
beside the face, not the static contact geometry.

The axis also **saturates at about 60 mm**: past that, more room buys nothing,
which makes it a threshold condition for Step 5 rather than a continuous cost.

**Why this is not the same search Day 8--9 does.**  The showcase escalates its
hip ladder until a cheap ``_reachable`` probe passes, then repairs -- and the
repair can push the trajectory back out of reach, with no backtracking.  That is
what leaves 150 mm infeasible between a feasible 140 mm and a feasible 160 mm.
Here the escalation criterion is ``plan.valid`` itself, evaluated on the full
``(hip_lift x liftoff_rise)`` grid.  Ascending both ladders and stopping at the
first *actually valid* combination yields the lexicographic minimum, so the
answer is a property of the terrain rather than of the search order.  Cells are
therefore stamped ``ConcessionSource.GRID_MINIMUM`` and may be compared.

Two names that are easy to confuse and are kept apart everywhere here:

``approach_clearance_m``
    the shared Day 10--11 axis -- the gap from the standing leg to the front
    face, in the currency Step 0 fixed.
``apex_clearance_m``
    the swing's own apex margin over the terrain, Day 8--9's ``clearance_m``.
"""

from __future__ import annotations

import os
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
from typing import Iterable, Sequence

import numpy as np

from legwheel.planners.hybrid import HipPose2D

from .cartesian_swing_contract_2d import (
    HipTrajectory2D,
    SwingFailure,
    build_swing_request_2d,
)
from .cartesian_swing_planner_2d import generate_swing_2d
from .day10_11_concession_2d import (
    _REACH_FAILURES,
    BindingCeiling,
    ConcessionSource,
    SwingConcession2D,
)
from .day10_11_shared_scene_2d import (
    SharedTerrainSpec2D,
    approach_hip_x_for_clearance_2d,
    standing_scene_2d,
)

__all__ = [
    "SwingGridSettings2D",
    "SwingGridCell2D",
    "minimum_swing_onto_concession_2d",
    "run_swing_onto_grid_2d",
    "run_swing_cells_2d",
    "swing_grid_rows",
    "DEFAULT_HEIGHTS_MM",
    "DEFAULT_APPROACH_CLEARANCES_M",
]

#: 150 mm is in here on purpose: it is the greedy hole, and Step 2's job is to
#: say whether it is geometry or search order.
DEFAULT_HEIGHTS_MM: tuple[int, ...] = (20, 40, 60, 80, 100, 120, 140, 150, 160, 180, 200)
DEFAULT_APPROACH_CLEARANCES_M: tuple[float, ...] = (
    0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16,
)


@dataclass(frozen=True)
class SwingGridSettings2D:
    """Everything held fixed across the sweep, in one picklable bundle."""

    top_length_m: float = 0.35
    x_start_m: float = 0.10
    #: Rim sampling for the scenes.  121 is the shared convention Step 0 fixed
    #: (it is what the Day 6--7 map was computed with).  Day 8--9's showcase
    #: used 241 for its own scenes, so small numeric differences against §28.1
    #: are expected and intended -- Step 0 deliberately re-based both sides.
    arc_samples: int = 121
    #: Leg sampling for the collision check along the trajectory.
    collision_arc_samples: int = 61
    theta_rad: float = float(np.deg2rad(60.0))
    #: Hip x past the leading edge at touchdown.  Held fixed here; Step 3 turns
    #: the descent's equivalent into an axis, because there it is the same
    #: physical quantity as the rolling side's top length.
    landing_distance_m: float = 0.16
    apex_clearance_m: float = 0.03
    swing_duration_s: float = 0.6
    sample_count: int = 31
    hip_lift_ladder_m: tuple[float, ...] = (0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12)
    liftoff_rise_ladder_m: tuple[float, ...] = (0.0, 0.01, 0.02, 0.03, 0.05)

    def spec_for(self, height_m: float) -> SharedTerrainSpec2D:
        return SharedTerrainSpec2D(
            height_m=float(height_m),
            top_length_m=self.top_length_m,
            x_start_m=self.x_start_m,
            arc_samples=self.arc_samples,
        )


@dataclass(frozen=True)
class SwingGridCell2D:
    """One ``(height, approach clearance)`` cell of the ascent map."""

    concession: SwingConcession2D
    approach_hip_x_m: float | None
    landing_distance_m: float
    theta_deg: float
    #: How many ``generate_swing_2d`` calls this cell cost.
    evaluations: int
    #: How many of them got far enough to be judged on fit rather than reach.
    reached_fit_stage: int
    seconds: float
    note: str = "ok"

    def as_dict(self) -> dict:
        row = {
            "height_mm": self.concession.obstacle_height_m * 1e3,
            "approach_clearance_mm": (
                None if self.concession.approach_clearance_m is None
                else self.concession.approach_clearance_m * 1e3
            ),
            "approach_hip_x_m": self.approach_hip_x_m,
            "landing_distance_m": self.landing_distance_m,
            "theta_deg": self.theta_deg,
        }
        row.update(self.concession.as_dict())
        row["evaluations"] = self.evaluations
        row["reached_fit_stage"] = self.reached_fit_stage
        row["seconds"] = round(self.seconds, 2)
        row["note"] = self.note
        return row


def _stance_failure_cell(
    height_m: float, approach_clearance_m: float, settings: SwingGridSettings2D,
    *, hip_x_m: float | None, error: Exception, started: float, which: str,
) -> SwingGridCell2D:
    """A cell where the leg cannot legally stand at one end, so no swing exists.

    Day 8--9's two-ceiling story does not cover this: it is refused before a
    trajectory is ever built, and the fix is approach or landing distance --
    not hip height and not the control points.
    """

    return SwingGridCell2D(
        concession=SwingConcession2D(
            feasible=False, direction="onto", obstacle_height_m=float(height_m),
            source=ConcessionSource.GRID_MINIMUM,
            binding_ceiling=BindingCeiling.STANCE,
            approach_clearance_m=float(approach_clearance_m),
            reason=f"the leg cannot stand at the {which} pose: {error}",
        ),
        approach_hip_x_m=hip_x_m,
        landing_distance_m=settings.landing_distance_m,
        theta_deg=float(np.rad2deg(settings.theta_rad)),
        evaluations=0,
        reached_fit_stage=0,
        seconds=time.time() - started,
        note=f"{which} stance illegal",
    )


def minimum_swing_onto_concession_2d(
    height_m: float,
    approach_clearance_m: float,
    settings: SwingGridSettings2D | None = None,
) -> SwingGridCell2D:
    """The least the body must give for a swing onto this step from this gap.

    Both ladders are walked in ascending order and the first combination whose
    plan is actually ``valid`` wins, which makes the result the lexicographic
    minimum ``(hip_lift, liftoff_rise)`` over the grid.  Nothing is decided by
    a proxy, so a cell reported infeasible has been refused by every
    combination the grid offers.
    """

    settings = SwingGridSettings2D() if settings is None else settings
    started = time.time()
    spec = settings.spec_for(height_m)
    theta = settings.theta_rad

    hip_x = None
    try:
        hip_x = approach_hip_x_for_clearance_2d(spec, theta, approach_clearance_m)
        start_scene = standing_scene_2d(spec, theta, hip_x_m=hip_x)
    except (ValueError, KeyError) as error:
        return _stance_failure_cell(
            height_m, approach_clearance_m, settings,
            hip_x_m=hip_x, error=error, started=started, which="start",
        )

    try:
        target_scene = standing_scene_2d(
            spec, theta,
            hip_x_m=spec.x_start_m + settings.landing_distance_m,
            support_height_m=spec.top_z_m,
        )
        base_request = build_swing_request_2d(
            start_scene, target_scene,
            clearance_m=settings.apex_clearance_m,
            swing_duration_s=settings.swing_duration_s,
            sample_count=settings.sample_count,
        )
    except (ValueError, KeyError) as error:
        return _stance_failure_cell(
            height_m, approach_clearance_m, settings,
            hip_x_m=hip_x, error=error, started=started, which="landing",
        )

    target_hip_xz = target_scene.hip_pose.position_world_xz_m
    evaluations = 0
    reached_fit = 0
    best_failure: SwingFailure | None = None
    best_clearance: float | None = None
    best_theta_min: float | None = None
    best_reason: str | None = None

    for hip_lift_m in settings.hip_lift_ladder_m:
        request = base_request
        if hip_lift_m > 0.0:
            request = replace(
                base_request,
                hip_trajectory=HipTrajectory2D(
                    start_scene.hip_pose,
                    HipPose2D(target_hip_xz + np.array([0.0, hip_lift_m])),
                ),
            )
        for liftoff_rise_m in settings.liftoff_rise_ladder_m:
            plan = generate_swing_2d(
                request,
                arc_samples=settings.collision_arc_samples,
                liftoff_rise_m=liftoff_rise_m,
            )
            evaluations += 1
            if plan.failure not in _REACH_FAILURES:
                reached_fit += 1
            if plan.valid:
                return SwingGridCell2D(
                    concession=SwingConcession2D(
                        feasible=True, direction="onto",
                        obstacle_height_m=float(height_m),
                        source=ConcessionSource.GRID_MINIMUM,
                        binding_ceiling=BindingCeiling.NONE,
                        approach_clearance_m=float(approach_clearance_m),
                        min_hip_lift_m=float(hip_lift_m),
                        min_liftoff_rise_m=float(liftoff_rise_m),
                        min_touchdown_drop_m=0.0,
                        duration_scale=1.0,
                        min_clearance_m=(
                            None if plan.collision is None
                            else plan.collision.minimum_clearance_m
                        ),
                        theta_min_deg=(
                            None if plan.joint_report is None
                            else plan.joint_report.get("theta_min_deg")
                        ),
                        failure=SwingFailure.NONE,
                    ),
                    approach_hip_x_m=hip_x,
                    landing_distance_m=settings.landing_distance_m,
                    theta_deg=float(np.rad2deg(theta)),
                    evaluations=evaluations,
                    reached_fit_stage=reached_fit,
                    seconds=time.time() - started,
                )
            # Keep the least-bad attempt so an infeasible cell still explains
            # itself with a margin rather than only with a verdict.
            margin = None if plan.collision is None else plan.collision.minimum_clearance_m
            if best_clearance is None or (margin is not None and margin > best_clearance):
                best_clearance = margin
                best_failure = plan.failure
                best_theta_min = (
                    None if plan.joint_report is None
                    else plan.joint_report.get("theta_min_deg")
                )
                best_reason = plan.result.failure_detail

    # Nothing on the grid worked.  If no attempt ever got past reach, the leg
    # never had a trajectory to fit; otherwise it had one and could not clear.
    ceiling = BindingCeiling.REACH if reached_fit == 0 else BindingCeiling.FIT
    return SwingGridCell2D(
        concession=SwingConcession2D(
            feasible=False, direction="onto", obstacle_height_m=float(height_m),
            source=ConcessionSource.GRID_MINIMUM, binding_ceiling=ceiling,
            approach_clearance_m=float(approach_clearance_m),
            min_clearance_m=best_clearance,
            theta_min_deg=best_theta_min,
            failure=best_failure,
            reason=best_reason,
        ),
        approach_hip_x_m=hip_x,
        landing_distance_m=settings.landing_distance_m,
        theta_deg=float(np.rad2deg(theta)),
        evaluations=evaluations,
        reached_fit_stage=reached_fit,
        seconds=time.time() - started,
    )


def _grid_task(task):
    height_m, approach_clearance_m, settings = task
    return minimum_swing_onto_concession_2d(height_m, approach_clearance_m, settings)


def run_swing_onto_grid_2d(
    *,
    heights_mm: Sequence[int] = DEFAULT_HEIGHTS_MM,
    approach_clearances_m: Sequence[float] = DEFAULT_APPROACH_CLEARANCES_M,
    settings: SwingGridSettings2D | None = None,
    workers: int | None = None,
    progress: bool = True,
) -> list[SwingGridCell2D]:
    """Sweep the ascent map.  Cells are independent, so they run in a pool."""

    settings = SwingGridSettings2D() if settings is None else settings
    workers = max(1, (os.cpu_count() or 2) - 1) if workers is None else workers
    tasks = [
        (height_mm / 1000.0, clearance, settings)
        for height_mm in heights_mm
        for clearance in approach_clearances_m
    ]
    cells: list[SwingGridCell2D] = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for index, cell in enumerate(pool.map(_grid_task, tasks), start=1):
            cells.append(cell)
            if progress:
                concession = cell.concession
                body = concession.body_deviation_m
                print(
                    f"  [{index}/{len(tasks)}] "
                    f"h={concession.obstacle_height_m * 1e3:>4.0f} mm "
                    f"c={concession.approach_clearance_m * 1e3:>5.0f} mm -> "
                    f"{'OK  ' if concession.feasible else 'FAIL'} "
                    f"hip={'--' if body is None else f'{body * 1e3:.0f} mm':>6} "
                    f"ceiling={concession.binding_ceiling.value:<8} "
                    f"({cell.evaluations} evals, {cell.seconds:.1f}s)",
                    flush=True,
                )
    return cells


def run_swing_cells_2d(
    tasks: Sequence[tuple[float, float, SwingGridSettings2D]],
    *,
    workers: int | None = None,
    progress: bool = True,
    label: str = "",
) -> list[SwingGridCell2D]:
    """Run an explicit list of ``(height, clearance, settings)`` cells.

    ``run_swing_onto_grid_2d`` sweeps two axes against one fixed ``settings``.
    Closing out Step 2 needs the opposite: a handful of heights against several
    *different* settings, because ``landing_distance_m`` and ``theta_rad`` were
    held fixed for the main map and their influence is unmeasured until
    something varies them.
    """

    workers = max(1, (os.cpu_count() or 2) - 1) if workers is None else workers
    cells: list[SwingGridCell2D] = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for index, cell in enumerate(pool.map(_grid_task, list(tasks)), start=1):
            cells.append(cell)
            if progress:
                concession = cell.concession
                body = concession.body_deviation_m
                print(
                    f"  {label}[{index}/{len(tasks)}] "
                    f"h={concession.obstacle_height_m * 1e3:>4.0f} mm "
                    f"c={concession.approach_clearance_m * 1e3:>4.0f} mm "
                    f"land={cell.landing_distance_m:.2f} m "
                    f"theta={cell.theta_deg:>4.0f} -> "
                    f"{'OK  ' if concession.feasible else 'FAIL'} "
                    f"hip={'--' if body is None else f'{body * 1e3:.0f} mm':>6} "
                    f"ceiling={concession.binding_ceiling.value:<7} "
                    f"({cell.evaluations} evals, {cell.seconds:.0f}s)",
                    flush=True,
                )
    return cells


def swing_grid_rows(cells: Iterable[SwingGridCell2D]) -> list[dict]:
    return [cell.as_dict() for cell in cells]
