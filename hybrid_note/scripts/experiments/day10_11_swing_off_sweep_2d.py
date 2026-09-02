"""Day 10--11 Step 3: the descent swing sweep, and the coupling to how the leg arrived.

Day 8--9 §26.7 puts the roll/swing switch-over on the *descent* side, so this is
the half of the map that decides most cells.  It is the mirror of Step 2 with
one knob swapped and one axis added:

``hip_hold_fraction``
    the body knob.  Climbing asks the hip to rise; descending asks it *not to
    fall* with the foot.  Stated as a fraction of the step height, because that
    is the whole distance the body would otherwise drop.

``touchdown_drop``
    the repair knob, the descent's counterpart to the ascent's ``liftoff_rise``:
    it approaches the touchdown from above instead of along the ground.

``takeoff_distance``
    how far back from the trailing edge the leg stands when it leaves.  This is
    the axis that matters for Step 5, because it is *the same physical quantity
    as the rolling side's top length* -- both sides are spending top run.

**Why the arrival matters, and why it is not a parameter but a second sweep.**
Spec §2.7: the descent's start is whatever the ascent left behind, and the two
ascents leave very different things.

``arrival = SWING_UP``
    a free standing pose anywhere the IK allows, on the foot rim at
    ``alpha = 0`` -- because that is what a standing pose always is.  The
    takeoff distance is then a genuine freedom, so it can be an axis.

``arrival = ROLL_UP``
    the roll-up exit, which Step 0 measured for 20 cells: **every one of them
    is on the right rim**, at ``theta = theta_climb`` and ``beta`` around
    -66 to -72 deg, with the hip only about 30 mm past the leading edge.  The
    takeoff distance is therefore *not* free -- it is set by how long the top
    is -- so for that arrival the sweep axis is ``top_length`` and the takeoff
    distance is derived.

That right-rim start is the interesting part.  Step 2b found that a swing which
must *land* on the left rim is blocked by the ``alpha = -40 deg`` seam between
the foot rim and the upper rims.  A ``ROLL_UP`` descent has the mirror-image
problem: it *starts* on the right rim and has to reach a foot-rim touchdown, so
it crosses the ``+40 deg`` seam instead.  Whether that is equally fatal is a
question this module can answer and Step 2b could not.

Run from the repository root via ``day10_11_step3_driver.py``.
"""

from __future__ import annotations

import os
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
from typing import Iterable, Sequence

import numpy as np

from legwheel.planners.hybrid import HipPose2D, RimId

from .cartesian_swing_contract_2d import (
    HipTrajectory2D,
    SwingFailure,
    build_swing_request_2d,
    swing_start_state_from_scene_2d,
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
    roll_exit_swing_start_2d,
    standing_scene_2d,
)
from .right_up_left_down_full_traversal_2d import STAGE_ROLL_UP
from .single_leg_rolling_scene_2d import build_single_leg_rolling_scene_2d

__all__ = [
    "SwingOffGridSettings2D",
    "SwingOffCell2D",
    "minimum_swing_off_concession_2d",
    "run_swing_off_cells_2d",
    "swing_off_rows",
    "RollUpExitPose2D",
    "roll_up_exit_pose_2d",
    "run_roll_up_exit_poses_2d",
    "minimum_roll_up_descent_2d",
    "run_roll_up_descent_cells_2d",
    "DEFAULT_HEIGHTS_MM",
    "DEFAULT_TAKEOFF_DISTANCES_M",
]

#: Same heights as Step 2, so the two halves of a sequence are priced on one
#: axis.  150 mm stays in for the same reason it did there.
DEFAULT_HEIGHTS_MM: tuple[int, ...] = (20, 40, 60, 80, 100, 120, 140, 150, 160, 180, 200)

#: The rolling side's top-length coordinate, in the descent's own units.
DEFAULT_TAKEOFF_DISTANCES_M: tuple[float, ...] = (
    0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24,
)


@dataclass(frozen=True)
class SwingOffGridSettings2D:
    """Everything held fixed across the descent sweep, in one picklable bundle."""

    top_length_m: float = 0.35
    x_start_m: float = 0.10
    arc_samples: int = 121
    collision_arc_samples: int = 61
    theta_rad: float = float(np.deg2rad(60.0))
    #: Where the hip comes down on the lower ground, past the trailing edge.
    landing_distance_m: float = 0.20
    apex_clearance_m: float = 0.03
    swing_duration_s: float = 0.6
    sample_count: int = 31
    #: Fractions of the step height, matching ``DEFAULT_HIP_HOLD_LADDER``.  The
    #: grid is finer than the showcase's because a grid minimum is only as
    #: sharp as its spacing, and the showcase's 0.25 steps were chosen for a
    #: ladder that stops at the first pass rather than for a map.
    hip_hold_ladder: tuple[float, ...] = (
        0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0,
    )
    #: The descent's repair knob.  Mirrors Step 2's ``liftoff_rise_ladder_m``.
    touchdown_drop_ladder_m: tuple[float, ...] = (0.0, 0.02, 0.05)
    #: **The descent needs a knob the ascent did not.**  Step 2's ascent sweep
    #: never once failed on touchdown velocity -- its 18 infeasible cells are
    #: all ``TERRAIN_COLLISION`` or reach.  The descent's binding constraint
    #: often *is* ``TOUCHDOWN_VELOCITY_TOO_HIGH``, because the foot arrives at
    #: the lower ground with the whole step height behind it, and the knob for
    #: that is time, not geometry.  ``swing_off_step_2d``'s repair already
    #: extends the duration for this reason; a grid without it would report
    #: cells infeasible that are merely hurried.
    duration_scale_ladder: tuple[float, ...] = (1.0, 1.5, 2.0)

    def spec_for(self, height_m: float) -> SharedTerrainSpec2D:
        return SharedTerrainSpec2D(
            height_m=float(height_m),
            top_length_m=self.top_length_m,
            x_start_m=self.x_start_m,
            arc_samples=self.arc_samples,
        )


@dataclass(frozen=True)
class SwingOffCell2D:
    """One ``(height, takeoff distance)`` cell of the descent map."""

    concession: SwingConcession2D
    takeoff_distance_m: float
    landing_distance_m: float
    theta_deg: float
    arrival: str
    top_length_m: float
    takeoff_hip_x_m: float | None
    landing_hip_x_m: float | None
    hip_travel_m: float | None
    #: Which rim the leg leaves on.  A free standing takeoff is always the foot
    #: rim; a roll-up arrival is always the right rim, and that difference is
    #: the whole point of sweeping both.
    takeoff_rim: str | None
    takeoff_alpha_deg: float | None
    evaluations: int
    reached_fit_stage: int
    seconds: float
    #: Evaluations the Day 8--9 planner refused to judge because the leg
    #: finished touching nothing.  Kept apart from ordinary failures: "checked
    #: and rejected" and "could not be checked" are different facts.
    planner_refusals: int = 0
    note: str = "ok"

    def as_dict(self) -> dict:
        row = {
            "height_mm": self.concession.obstacle_height_m * 1e3,
            "arrival": self.arrival,
            "top_length_m": self.top_length_m,
            "takeoff_distance_m": self.takeoff_distance_m,
            "landing_distance_m": self.landing_distance_m,
            "theta_deg": self.theta_deg,
            "takeoff_hip_x_m": self.takeoff_hip_x_m,
            "landing_hip_x_m": self.landing_hip_x_m,
            "hip_travel_m": self.hip_travel_m,
            "takeoff_rim": self.takeoff_rim,
            "takeoff_alpha_deg": self.takeoff_alpha_deg,
        }
        row.update(self.concession.as_dict())
        row["min_hip_hold_mm"] = (
            None if self.concession.min_hip_hold_fraction is None
            else self.concession.min_hip_hold_fraction
            * self.concession.obstacle_height_m * 1e3
        )
        row["evaluations"] = self.evaluations
        row["reached_fit_stage"] = self.reached_fit_stage
        row["planner_refusals"] = self.planner_refusals
        row["seconds"] = round(self.seconds, 2)
        row["note"] = self.note
        return row


def _stance_failure_cell(
    height_m: float, takeoff_distance_m: float, settings: SwingOffGridSettings2D,
    *, error: Exception, started: float, which: str, arrival: str,
    takeoff_hip_x_m: float | None = None,
) -> SwingOffCell2D:
    return SwingOffCell2D(
        concession=SwingConcession2D(
            feasible=False, direction="off", obstacle_height_m=float(height_m),
            source=ConcessionSource.GRID_MINIMUM,
            binding_ceiling=BindingCeiling.STANCE,
            reason=f"the leg cannot stand at the {which} pose: {error}",
        ),
        takeoff_distance_m=float(takeoff_distance_m),
        landing_distance_m=settings.landing_distance_m,
        theta_deg=float(np.rad2deg(settings.theta_rad)),
        arrival=arrival,
        top_length_m=settings.top_length_m,
        takeoff_hip_x_m=takeoff_hip_x_m,
        landing_hip_x_m=None,
        hip_travel_m=None,
        takeoff_rim=None,
        takeoff_alpha_deg=None,
        evaluations=0,
        reached_fit_stage=0,
        seconds=time.time() - started,
        note=f"{which} stance illegal",
    )


def _search_body_and_repairs(
    *, base_request, start_scene, target_hip_xz, height_m: float,
    settings: SwingOffGridSettings2D,
):
    """Walk the three ladders in order and stop at the first *actually valid* plan.

    Same discipline as Step 2: the escalation criterion is ``plan.valid``, not
    a cheap reach probe, so the answer is the lexicographic minimum over the
    grid rather than wherever a greedy search happened to stop.

    The nesting is not arbitrary.  ``hip_hold`` is the outer loop because it is
    the only one of the three that is a *body* demand -- the number Step 5 will
    compare against the rolling side.  ``touchdown_drop`` and the duration are
    trajectory shaping, so they are spent freely to make a given hold work
    rather than being traded against it.
    """

    evaluations = 0
    reached_fit = 0
    refusals = 0
    best = {"failure": None, "clearance": None, "theta_min": None, "reason": None}

    for hold_fraction in settings.hip_hold_ladder:
        held = base_request
        if hold_fraction > 0.0:
            held = replace(
                base_request,
                hip_trajectory=HipTrajectory2D(
                    start_scene.hip_pose,
                    HipPose2D(
                        target_hip_xz + np.array([0.0, hold_fraction * float(height_m)])
                    ),
                ),
            )
        for drop_m in settings.touchdown_drop_ladder_m:
            for scale in settings.duration_scale_ladder:
                request = held
                if scale != 1.0:
                    request = replace(
                        held, swing_duration_s=held.swing_duration_s * float(scale)
                    )
                try:
                    plan = generate_swing_2d(
                        request,
                        arc_samples=settings.collision_arc_samples,
                        touchdown_drop_m=drop_m,
                    )
                except ValueError as error:
                    # Day 8--9's planner cannot express "the leg finished in the
                    # air": Step 7 records ``final_rim = None`` when the final
                    # pose rests on nothing, and Step 8 then refuses to judge
                    # and raises rather than returning an invalid plan.  The
                    # descent's hold ladder reaches that state -- at a large
                    # hold the hip stays up and the leg never reaches the
                    # ground -- so the sweep has to survive it.  Counted, not
                    # swallowed: a cell whose whole grid is refusals is a
                    # different fact from one that was checked and rejected.
                    evaluations += 1
                    refusals += 1
                    if best["failure"] is None and best["clearance"] is None:
                        best["reason"] = (
                            "the planner refused to judge this combination "
                            f"({error}); the leg finishes touching nothing."
                        )
                    continue
                evaluations += 1
                if plan.failure not in _REACH_FAILURES:
                    reached_fit += 1
                if plan.valid:
                    best["refusals"] = refusals
                    return (plan, hold_fraction, drop_m, float(scale),
                            evaluations, reached_fit, best)
                margin = (
                    None if plan.collision is None
                    else plan.collision.minimum_clearance_m
                )
                if best["clearance"] is None or (
                    margin is not None and margin > best["clearance"]
                ):
                    best.update(
                        clearance=margin,
                        failure=plan.failure,
                        theta_min=(
                            None if plan.joint_report is None
                            else plan.joint_report.get("theta_min_deg")
                        ),
                        reason=plan.result.failure_detail,
                    )
    best["refusals"] = refusals
    return None, None, None, None, evaluations, reached_fit, best


def minimum_swing_off_concession_2d(
    height_m: float,
    takeoff_distance_m: float,
    settings: SwingOffGridSettings2D | None = None,
) -> SwingOffCell2D:
    """The least the body must refuse to fall, for a descent from this takeoff.

    ``arrival = SWING_UP``: the takeoff is a free standing pose on the top, so
    the leg leaves on the foot rim at ``alpha = 0``.  That is what a standing
    pose is, and Step 0 confirmed it for all 80 of its cells.
    """

    settings = SwingOffGridSettings2D() if settings is None else settings
    started = time.time()
    spec = settings.spec_for(height_m)
    theta = settings.theta_rad
    takeoff_hip_x = float(spec.x_max_m - float(takeoff_distance_m))

    try:
        start_scene = standing_scene_2d(
            spec, theta, hip_x_m=takeoff_hip_x, support_height_m=spec.top_z_m
        )
        start_state = swing_start_state_from_scene_2d(start_scene)
    except (ValueError, KeyError) as error:
        return _stance_failure_cell(
            height_m, takeoff_distance_m, settings, error=error, started=started,
            which="takeoff", arrival="SWING_UP", takeoff_hip_x_m=takeoff_hip_x,
        )

    landing_hip_x = float(spec.x_max_m + settings.landing_distance_m)
    try:
        target_scene = standing_scene_2d(spec, theta, hip_x_m=landing_hip_x)
        base_request = build_swing_request_2d(
            start_scene, target_scene,
            clearance_m=settings.apex_clearance_m,
            swing_duration_s=settings.swing_duration_s,
            sample_count=settings.sample_count,
        )
    except (ValueError, KeyError) as error:
        return _stance_failure_cell(
            height_m, takeoff_distance_m, settings, error=error, started=started,
            which="landing", arrival="SWING_UP", takeoff_hip_x_m=takeoff_hip_x,
        )

    plan, hold_fraction, drop_m, scale, evaluations, reached_fit, best = _search_body_and_repairs(
        base_request=base_request,
        start_scene=start_scene,
        target_hip_xz=target_scene.hip_pose.position_world_xz_m,
        height_m=height_m,
        settings=settings,
    )

    common = dict(
        takeoff_distance_m=float(takeoff_distance_m),
        landing_distance_m=settings.landing_distance_m,
        theta_deg=float(np.rad2deg(theta)),
        arrival="SWING_UP",
        top_length_m=settings.top_length_m,
        takeoff_hip_x_m=takeoff_hip_x,
        landing_hip_x_m=landing_hip_x,
        hip_travel_m=landing_hip_x - takeoff_hip_x,
        takeoff_rim=str(RimId(start_state.rim).value),
        takeoff_alpha_deg=float(np.rad2deg(start_state.alpha_rad)),
        evaluations=evaluations,
        reached_fit_stage=reached_fit,
        planner_refusals=int(best.get("refusals", 0)),
        seconds=time.time() - started,
    )

    if plan is not None:
        return SwingOffCell2D(
            concession=SwingConcession2D(
                feasible=True, direction="off", obstacle_height_m=float(height_m),
                source=ConcessionSource.GRID_MINIMUM,
                binding_ceiling=BindingCeiling.NONE,
                min_hip_hold_fraction=float(hold_fraction),
                min_touchdown_drop_m=float(drop_m),
                min_liftoff_rise_m=0.0,
                duration_scale=float(scale),
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
            **common,
        )

    ceiling = BindingCeiling.REACH if reached_fit == 0 else BindingCeiling.FIT
    return SwingOffCell2D(
        concession=SwingConcession2D(
            feasible=False, direction="off", obstacle_height_m=float(height_m),
            source=ConcessionSource.GRID_MINIMUM, binding_ceiling=ceiling,
            min_clearance_m=best["clearance"],
            theta_min_deg=best["theta_min"],
            failure=best["failure"],
            reason=best["reason"] or "no (hold, drop) combination produced a valid swing.",
        ),
        **common,
    )


# --------------------------------------------------------------------------
# Batch
# --------------------------------------------------------------------------


def _cell_task(task):
    height_m, takeoff_distance_m, settings = task
    return minimum_swing_off_concession_2d(height_m, takeoff_distance_m, settings)


def run_swing_off_cells_2d(
    tasks: Sequence[tuple],
    *,
    workers: int | None = None,
    label: str = "off",
    progress: bool = True,
) -> list[SwingOffCell2D]:
    """Evaluate explicit ``(height, takeoff distance, settings)`` tasks."""

    workers = max(1, (os.cpu_count() or 2) - 1) if workers is None else int(workers)
    cells: list[SwingOffCell2D] = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for index, cell in enumerate(pool.map(_cell_task, list(tasks)), start=1):
            cells.append(cell)
            if progress:
                hold = cell.concession.min_hip_hold_fraction
                print(
                    f"  {label} [{index}/{len(tasks)}] "
                    f"h={cell.concession.obstacle_height_m * 1e3:>4.0f} mm "
                    f"takeoff={cell.takeoff_distance_m:>5.2f} m "
                    f"theta={cell.theta_deg:>4.0f} -> "
                    f"{'OK  ' if cell.concession.feasible else 'FAIL'} "
                    f"hold={'--' if hold is None else f'{hold:.3f}'} "
                    f"drop={'--' if cell.concession.min_touchdown_drop_m is None else f'{cell.concession.min_touchdown_drop_m * 1e3:.0f}'} mm "
                    f"ceiling={cell.concession.binding_ceiling.value:<8} "
                    f"({cell.evaluations} evals, {cell.seconds:.0f}s)",
                    flush=True,
                )
    return cells


def swing_off_rows(cells: Iterable[SwingOffCell2D]) -> list[dict]:
    return [cell.as_dict() for cell in cells]


# --------------------------------------------------------------------------
# arrival = ROLL_UP: the descent that starts where the climb stopped
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class RollUpExitPose2D:
    """The pose a rolling ascent leaves on the top, as plain numbers.

    Plain numbers rather than a scene because this crosses a process boundary
    and because it is re-planted on obstacles of different top length: the pose
    is set by the front face and the leg, not by how much top follows it.  That
    is an assumption, so :func:`minimum_roll_up_descent_2d` re-runs the contact
    query on every replanted obstacle instead of trusting it.
    """

    height_m: float
    theta_climb_deg: float
    reached: bool
    feasible_traversal: bool
    theta_rad: float | None = None
    beta_rad: float | None = None
    hip_x_m: float | None = None
    hip_z_m: float | None = None
    rim: str | None = None
    alpha_deg: float | None = None
    note: str = "ok"

    def as_dict(self) -> dict:
        return {
            "height_mm": self.height_m * 1e3,
            "theta_climb_deg": self.theta_climb_deg,
            "reached_roll_up_exit": self.reached,
            "feasible_traversal": self.feasible_traversal,
            "exit_theta_deg": (
                None if self.theta_rad is None else float(np.rad2deg(self.theta_rad))
            ),
            "exit_beta_deg": (
                None if self.beta_rad is None else float(np.rad2deg(self.beta_rad))
            ),
            "exit_hip_x_m": self.hip_x_m,
            "exit_hip_z_m": self.hip_z_m,
            "exit_rim": self.rim,
            "exit_alpha_deg": self.alpha_deg,
            "note": self.note,
        }


def _exit_pose_task(task):
    height_m, theta_climb_deg, top_length_m, x_start_m, arc_samples, clearance_m = task
    return roll_up_exit_pose_2d(
        height_m, theta_climb_deg, top_length_m=top_length_m, x_start_m=x_start_m,
        arc_samples=arc_samples, clearance_m=clearance_m,
    )


def roll_up_exit_pose_2d(
    height_m: float,
    theta_climb_deg: float,
    *,
    top_length_m: float = 0.35,
    x_start_m: float = 0.10,
    arc_samples: int = 121,
    clearance_m: float = 0.04,
) -> RollUpExitPose2D:
    """Run one traversal and keep the pose its ``ROLL_UP`` stage ends in.

    Expensive -- a full traversal is a few minutes -- which is why the driver
    runs one per ``(height, theta_climb)`` and reuses it across top lengths.
    """

    spec = SharedTerrainSpec2D(
        height_m=float(height_m), top_length_m=float(top_length_m),
        x_start_m=float(x_start_m), arc_samples=int(arc_samples),
    )
    handoff = roll_exit_swing_start_2d(
        spec, float(np.deg2rad(theta_climb_deg)),
        clearance_m=float(clearance_m), stage=STAGE_ROLL_UP,
    )
    if handoff.scene is None or handoff.start is None:
        return RollUpExitPose2D(
            height_m=float(height_m), theta_climb_deg=float(theta_climb_deg),
            reached=bool(handoff.reached_stage),
            feasible_traversal=bool(handoff.feasible_traversal),
            note=handoff.note,
        )
    hip = np.asarray(handoff.scene.hip_pose.position_world_xz_m, dtype=float)
    return RollUpExitPose2D(
        height_m=float(height_m), theta_climb_deg=float(theta_climb_deg),
        reached=True, feasible_traversal=bool(handoff.feasible_traversal),
        theta_rad=float(handoff.scene.theta_rad),
        beta_rad=float(handoff.scene.beta_rad),
        hip_x_m=float(hip[0]), hip_z_m=float(hip[1]),
        rim=str(RimId(handoff.start.rim).value),
        alpha_deg=float(np.rad2deg(handoff.start.alpha_rad)),
        note=handoff.note,
    )


def run_roll_up_exit_poses_2d(
    tasks: Sequence[tuple], *, workers: int | None = None, progress: bool = True,
) -> list[RollUpExitPose2D]:
    """One traversal per task, in parallel.  Tasks are the tuple above."""

    workers = max(1, (os.cpu_count() or 2) - 1) if workers is None else int(workers)
    poses: list[RollUpExitPose2D] = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for index, pose in enumerate(pool.map(_exit_pose_task, list(tasks)), start=1):
            poses.append(pose)
            if progress:
                print(
                    f"  exit [{index}/{len(tasks)}] "
                    f"h={pose.height_m * 1e3:>4.0f} mm theta_climb={pose.theta_climb_deg:>4.0f} -> "
                    + ("no ROLL_UP exit" if not pose.reached else
                       f"rim={pose.rim} alpha={pose.alpha_deg:>7.1f} "
                       f"theta={np.rad2deg(pose.theta_rad):>5.1f} "
                       f"beta={np.rad2deg(pose.beta_rad):>7.1f} "
                       f"hip=({pose.hip_x_m:.4f}, {pose.hip_z_m:.4f}) "
                       f"traversal_feasible={pose.feasible_traversal}"),
                    flush=True,
                )
    return poses


def minimum_roll_up_descent_2d(
    pose: RollUpExitPose2D,
    top_length_m: float,
    settings: SwingOffGridSettings2D | None = None,
) -> SwingOffCell2D:
    """Price a descent that begins at the roll-up exit, on a top of this length.

    The takeoff distance is **not** a knob here: the exit sits where the front
    face put it, so lengthening the top only moves the trailing edge further
    away.  Sweeping ``top_length_m`` is therefore the same experiment Step 2b's
    section C ran, and it yields the ``ROLL_UP + SWING_DOWN`` top-length budget
    that spec §2.7's second row is missing.
    """

    settings = SwingOffGridSettings2D() if settings is None else settings
    settings = replace(settings, top_length_m=float(top_length_m))
    started = time.time()
    spec = settings.spec_for(pose.height_m)

    common_fail = dict(
        landing_distance_m=settings.landing_distance_m,
        arrival="ROLL_UP",
        top_length_m=float(top_length_m),
        landing_hip_x_m=None,
        hip_travel_m=None,
        evaluations=0,
        reached_fit_stage=0,
    )

    if not pose.reached:
        return SwingOffCell2D(
            concession=SwingConcession2D(
                feasible=False, direction="off",
                obstacle_height_m=float(pose.height_m),
                source=ConcessionSource.GRID_MINIMUM,
                binding_ceiling=BindingCeiling.STANCE,
                reason=f"the ascent never produced a ROLL_UP exit: {pose.note}",
            ),
            takeoff_distance_m=float("nan"),
            theta_deg=float(pose.theta_climb_deg),
            takeoff_hip_x_m=None, takeoff_rim=None, takeoff_alpha_deg=None,
            seconds=time.time() - started, note="no roll-up exit", **common_fail,
        )

    takeoff_distance = float(spec.x_max_m - pose.hip_x_m)
    try:
        start_scene = build_single_leg_rolling_scene_2d(
            float(pose.theta_rad), float(pose.beta_rad),
            float(pose.hip_x_m), float(pose.hip_z_m),
            **spec.rolling_obstacle.scene_kwargs,
        )
        start_state = swing_start_state_from_scene_2d(start_scene)
    except (ValueError, KeyError) as error:
        return SwingOffCell2D(
            concession=SwingConcession2D(
                feasible=False, direction="off",
                obstacle_height_m=float(pose.height_m),
                source=ConcessionSource.GRID_MINIMUM,
                binding_ceiling=BindingCeiling.STANCE,
                reason=(
                    "the roll-up exit pose is not a legal contact on a "
                    f"{top_length_m:.2f} m top: {error}"
                ),
            ),
            takeoff_distance_m=takeoff_distance,
            theta_deg=float(pose.theta_climb_deg),
            takeoff_hip_x_m=pose.hip_x_m, takeoff_rim=None, takeoff_alpha_deg=None,
            seconds=time.time() - started, note="exit pose illegal on this top",
            **common_fail,
        )

    landing_hip_x = float(spec.x_max_m + settings.landing_distance_m)
    try:
        target_scene = standing_scene_2d(spec, settings.theta_rad, hip_x_m=landing_hip_x)
        base_request = build_swing_request_2d(
            start_scene, target_scene,
            clearance_m=settings.apex_clearance_m,
            swing_duration_s=settings.swing_duration_s,
            sample_count=settings.sample_count,
        )
    except (ValueError, KeyError) as error:
        return SwingOffCell2D(
            concession=SwingConcession2D(
                feasible=False, direction="off",
                obstacle_height_m=float(pose.height_m),
                source=ConcessionSource.GRID_MINIMUM,
                binding_ceiling=BindingCeiling.STANCE,
                reason=f"the leg cannot stand at the landing pose: {error}",
            ),
            takeoff_distance_m=takeoff_distance,
            theta_deg=float(pose.theta_climb_deg),
            takeoff_hip_x_m=pose.hip_x_m,
            takeoff_rim=str(RimId(start_state.rim).value),
            takeoff_alpha_deg=float(np.rad2deg(start_state.alpha_rad)),
            seconds=time.time() - started, note="landing stance illegal", **common_fail,
        )

    plan, hold_fraction, drop_m, scale, evaluations, reached_fit, best = _search_body_and_repairs(
        base_request=base_request,
        start_scene=start_scene,
        target_hip_xz=target_scene.hip_pose.position_world_xz_m,
        height_m=pose.height_m,
        settings=settings,
    )

    common = dict(
        takeoff_distance_m=takeoff_distance,
        landing_distance_m=settings.landing_distance_m,
        theta_deg=float(pose.theta_climb_deg),
        arrival="ROLL_UP",
        top_length_m=float(top_length_m),
        takeoff_hip_x_m=float(pose.hip_x_m),
        landing_hip_x_m=landing_hip_x,
        hip_travel_m=landing_hip_x - float(pose.hip_x_m),
        takeoff_rim=str(RimId(start_state.rim).value),
        takeoff_alpha_deg=float(np.rad2deg(start_state.alpha_rad)),
        evaluations=evaluations,
        reached_fit_stage=reached_fit,
        planner_refusals=int(best.get("refusals", 0)),
        seconds=time.time() - started,
    )

    if plan is not None:
        return SwingOffCell2D(
            concession=SwingConcession2D(
                feasible=True, direction="off",
                obstacle_height_m=float(pose.height_m),
                source=ConcessionSource.GRID_MINIMUM,
                binding_ceiling=BindingCeiling.NONE,
                min_hip_hold_fraction=float(hold_fraction),
                min_touchdown_drop_m=float(drop_m),
                min_liftoff_rise_m=0.0,
                duration_scale=float(scale),
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
            **common,
        )

    ceiling = BindingCeiling.REACH if reached_fit == 0 else BindingCeiling.FIT
    return SwingOffCell2D(
        concession=SwingConcession2D(
            feasible=False, direction="off",
            obstacle_height_m=float(pose.height_m),
            source=ConcessionSource.GRID_MINIMUM, binding_ceiling=ceiling,
            min_clearance_m=best["clearance"],
            theta_min_deg=best["theta_min"],
            failure=best["failure"],
            reason=best["reason"] or "no (hold, drop) combination produced a valid swing.",
        ),
        **common,
    )


def _roll_up_cell_task(task):
    pose, top_length_m, settings = task
    return minimum_roll_up_descent_2d(pose, top_length_m, settings)


def run_roll_up_descent_cells_2d(
    tasks: Sequence[tuple], *, workers: int | None = None, progress: bool = True,
) -> list[SwingOffCell2D]:
    """Evaluate explicit ``(RollUpExitPose2D, top_length, settings)`` tasks."""

    workers = max(1, (os.cpu_count() or 2) - 1) if workers is None else int(workers)
    cells: list[SwingOffCell2D] = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for index, cell in enumerate(pool.map(_roll_up_cell_task, list(tasks)), start=1):
            cells.append(cell)
            if progress:
                hold = cell.concession.min_hip_hold_fraction
                print(
                    f"  rollup [{index}/{len(tasks)}] "
                    f"h={cell.concession.obstacle_height_m * 1e3:>4.0f} mm "
                    f"theta_climb={cell.theta_deg:>4.0f} L={cell.top_length_m:>5.2f} m "
                    f"takeoff={cell.takeoff_distance_m:>5.2f} m -> "
                    f"{'OK  ' if cell.concession.feasible else 'FAIL'} "
                    f"hold={'--' if hold is None else f'{hold:.3f}'} "
                    f"rim={cell.takeoff_rim} "
                    f"ceiling={cell.concession.binding_ceiling.value:<8} "
                    f"({cell.evaluations} evals, {cell.seconds:.0f}s)",
                    flush=True,
                )
    return cells
