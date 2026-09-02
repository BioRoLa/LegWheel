"""Day 10--11 Step 0: one scene convention shared by the rolling and swing worlds.

Day 6--7 and Day 8--9 each froze the approach in a different currency, and the
two are not interchangeable:

``right_up_left_down_sweep_2d.SweepSettings2D``
    fixes a **front-face clearance** (0.04 m) and bisects for the hip x that
    produces it.  Its own comment gives the reason, and the reason is right:
    the leg's reach grows with ``theta_climb``, so one hip x would leave every
    theta a different distance to roll.

``cartesian_swing_planner_2d.swing_onto_step_2d``
    fixes a **hip x distance** (0.20 m) from the leading edge, on an obstacle
    that is also a different size (0.20 / 0.45 rather than 0.10 / 0.35).

Overlaying two feasibility maps built on those two conventions would compare
cells that do not describe the same physical situation.  This module picks one
convention -- the clearance -- and builds both worlds from it.

Clearance is the right choice for a second reason beyond theta-dependence.
The mechanism Day 8--9 §26.5(6) found is itself a clearance problem: the wheel
radius (0.145 m) is larger than the distance from the contact point to the
obstacle, so the tyre is inside the step before the contact point has moved.
Measuring the approach as a gap names that mechanism; measuring it as a hip
coordinate hides it.

Nothing here re-implements geometry.  ``approach_hip_x_for_clearance_2d`` wraps
the bisection that already exists in the sweep module rather than copying it,
so there is exactly one definition of "standing this far from the face".
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from numpy.typing import NDArray

from legwheel.planners.hybrid import RimId

from .single_leg_rolling_scene_2d import (
    SingleLegRollingScene2D,
    build_single_leg_rolling_scene_2d,
)
from .cartesian_swing_contract_2d import (
    SwingStartState2D,
    build_leg_on_surface_scene_2d,
    swing_start_state_from_scene_2d,
)
from .cartesian_swing_ik_2d import rim_point_model_gap_2d
from .right_up_left_down_full_traversal_2d import (
    STAGE_APPROACH,
    _front_face_clearance_m,
    STAGE_ROLL_DOWN,
    STAGE_ROLL_UP,
    STAGE_WHEEL_TRANSITION,
    ObstacleSpec2D,
    RollingTraversalResult2D,
    TraversalConstraints2D,
    TraversalFrame2D,
    TraversalInitialState2D,
    check_right_up_left_down_traversal,
)
from .right_up_left_down_sweep_2d import (
    _hip_x_for_start_clearance,
    seam_bridge_for_sampling_m,
)

__all__ = [
    "SharedTerrainSpec2D",
    "approach_hip_x_for_clearance_2d",
    "standing_scene_2d",
    "rolling_inputs_2d",
    "swing_start_from_standing_2d",
    "StandingAlignment2D",
    "check_standing_alignment_2d",
    "RollExitHandoff2D",
    "roll_exit_swing_start_2d",
    "DEFAULT_ALIGNMENT_THETAS_DEG",
    "run_step0_alignment_2d",
    "alignment_rows",
    "handoff_rows",
    "write_rows_csv",
]


# --------------------------------------------------------------------------
# The shared terrain
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SharedTerrainSpec2D:
    """One rectangle, stated once, consumed by both worlds.

    ``top_length_m`` is the rectangle width.  It is named for what it means to
    the planner rather than for its geometry: Day 6--7 Step 12R showed the top
    run -- not the height -- is what a full rolling traversal runs out of, and
    Day 10--11 promotes it to a decision axis alongside the height.

    The defaults are the Day 6--7 sweep's obstacle, because that side already
    has a finished feasibility map: re-expressing the swing side is cheaper
    than re-running the rolling side.
    """

    height_m: float
    top_length_m: float = 0.35
    x_start_m: float = 0.10
    ground_height_m: float = 0.0
    obstacle_id: str = "day10_11_obstacle"
    #: Matches ``SweepSettings2D.arc_samples``, which is what the Day 6--7
    #: feasibility map was actually computed with.  Aligning to the finished
    #: side is the whole point of Step 0.  Note this is *not* the swing side's
    #: ``leg_arc_samples`` (241): that samples the leg for collision checks,
    #: not the rim for contact, so the two numbers do not contradict.
    #: Whatever this is set to, pair it with
    #: ``seam_bridge_for_sampling_m(arc_samples)`` -- the seam width the
    #: handover guard measures is a sampling artefact that scales as
    #: 1/arc_samples, so the default 5 mm guard spuriously rejects the
    #: right-to-left handover below about 140 samples.
    arc_samples: int = 121
    gamma_rad: float = 0.0

    def __post_init__(self) -> None:
        for name in ("height_m", "top_length_m", "x_start_m", "ground_height_m", "gamma_rad"):
            value = float(getattr(self, name))
            if not np.isfinite(value):
                raise ValueError(f"{name} must be finite.")
        if self.height_m <= 0.0 or self.top_length_m <= 0.0:
            raise ValueError("height_m and top_length_m must be positive.")
        if self.arc_samples < 3:
            raise ValueError("arc_samples must be at least 3.")
        if self.gamma_rad != 0.0:
            raise ValueError("the Day 10--11 first version is planar: gamma must be 0.")

    @property
    def x_max_m(self) -> float:
        """World x of the trailing face."""

        return float(self.x_start_m + self.top_length_m)

    @property
    def top_z_m(self) -> float:
        """World z of the obstacle top."""

        return float(self.ground_height_m + self.height_m)

    @property
    def rolling_obstacle(self) -> ObstacleSpec2D:
        """The same rectangle in the form every Day 6--7 stage expects."""

        return ObstacleSpec2D(
            x_start_m=self.x_start_m,
            width_m=self.top_length_m,
            height_m=self.height_m,
            ground_height_m=self.ground_height_m,
            obstacle_id=self.obstacle_id,
            gamma_rad=self.gamma_rad,
            arc_samples=self.arc_samples,
        )

    @property
    def surface_scene_kwargs(self) -> dict:
        """Keyword set for :func:`build_leg_on_surface_scene_2d`.

        It is deliberately not the same dict as ``ObstacleSpec2D.scene_kwargs``:
        that builder solves for the hip height itself and takes no ``gamma``.
        Keeping the two conversions here is what stops callers from guessing.
        """

        return {
            "ground_height_m": self.ground_height_m,
            "obstacle_x_start_m": self.x_start_m,
            "obstacle_width_m": self.top_length_m,
            "obstacle_height_m": self.height_m,
            "obstacle_id": self.obstacle_id,
            "arc_samples": self.arc_samples,
        }


# --------------------------------------------------------------------------
# The shared approach currency
# --------------------------------------------------------------------------


def approach_hip_x_for_clearance_2d(
    spec: SharedTerrainSpec2D,
    theta_rad: float,
    clearance_m: float,
    *,
    beta_rad: float = 0.0,
    iterations: int = 60,
) -> float:
    """Hip x that leaves the standing leg ``clearance_m`` short of the front face.

    This is a thin wrapper, on purpose.  The bisection lives in
    ``right_up_left_down_sweep_2d`` and the rolling map was built with it; a
    second copy here would be a second definition of the shared axis, which is
    the one thing Step 0 exists to prevent.
    """

    return float(
        _hip_x_for_start_clearance(
            spec.rolling_obstacle,
            float(theta_rad),
            float(beta_rad),
            float(clearance_m),
            iterations=iterations,
        )
    )


def standing_scene_2d(
    spec: SharedTerrainSpec2D,
    theta_rad: float,
    *,
    hip_x_m: float,
    support_height_m: float | None = None,
    beta_rad: float = 0.0,
    surface_offset_m: float = 1e-9,
) -> SingleLegRollingScene2D:
    """Stand the leg at ``hip_x_m`` on a surface, solving only for the hip height.

    ``support_height_m`` defaults to the lower ground.  Pass ``spec.top_z_m``
    to stand on the obstacle top instead -- that is the pose a descent starts
    from, and it has to be built the same way as the approach pose or the two
    ends of a sequence will not be comparable.
    """

    support = spec.ground_height_m if support_height_m is None else float(support_height_m)
    return build_leg_on_surface_scene_2d(
        float(theta_rad),
        float(beta_rad),
        float(hip_x_m),
        support,
        surface_offset_m=surface_offset_m,
        **spec.surface_scene_kwargs,
    )


def rolling_inputs_2d(
    spec: SharedTerrainSpec2D,
    theta_rad: float,
    clearance_m: float,
    *,
    beta_rad: float = 0.0,
) -> tuple[ObstacleSpec2D, TraversalInitialState2D]:
    """The Day 6--7 traversal inputs for one shared cell.

    ``hip_z_m`` stays ``None`` deliberately: "standing on the lower ground" is
    the initial condition, and letting the traversal realise it is what keeps
    the rolling side's hip height an *output* of theta rather than a number
    written down here.  That asymmetry against the swing side is not an
    oversight -- it is the thing Step 4 has to measure.
    """

    hip_x = approach_hip_x_for_clearance_2d(
        spec, theta_rad, clearance_m, beta_rad=beta_rad
    )
    return spec.rolling_obstacle, TraversalInitialState2D(
        hip_x_m=hip_x, beta_rad=float(beta_rad), hip_z_m=None
    )


def swing_start_from_standing_2d(
    spec: SharedTerrainSpec2D,
    theta_rad: float,
    clearance_m: float,
    *,
    beta_rad: float = 0.0,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
) -> tuple[SingleLegRollingScene2D, SwingStartState2D]:
    """The Day 8--9 swing start for the same shared cell."""

    hip_x = approach_hip_x_for_clearance_2d(
        spec, theta_rad, clearance_m, beta_rad=beta_rad
    )
    scene = standing_scene_2d(spec, theta_rad, hip_x_m=hip_x, beta_rad=beta_rad)
    start = swing_start_state_from_scene_2d(
        scene,
        contact_tolerance_m=contact_tolerance_m,
        collision_tolerance_m=collision_tolerance_m,
    )
    return scene, start


# --------------------------------------------------------------------------
# Alignment check
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class StandingAlignment2D:
    """Whether both worlds place the same leg in the same place for one cell."""

    height_m: float
    top_length_m: float
    theta_deg: float
    clearance_m: float
    hip_x_m: float
    rolling_hip_z_m: float
    swing_hip_z_m: float
    hip_z_difference_m: float
    surface_offset_m: float
    measured_clearance_m: float | None
    contact_rim: str | None
    contact_alpha_deg: float | None
    rim_geometry_gap_m: float | None
    note: str

    @property
    def explained_by_surface_offset(self) -> bool:
        """Is the whole difference the deliberate 1 nm lift, and nothing else?

        ``build_leg_on_surface_scene_2d`` lifts the pose by ``surface_offset_m``
        so a sampled point resting *exactly* on a surface does not leave a
        1e-17 rounding error to decide contact versus penetration.  The sweep's
        template does not lift.  A difference equal to that lift means the two
        conventions agree; anything else means they do not.
        """

        return bool(
            abs(self.hip_z_difference_m - self.surface_offset_m) <= 1e-15
        )

    def as_dict(self) -> dict:
        return {
            "height_mm": self.height_m * 1e3,
            "top_length_m": self.top_length_m,
            "theta_deg": self.theta_deg,
            "clearance_mm": self.clearance_m * 1e3,
            "hip_x_m": self.hip_x_m,
            "rolling_hip_z_m": self.rolling_hip_z_m,
            "swing_hip_z_m": self.swing_hip_z_m,
            "hip_z_difference_nm": self.hip_z_difference_m * 1e9,
            "explained_by_surface_offset": self.explained_by_surface_offset,
            "measured_clearance_mm": (
                None if self.measured_clearance_m is None
                else self.measured_clearance_m * 1e3
            ),
            "contact_rim": self.contact_rim,
            "contact_alpha_deg": self.contact_alpha_deg,
            "rim_geometry_gap_mm": (
                None if self.rim_geometry_gap_m is None
                else self.rim_geometry_gap_m * 1e3
            ),
            "note": self.note,
        }


def _rolling_template_hip_z_m(
    spec: SharedTerrainSpec2D, theta_rad: float, beta_rad: float
) -> float:
    """The hip height the sweep's bisection template stands the leg at.

    Reproduced from ``_hip_x_for_start_clearance`` rather than imported,
    because the sweep computes it inline and does not return it.  Any drift
    between this and the sweep would show up as a non-zero difference in
    :class:`StandingAlignment2D`, which is exactly what the check is for.
    """

    template = build_single_leg_rolling_scene_2d(
        float(theta_rad), float(beta_rad), 0.0, 0.0,
        **spec.rolling_obstacle.scene_kwargs,
    )
    return float(
        spec.ground_height_m - float(template.geometry.points_hip_xz_m[:, 1].min())
    )


def check_standing_alignment_2d(
    spec: SharedTerrainSpec2D,
    theta_rad: float,
    clearance_m: float,
    *,
    beta_rad: float = 0.0,
    surface_offset_m: float = 1e-9,
) -> StandingAlignment2D:
    """Build one cell's standing pose both ways and compare them."""

    hip_x = approach_hip_x_for_clearance_2d(
        spec, theta_rad, clearance_m, beta_rad=beta_rad
    )
    rolling_hip_z = _rolling_template_hip_z_m(spec, theta_rad, beta_rad)
    swing_scene = standing_scene_2d(
        spec, theta_rad, hip_x_m=hip_x, beta_rad=beta_rad,
        surface_offset_m=surface_offset_m,
    )
    swing_hip_z = float(swing_scene.hip_pose.position_world_xz_m[1])

    rim = alpha_deg = gap_m = None
    measured_clearance = None
    note = "ok"
    try:
        start = swing_start_state_from_scene_2d(swing_scene)
    except (ValueError, KeyError) as error:
        note = f"no legal standing contact: {error}"
    else:
        rim = str(RimId(start.rim).value)
        alpha_deg = float(np.rad2deg(start.alpha_rad))
        gap_m = float(
            rim_point_model_gap_2d(theta_rad, beta_rad, start.rim, start.alpha_rad)
        )
        # The same clearance function the bisection minimises against.  A
        # second definition here would be a second meaning for the shared
        # axis, which is the one thing Step 0 exists to prevent -- and it
        # would silently disagree, because the leg's nearest point to the
        # front *face segment* is not its largest x.
        measured_clearance = float(_front_face_clearance_m(swing_scene))

    return StandingAlignment2D(
        height_m=float(spec.height_m),
        top_length_m=float(spec.top_length_m),
        theta_deg=float(np.rad2deg(theta_rad)),
        clearance_m=float(clearance_m),
        hip_x_m=hip_x,
        rolling_hip_z_m=rolling_hip_z,
        swing_hip_z_m=swing_hip_z,
        hip_z_difference_m=swing_hip_z - rolling_hip_z,
        surface_offset_m=float(surface_offset_m),
        measured_clearance_m=measured_clearance,
        contact_rim=rim,
        contact_alpha_deg=alpha_deg,
        rim_geometry_gap_m=gap_m,
        note=note,
    )


# --------------------------------------------------------------------------
# Generalised roll-exit -> swing-start handoff
# --------------------------------------------------------------------------


_STAGE_CHOICES = (STAGE_APPROACH, STAGE_ROLL_UP, STAGE_WHEEL_TRANSITION, STAGE_ROLL_DOWN)


@dataclass(frozen=True)
class RollExitHandoff2D:
    """A rolling stage's exit, re-expressed as something a swing can start from.

    ``day6_7_roll_end_swing_start_2d`` did this for one cached pose.  Day 10--11
    needs it for any ``(height, top length, theta)`` and at any stage boundary,
    because the strategy space is a 2x2: the descent can begin from a rolling
    exit *or* from a swing touchdown, and the two have to be built the same way
    to be comparable.
    """

    stage: str
    phase: str | None
    feasible_traversal: bool
    reached_stage: bool
    scene: SingleLegRollingScene2D | None
    start: SwingStartState2D | None
    frame: TraversalFrame2D | None
    contact_drift_m: float | None
    alpha_difference_deg: float | None
    rim_matches_frame: bool | None
    rim_geometry_gap_m: float | None
    l_transition_m: float | None
    failure_stage: str | None
    failure_reason: str | None
    note: str

    def as_dict(self) -> dict:
        frame_pose = None if self.frame is None else self.frame.pose
        return {
            "stage": self.stage,
            "phase": self.phase,
            "feasible_traversal": self.feasible_traversal,
            "reached_stage": self.reached_stage,
            "theta_deg": None if frame_pose is None else frame_pose.as_dict()["theta_deg"],
            "beta_deg": None if frame_pose is None else frame_pose.as_dict()["beta_deg"],
            "hip_x_m": None if frame_pose is None else frame_pose.hip_x_m,
            "hip_z_m": None if frame_pose is None else frame_pose.hip_z_m,
            "frame_rim": None if frame_pose is None else frame_pose.active_rim,
            "rebuilt_rim": None if self.start is None else str(RimId(self.start.rim).value),
            "rim_matches_frame": self.rim_matches_frame,
            "contact_drift_mm": (
                None if self.contact_drift_m is None else self.contact_drift_m * 1e3
            ),
            "alpha_difference_deg": self.alpha_difference_deg,
            "rim_geometry_gap_mm": (
                None if self.rim_geometry_gap_m is None
                else self.rim_geometry_gap_m * 1e3
            ),
            "l_transition_m": self.l_transition_m,
            "failure_stage": self.failure_stage,
            "failure_reason": self.failure_reason,
            "note": self.note,
        }


def _last_accepted_frame(
    result: RollingTraversalResult2D, stage: str
) -> TraversalFrame2D | None:
    """The final usable frame of one stage, or ``None`` if the stage never ran."""

    chosen = None
    for frame in result.trajectory:
        if frame.stage == stage and frame.accepted:
            chosen = frame
    return chosen


def roll_exit_swing_start_2d(
    spec: SharedTerrainSpec2D,
    theta_climb_rad: float,
    *,
    clearance_m: float = 0.04,
    stage: str = STAGE_ROLL_UP,
    beta_rad: float = 0.0,
    constraints: TraversalConstraints2D | None = None,
    result: RollingTraversalResult2D | None = None,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
) -> RollExitHandoff2D:
    """Run the traversal for one cell and hand a stage's exit to the swing side.

    The contact is recovered by re-running the terrain query on the rebuilt
    scene rather than by trusting the frame's stored contact point -- the same
    rule ``day6_7_roll_end_swing_start_2d`` used, and for the same reason: a
    stored point that the current geometry model would not reproduce is a
    silent disagreement, and the drift is worth reporting rather than hiding.

    Unlike the cached version this does **not** raise when the rebuilt contact
    disagrees.  Day 10--11 needs the disagreement as a measurement: it is where
    the 1.2 mm rim-geometry difference between ``LegModel.rim_point`` (0.145 m)
    and the drawn arc (0.1438 m upper rims) first meets a real handoff.
    """

    if stage not in _STAGE_CHOICES:
        raise ValueError(f"stage must be one of {_STAGE_CHOICES}, got {stage!r}.")

    if constraints is None:
        constraints = TraversalConstraints2D(
            max_seam_bridge_m=seam_bridge_for_sampling_m(spec.arc_samples)
        )
    if result is None:
        obstacle, initial_state = rolling_inputs_2d(
            spec, theta_climb_rad, clearance_m, beta_rad=beta_rad
        )
        result = check_right_up_left_down_traversal(
            obstacle, initial_state, theta_climb=float(theta_climb_rad),
            constraints=constraints,
        )

    frame = _last_accepted_frame(result, stage)
    if frame is None:
        return RollExitHandoff2D(
            stage=stage, phase=None,
            feasible_traversal=bool(result.full_success), reached_stage=False,
            scene=None, start=None, frame=None,
            contact_drift_m=None, alpha_difference_deg=None,
            rim_matches_frame=None, rim_geometry_gap_m=None,
            l_transition_m=result.l_transition_m,
            failure_stage=result.failure_stage, failure_reason=result.failure_reason,
            note=f"the traversal never produced an accepted {stage} frame.",
        )

    try:
        start = swing_start_state_from_scene_2d(
            frame.scene,
            contact_tolerance_m=contact_tolerance_m,
            collision_tolerance_m=collision_tolerance_m,
        )
    except (ValueError, KeyError) as error:
        return RollExitHandoff2D(
            stage=stage, phase=frame.phase,
            feasible_traversal=bool(result.full_success), reached_stage=True,
            scene=frame.scene, start=None, frame=frame,
            contact_drift_m=None, alpha_difference_deg=None,
            rim_matches_frame=None, rim_geometry_gap_m=None,
            l_transition_m=result.l_transition_m,
            failure_stage=result.failure_stage, failure_reason=result.failure_reason,
            note=f"stage exit reached but no swing-legal contact: {error}",
        )

    drift = None
    if frame.contact_point_world_xz_m is not None:
        drift = float(
            np.linalg.norm(
                start.contact_point_world_xz_m
                - np.asarray(frame.contact_point_world_xz_m, dtype=float)
            )
        )
    alpha_difference = (
        None if frame.alpha_rad is None
        else float(np.rad2deg(start.alpha_rad - frame.alpha_rad))
    )
    rim_matches = (
        None if frame.active_rim is None
        else bool(str(RimId(start.rim).value) == str(frame.active_rim))
    )
    gap = float(
        rim_point_model_gap_2d(start.theta_rad, start.beta_rad, start.rim, start.alpha_rad)
    )

    return RollExitHandoff2D(
        stage=stage, phase=frame.phase,
        feasible_traversal=bool(result.full_success), reached_stage=True,
        scene=frame.scene, start=start, frame=frame,
        contact_drift_m=drift, alpha_difference_deg=alpha_difference,
        rim_matches_frame=rim_matches, rim_geometry_gap_m=gap,
        l_transition_m=result.l_transition_m,
        failure_stage=result.failure_stage, failure_reason=result.failure_reason,
        note="ok",
    )


# --------------------------------------------------------------------------
# Step 0 driver and output
# --------------------------------------------------------------------------


DEFAULT_ALIGNMENT_THETAS_DEG: tuple[float, ...] = (40.0, 50.0, 60.0, 70.0, 85.0)
DEFAULT_ALIGNMENT_HEIGHTS_M: tuple[float, ...] = (0.06, 0.10, 0.14)


def run_step0_alignment_2d(
    *,
    heights_m: Sequence[float] = DEFAULT_ALIGNMENT_HEIGHTS_M,
    thetas_deg: Sequence[float] = DEFAULT_ALIGNMENT_THETAS_DEG,
    clearances_m: Sequence[float] = (0.02, 0.04, 0.08),
    top_length_m: float = 0.35,
    arc_samples: int = 241,
    handoff_clearance_m: float = 0.04,
    handoff_stages: Sequence[str] = (STAGE_ROLL_UP, STAGE_WHEEL_TRANSITION),
    handoff_thetas_deg: Sequence[float] | None = None,
    progress: bool = False,
) -> tuple[list[StandingAlignment2D], list[tuple[SharedTerrainSpec2D, float, RollExitHandoff2D]]]:
    """Run both halves of Step 0 and return their raw results.

    The standing check is cheap and swept widely; the handoff check runs a full
    traversal per cell, so it is swept narrowly and reuses one traversal for
    every requested stage.
    """

    alignments: list[StandingAlignment2D] = []
    for height in heights_m:
        spec = SharedTerrainSpec2D(
            height_m=float(height), top_length_m=top_length_m, arc_samples=arc_samples
        )
        for theta_deg in thetas_deg:
            theta = float(np.deg2rad(theta_deg))
            for clearance in clearances_m:
                alignments.append(
                    check_standing_alignment_2d(spec, theta, float(clearance))
                )

    handoffs: list[tuple[SharedTerrainSpec2D, float, RollExitHandoff2D]] = []
    thetas = DEFAULT_ALIGNMENT_THETAS_DEG if handoff_thetas_deg is None else handoff_thetas_deg
    constraints = TraversalConstraints2D(
        max_seam_bridge_m=seam_bridge_for_sampling_m(arc_samples)
    )
    for height in heights_m:
        spec = SharedTerrainSpec2D(
            height_m=float(height), top_length_m=top_length_m, arc_samples=arc_samples
        )
        for theta_deg in thetas:
            theta = float(np.deg2rad(theta_deg))
            obstacle, initial_state = rolling_inputs_2d(spec, theta, handoff_clearance_m)
            result = check_right_up_left_down_traversal(
                obstacle, initial_state, theta_climb=theta, constraints=constraints
            )
            if progress:
                verdict = "full" if result.full_success else (result.failure_stage or "partial")
                print(f"  h={height * 1e3:.0f} mm theta={theta_deg:.0f} deg -> {verdict}")
            for stage in handoff_stages:
                handoffs.append(
                    (
                        spec,
                        theta_deg,
                        roll_exit_swing_start_2d(
                            spec, theta, clearance_m=handoff_clearance_m,
                            stage=stage, constraints=constraints, result=result,
                        ),
                    )
                )
    return alignments, handoffs


def alignment_rows(alignments: Iterable[StandingAlignment2D]) -> list[dict]:
    return [item.as_dict() for item in alignments]


def handoff_rows(
    handoffs: Iterable[tuple[SharedTerrainSpec2D, float, RollExitHandoff2D]]
) -> list[dict]:
    rows = []
    for spec, theta_deg, handoff in handoffs:
        row = {
            "height_mm": spec.height_m * 1e3,
            "top_length_m": spec.top_length_m,
            "theta_climb_deg": theta_deg,
        }
        row.update(handoff.as_dict())
        rows.append(row)
    return rows


def write_rows_csv(path, rows: Sequence[dict]) -> Path:
    """Write rows to CSV, using the first row's keys as the header."""

    path = Path(path)
    rows = list(rows)
    if not rows:
        raise ValueError("refusing to write an empty CSV.")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return path
