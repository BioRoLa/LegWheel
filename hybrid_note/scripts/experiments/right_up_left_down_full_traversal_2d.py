"""Day 6--7 Step 10R: the full right-up / left-down rolling traversal.

This module is *orchestration only*.  Every contact decision, no-slip step and
collision judgement still comes from the Step 4.5 / 7R / 8R / 9R functions that
were verified separately; nothing about leg geometry or terrain contact is
re-derived here.  What Step 10R adds is:

* one APPROACH stage that rolls the leg along the lower ground until the right
  rim actually reaches the obstacle front face, so the traversal starts from
  the ground rather than from a hand-placed pose;
* a single phase vocabulary spanning all four existing stages;
* strict state hand-off between stages, with the discontinuity measured rather
  than assumed;
* one ``RollingTraversalResult2D`` carrying the whole trajectory.

The phase sequence is::

    APPROACH
    -> RIGHT_RIM_FRONT_CONTACT
    -> RIGHT_RIM_ROLL_UP
    -> RIGHT_RIM_TOP
    -> RETRACT_TO_WHEEL
    -> WHEEL_MODE_TOP_ROLL
    -> LEFT_RIM_READY
    -> LEFT_RIM_TRAILING_TRANSITION
    -> LEFT_RIM_ROLL_DOWN
    -> LOWER_GROUND_CONTACT

Why APPROACH had to be added
----------------------------
Step 4.5 requires its initial pose to *already* be in right-rim front-face
contact, and the canonical entry pose used so far hangs 25 mm above the ground.
A traversal that starts there is not "from the lower ground", so requirement 7
could not be checked.  Rolling in on the ground first produces a genuine dual
contact -- foot rim on the ground, right rim on the front face -- which Step
4.5 accepts unchanged.  The approach reuses the same flat-ground no-slip solver
that Step 9R already uses after touchdown; it is the same motion, mirrored to
the front of the obstacle.

What this module deliberately does not do
-----------------------------------------
No airborne reset (Step 6.75), no Bezier swing, no direct swing, no recovery
strategy of any kind, and no sweeping.  A stage that fails ends the traversal
and is reported; nothing is retried with different parameters.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, replace
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from matplotlib.animation import FuncAnimation

from legwheel.planners.hybrid.terrain_query_2d import (
    query_point_to_terrain_surfaces_2d,
)
from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (
    ForwardRollingResult2D,
    SingleLegRollingScene2D,
    _translated_scene_with_sample_on_target_2d,
    build_single_leg_rolling_scene_2d,
    plot_single_leg_rolling_scene_2d,
    query_single_leg_rolling_scene_2d,
    run_forward_right_rim_roll_up_2d,
)
from hybrid_note.scripts.experiments.trailing_edge_roll_down_2d import (
    _candidate_for_sample,
    _contact_sample_indices,
    _contour_arc_length,
    _lowest_contact_sample,
    _solve_flat_roll_rotation,
)
from hybrid_note.scripts.experiments.right_up_left_down_traversal_2d import (
    LeftRimRollDownResult2D,
    PHASE_LEFT_CORNER_PIVOT,
    PHASE_LEFT_GROUND_CONTACT,
    PHASE_LEFT_GROUND_ROLL,
    PHASE_LEFT_ROLL_DOWN,
    WheelModeTransitionResult2D,
    run_left_rim_roll_down_2d,
    run_wheel_mode_transition_to_corner_2d,
    traversal_phase_label,
)

__all__ = [
    "ObstacleSpec2D",
    "TraversalInitialState2D",
    "TraversalConstraints2D",
    "TraversalPose2D",
    "TraversalFrame2D",
    "ApproachResult2D",
    "RollingTraversalResult2D",
    "run_approach_to_front_face_2d",
    "check_right_up_left_down_traversal",
    "traversal_frame_rows",
    "traversal_summary_row",
    "write_traversal_csv",
    "plot_traversal_key_frames_2d",
    "animate_full_traversal_2d",
    "TRAVERSAL_PHASE_ORDER",
    "STAGE_APPROACH",
    "STAGE_ROLL_UP",
    "STAGE_WHEEL_TRANSITION",
    "STAGE_ROLL_DOWN",
]


# --------------------------------------------------------------------------
# Stage and phase vocabulary
# --------------------------------------------------------------------------

STAGE_APPROACH = "APPROACH"
STAGE_ROLL_UP = "ROLL_UP"
STAGE_WHEEL_TRANSITION = "WHEEL_TRANSITION"
STAGE_ROLL_DOWN = "ROLL_DOWN"

PHASE_APPROACH = "APPROACH"
PHASE_FRONT_CONTACT = "RIGHT_RIM_FRONT_CONTACT"
PHASE_ROLL_UP = "RIGHT_RIM_ROLL_UP"
PHASE_RIGHT_RIM_TOP = "RIGHT_RIM_TOP"
PHASE_RETRACT_TO_WHEEL = "RETRACT_TO_WHEEL"
PHASE_WHEEL_MODE_TOP_ROLL = "WHEEL_MODE_TOP_ROLL"
PHASE_LEFT_RIM_READY = "LEFT_RIM_READY"
PHASE_TRAILING_TRANSITION = "LEFT_RIM_TRAILING_TRANSITION"
PHASE_ROLL_DOWN = "LEFT_RIM_ROLL_DOWN"
PHASE_LOWER_GROUND = "LOWER_GROUND_CONTACT"
PHASE_FAILED = "FAILED"

TRAVERSAL_PHASE_ORDER = (
    PHASE_APPROACH,
    PHASE_FRONT_CONTACT,
    PHASE_ROLL_UP,
    PHASE_RIGHT_RIM_TOP,
    PHASE_RETRACT_TO_WHEEL,
    PHASE_WHEEL_MODE_TOP_ROLL,
    PHASE_LEFT_RIM_READY,
    PHASE_TRAILING_TRANSITION,
    PHASE_ROLL_DOWN,
    PHASE_LOWER_GROUND,
)

# Step 4.5 names its own motion primitives; map them onto the shared vocabulary
# rather than renaming them at the source, which would disturb Step 1--6.75.
_ROLL_UP_PHASE_BY_ROLL_PHASE = {
    "FRONT_FACE_CONTACT": PHASE_FRONT_CONTACT,
    "LEADING_CORNER_TRANSITION": PHASE_ROLL_UP,
    "LEADING_CORNER_TO_TOP": PHASE_ROLL_UP,
    "TOP_ROLL": PHASE_RIGHT_RIM_TOP,
    "TOP_ROLL_COMPLETE": PHASE_RIGHT_RIM_TOP,
    "FAILED": PHASE_FAILED,
}

_TRANSITION_PHASE_BY_LABEL = {
    "RETRACT_TO_WHEEL": PHASE_RETRACT_TO_WHEEL,
    "WHEEL_MODE_TOP_ROLL": PHASE_WHEEL_MODE_TOP_ROLL,
    # Once the left rim carries the contact the readiness condition holds, and
    # it keeps holding while the top roll finishes.  Treating LEFT_RIM_READY as
    # the phase from the handover onward -- rather than as one isolated frame --
    # keeps the phase sequence monotone, which is what makes it a usable label.
    "LEFT_RIM_TOP_ROLL": PHASE_LEFT_RIM_READY,
}


def _transition_phase(label: str, frame) -> str:
    """Name one transition frame, refusing to call it ready when it is not.

    A corner arrival is only ``LEFT_RIM_READY`` if the left rim is actually
    carrying the contact.  On a top too short for the handover the leg still
    reaches the corner -- on the *right* rim -- and labelling that frame ready
    would make the phase say the opposite of what the readiness check found.
    """

    if label == "TRAILING_CORNER_ARRIVAL":
        return (
            PHASE_LEFT_RIM_READY if frame.active_rim == "left_rim"
            else PHASE_WHEEL_MODE_TOP_ROLL
        )
    return _TRANSITION_PHASE_BY_LABEL.get(label, PHASE_WHEEL_MODE_TOP_ROLL)

_DESCENT_PHASE_BY_PHASE = {
    PHASE_LEFT_CORNER_PIVOT: PHASE_TRAILING_TRANSITION,
    PHASE_LEFT_ROLL_DOWN: PHASE_ROLL_DOWN,
    PHASE_LEFT_GROUND_CONTACT: PHASE_LOWER_GROUND,
    PHASE_LEFT_GROUND_ROLL: PHASE_LOWER_GROUND,
    "FAILED": PHASE_FAILED,
}


# --------------------------------------------------------------------------
# Inputs
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class ObstacleSpec2D:
    """The rectangular obstacle, in the form every stage already expects."""

    x_start_m: float = 0.10
    width_m: float = 0.35
    height_m: float = 0.10
    ground_height_m: float = 0.0
    obstacle_id: str = "day6_7_obstacle"
    gamma_rad: float = 0.0
    arc_samples: int = 241

    def __post_init__(self) -> None:
        for name in ("x_start_m", "width_m", "height_m", "ground_height_m", "gamma_rad"):
            value = getattr(self, name)
            if not np.isfinite(value):
                raise ValueError(f"{name} must be finite.")
        if self.width_m <= 0.0 or self.height_m <= 0.0:
            raise ValueError("obstacle width and height must be positive.")
        if self.arc_samples < 3:
            raise ValueError("arc_samples must be at least 3.")

    @property
    def x_max_m(self) -> float:
        return float(self.x_start_m + self.width_m)

    @property
    def top_z_m(self) -> float:
        return float(self.ground_height_m + self.height_m)

    @property
    def scene_kwargs(self) -> dict:
        """Exactly the keyword set every existing stage builds scenes with."""

        return {
            "gamma_rad": self.gamma_rad,
            "ground_height_m": self.ground_height_m,
            "obstacle_x_start_m": self.x_start_m,
            "obstacle_width_m": self.width_m,
            "obstacle_height_m": self.height_m,
            "obstacle_id": self.obstacle_id,
            "arc_samples": self.arc_samples,
        }


@dataclass(frozen=True)
class TraversalInitialState2D:
    """Where the leg starts, on the lower ground in front of the obstacle.

    ``hip_z_m = None`` means "stand the leg on the lower ground", which is the
    only sense in which the start pose is chosen rather than simulated: it is
    an initial condition, not a mid-traversal teleport.
    """

    hip_x_m: float = -0.10
    beta_rad: float = 0.0
    hip_z_m: float | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.hip_x_m) or not np.isfinite(self.beta_rad):
            raise ValueError("hip_x_m and beta_rad must be finite.")
        if self.hip_z_m is not None and not np.isfinite(self.hip_z_m):
            raise ValueError("hip_z_m must be finite when provided.")


@dataclass(frozen=True)
class TraversalConstraints2D:
    """Every knob the four stages take, in one place.

    Defaults are the values each stage was verified with, so constructing this
    with no arguments reproduces the verified behaviour.
    """

    theta_wheel_rad: float = np.deg2rad(17.0)

    approach_step_m: float = 0.005
    approach_coarse_beta_step_rad: float = np.deg2rad(1.0)
    approach_max_rotation_rad: float = np.deg2rad(20.0)
    approach_max_steps: int = 200

    roll_up_dx_m: float = 0.002
    roll_up_beta_step_rad: float = -np.deg2rad(1.0)
    roll_up_top_beta_step_rad: float = -np.deg2rad(0.5)
    roll_up_top_roll_distance_m: float = 0.02
    roll_up_max_forward_steps: int = 100

    theta_step_rad: float = np.deg2rad(1.0)
    wheel_beta_step_rad: float = np.deg2rad(1.0)
    max_seam_bridge_m: float = 5e-3

    pivot_beta_step_rad: float = np.deg2rad(1.0)
    release_theta: bool = False
    theta_release_clearance_m: float = 0.02
    descent_rim_margin_m: float = 5e-3
    ground_roll_distance_m: float = 0.02

    contact_tolerance_m: float = 1e-3
    collision_tolerance_m: float = 1e-3
    surface_offset_m: float = 1e-9
    measure_collision_margin: bool = True

    def __post_init__(self) -> None:
        if self.approach_step_m <= 0.0:
            raise ValueError("approach_step_m must be positive.")
        if self.approach_max_steps < 0:
            raise ValueError("approach_max_steps must be non-negative.")
        if self.contact_tolerance_m < 0.0 or self.collision_tolerance_m < 0.0:
            raise ValueError("tolerances must be non-negative.")


@dataclass(frozen=True)
class TraversalPose2D:
    """A pose plus its contact, sufficient to restart a traversal from here."""

    theta_rad: float
    beta_rad: float
    hip_x_m: float
    hip_z_m: float
    active_rim: str | None
    alpha_rad: float | None
    contact_point_world_xz_m: tuple[float, float] | None
    terrain_surface_id: str | None

    def as_dict(self) -> dict:
        contact = self.contact_point_world_xz_m
        return {
            "theta_deg": float(np.rad2deg(self.theta_rad)),
            "beta_deg": float(np.rad2deg(self.beta_rad)),
            "hip_x_m": self.hip_x_m,
            "hip_z_m": self.hip_z_m,
            "active_rim": self.active_rim,
            "alpha_deg": (
                None if self.alpha_rad is None else float(np.rad2deg(self.alpha_rad))
            ),
            "contact_x_m": None if contact is None else contact[0],
            "contact_z_m": None if contact is None else contact[1],
            "terrain_surface_id": self.terrain_surface_id,
        }


@dataclass(frozen=True)
class TraversalFrame2D:
    """One frame of the whole traversal, in a shape that does not vary by stage."""

    index: int
    stage: str
    stage_step: int
    phase: str
    theta_rad: float
    beta_rad: float
    hip_position_world_xz_m: tuple[float, float]
    active_rim: str | None
    alpha_rad: float | None
    contact_point_world_xz_m: tuple[float, float] | None
    terrain_surface_id: str | None
    valid_contact: bool
    collision: bool
    penetration: bool
    link_collision: bool
    vertical_face_collision: bool
    accepted: bool
    failure_reason: str | None
    collision_margin_m: float | None
    scene: SingleLegRollingScene2D
    query_result: object

    @property
    def pose(self) -> TraversalPose2D:
        return TraversalPose2D(
            theta_rad=self.theta_rad,
            beta_rad=self.beta_rad,
            hip_x_m=self.hip_position_world_xz_m[0],
            hip_z_m=self.hip_position_world_xz_m[1],
            active_rim=self.active_rim,
            alpha_rad=self.alpha_rad,
            contact_point_world_xz_m=self.contact_point_world_xz_m,
            terrain_surface_id=self.terrain_surface_id,
        )


# --------------------------------------------------------------------------
# Collision margin
# --------------------------------------------------------------------------


def _collision_margin_m(scene, query_result) -> float | None:
    """How close the leg came to touching something it is not resting on.

    A rolling primitive holds its support contact at zero gap by construction,
    so "distance to the terrain" is identically zero and says nothing.  The
    margin that does carry information is the distance to every *other* terrain
    surface: the front face while climbing, the trailing face while descending,
    the lower ground while on top.

    Returns a negative value (the deepest penetration) when the frame is
    already illegal, and ``None`` when no other surface is reachable at all --
    which happens legitimately while the leg sits on the obstacle top, where
    the faces and the ground are occluded.  The same
    ``query_point_to_terrain_surfaces_2d`` primitive the contact detector uses
    supplies the geometry, so this adds a metric, not a second terrain model.
    """

    depths = [
        item.penetration_depth_m
        for group in (
            query_result.geometry_penetrations,
            query_result.link_collisions,
            query_result.collisions,
        )
        for item in group
    ]
    if depths:
        return -float(max(depths))

    resting_on = {candidate.terrain_surface_id for candidate in query_result.candidates}
    points = scene.geometry.points_world_xz_m
    best = np.inf
    for index in _contact_sample_indices(scene.geometry):
        point_query = query_point_to_terrain_surfaces_2d(
            points[index], scene.terrain, span_tolerance_m=0.0
        )
        for gap in point_query.relevant_surface_gaps:
            if gap.surface_id in resting_on:
                continue
            if gap.euclidean_distance_m < best:
                best = gap.euclidean_distance_m
    return None if not np.isfinite(best) else float(best)


def _front_face_clearance_m(scene) -> float:
    """Signed distance from the leg to the obstacle front face.

    Mirror of ``_back_face_clearance`` in the descent module: same segment, at
    ``x_min`` instead of ``x_max``.  The sign matters here because the approach
    has to stop *before* crossing the face, not merely notice that it has.
    """

    obstacle = scene.terrain.obstacle
    if obstacle is None:
        return np.inf
    top_z = scene.terrain.ground_height_m + obstacle.height_m
    points = scene.geometry.points_world_xz_m[_contact_sample_indices(scene.geometry)]
    clamped_z = np.clip(points[:, 1], scene.terrain.ground_height_m, top_z)
    distances = np.hypot(points[:, 0] - obstacle.x_min_m, points[:, 1] - clamped_z)
    past_face = (points[:, 0] > obstacle.x_min_m) & (points[:, 1] < top_z)
    return float(np.min(np.where(past_face, -distances, distances)))


# --------------------------------------------------------------------------
# Stage 1: APPROACH
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class ApproachResult2D:
    """Rolling along the lower ground until the right rim meets the front face."""

    frames: tuple
    success: bool
    failure_reason: str | None
    front_face_clearance_m: float
    rolled_distance_m: float
    support_rim: str | None

    @property
    def final_frame(self):
        return self.frames[-1]


def _approach_pose_at_rotation(
    theta_rad, beta_rad, rotation_rad, previous_sample, previous_points,
    contact_x_m, scene_kwargs, ground_height_m, surface_offset_m,
):
    """Place the leg after rotating it ``rotation_rad`` forward on flat ground.

    This is ``_flat_roll_template`` with one restriction lifted: it accepts a
    rotation small enough that the support sample does not change, which is a
    pure rotation about the pinned contact and still no-slip.  The approach
    needs those sub-sample rotations because it has to stop at a point defined
    by the *terrain* -- the front face -- not at a sample boundary.  Coarse
    sampling would otherwise make the last step overshoot into the obstacle.
    """

    beta = beta_rad - rotation_rad
    template = build_single_leg_rolling_scene_2d(
        theta_rad, beta, 0.0, 0.0, **scene_kwargs
    )
    sample = _lowest_contact_sample(template.geometry)
    regions = np.asarray(template.geometry.contact_regions)
    if sample < previous_sample or regions[sample] != regions[previous_sample]:
        return None
    arc = 0.5 * (
        _contour_arc_length(previous_points, previous_sample, sample)
        + _contour_arc_length(template.geometry.points_hip_xz_m, previous_sample, sample)
    )
    target = np.array(
        [contact_x_m + arc, ground_height_m + surface_offset_m], dtype=float
    )
    scene = _translated_scene_with_sample_on_target_2d(template, sample, target)
    return scene, sample, float(beta)


def _solve_approach_touch_rotation(
    theta_rad, beta_rad, crossing_rotation_rad, previous_sample, previous_points,
    contact_x_m, scene_kwargs, obstacle, constraints, query,
):
    """Bisect the last approach rotation so the leg just touches the front face.

    ``crossing_rotation_rad`` is known to go too far.  The clearance falls
    monotonically with the rotation, so the largest rotation that still leaves
    a legal, positive clearance is found by bisection and returned.
    """

    def evaluate(rotation):
        placed = _approach_pose_at_rotation(
            theta_rad, beta_rad, rotation, previous_sample, previous_points,
            contact_x_m, scene_kwargs, obstacle.ground_height_m,
            constraints.surface_offset_m,
        )
        if placed is None:
            return None
        scene, sample, beta = placed
        result = query(scene)
        candidate = _candidate_for_sample(
            result, sample, surface_ids=(scene.terrain.ground_surface_id,)
        )
        if result.collision or candidate is None:
            return None
        return scene, result, sample, beta, _front_face_clearance_m(scene)

    low, high = 0.0, float(crossing_rotation_rad)
    best = evaluate(low)
    if best is None or best[4] <= 0.0:
        return None
    for _ in range(40):
        if best[4] <= constraints.contact_tolerance_m:
            break
        middle = 0.5 * (low + high)
        probe = evaluate(middle)
        if probe is None or probe[4] <= 0.0:
            high = middle
        else:
            low, best = middle, probe
    scene, result, sample, beta, clearance = best
    return scene, result, sample, beta


def run_approach_to_front_face_2d(
    obstacle: ObstacleSpec2D,
    initial_state: TraversalInitialState2D,
    theta_climb_rad: float,
    constraints: TraversalConstraints2D,
) -> ApproachResult2D:
    """Roll the standing leg forward until it just touches the obstacle front.

    Every step is a no-slip rigid rotation solved by the same
    ``_solve_flat_roll_rotation`` used for post-touchdown ground rolling, so
    the approach obeys the same contact model as the rest of the traversal.
    The final step is refined by halving the requested advance until the leg
    stops just short of the face rather than inside it.
    """

    if not isinstance(obstacle, ObstacleSpec2D):
        raise TypeError("obstacle must be an ObstacleSpec2D.")
    if not isinstance(initial_state, TraversalInitialState2D):
        raise TypeError("initial_state must be a TraversalInitialState2D.")
    if not isinstance(constraints, TraversalConstraints2D):
        raise TypeError("constraints must be a TraversalConstraints2D.")

    scene_kwargs = obstacle.scene_kwargs
    theta = float(theta_climb_rad)
    beta = float(initial_state.beta_rad)

    def query(scene):
        return query_single_leg_rolling_scene_2d(
            scene,
            contact_tolerance_m=constraints.contact_tolerance_m,
            collision_tolerance_m=constraints.collision_tolerance_m,
        )

    if initial_state.hip_z_m is None:
        template = build_single_leg_rolling_scene_2d(
            theta, beta, 0.0, 0.0, **scene_kwargs
        )
        hip_z = (
            obstacle.ground_height_m
            - float(template.geometry.points_hip_xz_m[:, 1].min())
            + constraints.surface_offset_m
        )
    else:
        hip_z = float(initial_state.hip_z_m)

    scene = build_single_leg_rolling_scene_2d(
        theta, beta, float(initial_state.hip_x_m), hip_z, **scene_kwargs
    )
    query_result = query(scene)
    sample = _lowest_contact_sample(scene.geometry)
    frames = []

    def append(scene, query_result, *, accepted=True, failure_reason=None):
        candidate = _candidate_for_sample(
            query_result, sample, surface_ids=(scene.terrain.ground_surface_id,)
        )
        frames.append(
            _approach_frame(
                len(frames), scene, query_result, candidate, theta, beta,
                accepted=accepted, failure_reason=failure_reason,
                constraints=constraints,
            )
        )

    start_clearance = _front_face_clearance_m(scene)
    ground_candidate = _candidate_for_sample(
        query_result, sample, surface_ids=(scene.terrain.ground_surface_id,)
    )
    if query_result.collision or ground_candidate is None:
        append(scene, query_result, accepted=False,
               failure_reason="APPROACH_START_NOT_ON_LOWER_GROUND")
        return ApproachResult2D(tuple(frames), False,
                                "APPROACH_START_NOT_ON_LOWER_GROUND",
                                start_clearance, 0.0, None)
    if start_clearance <= 0.0:
        append(scene, query_result, accepted=False,
               failure_reason="APPROACH_START_ALREADY_AT_THE_FRONT_FACE")
        return ApproachResult2D(tuple(frames), False,
                                "APPROACH_START_ALREADY_AT_THE_FRONT_FACE",
                                start_clearance, 0.0, None)
    append(scene, query_result)

    support_rim = str(scene.geometry.contact_regions[sample])
    start_x = float(scene.geometry.points_world_xz_m[sample][0])
    steps = 0
    reason = None
    while steps < constraints.approach_max_steps:
        contact = scene.geometry.points_world_xz_m[sample]
        solved = _solve_flat_roll_rotation(
            theta,
            beta,
            sample,
            scene.geometry.points_hip_xz_m,
            scene_kwargs,
            target_advance_m=constraints.approach_step_m,
            beta_direction=-1.0,
            coarse_step_rad=constraints.approach_coarse_beta_step_rad,
            max_rotation_rad=constraints.approach_max_rotation_rad,
            iterations=24,
            rotation_resolution_rad=constraints.approach_coarse_beta_step_rad / 50.0,
        )
        if solved is None:
            reason = "RIM_ARC_EXHAUSTED_DURING_APPROACH"
            break
        template, next_sample, arc, rotation = solved
        target = np.array(
            [contact[0] + arc, obstacle.ground_height_m + constraints.surface_offset_m],
            dtype=float,
        )
        next_scene = _translated_scene_with_sample_on_target_2d(
            template, next_sample, target
        )
        next_query = query(next_scene)
        next_candidate = _candidate_for_sample(
            next_query, next_sample,
            surface_ids=(next_scene.terrain.ground_surface_id,),
        )
        clearance = _front_face_clearance_m(next_scene)
        if clearance > constraints.contact_tolerance_m and not (
            next_query.collision or next_candidate is None
        ):
            scene, query_result, sample, beta = (
                next_scene, next_query, next_sample, beta - rotation
            )
            append(scene, query_result)
            steps += 1
            continue

        # This step reaches the face.  Solve for *how far* to rotate so the leg
        # stops just short of it, instead of accepting a step sized by the rim
        # sampling.  Where the approach ends is set by the terrain, so it must
        # not be quantised to a sample boundary.
        landed = _solve_approach_touch_rotation(
            theta, beta, rotation, sample, scene.geometry.points_hip_xz_m,
            float(contact[0]), scene_kwargs, obstacle, constraints, query,
        )
        if landed is None:
            reason = "APPROACH_COULD_NOT_STOP_SHORT_OF_THE_FRONT_FACE"
            break
        scene, query_result, sample, beta = landed
        append(scene, query_result)
        steps += 1
        break
    else:
        reason = "APPROACH_STEP_LIMIT_REACHED"

    final_clearance = _front_face_clearance_m(scene)
    rolled = float(scene.geometry.points_world_xz_m[sample][0] - start_x)
    if reason is None and final_clearance > constraints.contact_tolerance_m:
        reason = "APPROACH_DID_NOT_REACH_THE_FRONT_FACE"
    success = reason is None
    if not success:
        frames[-1] = replace(frames[-1], accepted=False, failure_reason=reason,
                             phase=PHASE_FAILED)
    return ApproachResult2D(
        tuple(frames), success, reason, final_clearance, rolled, support_rim
    )


def _approach_frame(
    index, scene, query_result, candidate, theta, beta, *,
    accepted, failure_reason, constraints,
) -> TraversalFrame2D:
    """Wrap one approach pose in the shared frame shape."""

    contact = (
        None if candidate is None
        else tuple(float(value) for value in candidate.point_world_xz_m)
    )
    return TraversalFrame2D(
        index=index,
        stage=STAGE_APPROACH,
        stage_step=index,
        phase=PHASE_APPROACH,
        theta_rad=float(theta),
        beta_rad=float(beta),
        hip_position_world_xz_m=tuple(
            float(value) for value in scene.hip_pose.position_world_xz_m
        ),
        active_rim=(
            None if candidate is None
            else str(scene.geometry.contact_regions[candidate.sample_index])
        ),
        alpha_rad=None if candidate is None else float(candidate.alpha_rad),
        contact_point_world_xz_m=contact,
        terrain_surface_id=None if candidate is None else candidate.terrain_surface_id,
        valid_contact=candidate is not None,
        collision=bool(query_result.collision),
        penetration=bool(query_result.geometry_penetrations),
        link_collision=bool(query_result.link_collisions),
        vertical_face_collision=bool(query_result.collisions),
        accepted=accepted,
        failure_reason=failure_reason,
        collision_margin_m=(
            _collision_margin_m(scene, query_result)
            if constraints.measure_collision_margin else None
        ),
        scene=scene,
        query_result=query_result,
    )


# --------------------------------------------------------------------------
# Adapters: existing stage frames -> the shared frame shape
# --------------------------------------------------------------------------


def _adapt_frame(
    frame,
    *,
    index: int,
    stage: str,
    phase: str,
    constraints: TraversalConstraints2D,
) -> TraversalFrame2D:
    """Re-express one stage frame without recomputing anything about it.

    Each stage records slightly different bookkeeping, so the pose and the
    contact are read from whichever field that stage happens to use.  No
    contact decision is revisited here: ``accepted``, ``collision`` and
    ``valid_contact`` are carried over exactly as the stage decided them.
    """

    scene = frame.scene
    query_result = frame.query_result
    hip = getattr(frame, "hip_position_world_xz_m", None)
    if hip is None:
        hip = tuple(float(value) for value in scene.hip_pose.position_world_xz_m)
    return TraversalFrame2D(
        index=index,
        stage=stage,
        stage_step=int(frame.step),
        phase=phase,
        theta_rad=float(frame.theta_rad),
        beta_rad=float(frame.beta_rad),
        hip_position_world_xz_m=tuple(float(value) for value in hip),
        active_rim=frame.active_rim,
        alpha_rad=None if frame.alpha_rad is None else float(frame.alpha_rad),
        contact_point_world_xz_m=(
            None if frame.contact_point_world_xz_m is None
            else tuple(float(value) for value in frame.contact_point_world_xz_m)
        ),
        terrain_surface_id=frame.terrain_surface_id,
        valid_contact=bool(frame.valid_contact),
        collision=bool(frame.collision),
        penetration=bool(query_result.geometry_penetrations),
        link_collision=bool(query_result.link_collisions),
        vertical_face_collision=bool(query_result.collisions),
        accepted=bool(frame.accepted),
        failure_reason=frame.failure_reason,
        collision_margin_m=(
            _collision_margin_m(scene, query_result)
            if constraints.measure_collision_margin else None
        ),
        scene=scene,
        query_result=query_result,
    )


def _roll_up_frames(result, start_index, constraints) -> list[TraversalFrame2D]:
    return [
        _adapt_frame(
            frame,
            index=start_index + offset,
            stage=STAGE_ROLL_UP,
            phase=_ROLL_UP_PHASE_BY_ROLL_PHASE.get(frame.roll_phase, PHASE_ROLL_UP),
            constraints=constraints,
        )
        for offset, frame in enumerate(result.frames)
    ]


def _transition_frames(result, start_index, constraints) -> list[TraversalFrame2D]:
    corner_x = result.trailing_corner_world_xz_m[0]
    frames = []
    for offset, frame in enumerate(result.frames):
        label = traversal_phase_label(
            frame, result.theta_target_rad, trailing_corner_x_m=corner_x
        )
        frames.append(
            _adapt_frame(
                frame,
                index=start_index + offset,
                stage=STAGE_WHEEL_TRANSITION,
                phase=_transition_phase(label, frame),
                constraints=constraints,
            )
        )
    return frames


def _descent_frames(result, start_index, constraints) -> list[TraversalFrame2D]:
    return [
        _adapt_frame(
            frame,
            index=start_index + offset,
            stage=STAGE_ROLL_DOWN,
            phase=_DESCENT_PHASE_BY_PHASE.get(frame.phase, PHASE_ROLL_DOWN),
            constraints=constraints,
        )
        for offset, frame in enumerate(result.frames)
    ]


def _handoff_discontinuity_m(previous: TraversalFrame2D, following: TraversalFrame2D) -> float:
    """How far the pose jumped across a stage boundary.

    Stages hand over by passing the previous stage's final state, so the first
    frame of a stage should *be* the last frame of the one before it.  This
    measures that rather than trusting it, which is the only way to show no
    teleport happened.
    """

    hip = np.linalg.norm(
        np.asarray(following.hip_position_world_xz_m)
        - np.asarray(previous.hip_position_world_xz_m)
    )
    joints = abs(following.theta_rad - previous.theta_rad) + abs(
        following.beta_rad - previous.beta_rad
    )
    return float(hip + joints)


# --------------------------------------------------------------------------
# Result
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class RollingTraversalResult2D:
    """The whole traversal: one trajectory, one verdict, per-stage detail."""

    feasible: bool
    failure_stage: str | None
    failure_reason: str | None
    trajectory: tuple[TraversalFrame2D, ...]

    approach_success: bool
    roll_up_success: bool
    retract_success: bool
    left_rim_ready_success: bool
    roll_down_success: bool

    l_transition_m: float | None
    final_state: TraversalPose2D | None
    minimum_collision_margin_m: float | None

    obstacle: ObstacleSpec2D
    theta_climb_rad: float
    constraints: TraversalConstraints2D

    approach_result: ApproachResult2D | None = None
    roll_up_result: ForwardRollingResult2D | None = None
    transition_result: WheelModeTransitionResult2D | None = None
    descent_result: LeftRimRollDownResult2D | None = None

    reached_lower_ground_behind_obstacle: bool = False
    maximum_handoff_discontinuity_m: float | None = None
    # (from_stage, to_stage, jump) for each boundary.  Only the entry into
    # the descent is expected to be non-zero: Step 9R pins the contact sample
    # exactly on the trailing corner, and the corner-arrival frame is only
    # within the corner tolerance of it.  That snap is part of the verified
    # Step 9R, not a pose written down by this module.
    stage_handoff_discontinuities_m: tuple[tuple[str, str, float], ...] = ()
    stage_frame_counts: tuple[tuple[str, int], ...] = ()

    # The spec names these without unit suffixes; keep both so either reads.
    @property
    def L_transition(self) -> float | None:  # noqa: N802
        return self.l_transition_m

    @property
    def minimum_collision_margin(self) -> float | None:
        return self.minimum_collision_margin_m

    @property
    def full_success(self) -> bool:
        """Ground in front to ground behind, by rolling, with nothing skipped.

        Every stage must have succeeded *and* the leg must actually end
        supported on the lower ground past the trailing face.  Reaching the
        obstacle top with a healthy leg is not a traversal.
        """

        return bool(
            self.feasible
            and self.approach_success
            and self.roll_up_success
            and self.retract_success
            and self.left_rim_ready_success
            and self.roll_down_success
            and self.reached_lower_ground_behind_obstacle
        )

    @property
    def phases_visited(self) -> tuple[str, ...]:
        seen = []
        for frame in self.trajectory:
            if not seen or seen[-1] != frame.phase:
                seen.append(frame.phase)
        return tuple(seen)

    @property
    def final_frame(self) -> TraversalFrame2D | None:
        return self.trajectory[-1] if self.trajectory else None


# --------------------------------------------------------------------------
# Step 10R: the orchestrator
# --------------------------------------------------------------------------


def check_right_up_left_down_traversal(
    obstacle: ObstacleSpec2D | None = None,
    initial_state: TraversalInitialState2D | None = None,
    theta_climb: float = np.deg2rad(60.0),
    constraints: TraversalConstraints2D | None = None,
) -> RollingTraversalResult2D:
    """Run the whole right-up / left-down traversal and report one verdict.

    The four existing stages are called in order and each one receives the
    previous stage's final state -- ``run_forward_right_rim_roll_up_2d`` gets
    the approach's final hip pose and beta, the wheel transition gets the
    roll-up *result object*, and the descent gets the transition result.  No
    stage is started from a pose written down here.

    The first failing stage ends the run.  Nothing is retried, relaxed or
    recovered: a failure is an answer, not a problem to work around.
    """

    obstacle = ObstacleSpec2D() if obstacle is None else obstacle
    initial_state = (
        TraversalInitialState2D() if initial_state is None else initial_state
    )
    constraints = TraversalConstraints2D() if constraints is None else constraints
    if not isinstance(obstacle, ObstacleSpec2D):
        raise TypeError("obstacle must be an ObstacleSpec2D.")
    if not isinstance(initial_state, TraversalInitialState2D):
        raise TypeError("initial_state must be a TraversalInitialState2D.")
    if not isinstance(constraints, TraversalConstraints2D):
        raise TypeError("constraints must be a TraversalConstraints2D.")
    theta_climb = float(theta_climb)
    if not np.isfinite(theta_climb) or theta_climb <= 0.0:
        raise ValueError("theta_climb must be a positive finite angle.")

    trajectory: list[TraversalFrame2D] = []
    boundaries: list[tuple[str, str, float]] = []

    def finish(
        *,
        failure_stage,
        failure_reason,
        approach=None,
        roll_up=None,
        transition=None,
        descent=None,
        approach_ok=False,
        roll_up_ok=False,
        retract_ok=False,
        ready_ok=False,
        descent_ok=False,
    ) -> RollingTraversalResult2D:
        margins = [
            frame.collision_margin_m
            for frame in trajectory
            if frame.collision_margin_m is not None
        ]
        final = trajectory[-1] if trajectory else None
        landed = _landed_behind_obstacle(final, obstacle, constraints)
        counts: dict[str, int] = {}
        for frame in trajectory:
            counts[frame.stage] = counts.get(frame.stage, 0) + 1
        return RollingTraversalResult2D(
            feasible=failure_stage is None,
            failure_stage=failure_stage,
            failure_reason=failure_reason,
            trajectory=tuple(trajectory),
            approach_success=approach_ok,
            roll_up_success=roll_up_ok,
            retract_success=retract_ok,
            left_rim_ready_success=ready_ok,
            roll_down_success=descent_ok,
            l_transition_m=None if transition is None else transition.l_transition_m,
            final_state=None if final is None else final.pose,
            minimum_collision_margin_m=min(margins) if margins else None,
            obstacle=obstacle,
            theta_climb_rad=theta_climb,
            constraints=constraints,
            approach_result=approach,
            roll_up_result=roll_up,
            transition_result=transition,
            descent_result=descent,
            reached_lower_ground_behind_obstacle=landed,
            maximum_handoff_discontinuity_m=(
                max(value for _, _, value in boundaries) if boundaries else None
            ),
            stage_handoff_discontinuities_m=tuple(boundaries),
            stage_frame_counts=tuple(counts.items()),
        )

    # -- Stage 1: APPROACH -------------------------------------------------
    approach = run_approach_to_front_face_2d(
        obstacle, initial_state, theta_climb, constraints
    )
    trajectory.extend(approach.frames)
    # The last approach frame *is* the front-face contact, so name it that.
    if approach.success:
        trajectory[-1] = replace(trajectory[-1], phase=PHASE_FRONT_CONTACT)
    else:
        return finish(failure_stage=STAGE_APPROACH,
                      failure_reason=approach.failure_reason, approach=approach)

    handoff = approach.final_frame

    # -- Stage 2: RIGHT-RIM ROLL-UP (Step 4.5) -----------------------------
    roll_up = run_forward_right_rim_roll_up_2d(
        candidate_theta_rad=theta_climb,
        initial_beta_rad=handoff.beta_rad,
        hip_x_m=handoff.hip_position_world_xz_m[0],
        hip_z_m=handoff.hip_position_world_xz_m[1],
        dx_m=constraints.roll_up_dx_m,
        max_forward_steps=constraints.roll_up_max_forward_steps,
        beta_step_rad=constraints.roll_up_beta_step_rad,
        top_beta_step_rad=constraints.roll_up_top_beta_step_rad,
        top_roll_distance_m=constraints.roll_up_top_roll_distance_m,
        contact_tolerance_m=constraints.contact_tolerance_m,
        collision_tolerance_m=constraints.collision_tolerance_m,
        **obstacle.scene_kwargs,
    )
    roll_up_frames = _roll_up_frames(roll_up, len(trajectory), constraints)
    boundaries.append((STAGE_APPROACH, STAGE_ROLL_UP,
                   _handoff_discontinuity_m(trajectory[-1], roll_up_frames[0])))
    # Drop the duplicated hand-off pose: the roll-up's first frame is the
    # approach's last frame re-queried, not a new step.
    trajectory.extend(roll_up_frames[1:])
    if not roll_up.success:
        return finish(failure_stage=STAGE_ROLL_UP,
                      failure_reason=roll_up.failure_reason,
                      approach=approach, roll_up=roll_up, approach_ok=True)

    # -- Stage 3: RETRACT + WHEEL-MODE TOP ROLL (Steps 7R and 8R) ----------
    transition = run_wheel_mode_transition_to_corner_2d(
        roll_up,
        theta_target_rad=constraints.theta_wheel_rad,
        theta_step_rad=constraints.theta_step_rad,
        beta_step_rad=constraints.wheel_beta_step_rad,
        max_seam_bridge_m=constraints.max_seam_bridge_m,
        contact_tolerance_m=constraints.contact_tolerance_m,
        collision_tolerance_m=constraints.collision_tolerance_m,
    )
    transition_frames = _transition_frames(transition, len(trajectory), constraints)
    boundaries.append((STAGE_ROLL_UP, STAGE_WHEEL_TRANSITION,
                   _handoff_discontinuity_m(trajectory[-1], transition_frames[0])))
    trajectory.extend(transition_frames[1:])

    # Step 7R is the prefix of this one continuation that reaches theta; there
    # is no separate simulation to succeed or fail.
    retract_ok = transition.theta_reached_step is not None
    if not retract_ok:
        return finish(failure_stage=STAGE_WHEEL_TRANSITION,
                      failure_reason=transition.failure_reason
                      or "THETA_TARGET_NOT_REACHED_ON_TOP",
                      approach=approach, roll_up=roll_up, transition=transition,
                      approach_ok=True, roll_up_ok=True)
    if not transition.reached_trailing_corner:
        return finish(failure_stage=STAGE_WHEEL_TRANSITION,
                      failure_reason=transition.failure_reason
                      or "TRAILING_CORNER_NOT_REACHED",
                      approach=approach, roll_up=roll_up, transition=transition,
                      approach_ok=True, roll_up_ok=True, retract_ok=True)
    if not transition.left_rim_ready:
        return finish(failure_stage=STAGE_WHEEL_TRANSITION,
                      failure_reason=transition.readiness_failure,
                      approach=approach, roll_up=roll_up, transition=transition,
                      approach_ok=True, roll_up_ok=True, retract_ok=True)

    # -- Stage 4: LEFT-RIM DESCENT (Step 9R) -------------------------------
    descent = run_left_rim_roll_down_2d(
        transition,
        pivot_beta_step_rad=constraints.pivot_beta_step_rad,
        release_theta=constraints.release_theta,
        theta_release_clearance_m=constraints.theta_release_clearance_m,
        descent_rim_margin_m=constraints.descent_rim_margin_m,
        ground_roll_distance_m=constraints.ground_roll_distance_m,
        contact_tolerance_m=constraints.contact_tolerance_m,
        collision_tolerance_m=constraints.collision_tolerance_m,
        surface_offset_m=constraints.surface_offset_m,
    )
    descent_frames = _descent_frames(descent, len(trajectory), constraints)
    boundaries.append((STAGE_WHEEL_TRANSITION, STAGE_ROLL_DOWN,
                   _handoff_discontinuity_m(trajectory[-1], descent_frames[0])))
    trajectory.extend(descent_frames[1:])
    if not descent.ground_contact_success:
        return finish(failure_stage=STAGE_ROLL_DOWN,
                      failure_reason=descent.failure_reason,
                      approach=approach, roll_up=roll_up, transition=transition,
                      descent=descent, approach_ok=True, roll_up_ok=True,
                      retract_ok=True, ready_ok=True)

    return finish(failure_stage=None, failure_reason=None,
                  approach=approach, roll_up=roll_up, transition=transition,
                  descent=descent, approach_ok=True, roll_up_ok=True,
                  retract_ok=True, ready_ok=True, descent_ok=True)


def _landed_behind_obstacle(
    frame: TraversalFrame2D | None,
    obstacle: ObstacleSpec2D,
    constraints: TraversalConstraints2D,
) -> bool:
    """Is the leg resting on the lower ground past the trailing face?

    This is the half of ``full_success`` the stage flags cannot express: every
    stage can succeed and still leave the leg somewhere that is not "across".
    """

    if frame is None or frame.contact_point_world_xz_m is None:
        return False
    if not frame.accepted or frame.collision or not frame.valid_contact:
        return False
    if frame.terrain_surface_id != frame.scene.terrain.ground_surface_id:
        return False
    contact_x, contact_z = frame.contact_point_world_xz_m
    return bool(
        contact_x > obstacle.x_max_m
        and abs(contact_z - obstacle.ground_height_m)
        <= constraints.contact_tolerance_m
    )


# --------------------------------------------------------------------------
# Tables and export
# --------------------------------------------------------------------------


def traversal_frame_rows(result: RollingTraversalResult2D) -> list[dict]:
    """The whole trajectory as flat rows, one per frame."""

    if not isinstance(result, RollingTraversalResult2D):
        raise TypeError("result must be a RollingTraversalResult2D.")
    rows = []
    for frame in result.trajectory:
        contact = frame.contact_point_world_xz_m
        rows.append(
            {
                "index": frame.index,
                "stage": frame.stage,
                "stage_step": frame.stage_step,
                "phase": frame.phase,
                "theta_deg": float(np.rad2deg(frame.theta_rad)),
                "beta_deg": float(np.rad2deg(frame.beta_rad)),
                "hip_x_m": frame.hip_position_world_xz_m[0],
                "hip_z_m": frame.hip_position_world_xz_m[1],
                "active_rim": frame.active_rim,
                "alpha_deg": (
                    None if frame.alpha_rad is None
                    else float(np.rad2deg(frame.alpha_rad))
                ),
                "contact_x_m": None if contact is None else contact[0],
                "contact_z_m": None if contact is None else contact[1],
                "contact_surface": frame.terrain_surface_id,
                "collision_margin_m": frame.collision_margin_m,
                "valid_contact": frame.valid_contact,
                "collision": frame.collision,
                "penetration": frame.penetration,
                "link_collision": frame.link_collision,
                "vertical_face_collision": frame.vertical_face_collision,
                "accepted": frame.accepted,
                "failure_reason": frame.failure_reason,
            }
        )
    return rows


def traversal_summary_row(result: RollingTraversalResult2D) -> dict:
    """One row describing the whole traversal."""

    if not isinstance(result, RollingTraversalResult2D):
        raise TypeError("result must be a RollingTraversalResult2D.")
    final = result.final_state
    transition = result.transition_result
    descent = result.descent_result
    return {
        "full_success": result.full_success,
        "feasible": result.feasible,
        "failure_stage": result.failure_stage,
        "failure_reason": result.failure_reason,
        "approach_success": result.approach_success,
        "roll_up_success": result.roll_up_success,
        "retract_success": result.retract_success,
        "left_rim_ready_success": result.left_rim_ready_success,
        "roll_down_success": result.roll_down_success,
        "reached_lower_ground_behind_obstacle":
            result.reached_lower_ground_behind_obstacle,
        "obstacle_x_start_m": result.obstacle.x_start_m,
        "obstacle_width_m": result.obstacle.width_m,
        "obstacle_height_m": result.obstacle.height_m,
        "theta_climb_deg": float(np.rad2deg(result.theta_climb_rad)),
        "theta_wheel_deg": float(np.rad2deg(result.constraints.theta_wheel_rad)),
        "frame_count": len(result.trajectory),
        "phases_visited": " -> ".join(result.phases_visited),
        "L_transition_m": result.l_transition_m,
        "minimum_collision_margin_m": result.minimum_collision_margin_m,
        "maximum_handoff_discontinuity_m": result.maximum_handoff_discontinuity_m,
        "corner_snap_m": next(
            (value for source, _, value in result.stage_handoff_discontinuities_m
             if source == STAGE_WHEEL_TRANSITION), None),
        "approach_rolled_distance_m": (
            None if result.approach_result is None
            else result.approach_result.rolled_distance_m
        ),
        "required_beta_rotation_deg": (
            None if transition is None or transition.required_beta_rotation_rad is None
            else float(np.rad2deg(transition.required_beta_rotation_rad))
        ),
        "pivot_rotation_deg": (
            None if descent is None or descent.pivot_rotation_rad is None
            else float(np.rad2deg(descent.pivot_rotation_rad))
        ),
        "hip_drop_at_touchdown_m": (
            None if descent is None else descent.hip_drop_at_touchdown_m
        ),
        "ground_roll_distance_achieved_m": (
            None if descent is None else descent.ground_roll_distance_achieved_m
        ),
        "final_theta_deg": None if final is None else float(np.rad2deg(final.theta_rad)),
        "final_beta_deg": None if final is None else float(np.rad2deg(final.beta_rad)),
        "final_hip_x_m": None if final is None else final.hip_x_m,
        "final_hip_z_m": None if final is None else final.hip_z_m,
        "final_contact_x_m": (
            None if final is None or final.contact_point_world_xz_m is None
            else final.contact_point_world_xz_m[0]
        ),
        "final_active_rim": None if final is None else final.active_rim,
        "final_contact_surface": None if final is None else final.terrain_surface_id,
    }


def write_traversal_csv(
    result: RollingTraversalResult2D,
    summary_path,
    trajectory_path,
) -> tuple[Path, Path]:
    """Write the summary row and the full trajectory to two CSV files."""

    summary_path = Path(summary_path)
    trajectory_path = Path(trajectory_path)
    summary = traversal_summary_row(result)
    rows = traversal_frame_rows(result)
    for path, payload in ((summary_path, [summary]), (trajectory_path, rows)):
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(payload[0]))
            writer.writeheader()
            writer.writerows(payload)
    return summary_path, trajectory_path


# --------------------------------------------------------------------------
# Figures
# --------------------------------------------------------------------------


# Half-width of each key-frame panel's square window.  The leg spans about
# 0.22 m at theta = 60 deg, so this keeps it framed with the nearby terrain.
_KEY_FRAME_HALF_SPAN_M = 0.30


def plot_traversal_key_frames_2d(
    result: RollingTraversalResult2D,
    *,
    phases=TRAVERSAL_PHASE_ORDER,
):
    """The last accepted frame of each phase the traversal actually entered."""

    if not isinstance(result, RollingTraversalResult2D):
        raise TypeError("result must be a RollingTraversalResult2D.")
    selected = []
    for phase in phases:
        frames = [
            frame for frame in result.trajectory
            if frame.phase == phase and frame.accepted
        ]
        if frames:
            selected.append((phase, frames[-1]))
    failed = [frame for frame in result.trajectory if not frame.accepted]
    if failed:
        selected.append((PHASE_FAILED, failed[-1]))
    if not selected:
        raise ValueError("the traversal contains no drawable frame.")
    columns = min(5, len(selected))
    rows = int(np.ceil(len(selected) / columns))
    figure, axes = plt.subplots(
        rows, columns, figsize=(3.7 * columns, 3.7 * rows), squeeze=False
    )
    flat = [ax for row in axes for ax in row]
    for ax, (phase, frame) in zip(flat, selected):
        # No query overlay here: the panels are equal-aspect and small, and the
        # status/penetration annotations are already in the trajectory table.
        plot_single_leg_rolling_scene_2d(frame.scene, ax=ax)
        # Every panel gets the same square window centred on the leg, so the
        # poses can be compared directly instead of each being auto-scaled.
        centre = np.asarray(frame.hip_position_world_xz_m, dtype=float)
        ax.set_xlim(centre[0] - _KEY_FRAME_HALF_SPAN_M,
                    centre[0] + _KEY_FRAME_HALF_SPAN_M)
        ax.set_ylim(centre[1] - _KEY_FRAME_HALF_SPAN_M,
                    centre[1] + _KEY_FRAME_HALF_SPAN_M)
        margin = (
            "n/a" if frame.collision_margin_m is None
            else f"{frame.collision_margin_m * 1e3:.1f} mm"
        )
        ax.set_title(
            f"{phase}\n"
            f"theta={np.rad2deg(frame.theta_rad):.1f} deg, "
            f"beta={np.rad2deg(frame.beta_rad):.1f} deg\n"
            f"rim={frame.active_rim}, margin={margin}",
            fontsize=8.5,
        )
        legend = ax.get_legend()
        if legend is not None:
            legend.set_visible(False)
    for ax in flat[len(selected):]:
        ax.set_visible(False)
    figure.tight_layout()
    return figure, axes


def animate_full_traversal_2d(
    result: RollingTraversalResult2D,
    *,
    interval_ms: int = 110,
    frame_stride: int = 2,
    repeat: bool = False,
    show: bool = False,
):
    """Animate the whole traversal, labelled by phase, rim, theta/beta, surface."""

    if not isinstance(result, RollingTraversalResult2D):
        raise TypeError("result must be a RollingTraversalResult2D.")
    if min(interval_ms, frame_stride) <= 0:
        raise ValueError("interval and stride must be positive.")
    frames = list(result.trajectory[::frame_stride])
    if frames[-1] is not result.trajectory[-1]:
        frames.append(result.trajectory[-1])

    points = np.vstack([
        np.vstack((
            frame.scene.geometry.points_world_xz_m,
            np.asarray(frame.hip_position_world_xz_m)[None, :],
        ))
        for frame in frames
    ])
    x_pad = max(0.04, 0.06 * float(np.ptp(points[:, 0])))
    z_pad = max(0.04, 0.08 * float(np.ptp(points[:, 1])))
    x_limits = (float(np.min(points[:, 0]) - x_pad),
                float(np.max(points[:, 0]) + x_pad))
    z_limits = (
        min(result.obstacle.ground_height_m - 0.03,
            float(np.min(points[:, 1]) - z_pad)),
        float(np.max(points[:, 1]) + z_pad),
    )
    figure, ax = plt.subplots(figsize=(12.5, 5.6))
    # Colour the phase banner so the stage boundaries are visible at a glance.
    phase_colour = {
        PHASE_APPROACH: "#64748b",
        PHASE_FRONT_CONTACT: "#0891b2",
        PHASE_ROLL_UP: "#2563eb",
        PHASE_RIGHT_RIM_TOP: "#7c3aed",
        PHASE_RETRACT_TO_WHEEL: "#c026d3",
        PHASE_WHEEL_MODE_TOP_ROLL: "#db2777",
        PHASE_LEFT_RIM_READY: "#ea580c",
        PHASE_TRAILING_TRANSITION: "#ca8a04",
        PHASE_ROLL_DOWN: "#16a34a",
        PHASE_LOWER_GROUND: "#059669",
        PHASE_FAILED: "#dc2626",
    }

    def draw(index: int):
        frame = frames[index]
        ax.clear()
        plot_single_leg_rolling_scene_2d(
            frame.scene, ax=ax, query_result=frame.query_result
        )
        ax.set_xlim(*x_limits)
        ax.set_ylim(*z_limits)
        hip = np.asarray(
            [item.hip_position_world_xz_m for item in frames[: index + 1]], dtype=float
        )
        ax.plot(hip[:, 0], hip[:, 1], "--", color="#7c3aed",
                linewidth=1.5, label="hip path")
        contact = np.asarray(
            [
                item.contact_point_world_xz_m
                for item in frames[: index + 1]
                if item.contact_point_world_xz_m is not None
            ],
            dtype=float,
        )
        if contact.size:
            ax.plot(contact[:, 0], contact[:, 1], ".", color="#dc2626",
                    markersize=4.0, label="contact path")
        colour = phase_colour.get(frame.phase, "#334155")
        ax.set_title(
            f"Step 10R  [{frame.index + 1}/{len(result.trajectory)}]  {frame.phase}",
            color=colour, fontsize=13, fontweight="bold",
        )
        margin = (
            "n/a" if frame.collision_margin_m is None
            else f"{frame.collision_margin_m * 1e3:.1f} mm"
        )
        ax.text(
            0.01, 0.02,
            f"stage={frame.stage}\n"
            f"active rim={frame.active_rim}\n"
            f"contact surface={frame.terrain_surface_id}\n"
            f"theta={np.rad2deg(frame.theta_rad):.1f} deg\n"
            f"beta={np.rad2deg(frame.beta_rad):.1f} deg\n"
            f"collision margin={margin}\n"
            f"collision={frame.collision}",
            transform=ax.transAxes, fontsize=8.5, va="bottom",
            bbox={"facecolor": "white", "alpha": 0.88, "edgecolor": colour},
            zorder=16,
        )
        return tuple(ax.lines)

    animation = FuncAnimation(figure, draw, frames=len(frames),
                             interval=interval_ms, repeat=repeat, blit=False)
    draw(0)
    if show:
        plt.show()
    return animation
