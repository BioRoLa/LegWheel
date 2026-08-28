"""Day 6--7 Step 7R: trailing-edge roll-down for the single-leg 2D scene.

Step 4.5 established the leading-edge story::

    front face -> leading corner -> obstacle top

This module studies the opposite terrain transition::

    obstacle top -> trailing corner -> drop side -> lower ground

The leg model, the terrain model and the terrain-aware contact/collision query
are **not** re-implemented here.  Every pose is built by
``build_single_leg_rolling_scene_2d`` and validated by
``query_single_leg_rolling_scene_2d``; this module only adds the motion
primitive that drives them.

Three distinct rolling regimes are modelled, and they are deliberately not
treated as a mirror image of the roll-up:

``TOP_ROLL``
    No-slip rolling on the flat obstacle top.  The world contact advance
    equals the rim arc length swept between the previous and the next material
    support sample, so the active material point migrates along the rim.

``TRAILING_CORNER_TRANSITION`` / ``ROLL_DOWN``
    A sharp corner is a *point*, not a surface.  A rigid body turning over a
    point rotates about that point, so exactly one material sample stays
    pinned on the corner while beta continues in the rolling direction.  The
    contact material point does **not** migrate here; the leg descends because
    the hip orbits the pinned corner sample.  ``ROLL_DOWN`` labels the part of
    that same pivot where the lowest leg point has already dropped below the
    obstacle top.

``LOWER_GROUND_CONTACT`` / ``GROUND_ROLL``
    A second sample reaches the lower ground while the corner sample is still
    down.  After that hand-over, no-slip rolling resumes on the ground.

Because the pivot consumes rim arc without advancing the contact point, the
rim can run out before the leg reaches the ground.  That is a real physical
outcome, so it is reported as an explicit failure reason rather than hidden.
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from legwheel.planners.hybrid import ContactQueryResult2D, ContactStatus2D
from hybrid_note.scripts.experiments.single_leg_rolling_scene_2d import (
    ForwardRollingResult2D,
    SingleLegRollingScene2D,
    _translated_scene_with_sample_on_target_2d,
    build_single_leg_rolling_scene_2d,
    plot_single_leg_rolling_scene_2d,
    query_single_leg_rolling_scene_2d,
    run_forward_right_rim_roll_up_2d,
)

__all__ = [
    "RollDownFrame2D",
    "TrailingEdgeRollDownResult2D",
    "RollDownStartState2D",
    "roll_down_start_state_from_rolling_result",
    "save_roll_down_start_state",
    "load_roll_down_start_state",
    "load_or_build_roll_down_start_state",
    "run_trailing_edge_roll_down_2d",
    "roll_down_frame_rows",
    "roll_down_summary_row",
    "write_trailing_edge_roll_down_csv",
    "plot_trailing_edge_roll_down_2d",
    "plot_roll_down_key_frames_2d",
    "animate_trailing_edge_roll_down_2d",
]

# Phase labels.  ``TRAILING_CORNER_TRANSITION`` and ``ROLL_DOWN`` are two
# labels for one continuous corner pivot, split where the leg drops below the
# obstacle top.
PHASE_TOP_ROLL = "TOP_ROLL"
PHASE_CORNER = "TRAILING_CORNER_TRANSITION"
PHASE_ROLL_DOWN = "ROLL_DOWN"
PHASE_GROUND_CONTACT = "LOWER_GROUND_CONTACT"
PHASE_GROUND_ROLL = "GROUND_ROLL"
PHASE_FAILED = "FAILED"


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite.")
    return value


# --------------------------------------------------------------------------
# Frame and result records
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class RollDownFrame2D:
    """One accepted (or rejected) trailing-edge roll-down frame."""

    step: int
    phase: str
    hip_x_m: float
    hip_z_m: float
    hip_forward_displacement_m: float
    hip_vertical_displacement_m: float
    theta_rad: float
    beta_rad: float
    accumulated_rotation_rad: float
    active_rim: str | None
    active_sample_index: int | None
    alpha_rad: float | None
    contact_point_world_xz_m: tuple[float, float] | None
    terrain_surface_id: str | None
    trailing_edge_distance_m: float | None
    contact_advance_m: float
    lowest_point_world_xz_m: tuple[float, float]
    ground_clearance_m: float
    back_face_clearance_m: float | None
    # Rim arc left at the lowest physical sample; see _remaining_rim_budget_rad.
    remaining_rim_arc_rad: float | None
    corner_pinned: bool
    ground_supported: bool
    valid_contact: bool
    collision: bool
    status: ContactStatus2D
    accepted: bool
    failure_reason: str | None
    scene: SingleLegRollingScene2D
    query_result: ContactQueryResult2D

    @property
    def theta_deg(self) -> float:
        return float(np.rad2deg(self.theta_rad))

    @property
    def beta_deg(self) -> float:
        return float(np.rad2deg(self.beta_rad))


@dataclass(frozen=True)
class TrailingEdgeRollDownResult2D:
    """Outcome of one trailing-edge roll-down attempt."""

    frames: tuple[RollDownFrame2D, ...]
    success: bool
    failure_phase: str | None
    failure_reason: str | None
    top_roll_success: bool
    corner_transition_success: bool
    roll_down_success: bool
    ground_contact_success: bool
    ground_roll_success: bool
    trailing_corner_world_xz_m: tuple[float, float]
    corner_pivot_rotation_rad: float | None
    hip_drop_at_touchdown_m: float | None
    ground_contact_point_world_xz_m: tuple[float, float] | None
    ground_contact_rim: str | None
    ground_contact_alpha_rad: float | None
    remaining_rim_arc_rad_at_touchdown: float | None
    ground_roll_distance_m: float
    ground_roll_distance_achieved_m: float
    minimum_back_face_clearance_m: float | None

    @property
    def final_frame(self) -> RollDownFrame2D:
        return self.frames[-1]

    @property
    def accepted_frames(self) -> tuple[RollDownFrame2D, ...]:
        return tuple(frame for frame in self.frames if frame.accepted)

    def frames_in_phase(self, phase: str) -> tuple[RollDownFrame2D, ...]:
        return tuple(frame for frame in self.frames if frame.phase == phase)


@dataclass(frozen=True)
class RollDownStartState2D:
    """A legal obstacle-top rolling state used as the roll-down entry."""

    theta_rad: float
    beta_rad: float
    active_sample_index: int
    contact_point_world_xz_m: tuple[float, float]
    hip_position_world_xz_m: tuple[float, float]
    scene_kwargs: dict
    source_phase: str


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------


def _contact_sample_indices(geometry) -> np.ndarray:
    """Indices of samples that belong to a physical rim arc."""

    regions = np.asarray(geometry.contact_regions)
    return np.flatnonzero(regions != "non_contact_region")


def _lowest_contact_sample(geometry) -> int:
    """Global lowest physical rim sample; the tangency point on a flat floor."""

    indices = _contact_sample_indices(geometry)
    points = geometry.points_hip_xz_m
    return int(indices[int(np.argmin(points[indices, 1]))])


def _region_bounds(geometry, sample_index: int) -> tuple[int, int]:
    """Inclusive index bounds of the contiguous rim region owning a sample."""

    regions = np.asarray(geometry.contact_regions)
    region = regions[sample_index]
    if region == "non_contact_region":
        raise ValueError("sample_index is not on a physical rim arc.")
    start = sample_index
    while start > 0 and regions[start - 1] == region:
        start -= 1
    stop = sample_index
    while stop + 1 < len(regions) and regions[stop + 1] == region:
        stop += 1
    return int(start), int(stop)


def _contour_arc_length(points_hip_xz_m: np.ndarray, first: int, second: int) -> float:
    """Sampled rim arc length between two indices of the same rim region."""

    low, high = (first, second) if first <= second else (second, first)
    if low == high:
        return 0.0
    segment = points_hip_xz_m[low : high + 1]
    return float(np.sum(np.linalg.norm(np.diff(segment, axis=0), axis=1)))


def _back_face_clearance(
    scene: SingleLegRollingScene2D,
    *,
    corner_exclusion_m: float,
) -> float | None:
    """Smallest distance from the leg to the obstacle back face.

    Samples within ``corner_exclusion_m`` of the trailing corner are excluded:
    during the pivot the pinned sample sits exactly on that corner, so keeping
    it would report a meaningless zero clearance for every frame.
    """

    obstacle = scene.terrain.obstacle
    if obstacle is None:
        return None
    top_z = scene.terrain.ground_height_m + obstacle.height_m
    corner = np.array([obstacle.x_max_m, top_z], dtype=float)
    points = scene.geometry.points_world_xz_m
    indices = _contact_sample_indices(scene.geometry)
    points = points[indices]
    far_enough = np.linalg.norm(points - corner, axis=1) > corner_exclusion_m
    if not np.any(far_enough):
        return None
    points = points[far_enough]
    # Distance to the vertical back-face segment x = x_max, ground <= z <= top.
    clamped_z = np.clip(points[:, 1], scene.terrain.ground_height_m, top_z)
    distances = np.hypot(points[:, 0] - obstacle.x_max_m, points[:, 1] - clamped_z)
    return float(np.min(distances))


def _remaining_rim_arc_rad(geometry, sample_index: int) -> float:
    """Alpha still available on one rim region in the rolling direction.

    Forward rolling migrates the contact toward increasing ``alpha``.  When
    this reaches zero the rim region is exhausted and rolling can only
    continue by jumping across the non-contact seam, which is not a physical
    rolling contact.
    """

    _, stop = _region_bounds(geometry, sample_index)
    alpha = geometry.alpha_rad
    return float(abs(alpha[stop] - alpha[sample_index]))


def _remaining_rim_budget_rad(geometry) -> float:
    """Rim arc left before the *next* material point to touch down runs out.

    This is measured at the lowest physical rim sample rather than at the
    active contact.  During the corner pivot the active contact is pinned and
    does not move along the rim, but the material point heading for the lower
    ground keeps advancing, so only the lowest sample tracks the budget that
    the traversal is actually spending.
    """

    return _remaining_rim_arc_rad(geometry, _lowest_contact_sample(geometry))


# --------------------------------------------------------------------------
# Start state
# --------------------------------------------------------------------------


def roll_down_start_state_from_rolling_result(
    rolling_result: ForwardRollingResult2D,
    *,
    scene_kwargs: dict | None = None,
) -> RollDownStartState2D:
    """Take the Step 4.5 obstacle-top end state as the roll-down entry state.

    The active material sample is recovered as the lowest physical rim sample
    of the final pose, which is the tangency point on the flat obstacle top.
    """

    if not isinstance(rolling_result, ForwardRollingResult2D):
        raise TypeError("rolling_result must be a ForwardRollingResult2D.")
    if not rolling_result.success:
        raise ValueError(
            "trailing-edge roll-down requires a successful Step 4.5 roll-up."
        )
    frame = rolling_result.final_frame
    if frame.contact_point_world_xz_m is None:
        raise ValueError("the roll-up end frame carries no contact point.")
    scene = frame.scene
    sample_index = _lowest_contact_sample(scene.geometry)
    if scene_kwargs is None:
        obstacle = scene.terrain.obstacle
        if obstacle is None:
            raise ValueError("the roll-up scene has no rectangular obstacle.")
        scene_kwargs = {
            "gamma_rad": scene.gamma_rad,
            "ground_height_m": scene.terrain.ground_height_m,
            "obstacle_x_start_m": obstacle.x_min_m,
            "obstacle_width_m": obstacle.width_m,
            "obstacle_height_m": obstacle.height_m,
            "obstacle_id": obstacle.obstacle_id,
            "arc_samples": len(scene.geometry.points_hip_xz_m) // 3,
        }
    return RollDownStartState2D(
        theta_rad=float(frame.theta_rad),
        beta_rad=float(frame.beta_rad),
        active_sample_index=sample_index,
        contact_point_world_xz_m=tuple(
            float(value) for value in scene.geometry.points_world_xz_m[sample_index]
        ),
        hip_position_world_xz_m=tuple(
            float(value) for value in scene.hip_pose.position_world_xz_m
        ),
        scene_kwargs=dict(scene_kwargs),
        source_phase=frame.roll_phase,
    )


def save_roll_down_start_state(state: RollDownStartState2D, path) -> Path:
    """Persist a roll-down entry state as small, readable JSON.

    Only the entry state is cached, never the simulated frames: it is a handful
    of scalars that fully determine the pose, so it stays valid and reviewable
    even when the simulator around it changes.
    """

    if not isinstance(state, RollDownStartState2D):
        raise TypeError("state must be a RollDownStartState2D.")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "theta_rad": float(state.theta_rad),
        "beta_rad": float(state.beta_rad),
        "active_sample_index": int(state.active_sample_index),
        "contact_point_world_xz_m": [float(v) for v in state.contact_point_world_xz_m],
        "hip_position_world_xz_m": [float(v) for v in state.hip_position_world_xz_m],
        "scene_kwargs": dict(state.scene_kwargs),
        "source_phase": state.source_phase,
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return path


def load_roll_down_start_state(path) -> RollDownStartState2D:
    """Read back a roll-down entry state written by :func:`save_roll_down_start_state`."""

    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return RollDownStartState2D(
        theta_rad=float(payload["theta_rad"]),
        beta_rad=float(payload["beta_rad"]),
        active_sample_index=int(payload["active_sample_index"]),
        contact_point_world_xz_m=tuple(payload["contact_point_world_xz_m"]),
        hip_position_world_xz_m=tuple(payload["hip_position_world_xz_m"]),
        scene_kwargs=dict(payload["scene_kwargs"]),
        source_phase=payload["source_phase"],
    )


def load_or_build_roll_down_start_state(
    path,
    *,
    force_rebuild: bool = False,
    **roll_up_kwargs,
) -> tuple[RollDownStartState2D, bool]:
    """Reuse a cached roll-down entry state, or run Step 4.5 once to make one.

    Returns the state and whether the Step 4.5 roll-up actually ran.  The
    roll-up is the expensive part of the chain and its end state never changes
    unless its own inputs do, so a notebook should not pay for it on every
    execution.
    """

    path = Path(path)
    if path.exists() and not force_rebuild:
        return load_roll_down_start_state(path), False
    roll_up = run_forward_right_rim_roll_up_2d(**roll_up_kwargs)
    if not roll_up.success:
        raise RuntimeError(
            "the Step 4.5 roll-up used as the roll-down entry failed: "
            f"{roll_up.failure_reason}"
        )
    state = roll_down_start_state_from_rolling_result(roll_up)
    save_roll_down_start_state(state, path)
    return state, True



# --------------------------------------------------------------------------
# Rolling primitives
# --------------------------------------------------------------------------


def _candidate_for_sample(
    query_result: ContactQueryResult2D,
    sample_index: int,
    *,
    surface_ids: tuple[str, ...] | None = None,
):
    """Pick the query candidate that belongs to one material sample.

    The query still returns every candidate; this only adds the local
    selection rule that the active contact is the one nearest the sample the
    motion primitive is currently rolling or pivoting on.
    """

    options = [
        candidate
        for candidate in query_result.candidates
        if surface_ids is None or candidate.terrain_surface_id in surface_ids
    ]
    if not options:
        return None
    return min(
        options,
        key=lambda candidate: (
            abs(candidate.sample_index - sample_index),
            candidate.surface_distance_m,
        ),
    )


def _flat_roll_template(
    theta_rad: float,
    beta_rad: float,
    previous_sample: int,
    previous_points_hip_xz_m: np.ndarray,
    scene_kwargs: dict,
) -> tuple[SingleLegRollingScene2D, int, float] | None:
    """Rigid-rotation candidate for one no-slip step on a horizontal surface.

    Returns the untranslated template, the new lowest support sample and the
    rim arc length swept between the two material samples.  The arc length is
    the *only* source of the world contact advance, so the step is no-slip by
    construction rather than by a prescribed displacement.
    """

    template = build_single_leg_rolling_scene_2d(
        theta_rad, beta_rad, 0.0, 0.0, **scene_kwargs
    )
    sample_index = _lowest_contact_sample(template.geometry)
    if sample_index <= previous_sample:
        return None
    regions = np.asarray(template.geometry.contact_regions)
    if regions[sample_index] != regions[previous_sample]:
        return None
    arc = 0.5 * (
        _contour_arc_length(previous_points_hip_xz_m, previous_sample, sample_index)
        + _contour_arc_length(
            template.geometry.points_hip_xz_m, previous_sample, sample_index
        )
    )
    if arc <= 1e-12:
        return None
    return template, sample_index, arc


def _solve_flat_roll_rotation(
    theta_rad: float,
    previous_beta_rad: float,
    previous_sample: int,
    previous_points_hip_xz_m: np.ndarray,
    scene_kwargs: dict,
    *,
    target_advance_m: float,
    beta_direction: float,
    coarse_step_rad: float,
    max_rotation_rad: float,
    iterations: int,
    rotation_resolution_rad: float,
) -> tuple[SingleLegRollingScene2D, int, float, float] | None:
    """Choose how far to rotate so the no-slip advance reaches a target.

    The bisection only selects the rotation magnitude.  The reported advance
    always comes back out of the sampled rim geometry, so a rotation that
    overshoots the target is reported with its true arc length instead of
    being clipped.

    The achieved advance is piecewise constant in the rotation, because the
    support sample is an integer rim index.  The bisection therefore stops
    once the bracket is finer than one sample of rotation; refining further
    would only cost scene builds without moving the contact.
    """

    def evaluate(rotation: float):
        beta = previous_beta_rad + beta_direction * rotation
        return _flat_roll_template(
            theta_rad, beta, previous_sample, previous_points_hip_xz_m, scene_kwargs
        )

    low = 0.0
    high = coarse_step_rad
    solved = None
    while high <= max_rotation_rad:
        probe = evaluate(high)
        if probe is not None and probe[2] >= target_advance_m:
            solved = probe
            break
        if probe is not None:
            low = high
        high += coarse_step_rad
    if solved is None:
        return None
    for _ in range(iterations):
        if high - low <= rotation_resolution_rad:
            break
        middle = 0.5 * (low + high)
        probe = evaluate(middle)
        if probe is None or probe[2] < target_advance_m:
            low = middle
        else:
            high = middle
            solved = probe
    template, sample_index, arc = solved
    return template, sample_index, arc, high


def _pivot_scene(
    theta_rad: float,
    beta_rad: float,
    pinned_sample: int,
    corner_world_xz_m: np.ndarray,
    scene_kwargs: dict,
) -> SingleLegRollingScene2D:
    """Rotate the leg about the material sample pinned on the trailing corner.

    ``beta`` is a rigid rotation of the leg about the hip at fixed ``theta``,
    so re-placing the same material sample on the same world point turns that
    beta change into a pure rotation about the corner.  This is why the
    contact material point does not migrate during the corner transition.
    """

    template = build_single_leg_rolling_scene_2d(
        theta_rad, beta_rad, 0.0, 0.0, **scene_kwargs
    )
    return _translated_scene_with_sample_on_target_2d(
        template, pinned_sample, corner_world_xz_m
    )


def _ground_clearance(scene: SingleLegRollingScene2D) -> tuple[float, int]:
    """Height of the lowest physical rim sample above the lower ground."""

    indices = _contact_sample_indices(scene.geometry)
    points = scene.geometry.points_world_xz_m
    lowest = int(indices[int(np.argmin(points[indices, 1]))])
    return float(points[lowest, 1] - scene.terrain.ground_height_m), lowest


# --------------------------------------------------------------------------
# Step 7R main simulation
# --------------------------------------------------------------------------


def run_trailing_edge_roll_down_2d(
    start,
    *,
    obstacle_width_m: float | None = None,
    beta_direction: float = -1.0,
    top_contact_step_m: float = 0.002,
    ground_contact_step_m: float = 0.002,
    pivot_beta_step_rad: float = np.deg2rad(1.0),
    ground_roll_distance_m: float = 0.02,
    corner_tolerance_m: float = 1e-3,
    contact_tolerance_m: float = 1e-3,
    collision_tolerance_m: float = 1e-3,
    touchdown_tolerance_m: float = 5e-5,
    surface_offset_m: float = 1e-9,
    sample_match_tolerance: int = 3,
    corner_exclusion_m: float = 0.01,
    max_steps: int = 500,
    max_pivot_rotation_rad: float = np.deg2rad(180.0),
    max_flat_rotation_rad: float = np.deg2rad(30.0),
    bisection_iterations: int = 24,
) -> TrailingEdgeRollDownResult2D:
    """Roll a leg-wheel off the trailing edge of a rectangular obstacle.

    ``start`` is either a successful :class:`ForwardRollingResult2D` from Step
    4.5 or an explicit :class:`RollDownStartState2D`.  ``obstacle_width_m``
    may shorten or lengthen the obstacle top relative to the start state; the
    new trailing edge must still lie ahead of the entry contact point, which
    is checked rather than assumed.

    The three regimes are modelled separately on purpose.  Rolling on the flat
    top and on the lower ground migrates the contact material point and moves
    the world contact point by the swept rim arc length.  The corner is a
    point, so the corner transition instead pins one material sample and turns
    the leg about it: the world contact point stays put while the hip travels.
    Roll-down is therefore not the mirror image of roll-up.
    """

    if isinstance(start, ForwardRollingResult2D):
        start = roll_down_start_state_from_rolling_result(start)
    if not isinstance(start, RollDownStartState2D):
        raise TypeError("start must be a RollDownStartState2D or ForwardRollingResult2D.")
    beta_direction = _finite(beta_direction, "beta_direction")
    if not np.isclose(abs(beta_direction), 1.0):
        raise ValueError("beta_direction must be +1 or -1.")
    top_contact_step = _finite(top_contact_step_m, "top_contact_step_m")
    ground_contact_step = _finite(ground_contact_step_m, "ground_contact_step_m")
    pivot_step = abs(_finite(pivot_beta_step_rad, "pivot_beta_step_rad"))
    ground_roll_distance = _finite(ground_roll_distance_m, "ground_roll_distance_m")
    if min(top_contact_step, ground_contact_step, pivot_step) <= 0.0:
        raise ValueError("contact steps and the pivot step must be positive.")
    if ground_roll_distance < 0.0:
        raise ValueError("ground_roll_distance_m must be non-negative.")
    if max_steps <= 0 or bisection_iterations <= 0:
        raise ValueError("max_steps and bisection_iterations must be positive.")
    # Terrain penetration is detected with zero tolerance, so a support sample
    # placed exactly on a surface can be flagged by floating-point noise in a
    # neighbouring sample.  Every placement is lifted by this offset instead;
    # it is nine orders of magnitude below the contact tolerance, so the
    # sample still registers as a contact candidate.
    surface_offset = _finite(surface_offset_m, "surface_offset_m")
    if surface_offset < 0.0 or surface_offset >= contact_tolerance_m:
        raise ValueError("surface_offset_m must be in [0, contact_tolerance_m).")

    scene_kwargs = dict(start.scene_kwargs)
    # One rim sample of rotation; bisecting finer than this cannot change the
    # integer support sample and therefore cannot change the advance.
    rotation_resolution = np.deg2rad(180.0) / max(int(scene_kwargs["arc_samples"]), 2)
    if obstacle_width_m is not None:
        scene_kwargs["obstacle_width_m"] = _finite(obstacle_width_m, "obstacle_width_m")
    ground_height = float(scene_kwargs["ground_height_m"])
    obstacle_x_start = float(scene_kwargs["obstacle_x_start_m"])
    obstacle_height = float(scene_kwargs["obstacle_height_m"])
    top_z = ground_height + obstacle_height
    x_max = obstacle_x_start + float(scene_kwargs["obstacle_width_m"])
    corner = np.array([x_max, top_z], dtype=float)

    entry_contact = np.asarray(start.contact_point_world_xz_m, dtype=float)
    if entry_contact[0] >= x_max - corner_tolerance_m:
        raise ValueError(
            "the trailing edge must lie ahead of the roll-down entry contact; "
            f"entry contact x={entry_contact[0]:.4f} m, trailing edge x={x_max:.4f} m."
        )

    frames: list[RollDownFrame2D] = []
    state = {
        "rotation": 0.0,
        "advance": 0.0,
        "hip0": None,
        "min_back_clearance": None,
    }

    def append_frame(
        scene,
        query_result,
        candidate,
        *,
        phase,
        active_sample,
        corner_pinned=False,
        ground_supported=False,
        accepted=True,
        failure_reason=None,
    ) -> RollDownFrame2D:
        hip = np.asarray(scene.hip_pose.position_world_xz_m, dtype=float)
        if state["hip0"] is None:
            state["hip0"] = hip.copy()
        hip0 = state["hip0"]
        clearance, lowest = _ground_clearance(scene)
        back_clearance = _back_face_clearance(
            scene, corner_exclusion_m=corner_exclusion_m
        )
        if back_clearance is not None:
            previous = state["min_back_clearance"]
            state["min_back_clearance"] = (
                back_clearance if previous is None else min(previous, back_clearance)
            )
        contact_point = None
        alpha = None
        active_rim = None
        surface_id = None
        status = query_result.primary_status
        if candidate is not None:
            contact_point = tuple(float(v) for v in candidate.point_world_xz_m)
            alpha = float(candidate.alpha_rad)
            active_rim = candidate.rim.value
            surface_id = candidate.terrain_surface_id
        remaining_arc = _remaining_rim_budget_rad(scene.geometry)
        frame = RollDownFrame2D(
            step=len(frames),
            phase=phase,
            hip_x_m=float(hip[0]),
            hip_z_m=float(hip[1]),
            hip_forward_displacement_m=float(hip[0] - hip0[0]),
            hip_vertical_displacement_m=float(hip[1] - hip0[1]),
            theta_rad=float(scene.theta_rad),
            beta_rad=float(scene.beta_rad),
            accumulated_rotation_rad=float(state["rotation"]),
            active_rim=active_rim,
            active_sample_index=None if active_sample is None else int(active_sample),
            alpha_rad=alpha,
            contact_point_world_xz_m=contact_point,
            terrain_surface_id=surface_id,
            trailing_edge_distance_m=(
                None if contact_point is None else float(contact_point[0] - x_max)
            ),
            contact_advance_m=float(state["advance"]),
            lowest_point_world_xz_m=tuple(
                float(v) for v in scene.geometry.points_world_xz_m[lowest]
            ),
            ground_clearance_m=float(clearance),
            back_face_clearance_m=back_clearance,
            remaining_rim_arc_rad=remaining_arc,
            corner_pinned=bool(corner_pinned),
            ground_supported=bool(ground_supported),
            valid_contact=bool(accepted and candidate is not None),
            collision=bool(query_result.collision),
            status=status,
            accepted=bool(accepted),
            failure_reason=failure_reason,
            scene=scene,
            query_result=query_result,
        )
        frames.append(frame)
        return frame

    def query(scene):
        return query_single_leg_rolling_scene_2d(
            scene,
            contact_tolerance_m=contact_tolerance_m,
            collision_tolerance_m=collision_tolerance_m,
        )

    def fail(scene, query_result, phase, reason, *, active_sample=None):
        append_frame(
            scene,
            query_result,
            None,
            phase=PHASE_FAILED,
            active_sample=active_sample,
            accepted=False,
            failure_reason=reason,
        )
        return TrailingEdgeRollDownResult2D(
            frames=tuple(frames),
            success=False,
            failure_phase=phase,
            failure_reason=reason,
            top_roll_success=state.get("top_roll_success", False),
            corner_transition_success=state.get("corner_success", False),
            roll_down_success=state.get("roll_down_success", False),
            ground_contact_success=state.get("ground_success", False),
            ground_roll_success=False,
            trailing_corner_world_xz_m=(float(corner[0]), float(corner[1])),
            corner_pivot_rotation_rad=state.get("pivot_rotation"),
            hip_drop_at_touchdown_m=state.get("hip_drop"),
            ground_contact_point_world_xz_m=state.get("ground_point"),
            ground_contact_rim=state.get("ground_rim"),
            ground_contact_alpha_rad=state.get("ground_alpha"),
            remaining_rim_arc_rad_at_touchdown=state.get("ground_remaining_arc"),
            ground_roll_distance_m=ground_roll_distance,
            ground_roll_distance_achieved_m=state.get("ground_rolled", 0.0),
            minimum_back_face_clearance_m=state["min_back_clearance"],
        )

    # ---------------- Phase A: no-slip rolling to the trailing corner -------
    theta = start.theta_rad
    beta = start.beta_rad
    sample = int(start.active_sample_index)
    contact = entry_contact.copy()

    entry_template = build_single_leg_rolling_scene_2d(theta, beta, 0.0, 0.0, **scene_kwargs)
    entry_scene = _translated_scene_with_sample_on_target_2d(
        entry_template, sample, contact + np.array([0.0, surface_offset])
    )
    entry_query = query(entry_scene)
    entry_candidate = _candidate_for_sample(entry_query, sample)
    if entry_query.collision or entry_candidate is None:
        return fail(entry_scene, entry_query, PHASE_TOP_ROLL,
                    "ROLL_DOWN_ENTRY_STATE_INVALID", active_sample=sample)
    append_frame(entry_scene, entry_query, entry_candidate,
                 phase=PHASE_TOP_ROLL, active_sample=sample)
    previous_points = entry_template.geometry.points_hip_xz_m

    corner_landing_error = None
    while len(frames) < max_steps:
        remaining = x_max - contact[0]
        if remaining <= corner_tolerance_m:
            break
        target_advance = min(top_contact_step, remaining)
        solved = _solve_flat_roll_rotation(
            theta, beta, sample, previous_points, scene_kwargs,
            target_advance_m=target_advance,
            beta_direction=beta_direction,
            coarse_step_rad=pivot_step,
            max_rotation_rad=max_flat_rotation_rad,
            iterations=bisection_iterations,
            rotation_resolution_rad=rotation_resolution,
        )
        if solved is None:
            remaining_arc = _remaining_rim_budget_rad(frames[-1].scene.geometry)
            reason = (
                "RIM_ARC_EXHAUSTED_BEFORE_TRAILING_EDGE"
                if remaining_arc < np.deg2rad(2.0)
                else "NO_LEGAL_TOP_ROLL_CONTINUATION"
            )
            return fail(frames[-1].scene, frames[-1].query_result,
                        PHASE_TOP_ROLL, reason, active_sample=sample)
        template, next_sample, arc, rotation = solved
        reaches_corner = contact[0] + arc >= x_max - corner_tolerance_m
        if reaches_corner:
            target = corner.copy()
            corner_landing_error = float(contact[0] + arc - x_max)
        else:
            target = np.array([contact[0] + arc, top_z], dtype=float)
        scene = _translated_scene_with_sample_on_target_2d(
            template, next_sample, target + np.array([0.0, surface_offset])
        )
        query_result = query(scene)
        candidate = _candidate_for_sample(query_result, next_sample)
        if query_result.collision or candidate is None or (
            abs(candidate.sample_index - next_sample) > sample_match_tolerance
        ):
            return fail(scene, query_result, PHASE_TOP_ROLL,
                        "NO_LEGAL_TOP_ROLL_CONTINUATION", active_sample=next_sample)
        state["rotation"] += rotation
        state["advance"] += float(target[0] - contact[0])
        theta, beta, sample = float(scene.theta_rad), float(scene.beta_rad), next_sample
        contact = np.asarray(target, dtype=float)
        previous_points = template.geometry.points_hip_xz_m
        append_frame(scene, query_result, candidate,
                     phase=PHASE_TOP_ROLL, active_sample=sample)
        if reaches_corner:
            break
    else:
        return fail(frames[-1].scene, frames[-1].query_result, PHASE_TOP_ROLL,
                    "MAX_STEPS_BEFORE_TRAILING_EDGE", active_sample=sample)

    if abs(contact[0] - x_max) > corner_tolerance_m:
        return fail(frames[-1].scene, frames[-1].query_result, PHASE_TOP_ROLL,
                    "TRAILING_EDGE_NOT_REACHED", active_sample=sample)
    state["top_roll_success"] = True
    state["corner_landing_error"] = corner_landing_error

    # ---------------- Phase B: corner pivot and descent ---------------------
    # A sharp corner supports the leg at a point, so the leg turns about the
    # pinned material sample.  The world contact point stays on the corner and
    # ``contact_advance_m`` therefore does not grow: all forward progress in
    # this phase is hip travel, not contact travel.
    pin = sample
    pivot_beta0 = beta
    pivot_hip_z0 = frames[-1].hip_z_m
    pivot_rotation = 0.0
    touchdown = None

    pivot_target = corner + np.array([0.0, surface_offset])

    def pivot_at(rotation: float):
        return _pivot_scene(
            theta, pivot_beta0 + beta_direction * rotation, pin, pivot_target,
            scene_kwargs,
        )

    while len(frames) < max_steps and pivot_rotation < max_pivot_rotation_rad:
        next_rotation = min(pivot_rotation + pivot_step, max_pivot_rotation_rad)
        probe = pivot_at(next_rotation)
        probe_clearance, _ = _ground_clearance(probe)
        if probe_clearance <= touchdown_tolerance_m:
            # Converge from the clear side: the accepted touchdown rotation
            # must leave the ground sample strictly above the ground, because
            # terrain penetration is rejected with zero tolerance.
            low, high = pivot_rotation, next_rotation
            for _ in range(bisection_iterations):
                middle = 0.5 * (low + high)
                clearance, _ = _ground_clearance(pivot_at(middle))
                if clearance > 0.0:
                    low = middle
                else:
                    high = middle
            touchdown = low
            break
        scene = probe
        query_result = query(scene)
        candidate = _candidate_for_sample(query_result, pin)
        if query_result.collision:
            return fail(scene, query_result, PHASE_ROLL_DOWN,
                        "COLLISION_DURING_CORNER_PIVOT", active_sample=pin)
        if candidate is None or abs(candidate.sample_index - pin) > sample_match_tolerance:
            return fail(scene, query_result, PHASE_ROLL_DOWN,
                        "LOST_TRAILING_CORNER_CONTACT", active_sample=pin)
        state["rotation"] += next_rotation - pivot_rotation
        lowest_z = scene.geometry.points_world_xz_m[
            _lowest_contact_sample(scene.geometry), 1
        ]
        phase = (
            PHASE_CORNER
            if lowest_z >= top_z - corner_tolerance_m
            else PHASE_ROLL_DOWN
        )
        append_frame(scene, query_result, candidate,
                     phase=phase, active_sample=pin, corner_pinned=True)
        if phase == PHASE_CORNER:
            state["corner_success"] = True
        else:
            state["roll_down_success"] = True
        pivot_rotation = next_rotation

    if touchdown is None:
        return fail(frames[-1].scene, frames[-1].query_result, PHASE_ROLL_DOWN,
                    "MAX_PIVOT_ROTATION_BEFORE_GROUND_CONTACT", active_sample=pin)

    # ---------------- Phase B': lower-ground touchdown ----------------------
    scene = pivot_at(touchdown)
    query_result = query(scene)
    all_points = scene.geometry.points_world_xz_m
    lowest_any = int(np.argmin(all_points[:, 1]))
    ground_sample = _lowest_contact_sample(scene.geometry)
    if np.asarray(scene.geometry.contact_regions)[lowest_any] == "non_contact_region" and (
        all_points[lowest_any, 1] < all_points[ground_sample, 1] - 1e-9
    ):
        return fail(scene, query_result, PHASE_GROUND_CONTACT,
                    "RIM_SEAM_REACHES_GROUND_FIRST", active_sample=pin)
    if query_result.collision:
        return fail(scene, query_result, PHASE_GROUND_CONTACT,
                    "COLLISION_AT_LOWER_GROUND_TOUCHDOWN", active_sample=ground_sample)
    ground_surface_id = scene.terrain.ground_surface_id
    ground_candidate = _candidate_for_sample(
        query_result, ground_sample, surface_ids=(ground_surface_id,)
    )
    if ground_candidate is None or (
        abs(ground_candidate.sample_index - ground_sample) > sample_match_tolerance
    ):
        return fail(scene, query_result, PHASE_GROUND_CONTACT,
                    "NO_LEGAL_LOWER_GROUND_CONTACT", active_sample=ground_sample)
    state["rotation"] += touchdown - pivot_rotation
    state["corner_success"] = True
    state["roll_down_success"] = True
    state["ground_success"] = True
    state["pivot_rotation"] = float(touchdown)
    state["hip_drop"] = float(scene.hip_pose.position_world_xz_m[1] - pivot_hip_z0)
    state["ground_point"] = tuple(float(v) for v in ground_candidate.point_world_xz_m)
    state["ground_rim"] = ground_candidate.rim.value
    state["ground_alpha"] = float(ground_candidate.alpha_rad)
    state["ground_remaining_arc"] = _remaining_rim_arc_rad(
        scene.geometry, ground_sample
    )
    append_frame(scene, query_result, ground_candidate,
                 phase=PHASE_GROUND_CONTACT, active_sample=ground_sample,
                 corner_pinned=True, ground_supported=True)

    theta = float(scene.theta_rad)
    beta = float(scene.beta_rad)
    sample = ground_sample
    contact = np.asarray(ground_candidate.point_world_xz_m, dtype=float)
    previous_points = build_single_leg_rolling_scene_2d(
        theta, beta, 0.0, 0.0, **scene_kwargs
    ).geometry.points_hip_xz_m
    state["ground_rolled"] = 0.0

    # ---------------- Phase C: no-slip rolling away on lower ground ---------
    ground_roll_reason = None
    while state["ground_rolled"] < ground_roll_distance - 1e-12:
        if len(frames) >= max_steps:
            ground_roll_reason = "MAX_STEPS_DURING_GROUND_ROLL"
            break
        target_advance = min(
            ground_contact_step, ground_roll_distance - state["ground_rolled"]
        )
        solved = _solve_flat_roll_rotation(
            theta, beta, sample, previous_points, scene_kwargs,
            target_advance_m=target_advance,
            beta_direction=beta_direction,
            coarse_step_rad=pivot_step,
            max_rotation_rad=max_flat_rotation_rad,
            iterations=bisection_iterations,
            rotation_resolution_rad=rotation_resolution,
        )
        if solved is None:
            ground_roll_reason = "RIM_ARC_EXHAUSTED_DURING_GROUND_ROLL"
            break
        template, next_sample, arc, rotation = solved
        target = np.array([contact[0] + arc, ground_height], dtype=float)
        candidate_scene = _translated_scene_with_sample_on_target_2d(
            template, next_sample, target + np.array([0.0, surface_offset])
        )
        candidate_query = query(candidate_scene)
        candidate = _candidate_for_sample(
            candidate_query, next_sample, surface_ids=(ground_surface_id,)
        )
        if candidate_query.collision or candidate is None or (
            abs(candidate.sample_index - next_sample) > sample_match_tolerance
        ):
            ground_roll_reason = "NO_LEGAL_GROUND_ROLL_CONTINUATION"
            break
        state["rotation"] += rotation
        state["advance"] += arc
        state["ground_rolled"] += arc
        theta = float(candidate_scene.theta_rad)
        beta = float(candidate_scene.beta_rad)
        sample = next_sample
        contact = target
        previous_points = template.geometry.points_hip_xz_m
        append_frame(candidate_scene, candidate_query, candidate,
                     phase=PHASE_GROUND_ROLL, active_sample=sample,
                     ground_supported=True)

    ground_roll_success = ground_roll_reason is None
    return TrailingEdgeRollDownResult2D(
        frames=tuple(frames),
        success=True,
        failure_phase=None if ground_roll_success else PHASE_GROUND_ROLL,
        failure_reason=ground_roll_reason,
        top_roll_success=True,
        corner_transition_success=bool(state.get("corner_success", False)),
        roll_down_success=bool(state.get("roll_down_success", False)),
        ground_contact_success=True,
        ground_roll_success=ground_roll_success,
        trailing_corner_world_xz_m=(float(corner[0]), float(corner[1])),
        corner_pivot_rotation_rad=state["pivot_rotation"],
        hip_drop_at_touchdown_m=state["hip_drop"],
        ground_contact_point_world_xz_m=state["ground_point"],
        ground_contact_rim=state["ground_rim"],
        ground_contact_alpha_rad=state["ground_alpha"],
        remaining_rim_arc_rad_at_touchdown=state["ground_remaining_arc"],
        ground_roll_distance_m=ground_roll_distance,
        ground_roll_distance_achieved_m=float(state["ground_rolled"]),
        minimum_back_face_clearance_m=state["min_back_clearance"],
    )


# --------------------------------------------------------------------------
# Tables and CSV
# --------------------------------------------------------------------------

PHASE_COLORS = {
    PHASE_TOP_ROLL: "#2563eb",
    PHASE_CORNER: "#f59e0b",
    PHASE_ROLL_DOWN: "#dc2626",
    PHASE_GROUND_CONTACT: "#16a34a",
    PHASE_GROUND_ROLL: "#0d9488",
    PHASE_FAILED: "#6b7280",
}


def roll_down_frame_rows(result: TrailingEdgeRollDownResult2D) -> list[dict]:
    """One row per simulated frame, ready for a DataFrame or CSV.

    Accepts any result carrying ``frames`` of :class:`RollDownFrame2D`, so the
    left-rim descent of the revised traversal produces the same table shape
    instead of a parallel one.
    """

    if not hasattr(result, "frames"):
        raise TypeError("result must expose a frames sequence.")
    rows = []
    for frame in result.frames:
        contact = frame.contact_point_world_xz_m
        rows.append(
            {
                "step": frame.step,
                "phase": frame.phase,
                "hip_x_m": frame.hip_x_m,
                "hip_z_m": frame.hip_z_m,
                "hip_forward_displacement_m": frame.hip_forward_displacement_m,
                "hip_vertical_displacement_m": frame.hip_vertical_displacement_m,
                "theta_deg": frame.theta_deg,
                "beta_deg": frame.beta_deg,
                "accumulated_rotation_deg": float(
                    np.rad2deg(frame.accumulated_rotation_rad)
                ),
                "active_rim": frame.active_rim,
                "active_sample_index": frame.active_sample_index,
                "alpha_deg": (
                    None if frame.alpha_rad is None else float(np.rad2deg(frame.alpha_rad))
                ),
                "contact_x_m": None if contact is None else contact[0],
                "contact_z_m": None if contact is None else contact[1],
                "terrain_surface": frame.terrain_surface_id,
                "trailing_edge_distance_m": frame.trailing_edge_distance_m,
                "contact_advance_m": frame.contact_advance_m,
                "lowest_point_x_m": frame.lowest_point_world_xz_m[0],
                "lowest_point_z_m": frame.lowest_point_world_xz_m[1],
                "ground_clearance_m": frame.ground_clearance_m,
                "back_face_clearance_m": frame.back_face_clearance_m,
                "remaining_rim_arc_deg": (
                    None
                    if frame.remaining_rim_arc_rad is None
                    else float(np.rad2deg(frame.remaining_rim_arc_rad))
                ),
                "corner_pinned": frame.corner_pinned,
                "ground_supported": frame.ground_supported,
                "valid_contact": frame.valid_contact,
                "collision": frame.collision,
                # The combined ``collision`` flag is what acceptance uses; these
                # split it so a rejected frame says *what* was wrong.
                "penetration": bool(frame.query_result.geometry_penetrations),
                "link_collision": bool(frame.query_result.link_collisions),
                "vertical_face_collision": bool(frame.query_result.collisions),
                "status": frame.status.value,
                "accepted": frame.accepted,
                "failure_reason": frame.failure_reason,
            }
        )
    return rows


def roll_down_summary_row(result: TrailingEdgeRollDownResult2D) -> dict:
    """One-row summary of a trailing-edge roll-down attempt."""

    if not isinstance(result, TrailingEdgeRollDownResult2D):
        raise TypeError("result must be a TrailingEdgeRollDownResult2D.")
    final = result.final_frame
    ground = result.ground_contact_point_world_xz_m
    return {
        "success": result.success,
        "failure_phase": result.failure_phase,
        "failure_reason": result.failure_reason,
        "top_roll_success": result.top_roll_success,
        "corner_transition_success": result.corner_transition_success,
        "roll_down_success": result.roll_down_success,
        "ground_contact_success": result.ground_contact_success,
        "ground_roll_success": result.ground_roll_success,
        "trailing_corner_x_m": result.trailing_corner_world_xz_m[0],
        "trailing_corner_z_m": result.trailing_corner_world_xz_m[1],
        "corner_pivot_rotation_deg": (
            None
            if result.corner_pivot_rotation_rad is None
            else float(np.rad2deg(result.corner_pivot_rotation_rad))
        ),
        "total_rotation_deg": float(np.rad2deg(final.accumulated_rotation_rad)),
        "total_contact_advance_m": final.contact_advance_m,
        "hip_forward_displacement_m": final.hip_forward_displacement_m,
        "hip_vertical_displacement_m": final.hip_vertical_displacement_m,
        "hip_drop_at_touchdown_m": result.hip_drop_at_touchdown_m,
        "ground_contact_x_m": None if ground is None else ground[0],
        "ground_contact_z_m": None if ground is None else ground[1],
        "ground_contact_rim": result.ground_contact_rim,
        "ground_contact_alpha_deg": (
            None
            if result.ground_contact_alpha_rad is None
            else float(np.rad2deg(result.ground_contact_alpha_rad))
        ),
        "remaining_rim_arc_deg_at_touchdown": (
            None
            if result.remaining_rim_arc_rad_at_touchdown is None
            else float(np.rad2deg(result.remaining_rim_arc_rad_at_touchdown))
        ),
        "ground_roll_distance_m": result.ground_roll_distance_m,
        "ground_roll_distance_achieved_m": result.ground_roll_distance_achieved_m,
        "minimum_back_face_clearance_m": result.minimum_back_face_clearance_m,
        "frame_count": len(result.frames),
    }


def write_trailing_edge_roll_down_csv(
    result: TrailingEdgeRollDownResult2D,
    summary_path,
    frames_path,
) -> tuple[Path, Path]:
    """Persist the summary row and every simulated frame."""

    summary_path = Path(summary_path)
    frames_path = Path(frames_path)
    summary = roll_down_summary_row(result)
    rows = roll_down_frame_rows(result)
    for path, payload in ((summary_path, [summary]), (frames_path, rows)):
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(payload[0].keys()))
            writer.writeheader()
            writer.writerows(payload)
    return summary_path, frames_path


# --------------------------------------------------------------------------
# Visualization
# --------------------------------------------------------------------------


def plot_trailing_edge_roll_down_2d(
    result: TrailingEdgeRollDownResult2D,
    *,
    axes=None,
):
    """Contact/hip paths and the rim-arc budget for one roll-down attempt.

    The left panel shows why the corner transition is not a rolling step: the
    contact point stops while the hip keeps travelling.  The right panel shows
    the finite rim-arc budget that the whole traversal has to fit inside.
    """

    if not isinstance(result, TrailingEdgeRollDownResult2D):
        raise TypeError("result must be a TrailingEdgeRollDownResult2D.")
    owns_figure = axes is None
    if owns_figure:
        _, axes = plt.subplots(1, 2, figsize=(13.5, 5.4))
    left, right = axes

    reference = result.frames[0].scene
    from legwheel.planners.hybrid import plot_terrain_profile_2d

    xs = [frame.hip_x_m for frame in result.frames]
    contact_xs = [
        frame.contact_point_world_xz_m[0]
        for frame in result.frames
        if frame.contact_point_world_xz_m is not None
    ]
    x_low = min(xs + contact_xs) - 0.06
    x_high = max(xs + contact_xs) + 0.06
    plot_terrain_profile_2d(reference.terrain, ax=left, x_limits_m=(x_low, x_high))

    for phase, color in PHASE_COLORS.items():
        frames = result.frames_in_phase(phase)
        points = [
            frame.contact_point_world_xz_m
            for frame in frames
            if frame.contact_point_world_xz_m is not None
        ]
        if points:
            array = np.asarray(points, dtype=float)
            left.plot(array[:, 0], array[:, 1], "o", color=color, markersize=5.0,
                      label=f"contact: {phase}")
    hip = np.asarray([[frame.hip_x_m, frame.hip_z_m] for frame in result.frames])
    left.plot(hip[:, 0], hip[:, 1], "--", color="#7c3aed", linewidth=1.6, label="hip path")
    corner = result.trailing_corner_world_xz_m
    left.plot([corner[0]], [corner[1]], "*", color="black", markersize=14,
              label="trailing corner", zorder=8)
    zs = [frame.hip_z_m for frame in result.frames]
    zs.extend(
        frame.contact_point_world_xz_m[1]
        for frame in result.frames
        if frame.contact_point_world_xz_m is not None
    )
    left.set_xlim(x_low, x_high)
    left.set_ylim(min(zs) - 0.05, max(zs) + 0.05)
    left.set_aspect("equal", adjustable="box")
    left.set_xlabel("world x [m]")
    left.set_ylabel("world z [m]")
    left.set_title(
        f"Step 7R trailing-edge roll-down (success={result.success})"
    )
    left.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3, fontsize=7.0)
    left.grid(True, alpha=0.3)

    steps = [frame.step for frame in result.frames]
    remaining = [
        np.nan if frame.remaining_rim_arc_rad is None
        else float(np.rad2deg(frame.remaining_rim_arc_rad))
        for frame in result.frames
    ]
    advance = [1000.0 * frame.contact_advance_m for frame in result.frames]
    right.plot(steps, remaining, "-", color="#dc2626", linewidth=1.8,
               label="remaining rim arc [deg]")
    right.plot(steps, advance, "-", color="#2563eb", linewidth=1.8,
               label="contact advance [mm]")
    previous_phase = None
    for frame in result.frames:
        if frame.phase != previous_phase:
            right.axvline(frame.step, color=PHASE_COLORS.get(frame.phase, "0.6"),
                          linewidth=1.0, alpha=0.55)
            right.text(frame.step, right.get_ylim()[1], f" {frame.phase}",
                       rotation=90, va="top", fontsize=6.5,
                       color=PHASE_COLORS.get(frame.phase, "0.4"))
            previous_phase = frame.phase
    right.set_xlabel("simulation step")
    right.set_title("Rim-arc budget (at the lowest sample) and contact advance")
    right.legend(loc="upper right", fontsize=8)
    right.grid(True, alpha=0.3)
    if owns_figure:
        left.figure.tight_layout()
    return axes


def plot_roll_down_key_frames_2d(
    result: TrailingEdgeRollDownResult2D,
    *,
    phases=(PHASE_TOP_ROLL, PHASE_CORNER, PHASE_ROLL_DOWN,
            PHASE_GROUND_CONTACT, PHASE_GROUND_ROLL),
):
    """Draw the last accepted frame of each phase that actually occurred.

    Accepts any result exposing ``frames`` / ``frames_in_phase``: Step 9R's
    left-rim descent is the same motion with different phase names, so it
    reuses this renderer instead of carrying a near-duplicate copy.
    """

    if not (hasattr(result, "frames") and hasattr(result, "frames_in_phase")):
        raise TypeError("result must expose frames and frames_in_phase.")
    selected = []
    for phase in phases:
        frames = [frame for frame in result.frames_in_phase(phase) if frame.accepted]
        if frames:
            selected.append((phase, frames[-1]))
    failed = [frame for frame in result.frames if not frame.accepted]
    if failed:
        selected.append((PHASE_FAILED, failed[-1]))
    if not selected:
        raise ValueError("the result contains no drawable frame.")
    figure, axes = plt.subplots(
        1, len(selected), figsize=(4.6 * len(selected), 4.4), squeeze=False
    )
    for ax, (phase, frame) in zip(axes[0], selected):
        plot_single_leg_rolling_scene_2d(
            frame.scene, ax=ax, query_result=frame.query_result
        )
        ax.set_title(
            f"{phase}\ntheta={frame.theta_deg:.1f} deg, beta={frame.beta_deg:.1f} deg",
            fontsize=9,
        )
        ax.legend().set_visible(False)
    figure.tight_layout()
    return figure, axes


def animate_trailing_edge_roll_down_2d(
    result: TrailingEdgeRollDownResult2D,
    *,
    interval_ms: int = 110,
    frame_stride: int = 1,
    repeat: bool = False,
    show: bool = False,
):
    """Animate one trailing-edge roll-down attempt.

    Accepts any result carrying ``frames`` of :class:`RollDownFrame2D` and a
    ``trailing_corner_world_xz_m``, so the left-rim descent of the revised
    traversal reuses this animation instead of copying it.
    """

    if not (
        hasattr(result, "frames") and hasattr(result, "trailing_corner_world_xz_m")
    ):
        raise TypeError(
            "result must expose frames and trailing_corner_world_xz_m."
        )
    if min(interval_ms, frame_stride) <= 0:
        raise ValueError("interval and stride must be positive.")
    display_frames = list(result.frames[::frame_stride])
    if display_frames[-1] is not result.frames[-1]:
        display_frames.append(result.frames[-1])
    all_points = np.vstack(
        [
            np.vstack(
                (
                    frame.scene.geometry.points_world_xz_m,
                    frame.scene.hip_pose.position_world_xz_m[None, :],
                )
            )
            for frame in display_frames
        ]
    )
    x_pad = max(0.04, 0.08 * float(np.ptp(all_points[:, 0])))
    z_pad = max(0.04, 0.08 * float(np.ptp(all_points[:, 1])))
    x_limits = (float(np.min(all_points[:, 0]) - x_pad),
                float(np.max(all_points[:, 0]) + x_pad))
    z_limits = (
        min(display_frames[0].scene.terrain.ground_height_m - 0.03,
            float(np.min(all_points[:, 1]) - z_pad)),
        float(np.max(all_points[:, 1]) + z_pad),
    )
    figure, ax = plt.subplots(figsize=(11, 5.5))
    corner = result.trailing_corner_world_xz_m

    def draw_frame(index: int):
        frame = display_frames[index]
        ax.clear()
        plot_single_leg_rolling_scene_2d(
            frame.scene, ax=ax, query_result=frame.query_result
        )
        ax.set_xlim(*x_limits)
        ax.set_ylim(*z_limits)
        hip_trace = np.asarray(
            [[item.hip_x_m, item.hip_z_m] for item in display_frames[: index + 1]],
            dtype=float,
        )
        ax.plot(hip_trace[:, 0], hip_trace[:, 1], "--", color="#7c3aed",
                linewidth=1.6, label="hip path")
        contact_trace = np.asarray(
            [
                item.contact_point_world_xz_m
                for item in display_frames[: index + 1]
                if item.contact_point_world_xz_m is not None
            ],
            dtype=float,
        )
        if contact_trace.size:
            ax.plot(contact_trace[:, 0], contact_trace[:, 1], ".",
                    color="#dc2626", markersize=4.5, label="contact path")
        ax.plot([corner[0]], [corner[1]], "*", color="black", markersize=13, zorder=9)
        ax.set_title(
            f"Step 7R {frame.phase}: theta={frame.theta_deg:.1f} deg, "
            f"beta={frame.beta_deg:.1f} deg, rim={frame.active_rim or 'none'}"
        )
        remaining = (
            "n/a" if frame.remaining_rim_arc_rad is None
            else f"{np.rad2deg(frame.remaining_rim_arc_rad):.1f} deg"
        )
        back = (
            "n/a" if frame.back_face_clearance_m is None
            else f"{frame.back_face_clearance_m:.4f} m"
        )
        ax.text(
            0.01, 0.02,
            f"contact advance={frame.contact_advance_m:.4f} m\n"
            f"hip dx={frame.hip_forward_displacement_m:+.4f} m, "
            f"dz={frame.hip_vertical_displacement_m:+.4f} m\n"
            f"rotation={np.rad2deg(frame.accumulated_rotation_rad):.1f} deg\n"
            f"ground clearance={frame.ground_clearance_m:.4f} m\n"
            f"back-face clearance={back}\n"
            f"remaining rim arc={remaining}\n"
            f"collision={frame.collision}, failure={frame.failure_reason or 'none'}",
            transform=ax.transAxes, fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.86, "edgecolor": "0.75"},
            zorder=16,
        )
        return tuple(ax.lines)

    animation = FuncAnimation(
        figure, draw_frame, frames=len(display_frames),
        interval=interval_ms, repeat=repeat, blit=False,
    )
    draw_frame(0)
    if show:
        plt.show()
    return animation


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--theta-climb-deg", type=float, default=60.0)
    parser.add_argument("--initial-beta-deg", type=float, default=-20.0)
    parser.add_argument("--hip-x-m", type=float, default=0.0)
    parser.add_argument("--hip-z-m", type=float, default=0.24)
    parser.add_argument("--obstacle-x-start-m", type=float, default=0.10)
    parser.add_argument("--obstacle-height-m", type=float, default=0.10)
    parser.add_argument("--roll-up-width-m", type=float, default=0.60)
    parser.add_argument("--roll-down-width-m", type=float, default=0.03)
    parser.add_argument("--top-roll-distance-m", type=float, default=0.02)
    parser.add_argument("--ground-roll-distance-m", type=float, default=0.02)
    parser.add_argument("--arc-samples", type=int, default=241)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    roll_up = run_forward_right_rim_roll_up_2d(
        candidate_theta_rad=np.deg2rad(args.theta_climb_deg),
        initial_beta_rad=np.deg2rad(args.initial_beta_deg),
        hip_x_m=args.hip_x_m,
        hip_z_m=args.hip_z_m,
        dx_m=0.002,
        top_roll_distance_m=args.top_roll_distance_m,
        max_forward_steps=120,
        beta_step_rad=-np.deg2rad(1.0),
        top_beta_step_rad=-np.deg2rad(0.5),
        obstacle_x_start_m=args.obstacle_x_start_m,
        obstacle_width_m=args.roll_up_width_m,
        obstacle_height_m=args.obstacle_height_m,
        arc_samples=args.arc_samples,
    )
    print(f"roll-up success={roll_up.success} reason={roll_up.failure_reason}")
    if not roll_up.success:
        return
    result = run_trailing_edge_roll_down_2d(
        roll_up,
        obstacle_width_m=args.roll_down_width_m,
        ground_roll_distance_m=args.ground_roll_distance_m,
    )
    for key, value in roll_down_summary_row(result).items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
