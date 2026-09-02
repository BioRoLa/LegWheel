"""Day 10--11 Step 2b: can a swing land *already* in ``LEFT_RIM_READY``?

Strategy #3 of the 2x2 (``SWING_UP + ROLL_DOWN``) exists only if the answer is
yes.  Its whole appeal is that it never pays ``L_transition`` -- the 0.20--0.27 m
of top run a rolling ascent spends getting from its right-rim exit to the
left-rim handover, which is where Day 6--7 Step 12R's minimum top length comes
from.  A swing is contact-state-to-contact-state, so in principle it can be
*asked* to touch down in the state the descent needs and skip that budget
entirely.  Nothing in Day 8--9 ever asked it to: every showcase lands at
``theta = 60 deg`` on the foot rim, and ``theta = 17 deg`` on the left rim is
the most retracted pose the leg has.

Spec §2.8 splits the question into three, and they are answered here by three
different kinds of measurement rather than by one verdict:

``(b) is the rim reachable``
    a static pose sweep over beta -- cheap, and it either has a window or it
    does not.  :func:`left_rim_beta_window_2d`.

``(c) how close is the alpha seam``
    the same sweep, reported as a distance to each end of the left rim's arc.
    Day 8--9 §26.5(2) measured a 162 mm contact jump across the +/-180 deg
    seam, so a landing that sits near it is not usable even when it is legal.

``(a) is the swing feasible, and what does it cost the body``
    a full ``generate_swing_2d`` per cell.  :func:`minimum_swing_to_left_rim_ready_2d`.

**The finding that changes the contract.**  Step 2's ascent prices a swing in
``min_hip_lift`` -- how much *higher* than the nominal standing pose the hip
must be.  That knob does not exist here.  Fixing the touchdown contact state to
(point, left rim, alpha) *and* requiring ``theta = 17 deg`` leaves the hip no
freedom at all: the leg at 17 degrees is very nearly a circle, so the hip must
sit exactly one wheel radius above the top.  Lifting it would not make the
landing easier, it would make it a landing at some larger theta -- a different
contact state, and one that fails the ``LEFT_RIM_READY`` precondition.  The
demand is therefore an *equality*, recorded as
:attr:`BodyRequirementKind.PINNED`, and the only knob left is ``liftoff_rise``.

**Why the precondition is imported and not re-stated.**  ``LEFT_RIM_READY`` is
evaluated by calling Day 6--7's own ``_corner_readiness_failure`` on a frame
built from the swing's touchdown pose, and the descent is verified by running
Day 6--7's own ``run_left_rim_roll_down_2d`` from it.  A second definition of
readiness on the swing side is exactly the failure mode Step 0 exists to
prevent: the two sides would drift and the comparison would be meaningless.
The adapter that lets a swing touchdown enter Step 9R is
:func:`left_rim_ready_from_landing_2d`.

Run from the repository root via ``day10_11_step2b_driver.py``.
"""

from __future__ import annotations

import os
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
from typing import Iterable, Sequence

import numpy as np

from legwheel.planners.hybrid import RimId

from .cartesian_swing_contract_2d import (
    SwingFailure,
    active_contact_candidate_2d,
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
    approach_hip_x_for_clearance_2d,
    standing_scene_2d,
)
from .day10_11_swing_sweep_2d import SwingGridSettings2D
from .single_leg_rolling_scene_2d import (
    RetractResetFrame2D,
    SingleLegRollingScene2D,
    build_single_leg_rolling_scene_2d,
    query_single_leg_rolling_scene_2d,
)
from .right_up_left_down_traversal_2d import (
    WheelModeTransitionResult2D,
    _corner_readiness_failure,
    _wheel_radius_at_theta,
    run_left_rim_roll_down_2d,
)

__all__ = [
    "LEFT_RIM_READY_THETA_RAD",
    "LeftRimBetaWindow2D",
    "LeftRimBetaSample2D",
    "left_rim_beta_window_2d",
    "run_beta_windows_2d",
    "run_beta_windows_2d_at_thetas",
    "predicted_pivot_deg_2d",
    "LandingBetaChoice2D",
    "choose_landing_beta_2d",
    "left_rim_landing_scene_2d",
    "LeftRimReadiness2D",
    "left_rim_ready_from_landing_2d",
    "LeftRimLandingCell2D",
    "minimum_swing_to_left_rim_ready_2d",
    "run_left_rim_cells_2d",
    "left_rim_rows",
    "beta_window_rows",
]

#: Day 6--7 Step 8R's wheel-mode target, restated here only as a default.  The
#: number itself lives in ``run_wheel_mode_transition_to_corner_2d``.
LEFT_RIM_READY_THETA_RAD: float = float(np.deg2rad(17.0))

#: The two ends of the left rim's arc, in the alpha convention of
#: ``LEGACY_SURFACE_ALPHA_LIMITS_DEG['upper_tyre_l']``.  Both are seams: -180
#: is the one Day 8--9 §26.5(2) measured a 162 mm contact jump across, and -40
#: is the boundary the foot rim takes over at.
LEFT_RIM_ALPHA_LIMITS_DEG: tuple[float, float] = (-180.0, -40.0)


# --------------------------------------------------------------------------
# (b) and (c): where on the beta axis does the left rim carry the contact?
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class LeftRimBetaSample2D:
    """One beta, and everything the landing choice depends on.

    ``rim_budget_deg`` is measured the way Step 8R measures it -- the arc from
    the contact to the far end of the rim region *in the direction the samples
    run*, which is toward the -40 deg boundary.  Measured, not derived: the
    direction is a property of the geometry sampler, and assuming it would be
    exactly the kind of second definition Step 0 exists to prevent.
    """

    beta_deg: float
    alpha_deg: float
    seam_margin_deg: float
    rim_budget_deg: float
    hip_above_top_m: float

    def as_dict(self) -> dict:
        return {
            "beta_deg": self.beta_deg,
            "alpha_deg": self.alpha_deg,
            "seam_margin_deg": self.seam_margin_deg,
            "rim_budget_deg": self.rim_budget_deg,
            "hip_above_top_mm": self.hip_above_top_m * 1e3,
        }


def predicted_pivot_deg_2d(
    spec: SharedTerrainSpec2D, *, theta_rad: float = LEFT_RIM_READY_THETA_RAD
) -> float | None:
    """Step 8R's first-order corner pivot for this step height.

    Reproduces ``run_wheel_mode_transition_to_corner_2d``'s own prediction --
    ``arccos(1 - h / R)`` on the retracted leg's radius -- so the swing side
    asks for the same amount of rim the rolling side budgets for.
    """

    radius = _wheel_radius_at_theta(float(theta_rad), spec.rolling_obstacle.scene_kwargs)
    cosine = 1.0 - float(spec.height_m) / radius
    if not -1.0 <= cosine <= 1.0:
        return None
    return float(np.rad2deg(np.arccos(cosine)))


@dataclass(frozen=True)
class LeftRimBetaWindow2D:
    """The beta interval that puts the left rim on the ground at one theta.

    ``best_beta_rad`` maximises the distance to the *nearer* of the two alpha
    seams rather than to -180 alone.  Both ends are discontinuities: crossing
    -180 jumps the contact point, and crossing -40 hands the contact to the
    foot rim, which is no longer a left-rim descent at all.
    """

    theta_deg: float
    height_m: float
    sample_step_deg: float
    beta_min_deg: float | None
    beta_max_deg: float | None
    width_deg: float
    contiguous: bool
    best_beta_deg: float | None
    best_alpha_deg: float | None
    best_seam_margin_deg: float | None
    alpha_min_deg: float | None
    alpha_max_deg: float | None
    hip_above_top_m: float | None
    #: Alpha the rolling side actually arrives at, when it is known.  Kept as a
    #: field so the swing's freedom can be reported against the roll's habit
    #: rather than against nothing.
    rolling_arrival_alpha_deg: float | None = None
    #: Every sampled beta that put the left rim on the ground.  Kept so the
    #: landing choice can be made from measurements rather than from a formula.
    samples: tuple = ()

    def as_dict(self) -> dict:
        return {
            "theta_deg": self.theta_deg,
            "height_mm": self.height_m * 1e3,
            "sample_step_deg": self.sample_step_deg,
            "beta_min_deg": self.beta_min_deg,
            "beta_max_deg": self.beta_max_deg,
            "beta_width_deg": self.width_deg,
            "contiguous": self.contiguous,
            "best_beta_deg": self.best_beta_deg,
            "best_alpha_deg": self.best_alpha_deg,
            "best_seam_margin_deg": self.best_seam_margin_deg,
            "alpha_min_deg": self.alpha_min_deg,
            "alpha_max_deg": self.alpha_max_deg,
            "hip_above_top_mm": (
                None if self.hip_above_top_m is None else self.hip_above_top_m * 1e3
            ),
            "rolling_arrival_alpha_deg": self.rolling_arrival_alpha_deg,
        }


def _seam_margin_deg(alpha_deg: float) -> float:
    """Distance to the nearer end of the left rim's arc."""

    low, high = LEFT_RIM_ALPHA_LIMITS_DEG
    return float(min(alpha_deg - low, high - alpha_deg))


def left_rim_beta_window_2d(
    spec: SharedTerrainSpec2D,
    *,
    theta_rad: float = LEFT_RIM_READY_THETA_RAD,
    beta_range_deg: tuple[float, float] = (-360.0, 0.0),
    sample_step_deg: float = 0.5,
    hip_x_m: float | None = None,
    rolling_arrival_alpha_deg: float | None = None,
) -> LeftRimBetaWindow2D:
    """Sweep beta and report where the left rim is the contact.

    The leg stands on the obstacle top well inside it, so the answer is about
    the leg and not about running off an edge -- which is the same separation
    Step 8R makes when it calls readiness a property of the leg.
    """

    if hip_x_m is None:
        hip_x_m = float(spec.x_start_m + 0.5 * spec.top_length_m)

    betas = np.arange(
        beta_range_deg[0], beta_range_deg[1] + 0.5 * sample_step_deg, sample_step_deg
    )
    hits: list[LeftRimBetaSample2D] = []
    for beta_deg in betas:
        try:
            scene = standing_scene_2d(
                spec, theta_rad, hip_x_m=hip_x_m,
                support_height_m=spec.top_z_m, beta_rad=float(np.deg2rad(beta_deg)),
            )
            candidate = active_contact_candidate_2d(scene)
        except (ValueError, KeyError):
            continue
        if RimId(candidate.rim) is not RimId.LEFT:
            continue
        alpha_deg = float(np.rad2deg(candidate.alpha_rad))
        hits.append(LeftRimBetaSample2D(
            beta_deg=float(beta_deg),
            alpha_deg=alpha_deg,
            seam_margin_deg=_seam_margin_deg(alpha_deg),
            rim_budget_deg=_rim_budget_deg(
                scene, int(candidate.sample_index), float(candidate.alpha_rad)
            ),
            hip_above_top_m=(
                float(scene.hip_pose.position_world_xz_m[1]) - spec.top_z_m
            ),
        ))

    if not hits:
        return LeftRimBetaWindow2D(
            theta_deg=float(np.rad2deg(theta_rad)), height_m=float(spec.height_m),
            sample_step_deg=float(sample_step_deg),
            beta_min_deg=None, beta_max_deg=None, width_deg=0.0, contiguous=False,
            best_beta_deg=None, best_alpha_deg=None, best_seam_margin_deg=None,
            alpha_min_deg=None, alpha_max_deg=None, hip_above_top_m=None,
            rolling_arrival_alpha_deg=rolling_arrival_alpha_deg,
        )

    beta_values = [item.beta_deg for item in hits]
    alpha_values = [item.alpha_deg for item in hits]
    span = len(hits) - 1
    expected = (beta_values[-1] - beta_values[0]) / sample_step_deg
    best = max(hits, key=lambda item: item.seam_margin_deg)
    return LeftRimBetaWindow2D(
        theta_deg=float(np.rad2deg(theta_rad)),
        height_m=float(spec.height_m),
        sample_step_deg=float(sample_step_deg),
        beta_min_deg=beta_values[0],
        beta_max_deg=beta_values[-1],
        width_deg=float(beta_values[-1] - beta_values[0]),
        contiguous=bool(abs(expected - span) < 0.5),
        best_beta_deg=best.beta_deg,
        best_alpha_deg=best.alpha_deg,
        best_seam_margin_deg=best.seam_margin_deg,
        alpha_min_deg=min(alpha_values),
        alpha_max_deg=max(alpha_values),
        hip_above_top_m=best.hip_above_top_m,
        rolling_arrival_alpha_deg=rolling_arrival_alpha_deg,
        samples=tuple(hits),
    )


@dataclass(frozen=True)
class LandingBetaChoice2D:
    """Which beta to land at, and whether the rim can afford the descent.

    The rule is lexicographic and has no tunable knob: **first** buy enough rim
    arc for the corner pivot, **then** spend whatever is left maximising the
    distance to the nearer alpha seam.  Both halves are necessary and neither
    is free -- pushing alpha toward -180 deg buys budget and spends seam
    margin, and the exchange rate is 1:1 once the pivot needs more than half
    the rim.

    ``sufficient`` is False when *no* beta in the window has enough arc.  That
    is not a swing failure and must not be reported as one: it says the leg's
    left rim is too short for a corner of this height, which would stop a
    rolling arrival at the same corner just as hard.
    """

    beta_deg: float | None
    sample: LeftRimBetaSample2D | None
    required_budget_deg: float | None
    sufficient: bool
    reason: str

    def as_dict(self) -> dict:
        row = {
            "chosen_beta_deg": self.beta_deg,
            "required_budget_deg": self.required_budget_deg,
            "budget_sufficient": self.sufficient,
            "choice_reason": self.reason,
        }
        if self.sample is not None:
            row.update({
                "chosen_alpha_deg": self.sample.alpha_deg,
                "chosen_seam_margin_deg": self.sample.seam_margin_deg,
                "chosen_rim_budget_deg": self.sample.rim_budget_deg,
            })
        return row


def choose_landing_beta_2d(
    window: LeftRimBetaWindow2D,
    required_budget_deg: float | None,
) -> LandingBetaChoice2D:
    """Pick the landing beta: enough rim for the pivot, then maximum seam margin."""

    if not window.samples:
        return LandingBetaChoice2D(
            beta_deg=None, sample=None, required_budget_deg=required_budget_deg,
            sufficient=False, reason="no beta puts the left rim on the top.",
        )
    if required_budget_deg is None:
        best = max(window.samples, key=lambda item: item.seam_margin_deg)
        return LandingBetaChoice2D(
            beta_deg=best.beta_deg, sample=best, required_budget_deg=None,
            sufficient=True,
            reason="no pivot prediction available; maximised seam margin alone.",
        )

    affordable = [
        item for item in window.samples
        if item.rim_budget_deg >= float(required_budget_deg)
    ]
    if not affordable:
        best = max(window.samples, key=lambda item: item.rim_budget_deg)
        return LandingBetaChoice2D(
            beta_deg=best.beta_deg, sample=best,
            required_budget_deg=float(required_budget_deg), sufficient=False,
            reason=(
                f"the longest left-rim arc available is {best.rim_budget_deg:.1f} deg "
                f"but the corner pivot needs {float(required_budget_deg):.1f} deg."
            ),
        )
    best = max(affordable, key=lambda item: item.seam_margin_deg)
    return LandingBetaChoice2D(
        beta_deg=best.beta_deg, sample=best,
        required_budget_deg=float(required_budget_deg), sufficient=True,
        reason="enough rim arc for the pivot; seam margin maximised within that.",
    )


# --------------------------------------------------------------------------
# The landing pose, placed by its contact point
# --------------------------------------------------------------------------


def left_rim_landing_scene_2d(
    spec: SharedTerrainSpec2D,
    *,
    contact_x_m: float,
    beta_rad: float,
    theta_rad: float = LEFT_RIM_READY_THETA_RAD,
    surface_offset_m: float = 1e-9,
) -> tuple[SingleLegRollingScene2D, float]:
    """Stand the leg on the top with its contact at ``contact_x_m``.

    Placed by the *contact* rather than by the hip because the trailing corner
    is a property of the terrain and the contact is what has to arrive near it.
    The hip offset is measured once on a reference pose and reused: at a fixed
    ``(theta, beta)`` the contact sits a fixed distance from the hip, so one
    build is enough and no search is needed.

    Returns the scene and the realised contact x, so a caller can check the
    placement instead of trusting it.
    """

    reference = standing_scene_2d(
        spec, theta_rad, hip_x_m=float(spec.x_start_m + 0.5 * spec.top_length_m),
        support_height_m=spec.top_z_m, beta_rad=float(beta_rad),
        surface_offset_m=surface_offset_m,
    )
    start = swing_start_state_from_scene_2d(reference)
    offset = float(
        start.contact_point_world_xz_m[0] - reference.hip_pose.position_world_xz_m[0]
    )
    scene = standing_scene_2d(
        spec, theta_rad, hip_x_m=float(contact_x_m) - offset,
        support_height_m=spec.top_z_m, beta_rad=float(beta_rad),
        surface_offset_m=surface_offset_m,
    )
    realised = swing_start_state_from_scene_2d(scene)
    return scene, float(realised.contact_point_world_xz_m[0])


# --------------------------------------------------------------------------
# The adapter: a swing touchdown, judged by Day 6--7's own precondition
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class LeftRimReadiness2D:
    """Whether one landed pose passes ``LEFT_RIM_READY``, and then descends.

    Two separate verdicts on purpose.  ``readiness_failure`` is Step 8R's own
    precondition applied unchanged; ``descent_success`` is Step 9R actually
    running.  A pose can pass the first and fail the second -- that is the
    interesting case, because it means the precondition is not sufficient when
    the leg arrives by air instead of by rolling.
    """

    readiness_failure: str | None
    contact_rim: str | None
    contact_alpha_deg: float | None
    seam_margin_deg: float | None
    trailing_edge_distance_m: float | None
    rim_budget_deg: float | None
    predicted_pivot_deg: float | None
    descent_attempted: bool
    descent_success: bool | None
    descent_failure_phase: str | None
    descent_failure_reason: str | None
    descent_rim_preserved: bool | None
    minimum_back_face_clearance_m: float | None
    pivot_rotation_deg: float | None

    @property
    def ready(self) -> bool:
        return self.readiness_failure is None

    def as_dict(self) -> dict:
        return {
            "readiness_failure": self.readiness_failure,
            "left_rim_ready": self.ready,
            "landed_rim": self.contact_rim,
            "landed_alpha_deg": self.contact_alpha_deg,
            "alpha_seam_margin_deg": self.seam_margin_deg,
            "trailing_edge_distance_mm": (
                None if self.trailing_edge_distance_m is None
                else self.trailing_edge_distance_m * 1e3
            ),
            "rim_budget_deg": self.rim_budget_deg,
            "predicted_pivot_deg": self.predicted_pivot_deg,
            "descent_attempted": self.descent_attempted,
            "descent_success": self.descent_success,
            "descent_failure_phase": self.descent_failure_phase,
            "descent_failure_reason": self.descent_failure_reason,
            "descent_rim_preserved": self.descent_rim_preserved,
            "min_back_face_clearance_mm": (
                None if self.minimum_back_face_clearance_m is None
                else self.minimum_back_face_clearance_m * 1e3
            ),
            "pivot_rotation_deg": self.pivot_rotation_deg,
        }


def _synthetic_corner_frame(
    scene: SingleLegRollingScene2D,
) -> tuple[RetractResetFrame2D, object]:
    """Dress a posed scene as the corner-arrival frame Step 9R consumes.

    Only the fields Step 8R's precondition and Step 9R's pivot actually read
    are meaningful; the rolling bookkeeping (accumulated rotation, no-slip
    residual, displacements) is zeroed because this pose did not roll to get
    here, and pretending otherwise would put fiction in a data structure other
    code may later believe.
    """

    query = query_single_leg_rolling_scene_2d(scene)
    try:
        # The same selector the swing side uses, so "which rim is carrying
        # this pose" has one answer and not two.
        candidate = active_contact_candidate_2d(scene)
    except (ValueError, KeyError):
        candidate = None

    hip = np.asarray(scene.hip_pose.position_world_xz_m, dtype=float)
    frame = RetractResetFrame2D(
        step=0,
        branch="swing_touchdown",
        theta_rad=float(scene.theta_rad),
        beta_rad=float(scene.beta_rad),
        beta_unwrapped_rad=float(scene.beta_rad),
        accumulated_rotation_rad=0.0,
        active_rim=None if candidate is None else str(RimId(candidate.rim).value),
        active_sample_index=None if candidate is None else int(candidate.sample_index),
        alpha_rad=None if candidate is None else float(candidate.alpha_rad),
        contact_point_world_xz_m=(
            None if candidate is None
            else tuple(float(v) for v in candidate.point_world_xz_m)
        ),
        terrain_surface_id=(
            None if candidate is None else candidate.terrain_surface_id
        ),
        hip_position_world_xz_m=(float(hip[0]), float(hip[1])),
        hip_forward_displacement_m=0.0,
        contact_forward_displacement_m=0.0,
        step_hip_displacement_m=0.0,
        step_contact_displacement_m=0.0,
        no_slip_tangent_residual_m=0.0,
        valid_contact=bool(candidate is not None and query.valid_contact),
        collision=bool(query.collision),
        joint_limits_ok=True,
        foot_rim_ready=False,
        accepted=True,
        failure_reason=None,
        scene=scene,
        query_result=query,
    )
    return frame, candidate


def _rim_budget_deg(scene: SingleLegRollingScene2D, sample_index: int, alpha_rad: float) -> float:
    """Left-rim arc still ahead of the contact, in degrees.

    Reproduces Step 8R's ``rim_budget_at_corner_rad`` for a pose that never
    went through Step 8R.  It walks to the far end of the contact region the
    sample belongs to, exactly as ``_rim_region_alpha_end_rad`` does.
    """

    regions = np.asarray(scene.geometry.contact_regions)
    region = regions[sample_index]
    stop = sample_index
    while stop + 1 < len(regions) and regions[stop + 1] == region:
        stop += 1
    end = float(scene.geometry.alpha_rad[stop])
    return float(np.rad2deg(abs(end - float(alpha_rad))))


def left_rim_ready_from_landing_2d(
    spec: SharedTerrainSpec2D,
    scene: SingleLegRollingScene2D,
    *,
    theta_target_rad: float = LEFT_RIM_READY_THETA_RAD,
    run_descent: bool = True,
) -> LeftRimReadiness2D:
    """Judge a swing touchdown with Day 6--7's precondition, then descend it.

    The precondition is *called*, not restated: ``_corner_readiness_failure``
    is the same function Step 8R uses on a rolled arrival.  If a swing arrival
    and a rolled arrival are ever judged differently, it will be because the
    poses differ, not because the tests do.
    """

    frame, candidate = _synthetic_corner_frame(scene)
    readiness_failure = _corner_readiness_failure(frame, float(theta_target_rad))

    alpha_deg = None if frame.alpha_rad is None else float(np.rad2deg(frame.alpha_rad))
    trailing_distance = (
        None if frame.contact_point_world_xz_m is None
        else float(spec.x_max_m - frame.contact_point_world_xz_m[0])
    )
    budget = (
        None if frame.active_sample_index is None or frame.alpha_rad is None
        else _rim_budget_deg(scene, frame.active_sample_index, frame.alpha_rad)
    )

    wheel_radius = _wheel_radius_at_theta(
        float(theta_target_rad), spec.rolling_obstacle.scene_kwargs
    )
    cosine = 1.0 - float(spec.height_m) / wheel_radius
    predicted_pivot = (
        float(np.rad2deg(np.arccos(cosine))) if -1.0 <= cosine <= 1.0 else None
    )

    base = dict(
        readiness_failure=readiness_failure,
        contact_rim=frame.active_rim,
        contact_alpha_deg=alpha_deg,
        seam_margin_deg=None if alpha_deg is None else _seam_margin_deg(alpha_deg),
        trailing_edge_distance_m=trailing_distance,
        rim_budget_deg=budget,
        predicted_pivot_deg=predicted_pivot,
    )

    if readiness_failure is not None or not run_descent:
        return LeftRimReadiness2D(
            **base, descent_attempted=False, descent_success=None,
            descent_failure_phase=None, descent_failure_reason=None,
            descent_rim_preserved=None, minimum_back_face_clearance_m=None,
            pivot_rotation_deg=None,
        )

    transition = WheelModeTransitionResult2D(
        branch_result=_SingleFrameBranch(frames=(frame,)),
        theta_target_rad=float(theta_target_rad),
        trailing_corner_world_xz_m=(float(spec.x_max_m), float(spec.top_z_m)),
        reached_trailing_corner=True,
        left_rim_ready=True,
        readiness_failure=None,
        failure_reason=None,
        theta_reached_step=0,
        left_rim_handover_step=None,
        l_transition_m=None,
        required_beta_rotation_rad=None,
        required_hip_forward_distance_m=None,
        required_contact_forward_distance_m=None,
        corner_gap_m=trailing_distance,
        rim_budget_at_corner_rad=None if budget is None else float(np.deg2rad(budget)),
        wheel_radius_at_target_m=wheel_radius,
        predicted_pivot_rotation_rad=(
            None if predicted_pivot is None else float(np.deg2rad(predicted_pivot))
        ),
    )

    try:
        descent = run_left_rim_roll_down_2d(transition)
    except (ValueError, KeyError, IndexError) as error:
        return LeftRimReadiness2D(
            **base, descent_attempted=True, descent_success=False,
            descent_failure_phase="setup",
            descent_failure_reason=f"{type(error).__name__}: {error}",
            descent_rim_preserved=None, minimum_back_face_clearance_m=None,
            pivot_rotation_deg=None,
        )

    return LeftRimReadiness2D(
        **base,
        descent_attempted=True,
        descent_success=bool(descent.success),
        descent_failure_phase=descent.failure_phase,
        descent_failure_reason=descent.failure_reason,
        descent_rim_preserved=descent.descent_rim_preserved,
        minimum_back_face_clearance_m=descent.minimum_back_face_clearance_m,
        pivot_rotation_deg=(
            None if descent.pivot_rotation_rad is None
            else float(np.rad2deg(descent.pivot_rotation_rad))
        ),
    )


@dataclass(frozen=True)
class _SingleFrameBranch:
    """The minimum Step 9R reads off ``transition.branch_result``.

    Step 9R touches ``transition.final_frame`` and nothing else on the branch,
    so carrying a whole ``RetractResetBranchResult2D`` -- which would require a
    ``ForwardRollingResult2D``, i.e. a full rolling ascent -- would be paying
    for a rolling traversal in order to prove a swing does not need one.
    """

    frames: tuple


# --------------------------------------------------------------------------
# (a): the swing itself
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class LeftRimLandingCell2D:
    """One ``(height, top length, corner distance)`` cell of the Step 2b map."""

    concession: SwingConcession2D
    top_length_m: float
    d_corner_m: float
    landing_theta_deg: float
    landing_beta_deg: float
    target_alpha_deg: float | None
    approach_hip_x_m: float | None
    landing_hip_x_m: float | None
    hip_travel_m: float | None
    readiness: LeftRimReadiness2D | None
    landed_theta_deg: float | None
    evaluations: int
    seconds: float
    note: str = "ok"

    @property
    def usable(self) -> bool:
        """Feasible swing *and* a descent that actually ran from where it landed.

        Feasibility of the swing alone is not the claim Step 2b has to make.
        ``SWING_UP + ROLL_DOWN`` needs the two halves to join, so the verdict
        that matters is the conjunction.
        """

        return bool(
            self.concession.feasible
            and self.readiness is not None
            and self.readiness.ready
            and self.readiness.descent_success
        )

    @property
    def minimum_top_length_m(self) -> float:
        """The top run this cell consumes: leading edge to landing, plus the rest."""

        return float(self.top_length_m)

    def as_dict(self) -> dict:
        row = {
            "height_mm": self.concession.obstacle_height_m * 1e3,
            "top_length_m": self.top_length_m,
            "d_corner_mm": self.d_corner_m * 1e3,
            "landing_theta_deg": self.landing_theta_deg,
            "landing_beta_deg": self.landing_beta_deg,
            "target_alpha_deg": self.target_alpha_deg,
            "approach_hip_x_m": self.approach_hip_x_m,
            "landing_hip_x_m": self.landing_hip_x_m,
            "hip_travel_m": self.hip_travel_m,
            "landed_theta_deg": self.landed_theta_deg,
        }
        row.update(self.concession.as_dict())
        if self.readiness is None:
            row.update({
                key: None for key in
                LeftRimReadiness2D(
                    readiness_failure=None, contact_rim=None, contact_alpha_deg=None,
                    seam_margin_deg=None, trailing_edge_distance_m=None,
                    rim_budget_deg=None, predicted_pivot_deg=None,
                    descent_attempted=False, descent_success=None,
                    descent_failure_phase=None, descent_failure_reason=None,
                    descent_rim_preserved=None, minimum_back_face_clearance_m=None,
                    pivot_rotation_deg=None,
                ).as_dict()
            })
        else:
            row.update(self.readiness.as_dict())
        row["usable"] = self.usable
        row["evaluations"] = self.evaluations
        row["seconds"] = round(self.seconds, 2)
        row["note"] = self.note
        return row


def _failed_cell(
    *, height_m: float, top_length_m: float, d_corner_m: float, beta_deg: float,
    landing_theta_deg: float, approach_clearance_m: float,
    ceiling: BindingCeiling, reason: str,
    started: float, note: str, hip_x: float | None = None,
    landing_hip_x: float | None = None, target_alpha_deg: float | None = None,
    failure: SwingFailure | None = None, min_clearance_m: float | None = None,
    evaluations: int = 0, readiness: LeftRimReadiness2D | None = None,
) -> LeftRimLandingCell2D:
    return LeftRimLandingCell2D(
        concession=SwingConcession2D(
            feasible=False, direction="onto", obstacle_height_m=float(height_m),
            source=ConcessionSource.GRID_MINIMUM, binding_ceiling=ceiling,
            approach_clearance_m=float(approach_clearance_m),
            min_clearance_m=min_clearance_m, failure=failure, reason=reason,
        ),
        top_length_m=float(top_length_m),
        d_corner_m=float(d_corner_m),
        landing_theta_deg=float(landing_theta_deg),
        landing_beta_deg=float(beta_deg),
        target_alpha_deg=target_alpha_deg,
        approach_hip_x_m=hip_x,
        landing_hip_x_m=landing_hip_x,
        hip_travel_m=(
            None if hip_x is None or landing_hip_x is None else landing_hip_x - hip_x
        ),
        readiness=readiness,
        landed_theta_deg=None,
        evaluations=evaluations,
        seconds=time.time() - started,
        note=note,
    )


def minimum_swing_to_left_rim_ready_2d(
    height_m: float,
    top_length_m: float,
    d_corner_m: float,
    approach_clearance_m: float,
    *,
    settings: SwingGridSettings2D | None = None,
    landing_beta_deg: float | None = None,
    landing_theta_deg: float | None = None,
    run_descent: bool = True,
) -> LeftRimLandingCell2D:
    """The least a swing into ``LEFT_RIM_READY`` costs, for one terrain cell.

    Only ``liftoff_rise`` is searched.  There is no hip ladder here and that is
    a result, not an omission: see the module docstring.  The touchdown pose is
    then re-judged by Step 8R's precondition, because ``plan.valid`` answers
    "did the swing work", not "did it land in the state the descent needs" --
    a raised or mis-solved endpoint could be a perfectly valid swing to the
    wrong contact state.
    """

    settings = SwingGridSettings2D() if settings is None else settings
    settings = replace(settings, top_length_m=float(top_length_m))
    started = time.time()
    spec = settings.spec_for(height_m)
    theta_approach = settings.theta_rad
    #: The degenerate version of strategy #3 lands short of 17 degrees and
    #: retracts on the top.  Keeping it in the same function is deliberate:
    #: the only thing that changes is the target theta, so the two versions
    #: are priced by the same code and are directly comparable.
    landing_theta_rad = (
        LEFT_RIM_READY_THETA_RAD if landing_theta_deg is None
        else float(np.deg2rad(landing_theta_deg))
    )

    if landing_beta_deg is None:
        window = left_rim_beta_window_2d(
            spec, theta_rad=landing_theta_rad, sample_step_deg=1.0)
        if window.best_beta_deg is None:
            return _failed_cell(
                height_m=height_m, top_length_m=top_length_m, d_corner_m=d_corner_m,
                beta_deg=float("nan"),
                landing_theta_deg=float(np.rad2deg(landing_theta_rad)),
                approach_clearance_m=approach_clearance_m,
                ceiling=BindingCeiling.STANCE,
                reason=(f"no beta puts the left rim on the top at theta = "
                        f"{np.rad2deg(landing_theta_rad):.0f} deg."),
                started=started, note="no left-rim window",
            )
        landing_beta_deg = window.best_beta_deg

    hip_x = None
    try:
        hip_x = approach_hip_x_for_clearance_2d(spec, theta_approach, approach_clearance_m)
        start_scene = standing_scene_2d(spec, theta_approach, hip_x_m=hip_x)
    except (ValueError, KeyError) as error:
        return _failed_cell(
            height_m=height_m, top_length_m=top_length_m, d_corner_m=d_corner_m,
            beta_deg=landing_beta_deg,
            landing_theta_deg=float(np.rad2deg(landing_theta_rad)),
            approach_clearance_m=approach_clearance_m,
            ceiling=BindingCeiling.STANCE,
            reason=f"the leg cannot stand at the approach pose: {error}",
            started=started, note="start stance illegal", hip_x=hip_x,
        )

    try:
        target_scene, realised_x = left_rim_landing_scene_2d(
            spec,
            contact_x_m=spec.x_max_m - float(d_corner_m),
            beta_rad=float(np.deg2rad(landing_beta_deg)),
            theta_rad=landing_theta_rad,
        )
        base_request = build_swing_request_2d(
            start_scene, target_scene,
            clearance_m=settings.apex_clearance_m,
            swing_duration_s=settings.swing_duration_s,
            sample_count=settings.sample_count,
        )
    except (ValueError, KeyError) as error:
        return _failed_cell(
            height_m=height_m, top_length_m=top_length_m, d_corner_m=d_corner_m,
            beta_deg=landing_beta_deg,
            landing_theta_deg=float(np.rad2deg(landing_theta_rad)),
            approach_clearance_m=approach_clearance_m,
            ceiling=BindingCeiling.STANCE,
            reason=f"the leg cannot stand in LEFT_RIM_READY here: {error}",
            started=started, note="landing stance illegal", hip_x=hip_x,
        )

    landing_hip_x = float(target_scene.hip_pose.position_world_xz_m[0])
    pinned_hip = float(target_scene.hip_pose.position_world_xz_m[1]) - spec.top_z_m
    target_alpha_deg = float(np.rad2deg(base_request.target.target_alpha_rad))

    # The pose it *would* land in, judged before any swing is planned.  If the
    # descent cannot start from here, no swing to it is worth reporting as a
    # strategy-#3 cell, however valid the swing itself is.
    readiness = left_rim_ready_from_landing_2d(
        spec, target_scene, theta_target_rad=landing_theta_rad,
        run_descent=run_descent,
    )

    evaluations = 0
    best_failure: SwingFailure | None = None
    best_clearance: float | None = None
    best_reason: str | None = None
    reached_fit = 0

    for liftoff_rise_m in settings.liftoff_rise_ladder_m:
        plan = generate_swing_2d(
            base_request,
            arc_samples=settings.collision_arc_samples,
            liftoff_rise_m=liftoff_rise_m,
        )
        evaluations += 1
        if plan.failure not in _REACH_FAILURES:
            reached_fit += 1
        if not plan.valid:
            margin = None if plan.collision is None else plan.collision.minimum_clearance_m
            if best_clearance is None or (margin is not None and margin > best_clearance):
                best_clearance = margin
                best_failure = plan.failure
                best_reason = plan.result.failure_detail
            continue

        # ``plan.valid`` answers "did the swing work", not "did it land in the
        # state the descent needs".  The IK solves theta and beta from the hip
        # and the contact, so a valid plan can still arrive at a different
        # configuration than the one that was asked for -- and that one would
        # not be LEFT_RIM_READY.  So the *achieved* pose is rebuilt from the
        # last sample and re-judged, and it is that verdict that is reported.
        landed = plan.result.samples[-1]
        landed_theta_deg = (
            None if landed.theta_rad is None else float(np.rad2deg(landed.theta_rad))
        )
        if landed.theta_rad is not None and landed.beta_rad is not None:
            achieved = build_single_leg_rolling_scene_2d(
                float(landed.theta_rad), float(landed.beta_rad),
                float(target_scene.hip_pose.position_world_xz_m[0]),
                float(target_scene.hip_pose.position_world_xz_m[1]),
                **spec.rolling_obstacle.scene_kwargs,
            )
            readiness = left_rim_ready_from_landing_2d(
                spec, achieved, theta_target_rad=landing_theta_rad,
                run_descent=run_descent,
            )
        return LeftRimLandingCell2D(
            concession=SwingConcession2D(
                feasible=True, direction="onto", obstacle_height_m=float(height_m),
                source=ConcessionSource.GRID_MINIMUM,
                binding_ceiling=BindingCeiling.NONE,
                approach_clearance_m=float(approach_clearance_m),
                min_hip_lift_m=None,
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
                pinned_hip_above_surface_m=pinned_hip,
            ),
            top_length_m=float(top_length_m),
            d_corner_m=float(spec.x_max_m - realised_x),
            landing_theta_deg=float(np.rad2deg(landing_theta_rad)),
            landing_beta_deg=float(landing_beta_deg),
            target_alpha_deg=target_alpha_deg,
            approach_hip_x_m=hip_x,
            landing_hip_x_m=landing_hip_x,
            hip_travel_m=landing_hip_x - hip_x,
            readiness=readiness,
            landed_theta_deg=landed_theta_deg,
            evaluations=evaluations,
            seconds=time.time() - started,
        )

    return _failed_cell(
        height_m=height_m, top_length_m=top_length_m,
        d_corner_m=float(spec.x_max_m - realised_x),
        beta_deg=landing_beta_deg,
        landing_theta_deg=float(np.rad2deg(landing_theta_rad)),
        approach_clearance_m=approach_clearance_m,
        ceiling=BindingCeiling.REACH if reached_fit == 0 else BindingCeiling.FIT,
        reason=best_reason or "no liftoff rise on the ladder produced a valid swing.",
        started=started, note="swing infeasible", hip_x=hip_x,
        landing_hip_x=landing_hip_x, target_alpha_deg=target_alpha_deg,
        failure=best_failure, min_clearance_m=best_clearance,
        evaluations=evaluations, readiness=readiness,
    )


# --------------------------------------------------------------------------
# Batch
# --------------------------------------------------------------------------


def _cell_task(task):
    (height_m, top_length_m, d_corner_m, clearance_m, settings, beta_deg,
     run_descent, *rest) = task
    landing_theta_deg = rest[0] if rest else None
    return minimum_swing_to_left_rim_ready_2d(
        height_m, top_length_m, d_corner_m, clearance_m,
        settings=settings, landing_beta_deg=beta_deg,
        landing_theta_deg=landing_theta_deg, run_descent=run_descent,
    )


def run_left_rim_cells_2d(
    tasks: Sequence[tuple],
    *,
    workers: int | None = None,
    label: str = "step2b",
    progress: bool = True,
) -> list[LeftRimLandingCell2D]:
    """Evaluate explicit ``(h, L_top, d_corner, c, settings, beta, descent)`` tasks.

    An optional eighth element sets the landing theta, for the degenerate
    version of strategy #3 that lands short of 17 degrees.
    """

    workers = max(1, (os.cpu_count() or 2) - 1) if workers is None else int(workers)
    cells: list[LeftRimLandingCell2D] = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for index, cell in enumerate(pool.map(_cell_task, list(tasks)), start=1):
            cells.append(cell)
            if progress:
                verdict = (
                    "USABLE" if cell.usable
                    else ("swing-ok" if cell.concession.feasible else "FAIL")
                )
                ready = "-" if cell.readiness is None else (
                    "ready" if cell.readiness.ready
                    else str(cell.readiness.readiness_failure)
                )
                print(
                    f"  {label} [{index}/{len(tasks)}] "
                    f"h={cell.concession.obstacle_height_m * 1e3:>4.0f} mm "
                    f"L={cell.top_length_m:>5.2f} m d={cell.d_corner_m * 1e3:>4.0f} mm "
                    f"-> {verdict:<8} {ready:<38} "
                    f"lift={'--' if cell.concession.min_liftoff_rise_m is None else f'{cell.concession.min_liftoff_rise_m * 1e3:.0f}'} mm "
                    f"({cell.evaluations} evals, {cell.seconds:.0f}s)",
                    flush=True,
                )
    return cells


def _window_task(task):
    height_m, sample_step_deg = task
    return left_rim_beta_window_2d(
        SharedTerrainSpec2D(height_m=float(height_m)),
        sample_step_deg=float(sample_step_deg),
    )


def run_beta_windows_2d(
    heights_m: Sequence[float],
    *,
    sample_step_deg: float = 1.0,
    workers: int | None = None,
) -> list[LeftRimBetaWindow2D]:
    """One window per height, in parallel.

    Swept at several heights only to *check* that the window does not depend on
    the step -- the leg stands on a flat top either way, so it should not.  The
    check is cheap and a surprise here would invalidate everything downstream.
    """

    workers = max(1, min(len(heights_m), (os.cpu_count() or 2) - 1)) if workers is None else int(workers)
    tasks = [(float(height), float(sample_step_deg)) for height in heights_m]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(_window_task, tasks))


def _window_theta_task(task):
    spec, theta_deg, sample_step_deg = task
    return left_rim_beta_window_2d(
        spec, theta_rad=float(np.deg2rad(theta_deg)),
        sample_step_deg=float(sample_step_deg),
    )


def run_beta_windows_2d_at_thetas(
    spec: SharedTerrainSpec2D,
    thetas_deg: Sequence[float],
    *,
    sample_step_deg: float = 2.0,
    workers: int | None = None,
) -> list[LeftRimBetaWindow2D]:
    """One window per landing theta, in parallel.

    Unlike the height sweep this one is *expected* to differ per theta: which
    betas put the left rim lowest depends on how extended the leg is, and so
    does the hip height the landing pins.
    """

    workers = (max(1, min(len(thetas_deg), (os.cpu_count() or 2) - 1))
               if workers is None else int(workers))
    tasks = [(spec, float(theta), float(sample_step_deg)) for theta in thetas_deg]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(_window_theta_task, tasks))


def left_rim_rows(cells: Iterable[LeftRimLandingCell2D]) -> list[dict]:
    return [cell.as_dict() for cell in cells]


def beta_window_rows(windows: Iterable[LeftRimBetaWindow2D]) -> list[dict]:
    return [window.as_dict() for window in windows]
