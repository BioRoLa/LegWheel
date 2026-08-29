"""Day 8--9 Step 7: validate the touchdown contact state.

Arriving at the right *coordinates* is not the same as arriving in the right
*contact state*.  A swing can put the requested rim point exactly where it was
asked to go and still be wrong, because some other part of the leg-wheel
reaches the terrain first -- a different rim, a different alpha, or a
different terrain surface than the one the next rolling segment was planned
for.  The planning note is explicit about this: re-run the terrain-aware
contact query at the end and check what is *actually* touching.

So Step 7 asks two independent questions about the last sample:

1. Did the leg place the requested ``(rim, alpha)`` point at the requested
   position?  Answered by re-evaluating FK on the solved joints.
2. Is that point the one actually in contact, on the intended surface?
   Answered by ``query_contact`` on the full leg geometry.

Both can pass or fail on their own, and the second one is the reason this step
exists at all.

Passing Step 7 still does not make a swing valid: the touchdown *velocity*
(Step 8) has not been checked.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from legwheel.planners.hybrid import ContactCandidate2D, RimId, query_contact

from .cartesian_swing_contract_2d import (
    SwingFailure,
    SwingResult2D,
    select_active_contact_candidate_2d,
)
from .cartesian_swing_collision_2d import swing_sample_scene_2d
from .cartesian_swing_ik_2d import rim_contact_point_world_xz_m

#: Arc samples for the touchdown query.  Higher than the trajectory-wide
#: default on purpose: this runs on one pose, so the cost is irrelevant, and
#: the target contact states are themselves defined on a 241-sample grid.
#: Measuring the achieved alpha on a coarser grid would report a quantisation
#: artefact as an alpha error.
DEFAULT_TOUCHDOWN_ARC_SAMPLES = 241


@dataclass(frozen=True)
class TouchdownValidation2D:
    """Everything Step 7 checks about the final sample, and the verdict."""

    # 1. Did the requested rim point land at the requested position?
    requested_point_world_xz_m: NDArray[np.float64]
    achieved_point_world_xz_m: NDArray[np.float64]
    position_error_m: float
    position_ok: bool

    # 2. Is the requested point the one actually in contact?
    contact_found: bool
    actual_rim: RimId | None
    actual_alpha_rad: float | None
    actual_terrain_surface_id: str | None
    actual_gap_m: float | None
    rim_match: bool
    alpha_error_rad: float | None
    alpha_ok: bool
    surface_match: bool

    penetration_free: bool
    penetration_detail: str | None
    joint_limits_ok: bool

    passed: bool
    failure: SwingFailure | None
    failure_detail: str | None

    @property
    def alpha_error_deg(self) -> float | None:
        return None if self.alpha_error_rad is None else float(np.rad2deg(self.alpha_error_rad))

    def as_dict(self) -> dict:
        return {
            "position_error_mm": self.position_error_m * 1e3,
            "position_ok": self.position_ok,
            "contact_found": self.contact_found,
            "actual_rim": None if self.actual_rim is None else self.actual_rim.value,
            "rim_match": self.rim_match,
            "actual_alpha_deg": (
                None if self.actual_alpha_rad is None else float(np.rad2deg(self.actual_alpha_rad))
            ),
            "alpha_error_deg": self.alpha_error_deg,
            "alpha_ok": self.alpha_ok,
            "actual_terrain_surface_id": self.actual_terrain_surface_id,
            "surface_match": self.surface_match,
            "actual_gap_mm": None if self.actual_gap_m is None else self.actual_gap_m * 1e3,
            "penetration_free": self.penetration_free,
            "joint_limits_ok": self.joint_limits_ok,
            "passed": self.passed,
            "failure": None if self.failure is None else self.failure.value,
        }


def _penetration_detail(query, penetration_tolerance_m: float) -> str | None:
    """Describe any overlap at the touchdown pose, ignoring solver noise."""

    parts = []
    for kind, items in (
        ("geometry_penetration", query.geometry_penetrations),
        ("vertical_face_collision", query.collisions),
        ("link_collision", query.link_collisions),
    ):
        for item in items:
            if item.penetration_depth_m > penetration_tolerance_m:
                parts.append(
                    f"{kind} on {item.terrain_surface_id} "
                    f"({item.penetration_depth_m * 1e3:.2f} mm)"
                )
    return ", ".join(parts) if parts else None


def _verdict(
    *,
    position_ok: bool,
    contact_found: bool,
    rim_match: bool,
    alpha_ok: bool,
    surface_match: bool,
    penetration_free: bool,
    joint_limits_ok: bool,
    position_error_m: float,
    alpha_error_rad: float | None,
    actual_rim: RimId | None,
    actual_surface: str | None,
    requested_rim: RimId,
    requested_surface: str,
    penetration_detail: str | None,
) -> tuple[bool, SwingFailure | None, str | None]:
    """Order the checks so the most actionable reason is the one reported.

    Position first: if the leg is not where it was asked to be, nothing else
    about the landing means much.  Then the *identity* of the contact -- rim,
    surface, alpha -- because that is what the next rolling segment was
    planned against, and getting it wrong is a planning error even when the
    pose is perfectly legal.  Overlap and joint limits come last: at touchdown
    a sub-millimetre overlap is inside the contact model's own resolution, so
    it is the least informative thing to lead with.
    """

    if not position_ok:
        return (
            False,
            SwingFailure.TOUCHDOWN_POSITION_ERROR,
            f"touchdown position error {position_error_m * 1e3:.3f} mm.",
        )
    if not contact_found:
        return (
            False,
            SwingFailure.TOUCHDOWN_SURFACE_MISMATCH,
            "no rim sample is in contact with any terrain surface at the final pose; "
            "the leg is not touching down.",
        )
    if not rim_match:
        return (
            False,
            SwingFailure.TOUCHDOWN_RIM_MISMATCH,
            f"the leg lands on {actual_rim.value}, not the requested {requested_rim.value}.",
        )
    if not surface_match:
        return (
            False,
            SwingFailure.TOUCHDOWN_SURFACE_MISMATCH,
            f"the leg lands on {actual_surface!r}, not the requested {requested_surface!r}.",
        )
    if not alpha_ok:
        return (
            False,
            SwingFailure.TOUCHDOWN_ALPHA_ERROR,
            f"touchdown alpha error {np.rad2deg(alpha_error_rad):.3f} deg.",
        )
    if not penetration_free:
        return (False, SwingFailure.TERRAIN_COLLISION, f"at touchdown: {penetration_detail}")
    if not joint_limits_ok:
        return (
            False,
            SwingFailure.JOINT_LIMIT_VIOLATION,
            "the touchdown pose is outside the joint limits.",
        )
    return True, None, None


def validate_swing_touchdown_2d(
    result: SwingResult2D,
    *,
    arc_samples: int = DEFAULT_TOUCHDOWN_ARC_SAMPLES,
    penetration_tolerance_m: float | None = None,
) -> tuple[SwingResult2D, TouchdownValidation2D, ContactCandidate2D | None]:
    """Check that the last sample really lands in the requested ContactState.

    Returns the result with the touchdown fields filled in, the full
    validation record, and the contact candidate the leg actually rests on
    (``None`` when it rests on nothing).

    An earlier failure is never overwritten: if Step 5 or 6 already rejected
    the trajectory, that stays the headline and the touchdown findings are
    appended.  A trajectory that passes here is still reported as
    ``NOT_EVALUATED`` -- Step 8 owns the last word.
    """

    if not isinstance(result, SwingResult2D):
        raise TypeError("result must be a SwingResult2D.")
    if not result.samples:
        raise ValueError("result carries no samples; run Steps 2-5 first.")
    final = result.samples[-1]
    if final.theta_rad is None:
        raise ValueError("result has no joint solution; run Step 5 first.")

    request = result.request
    target = request.target
    limits = request.constraints
    if penetration_tolerance_m is None:
        # Touchdown is *supposed* to be in contact, so the tolerance here is
        # the contact model's own: the Day 3--5 query calls anything within
        # +-contact_tolerance of a surface "in contact", which means it cannot
        # tell "resting on it" from "0.5 mm into it".  Using the airborne
        # tolerance instead (the IK's ~10 um accuracy, right for Step 6 where
        # the leg should not be touching anything at all) would report the
        # discretisation of a legal landing as a collision.
        penetration_tolerance_m = limits.contact_tolerance_m

    # 1. Re-evaluate FK on the solved joints: did the requested rim point land
    #    where it was asked to?  This is an independent re-derivation, not a
    #    re-read of the number Step 5 already reported.
    scene = swing_sample_scene_2d(result, final, arc_samples=arc_samples)
    achieved = rim_contact_point_world_xz_m(
        final.theta_rad,
        final.beta_rad,
        target.target_rim,
        target.target_alpha_rad,
        scene.hip_pose,
    )
    requested = np.asarray(target.target_point_world_xz_m, dtype=float)
    position_error_m = float(np.linalg.norm(achieved - requested))
    position_ok = position_error_m <= limits.touchdown_position_tolerance_m

    # 2. Ask the terrain what is actually touching.
    query = query_contact(
        scene.geometry,
        scene.terrain,
        contact_tolerance_m=limits.contact_tolerance_m,
        collision_tolerance_m=limits.collision_tolerance_m,
    )
    candidate = (
        select_active_contact_candidate_2d(query.candidates) if query.candidates else None
    )
    if candidate is None:
        rim_match = surface_match = alpha_ok = False
        alpha_error_rad = None
    else:
        rim_match = candidate.rim is target.target_rim
        surface_match = candidate.terrain_surface_id == target.target_terrain_surface_id
        alpha_error_rad = float(abs(candidate.alpha_rad - target.target_alpha_rad))
        alpha_ok = alpha_error_rad <= limits.touchdown_alpha_tolerance_rad

    penetration_detail = _penetration_detail(query, penetration_tolerance_m)
    penetration_free = penetration_detail is None
    joint_limits_ok = bool(
        limits.theta_min_rad - 1e-12 <= final.theta_rad <= limits.theta_max_rad + 1e-12
    )

    passed, failure, detail = _verdict(
        position_ok=position_ok,
        contact_found=candidate is not None,
        rim_match=rim_match,
        alpha_ok=alpha_ok,
        surface_match=surface_match,
        penetration_free=penetration_free,
        joint_limits_ok=joint_limits_ok,
        position_error_m=position_error_m,
        alpha_error_rad=alpha_error_rad,
        actual_rim=None if candidate is None else candidate.rim,
        actual_surface=None if candidate is None else candidate.terrain_surface_id,
        requested_rim=target.target_rim,
        requested_surface=target.target_terrain_surface_id,
        penetration_detail=penetration_detail,
    )

    achieved.setflags(write=False)
    requested = requested.copy()
    requested.setflags(write=False)
    validation = TouchdownValidation2D(
        requested_point_world_xz_m=requested,
        achieved_point_world_xz_m=achieved,
        position_error_m=position_error_m,
        position_ok=position_ok,
        contact_found=candidate is not None,
        actual_rim=None if candidate is None else candidate.rim,
        actual_alpha_rad=None if candidate is None else float(candidate.alpha_rad),
        actual_terrain_surface_id=None if candidate is None else candidate.terrain_surface_id,
        actual_gap_m=None if candidate is None else float(candidate.terrain_gap_m),
        rim_match=rim_match,
        alpha_error_rad=alpha_error_rad,
        alpha_ok=alpha_ok,
        surface_match=surface_match,
        penetration_free=penetration_free,
        penetration_detail=penetration_detail,
        joint_limits_ok=joint_limits_ok,
        passed=passed,
        failure=failure,
        failure_detail=detail,
    )

    earlier_failure = result.failure not in (SwingFailure.NOT_EVALUATED, SwingFailure.NONE)
    if earlier_failure:
        # Keep the earlier, more fundamental rejection as the headline.
        reported_failure = result.failure
        reported_index = result.failure_sample_index
        reported_detail = (
            result.failure_detail
            if passed
            else f"{result.failure_detail} Also at touchdown: {detail}"
        )
    elif passed:
        reported_failure = SwingFailure.NOT_EVALUATED
        reported_index = None
        reported_detail = "Touchdown contact validated; Step 8 has not run."
    else:
        reported_failure = failure
        reported_index = final.index
        reported_detail = detail

    return (
        SwingResult2D(
            request=request,
            samples=result.samples,
            valid=False,
            failure=reported_failure,
            failure_sample_index=reported_index,
            failure_detail=reported_detail,
            minimum_terrain_clearance_m=result.minimum_terrain_clearance_m,
            final_contact_error_m=position_error_m,
            final_alpha_error_rad=alpha_error_rad,
            final_rim=None if candidate is None else candidate.rim,
            final_terrain_surface_id=(
                None if candidate is None else candidate.terrain_surface_id
            ),
        ),
        validation,
        candidate,
    )
