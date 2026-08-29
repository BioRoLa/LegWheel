"""Day 8--9 Step 8: validate the touchdown velocity, and close the swing out.

Touchdown *configuration* and touchdown *velocity* are separate questions, and
the planning note keeps them separate on purpose: a swing can arrive in
exactly the right contact state and still slam into it.  The first-version
requirement is only about the surface normal::

    |v_n^TD| < v_n,max

The tangential component is reported but not judged.  It is what a later
body-velocity-matched touchdown would tune -- landing with the contact point
already moving along the surface at the body's speed is what makes the next
ROLL segment continuous instead of a fresh impact -- so it is measured now and
left for Day 9's optional extension.

This module is also the only one allowed to set ``valid``.  Everything before
it reports ``NOT_EVALUATED`` at best, because a swing is not usable until the
joints, the collisions, the touchdown state *and* the touchdown velocity have
all been checked.  It therefore refuses to run on a result that has not been
through Steps 6 and 7: silently promoting an unchecked trajectory to
``valid=True`` is exactly the failure this ordering exists to prevent.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from legwheel.planners.hybrid import query_point_to_terrain_surfaces_2d

from .cartesian_swing_contract_2d import SwingFailure, SwingResult2D


@dataclass(frozen=True)
class TouchdownVelocity2D:
    """The touchdown velocity, resolved against the surface being landed on."""

    velocity_world_xz_mps: NDArray[np.float64]
    speed_mps: float
    surface_id: str
    outward_normal_world_xz: NDArray[np.float64]
    #: Signed along the outward normal: negative means moving into the surface.
    normal_velocity_mps: float
    #: Signed along the surface tangent, pointing in +x for a level surface.
    tangential_velocity_mps: float
    normal_speed_limit_mps: float
    normal_ok: bool
    approaching: bool
    #: Diagnostic for a later body-velocity-matched touchdown (Day 9 optional).
    hip_tangential_velocity_mps: float
    tangential_mismatch_mps: float
    passed: bool
    failure: SwingFailure | None
    failure_detail: str | None

    def as_dict(self) -> dict:
        return {
            "vx_mps": float(self.velocity_world_xz_mps[0]),
            "vz_mps": float(self.velocity_world_xz_mps[1]),
            "speed_mps": self.speed_mps,
            "surface_id": self.surface_id,
            "normal_velocity_mps": self.normal_velocity_mps,
            "normal_speed_mps": abs(self.normal_velocity_mps),
            "normal_speed_limit_mps": self.normal_speed_limit_mps,
            "normal_ok": self.normal_ok,
            "approaching": self.approaching,
            "tangential_velocity_mps": self.tangential_velocity_mps,
            "hip_tangential_velocity_mps": self.hip_tangential_velocity_mps,
            "tangential_mismatch_mps": self.tangential_mismatch_mps,
            "passed": self.passed,
            "failure": None if self.failure is None else self.failure.value,
        }


def _touchdown_surface_normal(result: SwingResult2D) -> tuple[str, NDArray[np.float64]]:
    """Outward normal of the surface the swing lands on.

    Taken from the existing point/terrain query rather than assumed from the
    surface kind, so a non-level touchdown surface would resolve correctly
    without this module growing its own terrain geometry.
    """

    target = result.request.target
    surface_id = target.target_terrain_surface_id
    query = query_point_to_terrain_surfaces_2d(
        target.target_point_world_xz_m, result.request.terrain
    )
    return surface_id, np.asarray(
        query.surface_gap_by_id(surface_id).outward_normal_world_xz, dtype=float
    )


def _steps_6_and_7_have_run(result: SwingResult2D) -> tuple[bool, str | None]:
    if any(sample.collision_free is None for sample in result.samples):
        return False, "the trajectory has not been collision-checked (Step 6)."
    if result.final_contact_error_m is None or result.final_rim is None:
        return False, "the touchdown contact state has not been validated (Step 7)."
    return True, None


def validate_swing_touchdown_velocity_2d(
    result: SwingResult2D,
) -> tuple[SwingResult2D, TouchdownVelocity2D]:
    """Check the touchdown normal speed and decide whether the swing is valid.

    Raises if Steps 6 and 7 have not run: ``valid=True`` may only be reached by
    passing through every check, never by calling this one early.
    """

    if not isinstance(result, SwingResult2D):
        raise TypeError("result must be a SwingResult2D.")
    if not result.samples:
        raise ValueError("result carries no samples; run Steps 2-5 first.")
    ready, missing = _steps_6_and_7_have_run(result)
    if not ready:
        raise ValueError(f"cannot decide validity: {missing}")

    request = result.request
    limits = request.constraints
    final = result.samples[-1]
    velocity = np.asarray(final.velocity_world_xz_mps, dtype=float)
    surface_id, normal = _touchdown_surface_normal(result)
    # Tangent rotated so it points along +x on a level, up-facing surface.
    tangent = np.array([normal[1], -normal[0]], dtype=float)

    normal_velocity_mps = float(np.dot(velocity, normal))
    tangential_velocity_mps = float(np.dot(velocity, tangent))
    limit_mps = float(limits.touchdown_normal_speed_max_mps)
    normal_ok = abs(normal_velocity_mps) <= limit_mps

    # What the body is doing at the same instant.  A touchdown that matches it
    # rolls on; one that does not has to absorb the difference.
    hip_velocity = request.hip_trajectory.displacement_world_xz_m / request.swing_duration_s
    hip_tangential_mps = float(np.dot(hip_velocity, tangent))

    if normal_ok:
        passed, failure, detail = True, None, None
    else:
        passed = False
        failure = SwingFailure.TOUCHDOWN_VELOCITY_TOO_HIGH
        detail = (
            f"touchdown normal speed {abs(normal_velocity_mps):.4f} m/s exceeds "
            f"{limit_mps:.4f} m/s."
        )

    touchdown_velocity = TouchdownVelocity2D(
        velocity_world_xz_mps=velocity,
        speed_mps=float(np.linalg.norm(velocity)),
        surface_id=surface_id,
        outward_normal_world_xz=normal,
        normal_velocity_mps=normal_velocity_mps,
        tangential_velocity_mps=tangential_velocity_mps,
        normal_speed_limit_mps=limit_mps,
        normal_ok=normal_ok,
        approaching=bool(normal_velocity_mps < 0.0),
        hip_tangential_velocity_mps=hip_tangential_mps,
        tangential_mismatch_mps=float(tangential_velocity_mps - hip_tangential_mps),
        passed=passed,
        failure=failure,
        failure_detail=detail,
    )

    earlier_failure = result.failure not in (SwingFailure.NOT_EVALUATED, SwingFailure.NONE)
    if earlier_failure:
        # Something upstream already rejected this swing; Step 8 adds evidence
        # but never promotes it.
        reported_failure = result.failure
        reported_index = result.failure_sample_index
        reported_detail = (
            result.failure_detail
            if passed
            else f"{result.failure_detail} Also: {detail}"
        )
        valid = False
    elif passed:
        reported_failure = SwingFailure.NONE
        reported_index = None
        reported_detail = None
        valid = True
    else:
        reported_failure = failure
        reported_index = final.index
        reported_detail = detail
        valid = False

    return (
        SwingResult2D(
            request=request,
            samples=result.samples,
            valid=valid,
            failure=reported_failure,
            failure_sample_index=reported_index,
            failure_detail=reported_detail,
            minimum_terrain_clearance_m=result.minimum_terrain_clearance_m,
            final_contact_error_m=result.final_contact_error_m,
            final_alpha_error_rad=result.final_alpha_error_rad,
            final_rim=result.final_rim,
            final_terrain_surface_id=result.final_terrain_surface_id,
            touchdown_normal_speed_mps=abs(normal_velocity_mps),
            touchdown_tangential_speed_mps=abs(tangential_velocity_mps),
        ),
        touchdown_velocity,
    )
